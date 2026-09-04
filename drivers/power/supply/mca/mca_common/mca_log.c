// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2025 The LineageOS Project
 *
 * The charging log.
 *
 * A charging problem is rarely visible at the moment it happens.  It shows up
 * as a phone that took six hours to charge overnight, or one that got warm and
 * slowed down, and the question is always what the stack was doing at the
 * time.  So the charging drivers do not log to dmesg, where they would be lost
 * in everything else and trimmed long before anyone looked: they log here, to
 * a ring of buffers this module owns, which userspace collects.
 *
 * Two things follow from that.  Each line carries a wall-clock timestamp,
 * because it will be read against a bug report and a battery history rather
 * than against other kernel messages; and the ring is filled buffer by buffer
 * and announced when one is full, so a collector can take it away rather than
 * poll for changes.
 *
 * Messages also reach dmesg, but only above a separately settable level: what
 * is worth keeping for later and what is worth interrupting the kernel log
 * with are different questions.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <mca/common/mca_event.h>
#include <mca/common/mca_log.h>
#include <mca/common/mca_sysfs.h>
#include <linux/module.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/time64.h>

/* How many filled buffers are kept before the oldest is dropped. */
#define MCA_LOG_MAX_CACHE_NUM		20

/* How much one buffer holds. */
#define MCA_LOG_BUFF_SIZE		(4 * 1024)

/* The most one line may come to. */
#define MCA_LOG_LINE_SIZE		256

/* Where the attributes appear. */
#define MCA_LOG_CLASS_NAME		"xm_power"
#define MCA_LOG_DEV_NAME		"charge_log"

/* What userspace can read and write here. */
enum mca_log_attr_list {
	MCA_LOG_ATTR_LOG_LEVEL,
	MCA_LOG_ATTR_CONSOLE_LOG_LEVEL,
	MCA_LOG_ATTR_LOG_ENABLE,
	MCA_LOG_ATTR_TIME_OFFSET,
	MCA_LOG_ATTR_LOG_INDEX,
	MCA_LOG_ATTR_CUR_BUFF,
	MCA_LOG_ATTR_MAX_BUFFER_NUM,
	MCA_LOG_ATTR_DUMP_LOG_BUFF,
	MCA_LOG_ATTR_WRITE_LOG,
	MCA_LOG_ATTR_CHARGE_LOG_HEAD,
	MCA_LOG_ATTR_CHARGE_LOG_INFO,
};

/**
 * struct mca_log_buf_info - the ring and what governs it
 * @dev:           the device the attributes hang off
 * @init_flag:     the ring is allocated and usable
 * @log_level:     messages above this level are not kept
 * @console_level: messages above this level do not reach dmesg
 * @log_enable:    logging happens at all
 * @time_offset:   hours to add to the timestamp, so a log read in one
 *                 timezone matches the phone's own clock
 * @log_index:     how many lines have been written since boot
 * @lastest_cache: which buffer is being filled
 * @dump_cache:    which buffer dump_log_buff hands out, chosen by writing
 *                 log_index.  Kept apart from lastest_cache so that reading
 *                 an older buffer does not drag the writer along with it
 * @cache_num:     how many hold anything
 * @max_cache_num: how many there are
 * @log_buff_cache: the ring
 * @log_buff:      scratch for assembling one line
 */
struct mca_log_buf_info {
	struct device	*dev;
	int		init_flag;
	int		log_level;
	int		console_level;
	int		log_enable;
	int		time_offset;
	int		log_index;
	int		lastest_cache;
	int		dump_cache;
	int		cache_num;
	int		max_cache_num;
	char		*log_buff_cache[MCA_LOG_MAX_CACHE_NUM];
	char		*log_buff;
};

static struct mca_log_buf_info mca_log = {
	.log_level	= MCA_LOG_LEVEL_INFO,
	.console_level	= MCA_LOG_LEVEL_ERROR,
	.log_enable	= 1,
	.max_cache_num	= MCA_LOG_MAX_CACHE_NUM,
};

/*
 * A spinlock rather than a mutex: the charging drivers log from interrupt
 * handlers, and a logger that can sleep is a logger they cannot call.
 */
static DEFINE_SPINLOCK(mca_log_lock);

/* What the charge log's table is assembled from. */
static struct mca_charge_log_ops_data {
	struct mca_log_charge_log_ops	*ops;
	void				*data;
} mca_log_sources[MCA_CHARGE_LOG_ID_MAX];

/*
 * Set on the kernel command line as mca_log.charge_boot_mode, from
 * androidboot.mode: the bootloader tells us whether this is a charging-only
 * boot, which the charging strategies read back through
 * mca_log_get_charge_boot_mode().
 */
static unsigned int charge_boot_mode;
module_param(charge_boot_mode, uint, 0644);
MODULE_PARM_DESC(charge_boot_mode, "charge_boot_mode");

/**
 * mca_log_proc_log_info() - put one finished line into the ring
 * @buf:  the line
 * @len:  how long it is
 * @info: the ring
 *
 * A buffer that cannot take the line is closed and the next one started, and
 * userspace is told a buffer is ready.  It is told outside the lock, because
 * announcing an event is not something to do with interrupts disabled.
 */
static noinline void mca_log_proc_log_info(char *buf, int len,
					   struct mca_log_buf_info *info)
{
	struct mca_event_notify_data n_data;
	char ebuf[64];
	unsigned long flags;
	bool full = false;
	char *cache;
	int used;

	if (!buf || len <= 0 || !info->init_flag)
		return;

	/*
	 * A line the caller did not end is ended here, so that the log reads
	 * as lines however carelessly it was written to.
	 */
	if (len <= MCA_LOG_LINE_SIZE - 2 && buf[len - 1] != '\n') {
		buf[len] = '\n';
		len++;
		buf[len] = '\0';
	}

	spin_lock_irqsave(&mca_log_lock, flags);

	cache = info->log_buff_cache[info->lastest_cache];
	if (!cache) {
		cache = kzalloc(MCA_LOG_BUFF_SIZE, GFP_ATOMIC);
		if (!cache) {
			spin_unlock_irqrestore(&mca_log_lock, flags);
			return;
		}
		info->log_buff_cache[info->lastest_cache] = cache;
		if (info->cache_num < info->max_cache_num)
			info->cache_num++;
	}

	used = strlen(cache);
	if (used + len + 1 >= MCA_LOG_BUFF_SIZE) {
		/*
		 * The ring wraps rather than growing: a phone that charges all
		 * night would otherwise hold every line of it in memory.  The
		 * oldest buffer is the one overwritten.
		 */
		info->lastest_cache = (info->lastest_cache + 1) %
				      info->max_cache_num;
		cache = info->log_buff_cache[info->lastest_cache];
		if (cache)
			cache[0] = '\0';
		full = true;
		used = 0;
	}

	if (cache)
		memcpy(cache + used, buf, len + 1);

	info->log_index++;

	spin_unlock_irqrestore(&mca_log_lock, flags);

	if (!full)
		return;

	n_data.event = ebuf;
	n_data.event_len = scnprintf(ebuf, sizeof(ebuf), "%s=%d",
				 MCA_LOG_FULL_EVENT, info->lastest_cache);
	mca_event_report_uevent(&n_data);
}

/**
 * mca_log_write() - timestamp a message and file it
 * @level: how important it is
 * @tag:   the level's letter, as it appears in the line
 * @fmt:   the message
 * @args:  its arguments
 *
 * The timestamp is wall clock rather than time since boot, because the log is
 * read against a battery history and a user's account of when the phone was
 * plugged in, neither of which is in boot time.
 */
static __always_inline void mca_log_write(int level, char tag,
					 const char *fmt, va_list args)
{
	char line[MCA_LOG_LINE_SIZE];
	struct timespec64 ts;
	struct rtc_time tm;
	int n;

	if (!mca_log.log_enable || level > mca_log.log_level)
		return;

	ktime_get_real_ts64(&ts);
	rtc_time64_to_tm(ts.tv_sec + mca_log.time_offset * 3600, &tm);

	n = snprintf(line, sizeof(line), "[%02d:%02d:%02d:%03ld-%c][%5d]",
		     tm.tm_hour, tm.tm_min, tm.tm_sec,
		     ts.tv_nsec / NSEC_PER_MSEC, tag, mca_log.log_index);
	if (n < 0 || n >= sizeof(line))
		return;

	n += vsnprintf(line + n, sizeof(line) - n, fmt, args);
	if (n >= sizeof(line))
		n = sizeof(line) - 1;

	mca_log_proc_log_info(line, n, &mca_log);

	/*
	 * What is worth keeping for later and what is worth putting in the
	 * kernel log are different questions, so they have separate levels.
	 */
	if (level <= mca_log.console_level)
		printk(KERN_ERR "%s", line);
}

void __mca_log_err(const char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	mca_log_write(MCA_LOG_LEVEL_ERROR, 'E', fmt, args);
	va_end(args);
}
EXPORT_SYMBOL_GPL(__mca_log_err);

void __mca_log_info(const char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	mca_log_write(MCA_LOG_LEVEL_INFO, 'I', fmt, args);
	va_end(args);
}
EXPORT_SYMBOL_GPL(__mca_log_info);

void __mca_log_debug(const char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	mca_log_write(MCA_LOG_LEVEL_DEBUG, 'D', fmt, args);
	va_end(args);
}
EXPORT_SYMBOL_GPL(__mca_log_debug);

void mca_log_charge_log_register(enum mca_charge_log_id_ele id,
				 struct mca_log_charge_log_ops *ops,
				 void *data)
{
	if (id >= MCA_CHARGE_LOG_ID_MAX || !ops)
		return;

	mca_log_sources[id].ops = ops;
	mca_log_sources[id].data = data;
}
EXPORT_SYMBOL_GPL(mca_log_charge_log_register);

int mca_log_get_charge_boot_mode(void)
{
	return charge_boot_mode;
}
EXPORT_SYMBOL_GPL(mca_log_get_charge_boot_mode);

static ssize_t mca_log_sysfs_show(struct device *dev,
				  struct device_attribute *attr, char *buf);
static ssize_t mca_log_sysfs_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count);

static struct mca_sysfs_attr_info mca_log_sysfs_field_tbl[] = {
	mca_sysfs_attr_rw(mca_log_sysfs, 0644, MCA_LOG_ATTR_LOG_LEVEL,
			  log_level),
	mca_sysfs_attr_rw(mca_log_sysfs, 0644, MCA_LOG_ATTR_CONSOLE_LOG_LEVEL,
			  console_log_level),
	mca_sysfs_attr_rw(mca_log_sysfs, 0644, MCA_LOG_ATTR_LOG_ENABLE,
			  log_enable),
	mca_sysfs_attr_rw(mca_log_sysfs, 0644, MCA_LOG_ATTR_TIME_OFFSET,
			  time_offset),
	mca_sysfs_attr_wo(mca_log_sysfs, 0200, MCA_LOG_ATTR_LOG_INDEX,
			  log_index),
	mca_sysfs_attr_ro(mca_log_sysfs, 0444, MCA_LOG_ATTR_CUR_BUFF,
			  cur_buff),
	mca_sysfs_attr_ro(mca_log_sysfs, 0444, MCA_LOG_ATTR_MAX_BUFFER_NUM,
			  max_buffer_num),
	mca_sysfs_attr_ro(mca_log_sysfs, 0444, MCA_LOG_ATTR_DUMP_LOG_BUFF,
			  dump_log_buff),
	mca_sysfs_attr_wo(mca_log_sysfs, 0200, MCA_LOG_ATTR_WRITE_LOG,
			  write_log),
	mca_sysfs_attr_ro(mca_log_sysfs, 0444, MCA_LOG_ATTR_CHARGE_LOG_HEAD,
			  charge_log_head),
	mca_sysfs_attr_ro(mca_log_sysfs, 0444, MCA_LOG_ATTR_CHARGE_LOG_INFO,
			  charge_log_info),
};

static struct attribute *mca_log_sysfs_attrs[ARRAY_SIZE(mca_log_sysfs_field_tbl) + 1];

static const struct attribute_group mca_log_sysfs_attr_group = {
	.attrs = mca_log_sysfs_attrs,
};

/**
 * mca_log_dump_charge_log() - assemble the charge log's table
 * @buf:  where to put it
 * @head: whether to write the headings or the values
 *
 * The table is one row assembled from every source that registered, in the
 * order the identifiers are numbered, so the headings and the values line up
 * whatever set of sources happens to be present.
 *
 * Return: how much was written.
 */
static int mca_log_dump_charge_log(char *buf, bool head)
{
	struct mca_charge_log_ops_data *s;
	int len = 0;
	int i, n;

	for (i = 0; i < MCA_CHARGE_LOG_ID_MAX; i++) {
		s = &mca_log_sources[i];
		if (!s->ops)
			continue;

		if (head)
			n = s->ops->dump_log_head ?
			    s->ops->dump_log_head(s->data, buf + len,
						  PAGE_SIZE - len) : 0;
		else
			n = s->ops->dump_log_context ?
			    s->ops->dump_log_context(s->data, buf + len,
						     PAGE_SIZE - len) : 0;
		if (n > 0)
			len += n;
	}

	len += scnprintf(buf + len, PAGE_SIZE - len, "\n");

	return len;
}

static ssize_t mca_log_sysfs_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct mca_sysfs_attr_info *field;
	unsigned long flags;
	char *cache;
	int val;

	field = mca_sysfs_lookup_attr(attr->attr.name, mca_log_sysfs_field_tbl,
				      ARRAY_SIZE(mca_log_sysfs_field_tbl));
	if (!field)
		return -EINVAL;

	switch (field->sysfs_attr_name) {
	case MCA_LOG_ATTR_LOG_LEVEL:
		val = mca_log.log_level;
		break;
	case MCA_LOG_ATTR_CONSOLE_LOG_LEVEL:
		val = mca_log.console_level;
		break;
	case MCA_LOG_ATTR_LOG_ENABLE:
		val = mca_log.log_enable;
		break;
	case MCA_LOG_ATTR_TIME_OFFSET:
		val = mca_log.time_offset;
		break;
	case MCA_LOG_ATTR_CUR_BUFF:
		val = mca_log.lastest_cache;
		break;
	case MCA_LOG_ATTR_MAX_BUFFER_NUM:
		val = mca_log.max_cache_num;
		break;
	case MCA_LOG_ATTR_CHARGE_LOG_HEAD:
		return mca_log_dump_charge_log(buf, true);
	case MCA_LOG_ATTR_CHARGE_LOG_INFO:
		return mca_log_dump_charge_log(buf, false);
	case MCA_LOG_ATTR_DUMP_LOG_BUFF:
		/*
		 * Reading a buffer hands it over: it is emptied so the next
		 * lines start a fresh one, and a collector that reads twice
		 * does not get the same lines again.
		 */
		spin_lock_irqsave(&mca_log_lock, flags);
		if (mca_log.dump_cache == mca_log.max_cache_num)
			cache = mca_log.log_buff;
		else
			cache = mca_log.log_buff_cache[mca_log.dump_cache];
		if (!cache) {
			spin_unlock_irqrestore(&mca_log_lock, flags);
			return 0;
		}
		val = strlen(cache);
		if (val >= PAGE_SIZE)
			val = PAGE_SIZE - 1;
		memcpy(buf, cache, val);
		buf[val] = '\0';
		memset(cache, 0, MCA_LOG_BUFF_SIZE);
		spin_unlock_irqrestore(&mca_log_lock, flags);
		return val;
	default:
		return -EINVAL;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}

static ssize_t mca_log_sysfs_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct mca_sysfs_attr_info *field;
	int val;

	field = mca_sysfs_lookup_attr(attr->attr.name, mca_log_sysfs_field_tbl,
				      ARRAY_SIZE(mca_log_sysfs_field_tbl));
	if (!field)
		return -EINVAL;

	/*
	 * The one attribute that takes text rather than a number: userspace
	 * marks the log where something it did begins, so the two can be read
	 * against each other.
	 */
	if (field->sysfs_attr_name == MCA_LOG_ATTR_WRITE_LOG) {
		__mca_log_info("[write_log] %s", buf);
		return count;
	}

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	switch (field->sysfs_attr_name) {
	case MCA_LOG_ATTR_LOG_LEVEL:
		mca_log.log_level = val;
		break;
	case MCA_LOG_ATTR_CONSOLE_LOG_LEVEL:
		mca_log.console_level = val;
		__mca_log_info("console level is %d", val);
		break;
	case MCA_LOG_ATTR_LOG_ENABLE:
		mca_log.log_enable = val;
		break;
	case MCA_LOG_ATTR_TIME_OFFSET:
		mca_log.time_offset = val;
		__mca_log_info("update time offset: %d", val);
		break;
	case MCA_LOG_ATTR_LOG_INDEX:
		/*
		 * Picks the buffer dump_log_buff hands out.  One past the last
		 * ring slot names the scratch the current line is assembled in.
		 */
		if (val < 0 || val > mca_log.max_cache_num)
			return -EINVAL;
		mca_log.dump_cache = val;
		break;
	default:
		return -EINVAL;
	}

	return count;
}

static int __init mca_log_init(void)
{
	mca_log.log_buff = kzalloc(MCA_LOG_BUFF_SIZE, GFP_KERNEL);
	if (!mca_log.log_buff)
		return -ENOMEM;

	mca_sysfs_init_attrs(mca_log_sysfs_attrs, mca_log_sysfs_field_tbl,
			     ARRAY_SIZE(mca_log_sysfs_field_tbl));

	mca_log.dev = mca_sysfs_create_group(MCA_LOG_CLASS_NAME,
					     MCA_LOG_DEV_NAME,
					     &mca_log_sysfs_attr_group);
	if (!mca_log.dev) {
		kfree(mca_log.log_buff);
		mca_log.log_buff = NULL;
		return -ENODEV;
	}

	mca_log.init_flag = 1;

	return 0;
}

static void __exit mca_log_exit(void)
{
	int i;

	mca_log.init_flag = 0;

	mca_sysfs_remove_group(MCA_LOG_CLASS_NAME, mca_log.dev,
			       &mca_log_sysfs_attr_group);

	for (i = 0; i < MCA_LOG_MAX_CACHE_NUM; i++)
		kfree(mca_log.log_buff_cache[i]);

	kfree(mca_log.log_buff);
}

module_init(mca_log_init);
module_exit(mca_log_exit);

MODULE_DESCRIPTION("MCA charging log");
MODULE_LICENSE("GPL");
