// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2025 The LineageOS Project
 *
 * Consumer IR emitter driven over SPI.
 *
 * The LED hangs off a SPI chip select and a burst is sent by clocking out a
 * bitmap of it: userspace works out the carrier and the mark/space timings,
 * renders them into bits, and hands the driver the finished pattern.  Nothing
 * is decoded here.
 *
 * drivers/media/rc/ir-spi.c drives the same arrangement through rc-core and
 * offers it as /dev/lirc0.  This is not that driver: dada's IR HAL,
 * consumerir.qcom.so, opens /dev/ir_spi, sets the burst length with an ioctl
 * and write()s the pattern, so the interface it expects is the character
 * device Xiaomi shipped, reproduced here.  The two cannot both claim the
 * "ir-spi" compatible, which is why this depends on IR_SPI being off.
 *
 * Three things Xiaomi's driver does are not reproduced, because they are
 * mistakes rather than behaviour: it hands the result of krealloc() straight
 * to the floor and keeps using the pointer it grew, which is a use-after-free
 * as soon as the buffer moves; it replaces the buffer probe() allocated the
 * first time a write() arrives without a length having been set, leaking it;
 * and it logs the size of every burst at warning level.
 *
 * Three things are done that Xiaomi's driver does not do.  The burst length
 * is bounded: Xiaomi's takes whatever the ioctl is given and allocates it, so
 * any process that can open the node can ask for an arbitrary allocation.
 * The ceiling is Xiaomi's own -- the size probe() preallocates.  Refusing to
 * resize a buffer two openers share returns -EBUSY rather than -EPERM, which
 * is what the situation is.  And remove() deregisters the character device
 * before freeing the buffer, rather than after: the shipped order leaves a
 * live node pointing at freed memory for as long as it takes to return.
 */

#define pr_fmt(fmt) "ir-spi: " fmt

#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>

/* Set the length of the burst the next write() will carry, in bytes. */
#define IR_SPI_IOC_SET_LEN	_IOW('i', 0x11, unsigned int)

/*
 * A burst is at most a few hundred milliseconds of carrier at 1.92 MHz, which
 * the HAL renders one bit per cycle.  150000 bytes covers every remote it
 * knows, and is what the buffer is sized to up front so that a transmission
 * never has to allocate.
 */
#define IR_SPI_MAX_LEN		150000
#define IR_SPI_FREQ		1920000
#define IR_SPI_BITS_PER_WORD	32

/* Refuse to open beyond this; the count is a u16 and must not wrap. */
#define IR_SPI_MAX_OPEN		0x7fff

struct ir_spi_data {
	u16 open_count;
	u32 buf_size;
	u8 *buf;
	struct spi_device *spi;
	struct spi_transfer xfer;
	struct mutex mutex;
};

/*
 * open() is given the misc device, not the SPI device, so the one instance
 * has to be reachable from a global.  There is one IR LED.
 */
static struct ir_spi_data *ir_spi_data_g;

/* Give the buffer room for a burst of @len bytes.  Call with the mutex held. */
static int ir_spi_resize(struct ir_spi_data *idata, unsigned int len)
{
	u8 *buf;

	if (idata->buf && idata->buf_size >= len)
		return 0;

	buf = krealloc(idata->buf, len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	idata->buf = buf;
	idata->buf_size = len;

	return 0;
}

static ssize_t ir_spi_chardev_write(struct file *file, const char __user *buf,
				    size_t count, loff_t *ppos)
{
	struct ir_spi_data *idata = file->private_data;
	struct spi_message msg;
	bool sized;
	int ret;

	/*
	 * The length is agreed in advance so that the buffer is allocated
	 * before the burst is timed; a write that disagrees with it is a
	 * userspace bug.
	 */
	if (idata->xfer.len && idata->xfer.len != count)
		return -EINVAL;

	if (!count || count > IR_SPI_MAX_LEN)
		return -EINVAL;

	mutex_lock(&idata->mutex);

	sized = idata->xfer.len != 0;

	ret = ir_spi_resize(idata, count);
	if (ret)
		goto out;

	if (copy_from_user(idata->buf, buf, count)) {
		ret = -EFAULT;
		goto out;
	}

	idata->xfer.len = count;
	idata->xfer.tx_buf = idata->buf;

	spi_message_init(&msg);
	spi_message_add_tail(&idata->xfer, &msg);

	ret = spi_sync(idata->spi, &msg);
	if (ret)
		dev_err(&idata->spi->dev, "unable to deliver the signal\n");
out:
	/* A write that set its own length does not fix it for the next one. */
	if (!sized)
		idata->xfer.len = 0;
	mutex_unlock(&idata->mutex);

	return ret ? ret : count;
}

static long ir_spi_chardev_ioctl(struct file *file, unsigned int cmd,
				 unsigned long arg)
{
	struct ir_spi_data *idata = file->private_data;
	unsigned int len;
	int ret;

	if (cmd != IR_SPI_IOC_SET_LEN)
		return -EINVAL;

	if (get_user(len, (unsigned int __user *)arg))
		return -EFAULT;

	if (!len || len > IR_SPI_MAX_LEN)
		return -EINVAL;

	if (idata->xfer.len == len)
		return 0;

	/* Two users would be sizing the same buffer against each other. */
	if (idata->open_count > 1)
		return -EBUSY;

	mutex_lock(&idata->mutex);
	ret = ir_spi_resize(idata, len);
	if (!ret)
		idata->xfer.len = len;
	mutex_unlock(&idata->mutex);

	return ret;
}

static int ir_spi_chardev_open(struct inode *inode, struct file *file)
{
	struct ir_spi_data *idata = ir_spi_data_g;

	if (idata->open_count >= IR_SPI_MAX_OPEN) {
		dev_err(&idata->spi->dev, "device busy\n");
		return -EBUSY;
	}

	file->private_data = idata;

	mutex_lock(&idata->mutex);
	idata->open_count++;
	mutex_unlock(&idata->mutex);

	return 0;
}

static int ir_spi_chardev_close(struct inode *inode, struct file *file)
{
	struct ir_spi_data *idata = file->private_data;

	mutex_lock(&idata->mutex);
	/* The last one out leaves the burst length and carrier as found. */
	if (!--idata->open_count) {
		idata->xfer.len = 0;
		idata->xfer.speed_hz = IR_SPI_FREQ;
	}
	mutex_unlock(&idata->mutex);

	return 0;
}

static const struct file_operations ir_spi_fops = {
	.owner		= THIS_MODULE,
	.llseek		= noop_llseek,
	.write		= ir_spi_chardev_write,
	.unlocked_ioctl	= ir_spi_chardev_ioctl,
	.compat_ioctl	= compat_ptr_ioctl,
	.open		= ir_spi_chardev_open,
	.release	= ir_spi_chardev_close,
};

static struct miscdevice ir_spi_miscdev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "ir_spi",
	.fops	= &ir_spi_fops,
	.mode	= 0666,
};

static int ir_spi_probe(struct spi_device *spi)
{
	struct ir_spi_data *idata;

	idata = devm_kzalloc(&spi->dev, sizeof(*idata), GFP_KERNEL);
	if (!idata)
		return -ENOMEM;

	mutex_init(&idata->mutex);
	idata->spi = spi;
	idata->xfer.bits_per_word = IR_SPI_BITS_PER_WORD;
	idata->xfer.speed_hz = IR_SPI_FREQ;

	idata->buf = kmalloc(IR_SPI_MAX_LEN, GFP_KERNEL);
	if (!idata->buf)
		return -ENOMEM;
	idata->buf_size = IR_SPI_MAX_LEN;

	spi_set_drvdata(spi, idata);
	ir_spi_data_g = idata;

	return misc_register(&ir_spi_miscdev);
}

static void ir_spi_remove(struct spi_device *spi)
{
	struct ir_spi_data *idata = spi_get_drvdata(spi);

	misc_deregister(&ir_spi_miscdev);
	ir_spi_data_g = NULL;
	kfree(idata->buf);
	idata->buf = NULL;
}

static const struct of_device_id ir_spi_of_match[] = {
	{ .compatible = "ir-spi" },
	{}
};
MODULE_DEVICE_TABLE(of, ir_spi_of_match);

static struct spi_driver ir_spi_driver = {
	.probe = ir_spi_probe,
	.remove = ir_spi_remove,
	.driver = {
		.name = "ir-spi",
		.of_match_table = ir_spi_of_match,
	},
};
module_spi_driver(ir_spi_driver);

MODULE_DESCRIPTION("SPI IR LED");
MODULE_LICENSE("GPL");
