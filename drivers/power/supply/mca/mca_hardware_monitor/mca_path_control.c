// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2025 The LineageOS Project
 *
 * Which way the power flows.  See
 * include/mca/common/mca_path_control.h.
 */

#define MCA_LOG_TAG "mca_path_control"

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/kstrtox.h>
#include <mca/common/mca_log.h>
#include <mca/common/mca_parse_dts.h>
#include <mca/hardware/hw_path_control.h>
#include <mca/platform/platform_buckchg_class.h>
#include <mca/platform/platform_cp_class.h>
#include <mca/platform/platform_wireless_class.h>
#include "../mca_strategy/strategy_wireless/inc/mca_wireless_revchg.h"
#include <mca/common/mca_sysfs.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

/* How many gates a board may have, and how many combinations of sources. */
#define PATH_CONTROL_ROLE_MAX		5
#define PATH_CONTROL_PATH_MAX		16

/* How long to leave the providers to register before the first poll. */
#define PATH_CONTROL_FIRST_POLL_MS	10000

/* How many boost cases one combination may distinguish, and gates each names. */
#define PATH_CONTROL_BOOST_MAX		5
#define PATH_CONTROL_GATE_MAX		5

/* How many numbers describe one entry of each list. */
#define PATH_CONTROL_SCHEME_COLS	3
#define PATH_CONTROL_CONDITION_COLS	2
#define PATH_CONTROL_PATH_COLS		5
#define PATH_CONTROL_GATE_COLS		2

/* A field of a path row that this board leaves out. */
#define PATH_CONTROL_NULL_PARA		"null"

/* How many boost sources a row can be conditioned on. */
#define PATH_CONTROL_BOOST_PARA_MAX	3

/* Which gate. */
enum path_control_role_para {
	OVPGATE_ROLE,
	WPCGATE_ROLE,
	EXT_BST_GATE_ROLE,
	WLS_VDD_GATE_ROLE,
	HBST_GATE_ROLE,
	MAX_ROLE,
};

/* How a gate is driven. */
enum path_control_scheme {
	GPIO_SCHEME,
	PMIC_REGISTER_SCHEME,
	CP_CHIP_SCHEME,
	MAX_SCHEME,
};

/* Which boost the phone is running to supply something else. */
enum boost_type_info {
	OTG_BOOST_TYPE,
	WLS_REV_BOOST_TYPE,
	BOOST_TYPE_MAX,
};

/* What userspace can look at. */
enum path_control_attr_list {
	USB_IN_PATH_CONTROL,
	WLS_IN_PATH_CONTROL,
	WLS_REV_PATH_CONTROL,
	OTG_IN_PATH_CONTROL,
	WLS_VDD_PATH_CONTROL,
	TOTAL_PATH_CONTROL,
	PATH_CONTROL_MAX,
};

/**
 * struct gate_enable_info - one gate, and what to do with it
 * @control_gate_role:   which gate
 * @control_gate_enable: whether to open it
 */
struct gate_enable_info {
	int control_gate_role;
	int control_gate_enable;
};

/*
 * A row of a path list says which gates to drive, and under what.  The two
 * boost values are matched against what the phone is actually running: -1
 * matches anything, so a row that does not care about one of them says so.
 */
#define PATH_CONTROL_BOOST_OTG_SRC	0
#define PATH_CONTROL_BOOST_WLS_REV	1
#define PATH_CONTROL_BOOST_GATE_NUM	2
#define PATH_CONTROL_BOOST_ANY		(-1)

/**
 * struct path_control_boost_para - what to do under one boost arrangement
 * @boost_para:       what the two boost sources must be, and how many gates
 * @gate_enable_info: the gates and what to do with them
 */
struct path_control_boost_para {
	int				boost_para[PATH_CONTROL_BOOST_PARA_MAX];
	struct gate_enable_info		*gate_enable_info;
};

/**
 * struct path_control_condition_cfg - what one combination of sources means
 * @condition:      the sources that are in force
 * @boost_cfg_num:  how many boost cases it distinguishes
 * @boost_cfg:      what to do in each
 */
struct path_control_condition_cfg {
	int				condition;
	int				boost_cfg_num;
	struct path_control_boost_para	*boost_cfg;
};

/**
 * struct path_control_scheme_cfg - how one gate is reached
 * @gate_role:           which gate
 * @scheme:              how it is driven
 * @gpio:                the pin, when it is one
 * @process_way:         which way round the gate is
 * @cp_chip_role:        which charge pump owns it, when one does
 * @control_enable_func: what to call to drive it
 */
struct path_control_scheme_cfg {
	int	gate_role;
	int	scheme;
	int	gpio;
	int	process_way;
	int	cp_chip_role;
	int	(*control_enable_func)(int role, bool enable);
};

/* Where the gates appear to userspace. */
#define PATH_CONTROL_DIR_NAME	"path_control"

/**
 * struct mca_path_control - the gates on this board
 * @dev:                    this device
 * @enable_handling_lock:   one caller changes the gates at a time
 * @update_status_work:     rereads what is connected and what is boosting
 * @condition:              the sources currently in force, as bits
 * @otg_boost_src:          which boost supplies OTG
 * @wls_rev_boost_default:  which boost supplies reverse wireless charging
 * @otg_boost_enable_sts:   whether the OTG boost is running
 * @usb_online:             charging by cable
 * @wireless_online:        charging by pad
 * @wireless_rev_en:        supplying a pad
 * @wireless_vdd_en:        the wireless rail is powered
 * @control_scheme:         how each gate is reached
 * @control_path:           what each combination of sources means
 */
struct mca_path_control {
	struct device			*dev;
	struct mutex			enable_handling_lock;
	struct delayed_work		update_status_work;
	int				condition;
	int				otg_boost_src;
	int				wls_rev_boost_default;
	int				otg_boost_enable_sts;
	int				usb_online;
	int				wireless_online;
	bool				wireless_rev_en;
	bool				wireless_vdd_en;
	struct path_control_scheme_cfg	control_scheme[PATH_CONTROL_ROLE_MAX];
	struct path_control_condition_cfg control_path[PATH_CONTROL_PATH_MAX];
};

static struct mca_path_control *g_path_control;

/* Which of the gate descriptions is for one gate. */
static struct path_control_scheme_cfg *
mca_path_control_find_scheme(struct mca_path_control *chip, int role)
{
	int i;

	for (i = 0; i < PATH_CONTROL_ROLE_MAX; i++) {
		if (chip->control_scheme[i].gate_role == role)
			return &chip->control_scheme[i];
	}

	return NULL;
}

/*
 * A gate on a pin.  Which way round the pin is depends on the board, so the
 * level written is the one the device tree says opens it.
 */
static int mac_path_control_gpio_scheme_func(int role, bool enable)
{
	struct path_control_scheme_cfg *cfg;
	int val;

	cfg = mca_path_control_find_scheme(g_path_control, role);
	if (!cfg)
		return 0;

	val = cfg->process_way ? !enable : enable;

	if (gpiod_direction_output_raw(gpio_to_desc(cfg->gpio), val))
		mca_log_err("set direction for enable-gate-gpio[%d] failed\n",
			    cfg->gpio);

	mca_log_info("enable-gate-gpio[%d] val is :%d\n", cfg->gpio,
		     gpiod_get_raw_value(gpio_to_desc(cfg->gpio)));

	return 0;
}

/*
 * A gate inside the charger itself.  Nothing on this board is wired this way;
 * the vendor leaves it unimplemented and so does this.
 */
static int mac_path_control_pmic_register_scheme_func(int role, bool enable)
{
	return 0;
}

/* A gate inside a charge pump. */
static int mac_path_control_cp_chip_scheme_func(int role, bool enable)
{
	struct path_control_scheme_cfg *cfg;

	cfg = mca_path_control_find_scheme(g_path_control, role);
	if (!cfg)
		return 0;

	switch (role) {
	case OVPGATE_ROLE:
		mca_log_info("set ovpgate: %d\n", enable);
		platform_class_cp_enable_ovpgate(cfg->cp_chip_role, enable);
		break;
	case WPCGATE_ROLE:
		/*
		 * The vendor logs this and does nothing else: on the boards
		 * it ships, the wireless gate is driven from elsewhere.
		 */
		mca_log_info("set wpcgate: %d\n", enable);
		break;
	default:
		mca_log_err("this switch cp cannot control, exit\n");
		break;
	}

	return 0;
}

/*
 * A source being connected or disconnected changes which gates should be
 * open, and the board's table says which.  The whole set is applied together,
 * because opening one gate before closing another is what shorts two supplies
 * into each other.
 */
static noinline int mca_path_control_handle_func(struct mca_path_control *chip,
						 int condition)
{
	struct path_control_condition_cfg *path = NULL;
	struct path_control_boost_para *boost;
	struct path_control_scheme_cfg *cfg;
	int i, j;

	for (i = 0; i < PATH_CONTROL_PATH_MAX; i++) {
		if (chip->control_path[i].condition == condition) {
			path = &chip->control_path[i];
			break;
		}
	}

	if (!path) {
		mca_log_err("this condition is not exit\n");

		return -1;
	}

	mca_log_info("condition is %d\n", condition);

	/*
	 * The same set of sources wants different gates depending on which
	 * boost is supplying them, so the row is picked on that as well.
	 */
	for (i = 0; i < path->boost_cfg_num; i++) {
		boost = &path->boost_cfg[i];

		if (boost->boost_para[PATH_CONTROL_BOOST_OTG_SRC] !=
		    PATH_CONTROL_BOOST_ANY &&
		    boost->boost_para[PATH_CONTROL_BOOST_OTG_SRC] !=
		    chip->otg_boost_src)
			continue;

		if (boost->boost_para[PATH_CONTROL_BOOST_WLS_REV] !=
		    PATH_CONTROL_BOOST_ANY &&
		    boost->boost_para[PATH_CONTROL_BOOST_WLS_REV] !=
		    chip->wls_rev_boost_default)
			continue;

		break;
	}

	if (i == path->boost_cfg_num) {
		mca_log_err("this boost combination is not exit\n");

		return -1;
	}

	mca_log_info("boost combination index is %d\n", i);

	boost = &path->boost_cfg[i];
	for (j = 0; j < boost->boost_para[PATH_CONTROL_BOOST_GATE_NUM]; j++) {
		int role = boost->gate_enable_info[j].control_gate_role;
		bool enable = boost->gate_enable_info[j].control_gate_enable;

		cfg = mca_path_control_find_scheme(chip, role);
		if (!cfg || !cfg->control_enable_func)
			continue;

		mca_log_info("gate[%d] ready to enable[%d]\n", role, enable);
		cfg->control_enable_func(role, enable);
	}

	return 0;
}

/**
 * mca_path_control_enable_gate() - connect or disconnect one source
 * @src:    which source
 * @enable: whether it should be connected
 *
 * The caller names the source rather than the gates, because which gates a
 * source needs is a property of the board and not of the caller.
 */
int mca_path_control_enable_gate(CONTROL_SRC src, bool enable)
{
	struct mca_path_control *chip = g_path_control;

	if (!chip)
		return -1;

	mca_log_info("src: %d, enable: %d\n", src, enable);

	mutex_lock(&chip->enable_handling_lock);

	if (enable)
		chip->condition |= src;
	else
		chip->condition &= ~src;

	mca_log_info("path's value is %d\n", chip->condition);

	mca_path_control_handle_func(chip, chip->condition);

	mutex_unlock(&chip->enable_handling_lock);

	return 0;
}
EXPORT_SYMBOL(mca_path_control_enable_gate);

/*
 * What the phone is doing, as one number.  Everything that could change the
 * gates is read together, because acting on a stale half of it is what leaves
 * two supplies connected to each other.
 */
static void mca_path_control_update_status_work(struct work_struct *work)
{
	struct mca_path_control *chip =
		container_of(work, struct mca_path_control,
			     update_status_work.work);
	int condition;

	mca_log_info("update condition status work\n");

	platform_class_buckchg_ops_get_otg_boost_src(MAIN_BUCK_CHARGER,
						     &chip->otg_boost_src);
	mca_wireless_rev_get_rev_boost_default(&chip->wls_rev_boost_default);
	mca_log_info("otg_boost_src is %d, wls_rev_boost_default is %d\n",
		     chip->otg_boost_src, chip->wls_rev_boost_default);

	platform_class_buckchg_ops_get_online(MAIN_BUCK_CHARGER,
					      &chip->usb_online);
	platform_class_buckchg_ops_get_otg_boost_enable_status(MAIN_BUCK_CHARGER,
							       &chip->otg_boost_enable_sts);
	platform_class_wireless_is_present(WIRELESS_ROLE_MASTER,
					   &chip->wireless_online);
	mca_wireless_rev_get_reverse_chg(&chip->wireless_rev_en);
	mca_log_info("usb_online: %d, wireless_online: %d, otg_boost_enable_sts: %d, wireless_rev_en: %d\n",
		     chip->usb_online, chip->wireless_online,
		     chip->otg_boost_enable_sts, chip->wireless_rev_en);

	condition = chip->condition & ~(PATH_CONTROL_USB | PATH_CONTROL_WLS |
					PATH_CONTROL_WLS_REV |
					PATH_CONTROL_OTG | PATH_CONTROL_VDD);

	if (chip->usb_online)
		condition |= PATH_CONTROL_USB;
	if (chip->wireless_online)
		condition |= PATH_CONTROL_WLS;
	if (chip->wireless_rev_en)
		condition |= PATH_CONTROL_WLS_REV;

	/*
	 * The boost being off is one state and being in either of its two
	 * running states is the other; what matters here is only whether the
	 * phone is supplying something through the connector.
	 */
	/*
	 * Only "enabling" and "enabled" mean the boost is sourcing.  Written as
	 * a subtraction this has to be unsigned: otg_boost_enable_sts is signed
	 * and idles at zero, which would otherwise satisfy the range.
	 */
	if (chip->otg_boost_enable_sts == 1 || chip->otg_boost_enable_sts == 2)
		condition |= PATH_CONTROL_OTG;
	if (chip->wireless_vdd_en)
		condition |= PATH_CONTROL_VDD;

	chip->condition = condition;
}

static ssize_t path_control_sysfs_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf);
static ssize_t path_control_sysfs_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count);

static struct mca_sysfs_attr_info path_control_sysfs_field_tbl[] = {
	mca_sysfs_attr_rw(path_control_sysfs, 0664, USB_IN_PATH_CONTROL,
			  usb_in),
	mca_sysfs_attr_rw(path_control_sysfs, 0664, WLS_IN_PATH_CONTROL,
			  wls_in),
	mca_sysfs_attr_rw(path_control_sysfs, 0664, WLS_REV_PATH_CONTROL,
			  wls_rev),
	mca_sysfs_attr_rw(path_control_sysfs, 0664, OTG_IN_PATH_CONTROL,
			  otg_in),
	mca_sysfs_attr_rw(path_control_sysfs, 0664, WLS_VDD_PATH_CONTROL,
			  wls_vdd),
	mca_sysfs_attr_rw(path_control_sysfs, 0664, TOTAL_PATH_CONTROL,
			  path_control),
};

static struct attribute *path_control_attrs[ARRAY_SIZE(path_control_sysfs_field_tbl) + 1];
static const struct attribute_group path_control_sysfs_attr_group = {
	.attrs = path_control_attrs,
};

static ssize_t path_control_sysfs_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct mca_path_control *chip = dev_get_drvdata(dev);
	struct mca_sysfs_attr_info *info;
	int val;

	info = mca_sysfs_lookup_attr(attr->attr.name,
				     path_control_sysfs_field_tbl,
				     ARRAY_SIZE(path_control_sysfs_field_tbl));
	if (!info)
		return -1;

	if (!chip) {
		mca_log_err("dev_driverdata is null\n");

		return -1;
	}

	switch (info->sysfs_attr_name) {
	case USB_IN_PATH_CONTROL:
		val = chip->condition & PATH_CONTROL_USB;
		break;
	case WLS_IN_PATH_CONTROL:
		val = chip->condition & PATH_CONTROL_WLS;
		break;
	case WLS_REV_PATH_CONTROL:
		val = chip->condition & PATH_CONTROL_WLS_REV;
		break;
	case OTG_IN_PATH_CONTROL:
		val = chip->condition & PATH_CONTROL_OTG;
		break;
	case WLS_VDD_PATH_CONTROL:
		val = chip->condition & PATH_CONTROL_VDD;
		break;
	case TOTAL_PATH_CONTROL:
		val = chip->condition;
		break;
	default:
		return 0;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}

static ssize_t path_control_sysfs_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct mca_path_control *chip = dev_get_drvdata(dev);
	struct mca_sysfs_attr_info *info;
	int val;

	info = mca_sysfs_lookup_attr(attr->attr.name,
				     path_control_sysfs_field_tbl,
				     ARRAY_SIZE(path_control_sysfs_field_tbl));
	if (!info)
		return -1;

	if (!chip) {
		mca_log_err("dev_driverdata is null\n");

		return -1;
	}

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	switch (info->sysfs_attr_name) {
	case USB_IN_PATH_CONTROL:
		mca_path_control_enable_gate(PATH_CONTROL_USB, val);
		break;
	case WLS_IN_PATH_CONTROL:
		mca_path_control_enable_gate(PATH_CONTROL_WLS, val);
		break;
	case WLS_REV_PATH_CONTROL:
		mca_path_control_enable_gate(PATH_CONTROL_WLS_REV, val);
		break;
	case OTG_IN_PATH_CONTROL:
		mca_path_control_enable_gate(PATH_CONTROL_OTG, val);
		break;
	case WLS_VDD_PATH_CONTROL:
		chip->wireless_vdd_en = val;
		mca_path_control_enable_gate(PATH_CONTROL_VDD, val);
		break;
	case TOTAL_PATH_CONTROL:
		/*
		 * Drives the gates straight from the mask written here, so a
		 * combination the sources would never ask for together can be
		 * set for testing. It deliberately leaves chip->condition
		 * alone, so the next source change puts the gates back.
		 */
		mca_path_control_handle_func(chip, val);
		break;
	default:
		break;
	}

	mca_log_info("set %d, enable = %d\n", info->sysfs_attr_name, val);

	return count;
}

/*
 * Everything the board tells us is written as lists of strings rather than as
 * numbers, because a row names the property that holds the next list down.
 */
static int mca_path_control_parse_process(struct mca_path_control *chip,
					  struct path_control_boost_para *boost,
					  const char *prop)
{
	struct device_node *np = chip->dev->of_node;
	const char *field;
	int count;
	int val;
	int i;

	count = mca_parse_dts_count_strings(np, prop, PATH_CONTROL_GATE_MAX,
					    PATH_CONTROL_GATE_COLS);
	if (count < 0) {
		mca_log_err("parse %s failed\n", prop);

		return -1;
	}

	boost->boost_para[PATH_CONTROL_BOOST_GATE_NUM] =
		count / PATH_CONTROL_GATE_COLS;

	boost->gate_enable_info = kcalloc(boost->boost_para[PATH_CONTROL_BOOST_GATE_NUM],
					  sizeof(*boost->gate_enable_info),
					  GFP_KERNEL);
	if (!boost->gate_enable_info) {
		mca_log_err("gate_enable para no mem\n");

		return -1;
	}

	for (i = 0; i < count; i++) {
		if (mca_parse_dts_string_index(np, prop, i, &field))
			return -1;

		mca_log_debug("[%d]control_gate_para %s\n", i, field);

		if (kstrtoint(field, 10, &val))
			return -1;

		if (i % PATH_CONTROL_GATE_COLS == 0)
			boost->gate_enable_info[i / PATH_CONTROL_GATE_COLS].control_gate_role = val;
		else
			boost->gate_enable_info[i / PATH_CONTROL_GATE_COLS].control_gate_enable = val;
	}

	return 0;
}

static int mca_path_control_parse_path(struct mca_path_control *chip,
				       struct path_control_condition_cfg *path,
				       const char *prop)
{
	struct device_node *np = chip->dev->of_node;
	const char *field;
	int count;
	int idx = 0;
	int val;
	int i;

	count = mca_parse_dts_count_strings(np, prop, PATH_CONTROL_PATH_MAX,
					    PATH_CONTROL_PATH_COLS);
	if (count < 0) {
		mca_log_err("parse %s failed\n", prop);

		return -1;
	}

	path->boost_cfg_num = count / PATH_CONTROL_PATH_COLS;
	path->boost_cfg = kcalloc(path->boost_cfg_num,
				  sizeof(*path->boost_cfg), GFP_KERNEL);
	if (!path->boost_cfg) {
		mca_log_err("boost cfg no mem\n");

		return -1;
	}

	for (i = 0; i < count; i++) {
		struct path_control_boost_para *boost =
			&path->boost_cfg[i / PATH_CONTROL_PATH_COLS];

		if (mca_parse_dts_string_index(np, prop, i, &field))
			return -1;

		mca_log_debug("[%d]process para %s\n", i, field);

		/*
		 * The last field of a row names the list of gates; the four
		 * before it are pairs saying which boost value the row is
		 * conditioned on and what it must be.
		 */
		if (i % PATH_CONTROL_PATH_COLS == PATH_CONTROL_PATH_COLS - 1) {
			if (!strcmp(field, PATH_CONTROL_NULL_PARA)) {
				mca_log_info("no need parse process_action para\n");
				continue;
			}

			if (mca_path_control_parse_process(chip, boost, field))
				return -1;

			continue;
		}

		if (kstrtoint(field, 10, &val))
			return -1;

		if (i % 2 == 0) {
			if (val > 1)
				return -1;
			idx = val;
		} else {
			boost->boost_para[idx] = val;
		}
	}

	return 0;
}

static int mca_path_control_parse_condition(struct mca_path_control *chip)
{
	struct device_node *np = chip->dev->of_node;
	const char *field;
	int count;
	int i;

	count = mca_parse_dts_count_strings(np, "path_condition",
					    PATH_CONTROL_PATH_MAX,
					    PATH_CONTROL_CONDITION_COLS);
	if (count < 0) {
		mca_log_err("parse %s failed\n", "path_condition");

		return -1;
	}

	for (i = 0; i < count; i++) {
		struct path_control_condition_cfg *path =
			&chip->control_path[i / PATH_CONTROL_CONDITION_COLS];

		if (mca_parse_dts_string_index(np, "path_condition", i,
					       &field))
			return -1;

		mca_log_debug("[%d]path condition %s\n", i, field);

		if (i % PATH_CONTROL_CONDITION_COLS == 0) {
			if (kstrtoint(field, 10, &path->condition))
				return -1;

			continue;
		}

		if (!strcmp(field, PATH_CONTROL_NULL_PARA)) {
			mca_log_info("no need parse process para\n");
			continue;
		}

		if (mca_path_control_parse_path(chip, path, field))
			return -1;
	}

	return 0;
}

static int mca_path_control_parse_scheme(struct mca_path_control *chip)
{
	struct device_node *np = chip->dev->of_node;
	struct path_control_scheme_cfg *cfg;
	const char *field;
	int count;
	int i;

	count = mca_parse_dts_count_strings(np, "control_scheme",
					    PATH_CONTROL_ROLE_MAX,
					    PATH_CONTROL_SCHEME_COLS);
	if (count < 0) {
		mca_log_err("parse %s failed\n", "control_scheme");

		return -1;
	}

	for (i = 0; i < count; i++) {
		cfg = &chip->control_scheme[i / PATH_CONTROL_SCHEME_COLS];

		if (mca_parse_dts_string_index(np, "control_scheme", i,
					       &field))
			return -1;

		mca_log_debug("[%d]control scheme %s\n", i, field);

		switch (i % PATH_CONTROL_SCHEME_COLS) {
		case 0:
			if (kstrtoint(field, 10, &cfg->gate_role))
				return -1;
			break;
		case 1:
			if (kstrtoint(field, 10, &cfg->scheme))
				return -1;

			switch (cfg->scheme) {
			case GPIO_SCHEME:
				cfg->control_enable_func =
					mac_path_control_gpio_scheme_func;
				break;
			case PMIC_REGISTER_SCHEME:
				cfg->control_enable_func =
					mac_path_control_pmic_register_scheme_func;
				break;
			case CP_CHIP_SCHEME:
				cfg->control_enable_func =
					mac_path_control_cp_chip_scheme_func;
				break;
			default:
				break;
			}
			break;
		default:
			/*
			 * The last field names the property that holds
			 * whatever else the scheme needs.  A gate that needs
			 * nothing says so rather than leaving it out.
			 */
			if (!strcmp(field, PATH_CONTROL_NULL_PARA)) {
				mca_log_info("no need parse scheme para\n");
				break;
			}

			if (cfg->scheme == CP_CHIP_SCHEME) {
				mca_parse_dts_u32(np, field,
						  &cfg->cp_chip_role, 0);
				break;
			}

			if (cfg->scheme != GPIO_SCHEME)
				break;

			cfg->gpio = of_get_named_gpio(np, field, 0);
			if (!gpio_is_valid(cfg->gpio) ||
			    gpio_request(cfg->gpio, field)) {
				mca_log_err("request enable-gate-gpio[%d] failed\n",
					    cfg->gpio);
				cfg->control_enable_func = NULL;
				break;
			}

			mca_parse_dts_u32(np, field, &cfg->process_way, 0);

			if (gpiod_direction_output_raw(gpio_to_desc(cfg->gpio),
						       0)) {
				mca_log_err("set direction for enable-gate-gpio[%d] failed\n",
					    cfg->gpio);
				gpio_free(cfg->gpio);
				cfg->control_enable_func = NULL;
			}
			break;
		}
	}

	return 0;
}

static int probe_cnt;

static int mca_path_control_probe(struct platform_device *pdev)
{
	struct mca_path_control *chip;

	mca_log_info("probe_cnt = %d\n", probe_cnt++);

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		mca_log_err("out of memory\n");

		return -ENOMEM;
	}

	chip->dev = &pdev->dev;
	g_path_control = chip;
	platform_set_drvdata(pdev, chip);
	mutex_init(&chip->enable_handling_lock);

	if (mca_path_control_parse_scheme(chip)) {
		mca_log_err("parse control_scheme failed\n");

		return -1;
	}

	if (mca_path_control_parse_condition(chip)) {
		mca_log_err("parse path_condition failed\n");

		return -1;
	}

	INIT_DELAYED_WORK(&chip->update_status_work,
			  mca_path_control_update_status_work);
	/*
	 * Ten seconds, not straight away: the first poll asks the buck charger,
	 * the wireless class and the reverse-charging strategy for their state,
	 * and latches the answer.  At probe those providers have not all
	 * registered yet.
	 */
	queue_delayed_work(system_wq, &chip->update_status_work,
			   msecs_to_jiffies(PATH_CONTROL_FIRST_POLL_MS));

	mca_sysfs_init_attrs(path_control_attrs, path_control_sysfs_field_tbl,
			     ARRAY_SIZE(path_control_sysfs_field_tbl));
	mca_sysfs_create_link_group(MCA_SYSFS_DEV_HW_MONITOR,
				    PATH_CONTROL_DIR_NAME, chip->dev,
				    &path_control_sysfs_attr_group);

	mca_log_err("probe %s\n", "OK");

	return 0;
}

static int mca_path_control_remove(struct platform_device *pdev)
{
	return 0;
}

static void mca_path_control_shutdown(struct platform_device *pdev)
{
}

static const struct of_device_id match_table[] = {
	{ .compatible = "mca,path_control" },
	{ }
};
MODULE_DEVICE_TABLE(of, match_table);

static struct platform_driver mca_path_control_driver = {
	.driver = {
		.name		= "mca_path_control",
		.of_match_table	= match_table,
	},
	.probe		= mca_path_control_probe,
	.remove		= mca_path_control_remove,
	.shutdown	= mca_path_control_shutdown,
};
module_platform_driver(mca_path_control_driver);

MODULE_DESCRIPTION("MCA charging path control");
MODULE_LICENSE("GPL");
