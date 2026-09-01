// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2025 The LineageOS Project
 *
 * Where userspace asks for charging to change.  See
 * include/mca/common/mca_charge_interface.h.
 */

#define MCA_LOG_TAG "mca_charge_if"

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/kstrtox.h>
#include <mca/common/mca_charge_interface.h>
#include <mca/common/mca_event.h>
#include <mca/common/mca_log.h>
#include <mca/common/mca_sysfs.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/string.h>

/* The longest request userspace may write. */
#define MCA_CHARGE_IF_INPUT_MAX		64

/* How much of one path's answer is kept, and how long a reported line is. */
#define MCA_CHARGE_IF_STATE_MAX		128
#define MCA_CHARGE_IF_LINE_MAX		256

/* What userspace can ask about. */
enum mca_charge_if_sysfs_type {
	MCA_CHRAGE_IF_SYSFS_BEGIN = 0,
	MCA_CHRAGE_IF_SYSFS_INPUT_SUSPEND = MCA_CHRAGE_IF_SYSFS_BEGIN,
	MCA_CHRAGE_IF_SYSFS_CHARGE_ENABLE,
	MCA_CHARGE_IF_SYSFS_IIN_LIMIT,
	MCA_CHRAGE_IF_SYSFS_ICHG_LIMIT,
	MCA_CHRAGE_IF_SYSFS_POWER_LIMIT,
	MCA_CHRAGE_IF_SYSFS_SUSPEND_STATUS,
	MCA_CHRAGE_IF_SYSFS_SHIPMODE,
	MCA_CHRAGE_IF_SYSFS_END,
};

/*
 * The names userspace uses for the charging paths.  They are matched by name
 * rather than by number so that a script written for one phone still names
 * the right path on another.
 */
static const char * const mca_charge_if_type_name[MCA_CHARGE_IF_CHG_TYPE_END] = {
	[MCA_CHARGE_IF_CHG_TYPE_BUCK]			= "buck",
	[MCA_CHARGE_IF_CHG_TYPE_MAIN_BUCK]		= "main_buck",
	[MCA_CHARGE_IF_CHG_TYPE_AUX_BUCK]		= "aux_buck",
	[MCA_CHARGE_IF_CHG_TYPE_QC]			= "quick",
	[MCA_CHARGE_IF_CHG_TYPE_QC_MAIN_PATH]		= "quick_main",
	[MCA_CHARGE_IF_CHG_TYPE_QC_AUX_PATH]		= "quick_aux",
	[MCA_CHARGE_IF_CHG_TYPE_QC_DIV1]		= "div1",
	[MCA_CHARGE_IF_CHG_TYPE_QC_DIV2]		= "div2",
	[MCA_CHARGE_IF_CHG_TYPE_QC_DIV4]		= "div4",
	[MCA_CHARGE_IF_CHG_TYPE_WL_BUCK]		= "wl_buck",
	[MCA_CHARGE_IF_CHG_TYPE_WL_MAIN_BUCK]		= "wl_main_buck",
	[MCA_CHARGE_IF_CHG_TYPE_WL_AUX_BUCK]		= "wl_aux_buck",
	[MCA_CHARGE_IF_CHG_TYPE_WL_QC]			= "wl_quick",
	[MCA_CHARGE_IF_CHG_TYPE_WL_QC_MAIN_PATH]	= "wl_quick_main",
	[MCA_CHARGE_IF_CHG_TYPE_WL_QC_AUX_PATH]		= "wl_quick_aux",
	[MCA_CHARGE_IF_CHG_TYPE_WL_QC_DIV1]		= "wl_div1",
	[MCA_CHARGE_IF_CHG_TYPE_WL_QC_DIV2]		= "wl_div2",
	[MCA_CHARGE_IF_CHG_TYPE_WL_QC_DIV4]		= "wl_div4",
	[MCA_CHARGE_IF_CHG_TYPE_ALL]			= "all",
};

/*
 * One implementation per charging path.  A request that does not name a path
 * is about the wired buck charger, which is the one every phone has.
 */
static struct mca_charge_if_ops *g_charge_if_ops[MCA_CHARGE_IF_CHG_TYPE_END];

#define MCA_CHARGE_IF_DEFAULT_TYPE	MCA_CHARGE_IF_CHG_TYPE_BUCK

static int mca_charge_if_get_op_type(const char *type_name)
{
	int i;

	for (i = MCA_CHARGE_IF_CHG_TYPE_BEGIN; i < MCA_CHARGE_IF_CHG_TYPE_END;
	     i++) {
		if (mca_charge_if_type_name[i] &&
		    !strcmp(mca_charge_if_type_name[i], type_name))
			return i;
	}

	mca_log_err("fail to find type_name %s\n", type_name);

	return -1;
}

int mca_charge_if_ops_register(struct mca_charge_if_ops *ops)
{
	int type;

	if (!ops || !ops->type_name)
		return -1;

	type = mca_charge_if_get_op_type(ops->type_name);
	if (type < 0 || type >= MCA_CHARGE_IF_CHG_TYPE_END) {
		mca_log_err("type is invalid %s\n", ops->type_name);

		return -1;
	}

	g_charge_if_ops[type] = ops;

	return 0;
}
EXPORT_SYMBOL(mca_charge_if_ops_register);

static ssize_t mca_charge_if_sysfs_show(struct device *dev,
					struct device_attribute *attr,
					char *buf);
static ssize_t mca_charge_if_sysfs_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count);

static struct mca_sysfs_attr_info mca_charge_if_sysfs_field_tbl[] = {
	mca_sysfs_attr_rw(mca_charge_if_sysfs, 0644,
			  MCA_CHRAGE_IF_SYSFS_INPUT_SUSPEND, input_suspend),
	mca_sysfs_attr_rw(mca_charge_if_sysfs, 0644,
			  MCA_CHRAGE_IF_SYSFS_CHARGE_ENABLE, charge_enable),
	mca_sysfs_attr_rw(mca_charge_if_sysfs, 0644,
			  MCA_CHARGE_IF_SYSFS_IIN_LIMIT, iin_limit),
	mca_sysfs_attr_rw(mca_charge_if_sysfs, 0644,
			  MCA_CHRAGE_IF_SYSFS_ICHG_LIMIT, ichg_limit),
	mca_sysfs_attr_rw(mca_charge_if_sysfs, 0644,
			  MCA_CHRAGE_IF_SYSFS_POWER_LIMIT, power_limit),
	mca_sysfs_attr_ro(mca_charge_if_sysfs, 0444,
			  MCA_CHRAGE_IF_SYSFS_SUSPEND_STATUS,
			  charge_suspend_state),
	mca_sysfs_attr_rw(mca_charge_if_sysfs, 0644,
			  MCA_CHRAGE_IF_SYSFS_SHIPMODE, shipmode_count_reset),
};

static ssize_t mca_charge_if_sysfs_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	char state[MCA_CHARGE_IF_STATE_MAX];
	char line[MCA_CHARGE_IF_LINE_MAX];
	struct mca_charge_if_ops *ops;
	struct mca_sysfs_attr_info *info;
	bool status = false;
	int len = 0;
	int i;

	info = mca_sysfs_lookup_attr(attr->attr.name,
				     mca_charge_if_sysfs_field_tbl,
				     ARRAY_SIZE(mca_charge_if_sysfs_field_tbl));
	if (!info)
		return -EINVAL;

	/*
	 * Every path is asked in turn here, because what userspace wants to
	 * know is which one is holding charging off, not whether any is.
	 */
	if (info->sysfs_attr_name == MCA_CHRAGE_IF_SYSFS_SUSPEND_STATUS) {
		for (i = MCA_CHARGE_IF_CHG_TYPE_BEGIN;
		     i < MCA_CHARGE_IF_CHG_TYPE_END; i++) {
			memset(state, 0, sizeof(state));
			memset(line, 0, sizeof(line));

			ops = g_charge_if_ops[i];
			if (!ops || !ops->get_input_suspend)
				continue;

			if (ops->get_input_suspend(state, ops->data))
				continue;

			if (!scnprintf(line, sizeof(line), "%s %s\n",
				       mca_charge_if_type_name[i], state))
				continue;

			if (len + strlen(line) >= PAGE_SIZE)
				break;

			memcpy(buf + len, line, strlen(line));
			len += strlen(line);
		}

		return len;
	}

	ops = g_charge_if_ops[MCA_CHARGE_IF_DEFAULT_TYPE];
	if (!ops)
		return -EINVAL;

	switch (info->sysfs_attr_name) {
	case MCA_CHRAGE_IF_SYSFS_INPUT_SUSPEND:
		if (!ops->get_input_suspend)
			return -EOPNOTSUPP;

		return ops->get_input_suspend(buf, ops->data);
	case MCA_CHRAGE_IF_SYSFS_CHARGE_ENABLE:
		if (!ops->get_charge_enable)
			return -EOPNOTSUPP;

		return ops->get_charge_enable(buf, ops->data);
	case MCA_CHARGE_IF_SYSFS_IIN_LIMIT:
		if (!ops->get_input_current_limit)
			return -EOPNOTSUPP;

		return ops->get_input_current_limit(buf, ops->data);
	case MCA_CHRAGE_IF_SYSFS_ICHG_LIMIT:
		if (!ops->get_charge_current_limit)
			return -EOPNOTSUPP;

		return ops->get_charge_current_limit(buf, ops->data);
	case MCA_CHRAGE_IF_SYSFS_POWER_LIMIT:
		if (!ops->get_charge_power_limit)
			return -EOPNOTSUPP;

		return ops->get_charge_power_limit(buf, ops->data);
	case MCA_CHRAGE_IF_SYSFS_SHIPMODE:
		if (!ops->get_ship_mode_status)
			return -EOPNOTSUPP;

		ops->get_ship_mode_status(&status, ops->data);

		return scnprintf(buf, PAGE_SIZE, "%d\n", status);
	default:
		return -EINVAL;
	}
}

static ssize_t mca_charge_if_sysfs_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	char input[MCA_CHARGE_IF_INPUT_MAX];
	char user[MCA_CHARGE_IF_INPUT_MAX];
	char type_name[MCA_CHARGE_IF_INPUT_MAX];
	char value[MCA_CHARGE_IF_INPUT_MAX];
	struct mca_charge_if_ops *ops;
	struct mca_sysfs_attr_info *info;
	int type;
	u32 val;

	info = mca_sysfs_lookup_attr(attr->attr.name,
				     mca_charge_if_sysfs_field_tbl,
				     ARRAY_SIZE(mca_charge_if_sysfs_field_tbl));
	if (!info)
		return -EINVAL;

	if (count >= sizeof(input)) {
		mca_log_err("input too long\n");

		return -EINVAL;
	}

	memcpy(input, buf, count);
	input[count] = '\0';

	if (sscanf(input, "%s %s %s", user, type_name, value) != 3)
		return -EINVAL;

	type = mca_charge_if_get_op_type(type_name);
	mca_log_info("user %s set node %d type %d value %s\n", user,
		     info->sysfs_attr_name, type, value);

	if (type < 0 || type >= MCA_CHARGE_IF_CHG_TYPE_END)
		return -EINVAL;

	ops = g_charge_if_ops[type];
	if (!ops)
		return -EINVAL;

	switch (info->sysfs_attr_name) {
	case MCA_CHRAGE_IF_SYSFS_INPUT_SUSPEND:
		if (!ops->set_input_suspend)
			return -EOPNOTSUPP;

		mca_log_info("set_input_suspend user: %s, value: %s\n", user,
			     value);
		ops->set_input_suspend(user, value, ops->data);
		break;
	case MCA_CHARGE_IF_SYSFS_IIN_LIMIT:
		if (!ops->set_input_current_limit)
			return -EOPNOTSUPP;

		ops->set_input_current_limit(user, value, ops->data);
		break;
	case MCA_CHRAGE_IF_SYSFS_ICHG_LIMIT:
		if (!ops->set_charge_current_limit)
			return -EOPNOTSUPP;

		ops->set_charge_current_limit(user, value, ops->data);
		break;
	case MCA_CHRAGE_IF_SYSFS_CHARGE_ENABLE:
		if (!ops->set_charge_enable)
			return -EOPNOTSUPP;

		if (kstrtouint(value, 0, &val))
			return -EINVAL;

		ops->set_charge_enable(user, val, ops->data);
		break;
	case MCA_CHRAGE_IF_SYSFS_POWER_LIMIT:
		if (!ops->set_charge_power_limit)
			return -EOPNOTSUPP;

		if (kstrtouint(value, 0, &val))
			return -EINVAL;

		ops->set_charge_power_limit(user, val, ops->data);
		break;
	case MCA_CHRAGE_IF_SYSFS_SHIPMODE:
		if (!ops->set_ship_mode_en)
			return -EOPNOTSUPP;

		if (kstrtouint(value, 0, &val))
			return -EINVAL;

		ops->set_ship_mode_en(user, val, ops->data);

		/*
		 * Clearing the ship-mode count is how a returned phone is put
		 * back into service, and what watches the battery has to be
		 * told that its stored state is no longer what it was.
		 */
		if (!val)
			mca_event_block_notify(MCA_EVENT_TYPE_BATTERY_INFO,
					       MCA_EVENT_BATTERY_STS_CHANGE,
					       NULL);
		break;
	default:
		return -EINVAL;
	}

	return count;
}

static struct attribute *mca_charge_if_attrs[ARRAY_SIZE(mca_charge_if_sysfs_field_tbl) + 1];
static const struct attribute_group mca_charge_if_attr_group = {
	.attrs = mca_charge_if_attrs,
};

/* The subdirectory the requests appear under. */
#define MCA_CHARGE_IF_DIR_NAME	"charge_interface"

static int mca_charge_if_probe(struct platform_device *pdev)
{
	struct device *dev;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	platform_set_drvdata(pdev, dev);

	mca_sysfs_init_attrs(mca_charge_if_attrs, mca_charge_if_sysfs_field_tbl,
			     ARRAY_SIZE(mca_charge_if_sysfs_field_tbl));

	if (mca_sysfs_create_link_group(MCA_SYSFS_DEV_CHARGER,
					MCA_CHARGE_IF_DIR_NAME, &pdev->dev,
					&mca_charge_if_attr_group))
		mca_log_err("create sysfs failed\n");

	return 0;
}

static int mca_charge_if_remove(struct platform_device *pdev)
{
	mca_sysfs_remove_link_group(MCA_SYSFS_DEV_CHARGER,
				    MCA_CHARGE_IF_DIR_NAME, &pdev->dev,
				    &mca_charge_if_attr_group);

	return 0;
}

static const struct of_device_id match_table[] = {
	{ .compatible = "mca,charge_interface" },
	{ }
};
MODULE_DEVICE_TABLE(of, match_table);

static struct platform_driver mca_charge_if_driver = {
	.driver = {
		.name		= "charge_interface",
		.of_match_table	= match_table,
	},
	.probe		= mca_charge_if_probe,
	.remove		= mca_charge_if_remove,
};
module_platform_driver(mca_charge_if_driver);

MODULE_DESCRIPTION("MCA charge interface");
MODULE_LICENSE("GPL");
