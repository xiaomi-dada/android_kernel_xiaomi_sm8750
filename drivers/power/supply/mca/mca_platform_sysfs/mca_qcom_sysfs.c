// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2025 The LineageOS Project
 *
 * The charging attributes Xiaomi's userspace expects to find.
 *
 * Android's own battery power supply covers what every phone has.  The
 * settings app, the charging animation and the diagnostics on this phone read
 * rather more than that: which protocol the adapter speaks, whether it
 * authenticated, what the wireless coil is doing, how many times ship mode
 * has been entered.
 *
 * Those live in a class of their own rather than as extra properties on the
 * battery, because a power supply property has to mean the same thing on
 * every device and these do not.  Each attribute is a single value read from,
 * or written straight through to, whichever part of the charging stack owns
 * it; nothing here decides anything.
 */

#define MCA_LOG_TAG "mca_qcom_sysfs"

#include <linux/device.h>
#include <linux/device/class.h>
#include <linux/errno.h>
#include "../mca_strategy/strategy_charger/inc/mca_charger_thermal.h"
#include <mca/common/mca_log.h>
#include <mca/platform/platform_buckchg_class.h>
#include <mca/platform/platform_cp_class.h>
#include <mca/platform/platform_wireless_class.h>
#include <mca/protocol/protocol_class.h>
#include <mca/protocol/protocol_pd_class.h>
#include <mca/strategy/strategy_class.h>
#include <mca/strategy/strategy_fg_class.h>
#include <mca/strategy/strategy_wireless_class.h>
#include "../mca_strategy/strategy_wireless/inc/mca_wireless_revchg.h"
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

/**
 * struct mca_qcom_sysfs - this module's state
 * @dev:                 this device
 * @mca_qcom_class:      the class the attributes hang off
 * @support_multi_typec: the board has more than one Type-C port
 */
struct mca_qcom_sysfs {
	struct device	*dev;
	struct class	mca_qcom_class;
	bool		support_multi_typec;
};

static struct mca_qcom_sysfs *g_qcom_sysfs;

/*
 * The adapter names userspace shows.  They are the strings Xiaomi's settings
 * app matches on, so they are the vendor's spelling rather than anything
 * derived from the protocol enums.
 */
static const char * const usb_real_type_text[] = {
	"Unknown", "USB_FLOAT", "SDP", "CDP", "DCP", "HVDCP", "HVDCP_3",
	"HVDCP_3_B", "HVDCP_3P5", "USB_PD", "PD_PPS", "ACA",
};

/* The same for a wireless pad. */
static const char * const wireless_type_text[] = {
	"Unknown", "BPP", "EPP", "HPP",
};

static ssize_t real_type_show(const struct class *c,
				const struct class_attribute *attr,
			      char *buf)
{
	u32 type = 0;

	protocol_class_get_adapter_type(ADAPTER_PROTOCOL_PD, &type);
	if (type >= ARRAY_SIZE(usb_real_type_text))
		type = 0;

	return scnprintf(buf, PAGE_SIZE, "%s\n", usb_real_type_text[type]);
}
static CLASS_ATTR_RO(real_type);

static ssize_t usb_real_type_show(const struct class *c,
				const struct class_attribute *attr,
				  char *buf)
{
	u32 type = 0;

	protocol_class_get_adapter_type(ADAPTER_PROTOCOL_BC12, &type);
	if (type >= ARRAY_SIZE(usb_real_type_text))
		type = 0;

	return scnprintf(buf, PAGE_SIZE, "%s\n", usb_real_type_text[type]);
}
static CLASS_ATTR_RO(usb_real_type);

static ssize_t wireless_type_show(const struct class *c,
				const struct class_attribute *attr,
				  char *buf)
{
	int type = 0;

	strategy_class_wireless_ops_get_wls_type(&type);
	if (type >= ARRAY_SIZE(wireless_type_text) || type < 0)
		type = 0;

	return scnprintf(buf, PAGE_SIZE, "%s\n", wireless_type_text[type]);
}
static CLASS_ATTR_RO(wireless_type);

/**
 * quick_charge_type_show() - which charging speed to show the user
 * @c:    the class
 * @attr: the attribute
 * @buf:  filled in with the number
 *
 * This is what decides which of the charging animations userspace plays, so
 * it reports the quick-charging strategy's own idea of what is happening
 * rather than being worked out again from the adapter type.
 *
 * Return: how much was written.
 */
static ssize_t quick_charge_type_show(const struct class *c,
				const struct class_attribute *attr, char *buf)
{
	int online = 0;
	int type = 0;

	/*
	 * Three strategies could each be charging, and whichever is answers
	 * for the type: the quick path first, the plain buck charger if it
	 * has nothing to say, and the wireless one over both if a pad is
	 * what the phone is actually on.
	 */
	mca_strategy_func_get_status(STRATEGY_FUNC_TYPE_QUICK_CHARGE,
				     STRATEGY_STATUS_TYPE_QC_TYPE, &type);
	if (!type)
		mca_strategy_func_get_status(STRATEGY_FUNC_TYPE_BUCK_CHARGE,
					     STRATEGY_STATUS_TYPE_QC_TYPE,
					     &type);

	mca_strategy_func_get_status(STRATEGY_FUNC_TYPE_BASIC_WIRELESS,
				     STRATEGY_STATUS_TYPE_ONLINE, &online);
	if (online)
		mca_strategy_func_get_status(STRATEGY_FUNC_TYPE_BASIC_WIRELESS,
					     STRATEGY_STATUS_TYPE_QC_TYPE,
					     &type);

	return snprintf(buf, PAGE_SIZE, "%d\n", type);
}
static CLASS_ATTR_RO(quick_charge_type);

static ssize_t authentic_show(const struct class *c,
				const struct class_attribute *attr,
			      char *buf)
{
	/*
	 * The battery on this phone is always the one it shipped with as far
	 * as userspace is concerned: the check that would say otherwise lives
	 * in a MIUI service that is not here, and answering anything else
	 * stops the settings app offering fast charging at all.
	 */
	return snprintf(buf, PAGE_SIZE, "%d\n", 1);
}

static ssize_t authentic_store(const struct class *c,
				const struct class_attribute *attr,
			       const char *buf, size_t count)
{
	return count;
}
static CLASS_ATTR_RW(authentic);

static ssize_t slave_authentic_show(const struct class *c,
				const struct class_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", 1);
}

static ssize_t slave_authentic_store(const struct class *c,
				const struct class_attribute *attr,
				     const char *buf, size_t count)
{
	return count;
}
static CLASS_ATTR_RW(slave_authentic);

static ssize_t pd_verifed_show(const struct class *c,
				const struct class_attribute *attr,
			       char *buf)
{
	int verified = 0;

	protocol_class_get_adapter_verified(ADAPTER_PROTOCOL_PD, &verified);
	mca_log_info("show pd_verifed = %d\n", verified);

	return snprintf(buf, PAGE_SIZE, "%d\n", verified);
}

static ssize_t pd_verifed_store(const struct class *c,
				const struct class_attribute *attr,
				const char *buf, size_t count)
{
	int val;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	mca_log_info("store pd_verifed = %d\n", val);
	protocol_class_set_adapter_verified(ADAPTER_PROTOCOL_PD, val);

	return count;
}
static CLASS_ATTR_RW(pd_verifed);

static ssize_t cc_toggle_show(const struct class *c,
				const struct class_attribute *attr,
			      char *buf)
{
	bool toggle = false;

	protocol_class_pd_get_cc_toggle(TYPEC_PORT_0, &toggle);

	return scnprintf(buf, PAGE_SIZE, "%d\n", toggle);
}

static ssize_t cc_toggle_store(const struct class *c,
				const struct class_attribute *attr,
			       const char *buf, size_t count)
{
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	protocol_class_pd_set_cc_toggle(TYPEC_PORT_0, val);

	return count;
}
static CLASS_ATTR_RW(cc_toggle);

static ssize_t cid_status_show(const struct class *c,
				const struct class_attribute *attr,
			       char *buf)
{
	bool status = false;

	protocol_class_pd_get_cid_status(TYPEC_PORT_0, &status);

	return scnprintf(buf, PAGE_SIZE, "%d\n", status);
}
static CLASS_ATTR_RO(cid_status);

static ssize_t has_dp_show(const struct class *c,
				const struct class_attribute *attr,
			   char *buf)
{
	bool has_dp = false;

	protocol_class_pd_get_has_dp(TYPEC_PORT_0, &has_dp);

	return scnprintf(buf, PAGE_SIZE, "%d\n", has_dp);
}
static CLASS_ATTR_RO(has_dp);

/*
 * Whether to offer the user a switch for feeding an attached device.  A board
 * whose cable-id pin cannot tell a plugged device from a plugged charger has
 * no way to offer it safely.
 */
static ssize_t otg_ui_support_show(const struct class *c,
				const struct class_attribute *attr,
				   char *buf)
{
	bool support = false;

	platform_class_buckchg_ops_is_support_cid(MAIN_BUCK_CHARGER, &support);

	return scnprintf(buf, PAGE_SIZE, "%d\n", support);
}
static CLASS_ATTR_RO(otg_ui_support);

static ssize_t soc_decimal_show(const struct class *c,
				const struct class_attribute *attr,
				char *buf)
{
	int soc_decimal = 0, rate = 0;

	strategy_class_fg_ops_get_soc_decimal(&soc_decimal, &rate);

	return scnprintf(buf, PAGE_SIZE, "%d\n", soc_decimal);
}
static CLASS_ATTR_RO(soc_decimal);

static ssize_t soc_decimal_rate_show(const struct class *c,
				const struct class_attribute *attr, char *buf)
{
	int soc_decimal = 0, rate = 0;

	strategy_class_fg_ops_get_soc_decimal(&soc_decimal, &rate);

	return scnprintf(buf, PAGE_SIZE, "%d\n", rate);
}
static CLASS_ATTR_RO(soc_decimal_rate);

static ssize_t soh_show(const struct class *c,
				const struct class_attribute *attr,
			char *buf)
{
	int soh = 0;

	strategy_class_fg_get_soh(&soh);

	return scnprintf(buf, PAGE_SIZE, "%d\n", soh);
}
static CLASS_ATTR_RO(soh);

static ssize_t power_max_show(const struct class *c,
				const struct class_attribute *attr,
			      char *buf)
{
	int online = 0;
	int power = 0;

	/*
	 * Only the wireless strategy has a figure to give: what a cable can
	 * supply is the adapter's business and is read from the adapter.
	 */
	mca_strategy_func_get_status(STRATEGY_FUNC_TYPE_BASIC_WIRELESS,
				     STRATEGY_STATUS_TYPE_ONLINE, &online);
	if (online)
		mca_strategy_func_get_status(STRATEGY_FUNC_TYPE_BASIC_WIRELESS,
					     STRATEGY_STATUS_TYPE_POWER_MAX,
					     &power);

	return snprintf(buf, PAGE_SIZE, "%d\n", power);
}
static CLASS_ATTR_RO(power_max);

static ssize_t pmic_ibat_show(const struct class *c,
				const struct class_attribute *attr,
			      char *buf)
{
	int ibat = 0;

	platform_class_buckchg_ops_get_pack_ibat(MAIN_BUCK_CHARGER, &ibat);

	return scnprintf(buf, PAGE_SIZE, "%d\n", ibat);
}
static CLASS_ATTR_RO(pmic_ibat);

/*
 * Ship mode disconnects the battery so a boxed phone does not go flat.  The
 * count exists because a unit that has been through it several times has been
 * opened and reboxed, which the returns process wants to know.
 */
static ssize_t shipmode_count_reset_show(const struct class *c,
				const struct class_attribute *attr,
					 char *buf)
{
	bool ship_mode = false;

	platform_class_buckchg_ops_get_ship_mode(MAIN_BUCK_CHARGER, &ship_mode);

	return scnprintf(buf, PAGE_SIZE, "%d\n", ship_mode);
}

static ssize_t shipmode_count_reset_store(const struct class *c,
				const struct class_attribute *attr,
					  const char *buf, size_t count)
{
	int val;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	platform_class_buckchg_ops_set_ship_mode(MAIN_BUCK_CHARGER, !!val);

	return count;
}
static CLASS_ATTR_RW(shipmode_count_reset);

/*
 * The gate in front of the charge pump.  Factory test opens it directly to
 * check the pump without the rest of the stack deciding otherwise.
 */
static ssize_t dam_ovpgate_show(const struct class *c,
				const struct class_attribute *attr,
				char *buf)
{
	bool status = false;

	platform_class_cp_get_ovpgate_status(CP_ROLE_MASTER, &status);

	return scnprintf(buf, PAGE_SIZE, "%d\n", status);
}

static ssize_t dam_ovpgate_store(const struct class *c,
				const struct class_attribute *attr,
				 const char *buf, size_t count)
{
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	platform_class_cp_enable_ovpgate(CP_ROLE_MASTER, val);

	return count;
}
static CLASS_ATTR_RW(dam_ovpgate);

static ssize_t rx_vout_show(const struct class *c,
				const struct class_attribute *attr,
			    char *buf)
{
	int val = 0;

	platform_class_wireless_get_vout(WIRELESS_ROLE_MASTER, &val);

	return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}
static CLASS_ATTR_RO(rx_vout);

static ssize_t rx_vrect_show(const struct class *c,
				const struct class_attribute *attr,
			     char *buf)
{
	int val = 0;

	platform_class_wireless_get_vrect(WIRELESS_ROLE_MASTER, &val);

	return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}
static CLASS_ATTR_RO(rx_vrect);

static ssize_t rx_iout_show(const struct class *c,
				const struct class_attribute *attr,
			    char *buf)
{
	int val = 0;

	platform_class_wireless_get_iout(WIRELESS_ROLE_MASTER, &val);

	return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}
static CLASS_ATTR_RO(rx_iout);

/*
 * How strongly the coil is coupled to the pad.  Userspace shows this while
 * the user is moving the phone about looking for the spot that charges.
 */
static ssize_t rx_ss_show(const struct class *c,
				const struct class_attribute *attr,
			  char *buf)
{
	int val = 0;

	platform_class_wireless_get_ss_voltage(WIRELESS_ROLE_MASTER, &val);

	return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}
static CLASS_ATTR_RO(rx_ss);

static ssize_t tx_adapter_show(const struct class *c,
				const struct class_attribute *attr,
			       char *buf)
{
	int val = 0;

	platform_class_wireless_get_tx_adapter(WIRELESS_ROLE_MASTER, &val);

	return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}
static CLASS_ATTR_RO(tx_adapter);

static ssize_t magnetic_case_flag_show(const struct class *c,
				const struct class_attribute *attr, char *buf)
{
	bool status = false;

	platform_class_wireless_get_hall_gpio_status(WIRELESS_ROLE_MASTER,
						     &status);

	return scnprintf(buf, PAGE_SIZE, "%d\n", status);
}
static CLASS_ATTR_RO(magnetic_case_flag);

/*
 * Which case is on the phone.  A case changes how well the coil couples, so
 * userspace tells the stack which one the user said they have.
 */
static ssize_t hall_phone_case_show(const struct class *c,
				const struct class_attribute *attr, char *buf)
{
	int category = 0;

	platform_class_wireless_get_phone_case_category(WIRELESS_ROLE_MASTER,
							&category);

	return scnprintf(buf, PAGE_SIZE, "%d\n", category);
}

static ssize_t hall_phone_case_store(const struct class *c,
				const struct class_attribute *attr,
				     const char *buf, size_t count)
{
	int val;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	platform_class_wireless_set_phone_case_category(WIRELESS_ROLE_MASTER,
							val);

	return count;
}
static CLASS_ATTR_RW(hall_phone_case);

static ssize_t wireless_chip_fw_show(const struct class *c,
				const struct class_attribute *attr, char *buf)
{
	char fw_version[32] = "Unknown";

	platform_class_wireless_get_fw_version(WIRELESS_ROLE_MASTER,
					       fw_version);

	return scnprintf(buf, PAGE_SIZE, "%s\n", fw_version);
}

static ssize_t wireless_chip_fw_store(const struct class *c,
				const struct class_attribute *attr,
				      const char *buf, size_t count)
{
	int val;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	mca_wireless_rev_update_fw_version(val);

	return count;
}
static CLASS_ATTR_RW(wireless_chip_fw);

static ssize_t reverse_chg_mode_show(const struct class *c,
				const struct class_attribute *attr, char *buf)
{
	bool enabled = false;

	mca_wireless_rev_get_reverse_chg(&enabled);

	return scnprintf(buf, PAGE_SIZE, "%d\n", enabled);
}

static ssize_t reverse_chg_mode_store(const struct class *c,
				const struct class_attribute *attr,
				      const char *buf, size_t count)
{
	int val = 0;
	int rc;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	/*
	 * Reverse charging is turned on here and the user's wish recorded
	 * separately, because the two come apart: the phone stops giving
	 * power when it is too cold or too empty, and has to know to start
	 * again when it is not.
	 */
	mca_wireless_rev_enable_reverse_charge(!!val);
	mca_log_err("zxy store reverse_chg_mode = %d\n", val);
	rc = mca_wireless_rev_set_user_reverse_chg(!!val);

	return rc < 0 ? rc : count;
}
static CLASS_ATTR_RW(reverse_chg_mode);

static ssize_t reverse_chg_state_show(const struct class *c,
				const struct class_attribute *attr, char *buf)
{
	int state = 0;

	mca_wireless_rev_get_reverse_chg_state(&state);

	return scnprintf(buf, PAGE_SIZE, "%d\n", state);
}
static CLASS_ATTR_RO(reverse_chg_state);

/*
 * Lets a test lift the thermal limit on wireless charging.  It is not
 * something a user can reach: sustained wireless charging with no thermal
 * budget is exactly what the budget exists to prevent.
 */
static ssize_t wls_thermal_remove_show(const struct class *c,
				const struct class_attribute *attr, char *buf)
{
	bool remove = false;

	mca_get_wls_charger_thermal_remove(&remove);

	return scnprintf(buf, PAGE_SIZE, "%d\n", remove);
}

static ssize_t wls_thermal_remove_store(const struct class *c,
				const struct class_attribute *attr,
					const char *buf, size_t count)
{
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	mca_set_wls_charger_thermal_remove(val);

	return count;
}
static CLASS_ATTR_RW(wls_thermal_remove);

static struct attribute *mca_qcom_class_attrs[] = {
	&class_attr_real_type.attr,
	&class_attr_usb_real_type.attr,
	&class_attr_wireless_type.attr,
	&class_attr_quick_charge_type.attr,
	&class_attr_authentic.attr,
	&class_attr_slave_authentic.attr,
	&class_attr_pd_verifed.attr,
	&class_attr_cc_toggle.attr,
	&class_attr_cid_status.attr,
	&class_attr_has_dp.attr,
	&class_attr_otg_ui_support.attr,
	&class_attr_soc_decimal.attr,
	&class_attr_soc_decimal_rate.attr,
	&class_attr_soh.attr,
	&class_attr_power_max.attr,
	&class_attr_pmic_ibat.attr,
	&class_attr_shipmode_count_reset.attr,
	&class_attr_dam_ovpgate.attr,
	&class_attr_rx_vout.attr,
	&class_attr_rx_vrect.attr,
	&class_attr_rx_iout.attr,
	&class_attr_rx_ss.attr,
	&class_attr_tx_adapter.attr,
	&class_attr_magnetic_case_flag.attr,
	&class_attr_hall_phone_case.attr,
	&class_attr_wireless_chip_fw.attr,
	&class_attr_reverse_chg_mode.attr,
	&class_attr_reverse_chg_state.attr,
	&class_attr_wls_thermal_remove.attr,
	NULL,
};
ATTRIBUTE_GROUPS(mca_qcom_class);

static int mca_qcom_sysfs_probe(struct platform_device *pdev)
{
	struct mca_qcom_sysfs *chip;
	int rc;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &pdev->dev;
	platform_set_drvdata(pdev, chip);

	chip->support_multi_typec =
		!!of_find_property(pdev->dev.of_node, "mi,support_multi_typec",
				   NULL);

	/*
	 * The name is what Xiaomi's userspace looks for under /sys/class, so
	 * it is theirs rather than anything descriptive.
	 */
	chip->mca_qcom_class.name = "qcom-battery";
	chip->mca_qcom_class.class_groups = mca_qcom_class_groups;

	rc = class_register(&chip->mca_qcom_class);
	if (rc) {
		mca_log_err("register class failed\n");
		return rc;
	}

	g_qcom_sysfs = chip;
	mca_log_info("probe ok\n");

	return 0;
}

static int mca_qcom_sysfs_remove(struct platform_device *pdev)
{
	struct mca_qcom_sysfs *chip = platform_get_drvdata(pdev);

	class_unregister(&chip->mca_qcom_class);
	g_qcom_sysfs = NULL;

	return 0;
}

static const struct of_device_id mca_qcom_sysfs_match[] = {
	{ .compatible = "mca,qcom_sysfs" },
	{ }
};
MODULE_DEVICE_TABLE(of, mca_qcom_sysfs_match);

static struct platform_driver mca_qcom_sysfs_driver = {
	.driver = {
		.name		= "mca_qcom_sysfs",
		.of_match_table	= mca_qcom_sysfs_match,
	},
	.probe	= mca_qcom_sysfs_probe,
	.remove	= mca_qcom_sysfs_remove,
};
module_platform_driver(mca_qcom_sysfs_driver);

MODULE_DESCRIPTION("MCA charging attributes for Xiaomi userspace");
MODULE_LICENSE("GPL");
