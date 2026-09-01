// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2025 The LineageOS Project
 *
 * Proving a charger is what it says it is.
 *
 * Fast charging hands the battery far more power than the USB standard
 * describes, which is only safe with an adapter built for it.  Xiaomi's
 * adapters answer a challenge to prove they are; the exchange itself runs in
 * userspace, because it involves a key the kernel has no business holding.
 * This is where userspace reaches the port to run it, and where the verdict
 * comes back so the charging strategies can act on it.
 */

#define MCA_LOG_TAG "pd_auth"

#include <linux/device.h>
#include <linux/errno.h>
#include <mca/common/mca_charge_mievent.h>
#include <mca/common/mca_event.h>
#include <mca/common/mca_log.h>
#include <mca/platform/platform_buckchg_class.h>
#include <mca/protocol/protocol_pd_class.h>
#include <mca/common/mca_sysfs.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>

/* What userspace can read and drive while running the exchange. */
enum pd_auth_attr_list {
	PD_AUTH_NAME,
	PD_AUTH_REQUEST_VDM_CMD,
	PD_AUTH_CURRENT_STATE,
	PD_AUTH_ADAPTER_ID,
	PD_AUTH_ADAPTER_SVID,
	PD_AUTH_VERIFY_PROCESS,
	PD_AUTH_USBPD_VERIFIED,
	PD_AUTH_CURRENT_PR,
	PD_AUTH_IS_PD_ADAPTER,
	PD_AUTH_USBPD_DATA_ROLE,
};

/*
 * A Xiaomi adapter that failed to prove itself is worth recording, because
 * the phone will have charged slowly and the owner will want to know why.
 * Only a Xiaomi adapter, and only one actually delivering power: anything
 * else failing the exchange is expected rather than a fault.
 */
static void strategy_pd_auth_fail_report_dfx(void)
{
	int port = protocol_class_pd_get_port_num();
	int adapter_id = 0;
	int online = 0;
	int svid = 0;

	platform_class_buckchg_ops_get_online(MAIN_BUCK_CHARGER, &online);
	if (!online)
		return;

	protocol_class_pd_get_adapter_svid(port, &svid);
	if (svid != XIAOMI_USB_SVID)
		return;

	protocol_class_pd_get_adapter_id(port, &adapter_id);
	mca_charge_mievent_report(CHARGE_DFX_PD_AUTH_FAILED, &adapter_id, 1);
}

/**
 * struct pd_auth_strategy - where the exchange stands
 * @dev:                this device
 * @verify_porcess_end: the exchange has finished, one way or the other
 * @pd_verified_type:   what it concluded
 */
struct pd_auth_strategy {
	struct device	*dev;
	int		verify_porcess_end;
	int		pd_verified_type;
};

static ssize_t strategy_pd_auth_sysfs_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf);
static ssize_t strategy_pd_auth_sysfs_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count);

static struct mca_sysfs_attr_info pd_auth_sysfs_field_tbl[] = {
	mca_sysfs_attr_ro(strategy_pd_auth_sysfs, 0444, PD_AUTH_NAME, name),
	mca_sysfs_attr_rw(strategy_pd_auth_sysfs, 0644,
			  PD_AUTH_REQUEST_VDM_CMD, request_vdm_cmd),
	mca_sysfs_attr_ro(strategy_pd_auth_sysfs, 0444, PD_AUTH_CURRENT_STATE,
			  current_state),
	mca_sysfs_attr_ro(strategy_pd_auth_sysfs, 0444, PD_AUTH_ADAPTER_ID,
			  adapter_id),
	mca_sysfs_attr_ro(strategy_pd_auth_sysfs, 0444, PD_AUTH_ADAPTER_SVID,
			  adapter_svid),
	mca_sysfs_attr_rw(strategy_pd_auth_sysfs, 0644, PD_AUTH_VERIFY_PROCESS,
			  verify_process),
	mca_sysfs_attr_rw(strategy_pd_auth_sysfs, 0644, PD_AUTH_USBPD_VERIFIED,
			  verified),
	mca_sysfs_attr_ro(strategy_pd_auth_sysfs, 0444, PD_AUTH_CURRENT_PR,
			  current_pr),
	mca_sysfs_attr_ro(strategy_pd_auth_sysfs, 0444, PD_AUTH_IS_PD_ADAPTER,
			  is_pd_adapter),
	mca_sysfs_attr_rw(strategy_pd_auth_sysfs, 0644,
			  PD_AUTH_USBPD_DATA_ROLE, data_role),
};

static struct attribute *pd_auth_attrs[ARRAY_SIZE(pd_auth_sysfs_field_tbl) + 1];
static const struct attribute_group pd_auth_group = {
	.attrs = pd_auth_attrs,
};

/* How much of the adapter's answer is printed, and how much is decoded. */
#define PD_AUTH_VDM_STR_LEN	128
#define PD_AUTH_VDM_WORD_LEN	16
#define PD_AUTH_VDM_DATA_LEN	64
#define PD_AUTH_VDM_BUF_LEN	64

/* The answer to a command that carries nothing back. */
#define PD_AUTH_VDM_NO_DATA	"%d,Null\n"

/*
 * Commands outside the enumerated set that are still expected: the two ranges
 * userspace uses to drive the exchange itself, which carry no reply.
 */
#define PD_AUTH_VDM_CMD_EXT1_BEGIN	50
#define PD_AUTH_VDM_CMD_EXT1_END	59
#define PD_AUTH_VDM_CMD_EXT2_BEGIN	100
#define PD_AUTH_VDM_CMD_EXT2_END	109

/*
 * What came back from the last command sent to the adapter, written the way
 * the userspace side of the exchange expects to read it: the command it asked
 * about, then whatever that command answers with.
 */
static ssize_t strategy_pd_auth_get_vdm_cmd(int port, char *buf)
{
	struct usbpd_vdm_data vdm_data = { 0 };
	char digest[PD_AUTH_VDM_STR_LEN] = { 0 };
	char word[PD_AUTH_VDM_WORD_LEN];
	int cmd = 0;
	int i;

	protocol_class_pd_get_vdm_cmd(port, &cmd, &vdm_data);

	switch (cmd) {
	case USBPD_UVDM_CHARGER_VERSION:
		return snprintf(buf, PAGE_SIZE, "%d,%x\n", cmd,
				vdm_data.ta_version);
	case USBPD_UVDM_CHARGER_VOLTAGE:
		return snprintf(buf, PAGE_SIZE, "%d,%d\n", cmd,
				vdm_data.ta_voltage);
	case USBPD_UVDM_CHARGER_TEMP:
		return snprintf(buf, PAGE_SIZE, "%d,%d\n", cmd,
				vdm_data.ta_temp);
	case USBPD_UVDM_REVERSE_AUTHEN:
		return snprintf(buf, PAGE_SIZE, "%d,%d", cmd, vdm_data.reauth);
	case USBPD_UVDM_AUTHENTICATION:
		for (i = 0; i < USBPD_UVDM_SS_LEN; i++) {
			memset(word, 0, sizeof(word));
			snprintf(word, sizeof(word), "%08lx",
				 vdm_data.digest[i]);
			strlcat(digest, word, sizeof(digest));
		}
		break;
	default:
		if ((cmd >= PD_AUTH_VDM_CMD_EXT1_BEGIN &&
		     cmd <= PD_AUTH_VDM_CMD_EXT1_END) ||
		    (cmd >= PD_AUTH_VDM_CMD_EXT2_BEGIN &&
		     cmd <= PD_AUTH_VDM_CMD_EXT2_END))
			return snprintf(buf, PAGE_SIZE, PD_AUTH_VDM_NO_DATA,
					cmd);

		mca_log_err("feedbak cmd:%d is not support\n", cmd);
		break;
	}

	return snprintf(buf, PAGE_SIZE, "%d,%s\n", cmd, digest);
}

static ssize_t strategy_pd_auth_sysfs_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct pd_auth_strategy *chip = dev_get_drvdata(dev);
	struct mca_sysfs_attr_info *info;
	struct pd_pdo pdo[PD_PDO_MAX_NUM] = { 0 };
	int port = protocol_class_pd_get_port_num();
	u32 val32 = 0;
	int val = 0;
	int ret = 0;

	if (!chip)
		return -1;

	info = mca_sysfs_lookup_attr(attr->attr.name, pd_auth_sysfs_field_tbl,
				     ARRAY_SIZE(pd_auth_sysfs_field_tbl));
	if (!info)
		return -1;

	switch (info->sysfs_attr_name) {
	case PD_AUTH_REQUEST_VDM_CMD:
		return strategy_pd_auth_get_vdm_cmd(port, buf);
	case PD_AUTH_CURRENT_STATE:
		{
			char state[PD_AUTH_VDM_BUF_LEN] = { 0 };

			protocol_class_pd_get_current_state(port, state,
							    sizeof(state));

			return snprintf(buf, PAGE_SIZE, "%s\n", state);
		}
	case PD_AUTH_ADAPTER_ID:
		protocol_class_pd_get_adapter_id(port, &val32);

		return snprintf(buf, PAGE_SIZE, "%08x\n", val32);
	case PD_AUTH_ADAPTER_SVID:
		protocol_class_pd_get_adapter_svid(port, &val32);

		return snprintf(buf, PAGE_SIZE, "%04x\n", val32);
	case PD_AUTH_VERIFY_PROCESS:
		protocol_class_pd_get_verify_process(port, &val);

		return snprintf(buf, PAGE_SIZE, "%d\n", val);
	case PD_AUTH_USBPD_VERIFIED:
		protocol_class_pd_get_pd_verifed(port, &val);

		return snprintf(buf, PAGE_SIZE, "%d\n", val);
	case PD_AUTH_CURRENT_PR:
		/*
		 * The vendor writes "none" when the role cannot be read and
		 * then answers from the role anyway, which for an unread role
		 * is zero and so reads back as a sink.  Kept as it is: this
		 * is what userspace has always been given.
		 */
		if (protocol_class_pd_get_power_role(port, &val))
			ret = snprintf(buf, PAGE_SIZE, "none\n");

		if (val == PD_ROLE_SOURCE)
			return snprintf(buf, PAGE_SIZE, "source\n");
		if (val == PD_ROLE_SINK)
			return snprintf(buf, PAGE_SIZE, "sink\n");

		return ret;
	case PD_AUTH_IS_PD_ADAPTER:
		/*
		 * An adapter that offered nothing to draw from is not a
		 * Power Delivery adapter, whatever else it answered.
		 */
		protocol_class_pd_get_pdos(port, pdo, PD_PDO_MAX_NUM);

		return snprintf(buf, PAGE_SIZE, "%s\n",
				pdo[0].min_voltage ? "true" : "false");
	case PD_AUTH_USBPD_DATA_ROLE:
		protocol_class_pd_get_data_role(port, &val);

		switch (val) {
		case XM_REQUEST_PD_DR_DFP:
			return snprintf(buf, PAGE_SIZE, "dfp\n");
		case XM_REQUEST_PD_DR_UFP:
			return snprintf(buf, PAGE_SIZE, "ufp\n");
		default:
			return snprintf(buf, PAGE_SIZE, "unknown\n");
		}
	default:
		return 0;
	}
}

/* One hex digit, as the userspace side of the exchange writes them. */
static u8 pd_auth_hex(char c)
{
	return (c >= 'A' ? c + 9 : c) & 0xf;
}

/*
 * Send a command to the adapter.  What userspace writes is the command and,
 * for the commands that carry one, its argument as a hex string; the string
 * is turned back into the bytes the adapter is given.
 */
static int strategy_pd_auth_set_vdm_cmd(int port, const char *buf)
{
	u8 data[PD_AUTH_VDM_DATA_LEN] = { 0 };
	char buffer[PD_AUTH_VDM_BUF_LEN] = { 0 };
	int cmd = 0;
	int len;
	int i;

	if (!sscanf(buf, "%d,%s", &cmd, buffer))
		return -EINVAL;

	mca_log_info("buf:%s cmd:%d, buffer:%s\n", buf, cmd, buffer);

	len = strlen(buffer);
	for (i = 0; i + 1 < len && i / 2 < (int)sizeof(data); i += 2)
		data[i / 2] = (pd_auth_hex(buffer[i]) << 4) |
			      pd_auth_hex(buffer[i + 1]);

	/* An odd number of digits leaves one last half byte to carry. */
	if (len & 1)
		data[len / 2] = pd_auth_hex(buffer[len - 1]);

	return protocol_class_pd_request_vdm_cmd(port, cmd, (u32 *)data,
						 (len + 1) / 2);
}

static ssize_t strategy_pd_auth_sysfs_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct pd_auth_strategy *chip = dev_get_drvdata(dev);
	struct mca_sysfs_attr_info *info;
	int port = protocol_class_pd_get_port_num();
	int val = 0;

	if (!chip)
		return -1;

	info = mca_sysfs_lookup_attr(attr->attr.name, pd_auth_sysfs_field_tbl,
				     ARRAY_SIZE(pd_auth_sysfs_field_tbl));
	if (!info)
		return -1;

	switch (info->sysfs_attr_name) {
	case PD_AUTH_REQUEST_VDM_CMD:
		strategy_pd_auth_set_vdm_cmd(port, buf);
		break;
	case PD_AUTH_VERIFY_PROCESS:
		if (sscanf(buf, "%d", &val) != 1) {
			mca_log_err("verify process value invalid %s\n", buf);
			return -EINVAL;
		}

		protocol_class_pd_set_verify_process(port, val);
		chip->verify_porcess_end = val;

		/*
		 * The exchange ending is what the charging strategies are
		 * waiting for: until it does, a charger that would support
		 * fast charging is treated as if it might not.
		 */
		if (!val)
			mca_event_block_notify(MCA_EVENT_TYPE_CHARGE_TYPE,
					       MCA_EVENT_CHARGE_VERIFY_PROCESS_END,
					       &chip->verify_porcess_end);
		break;
	case PD_AUTH_USBPD_VERIFIED:
		if (sscanf(buf, "%d", &val) != 1) {
			mca_log_err("verified value invalid %s\n", buf);
			return -EINVAL;
		}

		mca_log_info("set pd verified %d\n", val);
		protocol_class_pd_set_pd_verifed(port, val);

		if (!val) {
			strategy_pd_auth_fail_report_dfx();
			break;
		}

		chip->pd_verified_type = XM_CHARGER_TYPE_PD_VERIFY;
		mca_event_block_notify(MCA_EVENT_TYPE_CHARGE_TYPE,
				       MCA_EVENT_CHARGE_TYPE_CHANGE,
				       &chip->pd_verified_type);
		break;
	case PD_AUTH_USBPD_DATA_ROLE:
		mca_log_info("set data_role: %s\n", buf);

		if (!strncmp(buf, "ufp", 3))
			val = XM_REQUEST_PD_DR_UFP;
		else if (!strncmp(buf, "dfp", 3))
			val = XM_REQUEST_PD_DR_DFP;
		else
			return -EINVAL;

		protocol_class_pd_request_vdm_cmd(TYPEC_PORT_0,
						  USBPD_UVDM_REQUEST_PD_DR,
						  &val, sizeof(val));
		break;
	default:
		break;
	}

	return count;
}

static int strategy_pd_auth_probe(struct platform_device *pdev)
{
	struct pd_auth_strategy *chip;

	mca_log_info("probe begin\n");

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		mca_log_err("out of memory\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, chip);
	chip->dev = &pdev->dev;

	/*
	 * Nothing has been proved yet, and nothing is being proved either:
	 * until userspace starts an exchange there is no process to wait for.
	 */
	chip->verify_porcess_end = 1;

	mca_sysfs_init_attrs(pd_auth_attrs, pd_auth_sysfs_field_tbl,
			     ARRAY_SIZE(pd_auth_sysfs_field_tbl));

	mca_sysfs_create_link_group(MCA_SYSFS_DEV_TYPEC, "strategy_pd_auth",
				    &pdev->dev, &pd_auth_group);

	mca_log_err("probe end\n");

	return 0;
}

static int strategy_pd_auth_remove(struct platform_device *pdev)
{
	mca_sysfs_remove_link_group(MCA_SYSFS_DEV_TYPEC, "strategy_pd_auth",
				    &pdev->dev, &pd_auth_group);

	return 0;
}

static void strategy_pd_auth_shutdown(struct platform_device *pdev)
{
}

static const struct of_device_id match_table[] = {
	{ .compatible = "mca,strategy_pd_auth" },
	{ }
};
MODULE_DEVICE_TABLE(of, match_table);

static struct platform_driver strategy_pd_auth_driver = {
	.driver = {
		.name		= "strategy_pd_auth",
		.of_match_table	= match_table,
	},
	.probe		= strategy_pd_auth_probe,
	.remove		= strategy_pd_auth_remove,
	.shutdown	= strategy_pd_auth_shutdown,
};
module_platform_driver(strategy_pd_auth_driver);

MODULE_DESCRIPTION("MCA Power Delivery adapter authentication");
MODULE_LICENSE("GPL");
