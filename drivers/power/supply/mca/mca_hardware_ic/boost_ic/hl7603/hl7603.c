// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2025 The LineageOS Project
 *
 * The HL7603 bypass boost.
 *
 * This part sits between the battery and the parts of the phone that need
 * more voltage than the cell can give at the end of its discharge -- the
 * display's own supplies, chiefly.  Most of the time it passes the battery
 * straight through, which costs nothing; only when the cell falls below the
 * threshold set here does it start switching.
 *
 * So the whole driver is that threshold.  Setting it too high means the boost
 * runs when it need not, which wastes power; too low and the display browns
 * out on a nearly flat battery before the phone has decided to shut down.
 *
 * A board with a high-brightness mode gets a second threshold, because the
 * display draws more there and its supplies sag further.
 */

#define MCA_LOG_TAG "boost_hl7603"

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <mca/common/mca_event.h>
#include <mca/common/mca_log.h>
#include <mca/common/mca_parse_dts.h>
#include <mca/common/mca_sysfs.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of.h>

/* The register holding the voltage the boost starts at. */
#define HL7603_REG_VOUT_THRESHOLD	0x02

/* The register the part identifies itself in. */
#define HL7603_REG_CHIP_ID		0x0a

/*
 * The threshold is stored as steps above a fixed floor.  Values outside the
 * range wrap rather than saturating, so they are refused.
 */
#define HL7603_VOUT_BASE_MV		2850
/*
 * Where the boost is set where the board does not say.  It is not the lowest
 * setting: the rail this feeds does not come up at 2.85 V.
 */
#define HL7603_VOUT_DEFAULT_MV		3400
#define HL7603_VOUT_STEP_MV		50
#define HL7603_VOUT_MAX_MV		5500

/* What the debug files are. */
enum boost_attr_list {
	BOOST_PROP_VOUT_THRESHOLD,
	BOOST_PROP_CHIP_ID,
	BOOST_PROP_ADDRESS,
	BOOST_PROP_COUNT,
	BOOST_PROP_DATA,
	BOOST_PROP_MAX,
};

/**
 * struct boost_bypass_dev - this driver's state
 * @dev:                this device
 * @client:             the part on its bus
 * @panel_nb:           hears the screen changing brightness mode
 * @suport_hbm:         the board has a high-brightness mode
 * @vout_threshold:     where the boost starts, in millivolts
 * @hbm_vout_threshold: where it starts in high-brightness mode
 * @address:            the register a debug read or write will use
 * @count:              how many registers it will cover
 */
struct boost_bypass_dev {
	struct device		*dev;
	struct i2c_client	*client;
	struct notifier_block	panel_nb;
	bool			suport_hbm;
	u32			vout_threshold;
	u32			hbm_vout_threshold;
	int			address;
	int			count;
};

/**
 * hl7603_set_voltage_threshold() - set where the boost starts
 * @chip:           this driver's state
 * @vout_threshold: the voltage, in millivolts
 *
 * The register counts fifty-millivolt steps above a fixed floor and wraps
 * rather than saturating, so a value outside the range would set the
 * threshold somewhere else entirely.  It is refused instead.
 *
 * Return: 0, or a negative error.
 */
static noinline int hl7603_set_voltage_threshold(struct boost_bypass_dev *chip,
						 u32 vout_threshold)
{
	int rc;
	u8 val;

	if (vout_threshold < HL7603_VOUT_BASE_MV ||
	    vout_threshold > HL7603_VOUT_MAX_MV) {
		mca_log_err("vout_threshold no valid %d", vout_threshold);
		return -1;
	}

	val = (vout_threshold - HL7603_VOUT_BASE_MV) / HL7603_VOUT_STEP_MV;

	rc = i2c_smbus_write_byte_data(chip->client, HL7603_REG_VOUT_THRESHOLD,
				       val);
	if (rc < 0) {
		mca_log_err("i2c read reg 0x%02X faild\n",
			    HL7603_REG_VOUT_THRESHOLD);

		return rc;
	}

	rc = i2c_smbus_read_byte_data(chip->client, HL7603_REG_VOUT_THRESHOLD);
	mca_log_info("i2c read reg [0x%02X]=[%d]\n", HL7603_REG_VOUT_THRESHOLD,
		     rc);

	return 0;
}

static ssize_t boost_debugfs_show(void *data, char *buf)
{
	struct mca_debugfs_attr_data *d = data;
	struct boost_bypass_dev *chip;
	int len = 0;
	int i, rc;

	if (!d || !d->private) {
		mca_log_err("null pointer show\n");
		return -EINVAL;
	}

	chip = d->private;

	switch (d->attr_info->debugfs_attr_name) {
	case BOOST_PROP_VOUT_THRESHOLD:
		return scnprintf(buf, PAGE_SIZE, "%d\n", chip->vout_threshold);
	case BOOST_PROP_CHIP_ID:
		rc = i2c_smbus_read_byte_data(chip->client, HL7603_REG_CHIP_ID);
		if (rc < 0) {
			mca_log_err("i2c read reg 0x%02X faild\n",
				    HL7603_REG_CHIP_ID);
			return rc;
		}
		return scnprintf(buf, PAGE_SIZE, "%02x\n", rc);
	case BOOST_PROP_ADDRESS:
		return scnprintf(buf, PAGE_SIZE, "%02x\n", chip->address);
	case BOOST_PROP_COUNT:
		return scnprintf(buf, PAGE_SIZE, "%d\n", chip->count);
	case BOOST_PROP_DATA:
		/*
		 * Dumping a run of registers rather than one at a time is what
		 * makes a bring-up problem visible: the interesting thing is
		 * usually which of several neighbouring registers is wrong.
		 */
		for (i = 0; i < chip->count; i++) {
			rc = i2c_smbus_read_byte_data(chip->client,
						      chip->address + i);
			if (rc < 0) {
				mca_log_err("i2c read reg 0x%02X faild\n",
					    chip->address + i);
				return rc;
			}
			len += scnprintf(buf + len, PAGE_SIZE - len,
					 "%02x: %02x\n", chip->address + i, rc);
		}
		return len;
	default:
		return -EINVAL;
	}
}

static ssize_t boost_debugfs_store(void *data, const char *buf, size_t count)
{
	struct mca_debugfs_attr_data *d = data;
	struct boost_bypass_dev *chip;
	int val;

	if (!d || !d->private) {
		mca_log_err("null pointer store\n");
		return -EINVAL;
	}

	chip = d->private;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	mca_log_info("write attr[%d]: %d\n",
		     d->attr_info->debugfs_attr_name, val);

	switch (d->attr_info->debugfs_attr_name) {
	case BOOST_PROP_VOUT_THRESHOLD:
		if (hl7603_set_voltage_threshold(chip, val)) {
			mca_log_err("BOOST_PROP_VOUT_THRESHOLD store fail\n");
			return -EINVAL;
		}
		chip->vout_threshold = val;
		break;
	case BOOST_PROP_ADDRESS:
		chip->address = val;
		break;
	case BOOST_PROP_COUNT:
		chip->count = val;
		break;
	case BOOST_PROP_DATA:
		if (i2c_smbus_write_byte_data(chip->client, chip->address, val))
			return -EIO;
		break;
	default:
		return -EINVAL;
	}

	return count;
}

static struct mca_debugfs_attr_info hl7603_debugfs_tbl[] = {
	{ "vout_threshold", 0644, BOOST_PROP_VOUT_THRESHOLD,
	  boost_debugfs_show, boost_debugfs_store },
	{ "chip_id", 0444, BOOST_PROP_CHIP_ID, boost_debugfs_show, NULL },
	{ "address", 0644, BOOST_PROP_ADDRESS, boost_debugfs_show,
	  boost_debugfs_store },
	{ "count", 0644, BOOST_PROP_COUNT, boost_debugfs_show,
	  boost_debugfs_store },
	{ "data", 0644, BOOST_PROP_DATA, boost_debugfs_show,
	  boost_debugfs_store },
};

/**
 * hl7603_panel_notifier_cb() - the screen's brightness mode changed
 * @nb:    the notifier
 * @event: what happened
 * @data:  unused
 *
 * A display in high-brightness mode draws more, so its supplies sag further
 * and the boost has to come in sooner.
 *
 * Return: NOTIFY_OK.
 */
static int hl7603_panel_notifier_cb(struct notifier_block *nb,
				    unsigned long event, void *data)
{
	struct boost_bypass_dev *chip =
		container_of(nb, struct boost_bypass_dev, panel_nb);

	if (event != MCA_EVENT_PANEL_HBM_STATE_CHANGE)
		return NOTIFY_OK;

	if (!chip->suport_hbm)
		return NOTIFY_OK;

	/*
	 * The state comes with the event rather than being read back: the
	 * panel driver has it, and asking it again would race with the next
	 * change.
	 */
	hl7603_set_voltage_threshold(chip,
				     *(int *)data ? chip->hbm_vout_threshold :
						    chip->vout_threshold);

	return NOTIFY_OK;
}

static int hl7603_parse_dt(struct boost_bypass_dev *chip)
{
	struct device_node *np = chip->dev->of_node;

	if (!np) {
		mca_log_err("device tree info missing\n");
		return -ENODEV;
	}

	mca_parse_dts_u32(np, "vout_threshold", &chip->vout_threshold,
			  HL7603_VOUT_DEFAULT_MV);

	chip->suport_hbm = !!of_find_property(np, "support_hbm", NULL);
	if (chip->suport_hbm)
		mca_parse_dts_u32(np, "hbm_vout_threshold",
				  &chip->hbm_vout_threshold,
				  HL7603_VOUT_DEFAULT_MV);

	return 0;
}

static int hl7603_probe(struct i2c_client *client)
{
	struct boost_bypass_dev *chip;
	int rc;

	mca_log_info("%s start probe\n", "hl7603");

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &client->dev;
	chip->client = client;
	i2c_set_clientdata(client, chip);

	rc = hl7603_parse_dt(chip);
	if (rc)
		return rc;

	rc = hl7603_set_voltage_threshold(chip, chip->vout_threshold);
	if (rc) {
		mca_log_err("failed to init hl7603\n");
		return rc;
	}

	mca_debugfs_create_group("hl7603", hl7603_debugfs_tbl,
				 ARRAY_SIZE(hl7603_debugfs_tbl), chip);

	if (chip->suport_hbm) {
		chip->panel_nb.notifier_call = hl7603_panel_notifier_cb;
		mca_event_block_notify_register(MCA_EVENT_TYPE_PANEL,
						&chip->panel_nb);
	}

	mca_log_err("probe success\n");

	return 0;
}

static void hl7603_remove(struct i2c_client *client)
{
	struct boost_bypass_dev *chip = i2c_get_clientdata(client);

	if (chip->suport_hbm)
		mca_event_block_notify_unregister(MCA_EVENT_TYPE_PANEL,
						  &chip->panel_nb);
}

static const struct of_device_id hl7603_match[] = {
	{ .compatible = "hl7603" },
	{ }
};
MODULE_DEVICE_TABLE(of, hl7603_match);

static const struct i2c_device_id hl7603_id[] = {
	{ "hl7603", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, hl7603_id);

static struct i2c_driver hl7603_driver = {
	.driver = {
		.name		= "hl7603",
		.of_match_table	= hl7603_match,
	},
	.probe		= hl7603_probe,
	.remove		= hl7603_remove,
	.id_table	= hl7603_id,
};
module_i2c_driver(hl7603_driver);

MODULE_DESCRIPTION("HL7603 bypass boost");
MODULE_LICENSE("GPL");
