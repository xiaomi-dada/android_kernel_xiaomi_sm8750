// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2025 The LineageOS Project
 *
 * The SC8581 charge pump.
 *
 * A charge pump charges a battery at high current without turning the
 * difference between the input and the cell into heat: it switches a pair of
 * capacitors so the output is a fixed fraction of the input -- a half or a
 * quarter -- and passes almost all of the power through.  That is what makes
 * a hundred-watt charger possible in a phone.
 *
 * The price is that the fraction is fixed.  A pump running at one half needs
 * its input at twice the cell voltage and will not work far from it, which is
 * why the charging strategy spends its time walking the adapter's voltage
 * around: this driver only does what it is told and reports what it sees.
 *
 * It also watches for the ways this can go wrong.  A pump has capacitors
 * carrying the full input current at a switching frequency, and a fault in
 * one is not something to recover from gently: over-voltage on either side,
 * over-current on the bus, a capacitor that failed to charge at start-up.
 * Each of those raises an interrupt, and each is reported rather than
 * retried.
 *
 * The same part also carries the gates between the inputs and the battery,
 * which is why the wireless and reverse-charging paths reach into it.
 */

#define MCA_LOG_TAG "sc8581"

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <mca/common/mca_event.h>
#include <mca/common/mca_hwid.h>
#include <mca/common/mca_log.h>
#include <mca/common/mca_parse_dts.h>
#include <mca/platform/platform_cp_class.h>
#include <mca/common/mca_sysfs.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/pm_wakeup.h>
#include <linux/workqueue.h>

/* The registers this driver uses. */
#define SC8581_REG_BAT_OVP		0x01
#define SC8581_REG_BAT_OVP_ALM		0x02
#define SC8581_REG_WPC_OVP		0x03
#define SC8581_REG_OUT_OVP		0x04
#define SC8581_REG_BUS_OCP		0x05
#define SC8581_REG_BUS_OVP		0x06
#define SC8581_REG_BUS_UCP		0x07
#define SC8581_REG_USB_OVP		0x08
#define SC8581_REG_PMID2OUT		0x09
#define SC8581_REG_ERROR_HL		0x0a
#define SC8581_REG_CTRL			0x0b
#define SC8581_REG_FSW			0x0c
#define SC8581_REG_TIMEOUT		0x0d
#define SC8581_REG_MODE			0x0e
#define SC8581_REG_GATE_STATUS		0x0f
#define SC8581_REG_INT_STAT		0x10
#define  SC8581_VUSB_PRESENT		BIT(0)
#define SC8581_REG_INT_FLAG		0x11
#define SC8581_REG_ADC_CTRL		0x15
#define SC8581_REG_ADC_EN		0x16
#define SC8581_REG_ADC_BASE		0x17
#define SC8581_REG_TDIE_HI		0x27
#define SC8581_REG_TDIE_LO		0x28
#define SC8581_REG_RCP			0x40
#define SC8581_REG_ACDRV			0x42
/* Both gates driven, from software, at the strongest setting. */
#define  SC8581_ACDRV_INIT		0x82
#define SC8581_REG_DEVICE_ID		0x6e
#define SC8581_REG_BAT_OVP2		0x6c
#define SC8581_REG_BAT_OVP2_ALM		0x6d
#define SC8581_REG_DEGLITCH		0x70
#define SC8581_REG_KEY			0x76
#define SC8581_REG_TRIM			0x7c

/*
 * The thresholds the board writes in millivolts have to be turned into the
 * step each register counts in.  Anything below the base cannot be asked
 * for, so it is clamped rather than wrapping round to the top of the range.
 */
#define SC8581_BAT_OVP_BASE_MV		4450
#define SC8581_BAT_OVP_STEP_MV		25
#define SC8581_BAT_OVP_MASK		GENMASK(4, 0)

#define SC8581_PMID2OUT_UVP_BASE_MV	100
#define SC8581_PMID2OUT_UVP_STEP_MV	50
#define SC8581_PMID2OUT_UVP_MASK	GENMASK(2, 0)

#define SC8581_WPC_OVP_BASE_MV		11000
#define SC8581_WPC_OVP_STEP_MV		1000
#define SC8581_WPC_OVP_MASK		GENMASK(3, 0)

/*
 * A pump running below the bottom of the range is not protected at all, so
 * the lowest setting is used as "off" instead.
 */
#define SC8581_WPC_OVP_MIN_MV		6500
#define SC8581_WPC_OVP_DISABLED		15

/* How many times to write the whole configuration before giving up. */
#define SC8581_INIT_RETRIES		5

/* How many divisions the part offers, and so how wide the tables are. */
#define SC8581_MODE_COUNT		3

/*
 * The input thresholds count from the bottom of each register's range in
 * its own step; the bus one counts from whatever that division's floor is.
 */
#define SC8581_BUS_OVP_STEP_MV		150
#define SC8581_BUS_OVP_MASK		GENMASK(4, 0)
#define SC8581_USB_OVP_BASE_MV		2100
#define SC8581_USB_OVP_STEP_MV		400
#define SC8581_USB_OVP_MASK		GENMASK(2, 0)
#define SC8581_BUS_OCP_BASE_MA		2500
#define SC8581_BUS_OCP_STEP_MA		250
#define SC8581_BUS_OCP_MASK		GENMASK(4, 0)

/* The switching frequency, and what the register counts it in. */
#define SC8581_FSW_MIN_KHZ		300
#define SC8581_FSW_MAX_KHZ		1075
#define SC8581_FSW_STEP_KHZ		25
/* The SC8585 switches over a wider range, in coarser steps. */
#define SC8585_FSW_MIN_KHZ		180
#define SC8585_FSW_MAX_KHZ		1080
#define SC8585_FSW_STEP_KHZ		60
#define SC8581_FSW_DEFAULT_KHZ		600

/* What the watchdog register is set to: the longest it offers. */
#define SC8581_TIMEOUT_DEFAULT		0x28

/* SC8581_REG_BUS_UCP. */
#define SC8581_BUS_UCP_DIS		BIT(7)
#define SC8581_BUS_UCP_RISE_MASK	BIT(3)
#define SC8581_BUS_UCP_FALL_MASK	BIT(1)

/* SC8581_REG_CTRL. */
#define SC8581_CHG_EN			BIT(7)
#define SC8581_QB_EN			BIT(6)
#define SC8581_WPCGATE_EN		BIT(4)
#define SC8581_OVPGATE_EN		BIT(3)

/* How many times the pump is asked to turn around before giving up. */
#define SC8581_SET_REVCHG_RETRY		10

/* SC8581_REG_FSW: the switching frequency, in bits 6:3. */
#define SC8581_FSW_MASK			GENMASK(6, 3)
#define SC8581_FSW_SHIFT		3

/* SC8581_REG_MODE: how the pump divides, in the low three bits. */
#define SC8581_MODE_MASK		GENMASK(2, 0)

/* SC8581_REG_ADC_CTRL. */
#define SC8581_ADC_EN			BIT(7)

/* One ADC channel is a word, and the channels are consecutive. */
#define SC8581_ADC_REG(ch)		(SC8581_REG_ADC_BASE + (ch) * 2)

/*
 * Some registers are only writable once the part has been unlocked, which
 * takes this sequence written to the key register.  It exists so that a
 * glitch on the bus cannot change the thresholds that protect the cell.
 */
static const u8 sc8581_unlock_key[] = { 'D', 'A', 'G', 'A', 'M', 'A' };

/* How long a fault keeps the phone awake for, so the work can read it out. */
/* What each protection is set to where the board does not say. */
#define SC8581_BAT_OVP_DEFAULT_MV	4500
#define SC8581_BAT_OVP_ALARM_DEFAULT_MV	4400
#define SC8581_WPC_OVP_DEFAULT_MV	22000
#define SC8581_OUT_OVP_DEFAULT_MV	5000
#define SC8581_PMID2OUT_UVP_DEFAULT_MV	100
#define SC8581_PMID2OUT_OVP_DEFAULT_MV	600

#define SC8581_IRQ_WAKE_MS		500

/* The part identifies itself with one of these in SC8581_REG_DEVICE_ID. */
#define SC8581_DEVICE_ID_SC8561		0x61
#define SC8581_DEVICE_ID_SC8581		0x81
#define SC8581_DEVICE_ID_SC8585		0x85

/* How long the gate is given to settle before it is looked at. */
#define SC8581_GATE_SETTLE_MS		30

/* And how many times to insist before accepting that it will not move. */
#define SC8581_GATE_RETRIES		10
/* And how long to wait before asking again. */
#define SC8581_GATE_RETRY_MS		10

/* How long the part takes to leave low-power mode once the pin is raised. */
#define SC8581_NLPM_SETTLE_MS		400

/* How hard to look for the part before deciding the board has none. */
#define SC8581_DETECT_RETRIES		5
#define SC8581_DETECT_RETRY_MS		100

/* What the ADC measures, in the order the registers appear. */
enum sc8581_adc_ch {
	ADC_IBUS,
	ADC_VBUS,
	ADC_VUSB,
	ADC_VWPC,
	ADC_VOUT,
	ADC_VBAT,
	ADC_IBAT,
	ADC_TBAT,
	ADC_TDIE,
	ADC_MAX_NUM,
};

/* What the debug files are.  These are this driver's own, not the class's. */
enum sc8581_debugfs_attr {
	SC8581_DEBUGFS_ADDRESS,
	SC8581_DEBUGFS_COUNT,
	SC8581_DEBUGFS_DATA,
};

/**
 * struct sc8581_cfg - the thresholds the board asks for
 * @bat_ovp_th:       the cell voltage the pump shuts down at
 * @bat_ovp_alarm_th: the one it warns at first
 * @wpc_ovp_th:       the same for the wireless input
 * @out_ovp_th:       for the pump's own output
 * @pmid2out_uvp_th:  how far the flying capacitor may sag
 * @pmid2out_ovp_th:  and how far it may rise
 * @bus_ovp_th:       the input voltage limit, per division
 * @usb_ovp_th:       the USB input limit, per division
 * @bus_ocp_th:       the input current limit, per division
 *
 * The three that are per division are exactly the ones that depend on it: at
 * one quarter the input is twice the voltage and half the current of one
 * half, so a single figure would either be wrong or useless.
 */
struct sc8581_cfg {
	u32	bat_ovp_th;
	u32	bat_ovp_alarm_th;
	u32	wpc_ovp_th;
	u32	out_ovp_th;
	u32	pmid2out_uvp_th;
	u32	pmid2out_ovp_th;
	u32	bus_ovp_th[3];
	u32	usb_ovp_th[3];
	u32	bus_ocp_th[3];
};

/**
 * struct sc8581_fsw_cfg - the switching frequency window
 * @max_khz:  the highest the part will switch at
 * @min_khz:  the lowest, which is also what its register counts from
 * @step_khz: what one register count is worth
 *
 * Each part has its own window, so this is filled in once the part has said
 * which one it is rather than being fixed at build time.
 */
struct sc8581_fsw_cfg {
	u32	max_khz;
	u32	min_khz;
	u32	step_khz;
};

/**
 * struct sc8581_device - this driver's state
 * @client:          the part on its bus
 * @dev:             this device
 * @cfg:             the thresholds the board asks for
 * @fsw_cfg:         the switching frequency window this part offers
 * @chip_ok:         the part answered and identified itself
 * @log_tag:         which pump this is, for the log
 * @work_mode:       the division it is set to
 * @operation_mode:  forward or reverse
 * @chip_vendor:     which part it turned out to be
 * @adc_mode:        which channels the ADC is converting
 * @product_cfg:     what the part reported as its identity
 * @cp_role:         whether this is the first pump or the second
 * @irq_handle_work: acts on an interrupt
 * @irq_gpio:        the pin the part raises
 * @nlpm_gpio:       the pin that takes it out of low-power mode
 * @ovpgate_en:      the gate in front of the input is open
 * @i2c_is_working:  the bus is answering
 * @support_reverse_quick_charge: the board can feed a device quickly
 */
struct sc8581_device {
	struct i2c_client	*client;
	struct device		*dev;
	struct sc8581_cfg	cfg;
	struct sc8581_fsw_cfg	fsw_cfg;
	bool			chip_ok;
	char			log_tag[24];
	int			work_mode;
	int			operation_mode;
	int			chip_vendor;
	u8			adc_mode;
	u32			product_cfg;
	int			cp_role;
	struct delayed_work	irq_handle_work;
	int			irq_gpio;
	int			nlpm_gpio;
	bool			ovpgate_en;
	bool			i2c_is_working;
	bool			support_reverse_quick_charge;

	/* Set by the debug files, and used only by them. */
	int			address;
	int			count;
};

/*
 * Which set of per-division thresholds each operating mode uses.  Forward
 * and reverse at the same division see the same voltages, so they share.
 */
static const u8 sc8581_mode_work_mode[CP_MODE_MAX] = {
	[CP_MODE_FORWARD_4_1]	= 0,
	[CP_MODE_FORWARD_2_1]	= 1,
	[CP_MODE_FORWARD_1_1]	= 2,
	[CP_MODE_RESERVED_3]	= 2,
	[CP_MODE_REVERSE_1_4]	= 0,
	[CP_MODE_REVERSE_1_2]	= 1,
	[CP_MODE_REVERSE_1_1]	= 2,
	[CP_MODE_RESERVED_7]	= 2,
};

static int sc8581_init_bit(struct sc8581_device *chip, u8 reg, u8 bit,
			   bool set);
static int sc8581_set_busovp_th(struct sc8581_device *chip, int mode);
static int sc8581_set_pmid2outuvp_th(int th, void *data);
static int sc8581_enable_busucp(struct sc8581_device *chip, bool enable);

static int cp_read_byte(struct sc8581_device *chip, u8 reg, u8 *val)
{
	int rc = i2c_smbus_read_byte_data(chip->client, reg);

	if (rc < 0) {
		chip->i2c_is_working = false;
		mca_log_err("read reg 0x%02x failed: %d\n", reg, rc);
		return rc;
	}

	chip->i2c_is_working = true;
	*val = rc;

	return 0;
}

static int cp_write_byte(struct sc8581_device *chip, u8 reg, u8 val)
{
	int rc = i2c_smbus_write_byte_data(chip->client, reg, val);

	if (rc < 0) {
		chip->i2c_is_working = false;
		mca_log_err("write reg 0x%02x failed: %d\n", reg, rc);
		return rc;
	}

	chip->i2c_is_working = true;

	return 0;
}

static int cp_update_bits(struct sc8581_device *chip, u8 reg, u8 mask, u8 val)
{
	u8 tmp;
	int rc;

	rc = cp_read_byte(chip, reg, &tmp);
	if (rc)
		return rc;

	tmp = (tmp & ~mask) | (val & mask);

	return cp_write_byte(chip, reg, tmp);
}

static int cp_read_word(struct sc8581_device *chip, u8 reg, u16 *val)
{
	int rc = i2c_smbus_read_word_data(chip->client, reg);

	if (rc < 0) {
		chip->i2c_is_working = false;
		return rc;
	}

	chip->i2c_is_working = true;

	/* The part sends the high byte first; the bus hands them over the
	 * other way round.
	 */
	*val = be16_to_cpu((__force __be16)rc);

	return 0;
}

/**
 * sc8581_set_key() - unlock the protected registers
 * @chip: this driver's state
 *
 * The thresholds that shut the pump down are behind a key sequence, so that a
 * glitch on the bus cannot raise them.  Writing anything else in between
 * relocks the part.
 *
 * Return: 0, or a negative error.
 */
static int sc8581_set_key(struct sc8581_device *chip, bool enable)
{
	unsigned int i;
	u8 val = 0;
	int rc;

	rc = cp_read_byte(chip, SC8581_REG_KEY, &val);
	if (rc)
		return rc;

	mca_log_info("register 0x76 = 0x%x\n", val);

	if (!enable) {
		mca_log_info("exit key mode\n");
		return 0;
	}

	for (i = 0; i < ARRAY_SIZE(sc8581_unlock_key); i++) {
		rc = cp_write_byte(chip, SC8581_REG_KEY,
				   sc8581_unlock_key[i]);
		if (rc)
			return rc;
	}

	mca_log_info("enter key mode\n");

	return 0;
}

/*
 * Below the base the register cannot express the threshold at all, so the
 * board asking for less gets the lowest the part offers rather than the
 * wrapped-round value the arithmetic would otherwise produce.
 */
static u8 sc8581_threshold_val(int th_mv, int base_mv, int step_mv, u8 mask)
{
	if (th_mv <= base_mv)
		return 0;

	return min_t(unsigned int, (th_mv - base_mv) / step_mv, mask);
}

/*
 * What one ADC count is worth on each channel, as a fraction.  Voltages come
 * out in millivolts, currents in milliamps and temperatures in tenths of a
 * degree, which is what the strategies above expect.
 */
static const struct {
	u32 num;
	u32 den;
} sc8581_adc_scale[ADC_MAX_NUM] = {
	[ADC_IBUS] = { 15625, 10000 },		/* 1.5625 mA */
	[ADC_VBUS] = { 625, 100 },		/* 6.25 mV */
	[ADC_VUSB] = { 625, 100 },
	[ADC_VWPC] = { 625, 100 },
	[ADC_VOUT] = { 125, 100 },		/* 1.25 mV */
	[ADC_VBAT] = { 125, 100 },
	[ADC_IBAT] = { 3125, 1000 },		/* 3.125 mA */
	[ADC_TBAT] = { 9766, 100000 },
	[ADC_TDIE] = { 1, 2 },			/* 0.5 degrees */
};

/* The SC8585 counts bus current in larger steps than its siblings. */
#define SC8585_IBUS_SCALE_NUM	1875
#define SC8585_IBUS_SCALE_DEN	1000

/**
 * cp_get_adc_data() - read one of the pump's measurements
 * @chip: this driver's state
 * @ch:   which channel
 * @val:  filled in with the reading, in the channel's unit
 *
 * The two ADC control registers are read first because the part is known to
 * drop out of conversion on its own, and their values are the only way to see
 * that from a log.  The conversion is read either way: a stale count is still
 * closer to the truth than refusing to answer, which the strategies read as
 * the pump having gone away.
 *
 * Return: 0, or a negative error.
 */
static int cp_get_adc_data(struct sc8581_device *chip, enum sc8581_adc_ch ch,
			   int *val)
{
	u8 ctrl = 0, en = 0;
	u32 num, den;
	u16 raw = 0;
	int rc;

	if (ch >= ADC_MAX_NUM)
		return -EINVAL;

	(void)cp_read_byte(chip, SC8581_REG_ADC_CTRL, &ctrl);
	(void)cp_read_byte(chip, SC8581_REG_ADC_EN, &en);
	mca_log_info("adc ctrl 0x%02X en 0x%02X ch %d\n", ctrl, en, ch);

	rc = cp_read_word(chip, SC8581_ADC_REG(ch), &raw);
	if (rc)
		return rc;

	num = sc8581_adc_scale[ch].num;
	den = sc8581_adc_scale[ch].den;
	if (ch == ADC_IBUS && chip->chip_vendor == SC8585_VENDOR) {
		num = SC8585_IBUS_SCALE_NUM;
		den = SC8585_IBUS_SCALE_DEN;
	}

	*val = (u32)raw * num / den;

	return 0;
}

static int ops_cp_enable_adc(bool enable, void *data)
{
	struct sc8581_device *chip = data;

	return cp_update_bits(chip, SC8581_REG_ADC_CTRL, SC8581_ADC_EN,
			      enable ? SC8581_ADC_EN : 0);
}

static noinline int sc8581_enable_charge(bool enable, void *data)
{
	struct sc8581_device *chip = data;

	return cp_update_bits(chip, SC8581_REG_CTRL, SC8581_CHG_EN,
			      enable ? SC8581_CHG_EN : 0);
}

static int ops_cp_get_charge_enable(bool *enabled, void *data)
{
	struct sc8581_device *chip = data;
	u8 val = 0;
	int rc;

	rc = cp_read_byte(chip, SC8581_REG_CTRL, &val);
	*enabled = !!(val & SC8581_CHG_EN);

	return rc;
}

/*
 * The bypass switch carries the input straight to the cell without the pump
 * running, which is what a low current wants: the pump's own losses are worse
 * than the switch's below a certain point.
 */
static noinline int sc8581_enable_qb(bool enable, void *data)
{
	struct sc8581_device *chip = data;

	return cp_update_bits(chip, SC8581_REG_CTRL, SC8581_QB_EN,
			      enable ? SC8581_QB_EN : 0);
}

static int ops_cp_enable_wpcgate(bool enable, void *data)
{
	struct sc8581_device *chip = data;

	return cp_update_bits(chip, SC8581_REG_CTRL, SC8581_WPCGATE_EN,
			      enable ? SC8581_WPCGATE_EN : 0);
}

/**
 * sc8581_enable_ovpgate() - open or close the gate in front of the input
 * @enable: which
 * @data:   this driver's state
 *
 * The gate takes a moment to move and the voltage behind it settles more
 * slowly still, so the wait is here rather than left to every caller.
 *
 * Return: 0, or a negative error.
 */
static int sc8581_enable_ovpgate(bool enable, void *data)
{
	struct sc8581_device *chip = data;
	int count;
	u8 val = 0;
	int rc;

	/*
	 * Record what the gate was asked for before touching it: the register
	 * dump compares the pin against this and puts it back if they have
	 * drifted apart, and it must not put back the previous answer while
	 * this one is still being written.
	 */
	chip->ovpgate_en = enable;

	/* A part whose bus has only just come back will not take a write. */
	if (!chip->i2c_is_working)
		msleep(SC8581_GATE_SETTLE_MS);

	rc = cp_update_bits(chip, SC8581_REG_CTRL, SC8581_OVPGATE_EN,
			    enable ? SC8581_OVPGATE_EN : 0);
	if (rc)
		return rc;

	/*
	 * The gate is a physical switch that does not always do as it is
	 * told the first time, and the rest of the stack goes on to assume
	 * it has, so it is read back rather than trusted.
	 */
	for (count = 0; count < SC8581_GATE_RETRIES; count++) {
		rc = cp_read_byte(chip, SC8581_REG_CTRL, &val);
		if (rc)
			return rc;

		if (!!(val & SC8581_OVPGATE_EN) == enable)
			break;

		mca_log_info("count %d, ovpgate_en %d, penable %d\n", count,
			     !!(val & SC8581_OVPGATE_EN), enable);
		mca_log_err("set cp ovpgate not effective, repeat set ovpgate\n");
		cp_update_bits(chip, SC8581_REG_CTRL, SC8581_OVPGATE_EN,
			       enable ? SC8581_OVPGATE_EN : 0);
		msleep(SC8581_GATE_RETRY_MS);
	}

	mca_log_info("enable %d\n", enable);

	return 0;
}

/*
 * The same, for a caller that has already decided the gate is where it
 * wants it: asking again costs a bus transaction and a settle time each.
 */
static int ops_cp_enable_ovpgate_with_check(int type_temp, bool enable,
					   void *data)
{
	struct sc8581_device *chip = data;

	if (chip->ovpgate_en == enable)
		return 0;

	return sc8581_enable_ovpgate(enable, data);
}

/*
 * Whether the input gate has been asked to open, which is the control bit --
 * not SC8581_REG_GATE_STATUS, which says whether it actually did.
 */
static int ops_cp_get_ovpgate_enable(bool *enable, void *data)
{
	struct sc8581_device *chip = data;
	u8 val = 0;
	int rc;

	rc = cp_read_byte(chip, SC8581_REG_CTRL, &val);
	*enable = !!(val & SC8581_OVPGATE_EN);

	return rc;
}

static int ops_cp_get_ovpgate_status(bool *status, void *data)
{
	struct sc8581_device *chip = data;
	u8 val = 0;
	int rc;

	rc = cp_read_byte(chip, SC8581_REG_GATE_STATUS, &val);
	*status = !!val;

	return rc;
}

/*
 * Changing the division changes every voltage in the pump at once, so the
 * thresholds are rewritten before the mode itself: setting the mode first
 * would leave the part protecting against the voltages of the old one for
 * as long as the writes take.
 */
static int sc8581_set_operation_mode(struct sc8581_device *chip, int mode)
{
	int rc;

	if (mode >= CP_MODE_MAX) {
		mca_log_info("operation mode error%d\n", mode);
		return -1;
	}

	chip->work_mode = sc8581_mode_work_mode[mode];

	/* Disable each protection while its own threshold is moving. */
	sc8581_init_bit(chip, SC8581_REG_BAT_OVP, BIT(7), false);
	if (chip->cp_role != CP_ROLE_SLAVE)
		sc8581_init_bit(chip, SC8581_REG_BAT_OVP_ALM, BIT(7), true);
	sc8581_init_bit(chip, SC8581_REG_BUS_OVP, BIT(7), false);
	sc8581_init_bit(chip, SC8581_REG_USB_OVP, BIT(7), false);
	sc8581_init_bit(chip, SC8581_REG_PMID2OUT, BIT(7), false);
	sc8581_init_bit(chip, SC8581_REG_BAT_OVP2, BIT(7), false);
	sc8581_init_bit(chip, SC8581_REG_BAT_OVP2_ALM, BIT(7), false);

	cp_update_bits(chip, SC8581_REG_BAT_OVP, SC8581_BAT_OVP_MASK,
		       sc8581_threshold_val(chip->cfg.bat_ovp_th,
					    SC8581_BAT_OVP_BASE_MV,
					    SC8581_BAT_OVP_STEP_MV,
					    SC8581_BAT_OVP_MASK));

	if (chip->cp_role != CP_ROLE_SLAVE)
		cp_update_bits(chip, SC8581_REG_BAT_OVP_ALM, GENMASK(3, 0), 4);

	cp_update_bits(chip, SC8581_REG_BAT_OVP2, SC8581_BAT_OVP_MASK,
		       sc8581_threshold_val(chip->cfg.bat_ovp_alarm_th,
					    SC8581_BAT_OVP_BASE_MV,
					    SC8581_BAT_OVP_STEP_MV,
					    SC8581_BAT_OVP_MASK));

	rc = sc8581_set_busovp_th(chip, chip->work_mode);
	if (rc)
		return rc;

	sc8581_enable_busucp(chip, true);

	/*
	 * A pad that cannot reach the bottom of the register's range is not
	 * protected by it at all, so it gets the setting that means "never"
	 * rather than one that would fire immediately.
	 */
	cp_update_bits(chip, SC8581_REG_WPC_OVP, SC8581_WPC_OVP_MASK,
		       chip->cfg.wpc_ovp_th <= SC8581_WPC_OVP_MIN_MV ?
		       SC8581_WPC_OVP_DISABLED :
		       sc8581_threshold_val(chip->cfg.wpc_ovp_th,
					    SC8581_WPC_OVP_BASE_MV,
					    SC8581_WPC_OVP_STEP_MV,
					    SC8581_WPC_OVP_MASK));

	cp_update_bits(chip, SC8581_REG_OUT_OVP, GENMASK(3, 0),
		       sc8581_threshold_val(chip->cfg.out_ovp_th, 4800, 100,
					    GENMASK(3, 0)));

	sc8581_set_pmid2outuvp_th(chip->cfg.pmid2out_uvp_th, chip);

	mca_log_info("set operation mode %d reg %d work_mode %d\n", mode, mode,
		     chip->work_mode);

	return cp_update_bits(chip, SC8581_REG_MODE, SC8581_MODE_MASK, mode);
}

static int ops_cp_set_mode(int mode, void *data)
{
	return sc8581_set_operation_mode(data, mode);
}

static int ops_cp_get_mode(int *mode, void *data)
{
	struct sc8581_device *chip = data;
	u8 val = 0;
	int rc;

	rc = cp_read_byte(chip, SC8581_REG_MODE, &val);
	*mode = val & SC8581_MODE_MASK;

	return rc;
}

/**
 * sc8581_set_fsw() - set the switching frequency
 * @fsw:  the frequency, as the part numbers them
 * @data: this driver's state
 *
 * A pump is most efficient at a frequency that depends on how much it is
 * passing, and the capacitors it drives are chosen for a particular one, so
 * this is the board's figure rather than a fixed value.
 *
 * Return: 0, or a negative error.
 */
static int sc8581_set_fsw(int fsw, void *data)
{
	struct sc8581_device *chip = data;
	int val;

	fsw = clamp_t(int, fsw, chip->fsw_cfg.min_khz, chip->fsw_cfg.max_khz);
	val = (fsw - chip->fsw_cfg.min_khz) / chip->fsw_cfg.step_khz;

	mca_log_info("fsw: %d, val: %d\n", fsw, val);

	return cp_update_bits(chip, SC8581_REG_FSW, SC8581_FSW_MASK,
			      val << SC8581_FSW_SHIFT);
}

static int ops_cp_get_fsw(int *fsw, void *data)
{
	struct sc8581_device *chip = data;
	u8 val = 0;
	int rc;

	rc = cp_read_byte(chip, SC8581_REG_FSW, &val);
	*fsw = (val & SC8581_FSW_MASK) >> SC8581_FSW_SHIFT;

	return rc;
}

/*
 * Put the switching frequency back where it started.  It is the one figure
 * the pump runs at unless a strategy has moved it to shift a whine out of the
 * audible band, so there is nothing per-division to restore.
 */
static int sc8581_set_default_fsw(void *data)
{
	return sc8581_set_fsw(SC8581_FSW_DEFAULT_KHZ, data);
}

static int ops_cp_get_int_stat(int stat, bool *val, void *data)
{
	struct sc8581_device *chip = data;
	u8 reg = 0;
	int rc;

	rc = cp_read_byte(chip, SC8581_REG_INT_STAT, &reg);
	*val = !!(reg & BIT(stat));

	return rc;
}

/*
 * Whether the flying capacitor is sitting above or below where it should.
 * Either says the pump is not switching cleanly, and which way round it is
 * says whether the input or the output is at fault.
 */
static int ops_cp_get_errorhl_stat(int *stat, void *data)
{
	struct sc8581_device *chip = data;
	u8 val = 0;
	int rc;

	rc = cp_read_byte(chip, SC8581_REG_ERROR_HL, &val);
	if (rc)
		return rc;

	if (val & BIT(0))
		*stat = CP_PMID_ERROR_LOW;
	else if (val & BIT(1))
		*stat = CP_PMID_ERROR_HIGH;
	else
		*stat = CP_PMID_ERROR_OK;

	return 0;
}

/*
 * Input under-current is how the pump notices the cable has been pulled out
 * from under it, so it is on whenever the pump is switching; it is turned
 * off only while the mode is changing, where the input legitimately drops.
 *
 * Turning it off means masking its two interrupts as well, or the drop that
 * made us turn it off raises one anyway.  The shipped driver writes all three
 * bits but passes the same unshifted value to each, so only the disable ever
 * lands and the interrupts stay live; write the bits it meant to.
 */
static noinline int sc8581_enable_busucp(struct sc8581_device *chip, bool enable)
{
	u8 mask = SC8581_BUS_UCP_DIS | SC8581_BUS_UCP_FALL_MASK |
		  SC8581_BUS_UCP_RISE_MASK;

	return cp_update_bits(chip, SC8581_REG_BUS_UCP, mask,
			      enable ? 0 : mask);
}

static int ops_cp_enable_busucp(bool enable, void *data)
{
	return sc8581_enable_busucp(data, enable);
}

static int ops_cp_set_rcp(bool enable, void *data)
{
	struct sc8581_device *chip = data;

	return cp_update_bits(chip, SC8581_REG_RCP, BIT(0), enable ? BIT(0) : 0);
}

static int ops_cp_set_adjustadble_timeout(int timeout, void *data)
{
	struct sc8581_device *chip = data;

	return cp_update_bits(chip, SC8581_REG_TIMEOUT, GENMASK(3, 0), timeout);
}

static int ops_enable_acdrv_manual(bool enable, void *data)
{
	struct sc8581_device *chip = data;

	return cp_update_bits(chip, SC8581_REG_CTRL, BIT(0),
			      enable ? BIT(0) : 0);
}

static int sc8581_set_pmid2outuvp_th(int th, void *data)
{
	struct sc8581_device *chip = data;
	u8 val;

	/*
	 * The SC8585 does not count this threshold the way its siblings do,
	 * so it is given the fixed high setting rather than a value measured
	 * against its own input.
	 */
	if (chip->chip_vendor == SC8585_VENDOR)
		val = th > 7 ? 5 : 0;
	else
		val = sc8581_threshold_val(th, SC8581_PMID2OUT_UVP_BASE_MV,
					   SC8581_PMID2OUT_UVP_STEP_MV,
					   SC8581_PMID2OUT_UVP_MASK);

	mca_log_info("cp set vout2out uvp high: %d\n", val);

	/*
	 * This threshold is one of the protected ones: the part is unlocked
	 * for the write and locked again straight after, so a stray write
	 * cannot leave the flying capacitor unguarded.
	 */
	sc8581_set_key(chip, true);
	cp_update_bits(chip, SC8581_REG_PMID2OUT, SC8581_PMID2OUT_UVP_MASK,
		       val);

	return sc8581_set_key(chip, false);
}

/**
 * ops_cp_get_tdie() - the pump's own temperature
 * @tdie: filled in with it
 * @data: this driver's state
 *
 * This is read as two registers rather than through the ADC, because it is
 * the reading that decides whether to stop and the ADC may be off.
 *
 * Return: 0, or a negative error.
 */
static int ops_cp_get_tdie(int *tdie, void *data)
{
	struct sc8581_device *chip = data;
	u8 hi = 0, lo = 0;
	int rc;

	rc = cp_read_byte(chip, SC8581_REG_TDIE_HI, &hi);
	if (rc)
		return rc;

	rc = cp_read_byte(chip, SC8581_REG_TDIE_LO, &lo);
	if (rc)
		return rc;

	*tdie = (hi << 8) | lo;

	return 0;
}

/*
 * Turning the pump around means the division, the input gate and the charge
 * enable all have to end up agreeing, and the part does not always take the
 * new mode on the first attempt.  Apply the settings, read them back, and try
 * again from a clean twenty milliseconds if they did not stick.
 */
static int ops_cp_set_revchg(bool enable, void *data)
{
	struct sc8581_device *chip = data;
	int retry;

	for (retry = 0; retry < SC8581_SET_REVCHG_RETRY; retry++) {
		int mode = 0;
		bool gate = false, charging = false;

		if (retry) {
			mca_log_info("%s failed set revchg, retry: %d\n",
				     chip->log_tag, retry);
			mdelay(20);
		}

		if (enable) {
			sc8581_set_operation_mode(chip, CP_MODE_REVERSE_1_2);
			sc8581_enable_ovpgate(true, chip);
		} else {
			sc8581_enable_charge(false, chip);
			sc8581_enable_qb(false, chip);
			sc8581_set_operation_mode(chip, CP_MODE_FORWARD_2_1);
		}

		if (ops_cp_get_mode(&mode, chip)) {
			mca_log_info("%s get operation mode fail\n", chip->log_tag);
			continue;
		}

		if (enable) {
			if (ops_cp_get_ovpgate_enable(&gate, chip)) {
				mca_log_info("%s get ovpgate enable fail\n",
					     chip->log_tag);
				continue;
			}
			mca_log_info("cp mode: %d, ovpgate_enable status: %d\n",
				     mode, gate);
			if (gate && mode == CP_MODE_REVERSE_1_2)
				return 0;
		} else {
			if (ops_cp_get_charge_enable(&charging, chip))
				continue;
			mca_log_info("cp mode: %d, charging_enable: %d\n",
				     mode, charging);
			if (mode == CP_MODE_FORWARD_2_1 && !charging)
				return 0;
		}
	}

	return -1;
}

/* The readings the strategy layer asks for, each one ADC channel. */

static int ops_cp_get_ibus(int *ibus, void *data)
{
	return cp_get_adc_data(data, ADC_IBUS, ibus);
}

static int ops_cp_get_vbus(int *vbus, void *data)
{
	return cp_get_adc_data(data, ADC_VBUS, vbus);
}

static int ops_cp_get_vusb(int *vusb, void *data)
{
	return cp_get_adc_data(data, ADC_VUSB, vusb);
}

static int ops_cp_get_vbatt(int *vbat, void *data)
{
	return cp_get_adc_data(data, ADC_VBAT, vbat);
}

static int ops_cp_get_ibatt(int *ibat, void *data)
{
	return cp_get_adc_data(data, ADC_IBAT, ibat);
}

static int ops_cp_get_battery_temmperature(int *tbat, void *data)
{
	return cp_get_adc_data(data, ADC_TBAT, tbat);
}


static int ops_cp_get_battery_present(bool *present, void *data)
{
	struct sc8581_device *chip = data;
	u8 val = 0;
	int rc;

	rc = cp_read_byte(chip, SC8581_REG_INT_STAT, &val);
	*present = !!val;

	return rc;
}

static int ops_cp_get_present(bool *present, void *data)
{
	struct sc8581_device *chip = data;

	*present = chip->chip_ok;

	return 0;
}

static int ops_cp_get_chip_vendor(int *chip_vendor, void *data)
{
	struct sc8581_device *chip = data;

	*chip_vendor = chip->chip_vendor;

	return 0;
}

/*
 * Whether the pump can bypass itself.  A part that cannot has to keep
 * switching even at currents where that costs more than it saves.
 */
static int ops_cp_get_bypass_support(bool *support, void *data)
{
	*support = true;

	return 0;
}


/**
 * sc8581_set_busovp_th() - set the input voltage the pump shuts down at
 * @chip: this driver's state
 * @mode: the division the threshold is for
 *
 * The threshold is per division because the input voltage is: a pump running
 * at one quarter sees twice what one at one half does for the same cell, so a
 * single figure would either trip on a working quarter or miss a runaway
 * half.
 *
 * Return: 0, or a negative error.
 */
/*
 * What the input may reach depends on the division, so the board gives a
 * figure for each: at one quarter the input is twice the voltage of one
 * half, and a single threshold would be either wrong or useless.  The part
 * itself only accepts a range, and asking outside it protects nothing, so
 * the board's figure is clamped rather than truncated.
 */
static const int sc8581_bus_ovp_min_mv[SC8581_MODE_COUNT] = {
	13300, 5500, 3500,
};
static const int sc8581_bus_ovp_max_mv[SC8581_MODE_COUNT] = {
	22000, 14000, 7000,
};

/* The SC8585 takes a little more on its input than its siblings do. */
static const int sc8581_bus_ovp_min_sc8585_mv[SC8581_MODE_COUNT] = {
	13300, 5500, 3750,
};
static const int sc8581_bus_ovp_max_sc8585_mv[SC8581_MODE_COUNT] = {
	22000, 15000, 7500,
};


static int sc8581_set_busovp_th(struct sc8581_device *chip, int mode)
{
	const int *min_mv, *max_mv;
	int th;
	int rc;

	if (mode >= SC8581_MODE_COUNT)
		return -EINVAL;

	if (chip->chip_vendor == SC8585_VENDOR) {
		min_mv = sc8581_bus_ovp_min_sc8585_mv;
		max_mv = sc8581_bus_ovp_max_sc8585_mv;
	} else {
		min_mv = sc8581_bus_ovp_min_mv;
		max_mv = sc8581_bus_ovp_max_mv;
	}

	th = chip->cfg.bus_ovp_th[mode];
	mca_log_info("threshold= %d, min = %d, max =%d\n", th, min_mv[mode],
		     max_mv[mode]);
	th = clamp(th, min_mv[mode], max_mv[mode]);

	rc = sc8581_set_key(chip, true);
	if (rc)
		return rc;

	mca_log_info("bus_ovpth= %d, val = %d\n", th,
		     (th - min_mv[mode]) / SC8581_BUS_OVP_STEP_MV);

	rc = cp_update_bits(chip, SC8581_REG_BUS_OVP, SC8581_BUS_OVP_MASK,
			    (th - min_mv[mode]) / SC8581_BUS_OVP_STEP_MV);
	if (rc)
		return rc;

	rc = cp_update_bits(chip, SC8581_REG_USB_OVP, SC8581_USB_OVP_MASK,
			    sc8581_threshold_val(chip->cfg.usb_ovp_th[mode],
						 SC8581_USB_OVP_BASE_MV,
						 SC8581_USB_OVP_STEP_MV,
						 SC8581_USB_OVP_MASK));
	if (rc)
		return rc;

	return cp_update_bits(chip, SC8581_REG_BUS_OCP, SC8581_BUS_OCP_MASK,
			      sc8581_threshold_val(chip->cfg.bus_ocp_th[mode],
						   SC8581_BUS_OCP_BASE_MA,
						   SC8581_BUS_OCP_STEP_MA,
						   SC8581_BUS_OCP_MASK));
}


/**
 * sc8581_init_device() - put the part into a known state
 * @chip: this driver's state
 *
 * The thresholds go in before anything else: a pump brought up with the
 * defaults would run without the protection the board's design assumes.
 *
 * Return: 0, or a negative error.
 */
/*
 * One read-modify-write of a single bit, with the register and bit named at
 * the call site so the sequence below reads as what it turns on and off
 * rather than as a wall of hex.
 */
static int sc8581_init_bit(struct sc8581_device *chip, u8 reg, u8 bit,
			   bool set)
{
	return cp_update_bits(chip, reg, bit, set ? bit : 0);
}

/*
 * What the part has to be told before it will switch.  Most of it is turning
 * off protections that fire on the transients of starting up, and turning on
 * the ones that matter once it is running; the parts that depend on which
 * pump this is are marked.
 *
 * The whole sequence is retried, because a pump that was still powering up
 * when the first write went out answers its bus but does not keep the value.
 */
static int sc8581_init_device(struct sc8581_device *chip)
{
	bool sc8585 = chip->chip_vendor == SC8585_VENDOR;
	int retry;
	int rc;

	/* Leave the low-power mode the part comes up in. */
	rc = sc8581_init_bit(chip, SC8581_REG_MODE, BIT(3), true);
	if (rc)
		return rc;
	rc = sc8581_init_bit(chip, SC8581_REG_MODE, BIT(7), false);
	if (rc)
		return rc;

	for (retry = 0; retry < SC8581_INIT_RETRIES; retry++) {
		/*
		 * The gates start closed: nothing is connected to the cell
		 * until the thresholds below have been written.
		 */
		cp_update_bits(chip, SC8581_REG_CTRL, GENMASK(5, 3),
			       GENMASK(5, 3));

		/* The watchdog, at the longest interval the part offers. */
		cp_write_byte(chip, SC8581_REG_TIMEOUT,
			      SC8581_TIMEOUT_DEFAULT);

		/*
		 * The input under-current protection is what tells the pump
		 * the cable has gone; it needs its slowest setting or a
		 * transient trips it.
		 */
		cp_update_bits(chip, SC8581_REG_BUS_UCP, GENMASK(5, 4),
			       BIT(4));

		if (!sc8585)
			sc8581_init_bit(chip, SC8581_REG_TRIM, BIT(3), true);

		sc8581_init_bit(chip, SC8581_REG_MODE, BIT(4), false);
		sc8581_init_bit(chip, SC8581_REG_DEGLITCH, BIT(3), false);

		/* Both flying-capacitor faults are latched, not ignored. */
		cp_update_bits(chip, SC8581_REG_FSW, GENMASK(1, 0),
			       GENMASK(1, 0));

		sc8581_init_bit(chip, SC8581_REG_WPC_OVP, BIT(7), false);

		/* Start the ADC from a known state, then enable channels. */
		sc8581_init_bit(chip, SC8581_REG_ADC_CTRL, BIT(6), false);
		sc8581_init_bit(chip, SC8581_REG_ADC_CTRL, BIT(0), false);

		sc8581_init_bit(chip, SC8581_REG_ADC_EN, BIT(7), false);
		sc8581_init_bit(chip, SC8581_REG_ADC_EN, BIT(6), false);
		sc8581_init_bit(chip, SC8581_REG_ADC_EN, BIT(5), false);
		sc8581_init_bit(chip, SC8581_REG_ADC_EN, BIT(4), false);
		sc8581_init_bit(chip, SC8581_REG_ADC_EN, BIT(3), false);
		/*
		 * The SC8585 leaves this channel alone: it is wired to
		 * something the others do not have.
		 */
		if (!sc8585)
			sc8581_init_bit(chip, SC8581_REG_ADC_EN, BIT(2), false);
		sc8581_init_bit(chip, SC8581_REG_ADC_EN, BIT(1), false);
		sc8581_init_bit(chip, SC8581_REG_ADC_EN, BIT(0), false);

		sc8581_init_bit(chip, SC8581_REG_ADC_CTRL, BIT(7), false);

		/* The second cell's over-voltage, on boards that have one. */
		cp_update_bits(chip, SC8581_REG_BAT_OVP2,
			       SC8581_BAT_OVP_MASK,
			       sc8581_threshold_val(chip->cfg.bat_ovp_th,
						    SC8581_BAT_OVP_BASE_MV,
						    SC8581_BAT_OVP_STEP_MV,
						    SC8581_BAT_OVP_MASK));
		cp_update_bits(chip, SC8581_REG_BAT_OVP2_ALM,
			       SC8581_BAT_OVP_MASK,
			       sc8581_threshold_val(chip->cfg.bat_ovp_alarm_th,
						    SC8581_BAT_OVP_BASE_MV,
						    SC8581_BAT_OVP_STEP_MV,
						    SC8581_BAT_OVP_MASK));

		rc = sc8581_set_operation_mode(chip, CP_MODE_FORWARD_2_1);
		if (rc)
			continue;

		/*
		 * The AC gate is armed differently on each part: the SC8581
		 * has a bit for it beside the gate status, the SC8585 takes
		 * the whole setting as one write, and the SC8561 has neither.
		 */
		if (chip->chip_vendor == SC8581_VENDOR)
			sc8581_init_bit(chip, SC8581_REG_GATE_STATUS, BIT(5),
					true);
		else if (sc8585)
			cp_write_byte(chip, SC8581_REG_ACDRV,
				      SC8581_ACDRV_INIT);
		sc8581_init_bit(chip, SC8581_REG_TRIM, BIT(0), true);

		if (sc8585) {
			chip->fsw_cfg.max_khz = SC8585_FSW_MAX_KHZ;
			chip->fsw_cfg.min_khz = SC8585_FSW_MIN_KHZ;
			chip->fsw_cfg.step_khz = SC8585_FSW_STEP_KHZ;
		} else {
			chip->fsw_cfg.max_khz = SC8581_FSW_MAX_KHZ;
			chip->fsw_cfg.min_khz = SC8581_FSW_MIN_KHZ;
			chip->fsw_cfg.step_khz = SC8581_FSW_STEP_KHZ;
		}

		rc = sc8581_set_fsw(SC8581_FSW_DEFAULT_KHZ, chip);
		if (!rc)
			return 0;
	}

	mca_log_err("failed init cp init device\n");

	return rc;
}

static int ops_cp_device_init(int device_init, void *data)
{
	return sc8581_init_device(data);
}

static int sc8581_dump_log_context(void *data, char *buf, int size)
{
	struct sc8581_device *chip = data;
	int vusb = 0, vbus = 0, ibus = 0, ibat = 0, vbat = 0, vout = 0;

	cp_get_adc_data(chip, ADC_VUSB, &vusb);
	cp_get_adc_data(chip, ADC_VBUS, &vbus);
	cp_get_adc_data(chip, ADC_IBUS, &ibus);
	cp_get_adc_data(chip, ADC_IBAT, &ibat);
	cp_get_adc_data(chip, ADC_VBAT, &vbat);
	cp_get_adc_data(chip, ADC_VOUT, &vout);

	return scnprintf(buf, size, "%-8d%-8d%-8d%-8d%-8d%-8d", vusb, vbus,
			 ibus, ibat, vbat, vout);
}

/*
 * Which fault each bit of the status registers means.  A pump fault is not
 * something to recover from gently: whatever caused it is still there, and
 * switching again into a shorted capacitor or a cell that is already over
 * voltage makes it worse.  So each is reported and the stack decides.
 */
static const struct {
	u8	reg;
	u8	bit;
	int	event;
	const char *name;
} sc8581_faults[] = {
	{ SC8581_REG_INT_FLAG, BIT(0), MCA_EVENT_CP_VBAT_OVP,     "VBAT_OVP" },
	{ SC8581_REG_INT_FLAG, BIT(1), MCA_EVENT_CP_IBAT_OCP,     "IBAT_OCP" },
	{ SC8581_REG_INT_FLAG, BIT(2), MCA_EVENT_CP_VBUS_OVP,     "VBUS_OVP" },
	{ SC8581_REG_INT_FLAG, BIT(3), MCA_EVENT_CP_IBUS_OCP,     "IBUS_OCP" },
	{ SC8581_REG_INT_FLAG, BIT(4), MCA_EVENT_CP_IBUS_UCP,     "IBUS_UCP" },
	{ SC8581_REG_INT_FLAG, BIT(5), MCA_EVENT_CP_PMID2OUT_OVP, "PMID2OUT_OVP" },
	{ SC8581_REG_INT_FLAG, BIT(6), MCA_EVENT_CP_PMID2OUT_UVP, "PMID2OUT_UVP" },
	{ SC8581_REG_INT_FLAG, BIT(7), MCA_EVENT_CP_TSHUT_FLAG,   "TSHUT" },
	{ SC8581_REG_ERROR_HL, BIT(0), MCA_EVENT_CP_VUSB_OVP,     "VUSB_OVP" },
	{ SC8581_REG_ERROR_HL, BIT(1), MCA_EVENT_CP_VWPC_OVP,     "VWPC_OVP" },
	{ SC8581_REG_ERROR_HL, BIT(2), MCA_EVENT_CP_VOUT_UVLO,    "VOUT UVLO" },
	{ SC8581_REG_ERROR_HL, BIT(3), MCA_EVENT_CP_POR_FLAG,     "POR_FLAG" },
	{ SC8581_REG_ERROR_HL, BIT(4), MCA_EVENT_CP_CBOOT_FAIL,   "CBOOT SHORT/OPEN 111" },
};

/**
 * sc8581_dump_important_regs() - read the fault registers and report them
 * @chip: this driver's state
 *
 * Reading the flags clears them, so this is the only place they are read:
 * a second reader would take faults away from this one.
 */
static int sc8581_dump_important_regs(struct sc8581_device *chip)
{
	u8 vals[SC8581_REG_INT_FLAG + 1] = { };
	unsigned int i;
	int rc;

	rc = i2c_smbus_read_i2c_block_data(chip->client, 0, sizeof(vals),
					   vals);
	if (rc < 0) {
		mca_log_err("dump registers failed, base_addr: 0x%02X\n", 0);
		return rc;
	}

	/*
	 * The gate can be knocked out from under us -- by a fault, or by the
	 * part resetting -- long after it was set.  Nothing else notices, so
	 * compare it against what it was last told and put it back.
	 */
	if (!!(vals[SC8581_REG_CTRL] & SC8581_OVPGATE_EN) != chip->ovpgate_en) {
		mca_log_info("set cp ovpgate not effective, repeat set ovpgate\n");
		sc8581_enable_ovpgate(chip->ovpgate_en, chip);
	}

	for (i = 0; i < ARRAY_SIZE(sc8581_faults); i++) {
		if (!(vals[sc8581_faults[i].reg] & sc8581_faults[i].bit))
			continue;

		mca_log_err("%s\n", sc8581_faults[i].name);
		mca_event_block_notify(MCA_EVENT_TYPE_CP_INFO,
				       sc8581_faults[i].event, NULL);
	}

	return 0;
}

static int ops_cp_dump_register(void *data)
{
	return sc8581_dump_important_regs(data);
}

static int sc8581_dump_log_head(void *data, char *buf, int size)
{
	struct sc8581_device *chip = data;

	/*
	 * The second pump's columns are named apart from the first's, since
	 * the log is one table and the two would otherwise be
	 * indistinguishable.
	 */
	if (chip->cp_role == CP_ROLE_MASTER)
		return scnprintf(buf, size,
				 "cp_vusb cp_vbus cp_ibus cp_ibat cp_vbat cp_vout ");

	return scnprintf(buf, size,
			 "cp_vusb1 cp_vbus1 cp_ibus1 cp_ibat1 cp_vbat1 cp_vout1 ");
}

static struct mca_log_charge_log_ops sc8581_log_ops = {
	.dump_log_head		= sc8581_dump_log_head,
	.dump_log_context	= sc8581_dump_log_context,
};

static int ops_cp_enable_charge(bool enable, void *data)
{
	return sc8581_enable_charge(enable, data);
}

static int ops_cp_enable_qb(bool enable, void *data)
{
	return sc8581_enable_qb(enable, data);
}

static int ops_cp_enable_ovpgate(bool enable, void *data)
{
	return sc8581_enable_ovpgate(enable, data);
}

static int ops_cp_set_fsw(int fsw, void *data)
{
	return sc8581_set_fsw(fsw, data);
}

static int ops_cp_set_default_fsw(void *data)
{
	return sc8581_set_default_fsw(data);
}

static int ops_cp_set_pmid2outuvp_th(int th, void *data)
{
	return sc8581_set_pmid2outuvp_th(th, data);
}

static int ops_cp_get_fsw_step(int *step, void *data)
{
	struct sc8581_device *chip = data;

	/*
	 * Asked before the pump has probed, this still has to answer: the
	 * strategy uses it to work out how far it may move the frequency.
	 */
	*step = SC8581_FSW_DEFAULT_KHZ;
	if (chip)
		*step = chip->fsw_cfg.step_khz;

	return 0;
}

static const struct platform_class_cp_ops sc8581_cp_ops = {
	.cp_set_enable			= ops_cp_enable_charge,
	.cp_get_enabled			= ops_cp_get_charge_enable,
	.cp_get_present			= ops_cp_get_present,
	.cp_get_battery_voltage		= ops_cp_get_vbatt,
	.cp_get_battery_current		= ops_cp_get_ibatt,
	.cp_get_battery_temperature	= ops_cp_get_battery_temmperature,
	.cp_get_battery_present		= ops_cp_get_battery_present,
	.cp_get_bus_voltage		= ops_cp_get_vbus,
	.cp_get_bus_current		= ops_cp_get_ibus,
	.cp_get_die_temperature		= ops_cp_get_tdie,
	.cp_get_usb_voltage		= ops_cp_get_vusb,
	.cp_enable_wpcgate		= ops_cp_enable_wpcgate,
	.cp_enable_ovpgate		= ops_cp_enable_ovpgate,
	.cp_enable_ovpgate_with_check	= ops_cp_enable_ovpgate_with_check,
	.cp_dump_register		= ops_cp_dump_register,
	.cp_get_fsw_step		= ops_cp_get_fsw_step,
	.cp_get_ovpgate_status		= ops_cp_get_ovpgate_status,
	.cp_set_mode			= ops_cp_set_mode,
	.cp_get_mode			= ops_cp_get_mode,
	.cp_device_init			= ops_cp_device_init,
	.cp_enable_adc			= ops_cp_enable_adc,
	.cp_get_bypass_support		= ops_cp_get_bypass_support,
	.cp_get_chip_vendor		= ops_cp_get_chip_vendor,
	.cp_enable_acdrv_manual		= ops_enable_acdrv_manual,
	.cp_set_adjustadble_timeout	= ops_cp_set_adjustadble_timeout,
	.cp_get_int_stat		= ops_cp_get_int_stat,
	.cp_get_errorhl_stat		= ops_cp_get_errorhl_stat,
	.cp_enable_busucp		= ops_cp_enable_busucp,
	.cp_set_fsw			= ops_cp_set_fsw,
	.cp_set_default_fsw		= ops_cp_set_default_fsw,
	.cp_get_fsw			= ops_cp_get_fsw,
	.cp_get_tdie			= ops_cp_get_tdie,
	.cp_set_qb			= ops_cp_enable_qb,
	.cp_set_pmid2outuvp_th		= ops_cp_set_pmid2outuvp_th,
	.cp_set_rcp			= ops_cp_set_rcp,
	.cp_set_revchg			= ops_cp_set_revchg,
};


/*
 * The pump sees VBUS before the PMIC does, because it sits on the port side of
 * the path, so the stack takes the cable coming and going from here.  Only the
 * edges are reported: the interrupt fires for every fault as well, and the
 * consumers act on each report.
 */
static void sc8581_irq_handler(struct work_struct *work)
{
	struct sc8581_device *chip = container_of(to_delayed_work(work),
						  struct sc8581_device,
						  irq_handle_work);
	static bool vusb_present;
	bool present;
	u8 stat = 0;

	if (cp_read_byte(chip, SC8581_REG_INT_STAT, &stat)) {
		/*
		 * A pump that has stopped answering its bus cannot be asked
		 * whether VBUS is still there.  If it was, say it has gone:
		 * leaving the stack believing in a supply nothing can see any
		 * more is the worse of the two answers.
		 */
		if (!vusb_present)
			goto out;
		present = false;
	} else {
		present = stat & SC8581_VUSB_PRESENT;
		if (present == vusb_present)
			goto out;
	}

	mca_event_block_notify(MCA_EVENT_TYPE_CP_INFO,
			       present ? MCA_EVENT_CP_VUSB_INSERT :
					 MCA_EVENT_CP_VUSB_OUT, NULL);
	vusb_present = present;
out:
	mca_log_info("%s handler\n", chip->log_tag);

	if (chip->i2c_is_working)
		sc8581_dump_important_regs(chip);
}

static irqreturn_t sc8581_int_isr(int irq, void *dev_id)
{
	struct sc8581_device *chip = dev_id;

	/*
	 * Reading the status takes a bus transaction, which cannot happen
	 * here, and a fault arriving as the phone suspends still has to be
	 * acted on.
	 */
	pm_wakeup_dev_event(chip->dev, SC8581_IRQ_WAKE_MS, true);
	queue_delayed_work(system_wq, &chip->irq_handle_work, 0);

	return IRQ_HANDLED;
}

static ssize_t cp_debugfs_show(void *data, char *buf)
{
	struct mca_debugfs_attr_data *d = data;
	struct sc8581_device *chip;
	int len = 0;
	int i, rc;
	u8 val;

	if (!d || !d->private)
		return -EINVAL;

	chip = d->private;

	switch (d->attr_info->debugfs_attr_name) {
	case SC8581_DEBUGFS_ADDRESS:
		return scnprintf(buf, PAGE_SIZE, "%02x\n", chip->address);
	case SC8581_DEBUGFS_COUNT:
		return scnprintf(buf, PAGE_SIZE, "%d\n", chip->count);
	case SC8581_DEBUGFS_DATA:
		for (i = 0; i < chip->count; i++) {
			rc = cp_read_byte(chip, chip->address + i, &val);
			if (rc)
				return rc;
			len += scnprintf(buf + len, PAGE_SIZE - len,
					 "%02x: %02x\n", chip->address + i,
					 val);
		}
		return len;
	default:
		return -EINVAL;
	}
}

static ssize_t cp_debugfs_store(void *data, const char *buf, size_t count)
{
	struct mca_debugfs_attr_data *d = data;
	struct sc8581_device *chip;
	int val;

	if (!d || !d->private)
		return -EINVAL;

	chip = d->private;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	switch (d->attr_info->debugfs_attr_name) {
	case SC8581_DEBUGFS_ADDRESS:
		chip->address = val;
		break;
	case SC8581_DEBUGFS_COUNT:
		chip->count = val;
		break;
	case SC8581_DEBUGFS_DATA:
		if (cp_write_byte(chip, chip->address, val))
			return -EIO;
		break;
	default:
		return -EINVAL;
	}

	return count;
}

static struct mca_debugfs_attr_info sc8581_debugfs_tbl[] = {
	{ "address", 0644, SC8581_DEBUGFS_ADDRESS, cp_debugfs_show, cp_debugfs_store },
	{ "count", 0644, SC8581_DEBUGFS_COUNT, cp_debugfs_show, cp_debugfs_store },
	{ "data", 0644, SC8581_DEBUGFS_DATA, cp_debugfs_show, cp_debugfs_store },
};

static int sc8581_parse_dt(struct sc8581_device *chip)
{
	struct device_node *np = chip->dev->of_node;

	if (!np)
		return -ENODEV;

	mca_parse_dts_u32(np, "ic_role", &chip->cp_role, CP_ROLE_MASTER);
	/* Every line this driver logs says which of the two pumps it is. */
	strscpy(chip->log_tag, chip->cp_role == CP_ROLE_SLAVE ? "[1]" : "[0]",
		sizeof(chip->log_tag));

	/*
	 * A board that says nothing still gets a protection, not a zero: every
	 * one of these is written to the part, and the lowest setting each
	 * register offers is not a safe place to leave it.
	 */
	mca_parse_dts_u32(np, "bat-ovp-threshold", &chip->cfg.bat_ovp_th,
			  SC8581_BAT_OVP_DEFAULT_MV);
	mca_parse_dts_u32(np, "bat-ovp-alarm-threshold",
			  &chip->cfg.bat_ovp_alarm_th,
			  SC8581_BAT_OVP_ALARM_DEFAULT_MV);
	mca_parse_dts_u32(np, "wpc-ovp-threshold", &chip->cfg.wpc_ovp_th,
			  SC8581_WPC_OVP_DEFAULT_MV);
	mca_parse_dts_u32(np, "out-ovp-threshold", &chip->cfg.out_ovp_th,
			  SC8581_OUT_OVP_DEFAULT_MV);
	mca_parse_dts_u32(np, "pmid2-uvp-threshold",
			  &chip->cfg.pmid2out_uvp_th,
			  SC8581_PMID2OUT_UVP_DEFAULT_MV);
	mca_parse_dts_u32(np, "pmid2-ovp-threshold",
			  &chip->cfg.pmid2out_ovp_th,
			  SC8581_PMID2OUT_OVP_DEFAULT_MV);

	mca_parse_dts_u32_array(np, "bus-ovp-threshold", chip->cfg.bus_ovp_th,
				ARRAY_SIZE(chip->cfg.bus_ovp_th));
	mca_parse_dts_u32_array(np, "usb-ovp-threshold", chip->cfg.usb_ovp_th,
				ARRAY_SIZE(chip->cfg.usb_ovp_th));
	mca_parse_dts_u32_array(np, "bus-ocp-threshold", chip->cfg.bus_ocp_th,
				ARRAY_SIZE(chip->cfg.bus_ocp_th));

	chip->support_reverse_quick_charge =
		!!of_find_property(np, "support_reverse_quick_charge", NULL);

	chip->irq_gpio = of_get_named_gpio(np, "cp-int", 0);
	if (chip->irq_gpio < 0)
		mca_log_err("failed to parse irq_gpio\n");

	chip->nlpm_gpio = of_get_named_gpio(np, "cp-nlpm-gpio", 0);
	if (chip->nlpm_gpio < 0)
		mca_log_err("failed to parse  sc858_nlpm_gpio\n");

	return 0;
}

static int sc8581_register_irq(struct sc8581_device *chip)
{
	int irq, rc;

	if (!gpio_is_valid(chip->irq_gpio))
		return 0;

	rc = devm_gpio_request(chip->dev, chip->irq_gpio, "cp-int");
	if (rc)
		return rc;

	irq = gpio_to_irq(chip->irq_gpio);
	if (irq < 0)
		return irq;

	rc = devm_request_threaded_irq(chip->dev, irq, NULL, sc8581_int_isr,
				       IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				       "sc8581_int", chip);
	if (rc)
		return rc;

	/*
	 * A pump fault while the phone is asleep still has to be acted on:
	 * charging carries on across suspend.
	 */
	enable_irq_wake(irq);

	return 0;
}

/**
 * cp_charge_detect_device() - is this part actually here
 * @chip: this driver's state
 *
 * A board is laid out for one of several pumps, and which is fitted is not in
 * the device tree.  The part is asked, and a board without this one simply
 * gets no driver.
 *
 * Return: 0 if it answered, or a negative error.
 */
static int cp_charge_detect_device(struct sc8581_device *chip)
{
	u8 id = 0;
	int retry;
	int rc = -ENODEV;

	/*
	 * The pump shares its bus with the rest of the charging hardware and
	 * may still be settling, so a first read that fails is not proof it
	 * is not there.
	 */
	for (retry = 0; retry < SC8581_DETECT_RETRIES; retry++) {
		rc = cp_read_byte(chip, SC8581_REG_DEVICE_ID, &id);
		if (!rc)
			break;

		mca_log_info("failed to read device id, retry count = %d\n",
			     retry);
		msleep(SC8581_DETECT_RETRY_MS);
	}
	if (rc)
		return rc;

	mca_log_info("sucess read device id = %x\n", id);

	/*
	 * Three parts answer on this address and they are not interchangeable:
	 * the SC8585 measures bus current in different steps, so which one it
	 * is has to be recorded before any reading is scaled.
	 */
	switch (id) {
	case SC8581_DEVICE_ID_SC8561:
		chip->chip_vendor = SC8561_VENDOR;
		break;
	case SC8581_DEVICE_ID_SC8581:
		chip->chip_vendor = SC8581_VENDOR;
		break;
	case SC8581_DEVICE_ID_SC8585:
		chip->chip_vendor = SC8585_VENDOR;
		break;
	default:
		mca_log_info("device_id is invalid\n");
		return -ENODEV;
	}

	chip->product_cfg = id;
	chip->chip_ok = true;

	return 0;
}

/*
 * One hardware variant wires the charge pump to answer at a different address
 * than the device tree gives.  The device tree is shared, so the driver has to
 * ask what board this is and move the address itself; on anything else the
 * device tree address stands.
 */
#define SC8581_EN_BUCK_PLATFORM		3
#define SC8581_EN_BUCK_HWID		0x10004
#define SC8581_EN_BUCK_I2C_ADDR		0x6f

static void sc8581_adjust_en_buck_addr(struct i2c_client *client)
{
	struct mca_hwid_info *hwid = mca_get_hwid_info();

	if (!hwid) {
		mca_log_err("get hwid info failed\n");
		return;
	}

	if (hwid->platform_version != SC8581_EN_BUCK_PLATFORM ||
	    hwid->hwid_value != SC8581_EN_BUCK_HWID)
		return;

	mca_log_err("adjust en buck CP addr\n");
	client->addr = SC8581_EN_BUCK_I2C_ADDR;
}

static int sc8581_probe(struct i2c_client *client)
{
	struct sc8581_device *chip;
	int rc;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;
	chip->dev = &client->dev;
	i2c_set_clientdata(client, chip);

	sc8581_adjust_en_buck_addr(client);

	INIT_DELAYED_WORK(&chip->irq_handle_work, sc8581_irq_handler);

	rc = sc8581_parse_dt(chip);
	if (rc) {
		mca_log_err("failed to parse DTS\n");
		return rc;
	}

	/*
	 * The part comes out of reset in a low-power mode where it answers
	 * its bus but nothing else, and it takes a while to come out of it.
	 */
	if (gpio_is_valid(chip->nlpm_gpio)) {
		rc = devm_gpio_request(chip->dev, chip->nlpm_gpio,
				       "cp-nlpm-gpio");
		if (rc)
			mca_log_info("unable to request nlpm gpio [%d]\n",
				     chip->nlpm_gpio);

		rc = gpiod_direction_output_raw(gpio_to_desc(chip->nlpm_gpio),
						1);
		if (rc)
			mca_log_info("unable to set direction for nlpm gpio[%d]\n",
				     chip->nlpm_gpio);

		msleep(SC8581_NLPM_SETTLE_MS);
	}

	rc = cp_charge_detect_device(chip);
	if (rc) {
		/*
		 * A board laid out for this pump that does not answer has
		 * one fitted and broken, or none at all; either way the
		 * stack has to be told, or it waits for a pump that is
		 * never going to appear.
		 */
		mca_log_err("failed to detect device\n");
		mca_event_block_notify(MCA_EVENT_TYPE_CP_INFO,
				       MCA_EVENT_CP_IIC_ERROR, NULL);
		return rc;
	}

	rc = sc8581_init_device(chip);
	if (rc)
		return rc;

	rc = sc8581_register_irq(chip);
	if (rc)
		return rc;

	device_set_wakeup_capable(chip->dev, true);
	device_wakeup_enable(chip->dev);

	rc = platform_class_cp_register_ops(chip->cp_role, &sc8581_cp_ops,
					    chip);
	if (rc)
		return rc;

	mca_debugfs_create_group(chip->cp_role == CP_ROLE_SLAVE ?
				 "sc85xx_01" : "sc85xx_00", sc8581_debugfs_tbl,
				 ARRAY_SIZE(sc8581_debugfs_tbl), chip);

	mca_log_charge_log_register(chip->cp_role == CP_ROLE_MASTER ?
				    MCA_CHARGE_LOG_ID_CP_MASTER_IC :
				    MCA_CHARGE_LOG_ID_CP_SLAVE_IC,
				    &sc8581_log_ops, chip);

	/*
	 * Look once at startup: a fault the part latched before this driver
	 * loaded raises no edge for the interrupt to catch.
	 */
	queue_delayed_work(system_wq, &chip->irq_handle_work, 0);

	mca_log_info("probe success %d\n", chip->cp_role);

	return 0;
}

static void sc8581_remove(struct i2c_client *client)
{
	struct sc8581_device *chip = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&chip->irq_handle_work);
	ops_cp_enable_adc(false, chip);
}

static void sc8581_shutdown(struct i2c_client *client)
{
	struct sc8581_device *chip = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&chip->irq_handle_work);
	ops_cp_enable_adc(false, chip);

	/*
	 * Put the part back in low-power mode on the way down, so that a
	 * phone left off the charger is not held awake by a pump that is
	 * still converting.
	 */
	if (gpio_is_valid(chip->nlpm_gpio) &&
	    gpiod_direction_output_raw(gpio_to_desc(chip->nlpm_gpio), 0))
		mca_log_err("unable to reset adc and nlpm fail\n");

	mca_log_info("sc8581 shutdown!\n");
}

/*
 * The ADC runs continuously and costs current, which is the whole of the
 * pump's own consumption once it has stopped switching.
 */
static int sc8581_suspend(struct device *dev)
{
	struct sc8581_device *chip = dev_get_drvdata(dev);

	mca_log_info("sc8581 suspend!\n");
	chip->i2c_is_working = false;

	return ops_cp_enable_adc(false, chip);
}

/*
 * Nothing to undo: whatever wants the ADC turns it back on when it next
 * reads one, and turning it on here would run it through every resume the
 * phone does with no charger attached.
 */
static int sc8581_resume(struct device *dev)
{
	return 0;
}

static void sc8581_i2c_complete(struct device *dev)
{
	struct sc8581_device *chip = dev_get_drvdata(dev);

	/*
	 * The bus controller resumes after this driver does, so anything
	 * between the two would fail; this is the point from which reads
	 * are worth attempting again.
	 */
	chip->i2c_is_working = true;
	mca_log_info("sc8581 i2c complete!\n");
}

static const struct dev_pm_ops sc8581_pm_ops = {
	.suspend	= sc8581_suspend,
	.resume		= sc8581_resume,
	.complete	= sc8581_i2c_complete,
};

static const struct of_device_id sc8581_match[] = {
	{ .compatible = "sc8581_charger_pump" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc8581_match);

static const struct i2c_device_id sc8581_id[] = {
	{ "sc8581", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sc8581_id);

static struct i2c_driver sc8581_driver = {
	.driver = {
		.name		= "sc8581",
		.of_match_table	= sc8581_match,
		.pm		= &sc8581_pm_ops,
	},
	.probe		= sc8581_probe,
	.remove		= sc8581_remove,
	.shutdown	= sc8581_shutdown,
	.id_table	= sc8581_id,
};
module_i2c_driver(sc8581_driver);

MODULE_DESCRIPTION("SC8581 charge pump");
MODULE_LICENSE("GPL");
