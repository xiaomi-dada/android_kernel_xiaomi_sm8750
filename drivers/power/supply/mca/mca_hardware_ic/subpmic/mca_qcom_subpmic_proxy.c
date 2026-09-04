// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2025 The LineageOS Project
 *
 * The buck charger, as reached through the ADSP.
 *
 * On this platform the application processor does not own the charger.  The
 * ADSP does: it runs the charging loop, owns the registers, and is the only
 * thing that talks to the hardware.  Everything the rest of the charging
 * stack wants -- a reading, a limit, whether to charge at all -- is a message
 * to the ADSP and an answer back.
 *
 * So this driver holds no state that matters and makes no decisions.  It
 * registers as the buck charger, as the BC1.2 detector and as the Quick
 * Charge protocol, and turns each of those calls into the property read or
 * write that says the same thing to the firmware.  What little it does keep
 * -- the OTG boost wiring, whether this board has a cable-id pin -- is
 * configuration the firmware cannot read from the device tree itself, and is
 * pushed across once the link comes up and again whenever it comes back.
 *
 * Two of the exports here are not for the charging stack at all.  Audio and
 * haptics ask the QTI battery charger driver about the boost, and that driver
 * is not present; the symbols are provided so the callers link and get a
 * clear answer instead of an absent one.
 */

#define MCA_LOG_TAG "qcom_subpmic"

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <mca/common/mca_adsp_glink.h>
#include <mca/common/mca_event.h>
#include <linux/hwid.h>
#include <mca/common/mca_hwid.h>
#include <mca/common/mca_log.h>
#include <mca/common/mca_parse_dts.h>
#include <mca/platform/platform_bc12_class.h>
#include <mca/platform/platform_buckchg_class.h>
#include <mca/platform/platform_cp_class.h>
#include <mca/protocol/protocol_pd_class.h>
#include <mca/protocol/protocol_qc_class.h>
#include <mca/strategy/strategy_class.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/soc/qcom/battery_charger.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

/* The lowest and highest a Quick Charge adapter may be asked for. */
#define QC_VOLT_MIN_MV			5000
#define QC_VOLT_MAX_MV			12000

/*
 * How far the adapter moves per step, and how long that step takes.  A
 * QC3+ adapter is adjusted in fine steps and settles quickly; an older one
 * moves in coarse ones and takes correspondingly longer.
 */
#define QC3P5_STEP_MV			20
#define QC3P5_STEP_MS			5
#define QC3P5_SETTLE_MS			500
#define QC_STEP_MV			200
#define QC_STEP_MS			50
#define QC_SETTLE_MS			800

/* How often the ramp looks at where the adapter has got to. */
#define QC_POLL_MS			50

/* How long the wakeup is held after the ADSP says something. */
#define SUBPMIC_WAKE_MS			1000

/* How long to wait before pushing the configuration across again. */
#define SUBPMIC_SYNC_RETRY_MS		1000

/* How long the first configuration push waits behind the type probe. */
#define SUBPMIC_SYNC_START_MS		100

/* The ADSP reports voltages in microvolts; the stack works in millivolts. */
#define UV_PER_MV			1000

/**
 * struct otg_cfg_info - how this board's OTG boost is wired
 * @boost_src:       which supply provides the boost
 * @gpio_chip_type:  whether the gates below are SoC or PMIC pins
 * @vdd_boost_gpio:  the pin enabling the boost supply
 * @ovp_en_gpio:     the pin enabling the over-voltage gate in front of it
 *
 * The firmware drives these pins but cannot read the device tree, so the
 * board's wiring is pushed to it rather than compiled into it.
 */
struct otg_cfg_info {
	int	boost_src;
	int	gpio_chip_type;
	int	vdd_boost_gpio;
	int	ovp_en_gpio;
};

/**
 * struct adsp_notify_change_node - one notification waiting to be acted on
 * @node:         links it into the pending list
 * @notification: what the ADSP said
 * @value:        what came with it
 *
 * Notifications arrive in a context that must not block, and acting on one
 * means talking to the ADSP again, so they are queued rather than handled
 * where they land.
 */
struct adsp_notify_change_node {
	struct list_head	node;
	int			notification;
	int			value;
};

/**
 * struct qcom_subpmic_data - this driver's state
 * @dev:                       this device
 * @usb_type_work:             re-reads the adapter type once it has settled
 * @notify_change_work:        acts on what the ADSP has said
 * @ship_mode_en:              ship mode has been asked for
 * @reboot_notifier:           puts the board into ship mode on reboot
 * @shutdown_notifier:         tells the ADSP the board is going down
 * @sync_cfg_work:             pushes this board's wiring to the ADSP
 * @adsp_notify_list:          notifications not yet acted on
 * @notify_lock:               guards @adsp_notify_list
 * @support_2s_charging:       the pack is two cells in series
 * @support_dual_panel:        the board is a foldable with two panels
 * @support_multi_bc12:        more than one BC1.2 detector
 * @support_cid:               the board has a cable-id pin
 * @support_typec:             the second port negotiates Power Delivery of
 *                             its own, so what it reports is not this
 *                             driver's to announce.  No board here sets it,
 *                             and the vendor names no property for it
 * @cid_gpio_int:              which pin that is
 * @otg_cfg:                   how the OTG boost is wired
 * @real_type:                 the adapter type the ADSP last reported
 * @adsp_init_done:            the ADSP has finished starting
 * @lpd_status:                what it last said about moisture
 * @cc_short_vbus:             CC is shorted to VBUS
 * @pps_ptf:                   the adapter's temperature flag
 * @bsinkpowersuspend:         the adapter has suspended its output
 * @eu_model:                  this unit is sold where the EU limits apply
 * @cid_sts:                   what the cable-id pin reads
 */
/**
 * struct mca_revchg_notify - what the reverse charging path is told
 * @type:  which change this is
 * @value: what it changed to
 */
struct mca_revchg_notify {
	u16	type;
	u16	value;
};

/* The boost is no longer in the way of reverse charging. */
#define MCA_REVCHG_BOOST_FREE	256

struct qcom_subpmic_data {
	struct device			*dev;
	struct delayed_work		usb_type_work;
	struct work_struct		notify_change_work;
	bool				ship_mode_en;
	struct notifier_block		reboot_notifier;
	struct notifier_block		shutdown_notifier;
	struct delayed_work		sync_cfg_work;
	struct list_head		adsp_notify_list;
	spinlock_t			notify_lock;
	bool				support_2s_charging;
	bool				support_dual_panel;
	bool				support_multi_bc12;
	bool				support_cid;
	bool				support_typec;
	bool				support_ovpgate;
	struct mca_revchg_notify	revchg_notify;
	int				cid_gpio_int;
	struct otg_cfg_info		otg_cfg;
	int				usb_online;
	int				real_type;
	bool				glink_down;
	bool				pmic_init_notified;
	int				adsp_init_done;
	int				lpd_status;
	int				cc_short_vbus;
	int				pps_ptf;
	bool				bsinkpowersuspend;
	bool				eu_model;
	int				cid_sts;
};

static struct qcom_subpmic_data *g_subpmic;

/*
 * Most of what the charging stack asks for is one property of the firmware's,
 * so most of what follows is one call each.  They are grouped as the ops
 * structure lists them rather than by what they do.
 */

/*
 * Whether a cable is attached.  This is answered from what the monitor last
 * read rather than by asking the firmware again: it is asked often, from the
 * charging path, and a round trip to the ADSP there would be a lot of them.
 */
static int qcom_subpmic_get_online(void *data, int *online)
{
	struct qcom_subpmic_data *subpmic = data;

	*online = subpmic->usb_online;

	return 0;
}

static int qcom_subpmic_is_charge_done(void *data, bool *charge_done)
{
	int status;
	int rc;

	rc = mca_adsp_glink_read_prop(ADSP_PROP_ID_BUCK_CHGR_STATUS, &status,
				      sizeof(status));
	if (rc)
		return rc;

	*charge_done = status == CHGR_STATUS_TERMINATION;

	return 0;
}

static int qcom_subpmic_get_input_current_limit(void *data, int *input_curr_lmt)
{
	return mca_adsp_glink_read_prop(ADSP_PROP_ID_BUCK_INPUT_CURR_LIMIT,
					input_curr_lmt, sizeof(*input_curr_lmt));
}

static int qcom_subpmic_get_bus_curr(void *data, int *bus_curr)
{
	return mca_adsp_glink_read_prop(ADSP_PROP_ID_USB_CURRENT_NOW,
					bus_curr, sizeof(*bus_curr));
}

static int qcom_subpmic_get_bus_volt(void *data, int *bus_volt)
{
	return mca_adsp_glink_read_prop(ADSP_PROP_ID_USB_VOLTAGE_NOW,
					bus_volt, sizeof(*bus_volt));
}

static int qcom_subpmic_get_usb_sns_volt(void *data, int *usb_sns_volt)
{
	return mca_adsp_glink_read_prop(ADSP_PROP_ID_USB_SNS_VOLTAGE_NOW,
					usb_sns_volt, sizeof(*usb_sns_volt));
}

static int qcom_subpmic_get_vsys_volt(void *data, int *vsys_min)
{
	return mca_adsp_glink_read_prop(ADSP_PROP_ID_BUCK_VSYSMIN,
					vsys_min, sizeof(*vsys_min));
}

static int qcom_subpmic_get_chg_status(void *data, int *chg_status)
{
	return mca_adsp_glink_read_prop(ADSP_PROP_ID_BUCK_CHGR_STATUS,
					chg_status, sizeof(*chg_status));
}

static int qcom_subpmic_get_chg_type(void *data, int *chg_type)
{
	return mca_adsp_glink_read_prop(ADSP_PROP_ID_BUCK_CHGR_TYPE,
					chg_type, sizeof(*chg_type));
}

static int qcom_subpmic_get_term_current(void *data, int *term_curr)
{
	return mca_adsp_glink_read_prop(ADSP_PROP_ID_BUCK_TERM_CURR,
					term_curr, sizeof(*term_curr));
}

static int qcom_subpmic_get_term_volt(void *data, int *term_volt)
{
	return mca_adsp_glink_read_prop(ADSP_PROP_ID_BUCK_TERM_VOLT,
					term_volt, sizeof(*term_volt));
}

static int qcom_subpmic_get_wls_curr(void *data, int *wls_curr)
{
	return mca_adsp_glink_read_prop(ADSP_PROP_ID_DC_CURRENT_NOW,
					wls_curr, sizeof(*wls_curr));
}

static int qcom_subpmic_get_usb_aicl_cont_thd(void *data, int *aicl_cont_thd)
{
	return mca_adsp_glink_read_prop(ADSP_PROP_ID_BUCK_AICL_CONT_THD,
					aicl_cont_thd, sizeof(*aicl_cont_thd));
}

static int qcom_subpmic_get_otg_gate_enable_status(void *data, int *otg_gate_enable_status)
{
	return mca_adsp_glink_read_prop(ADSP_PROP_ID_BUCK_OTG_GATE_ENABLE_STATUS,
					otg_gate_enable_status,
					sizeof(*otg_gate_enable_status));
}

static int qcom_subpmic_get_otg_boost_enable_status(void *data, int *otg_boost_enable_status)
{
	return mca_adsp_glink_read_prop(ADSP_PROP_ID_BUCK_OTG_BOOST_ENABLE_STATUS,
					otg_boost_enable_status,
					sizeof(*otg_boost_enable_status));
}

static int qcom_subpmic_get_lpd_enable(void *data, int *lpd_enable)
{
	return mca_adsp_glink_read_prop(ADSP_PROP_ID_USB_MOISTURE_DETECTION_ENABLE,
					lpd_enable, sizeof(*lpd_enable));
}

static int qcom_subpmic_get_lpd_status(void *data, int *lpd_status)
{
	return mca_adsp_glink_read_prop(ADSP_PROP_ID_USB_MOISTURE_DETECTION_STATUS,
					lpd_status, sizeof(*lpd_status));
}

static int qcom_subpmic_get_lpd_sbu1(void *data, int *lpd_sbu1)
{
	return mca_adsp_glink_read_prop(ADSP_PROP_ID_BUCK_LPD_SBU1,
					lpd_sbu1, sizeof(*lpd_sbu1));
}

static int qcom_subpmic_get_lpd_sbu2(void *data, int *lpd_sbu2)
{
	return mca_adsp_glink_read_prop(ADSP_PROP_ID_BUCK_LPD_SBU2,
					lpd_sbu2, sizeof(*lpd_sbu2));
}

static int qcom_subpmic_get_lpd_cc1(void *data, int *lpd_cc1)
{
	return mca_adsp_glink_read_prop(ADSP_PROP_ID_BUCK_LPD_CC1,
					lpd_cc1, sizeof(*lpd_cc1));
}

static int qcom_subpmic_get_lpd_cc2(void *data, int *lpd_cc2)
{
	return mca_adsp_glink_read_prop(ADSP_PROP_ID_BUCK_LPD_CC2,
					lpd_cc2, sizeof(*lpd_cc2));
}

static int qcom_subpmic_get_lpd_dp(void *data, int *lpd_dp)
{
	return mca_adsp_glink_read_prop(ADSP_PROP_ID_BUCK_LPD_DP,
					lpd_dp, sizeof(*lpd_dp));
}

static int qcom_subpmic_get_lpd_dm(void *data, int *lpd_dm)
{
	return mca_adsp_glink_read_prop(ADSP_PROP_ID_BUCK_LPD_DM,
					lpd_dm, sizeof(*lpd_dm));
}

static int qcom_subpmic_get_lpd_control(void *data, int *lpd_control)
{
	return mca_adsp_glink_read_prop(ADSP_PROP_ID_BUCK_LPD_CONTROL,
					lpd_control, sizeof(*lpd_control));
}

static int qcom_subpmic_get_lpd_uart_control(void *data, int *lpd_uart_control)
{
	return mca_adsp_glink_read_prop(ADSP_PROP_ID_BUCK_LPD_UART_CONTROL,
					lpd_uart_control,
					sizeof(*lpd_uart_control));
}

static int qcom_subpmic_get_pack_vbat(void *data, int *pack_vbat)
{
	return mca_adsp_glink_read_prop(ADSP_PROP_ID_BUCK_PACK_VBAT,
					pack_vbat, sizeof(*pack_vbat));
}

static int qcom_subpmic_get_pack_ibat(void *data, int *pack_ibat)
{
	return mca_adsp_glink_read_prop(ADSP_PROP_ID_BUCK_PACK_IBAT,
					pack_ibat, sizeof(*pack_ibat));
}

static int qcom_subpmic_get_pack_tbat(void *data, int *pack_tbat)
{
	return mca_adsp_glink_read_prop(ADSP_PROP_ID_BUCK_PACK_TBAT,
					pack_tbat, sizeof(*pack_tbat));
}

static int qcom_subpmic_get_aicl_status(void *data, int *aicl_status)
{
	return mca_adsp_glink_read_prop(ADSP_PROP_ID_BUCK_AICL_STATUS,
					aicl_status, sizeof(*aicl_status));
}

static int qcom_subpmic_set_input_current_limit(void *data, int input_curr_lmt)
{
	return mca_adsp_glink_write_prop(ADSP_PROP_ID_BUCK_INPUT_CURR_LIMIT,
					 &input_curr_lmt, sizeof(input_curr_lmt));
}

static int qcom_subpmic_set_wls_input_current_limit(void *data, int wls_input_curr_lmt)
{
	return mca_adsp_glink_write_prop(ADSP_PROP_ID_DC_INPUT_CURR_LIMIT,
					 &wls_input_curr_lmt,
					 sizeof(wls_input_curr_lmt));
}

static int qcom_subpmic_set_input_volt_limit(void *data, int input_volt_lmt)
{
	return mca_adsp_glink_write_prop(ADSP_PROP_ID_BUCK_INPUT_VOLT_LIMIT,
					 &input_volt_lmt, sizeof(input_volt_lmt));
}

static int qcom_subpmic_set_charge_current(void *data, int ichg)
{
	return mca_adsp_glink_write_prop(ADSP_PROP_ID_BUCK_CHARGE_CURR,
					 &ichg, sizeof(ichg));
}

static int qcom_subpmic_set_term_current(void *data, int term_curr)
{
	return mca_adsp_glink_write_prop(ADSP_PROP_ID_BUCK_TERM_CURR,
					 &term_curr, sizeof(term_curr));
}

static int qcom_subpmic_set_term_volt(void *data, int term_volt)
{
	return mca_adsp_glink_write_prop(ADSP_PROP_ID_BUCK_TERM_VOLT,
					 &term_volt, sizeof(term_volt));
}

static int qcom_subpmic_set_prechg_volt(void *data, int prechg_volt)
{
	return mca_adsp_glink_write_prop(ADSP_PROP_ID_BUCK_PRECHG_VOLT,
					 &prechg_volt, sizeof(prechg_volt));
}

static int qcom_subpmic_set_prechg_current(void *data, int prechg_curr)
{
	return mca_adsp_glink_write_prop(ADSP_PROP_ID_BUCK_PRECHG_CURR,
					 &prechg_curr, sizeof(prechg_curr));
}

static int qcom_subpmic_set_usb_aicl_cont_thd(void *data, int aicl_cont_thd)
{
	return mca_adsp_glink_write_prop(ADSP_PROP_ID_BUCK_AICL_CONT_THD,
					 &aicl_cont_thd, sizeof(aicl_cont_thd));
}

static int qcom_subpmic_set_opt_fws(void *data, int opt_fws)
{
	return mca_adsp_glink_write_prop(ADSP_PROP_ID_BUCK_OPT_FWS,
					 &opt_fws, sizeof(opt_fws));
}

static int qcom_subpmic_set_qc3_volt(void *data, int qc3_volt)
{
	return mca_adsp_glink_write_prop(ADSP_PROP_ID_BUCK_QC3_VOLT,
					 &qc3_volt, sizeof(qc3_volt));
}

static int qcom_subpmic_set_lpd_sbu1(void *data, int lpd_sbu1)
{
	return mca_adsp_glink_write_prop(ADSP_PROP_ID_BUCK_LPD_SBU1,
					 &lpd_sbu1, sizeof(lpd_sbu1));
}

static int qcom_subpmic_set_lpd_control(void *data, int lpd_control)
{
	return mca_adsp_glink_write_prop(ADSP_PROP_ID_BUCK_LPD_CONTROL,
					 &lpd_control, sizeof(lpd_control));
}

static int qcom_subpmic_set_lpd_uart_control(void *data, int lpd_uart_control)
{
	return mca_adsp_glink_write_prop(ADSP_PROP_ID_BUCK_LPD_UART_CONTROL,
					 &lpd_uart_control,
					 sizeof(lpd_uart_control));
}

static int qcom_subpmic_set_too_hot_limit(void *data, int too_hot_limit)
{
	return mca_adsp_glink_write_prop(ADSP_PROP_ID_TOO_HOT_LIMIT,
					 &too_hot_limit, sizeof(too_hot_limit));
}
/*
 * The firmware takes a whole word for a flag, so the ones the stack passes as
 * a bool are widened here rather than at every call site.
 */

static int qcom_subpmic_enable_charging(void *data, bool enable)
{
	int val = enable;

	mca_log_info("enable charging: %d\n", enable);

	return mca_adsp_glink_write_prop(ADSP_PROP_ID_BUCK_ENABLE_CHARGING,
					 &val, sizeof(val));
}

static int qcom_subpmic_set_input_suspend(void *data, bool enable)
{
	int val = enable;

	mca_log_info("set input suspend %d\n", enable);

	return mca_adsp_glink_write_prop(ADSP_PROP_ID_BUCK_INPUT_SUSPEND,
					 &val, sizeof(val));
}

static int qcom_subpmic_set_wireless_input_suspend(void *data, bool enable)
{
	int val = enable;

	mca_log_info("set wls input suspend %d\n", enable);

	return mca_adsp_glink_write_prop(ADSP_PROP_ID_DC_INPUT_SUSPEND,
					 &val, sizeof(val));
}

static int qcom_subpmic_set_buck_fsw(void *data, int buck_fsw)
{
	mca_log_info("set buck fsw: %d\n", buck_fsw);

	return mca_adsp_glink_write_prop(ADSP_PROP_ID_BUCK_SET_FSW,
					 &buck_fsw, sizeof(buck_fsw));
}

static int qcom_subpmic_set_aicl_enable(void *data, bool enable)
{
	int val = enable;

	mca_log_info("set aicl enable: %d\n", enable);

	return mca_adsp_glink_write_prop(ADSP_PROP_ID_BUCK_ENABLE_AICL,
					 &val, sizeof(val));
}

static int qcom_subpmic_set_rerun_aicl(void *data, bool enable)
{
	int val = enable;

	mca_log_info("set rerun aicl: %d\n", enable);

	return mca_adsp_glink_write_prop(ADSP_PROP_ID_BUCK_RERUN_AICL,
					 &val, sizeof(val));
}

static int qcom_subpmic_set_restart_aicl(void *data, bool enable)
{
	int val = enable;

	mca_log_info("set restart aicl: %d\n", enable);

	return mca_adsp_glink_write_prop(ADSP_PROP_ID_BUCK_RESTART_AICL,
					 &val, sizeof(val));
}

static int qcom_subpmic_usb_adapter_allow_override(void *data, bool enable)
{
	int val = enable;

	return mca_adsp_glink_write_prop(
		ADSP_PROP_ID_BUCK_USB_ADAPTER_ALLOW_OVERRIDE, &val,
		sizeof(val));
}

static int qcom_subpmic_set_wls_vdd_flag(void *data, bool enable)
{
	int val = enable;

	mca_log_info("set wls vdd flag: %d\n", enable);

	return mca_adsp_glink_write_prop(ADSP_PROP_ID_DC_SET_VDD_FLAG,
					 &val, sizeof(val));
}

static int qcom_subpmic_set_eu_model(void *data, bool enable)
{
	struct qcom_subpmic_data *subpmic = data;
	int val = enable;
	int rc;

	mca_log_info("set_eu_model = %d\n", enable);

	subpmic->eu_model = enable;

	rc = mca_adsp_glink_write_prop(ADSP_PROP_ID_BUCK_EU_MODEL, &val,
				       sizeof(val));
	if (rc)
		mca_log_err("Failed to write eu model to adsp: %d\n", rc);

	return rc;
}

static int qcom_subpmic_set_boost_enable(void *data, int boost_enable)
{
	mca_log_info("set otg enable: 0x%2x\n", boost_enable);

	return mca_adsp_glink_write_prop(ADSP_PROP_ID_BUCK_ENABLE_BOOST,
					 &boost_enable, sizeof(boost_enable));
}

static int qcom_subpmic_set_boost_voltage(void *data, int boost_voltage)
{
	mca_log_info("set boost voltage: 0x%2x\n", boost_voltage);

	return mca_adsp_glink_write_prop(ADSP_PROP_ID_BUCK_ADJUST_BOOST_VOLTAGE,
					 &boost_voltage,
					 sizeof(boost_voltage));
}

static int qcom_subpmic_set_qc_volt_cmd(void *data, int hvdcp_cmd)
{
	mca_log_info("set_qc_volt_cmd: %d\n", hvdcp_cmd);

	return mca_adsp_glink_write_prop(ADSP_PROP_ID_BUCK_QC_VOLT_CMD,
					 &hvdcp_cmd, sizeof(hvdcp_cmd));
}

/*
 * The values below are the driver's own rather than the firmware's, and are
 * answered without asking it.
 */

static int qcom_subpmic_is_init_ok(void *data)
{
	struct qcom_subpmic_data *subpmic = data;

	if (!subpmic)
		return -EINVAL;

	return subpmic->adsp_init_done ? subpmic->adsp_init_done : 0;
}

static int qcom_subpmic_is_support_cid(void *data, bool *support_cid)
{
	struct qcom_subpmic_data *subpmic = data;

	*support_cid = subpmic->support_cid;

	return 0;
}

static int qcom_subpmic_get_otg_boost_src(void *data, int *otg_boost_src)
{
	struct qcom_subpmic_data *subpmic = data;

	*otg_boost_src = subpmic->otg_cfg.boost_src;
	mca_log_info("get otg_boost_src: %d\n", subpmic->otg_cfg.boost_src);

	return subpmic->otg_cfg.boost_src;
}

static int qcom_subpmic_get_ship_mode(void *data, bool *ship_mode)
{
	struct qcom_subpmic_data *subpmic = data;

	*ship_mode = subpmic->ship_mode_en;

	return 0;
}

/**
 * qcom_subpmic_set_ship_mode() - arrange for the board to power down fully
 * @data:   this driver's state
 * @enable: whether to
 *
 * Ship mode disconnects the battery, so it cannot be entered while the kernel
 * is still running.  The request is only remembered; it is acted on from the
 * reboot notifier, which is the last point at which the ADSP can still be
 * told.
 *
 * Return: 0.
 */
static int qcom_subpmic_set_ship_mode(void *data, bool enable)
{
	struct qcom_subpmic_data *subpmic = data;

	subpmic->ship_mode_en = enable;
	mca_log_err("is_enable_shipmode = %d\n", enable);

	return 0;
}

/*
 * BC1.2 detection is the firmware's and runs whether or not anyone asks, so
 * there is nothing to enable.  The entry exists because the class requires
 * one.
 */
static int qcom_subpmic_bc12_det_en(int role, void *data)
{
	return 0;
}

/**
 * qcom_subpmic_get_usb_real_type() - what kind of adapter is attached
 * @type: filled in with the type
 * @data: this driver's state
 *
 * The firmware is asked first, and the type it last reported is used if it
 * cannot answer: an adapter does not change kind while it is plugged in, so
 * the remembered answer is the right one rather than a guess.
 *
 * Return: 0.
 */
static int qcom_subpmic_get_usb_real_type(int *type, void *data)
{
	struct qcom_subpmic_data *subpmic = data;
	int rc;

	rc = mca_adsp_glink_read_prop(ADSP_PROP_ID_USB_USB_REAL_TYPE, type,
				      sizeof(*type));
	if (rc)
		*type = subpmic->real_type;

	return 0;
}

/**
 * qcom_subpmic_set_qc_volt() - walk a Quick Charge adapter to a voltage
 * @data: this driver's state
 * @volt: where to take it, in millivolts
 *
 * A Quick Charge adapter is moved by asking for one step at a time, and it
 * takes a while to get there.  The request is made once and then the input
 * voltage is watched until it arrives, so that a caller who asks for a large
 * change does not carry on as though it had already happened.
 *
 * Two adapters need no wait: a QC2 adapter has fixed steps and reaches them at
 * once, and anything asked for 5 V is being returned to its default.
 *
 * Return: 0, or a negative error.
 */
static int qcom_subpmic_set_qc_volt(void *data, int volt)
{
	struct qcom_subpmic_data *subpmic = data;
	int vbus = 0;
	int target = volt;
	int step, step_ms, settle_ms;
	int diff, steps, wait_ms;
	ktime_t start;
	int rc;

	start = ktime_get_boottime();

	if (subpmic->real_type == XM_CHARGER_TYPE_HVDCP3P5) {
		step = QC3P5_STEP_MV;
		step_ms = QC3P5_STEP_MS;
		settle_ms = QC3P5_SETTLE_MS;
	} else {
		step = QC_STEP_MV;
		step_ms = QC_STEP_MS;
		settle_ms = QC_SETTLE_MS;
	}

	if (volt < QC_VOLT_MIN_MV || volt > QC_VOLT_MAX_MV) {
		mca_log_err("invalid qc voltage: %d\n", volt);
		return -EINVAL;
	}

	mca_adsp_glink_read_prop(ADSP_PROP_ID_USB_VOLTAGE_NOW, &vbus,
				 sizeof(vbus));
	vbus /= UV_PER_MV;
	diff = volt - vbus;

	mca_log_info("real_type: %d, volt: %d, vbus: %d\n", subpmic->real_type,
		     volt, vbus);

	rc = mca_adsp_glink_write_prop(ADSP_PROP_ID_BUCK_QC_VOLT, &target,
				       sizeof(target));
	if (rc)
		return rc;

	if (subpmic->real_type == XM_CHARGER_TYPE_HVDCP2)
		return 0;

	if (target == QC_VOLT_MIN_MV)
		return 0;

	if (abs(diff) < step)
		return 0;

	steps = abs(diff) / step;
	wait_ms = steps * step_ms + settle_ms;

	while (true) {
		msleep(QC_POLL_MS);

		mca_adsp_glink_read_prop(ADSP_PROP_ID_USB_VOLTAGE_NOW, &vbus,
					 sizeof(vbus));
		vbus /= UV_PER_MV;
		mca_log_info("vbus: %d\n", vbus);

		if (!subpmic->real_type) {
			mca_log_info("usb removed, break\n");
			break;
		}

		if (abs(volt - vbus) < step)
			break;

		if (ktime_ms_delta(ktime_get_boottime(), start) > wait_ms) {
			mca_log_info("wait time out, break\n");
			break;
		}
	}

	return 0;
}

/**
 * qcom_subpmic_sync_cfg_work() - tell the firmware how this board is wired
 * @work: the work
 *
 * The firmware cannot read the device tree, so what it needs from it -- which
 * charge pump is fitted, how the OTG boost is wired, whether there is a
 * cable-id pin, whether the EU limits apply -- is pushed across from here.
 * This runs once the link is up and again every time it comes back, since a
 * firmware that has restarted has forgotten all of it.
 *
 * The charge pump probes separately, so its identity may not be known yet; in
 * that case the whole push is retried rather than sent incomplete.
 */
static void qcom_subpmic_sync_cfg_work(struct work_struct *work)
{
	struct qcom_subpmic_data *subpmic =
		container_of(to_delayed_work(work), struct qcom_subpmic_data,
			     sync_cfg_work);
	struct otg_cfg_info otg_cfg;
	bool cp_present = false;
	u8 cp_state;
	int val;
	int rc;

	rc = platform_class_cp_get_present(CP_ROLE_MASTER, &cp_present);
	if (rc) {
		mca_log_err("Failed to get cp state, rc=%d\n", rc);
		goto retry;
	}

	cp_state = cp_present;
	rc = mca_adsp_glink_write_prop(ADSP_PROP_ID_BUCK_CP_STATE, &cp_state,
				       sizeof(cp_state));
	if (rc)
		mca_log_err("Failed to send cp state, rc=%d\n", rc);
	else
		mca_log_err("Success to send cp state, rc=%d\n", rc);

	otg_cfg = subpmic->otg_cfg;
	rc = mca_adsp_glink_write_prop(ADSP_PROP_ID_BUCK_OTG_CFG, &otg_cfg,
				       sizeof(otg_cfg));
	if (rc) {
		mca_log_err("Failed to send otg cfg, rc=%d\n", rc);
		goto retry;
	}
	mca_log_err("Success to send otg cfg, rc=%d\n", rc);

	val = subpmic->support_cid ? subpmic->cid_gpio_int : 0;
	rc = mca_adsp_glink_write_prop(ADSP_PROP_ID_BUCK_CID_CFG, &val,
				       sizeof(val));
	if (rc)
		mca_log_err("Failed to send cid cfg, rc=%d\n", rc);
	else
		mca_log_err("Success to send cid cfg, rc=%d\n", rc);

	cp_state = subpmic->eu_model;
	rc = mca_adsp_glink_write_prop(ADSP_PROP_ID_BUCK_EU_MODEL, &cp_state,
				       sizeof(cp_state));
	if (rc)
		mca_log_err("Failed to send eu model, rc=%d\n", rc);
	else
		mca_log_err("Success to send eu model, rc=%d\n", rc);

	return;

retry:
	queue_delayed_work(system_wq, &subpmic->sync_cfg_work,
			   msecs_to_jiffies(SUBPMIC_SYNC_RETRY_MS));
}

/**
 * qcom_subpmic_update_usb_type_work() - re-read the adapter after it settles
 * @work: the work
 *
 * The type the firmware first reports is what BC1.2 saw, which for a charger
 * that also speaks a higher protocol is not the final answer.  This looks
 * again once the negotiation has had time to finish.
 */
static void qcom_subpmic_update_usb_type_work(struct work_struct *work)
{
	struct qcom_subpmic_data *subpmic =
		container_of(to_delayed_work(work), struct qcom_subpmic_data,
			     usb_type_work);
	int pd_active = 0;
	int real_type = 0;
	int online = 0;
	int rc;

	/*
	 * A port that has negotiated Power Delivery reports its own type and
	 * its own comings and goings; reading the charger's idea of it as
	 * well would announce the same connection twice.
	 */
	if (subpmic->support_typec) {
		protocol_class_pd_get_pd_active(TYPEC_PORT_1, &pd_active);
		if (pd_active)
			return;
	}

	rc = mca_adsp_glink_read_prop(ADSP_PROP_ID_USB_ONLINE, &online,
				      sizeof(online));
	if (rc < 0) {
		mca_log_err("Failed to read usb_online rc=%d\n", rc);
		goto retry;
	}

	/*
	 * Whatever came up before this driver did not hear the charger being
	 * set up, so it is told once, the first time round.
	 */
	if (!subpmic->pmic_init_notified) {
		mca_event_block_notify(MCA_EVENT_TYPE_HW_INFO,
				       MCA_EVENT_PMIC_INIT_DONE, NULL);
		subpmic->pmic_init_notified = true;
	}

	if (subpmic->usb_online != online) {
		mca_log_err("usb online: %d\n", online);
		subpmic->usb_online = online;

		if (online) {
			protocol_class_pd_set_pd_active(TYPEC_PORT_0, 1);
			mca_event_block_notify(MCA_EVENT_TYPE_CHARGER_CONNECT,
					       MCA_EVENT_USB_CONNECT, NULL);
		} else {
			protocol_class_pd_set_pd_active(TYPEC_PORT_0, 0);
			mca_event_block_notify(MCA_EVENT_TYPE_CHARGER_CONNECT,
					       MCA_EVENT_USB_DISCONNECT, NULL);
			pm_relax(subpmic->dev);
		}
	}

	rc = mca_adsp_glink_read_prop(ADSP_PROP_ID_USB_USB_REAL_TYPE,
				      &real_type, sizeof(real_type));
	if (rc < 0) {
		mca_log_err("Failed to read USB_ADAP_TYPE rc=%d\n", rc);
		goto retry;
	}

	if (subpmic->real_type != real_type) {
		mca_log_err("usb real_type: %d\n", real_type);
		subpmic->real_type = real_type;
	} else if (!subpmic->glink_down) {
		return;
	}

	/*
	 * The type is announced again after the link came back even when it
	 * has not changed: whoever was listening across the outage does not
	 * know what it missed.
	 */
	subpmic->glink_down = false;
	mca_event_block_notify(MCA_EVENT_TYPE_CHARGE_TYPE,
			       MCA_EVENT_CHARGE_TYPE_CHANGE,
			       &subpmic->real_type);

	return;

retry:
	queue_delayed_work(system_wq, &subpmic->usb_type_work,
			   msecs_to_jiffies(2000));
}

/**
 * qcom_subpmic_notify_change_work() - act on what the ADSP said
 * @work: the work
 *
 * These three notifications need another exchange with the ADSP, or a call
 * into a driver that may sleep, so they cannot be acted on where they land.
 */
static void qcom_subpmic_notify_change_work(struct work_struct *work)
{
	struct qcom_subpmic_data *subpmic =
		container_of(work, struct qcom_subpmic_data,
			     notify_change_work);
	struct adsp_notify_change_node *entry;
	unsigned long flags;

	while (true) {
		spin_lock_irqsave(&subpmic->notify_lock, flags);
		entry = list_first_entry_or_null(&subpmic->adsp_notify_list,
						 struct adsp_notify_change_node,
						 node);
		if (entry)
			list_del(&entry->node);
		spin_unlock_irqrestore(&subpmic->notify_lock, flags);

		if (!entry)
			break;

		switch (entry->notification) {
		case MCA_CHARGER_ADT_CAP_CHANGE_NOTIFY:
			mca_event_block_notify(MCA_EVENT_TYPE_CHARGE_TYPE,
					       MCA_EVENT_CHARGE_CAP_CHANGE,
					       NULL);
			break;
		case MCA_CHARGER_ENABLE_BOOST_NOTIFY:
			/*
			 * The boost coming on is announced first, so that
			 * whatever has to get out of its way has done so
			 * before the gate in front of it moves.
			 */
			mca_event_block_notify(MCA_EVENT_TYPE_HW_INFO,
					       MCA_EVENT_BOOST_STS,
					       &entry->value);

			/*
			 * The gate is closed while the boost is running and
			 * opened again when it stops, so what it is set to is
			 * the opposite of what was announced.
			 */
			if (subpmic->support_ovpgate) {
				mca_log_err("enable_ovpgate: %d\n",
					    !entry->value);
				platform_class_cp_enable_ovpgate_with_check(
					CP_ROLE_MASTER, OTG_TYPE,
					!entry->value);
			}

			if (entry->value)
				break;

			/*
			 * With the boost off the reverse charging path is
			 * free again, and what was waiting for it is told so.
			 */
			subpmic->revchg_notify.value = MCA_REVCHG_BOOST_FREE;
			mca_event_block_notify(MCA_EVENT_TYPE_HW_INFO,
					       MCA_EVENT_QUICK_REVCHG_CHANGE,
					       &subpmic->revchg_notify);
			subpmic->revchg_notify.type = 0;
			subpmic->revchg_notify.value = 0;
			break;
		case MCA_CHARGER_TYPEC_CHANGE_NOTIFY:
			mca_event_block_notify(MCA_EVENT_TYPE_TYPEC_PORT_STATUS,
					       MCA_EVENT_TYPEC_PORT_CHANGE,
					       NULL);
			break;
		default:
			break;
		}

		kfree(entry);
	}
}

/**
 * qcom_subpmic_notify_cb() - the ADSP has something to say
 * @notification: what it is
 * @buf:          the number that came with it
 * @len:          how long that is
 * @data:         this driver's state
 *
 * This runs where the message arrives, so it does as little as possible.  A
 * wakeup is held briefly so that a notification arriving as the board suspends
 * is still acted on, and anything needing another exchange with the ADSP is
 * queued rather than done here.
 */
static void qcom_subpmic_notify_cb(int notification, void *buf, int len,
				   void *data)
{
	struct qcom_subpmic_data *subpmic = data;
	struct mca_event_notify_data uevent;
	struct adsp_notify_change_node *entry;
	char ubuf[64];
	unsigned long flags;
	int value;

	if (!subpmic || notification > MCA_CHARGER_CID_STS_NOTIFY)
		return;

	value = *(int *)buf;

	pm_wakeup_dev_event(subpmic->dev, SUBPMIC_WAKE_MS, true);

	switch (notification) {
	case MCA_CHARGER_ADT_CAP_CHANGE_NOTIFY:
	case MCA_CHARGER_TYPEC_CHANGE_NOTIFY:
	case MCA_CHARGER_ENABLE_BOOST_NOTIFY:
		if (notification == MCA_CHARGER_ENABLE_BOOST_NOTIFY)
			mca_log_err("recv enable boost notify: %d\n", value);

		entry = kmalloc(sizeof(*entry), GFP_ATOMIC);
		if (!entry)
			return;

		entry->notification = notification;
		entry->value = value;

		spin_lock_irqsave(&subpmic->notify_lock, flags);
		list_add_tail(&entry->node, &subpmic->adsp_notify_list);
		spin_unlock_irqrestore(&subpmic->notify_lock, flags);

		queue_work(system_wq, &subpmic->notify_change_work);
		break;
	case MCA_CHARGER_LPD_CHANGE_NOTIFY:
		mca_log_err("recv adsp report lpd status: %d\n", value);
		subpmic->lpd_status = value;

		mca_log_info("moisture detection uevent notify\n");
		uevent.event = ubuf;
		uevent.event_len = scnprintf(ubuf, sizeof(ubuf),
					 "POWER_SUPPLY_MOISTURE_DET_STS=%d",
					 value);
		mca_event_report_uevent(&uevent);

		mca_event_block_notify(MCA_EVENT_TYPE_HW_INFO,
				       MCA_EVENT_LPD_STATUS_CHANGE, NULL);
		break;
	case MCA_CHARGER_CC_SHORT_VBUS_NOTIFY:
		mca_log_err("recv cc short vbus notify: %d\n", value);
		subpmic->cc_short_vbus = value;

		mca_log_info("cc short vbus uevent notify\n");
		uevent.event = ubuf;
		uevent.event_len = scnprintf(ubuf, sizeof(ubuf),
					 "POWER_SUPPLY_CC_SHORT_VBUS=%d",
					 value);
		mca_event_report_uevent(&uevent);

		mca_event_block_notify(MCA_EVENT_TYPE_HW_INFO,
				       MCA_EVENT_CC_SHORT_VBUS, NULL);
		break;
	case MCA_CHARGER_PPS_PTF_NOTIFY:
		mca_log_err("recv pps ptf notify: %d\n", value);
		subpmic->pps_ptf = value;
		mca_strategy_func_process(STRATEGY_FUNC_TYPE_QUICK_CHARGE,
					  MCA_EVENT_PPS_PTF, value);
		mca_strategy_func_process(STRATEGY_FUNC_TYPE_QUICK_WIRELESS,
					  MCA_EVENT_PPS_PTF, value);
		break;
	case MCA_CHARGER_SINK_PWR_SUSPEND_NOTIFY:
		mca_log_err("recv bsinkpowersuspend change\n");
		subpmic->bsinkpowersuspend = value;
		mca_event_block_notify(MCA_EVENT_TYPE_CHARGE_TYPE,
				       MCA_EVENT_SINK_PWR_SUSPEND_CHANGE, NULL);
		break;
	case MCA_CHARGER_QUICK_REVCHG_NOTIFY:
		mca_log_info("recv cp revert notify\n");
		break;
	case MCA_CHARGER_OTG_CP_CONFIG_NOTIFY:
		/*
		 * The firmware has decided which way the charge pump should
		 * run for the boost, and only the pump driver can act on it.
		 */
		mca_log_info("recv otg cp config notify: %d", value);
		platform_class_cp_set_revchg(CP_ROLE_MASTER, !!value);
		break;
	case MCA_CHARGER_CID_STS_NOTIFY:
		mca_log_info("recv cid_sts: %d\n", value);
		subpmic->cid_sts = value;
		mca_event_block_notify(MCA_EVENT_TYPE_HW_INFO,
				       MCA_EVENT_CID_STS, NULL);
		queue_delayed_work(system_wq, &subpmic->usb_type_work, 0);
		break;
	default:
		break;
	}
}

/**
 * qcom_subpmic_glink_down_cb() - the link to the ADSP has gone
 * @data: this driver's state
 *
 * Nothing can be asked of the firmware until it is back, so the stack is told
 * the charger is not ready rather than being given stale answers.
 *
 * Return: 0.
 */
static int qcom_subpmic_glink_down_cb(void *data)
{
	struct qcom_subpmic_data *subpmic = data;

	mca_log_info("glink down\n");
	subpmic->glink_down = true;

	/*
	 * Nothing that was asked of the firmware survives it restarting, so
	 * the charging stack is told to stop trusting what it was told.
	 */
	mca_event_block_notify(MCA_EVENT_CHARGE_STATUS,
			       MCA_EVENT_CHARGE_ABNORMAL, NULL);

	return 0;
}

/**
 * qcom_subpmic_sync_cb() - the link to the ADSP is back
 * @data: this driver's state
 *
 * A firmware that has restarted knows nothing about this board, so the whole
 * configuration goes across again before the charger is called ready.
 *
 * Return: 0.
 */
static int qcom_subpmic_sync_cb(void *data)
{
	struct qcom_subpmic_data *subpmic = data;

	mca_log_info("glink up\n");

	mca_event_block_notify(MCA_EVENT_CHARGE_STATUS,
			       MCA_EVENT_CHARGE_RESTORE, NULL);

	/*
	 * What the firmware knew is gone, so it is told again and everything
	 * that was read from it is read once more.
	 */
	queue_delayed_work(system_wq, &subpmic->usb_type_work, 0);
	queue_delayed_work(system_wq, &subpmic->sync_cfg_work, 0);

	return 0;
}

static struct mca_adsp_call_back subpmic_glink_cb = {
	.glink_down_cb	= qcom_subpmic_glink_down_cb,
	.sync_data_cb	= qcom_subpmic_sync_cb,
	.notify_cb	= qcom_subpmic_notify_cb,
};

/**
 * qcom_subpmic_ship_mode() - enter ship mode on the way down
 * @nb:     the reboot notifier
 * @action: whether this is a restart or a power-off
 * @data:   unused
 *
 * Ship mode disconnects the battery, so it is only ever entered on the way
 * down, and only when it was asked for.  Which kind of shutdown this is goes
 * across with the request: the firmware powers back up differently after a
 * restart than after a power-off.
 *
 * A charge pump that is holding its over-voltage gate closed would keep the
 * board alive through the disconnect, so the gate is dropped first.
 *
 * Return: NOTIFY_DONE.
 */
static int qcom_subpmic_ship_mode(struct notifier_block *nb,
				  unsigned long action, void *data)
{
	struct qcom_subpmic_data *subpmic =
		container_of(nb, struct qcom_subpmic_data, reboot_notifier);
	int chip_vendor = -1;
	int rc;

	if (action != SYS_RESTART && action != SYS_POWER_OFF)
		return NOTIFY_DONE;

	if (!subpmic->ship_mode_en)
		return NOTIFY_DONE;

	mca_log_info("set adsp shipmode\n");

	platform_class_cp_get_chip_vendor(CP_ROLE_MASTER, &chip_vendor);
	mca_log_info("cp type: %d\n", chip_vendor);

	/*
	 * The SC8541 releases its gate on its own; every other pump, and a
	 * pump that could not be identified, has to be told.
	 */
	if (chip_vendor != SC8541_VENDOR)
		platform_class_cp_enable_ovpgate(CP_ROLE_MASTER, false);

	rc = mca_adsp_glink_write_prop(ADSP_PROP_ID_SHIP_MODE, &action,
				       sizeof(action));
	if (rc)
		pr_err("Failed to write ship mode: %d\n", rc);

	return NOTIFY_DONE;
}

/**
 * qcom_subpmic_shutdown_cb() - tell the firmware the board is going down
 * @nb:     the shutdown notifier
 * @action: unused
 * @data:   unused
 *
 * The firmware keeps charging after the kernel has stopped, and needs to know
 * that what it is charging is no longer running.
 *
 * Return: NOTIFY_DONE.
 */
static int qcom_subpmic_shutdown_cb(struct notifier_block *nb,
				    unsigned long action, void *data)
{
	unsigned long mode = action;
	int rc;

	/*
	 * Which of the two it is matters to the firmware, so pass the action
	 * on rather than a fixed value.  SYS_HALT leaves the board powered and
	 * is not one of them.
	 */
	if (action != SYS_RESTART && action != SYS_POWER_OFF)
		return NOTIFY_DONE;

	mca_log_info("start adsp shutdown\n");

	rc = mca_adsp_glink_write_prop(ADSP_PROP_ID_SHUTDOWN_MODE, &mode,
				       sizeof(mode));
	if (rc)
		pr_err("Failed to write shutdown cmd to adsp: %d\n", rc);

	return NOTIFY_DONE;
}

static const struct platform_class_buckchg_ops subpmic_buckchg_ops = {
	.is_init_ok			= qcom_subpmic_is_init_ok,
	.get_online			= qcom_subpmic_get_online,
	.is_charge_done			= qcom_subpmic_is_charge_done,
	.get_input_curr_lmt		= qcom_subpmic_get_input_current_limit,
	.get_bus_curr			= qcom_subpmic_get_bus_curr,
	.get_bus_volt			= qcom_subpmic_get_bus_volt,
	.get_usb_sns_volt		= qcom_subpmic_get_usb_sns_volt,
	.get_sys_volt			= qcom_subpmic_get_vsys_volt,
	.get_chg_status			= qcom_subpmic_get_chg_status,
	.get_chg_type			= qcom_subpmic_get_chg_type,
	.get_term_curr			= qcom_subpmic_get_term_current,
	.get_term_volt			= qcom_subpmic_get_term_volt,
	.get_wls_curr			= qcom_subpmic_get_wls_curr,
	.set_input_curr_lmt		= qcom_subpmic_set_input_current_limit,
	.set_wls_input_curr_lmt		= qcom_subpmic_set_wls_input_current_limit,
	.set_input_volt_lmt		= qcom_subpmic_set_input_volt_limit,
	.set_ichg			= qcom_subpmic_set_charge_current,
	.set_chg			= qcom_subpmic_enable_charging,
	.set_buck_fsw			= qcom_subpmic_set_buck_fsw,
	.set_hiz			= qcom_subpmic_set_input_suspend,
	.set_wls_hiz			= qcom_subpmic_set_wireless_input_suspend,
	.set_term_curr			= qcom_subpmic_set_term_current,
	.set_term_volt			= qcom_subpmic_set_term_volt,
	.set_prechg_volt		= qcom_subpmic_set_prechg_volt,
	.set_prechg_curr		= qcom_subpmic_set_prechg_current,
	.set_qc_volt			= qcom_subpmic_set_qc_volt,
	.set_usb_aicl_cont_thd		= qcom_subpmic_set_usb_aicl_cont_thd,
	.get_usb_aicl_cont_thd		= qcom_subpmic_get_usb_aicl_cont_thd,
	.set_opt_fws			= qcom_subpmic_set_opt_fws,
	.usb_adapter_allow_override	= qcom_subpmic_usb_adapter_allow_override,
	.set_qc3_volt			= qcom_subpmic_set_qc3_volt,
	.get_otg_boost_src		= qcom_subpmic_get_otg_boost_src,
	.get_otg_gate_enable_status	= qcom_subpmic_get_otg_gate_enable_status,
	.get_otg_boost_enable_status	= qcom_subpmic_get_otg_boost_enable_status,
	.set_boost_enable		= qcom_subpmic_set_boost_enable,
	.set_boost_voltage		= qcom_subpmic_set_boost_voltage,
	.set_aicl_enable		= qcom_subpmic_set_aicl_enable,
	.set_rerun_aicl			= qcom_subpmic_set_rerun_aicl,
	.set_restart_aicl		= qcom_subpmic_set_restart_aicl,
	.is_support_cid			= qcom_subpmic_is_support_cid,
	.set_ship_mode			= qcom_subpmic_set_ship_mode,
	.get_ship_mode			= qcom_subpmic_get_ship_mode,
	.set_wls_vdd_flag		= qcom_subpmic_set_wls_vdd_flag,
	.get_lpd_enable			= qcom_subpmic_get_lpd_enable,
	.get_lpd_status			= qcom_subpmic_get_lpd_status,
	.get_lpd_sbu1			= qcom_subpmic_get_lpd_sbu1,
	.get_lpd_sbu2			= qcom_subpmic_get_lpd_sbu2,
	.get_lpd_cc1			= qcom_subpmic_get_lpd_cc1,
	.get_lpd_cc2			= qcom_subpmic_get_lpd_cc2,
	.get_lpd_dp			= qcom_subpmic_get_lpd_dp,
	.get_lpd_dm			= qcom_subpmic_get_lpd_dm,
	.set_lpd_sbu1			= qcom_subpmic_set_lpd_sbu1,
	.set_lpd_control		= qcom_subpmic_set_lpd_control,
	.get_lpd_control		= qcom_subpmic_get_lpd_control,
	.set_lpd_uart_control		= qcom_subpmic_set_lpd_uart_control,
	.get_lpd_uart_control		= qcom_subpmic_get_lpd_uart_control,
	.get_pack_vbat			= qcom_subpmic_get_pack_vbat,
	.get_pack_ibat			= qcom_subpmic_get_pack_ibat,
	.set_eu_model			= qcom_subpmic_set_eu_model,
	.get_aicl_status		= qcom_subpmic_get_aicl_status,
	.set_too_hot_limit		= qcom_subpmic_set_too_hot_limit,
	.get_pack_tbat			= qcom_subpmic_get_pack_tbat,
};

static const struct platform_bc12_class_ops subpmic_bc12_ops = {
	.bc12_det_en		= qcom_subpmic_bc12_det_en,
	.get_charge_type	= qcom_subpmic_get_usb_real_type,
};

static const struct protocol_class_qc_ops subpmic_qc_ops = {
	.protocol_qc_get_qc_type	= qcom_subpmic_get_usb_real_type,
	.protocol_qc_set_volt		= qcom_subpmic_set_qc_volt,
	.protocol_qc_set_volt_cmd	= qcom_subpmic_set_qc_volt_cmd,
};

/**
 * qti_battery_charger_get_prop() - answer for the absent QTI charger driver
 * @name: which power supply is being asked about
 * @prop: which property
 * @val:  filled in with the answer
 *
 * Audio and haptics ask this to find out how much the boost can supply.  The
 * QTI driver that would answer is not present -- the ADSP owns the charger and
 * this driver speaks to it instead -- so callers are told the property is not
 * available rather than being left with an unresolved symbol.
 *
 * Return: -ENODEV.
 */
int qti_battery_charger_get_prop(const char *name,
				 enum battery_charger_prop prop, int *val)
{
	return -ENODEV;
}
EXPORT_SYMBOL_GPL(qti_battery_charger_get_prop);

/*
 * The haptics driver watches for the boost going up and down so it can hold
 * off while the supply is unstable.  On this board the boost is the ADSP's and
 * announces nothing, so there is no chain to join.
 */
/*
 * The audio driver listens here for the boost that drives the speakers being
 * turned on and off, so that it can back off its own gain while it is.
 */
static RAW_NOTIFIER_HEAD(hboost_notifier);

int register_hboost_event_notifier(struct notifier_block *nb)
{
	return raw_notifier_chain_register(&hboost_notifier, nb);
}
EXPORT_SYMBOL_GPL(register_hboost_event_notifier);

int unregister_hboost_event_notifier(struct notifier_block *nb)
{
	return raw_notifier_chain_unregister(&hboost_notifier, nb);
}
EXPORT_SYMBOL_GPL(unregister_hboost_event_notifier);

/**
 * qcom_subpmic_parse_dt() - read what the firmware needs to be told
 * @subpmic: this driver's state
 *
 * Return: 0.
 */
static int qcom_subpmic_parse_dt(struct qcom_subpmic_data *subpmic)
{
	struct device_node *np = subpmic->dev->of_node;

	subpmic->support_2s_charging = !!of_find_property(np,
						"mi,support-2s-charging", NULL);
	subpmic->support_dual_panel = !!of_find_property(np,
						"mi,support-dual-panel", NULL);
	subpmic->support_multi_bc12 = !!of_find_property(np,
						"mi,support-multi-bc12", NULL);
	subpmic->support_cid = !!of_find_property(np, "mi,support-cid", NULL);

	mca_parse_dts_u32(np, "otg_boost_src", &subpmic->otg_cfg.boost_src,
			  BOOST_SRC_CHARGER);
	mca_parse_dts_u32(np, "gpio_chip_type",
			  &subpmic->otg_cfg.gpio_chip_type, GPIO_SOC_TYPE);
	/*
	 * Both are pin numbers, so a board that names neither has to end up
	 * with something that is not a pin: zero is a real one.
	 */
	mca_parse_dts_u32(np, "vdd_boost_en", &subpmic->otg_cfg.vdd_boost_gpio,
			  -1);
	mca_parse_dts_u32(np, "otg_ovp_en", &subpmic->otg_cfg.ovp_en_gpio, -1);
	mca_parse_dts_u32(np, "cid_gpio_int", &subpmic->cid_gpio_int, 0);

	mca_log_info("dts sync otg_cfg_src= %d,gpio_chip_type=%d, vdd_boost_gpio=%d, ovp_en_gpio=%d\n",
		     subpmic->otg_cfg.boost_src,
		     subpmic->otg_cfg.gpio_chip_type,
		     subpmic->otg_cfg.vdd_boost_gpio,
		     subpmic->otg_cfg.ovp_en_gpio);
	mca_log_info("dts sync cid_gpio_int= %#x\n", subpmic->cid_gpio_int);

	return 0;
}

static int qcom_subpmic_probe(struct platform_device *pdev)
{
	struct mca_hwid_info *hwid;
	struct qcom_subpmic_data *subpmic;
	int rc;

	mca_log_info("probe start\n");

	subpmic = devm_kzalloc(&pdev->dev, sizeof(*subpmic), GFP_KERNEL);
	if (!subpmic)
		return -ENOMEM;

	subpmic->dev = &pdev->dev;
	platform_set_drvdata(pdev, subpmic);

	INIT_LIST_HEAD(&subpmic->adsp_notify_list);
	spin_lock_init(&subpmic->notify_lock);
	INIT_DELAYED_WORK(&subpmic->usb_type_work,
			  qcom_subpmic_update_usb_type_work);
	INIT_DELAYED_WORK(&subpmic->sync_cfg_work, qcom_subpmic_sync_cfg_work);
	INIT_WORK(&subpmic->notify_change_work,
		  qcom_subpmic_notify_change_work);

	rc = qcom_subpmic_parse_dt(subpmic);
	if (rc) {
		mca_log_err("%s parse dt fail\n", __func__);
		return rc;
	}

	device_set_wakeup_capable(subpmic->dev, true);
	device_wakeup_enable(subpmic->dev);

	rc = mca_adsp_glink_resister_ops(&subpmic_glink_cb, subpmic);
	if (rc) {
		mca_log_err("%s register glink ops fail\n", __func__);
		return rc;
	}

	rc = platform_class_buckchg_ops_register(MAIN_BUCK_CHARGER,
						 &subpmic_buckchg_ops, subpmic);
	if (rc) {
		mca_log_err("%s register buckchg ops fail\n", __func__);
		return rc;
	}

	rc = platform_bc12_class_ops_register(BC12_MAIN_ROLE, &subpmic_bc12_ops,
					      subpmic);
	if (rc) {
		mca_log_err("%s register protocol ops fail\n", __func__);
		return rc;
	}

	rc = protocol_class_qc_register_ops(ADAPTER_PROTOCOL_QC,
					    &subpmic_qc_ops, subpmic);
	if (rc) {
		mca_log_err("%s register qc ops fail\n", __func__);
		return rc;
	}

	subpmic->reboot_notifier.notifier_call = qcom_subpmic_ship_mode;
	register_reboot_notifier(&subpmic->reboot_notifier);

	subpmic->shutdown_notifier.notifier_call = qcom_subpmic_shutdown_cb;
	register_reboot_notifier(&subpmic->shutdown_notifier);

	/*
	 * Which region the phone was sold in decides whether its charging is
	 * capped by regulation, and the firmware has to be told.
	 */
	hwid = mca_get_hwid_info();
	if (hwid)
		qcom_subpmic_set_eu_model(subpmic,
					  hwid->country_version == CountryGlobal);

	g_subpmic = subpmic;

	/* Read what is attached now rather than waiting to be told. */
	queue_delayed_work(system_wq, &subpmic->usb_type_work, 0);
	queue_delayed_work(system_wq, &subpmic->sync_cfg_work,
			   msecs_to_jiffies(SUBPMIC_SYNC_START_MS));

	mca_log_info("probe ok\n");

	return 0;
}

static int qcom_subpmic_remove(struct platform_device *pdev)
{
	struct qcom_subpmic_data *subpmic = platform_get_drvdata(pdev);

	unregister_reboot_notifier(&subpmic->shutdown_notifier);
	unregister_reboot_notifier(&subpmic->reboot_notifier);
	cancel_delayed_work_sync(&subpmic->sync_cfg_work);
	cancel_delayed_work_sync(&subpmic->usb_type_work);
	cancel_work_sync(&subpmic->notify_change_work);
	g_subpmic = NULL;

	return 0;
}

static void qcom_subpmic_shutdown(struct platform_device *pdev)
{
	struct qcom_subpmic_data *subpmic = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&subpmic->sync_cfg_work);
	cancel_delayed_work_sync(&subpmic->usb_type_work);
}

static const struct of_device_id qcom_subpmic_match[] = {
	{ .compatible = "mca,qcom_subpmic" },
	{ }
};
MODULE_DEVICE_TABLE(of, qcom_subpmic_match);

static struct platform_driver qcom_subpmic_driver = {
	.driver = {
		.name		= "qcom_subpmic",
		.of_match_table	= qcom_subpmic_match,
	},
	.probe		= qcom_subpmic_probe,
	.remove		= qcom_subpmic_remove,
	.shutdown	= qcom_subpmic_shutdown,
};
module_platform_driver(qcom_subpmic_driver);

MODULE_DESCRIPTION("MCA buck charger through the ADSP");
MODULE_LICENSE("GPL");
