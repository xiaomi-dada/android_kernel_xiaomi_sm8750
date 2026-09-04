// SPDX-License-Identifier: GPL-2.0
/*
 *mca_strategy_buckchg.c
 *
 * mca buck charger strategy driver
 *
 * Copyright (c) 2023-2023 Xiaomi Technologies Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */
#ifndef MCA_LOG_TAG
#define MCA_LOG_TAG "mca_strategy_buckchg"
#endif

#include <linux/module.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <mca/common/mca_parse_dts.h>
#include <mca/common/mca_voter.h>
#include <mca/common/mca_log.h>
#include <mca/common/mca_event.h>
#include <mca/common/mca_charge_interface.h>
#include <mca/strategy/strategy_class.h>
#include "../strategy_wireless/inc/mca_wireless_revchg.h"
#include <mca/platform/platform_buckchg_class.h>
#include <mca/platform/platform_cp_class.h>
#include <mca/platform/platform_fg_ic_ops.h>
#include <mca/platform/platform_loadsw_class.h>
#include <mca/protocol/protocol_class.h>
#include <mca/protocol/protocol_pd_class.h>
#include "inc/mca_strategy_buckchg.h"
#include <mca/strategy/strategy_fg_class.h>
#include <mca/strategy/strategy_wireless_class.h>
#include <mca/smartchg/smart_chg_class.h>
#include <mca/common/mca_hwid.h>
#include <linux/hwid.h>
//#include "hwid.h"
#include <linux/types.h>
#include <linux/jiffies.h>
#include <linux/time.h>
#include <linux/timekeeping.h>
#include <mca/common/mca_charge_mievent.h>
#include <mca/common/mca_workqueue.h>
#include <mca/common/mca_smem.h>
#include <mca/common/mca_event.h>

#define CHECK_VBUS_9V_HIGH_TH		7600
#define CHECK_VBUS_5V_LOW_TH		6000
#define DEFAULT_PD_CURRENT_MA		500

static void strategy_buckchg_set_charge_volt(struct strategy_buckchg_dev *info, int target_volt);
static void strategy_buck_update_req_volt(struct strategy_buckchg_dev *info);
static void strategy_buckchg_exit_wireless_revchg(struct strategy_buckchg_dev *info);
static void strategy_wls_revchg_monitor_workfunc(struct work_struct *work);
static int strategy_buckchg_check_charger_change(struct strategy_buckchg_dev *info);
static void strategy_buckchg_sw_cv_start(struct strategy_buckchg_dev *info);
static void strategy_buckchg_sw_cv_stop(struct strategy_buckchg_dev *info);

static struct strategy_buckchg_dev *g_buckchg_info;

static int strategy_class_buckchg_ops_set_input_volt(struct strategy_buckchg_dev *info,
	int mv)
{
	int ret;

	ret = platform_class_buckchg_ops_set_input_volt_lmt(MAIN_BUCK_CHARGER, mv);
	if (info && info->support_multi_buck)
		ret |= platform_class_buckchg_ops_set_input_volt_lmt(AUX_BUCK_CHARGER, mv);

	return ret;
}

static int strategy_class_buckchg_ops_set_chg(struct strategy_buckchg_dev *info, bool en)
{
	int ret;

	ret = platform_class_buckchg_ops_set_chg(MAIN_BUCK_CHARGER, en);
	if (info && info->support_multi_buck)
		ret |= platform_class_buckchg_ops_set_chg(AUX_BUCK_CHARGER, en);

	return ret;
}

static int strategy_class_buckchg_ops_set_opt_fws(struct strategy_buckchg_dev *info, int mv)
{
	int ret;

	ret = platform_class_buckchg_ops_set_opt_fws(MAIN_BUCK_CHARGER, mv);
	if (info && info->support_multi_buck)
		ret |= platform_class_buckchg_ops_set_opt_fws(AUX_BUCK_CHARGER, mv);

	return ret;
}

static int strategy_class_buckchg_ops_adc_enable(struct strategy_buckchg_dev *info, bool en)
{
	int ret;

	ret = platform_class_buckchg_ops_adc_enable(MAIN_BUCK_CHARGER, en);
	if (info && info->support_multi_buck)
		ret |= platform_class_buckchg_ops_adc_enable(AUX_BUCK_CHARGER, en);

	return ret;
}

static int strategy_class_buckchg_ops_set_usb_aicl_cont_thd(struct strategy_buckchg_dev *info, int mv)
{
	int ret;

	ret = platform_class_buckchg_ops_set_usb_aicl_cont_thd(MAIN_BUCK_CHARGER, mv);
	if (info && info->support_multi_buck)
		ret |= platform_class_buckchg_ops_set_usb_aicl_cont_thd(AUX_BUCK_CHARGER, mv);

	return ret;
}

static void strategy_buckchg_parse_dt(struct strategy_buckchg_dev *info)
{
	u32 idata[REV_USBIN_TYPE_MAX] = { 0 };
	int ret = 0;

	mca_parse_dts_u32(info->dev->of_node, "batt_type",
			  &info->batt_type, MCA_BATTERY_TYPE_SINGLE);
	mca_parse_dts_u32(info->dev->of_node, "support_multi_buck",
		&info->support_multi_buck, STATEGY_SUPPORT_MULTI_BUCK);
	mca_parse_dts_u32(info->dev->of_node, "ship_mode_chip",
		&info->ship_mode_chip, MAIN_BUCK_CHARGER);
	mca_parse_dts_u32(info->dev->of_node, "allow_start_ffc_batt_soc_thr",
		&info->allow_start_ffc_batt_soc_thr, ALLOW_START_FFC_BATT_SOC_THR);
	mca_parse_dts_u32(info->dev->of_node, "curr_terminate_compensation",
		&info->curr_term_compensation, 0);
	mca_parse_dts_u32(info->dev->of_node, "terminated_by_cp",
		&info->terminated_by_cp, 0);
	mca_parse_dts_u32(info->dev->of_node, "vusb_ovp_location",
		&info->vusb_ovp_location, BUCKCHG_VUSB_OVP_BEFORE_CP);
	mca_parse_dts_u32(info->dev->of_node, "in_dcp",
		&info->in_dcp, CHARGE_DCP_INPUT_DEFAULT);
	mca_parse_dts_u32(info->dev->of_node, "in_pd",
		&info->in_pd, CHARGE_DCP_INPUT_DEFAULT);
	mca_parse_dts_u32(info->dev->of_node, "in_pps",
		&info->in_pps, info->in_pd);
	mca_parse_dts_u32(info->dev->of_node, "in_hvdcp",
		&info->in_hvdcp, CHARGE_DCP_INPUT_DEFAULT);
	mca_parse_dts_u32(info->dev->of_node, "in_hvdcp3",
		&info->in_hvdcp3, CHARGE_DCP_INPUT_DEFAULT);
	mca_parse_dts_u32(info->dev->of_node, "in_hvdcp3p5",
		&info->in_hvdcp3p5, CHARGE_DCP_INPUT_DEFAULT);
	mca_parse_dts_u32(info->dev->of_node, "in_cdp",
		&info->in_cdp, CHARGE_CDP_INPUT_DEFAULT);
	mca_parse_dts_u32(info->dev->of_node, "in_sdp",
		&info->in_sdp, CHARGE_SDP_CHARGE_DEFAULT);
	mca_parse_dts_u32(info->dev->of_node, "in_float",
		&info->in_float, CHARGE_FLOAT_INPUT_DEFAULT);
	mca_parse_dts_u32(info->dev->of_node, "chg_dcp",
		&info->chg_dcp, CHARGE_DCP_CHARGE_DEFAULT);
	mca_parse_dts_u32(info->dev->of_node, "chg_pd",
		&info->chg_pd, CHARGE_DCP_CHARGE_DEFAULT);
	mca_parse_dts_u32(info->dev->of_node, "chg_pps",
		&info->chg_pps, info->chg_pd);
	mca_parse_dts_u32(info->dev->of_node, "chg_hvdcp",
		&info->chg_hvdcp, CHARGE_DCP_CHARGE_DEFAULT);
	mca_parse_dts_u32(info->dev->of_node, "chg_hvdcp3",
		&info->chg_hvdcp3, CHARGE_DCP_CHARGE_DEFAULT);
	mca_parse_dts_u32(info->dev->of_node, "chg_hvdcp3p5",
		&info->chg_hvdcp3p5, CHARGE_DCP_CHARGE_DEFAULT);
	mca_parse_dts_u32(info->dev->of_node, "chg_cdp",
		&info->chg_cdp, CHARGE_CDP_CHARGE_DEFAULT);
	mca_parse_dts_u32(info->dev->of_node, "chg_sdp",
		&info->chg_sdp, CHARGE_SDP_CHARGE_DEFAULT);
	mca_parse_dts_u32(info->dev->of_node, "chg_float",
		&info->chg_float, CHARGE_FLOAT_CHARGE_DEFAULT);
	mca_parse_dts_u32(info->dev->of_node, "chg_batt_auth_failed",
		&info->chg_batt_auth_failed, CHARGE_BATT_AUTH_FAIL_DEFAULT);
	mca_parse_dts_u32(info->dev->of_node, "ffc_temp_low",
		&info->ffc_temp_low, ALLOW_FFC_TEMP_LOW_THR);
	mca_parse_dts_u32(info->dev->of_node, "ffc_temp_high",
		&info->ffc_temp_high, ALLOW_FFC_TEMP_HIGH_THR);
	mca_parse_dts_u32(info->dev->of_node, "pmic_fv_compensation",
		&info->pmic_fv_compensation, 0);
	mca_parse_dts_u32(info->dev->of_node, "pmic_fv_compensation_cold",
		&info->pmic_fv_compensation_cold, 0);
	mca_parse_dts_u32(info->dev->of_node, "bat_temp_fv_comp_cold_th",
		&info->bat_temp_fv_comp_cold_th, 0);
	mca_parse_dts_u32(info->dev->of_node, "pmic_fv_compensation_hot",
		&info->pmic_fv_compensation_hot, 0);
	mca_parse_dts_u32(info->dev->of_node, "bat_temp_fv_comp_hot_th",
		&info->bat_temp_fv_comp_hot_th, BAT_TEMP_FV_COMP_HOT_TH_DEFAULT);
	mca_parse_dts_u32(info->dev->of_node, "pmic_middle_fv_compensation",
		&info->pmic_middle_fv_compensation, 0);
	mca_parse_dts_u32(info->dev->of_node, "pmic_high_fv_compensation",
		&info->pmic_high_fv_compensation, 0);
	mca_parse_dts_u32(info->dev->of_node, "pmic_wls_fv_compensation",
		&info->pmic_wls_fv_compensation, 0);
	mca_parse_dts_u32(info->dev->of_node, "pmic_iterm_compensation",
		&info->pmic_iterm_compensation, PMIC_ITERM_COMPENSATION_DEFAULT);
	mca_parse_dts_u32(info->dev->of_node, "sw_cv_fcc_step",
		&info->sw_cv_fcc_step, SW_CV_FCC_STEP_DEFAULT);
	mca_parse_dts_u32(info->dev->of_node, "sw_cv_fv_step",
		&info->sw_cv_fv_step, SW_CV_FV_STEP_DEFAULT);
	mca_parse_dts_u32(info->dev->of_node, "mca_wire_use_sc_buck",
		&info->use_sc_buck, CHARGE_BATT_USE_SC6601A_BUCK);
	mca_parse_dts_u32(info->dev->of_node, "support_hw_bc12",
		&info->hw_bc12, DEFAULT_SUPPORT_HW_BC12);
	mca_parse_dts_u32(info->dev->of_node, "vbat_fg_to_pmic_ratio",
		&info->vbat_fg_to_pmic_ratio, VBAT_FG_TO_PMIC_RATIO_DEFAULT);
	mca_parse_dts_u32(info->dev->of_node, "vote_buck_vterm_buf",
		&info->vote_buck_vterm_buf, VOTE_BUCK_VTERM_BUF_DEFAULT);
	mca_parse_dts_u32(info->dev->of_node, "vote_buck_iterm_buf",
		&info->vote_buck_iterm_buf, VOTE_BUCK_ITERM_BUF_DEFAULT);
	info->support_reverse_quick_charge = of_property_read_bool(info->dev->of_node, "support_reverse_quick_charge");
	info->support_revchg_screenon = of_property_read_bool(info->dev->of_node, "support_revchg_screenon");
	info->support_diff_temp_comp = of_property_read_bool(info->dev->of_node, "support_diff_temp_comp");
	info->support_charge_more = of_property_read_bool(info->dev->of_node, "support-charge-more");
	info->support_base_flip = of_property_read_bool(info->dev->of_node, "support-base-flip");
	info->base_flip_same = of_property_read_bool(info->dev->of_node, "base-flip-same");
	info->need_cp_to_pmic = of_property_read_bool(info->dev->of_node, "need-cp-to-pmic");
	info->support_pmic_vterm_dynamics_adjust = of_property_read_bool(info->dev->of_node,
		"support-pmic-vterm-dynamics-adjust");
	mca_parse_dts_u32(info->dev->of_node, "sw_cv_vterm_th",
		&info->sw_cv_vterm_th, STATEGY_CHARGE_VTERM_LOW_TH);
	mca_parse_dts_u32(info->dev->of_node, "full_replug_ichg_limit",
	&info->full_replug_ichg_limit, 0);


	ret = mca_parse_dts_u32_array(info->dev->of_node, "rev_req_vadp", idata, REV_USBIN_TYPE_MAX);
	if (ret) {
		info->rev_req_vadp[REV_USBIN_TYPE_PPS] = STATEGY_USBIN_WLS_REV_CHG_VOLTAGE_DEFAULT;
		info->rev_req_vadp[REV_USBIN_TYPE_OTHER] = STATEGY_USBIN_WLS_REV_CHG_VOLTAGE_DEFAULT;
	} else
		memcpy(info->rev_req_vadp, idata, sizeof(idata));
	mca_log_debug("rev_req_vadp %d %d\n", info->rev_req_vadp[REV_USBIN_TYPE_PPS],
		info->rev_req_vadp[REV_USBIN_TYPE_OTHER]);

	ret = mca_parse_dts_u32_array(info->dev->of_node, "rev_vadp_valid_h", idata, REV_USBIN_TYPE_MAX);
	if (ret) {
		info->rev_vadp_valid_h[REV_USBIN_TYPE_PPS] = STATEGY_USBIN_WLS_REV_CHG_VOLTAGE_BUF_H;
		info->rev_vadp_valid_h[REV_USBIN_TYPE_OTHER] = STATEGY_USBIN_WLS_REV_CHG_VOLTAGE_BUF_H;
	} else
		memcpy(info->rev_vadp_valid_h, idata, sizeof(idata));
	mca_log_debug("rev_vadp_valid_h %d %d\n", info->rev_vadp_valid_h[REV_USBIN_TYPE_PPS],
		info->rev_vadp_valid_h[REV_USBIN_TYPE_OTHER]);

	ret = mca_parse_dts_u32_array(info->dev->of_node, "rev_vadp_valid_l", idata, REV_USBIN_TYPE_MAX);
	if (ret) {
		info->rev_vadp_valid_l[REV_USBIN_TYPE_PPS] = STATEGY_USBIN_WLS_REV_CHG_VOLTAGE_BUF_L;
		info->rev_vadp_valid_l[REV_USBIN_TYPE_OTHER] = STATEGY_USBIN_WLS_REV_CHG_VOLTAGE_BUF_L;
	} else
		memcpy(info->rev_vadp_valid_l, idata, sizeof(idata));
	mca_log_debug("rev_vadp_valid_l %d %d\n", info->rev_vadp_valid_l[REV_USBIN_TYPE_PPS],
		info->rev_vadp_valid_l[REV_USBIN_TYPE_OTHER]);
}

static int strategy_buckchg_enable(struct mca_votable *votable,
	void *data, int effective_result, const char *effective_client)
{
	struct strategy_buckchg_dev *info = data;

	if (!data)
		return -1;
	mca_log_err("%d\n", effective_result);
	if (effective_result)
		return strategy_class_buckchg_ops_set_chg(info, true);
	else
		return strategy_class_buckchg_ops_set_chg(info, false);
}

static int strategy_buckchg_input_suspend(struct mca_votable *votable,
	void *data, int effective_result, const char *effective_client)
{
	struct strategy_buckchg_dev *info = data;

	if (!data)
		return -1;
	mca_log_err("%d\n", effective_result);
	if (effective_result)
		mca_vote(info->input_limit_voter, "input_suspend", true, 0);
	else
		mca_vote(info->input_limit_voter, "input_suspend", false, 0);
	return 0;
}

static int strategy_buckchg_input_voltage(struct mca_votable *votable,
	void *data, int effective_result, const char *effective_client)
{
	struct strategy_buckchg_dev *info = data;

	if (!data)
		return -1;

	/*
	 * Only record which rail the input is on.  The four per-rail voters
	 * each cast their own vote when they see the rail they belong to, so
	 * casting one here as well puts a stale limit in place until the next
	 * time one of them runs -- and passes a current where an enable flag
	 * belongs.
	 */
	info->hvdcp_allow_flag = effective_result;
	info->proc_data.voltage = effective_result ? STATEGY_CHARGE_VBUS_9V :
						     STATEGY_CHARGE_VBUS_5V;

	return 0;
}

static int strategy_buckchg_process_multi_input_limit(struct strategy_buckchg_dev *info,
	int result)
{
	return 0;
}

static int strategy_buckchg_input_limit(struct mca_votable *votable,
	void *data, int effective_result, const char *effective_client)
{
	struct strategy_buckchg_dev *info = data;
	struct timespec64 ts;
	u32 is_zero_speed = 0;
	int result = effective_result;

	if (!data)
		return -1;

	/*
	 * A pack that started from zero cannot take a small input without
	 * collapsing again, so for the first minute after boot the limit has a
	 * floor under it.  UEFI is what noticed, and left the answer in shared
	 * memory.
	 */
	ktime_get_boottime_ts64(&ts);
	if ((u64)ts.tv_sec < BUCKCHG_ZERO_SPEED_BOOT_WINDOW_SEC) {
		get_smem_battery_info(&is_zero_speed);
		if (is_zero_speed && effective_result) {
			result = effective_result < BUCKCHG_ZERO_SPEED_INPUT_MIN ?
					BUCKCHG_ZERO_SPEED_INPUT_MIN : effective_result;
			mca_log_err("is_zero_speed, result = %d, effective_result = %d\n", result, effective_result);
		}
	}

	mca_log_err("set input current %d\n", result);

	if (info->support_multi_buck)
		return strategy_buckchg_process_multi_input_limit(info, result);

	return platform_class_buckchg_ops_set_input_curr_lmt(MAIN_BUCK_CHARGER, result);
}

static int strategy_buckchg_process_multi_charge_limit(struct strategy_buckchg_dev *info,
	int result)
{
	return 0;
}

static int strategy_buckchg_charge_limit(struct mca_votable *votable,
	void *data, int effective_result, const char *effective_client)
{
	struct strategy_buckchg_dev *info = data;

	if (!data)
		return -1;

	mca_log_err("set chg current %d\n", effective_result);

	effective_result *= info->vbat_fg_to_pmic_ratio;

	if (info->support_multi_buck)
		return strategy_buckchg_process_multi_charge_limit(info, effective_result);

	return platform_class_buckchg_ops_set_ichg(MAIN_BUCK_CHARGER, effective_result);
}

static int strategy_buckchg_process_multi_vterm(struct strategy_buckchg_dev *info,
	int result)
{
	return 0;
}

/*
 * The termination voltage the PMIC is given is the voted one plus a headroom
 * that covers the drop between the gauge's sense point and the charger's.  How
 * much headroom depends on the pack, on how cold or hot it is, and -- where
 * the board supports it -- on which of three temperature bands it sits in.
 */
static int strategy_buckchg_set_vterm(struct mca_votable *votable,
	void *data, int effective_result, const char *effective_client)
{
	struct strategy_buckchg_dev *info = data;
	int batt_temp = 0;
	int vterm;

	if (!data)
		return -1;

	/*
	 * The flip base charges a second pack through the same charger, so
	 * the headroom is the base's rather than the phone's.
	 */
	if (info->support_base_flip) {
		if (!strategy_class_fg_get_fastcharge())
			info->pmic_fv_compensation = BASE_FLIP_FV_COMP_SLOW;
		else if (info->wls_online)
			info->pmic_fv_compensation = info->pmic_wls_fv_compensation;
		else
			info->pmic_fv_compensation = BASE_FLIP_FV_COMP_FAST;
	}

	strategy_class_fg_ops_get_temperature(&batt_temp);
	batt_temp /= 10;

	if (batt_temp < info->bat_temp_fv_comp_cold_th && info->support_diff_temp_comp)
		vterm = info->pmic_fv_compensation_cold + effective_result;
	else if (batt_temp > info->bat_temp_fv_comp_hot_th && info->support_diff_temp_comp)
		vterm = info->pmic_fv_compensation_hot + effective_result;
	else
		vterm = info->pmic_fv_compensation + effective_result;

	if (info->support_pmic_vterm_dynamics_adjust) {
		if (!strategy_class_fg_get_fastcharge()) {
			/* No headroom at all while the pack is not fast charging. */
			vterm = effective_result;
		} else {
			strategy_class_fg_ops_get_temperature(&batt_temp);
			batt_temp /= 10;
			if (batt_temp >= FV_COMP_WARM_LOW && batt_temp <= FV_COMP_WARM_HIGH)
				vterm = info->pmic_fv_compensation + effective_result;
			else if (batt_temp >= FV_COMP_MIDDLE_LOW && batt_temp <= FV_COMP_MIDDLE_HIGH)
				vterm = info->pmic_middle_fv_compensation + effective_result;
			else if (batt_temp >= FV_COMP_HIGH_LOW)
				vterm = info->pmic_high_fv_compensation + effective_result;
		}
	}

	/* A base that carries the same pack as the phone shares its headroom. */
	if (info->base_flip_same) {
		if (strategy_class_fg_get_fastcharge())
			vterm = info->pmic_fv_compensation + effective_result;
		else if (batt_temp >= BASE_FLIP_SAME_HOT_LOW &&
			 batt_temp <= BASE_FLIP_SAME_HOT_HIGH)
			vterm = effective_result + BASE_FLIP_SAME_HOT_FV_COMP;
		else
			vterm = effective_result;
	}

	vterm = (vterm / info->vbat_fg_to_pmic_ratio) + info->vote_buck_vterm_buf;

	if (info->support_multi_buck)
		return strategy_buckchg_process_multi_vterm(info, vterm);

	return platform_class_buckchg_ops_set_term_volt(MAIN_BUCK_CHARGER, vterm);
}

static int strategy_buckchg_process_multi_iterm(struct strategy_buckchg_dev *info,
	int result)
{
	return 0;
}

/*
 * Termination current works the same way: the voted value plus a headroom that
 * depends on which pack is being charged and how warm it is.
 */
static int strategy_buckchg_set_iterm(struct mca_votable *votable,
	void *data, int effective_result, const char *effective_client)
{
	struct strategy_buckchg_dev *info = data;
	int batt_temp = 0;

	if (!data)
		return -1;

	if (info->support_base_flip) {
		if (!strategy_class_fg_get_fastcharge()) {
			info->pmic_iterm_compensation =
				(info->parallel_iterm == BASE_FLIP_ITERM_SLOW) ?
					BASE_FLIP_ITERM_COMP_SLOW :
					BASE_FLIP_ITERM_COMP_SLOW_OTHER;
		} else {
			strategy_class_fg_ops_get_temperature(&batt_temp);
			batt_temp /= 10;
			if (batt_temp >= ITERM_COMP_HOT_LOW && batt_temp <= ITERM_COMP_HOT_HIGH &&
			    info->parallel_iterm == BASE_FLIP_ITERM_HOT)
				info->pmic_iterm_compensation = BASE_FLIP_ITERM_COMP_HOT;
			else if (batt_temp >= ITERM_COMP_WARM_LOW &&
				 batt_temp <= ITERM_COMP_WARM_HIGH &&
				 info->parallel_iterm == BASE_FLIP_ITERM_WARM)
				info->pmic_iterm_compensation = BASE_FLIP_ITERM_COMP_WARM;
			else
				info->pmic_iterm_compensation = BASE_FLIP_ITERM_COMP_OTHER;
		}
	}

	if (info->base_flip_same) {
		if (!strategy_class_fg_get_fastcharge()) {
			info->pmic_iterm_compensation = BASE_FLIP_SAME_ITERM_COMP_SLOW;
		} else {
			strategy_class_fg_ops_get_temperature(&batt_temp);
			batt_temp /= 10;
			info->pmic_iterm_compensation = (batt_temp < BASE_FLIP_SAME_ITERM_TEMP_TH) ?
				BASE_FLIP_SAME_ITERM_COMP_COOL :
				BASE_FLIP_SAME_ITERM_COMP_WARM;
		}
	}

	effective_result = ((info->pmic_iterm_compensation + effective_result) *
		info->vbat_fg_to_pmic_ratio) + info->vote_buck_iterm_buf;

	if (info->support_multi_buck)
		return strategy_buckchg_process_multi_iterm(info, effective_result);

	return platform_class_buckchg_ops_set_term_curr(MAIN_BUCK_CHARGER, effective_result);
}

static int strategy_buckchg_set_5v_input(struct mca_votable *votable,
	void *data, int effective_result, const char *effective_client)
{
	struct strategy_buckchg_dev *info = data;

	if (!data)
		return -1;

	info->buck_5v_in = effective_result;

	if (info->proc_data.voltage != STATEGY_CHARGE_VBUS_5V)
		return 0;

	if (info->buck_5v_in)
		mca_vote(info->input_limit_voter, "volt_limit", true, effective_result);
	else
		mca_vote(info->input_limit_voter, "volt_limit", false, effective_result);

	return 0;
}

static int strategy_buckchg_set_5v_ichg(struct mca_votable *votable,
	void *data, int effective_result, const char *effective_client)
{
	struct strategy_buckchg_dev *info = data;

	if (!data)
		return -1;

	info->buck_5v_ich = effective_result;

	if (info->proc_data.voltage != STATEGY_CHARGE_VBUS_5V)
		return 0;

	if (info->buck_5v_ich)
		mca_vote(info->charge_limit_voter, "volt_thermal_limit", true, effective_result);
	else
		mca_vote(info->charge_limit_voter, "volt_thermal_limit", false, effective_result);

	return 0;
}

static int strategy_buckchg_set_9v_input(struct mca_votable *votable,
	void *data, int effective_result, const char *effective_client)
{
	struct strategy_buckchg_dev *info = data;

	if (!data)
		return -1;

	info->buck_9v_in = effective_result;

	if (info->proc_data.voltage != STATEGY_CHARGE_VBUS_9V)
		return 0;

	if (info->buck_9v_in)
		mca_vote(info->input_limit_voter, "volt_limit", true, effective_result);
	else
		mca_vote(info->input_limit_voter, "volt_limit", false, effective_result);

	return 0;
}

static int strategy_buckchg_set_9v_ichg(struct mca_votable *votable,
	void *data, int effective_result, const char *effective_client)
{
	struct strategy_buckchg_dev *info = data;

	if (!data)
		return -1;

	info->buck_9v_ich = effective_result;

	if (info->proc_data.voltage != STATEGY_CHARGE_VBUS_9V)
		return 0;

	if (info->buck_9v_ich)
		mca_vote(info->charge_limit_voter, "volt_thermal_limit", true, effective_result);
	else
		mca_vote(info->charge_limit_voter, "volt_thermal_limit", false, effective_result);

	return 0;
}

static int strategy_buckchg_init_voter(struct strategy_buckchg_dev *info)
{
	/* buck charge */
	struct mca_hwid_info *hwid;
	info->chg_enable_voter = mca_create_votable("chg_enable", MCA_VOTE_AND,
			strategy_buckchg_enable, STATEGY_CHARGE_DISENABLE, info);
	if (IS_ERR(info->chg_enable_voter))
		return -1;
	info->input_suppend_voter = mca_create_votable("input_suspend", MCA_VOTE_OR,
		strategy_buckchg_input_suspend, STATEGY_CHARGE_INPUT_SUSPEND, info);
	if (IS_ERR(info->input_suppend_voter))
		return -1;
	info->input_limit_voter = mca_create_votable("buck_input", MCA_VOTE_MIN,
		strategy_buckchg_input_limit, STATEGY_INPUT_DEFAULT_VALUE, info);
	if (IS_ERR(info->input_limit_voter))
		return -1;
	info->charge_limit_voter = mca_create_votable("buck_charge_curr", MCA_VOTE_MIN,
		strategy_buckchg_charge_limit, STATEGY_CHARGE_CURRENT_DEFAULT_VALUE, info);
	if (IS_ERR(info->charge_limit_voter))
		return -1;

	/*
	 * Two platforms take the smallest termination current their clients ask
	 * for rather than the largest, so charging stops at the first client
	 * that is satisfied instead of the last.
	 */
	hwid = mca_get_hwid_info();
	info->iterm_voter = mca_create_votable("term_curr",
		(hwid && (hwid->platform_version & ~1u) == STRATEGY_ITERM_MIN_PLATFORM) ?
			MCA_VOTE_MIN : MCA_VOTE_MAX,
		strategy_buckchg_set_iterm, STATEGY_ITERM_DEFAULT_VALUE, info);
	if (IS_ERR(info->iterm_voter))
		return -1;

	info->vterm_voter = mca_create_votable("term_volt", MCA_VOTE_MIN,
		strategy_buckchg_set_vterm, STATEGY_VTERM_DEFAULT_VALUE, info);
	if (IS_ERR(info->vterm_voter))
		return -1;

	/* wire voter */
	info->input_voltage_voter = mca_create_votable("input_voltage", MCA_VOTE_AND,
		strategy_buckchg_input_voltage, 1, info);
	if (IS_ERR(info->input_voltage_voter))
		return -1;
	info->buck_5v_in_voter = mca_create_votable("buck_5v_in", MCA_VOTE_MIN,
		strategy_buckchg_set_5v_input, STATEGY_INPUT_5V_DEFAULT_VALUE, info);
	if (IS_ERR(info->buck_5v_in_voter))
		return -1;
	info->buck_5v_ich_voter = mca_create_votable("buck_5v_ich", MCA_VOTE_MIN,
		strategy_buckchg_set_5v_ichg, STATEGY_CHARGE_CURRENT_DEFAULT_VALUE, info);
	if (IS_ERR(info->buck_5v_ich_voter))
		return -1;
	info->buck_9v_in_voter = mca_create_votable("buck_9v_in", MCA_VOTE_MIN,
		strategy_buckchg_set_9v_input, STATEGY_INPUT_9V_DEFAULT_VALUE, info);
	if (IS_ERR(info->buck_9v_in_voter))
		return -1;
	info->buck_9v_ich_voter = mca_create_votable("buck_9v_ich", MCA_VOTE_MIN,
		strategy_buckchg_set_9v_ichg, STATEGY_CHARGE_CURRENT_DEFAULT_VALUE, info);
	if (IS_ERR(info->buck_9v_ich_voter))
		return -1;

	return 0;
}

static void strategy_buckchg_limit_full_replug_ichg(struct strategy_buckchg_dev *info, bool plugin)
{
	int rawsoc = 0;

	if (!info->full_replug_ichg_limit) {
		return;
	}

	strategy_class_fg_ops_get_rsoc(&rawsoc);
	if (plugin && rawsoc >= FULL_REPLUG_LIMIT_RAWSOC_TH) {
		mca_vote(info->charge_limit_voter, "full_replug", true, info->full_replug_ichg_limit);
	} else if (rawsoc < FULL_REPLUG_LIMIT_RAWSOC_TH || info->proc_data.chg_status == MCA_BUCK_CHG_STS_CHARGE_DONE) {
		mca_vote(info->charge_limit_voter, "full_replug", false, 0);
	}
}

static void strategy_buckchg_resume_buck_ichg_limit(struct strategy_buckchg_dev *info)
{
	int rawsoc = 0;

	strategy_class_fg_ops_get_rsoc(&rawsoc);
	if (rawsoc < BUCKCHG_OK_TO_HIGH_IBAT_RAWSOC_TH) {
		mca_vote(info->charge_limit_voter, "qc_done", false, 0);
	}
}

static void strategy_buckchg_start_charging(struct strategy_buckchg_dev *info)
{
	mca_log_info("start charging\n");
	strategy_buckchg_limit_full_replug_ichg(info, true);
	strategy_buck_update_req_volt(info);
	cancel_delayed_work_sync(&info->monitor_work);
	strategy_class_buckchg_ops_adc_enable(info, true);
	schedule_delayed_work(&info->monitor_work,  0);
}

static void strategy_buckchg_reset_charge_para(struct strategy_buckchg_dev *info)
{
	memset(&info->proc_data, 0, sizeof(info->proc_data));
	memset(&info->pwr_cap, 0, sizeof(info->pwr_cap));
	info->proc_data.voltage = STATEGY_CHARGE_VBUS_5V;
	info->is_non_compliant_qc = false;
	info->non_compliant_run_once = false;
	info->vbat_ov_count = 0;
	mca_vote(info->input_voltage_voter, "non_compliant_qc", false, 0);
	mca_vote(info->input_limit_voter, "non_compliant_qc", false, 0);

	mca_vote(info->chg_enable_voter, "online", true, STATEGY_CHARGE_DISENABLE);
	mca_vote(info->buck_5v_in_voter, "wire_chg_type",
		true, STATEGY_INPUT_DEFAULT_VALUE);

	mca_vote(info->buck_9v_in_voter, "wire_chg_type",
		true, STATEGY_INPUT_DEFAULT_VALUE);
	mca_vote(info->input_limit_voter, "icl_limit", true, STATEGY_INPUT_DEFAULT_VALUE);

	mca_vote(info->buck_9v_in_voter, "pmic_inductor", false, 0);
	mca_vote(info->buck_5v_in_voter, "pmic_inductor", false, 0);

	mca_vote(info->buck_5v_ich_voter, "wire_chg_type",
		true, STATEGY_INPUT_DEFAULT_VALUE);

	mca_vote(info->buck_9v_ich_voter, "wire_chg_type",
		true, STATEGY_CHARGE_CURRENT_DEFAULT_VALUE);
	mca_vote(info->input_voltage_voter, "real_type", true, 0);
	mca_vote(info->input_voltage_voter, "eoc_5v", false, 0);
	mca_vote(info->chg_enable_voter, "vbat_ovp", false, STATEGY_CHARGE_ENABLE);
	mca_vote(info->input_voltage_voter, "lpd", false, 0);
	cancel_delayed_work_sync(&info->csd_pulse_process_work);
	mca_vote(info->charge_limit_voter, "csd_pulse", false, 0);
	mca_vote(info->charge_limit_voter, "qc_done", false, 0);
	mca_vote(info->chg_enable_voter, "csd_pulse", false, STATEGY_CHARGE_DISENABLE);
	mca_vote(info->charge_limit_voter, "full_replug", false, 0);
	info->csd_flag = false;

	/*xring system abnormal use default ibus and ibat 500mA */
	if (info->use_sc_buck) {
		mca_vote(info->chg_enable_voter, "online", true, STATEGY_CHARGE_ENABLE);
		mca_vote(info->buck_5v_in_voter, "wire_chg_type",
				false, STATEGY_INPUT_DEFAULT_VALUE);
		mca_vote(info->buck_5v_ich_voter, "wire_chg_type",
				false, STATEGY_INPUT_DEFAULT_VALUE);
		mca_vote(info->buck_9v_in_voter, "wire_chg_type",
				false, STATEGY_INPUT_DEFAULT_VALUE);
		mca_vote(info->input_limit_voter, "icl_limit",
				false, STATEGY_INPUT_DEFAULT_VALUE);
		mca_vote(info->input_limit_voter, "subpmic_hw",
				true, MCA_WIRE_CHARGE_DEFAULT_IBUS_CURRENT);
		mca_vote(info->charge_limit_voter, "subpmic_hw",
				true, MCA_WIRE_CHARGE_DEFAULT_IBUS_CURRENT);
	}
}

/*
 * Where the float voltage follows the pack temperature, an override is placed
 * from outside to hold the target and the input down while the pack is hot.
 * Nothing here puts it on; this is the only thing that lifts it, and it has
 * to be lifted whenever the charge is no longer running under it, or the next
 * charge starts held back by a condition that has passed.
 */
static void strategy_buckchg_clear_pmic_temp_term(struct strategy_buckchg_dev *info)
{
	if (!info->support_pmic_vterm_dynamics_adjust)
		return;

	mca_vote_override(info->vterm_voter, "pmic_temp_term", false, 0);
	mca_vote_override(info->input_limit_voter, "pmic_temp_term", false, 0);
}

static void strategy_buckchg_stop_charging(struct strategy_buckchg_dev *info)
{
	mca_log_info("stop charging\n");
	cancel_delayed_work_sync(&info->monitor_work);
	cancel_delayed_work_sync(&info->wls_revchg_monitor_work);
	cancel_delayed_work_sync(&info->check_pd_secret_work);
	cancel_delayed_work_sync(&info->rerun_handle_pd_auth_work);
	strategy_buckchg_sw_cv_stop(info);
	strategy_buckchg_reset_charge_para(info);
	strategy_class_buckchg_ops_set_input_volt(info, STATEGY_CHARGE_INPUT_VOLT_DEFAULT);
	strategy_class_buckchg_ops_adc_enable(info, false);
	strategy_class_buckchg_ops_set_opt_fws(info, STATEGY_CHARGE_FWS_DEFAULT);
	strategy_class_buckchg_ops_set_usb_aicl_cont_thd(info, STATEGY_CHARGE_AICL_TH_4P4V);

	strategy_buckchg_clear_pmic_temp_term(info);
	info->pmic_temp_term_flag = false;

	info->aicl_thd = 0;
	info->pdo_nums = 0;
}

static void strategy_buckchg_process_online_change(int value, struct strategy_buckchg_dev *info)
{
	if (value == info->proc_data.online)
		return;

	mca_event_block_notify(MCA_EVENT_TYPE_BATTERY_INFO, MCA_EVENT_BATTERY_STS_CHANGE, NULL);
	info->proc_data.online = value;
	if (value) {
		/*
		 * A flip base's load switch is left in low-power mode while
		 * nothing is attached; it has to come out of it before the
		 * base cell can be charged, unless the gauges say the base is
		 * not answering at all.
		 */
		if (info->support_base_flip) {
			bool err_base = false, err_flip = false;

			(void)platform_fg_ops_get_error_state(FG_IC_MASTER,
							      &err_base);
			(void)platform_fg_ops_get_error_state(FG_IC_SLAVE,
							      &err_flip);
			if (!err_base || err_flip)
				(void)platform_class_loadsw_set_lowpower_mode(0,
									     false);
		}
		strategy_buckchg_start_charging(info);
	} else {
		/* Nothing to feed the base cell from, so the switch idles. */
		(void)platform_class_loadsw_set_lowpower_mode(0, true);
		info->pps_ptf = USBPD_BUCK_DPM_PORT_PPS_PTF_NOT_SUPPORTED;
		strategy_buckchg_stop_charging(info);
		info->proc_data.charge_done_force_5v = false;
		info->verify_process_end = 0;
		info->buck_abnormal_cnt = 0;
		/*
		 * A pack of two cells discharges through the pump, so its bus
		 * over-voltage has to be raised to the pack's own range once
		 * the charger is gone.
		 */
		if (info->base_flip_same) {
			bool cp_enabled = false;

			(void)platform_class_cp_set_busovp(CP_ROLE_MASTER,
				BUCKCHG_DISCHARGE_CP_BUSOVP_MV);
			(void)platform_class_cp_get_charging_enabled(
				CP_ROLE_MASTER, &cp_enabled);
			mca_log_info("cp_enabled = %d in discharging\n",
				     cp_enabled);
		}
	}
}

#define PDO_9V_VOLATGE 9000
/*
 * An adapter that comes back with more operating points than it first offered
 * has just finished authenticating.  Telling the stack about it is left a
 * second, so the PD side has settled before anything acts on it.
 */
static void strategy_rerun_handle_pd_auth_workfunc(struct work_struct *work)
{
	mca_event_block_notify(MCA_EVENT_CHARGE_STATUS, MCA_EVENT_USB_STS_CHANGE,
			       NULL);
	mca_log_info("send usb power_supply trigger handle pd auth\n");
}

static void stategy_buckchg_is_pdo_9v(struct strategy_buckchg_dev *info)
{
	if (info->proc_data.real_type != XM_CHARGER_TYPE_PD &&
		info->proc_data.real_type != XM_CHARGER_TYPE_PPS &&
		info->proc_data.real_type != XM_CHARGER_TYPE_PD_VERIFY)
		return;

	if (info->pdo_nums == 1 && info->pwr_cap.pdo_nums > 1)
		queue_delayed_work(system_wq, &info->rerun_handle_pd_auth_work,
				   msecs_to_jiffies(1000));
	info->pdo_nums = info->pwr_cap.pdo_nums;

	if (info->pwr_cap.pdo_nums == 0) {
		mca_log_info(" pwr_cap.pdo_nums is null\n");
		return;
	}

	for (int i = 0; i < info->pwr_cap.pdo_nums; i++) {
		if (info->pwr_cap.cap[i].max_voltage == info->pwr_cap.cap[i].min_voltage &&
			info->pwr_cap.cap[i].max_voltage == PDO_9V_VOLATGE) {
			info->proc_data.is_pd_9v = true;
			info->proc_data.curr_pd_pos = i;
			mca_log_info("pdo[%d] can support 9v: %d\n", i, info->proc_data.is_pd_9v);
			return;
		}
	}

	info->proc_data.is_pd_9v = false;
	info->proc_data.curr_pd_pos = 0;
	return;

}

static noinline void strategy_buckchg_process_type_change(int value, struct strategy_buckchg_dev *info)
{
	struct timespec64 ts;

	if (value == info->proc_data.real_type) {
		if (value == XM_CHARGER_TYPE_UNKNOW)
			strategy_buckchg_check_charger_change(info);
		return;
	}

	info->proc_data.real_type = value;
	strategy_buckchg_check_charger_change(info);

	if (value == XM_CHARGER_TYPE_UNKNOW)
		return;

	if (info->proc_data.real_type == XM_CHARGER_TYPE_PD ||
		info->proc_data.real_type == XM_CHARGER_TYPE_PPS ||
		info->proc_data.real_type == XM_CHARGER_TYPE_PD_VERIFY) {
		protocol_class_get_adapter_power_cap(ADAPTER_PROTOCOL_PD, &info->pwr_cap);
		protocol_class_pd_get_suspend_support_status(TYPEC_PORT_0, &info->proc_data.pdsuspendsupported);
	}

	if (info->is_eu_model && info->proc_data.real_type == XM_CHARGER_TYPE_PPS) {
			info->proc_data.eu_start_time = ktime_get_boottime();
			mca_log_info("is_eu_model for PPS eu_start_time = %lld\n", info->proc_data.eu_start_time);
	}
	stategy_buckchg_is_pdo_9v(info);
	strategy_buck_update_req_volt(info);
	mod_delayed_work(system_wq, &info->monitor_work, 0);

	ktime_get_boottime_ts64(&ts);
	mca_log_info("verify_process_end: %d\n", info->verify_process_end);
	if (info->proc_data.real_type == XM_CHARGER_TYPE_PD) {
		if (!info->verify_process_end && (u64)ts.tv_sec > 60) {
			mca_vote(info->input_limit_voter, "icl_limit", true, STATEGY_INPUT_DEFAULT_VALUE);
			schedule_delayed_work(&info->check_pd_secret_work, msecs_to_jiffies(10000));
		}
	} else {
		mca_vote(info->input_limit_voter, "icl_limit", false, STATEGY_INPUT_DEFAULT_VALUE);
	}
}

static noinline void strategy_buckchg_process_antiburn_change(int value, struct strategy_buckchg_dev *info)
{
	mca_log_info("value: %d\n", value);
	strategy_buckchg_set_charge_volt(info, STATEGY_CHARGE_VBUS_5V);
	if (value)
		strategy_buckchg_stop_charging(info);
	else if (info->proc_data.online)
		strategy_buckchg_start_charging(info);
}

static noinline void strategy_buckchg_process_batt_btb_change(int value, struct strategy_buckchg_dev *info)
{
	if (value) {
		mca_vote(info->input_voltage_voter, "batt_miss", true, 0);
		/*
		 * A pack of two cells behind one connector can still be fed
		 * twice as hard with one of them gone.
		 */
		mca_vote(info->input_limit_voter, "batt_miss", true,
			 info->batt_type == MCA_BATTERY_TYPE_SINGLE ?
				STAEGY_BATT_MISS_ICL :
				STAEGY_BATT_MISS_ICL_PARALLEL);
#ifndef CONFIG_FACTORY_BUILD
		mca_vote(info->vterm_voter, "batt_miss", true, STAEGY_BATT_MISS_FV);
#endif
	} else {
		mca_vote(info->input_voltage_voter, "batt_miss", false, 0);
		mca_vote(info->input_limit_voter, "batt_miss", false, STATEGY_CHARGE_CURRENT_DEFAULT_VALUE);
		mca_vote(info->vterm_voter, "batt_miss", false, STATEGY_VTERM_DEFAULT_VALUE);
		/*
		 * The pack is back, so the hard limit the missing-cell path
		 * forced on the input is released too.
		 */
		mca_vote_override(info->input_limit_voter, "master_batt_missing",
				  false, 0);
	}
}

/*
 * Coming off a charge because the level cap was reached is done in steps a
 * second apart rather than in one cut: dropping the input from full current to
 * nothing in a single write drags the supply voltage down with it, and the
 * adapter sees that as a fault.  Each step lowers both limits, and only the
 * last one actually stops the charge.
 */
static const struct {
	int fcc_value;
	int icl_value;
} soc_limit_stepper_table[] = {
	{ 0, 0 },
	{ 1990, 850 },
	{ 1400, 750 },
	{ 1100, 650 },
	{ 900, 550 },
	{ 700, 450 },
	{ 500, 350 },
	{ 300, 350 },
	{ 200, 350 },
	{ 100, 250 },
	{ 0, 250 },
};

#define SOC_LIMIT_MAX_STEP	(ARRAY_SIZE(soc_limit_stepper_table) - 1)
#define SOC_LIMIT_STEP_MS	1000

/* Reverse quick charge. */
#define STRATEGY_BUCKCHG_REVCHG_BOARD_TEMP_MAX	401
#define STRATEGY_BUCKCHG_REVCHG_SOC_START	69
#define STRATEGY_BUCKCHG_REVCHG_SOC_KEEP	29
#define STRATEGY_BUCKCHG_REVCHG_TIMEOUT		40
#define STRATEGY_BUCKCHG_REVCHG_SETTLE_MS	400
#define STRATEGY_BUCKCHG_REVCHG_POS_BOOST	1
#define STRATEGY_BUCKCHG_REVCHG_POS_DIV2	2

static noinline void strategy_buckchg_process_soc_limit_change_more(int value,
								    struct strategy_buckchg_dev *info)
{
	static u8 cur_step;

	if (value)
		mca_log_info("SOC limit triggered\n");
	else
		mca_log_info("SOC limit released\n");

	if (value) {
		if (cur_step < SOC_LIMIT_MAX_STEP)
			cur_step += 1;
	} else {
		cur_step = 0;
	}

	if (cur_step == 0) {
		mca_vote(info->charge_limit_voter, "soc_limit", false, 0);
		mca_vote(info->input_limit_voter, "soc_limit", false, 0);
		mca_vote(info->chg_enable_voter, "soc_limit", false,
			 STATEGY_CHARGE_ENABLE);
	} else {
		mca_vote(info->charge_limit_voter, "soc_limit", true,
			 soc_limit_stepper_table[cur_step].fcc_value);
		mca_vote(info->input_limit_voter, "soc_limit", true,
			 soc_limit_stepper_table[cur_step].icl_value);

		if (cur_step != SOC_LIMIT_MAX_STEP) {
			mca_vote(info->chg_enable_voter, "soc_limit", false,
				 STATEGY_CHARGE_ENABLE);
			schedule_delayed_work(&info->soc_limit_stepper_work,
					      msecs_to_jiffies(SOC_LIMIT_STEP_MS));
		} else {
			mca_vote(info->chg_enable_voter, "soc_limit", true,
				 STATEGY_CHARGE_DISENABLE);
		}
	}

	mca_log_info("cur_step = %d\n", cur_step);
}

/* Take the next step down, or the next step back up. */
static void strategy_buckchg_soc_limit_stepper_workfunc(struct work_struct *work)
{
	struct strategy_buckchg_dev *info =
		container_of(to_delayed_work(work), struct strategy_buckchg_dev,
			     soc_limit_stepper_work);

	strategy_buckchg_process_soc_limit_change_more(info->soc_limit_sts, info);
}

static noinline void strategy_buckchg_process_cap_change(struct strategy_buckchg_dev *info)
{
	if (info->proc_data.real_type == XM_CHARGER_TYPE_PD ||
		info->proc_data.real_type == XM_CHARGER_TYPE_PPS ||
		info->proc_data.real_type == XM_CHARGER_TYPE_PD_VERIFY) {
		protocol_class_get_adapter_power_cap(ADAPTER_PROTOCOL_PD, &info->pwr_cap);
		protocol_class_pd_get_suspend_support_status(TYPEC_PORT_0, &info->proc_data.pdsuspendsupported);
	}

	stategy_buckchg_is_pdo_9v(info);
	strategy_buck_update_req_volt(info);
	mod_delayed_work(system_wq, &info->monitor_work, 0);
}

static void strategy_buckchg_process_cancel_monitor_work(struct strategy_buckchg_dev *info)
{
	int ret;
	ret = cancel_delayed_work_sync(&info->monitor_work);
	return;
}

static noinline void strategy_buckchg_process_csd_pulse(int value, struct strategy_buckchg_dev *info)
{
	int ffc_sts;

	ffc_sts = strategy_class_fg_get_fastcharge();

	mca_log_info("value: %d, ffc_sts: %d\n", value, ffc_sts);
	if (ffc_sts == true && value > 0)
		strategy_class_fg_set_fastcharge(false);

	if (value == 1 || value == 2) {
		mca_vote(info->charge_limit_voter, "csd_pulse", true, 1500);
		info->csd_flag = true;
		schedule_delayed_work(&info->csd_pulse_process_work, msecs_to_jiffies(75000));
	} else if (value == 3) {
		mca_vote(info->charge_limit_voter, "csd_pulse", true, 1500);
	} else if (value == 0) {
		cancel_delayed_work_sync(&info->csd_pulse_process_work);
		mca_vote(info->charge_limit_voter, "csd_pulse", false, 0);
		mca_vote(info->chg_enable_voter, "csd_pulse", false, STATEGY_CHARGE_DISENABLE);
		info->csd_flag = false;
	}
}

/*
 * Xiaomi's own release also handles the charge pump being reversed here.
 * That event does not exist on this platform, so the handler it drove is
 * left out with it.
 */

/*
 * The PMIC comes up after the strategies do, and every vote cast before it was
 * ready went to a charger that could not act on it.  Re-run the elections so
 * each votable applies its standing result to the hardware that now exists.
 */
static noinline void strategy_buckchg_process_pmic_init_done(struct strategy_buckchg_dev *info)
{
	mca_log_info("deal with pmic_init_done\n");

	mca_rerun_election(info->chg_enable_voter);
	mca_rerun_election(info->input_limit_voter);
	mca_rerun_election(info->charge_limit_voter);
	mca_rerun_election(info->iterm_voter);
	mca_rerun_election(info->vterm_voter);
}

/* The charging current the smart charging layer asks for. */
static int strategy_buckchg_smartchg_set_fcc_callback(void *data, int fcc)
{
	struct strategy_buckchg_dev *info = data;

	if (!info)
		return -1;

	mca_vote(info->charge_limit_voter, "smartchg", !!fcc, fcc);

	return 0;
}

/*
 * Whether the phone may fast-charge another device over the port.  It must
 * not be taking a wireless charge at the same time, the cell must be above
 * freezing and hold enough charge to have some to give, and the board must
 * not already be hot.  The level needed to start is well above the level the
 * charge carries on from, so one that has begun is not dropped on a dip.
 */
static noinline void strategy_buckchg_check_reverse_quick_charge(struct strategy_buckchg_dev *info,
								 int *allowed)
{
	int batt_temp = 0;
	int vbus = 0;
	bool otg_present = false;
	int soc_thres;
	int soc;

	strategy_class_fg_ops_get_temperature(&batt_temp);
	soc = strategy_class_fg_ops_get_soc();
	batt_temp /= 10;

	protocol_class_pd_get_otg_plugin_status(TYPEC_PORT_0, &otg_present);

	mca_log_info("otg_present = %d, batt_temp = %d, thermal_board_temp =%d\n",
		     otg_present, batt_temp, info->thermal_board_temp);

	if (!otg_present || info->wls_online || info->wls_revchg_online ||
	    batt_temp < 0 ||
	    info->thermal_board_temp >= STRATEGY_BUCKCHG_REVCHG_BOARD_TEMP_MAX) {
		*allowed = 0;
		return;
	}

	soc_thres = info->reverse_auth_sts ? STRATEGY_BUCKCHG_REVCHG_SOC_KEEP :
					     STRATEGY_BUCKCHG_REVCHG_SOC_START;
	*allowed = soc > soc_thres;

	if (info->cp_vendor != SC8541_VENDOR || soc <= soc_thres)
		return;

	platform_class_buckchg_ops_get_bus_volt(MAIN_BUCK_CHARGER, &vbus);
	vbus /= 1000;
	platform_class_cp_get_pmid2out_uvp_dis(CP_ROLE_MASTER, false);

	mca_log_err("quick revchg, check vbus: %d uvp_dis_enable %d\n", vbus, 0);
}

/*
 * The adapter tells us where it is in reverse authentication, packed one byte
 * per field.  Position 1 brings the external boost up and takes the pump out
 * of the way; position 2 turns the pump around to halve the cell voltage into
 * the port.  Losing the plug at any point ends it and puts everything back.
 *
 * Which way the pump is turned around depends on the part: one takes a
 * dedicated reverse mode, the other is driven manually.
 */
static noinline void strategy_buckchg_cp_revert_handler(int auth_pos,
							struct strategy_buckchg_dev *info)
{
	static int last_pos;
	static bool bq_reverse_mode;
	char event[MCA_EVENT_NOTIFY_SIZE] = { 0 };
	struct mca_event_notify_data event_data = { 0 };
	int otg_boost_disable = (auth_pos >> 24) & 0xff;
	int auth_done = (auth_pos >> 16) & 0xff;
	int pos = auth_pos & 0xff;
	bool otg_present = false;
	int allowed = 0;
	int len;

	info->reverse_auth_sts = (auth_pos & 0xff00) != 0;

	if (!otg_boost_disable)
		protocol_class_pd_get_otg_plugin_status(TYPEC_PORT_0, &otg_present);

	mca_log_err("otg_present: %d, otg_boost_disable: %d, reverse_auth_sts: %d, pos: %d, last_pos: %d, auth_done: %d\n",
		    otg_present, otg_boost_disable, info->reverse_auth_sts, pos,
		    last_pos, auth_done);

	if (auth_done && otg_present) {
		strategy_buckchg_check_reverse_quick_charge(info, &allowed);

		len = snprintf(event, MCA_EVENT_NOTIFY_SIZE,
			       "POWER_SUPPLY_REVERSE_QUICK_CHARGE=%d", allowed);
		event_data.event = event;
		event_data.event_len = len;
		mca_event_report_uevent(&event_data);

		mca_log_err("status: %d\n", allowed);
		return;
	}

	if (otg_present && last_pos == pos) {
		mca_log_err("same handler, ignore...\n");
		goto dump;
	}

	last_pos = pos;

	if (!otg_present)
		cancel_delayed_work_sync(&info->wls_revchg_monitor_work);

	platform_class_buckchg_ops_get_otg_boost_src(MAIN_BUCK_CHARGER,
						     &info->otg_boost_src);

	if (!otg_present) {
		mca_log_err("end revert 1_2 cp\n");

		if (info->otg_boost_src) {
			platform_class_cp_set_charging_enable(CP_ROLE_MASTER, false);
			platform_class_cp_enable_ovpgate(CP_ROLE_MASTER, true);
			platform_class_cp_enable_wpcgate(CP_ROLE_MASTER, false);
			platform_class_cp_set_qb(CP_ROLE_MASTER, false);
			platform_class_cp_set_mode(CP_ROLE_MASTER, CP_MODE_FORWARD_2_1);
			goto ended;
		}

		platform_class_cp_set_adjustadble_timeout(CP_ROLE_MASTER,
							  STRATEGY_BUCKCHG_REVCHG_TIMEOUT);

		if (info->cp_vendor == SC8541_VENDOR) {
			mca_log_err("exit sc8541 1_2 cp manual mode\n");
			platform_class_cp_set_manual_revchg_mode(CP_ROLE_MASTER, false);
		}

		if (info->cp_vendor == BQ25960_VENDOR) {
			mca_log_err("exit bq25960 1_2 cp reverse mode\n");
			platform_class_cp_set_cp_reverse_mode(CP_ROLE_MASTER, false);
			bq_reverse_mode = false;
		}

		goto ended;
	}

	platform_class_cp_get_chip_vendor(CP_ROLE_MASTER, &info->cp_vendor);

	switch (pos) {
	case STRATEGY_BUCKCHG_REVCHG_POS_BOOST:
		if (!info->otg_boost_src)
			goto dump;

		mca_log_err("start external boost\n");
		platform_class_buckchg_ops_set_boost_enable(MAIN_BUCK_CHARGER,
			(OTG_EN_BOOST << 16) | (EXTERNAL_BOOST << 8) | 1);
		msleep(STRATEGY_BUCKCHG_REVCHG_SETTLE_MS);
		platform_class_cp_set_charging_enable(CP_ROLE_MASTER, false);
		platform_class_cp_enable_ovpgate(CP_ROLE_MASTER, false);
		platform_class_cp_enable_wpcgate(CP_ROLE_MASTER, false);
		platform_class_cp_set_qb(CP_ROLE_MASTER, false);
		platform_class_cp_set_mode(CP_ROLE_MASTER, CP_MODE_FORWARD_2_1);
		goto dump;

	case STRATEGY_BUCKCHG_REVCHG_POS_DIV2:
		mca_log_err("start revert 1_2 cp\n");

		if (info->cp_vendor == BQ25960_VENDOR && !bq_reverse_mode) {
			mca_log_err("entry bq25960 1_2 cp reverse mode\n");
			platform_class_cp_set_cp_reverse_mode(CP_ROLE_MASTER, true);
			bq_reverse_mode = true;
			break;
		}

		platform_class_cp_enable_acdrv_manual(CP_ROLE_MASTER, true);
		platform_class_cp_set_adjustadble_timeout(CP_ROLE_MASTER, 0);
		platform_class_cp_set_mode(CP_ROLE_MASTER, CP_MODE_REVERSE_1_2);
		platform_class_cp_enable_wpcgate(CP_ROLE_MASTER, false);
		platform_class_cp_enable_ovpgate(CP_ROLE_MASTER, true);
		platform_class_cp_set_qb(CP_ROLE_MASTER, true);
		platform_class_cp_set_charging_enable(CP_ROLE_MASTER, true);

		if (info->cp_vendor == SC8541_VENDOR) {
			mca_log_err("entry sc8541 1_2 cp manual mode\n");
			platform_class_cp_set_manual_revchg_mode(CP_ROLE_MASTER, true);
		}
		break;

	default:
		goto dump;
	}

	if (info->otg_boost_src) {
		msleep(STRATEGY_BUCKCHG_REVCHG_SETTLE_MS);
		platform_class_buckchg_ops_set_boost_enable(MAIN_BUCK_CHARGER,
			(OTG_EN_BOOST << 16) | (EXTERNAL_BOOST << 8) | 0);
	}
	goto dump;

ended:
	last_pos = STRATEGY_BUCKCHG_REVCHG_POS_BOOST;
	info->source_boost_status = 0;
	info->start_quick_revchg = false;
	info->revchg_bcl = false;

	len = snprintf(event, MCA_EVENT_NOTIFY_SIZE,
		       "POWER_SUPPLY_REVERSE_QUICK_CHARGE=0");
	event_data.event = event;
	event_data.event_len = len;
	mca_event_report_uevent(&event_data);

dump:
	platform_class_cp_dump_register(CP_ROLE_MASTER);
}

/*
 * The soc_limit knob is a test aid: hold the charge inside a band rather than
 * charging to full.  Only the edges of the band are acted on, so a battery
 * sitting inside it is left to whatever the rest of the stack decided.  With
 * the knob off, a vote left over from when it was on still has to be released.
 */
static void strategy_buckchg_check_debug_soc_limit(struct strategy_buckchg_dev *info,
	int system_soc)
{
	bool limit;

	if (info->soc_limit_low >= 1 && info->soc_limit_high >= 1 &&
	    info->soc_limit_low < info->soc_limit_high) {
		if (system_soc >= info->soc_limit_high)
			limit = true;
		else if (system_soc <= info->soc_limit_low)
			limit = false;
		else
			return;
	} else if (mca_is_client_vote_enabled(info->chg_enable_voter, "debug_soc_limit")) {
		limit = false;
	} else {
		return;
	}

	mca_vote(info->chg_enable_voter, "debug_soc_limit", limit, 0);
	mca_vote(info->input_limit_voter, "debug_soc_limit", limit, 0);
	mca_strategy_func_process(STRATEGY_FUNC_TYPE_QUICK_CHARGE,
		MCA_EVENT_DEBUG_CTRL_SOC_LIMIT, limit);
}

/*
 * What the source is asked for depends on whether the pad is also on the
 * port, so a change on the wireless side has to be looked at again straight
 * away rather than at the monitor's own pace.
 */
static void strategy_buckchg_rekick_source_monitor(struct strategy_buckchg_dev *info)
{
	bool otg_plugin = false;

	(void)protocol_class_pd_get_otg_plugin_status(TYPEC_PORT_0, &otg_plugin);
	if (!info->support_reverse_quick_charge || !otg_plugin ||
	    !info->start_quick_revchg)
		return;

	cancel_delayed_work_sync(&info->source_status_monitor_work);
	schedule_delayed_work(&info->source_status_monitor_work, 0);
}

static int strategy_buckchg_process_event(int event, int value, void *data)
{
	struct strategy_buckchg_dev *info = data;

	if (!data)
		return -1;

	mca_log_info("receive event %d, value %d\n", event, value);
	switch (event) {
	case MCA_EVENT_USB_CONNECT:
	case MCA_EVENT_USB_DISCONNECT:
		strategy_buckchg_process_online_change(value, info);
		break;
	case MCA_EVENT_CHARGE_TYPE_CHANGE:
		strategy_buckchg_process_type_change(value, info);
		break;
	case MCA_EVENT_CHARGE_CAP_CHANGE:
		strategy_buckchg_process_cap_change(info);
		break;
		strategy_buckchg_process_cancel_monitor_work(info);
		break;
	case MCA_EVENT_CONN_ANTIBURN_CHANGE:
		strategy_buckchg_process_antiburn_change(value, info);
		break;
	case MCA_EVENT_BATT_BTB_CHANGE:
		strategy_buckchg_process_batt_btb_change(value, info);
		break;
	case MCA_EVENT_BATT_AUTH_PASS:
		mca_log_err("receive batt_auth event, value %d", value);
		info->batt_auth = 1;
		mca_vote(info->input_voltage_voter, "batt_auth", false, 0);
		mca_vote(info->charge_limit_voter, "batt_auth", false, info->chg_batt_auth_failed);
		if (info->proc_data.online)
			mod_delayed_work(system_wq, &info->monitor_work, 0);
		break;
	case MCA_EVENT_PMIC_INIT_DONE:
		strategy_buckchg_process_pmic_init_done(info);
		break;
	case MCA_EVENT_CHARGE_ABNORMAL:
		strategy_buckchg_stop_charging(info);
		break;
	case MCA_EVENT_CHARGE_RESTORE:
		(void)platform_class_buckchg_ops_get_online(MAIN_BUCK_CHARGER,
			&info->proc_data.online);
		if (info->proc_data.online)
			strategy_buckchg_start_charging(info);
		break;
	case MCA_EVENT_LPD_STATUS_CHANGE:
		if (value)
			mca_vote(info->input_voltage_voter, "lpd", true, 0);
		else
			mca_vote(info->input_voltage_voter, "lpd", false, 0);
		break;
	case MCA_EVENT_QUICK_REVCHG_CHANGE:
		strategy_buckchg_cp_revert_handler(value, info);
		break;
	case MCA_EVENT_WIRELESS_CONNECT:
	case MCA_EVENT_WIRELESS_DISCONNECT:
		info->wls_online = !!value;
		strategy_buckchg_cp_revert_handler(0, info);
		strategy_buckchg_rekick_source_monitor(info);
		break;
	case MCA_EVENT_WIRELESS_REVCHG:
		info->wls_revchg_online = !!value;
		strategy_buckchg_rekick_source_monitor(info);
		break;
	/*
	 * The gauge says a cell went over its limit.  Nothing is decided here
	 * -- the monitor is what re-reads the pack and lowers the target --
	 * only that it should not wait out the rest of its interval.  Software
	 * CV is already stepping the voltage down every second, so leave it be.
	 */
	case MCA_EVENT_BUCKCHG_BATT_OV:
		if (!info->sw_cv_running)
			mod_delayed_work(system_wq, &info->monitor_work, 0);
		break;
	case MCA_EVENT_WIRELESS_USB_REVCHG:
		mca_log_info("wireless revchg event %d\n", value);
		info->wls_revchg_en = value;
		if (value)
			strategy_wls_revchg_monitor_workfunc(&info->wls_revchg_monitor_work.work);
		else {
			cancel_delayed_work_sync(&info->wls_revchg_monitor_work);
			strategy_buckchg_exit_wireless_revchg(info);
		}
		break;
	case MCA_EVENT_CHARGE_VERIFY_PROCESS_END:
		mca_log_info("receive pd verify process end event, value %d\n", value);
		info->verify_process_end = value;
		if (info->verify_process_end)
			mca_vote(info->input_limit_voter, "icl_limit", false, STATEGY_INPUT_DEFAULT_VALUE);
		break;
	case MCA_EVENT_CC_SHORT_VBUS:
		if (value)
			mca_vote(info->input_voltage_voter, "cc_short_vbus", true, !value);
		else
			mca_vote(info->input_voltage_voter, "cc_short_vbus", false, !value);
		break;
	case MCA_EVENT_VBAT_OVP_CHANGE:
		if (value)
			mca_vote(info->chg_enable_voter, "vbat_ovp", true, STATEGY_CHARGE_DISENABLE);
		else
			mca_vote(info->chg_enable_voter, "vbat_ovp", false, STATEGY_CHARGE_ENABLE);
		break;
	case MCA_EVENT_CP_CBOOT_FAIL:
		if (value) {
			mca_vote(info->input_voltage_voter, "cp_cboot_short", true, !value);
			(void)platform_class_buckchg_ops_get_online(MAIN_BUCK_CHARGER,
														&info->proc_data.online);
			if (info->proc_data.online)
				strategy_buckchg_start_charging(info);
		} else {
			mca_vote(info->input_voltage_voter, "cp_cboot_short", false, !value);
		}
		break;
	case MCA_EVENT_PPS_PTF:
		info->pps_ptf = value;
		mca_log_info("set PPS_PTF %d\n", info->pps_ptf);
		if (info->pps_ptf == USBPD_BUCK_DPM_PORT_PPS_PTF_NOT_OVERTEMP)
			mca_vote(info->input_voltage_voter, "ptf", true, 1);
		break;
	case MCA_EVENT_IS_EU_MODEL:
		info->is_eu_model = value;
		mca_log_err("set buck is_eu_model %d\n", value);
		platform_class_buckchg_ops_set_eu_model(MAIN_BUCK_CHARGER, value);
		break;
	case MCA_EVENT_PLATE_SHOCK:
		if (value)
			mca_vote(info->input_limit_voter, "plate_shock", true, STAEGY_CHARGE_PLATE_SHOCK);
		else
			mca_vote(info->input_limit_voter, "plate_shock", false, 0);
		break;
	case MCA_EVENT_CSD_SEND_PULSE:
		strategy_buckchg_process_csd_pulse(value, info);
		break;
	case MCA_EVENT_START_QUICK_REVCHG:
		/*
		 * Reverse charging is already running by the time this
		 * arrives; the source monitor is what raises its current, so
		 * run it now instead of waiting out the rest of its interval.
		 */
		info->start_quick_revchg = true;
		if (info->support_reverse_quick_charge)
			schedule_delayed_work(&info->source_status_monitor_work, 0);
		break;
	case MCA_EVENT_REVCHG_BCL:
		info->revchg_bcl = !!value;
		break;
	case MCA_EVENT_BATTERY_TOTAL_ITERM:
		/*
		 * Which flip base is attached is told apart by this, so it is
		 * kept even on a board that has none.
		 */
		mca_log_info("parallel_iterm: %d\n", value);
		info->parallel_iterm = value;
		break;
	case MCA_EVENT_DEBUG_CTRL_MEMORY_TEST:
		/*
		 * The memory test wants the cell held near empty for the whole
		 * run, so it borrows the soc_limit band rather than having one
		 * of its own.
		 */
		mca_log_info("memory_test enable soc_limit: %d\n", value);
		info->soc_limit_low = value ? MEMORY_TEST_SOC_LIMIT_LOW : 0;
		info->soc_limit_high = value ? MEMORY_TEST_SOC_LIMIT_HIGH : 0;
		break;
	case MCA_EVENT_DEBUG_CTRL_SOC_LIMIT:
		info->soc_limit_low = value >> DEBUG_CTRL_SOC_SHIFT;
		info->soc_limit_high = value & DEBUG_CTRL_SOC_MASK;
		mca_log_info("debug_ctrl set soc_limit: %d %d\n",
			info->soc_limit_low, info->soc_limit_high);
		break;
	case MCA_EVENT_HANDLE_ALLOW_CHARGE:
		/*
		 * The accessory grip is feeding the phone, so the port has to
		 * stop drawing: hold the input at zero and the charger off
		 * until the grip hands it back.
		 */
		mca_log_info("handle_allow_charge: %d\n", value);
		mca_vote(info->input_limit_voter, "handle", !value, 0);
		mca_vote(info->chg_enable_voter, "handle", !value, 0);
		break;
	/*
	 * Xiaomi's own release also handles reverse charge-pump changes and
	 * the port starting to source here.  Neither event exists on this
	 * platform, where the port's own status is reported through the hw
	 * notifier instead.
	 */
	default:
		break;
	}

	return 0;
}

/*
 * The most the port may draw for the adapter it has, which is the same choice
 * select_charg_para() makes when it votes the limit -- asked for here so a
 * caller can have the number without waiting for the next monitor round.
 */
static int strategy_buckchg_get_current_max(struct strategy_buckchg_dev *info)
{
	int pos;

	switch (info->proc_data.real_type) {
	case XM_CHARGER_TYPE_PD:
	case XM_CHARGER_TYPE_PPS:
	case XM_CHARGER_TYPE_PD_VERIFY:
		if (info->pps_ptf == USBPD_BUCK_DPM_PORT_PPS_PTF_NOT_OVERTEMP)
			return CHARGE_PPS_PTF_INPUT_DEFAULT;
		pos = info->proc_data.curr_pd_pos;
		if (pos >= ADAPTER_POWER_CAP_MAX)
			break;
		if (!info->pwr_cap.cap[pos].max_current) {
			mca_log_info("pdo broadcast abnormal %d \n", pos);
			return info->in_pd;
		}
		return min(info->in_pd,
			(unsigned int)info->pwr_cap.cap[pos].max_current);
	case XM_CHARGER_TYPE_FLOAT:
	case XM_CHARGER_TYPE_OCP:
		return info->in_float;
	case XM_CHARGER_TYPE_HVDCP3:
	case XM_CHARGER_TYPE_HVDCP3_B:
		return info->in_hvdcp3;
	case XM_CHARGER_TYPE_HVDCP3P5:
		return info->in_hvdcp3p5;
	case XM_CHARGER_TYPE_HVDCP2:
		return info->in_hvdcp;
	case XM_CHARGER_TYPE_SDP:
		return info->is_eu_model ? info->in_sdp - 50 : info->in_sdp;
	case XM_CHARGER_TYPE_CDP:
		return info->in_cdp;
	case XM_CHARGER_TYPE_DCP:
		return info->smartchg_data.pwr_boost_state ?
			CHARGE_DCP_INPUT_BOOST : info->in_dcp;
	case XM_CHARGER_TYPE_TYPEC:
		return info->in_dcp;
	case XM_CHARGER_TYPE_SRC_UFP:
	case XM_CHARGER_TYPE_ACA:
		return info->in_sdp;
	default:
		break;
	}

	return CHARGE_SDP_INPUT_DEFAULT;
}

static int strategy_buckchg_get_status(int status, int *value, void *data)
{
	struct strategy_buckchg_dev *info = (struct strategy_buckchg_dev *)data;
	int *cur_val = (int *)value;

	if (!info || !value)
		return -1;

	switch (status) {
	case STRATEGY_STATUS_TYPE_ONLINE:
		platform_class_buckchg_ops_get_online(MAIN_BUCK_CHARGER, cur_val);
		break;
	case STRATEGY_STATUS_TYPE_QC_TYPE:
		*cur_val = info->proc_data.qc_type;
		break;
	case STRATEGY_STATUS_TYPE_ENABLE:
		*cur_val = mca_get_effective_result(info->chg_enable_voter);
		break;
	case STRATEGY_STATUS_TYPE_VBUS:
		*cur_val = info->proc_data.vbus;
		break;
	case STRATEGY_STATUS_TYPE_IBUS:
		*cur_val = info->proc_data.ibus;
		break;
	case STRATEGY_STATUS_TYPE_INIT_OK:
		*cur_val = info->init_ok;
		break;
	case STRATEGY_STATUS_TYPE_GET_VOLTAGE_MAX:
		*cur_val = info->proc_data.voltage;
		break;
	case STRATEGY_STATUS_TYPE_GET_CURRENT_MAX:
		*cur_val = strategy_buckchg_get_current_max(info);
		mca_log_info("current_max %d\n", *cur_val);
		break;
	default:
		return -1;
	}

	return 0;
}

#define USB_ICL_UNENUMERATED 100
static int strategy_buckchg_check_charger_change(struct strategy_buckchg_dev *info)
{
#ifdef CONFIG_FACTORY_BUILD
	return 0;
#endif
	static bool sdp_vote_completed;
	int charge_boot_mode = mca_log_get_charge_boot_mode();

	/* Release USB related vote if USB detach has been detected */
	if (info->proc_data.real_type == XM_CHARGER_TYPE_UNKNOW) {
		mca_vote(info->input_limit_voter, "usbicl", false, 0);
		mca_vote(info->input_limit_voter, "sdpicl", false, 0);
		sdp_vote_completed = false;
		mca_log_info("charger have been removed reset vote");
		return 0;
	}

	if (charge_boot_mode || !info->is_eu_model) {
		mca_log_info(" charge_boot_mode[%d] cancel sdp enumerated process", charge_boot_mode);
		return 0;
	}

	if (info->proc_data.real_type == XM_CHARGER_TYPE_SDP && !sdp_vote_completed) {
		mca_vote(info->input_limit_voter, "sdpicl", true, USB_ICL_UNENUMERATED);
		sdp_vote_completed = true;
		mca_log_info("sdp charger unenumerated");
	}

	if (info->proc_data.real_type != XM_CHARGER_TYPE_SDP && sdp_vote_completed) {
		mca_vote(info->input_limit_voter, "sdpicl", false, 0);
		mca_vote(info->input_limit_voter, "usbicl", false, 0);
		sdp_vote_completed = false;
		mca_log_info("sdp charger error detected reset vote%d\n", info->proc_data.real_type);
	}


	return 0;
}

#define USB_SUSPEND_ICL 0
#define USB_UNSUSPEND_ICL -1
#define USB_SUSPEND_ICL_DEFAULT 2
static int stategy_buckchg_set_usb_icl(int value, struct strategy_buckchg_dev *info)
{
#ifdef CONFIG_FACTORY_BUILD
	return 0;
#endif
	int icl_ma = value / 1000;//ua switch ma;
	static bool usb_suspend;
	int charge_boot_mode = mca_log_get_charge_boot_mode();

	if (charge_boot_mode || !info->is_eu_model) {
		mca_log_info("charge_boot_mode[%d] is_eu_model[%d] exit usb_icl", charge_boot_mode, info->is_eu_model);
		return 0;
	}
	mca_log_info("usb type[%d]icl_ma[%d]", info->proc_data.real_type, icl_ma);
	if (info->proc_data.real_type != XM_CHARGER_TYPE_SDP) {
		if (info->proc_data.real_type == XM_CHARGER_TYPE_PD ||
			info->proc_data.real_type == XM_CHARGER_TYPE_PPS) {
			if (!info->proc_data.pdsuspendsupported) {
				mca_log_info("USB suspend is not supported");
				return 0;
			} else {
				if (icl_ma == USB_SUSPEND_ICL_DEFAULT) {
					icl_ma = 0;
					mca_vote(info->input_limit_voter, "usbicl", true, icl_ma);
					mca_log_info("USB suspend for pd");
				} else if (icl_ma == 900 || icl_ma == 500 || icl_ma == 100) {
					mca_vote(info->input_limit_voter, "usbicl", false, 0);
					mca_log_info("USB Unsuspend for pd");
				} else
					mca_log_info("invalid ICL value for pd %d", icl_ma);
				return 0;
			}
		} else {
			mca_log_info("ICL setting is not allowed for usb type[%d]", info->proc_data.real_type);
			return 0;
		}
	}

	if (icl_ma == USB_SUSPEND_ICL_DEFAULT) {
		mca_log_info("USB Input Suspended");
		mca_vote(info->input_limit_voter, "usbicl", true, 0);
		usb_suspend = true;
		mca_event_block_notify(MCA_EVENT_TYPE_CHARGER_CONNECT, MCA_EVENT_USB_SUSPEND, &usb_suspend);
	} else if (icl_ma == USB_UNSUSPEND_ICL) {
		mca_log_info("USB Input UnSuspended");
		mca_vote(info->input_limit_voter, "usbicl", false, 0);
		if (usb_suspend) {
			usb_suspend = false;
			mca_event_block_notify(MCA_EVENT_TYPE_CHARGER_CONNECT, MCA_EVENT_USB_SUSPEND, &usb_suspend);
		}
	} else {
		if (info->proc_data.real_type == XM_CHARGER_TYPE_SDP &&
			(icl_ma == 900 || icl_ma == 500)) {
			mca_vote(info->input_limit_voter, "sdpicl", true, icl_ma);
			mca_vote(info->input_limit_voter, "usbicl", true, icl_ma);
			mca_log_info("ICL for SDP set by HLOS is %d mA", icl_ma);
		} else {
			mca_log_info("Invalid ICL for SDP set by HLOS is %d mA", icl_ma);
		}

		if (usb_suspend) {
			usb_suspend = false;
			mca_event_block_notify(MCA_EVENT_TYPE_CHARGER_CONNECT, MCA_EVENT_USB_SUSPEND, &usb_suspend);
		}
	}

	return 0;
}
static int strategy_buckchg_set_config(int config, int value, void *data)
{
	struct strategy_buckchg_dev *info = (struct strategy_buckchg_dev *)data;

	if (!info)
		return -1;

	switch (config) {
	case STRATEGY_CONFIG_INPUT_CURRENT_LIMIT:
		stategy_buckchg_set_usb_icl(value, info);
		break;
	default:
		break;
	}
	return 0;
}

static void strategy_buckchg_update_aicl_cfg(struct strategy_buckchg_dev *info)
{
	int aicl_thd = STATEGY_CHARGE_AICL_TH_4P4V;
	int reg_aicl_thd = 0;

	if (info->proc_data.chg_status == MCA_BUCK_CHG_NO_CHARGING
		|| info->proc_data.chg_status == MCA_BUCK_CHG_STS_CHARGE_DONE) {
		aicl_thd = STATEGY_CHARGE_AICL_TH_4P1V;
		mca_log_info("discharging or done, should set aicl vth to 4.1V\n");
		goto update_aicl_cfg;
	}

	if (info->proc_data.real_type >= XM_CHARGER_TYPE_HVDCP3 &&
		info->proc_data.real_type <= XM_CHARGER_TYPE_HVDCP3P5)
		return;

	if (info->proc_data.vbat >= STATEGY_CHARGE_AICL_VBAT_TH)
		aicl_thd = STATEGY_CHARGE_AICL_TH_4P5V;

#ifdef CONFIG_FACTORY_BUILD
	aicl_thd = STATEGY_CHARGE_AICL_TH_4P1V;
#endif

update_aicl_cfg:
	platform_class_buckchg_ops_get_usb_aicl_cont_thd(MAIN_BUCK_CHARGER, &reg_aicl_thd);
	if (aicl_thd != info->aicl_thd || aicl_thd != reg_aicl_thd) {
		mca_log_info("vbat: %d, aicl_thd: %d, reg_aicl_thd: %d\n", info->proc_data.vbat, aicl_thd, reg_aicl_thd);
		info->aicl_thd = aicl_thd;
		strategy_class_buckchg_ops_set_usb_aicl_cont_thd(info, aicl_thd);
	}
}

static void strategy_buckchg_select_charg_para(struct strategy_buckchg_dev *info)
{
	int ibus_limit = CHARGE_SDP_INPUT_DEFAULT;
	int ibat_limit = CHARGE_SDP_CHARGE_DEFAULT;
	int real_type = info->proc_data.real_type;
	struct mca_votable *in_voter, *ich_voter;
	bool volt_changed;
	int pd_active;
#ifdef CONFIG_FACTORY_BUILD
	bool cc_attach = false;
#endif

	if (info->proc_data.voltage != STATEGY_CHARGE_VBUS_9V &&
		real_type >= XM_CHARGER_TYPE_HVDCP2 && real_type <= XM_CHARGER_TYPE_HVDCP3P5)
		real_type = XM_CHARGER_TYPE_DCP;

	/*
	 * A float charger on a pack of two cells leaves the pump seeing the
	 * pack on its bus, so its over-voltage has to clear the pack's own
	 * range rather than the port's.
	 */
	if (info->base_flip_same && real_type == XM_CHARGER_TYPE_FLOAT) {
		bool cp_enabled = false;

		(void)platform_class_cp_set_busovp(CP_ROLE_MASTER,
			BUCKCHG_FLOAT_CP_BUSOVP_MV);
		(void)platform_class_cp_get_charging_enabled(CP_ROLE_MASTER,
			&cp_enabled);
		mca_log_info("cp_enabled = %d \n", cp_enabled);
	}

#ifdef CONFIG_FACTORY_BUILD
	protocol_class_pd_get_cc_status(TYPEC_PORT_0, &cc_attach);
	if (!cc_attach && (real_type == XM_CHARGER_TYPE_SDP || real_type == XM_CHARGER_TYPE_FLOAT))
		real_type = XM_CHARGER_TYPE_DCP;
	mca_log_err("cc_attach %d real_type %d\n", cc_attach, real_type);
#endif

	switch (real_type) {
	case XM_CHARGER_TYPE_UNKNOW:
		ibus_limit = 100;
		ibat_limit = 100;
		break;
	case XM_CHARGER_TYPE_SDP:
#ifdef CONFIG_FACTORY_BUILD
		ibus_limit = 600;
		ibat_limit = 600;
#else
		if (info->is_eu_model) {
				ibus_limit = info->in_sdp - 50;
		} else {
				ibus_limit = info->in_sdp;
		}
		ibat_limit = info->chg_sdp;
#endif
		break;
	case XM_CHARGER_TYPE_CDP:
		ibus_limit = info->in_cdp;
		ibat_limit = info->chg_cdp;
		break;
	case XM_CHARGER_TYPE_FLOAT:
	case XM_CHARGER_TYPE_OCP:
		ibus_limit = info->in_float;
		ibat_limit = info->chg_float;
		break;
	case XM_CHARGER_TYPE_DCP:
		if (info->smartchg_data.pwr_boost_state) {
			ibus_limit = CHARGE_DCP_INPUT_BOOST;
			ibat_limit = CHARGE_DCP_CHARGE_BOOST;
		} else {
			ibus_limit = info->in_dcp;
			ibat_limit = info->chg_dcp;
		}
		break;
	case XM_CHARGER_TYPE_HVDCP2:
		ibus_limit = info->in_hvdcp;
		ibat_limit = info->chg_hvdcp;
		break;
	case XM_CHARGER_TYPE_HVDCP3:
	case XM_CHARGER_TYPE_HVDCP3_B:
		ibus_limit = info->in_hvdcp3;
		ibat_limit = info->chg_hvdcp3;
		break;
	case XM_CHARGER_TYPE_HVDCP3P5:
		ibus_limit = info->in_hvdcp3p5;
		ibat_limit = info->chg_hvdcp3p5;
		break;
	case XM_CHARGER_TYPE_TYPEC:
		ibus_limit = info->in_dcp;
		ibat_limit = info->chg_dcp;
		break;
	case XM_CHARGER_TYPE_PD:
	case XM_CHARGER_TYPE_PPS:
	case XM_CHARGER_TYPE_PD_VERIFY:
		if (info->pps_ptf == USBPD_BUCK_DPM_PORT_PPS_PTF_NOT_OVERTEMP) {
			ibus_limit = CHARGE_PPS_PTF_INPUT_DEFAULT;
			ibat_limit = CHARGE_PPS_PTF_CHARGE_DEFAULT;
		} else {
			ibus_limit = min(info->in_pd, (unsigned int)info->pwr_cap.cap[info->proc_data.curr_pd_pos].max_current);
			ibat_limit = info->chg_pd;
			if (info->pwr_cap.cap[info->proc_data.curr_pd_pos].max_current == 0) {
				ibus_limit = min(info->in_pd, DEFAULT_PD_CURRENT_MA);
				mca_log_info("pdo broadcast abnormal %d \n", info->proc_data.curr_pd_pos);
			}
		}
		break;
	default:
		ibus_limit = info->in_sdp;
		ibat_limit = info->chg_sdp;
		break;
	};

	/*
	 * Where the two cells share a temperature reading, a hot pack on a
	 * verified adapter takes the separate, lower pair of limits.
	 */
	if (info->base_flip_same && strategy_class_fg_get_fastcharge() &&
	    real_type == XM_CHARGER_TYPE_PD_VERIFY) {
		int temp = 0, temp_offset_flag = 0;

		(void)strategy_class_fg_ops_get_temperature(&temp);
		temp = temp / 10;
		(void)strategy_class_fg_get_temp_offset_flag(&temp_offset_flag);
		if (temp > BUCKCHG_HOT_PACK_TEMP_C ||
		    (temp >= BUCKCHG_WARM_PACK_TEMP_C && temp_offset_flag)) {
			ibus_limit = info->in_pps;
			ibat_limit = info->chg_pps;
		}
	}
	mca_log_info("iin_pd = %d, ichg_pd = %d\n", ibus_limit, ibat_limit);

	protocol_class_pd_get_pd_active(TYPEC_PORT_0, &pd_active);
	/*
	 * Xiaomi's own release waits here for the charger class to say that
	 * BC1.2 detection has finished.  On this platform the ADSP owns that
	 * and has already finished by the time the event arrives.
	 */
	info->dpdm_detect_done = true;

	/*
	 * There is a pair of votables for each bus voltage, and each pair's
	 * callbacks only touch the hardware while their own voltage is the
	 * one in use.  Vote the live pair, and where the voltage itself has
	 * just changed ask that pair's elections again, so the limits recorded
	 * for it reach the charger instead of waiting for a limit to move.
	 */
	volt_changed = info->proc_data.voltage != info->proc_data.pre_volt;
	in_voter = info->proc_data.voltage == STATEGY_CHARGE_VBUS_5V ?
			info->buck_5v_in_voter : info->buck_9v_in_voter;
	ich_voter = info->proc_data.voltage == STATEGY_CHARGE_VBUS_5V ?
			info->buck_5v_ich_voter : info->buck_9v_ich_voter;

	if ((ibus_limit != info->proc_data.ibus_limit || volt_changed) &&
	    info->dpdm_detect_done) {
		mca_vote(in_voter, "wire_chg_type", true, ibus_limit);
		mca_vote(info->input_limit_voter, "subpmic_hw", false, 0); //avoid vote default vaule
		info->proc_data.ibus_limit = ibus_limit;
		mca_log_info("set ibus_limit = %d\n", ibus_limit);
	}

	if ((ibat_limit != info->proc_data.ibat_limit || volt_changed) &&
	    info->dpdm_detect_done) {
		mca_vote(ich_voter, "wire_chg_type", true, ibat_limit);
		mca_vote(info->charge_limit_voter, "subpmic_hw", false, 0); //avoid vote default vaule
		info->proc_data.ibat_limit = ibat_limit;
		mca_log_info("set ibat_limit = %d\n", ibat_limit);

		if (volt_changed) {
			mca_rerun_election(in_voter);
			mca_rerun_election(ich_voter);
			info->proc_data.pre_volt = info->proc_data.voltage;
		}
	}

	mca_log_info("ibus_limit = %d, ibat_limit = %d, bc1.2 det process = %d, pd_active = %d\n",
				ibus_limit, ibat_limit, info->dpdm_detect_done, pd_active);

	/*
	 * The limits above are what the charger is allowed to draw; what it
	 * can actually draw is what AICL last found, and an adapter that has
	 * recovered since then will not be noticed until AICL runs again.
	 */
	platform_class_buckchg_ops_set_restart_aicl(MAIN_BUCK_CHARGER, true);
}

static void strategy_buckchg_set_charge_volt(struct strategy_buckchg_dev *info, int target_volt)
{
	int volt = target_volt;

	if (!target_volt) {
		mca_log_info("target_volt = 0v is invalid\n");
		return;
	}

	switch (info->proc_data.real_type) {
	case XM_CHARGER_TYPE_HVDCP2:
	case XM_CHARGER_TYPE_HVDCP3:
	case XM_CHARGER_TYPE_HVDCP3_B:
	case XM_CHARGER_TYPE_HVDCP3P5:
		/*
		 * Xiaomi's own release drops to five volts here when the
		 * charge pump has stopped answering its bus.  There is no
		 * way to ask that on this platform, so the voltage the
		 * strategy chose stands.
		 */
		platform_class_buckchg_ops_set_qc_volt(MAIN_BUCK_CHARGER, volt);
		break;
	case XM_CHARGER_TYPE_PD:
		if (info->pwr_cap.pdo_nums == 0) {
			mca_log_info("pwr_cap nums is null\n");
			return;
		}
		/*
		 * Xiaomi's own release drops to five volts here when the
		 * charge pump has stopped answering its bus.  There is no
		 * way to ask that on this platform, so the voltage the
		 * strategy chose stands.
		 */
		(void)protocol_class_pd_set_fixed_volt(TYPEC_PORT_0, volt);
		break;
	case XM_CHARGER_TYPE_PPS:
	case XM_CHARGER_TYPE_PD_VERIFY:
		if (info->pwr_cap.pdo_nums == 0) {
			mca_log_info("pwr_cap nums is null\n");
			return;
		}
		/*
		 * Xiaomi's own release drops to five volts here when the
		 * charge pump has stopped answering its bus.  There is no
		 * way to ask that on this platform, so the voltage the
		 * strategy chose stands.
		 */
		if (volt == STATEGY_CHARGE_VBUS_5V || volt == STATEGY_CHARGE_VBUS_9V || volt == STATEGY_CHARGE_VBUS_12V) {
			(void)protocol_class_pd_set_fixed_volt(TYPEC_PORT_0, volt);
		} else {
			mca_log_info("set no fix pdo\n");
			protocol_class_set_adapter_volt_and_curr(ADAPTER_PROTOCOL_PPS, volt, CHARGE_PPS_INPUT_DEFAULT);
		}
		break;
	default:
		break;
	}
}

static void strategy_buck_update_req_volt(struct strategy_buckchg_dev *info)
{
	switch (info->proc_data.real_type) {
	case XM_CHARGER_TYPE_HVDCP2:
	case XM_CHARGER_TYPE_HVDCP3:
	case XM_CHARGER_TYPE_HVDCP3_B:
	case XM_CHARGER_TYPE_HVDCP3P5:
		mca_vote(info->input_voltage_voter, "real_type", true, 1);
		break;
	case XM_CHARGER_TYPE_PD:
	case XM_CHARGER_TYPE_PPS:
	case XM_CHARGER_TYPE_PD_VERIFY:
		if (info->proc_data.is_pd_9v)
			mca_vote(info->input_voltage_voter, "real_type", true, 1);
		else
			mca_vote(info->input_voltage_voter, "real_type", true, 0);
		break;
	default:
		mca_vote(info->input_voltage_voter, "real_type", true, 0);
		break;
	}
	mca_log_info("real_type = %d, input_voltage_voter effective_result: %d\n",
	info->proc_data.real_type, mca_get_effective_result(info->input_voltage_voter));
}

/*static int strategy_buckchg_check_online_msleep(int ms, struct strategy_buckchg_dev *info)
{
	int i, count;
	count = ms / 10;
	for (i = 0; i < count; i++) {
		if (!info->proc_data.online)
			return -1;
		usleep_range(9900, 11000);
	}
	return 0;
}*/

#define VBUS_9V_OVP_VOLTAGE 10000
#define VBUS_5V_OVP_VOLTAGE 7500
#define PMIC_INDUCTOR_SECURE_ICL 1000
static void strategy_buckchg_check_charge_volt(struct strategy_buckchg_dev *info)
{
	int target_volt = info->proc_data.voltage;
	int vbus = 0;

	if (info->wls_revchg_en) {
		mca_log_info("wireless reverse is charging, do not request volt\n");
		return;
	}

	if ((info->proc_data.real_type >= XM_CHARGER_TYPE_PD &&
		info->proc_data.real_type <= XM_CHARGER_TYPE_PD_VERIFY) && !info->pwr_cap.pdo_nums) {
		mca_log_info("pd pwr_cap nums is null\n");
		return;
	}

	platform_class_buckchg_ops_get_bus_volt(MAIN_BUCK_CHARGER, &vbus);
	vbus /= 1000;
	if ((target_volt == STATEGY_CHARGE_VBUS_9V && vbus < CHECK_VBUS_9V_HIGH_TH) ||
		(target_volt == STATEGY_CHARGE_VBUS_5V && vbus > CHECK_VBUS_5V_LOW_TH)) {
		mca_log_info("target_volt: %d, vbus: %d\n", target_volt, vbus);
		strategy_buckchg_set_charge_volt(info, target_volt);
	}

	platform_class_buckchg_ops_get_bus_volt(MAIN_BUCK_CHARGER, &vbus);
	vbus /= 1000;
	if (target_volt == STATEGY_CHARGE_VBUS_9V && vbus >= VBUS_9V_OVP_VOLTAGE) {
		mca_vote(info->buck_9v_in_voter, "pmic_inductor", true, PMIC_INDUCTOR_SECURE_ICL);
		mca_log_err("hvdcp_9v vbus_ovp decrease input current limit\n");
	} else {
		mca_vote(info->buck_9v_in_voter, "pmic_inductor", false, 0);
	}

	if (target_volt == STATEGY_CHARGE_VBUS_5V && vbus > VBUS_5V_OVP_VOLTAGE) {
		mca_vote(info->buck_5v_in_voter, "pmic_inductor", true, PMIC_INDUCTOR_SECURE_ICL);
		mca_log_err("lvdcp_5v vbus_ovp decrease input current limit\n");
	} else {
		mca_vote(info->buck_5v_in_voter, "pmic_inductor", false, 0);
	}

	if (info->is_non_compliant_qc && !info->non_compliant_run_once) {
		mca_log_err("non_compliant_qc: target_volt: %d\n", target_volt);
		strategy_buckchg_set_charge_volt(info, target_volt);
		mca_vote(info->input_limit_voter, "non_compliant_qc", true, 1500);
		strategy_class_buckchg_ops_set_chg(info, false);
		strategy_class_buckchg_ops_set_chg(info, true);
		mca_log_err("restart aicl and recover buck charging\n");
		/*
		 * The released driver's log says restart here but the call it
		 * makes is the rerun; the call is what the ADSP acts on.
		 */
		platform_class_buckchg_ops_set_rerun_aicl(MAIN_BUCK_CHARGER, true);
		info->non_compliant_run_once = true;
	}
}

#define VBUS_QC_VOL_THRESHOLD_LOW 7200
#define NON_COMPLIANT_QC_IBUS_THR_LOW 200
static void strategy_buckchg_check_non_compliant_qc_charger(struct strategy_buckchg_dev *info)
{
	int target_volt = info->proc_data.voltage;
	int vbus = 0, ibus = 0;
	static int count;

	if (target_volt == STATEGY_CHARGE_VBUS_5V || info->is_non_compliant_qc)
		return;

	if (info->proc_data.chg_status == MCA_BUCK_CHG_STS_CHARGE_DONE) {
		mca_log_info("charge done, no need check non_compliant_qc_charger\n");
		return;
	}

	if (info->proc_data.real_type != XM_CHARGER_TYPE_HVDCP2 &&
		info->proc_data.real_type != XM_CHARGER_TYPE_HVDCP3) {
		mca_log_info("only qc2/qc3 need check\n");
		return;
	}

	platform_class_buckchg_ops_get_bus_volt(MAIN_BUCK_CHARGER, &vbus);
	vbus /= 1000;
	(void)platform_class_buckchg_ops_get_bus_curr(MAIN_BUCK_CHARGER, &info->proc_data.ibus);
	ibus = info->proc_data.ibus / 1000;
	if (target_volt == STATEGY_CHARGE_VBUS_9V && vbus < VBUS_QC_VOL_THRESHOLD_LOW
		&& ibus < NON_COMPLIANT_QC_IBUS_THR_LOW) {
		mca_log_info("target_volt: %d, vbus: %d\n", target_volt, vbus);
		count++;
	}
	if (count >= 3 && !info->is_non_compliant_qc) {
		info->is_non_compliant_qc = true;
		count = 0;
		mca_log_err("qc is non-compliant, set request vol to 5V\n");
		mca_vote(info->input_voltage_voter, "non_compliant_qc", true, 0);
	}
}

static int strategy_buckchg_update_charge_status(struct strategy_buckchg_dev *info)
{
	int main_chg_status = 0;
	int aux_chg_status = 0;
	int icl = 0;
	int ichg = 0;
	int suspend = 0;
	int charging_done = 0;
	int adsp_icl = 0;

	(void)platform_class_buckchg_ops_get_chg_status(MAIN_BUCK_CHARGER, &main_chg_status);
	(void)platform_class_buckchg_ops_get_chg_status(AUX_BUCK_CHARGER, &aux_chg_status);
	(void)platform_class_buckchg_ops_get_bus_curr(MAIN_BUCK_CHARGER, &info->proc_data.ibus);
	(void)platform_class_buckchg_ops_get_bus_volt(MAIN_BUCK_CHARGER, &info->proc_data.vbus);
	(void)strategy_class_fg_ops_get_voltage(&info->proc_data.vbat);
	(void)platform_class_buckchg_ops_get_input_curr_lmt(MAIN_BUCK_CHARGER, &adsp_icl);
	icl = mca_get_effective_result(info->input_limit_voter);
	ichg = mca_get_effective_result(info->charge_limit_voter);
	suspend = mca_get_effective_result(info->input_suppend_voter);

	switch (main_chg_status) {
	case MCA_BATT_CHGR_STATUS_INHIBIT:
	case MCA_BATT_CHGR_STATUS_TRICKLE:
	case MCA_BATT_CHGR_STATUS_PRECHARGE:
	case MCA_BATT_CHGR_STATUS_FULLON:
	case MCA_BATT_CHGR_STATUS_TAPER:
	case MCA_BATT_CHGR_STATUS_FAST_LINEAR:
		info->proc_data.chg_status = MCA_BUCK_CHG_STS_CHARGING;
		break;
	case MCA_BATT_CHGR_STATUS_TERMINATION:
		info->proc_data.chg_status =  MCA_BUCK_CHG_STS_CHARGE_DONE;
		break;
	case MCA_BATT_CHGR_STATUS_PAUSE:
	case MCA_BATT_CHGR_STATUS_CHARGING_DISABLED:
		info->proc_data.chg_status = MCA_BUCK_CHG_NO_CHARGING;
		break;
	default:
		info->proc_data.chg_status = MCA_BUCK_CHG_STS_NA;
		break;
	}

	charging_done = strategy_class_fg_ops_get_charging_done();
	if (charging_done)
		info->proc_data.chg_status =  MCA_BUCK_CHG_STS_CHARGE_DONE;

	/*
	 * Trickle means the charger has given up on the target it was set,
	 * so whatever was holding it there has already had its effect.
	 */
	if (main_chg_status == MCA_BATT_CHGR_STATUS_TRICKLE &&
	    !info->pmic_temp_term_flag) {
		strategy_buckchg_clear_pmic_temp_term(info);
		info->pmic_temp_term_flag = true;
	}

	mca_log_err("pmic_chg_status: %d, chg_status: %d, chg_en: [%d][%s], chg_type: %d, vbat: %d, ibat: %d, vbus: %d, ibus: %d\n",
		main_chg_status, info->proc_data.chg_status,
		mca_get_effective_result(info->chg_enable_voter), mca_get_effective_client(info->chg_enable_voter),
		info->proc_data.real_type, info->proc_data.vbat, info->proc_data.ibat, info->proc_data.vbus, info->proc_data.ibus);
	mca_log_err("ap_icl:[%s][%d], adsp_icl[%d], ichg:[%s][%d], input_suspend:[%s][%d]\n",
		mca_get_effective_client(info->input_limit_voter), icl, adsp_icl,
		mca_get_effective_client(info->charge_limit_voter), ichg,
		mca_get_effective_client(info->input_suppend_voter), suspend);
	return info->proc_data.chg_status;
}

/*
 * A charge pump that has stopped answering its bus leaves the port with no
 * over-voltage protection at all on the boards whose protection sits behind
 * the pump, so once it has failed to answer for long enough the adapter is
 * asked back down to five volts.  A few passes of slack come first: the pump
 * is also unreachable for a moment while it is being reconfigured.
 */
static void strategy_buckchg_check_cp_i2c_err(struct strategy_buckchg_dev *info)
{
	bool cp_present = false;

	if (!info->vusb_ovp_location)
		return;

	if (!platform_class_cp_get_present(CP_ROLE_MASTER, &cp_present) &&
	    cp_present) {
		info->cp_absent_retry_count = 0;
		info->cp_i2c_error_reported = false;
		return;
	}

	if (info->cp_absent_retry_count < BUCKCHG_CP_ABSENT_RETRY_MAX) {
		info->cp_absent_retry_count++;
		mca_log_info("CP not present, retrying... retry_count: %d",
			     info->cp_absent_retry_count);
		return;
	}

	if (info->cp_i2c_error_reported ||
	    info->vusb_ovp_location != BUCKCHG_VUSB_OVP_BEFORE_CP)
		return;

	mca_log_err("Max retries reached, CP still not present.");
	mca_log_info("vusb_ovp_location is BEFORE_CP, forcing 5V, reporting error.");
	mca_vote(info->input_voltage_voter, "cp_i2c_error", true, 0);
	info->cp_i2c_error_reported = true;
}

static void strategy_buckchg_enable_fast_charge_mode(struct strategy_buckchg_dev *info, int soc)
{
	int batt_temp, fastcharge_mode, buck_jeita_ffc_size;
	int iterm = mca_get_effective_result(info->iterm_voter);
	int fcc = mca_get_effective_result(info->charge_limit_voter);
	int quick_charge_status = MCA_QUICK_CHG_STS_CHARGE_FAILED;

	/* A charge the pump ends is none of this loop's business. */
	if (info->terminated_by_cp)
		return;

	/*
	 * A flip pack that has reached its own termination current on a
	 * verified adapter, while the reading the two cells share says it is
	 * hot, is moved onto the lower pair of limits at once rather than
	 * waiting for the next pass through the charger-type selection.
	 */
	if (info->support_base_flip || info->base_flip_same) {
		int iterm_para = info->parallel_iterm;

		mca_log_info("iterm: %d\n", iterm_para);

		if (info->base_flip_same && strategy_class_fg_get_fastcharge() &&
		    info->proc_data.real_type == XM_CHARGER_TYPE_PD_VERIFY &&
		    fcc <= iterm_para) {
			int temp = 0, temp_offset_flag = 0;

			(void)strategy_class_fg_ops_get_temperature(&temp);
			temp = temp / 10;
			(void)strategy_class_fg_get_temp_offset_flag(&temp_offset_flag);
			if (temp > BUCKCHG_HOT_PACK_TEMP_C ||
			    (temp >= BUCKCHG_WARM_PACK_TEMP_C && temp_offset_flag)) {
				struct mca_votable *ich;

				if (info->proc_data.voltage ==
				    STATEGY_CHARGE_VBUS_5V) {
					mca_vote(info->buck_5v_in_voter,
						 "wire_chg_type", true,
						 info->in_pps);
					ich = info->buck_5v_ich_voter;
				} else {
					mca_vote(info->buck_9v_in_voter,
						 "wire_chg_type", true,
						 info->in_pps);
					ich = info->buck_9v_ich_voter;
				}
				mca_vote(ich, "wire_chg_type", true,
					 info->chg_pps);
			}
		}
	}

	mca_log_info("allow_start_ffc_batt_soc_thr: %d\n",
		     info->allow_start_ffc_batt_soc_thr);

	if (info->proc_data.real_type == XM_CHARGER_TYPE_PD_VERIFY) {
		(void)mca_strategy_func_get_status(STRATEGY_FUNC_TYPE_QUICK_CHARGE,
				STRATEGY_STATUS_TYPE_CHARGING, &quick_charge_status);
		(void)strategy_class_fg_ops_get_temperature(&batt_temp);
		batt_temp /= 10;

		mca_log_info("batt_temp = %d, soc =%d, fcc = %d, iterm = %d, quickchg_sts = %d\n",
			batt_temp, soc, fcc, iterm, quick_charge_status);

		/*
		 * Xiaomi's own release asks the jeita strategy how many
		 * fast-charge bands its table has.  This platform's jeita
		 * answers no such status, so the termination current it
		 * offers stands in: a band that has one is a band that can
		 * fast charge.
		 */
		mca_strategy_func_get_status(STRATEGY_FUNC_TYPE_JEITA,
					     STRATEGY_STATUS_TYPE_JEITA_FFC_ITERM,
					     &buck_jeita_ffc_size);

		fastcharge_mode = strategy_class_fg_get_fastcharge();
		if (!fastcharge_mode && soc < info->allow_start_ffc_batt_soc_thr
			&& batt_temp >= info->ffc_temp_low && batt_temp <= info->ffc_temp_high && info->batt_auth
			&& buck_jeita_ffc_size) {
			strategy_class_fg_set_fastcharge(true);
			mca_log_info("buck charger enable fast charge mode\n");
		} else if (batt_temp < info->ffc_temp_low
			|| batt_temp > info->ffc_temp_high) {
			strategy_class_fg_set_fastcharge(false);
			mca_log_info("buck charger disable fast charge mode\n");
		} else if (fastcharge_mode && soc >= info->allow_start_ffc_batt_soc_thr
			&& fcc <= iterm + info->curr_term_compensation
			&& quick_charge_status != MCA_QUICK_CHG_STS_CHARGING) {
			const char *client;

			/*
			 * On a flip pack the charge pump runs alongside the
			 * buck charger and holds its share down through this
			 * very election, so a low current from that client
			 * says nothing about how full the pack is.
			 */
			if (info->support_base_flip || info->base_flip_same) {
				client = mca_get_effective_client(info->charge_limit_voter);
				if (client && !strcmp(client, "fcc_limit")) {
					mca_log_info("client_srt:%s is pmic + cp charging, skip\n",
						     client);
					return;
				}
			}

			strategy_class_fg_set_fastcharge(false);
			/*
			 * The quick charge strategy sets its own terms from
			 * whether the pack is fast charging, so it has to
			 * hear that this has stopped.
			 */
			mca_strategy_func_process(STRATEGY_FUNC_TYPE_QUICK_CHARGE,
				MCA_EVENT_FCC_TOO_LOW, 0);
			mca_log_info("buck charger disable fast charge mode by fcc too low\n");
		}
	}
}

static void strategy_buckchg_eoc_force_5v(struct strategy_buckchg_dev *info)
{
	mca_vote(info->input_voltage_voter, "eoc_5v", true, 0);
}

#define COLD_ZONE_LOW -100
#define COLD_ZONE_HIGH 50
#define HOT_ZONE_LOW 480
#define HOT_ZONE_HIGH 550
#define IS_BETWEEN(val, lval, rval)	((val >= lval) ? ((val <= rval) ? true :  false) : \
								((val >= rval) ? true : false))
static int strategy_buckchg_charge_abnormal_cold_or_hot_zone(struct strategy_buckchg_dev *info)
{
	int icl = 0;
	int chg_en = 0;
	int temp = 0;
	int vterm = 0;
	static int count;

	strategy_class_fg_ops_get_temperature(&temp);
	if (!IS_BETWEEN(temp, COLD_ZONE_LOW, COLD_ZONE_HIGH) &&
		!IS_BETWEEN (temp, HOT_ZONE_LOW, HOT_ZONE_HIGH)) {
		mca_log_info("temp: %d, not in cold or hot zone\n", temp);
		return 0;
	}

	icl = mca_get_effective_result(info->input_limit_voter);
	chg_en = mca_get_effective_result(info->chg_enable_voter);
	vterm = mca_get_effective_result(info->vterm_voter);
	if (!icl || !chg_en || info->proc_data.vbat >= vterm) {
		mca_log_info("user can stop charging icl: %d, chg_en: %d, vterm: %d, vbat: %d, temp:%d\n",
			icl, chg_en, vterm, info->proc_data.vbat, temp);
		return 0;
	}

	if (info->proc_data.chg_status == MCA_BUCK_CHG_NO_CHARGING) {
		/* temp ranges aren't overlaping, so using the same count variable is safe */
		if (IS_BETWEEN (temp, COLD_ZONE_LOW, COLD_ZONE_HIGH)) {
			count++;
			if (count > 3) {
				count = 0;
				mca_charge_mievent_report(CHARGE_DFX_LOW_TEMP_DISCHARGING, &temp, 1);
			}
		} else if (IS_BETWEEN (temp, HOT_ZONE_LOW, HOT_ZONE_HIGH)) {
			count++;
			if (count > 3) {
				count = 0;
				mca_charge_mievent_report(CHARGE_DFX_HIGH_TEMP_DISCHARGING, &temp, 1);
			}
		} else {
			count = 0;
		}
		mca_log_info("count: %d\n", count);
	} else {
		count = 0;
	}

	return 0;
}

#define STRATEGY_BUCKCHG_ENTER_QUICKCHG_TIME_MS 7000
#define VBAT_DROP_COUNT_TH 3
/* How far under jeita's target the buck charger is held once the pump lets go. */
#define BUCKCHG_CP_TO_PMIC_VTERM_DROP	5

/*
 * On a board where the charge pump hands the pack back near the top of the
 * charge rather than terminating itself, the buck charger has to finish under
 * the target the pump was working to, or it will simply sit at that target and
 * never terminate.  Once the pack has terminated once the hold comes off
 * again.
 */
static void strategy_buckchg_cp_to_pmic_decrease_vterm(struct strategy_buckchg_dev *info)
{
	static int last_jeita_vterm;
	int first_termination = 0;
	int taper_cp_to_pmic = 0;
	int target_vterm = 0;

	if (!info->need_cp_to_pmic ||
	    info->proc_data.real_type != XM_CHARGER_TYPE_HVDCP3_B)
		return;

	strategy_class_fg_get_first_termination(&first_termination);
	(void)mca_strategy_func_get_status(STRATEGY_FUNC_TYPE_QUICK_CHARGE,
					   STRATEGY_STATUS_TYPE_CP_TO_PMIC,
					   &taper_cp_to_pmic);

	if (first_termination) {
		mca_vote(info->vterm_voter, "cp_to_pmic", false,
			 STATEGY_VTERM_DEFAULT_VALUE);
	} else if (taper_cp_to_pmic) {
		int jeita_vterm = mca_get_client_vote(info->vterm_voter,
						      "jeita");

		if (jeita_vterm != last_jeita_vterm) {
			target_vterm = jeita_vterm -
				       BUCKCHG_CP_TO_PMIC_VTERM_DROP;
			mca_vote(info->vterm_voter, "cp_to_pmic", true,
				 target_vterm);
			last_jeita_vterm = jeita_vterm;
		}
	}

	mca_log_err("first_termination_flag: %d, taper_cp_to_pmic: %d, target_vterm: %d\n",
		    first_termination, taper_cp_to_pmic, target_vterm);
}

static void strategy_buckchg_monitor_workfunc(struct work_struct *work)
{
	struct strategy_buckchg_dev *info = container_of(work,
				struct strategy_buckchg_dev, monitor_work.work);
	int interval = CHARGE_MONITOR_WORK_NORMAL_INTERVAL;
	int quick_charge_status = MCA_QUICK_CHG_STS_CHARGE_FAILED;
	int input_suspned = 0;
	int chg_en = 0, system_soc;
	int vterm = 0;
	int jeita_hot_result = 1;
	ktime_t time_now;
	int verifed = 0;
	bool vbat_drop_exit_flag = false;
	static int vbat_drop_cnt;
	int active_port = protocol_class_pd_get_port_num();
	struct mca_hwid_info *hwid = mca_get_hwid_info();

	system_soc = strategy_class_fg_ops_get_soc();

	strategy_buckchg_check_debug_soc_limit(info, system_soc);

	if (system_soc <= ALLOW_QUICK_CHG_SOC_THR) {
		jeita_hot_result = mca_get_client_vote(info->chg_enable_voter, "jeita-hot");
		mca_log_info("jeita_hot vote value: %d\n", jeita_hot_result);
		switch (info->proc_data.real_type) {
		case XM_CHARGER_TYPE_HVDCP3P5:
			/*
			 * QC3+ fast charging is only offered on Chinese units;
			 * elsewhere such an adapter charges as an ordinary
			 * one.
			 */
			if (!hwid || hwid->country_version != CountryCN)
				break;
			fallthrough;
		case XM_CHARGER_TYPE_HVDCP3_B:
		case XM_CHARGER_TYPE_PD_VERIFY:
			if (jeita_hot_result)
				mca_strategy_func_process(STRATEGY_FUNC_TYPE_QUICK_CHARGE,
					MCA_EVENT_CHARGE_ACTION, 0);
			break;
		case XM_CHARGER_TYPE_PPS:
			time_now = ktime_get_boottime();
			protocol_class_pd_get_pd_verifed(active_port, &verifed);
			if (info->is_eu_model) {
				mca_log_info("eu_start_time %lld, time_now %lld  delta = %lld\n", info->proc_data.eu_start_time, time_now,
					ktime_ms_delta(time_now, info->proc_data.eu_start_time));
				if (ktime_ms_delta(time_now, info->proc_data.eu_start_time) > STRATEGY_BUCKCHG_ENTER_QUICKCHG_TIME_MS) {
					mca_strategy_func_process(STRATEGY_FUNC_TYPE_QUICK_CHARGE,
					MCA_EVENT_CHARGE_ACTION, 0);
					info->proc_data.eu_start_time = time_now;
					mca_log_info("trigger MCA_EVENT_CHARGE_ACTION\n");
				}
			} else {
				/*
				 * The shipped module triggers on the jeita vote
				 * and verify_process_end alone.  Skipping an
				 * adapter that has already come back verified
				 * only avoids asking for a quick charge that is
				 * running; nothing else here reads verifed.
				 */
				if (jeita_hot_result && info->verify_process_end && !verifed) {
					mca_strategy_func_process(STRATEGY_FUNC_TYPE_QUICK_CHARGE,
						MCA_EVENT_CHARGE_ACTION, 0);
				}
				mca_log_info("verify_process_end: %d, verifed: %d\n", info->verify_process_end, verifed);
			}
			break;
		default:
			break;
		}

	} else {
		strategy_buckchg_enable_fast_charge_mode(info, system_soc);
	}

	(void)mca_strategy_func_get_status(STRATEGY_FUNC_TYPE_QUICK_CHARGE,
				STRATEGY_STATUS_TYPE_CHARGING, &quick_charge_status);
	if (quick_charge_status == MCA_QUICK_CHG_STS_CHARGING)
		goto out;

	strategy_buckchg_check_cp_i2c_err(info);
	strategy_buckchg_check_charge_volt(info);
	strategy_buckchg_select_charg_para(info);
	strategy_buckchg_update_charge_status(info);
	strategy_buckchg_check_non_compliant_qc_charger(info);
	strategy_buckchg_update_aicl_cfg(info);
	strategy_buckchg_limit_full_replug_ichg(info, false);
	strategy_buckchg_resume_buck_ichg_limit(info);
	if (info->proc_data.chg_status == MCA_BUCK_CHG_STS_CHARGING) {
		if (info->proc_data.charge_done_force_5v) {
			info->proc_data.charge_done_force_5v = false;
			mca_vote(info->input_voltage_voter, "eoc_5v", false, 0);
		}
		// WA: rerun aicl if ibus too low
		if (info->proc_data.ibus < 150000 && mca_get_effective_result(info->input_limit_voter) >= 500) {
			mca_log_info("ibus abnormally low: %d, restart aicl\n", info->proc_data.ibus);
			platform_class_buckchg_ops_set_restart_aicl(MAIN_BUCK_CHARGER, true);
		}
		strategy_buckchg_cp_to_pmic_decrease_vterm(info);
	}
	if ((info->proc_data.chg_status == MCA_BUCK_CHG_STS_CHARGE_DONE) &&
		!info->proc_data.charge_done_force_5v) {
		strategy_buckchg_sw_cv_stop(info);
		strategy_buckchg_eoc_force_5v(info);
		info->proc_data.charge_done_force_5v = true;
	}
	if (!info->proc_data.chg_en) {
		mca_vote(info->chg_enable_voter, "online", true, STATEGY_CHARGE_ENABLE);
		info->proc_data.chg_en = STATEGY_CHARGE_ENABLE;
	}

	// WA: solve pmic abnormal status
	vterm = mca_get_effective_result(info->vterm_voter);
	input_suspned = mca_get_effective_result(info->input_suppend_voter);
	chg_en = mca_get_effective_result(info->chg_enable_voter);
	/*
	 * The charger having started again is what lifts the forced five
	 * volts below; nothing else clears it.
	 */
	if (mca_is_client_vote_enabled(info->input_voltage_voter,
				       "recover_force_5v") &&
	    info->proc_data.chg_status == MCA_BUCK_CHG_STS_CHARGING) {
		mca_log_err("buck charging recovered, disable force 5V\n");
		mca_vote(info->input_voltage_voter, "recover_force_5v",
			 false, 0);
	}

	if (vterm >= info->sw_cv_vterm_th && !input_suspned) {
		if (chg_en && info->proc_data.chg_status == MCA_BUCK_CHG_NO_CHARGING) {
			/*
			 * A charger that will not start while it is being held
			 * above six volts is asked down to five before it is
			 * bounced, since the higher rail is the usual reason.
			 */
			if (info->proc_data.vbus > STATEGY_CHARGE_VBUS_ABNORMAL_UV) {
				mca_log_err("9V buck charging abnormal, set to 5v\n");
				mca_vote(info->input_voltage_voter,
					 "recover_force_5v", true, 0);
			}
			strategy_class_buckchg_ops_set_chg(info, false);
			strategy_class_buckchg_ops_set_chg(info, true);
			mca_log_err("recover buck charging\n");
		}
	}
	strategy_buckchg_charge_abnormal_cold_or_hot_zone(info);

	if (info->proc_data.vbat <= vterm - (20 * info->vbat_fg_to_pmic_ratio)) {
		vbat_drop_cnt++;
	}
	vbat_drop_exit_flag = vbat_drop_cnt >= VBAT_DROP_COUNT_TH ? true : false;
	vbat_drop_cnt = vbat_drop_exit_flag ? 0 : vbat_drop_cnt;

	if (info->proc_data.chg_status == MCA_BUCK_CHG_STS_CHARGING &&
		!info->sw_cv_running && vterm >= info->sw_cv_vterm_th &&
		info->proc_data.vbat >= vterm - CHARGE_SW_CV_VBAT_ALARM_DELTA) {
		mca_log_err("vbat: %d, vterm: %d, start sw_cv_work\n", info->proc_data.vbat, vterm);
		strategy_buckchg_sw_cv_start(info);
	} else if (info->sw_cv_running && (!chg_en || vbat_drop_exit_flag)) {
		mca_log_err("vbat: %d, vterm: %d, chg_en: %d, stop sw_cv_work\n", info->proc_data.vbat, vterm, chg_en);
		strategy_buckchg_sw_cv_stop(info);
	}

	if (quick_charge_status == MCA_QUICK_CHG_STS_CHARGE_DONE)
		interval = CHARGE_MONITOR_WORK_FAST_INTERVAL;

out:
	schedule_delayed_work(&info->monitor_work, msecs_to_jiffies(interval));
}

static void strategy_buckchg_sw_cv_start(struct strategy_buckchg_dev *info)
{
	info->sw_cv_running = true;
	mca_vote(info->charge_limit_voter, "sw_cv", false, 0);
	mca_queue_delayed_work(&info->sw_cv_work, msecs_to_jiffies(CHARGE_SW_CV_WORK_FAST_INTERVAL));
}

static void strategy_buckchg_sw_cv_stop(struct strategy_buckchg_dev *info)
{
	mca_log_err("chg_status: %d, stop sw_cv_work\n", info->proc_data.chg_status);
	cancel_delayed_work_sync(&info->sw_cv_work);
	cancel_delayed_work_sync(&info->base_flip_sw_cv_work);
	mca_vote(info->charge_limit_voter, "sw_cv", false, 0);
	info->sw_cv_running = false;
	info->vbat_ov_count = 0;
}

/*
 * The charger's own constant-voltage phase overshoots on this pack, so the
 * last stretch is driven from here: the charge current is stepped down as the
 * pack approaches its termination voltage, and if the voltage gets there
 * anyway the charger's own target is pulled back under it.
 *
 * How close to the target counts as "approaching" depends on how much current
 * is still going in, since a larger current means a larger IR drop between
 * where the gauge measures and where the cell actually is.
 */
static void strategy_buckchg_sw_cv_workfunc(struct work_struct *work)
{
	struct strategy_buckchg_dev *info = container_of(work,
		struct strategy_buckchg_dev, sw_cv_work.work);
	static const struct {
		int ibat;
		int volt_delta;
	} sw_cv_volt_delta_map[] = {
		{ 1000, 4 },
		{ 0, 2 },
	};
	int interval = CHARGE_SW_CV_WORK_NORMAL_INTERVAL;
	int vbat, ibat, batt_temp = 0;
	int vterm = mca_get_effective_result(info->vterm_voter);
	int iterm = mca_get_effective_result(info->iterm_voter);
	int fcc = mca_get_effective_result(info->charge_limit_voter);
	int fv_comp = info->pmic_fv_compensation;
	int volt_delta = sw_cv_volt_delta_map[0].volt_delta;
	bool fastcharge;
	int i;

	strategy_class_fg_ops_get_voltage(&info->proc_data.vbat);
	strategy_class_fg_ops_get_current(&info->proc_data.ibat);
	vbat = info->proc_data.vbat;
	ibat = -info->proc_data.ibat / 1000;
	mca_log_info("vbat: %d, ibat: %d, vterm: %d, iterm: %d, fcc: %d\n",
		vbat, ibat, vterm, iterm, fcc);

	for (i = 0; i < ARRAY_SIZE(sw_cv_volt_delta_map); i++) {
		if (ibat > sw_cv_volt_delta_map[i].ibat) {
			volt_delta = sw_cv_volt_delta_map[i].volt_delta;
			break;
		}
	}

	if (vbat >= vterm - volt_delta) {
		interval = CHARGE_SW_CV_WORK_FAST_INTERVAL;
		if (ibat - info->sw_cv_fcc_step > iterm) {
			if (fcc - ibat >= 2 * info->sw_cv_fcc_step)
				mca_vote(info->charge_limit_voter, "sw_cv", true,
					ibat / info->sw_cv_fcc_step * info->sw_cv_fcc_step);
			else
				mca_vote(info->charge_limit_voter, "sw_cv", true,
					fcc - info->sw_cv_fcc_step);
		}
	} else {
		interval = CHARGE_SW_CV_WORK_NORMAL_INTERVAL;
	}

	/*
	 * Where the board asks for it, the headroom the charger was given
	 * follows the pack temperature, so pulling the target back has to use
	 * the same headroom rather than the flat one.
	 */
	if (info->support_pmic_vterm_dynamics_adjust) {
		fastcharge = strategy_class_fg_get_fastcharge();
		strategy_class_fg_ops_get_temperature(&batt_temp);
		batt_temp /= 10;
		if (fastcharge && batt_temp >= FV_COMP_WARM_LOW &&
		    batt_temp <= FV_COMP_WARM_HIGH)
			fv_comp = info->pmic_fv_compensation;
		else if (fastcharge && batt_temp >= FV_COMP_MIDDLE_LOW &&
			 batt_temp <= FV_COMP_MIDDLE_HIGH)
			fv_comp = info->pmic_middle_fv_compensation;
		else if (fastcharge && batt_temp >= FV_COMP_HIGH_LOW &&
			 batt_temp <= FV_COMP_HIGH_HIGH)
			fv_comp = info->pmic_high_fv_compensation;
		else
			fv_comp = 0;
	}

	if (vbat >= (vterm - 1)) {
		mca_log_err("WARNING: batt ov, reduce fv, count: %d\n", info->vbat_ov_count);
		++info->vbat_ov_count;
		if (info->vbat_ov_count == 1)
			platform_class_buckchg_ops_set_term_volt(MAIN_BUCK_CHARGER,
				vterm + fv_comp - info->sw_cv_fv_step);
		else if (info->vbat_ov_count >= 2)
			platform_class_buckchg_ops_set_term_volt(MAIN_BUCK_CHARGER,
				vterm + fv_comp - 2 * info->sw_cv_fv_step);
	}

	mca_queue_delayed_work(&info->sw_cv_work, msecs_to_jiffies(interval));
}

/*
 * The flip-base variant of the sw_cv loop. Unlike the plain one it only
 * trims the current while the gauge reports fastcharge, takes the
 * termination target from jeita rather than the effective vote, holds the
 * pack termination current the base reported rather than the iterm vote,
 * and on repeated over-voltage walks the term volt down by up to five
 * steps instead of two.
 */
#define BASE_FLIP_FV_STEP		3
#define BASE_FLIP_FASTCHG_FV_STEP	5
#define BASE_FLIP_OV_STEP		5
#define BASE_FLIP_FASTCHG_OV_STEP	10
#define BASE_FLIP_OV_STEP_MAX		5

static void strategy_buckchg_base_flip_sw_cv_workfunc(struct work_struct *work)
{
	struct strategy_buckchg_dev *info = container_of(work,
		struct strategy_buckchg_dev, base_flip_sw_cv_work.work);
	static const struct {
		int ibat;
		int volt_delta;
	} base_flip_sw_cv_volt_delta_map[] = {
		{ 1000, 7 },
		{ 0, 2 },
	};
	int interval = CHARGE_SW_CV_WORK_NORMAL_INTERVAL;
	int first_termination = 0;
	int vbat, ibat, actual_vterm, fv_step, ov_step;
	int vterm = mca_get_client_vote(info->vterm_voter, "jeita");
	int iterm = info->parallel_iterm;
	int fcc = mca_get_effective_result(info->charge_limit_voter);
	int volt_delta = base_flip_sw_cv_volt_delta_map[0].volt_delta;
	bool fastcharge;
	int i;

	strategy_class_fg_ops_get_voltage(&info->proc_data.vbat);
	strategy_class_fg_ops_get_current(&info->proc_data.ibat);
	fastcharge = strategy_class_fg_get_fastcharge();
	vbat = info->proc_data.vbat;
	ibat = -info->proc_data.ibat / 1000;

	strategy_class_fg_get_first_termination(&first_termination);
	mca_log_info("first_termination_flag:%d\n", first_termination);

	fv_step = fastcharge ? BASE_FLIP_FASTCHG_FV_STEP : BASE_FLIP_FV_STEP;
	ov_step = fastcharge ? BASE_FLIP_FASTCHG_OV_STEP : BASE_FLIP_OV_STEP;
	/*
	 * Only the value the buck is programmed with follows the effective
	 * vote after a first termination; the thresholds stay on jeita's.
	 */
	actual_vterm = info->pmic_fv_compensation + (first_termination ?
		mca_get_effective_result(info->vterm_voter) : vterm);

	mca_log_info("vbat: %d, ibat: %d, vterm: %d, actual_vterm: %d, iterm: %d, fcc: %d, fast_charge: %d, fv_step: %d\n",
		vbat, ibat, vterm, actual_vterm, iterm, fcc, fastcharge, ov_step);

	for (i = 0; i < ARRAY_SIZE(base_flip_sw_cv_volt_delta_map); i++) {
		if (ibat > base_flip_sw_cv_volt_delta_map[i].ibat) {
			volt_delta = base_flip_sw_cv_volt_delta_map[i].volt_delta;
			break;
		}
	}

	if (fastcharge && vbat >= vterm - volt_delta) {
		interval = CHARGE_SW_CV_WORK_FAST_INTERVAL;
		if (ibat - SW_CV_FCC_STEP_DEFAULT > iterm &&
		    fcc - ibat >= 2 * SW_CV_FCC_STEP_DEFAULT)
			mca_vote(info->charge_limit_voter, "sw_cv", true,
				ibat / SW_CV_FCC_STEP_DEFAULT * SW_CV_FCC_STEP_DEFAULT);
	}

	if (vbat >= vterm - fv_step) {
		mca_log_err("WARNING: batt ov, reduce fv, count: %d\n", info->vbat_ov_count);
		++info->vbat_ov_count;
		platform_class_buckchg_ops_set_term_volt(MAIN_BUCK_CHARGER,
			actual_vterm - min(info->vbat_ov_count, BASE_FLIP_OV_STEP_MAX) * ov_step);
	}

	mca_queue_delayed_work(&info->base_flip_sw_cv_work, msecs_to_jiffies(interval));
}

static int strategy_buckchg_wireless_revchg_msleep(int ms, struct strategy_buckchg_dev *info)
{
	int i, count;

	count = ms / 10;

	for (i = 0; i < count; i++) {
		if (!info->proc_data.online || !info->wls_revchg_en)
			return -1;
		usleep_range(9900, 11000);
	}

	return 0;
}

static void strategy_buckchg_exit_wireless_revchg(struct strategy_buckchg_dev *info)
{
	mca_vote(info->input_limit_voter, "wireless_revchg", false, 0);
	info->proc_data.wls_revchg_init_done = false;
	info->rev_icl_for_qc2 = false;
}

static int strategy_buckchg_process_wireless_revchg(struct strategy_buckchg_dev *info)
{
	int real_type = info->proc_data.real_type;
	int vbus = 0, cnt = 0;
	int rev_req_vadp = 0, req_volt_valid_h = 0, req_volt_valid_l = 0;
	int ret = 0;

	if (real_type < XM_CHARGER_TYPE_DCP || real_type > XM_CHARGER_TYPE_PD_VERIFY)
		return -1;

	if (real_type == XM_CHARGER_TYPE_PD) {
		if (strategy_buckchg_wireless_revchg_msleep(WLS_REVCHG_NORMAL_INTERVAL, info))
			return -1;
	} else if (real_type == XM_CHARGER_TYPE_DCP) {
		if (strategy_buckchg_wireless_revchg_msleep(WLS_REVCHG_SLOW_INTERVAL, info))
			return -1;
	} else if (real_type == XM_CHARGER_TYPE_HVDCP3) {
		if (strategy_buckchg_wireless_revchg_msleep(WLS_REVCHG_FAST_INTERVAL, info))
			return -1;
	}

	real_type = info->proc_data.real_type;
	mca_vote(info->input_limit_voter, "wireless_revchg", true, CHARGE_WLS_REVCHG_INPUT_DEFAULT);
	if (real_type == XM_CHARGER_TYPE_PPS || real_type == XM_CHARGER_TYPE_PD_VERIFY) {
		rev_req_vadp = info->rev_req_vadp[0];
		req_volt_valid_h = info->rev_vadp_valid_h[0] * 1000;
		req_volt_valid_l = info->rev_vadp_valid_l[0] * 1000;
	} else {
		rev_req_vadp = info->rev_req_vadp[1];
		req_volt_valid_h = info->rev_vadp_valid_h[1] * 1000;
		req_volt_valid_l = info->rev_vadp_valid_l[1] * 1000;
	}
	strategy_buckchg_set_charge_volt(info, rev_req_vadp);

	if (strategy_buckchg_wireless_revchg_msleep(300, info))
		return -1;
	(void)platform_class_buckchg_ops_get_bus_volt(MAIN_BUCK_CHARGER, &vbus);
	mca_log_info("chg_type: %d, req_vadp: %d, vbus: %d, rev_vadp_valid_h: %d, rev_vadp_valid_l: %d\n",
		real_type, rev_req_vadp, vbus, req_volt_valid_h, req_volt_valid_l);
	while (vbus < req_volt_valid_l || vbus > req_volt_valid_h) {
		if (cnt > 5)
			break;
		if (strategy_buckchg_wireless_revchg_msleep(300, info))
			return -1;
		(void)platform_class_buckchg_ops_get_bus_volt(MAIN_BUCK_CHARGER, &vbus);
		cnt++;
	}
	if (cnt > 5) {
		mca_log_err("request 9V vbus fail\n");
		return -1;
	}

	info->proc_data.wls_revchg_init_done = true;
	ret = mca_wireless_rev_set_wired_chg_ok(true);
	return ret;
}

static void strategy_wls_revchg_monitor_workfunc(struct work_struct *work)
{
	struct strategy_buckchg_dev *info = container_of(work,
				struct strategy_buckchg_dev, wls_revchg_monitor_work.work);
	int ret = 0;
	int interval = CHARGE_MONITOR_WORK_NORMAL_INTERVAL;

	if (!info->proc_data.wls_revchg_init_done)
		ret = strategy_buckchg_process_wireless_revchg(info);
	if (ret || !info->wls_revchg_en)
		goto err_out;
	else {
		if (info->proc_data.real_type == XM_CHARGER_TYPE_HVDCP2 && !info->rev_icl_for_qc2) {
			mca_vote(info->input_limit_voter, "wireless_revchg", true, CHARGE_WLS_REVCHG_INPUT_QC2);
			info->rev_icl_for_qc2 = true;
		}
		schedule_delayed_work(&info->wls_revchg_monitor_work,
			msecs_to_jiffies(interval));
		return;
	}

err_out:
	strategy_buckchg_exit_wireless_revchg(info);
}

static void strategy_csd_pulse_process_workfunc(struct work_struct *work)
{
	struct strategy_buckchg_dev *info = container_of(work,
				struct strategy_buckchg_dev, csd_pulse_process_work.work);

	if (info->csd_flag) {
		mca_vote(info->charge_limit_voter, "csd_pulse", false, 0);
		mca_vote(info->chg_enable_voter, "csd_pulse", true, STATEGY_CHARGE_DISENABLE);
		info->csd_flag = false;
		schedule_delayed_work(&info->csd_pulse_process_work, msecs_to_jiffies(75000));
		mca_log_info("vote fcc and discharge logic\n");
	} else {
		mca_vote(info->chg_enable_voter, "csd_pulse", false, STATEGY_CHARGE_DISENABLE);
		mca_strategy_func_process(STRATEGY_FUNC_TYPE_QUICK_CHARGE,
			MCA_EVENT_CSD_SEND_PULSE, 0);
		mca_log_info("resume charging logic\n");
	}
}

/*
 * While the port is sourcing, the gear it sources at is revisited here.
 *
 * On a board that allows the quick gear with the screen on, the gear is
 * stepped down while the screen is lit; with the screen off the battery
 * current decides, and once it has stayed low for long enough the quick gear
 * is not earning its keep.  Rather than simply dropping it, userspace is asked
 * for a battery current limit -- it answers by writing revchg_bcl, and the
 * quick gear is kept while that stands.
 *
 * Stepping down out of the quick gear waits for the pump to actually stop
 * sourcing before telling userspace, so a case that is still drawing is not
 * told the charge has slowed.  The pump's ADC needs a moment after being
 * enabled before a reading means anything.
 */
static void strategy_source_status_monitor_workfunc(struct work_struct *work)
{
	struct strategy_buckchg_dev *info = container_of(work,
				struct strategy_buckchg_dev, source_status_monitor_work.work);
	char event[MCA_EVENT_NOTIFY_SIZE] = { 0 };
	struct mca_event_notify_data event_data = { 0 };
	int status = 0;
	int ibus = 0;
	int retry, ret, len;

	strategy_buckchg_check_reverse_quick_charge(info, &status);

	if (info->support_revchg_screenon) {
		if (info->screen_status) {
			if (status == REVCHG_GEAR_QUICK)
				status = REVCHG_GEAR_STEPDOWN;
			info->ibat_check_cnt = 0;
		} else if (status == REVCHG_GEAR_QUICK &&
			   info->ibat_check_cnt >= REVCHG_IBAT_CHECK_MAX) {
			if (info->revchg_bcl) {
				status = REVCHG_GEAR_QUICK;
			} else {
				len = snprintf(event, MCA_EVENT_NOTIFY_SIZE,
					       "POWER_SUPPLY_REVERSE_QUICK_CHARGE=%d",
					       REVCHG_UEVENT_REQUEST_BCL);
				event_data.event = event;
				event_data.event_len = len;
				mca_event_report_uevent(&event_data);
				mca_log_err("revchg request bcl\n");
				status = REVCHG_GEAR_STEPDOWN;
			}
		} else {
			if (status == REVCHG_GEAR_QUICK)
				status = REVCHG_GEAR_STEPDOWN;

			strategy_class_fg_ops_get_current(&info->proc_data.ibat);
			mca_log_info("source_status_monitor ibat: %d\n", info->proc_data.ibat);
			if (info->proc_data.ibat > REVCHG_IBAT_THRESHOLD)
				info->ibat_check_cnt = 0;
			else
				info->ibat_check_cnt++;
		}
	}

	if (status == info->source_boost_status)
		goto out;

	if (!info->support_revchg_screenon) {
		info->source_boost_status = status;
		mca_log_err("set gear shift: %d\n", status);
		protocol_class_pd_set_gear_shift(TYPEC_PORT_0, info->source_boost_status);
		goto out;
	}

	if (info->last_gear_shift != status) {
		mca_log_err("set gear shift: %d\n", status);
		protocol_class_pd_set_gear_shift(TYPEC_PORT_0, status);
		info->last_gear_shift = status;
	}

	if (status == REVCHG_GEAR_STEPDOWN &&
	    info->source_boost_status == REVCHG_GEAR_QUICK) {
		info->revchg_bcl = false;
		for (retry = REVCHG_IBUS_SETTLE_RETRY; retry > 0; retry--) {
			platform_class_cp_enable_adc(CP_ROLE_MASTER, true);
			mdelay(REVCHG_ADC_SETTLE_MS);
			ret = platform_class_cp_get_bus_current(CP_ROLE_MASTER, &ibus);
			mca_log_err("ret: %d, ibus: %d\n", ret, ibus);
			/*
			 * Still sourcing, or no reading at all: leave the
			 * standing status alone and look again next round.
			 */
			if (ret || ibus > REVCHG_IBUS_SETTLED)
				goto out;
		}

		len = snprintf(event, MCA_EVENT_NOTIFY_SIZE,
			       "POWER_SUPPLY_REVERSE_QUICK_CHARGE=%d", REVCHG_GEAR_STEPDOWN);
		event_data.event = event;
		event_data.event_len = len;
		mca_event_report_uevent(&event_data);
	}

	info->source_boost_status = status;

out:
	schedule_delayed_work(&info->source_status_monitor_work,
		(info->support_revchg_screenon && status) ?
			msecs_to_jiffies(SOURCE_STATUS_MONITOR_FAST_INTERVAL) :
			msecs_to_jiffies(SOURCE_STATUS_MONITOR_INTERVAL));
}

static void strategy_buckchg_check_pdsecret_workfunc(struct work_struct *work)
{
	struct strategy_buckchg_dev *info = container_of(work,
				struct strategy_buckchg_dev, check_pd_secret_work.work);

	if (!info->verify_process_end) {
		mca_vote(info->input_limit_voter, "icl_limit", false, STATEGY_INPUT_DEFAULT_VALUE);
		info->verify_process_end = 1;
		mca_log_info("cancel icl_limit vote and pd_end flag when using pd charger\n");
	}
}

#define NTC_SCALE_BOARDTEMP 100
static int strategy_buckchg_thermal_notifier_cb(struct notifier_block *nb,
			unsigned long event, void *val)
{
	struct strategy_buckchg_dev *info = container_of(nb,
				struct strategy_buckchg_dev, thermal_board_nb);

	switch (event) {
	case MCA_EVENT_THERMAL_BOARD_TEMP_CHANGE:
		info->thermal_board_temp  = *(int *)val/NTC_SCALE_BOARDTEMP;
		break;
	default:
		break;
	}
	return NOTIFY_DONE;
}

/* Whether the screen is on, which the charging limits are allowed to care about. */
static int strategy_buckchg_panel_notifier_cb(struct notifier_block *nb,
					      unsigned long event, void *val)
{
	struct strategy_buckchg_dev *info = container_of(nb,
				struct strategy_buckchg_dev, panel_nb);
	int state;

	if (event != MCA_EVENT_PANEL_SCREEN_STATE_CHANGE)
		return NOTIFY_DONE;

	state = *(int *)val;
	mca_log_info("update screen_state: %d => %d\n", info->screen_status,
		     !!state);
	info->screen_status = !!state;

	return NOTIFY_DONE;
}

static int strategy_buckchg_if_set_chg_cur(const char *user,
	char *value, void *data)
{
	int temp_value = 0;
	struct strategy_buckchg_dev *info = data;

	if (!user || !value || !data)
		return -1;

	if (kstrtoint(value, 0, &temp_value))
		return -1;

	if (temp_value)
		(void)mca_vote(info->charge_limit_voter, user, true, temp_value);
	else
		(void)mca_vote(info->charge_limit_voter, user, false, 0);

	return 0;
}

static int strategy_buckchg_if_get_chg_cur(char *buf, void *data)
{
	const char *client_str;
	int value;
	struct strategy_buckchg_dev *info = data;

	if (!buf || !data)
		return -1;

	client_str = mca_get_effective_client(info->charge_limit_voter);
	if (!client_str)
		return -1;
	value = mca_get_effective_result(info->charge_limit_voter);

	scnprintf(buf, MCA_CHARGE_IF_MAX_VALUE_BUFF, "eff_client:%s %d",
		client_str, value);

	return 0;
}


#define MTBF_CLIENT_SUBSTR_UPPER "MTBF\0"
#define MTBF_CLIENT_SUBSTR_LOW "mtbf\0"
#define MTBF_ACTIVE_TEST_CLIENT_SUBSTR "shell\0"
#define MTBF_TEST_MAX_CURRENT_MA 1500
static int strategy_buckchg_if_set_input_cur(const char *user,
	char *value, void *data)
{
	int temp_value = 0;
	struct strategy_buckchg_dev *info = data;

	if (!user || !value || !data)
		return -1;

	if (kstrtoint(value, 0, &temp_value))
		return -1;

	if (strstr(user, MTBF_ACTIVE_TEST_CLIENT_SUBSTR) != NULL) {
			if (temp_value > MTBF_TEST_MAX_CURRENT_MA)
				temp_value = MTBF_TEST_MAX_CURRENT_MA;
	}

	if (temp_value) {
		if (strstr(user, MTBF_CLIENT_SUBSTR_UPPER) != NULL ||
			strstr(user, MTBF_CLIENT_SUBSTR_LOW) != NULL ||
			strstr(user, MTBF_ACTIVE_TEST_CLIENT_SUBSTR) != NULL)
			mca_vote_override(info->input_limit_voter, user, true, temp_value);
		else
			(void)mca_vote(info->input_limit_voter, user, true, temp_value);
	} else {
		if (strstr(user, MTBF_CLIENT_SUBSTR_UPPER) != NULL || strstr(user, MTBF_CLIENT_SUBSTR_LOW) != NULL)
			mca_vote_override(info->input_limit_voter, user, false, 0);
		else
			(void)mca_vote(info->input_limit_voter, user, false, 0);
	}

	return 0;
}

static int strategy_buckchg_if_get_input_cur(char *buf, void *data)
{
	const char *client_str;
	int value;
	struct strategy_buckchg_dev *info = data;

	if (!buf || !data)
		return -1;

	client_str = mca_get_effective_client(info->input_limit_voter);
	if (!client_str)
		return -1;
	value = mca_get_effective_result(info->input_limit_voter);

	scnprintf(buf, MCA_CHARGE_IF_MAX_VALUE_BUFF, "eff_client:%s %d",
		client_str, value);

	return 0;
}

static int strategy_buckchg_if_set_chg_en(const char *user,
	unsigned int value, void *data)
{
	struct strategy_buckchg_dev *info = data;

	if (!user || !data)
		return -1;

	if (value) {
		strategy_buckchg_clear_pmic_temp_term(info);
		(void)mca_vote(info->chg_enable_voter, user,
			true, STATEGY_CHARGE_ENABLE);
	} else {
		(void)mca_vote(info->chg_enable_voter, user,
			true, STATEGY_CHARGE_DISENABLE);
	}

	return 0;
}

static int strategy_buckchg_if_get_chg_en(char *buf, void *data)
{
	const char *client_str;
	int value;
	struct strategy_buckchg_dev *info = data;

	if (!buf || !data)
		return -1;

	client_str = mca_get_effective_client(info->chg_enable_voter);
	if (!client_str)
		return -1;
	value = mca_get_effective_result(info->chg_enable_voter);

	scnprintf(buf, MCA_CHARGE_IF_MAX_VALUE_BUFF, "eff_client:%s %d",
		client_str, value);

	return 0;
}

static int strategy_buckchg_if_get_input_suspend(char *buf, void *data)
{
	const char *client_str;
	int value;
	struct strategy_buckchg_dev *info = data;

	if (!buf || !data)
		return -1;

	client_str = mca_get_effective_client(info->input_suppend_voter);
	if (!client_str)
		return -1;
	value = mca_get_effective_result(info->input_suppend_voter);

	scnprintf(buf, MCA_CHARGE_IF_MAX_VALUE_BUFF, "eff_client:%s %d",
		client_str, value);

	return 0;
}

static int strategy_buckchg_if_set_input_suspend(const char *user,
	char *value, void *data)
{
	int temp_value = 0;
	struct strategy_buckchg_dev *info = data;

	if (!user || !value || !data)
		return -1;

	if (kstrtoint(value, 0, &temp_value))
		return -1;

	mca_log_err("set_input_suspend: %d\n", temp_value);
	if (temp_value) {
		(void)mca_vote(info->input_suppend_voter, user, true, 1);
	} else {
		strategy_buckchg_clear_pmic_temp_term(info);
		(void)mca_vote(info->input_suppend_voter, user, true, 0);
	}

	return 0;
}

static int strategy_buckchg_if_set_ship_mode(const char *user,
	unsigned int val, void *data)
{
	int rc;
	struct strategy_buckchg_dev *info = data;
	mca_log_err("set shipmode chip:%d val:%d\n", info->ship_mode_chip, val);

	rc = platform_class_buckchg_ops_set_ship_mode(info->ship_mode_chip, !!val);
	if (rc < 0)
		return rc;

	return 0;
}

static int strategy_buckchg_if_get_ship_mode(bool *val, void *data)
{
	int rc;
	struct strategy_buckchg_dev *info = data;

	rc = platform_class_buckchg_ops_get_ship_mode(info->ship_mode_chip, val);
	if (rc < 0)
		return rc;

	return 0;
}

static struct mca_charge_if_ops g_strategy_buckchg_if_ops = {
	.type_name = "buck",
	.set_input_suspend = strategy_buckchg_if_set_input_suspend,
	.get_input_suspend = strategy_buckchg_if_get_input_suspend,
	.set_charge_enable = strategy_buckchg_if_set_chg_en,
	.get_charge_enable = strategy_buckchg_if_get_chg_en,
	.set_input_current_limit = strategy_buckchg_if_set_input_cur,
	.get_input_current_limit = strategy_buckchg_if_get_input_cur,
	.set_charge_current_limit = strategy_buckchg_if_set_chg_cur,
	.get_charge_current_limit = strategy_buckchg_if_get_chg_cur,
	.set_ship_mode_en = strategy_buckchg_if_set_ship_mode,
	.get_ship_mode_status = strategy_buckchg_if_get_ship_mode,
};

/*
 * Userspace can cap how far the pack is charged.  A board that supports
 * charging past the cap steps the current down towards it instead of stopping
 * dead; every other board simply stops, drops the input back to 5 V, and
 * starts again when the cap is lifted.
 */
static int strategy_buckchg_soc_limit_sts_callback(void *data, int effective_result)
{
	struct strategy_buckchg_dev *info = (struct strategy_buckchg_dev *)data;

	if (!data)
		return -1;

	info->soc_limit_sts = effective_result;
	mca_log_info("effective_result: %d\n", effective_result);

	if (info->support_charge_more) {
		if (effective_result && info->proc_data.online) {
			strategy_buckchg_process_soc_limit_change_more(effective_result, info);
		} else {
			cancel_delayed_work_sync(&info->soc_limit_stepper_work);
			strategy_buckchg_process_soc_limit_change_more(0, info);
		}
		return 0;
	}

	if (effective_result && info->proc_data.online) {
		mca_vote(info->chg_enable_voter, "soc_limit", true, STATEGY_CHARGE_DISENABLE);
		mca_vote(info->input_voltage_voter, "soc_limit", true, 0);
		mca_log_info("SOC limit triggered, stopping charge\n");
		/* A pulse that was asked for before the cap is no longer wanted. */
		mca_vote(info->charge_limit_voter, "csd_pulse", false, 0);
	} else {
		mca_vote(info->chg_enable_voter, "soc_limit", false, STATEGY_CHARGE_ENABLE);
		mca_vote(info->input_voltage_voter, "soc_limit", false, 0);
		mca_log_info("SOC limit released, starting charge\n");
		if (info->proc_data.online)
			mod_delayed_work(system_wq, &info->monitor_work, 0);
	}

	return 0;
}

static int mca_charger_buckchg_pwr_boost_sts_callback(void *data, int enable)
{
	struct strategy_buckchg_dev *info = (struct strategy_buckchg_dev *)data;

	if (!data)
		return -1;

	info->smartchg_data.pwr_boost_state = enable;

	return 0;
}

static struct mca_smartchg_if_ops g_buck_smartchg_if_ops = {
	.type = MCA_SMARTCHG_IF_CHG_TYPE_BUCK,
	.data = NULL,
	.set_fcc = strategy_buckchg_smartchg_set_fcc_callback,
	.set_soc_limit_sts = strategy_buckchg_soc_limit_sts_callback,
	.set_pwr_boost_sts = mca_charger_buckchg_pwr_boost_sts_callback,
};

static int strategy_buckchg_class_probe(struct platform_device *pdev)
{
	struct strategy_buckchg_dev *info;
	int online;
	int ret;

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = &pdev->dev;
	strategy_buckchg_parse_dt(info);
	ret = strategy_buckchg_init_voter(info);
	if (ret) {
		mca_log_err("init voter err\n");
		return -1;
	}

	mca_vote(info->input_voltage_voter, "batt_auth", true, 0);
	mca_vote(info->charge_limit_voter, "batt_auth", true, info->chg_batt_auth_failed);
	/*
	 * Hold the float voltage down until a battery-present event proves
	 * there is a pack to charge.  Without this the voter starts at its
	 * registration default, which is the full JEITA float voltage.
	 */
	mca_vote(info->vterm_voter, "batt_miss", true, STAEGY_BATT_MISS_FV);
	info->proc_data.voltage = STATEGY_CHARGE_VBUS_5V;
	info->hvdcp_allow_flag = 0;
	info->vbat_ov_count = 0;
	INIT_DELAYED_WORK(&info->monitor_work, strategy_buckchg_monitor_workfunc);
	INIT_DELAYED_WORK(&info->soc_limit_stepper_work,
			  strategy_buckchg_soc_limit_stepper_workfunc);
	INIT_DELAYED_WORK(&info->rerun_handle_pd_auth_work,
			  strategy_rerun_handle_pd_auth_workfunc);
	INIT_DELAYED_WORK(&info->sw_cv_work, strategy_buckchg_sw_cv_workfunc);
	INIT_DELAYED_WORK(&info->base_flip_sw_cv_work, strategy_buckchg_base_flip_sw_cv_workfunc);
	INIT_DELAYED_WORK(&info->wls_revchg_monitor_work, strategy_wls_revchg_monitor_workfunc);
	INIT_DELAYED_WORK(&info->csd_pulse_process_work, strategy_csd_pulse_process_workfunc);
	INIT_DELAYED_WORK(&info->source_status_monitor_work, strategy_source_status_monitor_workfunc);
	INIT_DELAYED_WORK(&info->check_pd_secret_work, strategy_buckchg_check_pdsecret_workfunc);
	(void)mca_strategy_ops_register(STRATEGY_FUNC_TYPE_BUCK_CHARGE,
		strategy_buckchg_process_event, strategy_buckchg_get_status, strategy_buckchg_set_config, info);
	g_strategy_buckchg_if_ops.data = info;
	(void)mca_charge_if_ops_register(&g_strategy_buckchg_if_ops);
	g_buck_smartchg_if_ops.data = info;
	(void)mca_smartchg_if_ops_register(&g_buck_smartchg_if_ops);
	info->thermal_board_nb.notifier_call = strategy_buckchg_thermal_notifier_cb;
	mca_event_block_notify_register(MCA_EVENT_TYPE_THERMAL_TEMP, &info->thermal_board_nb);
	info->panel_nb.notifier_call = strategy_buckchg_panel_notifier_cb;
	mca_event_block_notify_register(MCA_EVENT_TYPE_PANEL, &info->panel_nb);

	platform_class_buckchg_ops_get_online(MAIN_BUCK_CHARGER, &online);
	if (online) {
		mca_log_info("avoid missing first usb connect event\n");
		mca_event_block_notify(MCA_EVENT_TYPE_CHARGER_CONNECT, MCA_EVENT_USB_CONNECT, NULL);
	}

	g_buckchg_info = info;
	info->init_ok = 1;
	mca_log_err("androidboot.mode=%d\n", mca_log_get_charge_boot_mode());

	mca_log_err("probe success\n");
	return 0;
}

static int strategy_buckchg_class_remove(struct platform_device *pdev)
{
	struct strategy_buckchg_dev *info = g_buckchg_info;

	if (!info)
		return 0;

	/*
	 * Both chains hold a notifier block that lives inside info, which is
	 * devm memory and goes away once this returns.  Drop them before the
	 * works, so nothing is queued behind the cancels.
	 */
	mca_event_block_notify_unregister(MCA_EVENT_TYPE_THERMAL_TEMP,
					  &info->thermal_board_nb);
	mca_event_block_notify_unregister(MCA_EVENT_TYPE_PANEL,
					  &info->panel_nb);

	cancel_delayed_work_sync(&info->monitor_work);
	cancel_delayed_work_sync(&info->wls_revchg_monitor_work);
	cancel_delayed_work_sync(&info->check_pd_secret_work);
	cancel_delayed_work_sync(&info->rerun_handle_pd_auth_work);
	cancel_delayed_work_sync(&info->csd_pulse_process_work);
	cancel_delayed_work_sync(&info->source_status_monitor_work);
	cancel_delayed_work_sync(&info->sw_cv_work);
	cancel_delayed_work_sync(&info->base_flip_sw_cv_work);
	cancel_delayed_work_sync(&info->soc_limit_stepper_work);

	g_buckchg_info = NULL;

	return 0;
}

static void strategy_buckchg_class_shutdown(struct platform_device *pdev)
{
}

static const struct of_device_id match_table[] = {
	{.compatible = "mca,strategy_buckchg"},
	{},
};

static struct platform_driver strategy_buckchg_class_driver = {
	.driver	= {
		.name = "strategy_buckchg_class",
		.owner = THIS_MODULE,
		.of_match_table = match_table,
	},
	.probe = strategy_buckchg_class_probe,
	.remove = strategy_buckchg_class_remove,
	.shutdown = strategy_buckchg_class_shutdown,
};

static int __init strategy_buckchg_class_init(void)
{
	return platform_driver_register(&strategy_buckchg_class_driver);
}
module_init(strategy_buckchg_class_init);

static void __exit strategy_buckchg_class_exit(void)
{
	platform_driver_unregister(&strategy_buckchg_class_driver);
}
module_exit(strategy_buckchg_class_exit);


MODULE_DESCRIPTION("strategy buckchg class");
MODULE_AUTHOR("liyuze1@xiaomi.com");
MODULE_LICENSE("GPL");
