/* SPDX-License-Identifier: GPL-2.0 */
/*
 * mca_strategy_buckchg.h
 *
 * mca buck charger strategy driver
 *
 * Copyright (c) 2024-2024 Xiaomi Technologies Co., Ltd.
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

#ifndef __MCA_STRATEGY_BUCKCHG_H__
#define __MCA_STRATEGY_BUCKCHG_H__

#define STATEGY_INPUT_DEFAULT_VALUE 100
#define STATEGY_INPUT_5V_DEFAULT_VALUE 2000
#define STATEGY_INPUT_9V_DEFAULT_VALUE 1600
#define STATEGY_CHARGE_CURRENT_DEFAULT_VALUE 4000
#define STATEGY_ITERM_DEFAULT_VALUE 500
#define STATEGY_VTERM_DEFAULT_VALUE 4400
#define STATEGY_SUPPORT_MULTI_BUCK 0
#define STATEGY_CHARGE_ENABLE 1
#define STATEGY_CHARGE_DISENABLE 0
#define STATEGY_CHARGE_INPUT_SUSPEND 0
#define STATEGY_CHARGE_INPUT_VOLT_DEFAULT 4600
#define STATEGY_CHARGE_FWS_DEFAULT 5000
#define STATEGY_CHARGE_AICL_VBAT_TH 4100
#define STATEGY_CHARGE_AICL_VBAT_TH_4P2V 4200
#define STATEGY_CHARGE_AICL_TH_4P1V 4100
#define STATEGY_CHARGE_AICL_TH_4P5V 4500
#define STATEGY_CHARGE_AICL_TH_4P4V 4400
#define STATEGY_CHARGE_VBUS_5V 5000
#define STATEGY_CHARGE_VBUS_9V 9000
/* Above this the bus is high enough to be why the charger will not start. */
#define STATEGY_CHARGE_VBUS_ABNORMAL_UV 6000000
#define STATEGY_CHARGE_VBUS_12V 12000
#define STAEGY_BATT_MISS_ICL 500
#define STAEGY_BATT_MISS_ICL_PARALLEL 1000
/* Where the pump's bus over-voltage sits once the pack is discharging. */
#define BUCKCHG_DISCHARGE_CP_BUSOVP_MV 13000
/* And what it is held to while a float charger is on the port. */
#define BUCKCHG_FLOAT_CP_BUSOVP_MV 7000
/* Where a pack sharing one reading counts as hot, in whole degrees. */
#define BUCKCHG_WARM_PACK_TEMP_C 33
#define BUCKCHG_HOT_PACK_TEMP_C 34

/* How the pack is put together, as the device tree numbers it. */
enum mca_buckchg_battery_type {
	MCA_BATTERY_TYPE_SINGLE = 0,
	MCA_BATTERY_TYPE_PARALLEL,
	MCA_BATTERY_TYPE_SERIES,
};
#define STAEGY_BATT_MISS_FV 4200
#define STAEGY_CHARGE_PLATE_SHOCK 600
#define STATEGY_CHARGE_VTERM_LOW_TH 4300

#define STATEGY_USBIN_WLS_REV_CHG_VOLTAGE_DEFAULT 9000
#define STATEGY_USBIN_WLS_REV_CHG_VOLTAGE_BUF_H 9200
#define STATEGY_USBIN_WLS_REV_CHG_VOLTAGE_BUF_L 8400

#define CHARGE_PPS_INPUT_DEFAULT 3000
#define CHARGE_DCP_INPUT_DEFAULT 1500
#define CHARGE_DCP_INPUT_BOOST 2000
#define CHARGE_CDP_INPUT_DEFAULT 1500
#define CHARGE_SDP_INPUT_DEFAULT 500
#define CHARGE_FLOAT_INPUT_DEFAULT 1000
#define CHARGE_PPS_PTF_INPUT_DEFAULT 1000
#define CHARGE_PPS_PTF_CHARGE_DEFAULT 1100
#define CHARGE_DCP_CHARGE_DEFAULT 2000
#define CHARGE_DCP_CHARGE_BOOST 2000
#define CHARGE_CDP_CHARGE_DEFAULT 1500
#define CHARGE_SDP_CHARGE_DEFAULT 500
#define CHARGE_FLOAT_CHARGE_DEFAULT 500
#define CHARGE_BATT_AUTH_FAIL_DEFAULT 2000
#define CHARGE_WLS_REVCHG_INPUT_DEFAULT 1000
#define CHARGE_WLS_REVCHG_INPUT_QC2 500
#define CHARGE_MONITOR_WORK_NORMAL_INTERVAL 10000
#define CHARGE_MONITOR_WORK_FAST_INTERVAL 5000
#define WLS_REVCHG_FAST_INTERVAL 3000
#define WLS_REVCHG_NORMAL_INTERVAL 4000
#define WLS_REVCHG_SLOW_INTERVAL 6000
#define SOURCE_STATUS_MONITOR_INTERVAL 5000
/* How often to look while a gear other than off is in force. */
#define SOURCE_STATUS_MONITOR_FAST_INTERVAL 1000

/*
 * The gears the port can be asked to source at.  0 is the plain reverse
 * charge; 1 is the quick one; 2 steps back down from it without giving up.
 */
#define REVCHG_GEAR_NORMAL	0
#define REVCHG_GEAR_QUICK	1
#define REVCHG_GEAR_STEPDOWN	2
/* Not a gear: what the uevent asks userspace for. */
#define REVCHG_UEVENT_REQUEST_BCL	3

/* Battery current, in uA, above which the quick gear is still earning its keep. */
#define REVCHG_IBAT_THRESHOLD	2999999
/* How many rounds under that before the quick gear is given up. */
#define REVCHG_IBAT_CHECK_MAX	2
/* Bus current, in mA, that says the pump has actually stopped sourcing. */
#define REVCHG_IBUS_SETTLED	1500
/* How many times to look for it before reporting the step down anyway. */
#define REVCHG_IBUS_SETTLE_RETRY	5
/* How long the pump needs after its ADC is enabled before a reading means anything. */
#define REVCHG_ADC_SETTLE_MS	20
#define CHARGE_SW_CV_WORK_NORMAL_INTERVAL 5000
#define CHARGE_SW_CV_WORK_FAST_INTERVAL 2000
#define CHARGE_SW_CV_VBAT_ALARM_DELTA 10

#define ALLOW_QUICK_CHG_SOC_THR    90

/* How the two halves of a debug_ctrl soc_limit share one int. */
#define DEBUG_CTRL_SOC_SHIFT	8
#define DEBUG_CTRL_SOC_MASK	0xff

/* The band the memory test borrows. */
#define MEMORY_TEST_SOC_LIMIT_LOW	1
#define MEMORY_TEST_SOC_LIMIT_HIGH	3
#define ALLOW_START_FFC_BATT_SOC_THR	95
/* Where the port's over-voltage protection sits. */
#define BUCKCHG_VUSB_OVP_BEFORE_CP	1
/* Passes to wait for a silent charge pump before giving up on it. */
#define BUCKCHG_CP_ABSENT_RETRY_MAX	6
#define ALLOW_FFC_TEMP_LOW_THR    20
#define ALLOW_FFC_TEMP_HIGH_THR    48
#define CHARGE_BATT_USE_SC6601A_BUCK 0
#define PMIC_ITERM_COMPENSATION_DEFAULT 30
#define BAT_TEMP_FV_COMP_HOT_TH_DEFAULT 50
/* How long after boot a pack started from zero still needs an input floor. */
#define BUCKCHG_ZERO_SPEED_BOOT_WINDOW_SEC 60
#define BUCKCHG_ZERO_SPEED_INPUT_MIN 1500

#define SW_CV_FCC_STEP_DEFAULT 50
#define SW_CV_FV_STEP_DEFAULT 5

/* The headroom a flip base wants, in mV, when it is and is not fast charging. */
#define BASE_FLIP_FV_COMP_SLOW		10
#define BASE_FLIP_FV_COMP_FAST		20

/* The three temperature bands, in degrees C, the dynamic headroom picks from. */
#define FV_COMP_WARM_LOW		18
#define FV_COMP_WARM_HIGH		29
#define FV_COMP_MIDDLE_LOW		30
#define FV_COMP_MIDDLE_HIGH		39
#define FV_COMP_HIGH_LOW		40
#define FV_COMP_HIGH_HIGH		47

/* A base carrying the phone's own pack wants extra headroom when hot. */
#define BASE_FLIP_SAME_HOT_LOW		48
#define BASE_FLIP_SAME_HOT_HIGH		54
#define BASE_FLIP_SAME_HOT_FV_COMP	30

/*
 * Which flip base is attached is told apart by the termination current the
 * packs were asked to share, and each wants its own current headroom.
 */
#define BASE_FLIP_ITERM_SLOW		187
#define BASE_FLIP_ITERM_COMP_SLOW	20
#define BASE_FLIP_ITERM_COMP_SLOW_OTHER	14
#define BASE_FLIP_ITERM_HOT		1046
#define BASE_FLIP_ITERM_COMP_HOT	104
#define BASE_FLIP_ITERM_WARM		598
#define BASE_FLIP_ITERM_COMP_WARM	82
#define BASE_FLIP_ITERM_COMP_OTHER	50
#define ITERM_COMP_HOT_LOW		30
#define ITERM_COMP_HOT_HIGH		39
#define ITERM_COMP_WARM_LOW		18
#define ITERM_COMP_WARM_HIGH		29

/* A base carrying the phone's own pack picks its headroom off one threshold. */
#define BASE_FLIP_SAME_ITERM_COMP_SLOW	20
#define BASE_FLIP_SAME_ITERM_TEMP_TH	35
#define BASE_FLIP_SAME_ITERM_COMP_COOL	75
#define BASE_FLIP_SAME_ITERM_COMP_WARM	68
#define VBAT_FG_TO_PMIC_RATIO_DEFAULT 1
#define VOTE_BUCK_VTERM_BUF_DEFAULT 0
#define VOTE_BUCK_ITERM_BUF_DEFAULT 0
#define DEFAULT_SUPPORT_HW_BC12 0
#define MCA_WIRE_CHARGE_DEFAULT_IBUS_CURRENT    500
#define MCA_WIRE_CHARGE_DEFAULT_IBAT_CURRENT    500
#define FULL_REPLUG_LIMIT_RAWSOC_TH 9900
#define BUCKCHG_OK_TO_HIGH_IBAT_RAWSOC_TH 9600

enum usbin_wlsrevchg_type {
	REV_USBIN_TYPE_PPS = 0,
	REV_USBIN_TYPE_OTHER,
	REV_USBIN_TYPE_MAX,
};

enum mca_buck_usbpd_dpm_port_pps_ptf_type {
    USBPD_BUCK_DPM_PORT_PPS_PTF_NOT_SUPPORTED = 0,
    USBPD_BUCK_DPM_PORT_PPS_PTF_NOT_NORMAL,
    USBPD_BUCK_DPM_PORT_PPS_PTF_NOT_WARNING,
    USBPD_BUCK_DPM_PORT_PPS_PTF_NOT_OVERTEMP,
};

struct strategy_buckchg_proc_data {
	int online;
	int qc_type;
	int real_type;
	int chg_en;
	int chg_status;
	int ibus_limit;
	int ibat_limit;
	int voltage;
	/* The voltage the limits above were last voted for. */
	int pre_volt;
	int vbus;
	int ibus;
	int vbat;
	int ibat;
	int curr_fv;
	int curr_pd_pos;
	bool wls_revchg_init_done;
	bool pdsuspendsupported;
	bool charge_done_force_5v;
	bool is_pd_9v;
	ktime_t eu_start_time;
	int num_pwr_caps;
};

struct mca_smartchg_data {
       int pwr_boost_state;
};

struct strategy_buckchg_dev {
	struct device *dev;
	/* dt config */
	int support_multi_buck;
	int ship_mode_chip;
	unsigned int in_dcp;
	unsigned int in_pd;
	unsigned int in_pps;
	unsigned int in_hvdcp;
	unsigned int in_hvdcp3;
	unsigned int in_hvdcp3p5;
	unsigned int in_cdp;
	unsigned int in_sdp;
	unsigned int in_float;
	unsigned int chg_dcp;
	unsigned int chg_pd;
	unsigned int chg_pps;
	unsigned int chg_hvdcp;
	unsigned int chg_hvdcp3;
	unsigned int chg_hvdcp3p5;
	unsigned int chg_cdp;
	unsigned int chg_sdp;
	unsigned int chg_float;
	unsigned int chg_batt_auth_failed;
	unsigned int ffc_temp_low;
	unsigned int ffc_temp_high;
	int pmic_fv_compensation;
	/*
	 * How far the charger's own termination current sits above the one
	 * voted for, to cover the difference between where the gauge measures
	 * and where the charger does.
	 */
	int pmic_iterm_compensation;
	/*
	 * Where the board carries a second pack -- a flip base -- the headroom
	 * is that pack's rather than the phone's.  base_flip_same says the
	 * base carries the same pack the phone does.
	 */
	/* Set once probe has finished, so callers know it is safe to ask. */
	int init_ok;
	/*
	 * Whether this board steps the charge current down towards the SOC cap
	 * rather than stopping dead at it.
	 */
	int support_charge_more;
	int support_base_flip;
	int base_flip_same;
	/*
	 * Set where the charge pump hands the pack back to the buck charger
	 * near the top of the charge rather than terminating itself.
	 */
	int need_cp_to_pmic;
	int pmic_wls_fv_compensation;
	/* A colder or hotter pack wants a different voltage headroom. */
	int support_diff_temp_comp;
	int bat_temp_fv_comp_cold_th;
	int pmic_fv_compensation_cold;
	int bat_temp_fv_comp_hot_th;
	int pmic_fv_compensation_hot;
	/* Or, where the board asks for it, one of three temperature bands. */
	/* How many cells the pack is built from. */
	int batt_type;
	int support_pmic_vterm_dynamics_adjust;
	/* Whether the temperature override has already been lifted. */
	bool pmic_temp_term_flag;
	/* How many times the buck has been found not charging when it should. */
	int buck_abnormal_cnt;
	/* Charge level above which fast charge is no longer started. */
	int allow_start_ffc_batt_soc_thr;
	/* Slack added to the termination current before fast charge stops. */
	int curr_term_compensation;
	/* Set while the charge pump, not the buck charger, ends the charge. */
	int terminated_by_cp;
	/*
	 * Where the port's over-voltage protection sits relative to the charge
	 * pump.  BEFORE_CP means a pump that has stopped answering leaves the
	 * port unprotected above five volts.
	 */
	int vusb_ovp_location;
	int cp_absent_retry_count;
	bool cp_i2c_error_reported;
	int pmic_middle_fv_compensation;
	int pmic_high_fv_compensation;
	/* The termination current the packs were last told to share. */
	int parallel_iterm;
	/*
	 * How big a step the software constant-voltage phase takes when it
	 * pulls the charge current, and the charger's own target voltage,
	 * back.
	 */
	int sw_cv_fcc_step;
	int sw_cv_fv_step;
	int support_reverse_quick_charge;
	int rev_req_vadp[REV_USBIN_TYPE_MAX];
	int rev_vadp_valid_h[REV_USBIN_TYPE_MAX];
	int rev_vadp_valid_l[REV_USBIN_TYPE_MAX];
	/* voter */
	struct mca_votable *chg_enable_voter;
	struct mca_votable *input_suppend_voter;
	struct mca_votable *input_voltage_voter;
	struct mca_votable *input_limit_voter;
	struct mca_votable *charge_limit_voter;
	struct mca_votable *iterm_voter;
	struct mca_votable *vterm_voter;
	struct mca_votable *buck_5v_in_voter;
	struct mca_votable *buck_5v_ich_voter;
	struct mca_votable *buck_9v_in_voter;
	struct mca_votable *buck_9v_ich_voter;
	int buck_5v_in;
	int buck_5v_ich;
	int buck_9v_in;
	int buck_9v_ich;
	int hvdcp_allow_flag;
	int vbat_ov_count;
	/* proc */
	struct mca_smartchg_data smartchg_data;
	struct strategy_buckchg_proc_data proc_data;
	struct adapter_power_cap_info pwr_cap;
	struct delayed_work monitor_work;
	struct delayed_work soc_limit_stepper_work;
	struct delayed_work rerun_handle_pd_auth_work;
	/* The level cap as it last stood, so the stepper knows which way to go. */
	int soc_limit_sts;
	/*
	 * The band the debug_ctrl soc_limit knob asks for: charging stops at
	 * or above soc_limit_high and resumes at or below soc_limit_low.  A
	 * low that is not below the high, or either below 1, means the knob
	 * is off.
	 */
	int soc_limit_low;
	int soc_limit_high;
	struct delayed_work sw_cv_work;
	/*
	 * The sw_cv variant for flip-cover bases. As in the blob it is only
	 * created and cancelled; nothing queues it on a board without one.
	 */
	struct delayed_work base_flip_sw_cv_work;
	struct delayed_work wls_revchg_monitor_work;
	struct delayed_work csd_pulse_process_work;
	struct delayed_work source_status_monitor_work;
	struct delayed_work check_pd_secret_work;
	struct notifier_block thermal_board_nb;
	struct notifier_block panel_nb;
	/* Whether the screen is on, as the panel last reported it. */
	bool screen_status;

	int thermal_board_temp;
	int source_boost_status;
	/*
	 * Whether this board lets reverse quick charge keep running with the
	 * screen on.  Where it does, the gear is lowered while the screen is
	 * lit and the battery current is watched instead.
	 */
	int support_revchg_screenon;
	/* How many rounds the battery current has stayed under the threshold. */
	int ibat_check_cnt;
	/* The gear the port was last told to use, so it is not told twice. */
	int last_gear_shift;
	/* Reverse quick charge: fast-charging another device over the port. */
	int otg_boost_src;
	int cp_vendor;
	bool reverse_auth_sts;
	bool wls_online;
	bool wls_revchg_online;
	bool start_quick_revchg;
	bool revchg_bcl;
	int force_stop;
	int quick_charge_status;
	int aicl_thd;
	int wls_revchg_en;
	int verify_process_end;
	int batt_auth;
	int pdo_nums;
	bool rev_icl_for_qc2;
	bool csd_flag;
	int pps_ptf;
	bool is_eu_model;
	bool cp_revert_auth;
	bool sw_cv_running;
	bool is_non_compliant_qc;
	bool non_compliant_run_once;
	unsigned int use_sc_buck;
	int hw_bc12;
	int vbat_fg_to_pmic_ratio;
	int vote_buck_vterm_buf;
	int vote_buck_iterm_buf;
	int sw_cv_vterm_th;
	int full_replug_ichg_limit;
	bool dpdm_detect_done;
};

#endif /*__MCA_STRATEGY_BUCKCHG_H__ */
