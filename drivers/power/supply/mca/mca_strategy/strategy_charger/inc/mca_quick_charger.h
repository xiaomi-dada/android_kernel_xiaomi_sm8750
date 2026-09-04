/* SPDX-License-Identifier: GPL-2.0 */
/*
 *quick_charger.h
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
#ifndef __MCA_QUICK_CHARGE_H__
#define __MCA_QUICK_CHARGE_H__

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/version.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 9))
#include <linux/usb/pd.h>
#else
#include <linux/usb/usbpd.h>
#endif
#include <linux/of.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/poll.h>

#include <mca/protocol/protocol_class.h>
#include <mca/strategy/strategy_class.h>
#include <mca/common/mca_voter.h>

/* default value*/
#define MCA_QUICK_CHG_NAME_LEN 16
#define MCA_QUICK_CHG_MAX_BUFF_LEN 256
#define MCA_QUICK_CHG_MIN_VBAT_DEFAULT 3300
#define MCA_QUICK_CHG_MAX_VBAT_DEFAULT 4400
#define MCA_QUICK_CHARGE_RECHARGE_VBAT_DEFAULT 4300

/* What a QC adapter is taken to be good for, and the bus band that beats it. */
#define QC_VOLTAGE_MAX_DEFAULT_MV	9000
#define QC_CURRENT_MAX_DEFAULT_MA	3000
#define QC_CURRENT_MAX_HIGH_MA		5000
#define QC_CURRENT_MAX_HIGH_VBUS_LOW	8400
#define QC_CURRENT_MAX_HIGH_VBUS_HIGH	9900
#define MCA_QUICK_CHARGE_MAX_VBAT_THRESHOLD_DEFAULT 5000
#define MCA_QUICK_CHG_RECHARGE_VBAT_DELTA_DEFAULT 250
#define MCA_QUICK_CHG_MAX_TDIE_DEFAULT 125
#define MCA_QUICK_CHG_MAX_TADP_DEFAULT 125
#define MCA_QUICK_CHG_DIV1_VOLT_DELTA_DEFAULT 300
#define MCA_QUICK_CHG_DIV2_VOLT_DELTA_DEFAULT 300
#define MCA_QUICK_CHG_DIV4_VOLT_DELTA_DEFAULT 500
#define MCA_QUICK_CHG_SINGLE_DIV4_CURR_TH 14000
#define MCA_QUICK_CHG_SINGLE_DIV4_CURR_EN_BUCK 19000
#define MCA_QUICK_CHG_SINGLE_DIV2_CURR_TH 12000
#define MCA_QUICK_CHG_SINGLE_DIV1_CURR_TH 5000
#define MCA_QUICK_CHG_BUCK_ICL_CURR_TH 0
#define MCA_QUICK_CHG_BUCK_FCC_CURR_TH 0
#define MCA_QUICK_CHG_PPS_BOOST_FCC_CURR_TH 11000
#define MCA_QUICK_CHG_IBUS_TH_DEFAULT 1500
#define MCA_QUICK_CHG_AGAIN_IBUS_TH_DEFAULT 2800
#define MCA_QUICK_CHG_IBUS_INC_HYS_DEFAULT 200
#define MCA_QUICK_CHG_IBUS_DEC_HYS_DEFAULT 200
#define MCA_QUICK_CHG_DIV4_VOLT_TH_HIGH 18000
#define MCA_QUICK_CHG_DIV4_VOLT_TH_LOW 12000
#define MCA_QUICK_CHG_DIV2_VOLT_TH_HIGH 9000
#define MCA_QUICK_CHG_DIV2_VOLT_TH_LOW 6000
#define MCA_QUICK_CHG_DIV1_VOLT_TH_HIGH 4500
#define MCA_QUICK_CHG_DIV1_VOLT_TH_LOW 3600
#define MCA_QUICK_CHG_DIV4_MAX_VOLT 21000
#define MCA_QUICK_CHG_DIV2_MAX_VOLT 11000
#define MCA_QUICK_CHG_DIV1_MAX_VOLT 5500
#define MCA_QUICK_CHG_VBAT_MAX 5000
#define MCA_QUICK_CHG_VBAT_SERIES_MAX 10000
#define MCA_QUICK_CHG_DEFAULT_IBAT_DELTA 200
#define MCA_QUICK_CHG_ADP_DEFAULT_VOLT 5000
#define MCA_QUICK_CHG_VBUS_OK_HIGH_TH 12000
/* Below this the division the pump runs at is not worth choosing. */
#define MCA_QUICK_CHG_MODE_SWITCH_POWER_MIN 65
/* Above this the pump is stopped rather than the target moved. */
#define CP_RESET_HOT_TEMP_C 48
#define CURR_MONITOR_PROJECT 4
/* Projects whose gauge is told the pack is full outside fast charge. */
#define FORCE_DONE_PROJECT_A 10
#define FORCE_DONE_PROJECT_B 7
/* Headroom the buck is given when it takes the pack back from the pump. */
#define STOP_TERM_VOLT_MARGIN_MV 30
#define STOP_INPUT_LIMIT_MA 500
#define STOP_BASE_TERM_VOLT_MARGIN_MV 60
#define MCA_QUICK_QC_CHG_ADP_DEFAULT_VOLT 9000
#define MCA_QUICK_PD_FIXED_CHG_ADP_DEFAULT_VOLT 9000
#define MCA_QUICK_CHG_ADP_DEFAULT_CURR 2000
#define MCA_QUICK_CHG_ADP_QC_MAX_VOLT 9500
#define MCA_QUICK_CHG_ADP_QC_MAX_CURR 2500
#define MCA_QUICK_CHG_FLASH_CHG_POWER 20000
#define MCA_QUICK_CHG_TURBO_CHG_POWER 30000
#define MCA_QUICK_CHG_SUPER_CHG_POWER 50000
/* for platform qc3 27w and qc3p5 only begin*/
#define MCA_QUICK_CHG_QC3B_VBUS_LIMIT_DEFAULT 9500
#define MCA_QUICK_CHG_QC3B_IBUS_LIMIT_DEFAULT 2000
#define MCA_QUICK_CHG_QC3P5_VBUS_LIMIT_DEFAULT 10000
#define MCA_QUICK_CHG_QC3P5_IBUS_LIMIT_DEFAULT 2000
#define MCA_QUICK_CHG_QC3_IBAT_LIMIT_DEFAULT 4000
#define MCA_QUICK_CHG_QC_FV_LIMIT_DEFAULT 4440
#define MCA_QUICK_CHG_QC_TAPER_HYS_QC3B 30
#define MCA_QUICK_CHG_QC_TAPER_HYS_QC3P5 20
#define MCA_QUICK_CHG_QC_FV_LIMIT_DEFAULT 4440
#define MCA_TAPER_TIMEOUT 3
/* Below this level the pack always uses the first dual-pump threshold. */
#define MCA_QUICK_CHG_MULTI_PATH_SOC_TH 60
/* Charge level at which an EU pack is handed back to the PMIC. */
#define EU_MODEL_SWITCH_PMIC_SOC 94
/* Where the taper current steps, in whole and in tenths of a degree. */
#define HW_TAPER_WARM_TEMP_C 33
#define HW_TAPER_HOT_TEMP_C 34
#define HW_TAPER_MIDDLE_TEMP_DC 300
#define HW_TAPER_HIGH_TEMP_DC 400
#define BUS_VOLTAGE_MIN	3000
#define MCA_QUICK_CHG_QC_TAPER_FCC_THR_DEFAULT 2500
/* schedule value */
#define MCA_QUICK_CHG_NORMAL_INTERVAL 20000
#define MCA_QUICK_CHG_FAST_INTERVAL 700
#define MCA_QUICK_CHG_PPS_PTF_INTERVAL 10000
#define MCA_QUICK_CHG_PPS_PTF_MAX_CHECK 4
#define MCA_QUICK_CHG_EVENT_WORK_INTERVAL 5000
#define MCA_QUICK_CHG_EVENT_WORK_SLOW_INTERVAL 60000
#define MCA_QUICK_CHG_MAX_ERR_COUNT 10
#define MCA_QUICK_CHG_SINGLE_MAX_ERR_COUNT 5
#define MCA_QUICK_CHG_OPEN_PATH_CURR 2000
#define MCA_QUICK_CHG_OPEN_PATH_IBUS_TH 300
#define MCA_QUICK_CHG_DEFAULT_IBUS_COMPENSATION 0
#define MCA_QUICK_CHG_TUNE_VBUS_MAX_COUNT 15
#define MCA_QUICK_CHG_TUNE_VBUS_INTERVAL 300
#define MCA_QUICK_CHG_TUNE_VBUS_WINDOW_MV 1500
#define MCA_QUICK_CHG_OPEN_PATH_COUNT 10
#define MCA_QUICK_CHG_OPEN_PATH_INTERVAL 200
#define MCA_QUICK_CHG_ADP_GAIN_CURR 200
#define MCA_QUICK_CHG_DEFAULT_VSTEP 20
#define MCA_QUICK_CHG_QC3B_DEFAULT_VSTEP 200
#define MCA_QC_QUICK_CHG_OPEN_PATH_COUNT 40
#define CP_ENABLE_IBUS_UCP_RISING_MA 250
#define ALLOW_ENABLE_CP_BATT_SOC_THR 90
#define ALLOW_START_FFC_BATT_SOC_THR 95
#define MCA_QUICK_CHG_PPS_TAPER_HYS 10
#define MCA_QUICK_CHG_FV_HYS 1
#define MCA_QUICK_CHG_VFC_INTERVAL 5000
#define MCA_QUICK_CHG_CP_DEFAULT_FSW 480
#define MCA_QUICK_CHG_IBUS_QUENE_SIZE 20
#define MCA_QUICK_CHG_SWITCH_PMIC_TH 1500
#define OCP_THRESHOLD_MAINT	9280000
#define OCP_THRESHOLD_FLIP	3540000
#define MCA_ZIMI_CYPRESS_HYS_MV 1000
#define MCA_PPS_MAX_VOLT 10000
#define MCA_THIRD_PARTY_PPS_HYS_MV 1000
#define MCA_QUICK_CHG_MAX_CURR_MA 15600

/* batt_para */
#define BATT_PARA_MAX_GROUP 16
enum mca_quick_charge_batt_para_ele {
	MCA_QUICK_CHG_BATT_ROLE = 0,
	MCA_QUICK_CHG_BATT_ID,
	MCA_QUICK_CHG_TEMP_PARA_NAME,
	MCA_QUICK_CHG_BATT_PARA_MAX,
};

enum mca_quick_usbpd_dpm_port_pps_ptf_type {
    USBPD_QUICK_DPM_PORT_PPS_PTF_NOT_SUPPORTED = 0,
    USBPD_QUICK_DPM_PORT_PPS_PTF_NOT_NORMAL,
    USBPD_QUICK_DPM_PORT_PPS_PTF_NOT_WARNING,
    USBPD_QUICK_DPM_PORT_PPS_PTF_NOT_OVERTEMP,
};

/* temp para */
#define TEMP_PARA_MAX_GROUP 10
enum mca_quick_charge_temp_para_ele {
	MCA_QUICK_CHG_TEMP_LOW = 0,
	MCA_QUICK_CHG_TEMP_HIGH,
	MCA_QUICK_CHG_LOW_TEMP_HYS,
	MCA_QUICK_CHG_HIGH_TEMP_HYS,
	MCA_QUICK_CHG_TEMP_MAX_CURRENT,
	MCA_QUICK_CHG_TEMP_NROMAL_FV,
	MCA_QUICK_CHG_TEMP_FFC_FV,
	MCA_QUICK_CHG_VOLT_PARA_NAME,
	MCA_QUICK_CHG_VOLT_FFC_PARA_NAME,
	MCA_QUICK_CHG_STAGE_PARA_NAME,
	MCA_QUICK_CHG_FFC_STAGE_PARA_NAME,
	MCA_QUICK_CHG_TEMP_PARA_MAX,
};

struct mca_quick_charge_temp_para {
	int temp_low;
	int temp_high;
	int low_temp_hysteresis;
	int high_temp_hysteresis;
	int max_current;
	int normal_max_fv;
	int ffc_max_fv;
};

/* volt para */
#define VOLT_PARA_MAX_GROUP 10
enum mca_quick_charge_volt_para_ele {
	MCA_QUICK_CHG_VOLTAGE = 0,
	MCA_QUICK_CHG_CURRENT_MAX,
	MCA_QUICK_CHG_CURRENT_MIN,
	MCA_QUICK_CHG_VOLT_PARA_MAX,
};

struct mca_quick_charge_volt_para {
	int voltage;
	int current_max;
	int current_min;
};

/* volt step para */
#define VSTEP_PARA_MAX_GROUP 16
#define VSTEP_PARA_EX_MAX_GROUP 8
#define MCA_QUICK_CHG_CUR_MAX_TH_DEFAULT 10000
#define MCA_QUICK_CHG_IBUS_TH_PMIH_DEFAULT 5000
#define MCA_QUICK_CHG_PMIH_FCC_DEFAULT 4000
enum mca_quick_charge_volt_step_para_ele {
	MCA_QUICK_CHG_VOLT_RATIO = 0,
	MCA_QUICK_CHG_CURRENT,
	MCA_QUICK_CHG_VSTEP_RATIO,
	MCA_QUICK_CHG_VSTEP_PARA_MAX,
};

struct mca_quick_charge_volt_step_ex_para {
	int delta_v_thr;
	int vstep_multiplier;
	int vbat_change_thr;
	ktime_t boost_time_thr;
};

struct mca_quick_charge_volt_step_para {
	int volt_ratio;
	int cur_gap;
	int vstep_ratio;
};

/* single cp limit para */
struct mca_quick_charge_single_cp_limit_para {
	int div1_limit;
	int div2_limit;
	int div4_limit;
};

enum mca_quick_charge_battery_type {
	MCA_BATTERY_TYPE_SINGLE = 0,
	MCA_BATTERY_TYPE_SINGLE_SERIES,	/* single fuelgauge, two cells in series */
	MCA_BATTERY_TYPE_SINGLE_NUM_MAX = MCA_BATTERY_TYPE_SINGLE_SERIES,
	MCA_BATTERY_TYPE_PARALLEL,
	MCA_BATTERY_TYPE_SERIES,
	MCA_BATTERY_TYPE_MAX = MCA_BATTERY_TYPE_SERIES,
};

enum mca_quick_charge_cp_type {
	MCA_CP_TYPE_SINGLE = 0,
	MCA_CP_TYPE_PARALLEL,
	MCA_CP_TYPE_SERIES,
	MCA_CP_TYPE_MAX = MCA_BATTERY_TYPE_SERIES,
};

enum mca_quick_charge_work_cp {
	MCA_QUICK_CHG_CP_MASTER = 0,
	MCA_QUICK_CHG_CP_SLAVE,
	MCA_QUICK_CHG_CP_DUAL,
	MCA_QUICK_CHG_CP_MODE_MAX,
};

enum mac_quick_charge_chn_ele {
	MCA_QUICK_CHG_CH_SINGLE = 0,
	MCA_QUICK_CHG_CH_MULTI,
	MCA_QUICK_CHG_CH_MAX,
};

struct mca_quick_charge_volt_para_info {
	int volt_para_size;
	int *stage_para;
	struct mca_quick_charge_volt_para *volt_para;
};

struct mca_quick_charge_temp_para_info {
	struct mca_quick_charge_temp_para temp_para;
	struct mca_quick_charge_volt_para_info volt_info;
	struct mca_quick_charge_volt_para_info volt_ffc_info;
};

struct mca_quick_charge_batt_para_info {
	int temp_para_size;
	struct mca_quick_charge_temp_para_info *temp_info;
};

#define MCA_VFC_PARA_MAX_GROUP 15
enum mca_quick_charge_vfc_para_ele {
	MCA_QUICK_CHG_VFC_PARA_IOUT = 0,
	MCA_QUICK_CHG_VFC_PARA_FSW,
	MCA_QUICK_CHG_VFC_PARA_SIZE,
};

struct mca_quick_charge_vfc_iout_fsw_map {
	int iout;
	int fsw;
};

struct mca_quick_charge_vfc_para {
	int support_cp_vfc;
	int vfc_para_size;
	struct mca_quick_charge_vfc_iout_fsw_map iout_fsw_map[MCA_VFC_PARA_MAX_GROUP];
};

#define MCA_QUICK_CHG_MAX_MODE_CNT 3
enum mca_quick_charge_support_mode {
	MCA_QUICK_CHG_MODE_DIV_1 = 1,
	MCA_QUICK_CHG_MODE_DIV_2 = 2,
	MCA_QUICK_CHG_MODE_DIV_4 = 4,
};

enum mca_quick_charge_chg_mode {
	CHG_MODE_DIV1 = 0,
	CHG_MODE_DIV2,
	CHG_MODE_DIV4,
	CHG_MODE_MAX,
};

/**
 * struct cp_work_mode_unit - the current window one charge-pump mode covers
 * @l_thres: the most current this mode is used for
 * @h_thres: the least
 */
struct cp_work_mode_unit {
	int l_thres;
	int h_thres;
};

/**
 * struct mca_cp_work_mode - which division the charge pump runs at
 * @support_mode_switch: whether the board lets the division change mid-charge
 * @cur_mode:            the division running now, as a CHG_MODE_*
 * @default_mode:        the division to fall back to, and the highest allowed
 * @div1_vbat_low:       the cell is below what the adapter can deliver, so a
 *                       one-to-one division has nothing to work with
 * @last_stage:          the charge has reached its final stage; the division
 *                       stops moving there
 * @mode_table:          the current window each division covers
 */
struct mca_cp_work_mode {
	bool support_mode_switch;
	int cur_mode;
	int default_mode;
	bool div1_vbat_low;
	bool last_stage;
	struct cp_work_mode_unit mode_table[CHG_MODE_MAX];
};

struct mca_quick_charge_adp_info {
	int adp_mode;
	struct adapter_power_cap cap_info;
};

struct mca_quick_charge_ibus_queue_info {
	int data[MCA_QUICK_CHG_IBUS_QUENE_SIZE];
	int count;
	int index;
	unsigned int sum;
	int avg;
};

enum mca_quick_charge_temp_hys_ele {
	MCA_QUICK_TEMP_HYS_DIS = 0,
	MCA_QUICK_TEMP_HYS_HIGH,
	MCA_QUICK_TEMP_HYS_LOW,
};

enum mca_quick_chg_attr_list {
	MCA_QUICK_CHG_MODE_ENABLE = 0,
	MCA_QUICK_CHG_CP_PATH_ENABLE,
	MCA_QUICK_CHG_POWER_MAX,
	MCA_QUICK_CHG_FAKE_PPS_PTF,
	MCA_QUICK_CHG_TYPE,
	MCA_QUICK_CHG_CURR_LIMIT,
	MCA_QUICK_CHG_CURR_RATIO,
	MCA_QUICK_CHG_VOLT_DEC,
	MCA_QUICK_CHG_VFC_IOUT,
};

enum qc_vbus_voltage_tune {
	DIR_HOLD,
	DIR_UP,
	DIR_DOWN,
};

enum VBUS_TUNE_STAT {
	XM_VBUS_TUNE_INIT,
	XM_VBUS_TUNE_VBUS_LOW,
	XM_VBUS_TUNE_WAIT,
	XM_VBUS_TUNE_READY_UP,
	XM_VBUS_TUNE_READY_DOWN,
	XM_VBUS_TUNE_OK,
	XM_VBUS_TUNE_FAIL,
};

struct secure_quick_charge_data {
	int max_vbatt;
	int secure_cur;
};

struct mca_quick_charge_process_data {
	int adp_type;
	int charge_flag;
	int type_chg;
	int temp_hys_en;
	int total_err;
	int error_num[MCA_QUICK_CHG_CP_MODE_MAX];
	int cur_protocol;
	int work_mode;
	int cur_cp_mode;
	int cur_work_cp;
	/* How many times both pumps have been brought up this charge. */
	int dual_cp_count;
	/* Passes spent below the threshold to drop back to one pump. */
	int to_single_count;
	int adp_mode;
	int adp_mode_power[CHG_MODE_MAX];
	int ui_power;
	int max_power;
	int quick_charge_type;
	int cur_adp_volt;
	int cur_adp_cur;
	int min_adp_volt;
	int max_adp_volt;
	int max_adp_curr;
	int max_ibat_final;
	/* Ceiling an EU adapter is held to once it has run at its rating. */
	int max_adap_ibat;
	int ratio;
	int delta_volt;
	int delta_ibat;
	int single_curr;
	int max_curr;
	int open_path;
	int multi_ibus_th;
	/* Threshold for every dual-pump attempt after the first. */
	int again_multi_ibus_th;
	int ibus_inc;
	int ibus_dec;
	int temp_max_cur[FG_IC_MAX];
	int temp_max_fv[FG_IC_MAX];
	int vbat[FG_IC_MAX];
	int parall_vbat[FG_SITE_MAX];
	int max_vcell[FG_IC_MAX];
	int ibat[FG_IC_MAX];
	int parall_ibat[FG_SITE_MAX];
	int soc;
	int ibat_total;
	int vbus;
	int ibus;
	int adp_info_index[CHG_MODE_MAX];
	int *thermal_cur;
	int *cp_path_enable;
	int temp_para_index[FG_IC_MAX];
	int parall_temp_para_index[FG_SITE_MAX];
	/*
	 * Set when a site's temperature left every zone its table describes,
	 * remembering which end it left by so the hysteresis that lets it back
	 * in is taken from that end: 1 came down from a higher zone, 2 came up
	 * from a lower one.
	 */
	int parallel_temp_hys_en[FG_SITE_MAX];
	int parall_zone_changed[FG_SITE_MAX];
	int cur_stage[FG_IC_MAX];
	int parall_cur_stage[FG_SITE_MAX];
	int ffc_flag;
	int zone_changed;
	int vfc_iout;
	int ibus_compensation;
	struct mca_quick_charge_adp_info adp_info[ADAPTER_CAP_MAX_NR];
	struct mca_quick_charge_volt_para_info *cur_volt_para[FG_IC_MAX];
	struct mca_quick_charge_volt_para_info *cur_volt_paraller[FG_SITE_MAX];
	struct secure_quick_charge_data secure_info;
	struct mca_quick_charge_ibus_queue_info ibus_queue;
	int sw_ocp_curr;
	/* The two currents the software over-current watch trips at. */
	int ocp_thr[2];
	bool stop_charging;
	/* Set while the adapter has yet to finish verifying itself. */
	bool to_pd_verify;
	/*
	 * The base cell has been taken off charge, and what its own election
	 * is held to while it is.
	 */
	bool base_batt_close;
	int base_close_curr;
	/*
	 * Set when the adapter announces itself as verified, so the next
	 * refresh of the flip sites' tables runs the entry pass rather than
	 * the in-flight one; cleared once that pass has run.
	 */
	bool parallel_to_pd_verify;
};

struct mca_quick_charge_sysfs_data {
	int chg_enable;
	int chg_limit[MCA_QUICK_CHG_CH_MAX];
	int mode_enable[CHG_MODE_MAX];
	int cp_path_enable[CHG_MODE_MAX][CP_ROLE_MAX];
	int curr_limit[CHG_MODE_MAX][MCA_QUICK_CHG_CH_MAX];
	int cur_ratio;
	int volt_dec;
};

struct mca_quick_charge_smartchg_data {
	int delta_fv;
	int delta_ichg;
	int pwr_boost_state;
};

struct mca_quick_charge_info {
	struct device *dev;
	struct mca_votable *input_suspend_voter;
	struct mca_votable *buck_input_voter;
	struct mca_votable *buck_charge_curr_voter;
	/*
	 * Held while the quick charge is being torn down or built up, so the
	 * two cannot walk the same per-charge state at once.
	 */
	struct mutex data_lock;
	struct delayed_work monitor_work;
	struct delayed_work pps_ptf_work;
	struct delayed_work vfc_work;
	struct delayed_work float_vbat_drop_work;
	struct notifier_block shutdown_notifier;
	struct mca_votable *voter[MCA_QUICK_CHG_CH_MAX * CHG_MODE_MAX];
	struct mca_votable *thermal_flip_voter;
	/*
	 * Set when the buck charger reports the charge current has fallen
	 * below what fast charging needs; makes the next table pick take the
	 * normal voltages, and is cleared once it has.
	 */
	/* Which charge pump this board carries, as the pump reports itself. */
	int cp_vendor;
	bool fcc_too_low;
	bool parallel_fcc_too_low[FG_SITE_MAX];
	/* The lower edge of the cycle-count band each gauge's cell is in. */
	int curr_batt_cycle[FG_IC_MAX];
	struct mca_votable *chg_disable_voter;
	struct mca_votable *chg_en_voter;
	struct mca_votable *single_chg_cur_voter;
	struct mca_votable *multi_chg_cur_voter;
	struct mca_quick_charge_smartchg_data smartchg_data;
	/* Ceiling on the fast-charge current, voted by the thermal layer. */
	int thermal_flip_cur;
	/* Ceiling on the fast-charge current, asked for by smart charging. */
	int bypass_curr;
	/* dts config */
	int batt_type;
	int cp_type;
	int min_vbat;
	int max_vbat;
	/* The pack voltage under which charging starts again after a full charge. */
	int recharge_vbat;
	/*
	 * Whether to drop the bus current compensation once the pack is close
	 * enough to full that the compensation alone would overshoot, and the
	 * voltage that counts as close enough.
	 */
	bool support_cancel_compensation;
	int max_vbat_threshold;
	int die_temp_max;
	int adp_temp_max;
	int en_buck_parallel_chg;
	int fv_hys_delta_mv;
	int curr_terminate_ratio;
	int buck_icl_fcc_curr[2];
	int div_delta_ibat[CHG_MODE_MAX];
	int div_delta_volt[CHG_MODE_MAX];
	int div_single_curr[CHG_MODE_MAX];
	int div_max_curr[CHG_MODE_MAX];
	int open_path_th[CHG_MODE_MAX];
	int multi_ibus_th[CHG_MODE_MAX];
	int ibus_inc_hysteresis[CHG_MODE_MAX];
	int ibus_dec_hysteresis[CHG_MODE_MAX];
	int ibus_compensation[CHG_MODE_MAX];
	int support_mode;
	int max_power;
	int ui_max_power_limit;
	bool is_platform_qc;
	int qc3_max_vbus_limit_mv;
	int qc3p5_max_vbus_limit_mv;
	int qc3_ibat_max_limit_ma;
	int qc3_max_ibus_limit_ma;
	int qc3p5_max_ibus_limit_ma;
	int qc3p5_max_ibat_limit_ma;
	int qc_taper_fcc_ma;
	int pps_taper_fcc_ma;
	int pps_high_taper_fcc_ma;
	int qc_normal_charge_fv_max_mv;
	int qc3_taper_vol_hys;
	int qc3p5_taper_vol_hys;
	int pps_taper_vol_hys;
	int fc2_taper_timer;
	int hardware_cv;
	int rawsoc_swith_pmic_th;
	int has_gbl_batt_para;
	bool taper_done_no_retry;
	bool pd_switch_to_pmic;
	struct mca_quick_charge_volt_step_para vstep_para[VSTEP_PARA_MAX_GROUP];
	struct mca_quick_charge_batt_para_info batt_para[FG_IC_MAX];
	struct mca_quick_charge_batt_para_info base_flip_para[FG_SITE_MAX];
	struct mca_quick_charge_vfc_para vfc_para;
	/* process data */
	int online;
	int cur_support_mode;
	int thermal_cur[CHG_MODE_MAX][MCA_QUICK_CHG_CH_MAX];
	struct mca_quick_charge_process_data proc_data;
	struct mca_quick_charge_sysfs_data sysfs_data;
	int batt_auth;
	int force_stop;
	int tune_vbus_retry;
	int master_cp_enable_count;
	int dtpt_status;
	int trigger_antr_burn;
	int is_eu_model;
	int pps_ptf;
	int fake_pps_ptf;
	int support_curr_monitor;
	int curr_monitor_time_s;
	int cp_switch_pmic_th;
	bool fastchg_temp_flag;
	ktime_t time_start;
	bool boost_done;
	bool check_vbat_ov;
	int support_base_flip;
	/* Whether the smart-charge trim also applies once DTPT has scaled. */
	int support_dtpt_ichg_dec;
	/* Set while the phone is going down, so the port is put back to 5V. */
	int shutdown_mode;
	/*
	 * How far the board lets a pump-fed charge run before the buck is
	 * asked to help, and the ceiling one sales region puts on what may be
	 * asked of the adapter.
	 */
	int use_buck_fcc_threshold;
	int third_pps_ibat_limit;
	bool support_eu_power;
	int eu_vbus_max;
	int eu_ibus_max;
	int vbat_threshold;
	int pmic_single_cp_chg;
	/*
	 * The buck's share while one pump runs beside it, and the float
	 * voltage the buck is held to so the two do not fight over the pack.
	 */
	struct mca_votable *buck_vterm_voter;
	struct mca_votable *vterm_voter;
	struct mca_votable *flip_charge_curr_voter;
	int pmih_fcc;
	bool stop_pmic;
	int final_vterm;
	int override_vterm;
	int last_override_vterm;
	int vterm_count;
	int pmic_fv_compensation;
	int support_ichg_cutoff_priority;
	int enable_vstep_ex;
	int support_pmic_vterm_dynamics_adjust;
	bool pmic_chg_need_fixed_volt;
	int base_flip_same;
	/*
	 * Read because the board sets them; nothing in this module acts on
	 * them, and nothing does in the blob either.
	 */
	int curr_terminate_section;
	int qc_taper_soc_thr;
	int pps_middle_taper_fcc_ma;
	int third_pps_switch_pmic;
	int quick_chg_fv_hys;
	int mult_to_single_count;
	int again_multi_ibus_th[CHG_MODE_MAX];
	int cur_max_threshold[3];
	int ibus_threshold[3];
	int pmih_fcc_value[4];
	/*
	 * The finer voltage-step table a board can ask for with
	 * enable-vstep-ex; each row also carries how long the step may be
	 * boosted for.
	 */
	struct mca_quick_charge_volt_step_ex_para vstep_para_ex[VSTEP_PARA_EX_MAX_GROUP];
	struct mca_cp_work_mode cp_work_mode;
};

#endif /* __MCA_QUICK_CHARGE_H__ */

