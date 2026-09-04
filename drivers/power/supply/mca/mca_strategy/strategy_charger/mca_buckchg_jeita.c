// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2025 The LineageOS Project
 *
 * JEITA charging limits for the buck charger.
 *
 * A lithium cell will take its full current only over a narrow band of
 * temperatures.  Outside it the cell is charged more slowly and to a lower
 * float voltage, and outside that not at all.  The board's device tree gives
 * the bands as a table: for each, the temperature it covers, the current and
 * float voltage allowed there, and the hysteresis to apply when leaving it.
 *
 * Two things make this more than a table lookup.
 *
 * The first is that the current a band allows is not one number: a nearly
 * empty cell will take more than a nearly full one at the same temperature,
 * so a band may carry a second table indexed by battery voltage.  Both
 * lookups get hysteresis, because a cell that is being charged warms and its
 * voltage rises, so a bare threshold would be crossed back and forth every
 * few seconds and the charger would spend its time changing its mind.
 *
 * The second is that a foldable has two cells.  When the board says it has,
 * the base and flip halves each get their own table and their own limits,
 * and the flip half's is applied through the load switch rather than the
 * charger.
 *
 * Nothing here talks to the charger directly.  Every limit is cast as a vote,
 * so a limit from somewhere else -- the thermal budget, the user's own
 * setting -- can be more restrictive without this module knowing about it.
 */

#define MCA_LOG_TAG "mca_buckchg_jeita"

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/hwid.h>
#include <linux/kernel.h>
#include <mca/common/mca_voter.h>
#include <mca/common/mca_event.h>
#include <linux/delay.h>
#include <mca/common/mca_hwid.h>
#include <mca/common/mca_log.h>
#include <mca/common/mca_parse_dts.h>
#include <mca/platform/platform_fg_ic_ops.h>
#include <mca/platform/platform_loadsw_class.h>
#include <mca/smartchg/smart_chg_class.h>
#include <mca/strategy/strategy_class.h>
#include <mca/strategy/strategy_fg_class.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/workqueue.h>

/* How often the temperature and voltage are looked at again. */
/*
 * How long after probe to take the first reading when nothing is attached:
 * far enough out that the rest of the stack has finished registering, and
 * near enough that a charger plugged in meanwhile is not missed.
 */
#define JEITA_PROBE_DELAY_JIFFIES	250

/* How long to wait before looking for the tables again, and how often. */
#define JEITA_PARSE_RETRY_DELAY_MS	100
#define JEITA_PARSE_RETRY_MAX		49

#define JEITA_MONITOR_INTERVAL_MS	3000

/*
 * A band whose float voltage is at or above this is a normal one, limited by
 * voltage.  Below it the band is a hot or cold one, and charging is stopped
 * on reaching the float voltage rather than tapered towards it.
 */
#define JEITA_NORMAL_VTERM_MIN_MV	4201

/* How far the voltage must fall before a stopped hot band charges again. */
#define JEITA_HOT_RESUME_MARGIN_MV	150

/* The band below this index is close enough to empty to use the low margin. */
#define JEITA_VBAT_LOW_HYST_BANDS	3

/* Default margin for a battery-voltage step, in millivolts. */
#define JEITA_VBAT_HYST_DEFAULT		50

/*
 * The one project whose float-voltage offset is capped for global units; the
 * cells shipped there are qualified to a lower voltage than the Chinese ones.
 */
#define JEITA_DELTA_FV_GLOBAL_MAX_MV	20

/* The fuel gauge a room-temperature test fixture reports. */
#define JEITA_TMP_TEST_FG_NAME		"2@BP"

/* Who this module votes as. */
#define JEITA_VOTER			"jeita"
#define JEITA_HOT_VOTER			"jeita-hot"
#define JEITA_SWOCP_VOTER		"swocp"

/* The gauge reports in microamps; the limits are in milliamps. */
#define UA_PER_MA			1000

/* Consecutive over-current samples before the vote is placed. */
#define JEITA_SWOCP_DEBOUNCE		3

/* How far the fast charge current is pulled down each time. */
#define JEITA_SWOCP_STEP_MA		100

/* How far under the limit the reading must come back before letting go. */
#define JEITA_SWOCP_RELEASE_MA		500

/* The elections this module takes part in. */
#define JEITA_FLIP_FCC_VOTABLE		"flip_charge_curr"
#define JEITA_EN_VOTABLE		"chg_enable"
#define JEITA_FCC_VOTABLE		"buck_charge_curr"
#define JEITA_VTERM_VOTABLE		"term_volt"
#define JEITA_ITERM_VOTABLE		"term_curr"

/* The most bands, and the most voltage steps within one, a board may give. */
#define JEITA_TEMP_PARA_MAX_ROWS	15
#define JEITA_VOLT_PARA_MAX_ROWS	4

/* What the flip half is allowed until its own table says otherwise. */
#define JEITA_FLIP_FCC_DEFAULT_MA	2950

/* The columns of one row of a jeita table, in the order the property gives. */
enum jeita_temp_para_ele {
	JEITA_TEMP_PARA_TEMP_LOW,
	JEITA_TEMP_PARA_TEMP_HIGH,
	JEITA_TEMP_PARA_LOW_TEMP_HYS,
	JEITA_TEMP_PARA_HIGH_TEMP_HYS,
	JEITA_TEMP_PARA_MAX_CURRENT,
	JEITA_TEMP_PARA_VTERM,
	JEITA_TEMP_PARA_ITERM,
	JEITA_TEMP_PARA_VOLT_PARA_NAME,
	JEITA_TEMP_PARA_MAX,
};

/* The columns of one row of a band's battery-voltage table. */
enum jeita_volt_para_ele {
	BUCKCHG_VOLTAGE_TH,
	BUCKCHG_CURRENT_MAX,
	BUCKCHG_VOLTAGE_PARA_MAX,
};

/**
 * struct mca_buckchg_jeita_volt_data - one step of a band's voltage table
 * @voltage:     the battery voltage this step covers up to, in millivolts
 * @max_current: what the cell will take below that voltage, in milliamps
 */
struct mca_buckchg_jeita_volt_data {
	int	voltage;
	int	max_current;
};

/**
 * struct mca_buckchg_jeita_volt_para - a band's battery-voltage table
 * @size:      how many steps it has
 * @volt_data: the steps, lowest voltage first
 */
struct mca_buckchg_jeita_volt_para {
	int					size;
	struct mca_buckchg_jeita_volt_data	*volt_data;
};

/**
 * struct mca_buckchg_jeita_data - one temperature band
 * @temp_low:       the band starts here, in degrees
 * @temp_high:      and ends below here
 * @low_temp_hys:   how far below @temp_low the cell must fall to leave
 * @high_temp_hys:  how far above @temp_high it must rise to leave
 * @max_current:    what the band allows, in milliamps
 * @vterm:          the float voltage it allows, in millivolts
 * @iterm:          the current charging is finished at, in milliamps
 * @volt_para:      a further limit by battery voltage, empty if there is none
 */
struct mca_buckchg_jeita_data {
	int					temp_low;
	int					temp_high;
	int					low_temp_hys;
	int					high_temp_hys;
	int					max_current;
	int					vterm;
	int					iterm;
	struct mca_buckchg_jeita_volt_para	volt_para;
};

/**
 * struct mca_buckchg_jeita_para - a cell's bands
 * @size:           how many normal bands there are
 * @fcc_size:       how many fast-charge bands there are
 * @jeita_data:     the bands used when not fast charging
 * @jeita_ffc_data: the bands used when fast charging
 *
 * A cell takes more when it is being fast charged and the bands are drawn
 * differently, so the two are separate tables rather than one with a column
 * for the mode.
 */
struct mca_buckchg_jeita_para {
	int				size;
	int				fcc_size;
	struct mca_buckchg_jeita_data	*jeita_data;
	struct mca_buckchg_jeita_data	*jeita_ffc_data;
};

/**
 * struct mca_buckchg_jeita_proc_data - where one cell's limits stand
 * @cur_jeita_index: the band in force, -1 before the first look
 * @temp_hys_en:     whether hysteresis is holding it in that band
 * @max_chg_curr:    the current that band came to, in milliamps
 */
struct mca_buckchg_jeita_proc_data {
	int	cur_jeita_index;
	int	temp_hys_en;
	int	max_chg_curr;
};

/**
 * struct mca_buck_jeita_smartchg_data - what smart charging has asked for
 * @delta_fv:   lower the float voltage by this many millivolts
 * @delta_ichg: lower the charge current by this many milliamps
 */
struct mca_buck_jeita_smartchg_data {
	int	delta_fv;
	int	delta_ichg;
};

/**
 * struct mca_buckchg_jeita_dev - this module's state
 * @dev:                        this device
 * @online:                     whether an adapter is connected
 * @voter_ok:                   whether every election was found
 * @dtpt_status:                whether the battery is under a temperature test
 * @has_gbl_batt_para:          the bands come from the global node
 * @has_tmp_batt_para:          the bands come from the test-fixture node
 * @vbat_high_hyst:             margin for a voltage step in a full cell
 * @vbat_low_hyst:              margin for a voltage step in an empty one
 * @vbat_low_cold_hyst:         margin for a voltage step in a cold cell
 * @base_vbat_low_hyst:         the same, for a foldable's base half
 * @flip_vbat_low_hyst:         the same, for its flip half
 * @support_base_flip:          the board has two cells with separate tables
 * @base_flip_same:             both halves take the one table after all
 * @monitor_work:               looks at the temperature again every few seconds
 * @flip_fcc_voter:             what the flip half is allowed
 * @en_voter:                   whether charging happens at all
 * @fcc_voter:                  what the charger is allowed
 * @vterm_voter:                what float voltage it charges to
 * @iterm_voter:                what current it stops at
 * @proc_data:                  where the single-cell limits stand
 * @base_proc_data:             where the base half's stand
 * @flip_proc_data:             where the flip half's stand
 * @jeita_para:                 the single-cell bands
 * @base_jeita_para:            the base half's bands
 * @flip_jeita_para:            the flip half's bands
 * @smartchg_data:              what smart charging has asked for
 * @baacfg_update:              the bands were just rewritten, skip hysteresis
 * @vterm:                      the float voltage last voted for
 * @jeita_hot_termination_hyst: how far below the float voltage a hot band
 *                              stops charging
 */
struct mca_buckchg_jeita_dev {
	struct device				*dev;
	int					online;
	int					voter_ok;
	int					dtpt_status;
	int					has_gbl_batt_para;
	int					has_tmp_batt_para;
	int					vbat_high_hyst;
	int					vbat_low_hyst;
	int					vbat_low_cold_hyst;
	int					base_vbat_low_hyst;
	int					flip_vbat_low_hyst;
	int					support_base_flip;
	int					base_flip_same;
	struct delayed_work			monitor_work;
	struct mca_votable			*flip_fcc_voter;
	struct mca_votable			*en_voter;
	struct mca_votable			*fcc_voter;
	struct mca_votable			*vterm_voter;
	struct mca_votable			*iterm_voter;
	struct mca_buckchg_jeita_proc_data	proc_data;
	struct mca_buckchg_jeita_proc_data	base_proc_data;
	struct mca_buckchg_jeita_proc_data	flip_proc_data;
	struct mca_buckchg_jeita_para		jeita_para;
	struct mca_buckchg_jeita_para		base_jeita_para;
	struct mca_buckchg_jeita_para		flip_jeita_para;
	struct mca_buck_jeita_smartchg_data	smartchg_data;
	bool					baacfg_update;
	int					vterm;
	int					jeita_hot_termination_hyst;
};

static struct mca_buckchg_jeita_dev *g_jeita;

/*
 * What the last pass came to.  A limit is only voted for when it differs from
 * the last one, and hysteresis needs to know which way the last step went.
 */
static int last_fastcharge;
static int last_chg_curr;

/**
 * jeita_find_index() - which band a temperature falls in
 * @data: the bands
 * @size: how many there are
 * @temp: the temperature, in degrees
 *
 * The bands do not overlap and are given lowest first, so the first one whose
 * window contains @temp is the answer.
 *
 * Return: the band's index, or -1 if the temperature is off both ends of the
 * table -- which is a table that does not describe this cell, not a cell that
 * is too hot.
 */
static int jeita_find_index(const struct mca_buckchg_jeita_data *data, int size,
			    int temp)
{
	int i;

	for (i = 0; i < size; i++) {
		if (temp >= data[i].temp_high)
			continue;
		if (temp < data[i].temp_low)
			continue;
		return i;
	}

	return -1;
}

/**
 * jeita_hold_index() - whether hysteresis keeps the cell in the band it is in
 * @data: the bands
 * @cur:  the band in force
 * @next: the band the temperature now falls in
 * @temp: the temperature, in degrees
 *
 * Leaving a band costs the cell its current or its float voltage, and the
 * temperature drifts by a degree either way as the charger works, so a band
 * is only left once the temperature has gone past it by the margin the band
 * itself gives.  The margin is asymmetric: warming out of a band uses that
 * band's high margin, cooling out of it the low one.
 *
 * Return: true to stay in @cur, false to move to @next.
 */
static bool jeita_hold_index(const struct mca_buckchg_jeita_data *data, int cur,
			     int next, int temp)
{
	if (next > cur)
		return temp < data[cur].temp_high + data[cur].high_temp_hys;

	if (cur > 0)
		return temp >= data[cur].temp_low - data[cur].low_temp_hys;

	/* The coldest band has nothing below it to fall into. */
	return false;
}

/**
 * jeita_volt_limit() - what the cell's own voltage allows within a band
 * @row:   the band
 * @vbat:  the battery voltage, in millivolts
 * @index: filled in with the step that decided it, -1 if the band has no table
 *
 * The steps are given lowest voltage first.  Below the first step's threshold
 * that step decides; above every threshold the cell is near full and takes the
 * least of what any step allows.
 *
 * Return: the current the band allows at this voltage, in milliamps.
 */
static int jeita_volt_limit(const struct mca_buckchg_jeita_data *row, int vbat,
			    int *index)
{
	const struct mca_buckchg_jeita_volt_para *para = &row->volt_para;
	int curr = row->max_current;
	int i;

	if (para->size < 1) {
		*index = -1;
		return curr;
	}

	for (i = 0; i < para->size; i++) {
		if (para->volt_data[i].voltage >= vbat) {
			*index = i;
			return para->volt_data[i].max_current;
		}
		curr = min(curr, para->volt_data[i].max_current);
	}

	*index = para->size;
	return curr;
}

/**
 * jeita_vbat_hyst() - the margin a battery-voltage step is left by
 * @jeita: this module's state
 * @index: the temperature band in force
 *
 * A nearly empty cell's voltage sags under load and recovers when the current
 * is cut, by more than a full one's does, so the lower bands are given the
 * wider margin.
 *
 * Return: the margin, in millivolts.
 */
static int jeita_vbat_hyst(const struct mca_buckchg_jeita_dev *jeita, int index)
{
	if ((unsigned int)(index - 1) < JEITA_VBAT_LOW_HYST_BANDS)
		return jeita->vbat_low_hyst;

	return jeita->vbat_high_hyst;
}

/**
 * jeita_apply_curr() - decide whether a change in current is acted on
 * @jeita:      this module's state
 * @row:        the band in force
 * @curr:       what the tables now say
 * @last:       what was last applied
 * @vbat:       the battery voltage, in millivolts
 * @index:      the temperature band in force
 * @volt_index: the voltage step that decided @curr, -1 if there was none
 *
 * Charging raises the battery voltage and cutting the current lets it fall
 * again, so a step taken on the bare threshold would be undone a few seconds
 * later and taken again after that.  Going down through a step happens as
 * soon as the threshold is reached; coming back up waits until the voltage
 * has fallen a margin below it.
 *
 * Return: the current to apply, in milliamps.
 */
static int jeita_apply_curr(const struct mca_buckchg_jeita_dev *jeita,
			    const struct mca_buckchg_jeita_data *row, int curr,
			    int last, int vbat, int index, int volt_index)
{
	int thre;

	if (volt_index < 0)
		return curr;

	if (curr > last) {
		thre = row->volt_para.volt_data[volt_index].voltage -
		       jeita_vbat_hyst(jeita, index);
		if (vbat >= thre)
			return last;

		mca_log_info("vbat_thre[s->l]: max_chg_curr:%d, last_chg_curr:%d, vbat_index:%d\n",
			     curr, last, volt_index);
	} else if (curr < last) {
		thre = row->volt_para.volt_data[volt_index - 1].voltage;
		if (vbat <= thre)
			return curr;

		mca_log_info("vbat_thre[l->s]: max_chg_curr:%d, last_chg_curr:%d, vbat_index:%d, vbat:%d\n",
			     curr, last, volt_index, vbat);
	}

	return curr;
}

/**
 * jeita_vote_term() - vote the float voltage and whether to charge at all
 * @jeita: this module's state
 * @row:   the band in force
 * @vbat:  the battery voltage, in millivolts
 *
 * A normal band is charged towards its float voltage and finishes there.  A
 * hot or cold one has a float voltage low enough that the cell may already be
 * above it, and there is nothing to taper towards: charging is stopped short
 * of it and does not resume until the cell has fallen well below, so that a
 * cell sitting right at the threshold is not switched on and off.
 */
static void jeita_vote_term(struct mca_buckchg_jeita_dev *jeita,
			    const struct mca_buckchg_jeita_data *row, int vbat)
{
	int vterm = row->vterm;

	if (vterm >= JEITA_NORMAL_VTERM_MIN_MV) {
		mca_vote(jeita->en_voter, JEITA_HOT_VOTER, true, 1);

		vterm -= jeita->smartchg_data.delta_fv;
		if (vterm != jeita->vterm) {
			mca_vote(jeita->vterm_voter, JEITA_VOTER, true, vterm);
			jeita->vterm = vterm;
		}
		return;
	}

	if (vbat >= vterm - jeita->jeita_hot_termination_hyst)
		mca_vote(jeita->en_voter, JEITA_HOT_VOTER, true, 0);
	else if (vbat < vterm - JEITA_HOT_RESUME_MARGIN_MV)
		mca_vote(jeita->en_voter, JEITA_HOT_VOTER, true, 1);
}

/**
 * mca_buckchg_jeita_update() - apply the limits for the temperature now
 * @jeita: this module's state
 *
 * Return: 0, or a negative error if the cell could not be read or the tables
 * do not cover it.
 */
static int mca_buckchg_jeita_update(struct mca_buckchg_jeita_dev *jeita)
{
	struct mca_buckchg_jeita_para *para = &jeita->jeita_para;
	struct mca_buckchg_jeita_proc_data *proc = &jeita->proc_data;
	struct mca_buckchg_jeita_data *data;
	struct mca_buckchg_jeita_data *row;
	int temp, vbat, fastcharge;
	int size, index, curr, volt_index;
	int hys_affect = 0;
	int ret;

	ret = strategy_class_fg_ops_get_temperature(&temp);
	if (ret) {
		mca_log_err("get battery temp failed\n");
		return ret;
	}
	temp /= 10;

	fastcharge = strategy_class_fg_get_fastcharge();

	ret = strategy_class_fg_ops_get_voltage(&vbat);
	if (ret) {
		mca_log_err("get battery volt failed\n");
		return ret;
	}

	if (fastcharge) {
		data = para->jeita_ffc_data;
		size = para->fcc_size;
	} else {
		data = para->jeita_data;
		size = para->size;
	}

	if (!data || size < 1) {
		mca_log_err("jeita data not ready\n");
		return -ENODEV;
	}

	index = jeita_find_index(data, size, temp);
	if (index < 0) {
		if (fastcharge)
			mca_log_err("can not find jeita fcc para\n");
		else
			mca_log_err("can not find jeita para\n");
		return -ERANGE;
	}

	/*
	 * Hysteresis compares against the band in force, so it means nothing
	 * on the first pass, and nothing once the tables it was measured
	 * against have been replaced or the cell has changed charging mode.
	 */
	if (proc->cur_jeita_index != -1 && !jeita->baacfg_update &&
	    proc->cur_jeita_index != index && fastcharge == last_fastcharge) {
		row = &data[proc->cur_jeita_index];

		mca_log_info("temp_low: %d, temp_high: %d, temp: %d\n",
			     row->temp_low - row->low_temp_hys,
			     row->temp_high + row->high_temp_hys, temp);

		/*
		 * Extreme cold suspends the hysteresis: down there the cell
		 * is being warmed by the charge itself, and holding it in
		 * the band it started in would keep it on the coldest,
		 * slowest limit long after it had warmed out of it.
		 */
		if (!mca_smartchg_is_extreme_cold_enabled() &&
		    jeita_hold_index(data, proc->cur_jeita_index, index,
				     temp)) {
			hys_affect = 1;
			index = proc->cur_jeita_index;
		}
	}

	if (jeita->baacfg_update) {
		mca_log_info("BAA config data update\n");
		jeita->baacfg_update = false;
	}

	row = &data[index];
	last_fastcharge = fastcharge;

	curr = jeita_volt_limit(row, vbat, &volt_index);

	if (hys_affect || proc->cur_jeita_index == index) {
		curr = jeita_apply_curr(jeita, row, curr, last_chg_curr, vbat,
					index, volt_index);
	} else {
		mca_log_info("jeita_index: cur_jeita_index:%d, i:%d, hys_affect:%d\n",
			     proc->cur_jeita_index, index, hys_affect);
	}

	proc->cur_jeita_index = index;
	proc->temp_hys_en = hys_affect;
	proc->max_chg_curr = curr;
	last_chg_curr = curr;

	jeita_vote_term(jeita, row, vbat);

	mca_log_info("cur index %d/%d last_chg_curr %d chg_curr %d/%d fastcharge %d jeita_vterm %d jeita_iterm %d vterm %d delta_fv %d hys_affect %d\n",
		     index, size, last_chg_curr, curr, row->max_current,
		     fastcharge, row->vterm, row->iterm, jeita->vterm,
		     jeita->smartchg_data.delta_fv, hys_affect);

	mca_vote(jeita->fcc_voter, JEITA_VOTER, true,
		 curr - jeita->smartchg_data.delta_ichg);
	mca_vote(jeita->iterm_voter, JEITA_VOTER, true, row->iterm);
	mca_vote(jeita->en_voter, JEITA_VOTER, true, 1);

	return 0;
}

/**
 * jeita_half_update() - apply one half of a foldable's limits
 * @jeita: this module's state
 * @para:  that half's bands
 * @proc:  where that half's limits stand
 * @role:  which fuel gauge reads that half
 * @hyst:  the margin its voltage steps are left by
 * @what:  what to call it in the log
 *
 * The two halves are read through their own fuel gauges rather than the
 * strategy layer, which only knows about the pack as a whole.  The base half
 * additionally watches its charge current: a cell taking more than its band
 * allows means the switch feeding it is passing more than it was told to, and
 * the limit is pulled down until it obeys.
 *
 * Return: the current that half is allowed, in milliamps, or a negative error.
 */
static __always_inline int jeita_half_update(struct mca_buckchg_jeita_dev *jeita,
			     struct mca_buckchg_jeita_para *para,
			     struct mca_buckchg_jeita_proc_data *proc,
			     enum fg_ic_role role, int hyst, const char *what)
{
	struct mca_buckchg_jeita_data *data;
	struct mca_buckchg_jeita_data *row;
	int temp, vbat, fastcharge;
	int size, index, curr, volt_index;
	int hys_affect = 0;
	int ret;

	ret = platform_fg_ops_get_temp(role, &temp);
	if (ret) {
		mca_log_err("get battery temp failed\n");
		return ret;
	}
	temp /= 10;

	fastcharge = strategy_class_fg_get_fastcharge();
	mca_log_err("fastcharge_mode is %d\n", fastcharge);

	ret = platform_fg_ops_get_volt(role, &vbat);
	if (ret) {
		mca_log_err("get battery volt failed\n");
		return ret;
	}
	mca_log_info("vbat is %d\n", vbat);

	if (fastcharge) {
		data = para->jeita_ffc_data;
		size = para->fcc_size;
	} else {
		data = para->jeita_data;
		size = para->size;
	}

	if (!data || size < 1) {
		mca_log_err("jeita data not ready\n");
		return -ENODEV;
	}

	index = jeita_find_index(data, size, temp);
	if (index < 0) {
		if (fastcharge)
			mca_log_err("can not find flip jeita fcc para\n");
		else
			mca_log_err("can not find flip jeita para\n");
		return -ERANGE;
	}

	if (proc->cur_jeita_index != -1 && proc->cur_jeita_index != index) {
		row = &data[proc->cur_jeita_index];

		mca_log_info("temp_low: %d, temp_high: %d, temp: %d\n",
			     row->temp_low - row->low_temp_hys,
			     row->temp_high + row->high_temp_hys, temp);

		/*
		 * Extreme cold suspends the hysteresis: down there the cell
		 * is being warmed by the charge itself, and holding it in
		 * the band it started in would keep it on the coldest,
		 * slowest limit long after it had warmed out of it.
		 */
		if (!mca_smartchg_is_extreme_cold_enabled() &&
		    jeita_hold_index(data, proc->cur_jeita_index, index,
				     temp)) {
			hys_affect = 1;
			index = proc->cur_jeita_index;
		}
	}

	mca_log_info("jeita_index: cur_jeita_index:%d, i:%d, hys_affect:%d\n",
		     proc->cur_jeita_index, index, hys_affect);

	row = &data[index];
	curr = jeita_volt_limit(row, vbat, &volt_index);

	if (volt_index >= 0 && proc->max_chg_curr) {
		if (curr > proc->max_chg_curr) {
			if (vbat >= row->volt_para.volt_data[volt_index].voltage - hyst)
				curr = proc->max_chg_curr;
			else
				mca_log_info("vbat_thre[s->l]: max_chg_curr:%d, last_chg_curr:%d, vbat_index:%d\n",
					     curr, proc->max_chg_curr,
					     volt_index);
		} else if (curr < proc->max_chg_curr) {
			mca_log_info("vbat_thre[l->s]: max_chg_curr:%d, last_chg_curr:%d, vbat_index:%d, vbat:%d\n",
				     curr, proc->max_chg_curr, volt_index, vbat);
		}
	}

	proc->cur_jeita_index = index;
	proc->temp_hys_en = hys_affect;
	proc->max_chg_curr = curr;

	mca_log_info("%s cur index %d/%d max_chg_curr %d chg_curr %d hys_affect %d\n",
		     what, index, size, curr, row->max_current, hys_affect);

	return curr;
}

/**
 * mca_buckchg_base_jeita_update() - apply the base half's limits
 * @jeita: this module's state
 *
 * The base cell is the one the charger feeds directly, so what it actually
 * draws can be measured.  Drawing more than its band allows means the current
 * is going somewhere it was not sent, and the software over-current vote pulls
 * the whole charger down until the reading agrees again.
 */
static void mca_buckchg_base_jeita_update(struct mca_buckchg_jeita_dev *jeita)
{
	static unsigned int over_curr_count;
	static bool swocp_engaged;
	int curr, now_curr, effective_curr;
	int ret;

	curr = jeita_half_update(jeita, &jeita->base_jeita_para,
				 &jeita->base_proc_data, FG_IC_MASTER,
				 jeita->base_vbat_low_hyst, "base");
	if (curr < 0)
		return;

	ret = platform_fg_ops_get_curr(FG_IC_MASTER, &now_curr);
	if (ret) {
		mca_log_err("get battery temp or currfailed\n");
		return;
	}

	effective_curr = mca_get_effective_result(jeita->fcc_voter);

	mca_log_info("cur index %d/%d max_chg_curr %d chg_curr %d hys_affect %d, now_curr %d, effective_curr %d\n",
		     jeita->base_proc_data.cur_jeita_index,
		     jeita->base_jeita_para.size, curr, curr,
		     jeita->base_proc_data.temp_hys_en, now_curr,
		     effective_curr);

	/*
	 * The gauge reports microamps and counts charge as negative, so the
	 * reading has to come back to milliamps and be compared against the
	 * negated limit.  One sample is not enough to act on: the reading
	 * swings, and voting the charger down and up again on alternate
	 * passes is worse than a brief overshoot.
	 */
	now_curr /= UA_PER_MA;

	if (effective_curr && now_curr < -curr)
		over_curr_count++;
	else
		over_curr_count = 0;

	if (over_curr_count > JEITA_SWOCP_DEBOUNCE && effective_curr) {
		over_curr_count = 0;
		swocp_engaged = true;
		mca_vote(jeita->fcc_voter, JEITA_SWOCP_VOTER, true,
			 effective_curr - JEITA_SWOCP_STEP_MA);
		mca_log_err("reduce fcc %d by swocp\n",
			    effective_curr - JEITA_SWOCP_STEP_MA);
	}

	/*
	 * Let go only once the reading has come back with room to spare, or
	 * the vote would be dropped the moment it started working.
	 */
	if (swocp_engaged && now_curr > JEITA_SWOCP_RELEASE_MA - curr) {
		swocp_engaged = false;
		mca_vote(jeita->fcc_voter, JEITA_SWOCP_VOTER, false, 0);
		mca_log_err("disable swocp\n");
	}
}

/**
 * mca_buckchg_flip_jeita_update() - apply the flip half's limits
 * @jeita: this module's state
 *
 * The flip cell sits behind a load switch, so its limit is voted rather than
 * applied: the election's callback is what programs the switch.
 */
static void mca_buckchg_flip_jeita_update(struct mca_buckchg_jeita_dev *jeita)
{
	int curr;

	curr = jeita_half_update(jeita, &jeita->flip_jeita_para,
				 &jeita->flip_proc_data, FG_IC_SLAVE,
				 jeita->flip_vbat_low_hyst, "flip");
	if (curr < 0)
		return;

	mca_vote(jeita->flip_fcc_voter, JEITA_VOTER, true, curr);
}

/**
 * mca_buckchg_jeita_flip_charge_limit() - program the flip half's load switch
 * @votable:          the flip current election
 * @data:             this module's state
 * @effective_result: what the election came to, in milliamps
 * @effective_client: who asked for it
 *
 * Return: 0, or -EINVAL if the election was created without state.
 */
static int mca_buckchg_jeita_flip_charge_limit(struct mca_votable *votable,
					       void *data, int effective_result,
					       const char *effective_client)
{
	if (!data)
		return -EINVAL;

	mca_log_err("set flip limit current %d\n", effective_result);

	return platform_class_loadsw_set_ibat_limit(0, effective_result);
}

/**
 * mca_buckchg_jeita_init_voter() - join the elections this module votes in
 * @jeita: this module's state
 *
 * The flip half's election is created here because nothing else votes in it;
 * the others belong to the buck charger and are only joined once it has
 * created them, which is why this is retried from the monitor rather than
 * failing the probe.
 */
static void mca_buckchg_jeita_init_voter(struct mca_buckchg_jeita_dev *jeita)
{
	jeita->flip_fcc_voter =
		mca_create_votable(JEITA_FLIP_FCC_VOTABLE, MCA_VOTE_MIN,
				   mca_buckchg_jeita_flip_charge_limit,
				   JEITA_FLIP_FCC_DEFAULT_MA, jeita);
	if (IS_ERR(jeita->flip_fcc_voter))
		return;

	jeita->en_voter = mca_find_votable(JEITA_EN_VOTABLE);
	if (!jeita->en_voter)
		return;

	jeita->fcc_voter = mca_find_votable(JEITA_FCC_VOTABLE);
	if (!jeita->fcc_voter)
		return;

	jeita->vterm_voter = mca_find_votable(JEITA_VTERM_VOTABLE);
	if (!jeita->vterm_voter)
		return;

	jeita->iterm_voter = mca_find_votable(JEITA_ITERM_VOTABLE);
	if (!jeita->iterm_voter)
		return;

	jeita->voter_ok = 1;
}

static void mca_buckchg_jeita_monitor_work(struct work_struct *work)
{
	struct mca_buckchg_jeita_dev *jeita =
		container_of(to_delayed_work(work), struct mca_buckchg_jeita_dev,
			     monitor_work);

	if (!jeita->online) {
		mca_log_info("adapter is plugout\n");
		return;
	}

	if (!jeita->voter_ok) {
		mca_log_info("voter not ok\n");
		mca_buckchg_jeita_init_voter(jeita);
		goto again;
	}

	mca_buckchg_jeita_update(jeita);

	if (jeita->support_base_flip && !jeita->base_flip_same) {
		mca_buckchg_base_jeita_update(jeita);
		mca_buckchg_flip_jeita_update(jeita);
	}

again:
	queue_delayed_work(system_wq, &jeita->monitor_work,
			   msecs_to_jiffies(JEITA_MONITOR_INTERVAL_MS));
}

/**
 * mca_buckchg_jeita_process_event() - react to something the stack noticed
 * @func:  which event
 * @value: what came with it
 * @data:  this module's state
 *
 * Return: 0, or -EINVAL if the strategy layer called without state.
 */
static int mca_buckchg_jeita_process_event(int func, int value, void *data)
{
	struct mca_buckchg_jeita_dev *jeita = data;

	if (!jeita)
		return -EINVAL;

	mca_log_info("receive event %d, value %d\n", func, value);

	switch (func) {
	case MCA_EVENT_USB_DISCONNECT:
	case MCA_EVENT_WIRELESS_DISCONNECT:
		jeita->online = 0;
		cancel_delayed_work_sync(&jeita->monitor_work);
		break;
	case MCA_EVENT_USB_CONNECT:
	case MCA_EVENT_WIRELESS_CONNECT:
		jeita->online = 1;
		/*
		 * A band is only held by hysteresis against the band that was
		 * in force while the last adapter was connected; after an
		 * unplug the cell has been sitting on its own and the first
		 * reading of the new session stands on its own too.
		 */
		jeita->proc_data.cur_jeita_index = -1;
		jeita->proc_data.max_chg_curr = 0;
		jeita->base_proc_data.cur_jeita_index = -1;
		jeita->base_proc_data.max_chg_curr = 0;
		jeita->flip_proc_data.cur_jeita_index = -1;
		jeita->flip_proc_data.max_chg_curr = 0;

		cancel_delayed_work_sync(&jeita->monitor_work);
		queue_delayed_work(system_wq, &jeita->monitor_work, 0);
		break;
	case MCA_EVENT_BATTERY_DTPT:
		jeita->dtpt_status = value;
		return 0;
	default:
		break;
	}

	return 0;
}

/**
 * mca_strategy_buckchg_jeita_get_status() - what a band allows at this moment
 * @func:   which limit is being asked for
 * @status: filled in with the answer
 * @data:   this module's state
 *
 * The band is looked up afresh rather than reported from the last pass, since
 * a caller may ask between passes.
 *
 * Return: 0.  A cell that cannot be read, or a temperature the tables do not
 * cover, answers zero rather than an error, which is what the rest of the
 * stack was built to expect.
 */
static int mca_strategy_buckchg_jeita_get_status(int func, int *status,
						 void *data)
{
	struct mca_buckchg_jeita_dev *jeita = data;
	struct mca_buckchg_jeita_data *table;
	int size, index, temp, ret;

	if (!status || !jeita)
		return -1;

	switch (func) {
	case STRATEGY_STATUS_TYPE_JEITA_FFC_ITERM:
	case STRATEGY_STATUS_TYPE_JEITA_FFC_VTERM:
		table = jeita->jeita_para.jeita_ffc_data;
		size = jeita->jeita_para.fcc_size;
		break;
	case STRATEGY_STATUS_TYPE_JEITA_NORMAL_VTERM:
		table = jeita->jeita_para.jeita_data;
		size = jeita->jeita_para.size;
		break;
	default:
		return -1;
	}

	ret = strategy_class_fg_ops_get_temperature(&temp);
	if (ret) {
		mca_log_err("get battery temp failed\n");
		*status = 0;
		return 0;
	}
	temp /= 10;

	index = table ? jeita_find_index(table, size, temp) : -1;
	if (index < 0) {
		if (func == STRATEGY_STATUS_TYPE_JEITA_NORMAL_VTERM)
			mca_log_err("can not find jeita para\n");
		else
			mca_log_err("can not find jeita fcc para\n");
		*status = 0;
		return 0;
	}

	if (func == STRATEGY_STATUS_TYPE_JEITA_FFC_ITERM)
		*status = table[index].iterm;
	else
		*status = table[index].vterm;

	return 0;
}

/**
 * mca_jeita_smartchg_delta_fv_callback() - take a float-voltage offset
 * @data:             this module's state
 * @effective_result: how far to lower the float voltage, in millivolts
 *
 * The offset is applied on the next pass rather than voted here, since the
 * float voltage a band allows is what it is subtracted from.
 *
 * Return: 0, or -EINVAL if called without state.
 */
static int mca_jeita_smartchg_delta_fv_callback(void *data,
						int effective_result)
{
	struct mca_buckchg_jeita_dev *jeita = data;
	struct mca_hwid_info *hwid;

	hwid = mca_get_hwid_info();
	if (!jeita)
		return -EINVAL;

	/*
	 * The cells qualified for units sold outside China are rated to a
	 * lower voltage, so however far smart charging asks the float voltage
	 * to be raised, it is not raised past what those cells will take.
	 */
	if (hwid && hwid->platform_version == HARDWARE_PROJECT_O9 &&
	    hwid->country_version == CountryGlobal)
		effective_result = min(effective_result,
				       JEITA_DELTA_FV_GLOBAL_MAX_MV);

	jeita->smartchg_data.delta_fv = effective_result;
	mca_log_err("effective_result: %d\n", effective_result);

	return 0;
}

/**
 * mca_buckchg_jeita_smartchg_baa_update_jeita_data() - rewrite one band
 * @jeita_data: the band to rewrite
 * @para:       what battery-aware adjustment has decided for it
 *
 * Only the float voltage and the finish current move.  The band's temperature
 * window is the cell chemistry's and stays where the board put it; the window
 * @para carries is logged beside the band's so the two can be seen to agree.
 *
 * The top step of the band's voltage table is moved with the float voltage,
 * since that step exists to taper the last few millivolts and has nothing to
 * taper towards if it is left where the old float voltage was.
 *
 * Return: 0, or -EINVAL if either side is missing.
 */
static int
mca_buckchg_jeita_smartchg_baa_update_jeita_data(struct mca_buckchg_jeita_data *jeita_data,
						 struct smart_batt_jeita_term_para *para)
{
	struct mca_buckchg_jeita_volt_para *volt_para;
	int last;

	if (!jeita_data || !para) {
		mca_log_err("data or baa_para is NULL\n");
		return -EINVAL;
	}

	mca_log_info("jeita_term_para idx: %d, temp: %d:%d => %d:%d, vterm: %d => %d, iterm: %d => %d\n",
		     para->t_range.idx, para->t_range.min,
		     para->t_range.max, jeita_data->temp_low,
		     jeita_data->temp_high, jeita_data->vterm, para->vterm,
		     jeita_data->iterm, para->iterm);

	jeita_data->vterm = para->vterm;
	jeita_data->iterm = para->iterm;

	volt_para = &jeita_data->volt_para;
	if (volt_para->size < 1)
		return 0;

	last = volt_para->size - 1;
	mca_log_info("volt_data[%d].voltage: %d => %d\n", last,
		     volt_para->volt_data[last].voltage, jeita_data->vterm);
	volt_para->volt_data[last].voltage = jeita_data->vterm;

	return 0;
}

/**
 * mca_buckchg_jeita_smartchg_update_baa_para() - replace the bands wholesale
 * @data:        this module's state
 * @baa_para:    the new bands, fast-charge ones first
 * @ffc_size:    how many fast-charge bands are in @baa_para
 * @normal_size: how many normal ones follow them
 *
 * Battery-aware adjustment measures how the cell has actually aged and redraws
 * the bands around it, which is why the tables are written into rather than
 * replaced: the voltage sub-tables they point at are still the board's.
 *
 * Return: 0, or a negative error.
 */
static int mca_buckchg_jeita_smartchg_update_baa_para(void *data, char *baa_para,
						      int ffc_size,
						      int normal_size)
{
	struct mca_buckchg_jeita_dev *jeita = data;
	struct smart_batt_jeita_term_para *para;
	int i, ret;

	if (!jeita || !baa_para) {
		mca_log_err("data or baa_para is NULL\n");
		return -EINVAL;
	}

	para = (struct smart_batt_jeita_term_para *)baa_para;

	for (i = 0; i < ffc_size && i < jeita->jeita_para.fcc_size; i++) {
		ret = mca_buckchg_jeita_smartchg_baa_update_jeita_data(
			&jeita->jeita_para.jeita_ffc_data[i], &para[i]);
		if (ret)
			return ret;
	}

	for (i = 0; i < normal_size && i < jeita->jeita_para.size; i++) {
		ret = mca_buckchg_jeita_smartchg_baa_update_jeita_data(
			&jeita->jeita_para.jeita_data[i], &para[ffc_size + i]);
		if (ret)
			return ret;
	}

	/*
	 * The band in force was chosen against the old windows, so the next
	 * pass starts over rather than holding to it.
	 */
	jeita->baacfg_update = true;

	return 0;
}

static struct mca_smartchg_if_ops jeita_smartchg_ops = {
	.type		= MCA_SMARTCHG_IF_CHG_TYPE_JEITA,
	.set_delta_fv	= mca_jeita_smartchg_delta_fv_callback,
	.update_baa_para = mca_buckchg_jeita_smartchg_update_baa_para,
};

/**
 * mca_buckchg_parse_volt_para() - read a band's battery-voltage table
 * @np:        the node the property is on
 * @prop:      the property naming the table
 * @volt_para: filled in with the steps
 *
 * A band without a voltage table is not an error; most bands do not have one.
 *
 * Return: 0, or a negative error.
 */
static noinline int mca_buckchg_parse_volt_para(struct device_node *np, const char *prop,
					       struct mca_buckchg_jeita_volt_para *volt_para)
{
	int count, rows, i, ret;
	const char *str;

	/* A band with no table of its own names the property "null". */
	if (!strcmp(prop, "null")) {
		mca_log_info("no need parse volt para\n");
		return 0;
	}

	/* The helper counts strings, not rows. */
	count = mca_parse_dts_count_strings(np, prop, JEITA_VOLT_PARA_MAX_ROWS,
					    BUCKCHG_VOLTAGE_PARA_MAX);
	if (count <= 0) {
		mca_log_err("parse %s failed\n", prop);
		return count ? count : -EINVAL;
	}

	rows = count / BUCKCHG_VOLTAGE_PARA_MAX;

	volt_para->volt_data = kcalloc(rows, sizeof(*volt_para->volt_data),
				       GFP_KERNEL);
	if (!volt_para->volt_data) {
		mca_log_err("volt para no mem\n");
		return -ENOMEM;
	}

	for (i = 0; i < count; i++) {
		int *row = (int *)&volt_para->volt_data[i / BUCKCHG_VOLTAGE_PARA_MAX];

		ret = mca_parse_dts_string_index(np, prop, i, &str);
		if (ret) {
			mca_log_err("parse %s failed\n", prop);
			return ret;
		}

		ret = kstrtoint(str, 10, &row[i % BUCKCHG_VOLTAGE_PARA_MAX]);
		if (ret) {
			mca_log_err("parse %s failed\n", prop);
			kfree(volt_para->volt_data);
			volt_para->volt_data = NULL;
			return ret;
		}
	}

	for (i = 0; i < rows; i++)
		mca_log_debug("volt_para %d %d\n", volt_para->volt_data[i].voltage,
			      volt_para->volt_data[i].max_current);

	volt_para->size = rows;

	return 0;
}

/**
 * jeita_parse_table() - read one jeita table
 * @np:   the node the property is on
 * @prop: the property naming the table
 * @out:  filled in with the bands
 * @size: filled in with how many there are
 * @what: what to call the table in the log
 *
 * Return: 0, or a negative error.
 */
static int jeita_parse_table(struct device_node *np, const char *prop,
			     struct mca_buckchg_jeita_data **out, int *size,
			     const char *what)
{
	struct mca_buckchg_jeita_data *data;
	int count, rows, i, j, ret;
	const char *str;
	int val;

	/* The helper counts strings, not rows. */
	count = mca_parse_dts_count_strings(np, prop, JEITA_TEMP_PARA_MAX_ROWS,
					    JEITA_TEMP_PARA_MAX);
	if (count <= 0) {
		mca_log_err("parse jeita failed\n");
		return count ? count : -EINVAL;
	}

	rows = count / JEITA_TEMP_PARA_MAX;

	data = kcalloc(rows, sizeof(*data), GFP_KERNEL);
	if (!data) {
		mca_log_err("temp para no mem\n");
		return -ENOMEM;
	}

	for (i = 0; i < rows; i++) {
		int *row = (int *)&data[i];

		/*
		 * Every column but the last is a number; the last names the
		 * property holding that band's voltage table, so it is read
		 * as the string it is.
		 */
		for (j = 0; j < JEITA_TEMP_PARA_VOLT_PARA_NAME; j++) {
			ret = mca_parse_dts_string_index(np, prop,
							 i * JEITA_TEMP_PARA_MAX + j,
							 &str);
			if (!ret)
				ret = kstrtoint(str, 0, &val);
			if (ret) {
				mca_log_err("parse jeita failed\n");
				goto err;
			}
			row[j] = val;
		}

		ret = mca_parse_dts_string_index(np, prop,
						 i * JEITA_TEMP_PARA_MAX +
						 JEITA_TEMP_PARA_VOLT_PARA_NAME,
						 &str);
		if (ret) {
			mca_log_err("parse jeita failed\n");
			goto err;
		}

		ret = mca_buckchg_parse_volt_para(np, str, &data[i].volt_para);
		if (ret)
			goto err;

		mca_log_info("%s %d %d %d %d %d %d %d %d\n", what,
			     data[i].temp_low, data[i].temp_high,
			     data[i].low_temp_hys, data[i].high_temp_hys,
			     data[i].max_current, data[i].vterm, data[i].iterm,
			     data[i].volt_para.size);
	}

	*out = data;
	*size = rows;

	return 0;

err:
	for (i = 0; i < rows; i++)
		kfree(data[i].volt_para.volt_data);
	kfree(data);
	return ret;
}

/**
 * mca_buckchg_jeita_parse_para() - read a cell's normal and fast-charge tables
 * @np:         the node the properties are on
 * @prop:       the property naming the normal table
 * @jeita_info: filled in with both
 *
 * The fast-charge table is the same property name with "_ffc" appended, which
 * is a convention rather than a second property in the binding.
 *
 * Return: 0, or a negative error.
 */
static int mca_buckchg_jeita_parse_para(struct device_node *np, const char *prop,
					struct mca_buckchg_jeita_para *jeita_info)
{
	char ffc_prop[64];
	int ret;

	ret = jeita_parse_table(np, prop, &jeita_info->jeita_data,
				&jeita_info->size, "jeita para");
	if (ret)
		return ret;

	snprintf(ffc_prop, sizeof(ffc_prop), "%s%s", prop, "_ffc");
	mca_log_info("new name:%s\n", ffc_prop);

	ret = jeita_parse_table(np, ffc_prop, &jeita_info->jeita_ffc_data,
				&jeita_info->fcc_size, "jeita ffc para");
	if (ret) {
		mca_log_err("parse jeita para ffc failed\n");
		return ret;
	}

	return 0;
}

/**
 * mca_buckchg_jeita_parse_dt() - read this board's tables and margins
 * @jeita: this module's state
 *
 * Which node the tables come from depends on the board: most read the node
 * their own device tree gives, some carry a second set for units sold outside
 * China, and a test fixture reports a fuel gauge of its own and gets a third.
 * The fixture's is only known once the gauge has probed, so this waits for it.
 *
 * Return: 0, or a negative error.
 */
static int mca_buckchg_jeita_parse_dt(struct mca_buckchg_jeita_dev *jeita)
{
	struct device_node *np = jeita->dev->of_node;
	struct device_node *para_np = np;
	struct mca_hwid_info *hwid = mca_get_hwid_info();
	const char *fg_name;
	int ret;

	if (!np) {
		mca_log_err("node in null\n");
		return -ENODEV;
	}

	mca_parse_dts_u32(np, "vbat_high_hyst", &jeita->vbat_high_hyst,
			  JEITA_VBAT_HYST_DEFAULT);
	mca_parse_dts_u32(np, "vbat_low_hyst", &jeita->vbat_low_hyst,
			  JEITA_VBAT_HYST_DEFAULT);
	mca_parse_dts_u32(np, "vbat_low_cold_hyst", &jeita->vbat_low_cold_hyst,
			  JEITA_VBAT_HYST_DEFAULT);
	mca_parse_dts_u32(np, "base_vbat_low_hyst", &jeita->base_vbat_low_hyst,
			  JEITA_VBAT_HYST_DEFAULT);
	mca_parse_dts_u32(np, "flip_vbat_low_hyst", &jeita->flip_vbat_low_hyst,
			  JEITA_VBAT_HYST_DEFAULT);
	mca_parse_dts_u32(np, "jeita_hot_termination_hyst",
			  &jeita->jeita_hot_termination_hyst, 0);

	/*
	 * Units sold outside China ship a different cell, and their tables
	 * live in a node of their own.  A board that carries both still uses
	 * the ordinary one at home.
	 */
	jeita->has_gbl_batt_para = !!of_find_property(np, "has-global-batt-para",
						      NULL);
	if (jeita->has_gbl_batt_para && hwid && hwid->country_version)
		para_np = of_find_node_by_name(NULL,
					       "mca_buckchg_jeita_gbl_para");

	jeita->has_tmp_batt_para = !!of_find_property(np, "has-tmp-batt-para",
						      NULL);
	if (jeita->has_tmp_batt_para) {
		ret = platform_fg_ops_get_device_name(FG_IC_MASTER, &fg_name);
		if (ret) {
			mca_log_err("get device name fail, wait for it\n");
			return -EPROBE_DEFER;
		}

		mca_log_err("project O9 tmp test: device name: %s\n", fg_name);
		if (!strcmp(fg_name, JEITA_TMP_TEST_FG_NAME))
			para_np = of_find_node_by_name(NULL,
						       "mca_buckchg_jeita_tmp_para");
	}

	if (!para_np) {
		mca_log_err("node in null\n");
		return -ENODEV;
	}

	ret = mca_buckchg_jeita_parse_para(para_np, "jeita_para",
					   &jeita->jeita_para);
	if (ret)
		return ret;

	jeita->support_base_flip = !!of_find_property(np, "support-base-flip",
						      NULL);
	jeita->base_flip_same = !!of_find_property(np, "base-flip-same", NULL);
	if (!jeita->support_base_flip || jeita->base_flip_same)
		return 0;

	mca_log_err("support base flip, start config buckchg jeita\n");

	ret = mca_buckchg_jeita_parse_para(para_np, "base_jeita_para",
					   &jeita->base_jeita_para);
	if (ret)
		return ret;

	return mca_buckchg_jeita_parse_para(para_np, "flip_jeita_para",
					    &jeita->flip_jeita_para);
}

static int mca_buckchg_jeita_probe(struct platform_device *pdev)
{
	static int parse_dt_cnt;
	int usb_online = 0;
	int wls_online = 0;
	struct mca_buckchg_jeita_dev *jeita;
	int ret;

	jeita = devm_kzalloc(&pdev->dev, sizeof(*jeita), GFP_KERNEL);
	if (!jeita)
		return -ENOMEM;

	jeita->dev = &pdev->dev;
	platform_set_drvdata(pdev, jeita);

	ret = mca_buckchg_jeita_parse_dt(jeita);
	if (ret) {
		mca_log_err("parse dts failed, ret: %d\n", ret);
		/*
		 * A malformed row is not worth refusing to load over: the
		 * bands that did parse still protect the cell.  Anything
		 * else means the tables were not there to read yet, which
		 * is worth waiting for.
		 */
		if (ret != -1) {
			dev_err(&pdev->dev, "%s buckchg_jeita parse_dt_cnt = %d\n",
				__func__, ++parse_dt_cnt);
			msleep(JEITA_PARSE_RETRY_DELAY_MS);
			return parse_dt_cnt <= JEITA_PARSE_RETRY_MAX ?
				-EPROBE_DEFER : -1;
		}
	}

	jeita->proc_data.cur_jeita_index = -1;
	jeita->base_proc_data.cur_jeita_index = -1;
	jeita->flip_proc_data.cur_jeita_index = -1;

	INIT_DELAYED_WORK(&jeita->monitor_work, mca_buckchg_jeita_monitor_work);

	mca_buckchg_jeita_init_voter(jeita);

	ret = mca_strategy_ops_register(STRATEGY_FUNC_TYPE_JEITA,
					mca_buckchg_jeita_process_event,
					mca_strategy_buckchg_jeita_get_status,
					NULL, jeita);
	if (ret)
		return ret;

	jeita_smartchg_ops.data = jeita;
	ret = mca_smartchg_if_ops_register(&jeita_smartchg_ops);
	if (ret)
		return ret;

	/*
	 * The charger may already have been attached when this module
	 * loaded, in which case no connect event is coming and nothing else
	 * would ever start the monitor.
	 */
	mca_strategy_func_get_status(STRATEGY_FUNC_TYPE_BUCK_CHARGE,
				     STRATEGY_STATUS_TYPE_ONLINE, &usb_online);
	mca_strategy_func_get_status(STRATEGY_FUNC_TYPE_BASIC_WIRELESS,
				     STRATEGY_STATUS_TYPE_ONLINE, &wls_online);

	if (usb_online || wls_online) {
		jeita->online = 1;
		queue_delayed_work(system_wq, &jeita->monitor_work, 0);
	} else if (!jeita->voter_ok) {
		queue_delayed_work(system_wq, &jeita->monitor_work,
				   JEITA_PROBE_DELAY_JIFFIES);
	}

	g_jeita = jeita;
	mca_log_err("probe ok\n");

	return 0;
}

static int mca_buckchg_jeita_remove(struct platform_device *pdev)
{
	struct mca_buckchg_jeita_dev *jeita = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&jeita->monitor_work);
	g_jeita = NULL;

	return 0;
}

static void mca_buckchg_jeita_shutdown(struct platform_device *pdev)
{
	struct mca_buckchg_jeita_dev *jeita = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&jeita->monitor_work);
}

static const struct of_device_id mca_buckchg_jeita_match[] = {
	{ .compatible = "mca,buckchg_jeita" },
	{ }
};
MODULE_DEVICE_TABLE(of, mca_buckchg_jeita_match);

static struct platform_driver mca_buckchg_jeita_driver = {
	.driver = {
		.name		= "mca_buckchg_jeita",
		.of_match_table	= mca_buckchg_jeita_match,
	},
	.probe		= mca_buckchg_jeita_probe,
	.remove		= mca_buckchg_jeita_remove,
	.shutdown	= mca_buckchg_jeita_shutdown,
};
module_platform_driver(mca_buckchg_jeita_driver);

MODULE_DESCRIPTION("MCA buck charger JEITA limits");
MODULE_LICENSE("GPL");
