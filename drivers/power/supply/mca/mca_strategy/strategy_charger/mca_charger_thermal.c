// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2025 The LineageOS Project
 *
 * Charging within a thermal budget.  See
 * include/mca/common/mca_charger_thermal.h.
 *
 * The thermal framework hands down one number -- how far to back off -- and
 * this turns it into a limit for every way the phone can be charged, because
 * the same amount of heat comes from very different currents depending on
 * whether the charge is going through the buck, through a charge pump, or in
 * over a coil.  Each of those has an election of its own, and this module is
 * one voter in each.
 */

#define MCA_LOG_TAG "mca_thermal"

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/ktime.h>
#include "inc/mca_charger_thermal.h"
#include <mca/common/mca_voter.h>
#include <mca/common/mca_event.h>
#include <mca/common/mca_log.h>
#include <mca/common/mca_parse_dts.h>
#include <mca/platform/platform_fg_ic_ops.h>
#include <mca/protocol/protocol_class.h>
#include <mca/protocol/protocol_pd_class.h>
#include <mca/smartchg/smart_chg_class.h>
#include <mca/common/mca_smem.h>
#include <mca/strategy/strategy_class.h>
#include <mca/common/mca_sysfs.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/thermal.h>
#include <linux/time64.h>
#include <linux/workqueue.h>

/* How many thermal levels the tables describe. */
#define MCA_THERMAL_LEVEL_MAX		16

/* How many cells a board may name as allowed to take a current from userspace. */
#define MCA_THERMAL_SIC_SELECT_MAX	2

/* Who is voting, as the votables record it. */
#define MCA_THERMAL_VOTER		"mca_thermal"
#define MCA_WIRELESS_THERMAL_VOTER	"mca_wireless_thermal"

/* How long to wait before looking for the elections again. */
#define MCA_THERMAL_VOTER_RETRY_JIFFIES	250

/*
 * Before the clock has been set, wall time is still counting from the epoch.
 * A reading this small means the phone has only just started.
 */
#define MCA_THERMAL_BOOT_WINDOW_SEC	60

/* The wireless strategies' own code for "the thermal level moved". */
#define STRATEGY_PROCESS_THERMAL_LEVEL	15

/* What the fast-charge path is asked to cap itself at while it starts up. */
#define MCA_THERMAL_SIC_DEFAULT_FCC	6000

/* Which charging mode a level is being applied to. */
enum mca_thermal_mode_ele {
	THERMAL_MODE_BUCK_5V_IN = 0,
	THERMAL_MODE_BUCK_9V_IN,
	THERMAL_MODE_BUCK_5V_ICH,
	THERMAL_MODE_BUCK_9V_ICH,
	THERMAL_MODE_DIV1_SINGLE_CURR,
	THERMAL_MODE_DIV1_MULTI_CURR,
	THERMAL_MODE_DIV2_SINGLE_CURR,
	THERMAL_MODE_DIV2_MULTI_CURR,
	THERMAL_MODE_DIV4_SINGLE_CURR,
	THERMAL_MODE_DIV4_MULTI_CURR,
	THERMAL_MODE_MAX,
};

/* The same, for the wireless paths, which are told apart by what they proved. */
enum mca_wireless_thermal_mode_ele {
	THERMAL_MODE_WIRELESS_BPP_IN = 0,
	THERMAL_MODE_WIRELESS_BPPQC2_IN,
	THERMAL_MODE_WIRELESS_BPPQC3_IN,
	THERMAL_MODE_WIRELESS_EPP_IN,
	THERMAL_MODE_WIRELESS_AUTHEN_20W,
	THERMAL_MODE_WIRELESS_AUTHEN_30W,
	THERMAL_MODE_WIRELESS_AUTHEN_50W,
	THERMAL_MODE_WIRELESS_AUTHEN_80W,
	THERMAL_MODE_WIRELESS_AUTHEN_VOICE_BOX,
	THERMAL_MODE_WIRELESS_AUTHEN_MAGNET_30W,
	THERMAL_MODE_WIRELESS_MAX,
};

/* What userspace can look at and set. */
enum mca_thermal_sysfs_attr_ele {
	MCA_THERMAL_WIRED_CHG_CURR,
	MCA_THERMAL_WIRED_CHG_CURR2,
	MCA_THERMAL_WIRED_CTRL_LIMIT,
	MCA_THERMAL_WIRED_THERMAL_REMOVE,
	MCA_THERMAL_WIRELESS_CHG_CURR,
	MCA_THERMAL_WIRELESS_CTRL_LIMIT,
	MCA_THERMAL_WLS_QUICK_CHG_CTRL_LIMIT,
	MCA_THERMAL_WIRELESS_THERMAL_REMOVE,
	MCA_THERMAL_WIRELESS_MAG_CTRL_LIMIT,
	MCA_THERMAL_ATTR_MAX,
};

/**
 * struct mca_thermal_ctrl_info - where one charging path currently stands
 * @limit_level:          how many levels its table has
 * @voter_ok:             every election for this path has been found
 * @sic_chg_curr:         the current userspace last asked this path to take
 * @sic_init_fcc:         what the adapter is rated to give it
 * @chg_curr:             the current in force, in milliamps
 * @cur_level:            the level the thermal framework asked for
 * @magnet_limit_level:   the level to use on a magnetic pad instead
 * @quickchg_limit_level: the level to use while fast charging instead
 * @thermal_remove:       the limits are lifted for a test
 * @thermal_removed:      and the votes have already been withdrawn
 */
struct mca_thermal_ctrl_info {
	int	limit_level;
	int	voter_ok;
	int	sic_chg_curr;
	int	sic_init_fcc;
	int	chg_curr;
	int	cur_level;
	int	magnet_limit_level;
	int	quickchg_limit_level;
	int	thermal_remove;
	bool	thermal_removed;
};

/**
 * struct mca_thermal_data - one thermal level, for every wired mode
 * @buck_5v_in:        input current at five volts, in milliamps
 * @buck_9v_in:        input current at nine volts
 * @buck_5v_ich:       charge current at five volts
 * @buck_9v_ich:       charge current at nine volts
 * @div1_single_curr:  one charge pump, not dividing
 * @div1_multi_curr:   several charge pumps, not dividing
 * @div2_single_curr:  one charge pump, halving
 * @div2_multi_curr:   several charge pumps, halving
 * @div4_single_curr:  one charge pump, quartering
 * @div4_multi_curr:   several charge pumps, quartering
 */
struct mca_thermal_data {
	int buck_5v_in;
	int buck_9v_in;
	int buck_5v_ich;
	int buck_9v_ich;
	int div1_single_curr;
	int div1_multi_curr;
	int div2_single_curr;
	int div2_multi_curr;
	int div4_single_curr;
	int div4_multi_curr;
};

/**
 * struct mca_wireless_thermal_data - one thermal level, for every pad
 * @wireless_bpp_in:            a plain pad
 * @wireless_bppqc2_in:         a plain pad behind a Quick Charge adapter
 * @wireless_bppqc3_in:         the same, one revision on
 * @wireless_epp_in:            an extended power pad
 * @wireless_authen_20w:        a pad that proved it can supply twenty watts
 * @wireless_authen_30w:        thirty
 * @wireless_authen_50w:        fifty
 * @wireless_authen_80w:        eighty
 * @wireless_authen_voice_box:  a speaker that charges the phone
 * @wireless_authen_magnet_30w: a magnetic thirty watt pad
 */
struct mca_wireless_thermal_data {
	int wireless_bpp_in;
	int wireless_bppqc2_in;
	int wireless_bppqc3_in;
	int wireless_epp_in;
	int wireless_authen_20w;
	int wireless_authen_30w;
	int wireless_authen_50w;
	int wireless_authen_80w;
	int wireless_authen_voice_box;
	int wireless_authen_magnet_30w;
};

/**
 * struct mca_thermal_info - the thermal budget for charging
 * @dev:                     this device
 * @init_voter_work:         finds the elections once the paths have registered
 * @cdev:                    what the thermal framework governs this through
 * @support_wireless:        the board charges wirelessly
 * @support_base_flip:       the board has a second cell in its base
 * @support_mag_wls_thermal: the board tells magnetic pads apart
 * @wireless_phone_level:    the level at which the pad is told to stand down
 * @not_support_sic:         userspace may not set a current on this board
 * @not_support_sic_curr:    what to use instead when it may not
 * @sic_select_cnt:          how many cells userspace may set a current for
 * @sic_select:              which cells those are
 * @batt_cell_name:          which cell this phone was built with
 * @usb_online:              charging by cable
 * @wls_online:              charging over a coil
 * @real_type:               what the attached charger turned out to be
 * @use_magnet:              a magnetic case is attached
 * @use_magnet_tx:           and it is the pad that is magnetic
 * @wls_super_sts:           the wireless path is fast charging
 * @wired_thermal_data:      the wired tables, one row per level
 * @wireless_thermal_data:   the wireless tables
 * @wired_flip_thermal:      the base cell's wired limit, per level
 * @wireless_flip_thermal:   and its wireless one
 * @wired_voter:             the election each wired mode's limit is decided in
 * @wireless_voter:          the same, per wireless mode
 * @flip_voter:              the base cell's wired election
 * @wls_flip_voter:          and its wireless one
 * @wired_ctrl_info:         where the wired path stands
 * @wireless_ctrl_info:      where the wireless path stands
 * @flip_ctrl_info:          where the base cell's wired path stands
 * @wls_flip_ctrl_info:      and its wireless one
 */
struct mca_thermal_info {
	struct device			*dev;
	struct delayed_work		init_voter_work;
	struct thermal_cooling_device	*cdev;
	int				support_wireless;
	int				support_base_flip;
	int				support_mag_wls_thermal;
	int				wireless_phone_level;
	int				not_support_sic;
	int				not_support_sic_curr;
	int				sic_select_cnt;
	const char			*sic_select[MCA_THERMAL_SIC_SELECT_MAX];
	const char			*batt_cell_name;
	int				usb_online;
	int				wls_online;
	int				real_type;
	int				use_magnet;
	int				use_magnet_tx;
	int				wls_super_sts;
	struct mca_thermal_data		wired_thermal_data[MCA_THERMAL_LEVEL_MAX];
	struct mca_wireless_thermal_data wireless_thermal_data[MCA_THERMAL_LEVEL_MAX];
	int				wired_flip_thermal[MCA_THERMAL_LEVEL_MAX];
	int				wireless_flip_thermal[MCA_THERMAL_LEVEL_MAX];
	struct mca_votable		*wired_voter[THERMAL_MODE_MAX];
	struct mca_votable		*wireless_voter[THERMAL_MODE_WIRELESS_MAX];
	struct mca_votable		*flip_voter;
	struct mca_votable		*wls_flip_voter;
	struct mca_thermal_ctrl_info	wired_ctrl_info;
	struct mca_thermal_ctrl_info	wireless_ctrl_info;
	struct mca_thermal_ctrl_info	flip_ctrl_info;
	struct mca_thermal_ctrl_info	wls_flip_ctrl_info;
};

static struct mca_thermal_info *g_wlscharger_thermal_info;

/**
 * mca_get_wls_charger_thermal_remove() - whether the wireless limits are lifted
 * @wls_thermal_remove: filled in
 */
int mca_get_wls_charger_thermal_remove(bool *wls_thermal_remove)
{
	if (!g_wlscharger_thermal_info || !wls_thermal_remove)
		return -EINVAL;

	*wls_thermal_remove =
		!!g_wlscharger_thermal_info->wireless_ctrl_info.thermal_remove;

	return 0;
}
EXPORT_SYMBOL(mca_get_wls_charger_thermal_remove);

static void
mca_wireless_charger_thermal_handle_limit(struct mca_thermal_info *chip);

/**
 * mca_set_wls_charger_thermal_remove() - lift or restore the wireless limits
 * @wls_thermal_remove: whether to lift them
 *
 * A test rig needs the limits out of the way.  Lifting them is remembered
 * rather than silently applied, because a phone that ran hot with them lifted
 * should be readable as such afterwards.
 */
int mca_set_wls_charger_thermal_remove(bool wls_thermal_remove)
{
	struct mca_thermal_info *chip = g_wlscharger_thermal_info;

	if (!chip)
		return -ENODEV;

	chip->wireless_ctrl_info.thermal_remove = wls_thermal_remove;
	mca_log_err("set wireless thermal remove %d\n", wls_thermal_remove);
	mca_wireless_charger_thermal_handle_limit(chip);

	return 0;
}
EXPORT_SYMBOL(mca_set_wls_charger_thermal_remove);

/*
 * Two limits, either of which may be "no limit at all".  Zero is what the
 * tables use for that, so it cannot simply go through min().
 */
static int mca_thermal_min(int a, int b)
{
	if (!a)
		return b;
	if (!b)
		return a;

	return min(a, b);
}

/*
 * A phone coming off the line is charged flat out to test it, and the thermal
 * tables would get in the way of that.  Shared memory says whether this is
 * such a boot, but only while the clock has yet to be set: once real time is
 * running the phone is in a user's hands and the limits are not optional.
 */
static bool mca_charger_thermal_ignore(void)
{
	struct timespec64 ts;
	u32 is_zero_speed = 0;

	/*
	 * Time since boot, not since the epoch: this is an "are we still
	 * starting up" test, and against the wall clock it can never be true.
	 */
	ts = ns_to_timespec64(ktime_get_with_offset(TK_OFFS_BOOT));
	if (ts.tv_sec >= MCA_THERMAL_BOOT_WINDOW_SEC)
		return false;

	get_smem_battery_info(&is_zero_speed);
	if (!is_zero_speed)
		return false;

	mca_log_err("is_zero_speed = %d, ignore thermal...\n", is_zero_speed);

	return true;
}

/*
 * Every wired mode is capped at whichever is smaller: what this thermal level
 * allows it, or what userspace asked the phone to draw.  Level zero means the
 * framework is not asking for anything, so only userspace's number applies.
 */
static void mca_charger_thermal_handle_limit(struct mca_thermal_info *chip)
{
	struct mca_thermal_ctrl_info *ctrl = &chip->wired_ctrl_info;
	const int *row = NULL;
	int limit[THERMAL_MODE_MAX];
	int level;
	int i;

	if (mca_charger_thermal_ignore())
		return;

	if (!chip) {
		mca_log_err("thermal info is NULL\n");
		return;
	}
	if (!ctrl->voter_ok) {
		mca_log_err("voter is not ready\n");
		return;
	}
	if (!chip->usb_online) {
		mca_log_err("charger is not online\n");
		return;
	}

	level = ctrl->cur_level;
	if (level > ctrl->limit_level) {
		mca_log_err("thermal level invalid\n");
		return;
	}

	if (chip->not_support_sic) {
		ctrl->chg_curr = chip->not_support_sic_curr;
		mca_log_info("chg_curr = %d\n", ctrl->chg_curr);
	}

	if (level)
		row = (const int *)&chip->wired_thermal_data[level - 1];

	for (i = 0; i < THERMAL_MODE_MAX; i++)
		limit[i] = row ? mca_thermal_min(row[i], ctrl->chg_curr)
			       : ctrl->chg_curr;

	if (ctrl->thermal_remove) {
		if (ctrl->thermal_removed)
			return;

		for (i = 0; i < THERMAL_MODE_MAX; i++)
			mca_vote(chip->wired_voter[i], MCA_THERMAL_VOTER, 0, 0);

		ctrl->thermal_removed = true;
		mca_log_err("wired thermal_removed: %d\n", 1);
		return;
	}

	if (ctrl->thermal_removed) {
		ctrl->thermal_removed = false;
		mca_log_err("wired thermal_removed: %d\n", 0);
	}

	/*
	 * The input limits are left out here and decided below: which of them
	 * applies depends on the adapter, not on the level.
	 */
	for (i = THERMAL_MODE_BUCK_5V_ICH; i < THERMAL_MODE_MAX; i++)
		mca_vote(chip->wired_voter[i], MCA_THERMAL_VOTER,
			 !!limit[i], limit[i]);

	/*
	 * On an adapter that can hold its own voltage the charge pump carries
	 * the current, and capping the input would throttle a path the heat
	 * is not coming from.  The same goes at level zero, where there is
	 * nothing to cap.
	 */
	if (!level ||
	    chip->real_type == XM_CHARGER_TYPE_HVDCP3_B ||
	    chip->real_type == XM_CHARGER_TYPE_HVDCP3P5 ||
	    chip->real_type == XM_CHARGER_TYPE_PPS ||
	    chip->real_type == XM_CHARGER_TYPE_PD_VERIFY) {
		mca_log_info("real_type = %d wired remove thermal for buck_in\n",
			     chip->real_type);
		mca_vote(chip->wired_voter[THERMAL_MODE_BUCK_5V_IN],
			 MCA_THERMAL_VOTER, 0, 0);
		mca_vote(chip->wired_voter[THERMAL_MODE_BUCK_9V_IN],
			 MCA_THERMAL_VOTER, 0, 0);
		return;
	}

	mca_vote(chip->wired_voter[THERMAL_MODE_BUCK_5V_IN], MCA_THERMAL_VOTER,
		 !!row[THERMAL_MODE_BUCK_5V_IN], row[THERMAL_MODE_BUCK_5V_IN]);
	mca_vote(chip->wired_voter[THERMAL_MODE_BUCK_9V_IN], MCA_THERMAL_VOTER,
		 !!row[THERMAL_MODE_BUCK_9V_IN], row[THERMAL_MODE_BUCK_9V_IN]);
}

/*
 * The base cell of a foldable has one limit per level rather than a table:
 * it is fed through a load switch, so there is only one current to cap.
 */
static void mca_charger_thermal_flip_handle_limit(struct mca_thermal_info *chip)
{
	struct mca_thermal_ctrl_info *ctrl = &chip->flip_ctrl_info;
	int level;

	if (mca_charger_thermal_ignore())
		return;

	if (!chip) {
		mca_log_err("thermal info is NULL\n");
		return;
	}
	if (!ctrl->voter_ok) {
		mca_log_err("voter is not ready\n");
		return;
	}
	if (!chip->usb_online) {
		mca_log_err("charger is not online\n");
		return;
	}

	level = ctrl->cur_level;
	if (level > ctrl->limit_level) {
		mca_log_err("thermal level invalid\n");
		return;
	}

	if (ctrl->thermal_remove) {
		if (ctrl->thermal_removed)
			return;
		mca_vote(chip->flip_voter, MCA_THERMAL_VOTER, 0,
			 chip->wired_flip_thermal[level]);
		ctrl->thermal_removed = true;
		mca_log_err("wired flip thermal_removed: %d\n", 1);
		return;
	}

	if (ctrl->thermal_removed) {
		ctrl->thermal_removed = false;
		mca_log_err("wired flip thermal_removed: %d\n", 0);
	}

	mca_vote(chip->flip_voter, MCA_THERMAL_VOTER, 1,
		 chip->wired_flip_thermal[level]);
}

static void
mca_wireless_charger_thermal_flip_handle_limit(struct mca_thermal_info *chip)
{
	struct mca_thermal_ctrl_info *ctrl = &chip->wls_flip_ctrl_info;
	int level;

	if (mca_charger_thermal_ignore())
		return;

	if (!chip) {
		mca_log_err("thermal info is NULL\n");
		return;
	}
	if (!ctrl->voter_ok) {
		mca_log_err("voter is not ready\n");
		return;
	}
	if (!chip->wls_online) {
		mca_log_err("charger is not online\n");
		return;
	}

	level = ctrl->cur_level;
	if (level > ctrl->limit_level) {
		mca_log_err("thermal level invalid\n");
		return;
	}

	if (ctrl->thermal_remove) {
		if (ctrl->thermal_removed)
			return;
		mca_vote(chip->wls_flip_voter, MCA_WIRELESS_THERMAL_VOTER, 0,
			 chip->wireless_flip_thermal[level]);
		ctrl->thermal_removed = true;
		mca_log_err("wireless flip thermal_removed: %d\n", 1);
		return;
	}

	if (ctrl->thermal_removed) {
		ctrl->thermal_removed = false;
		mca_log_err("wireless flip thermal_removed: %d\n", 0);
	}

	mca_vote(chip->wls_flip_voter, MCA_WIRELESS_THERMAL_VOTER, 1,
		 chip->wireless_flip_thermal[level]);
}

/*
 * The wireless path picks its level from one of three places: the level the
 * framework asked for, a lower one for a magnetic case (which sits between
 * the coils and traps the heat), or a lower one again while fast charging.
 */
static void
mca_wireless_charger_thermal_handle_limit(struct mca_thermal_info *chip)
{
	struct mca_thermal_ctrl_info *ctrl;
	const int *row = NULL;
	int limit[THERMAL_MODE_WIRELESS_MAX];
	int level;
	int i;

	if (!chip) {
		mca_log_err("thermal info is NULL\n");
		return;
	}

	ctrl = &chip->wireless_ctrl_info;
	if (!ctrl->voter_ok) {
		mca_log_err("voter is not ready\n");
		return;
	}

	level = ctrl->cur_level;

	if (chip->support_mag_wls_thermal && chip->use_magnet &&
	    chip->use_magnet_tx) {
		level = ctrl->magnet_limit_level;
		mca_log_err("thermal_levle %d\n", level);
	}

	if (chip->wls_super_sts) {
		level = ctrl->quickchg_limit_level;
		mca_log_err("thermal_levle %d\n", level);
	}

	if (level > ctrl->limit_level) {
		mca_log_err("thermal level invalid\n");
		return;
	}

	/*
	 * Past this level the pad is asked to stop offering the power rather
	 * than the phone being asked to refuse it, which is what actually
	 * stops the coil heating.
	 */
	mca_strategy_func_process(STRATEGY_FUNC_TYPE_QUICK_WIRELESS,
				  STRATEGY_PROCESS_THERMAL_LEVEL,
				  level >= chip->wireless_phone_level);
	mca_strategy_func_process(STRATEGY_FUNC_TYPE_BASIC_WIRELESS,
				  STRATEGY_PROCESS_THERMAL_LEVEL,
				  level >= chip->wireless_phone_level);

	if (!chip->wls_online) {
		mca_log_err("charger is not online\n");
		return;
	}

	if (level)
		row = (const int *)&chip->wireless_thermal_data[level - 1];

	for (i = 0; i < THERMAL_MODE_WIRELESS_MAX; i++)
		limit[i] = row ? mca_thermal_min(row[i], ctrl->chg_curr)
			       : ctrl->chg_curr;

	if (ctrl->thermal_remove) {
		if (ctrl->thermal_removed)
			return;

		for (i = 0; i < THERMAL_MODE_WIRELESS_MAX; i++)
			mca_vote(chip->wireless_voter[i],
				 MCA_WIRELESS_THERMAL_VOTER, 0, 0);

		ctrl->thermal_removed = true;
		mca_log_err("set wireless thermal remove: %d\n", 1);
		return;
	}

	if (ctrl->thermal_removed) {
		ctrl->thermal_removed = false;
		mca_log_err("set wireless thermal remove: %d\n", 0);
	}

	for (i = 0; i < THERMAL_MODE_WIRELESS_MAX; i++)
		mca_vote(chip->wireless_voter[i], MCA_WIRELESS_THERMAL_VOTER,
			 !!limit[i], limit[i]);

	/*
	 * The wireless input election is what decides how much the receiver
	 * asks for, and it is not re-run by a vote that did not change: run
	 * it here so a level that came out the same still takes effect.
	 */
	mca_rerun_election(chip->wireless_voter[THERMAL_MODE_WIRELESS_BPP_IN]);
}

/*
 * Nothing is attached any more, so every limit this module was holding comes
 * off: leaving them in place would cap the next charger before its own
 * strategy had a chance to decide anything.
 */
static void mca_charger_thermal_poweroff_clear_voter(struct mca_thermal_info *chip)
{
	int i;

	mca_log_info("usb disconnect, clear voter\n");

	if (chip->usb_online)
		return;

	for (i = 0; i < THERMAL_MODE_MAX; i++)
		mca_vote(chip->wired_voter[i], MCA_THERMAL_VOTER, 0, 0);

	if (!chip->support_base_flip)
		return;

	mca_vote(chip->flip_voter, MCA_THERMAL_VOTER, 0, 0);
	mca_vote(chip->wls_flip_voter, MCA_THERMAL_VOTER, 0, 0);
}

static void
mca_wireless_charger_thermal_poweroff_clear_voter(struct mca_thermal_info *chip)
{
	int i;

	mca_log_info("wireless disconnect, clear voter\n");

	if (chip->wls_online)
		return;

	for (i = 0; i < THERMAL_MODE_WIRELESS_MAX; i++)
		mca_vote(chip->wireless_voter[i], MCA_WIRELESS_THERMAL_VOTER,
			 0, 0);
}

/*
 * What userspace asked for and what the adapter can give are two different
 * caps; the one that ends up in force is whichever is smaller, and neither
 * being set means there is nothing to cap.
 */
static noinline void mca_charger_thermal_update_chg_curr(struct mca_thermal_info *chip)
{
	struct mca_thermal_ctrl_info *ctrl = &chip->wired_ctrl_info;
	int curr;

	curr = mca_thermal_min(ctrl->sic_chg_curr, ctrl->sic_init_fcc);
	if (!curr) {
		mca_log_info("invalid chg_curr\n");
		return;
	}

	ctrl->chg_curr = curr;
	mca_log_info("chg_curr = %d, sic_init_fcc = %d sic_chg_curr = %d \n",
		     ctrl->chg_curr, ctrl->sic_init_fcc, ctrl->sic_chg_curr);
}

/*
 * How much the adapter is rated for decides where the fast-charge path can
 * start from.  A Quick Charge adapter does not say, so it gets a fixed
 * figure; a PD one is asked, and its answer is mapped onto the same steps
 * the tables are written in.
 */
static noinline void
mca_charger_thermal_sic_initial_chg_curr(struct mca_thermal_info *chip)
{
	struct mca_thermal_ctrl_info *ctrl = &chip->wired_ctrl_info;
	u32 pwr_max = 0;
	u32 max_power = 0;
	int power_max = 0;

	if (chip->real_type < XM_CHARGER_TYPE_HVDCP2) {
		mca_log_info("not support type %d\n", chip->real_type);
		return;
	}

	/*
	 * Every Quick Charge type belongs here, not just the first two: a
	 * QC3 B or 3+ adapter has no PD power to report, so asking one leaves
	 * sic_init_fcc at zero and update_chg_curr declines the update.
	 */
	if (chip->real_type <= XM_CHARGER_TYPE_HVDCP3P5) {
		ctrl->sic_init_fcc = MCA_THERMAL_SIC_DEFAULT_FCC;
		mca_charger_thermal_update_chg_curr(chip);
		if (chip->usb_online)
			mca_charger_thermal_handle_limit(chip);
		mca_log_info("QC3.0 sic_init_fcc %d\n", ctrl->sic_init_fcc);
		return;
	}

	mca_strategy_func_get_status(STRATEGY_FUNC_TYPE_QUICK_CHARGE,
				     STRATEGY_STATUS_TYPE_POWER_MAX,
				     &power_max);
	if (power_max > 67)
		protocol_class_get_adapter_max_power(ADAPTER_PROTOCOL_PD,
						     &max_power);
	else
		protocol_class_get_adapter_pwr_max_power(ADAPTER_PROTOCOL_PD,
							 &pwr_max);

	ctrl->sic_init_fcc = max_power ? max_power : pwr_max;
	mca_charger_thermal_update_chg_curr(chip);

	if (chip->usb_online)
		mca_charger_thermal_handle_limit(chip);
}

static int mca_charger_thermal_get_max_level(struct thermal_cooling_device *cdev,
					     unsigned long *state)
{
	struct mca_thermal_info *chip = cdev->devdata;

	if (!chip)
		return -1;

	/*
	 * One more than the highest level handle_limit() accepts.  The shipped
	 * driver advertises it this way and the thermal HAL is calibrated
	 * against that, so the count is kept even though the top level it
	 * implies is one the driver itself rejects.
	 */
	*state = chip->wired_ctrl_info.limit_level + 1;

	return 0;
}

static int mca_charger_thermal_get_cur_level(struct thermal_cooling_device *cdev,
					     unsigned long *state)
{
	struct mca_thermal_info *chip = cdev->devdata;

	if (!chip)
		return -1;

	*state = chip->wired_ctrl_info.cur_level;

	return 0;
}

static int mca_charger_thermal_set_cur_level(struct thermal_cooling_device *cdev,
					     unsigned long state)
{
	struct mca_thermal_info *chip = cdev->devdata;

	if (!chip)
		return -1;

	chip->wired_ctrl_info.cur_level = state;
	mca_charger_thermal_handle_limit(chip);

	if (chip->support_base_flip) {
		chip->flip_ctrl_info.cur_level = state;
		mca_charger_thermal_flip_handle_limit(chip);
	}

	mca_log_info("tcd set wired level %lu\n", state);

	return 0;
}

static const struct thermal_cooling_device_ops g_mca_charger_tcd_ops = {
	.get_max_state	= mca_charger_thermal_get_max_level,
	.get_cur_state	= mca_charger_thermal_get_cur_level,
	.set_cur_state	= mca_charger_thermal_set_cur_level,
};

static int mca_charger_thermal_wls_super_sts_callback(void *data,
						      int effective_result)
{
	struct mca_thermal_info *chip = data;

	if (!chip)
		return -1;

	chip->wls_super_sts = effective_result;
	mca_wireless_charger_thermal_handle_limit(chip);
	mca_log_info("effective_result: %d\n", effective_result);

	return 0;
}

static const char * const mca_wired_voter_name[THERMAL_MODE_MAX] = {
	[THERMAL_MODE_BUCK_5V_IN]	= "buck_5v_in",
	[THERMAL_MODE_BUCK_9V_IN]	= "buck_9v_in",
	[THERMAL_MODE_BUCK_5V_ICH]	= "buck_5v_ich",
	[THERMAL_MODE_BUCK_9V_ICH]	= "buck_9v_ich",
	[THERMAL_MODE_DIV1_SINGLE_CURR]	= "div1_single",
	[THERMAL_MODE_DIV1_MULTI_CURR]	= "div1_multi",
	[THERMAL_MODE_DIV2_SINGLE_CURR]	= "div2_single",
	[THERMAL_MODE_DIV2_MULTI_CURR]	= "div2_multi",
	[THERMAL_MODE_DIV4_SINGLE_CURR]	= "div4_single",
	[THERMAL_MODE_DIV4_MULTI_CURR]	= "div4_multi",
};

static const char * const mca_wireless_voter_name[THERMAL_MODE_WIRELESS_MAX] = {
	[THERMAL_MODE_WIRELESS_BPP_IN]		 = "wireless_bpp_in",
	[THERMAL_MODE_WIRELESS_BPPQC2_IN]	 = "wireless_bppqc2_in",
	[THERMAL_MODE_WIRELESS_BPPQC3_IN]	 = "wireless_bppqc3_in",
	[THERMAL_MODE_WIRELESS_EPP_IN]		 = "wireless_epp_in",
	[THERMAL_MODE_WIRELESS_AUTHEN_20W]	 = "wireless_auth_20w",
	[THERMAL_MODE_WIRELESS_AUTHEN_30W]	 = "wireless_auth_30w",
	[THERMAL_MODE_WIRELESS_AUTHEN_50W]	 = "wireless_auth_50w",
	[THERMAL_MODE_WIRELESS_AUTHEN_80W]	 = "wireless_auth_80w",
	[THERMAL_MODE_WIRELESS_AUTHEN_VOICE_BOX] = "wireless_auth_voice_box",
	[THERMAL_MODE_WIRELESS_AUTHEN_MAGNET_30W] = "wireless_auth_magnet_30w",
};

static int mca_charger_thermal_init_wired_voter(struct mca_thermal_info *chip)
{
	int i;

	for (i = 0; i < THERMAL_MODE_MAX; i++) {
		chip->wired_voter[i] = mca_find_votable(mca_wired_voter_name[i]);
		if (!chip->wired_voter[i]) {
			mca_log_err("init wire voter failed\n");
			return -EINVAL;
		}
	}

	chip->wired_ctrl_info.voter_ok = 1;

	return 0;
}

static int mca_charger_thermal_init_wireless_voter(struct mca_thermal_info *chip)
{
	int i;

	for (i = 0; i < THERMAL_MODE_WIRELESS_MAX; i++) {
		chip->wireless_voter[i] =
			mca_find_votable(mca_wireless_voter_name[i]);
		if (!chip->wireless_voter[i]) {
			mca_log_err("init wireless voter failed\n");
			return -EINVAL;
		}
	}

	chip->wireless_ctrl_info.voter_ok = 1;

	return 0;
}

static int mca_charger_thermal_init_flip_voter(struct mca_thermal_info *chip)
{
	chip->flip_voter = mca_find_votable("thermal_flip");
	chip->wls_flip_voter = mca_find_votable("wls_thermal_flip");

	if (!chip->flip_voter || !chip->wls_flip_voter) {
		mca_log_err("init flip voter failed\n");
		return -EINVAL;
	}

	chip->flip_ctrl_info.voter_ok = 1;
	chip->wls_flip_ctrl_info.voter_ok = 1;

	return 0;
}

/*
 * The paths below register their elections as they probe, so the elections do
 * not exist until they are all there.  Try again until they are, and apply
 * the level as soon as each set turns up rather than waiting for the rest.
 */
static void mca_charger_thermal_init_voter_work(struct work_struct *work)
{
	struct mca_thermal_info *chip =
		container_of(work, struct mca_thermal_info,
			     init_voter_work.work);
	int ret = 0;

	if (!chip->wired_ctrl_info.voter_ok) {
		if (mca_charger_thermal_init_wired_voter(chip))
			ret = -1;
		else
			mca_charger_thermal_handle_limit(chip);
	}

	if (chip->support_wireless && !chip->wireless_ctrl_info.voter_ok) {
		if (mca_charger_thermal_init_wireless_voter(chip))
			ret = -1;
		else
			mca_wireless_charger_thermal_handle_limit(chip);
	}

	if (chip->support_base_flip && !chip->flip_ctrl_info.voter_ok) {
		if (mca_charger_thermal_init_flip_voter(chip)) {
			ret = -1;
		} else {
			mca_charger_thermal_flip_handle_limit(chip);
			mca_wireless_charger_thermal_flip_handle_limit(chip);
		}
	}

	if (ret)
		queue_delayed_work(system_wq, &chip->init_voter_work,
				   MCA_THERMAL_VOTER_RETRY_JIFFIES);
}

static ssize_t mca_charger_thermal_sysfs_show(struct device *dev,
					      struct device_attribute *attr,
					      char *buf);
static ssize_t mca_charger_thermal_sysfs_store(struct device *dev,
					       struct device_attribute *attr,
					       const char *buf, size_t count);

static struct mca_sysfs_attr_info mca_charger_thermal_sysfs_field_tbl[] = {
	mca_sysfs_attr_rw(mca_charger_thermal_sysfs, 0664,
			  MCA_THERMAL_WIRED_CHG_CURR, wired_chg_curr),
	mca_sysfs_attr_rw(mca_charger_thermal_sysfs, 0664,
			  MCA_THERMAL_WIRED_CHG_CURR2, wired_chg_curr2),
	mca_sysfs_attr_rw(mca_charger_thermal_sysfs, 0664,
			  MCA_THERMAL_WIRED_CTRL_LIMIT, wired_ctrl_limit),
	mca_sysfs_attr_rw(mca_charger_thermal_sysfs, 0664,
			  MCA_THERMAL_WIRED_THERMAL_REMOVE,
			  wired_thermal_remove),
	mca_sysfs_attr_rw(mca_charger_thermal_sysfs, 0664,
			  MCA_THERMAL_WIRELESS_CHG_CURR, wireless_chg_curr),
	mca_sysfs_attr_rw(mca_charger_thermal_sysfs, 0664,
			  MCA_THERMAL_WIRELESS_CTRL_LIMIT, wireless_ctrl_limit),
	mca_sysfs_attr_rw(mca_charger_thermal_sysfs, 0664,
			  MCA_THERMAL_WLS_QUICK_CHG_CTRL_LIMIT,
			  wls_quick_chg_control_limit),
	mca_sysfs_attr_rw(mca_charger_thermal_sysfs, 0664,
			  MCA_THERMAL_WIRELESS_THERMAL_REMOVE,
			  wireless_thermal_remove),
	mca_sysfs_attr_rw(mca_charger_thermal_sysfs, 0664,
			  MCA_THERMAL_WIRELESS_MAG_CTRL_LIMIT,
			  wireless_mag_ctrl_limit),
};

static struct attribute *mca_charger_thermal_sysfs_attrs[
	ARRAY_SIZE(mca_charger_thermal_sysfs_field_tbl) + 1];

static const struct attribute_group mca_charger_thermal_sysfs_attr_group = {
	.attrs = mca_charger_thermal_sysfs_attrs,
};

static ssize_t mca_charger_thermal_sysfs_show(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	struct mca_thermal_info *chip = g_wlscharger_thermal_info;
	struct mca_sysfs_attr_info *info;
	int val;

	info = mca_sysfs_lookup_attr(attr->attr.name,
				     mca_charger_thermal_sysfs_field_tbl,
				     ARRAY_SIZE(mca_charger_thermal_sysfs_field_tbl));
	if (!info || !chip)
		return -EINVAL;

	switch (info->sysfs_attr_name) {
	case MCA_THERMAL_WIRED_CHG_CURR:
	case MCA_THERMAL_WIRED_CHG_CURR2:
		/* Reported in microamps, the unit it is written in. */
		val = chip->wired_ctrl_info.chg_curr * 1000;
		break;
	case MCA_THERMAL_WIRED_CTRL_LIMIT:
		val = chip->wired_ctrl_info.cur_level;
		break;
	case MCA_THERMAL_WIRED_THERMAL_REMOVE:
		val = chip->wired_ctrl_info.thermal_remove;
		break;
	case MCA_THERMAL_WIRELESS_CHG_CURR:
		val = chip->wireless_ctrl_info.chg_curr;
		break;
	case MCA_THERMAL_WIRELESS_CTRL_LIMIT:
		val = chip->wireless_ctrl_info.cur_level;
		break;
	case MCA_THERMAL_WLS_QUICK_CHG_CTRL_LIMIT:
		val = chip->wireless_ctrl_info.quickchg_limit_level;
		break;
	case MCA_THERMAL_WIRELESS_THERMAL_REMOVE:
		val = chip->wireless_ctrl_info.thermal_remove;
		break;
	case MCA_THERMAL_WIRELESS_MAG_CTRL_LIMIT:
		val = chip->wireless_ctrl_info.magnet_limit_level;
		break;
	default:
		return -EINVAL;
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", val);
}

/*
 * Only the cell the board was built with may be given a current from
 * userspace: the figure comes from a table that was measured against one
 * particular cell, and applying it to another is how a phone gets too hot.
 */
static __always_inline bool mca_charger_thermal_sic_allowed(struct mca_thermal_info *chip,
					    int index)
{
	if (chip->not_support_sic)
		return false;

	if (chip->sic_select_cnt < index + 1)
		return false;

	if (!chip->batt_cell_name || !chip->sic_select[index])
		return false;

	return !strncmp(chip->batt_cell_name, chip->sic_select[index],
			strlen(chip->sic_select[index]));
}

static ssize_t mca_charger_thermal_sysfs_store(struct device *dev,
					       struct device_attribute *attr,
					       const char *buf, size_t count)
{
	struct mca_thermal_info *chip = g_wlscharger_thermal_info;
	struct mca_sysfs_attr_info *info;
	int val;

	info = mca_sysfs_lookup_attr(attr->attr.name,
				     mca_charger_thermal_sysfs_field_tbl,
				     ARRAY_SIZE(mca_charger_thermal_sysfs_field_tbl));
	if (!info || !chip)
		return -EINVAL;

	if (!chip->batt_cell_name) {
		platform_fg_ops_get_batt_cell_info(FG_IC_MASTER,
						   &chip->batt_cell_name);
		mca_log_info("batt_cell_name: %s\n", chip->batt_cell_name);
	}

	switch (info->sysfs_attr_name) {
	case MCA_THERMAL_WIRED_CHG_CURR:
		if (!mca_charger_thermal_sic_allowed(chip,
						     MCA_THERMAL_WIRED_CHG_CURR))
			break;
		if (sscanf(buf, "%d", &val) != 1)
			return -EINVAL;
		/* Written in microamps; the tables are in milliamps. */
		chip->wired_ctrl_info.sic_chg_curr = val / 1000;
		mca_log_info("set wired_chg_curr: %d\n", val / 1000);
		mca_charger_thermal_update_chg_curr(chip);
		mca_charger_thermal_handle_limit(chip);
		break;
	case MCA_THERMAL_WIRED_CHG_CURR2:
		/*
		 * A second cell the board may have been built with, named
		 * separately so that either can be given a current without
		 * the other's table being applied to it.
		 */
		if (!mca_charger_thermal_sic_allowed(chip,
						     MCA_THERMAL_WIRED_CHG_CURR2))
			break;
		if (sscanf(buf, "%d", &val) != 1)
			return -EINVAL;
		chip->wired_ctrl_info.sic_chg_curr = val / 1000;
		mca_log_info("set wired_chg_curr2: %d\n", val / 1000);
		mca_charger_thermal_update_chg_curr(chip);
		mca_charger_thermal_handle_limit(chip);
		break;
	case MCA_THERMAL_WIRED_CTRL_LIMIT:
		if (sscanf(buf, "%d", &val) != 1)
			return -EINVAL;
		chip->wired_ctrl_info.cur_level = val;
		mca_log_info("set wired limit level %d\n", val);
		mca_charger_thermal_handle_limit(chip);
		if (chip->support_base_flip) {
			chip->flip_ctrl_info.cur_level = val;
			mca_charger_thermal_flip_handle_limit(chip);
		}
		break;
	case MCA_THERMAL_WIRED_THERMAL_REMOVE:
		if (sscanf(buf, "%d", &val) != 1)
			return -EINVAL;
		chip->wired_ctrl_info.thermal_remove = val;
		mca_log_err("set wired thermal remove: %d\n", val);
		mca_charger_thermal_handle_limit(chip);
		if (chip->support_base_flip) {
			chip->flip_ctrl_info.thermal_remove = val;
			mca_charger_thermal_flip_handle_limit(chip);
		}
		break;
	case MCA_THERMAL_WIRELESS_CHG_CURR:
		if (sscanf(buf, "%d", &val) != 1)
			return -EINVAL;
		chip->wireless_ctrl_info.chg_curr = val / 1000;
		mca_log_info("set wireless chg cur %d\n", val / 1000);
		mca_wireless_charger_thermal_handle_limit(chip);
		break;
	case MCA_THERMAL_WIRELESS_CTRL_LIMIT:
		if (sscanf(buf, "%d", &val) != 1)
			return -EINVAL;
		chip->wireless_ctrl_info.cur_level = val;
		/*
		 * A magnetic case on a pad that is not itself magnetic sits
		 * between the coils and traps the heat, so that combination
		 * is governed by its own limit and this one is not applied.
		 */
		if (chip->support_mag_wls_thermal && chip->use_magnet &&
		    !chip->use_magnet_tx)
			break;
		mca_log_info(" %d %d %d\n", chip->support_mag_wls_thermal,
			     chip->use_magnet, chip->use_magnet_tx);
		mca_log_info("set wireless limit level %d\n", val);
		mca_wireless_charger_thermal_handle_limit(chip);
		if (chip->support_base_flip) {
			chip->wls_flip_ctrl_info.cur_level = val;
			mca_wireless_charger_thermal_flip_handle_limit(chip);
		}
		break;
	case MCA_THERMAL_WLS_QUICK_CHG_CTRL_LIMIT:
		if (sscanf(buf, "%d", &val) != 1)
			return -EINVAL;
		chip->wireless_ctrl_info.quickchg_limit_level = val;
		mca_log_info("set super wireless limit level %d\n", val);
		mca_wireless_charger_thermal_handle_limit(chip);
		break;
	case MCA_THERMAL_WIRELESS_THERMAL_REMOVE:
		if (sscanf(buf, "%d", &val) != 1)
			return -EINVAL;
		chip->wireless_ctrl_info.thermal_remove = val;
		mca_log_err("set wireless thermal remove: %d\n", val);
		mca_wireless_charger_thermal_handle_limit(chip);
		if (chip->support_base_flip) {
			chip->wls_flip_ctrl_info.thermal_remove = val;
			mca_wireless_charger_thermal_flip_handle_limit(chip);
		}
		break;
	case MCA_THERMAL_WIRELESS_MAG_CTRL_LIMIT:
		if (sscanf(buf, "%d", &val) != 1)
			return -EINVAL;
		chip->wireless_ctrl_info.magnet_limit_level = val;
		mca_log_info("%d %d %d\n", chip->support_mag_wls_thermal,
			     chip->use_magnet, chip->use_magnet_tx);
		if (!chip->support_mag_wls_thermal || !chip->use_magnet ||
		    chip->use_magnet_tx)
			break;
		mca_log_info("set wireless mag limit level %d\n", val);
		mca_wireless_charger_thermal_handle_limit(chip);
		break;
	default:
		return -EINVAL;
	}

	return count;
}

static int mca_charger_thermal_process_event(int func, int value, void *data)
{
	struct mca_thermal_info *chip = data;

	if (!chip)
		return -1;

	mca_log_info("receive event %d, value %d\n", func, value);

	switch (func) {
	case MCA_EVENT_USB_DISCONNECT:
	case MCA_EVENT_USB_CONNECT:
		chip->usb_online = value;
		if (!value) {
			mca_charger_thermal_poweroff_clear_voter(chip);
			break;
		}
		mca_charger_thermal_handle_limit(chip);
		if (chip->support_base_flip)
			mca_charger_thermal_flip_handle_limit(chip);
		break;
	case MCA_EVENT_WIRELESS_DISCONNECT:
	case MCA_EVENT_WIRELESS_CONNECT:
		chip->wls_online = value;
		/*
		 * Nothing to apply on attach: the level for a coil comes from
		 * userspace, which writes it as soon as it sees the supply.
		 */
		if (value)
			break;
		mca_wireless_charger_thermal_poweroff_clear_voter(chip);
		if (chip->support_base_flip)
			mca_wireless_charger_thermal_flip_handle_limit(chip);
		break;
	case MCA_EVENT_CHARGE_TYPE_CHANGE:
	case MCA_EVENT_CHARGE_CAP_CHANGE:
		chip->real_type = value;
		if (!chip->not_support_sic)
			mca_charger_thermal_sic_initial_chg_curr(chip);
		break;
	case MCA_EVENT_WIRELESS_MAGNETIC_CASE_INT:
		chip->use_magnet = value;
		mca_log_info("use magnet: %d\n", value);
		mca_wireless_charger_thermal_handle_limit(chip);
		break;
	case MCA_EVENT_WIRELESS_MAGNETIC_TX_INT:
		chip->use_magnet_tx = value;
		mca_log_info("use magnet TX: %d\n", value);
		mca_wireless_charger_thermal_handle_limit(chip);
		break;
	case MCA_EVENT_DEBUG_CTRL_DOUBLE85:
	case MCA_EVENT_DEBUG_CTRL_MEMORY_TEST:
		/*
		 * Both of these run the phone hot on purpose for days, which
		 * is exactly what the limits exist to prevent; the test would
		 * never reach its temperature with them in force.
		 */
		mca_log_info("double85/memory_test remove thermal limit: %d\n",
			     value);
		if (!value)
			break;
		chip->wired_ctrl_info.thermal_remove = 1;
		chip->wireless_ctrl_info.thermal_remove = 1;
		mca_charger_thermal_poweroff_clear_voter(chip);
		mca_wireless_charger_thermal_poweroff_clear_voter(chip);
		break;
	default:
		break;
	}

	return 0;
}

static int mca_charge_thermal_dump_log_head(void *data, char *buf, int size)
{
	return snprintf(buf, size, "W_TL W_SIC   WL_TL WL_STL WL_SIC   ");
}

static int mca_charge_thermal_dump_log_context(void *data, char *buf, int size)
{
	struct mca_thermal_info *chip = data;

	if (!chip)
		return snprintf(buf, size, "%-5d%-8d%-6d%-7d%-9d", -1, -1, -1,
				-1, -1);

	return snprintf(buf, size, "%-5d%-8d%-6d%-7d%-9d",
			chip->wired_ctrl_info.cur_level,
			chip->wired_ctrl_info.chg_curr,
			chip->wireless_ctrl_info.cur_level,
			chip->wireless_ctrl_info.quickchg_limit_level,
			chip->wireless_ctrl_info.chg_curr);
}

static struct mca_log_charge_log_ops mca_charger_thermal_log_ops = {
	.dump_log_head		= mca_charge_thermal_dump_log_head,
	.dump_log_context	= mca_charge_thermal_dump_log_context,
};

static struct mca_smartchg_if_ops g_thermal_smartchg_if_ops = {
	.set_wls_super_sts = mca_charger_thermal_wls_super_sts_callback,
};

/*
 * One row of the property per thermal level, each row as wide as there are
 * modes, so a board that charges differently only needs a different table.
 */
static __always_inline int mca_charger_thermal_parse_data(struct device_node *np,
					  const char *prop, int *dst,
					  int stride, int *levels)
{
	int count;
	int i, j;

	memset(dst, 0, MCA_THERMAL_LEVEL_MAX * stride * sizeof(*dst));

	count = mca_parse_dts_u32_count(np, prop, MCA_THERMAL_LEVEL_MAX,
					stride);
	if (count < 0) {
		mca_log_err("parse %s failed\n", prop);
		return count;
	}

	if (mca_parse_dts_u32_array(np, prop, (u32 *)dst, count)) {
		mca_log_err("parse %s data failed\n", prop);
		return -EINVAL;
	}

	*levels = count / stride;

	for (i = 0; i < *levels; i++) {
		char row[16 * 12];
		int len = 0;

		for (j = 0; j < stride; j++)
			len += scnprintf(row + len, sizeof(row) - len, " %d",
					 dst[i * stride + j]);
		mca_log_debug("[%s]level-%d%s\n", prop, i, row);
	}

	return 0;
}

/* The base cell has a single limit per level rather than a row of them. */
static int mca_charger_thermal_flip_parse_data(struct device_node *np,
					       const char *prop, int *dst,
					       int *levels)
{
	int i;

	/*
	 * One entry per level and the levels are fixed, so the property is
	 * read whole rather than counted first.
	 */
	if (mca_parse_dts_u32_array(np, prop, (u32 *)dst,
				    MCA_THERMAL_LEVEL_MAX)) {
		mca_log_err("parse %s data failed\n", prop);
		return -EINVAL;
	}

	*levels = MCA_THERMAL_LEVEL_MAX;

	for (i = 0; i < MCA_THERMAL_LEVEL_MAX; i++)
		mca_log_err("thermal:%d flip curr:%d\n", i, dst[i]);

	return 0;
}

static int mca_charger_thermal_parse_dt(struct mca_thermal_info *chip)
{
	struct device_node *np = chip->dev->of_node;
	int levels = 0;
	int i;

	if (!np)
		return -ENODEV;

	mca_parse_dts_u32(np, "support_wireless",
			  (u32 *)&chip->support_wireless, 0);
	mca_parse_dts_u32(np, "wireless_phone_level",
			  (u32 *)&chip->wireless_phone_level, 12);

	/*
	 * Two numbers: whether userspace may set a current at all, and what
	 * to use instead when it may not.
	 */
	mca_parse_dts_u32_array(np, "not_support_sic",
				(u32 *)&chip->not_support_sic, 2);

	chip->support_base_flip = !!of_find_property(np, "support-base-flip",
						     NULL);
	chip->support_mag_wls_thermal =
		!!of_find_property(np, "support-wls-mag-thermal", NULL);
	mca_log_err("parse support_mag_wls_thermal %d\n",
		    chip->support_mag_wls_thermal);

	if (mca_charger_thermal_parse_data(np, "wired_thermal",
					   (int *)chip->wired_thermal_data,
					   THERMAL_MODE_MAX, &levels)) {
		mca_log_err("parse wire thermal failed\n");
		return -EINVAL;
	}
	chip->wired_ctrl_info.limit_level = levels;

	if (chip->support_wireless) {
		if (mca_charger_thermal_parse_data(np, "wireless_thermal",
						   (int *)chip->wireless_thermal_data,
						   THERMAL_MODE_WIRELESS_MAX,
						   &levels)) {
			mca_log_err("parse wireless thermal failed\n");
			return -EINVAL;
		}
		chip->wireless_ctrl_info.limit_level = levels;
	}

	if (chip->support_base_flip) {
		if (!mca_charger_thermal_flip_parse_data(np,
							 "wired_flip_thermal",
							 chip->wired_flip_thermal,
							 &levels))
			chip->flip_ctrl_info.limit_level = levels;

		if (!mca_charger_thermal_flip_parse_data(np,
							 "wireless_flip_thermal",
							 chip->wireless_flip_thermal,
							 &levels))
			chip->wls_flip_ctrl_info.limit_level = levels;
	}

	chip->sic_select_cnt = of_property_read_string_array(np,
							     "wired_sic_select",
							     chip->sic_select,
							     ARRAY_SIZE(chip->sic_select));
	if (chip->sic_select_cnt < 0) {
		mca_log_err("get wired_sic_select count failed\n");
		chip->sic_select_cnt = 0;
	}

	for (i = 0; i < chip->sic_select_cnt; i++)
		mca_log_err("wired_sic_select[%d] = %s\n", i,
			    chip->sic_select[i]);

	return 0;
}

static int mca_charger_thermal_probe(struct platform_device *pdev)
{
	struct mca_thermal_info *chip;
	int ret;

	mca_log_info("probe begin\n");

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		mca_log_err("out of memory\n");
		return -ENOMEM;
	}

	chip->dev = &pdev->dev;
	platform_set_drvdata(pdev, chip);

	ret = mca_charger_thermal_parse_dt(chip);
	if (ret)
		return ret;

	INIT_DELAYED_WORK(&chip->init_voter_work,
			  mca_charger_thermal_init_voter_work);

	mca_sysfs_init_attrs(mca_charger_thermal_sysfs_attrs,
			     mca_charger_thermal_sysfs_field_tbl,
			     ARRAY_SIZE(mca_charger_thermal_sysfs_field_tbl));
	mca_sysfs_create_link_group(MCA_SYSFS_DEV_CHARGER, "charger_thermal",
				    chip->dev,
				    &mca_charger_thermal_sysfs_attr_group);

	mca_strategy_ops_register(STRATEGY_FUNC_TYPE_THERMAL,
				  mca_charger_thermal_process_event, NULL,
				  NULL, chip);

	ret = mca_charger_thermal_init_wired_voter(chip);
	if (chip->support_wireless &&
	    mca_charger_thermal_init_wireless_voter(chip))
		ret = -EINVAL;
	if (ret)
		queue_delayed_work(system_wq, &chip->init_voter_work,
				   MCA_THERMAL_VOTER_RETRY_JIFFIES);

	/*
	 * One cooling device covers every mode: the framework asks for one
	 * level, and which currents that level means is decided here rather
	 * than by giving the framework ten knobs it cannot tell apart.
	 */
	chip->cdev = devm_thermal_of_cooling_device_register(chip->dev,
							     chip->dev->of_node,
							     "battery", chip,
							     &g_mca_charger_tcd_ops);
	if (IS_ERR_OR_NULL(chip->cdev)) {
		mca_log_err("register cooling device failed\n");
		return -1;
	}

	g_thermal_smartchg_if_ops.data = chip;
	mca_smartchg_if_ops_register(&g_thermal_smartchg_if_ops);

	mca_log_charge_log_register(MCA_CHARGE_LOG_ID_THERMAL,
				    &mca_charger_thermal_log_ops, chip);

	g_wlscharger_thermal_info = chip;
	mca_log_err("probe end\n");

	return 0;
}

static int mca_charger_thermal_remove(struct platform_device *pdev)
{
	struct mca_thermal_info *chip = platform_get_drvdata(pdev);

	/*
	 * The vendor leaves the retry work queued, which fires into memory
	 * this is about to free.
	 */
	cancel_delayed_work_sync(&chip->init_voter_work);

	mca_sysfs_remove_link_group(MCA_SYSFS_DEV_CHARGER, "charger_thermal",
				    chip->dev,
				    &mca_charger_thermal_sysfs_attr_group);

	g_wlscharger_thermal_info = NULL;

	return 0;
}

static const struct of_device_id match_table[] = {
	{ .compatible = "mca,charger_thermal" },
	{ }
};
MODULE_DEVICE_TABLE(of, match_table);

static struct platform_driver mca_charger_thermal_driver = {
	.driver = {
		.name		= "charger_thermal",
		.of_match_table	= match_table,
	},
	.probe		= mca_charger_thermal_probe,
	.remove		= mca_charger_thermal_remove,
};
module_platform_driver(mca_charger_thermal_driver);

MODULE_DESCRIPTION("MCA charger thermal");
MODULE_LICENSE("GPL");
