// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2025 The LineageOS Project
 *
 * Keeping the connector from burning.  See
 * include/mca/common/mca_connector_antiburn.h.
 */

#define MCA_LOG_TAG "mca_connector_antiburn"

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/kstrtox.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <mca/common/mca_charge_mievent.h>
#include <mca/hardware/hw_connector_antiburn.h>
#include <mca/common/mca_event.h>
#include <mca/common/mca_hwid.h>
#include <mca/common/mca_log.h>
#include <mca/common/mca_parse_dts.h>
#include <mca/platform/platform_buckchg_class.h>
#include <mca/protocol/protocol_class.h>
#include <mca/protocol/protocol_pd_class.h>
#include <mca/common/mca_sysfs.h>
#include <mca/common/mca_workqueue.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/thermal.h>
#include <linux/workqueue.h>

/* One thermistor at the connector, and a second one on boards that have it. */
#define ANTIBURN_NTC_NUM		2

/* How long to wait between readings once something is attached, in ms. */
#define ANTIBURN_FAST_INTERVAL_MS	1000

/*
 * And how often once nothing has been attached and both thermistors have read
 * cool for a while.  The port cannot get hot with nothing in it, so there is no
 * reason to keep waking up at 1 Hz.
 */
#define ANTIBURN_IDLE_INTERVAL_MS	5000
#define ANTIBURN_IDLE_CYCLES		5
#define ANTIBURN_IDLE_TEMP_MAX		55000

/* And how soon to take the next one after being told something changed. */
#define ANTIBURN_KICK_DELAY_MS		250

/*
 * A reading is only worth differentiating if enough time has passed since the
 * last one; below this the quotient is dominated by the timer's own jitter.
 */
#define ANTIBURN_MIN_TIME_GAP_MS	1

/*
 * The connector cannot plausibly be this hot: past here the thermistor is
 * open or shorted, and the reading is not evidence of anything.  Treated as a
 * fault anyway, because a connector whose thermistor has just failed is not a
 * connector to keep charging through.
 */
#define ANTIBURN_TEMP_IMPLAUSIBLE	75000

/* VBUS is still live above this, and has collapsed below this, in uV. */
#define ANTIBURN_VBUS_LIVE_UV		6000000
#define ANTIBURN_VBUS_SAFE_UV		3600000
/*
 * And what counts as still live once the pin has opened.  It is the higher
 * figure: the port is open, so anything the adapter is still pushing has to
 * be getting past a pin that was told to stop it.
 */
#define ANTIBURN_VBUS_STILL_UP_UV	4100000

/* How long to wait for the adapter to act on being asked to drop VBUS. */
#define ANTIBURN_VSAFE0V_SETTLE_MS	200

/* And how long between the steps of the trigger sequence itself. */
#define ANTIBURN_TRIGGER_STEP_MS	50

/* How many times to ask before giving up on VBUS coming down. */
#define ANTIBURN_VSAFE0V_TRIES		3

/* What userspace can look at and force. */
enum connector_antiburn_attr_list {
	CONN_TEMP_1,
	CONN_TEMP_2,
	CONN_ATTR_UNUSED,
	CONN_RESET_VSAFE0V,
	CONN_NTC_ALARM,
	CONN_MOS_CTRL,
	CONN_ATTR_MAX,
};

/**
 * struct connector_antiburn - the connector being watched
 * @dev:                    this device
 * @tzd_conn:               the connector's thermistor
 * @tzd_conn2:              the second one, on boards that have it
 * @thermal_board_nb:       told how warm the board is
 * @debug_nb:               told when a test turns this off
 * @connect_nb:             told when a charger is plugged in
 * @hw_nb:                  told when the port sources or senses a cable
 * @triggered:              a fault is in force
 * @is_reset_vsafe0V:       the port was taken to zero volts to break the short
 * @ntc_alarm:              the connector was found too hot
 * @connector_temp:         the last reading from each thermistor
 * @temp_increase_rate:     how fast each is climbing, in millidegrees a second
 * @fake_connector_temp:    what a test is forcing, per thermistor
 * @thermal_board_temp:     how warm the board is
 * @max_thermal_board_temp: above which a hot connector is just a hot phone
 * @comb_sensorboard_con_trigger_temp: the connector temperature that counts as
 *                          a fault when the board is *not* warm as well
 * @comb_rate_conn_trigger_temp: the temperature that counts as a fault when it
 *                          is climbing fast
 * @trigger_temp:           the temperature that counts as a fault on its own
 * @recharge_temp:          how far it must fall before charging resumes
 * @mos_ctrl_gpio:          the pin that disconnects the connector
 * @otg_detect_en:          watch while supplying something else too
 * @support_soft_antiburn:  the board relies on this driver
 * @support_hw_antiburn:    the board has a pin that cuts the connector
 * @use_double_ntc:         the board has two thermistors
 * @monitor_interval:       how often to look, in milliseconds
 * @max_temp_increase_rate: the fastest climb that is not a fault
 * @thermal_zone_name:      what the thermistor is called
 * @thermal_zone_name2:     and the second one
 * @otg_boost_src:          which boost supplies the port
 * @en_src:                 which enable controls it
 * @support_base_flip:      the board is one of the flip bases
 * @disable_antiburn:       a test has turned this off
 * @support_elaboration_anti_strategy: read the thermistors in millidegrees and
 *                          judge them against the rate as well as the level
 * @otg_plugin_status:      something is being supplied from the port
 * @usb_online:             charging by cable
 * @cid_status:             a cable is detected in the port
 * @monitor_work:           reads the thermistors
 */
struct connector_antiburn {
	struct device			*dev;
	struct thermal_zone_device	*tzd_conn;
	struct thermal_zone_device	*tzd_conn2;
	struct notifier_block		thermal_board_nb;
	struct notifier_block		debug_nb;
	struct notifier_block		connect_nb;
	struct notifier_block		hw_nb;
	int				triggered;
	int				is_reset_vsafe0V;
	int				ntc_alarm;
	int				connector_temp[ANTIBURN_NTC_NUM];
	int				temp_increase_rate[ANTIBURN_NTC_NUM];
	int				fake_connector_temp[ANTIBURN_NTC_NUM];
	int				thermal_board_temp;
	int				max_thermal_board_temp;
	int				comb_sensorboard_con_trigger_temp;
	int				comb_rate_conn_trigger_temp;
	int				trigger_temp;
	int				recharge_temp;
	int				mos_ctrl_gpio;
	int				otg_detect_en;
	int				support_soft_antiburn;
	int				support_hw_antiburn;
	int				use_double_ntc;
	int				monitor_interval;
	int				max_temp_increase_rate;
	const char			*thermal_zone_name;
	const char			*thermal_zone_name2;
	int				otg_boost_src;
	int				en_src;
	int				support_base_flip;
	int				disable_antiburn;
	int				support_elaboration_anti_strategy;
	bool				otg_plugin_status;
	int				usb_online;
	bool				cid_status;
	struct delayed_work		monitor_work;
};

static struct connector_antiburn *g_connector_antiburn;

/*
 * A thermistor that has just started reading again is not to be trusted for
 * one more sample: the previous reading it is differentiated against is
 * whatever was last known, from some unknown time ago, so the rate that comes
 * out of it is meaningless and would trigger on its own.
 */
static bool g_temp_read_failed;
static bool g_temp_read_recovered;

/**
 * connector_antiburn_is_triggered() - whether the connector is in a fault
 *
 * The charging strategies ask before deciding why charging stopped, so that a
 * hot connector is reported as such rather than as a charger fault.
 */
int connector_antiburn_is_triggered(void)
{
	if (!g_connector_antiburn)
		return 0;

	return g_connector_antiburn->triggered;
}
EXPORT_SYMBOL(connector_antiburn_is_triggered);

/*
 * The thermal core reports millidegrees.  Boards on the elaborated strategy
 * keep them, because the rate it judges is too small a number to survive
 * being rounded to whole degrees first; the others round here, once, so that
 * every threshold they are compared against can be written in degrees.
 */
static int connector_antiburn_get_temperature(struct connector_antiburn *conn,
					      int index)
{
	struct thermal_zone_device *tzd;
	int temp = conn->support_elaboration_anti_strategy ? 25000 : 25;
	int rc;

	if (conn->fake_connector_temp[index])
		return conn->fake_connector_temp[index];

	tzd = index ? conn->tzd_conn2 : conn->tzd_conn;

	rc = thermal_zone_get_temp(tzd, &temp);
	if (rc) {
		mca_log_err("iio get temp error, index is %d, connector_temp is %d, ret is %d\n",
			    index, conn->connector_temp[index], rc);
		g_temp_read_failed = true;
		return conn->connector_temp[index];
	}

	if (g_temp_read_failed) {
		g_temp_read_failed = false;
		g_temp_read_recovered = true;
	}

	mca_log_info(" CONNECTOR_PROP_TEMP read %d, index is %d\n", temp, index);

	if (conn->support_elaboration_anti_strategy)
		return temp;

	return temp / 1000;
}

/*
 * How fast the connector is heating tells a short from a warm room: ambient
 * moves in minutes, a short moves in seconds.  Both thermistors are
 * differentiated against the same timestamp, so a slow work cycle stretches
 * both denominators equally.
 */
static void
connector_antiburn_get_temp_increase_rate(struct connector_antiburn *conn)
{
	static ktime_t last_time;
	static int last_temp[ANTIBURN_NTC_NUM];
	static bool started;
	s64 time_gap_ms;
	ktime_t now;
	int i;

	for (i = 0; i < ANTIBURN_NTC_NUM; i++)
		conn->connector_temp[i] =
			connector_antiburn_get_temperature(conn, i);

	if (!started) {
		last_time = ktime_get();
		for (i = 0; i < ANTIBURN_NTC_NUM; i++) {
			last_temp[i] = conn->connector_temp[i];
			conn->temp_increase_rate[i] = 0;
		}
		started = true;
		return;
	}

	now = ktime_get();
	time_gap_ms = ktime_ms_delta(now, last_time);
	if (time_gap_ms < ANTIBURN_MIN_TIME_GAP_MS) {
		mca_log_err("timestamp is error, time_gap is %ld, current time is %ld, last time is %ld\n",
			    (long)time_gap_ms, (long)ktime_to_ms(now),
			    (long)ktime_to_ms(last_time));
		return;
	}

	for (i = 0; i < ANTIBURN_NTC_NUM; i++) {
		int temp_gap = conn->connector_temp[i] - last_temp[i];

		conn->temp_increase_rate[i] =
			(int)div_s64((s64)temp_gap * 1000, time_gap_ms);
		last_temp[i] = conn->connector_temp[i];

		/*
		 * A thermistor that has just come back reads as an enormous
		 * jump.  Throw that one sample away rather than cut the
		 * connector over it.
		 */
		if (conn->temp_increase_rate[i] > conn->max_temp_increase_rate &&
		    g_temp_read_recovered) {
			mca_log_info("temp_gap is %d, current temp is %d, last temp is %d, temp increase rate is %d, index is %d\n",
				     temp_gap, conn->connector_temp[i],
				     last_temp[i],
				     conn->temp_increase_rate[i], i);
			conn->temp_increase_rate[i] = 0;
			g_temp_read_recovered = false;
		}
	}

	last_time = now;
}

static void connector_antiburn_temp_uevent(struct connector_antiburn *conn,
					   int temp)
{
	struct mca_event_notify_data n_data;
	char buf[128];

	mca_log_info("connector temp uevent notify\n");

	n_data.event = buf;
	n_data.event_len = snprintf(buf, sizeof(buf),
				"POWER_SUPPLY_CONNECTOR_TEMP=%d", temp);
	mca_event_report_uevent(&n_data);
}

static void connector_antiburn_ntc_alarm_uevent(struct connector_antiburn *conn,
						int alarm)
{
	struct mca_event_notify_data n_data;
	char buf[128];

	mca_log_info("ntc alarm notify\n");

	n_data.event = buf;
	n_data.event_len = snprintf(buf, sizeof(buf),
				"POWER_SUPPLY_NTC_ALARM=%d", alarm);
	mca_event_report_uevent(&n_data);
}

static void
connector_antiburn_reset_vsafe0V_uevent(struct connector_antiburn *conn,
					int reset)
{
	struct mca_event_notify_data n_data;
	char buf[128];

	mca_log_info("adapter vbus drop 0V notify\n");

	n_data.event = buf;
	n_data.event_len = snprintf(buf, sizeof(buf),
				"POWER_SUPPLY_ADAPTER_RESET_VSAFE0V=%d", reset);
	mca_event_report_uevent(&n_data);
}

/*
 * Cutting the connector while VBUS is still up would arc across whatever is
 * bridging the pins, which is the thing that is burning.  A PD adapter can be
 * told to go to vSafe0V; a QC one cannot, so all that is left is to wait for
 * its own discharge.  Either way the port is only opened once VBUS is down.
 */
static void connector_antiburn_ensure_vbus_sense5V(struct connector_antiburn *conn)
{
	int bus_volt = 0;
	u32 real_type_pd = 0;
	u32 real_type_bc = 0;
	u32 data_out = 0;
	int real_type;
	int i;

	for (i = 0; i < ANTIBURN_VSAFE0V_TRIES; i++) {
		protocol_class_get_adapter_type(ADAPTER_PROTOCOL_PD,
						&real_type_pd);
		protocol_class_get_adapter_type(ADAPTER_PROTOCOL_BC12,
						&real_type_bc);
		real_type = real_type_pd ? real_type_pd : real_type_bc;

		mca_log_err("real_type is %d, bus_volt is %d\n", real_type,
			    bus_volt / 1000);

		if (real_type >= XM_CHARGER_TYPE_HVDCP2 &&
		    real_type <= XM_CHARGER_TYPE_HVDCP3P5) {
			/* Nothing to ask a QC adapter; wait it out. */
			platform_class_buckchg_ops_get_bus_volt(
				MAIN_BUCK_CHARGER, &bus_volt);
			if (bus_volt < ANTIBURN_VBUS_LIVE_UV)
				break;
			msleep(ANTIBURN_VSAFE0V_SETTLE_MS);
			continue;
		}

		if (real_type < XM_CHARGER_TYPE_PD ||
		    real_type > XM_CHARGER_TYPE_PD_VERIFY)
			return;

		protocol_class_pd_request_vdm_cmd(TYPEC_PORT_0,
						  USBPD_UVDM_RESET_VSAFE0V,
						  &data_out, 0);
		msleep(ANTIBURN_VSAFE0V_SETTLE_MS);
		platform_class_buckchg_ops_get_bus_volt(MAIN_BUCK_CHARGER,
							&bus_volt);
		if (bus_volt < ANTIBURN_VBUS_LIVE_UV)
			break;
	}

	if (i == ANTIBURN_VSAFE0V_TRIES || bus_volt > ANTIBURN_VBUS_SAFE_UV)
		return;

	conn->is_reset_vsafe0V = 1;
	connector_antiburn_reset_vsafe0V_uevent(conn, conn->is_reset_vsafe0V);
}

/* Everything that has to happen, in order, once the connector is condemned. */
static __always_inline void connector_antiburn_trigger(struct connector_antiburn *conn)
{
	u32 real_type_pd = 0;
	u32 real_type_bc = 0;
	u32 data_out = 0;
	int real_type;
	int bus_volt = 0;
	int temp = max(conn->connector_temp[0], conn->connector_temp[1]);
	int mievent_data;

	mca_log_err("triggering antiburn conn->usb_online is %d, conn->otg_plugin_status is %d, conn->cid_status is %d\n",
		    conn->usb_online, conn->otg_plugin_status,
		    conn->cid_status);
	mca_log_err("usb_online: %d, otg_plugin_status: %d, conn_therm: %d/%d, temp_increase_rate: %d, thermal_board_temp: %d\n",
		    conn->usb_online, conn->otg_plugin_status,
		    conn->connector_temp[0], conn->connector_temp[1],
		    conn->temp_increase_rate[0], conn->thermal_board_temp);

	conn->triggered = 1;
	conn->ntc_alarm = 1;

	connector_antiburn_temp_uevent(conn, temp);
	connector_antiburn_ntc_alarm_uevent(conn, conn->ntc_alarm);

	protocol_class_get_adapter_type(ADAPTER_PROTOCOL_PD, &real_type_pd);
	protocol_class_get_adapter_type(ADAPTER_PROTOCOL_BC12, &real_type_bc);
	real_type = real_type_pd ? real_type_pd : real_type_bc;
	mca_log_info("real_type:%d, real_type_pd:%d, real_type_bc:%d\n",
		     real_type, real_type_pd, real_type_bc);

	/*
	 * Ask first and check afterwards.  The adapter is given its chance to
	 * stand down before anything else in the stack is told, because the
	 * charging path being torn down under it is what makes it hold VBUS.
	 */
	protocol_class_pd_request_vdm_cmd(TYPEC_PORT_0,
					  USBPD_UVDM_RESET_VSAFE0V,
					  &data_out, 0);
	msleep(ANTIBURN_TRIGGER_STEP_MS);

	mca_event_block_notify(MCA_EVENT_TYPE_HW_INFO,
			       MCA_EVENT_CONN_ANTIBURN_CHANGE, NULL);
	mievent_data = temp;
	mca_charge_mievent_report(CHARGE_DFX_ANTI_BURN_TRIGGERED,
				  &mievent_data, 1);

	msleep(ANTIBURN_VSAFE0V_SETTLE_MS);
	connector_antiburn_ensure_vbus_sense5V(conn);
	platform_class_buckchg_ops_get_bus_volt(MAIN_BUCK_CHARGER, &bus_volt);
	connector_antiburn_reset_vsafe0V_uevent(conn, conn->is_reset_vsafe0V);

	mca_event_block_notify(MCA_EVENT_TYPE_HW_INFO,
			       MCA_EVENT_CONN_ANTIBURN_CHANGE, NULL);
	mca_charge_mievent_report(CHARGE_DFX_ANTI_BURN_TRIGGERED,
				  &mievent_data, 1);

	msleep(ANTIBURN_VSAFE0V_SETTLE_MS);
	connector_antiburn_ensure_vbus_sense5V(conn);

	/*
	 * The boost has to stop before the pin opens, or the port is opened
	 * with its own supply still driving it.
	 */
	if (conn->otg_plugin_status)
		platform_class_buckchg_ops_set_boost_enable(
			MAIN_BUCK_CHARGER,
			(conn->otg_boost_src << 8) | (conn->en_src << 16));

	if (conn->support_hw_antiburn) {
		gpiod_direction_output_raw(gpio_to_desc(conn->mos_ctrl_gpio), 1);
		mca_log_err("triggering hw anti burn\n");
	}

	msleep(ANTIBURN_TRIGGER_STEP_MS);
	platform_class_buckchg_ops_get_bus_volt(MAIN_BUCK_CHARGER, &bus_volt);

	/*
	 * VBUS still being up after the pin has opened means the pin did not
	 * do what it was told, which is worth filing separately: the phone is
	 * now relying on a protection that is not working.
	 */
	if (bus_volt > ANTIBURN_VBUS_STILL_UP_UV)
		mca_charge_mievent_report(CHARGE_DFX_ANTIBURN_ERR, NULL, 0);
}

/* And everything that undoes it once the connector has cooled. */
static __always_inline void connector_antiburn_recover(struct connector_antiburn *conn)
{
	int temp = max(conn->connector_temp[0], conn->connector_temp[1]);

	mca_log_err("recovery antiburn conn->usb_online is %d, conn->otg_plugin_status is %d, conn->cid_status is %d\n",
		    conn->usb_online, conn->otg_plugin_status,
		    conn->cid_status);

	conn->triggered = 0;
	conn->ntc_alarm = 0;
	conn->is_reset_vsafe0V = 0;

	connector_antiburn_temp_uevent(conn, temp);
	connector_antiburn_reset_vsafe0V_uevent(conn, conn->is_reset_vsafe0V);

	if (conn->support_hw_antiburn) {
		gpiod_direction_output_raw(gpio_to_desc(conn->mos_ctrl_gpio), 0);
		mca_log_err("close hw anti burn\n");
	}

	mca_event_block_notify(MCA_EVENT_TYPE_HW_INFO,
			       MCA_EVENT_CONN_ANTIBURN_CHANGE, NULL);
	mca_charge_mievent_set_state(MIEVENT_STATE_END,
				     CHARGE_DFX_ANTI_BURN_TRIGGERED);
}

/*
 * The level alone is a poor test: a connector can be hot because the phone is
 * hot, and it can be about to be dangerous while still merely warm.  So three
 * things are asked, and any one of them condemns the connector:
 *
 *   - it is past the temperature nothing explains;
 *   - it is warm while the board is not, so the heat is local to it;
 *   - it is warm and climbing faster than heating through a case can.
 *
 * None of them apply unless the port is carrying power, because a connector
 * with nothing across it cannot be shorted by anything that matters.
 */
static void connector_antiburn_check_status_v2(struct connector_antiburn *conn)
{
	int temp = max(conn->connector_temp[0], conn->connector_temp[1]);
	bool hot;
	int i;

	protocol_class_pd_get_otg_plugin_status(TYPEC_PORT_0,
						&conn->otg_plugin_status);
	platform_class_buckchg_ops_get_online(MAIN_BUCK_CHARGER,
					      &conn->usb_online);
	protocol_class_pd_get_cid_status(TYPEC_PORT_0, &conn->cid_status);

	mca_log_info("connector_temp:%d:%d, temp_increase_rate:%d:%d, triggered:%d, cid_status:%d, otg_plugin_status: %d, usb_online:%d\n",
		     conn->connector_temp[0], conn->connector_temp[1],
		     conn->temp_increase_rate[0], conn->temp_increase_rate[1],
		     conn->triggered, conn->cid_status,
		     conn->otg_plugin_status, conn->usb_online);

	/*
	 * The lower combined threshold is for a connector that is hot while
	 * the board is not, which says the heat is local to the connector
	 * rather than the phone being warm all over.  That reading comes from
	 * a thermal module this build does not have, so without it fall back
	 * to the connector's own threshold: taking "no reading" for "board is
	 * cold" would cut charging off on any phone that is merely warm.
	 */
	hot = temp > ANTIBURN_TEMP_IMPLAUSIBLE ||
	      temp >= conn->trigger_temp ||
	      (conn->thermal_board_temp > 0 &&
	       temp >= conn->comb_sensorboard_con_trigger_temp &&
	       conn->thermal_board_temp <= conn->max_thermal_board_temp);

	for (i = 0; !hot && i < ANTIBURN_NTC_NUM; i++)
		hot = conn->connector_temp[i] >=
			      conn->comb_rate_conn_trigger_temp &&
		      conn->temp_increase_rate[i] >=
			      conn->max_temp_increase_rate;

	if (hot && (conn->otg_plugin_status || conn->usb_online) &&
	    !conn->disable_antiburn && !conn->triggered) {
		connector_antiburn_trigger(conn);
		return;
	}

	/*
	 * Recovering needs both thermistors cool *and* steady, and the cable
	 * gone: putting the port back while whatever shorted it is still in
	 * there just starts the cycle again.
	 */
	for (i = 0; i < ANTIBURN_NTC_NUM; i++) {
		if (conn->connector_temp[i] >= conn->recharge_temp ||
		    conn->temp_increase_rate[i] >= conn->max_temp_increase_rate)
			return;
	}

	if (conn->triggered && !conn->cid_status && !conn->otg_plugin_status)
		connector_antiburn_recover(conn);
}

/* The same question, on a board with one thermistor and no rate to judge. */
static void connector_antiburn_check_status(struct connector_antiburn *conn)
{
	int temp = conn->connector_temp[0];

	protocol_class_pd_get_otg_plugin_status(TYPEC_PORT_0,
						&conn->otg_plugin_status);
	platform_class_buckchg_ops_get_online(MAIN_BUCK_CHARGER,
					      &conn->usb_online);
	protocol_class_pd_get_cid_status(TYPEC_PORT_0, &conn->cid_status);

	mca_log_info("connector_temp:%d triggered:%d, cid_status:%d, otg_plugin_status: %d, usb_online:%d\n",
		     temp, conn->triggered, conn->cid_status,
		     conn->otg_plugin_status, conn->usb_online);

	if (temp >= conn->trigger_temp &&
	    (conn->otg_plugin_status || conn->usb_online) &&
	    !conn->disable_antiburn && !conn->triggered) {
		connector_antiburn_trigger(conn);
		return;
	}

	if (temp < conn->recharge_temp && conn->triggered &&
	    !conn->cid_status && !conn->otg_plugin_status)
		connector_antiburn_recover(conn);
}

/*
 * Back off to the idle period once nothing has been attached and both
 * thermistors have read cool for several consecutive readings.  Anything
 * happening at the port puts it straight back to the attached period, and the
 * count starts again.
 */
static void connector_antiburn_pick_interval(struct connector_antiburn *conn)
{
	static unsigned int idle_cycles;

	if (conn->cid_status || conn->usb_online == 1 ||
	    conn->otg_plugin_status ||
	    conn->connector_temp[0] >= ANTIBURN_IDLE_TEMP_MAX ||
	    conn->connector_temp[1] >= ANTIBURN_IDLE_TEMP_MAX) {
		idle_cycles = 0;
		conn->monitor_interval = ANTIBURN_FAST_INTERVAL_MS;
		return;
	}

	conn->monitor_interval = idle_cycles++ < ANTIBURN_IDLE_CYCLES ?
					 ANTIBURN_FAST_INTERVAL_MS :
					 ANTIBURN_IDLE_INTERVAL_MS;
}

static void connector_antiburn_monitor_workfunc(struct work_struct *work)
{
	struct connector_antiburn *conn = container_of(to_delayed_work(work),
						       struct connector_antiburn,
						       monitor_work);
	int temp;

	connector_antiburn_get_temp_increase_rate(conn);

	if (conn->support_elaboration_anti_strategy)
		connector_antiburn_check_status_v2(conn);
	else
		connector_antiburn_check_status(conn);

	/*
	 * Userspace is told every cycle rather than only on a change, because
	 * it draws the connector temperature and wants it to keep moving.
	 */
	temp = max(conn->connector_temp[0], conn->connector_temp[1]);
	connector_antiburn_temp_uevent(conn, temp);
	connector_antiburn_ntc_alarm_uevent(conn, conn->ntc_alarm);

	connector_antiburn_pick_interval(conn);

	mca_queue_delayed_work(&conn->monitor_work,
			       msecs_to_jiffies(conn->monitor_interval));
}

static int connector_antiburn_thermal_notifier_event(struct notifier_block *nb,
						     unsigned long event,
						     void *data)
{
	struct connector_antiburn *conn = container_of(nb,
						       struct connector_antiburn,
						       thermal_board_nb);

	if (event != MCA_EVENT_THERMAL_BOARD_TEMP_CHANGE) {
		mca_log_info("not supported thermal board notifier event: %lu\n",
			     event);
		return NOTIFY_DONE;
	}

	conn->thermal_board_temp = *(int *)data / 1000;
	mca_log_info("get thermal_board_temp: %d\n", conn->thermal_board_temp);

	return NOTIFY_DONE;
}

static int connector_antiburn_debug_notifier_cb(struct notifier_block *nb,
						unsigned long event,
						void *data)
{
	struct connector_antiburn *conn = container_of(nb,
						       struct connector_antiburn,
						       debug_nb);

	/*
	 * All three of these turn the phone hot on purpose -- an 85/85 soak,
	 * a run with the temperature limits lifted, a memory soak -- so each
	 * of them is a reason to stop cutting the connector for being warm.
	 */
	if (event < MCA_EVENT_DEBUG_CTRL_DOUBLE85 ||
	    event > MCA_EVENT_DEBUG_CTRL_MEMORY_TEST)
		return NOTIFY_DONE;

	conn->disable_antiburn = *(int *)data;
	mca_log_info("debug[%lu] disable_antiburn: %d\n", event,
		     conn->disable_antiburn);

	return NOTIFY_DONE;
}

/*
 * A charger being plugged in is the moment the connector is most likely to
 * start heating, so the interval drops to a second and the next reading is
 * taken almost immediately rather than waiting out the idle one.
 */
static int connector_antiburn_connect_cb(struct notifier_block *nb,
					 unsigned long event, void *data)
{
	struct connector_antiburn *conn = container_of(nb,
						       struct connector_antiburn,
						       connect_nb);

	if (!conn->support_elaboration_anti_strategy)
		return NOTIFY_DONE;

	if (conn->monitor_interval == ANTIBURN_FAST_INTERVAL_MS) {
		mca_log_info("usb connect, monitor_work already quick, return...\n");
		return NOTIFY_DONE;
	}

	if (event != MCA_EVENT_USB_CONNECT)
		return NOTIFY_DONE;

	mca_log_info("usb connect, run monitor_work now\n");
	cancel_delayed_work_sync(&conn->monitor_work);
	mca_queue_delayed_work(&conn->monitor_work,
			       msecs_to_jiffies(ANTIBURN_KICK_DELAY_MS));
	conn->monitor_interval = ANTIBURN_FAST_INTERVAL_MS;

	return NOTIFY_DONE;
}

/* And so is the port starting to source, or a cable merely being sensed. */
static int connector_antiburn_hw_cb(struct notifier_block *nb,
				    unsigned long event, void *data)
{
	struct connector_antiburn *conn = container_of(nb,
						       struct connector_antiburn,
						       hw_nb);
	int value;

	if (!conn->support_elaboration_anti_strategy)
		return NOTIFY_DONE;

	if (conn->monitor_interval == ANTIBURN_FAST_INTERVAL_MS) {
		mca_log_info("cid/otg connect, monitor_work already quick, return...\n");
		return NOTIFY_DONE;
	}

	switch (event) {
	case MCA_EVENT_BOOST_STS:
		value = *(int *)data;
		mca_log_info("boost_sts = %d\n", value);
		if (value != 1)
			return NOTIFY_DONE;
		mca_log_info("otg connect, run monitor_work now\n");
		break;
	case MCA_EVENT_CID_STS:
		value = *(int *)data;
		mca_log_info("cid_sts = %d\n", value);
		if (value != 1)
			return NOTIFY_DONE;
		mca_log_info("cid_sts, run monitor_work now\n");
		break;
	default:
		return NOTIFY_DONE;
	}

	cancel_delayed_work_sync(&conn->monitor_work);
	mca_queue_delayed_work(&conn->monitor_work,
			       msecs_to_jiffies(ANTIBURN_KICK_DELAY_MS));
	conn->monitor_interval = ANTIBURN_FAST_INTERVAL_MS;

	return NOTIFY_DONE;
}

static ssize_t antiburn_sysfs_show(struct device *dev,
				   struct device_attribute *attr, char *buf);
static ssize_t antiburn_sysfs_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count);

static struct mca_sysfs_attr_info antiburn_sysfs_field_tbl[] = {
	mca_sysfs_attr_rw(antiburn_sysfs, 0664, CONN_TEMP_1, connector_temp_1),
	mca_sysfs_attr_rw(antiburn_sysfs, 0664, CONN_TEMP_2, connector_temp_2),
	mca_sysfs_attr_rw(antiburn_sysfs, 0664, CONN_RESET_VSAFE0V,
			  reset_vsafe0V),
	mca_sysfs_attr_rw(antiburn_sysfs, 0664, CONN_NTC_ALARM, ntc_alarm),
	mca_sysfs_attr_rw(antiburn_sysfs, 0664, CONN_MOS_CTRL, mos_ctrl),
};

static struct attribute *antiburn_sysfs_attrs[
	ARRAY_SIZE(antiburn_sysfs_field_tbl) + 1];

static const struct attribute_group antiburn_sysfs_attr_group = {
	.attrs = antiburn_sysfs_attrs,
};

static ssize_t antiburn_sysfs_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct connector_antiburn *conn = dev_get_drvdata(dev);
	struct mca_sysfs_attr_info *info;
	int temp;
	int value;

	info = mca_sysfs_lookup_attr(attr->attr.name, antiburn_sysfs_field_tbl,
				     ARRAY_SIZE(antiburn_sysfs_field_tbl));
	if (!info)
		return -EINVAL;

	if (!conn) {
		mca_log_err("%s dev_driverdata is null\n", __func__);
		return -EINVAL;
	}

	switch (info->sysfs_attr_name) {
	case CONN_TEMP_1:
	case CONN_TEMP_2:
		temp = connector_antiburn_get_temperature(
			conn, info->sysfs_attr_name);
		/*
		 * Whole degrees are too coarse for what userspace draws, so
		 * the boards that read in them are scaled up by ten here;
		 * the ones already reading millidegrees are left alone.
		 */
		value = conn->support_elaboration_anti_strategy ? temp
							       : temp * 10;
		break;
	case CONN_RESET_VSAFE0V:
		value = conn->is_reset_vsafe0V;
		break;
	case CONN_NTC_ALARM:
		value = conn->ntc_alarm;
		break;
	case CONN_MOS_CTRL:
		if (!conn->support_hw_antiburn) {
			mca_log_err("not support_hw_antiburn\n");
			return -EINVAL;
		}
		value = gpiod_get_raw_value(gpio_to_desc(conn->mos_ctrl_gpio));
		mca_log_err("show mos_ctrl:%d\n", value);
		break;
	default:
		return 0;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", value);
}

static ssize_t antiburn_sysfs_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct connector_antiburn *conn = dev_get_drvdata(dev);
	struct mca_sysfs_attr_info *info;
	int value = 0;

	info = mca_sysfs_lookup_attr(attr->attr.name, antiburn_sysfs_field_tbl,
				     ARRAY_SIZE(antiburn_sysfs_field_tbl));
	if (!info)
		return -EINVAL;

	if (!conn) {
		mca_log_err("%s dev_driverdata is null\n", __func__);
		return -EINVAL;
	}

	if (kstrtoint(buf, 10, &value))
		return -EINVAL;

	switch (info->sysfs_attr_name) {
	case CONN_TEMP_1:
	case CONN_TEMP_2:
		/*
		 * Written in the tenths of a degree the show side reports, so
		 * that a test can echo back what it read.
		 */
		conn->fake_connector_temp[info->sysfs_attr_name] = value / 10;
		mca_log_err("set the %d ntc = %d\n", info->sysfs_attr_name,
			    value);
		cancel_delayed_work_sync(&conn->monitor_work);
		mca_queue_delayed_work(&conn->monitor_work,
				       msecs_to_jiffies(ANTIBURN_KICK_DELAY_MS));
		break;
	case CONN_MOS_CTRL:
		if (!conn->support_hw_antiburn) {
			mca_log_err("not support_hw_antiburn\n");
			break;
		}
		if (value <= 1)
			gpiod_direction_output_raw(
				gpio_to_desc(conn->mos_ctrl_gpio), value);
		mca_log_err("set mos_ctrl:%d\n", value);
		break;
	default:
		break;
	}

	return count;
}

static int connector_antiburn_dump_log_head(void *data, char *buf, int size)
{
	return snprintf(buf, size, "port_temp port_temp1 shell_temp ");
}

static int connector_antiburn_dump_log_context(void *data, char *buf, int size)
{
	struct connector_antiburn *conn = data;

	return snprintf(buf, size, "%-10d%-11d%-11d",
			connector_antiburn_get_temperature(conn, 0),
			connector_antiburn_get_temperature(conn, 1),
			conn->thermal_board_temp);
}

static struct mca_log_charge_log_ops connector_antiburn_log_ops = {
	.dump_log_head = connector_antiburn_dump_log_head,
	.dump_log_context = connector_antiburn_dump_log_context,
};

static int connector_antiburn_parse_dt(struct connector_antiburn *conn)
{
	struct device_node *np = conn->dev->of_node;
	bool fine;

	/*
	 * Read first: it decides whether everything below is written in whole
	 * degrees or in millidegrees, and so what the defaults have to be.
	 */
	mca_parse_dts_u32(np, "support_elaboration_anti_strategy",
			  (u32 *)&conn->support_elaboration_anti_strategy, 0);
	fine = conn->support_elaboration_anti_strategy;

	mca_parse_dts_u32(np, "trigger_temp",
			  (u32 *)&conn->trigger_temp,
			  fine ? 65000 : 65);
	mca_parse_dts_u32(np, "recharge_temp",
			  (u32 *)&conn->recharge_temp,
			  fine ? 55000 : 55);
	mca_parse_dts_u32(np, "support_soft_antiburn",
			  (u32 *)&conn->support_soft_antiburn, 1);
	mca_parse_dts_u32(np, "support_hw_antiburn",
			  (u32 *)&conn->support_hw_antiburn, 1);
	mca_parse_dts_u32(np, "use_double_ntc",
			  (u32 *)&conn->use_double_ntc, 0);
	mca_parse_dts_u32(np, "antiburn_otg_detect",
			  (u32 *)&conn->otg_detect_en, 1);
	mca_parse_dts_u32(np, "monitor_interval",
			  (u32 *)&conn->monitor_interval,
			  1000);
	mca_parse_dts_u32(np, "max_temp_increase_rate",
			  (u32 *)&conn->max_temp_increase_rate, fine ? 4000 : 4);
	mca_parse_dts_string(np, "thermal-zone-name", &conn->thermal_zone_name);
	mca_parse_dts_string(np, "thermal-zone-name2",
			     &conn->thermal_zone_name2);
	mca_parse_dts_u32(np, "comb_sensorboard_con_trigger_temp",
			  (u32 *)&conn->comb_sensorboard_con_trigger_temp,
			  fine ? 60000 : 60);
	mca_parse_dts_u32(np, "comb_rate_conn_trigger_temp",
			  (u32 *)&conn->comb_rate_conn_trigger_temp,
			  fine ? 35000 : 35);
	mca_parse_dts_u32(np, "max_thermal_board_temp",
			  (u32 *)&conn->max_thermal_board_temp, 50);
	mca_parse_dts_u32(np, "otg_boost_src",
			  (u32 *)&conn->otg_boost_src, 2);
	mca_parse_dts_u32(np, "en_src",
			  (u32 *)&conn->en_src, 0);

	conn->support_base_flip = of_find_property(np, "support-base-flip",
						   NULL) ? 1 : 0;

	mca_log_err("%d:%d:%d:%d:%d:%d\n",
		    conn->support_elaboration_anti_strategy,
		    conn->trigger_temp, conn->recharge_temp,
		    conn->max_temp_increase_rate, conn->monitor_interval,
		    conn->support_base_flip);

	return 0;
}

static int connector_antiburn_gpio_init(struct connector_antiburn *conn)
{
	int rc;

	mca_log_info("Hw antiburn init gpio\n");

	if (!conn->support_hw_antiburn) {
		mca_log_info("No gpio config\n");
		return -1;
	}

	conn->mos_ctrl_gpio = of_get_named_gpio(conn->dev->of_node,
						"mos-ctrl-gpio", 0);
	if (conn->mos_ctrl_gpio < 0)
		mca_log_err("failed to parse mos ctrl gpio\n");

	rc = gpio_request(conn->mos_ctrl_gpio, "mos-ctrl-gpio");
	if (rc) {
		mca_log_err("unable to request antiburn mos ctrl gpio ret is %d\n",
			    rc);
		return rc;
	}

	rc = gpiod_direction_output_raw(gpio_to_desc(conn->mos_ctrl_gpio), 0);
	if (rc) {
		mca_log_err("unable to set direction for pmic gpio\n");
		return rc;
	}

	return 0;
}

static int connector_antiburn_probe(struct platform_device *pdev)
{
	static int probe_cnt;
	struct mca_hwid_info *hwid;
	struct connector_antiburn *conn;
	int rc;

	hwid = mca_get_hwid_info();
	mca_log_info("probe_cnt = %d\n", ++probe_cnt);
	if (!hwid)
		return -ENOMEM;

	/*
	 * The P0.1 build wired the connector thermistor to a pin that reads
	 * nothing, so watching it would only ever produce false triggers.
	 */
	if (hwid->platform_version == 1 && hwid->build_version == 0 &&
	    hwid->minor_version == 1) {
		mca_log_err("Do not support antiburn in %s P%d.%d\n",
			    hwid->product_name, hwid->build_version,
			    hwid->minor_version);
		return 0;
	}

	conn = devm_kzalloc(&pdev->dev, sizeof(*conn), GFP_KERNEL);
	if (!conn) {
		mca_log_err("out of memory\n");
		return -ENOMEM;
	}

	conn->dev = &pdev->dev;
	platform_set_drvdata(pdev, conn);

	connector_antiburn_parse_dt(conn);

	/*
	 * The early builds have no internal boost on the port, so the OTG
	 * supply that has to be shut down before the pin opens is the
	 * external one regardless of what the tree says.
	 */
	if (hwid->platform_version == 1 &&
	    (hwid->build_version == 0 ||
	     (hwid->build_version == 1 && hwid->minor_version == 0))) {
		conn->otg_boost_src = 2;
		mca_log_err("start to use external boost in O2\n");
	}

	rc = connector_antiburn_gpio_init(conn);
	if (rc) {
		mca_log_err("connector_antiburn_gpio_init failed, err is %d\n",
			    rc);
		goto err_gpio;
	}

	conn->tzd_conn = thermal_zone_get_zone_by_name(conn->thermal_zone_name);
	if (IS_ERR(conn->tzd_conn)) {
		mca_log_err("thermal zone get conn_therm failed\n");
		rc = -EPROBE_DEFER;
		goto err_gpio;
	}

	if (conn->use_double_ntc) {
		conn->tzd_conn2 = thermal_zone_get_zone_by_name(
			conn->thermal_zone_name2);
		if (IS_ERR(conn->tzd_conn2)) {
			mca_log_err("thermal zone get conn_therm2 failed\n");
			rc = -EPROBE_DEFER;
			goto err_gpio;
		}
	}

	conn->thermal_board_nb.notifier_call =
		connector_antiburn_thermal_notifier_event;
	mca_event_block_notify_register(MCA_EVENT_TYPE_THERMAL_TEMP,
					&conn->thermal_board_nb);
	conn->debug_nb.notifier_call = connector_antiburn_debug_notifier_cb;
	mca_event_block_notify_register(MCA_EVENT_TYPE_DEBUG, &conn->debug_nb);
	conn->connect_nb.notifier_call = connector_antiburn_connect_cb;
	mca_event_block_notify_register(MCA_EVENT_TYPE_CHARGER_CONNECT,
					&conn->connect_nb);
	conn->hw_nb.notifier_call = connector_antiburn_hw_cb;
	mca_event_block_notify_register(MCA_EVENT_TYPE_HW_INFO, &conn->hw_nb);

	INIT_DELAYED_WORK(&conn->monitor_work,
			  connector_antiburn_monitor_workfunc);
	mca_queue_delayed_work(&conn->monitor_work,
			       msecs_to_jiffies(conn->monitor_interval));

	mca_log_charge_log_register(MCA_CHARGE_LOG_ID_USCP,
				    &connector_antiburn_log_ops, conn);

	mca_sysfs_init_attrs(antiburn_sysfs_attrs, antiburn_sysfs_field_tbl,
			     ARRAY_SIZE(antiburn_sysfs_field_tbl));
	mca_sysfs_create_link_group(MCA_SYSFS_DEV_HW_MONITOR, "connector",
				    conn->dev, &antiburn_sysfs_attr_group);

	g_connector_antiburn = conn;

	mca_log_info("probe OK\n");

	return 0;

err_gpio:
	gpio_free(conn->mos_ctrl_gpio);
	devm_kfree(&pdev->dev, conn);

	return rc;
}

static int connector_antiburn_remove(struct platform_device *pdev)
{
	struct connector_antiburn *conn = platform_get_drvdata(pdev);

	g_connector_antiburn = NULL;

	cancel_delayed_work_sync(&conn->monitor_work);

	/*
	 * The vendor leaves the four notifier blocks registered and the sysfs
	 * link in place, both of which point into memory this is about to
	 * free.  Unwind them.
	 */
	mca_sysfs_remove_link_group(MCA_SYSFS_DEV_HW_MONITOR, "connector",
				    conn->dev, &antiburn_sysfs_attr_group);
	mca_event_block_notify_unregister(MCA_EVENT_TYPE_HW_INFO,
					  &conn->hw_nb);
	mca_event_block_notify_unregister(MCA_EVENT_TYPE_CHARGER_CONNECT,
					  &conn->connect_nb);
	mca_event_block_notify_unregister(MCA_EVENT_TYPE_DEBUG,
					  &conn->debug_nb);
	mca_event_block_notify_unregister(MCA_EVENT_TYPE_THERMAL_TEMP,
					  &conn->thermal_board_nb);

	if (conn->support_hw_antiburn) {
		gpio_free(conn->mos_ctrl_gpio);
		mca_log_info("remove mos ctrl gpio success\n");
	}

	devm_kfree(&pdev->dev, conn);

	return 0;
}

static void connector_antiburn_shutdown(struct platform_device *pdev)
{
}

static const struct of_device_id connector_antiburn_match_table[] = {
	{ .compatible = "xiaomi,connector_antiburn" },
	{},
};
MODULE_DEVICE_TABLE(of, connector_antiburn_match_table);

static struct platform_driver connector_antiburn_driver = {
	.driver = {
		.name = "connector_antiburn",
		.of_match_table = connector_antiburn_match_table,
	},
	.probe = connector_antiburn_probe,
	.remove = connector_antiburn_remove,
	.shutdown = connector_antiburn_shutdown,
};
module_platform_driver(connector_antiburn_driver);

MODULE_DESCRIPTION("MCA connector anti-burn");
MODULE_LICENSE("GPL");
