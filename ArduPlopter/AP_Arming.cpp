/*
  additional arming checks for plopter
 */
#include "AP_Arming.h"
#include "Plopter.h"

#include "qautotune.h"

constexpr uint32_t AP_ARMING_DELAY_MS = 2000; // delay from arming to start of motor spoolup

const AP_Param::GroupInfo AP_Arming_Plopter::var_info[] = {
    // variables from parent vehicle
    AP_NESTEDGROUPINFO(AP_Arming, 0),

    // index 3 was RUDDER and should not be used

    AP_GROUPEND
};

/*
  additional arming checks for plopter

 */
bool AP_Arming_Plopter::pre_arm_checks(bool display_failure)
{
    if (armed || require == (uint8_t)Required::NO) {
        // if we are already armed or don't need any arming checks
        // then skip the checks
        return true;
    }
    //are arming checks disabled?
    if (checks_to_perform == 0) {
        return true;
    }
    if (hal.util->was_watchdog_armed()) {
        // on watchdog reset bypass arming checks to allow for
        // in-flight arming if we were armed before the reset. This
        // allows a reset on a BVLOS flight to return home if the
        // operator can command arming over telemetry
        return true;
    }

    // call parent class checks
    bool ret = AP_Arming::pre_arm_checks(display_failure);

    // Check airspeed sensor
    ret &= AP_Arming::airspeed_checks(display_failure);

    if (plopter.g.fs_timeout_long < plopter.g.fs_timeout_short && plopter.g.fs_action_short != FS_ACTION_SHORT_DISABLED) {
        check_failed(display_failure, "FS_LONG_TIMEOUT < FS_SHORT_TIMEOUT");
        ret = false;
    }

    if (plopter.aparm.roll_limit_cd < 300) {
        check_failed(display_failure, "LIM_ROLL_CD too small (%u)", (unsigned)plopter.aparm.roll_limit_cd);
        ret = false;
    }

    if (plopter.aparm.pitch_limit_max_cd < 300) {
        check_failed(display_failure, "LIM_PITCH_MAX too small (%u)", (unsigned)plopter.aparm.pitch_limit_max_cd);
        ret = false;
    }

    if (plopter.aparm.pitch_limit_min_cd > -300) {
        check_failed(display_failure, "LIM_PITCH_MIN too large (%u)", (unsigned)plopter.aparm.pitch_limit_min_cd);
        ret = false;
    }

    if (plopter.channel_throttle->get_reverse() &&
        Plopter::ThrFailsafe(plopter.g.throttle_fs_enabled.get()) != Plopter::ThrFailsafe::Disabled &&
        plopter.g.throttle_fs_value <
        plopter.channel_throttle->get_radio_max()) {
        check_failed(display_failure, "Invalid THR_FS_VALUE for rev throttle");
        ret = false;
    }

#if HAL_QUADPLANE_ENABLED
    ret &= quadplopter_checks(display_failure);
#endif

    if (plopter.control_mode == &plopter.mode_auto && plopter.mission.num_commands() <= 1) {
        check_failed(display_failure, "No mission loaded");
        ret = false;
    }

    // check adsb avoidance failsafe
    if (plopter.failsafe.adsb) {
        check_failed(display_failure, "ADSB threat detected");
        ret = false;
    }

    if (SRV_Channels::get_emergency_stop()) {
        check_failed(display_failure,"Motors Emergency Stopped");
        ret = false;
    }

    if (plopter.g2.flight_options & FlightOptions::CENTER_THROTTLE_TRIM){
       int16_t trim = plopter.channel_throttle->get_radio_trim();
       if (trim < 1250 || trim > 1750) {
           check_failed(display_failure, "Throttle trim not near center stick(%u)",trim );
           ret = false;
       }
    }

    if (plopter.mission.get_in_landing_sequence_flag()) {
        check_failed(display_failure,"In landing sequence");
        ret = false;
    }
    
    return ret;
}

#if HAL_QUADPLANE_ENABLED
bool AP_Arming_Plopter::quadplopter_checks(bool display_failure)
{
    if (!plopter.quadplopter.enabled()) {
        return true;
    }

    if (!plopter.quadplopter.available()) {
        check_failed(display_failure, "Quadplopter enabled but not running");
        return false;
    }

    bool ret = true;

    if (plopter.scheduler.get_loop_rate_hz() < 100) {
        check_failed(display_failure, "quadplopter needs SCHED_LOOP_RATE >= 100");
        ret = false;
    }

    if (!plopter.quadplopter.motors->initialised_ok()) {
        check_failed(display_failure, "Quadplopter: check motor setup");
        ret = false;
    }

    // lean angle parameter check
    if (plopter.quadplopter.aparm.angle_max < 1000 || plopter.quadplopter.aparm.angle_max > 8000) {
        check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Check Q_ANGLE_MAX");
        ret = false;
    }

    if ((plopter.quadplopter.tailsitter.enable > 0) && (plopter.quadplopter.tiltrotor.enable > 0)) {
        check_failed(ARMING_CHECK_PARAMETERS, display_failure, "set TAILSIT_ENABLE 0 or TILT_ENABLE 0");
        ret = false;

    } else {

        if ((plopter.quadplopter.tailsitter.enable > 0) && !plopter.quadplopter.tailsitter.enabled()) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "tailsitter setup not complete, reboot");
            ret = false;
        }

        if ((plopter.quadplopter.tiltrotor.enable > 0) && !plopter.quadplopter.tiltrotor.enabled()) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "tiltrotor setup not complete, reboot");
            ret = false;
        }
    }

    // ensure controllers are OK with us arming:
    char failure_msg[50];
    if (!plopter.quadplopter.pos_control->pre_arm_checks("PSC", failure_msg, ARRAY_SIZE(failure_msg))) {
        check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Bad parameter: %s", failure_msg);
        ret = false;
    }
    if (!plopter.quadplopter.attitude_control->pre_arm_checks("ATC", failure_msg, ARRAY_SIZE(failure_msg))) {
        check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Bad parameter: %s", failure_msg);
        ret = false;
    }
    if (!plopter.quadplopter.motors->check_mot_pwm_params()) {
        check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Check Q_M_PWM_MIN/MAX");
        ret = false;
    }

    if ((plopter.quadplopter.options & QuadPlopter::OPTION_ONLY_ARM_IN_QMODE_OR_AUTO) != 0) {
        if (!plopter.control_mode->is_vtol_mode() && (plopter.control_mode != &plopter.mode_auto) && (plopter.control_mode != &plopter.mode_guided)) {
            check_failed(display_failure,"not in Q mode");
            ret = false;
        }
        if ((plopter.control_mode == &plopter.mode_auto) && !plopter.quadplopter.is_vtol_takeoff(plopter.mission.get_current_nav_cmd().id)) {
            check_failed(display_failure,"not in VTOL takeoff");
            ret = false;
        }
    }

    if ((plopter.control_mode == &plopter.mode_auto) && !plopter.mission.starts_with_takeoff_cmd()) {
        check_failed(display_failure,"missing takeoff waypoint");
        ret = false;
    }

    if (plopter.control_mode == &plopter.mode_rtl) {
        check_failed(display_failure,"in RTL mode");
        ret = false;
    }
    
    /*
      Q_ASSIST_SPEED really should be enabled for all quadplopters except tailsitters
     */
    if (check_enabled(ARMING_CHECK_PARAMETERS) &&
        is_zero(plopter.quadplopter.assist_speed) &&
        !plopter.quadplopter.tailsitter.enabled()) {
        check_failed(display_failure,"Q_ASSIST_SPEED is not set");
        ret = false;
    }

    return ret;
}
#endif // HAL_QUADPLANE_ENABLED

bool AP_Arming_Plopter::ins_checks(bool display_failure)
{
    // call parent class checks
    if (!AP_Arming::ins_checks(display_failure)) {
        return false;
    }

    // additional plopter specific checks
    if ((checks_to_perform & ARMING_CHECK_ALL) ||
        (checks_to_perform & ARMING_CHECK_INS)) {
        char failure_msg[50] = {};
        if (!AP::ahrs().pre_arm_check(true, failure_msg, sizeof(failure_msg))) {
            check_failed(ARMING_CHECK_INS, display_failure, "AHRS: %s", failure_msg);
            return false;
        }
    }

    return true;
}

bool AP_Arming_Plopter::arm_checks(AP_Arming::Method method)
{
    if (method == AP_Arming::Method::RUDDER) {
        const AP_Arming::RudderArming arming_rudder = get_rudder_arming_type();

        if (arming_rudder == AP_Arming::RudderArming::IS_DISABLED) {
            //parameter disallows rudder arming/disabling

            // if we emit a message here then someone doing surface
            // checks may be bothered by the message being emitted.
            // check_failed(true, "Rudder arming disabled");
            return false;
        }

        // if throttle is not down, then pilot cannot rudder arm/disarm
        if (!is_zero(plopter.get_throttle_input())){
            check_failed(true, "Non-zero throttle");
            return false;
        }
    }

    if (!plopter.control_mode->allows_arming()) {
        check_failed(true, "Mode does not allow arming");
        return false;
    }

    //are arming checks disabled?
    if (checks_to_perform == 0) {
        return true;
    }

    if (hal.util->was_watchdog_armed()) {
        // on watchdog reset bypass arming checks to allow for
        // in-flight arming if we were armed before the reset. This
        // allows a reset on a BVLOS flight to return home if the
        // operator can command arming over telemetry
        gcs().send_text(MAV_SEVERITY_WARNING, "watchdog: Bypassing arming checks");
        return true;
    }

    // call parent class checks
    return AP_Arming::arm_checks(method);
}

/*
  update HAL soft arm state
*/
void AP_Arming_Plopter::change_arm_state(void)
{
    update_soft_armed();
#if HAL_QUADPLANE_ENABLED
    plopter.quadplopter.set_armed(hal.util->get_soft_armed());
#endif
}

bool AP_Arming_Plopter::arm(const AP_Arming::Method method, const bool do_arming_checks)
{
    if (!AP_Arming::arm(method, do_arming_checks)) {
        return false;
    }

    change_arm_state();

    // rising edge of delay_arming oneshot
    delay_arming = true;

    gcs().send_text(MAV_SEVERITY_INFO, "Throttle armed");

    return true;
}

/*
  disarm motors
 */
bool AP_Arming_Plopter::disarm(const AP_Arming::Method method, bool do_disarm_checks)
{
    if (do_disarm_checks &&
        (method == AP_Arming::Method::MAVLINK ||
         method == AP_Arming::Method::RUDDER)) {
        if (plopter.is_flying()) {
            // don't allow mavlink or rudder disarm while flying
            return false;
        }
    }
    
    if (do_disarm_checks && method == AP_Arming::Method::RUDDER) {
        // option must be enabled:
        if (get_rudder_arming_type() != AP_Arming::RudderArming::ARMDISARM) {
            gcs().send_text(MAV_SEVERITY_INFO, "Rudder disarm: disabled");
            return false;
        }
    }

    if (!AP_Arming::disarm(method, do_disarm_checks)) {
        return false;
    }
    if (plopter.control_mode != &plopter.mode_auto) {
        // reset the mission on disarm if we are not in auto
        plopter.mission.reset();
    }

    // suppress the throttle in auto-throttle modes
    plopter.throttle_suppressed = plopter.control_mode->does_auto_throttle();

    // if no airmode switch assigned, ensure airmode is off:
#if HAL_QUADPLANE_ENABLED
    if ((plopter.quadplopter.air_mode == AirMode::ON) && (rc().find_channel_for_option(RC_Channel::AUX_FUNC::AIRMODE) == nullptr)) {
        plopter.quadplopter.air_mode = AirMode::OFF;
    }
#endif

    //only log if disarming was successful
    change_arm_state();

#if QAUTOTUNE_ENABLED
    //save qautotune gains if enabled and success
    if (plopter.control_mode == &plopter.mode_qautotune) {
        plopter.quadplopter.qautotune.save_tuning_gains();
    } else {
        plopter.quadplopter.qautotune.reset();
    }
#endif

    // re-initialize speed variable used in AUTO and GUIDED for
    // DO_CHANGE_SPEED commands
    plopter.new_airspeed_cm = -1;
    
    gcs().send_text(MAV_SEVERITY_INFO, "Throttle disarmed");

    return true;
}

void AP_Arming_Plopter::update_soft_armed()
{
    bool _armed = is_armed();
#if HAL_QUADPLANE_ENABLED
    if (plopter.quadplopter.motor_test.running){
        _armed = true;
    }
#endif
    _armed = _armed && hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED;

    hal.util->set_soft_armed(_armed);
    AP::logger().set_vehicle_armed(hal.util->get_soft_armed());

    // update delay_arming oneshot
    if (delay_arming &&
        (AP_HAL::millis() - hal.util->get_last_armed_change() >= AP_ARMING_DELAY_MS)) {

        delay_arming = false;
    }
}

/*
  extra plopter mission checks
 */
bool AP_Arming_Plopter::mission_checks(bool report)
{
    // base checks
    bool ret = AP_Arming::mission_checks(report);
    if (plopter.mission.get_landing_sequence_start() > 0 && plopter.g.rtl_autoland == RtlAutoland::RTL_DISABLE) {
        ret = false;
        check_failed(ARMING_CHECK_MISSION, report, "DO_LAND_START set and RTL_AUTOLAND disabled");
    }
    return ret;
}
