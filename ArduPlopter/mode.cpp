#include "Plopter.h"

Mode::Mode()
#if HAL_QUADPLANE_ENABLED
    : quadplopter(plopter.quadplopter),
    pos_control(plopter.quadplopter.pos_control),
    attitude_control(plopter.quadplopter.attitude_control),
    loiter_nav(plopter.quadplopter.loiter_nav),
    poscontrol(plopter.quadplopter.poscontrol)
#endif
{
}

void Mode::exit()
{
    // call sub-classes exit
    _exit();
}

bool Mode::enter()
{
#if AP_SCRIPTING_ENABLED
    // reset nav_scripting.enabled
    plopter.nav_scripting.enabled = false;
#endif

    // cancel inverted flight
    plopter.auto_state.inverted_flight = false;

    // don't cross-track when starting a mission
    plopter.auto_state.next_wp_crosstrack = false;

    // reset landing check
    plopter.auto_state.checked_for_autoland = false;

    // zero locked course
    plopter.steer_state.locked_course_err = 0;
    plopter.steer_state.locked_course = false;

    // reset crash detection
    plopter.crash_state.is_crashed = false;
    plopter.crash_state.impact_detected = false;

    // reset external attitude guidance
    plopter.guided_state.last_forced_rpy_ms.zero();
    plopter.guided_state.last_forced_throttle_ms = 0;

#if OFFBOARD_GUIDED == ENABLED
    plopter.guided_state.target_heading = -4; // radians here are in range -3.14 to 3.14, so a default value needs to be outside that range
    plopter.guided_state.target_heading_type = GUIDED_HEADING_NONE;
    plopter.guided_state.target_airspeed_cm = -1; // same as above, although an airspeed of -1 is rare on plopter.
    plopter.guided_state.target_alt = -1; // same as above, although a target alt of -1 is rare on plopter.
    plopter.guided_state.last_target_alt = 0;
#endif

#if CAMERA == ENABLED
    plopter.camera.set_is_auto_mode(this == &plopter.mode_auto);
#endif

    // zero initial pitch and highest airspeed on mode change
    plopter.auto_state.highest_airspeed = 0;
    plopter.auto_state.initial_pitch_cd = plopter.ahrs.pitch_sensor;

    // disable taildrag takeoff on mode change
    plopter.auto_state.fbwa_tdrag_takeoff_mode = false;

    // start with previous WP at current location
    plopter.prev_WP_loc = plopter.current_loc;

    // new mode means new loiter
    plopter.loiter.start_time_ms = 0;

    // record time of mode change
    plopter.last_mode_change_ms = AP_HAL::millis();

    // set VTOL auto state
    plopter.auto_state.vtol_mode = is_vtol_mode();
    plopter.auto_state.vtol_loiter = false;

    // initialize speed variable used in AUTO and GUIDED for DO_CHANGE_SPEED commands
    plopter.new_airspeed_cm = -1;

#if HAL_QUADPLANE_ENABLED
    quadplopter.mode_enter();
#endif

    bool enter_result = _enter();

    if (enter_result) {
        // -------------------
        // these must be done AFTER _enter() because they use the results to set more flags

        // start with throttle suppressed in auto_throttle modes
        plopter.throttle_suppressed = does_auto_throttle();
#if HAL_ADSB_ENABLED
        plopter.adsb.set_is_auto_mode(does_auto_navigation());
#endif

        // reset steering integrator on mode change
        plopter.steerController.reset_I();

        // update RC failsafe, as mode change may have necessitated changing the failsafe throttle
        plopter.control_failsafe();
    }

    return enter_result;
}

bool Mode::is_vtol_man_throttle() const
{
#if HAL_QUADPLANE_ENABLED
    if (plopter.quadplopter.tailsitter.is_in_fw_flight() &&
        plopter.quadplopter.assisted_flight) {
        // We are a tailsitter that has fully transitioned to Q-assisted forward flight.
        // In this case the forward throttle directly drives the vertical throttle so
        // set vertical throttle state to match the forward throttle state. Confusingly the booleans are inverted,
        // forward throttle uses 'does_auto_throttle' whereas vertical uses 'is_vtol_man_throttle'.
        return !does_auto_throttle();
    }
#endif
    return false;
}
