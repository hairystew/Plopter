#include "mode.h"
#include "Plopter.h"

bool ModeAuto::_enter()
{
#if HAL_QUADPLANE_ENABLED
    // check if we should refuse auto mode due to a missing takeoff in
    // guided_wait_takeoff state
    if (plopter.previous_mode == &plopter.mode_guided &&
        quadplopter.guided_wait_takeoff_on_mode_enter) {
        if (!plopter.mission.starts_with_takeoff_cmd()) {
            gcs().send_text(MAV_SEVERITY_ERROR,"Takeoff waypoint required");
            quadplopter.guided_wait_takeoff = true;
            return false;
        }
    }
    
    if (plopter.quadplopter.available() && plopter.quadplopter.enable == 2) {
        plopter.auto_state.vtol_mode = true;
    } else {
        plopter.auto_state.vtol_mode = false;
    }
#else
    plopter.auto_state.vtol_mode = false;
#endif
    plopter.next_WP_loc = plopter.prev_WP_loc = plopter.current_loc;
    // start or resume the mission, based on MIS_AUTORESET
    plopter.mission.start_or_resume();

    if (hal.util->was_watchdog_armed()) {
        if (hal.util->persistent_data.waypoint_num != 0) {
            gcs().send_text(MAV_SEVERITY_INFO, "Watchdog: resume WP %u", hal.util->persistent_data.waypoint_num);
            plopter.mission.set_current_cmd(hal.util->persistent_data.waypoint_num);
            hal.util->persistent_data.waypoint_num = 0;
        }
    }

#if HAL_SOARING_ENABLED
    plopter.g2.soaring_controller.init_cruising();
#endif

    return true;
}

void ModeAuto::_exit()
{
    if (plopter.mission.state() == AP_Mission::MISSION_RUNNING) {
        plopter.mission.stop();

        bool restart = plopter.mission.get_current_nav_cmd().id == MAV_CMD_NAV_LAND;
#if HAL_QUADPLANE_ENABLED
        if (plopter.quadplopter.is_vtol_land(plopter.mission.get_current_nav_cmd().id)) {
            restart = false;
        }
#endif
        if (restart) {
            plopter.landing.restart_landing_sequence();
        }
    }
    plopter.auto_state.started_flying_in_auto_ms = 0;
}

void ModeAuto::update()
{
    if (plopter.mission.state() != AP_Mission::MISSION_RUNNING) {
        // this could happen if AP_Landing::restart_landing_sequence() returns false which would only happen if:
        // restart_landing_sequence() is called when not executing a NAV_LAND or there is no previous nav point
        plopter.set_mode(plopter.mode_rtl, ModeReason::MISSION_END);
        gcs().send_text(MAV_SEVERITY_INFO, "Aircraft in auto without a running mission");
        return;
    }

    uint16_t nav_cmd_id = plopter.mission.get_current_nav_cmd().id;

#if HAL_QUADPLANE_ENABLED
    if (plopter.quadplopter.in_vtol_auto()) {
        plopter.quadplopter.control_auto();
        return;
    }
#endif

    if (nav_cmd_id == MAV_CMD_NAV_TAKEOFF ||
        (nav_cmd_id == MAV_CMD_NAV_LAND && plopter.flight_stage == AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND)) {
        plopter.takeoff_calc_roll();
        plopter.takeoff_calc_pitch();
        plopter.calc_throttle();
    } else if (nav_cmd_id == MAV_CMD_NAV_LAND) {
        plopter.calc_nav_roll();
        plopter.calc_nav_pitch();

        // allow landing to restrict the roll limits
        plopter.nav_roll_cd = plopter.landing.constrain_roll(plopter.nav_roll_cd, plopter.g.level_roll_limit*100UL);

        if (plopter.landing.is_throttle_suppressed()) {
            // if landing is considered complete throttle is never allowed, regardless of landing type
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
        } else {
            plopter.calc_throttle();
        }
#if AP_SCRIPTING_ENABLED
    } else if (nav_cmd_id == MAV_CMD_NAV_SCRIPT_TIME) {
        // NAV_SCRIPTING has a desired roll and pitch rate and desired throttle
        plopter.nav_roll_cd = plopter.ahrs.roll_sensor;
        plopter.nav_pitch_cd = plopter.ahrs.pitch_sensor;
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plopter.nav_scripting.throttle_pct);
#endif
    } else {
        // we are doing normal AUTO flight, the special cases
        // are for takeoff and landing
        if (nav_cmd_id != MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT) {
            plopter.steer_state.hold_course_cd = -1;
        }
        plopter.calc_nav_roll();
        plopter.calc_nav_pitch();
        plopter.calc_throttle();
    }
}

void ModeAuto::navigate()
{
    if (AP::ahrs().home_is_set()) {
        plopter.mission.update();
    }
}


bool ModeAuto::does_auto_navigation() const
{
#if AP_SCRIPTING_ENABLED
   return (!plopter.nav_scripting_active());
#endif
   return true;
}

bool ModeAuto::does_auto_throttle() const
{
#if AP_SCRIPTING_ENABLED
   return (!plopter.nav_scripting_active());
#endif
   return true;
}
