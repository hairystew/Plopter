#include "mode.h"
#include "Plopter.h"

bool ModeRTL::_enter()
{
    plopter.prev_WP_loc = plopter.current_loc;
    plopter.do_RTL(plopter.get_RTL_altitude_cm());
    plopter.rtl.done_climb = false;
#if HAL_QUADPLANE_ENABLED
    plopter.vtol_approach_s.approach_stage = Plopter::Landing_ApproachStage::RTL;

    // treat RTL as QLAND if we are in guided wait takeoff state, to cope
    // with failsafes during GUIDED->AUTO takeoff sequence
    if (plopter.quadplopter.guided_wait_takeoff_on_mode_enter) {
       plopter.set_mode(plopter.mode_qland, ModeReason::QLAND_INSTEAD_OF_RTL);
       return true;
    }

    // do not check if we have reached the loiter target if switching from loiter this will trigger as the nav controller has not yet proceeded the new destination
    switch_QRTL(false);
#endif

    return true;
}

void ModeRTL::update()
{
    plopter.calc_nav_roll();
    plopter.calc_nav_pitch();
    plopter.calc_throttle();

    bool alt_threshold_reached = false;
    if (plopter.g2.flight_options & FlightOptions::CLIMB_BEFORE_TURN) {
        // Climb to ALT_HOLD_RTL before turning. This overrides RTL_CLIMB_MIN.
        alt_threshold_reached = plopter.current_loc.alt > plopter.next_WP_loc.alt;
    } else if (plopter.g2.rtl_climb_min > 0) {
        /*
           when RTL first starts limit bank angle to LEVEL_ROLL_LIMIT
           until we have climbed by RTL_CLIMB_MIN meters
           */
        alt_threshold_reached = (plopter.current_loc.alt - plopter.prev_WP_loc.alt)*0.01 > plopter.g2.rtl_climb_min;
    } else {
        return;
    }

    if (!plopter.rtl.done_climb && alt_threshold_reached) {
        plopter.prev_WP_loc = plopter.current_loc;
        plopter.setup_glide_slope();
        plopter.rtl.done_climb = true;
    }
    if (!plopter.rtl.done_climb) {
        // Constrain the roll limit as a failsafe, that way if something goes wrong the plopter will
        // eventually turn back and go to RTL instead of going perfectly straight. This also leaves
        // some leeway for fighting wind.
        plopter.roll_limit_cd = MIN(plopter.roll_limit_cd, plopter.g.level_roll_limit*100);
        plopter.nav_roll_cd = constrain_int32(plopter.nav_roll_cd, -plopter.roll_limit_cd, plopter.roll_limit_cd);
    }
}

void ModeRTL::navigate()
{
#if HAL_QUADPLANE_ENABLED
    if (plopter.control_mode->mode_number() != QRTL) {
        // QRTL shares this navigate function with RTL

        if (plopter.quadplopter.available() && (plopter.quadplopter.rtl_mode == QuadPlopter::RTL_MODE::VTOL_APPROACH_QRTL)) {
            // VTOL approach landing
            AP_Mission::Mission_Command cmd;
            cmd.content.location = plopter.next_WP_loc;
            plopter.verify_landing_vtol_approach(cmd);
            if (plopter.vtol_approach_s.approach_stage == Plopter::Landing_ApproachStage::VTOL_LANDING) {
                plopter.set_mode(plopter.mode_qrtl, ModeReason::RTL_COMPLETE_SWITCHING_TO_VTOL_LAND_RTL);
            }
            return;
        }

        if ((AP_HAL::millis() - plopter.last_mode_change_ms > 1000) && switch_QRTL()) {
            return;
        }
    }
#endif

    if (plopter.g.rtl_autoland == RtlAutoland::RTL_THEN_DO_LAND_START &&
        !plopter.auto_state.checked_for_autoland &&
        plopter.reached_loiter_target() &&
        labs(plopter.altitude_error_cm) < 1000) {
        // we've reached the RTL point, see if we have a landing sequence
        if (plopter.mission.jump_to_landing_sequence()) {
            // switch from RTL -> AUTO
            plopter.mission.set_force_resume(true);
            plopter.set_mode(plopter.mode_auto, ModeReason::RTL_COMPLETE_SWITCHING_TO_FIXEDWING_AUTOLAND);
        }

        // prevent running the expensive jump_to_landing_sequence
        // on every loop
        plopter.auto_state.checked_for_autoland = true;
    }
    else if (plopter.g.rtl_autoland == RtlAutoland::RTL_IMMEDIATE_DO_LAND_START &&
        !plopter.auto_state.checked_for_autoland) {
        // Go directly to the landing sequence
        if (plopter.mission.jump_to_landing_sequence()) {
            // switch from RTL -> AUTO
            plopter.mission.set_force_resume(true);
            plopter.set_mode(plopter.mode_auto, ModeReason::RTL_COMPLETE_SWITCHING_TO_FIXEDWING_AUTOLAND);
        }

        // prevent running the expensive jump_to_landing_sequence
        // on every loop
        plopter.auto_state.checked_for_autoland = true;
    }
    uint16_t radius = abs(plopter.g.rtl_radius);
    if (radius > 0) {
        plopter.loiter.direction = (plopter.g.rtl_radius < 0) ? -1 : 1;
    }

    plopter.update_loiter(radius);
}

#if HAL_QUADPLANE_ENABLED
// Switch to QRTL if enabled and within radius
bool ModeRTL::switch_QRTL(bool check_loiter_target)
{
    if (!plopter.quadplopter.available() || ((plopter.quadplopter.rtl_mode != QuadPlopter::RTL_MODE::SWITCH_QRTL) && (plopter.quadplopter.rtl_mode != QuadPlopter::RTL_MODE::QRTL_ALWAYS))) {
        return false;
    }

   // if Q_RTL_MODE is QRTL always, then immediately switch to QRTL mode
   if (plopter.quadplopter.rtl_mode == QuadPlopter::RTL_MODE::QRTL_ALWAYS) {
       plopter.set_mode(plopter.mode_qrtl, ModeReason::QRTL_INSTEAD_OF_RTL);
       return true;
   }

    uint16_t qrtl_radius = abs(plopter.g.rtl_radius);
    if (qrtl_radius == 0) {
        qrtl_radius = abs(plopter.aparm.loiter_radius);
    }

    if ( (check_loiter_target && plopter.nav_controller->reached_loiter_target()) ||
         plopter.current_loc.past_interval_finish_line(plopter.prev_WP_loc, plopter.next_WP_loc) ||
         plopter.auto_state.wp_distance < MAX(qrtl_radius, plopter.quadplopter.stopping_distance())) {
        /*
          for a quadplopter in RTL mode we switch to QRTL when we
          are within the maximum of the stopping distance and the
          RTL_RADIUS
         */
        plopter.set_mode(plopter.mode_qrtl, ModeReason::RTL_COMPLETE_SWITCHING_TO_VTOL_LAND_RTL);
        return true;
    }

    return false;
}

#endif  // HAL_QUADPLANE_ENABLED
