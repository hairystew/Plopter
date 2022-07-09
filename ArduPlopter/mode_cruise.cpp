#include "mode.h"
#include "Plopter.h"

bool ModeCruise::_enter()
{
    locked_heading = false;
    lock_timer_ms = 0;

#if HAL_SOARING_ENABLED
    // for ArduSoar soaring_controller
    plopter.g2.soaring_controller.init_cruising();
#endif

    plopter.set_target_altitude_current();

    return true;
}

void ModeCruise::update()
{
    /*
      in CRUISE mode we use the navigation code to control
      roll when heading is locked. Heading becomes unlocked on
      any aileron or rudder input
    */
    if (plopter.channel_roll->get_control_in() != 0 || plopter.channel_rudder->get_control_in() != 0) {
        locked_heading = false;
        lock_timer_ms = 0;
    }

    if (!locked_heading) {
        plopter.nav_roll_cd = plopter.channel_roll->norm_input() * plopter.roll_limit_cd;
        plopter.update_load_factor();
    } else {
        plopter.calc_nav_roll();
    }
    plopter.update_fbwb_speed_height();
}

/*
  handle CRUISE mode, locking heading to GPS course when we have
  sufficient ground speed, and no aileron or rudder input
 */
void ModeCruise::navigate()
{
    if (!locked_heading &&
        plopter.channel_roll->get_control_in() == 0 &&
        plopter.rudder_input() == 0 &&
        plopter.gps.status() >= AP_GPS::GPS_OK_FIX_2D &&
        plopter.gps.ground_speed() >= 3 &&
        lock_timer_ms == 0) {
        // user wants to lock the heading - start the timer
        lock_timer_ms = millis();
    }
    if (lock_timer_ms != 0 &&
        (millis() - lock_timer_ms) > 500) {
        // lock the heading after 0.5 seconds of zero heading input
        // from user
        locked_heading = true;
        lock_timer_ms = 0;
        locked_heading_cd = plopter.gps.ground_course_cd();
        plopter.prev_WP_loc = plopter.current_loc;
    }
    if (locked_heading) {
        plopter.next_WP_loc = plopter.prev_WP_loc;
        // always look 1km ahead
        plopter.next_WP_loc.offset_bearing(locked_heading_cd*0.01f, plopter.prev_WP_loc.get_distance(plopter.current_loc) + 1000);
        plopter.nav_controller->update_waypoint(plopter.prev_WP_loc, plopter.next_WP_loc);
    }
}

bool ModeCruise::get_target_heading_cd(int32_t &target_heading) const
{
    target_heading = locked_heading_cd;
    return locked_heading;
}
