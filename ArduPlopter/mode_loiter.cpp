#include "mode.h"
#include "Plopter.h"

bool ModeLoiter::_enter()
{
    plopter.do_loiter_at_location();
    plopter.setup_terrain_target_alt(plopter.next_WP_loc);

    // make sure the local target altitude is the same as the nav target used for loiter nav
    // this allows us to do FBWB style stick control
    /*IGNORE_RETURN(plopter.next_WP_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, plopter.target_altitude.amsl_cm));*/
    if (plopter.stick_mixing_enabled() && (plopter.g2.flight_options & FlightOptions::ENABLE_LOITER_ALT_CONTROL)) {
        plopter.set_target_altitude_current();
    }

    plopter.loiter_angle_reset();

    return true;
}

void ModeLoiter::update()
{
    plopter.calc_nav_roll();
    if (plopter.stick_mixing_enabled() && (plopter.g2.flight_options & FlightOptions::ENABLE_LOITER_ALT_CONTROL)) {
        plopter.update_fbwb_speed_height();
    } else {
        plopter.calc_nav_pitch();
        plopter.calc_throttle();
    }
}

bool ModeLoiter::isHeadingLinedUp(const Location loiterCenterLoc, const Location targetLoc)
{
    // Return true if current heading is aligned to vector to targetLoc.
    // Tolerance is initially 10 degrees and grows at 10 degrees for each loiter circle completed.

    const uint16_t loiterRadius = abs(plopter.aparm.loiter_radius);
    if (loiterCenterLoc.get_distance(targetLoc) < loiterRadius + loiterRadius*0.05) {
        /* Whenever next waypoint is within the loiter radius plus 5%,
           maintaining loiter would prevent us from ever pointing toward the next waypoint.
           Hence break out of loiter immediately
         */
        return true;
    }

    // Bearing in centi-degrees
    const int32_t bearing_cd = plopter.current_loc.get_bearing_to(targetLoc);
    return isHeadingLinedUp_cd(bearing_cd);
}


bool ModeLoiter::isHeadingLinedUp_cd(const int32_t bearing_cd)
{
    // Return true if current heading is aligned to bearing_cd.
    // Tolerance is initially 10 degrees and grows at 10 degrees for each loiter circle completed.

    // get current heading.
    const int32_t heading_cd = (wrap_360(degrees(plopter.ahrs.groundspeed_vector().angle())))*100;

    const int32_t heading_err_cd = wrap_180_cd(bearing_cd - heading_cd);

    /*
      Check to see if the the plopter is heading toward the land
      waypoint. We use 20 degrees (+/-10 deg) of margin so that
      we can handle 200 degrees/second of yaw.

      After every full circle, extend acceptance criteria to ensure
      aircraft will not loop forever in case high winds are forcing
      it beyond 200 deg/sec when passing the desired exit course
    */

    // Use integer division to get discrete steps
    const int32_t expanded_acceptance = 1000 * (labs(plopter.loiter.sum_cd) / 36000);

    if (labs(heading_err_cd) <= 1000 + expanded_acceptance) {
        // Want to head in a straight line from _here_ to the next waypoint instead of center of loiter wp

        // 0 to xtrack from center of waypoint, 1 to xtrack from tangent exit location
        if (plopter.next_WP_loc.loiter_xtrack) {
            plopter.next_WP_loc = plopter.current_loc;
        }
        return true;
    }
    return false;
}

void ModeLoiter::navigate()
{
    if (plopter.g2.flight_options & FlightOptions::ENABLE_LOITER_ALT_CONTROL) {
        // update the WP alt from the global target adjusted by update_fbwb_speed_height
        plopter.next_WP_loc.set_alt_cm(plopter.target_altitude.amsl_cm, Location::AltFrame::ABSOLUTE);
    }

    // Zero indicates to use WP_LOITER_RAD
    plopter.update_loiter(0);
}

