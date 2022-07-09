#include "mode.h"
#include "Plopter.h"

bool ModeGuided::_enter()
{
    plopter.guided_throttle_passthru = false;
    /*
      when entering guided mode we set the target as the current
      location. This matches the behaviour of the copter code
    */
    Location loc{plopter.current_loc};

#if HAL_QUADPLANE_ENABLED
    if (plopter.quadplopter.guided_mode_enabled()) {
        /*
          if using Q_GUIDED_MODE then project forward by the stopping distance
        */
        loc.offset_bearing(degrees(plopter.ahrs.groundspeed_vector().angle()),
                           plopter.quadplopter.stopping_distance());
    }
#endif

    plopter.set_guided_WP(loc);
    return true;
}

void ModeGuided::update()
{
#if HAL_QUADPLANE_ENABLED
    if (plopter.auto_state.vtol_loiter && plopter.quadplopter.available()) {
        plopter.quadplopter.guided_update();
        return;
    }
#endif
    plopter.calc_nav_roll();
    plopter.calc_nav_pitch();
    plopter.calc_throttle();
}

void ModeGuided::navigate()
{
    // Zero indicates to use WP_LOITER_RAD
    plopter.update_loiter(0);
}

bool ModeGuided::handle_guided_request(Location target_loc)
{
    // add home alt if needed
    if (target_loc.relative_alt) {
        target_loc.alt += plopter.home.alt;
        target_loc.relative_alt = 0;
    }

    plopter.set_guided_WP(target_loc);

    return true;
}
