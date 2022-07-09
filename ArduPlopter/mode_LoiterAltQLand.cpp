#include "mode.h"
#include "Plopter.h"

#if HAL_QUADPLANE_ENABLED

bool ModeLoiterAltQLand::_enter()
{
    if (plopter.previous_mode->is_vtol_mode() || plopter.quadplopter.in_vtol_mode()) {
        plopter.set_mode(plopter.mode_qland, ModeReason::LOITER_ALT_IN_VTOL);
        return true;
    }

    ModeLoiter::_enter();

#if AP_TERRAIN_AVAILABLE
    if (plopter.terrain_enabled_in_mode(Mode::Number::QLAND)) {
        plopter.next_WP_loc.set_alt_cm(quadplopter.qrtl_alt*100UL, Location::AltFrame::ABOVE_TERRAIN);
    } else {
        plopter.next_WP_loc.set_alt_cm(quadplopter.qrtl_alt*100UL, Location::AltFrame::ABOVE_HOME);
    }
#else
    plopter.next_WP_loc.set_alt_cm(quadplopter.qrtl_alt*100UL, Location::AltFrame::ABOVE_HOME);
#endif

    switch_qland();

    return true;
}

void ModeLoiterAltQLand::navigate()
{
    switch_qland();

    ModeLoiter::navigate();
}

void ModeLoiterAltQLand::switch_qland()
{
    ftype dist;
    if ((!plopter.current_loc.get_alt_distance(plopter.next_WP_loc, dist) || is_negative(dist)) && plopter.nav_controller->reached_loiter_target()) {
        plopter.set_mode(plopter.mode_qland, ModeReason::LOITER_ALT_REACHED_QLAND);
    }
}

bool ModeLoiterAltQLand::handle_guided_request(Location target_loc)
{
    // setup altitude
#if AP_TERRAIN_AVAILABLE
    if (plopter.terrain_enabled_in_mode(Mode::Number::QLAND)) {
        target_loc.set_alt_cm(quadplopter.qrtl_alt*100UL, Location::AltFrame::ABOVE_TERRAIN);
    } else {
        target_loc.set_alt_cm(quadplopter.qrtl_alt*100UL, Location::AltFrame::ABOVE_HOME);
    }
#else
    target_loc.set_alt_cm(quadplopter.qrtl_alt*100UL, Location::AltFrame::ABOVE_HOME);
#endif

    plopter.set_guided_WP(target_loc);

    return true;
}

#endif
