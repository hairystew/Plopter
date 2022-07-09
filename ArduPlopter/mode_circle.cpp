#include "mode.h"
#include "Plopter.h"

bool ModeCircle::_enter()
{
    // the altitude to circle at is taken from the current altitude
    plopter.next_WP_loc.alt = plopter.current_loc.alt;

    return true;
}

void ModeCircle::update()
{
    // we have no GPS installed and have lost radio contact
    // or we just want to fly around in a gentle circle w/o GPS,
    // holding altitude at the altitude we set when we
    // switched into the mode
    plopter.nav_roll_cd  = plopter.roll_limit_cd / 3;
    plopter.update_load_factor();
    plopter.calc_nav_pitch();
    plopter.calc_throttle();
}

