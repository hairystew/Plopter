#include "mode.h"
#include "Plopter.h"

bool ModeAcro::_enter()
{
    plopter.acro_state.locked_roll = false;
    plopter.acro_state.locked_pitch = false;
    return true;
}

void ModeAcro::update()
{
    // handle locked/unlocked control
    if (plopter.acro_state.locked_roll) {
        plopter.nav_roll_cd = plopter.acro_state.locked_roll_err;
    } else {
        plopter.nav_roll_cd = plopter.ahrs.roll_sensor;
    }
    if (plopter.acro_state.locked_pitch) {
        plopter.nav_pitch_cd = plopter.acro_state.locked_pitch_cd;
    } else {
        plopter.nav_pitch_cd = plopter.ahrs.pitch_sensor;
    }
}

