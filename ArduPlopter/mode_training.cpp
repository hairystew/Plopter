#include "mode.h"
#include "Plopter.h"

void ModeTraining::update()
{
    plopter.training_manual_roll = false;
    plopter.training_manual_pitch = false;
    plopter.update_load_factor();

    // if the roll is past the set roll limit, then
    // we set target roll to the limit
    if (plopter.ahrs.roll_sensor >= plopter.roll_limit_cd) {
        plopter.nav_roll_cd = plopter.roll_limit_cd;
    } else if (plopter.ahrs.roll_sensor <= -plopter.roll_limit_cd) {
        plopter.nav_roll_cd = -plopter.roll_limit_cd;
    } else {
        plopter.training_manual_roll = true;
        plopter.nav_roll_cd = 0;
    }

    // if the pitch is past the set pitch limits, then
    // we set target pitch to the limit
    if (plopter.ahrs.pitch_sensor >= plopter.aparm.pitch_limit_max_cd) {
        plopter.nav_pitch_cd = plopter.aparm.pitch_limit_max_cd;
    } else if (plopter.ahrs.pitch_sensor <= plopter.pitch_limit_min_cd) {
        plopter.nav_pitch_cd = plopter.pitch_limit_min_cd;
    } else {
        plopter.training_manual_pitch = true;
        plopter.nav_pitch_cd = 0;
    }
    if (plopter.fly_inverted()) {
        plopter.nav_pitch_cd = -plopter.nav_pitch_cd;
    }
}

