#include "mode.h"
#include "Plopter.h"

void ModeStabilize::update()
{
    plopter.nav_roll_cd = 0;
    plopter.nav_pitch_cd = 0;
}

