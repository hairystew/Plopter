#include "mode.h"
#include "Plopter.h"

#if HAL_ADSB_ENABLED

bool ModeAvoidADSB::_enter()
{
    return plopter.mode_guided.enter();
}

void ModeAvoidADSB::update()
{
    plopter.mode_guided.update();
}

void ModeAvoidADSB::navigate()
{
    // Zero indicates to use WP_LOITER_RAD
    plopter.update_loiter(0);
}

#endif // HAL_ADSB_ENABLED

