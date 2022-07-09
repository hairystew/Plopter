#include "mode.h"
#include "Plopter.h"

bool ModeFBWB::_enter()
{
#if HAL_SOARING_ENABLED
    // for ArduSoar soaring_controller
    plopter.g2.soaring_controller.init_cruising();
#endif

    plopter.set_target_altitude_current();

    return true;
}

void ModeFBWB::update()
{
    // Thanks to Yury MonZon for the altitude limit code!
    plopter.nav_roll_cd = plopter.channel_roll->norm_input() * plopter.roll_limit_cd;
    plopter.update_load_factor();
    plopter.update_fbwb_speed_height();

}

