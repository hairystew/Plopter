#include "mode.h"
#include "Plopter.h"

#if HAL_QUADPLANE_ENABLED

bool ModeQLand::_enter()
{
    plopter.mode_qloiter._enter();
    quadplopter.throttle_wait = false;
    quadplopter.setup_target_position();
    poscontrol.set_state(QuadPlopter::QPOS_LAND_DESCEND);
    poscontrol.pilot_correction_done = false;
    quadplopter.last_land_final_agl = plopter.relative_ground_altitude(plopter.g.rangefinder_landing);
    quadplopter.landing_detect.lower_limit_start_ms = 0;
    quadplopter.landing_detect.land_start_ms = 0;
#if LANDING_GEAR_ENABLED == ENABLED
    plopter.g2.landing_gear.deploy_for_landing();
#endif
#if AC_FENCE == ENABLED
    plopter.fence.auto_disable_fence_for_landing();
#endif
    return true;
}

void ModeQLand::update()
{
    plopter.mode_qstabilize.update();
}

void ModeQLand::run()
{
    plopter.mode_qloiter.run();
}

#endif
