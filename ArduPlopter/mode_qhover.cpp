#include "mode.h"
#include "Plopter.h"

#if HAL_QUADPLANE_ENABLED

bool ModeQHover::_enter()
{
    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-quadplopter.get_pilot_velocity_z_max_dn(), quadplopter.pilot_velocity_z_max_up, quadplopter.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-quadplopter.get_pilot_velocity_z_max_dn(), quadplopter.pilot_velocity_z_max_up, quadplopter.pilot_accel_z);
    quadplopter.set_climb_rate_cms(0, false);

    quadplopter.init_throttle_wait();
    return true;
}

void ModeQHover::update()
{
    plopter.mode_qstabilize.update();
}

/*
  control QHOVER mode
 */
void ModeQHover::run()
{
    if (quadplopter.throttle_wait) {
        quadplopter.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0, true, 0);
        quadplopter.relax_attitude_control();
        pos_control->relax_z_controller(0);
    } else {
        quadplopter.hold_hover(quadplopter.get_pilot_desired_climb_rate_cms());
    }
}

#endif
