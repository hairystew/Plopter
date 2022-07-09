#include "mode.h"
#include "Plopter.h"

#if HAL_QUADPLANE_ENABLED

bool ModeQAcro::_enter()
{
    quadplopter.throttle_wait = false;
    quadplopter.transition->force_transistion_complete();
    attitude_control->relax_attitude_controllers();
    return true;
}

void ModeQAcro::update()
{
    // get nav_roll and nav_pitch from multicopter attitude controller
    Vector3f att_target = plopter.quadplopter.attitude_control->get_att_target_euler_cd();
    plopter.nav_pitch_cd = att_target.y;
    plopter.nav_roll_cd = att_target.x;
    return;
}


/*
  control QACRO mode
 */
void ModeQAcro::run()
{
    if (quadplopter.throttle_wait) {
        quadplopter.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0, true, 0);
        quadplopter.relax_attitude_control();
    } else {
        quadplopter.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // convert the input to the desired body frame rate
        float target_roll = 0;
        float target_pitch = plopter.channel_pitch->norm_input() * quadplopter.acro_pitch_rate * 100.0f;
        float target_yaw = 0;
        if (quadplopter.tailsitter.enabled()) {
            // Note that the 90 degree Y rotation for copter mode swaps body-frame roll and yaw
            target_roll =  plopter.channel_rudder->norm_input() * quadplopter.acro_yaw_rate * 100.0f;
            target_yaw  = -plopter.channel_roll->norm_input() * quadplopter.acro_roll_rate * 100.0f;
        } else {
            target_roll = plopter.channel_roll->norm_input() * quadplopter.acro_roll_rate * 100.0f;
            target_yaw  = plopter.channel_rudder->norm_input() * quadplopter.acro_yaw_rate * 100.0;
        }

        float throttle_out = quadplopter.get_pilot_throttle();

        // run attitude controller
        if (plopter.g.acro_locking) {
            attitude_control->input_rate_bf_roll_pitch_yaw_3(target_roll, target_pitch, target_yaw);
        } else {
            attitude_control->input_rate_bf_roll_pitch_yaw_2(target_roll, target_pitch, target_yaw);
        }

        // output pilot's throttle without angle boost
        attitude_control->set_throttle_out(throttle_out, false, 10.0f);
    }
}

#endif
