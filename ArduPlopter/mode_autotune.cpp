#include "Plopter.h"

/*
  autotune mode is a wrapper around the AC_AutoTune library
 */

#if AUTOTUNE_ENABLED == ENABLED

bool AutoTune::init()
{
    // only allow AutoTune from some flight modes, for example Stabilize, AltHold,  PosHold or Loiter modes
    if (!plopter.flightmode->allows_autotune()) {
        return false;
    }

    // ensure throttle is above zero
    if (plopter.ap.throttle_zero) {
        return false;
    }

    // ensure we are flying
    if (!plopter.motors->armed() || !plopter.ap.auto_armed || plopter.ap.land_complete) {
        return false;
    }

    // use position hold while tuning if we were in QLOITER
    bool position_hold = (plopter.flightmode->mode_number() == Mode::Number::LOITER || plopter.flightmode->mode_number() == Mode::Number::POSHOLD);

    return init_internals(position_hold,
                          plopter.attitude_control,
                          plopter.pos_control,
                          plopter.ahrs_view,
                          &plopter.inertial_nav);
}

void AutoTune::run()
{
    // apply SIMPLE mode transform to pilot inputs
    plopter.update_simple_mode();

    // reset target lean angles and heading while landed
    if (plopter.ap.land_complete) {
        // we are landed, shut down
        float target_climb_rate = get_pilot_desired_climb_rate_cms();

        // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
        if (target_climb_rate < 0.0f) {
            plopter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        } else {
            plopter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        }
        plopter.attitude_control->reset_rate_controller_I_terms_smoothly();
        plopter.attitude_control->reset_yaw_target_and_rate();

        float target_roll, target_pitch, target_yaw_rate;
        get_pilot_desired_rp_yrate_cd(target_roll, target_pitch, target_yaw_rate);

        plopter.attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
        plopter.pos_control->relax_z_controller(0.0f);
        plopter.pos_control->update_z_controller();
    } else {
        // run autotune mode
        AC_AutoTune::run();
    }
}


/*
  get stick input climb rate
 */
float AutoTune::get_pilot_desired_climb_rate_cms(void) const
{
    float target_climb_rate = plopter.get_pilot_desired_climb_rate(plopter.channel_throttle->get_control_in());

    // get avoidance adjusted climb rate
    target_climb_rate = plopter.mode_autotune.get_avoidance_adjusted_climbrate(target_climb_rate);

    return target_climb_rate;
}

/*
  get stick roll, pitch and yaw rate
 */
void AutoTune::get_pilot_desired_rp_yrate_cd(float &des_roll_cd, float &des_pitch_cd, float &yaw_rate_cds)
{
    plopter.mode_autotune.get_pilot_desired_lean_angles(des_roll_cd, des_pitch_cd, plopter.aparm.angle_max,
                                                       plopter.attitude_control->get_althold_lean_angle_max_cd());
    yaw_rate_cds = plopter.mode_autotune.get_pilot_desired_yaw_rate(plopter.channel_yaw->norm_input_dz());
}

/*
  setup z controller velocity and accel limits
 */
void AutoTune::init_z_limits()
{
    // set vertical speed and acceleration limits
    plopter.pos_control->set_max_speed_accel_z(-plopter.get_pilot_speed_dn(), plopter.g.pilot_speed_up, plopter.g.pilot_accel_z);
    plopter.pos_control->set_correction_speed_accel_z(-plopter.get_pilot_speed_dn(), plopter.g.pilot_speed_up, plopter.g.pilot_accel_z);
}

void AutoTune::log_pids()
{
    plopter.logger.Write_PID(LOG_PIDR_MSG, plopter.attitude_control->get_rate_roll_pid().get_pid_info());
    plopter.logger.Write_PID(LOG_PIDP_MSG, plopter.attitude_control->get_rate_pitch_pid().get_pid_info());
    plopter.logger.Write_PID(LOG_PIDY_MSG, plopter.attitude_control->get_rate_yaw_pid().get_pid_info());
}

/*
  check if we have a good position estimate
 */
bool AutoTune::position_ok()
{
    return plopter.position_ok();
}

/*
  initialise autotune mode
*/
bool ModeAutoTune::init(bool ignore_checks)
{
    return autotune.init();
}

void ModeAutoTune::run()
{
    autotune.run();
}

void ModeAutoTune::save_tuning_gains()
{
    autotune.save_tuning_gains();
}

void ModeAutoTune::exit()
{
    autotune.stop();
}

void ModeAutoTune::reset()
{
    autotune.reset();
}

#endif  // AUTOTUNE_ENABLED == ENABLED
