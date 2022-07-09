#include "Plopter.h"
#include "qautotune.h"

#if QAUTOTUNE_ENABLED

/*
  initialise QAUTOTUNE mode
 */
bool QAutoTune::init()
{
    if (!plopter.quadplopter.available()) {
        return false;
    }

    // use position hold while tuning if we were in QLOITER
    bool position_hold = (plopter.previous_mode == &plopter.mode_qloiter);

    return init_internals(position_hold,
                          plopter.quadplopter.attitude_control,
                          plopter.quadplopter.pos_control,
                          plopter.quadplopter.ahrs_view,
                          &plopter.quadplopter.inertial_nav);
}

float QAutoTune::get_pilot_desired_climb_rate_cms(void) const
{
    return plopter.quadplopter.get_pilot_desired_climb_rate_cms();
}

void QAutoTune::get_pilot_desired_rp_yrate_cd(float &des_roll_cd, float &des_pitch_cd, float &yaw_rate_cds)
{
    if (plopter.channel_roll->get_control_in() == 0 && plopter.channel_pitch->get_control_in() == 0) {
        des_roll_cd = 0;
        des_pitch_cd = 0;
    } else {
        des_roll_cd = plopter.nav_roll_cd;
        des_pitch_cd = plopter.nav_pitch_cd;
    }
    yaw_rate_cds = plopter.quadplopter.get_desired_yaw_rate_cds();
}

void QAutoTune::init_z_limits()
{
    // set vertical speed and acceleration limits
    plopter.quadplopter.pos_control->set_max_speed_accel_z(-plopter.quadplopter.get_pilot_velocity_z_max_dn(),
        plopter.quadplopter.pilot_velocity_z_max_up,
        plopter.quadplopter.pilot_accel_z);
    plopter.quadplopter.pos_control->set_correction_speed_accel_z(-plopter.quadplopter.get_pilot_velocity_z_max_dn(),
        plopter.quadplopter.pilot_velocity_z_max_up,
        plopter.quadplopter.pilot_accel_z);
}


// log VTOL PIDs for during twitch
void QAutoTune::log_pids(void)
{
    AP::logger().Write_PID(LOG_PIQR_MSG, plopter.quadplopter.attitude_control->get_rate_roll_pid().get_pid_info());
    AP::logger().Write_PID(LOG_PIQP_MSG, plopter.quadplopter.attitude_control->get_rate_pitch_pid().get_pid_info());
    AP::logger().Write_PID(LOG_PIQY_MSG, plopter.quadplopter.attitude_control->get_rate_yaw_pid().get_pid_info());
}

#endif // QAUTOTUNE_ENABLED

