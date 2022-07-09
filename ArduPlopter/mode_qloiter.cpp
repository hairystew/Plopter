#include "mode.h"
#include "Plopter.h"

#if HAL_QUADPLANE_ENABLED

bool ModeQLoiter::_enter()
{
    // initialise loiter
    loiter_nav->clear_pilot_desired_acceleration();
    loiter_nav->init_target();

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-quadplopter.get_pilot_velocity_z_max_dn(), quadplopter.pilot_velocity_z_max_up, quadplopter.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-quadplopter.get_pilot_velocity_z_max_dn(), quadplopter.pilot_velocity_z_max_up, quadplopter.pilot_accel_z);

    quadplopter.init_throttle_wait();

    // prevent re-init of target position
    quadplopter.last_loiter_ms = AP_HAL::millis();
    return true;
}

void ModeQLoiter::update()
{
    plopter.mode_qstabilize.update();
}

// run quadplopter loiter controller
void ModeQLoiter::run()
{
    if (quadplopter.throttle_wait) {
        quadplopter.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0, true, 0);
        quadplopter.relax_attitude_control();
        pos_control->relax_z_controller(0);
        loiter_nav->clear_pilot_desired_acceleration();
        loiter_nav->init_target();
        return;
    }
    if (!quadplopter.motors->armed()) {
        plopter.mode_qloiter._enter();
    }

    if (quadplopter.should_relax()) {
        loiter_nav->soften_for_landing();
    }

    const uint32_t now = AP_HAL::millis();
    if (now - quadplopter.last_loiter_ms > 500) {
        loiter_nav->clear_pilot_desired_acceleration();
        loiter_nav->init_target();
    }
    quadplopter.last_loiter_ms = now;

    // motors use full range
    quadplopter.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-quadplopter.get_pilot_velocity_z_max_dn(), quadplopter.pilot_velocity_z_max_up, quadplopter.pilot_accel_z);

    // process pilot's roll and pitch input
    float target_roll_cd, target_pitch_cd;
    quadplopter.get_pilot_desired_lean_angles(target_roll_cd, target_pitch_cd, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());
    loiter_nav->set_pilot_desired_acceleration(target_roll_cd, target_pitch_cd);
    
    // run loiter controller
    if (!pos_control->is_active_xy()) {
        pos_control->init_xy_controller();
    }
    loiter_nav->update();

    // nav roll and pitch are controller by loiter controller
    plopter.nav_roll_cd = loiter_nav->get_roll();
    plopter.nav_pitch_cd = loiter_nav->get_pitch();

    if (quadplopter.transition->set_VTOL_roll_pitch_limit(plopter.nav_roll_cd, plopter.nav_pitch_cd)) {
        pos_control->set_externally_limited_xy();
    }

    // call attitude controller with conservative smoothing gain of 4.0f
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(plopter.nav_roll_cd,
                                                                  plopter.nav_pitch_cd,
                                                                  quadplopter.get_desired_yaw_rate_cds());

    if (plopter.control_mode == &plopter.mode_qland) {
        if (poscontrol.get_state() < QuadPlopter::QPOS_LAND_FINAL && quadplopter.check_land_final()) {
            poscontrol.set_state(QuadPlopter::QPOS_LAND_FINAL);
            quadplopter.setup_target_position();
            // cut IC engine if enabled
            if (quadplopter.land_icengine_cut != 0) {
                plopter.g2.ice_control.engine_control(0, 0, 0);
            }
        }
        float height_above_ground = plopter.relative_ground_altitude(plopter.g.rangefinder_landing);
        float descent_rate_cms = quadplopter.landing_descent_rate_cms(height_above_ground);

        if (poscontrol.get_state() == QuadPlopter::QPOS_LAND_FINAL && (quadplopter.options & QuadPlopter::OPTION_DISABLE_GROUND_EFFECT_COMP) == 0) {
            quadplopter.ahrs.set_touchdown_expected(true);
        }

        quadplopter.set_climb_rate_cms(-descent_rate_cms, descent_rate_cms>0);
        quadplopter.check_land_complete();
    } else if (plopter.control_mode == &plopter.mode_guided && quadplopter.guided_takeoff) {
        quadplopter.set_climb_rate_cms(0, false);
    } else {
        // update altitude target and call position controller
        quadplopter.set_climb_rate_cms(quadplopter.get_pilot_desired_climb_rate_cms(), false);
    }
    quadplopter.run_z_controller();
}

#endif
