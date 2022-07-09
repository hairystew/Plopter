/*
  plopter specific AP_AdvancedFailsafe class
 */

#include "Plopter.h"

#if ADVANCED_FAILSAFE == ENABLED

/*
  setup radio_out values for all channels to termination values
 */
void AP_AdvancedFailsafe_Plopter::terminate_vehicle(void)
{
#if HAL_QUADPLANE_ENABLED
    if (plopter.quadplopter.available() && _terminate_action == TERMINATE_ACTION_LAND) {
        // perform a VTOL landing
        plopter.set_mode(plopter.mode_qland, ModeReason::FENCE_BREACHED);
        return;
    }
#endif

    plopter.g2.servo_channels.disable_passthrough(true);
    
    if (_terminate_action == TERMINATE_ACTION_LAND) {
        plopter.landing.terminate();
    } else {
        // remove flap slew limiting
        SRV_Channels::set_slew_rate(SRV_Channel::k_flap_auto, 0.0, 100, plopter.G_Dt);
        SRV_Channels::set_slew_rate(SRV_Channel::k_flap, 0.0, 100, plopter.G_Dt);

        // aerodynamic termination is the default approach to termination
        SRV_Channels::set_output_scaled(SRV_Channel::k_flap_auto, 100.0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_flap, 100.0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, SERVO_MAX);
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, SERVO_MAX);
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, SERVO_MAX);
        if (plopter.have_reverse_thrust()) {
            // configured for reverse thrust, use TRIM
            SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::TRIM);
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleLeft, SRV_Channel::Limit::TRIM);
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleRight, SRV_Channel::Limit::TRIM);
        } else {
            // use MIN
            SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::MIN);
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleLeft, SRV_Channel::Limit::MIN);
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleRight, SRV_Channel::Limit::MIN);
        }
        SRV_Channels::set_output_limit(SRV_Channel::k_manual, SRV_Channel::Limit::TRIM);
        SRV_Channels::set_output_limit(SRV_Channel::k_none, SRV_Channel::Limit::TRIM);
    }

    plopter.servos_output();

#if HAL_QUADPLANE_ENABLED
    plopter.quadplopter.afs_terminate();
#endif

    // also disarm to ensure that ignition is cut
    plopter.arming.disarm(AP_Arming::Method::AFS);
}

void AP_AdvancedFailsafe_Plopter::setup_IO_failsafe(void)
{
    // all aux channels
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_flap_auto, SRV_Channel::Limit::MAX);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_flap, SRV_Channel::Limit::MAX);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_aileron, SRV_Channel::Limit::MIN);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_rudder, SRV_Channel::Limit::MAX);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_elevator, SRV_Channel::Limit::MAX);
    if (plopter.have_reverse_thrust()) {
        // configured for reverse thrust, use TRIM
        SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::TRIM);
        SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttleLeft, SRV_Channel::Limit::TRIM);
        SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttleRight, SRV_Channel::Limit::TRIM);
    } else {
        // normal throttle, use MIN
        SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::MIN);
        SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttleLeft, SRV_Channel::Limit::MIN);
        SRV_Channels::set_failsafe_limit(SRV_Channel::k_throttleRight, SRV_Channel::Limit::MIN);
    }
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_manual, SRV_Channel::Limit::TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_none, SRV_Channel::Limit::TRIM);

#if HAL_QUADPLANE_ENABLED
    if (plopter.quadplopter.available()) {
        // setup AP_Motors outputs for failsafe
        uint32_t mask = plopter.quadplopter.motors->get_motor_mask();
        hal.rcout->set_failsafe_pwm(mask, plopter.quadplopter.motors->get_pwm_output_min());
    }
#endif
}

/*
  return an AFS_MODE for current control mode
 */
AP_AdvancedFailsafe::control_mode AP_AdvancedFailsafe_Plopter::afs_mode(void)
{
    if (plopter.control_mode->does_auto_throttle()) {
        return AP_AdvancedFailsafe::AFS_AUTO;
    }
    if (plopter.control_mode == &plopter.mode_manual) {
        return AP_AdvancedFailsafe::AFS_MANUAL;
    }
    return AP_AdvancedFailsafe::AFS_STABILIZED;
}
#endif // ADVANCED_FAILSAFE
