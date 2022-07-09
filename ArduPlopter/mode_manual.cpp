#include "mode.h"
#include "Plopter.h"

void ModeManual::update()
{
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plopter.roll_in_expo(false));
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plopter.pitch_in_expo(false));
    plopter.steering_control.steering = plopter.steering_control.rudder = plopter.rudder_in_expo(false);

    plopter.nav_roll_cd = plopter.ahrs.roll_sensor;
    plopter.nav_pitch_cd = plopter.ahrs.pitch_sensor;
}

