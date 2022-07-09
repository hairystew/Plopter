/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_MotorsPlopter.cpp - ArduCopter motors library for tailsitters and bicopters
 *
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AP_MotorsPlopter.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define SERVO_OUTPUT_RANGE  4500

// init
void AP_MotorsPlopter::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // setup default motor and servo mappings
    _has_diff_thrust = SRV_Channels::function_assigned(SRV_Channel::k_throttleRight) || SRV_Channels::function_assigned(SRV_Channel::k_throttleLeft);

    // right throttle defaults to servo output 1
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_throttleRight, CH_1);

    // left throttle defaults to servo output 2
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_throttleLeft, CH_2);

    // right servo defaults to servo output 3
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorRight, CH_3);
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorRight, SERVO_OUTPUT_RANGE);

    // left servo defaults to servo output 4
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorLeft, CH_4);
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorLeft, SERVO_OUTPUT_RANGE);

    _mav_type = MAV_TYPE_PLOPTER;

    // record successful initialisation if what we setup was the desired frame_class
    set_initialised_ok(frame_class == MOTOR_FRAME_PLOPTER);
}


/// Constructor
AP_MotorsPlopter::AP_MotorsPlopter(uint16_t loop_rate, uint16_t speed_hz) :
        AP_MotorsMulticopter(loop_rate, speed_hz)
{
    set_update_rate(speed_hz);
}


// set update rate to motors - a value in hertz
void AP_MotorsPlopter::set_update_rate(uint16_t speed_hz)
{
    // record requested speed
    _speed_hz = speed_hz;

    SRV_Channels::set_rc_frequency(SRV_Channel::k_throttleLeft, speed_hz);
    SRV_Channels::set_rc_frequency(SRV_Channel::k_throttleRight, speed_hz);
}

void AP_MotorsPlopter::output_to_motors()
{
    if (!initialised_ok()) {
        return;
    }

    switch (_spool_state) {
        case SpoolState::SHUT_DOWN:
            _actuator[0] = 0.0f;
            _actuator[1] = 0.0f;
            _external_min_throttle = 0.0;
            break;
        case SpoolState::GROUND_IDLE:
            set_actuator_with_slew(_actuator[0], actuator_spin_up_to_ground_idle());
            set_actuator_with_slew(_actuator[1], actuator_spin_up_to_ground_idle());
            _external_min_throttle = 0.0;
            break;
        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
        case SpoolState::SPOOLING_DOWN:
            set_actuator_with_slew(_actuator[0], thrust_to_actuator(_thrust_left));
            set_actuator_with_slew(_actuator[1], thrust_to_actuator(_thrust_right));
            break;
    }

    SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, output_to_pwm(_actuator[0]));
    SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, output_to_pwm(_actuator[1]));
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, _tilt_left*SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, _tilt_right*SERVO_OUTPUT_RANGE);

}

// get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint32_t AP_MotorsPlopter::get_motor_mask()
{
    uint32_t motor_mask = 0;
    uint8_t chan;
    if (SRV_Channels::find_channel(SRV_Channel::k_throttleLeft, chan)) {
        motor_mask |= 1U << chan;
    }
    if (SRV_Channels::find_channel(SRV_Channel::k_throttleRight, chan)) {
        motor_mask |= 1U << chan;
    }

    // add parent's mask
    motor_mask |= AP_MotorsMulticopter::get_motor_mask();

    return motor_mask;
}

// calculate outputs to the motors
void AP_MotorsPlopter::output_armed_stabilizing()
{

//    // Add adjustment to reduce average throttle
//    _thrust_left  = constrain_float(_thrust_left, 0.0f, 1.0f);
//    _thrust_right = constrain_float(_thrust_right, 0.0f, 1.0f);
//
//    // thrust vectoring
//    _tilt_left  = pitch_thrust - yaw_thrust;
//    _tilt_right = pitch_thrust + yaw_thrust;
}

// output_test_seq - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsPlopter::_output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    // output to motors and servos
    switch (motor_seq) {
        case 1:
            // right throttle
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, pwm);
            break;
        case 2:
            // right tilt servo
            SRV_Channels::set_output_pwm(SRV_Channel::k_tiltMotorRight, pwm);
            break;
        case 3:
            // left throttle
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, pwm);
            break;
        case 4:
            // left tilt servo
            SRV_Channels::set_output_pwm(SRV_Channel::k_tiltMotorLeft, pwm);
            break;
        default:
            // do nothing
            break;
    }
}



void AP_MotorsPlopter::set_output(float thrust_left, float tilt_left, float thrust_right, float tilt_right)
{

    //tilt angles need to be mapped from -pi / 2 to pi / 2 in radians to +-1
    _tilt_left = constrain_float(tilt_left / (PI / 2.), -1, 1);
    _tilt_right = constrain_float(tilt_right / (PI / 2.), -1, 1);
    //thrust output from lqr controller is in N, need some conversion to map N in thrust to 0-100% throttle
    _thrust_left = constrain_float((thrust_left + (mass * 9.81 / 2)) / (1.35 * 9.81), 0, 1);
    _thrust_right = constrain_float((thrust_right + (mass * 9.81 / 2)) / (1.35 * 9.81), 0, 1);


    output_to_motors();
}