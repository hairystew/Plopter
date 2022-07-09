#include "mode.h"
#include "Plopter.h"

#define SERVO_OUTPUT_RANGE  4500

bool ModeBaba::_enter()
{
    plopter.guided_throttle_passthru = false;
    /*
      when entering guided mode we set the target as the current
      location. This matches the behaviour of the copter code
    */
    Location loc{plopter.current_loc};

    plopter.set_guided_WP(loc);

    state_offset = sm::Matrix<float>(12, 1, 0);
    state = sm::Matrix<float>(12, 1, 0);
    K_Gain_Matrix = sm::Matrix<float>(4, 12, 0);
    Offset_Forcing_Matrix = sm::Matrix<float>(4, 12, 0);
    uD = sm::Matrix<float>(4, 0, 0);
    u = sm::Matrix<float>(4, 0, 0);

    /*K
[-0.0000   -0.2236   -0.2236   -0.0000   -0.8510   -0.7345   -4.9191    0.0000   -0.0000   -1.4197    0.0000   -0.0000;
   -0.2235    0.0000    0.0000   -0.8507    0.0000   -0.0000    0.0000    4.9187    0.7074    0.0000    1.4197    1.4157;
    0.0000    0.2236   -0.2236    0.0000    0.8510   -0.7345    4.9191   -0.0000    0.0000    1.4197   -0.0000    0.0000;
   -0.2237    0.0000    0.0000   -0.8513    0.0000   -0.0000    0.0000    4.9186   -0.7068    0.0000    1.4197   -1.4140]
forcing
[0         0         0         0         0         0         0         0         0         0         0         0;
 0         0         0         0         0         0         0    0.00003285         0         0         0         0;
 0         0         0         0         0         0         0         0         0         0         0         0;
 0         0         0         0         0         0         0    0.00000775         0         0         0         0]*/


    //DONT HARDCODE THESE
    K_Gain_Matrix(0, 0) = 0.0;      K_Gain_Matrix(0, 1) = -0.2236;  K_Gain_Matrix(0, 2) = -0.2236;  K_Gain_Matrix(0, 3) = 0.0;      K_Gain_Matrix(0, 4) = -0.8510;  K_Gain_Matrix(0, 5) = -0.7345;  K_Gain_Matrix(0, 6) = -4.9191;  K_Gain_Matrix(0, 7) = 0.0;      K_Gain_Matrix(0, 8) = 0.0;      K_Gain_Matrix(0, 9) = -1.4197;  K_Gain_Matrix(0, 10) = 0.0;K_Gain_Matrix(0, 11) = 0.0;
    K_Gain_Matrix(1, 0) = -0.2235;  K_Gain_Matrix(1, 1) = 0.0;      K_Gain_Matrix(1, 2) = 0.0;      K_Gain_Matrix(1, 3) = -0.8507;  K_Gain_Matrix(1, 4) = 0.0;      K_Gain_Matrix(1, 5) = 0.0;      K_Gain_Matrix(1, 6) = 0.0;      K_Gain_Matrix(1, 7) = 4.9187;   K_Gain_Matrix(1, 8) = 0.7074;   K_Gain_Matrix(1, 9) = 0.0;      K_Gain_Matrix(1, 10) = 1.4197;K_Gain_Matrix(1, 11) = 1.4157;
    K_Gain_Matrix(2, 0) = 0.0;      K_Gain_Matrix(2, 1) = 0.2236;   K_Gain_Matrix(2, 2) = -0.2236;  K_Gain_Matrix(2, 3) = 0.0;      K_Gain_Matrix(2, 4) = 0.8510;   K_Gain_Matrix(2, 5) = -0.7345;  K_Gain_Matrix(2, 6) = 4.9191;   K_Gain_Matrix(2, 7) = 0.0;      K_Gain_Matrix(2, 8) = 0.0;      K_Gain_Matrix(2, 9) = 1.4197;   K_Gain_Matrix(2, 10) = 0.0;K_Gain_Matrix(2, 11) = 0.0;
    K_Gain_Matrix(3, 0) = -0.2237;  K_Gain_Matrix(3, 1) = 0.0;      K_Gain_Matrix(3, 2) = 0.0;      K_Gain_Matrix(3, 3) = -0.8513;  K_Gain_Matrix(3, 4) = 0.0;      K_Gain_Matrix(3, 5) = 0.0;      K_Gain_Matrix(3, 6) = 0.0;      K_Gain_Matrix(3, 7) = 4.9186;   K_Gain_Matrix(3, 8) = -0.7068;  K_Gain_Matrix(3, 9) = 0.0;      K_Gain_Matrix(3, 10) = 1.4197;K_Gain_Matrix(3, 11) = -1.4140;

    Offset_Forcing_Matrix(1,7) = 0.00003285;
    Offset_Forcing_Matrix(3,7) = 0.00000775;


    // right throttle defaults to servo output 1
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_throttleRight, CH_1);

    // left throttle defaults to servo output 2
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_throttleLeft, CH_2);

    // right servo defaults to servo output 3
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorRight, CH_3);
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorRight, 0.);

    // left servo defaults to servo output 4
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorLeft, CH_4);
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorLeft, 0.);


    return true;
}

void ModeBaba::update()
{
    AP::ahrs().get_velocity_NED(velVec);
    AP::ahrs().get_relative_position_NED_origin(relPosVec);


    state(0, 0)  = relPosVec.x;
    state(1, 0)  = relPosVec.y;
    state(2, 0)  = relPosVec.z;
    state(3, 0)  = velVec.x;
    state(4, 0)  = velVec.y;
    state(5, 0)  = velVec.z;
    state(6, 0)  = AP::ahrs().get_roll();
    state(7, 0)  = AP::ahrs().get_pitch();
    state(8, 0)  = AP::ahrs().get_yaw();
    state(9, 0)  = AP::ahrs().get_gyro().x;
    state(10, 0) = AP::ahrs().get_gyro().y;
    state(11, 0) = AP::ahrs().get_gyro().z;

    //Convert Current and Desired States into Body Frame (COME BACK TO THIS)
    //Matrix Math
    desired_state = state + state_offset;
    uD = Offset_Forcing_Matrix * desired_state;
    u = (K_Gain_Matrix * state) - uD;
    //Convert Matrix Mult. Output to Thrusts and Angles
    right_thrust = sqrt(pow(u(0,0),2) + pow(u(1,0),2));
    left_thrust = sqrt(pow(u(2,0),2) + pow(u(3,0),2));
    right_angle = atan2(u(1, 0), u(0, 0));
    left_angle = atan2(u(3, 0), u(2, 0));


    //tilt angles need to be mapped from -pi / 2 to pi / 2 in radians to +-1
    left_angle = constrain_float(left_angle / (PI / 2.), -1, 1);
    right_angle = constrain_float(right_angle / (PI / 2.), -1, 1);
    //thrust output from lqr controller is in N, need some conversion to map N in thrust to 0-100% throttle
    left_thrust = constrain_float((left_thrust + (mass * 9.81 / 2)) / (1.35 * 9.81), 0, 1);
    right_thrust = constrain_float((right_thrust + (mass * 9.81 / 2)) / (1.35 * 9.81), 0, 1);

    SRV_Channels::set_output_norm(SRV_Channel::k_throttleLeft, left_thrust);
    SRV_Channels::set_output_norm(SRV_Channel::k_throttleRight, right_thrust);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, left_angle*SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, right_angle*SERVO_OUTPUT_RANGE);

}

void ModeBaba::navigate()
{

}

bool ModeBaba::handle_guided_request(Location target_loc)
{

    return true;
}
