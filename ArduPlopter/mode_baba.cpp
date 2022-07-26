#include "mode.h"
#include "Plopter.h"

#define SERVO_OUTPUT_RANGE  4500


ModeBaba::ModeBaba() :
        Mode()
{
}


bool ModeBaba::_enter()
{
    std::cout << "SET BABA\n";
    plopter.guided_throttle_passthru = false;
    /*
      when entering guided mode we set the target as the current
      location. This matches the behaviour of the copter code
    */
    Location loc{plopter.current_loc};

    plopter.set_guided_WP(loc);
    state_offset = sm::Matrix<float>(12, 1, 0);
    desired_state = sm::Matrix<float>(12, 1, 0);
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








    K_Gain_Matrix(0, 0) = 0.000000;
    K_Gain_Matrix(0, 1) = -0.035355;
    K_Gain_Matrix(0, 2) = -0.035355;
    K_Gain_Matrix(0, 3) = 0.000000;
    K_Gain_Matrix(0, 4) = -0.128550;
    K_Gain_Matrix(0, 5) = -0.137011;
    K_Gain_Matrix(0, 6) = -0.558427;
    K_Gain_Matrix(0, 7) = -0.000000;
    K_Gain_Matrix(0, 8) = -0.000000;
    K_Gain_Matrix(0, 9) = -0.119099;
    K_Gain_Matrix(0, 10) = -0.000000;
    K_Gain_Matrix(0, 11) = -0.000000;
    K_Gain_Matrix(1, 0) = -0.024983;
    K_Gain_Matrix(1, 1) = -0.000000;
    K_Gain_Matrix(1, 2) = -0.000000;
    K_Gain_Matrix(1, 3) = -0.090671;
    K_Gain_Matrix(1, 4) = -0.000000;
    K_Gain_Matrix(1, 5) = -0.000000;
    K_Gain_Matrix(1, 6) = -0.000000;
    K_Gain_Matrix(1, 7) = 0.388680;
    K_Gain_Matrix(1, 8) = 0.079111;
    K_Gain_Matrix(1, 9) = -0.000000;
    K_Gain_Matrix(1, 10) = 0.081516;
    K_Gain_Matrix(1, 11) = 0.080367;
    K_Gain_Matrix(2, 0) = -0.000000;
    K_Gain_Matrix(2, 1) = 0.035355;
    K_Gain_Matrix(2, 2) = -0.035355;
    K_Gain_Matrix(2, 3) = -0.000000;
    K_Gain_Matrix(2, 4) = 0.128550;
    K_Gain_Matrix(2, 5) = -0.137011;
    K_Gain_Matrix(2, 6) = 0.558427;
    K_Gain_Matrix(2, 7) = 0.000000;
    K_Gain_Matrix(2, 8) = 0.000000;
    K_Gain_Matrix(2, 9) = 0.119099;
    K_Gain_Matrix(2, 10) = 0.000000;
    K_Gain_Matrix(2, 11) = -0.000000;
    K_Gain_Matrix(3, 0) = -0.025017;
    K_Gain_Matrix(3, 1) = 0.000000;
    K_Gain_Matrix(3, 2) = -0.000000;
    K_Gain_Matrix(3, 3) = -0.090778;
    K_Gain_Matrix(3, 4) = 0.000000;
    K_Gain_Matrix(3, 5) = -0.000000;
    K_Gain_Matrix(3, 6) = 0.000000;
    K_Gain_Matrix(3, 7) = 0.388638;
    K_Gain_Matrix(3, 8) = -0.079003;
    K_Gain_Matrix(3, 9) = 0.000000;
    K_Gain_Matrix(3, 10) = 0.081516;
    K_Gain_Matrix(3, 11) = -0.080202;

    Offset_Forcing_Matrix(1,7) = 0.00002952;
    Offset_Forcing_Matrix(3,7) = 0.00000754;


    // right throttle defaults to servo output 1
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor1, CH_1);
    SRV_Channels::set_output_limit(SRV_Channel::k_motor1, SRV_Channel::Limit::MIN);
    SRV_Channels::set_angle(SRV_Channel::k_motor1, 100.);
    //SRV_Channels::set_range(SRV_Channel::k_throttleRight, 1000);


    // left throttle defaults to servo output 2
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor2, CH_2);
    SRV_Channels::set_output_limit(SRV_Channel::k_motor2, SRV_Channel::Limit::MIN);
    SRV_Channels::set_angle(SRV_Channel::k_motor2, 100.);
    //SRV_Channels::set_range(SRV_Channel::k_throttleLeft, 1000);

    // right servo defaults to servo output 3
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorRight, CH_3);
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorRight, 4500.);

    // left servo defaults to servo output 4
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltMotorLeft, CH_4);
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorLeft, 4500.);

    return true;
}

void ModeBaba::update()
{
    AP::ahrs().get_velocity_NED(velNED);
    AP::ahrs().get_relative_position_NED_origin(relPosNED);
    //copying over for readability
    velBody = AP::ahrs().earth_to_body(velNED);
    relPosBody = AP::ahrs().earth_to_body(relPosNED);


    Vector3f desPos(0., 0., -1.);
    desPos = AP::ahrs().earth_to_body(desPos);

    state(0, 0)  = relPosBody.x;
    state(1, 0)  = relPosBody.y;
    state(2, 0)  = relPosBody.z;
    state(3, 0)  = velNED.x;
    state(4, 0)  = velNED.y;
    state(5, 0)  = velNED.z;
    state(6, 0)  = AP::ahrs().get_roll();
    state(7, 0)  = AP::ahrs().get_pitch();
    state(8, 0)  = AP::ahrs().get_yaw();
    state(9, 0)  = AP::ahrs().get_gyro().x;
    state(10, 0) = AP::ahrs().get_gyro().y;
    state(11, 0) = AP::ahrs().get_gyro().z;
    //Convert Current and Desired States into Body Frame (COME BACK TO THIS)
    //Matrix Math
    desired_state(0, 0) = desPos.x;
    desired_state(1, 0) = desPos.y;
    desired_state(2, 0) = desPos.z;
    desired_state(6, 0) = plopter.channel_roll->norm_input() * PI / 18.;
    desired_state(7, 0) = -plopter.channel_pitch->norm_input()  * PI / 18.;
    desired_state(8, 0) += (.01 * plopter.channel_rudder->norm_input());
    state_offset = desired_state - state;
    state_offset(0,0) = constrain_float(state_offset(0,0), -1., 1.);
    state_offset(1,0) = constrain_float(state_offset(1,0), -1., 1.);
    state_offset(2,0) = constrain_float(state_offset(2,0), -1., 1.);
    state_offset(3,0) = constrain_float(state_offset(3,0), -1., 1.);
    state_offset(4,0) = constrain_float(state_offset(4,0), -1., 1.);
    state_offset(5,0) = constrain_float(state_offset(5,0), -1., 1.);
//
    std::cout << "\n\n\n\n\n";
//    state_offset.print();
    state.print();

    uD = Offset_Forcing_Matrix * desired_state;
    u = (K_Gain_Matrix * state_offset) - uD;
    //Convert Matrix Mult. Output to Thrusts and Angles
    right_thrust = sqrt(pow(u(0,0) + (mass / 2.),2) + pow(u(1,0),2));
    left_thrust = sqrt(pow(u(2,0) + (mass / 2.),2) + pow(u(3,0),2));
//    u.print();
    //taking LQR desired force in N, converting from 0 to 100% Throttle. Estimating 100% throttle 1.35 * 9.81 N
    left_thrust = constrain_float(left_thrust / .8, 0, .8);
    right_thrust = constrain_float(right_thrust / .8, 0, .8);

//    std::cout << "\n\n\n\n\n";
//    state.print();
//    std::cout << "\n\n\n\n\n";
//    u.print();
//    std::cout << "\n\n\n\n\n";
    right_angle = atan2(u(1, 0), right_thrust);
    left_angle = atan2(u(3, 0), left_thrust);
    //mapping 0 - 1 to -100 to 100
    left_thrust = (200. * left_thrust) - 100.;
    right_thrust = (200 * right_thrust) - 100.;
    //tilt angles need to be mapped from -pi / 2 to pi / 2 in radians to +-1
    left_angle = constrain_float(left_angle / (PI / 2.), -.5, .5);
    right_angle = constrain_float(right_angle / (PI / 2.), -.5, .5);
    //thrust output from lqr controller is in N, need some conversion to map N in thrust to 0-100% throttle
//    std::cout << left_thrust << ", " << right_thrust << ", " << left_angle << ", " << right_angle << std::endl;
    left_accum = (1. * left_angle) + (0. * left_accum);
    right_accum = (1. * right_angle) + (0. * right_accum);


    if(plopter.arming.is_armed()) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_motor2,
                                        ((left_thrust + 100.) * ((plopter.channel_throttle->norm_input() + 1) / 2)) - 100.);
        SRV_Channels::set_output_scaled(SRV_Channel::k_motor1,
                                        ((right_thrust + 100.) * ((plopter.channel_throttle->norm_input() + 1) / 2)) - 100.);
//        SRV_Channels::set_output_scaled(SRV_Channel::k_motor2, plopter.channel_throttle->norm_input() * 100.);
//        SRV_Channels::set_output_scaled(SRV_Channel::k_motor1, plopter.channel_throttle->norm_input() * 100.);
    } else {
            SRV_Channels::set_output_scaled(SRV_Channel::k_motor2, -100.);
            SRV_Channels::set_output_scaled(SRV_Channel::k_motor1, -100.);
    }
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, -left_accum * SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, right_accum * SERVO_OUTPUT_RANGE);
//    std::cout << SRV_Channels::get_output_scaled(SRV_Channel::k_motor2) << " " <<
//        SRV_Channels::get_output_scaled(SRV_Channel::k_motor1) << std::endl;
//    std::cout << plopter.channel_throttle->norm_input() << std::endl;
//    std::cout << "\n\n\n\n\n\n\n";
//    std::cout << "RPY: " << AP::ahrs().get_roll() << " " << AP::ahrs().get_pitch() << " " << AP::ahrs().get_yaw() << std::endl;
//    std::cout << "dt RPY: " << AP::ahrs().get_gyro().x << " " << AP::ahrs().get_gyro().y << " " << AP::ahrs().get_gyro().z << std::endl;
//    std::cout << (left_thrust + 100.) * ((plopter.channel_throttle->norm_input() + 1) / 2) - 100. << " " <<
//                  (right_thrust + 100.) * ((plopter.channel_throttle->norm_input() + 1) / 2) - 100. << std::endl;
//    std::cout << left_thrust << " " <<
//              right_thrust << std::endl;
//    state_offset.print();


}

void ModeBaba::navigate()
{

}

bool ModeBaba::handle_guided_request(Location target_loc)
{
    Vector3f desPos;
    target_loc.get_vector_from_origin_NEU(desPos);
    desired_state(0, 0) = desPos.x;
    desired_state(1, 0) = desPos.y;
    desired_state(2, 0) = desPos.z;
    return true;
}
