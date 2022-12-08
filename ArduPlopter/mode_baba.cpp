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
    uD = sm::Matrix<float>(4, 1, 0);
    u = sm::Matrix<float>(4, 1, 0);
    integrated_offset = sm::Matrix<float>(4, 1, 0);
    integrated_force = sm::Matrix<float>(4, 1, 0);



    K_Gain_Matrix(0, 0) = 0.000000;
    K_Gain_Matrix(0, 1) = -0.070711;
    K_Gain_Matrix(0, 2) = -0.223607;
    K_Gain_Matrix(0, 3) = 0.000000;
    K_Gain_Matrix(0, 4) = -0.293460;
    K_Gain_Matrix(0, 5) = -0.596201;
    K_Gain_Matrix(0, 6) = -2.505456;
    K_Gain_Matrix(0, 7) = -0.000000;
    K_Gain_Matrix(0, 8) = 0.000000;
    K_Gain_Matrix(0, 9) = -1.003772;
    K_Gain_Matrix(0, 10) = -0.000000;
    K_Gain_Matrix(0, 11) = -0.000000;
    K_Gain_Matrix(1, 0) = -0.100000;
    K_Gain_Matrix(1, 1) = -0.000000;
    K_Gain_Matrix(1, 2) = 0.000000;
    K_Gain_Matrix(1, 3) = -0.414901;
    K_Gain_Matrix(1, 4) = -0.000000;
    K_Gain_Matrix(1, 5) = 0.000000;
    K_Gain_Matrix(1, 6) = -0.000000;
    K_Gain_Matrix(1, 7) = 3.538593;
    K_Gain_Matrix(1, 8) = 1.000000;
    K_Gain_Matrix(1, 9) = -0.000000;
    K_Gain_Matrix(1, 10) = 1.415529;
    K_Gain_Matrix(1, 11) = 1.001236;
    K_Gain_Matrix(2, 0) = -0.000000;
    K_Gain_Matrix(2, 1) = 0.070711;
    K_Gain_Matrix(2, 2) = -0.223607;
    K_Gain_Matrix(2, 3) = -0.000000;
    K_Gain_Matrix(2, 4) = 0.293460;
    K_Gain_Matrix(2, 5) = -0.596201;
    K_Gain_Matrix(2, 6) = 2.505456;
    K_Gain_Matrix(2, 7) = 0.000000;
    K_Gain_Matrix(2, 8) = 0.000000;
    K_Gain_Matrix(2, 9) = 1.003772;
    K_Gain_Matrix(2, 10) = 0.000000;
    K_Gain_Matrix(2, 11) = 0.000000;
    K_Gain_Matrix(3, 0) = -0.100000;
    K_Gain_Matrix(3, 1) = -0.000000;
    K_Gain_Matrix(3, 2) = 0.000000;
    K_Gain_Matrix(3, 3) = -0.414901;
    K_Gain_Matrix(3, 4) = -0.000000;
    K_Gain_Matrix(3, 5) = 0.000000;
    K_Gain_Matrix(3, 6) = -0.000000;
    K_Gain_Matrix(3, 7) = 3.538593;
    K_Gain_Matrix(3, 8) = -1.000000;
    K_Gain_Matrix(3, 9) = -0.000000;
    K_Gain_Matrix(3, 10) = 1.415529;
    K_Gain_Matrix(3, 11) = -1.001236;


    Offset_Forcing_Matrix(1,7) = 0.00002952;
    Offset_Forcing_Matrix(3,7) = 0.00000754;

    //TESTING INTEGRAL ACTION
    uD(0, 0) = 6.7004;
    uD(2, 0) = 6.7004;
    K_Integral_Action_Matrix = sm::Matrix<float>(4, 12, 0);
//    K_Integral_Action_Matrix(0,6) = -1.;
//    K_Integral_Action_Matrix(2,6) = 1.;
//    K_Integral_Action_Matrix(1,7) = .35;
//    K_Integral_Action_Matrix(3,7) = .35;
//    K_Integral_Action_Matrix(1,8) = .35;
//    K_Integral_Action_Matrix(3,8) = -.35;
    K_Integral_Action_Matrix(0,2) = 0.0;
    K_Integral_Action_Matrix(2,2) = 0.0;

    //END TESTING INTEGRAL ACTION



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


    plopter.optflow.start_calibration();
    plopter.rangefinder_state.in_use = true;
    plopter.terrain_disabled();

    return true;
}

void ModeBaba::update()
{
    plopter.ahrs.get_relative_position_NED_origin(relPosNED);
    plopter.ahrs.EKF3.getVelNED(velBody);
    //plopter.ahrs.EKF3.getPosNE(posNE);
    plopter.ahrs.EKF3.getPosD(posD);

//    // get corrected raw flow rate
//    Vector2f raw_flow = plopter.optflow.flowRate() - plopter.optflow.bodyRate();
//
//    // limit sensor flow, this prevents oscillation at low altitudes
//    raw_flow.x = constrain_float(raw_flow.x, -0.6, 0.6);
//    raw_flow.y = constrain_float(raw_flow.y, -0.6, 0.6);
//
//    // filter the flow rate
//    Vector2f sensor_flow = flow_filter.apply(raw_flow);
//
//    // scale by height estimate, limiting it to height_min to height_max
//    float ins_height = plopter.relative_ground_altitude(true) * 0.01;
//    float height_estimate = ins_height + height_offset;
//
//    // compensate for height, this converts to (approx) m/s
//    sensor_flow *= constrain_float(height_estimate, 0, 2);
//
//    // rotate controller input to earth frame
////    Vector2f input_ef = plopter.ahrs.body_to_earth2D(sensor_flow);



    //copying over for readability
//    velBody = AP::ahrs().earth_to_body(velNED);
//    relPosBody = AP::ahrs().earth_to_body(relPosNED);


    Vector3f desPos(0., 0., -0.5);
    desPos = AP::ahrs().earth_to_body(desPos);
    plopter.ahrs.get_velocity_NED(velNED);

//    std::cout << plopter.rangefinder_state.height_estimate << " BRRRRRRRRRUHUHUHUHUH\n";
    if ((plopter.rangefinder.status_orient(ROTATION_PITCH_270) == RangeFinder::Status::Good)) {
        state(2,0) = plopter.relative_ground_altitude(true);
    } else {
        state(2, 0)  = 0;
    }

//    state(0, 0)  = relPosBody.x;
//    state(1, 0)  = relPosBody.y;

//    state(3, 0)  = velNED.x;
//    state(4, 0)  = velNED.y;
//    state(5, 0)  = velNED.z;
    state(6, 0)  = AP::ahrs().get_roll();
    state(7, 0)  = AP::ahrs().get_pitch();
    state(8, 0)  = AP::ahrs().get_yaw();
    state(9, 0)  = AP::ahrs().get_gyro().x;
    state(10, 0) = AP::ahrs().get_gyro().y;
    state(11, 0) = AP::ahrs().get_gyro().z;
    //Convert Current and Desired States into Body Frame (COME BACK TO THIS)
    //Matrix Math
//    desired_state(0, 0) = desPos.x;
//    desired_state(1, 0) = desPos.y;
    desired_state(2, 0) = .3;
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
    //std::cout << -plopter.channel_roll->norm_input()  * PI / 18. << std::endl;
//
//    state_offset.print();
//    state.print();

    uint32_t tnow = AP_HAL::millis();
    uint32_t dt = tnow - _last_t;
    float delta_time = 0;

    if (_last_t == 0 || dt > 1000) {
        dt = 0;

        // if this hasn't been used for a full second then zero
        // the intergator term. This prevents I buildup from a
        // previous fight mode from causing a massive return before
        // the integrator gets a chance to correct itself
        integrated_force = sm::Matrix<float>(4, 1, 0);
    }
    _last_t = tnow;
    delta_time = (float)(dt) * 0.001f;
    u = (K_Gain_Matrix * state_offset) + (integrated_force) + uD;

    right_thrust = u(0,0);
    left_thrust = u(2,0);

    //taking LQR desired force in N, converting from 0 to 100% Throttle. Estimating 100% throttle 0.9 * 9.81 N
    left_thrust = constrain_float(left_thrust / (0.9 * 9.81), 0, 1.);
    right_thrust = constrain_float(right_thrust / (0.9 * 9.81), 0, 1.);


    right_angle = atan2(u(1, 0), u(0,0));
    left_angle = atan2(u(3, 0), u(2,0));
    //mapping 0 - 1 to -100 to 100
    left_thrust = (200. * left_thrust) - 100.;
    right_thrust = (200 * right_thrust) - 100.;
    //tilt angles need to be mapped from -pi / 2 to pi / 2 in radians to +-1
    left_angle = constrain_float(left_angle / (PI / 2.), -.5, .5);
    right_angle = constrain_float(right_angle / (PI / 2.), -.5, .5);
    //thrust output from lqr controller is in N, need some conversion to map N in thrust to 0-100% throttle
    throttle = plopter.channel_throttle->norm_input();


//    integrated_force += (K_Integral_Action_Matrix * state_offset) * delta_time;
//    std::cout << "\n\n\n\n\n" << state(2,0) << std::endl;
//    u.print();
//




    if(plopter.arming.is_armed()) {


        // throttle input will scale from zero to two with a flat portion at unity from .3 to .7 on the input
        if(throttle < .3){
            thrust_scalar = throttle / .3;
        } else if(throttle > .7){
            thrust_scalar = (throttle - .4) / .3;
        } else {
            integrated_force += (K_Integral_Action_Matrix * state_offset) * delta_time;
            thrust_scalar = 1.;
        }



        SRV_Channels::set_output_scaled(SRV_Channel::k_motor2,
            ((left_thrust + 100.) * thrust_scalar) - 100.);
        SRV_Channels::set_output_scaled(SRV_Channel::k_motor1,
            ((right_thrust + 100.) * thrust_scalar) - 100.);
//        SRV_Channels::set_output_scaled(SRV_Channel::k_motor2, plopter.channel_throttle->norm_input() * 100.);
//        SRV_Channels::set_output_scaled(SRV_Channel::k_motor1, plopter.channel_throttle->norm_input() * 100.);
    } else {
            integrated_force = sm::Matrix<float>(4, 1, 0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_motor2, -100.);
            SRV_Channels::set_output_scaled(SRV_Channel::k_motor1, -100.);
    }
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, -left_angle * SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, right_angle * SERVO_OUTPUT_RANGE);
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
//    std::cout << "OptFlow Health: " << plopter.optflow.healthy() << std::endl;
//    std::cout << "OptFlow Enabled: " << plopter.optflow.bodyRate().x << ", " << plopter.optflow.bodyRate().y << std::endl;
//    std::cout << plopter.optflow.enabled()  << " " << plopter.optflow.healthy() << "JOPEEEEE\n";
    velBod.x = (plopter.optflow.flowRate().x - plopter.optflow.bodyRate().x) * 0.01;
    velBod.y = (plopter.optflow.flowRate().y - plopter.optflow.bodyRate().y) * 0.01;
    velNED.x = velBod.x * cos(state(8,0)) + velBod.y * sin(state(8,0));
    velNED.y = velBod.x * sin(state(8,0)) - velBod.y * cos(state(8,0));
    posNE.x += (plopter.optflow.flowRate().x - plopter.optflow.bodyRate().x) * 0.01;
    posNE.y += (plopter.optflow.flowRate().y - plopter.optflow.bodyRate().y) * 0.01;
    bruhx += velNED.x;
    bruhy += velNED.y;
    //std::cout << "X: " << velNED.x << " Y: " << velNED.y << " Z: " << plopter.relative_ground_altitude(true) << std::endl;
    //std::cout << "X: " << plopter.optflow.flowRate().x << " Y: " << plopter.optflow.flowRate().y << " Z: " << plopter.relative_ground_altitude(true) << std::endl;
    //BRUHX AND BRUHY ARE KINDA LIKE POS IN CM
//    std::cout << "\n\n\n\n\n\n";
//    std::cout << "X: " << bruhx << "\nY: " << bruhy << "\nZ: " << plopter.relative_ground_altitude(true) << std::endl;
//    std::cout << "VelX: " << (plopter.optflow.bodyRate().x - plopter.optflow.flowRate().x) * 0.01 <<
//    "\nVelY: " << (plopter.optflow.bodyRate().y - plopter.optflow.flowRate().y) * 0.01 << std::endl;
//    std::cout << "YAW: " << state(8, 0) << std::endl;
//    std::cout << "X1: " << posNE.x << "\nY1: " << posNE.y << std::endl;
//    std::cout << "X2: " << bruhx << "\nY2: " << bruhy << std::endl;
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
