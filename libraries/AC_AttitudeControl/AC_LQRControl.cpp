#include "AC_LQRControl.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/stewMath.h>


extern const AP_HAL::HAL& hal;

// idk what all this does but ill copy it here for now

//const AP_Param::GroupInfo AC_LQRControl::var_info[] = {
//        // parameters from parent vehicle
//        AP_NESTEDGROUPINFO(AC_LQRControl, 0),
//
//
//        AP_GROUPEND
//};


AC_LQRControl::AC_LQRControl(AP_AHRS_View &ahrs, const AP_InertialNav& inav,
              const AP_Vehicle::MultiCopter &aparm, AP_MotorsPlopter& motors, float dt) :
        _motors(motors),
        _ahrs(ahrs),
        _inav(inav)
{

//    _singleton = this;
//    AP_Param::setup_object_defaults(this, var_info);
//
//    AP_Param::setup_object_defaults(this, var_info);
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
    K_Gain_Matrix(2, 0) = 0.0;      K_Gain_Matrix(2, 1) = 0.2236;   K_Gain_Matrix(2, 2) = -0.2236;  K_Gain_Matrix(2, 3) = 0.0;      K_Gain_Matrix(2, 4) = 0.8510;   K_Gain_Matrix(2, 5) = -0.7345;  K_Gain_Matrix(2, 6) = 4,9191;   K_Gain_Matrix(2, 7) = 0.0;      K_Gain_Matrix(2, 8) = 0.0;      K_Gain_Matrix(2, 9) = 1.4197;   K_Gain_Matrix(2, 10) = 0.0;K_Gain_Matrix(2, 11) = 0.0;
    K_Gain_Matrix(3, 0) = -0.2237;  K_Gain_Matrix(3, 1) = 0.0;      K_Gain_Matrix(3, 2) = 0.0;      K_Gain_Matrix(3, 3) = -0.8513;  K_Gain_Matrix(3, 4) = 0.0;      K_Gain_Matrix(3, 5) = 0.0;      K_Gain_Matrix(3, 6) = 0.0;      K_Gain_Matrix(3, 7) = 4.9186;   K_Gain_Matrix(3, 8) = -0.7068;  K_Gain_Matrix(3, 9) = 0.0;      K_Gain_Matrix(3, 10) = 1.4197;K_Gain_Matrix(3, 11) = -1.4140;

    Offset_Forcing_Matrix(1,7) = 0.00003285;
    Offset_Forcing_Matrix(3,7) = 0.00000775;


}

void AC_LQRControl::controller_run()
{
    Vector3f gyro_latest = _ahrs.get_gyro_latest();


    //Get Current State
    state(0, 0) = _inav.get_position_xy_cm().x / 100.;
    state(1, 0) = _inav.get_position_xy_cm().y / 100.;
    state(2, 0) = _inav.get_position_z_up_cm() / 100.;
    state(3, 0) = _inav.get_velocity_xy_cms().x / 100.;
    state(4, 0) = _inav.get_velocity_xy_cms().y / 100.;
    state(5, 0) = _inav.get_velocity_z_up_cms() / 100.;
    state(6, 0) = AP::ahrs().get_roll();
    state(7, 0) = AP::ahrs().get_pitch();
    state(8, 0) = AP::ahrs().get_yaw();
    state(9, 0) = gyro_latest.x;
    state(10, 0) =gyro_latest.y;
    state(11, 0) =gyro_latest.z;


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
    //motors set output
    _motors.set_output(left_thrust, left_angle, right_thrust, right_angle);

    //control_monitor_update();
}



void AC_LQRControl::set_state_offset(sm::Matrix<float> _state_offset){
    state_offset = _state_offset;
}















