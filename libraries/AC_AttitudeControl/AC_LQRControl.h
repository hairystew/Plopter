//
// Created by hairystew on 7/8/22.
//

#ifndef PLOPTER_AC_LQRCONTROL_H
#define PLOPTER_AC_LQRCONTROL_H
#endif //PLOPTER_AC_LQRCONTROL_H

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_Motors/AP_MotorsPlopter.h>
#include <AP_Math/stewMath.h>
#include <AP_InertialNav/AP_InertialNav.h>  // Inertial Navigation library


class AC_LQRControl {
public:
    AC_LQRControl(AP_AHRS_View &ahrs, const AP_InertialNav& inav,
        const AP_Vehicle::MultiCopter &aparm, AP_MotorsPlopter& motors, float dt);
    virtual ~AC_LQRControl() {}

//    static AC_LQRControl *get_singleton(void) {
//        return _singleton;
//    }
//
//    static AC_LQRControl *_singleton;

    void controller_run();
    void set_state_offset(sm::Matrix<float> _state_offset);
//
//    // User settable parameters
//    static const struct AP_Param::GroupInfo var_info[];

protected:

    sm::Matrix<float> K_Gain_Matrix;
    sm::Matrix<float> Offset_Forcing_Matrix;
    sm::Matrix<float> uD;
    sm::Matrix<float> u;
    sm::Matrix<float> state_offset;
    sm::Matrix<float> desired_state;
    sm::Matrix<float> state;

    float left_thrust = 0;
    float right_thrust = 0;
    float left_angle = 0;
    float right_angle = 0;
    float mass = 1.30604; //kg just hardcoding this for now

private:

    // references to inertial nav and ahrs libraries
    AP_AHRS_View&           _ahrs;
    const AP_InertialNav&   _inav;
    AP_MotorsPlopter&        _motors;
    Vector3f _sysid_ang_vel_body;
    Vector3f _ang_vel_body;

};