#include "mode.h"
#include "Plopter.h"

#if HAL_QUADPLANE_ENABLED

bool ModeQStabilize::_enter()
{
    quadplopter.throttle_wait = false;
    return true;
}

void ModeQStabilize::update()
{
    // set nav_roll and nav_pitch using sticks
    // Beware that QuadPlopter::tailsitter_check_input (called from Plopter::read_radio)
    // may alter the control_in values for roll and yaw, but not the corresponding
    // radio_in values. This means that the results for norm_input would not necessarily
    // be correct for tailsitters, so get_control_in() must be used instead.
    // normalize control_input to [-1,1]
    const float roll_input = (float)plopter.channel_roll->get_control_in() / plopter.channel_roll->get_range();
    const float pitch_input = (float)plopter.channel_pitch->get_control_in() / plopter.channel_pitch->get_range();

    // then scale to target angles in centidegrees
    if (plopter.quadplopter.tailsitter.active()) {
        // tailsitters are different
        set_tailsitter_roll_pitch(roll_input, pitch_input);
        return;
    }

    if ((plopter.quadplopter.options & QuadPlopter::OPTION_INGORE_FW_ANGLE_LIMITS_IN_Q_MODES) == 0) {
        // by default angles are also constrained by forward flight limits
        set_limited_roll_pitch(roll_input, pitch_input);
    } else {
        // use angle max for both roll and pitch
        plopter.nav_roll_cd = roll_input * plopter.quadplopter.aparm.angle_max;
        plopter.nav_pitch_cd = pitch_input * plopter.quadplopter.aparm.angle_max;
    }
}

// quadplopter stabilize mode
void ModeQStabilize::run()
{
    // special check for ESC calibration in QSTABILIZE
    if (quadplopter.esc_calibration != 0) {
        quadplopter.run_esc_calibration();
        return;
    }

    // normal QSTABILIZE mode
    float pilot_throttle_scaled = quadplopter.get_pilot_throttle();
    quadplopter.hold_stabilize(pilot_throttle_scaled);
}

// set the desired roll and pitch for a tailsitter
void ModeQStabilize::set_tailsitter_roll_pitch(const float roll_input, const float pitch_input)
{
    // separate limit for roll, if set
    if (plopter.quadplopter.tailsitter.max_roll_angle > 0) {
        // roll param is in degrees not centidegrees
        plopter.nav_roll_cd = plopter.quadplopter.tailsitter.max_roll_angle * 100.0f * roll_input;
    } else {
        plopter.nav_roll_cd = roll_input * plopter.quadplopter.aparm.angle_max;
    }

    // angle max for tailsitter pitch
    plopter.nav_pitch_cd = pitch_input * plopter.quadplopter.aparm.angle_max;

    plopter.quadplopter.transition->set_VTOL_roll_pitch_limit(plopter.nav_roll_cd, plopter.nav_pitch_cd);
}

// set the desired roll and pitch for normal quadplopters, also limited by forward flight limtis
void ModeQStabilize::set_limited_roll_pitch(const float roll_input, const float pitch_input)
{
    plopter.nav_roll_cd = roll_input * MIN(plopter.roll_limit_cd, plopter.quadplopter.aparm.angle_max);
    // pitch is further constrained by LIM_PITCH_MIN/MAX which may impose
    // tighter (possibly asymmetrical) limits than Q_ANGLE_MAX
    if (pitch_input > 0) {
        plopter.nav_pitch_cd = pitch_input * MIN(plopter.aparm.pitch_limit_max_cd, plopter.quadplopter.aparm.angle_max);
    } else {
        plopter.nav_pitch_cd = pitch_input * MIN(-plopter.pitch_limit_min_cd, plopter.quadplopter.aparm.angle_max);
    }
}

#endif
