#include "mode.h"
#include "Plopter.h"

void ModeFBWA::update()
{
    // set nav_roll and nav_pitch using sticks
    plopter.nav_roll_cd  = plopter.channel_roll->norm_input() * plopter.roll_limit_cd;
    plopter.update_load_factor();
    float pitch_input = plopter.channel_pitch->norm_input();
    if (pitch_input > 0) {
        plopter.nav_pitch_cd = pitch_input * plopter.aparm.pitch_limit_max_cd;
    } else {
        plopter.nav_pitch_cd = -(pitch_input * plopter.pitch_limit_min_cd);
    }
    plopter.adjust_nav_pitch_throttle();
    plopter.nav_pitch_cd = constrain_int32(plopter.nav_pitch_cd, plopter.pitch_limit_min_cd, plopter.aparm.pitch_limit_max_cd.get());
    if (plopter.fly_inverted()) {
        plopter.nav_pitch_cd = -plopter.nav_pitch_cd;
    }
    if (plopter.failsafe.rc_failsafe && plopter.g.fs_action_short == FS_ACTION_SHORT_FBWA) {
        // FBWA failsafe glide
        plopter.nav_roll_cd = 0;
        plopter.nav_pitch_cd = 0;
        SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::MIN);
    }
    RC_Channel *chan = rc().find_channel_for_option(RC_Channel::AUX_FUNC::FBWA_TAILDRAGGER);
    if (chan != nullptr) {
        // check for the user enabling FBWA taildrag takeoff mode
        bool tdrag_mode = chan->get_aux_switch_pos() == RC_Channel::AuxSwitchPos::HIGH;
        if (tdrag_mode && !plopter.auto_state.fbwa_tdrag_takeoff_mode) {
            if (plopter.auto_state.highest_airspeed < plopter.g.takeoff_tdrag_speed1) {
                plopter.auto_state.fbwa_tdrag_takeoff_mode = true;
                plopter.gcs().send_text(MAV_SEVERITY_WARNING, "FBWA tdrag mode");
            }
        }
    }
}
