#pragma once

#include <AP_Arming/AP_Arming.h>

/*
  a plopter specific arming class
 */
class AP_Arming_Plopter : public AP_Arming
{
public:
    AP_Arming_Plopter()
        : AP_Arming()
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    /* Do not allow copies */
    AP_Arming_Plopter(const AP_Arming_Plopter &other) = delete;
    AP_Arming_Plopter &operator=(const AP_Arming_Plopter&) = delete;

    bool pre_arm_checks(bool report) override;
    bool arm_checks(AP_Arming::Method method) override;

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    bool disarm(AP_Arming::Method method, bool do_disarm_checks=true) override;
    bool arm(AP_Arming::Method method, bool do_arming_checks=true) override;

    void update_soft_armed();
    bool get_delay_arming() const { return delay_arming; };

protected:
    bool ins_checks(bool report) override;

    bool quadplopter_checks(bool display_failure);
    bool mission_checks(bool report) override;

private:
    void change_arm_state(void);

    // oneshot with duration AP_ARMING_DELAY_MS used by quadplopter to delay spoolup after arming:
    // ignored unless OPTION_DELAY_ARMING or OPTION_TILT_DISARMED is set
    bool delay_arming;
};
