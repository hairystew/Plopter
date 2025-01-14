#pragma once

#include <AP_Avoidance/AP_Avoidance.h>

// Provide Plopter-specific implementation of avoidance.  While most of
// the logic for doing the actual avoidance is present in
// AP_Avoidance, this class allows Plopter to override base
// functionality - for example, not doing anything while landed.
class AP_Avoidance_Plopter : public AP_Avoidance {
public:

    using AP_Avoidance::AP_Avoidance;

    /* Do not allow copies */
    AP_Avoidance_Plopter(const AP_Avoidance_Plopter &other) = delete;
    AP_Avoidance_Plopter &operator=(const AP_Avoidance_Plopter&) = delete;

protected:
    // override avoidance handler
    MAV_COLLISION_ACTION handle_avoidance(const AP_Avoidance::Obstacle *obstacle, MAV_COLLISION_ACTION requested_action) override;

    // override recovery handler
    void handle_recovery(RecoveryAction recovery_action) override;

    // check flight mode is avoid_adsb
    bool check_flightmode(bool allow_mode_change);

    // vertical avoidance handler
    bool handle_avoidance_vertical(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change, Location &new_loc);

    // horizontal avoidance handler
    bool handle_avoidance_horizontal(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change, Location &new_loc);

    // control mode before avoidance began
    enum Mode::Number prev_control_mode_number = Mode::Number::RTL;
};
