/*
  Servo controlled mount backend class
 */
#pragma once

#include "AP_Mount_config.h"

#if HAL_MOUNT_SERVO_ENABLED

#include "AP_Mount_Backend.h"

#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>

class AP_Mount_Servo : public AP_Mount_Backend
{
public:
    // Constructor
    AP_Mount_Servo(AP_Mount &frontend, AP_Mount_Params &params, bool requires_stab, uint8_t instance):
        AP_Mount_Backend(frontend, params, instance),
        requires_stabilization(requires_stab),
        _roll_idx(SRV_Channel::k_none),
        _tilt_idx(SRV_Channel::k_none),
        _pan_idx(SRV_Channel::k_none)
    {
    }

    // init - performs any required initialisation for this instance
    void init() override;

    // update mount position - should be called periodically
    void update() override;

    // custom command handler (MAV_CMD_USER_1) used for ILC table dump/load
    MAV_RESULT handle_command_user1(const mavlink_command_int_t &packet, const mavlink_message_t &msg) override;

    // returns true if this mount can control its roll
    bool has_roll_control() const override;

    // returns true if this mount can control its tilt
    bool has_pitch_control() const override;

    // returns true if this mount can control its pan (required for multicopters)
    bool has_pan_control() const override;

protected:

    // get attitude as a quaternion.  returns true on success
    bool get_attitude_quaternion(Quaternion& att_quat) override;

    // servo only natively supports angles:
    uint8_t natively_supported_mount_target_types() const override {
        return NATIVE_ANGLES_ONLY;
    };

private:

    // called by the backend to set the servo angles:
    void send_target_angles(const MountAngleTarget& angle_rad) override;

    // update body-frame angle outputs from earth-frame targets
    void update_angle_outputs(const MountAngleTarget& angle_rad);

    ///  moves servo with the given function id to the specified angle.  all angles are in body-frame and degrees * 10
    void move_servo(uint8_t rc, int16_t angle, int16_t angle_min, int16_t angle_max);

    // ILC (Iterative Learning Control) helper. Keeps learned state in RAM (non-persistent).
    static constexpr uint16_t ILC_MAX_SAMPLES = 200;

    struct ILCAxisState {
        bool active = false;
        int8_t sign0 = 1;          // +1 or -1 based on initial error sign
        uint16_t last_k = 0;
        uint32_t start_ms = 0;
        uint16_t settle_count = 0;
        uint32_t episode_count = 0;

        float ff_pos[ILC_MAX_SAMPLES];
        float ff_neg[ILC_MAX_SAMPLES];
        float e_trace[ILC_MAX_SAMPLES];
        uint8_t sat_trace[ILC_MAX_SAMPLES];

        void reset_runtime();
        void clear_tables();
    };

    float ilc_step_axis(
        ILCAxisState &st,
        bool axis_enabled,
        float error_rad,
        float rate_rad_s,
        float u_fb_rad,
        float u_min_rad,
        float u_max_rad,
        float dt_s,
        uint32_t now_ms,
        char axis_char
    );

    /// Servo gimbals require stabilization, BrushlessPWM gimbals self-stabilize
    const bool requires_stabilization;

    // SRV_Channel - different id numbers are used depending upon the instance number
    SRV_Channel::Function    _roll_idx;  // SRV_Channel mount roll function index
    SRV_Channel::Function    _tilt_idx;  // SRV_Channel mount tilt function index
    SRV_Channel::Function    _pan_idx;   // SRV_Channel mount pan  function index

    Vector3f _angle_bf_output_rad;  // final body frame output angle in radians
    Vector3f _angle_bf_output_slew_rad; // slew-limited body frame output angle in radians
    uint32_t _slew_last_update_ms = 0;  // last time slew limiter ran (ms)
    Vector2f _lvl_i_out_rad;        // PI leveling integrator output (radians)
    uint32_t _lvl_last_update_ms = 0; // last time update_angle_outputs ran (ms)
    bool _lvl_fast_roll = false;    // true when roll uses fast leveling gain (MNTx_LVL_THR)
    bool _lvl_fast_pitch = false;   // true when pitch uses fast leveling gain (MNTx_LVL_THR)

    // Step-once leveling bias (used when MNTx_LVL_MODE=StepOnce).
    Vector2f _lvl_step_bias_rad;    // bias added to earth-frame roll/pitch targets (radians)
    bool _lvl_step_armed_roll = true;
    bool _lvl_step_armed_pitch = true;
    uint32_t _lvl_step_last_ms_roll = 0;
    uint32_t _lvl_step_last_ms_pitch = 0;

    ILCAxisState _ilc_roll{};
    ILCAxisState _ilc_pitch{};
};
#endif // HAL_MOUNT_SERVO_ENABLED
