#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

class AP_Mount_Params {

public:

    static const struct AP_Param::GroupInfo var_info[];

    AP_Mount_Params(void);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Mount_Params);

    AP_Int8     type;               // mount type (see Mount::Type enum)
    AP_Int8     default_mode;       // default mode on startup and when control is returned from autopilot
    AP_Int16    rc_rate_max;        // Pilot rate control's maximum rate.  Set to zero to use angle control
    AP_Int16    roll_angle_min;     // roll angle min in degrees
    AP_Int16    roll_angle_max;     // roll angle max in degrees
    AP_Int16    pitch_angle_min;    // pitch angle min in degrees
    AP_Int16    pitch_angle_max;    // pitch angle max in degrees
    AP_Int16    yaw_angle_min;      // yaw angle min in degrees
    AP_Int16    yaw_angle_max;      // yaw angle max in degrees

    AP_Vector3f retract_angles;     // retracted position in degrees. vector.x = roll vector.y = pitch, vector.z=yaw
    AP_Vector3f neutral_angles;     // neutral position in degrees.  vector.x = roll vector.y = pitch, vector.z=yaw

    AP_Float    upd_hz;             // servo backend: mount output update rate (Hz). 0 disables limiting (runs every scheduler tick)
    AP_Float    roll_stb_lead;      // roll lead control gain (only used by servo backend)
    AP_Float    pitch_stb_lead;     // pitch lead control gain (only used by servo backend)
    AP_Float    lvl_p;              // servo backend: roll/pitch PI leveling proportional gain (1.0 matches legacy)
    AP_Float    lvl_p_fast;         // servo backend: roll/pitch fast leveling proportional gain (used when abs(error) > lvl_thr)
    AP_Float    lvl_i;              // servo backend: roll/pitch PI leveling integral gain
    AP_Float    lvl_imax;           // servo backend: roll/pitch PI leveling integrator max output in degrees (0 disables I)
    AP_Float    lvl_thr;            // servo backend: leveling error threshold (deg) for switching to fast leveling (0 disables)
    AP_Int8     lvl_mode;           // servo backend: leveling mode (0=continuous PI, 1=step-once)
    AP_Float    lvl_trig;           // servo backend: step-once trigger threshold (deg). 0 disables
    AP_Float    lvl_stop;           // servo backend: step-once re-arm threshold (deg)
    AP_Float    lvl_step;           // servo backend: step-once gain applied to error (unitless)
    AP_Float    lvl_rate;           // servo backend: step-once trigger max rate (deg/s). 0 disables rate gating
    AP_Float    lvl_period;         // servo backend: step-once minimum time between additional steps while active (s). 0 disables repeat
    AP_Float    lvl_slew;           // servo backend: output slew rate limit (deg/s). 0 disables slew limiting

    // Servo backend: Iterative Learning Control (ILC) feedforward overlay
    AP_Int8     ilc_enable;         // enable ILC overlay (0=disabled)
    AP_Int8     ilc_axis;           // axis selection (0=both, 1=pitch, 2=roll)
    AP_Float    ilc_hz;             // ILC table sample rate (Hz)
    AP_Float    ilc_horizon;        // ILC episode horizon (s)
    AP_Float    ilc_learn;          // ILC learning gain (unitless, output per error)
    AP_Float    ilc_forget;         // ILC forgetting factor (0..1)
    AP_Float    ilc_ff_max;         // ILC feedforward clamp (deg)
    AP_Float    ilc_start_min;      // ILC episode start window min abs error (deg)
    AP_Float    ilc_start_max;      // ILC episode start window max abs error (deg)
    AP_Float    ilc_start_rate;     // ILC episode start max rate (deg/s)
    AP_Float    ilc_settle_deg;     // ILC settle threshold (deg)
    AP_Float    ilc_settle_rate;    // ILC settle threshold (deg/s)
    AP_Float    ilc_settle_time;    // ILC settle time (s)
    AP_Float    ilc_smooth;         // ILC post-update smoothing (0..1), 0 disables
    AP_Int8     sysid_default;      // target sysid for mount to follow
    AP_Int32    dev_id;             // Device id taking into account bus
    AP_Int8     options;            // mount options bitmask
};
