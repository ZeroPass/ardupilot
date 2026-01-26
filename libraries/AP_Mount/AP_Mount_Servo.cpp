#include "AP_Mount_config.h"

#if HAL_MOUNT_SERVO_ENABLED

#include "AP_Mount_Servo.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#if HAL_GCS_ENABLED
#include <GCS_MAVLink/GCS.h>
#endif
#include <stdio.h>
#include <string.h>

extern const AP_HAL::HAL& hal;

// init - performs any required initialisation for this instance
void AP_Mount_Servo::init()
{
    if (_instance == 0) {
        _roll_idx = SRV_Channel::k_mount_roll;
        _tilt_idx = SRV_Channel::k_mount_tilt;
        _pan_idx  = SRV_Channel::k_mount_pan;
    } else {
        // this must be the 2nd mount
        _roll_idx = SRV_Channel::k_mount2_roll;
        _tilt_idx = SRV_Channel::k_mount2_tilt;
        _pan_idx  = SRV_Channel::k_mount2_pan;
    }
    AP_Mount_Backend::init();

    _angle_bf_output_rad = {};
    _angle_bf_output_slew_rad = {};
    _slew_last_update_ms = 0;
    _lvl_i_out_rad = {};
    _lvl_last_update_ms = 0;
    _lvl_step_bias_rad = {};
    _lvl_step_armed_roll = true;
    _lvl_step_armed_pitch = true;

    _ilc_roll.clear_tables();
    _ilc_pitch.clear_tables();
}

void AP_Mount_Servo::ILCAxisState::reset_runtime()
{
    active = false;
    sign0 = 1;
    last_k = 0;
    start_ms = 0;
    settle_count = 0;
}

void AP_Mount_Servo::ILCAxisState::clear_tables()
{
    memset(ff_pos, 0, sizeof(ff_pos));
    memset(ff_neg, 0, sizeof(ff_neg));
    memset(e_trace, 0, sizeof(e_trace));
    memset(sat_trace, 0, sizeof(sat_trace));
    reset_runtime();
    episode_count = 0;
}

float AP_Mount_Servo::ilc_step_axis(
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
)
{
    // Always keep runtime state consistent when disabled
    if (_params.ilc_enable.get() == 0 || !axis_enabled) {
        st.reset_runtime();
        return u_fb_rad;
    }

    const float hz = _params.ilc_hz.get();
    const float horizon_s = _params.ilc_horizon.get();
    const float learn_gain = _params.ilc_learn.get();
    const float forget = constrain_float(_params.ilc_forget.get(), 0.0f, 1.0f);
    const float ff_max_rad = radians(_params.ilc_ff_max.get());
    const float start_min_rad = radians(_params.ilc_start_min.get());
    const float start_max_rad = radians(_params.ilc_start_max.get());
    const float start_rate_rad_s = radians(_params.ilc_start_rate.get());
    const float settle_rad = radians(_params.ilc_settle_deg.get());
    const float settle_rate_rad_s = radians(_params.ilc_settle_rate.get());
    const float settle_time_s = _params.ilc_settle_time.get();
    const float smooth = constrain_float(_params.ilc_smooth.get(), 0.0f, 1.0f);

    if (!is_positive(hz) || !is_positive(horizon_s) || ff_max_rad <= 0.0f) {
        st.reset_runtime();
        return u_fb_rad;
    }

    const uint16_t N = (uint16_t)constrain_int32(lroundf(horizon_s * hz), 1, (int32_t)ILC_MAX_SAMPLES);
    const uint16_t settle_need = is_positive(settle_time_s) ?
        (uint16_t)constrain_int32(lroundf(settle_time_s * hz), 1, (int32_t)ILC_MAX_SAMPLES) : 0;

    auto finish_episode = [&](bool settled) {
        if (is_positive(learn_gain) && st.last_k != 0xFFFF) {
            float *tbl = (st.sign0 > 0) ? st.ff_pos : st.ff_neg;
            const uint16_t k_end = MIN(st.last_k, (uint16_t)(N - 1));

            for (uint16_t i = 0; i <= k_end; i++) {
                if (st.sat_trace[i] != 0) {
                    continue; // avoid learning on saturation
                }
                tbl[i] = constrain_float((1.0f - forget) * tbl[i] + learn_gain * st.e_trace[i], -ff_max_rad, +ff_max_rad);
            }

            // Optional smoothing (acts like a simple Q-filter)
            if (smooth > 0.0f && k_end >= 2) {
                // reuse e_trace as scratch (episode is finished)
                for (uint16_t i = 0; i <= k_end; i++) {
                    st.e_trace[i] = tbl[i];
                }
                for (uint16_t i = 1; i < k_end; i++) {
                    const float avg = 0.25f * st.e_trace[i - 1] + 0.5f * st.e_trace[i] + 0.25f * st.e_trace[i + 1];
                    tbl[i] = (1.0f - smooth) * tbl[i] + smooth * avg;
                }
            }
        }

        st.active = false;
        st.start_ms = 0;
        st.last_k = 0;
        st.settle_count = 0;
        st.episode_count++;

#if HAL_GCS_ENABLED
        // Emit a small MAVLink event when an episode ends so the host can prompt the user.
        // Uses NAMED_VALUE_FLOAT so it is easy to read from scripts.
        // Name format (<=10 chars): ILC + axis + sign + (EP|EV)
        //  - ILC{axis}{sign}EP = episode count
        //  - ILC{axis}{sign}EV = event code (1=settled, 2=timeout)
        if (is_positive(learn_gain)) {
            const char sign_char = (st.sign0 > 0) ? '+' : '-';
            auto send_evt = [&](char c1, char c2, float value) {
                char name[MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN] {};
                name[0] = 'I';
                name[1] = 'L';
                name[2] = 'C';
                name[3] = axis_char;
                name[4] = sign_char;
                name[5] = c1;
                name[6] = c2;
                name[7] = '\0';
                gcs().send_named_float(name, value);
            };
            send_evt('E', 'P', (float)st.episode_count);
            send_evt('E', 'V', settled ? 1.0f : 2.0f);
        }
#endif
    };

    // Start condition (episode boundary)
    if (!st.active) {
        const float ae = fabsf(error_rad);
        if (!(ae >= start_min_rad && ae <= start_max_rad)) {
            return u_fb_rad;
        }
        if (is_positive(start_rate_rad_s) && fabsf(rate_rad_s) > start_rate_rad_s) {
            return u_fb_rad;
        }

        st.active = true;
        st.sign0 = (error_rad >= 0.0f) ? 1 : -1;
        st.start_ms = now_ms;
        st.last_k = 0xFFFF; // force record at k=0
        st.settle_count = 0;

        // clear traces for this episode
        for (uint16_t i = 0; i < N; i++) {
            st.e_trace[i] = 0.0f;
            st.sat_trace[i] = 0;
        }
    }

    // Determine time index within the episode.
    const float t_s = (now_ms - st.start_ms) * 0.001f;
    if (t_s < 0.0f) {
        st.start_ms = now_ms;
    }
    const int32_t k32 = (int32_t)floorf(MAX(t_s, 0.0f) * hz);
    const uint16_t k = (uint16_t)k32;
    if (k >= N) {
        finish_episode(false);
        return u_fb_rad;
    }

    // Apply learned feedforward
    const float ff = (st.sign0 > 0) ? st.ff_pos[k] : st.ff_neg[k];
    const float u = u_fb_rad + ff;
    const float u_clamped = constrain_float(u, u_min_rad, u_max_rad);
    const bool sat = !is_zero(u - u_clamped);

    // Record one sample per index
    if (k != st.last_k) {
        st.e_trace[k] = error_rad;
        st.sat_trace[k] = sat ? 1 : 0;
        st.last_k = k;
    }

    // Settle detection
    if (settle_need > 0 && dt_s > 0.0f) {
        const bool settled =
            fabsf(error_rad) <= settle_rad &&
            (!is_positive(settle_rate_rad_s) || fabsf(rate_rad_s) <= settle_rate_rad_s);
        if (settled) {
            st.settle_count++;
        } else {
            st.settle_count = 0;
        }
        if (st.settle_count >= settle_need) {
            finish_episode(true);
        }
    }

    return u_clamped;
}

// update mount position - should be called periodically
void AP_Mount_Servo::update()
{
    AP_Mount_Backend::update();

    update_mnt_target();

    // have our base class call send_target_angles to command the gimbal:
    send_target_to_gimbal();
}

MAV_RESULT AP_Mount_Servo::handle_command_user1(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    (void)msg;

#if !HAL_GCS_ENABLED
    return MAV_RESULT_UNSUPPORTED;
#else
    // Mount ILC table helper (used by host tooling to dump/load learned feedforward).
    // Command encoding (MAV_CMD_USER_1 / COMMAND_LONG recommended):
    //   param1: op (1=GET_CHUNK, 2=SET_VALUE, 3=CLEAR)
    //   param2: axis (0=roll, 1=pitch, 2=both for CLEAR)
    //   param3: sign (0=pos, 1=neg, 2=both for CLEAR)
    //   x (param5): index or start index (int)
    //   y (param6): count (int, GET_CHUNK only)
    //   z (param7): value_deg (float, SET_VALUE only)

    const int op = (int)lroundf(packet.param1);
    const int axis = (int)lroundf(packet.param2);
    const int sign = (int)lroundf(packet.param3);
    const int32_t x = packet.x;
    const int32_t y = packet.y;
    const float z = packet.z;

    const float hz = _params.ilc_hz.get();
    const float horizon_s = _params.ilc_horizon.get();
    if (!is_positive(hz) || !is_positive(horizon_s)) {
        return MAV_RESULT_DENIED;
    }
    const uint16_t N = (uint16_t)constrain_int32(lroundf(horizon_s * hz), 1, (int32_t)ILC_MAX_SAMPLES);

    auto select_state = [&](int axis_sel) -> ILCAxisState* {
        switch (axis_sel) {
        case 0:
            return &_ilc_roll;
        case 1:
            return &_ilc_pitch;
        default:
            return nullptr;
        }
    };

    auto send_sample = [&](char axis_char, char sign_char, uint16_t idx, float value_rad) {
        // 10-char name field: "ILC" + axis + sign + 3-digit index (<=199).
        // Use manual formatting to avoid -Werror=format-truncation.
        char name[MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN] {};
        name[0] = 'I';
        name[1] = 'L';
        name[2] = 'C';
        name[3] = axis_char;
        name[4] = sign_char;
        const uint16_t i3 = MIN(idx, (uint16_t)999);
        name[5] = char('0' + ((i3 / 100) % 10));
        name[6] = char('0' + ((i3 / 10) % 10));
        name[7] = char('0' + (i3 % 10));
        name[8] = '\0';
        gcs().send_named_float(name, degrees(value_rad));
    };

    switch (op) {
    case 1: { // GET_CHUNK
        ILCAxisState *st = select_state(axis);
        if (st == nullptr) {
            return MAV_RESULT_DENIED;
        }
        if (!(sign == 0 || sign == 1)) {
            return MAV_RESULT_DENIED;
        }

        int32_t start = x;
        int32_t count = y;
        if (count <= 0) {
            return MAV_RESULT_DENIED;
        }
        start = constrain_int32(start, 0, (int32_t)N - 1);
        const uint16_t end = (uint16_t)MIN((int32_t)N, start + count);

        const char axis_char = (axis == 0) ? 'R' : 'P';
        const char sign_char = (sign == 0) ? '+' : '-';
        const float *tbl = (sign == 0) ? st->ff_pos : st->ff_neg;

        for (uint16_t i = (uint16_t)start; i < end; i++) {
            send_sample(axis_char, sign_char, i, tbl[i]);
        }
        return MAV_RESULT_ACCEPTED;
    }

    case 2: { // SET_VALUE
        ILCAxisState *st = select_state(axis);
        if (st == nullptr) {
            return MAV_RESULT_DENIED;
        }
        if (!(sign == 0 || sign == 1)) {
            return MAV_RESULT_DENIED;
        }
        if (x < 0 || x >= (int32_t)N) {
            return MAV_RESULT_DENIED;
        }

        const float ff_max_rad = radians(_params.ilc_ff_max.get());
        float value_rad = radians(z);
        if (is_positive(ff_max_rad)) {
            value_rad = constrain_float(value_rad, -ff_max_rad, +ff_max_rad);
        }

        float *tbl = (sign == 0) ? st->ff_pos : st->ff_neg;
        tbl[(uint16_t)x] = value_rad;
        return MAV_RESULT_ACCEPTED;
    }

    case 3: { // CLEAR
        if (!(sign == 0 || sign == 1 || sign == 2)) {
            return MAV_RESULT_DENIED;
        }
        const bool clear_roll = (axis == 0 || axis == 2);
        const bool clear_pitch = (axis == 1 || axis == 2);
        if (!(clear_roll || clear_pitch)) {
            return MAV_RESULT_DENIED;
        }

        auto clear_state = [&](ILCAxisState &st) {
            if (sign == 0 || sign == 2) {
                memset(st.ff_pos, 0, sizeof(st.ff_pos));
            }
            if (sign == 1 || sign == 2) {
                memset(st.ff_neg, 0, sizeof(st.ff_neg));
            }
            st.reset_runtime();
        };

        if (clear_roll) {
            clear_state(_ilc_roll);
        }
        if (clear_pitch) {
            clear_state(_ilc_pitch);
        }
        return MAV_RESULT_ACCEPTED;
    }

    default:
        return MAV_RESULT_UNSUPPORTED;
    }
#endif
}

// called by the backend to set the servo angles:
void AP_Mount_Servo::send_target_angles(const MountAngleTarget& angle_rad)
{
    update_angle_outputs(angle_rad);

    // Optional slew limiting (deg/s) to soften response and reduce "step" motion.
    const float slew_deg_s = _params.lvl_slew.get();
    if (is_positive(slew_deg_s)) {
        const uint32_t now_ms = AP_HAL::millis();
        float dt = 0.0f;
        if (_slew_last_update_ms != 0) {
            dt = (now_ms - _slew_last_update_ms) * 0.001f;
            // guard against very large dt (e.g. after pauses)
            dt = constrain_float(dt, 0.0f, 2.0f);
        }
        _slew_last_update_ms = now_ms;

        if (dt > 0.0f) {
            const float max_delta_rad = radians(slew_deg_s) * dt;

            Vector3f delta = _angle_bf_output_rad - _angle_bf_output_slew_rad;
            delta.x = constrain_float(delta.x, -max_delta_rad, max_delta_rad);
            delta.y = constrain_float(delta.y, -max_delta_rad, max_delta_rad);
            delta.z = constrain_float(delta.z, -max_delta_rad, max_delta_rad);

            _angle_bf_output_slew_rad += delta;
            _angle_bf_output_rad = _angle_bf_output_slew_rad;
        } else {
            // first update (or no time delta): initialise slew state
            _angle_bf_output_slew_rad = _angle_bf_output_rad;
        }
    } else {
        // slew limiting disabled
        _angle_bf_output_slew_rad = _angle_bf_output_rad;
        _slew_last_update_ms = 0;
    }

    // write the results to the servos
    move_servo(_roll_idx, degrees(_angle_bf_output_rad.x)*10, _params.roll_angle_min*10, _params.roll_angle_max*10);
    move_servo(_tilt_idx, degrees(_angle_bf_output_rad.y)*10, _params.pitch_angle_min*10, _params.pitch_angle_max*10);
    move_servo(_pan_idx,  degrees(_angle_bf_output_rad.z)*10, _params.yaw_angle_min*10, _params.yaw_angle_max*10);
}

// returns true if this mount can control its roll
bool AP_Mount_Servo::has_roll_control() const
{
    return SRV_Channels::function_assigned(_roll_idx) && roll_range_valid();
}

// returns true if this mount can control its tilt
bool AP_Mount_Servo::has_pitch_control() const
{
    return SRV_Channels::function_assigned(_tilt_idx) && pitch_range_valid();
}

// returns true if this mount can control its pan (required for multicopters)
bool AP_Mount_Servo::has_pan_control() const
{
    return SRV_Channels::function_assigned(_pan_idx) && yaw_range_valid();
}

// get attitude as a quaternion.  returns true on success
bool AP_Mount_Servo::get_attitude_quaternion(Quaternion& att_quat)
{
    // No feedback from gimbal so simply report demanded servo angles (which is
    // not the same as target angles).
    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    float yaw_rad = 0.0f;
    if (has_roll_control()) {
        roll_rad = constrain_float(_angle_bf_output_rad.x, radians(_params.roll_angle_min), radians(_params.roll_angle_max));
    }
    if (has_pitch_control()) {
        pitch_rad = constrain_float(_angle_bf_output_rad.y, radians(_params.pitch_angle_min), radians(_params.pitch_angle_max));
    }
    if (has_pan_control()) {
        yaw_rad = constrain_float(_angle_bf_output_rad.z, radians(_params.yaw_angle_min), radians(_params.yaw_angle_max));
    }

    // convert to quaternion
    att_quat.from_euler(roll_rad, pitch_rad, yaw_rad);
    return true;
}

// private methods

// update body-frame angle outputs from earth-frame angle targets
void AP_Mount_Servo::update_angle_outputs(const MountAngleTarget& angle_rad)
{
    const AP_AHRS &ahrs = AP::ahrs();

    // Use elapsed time between mount updates (not scheduler loop period).
    // This keeps I-term behaviour consistent when outputs are updated at low rate.
    const uint32_t now_ms = AP_HAL::millis();
    float dt = 0.0f;
    if (_lvl_last_update_ms != 0) {
        dt = (now_ms - _lvl_last_update_ms) * 0.001f;
        // guard against very large dt (e.g. after pauses)
        dt = constrain_float(dt, 0.0f, 2.0f);
    }
    _lvl_last_update_ms = now_ms;

    // get target yaw in body-frame with limits applied
    const float yaw_bf_rad = constrain_float(angle_rad.get_bf_yaw(), radians(_params.yaw_angle_min), radians(_params.yaw_angle_max));

    // default output to target earth-frame roll and pitch angles, body-frame yaw
    _angle_bf_output_rad.x = angle_rad.roll;
    _angle_bf_output_rad.y = angle_rad.pitch;
    _angle_bf_output_rad.z = yaw_bf_rad;

    // do no stabilization in retract or neutral:
    switch (mnt_target.target_type) {
    case MountTargetType::NEUTRAL:
    case MountTargetType::RETRACTED:
        _lvl_i_out_rad = {};   // avoid windup when stabilization is disabled
        _lvl_step_bias_rad = {};
        _lvl_step_armed_roll = true;
        _lvl_step_armed_pitch = true;
        _lvl_step_last_ms_roll = 0;
        _lvl_step_last_ms_pitch = 0;
        _ilc_roll.reset_runtime();
        _ilc_pitch.reset_runtime();
        return;
    case MountTargetType::ANGLE:
    case MountTargetType::RATE:
        break;
    }

    // only have to adjust roll/pitch for body frame in self-stabilising brushless gimbals
    if (!requires_stabilization) {
        //since this is a shared backend, must call this directly
        AP_Mount_Backend::adjust_mnt_target_if_RP_locked();
        _lvl_i_out_rad = {};
        _lvl_step_bias_rad = {};
        _lvl_step_armed_roll = true;
        _lvl_step_armed_pitch = true;
        _lvl_step_last_ms_roll = 0;
        _lvl_step_last_ms_pitch = 0;
        _ilc_roll.reset_runtime();
        _ilc_pitch.reset_runtime();
        return;
    }

    // retrieve lean angles from ahrs
    Vector2f ahrs_angle_rad = {ahrs.get_roll_rad(), ahrs.get_pitch_rad()};

    // rotate ahrs roll and pitch angles to gimbal yaw
    if (has_pan_control()) {
        ahrs_angle_rad.rotate(-yaw_bf_rad);
    }

    // PI on angle error (target - vehicle angle) for earth-frame targets.
    const float p = _params.lvl_p.get();
    const float i = _params.lvl_i.get();
    const float imax_rad = radians(_params.lvl_imax.get());

    Vector2f err{};
    if (angle_rad.roll_is_ef) {
        err.x = angle_rad.roll - ahrs_angle_rad.x;
    }
    if (angle_rad.pitch_is_ef) {
        err.y = angle_rad.pitch - ahrs_angle_rad.y;
    }

    // Step-once leveling: update held compensation only once per large disturbance.
    if (_params.lvl_mode.get() == 1) {
        _ilc_roll.reset_runtime();
        _ilc_pitch.reset_runtime();
        const float trig_deg = _params.lvl_trig.get();
        const float stop_deg = _params.lvl_stop.get();
        const float step_gain = _params.lvl_step.get();
        const float rate_max_deg_s = _params.lvl_rate.get();
        const float period_s = _params.lvl_period.get();
        const float slew_deg_s = _params.lvl_slew.get();
        if (trig_deg > 0.0f && stop_deg >= 0.0f && step_gain > 0.0f) {
            const float trig_rad = radians(trig_deg);
            const float stop_rad = radians(stop_deg);
            const float rate_max_rad_s = rate_max_deg_s > 0.0f ? radians(rate_max_deg_s) : 0.0f;
            const uint32_t period_ms = period_s > 0.0f ? (uint32_t)(period_s * 1000.0f + 0.5f) : 0;
            const float slew_rad_s = slew_deg_s > 0.0f ? radians(slew_deg_s) : 0.0f;

            // When using repeated StepOnce kicks + output slew limiting, the bias can "run ahead"
            // of the slew-limited output and cause occasional overshoot. Clamp each kick so the
            // bias can't change faster than the output can move.
            auto limit_step_rad = [&](float step_rad, uint32_t last_step_ms) -> float {
                if (period_ms == 0 || slew_rad_s <= 0.0f) {
                    return step_rad;
                }
                float dt_step_s = period_ms * 0.001f;
                if (last_step_ms != 0 && now_ms > last_step_ms) {
                    dt_step_s = (now_ms - last_step_ms) * 0.001f;
                    dt_step_s = constrain_float(dt_step_s, 0.0f, 2.0f);
                }
                const float max_step_rad = slew_rad_s * dt_step_s;
                return constrain_float(step_rad, -max_step_rad, +max_step_rad);
            };

            // Optional: only trigger the step once rotation slows. This helps when
            // the user is still moving the platform while crossing the threshold.
            bool rate_ok_roll = true;
            bool rate_ok_pitch = true;
            if (rate_max_rad_s > 0.0f) {
                const Vector3f &gyro = ahrs.get_gyro();

                // Approximate euler pitch rate (matches lead-filter derivation below)
                const float pitch_rate = ahrs.cos_pitch() * gyro.y - ahrs.sin_roll() * gyro.z;
                rate_ok_pitch = fabsf(pitch_rate) <= rate_max_rad_s;

                // Approximate euler roll rate; use full expression when not near gimbal lock,
                // else fall back to body x gyro.
                float roll_rate = gyro.x;
                if (fabsf(ahrs.get_pitch_rad()) < M_PI/3.0f) {
                    roll_rate = gyro.x + (ahrs.sin_pitch() / ahrs.cos_pitch()) * (gyro.y * ahrs.sin_roll() + gyro.z * ahrs.cos_roll());
                }
                rate_ok_roll = fabsf(roll_rate) <= rate_max_rad_s;
            }

            const float roll_min_rad = radians(_params.roll_angle_min);
            const float roll_max_rad = radians(_params.roll_angle_max);
            const float pitch_min_rad = radians(_params.pitch_angle_min);
            const float pitch_max_rad = radians(_params.pitch_angle_max);

            if (angle_rad.roll_is_ef && has_roll_control()) {
                if (_lvl_step_armed_roll) {
                    if (fabsf(err.x) >= trig_rad && rate_ok_roll) {
                        _lvl_step_bias_rad.x += limit_step_rad(err.x * step_gain, _lvl_step_last_ms_roll);
                        _lvl_step_armed_roll = false;
                        _lvl_step_last_ms_roll = now_ms;
                    }
                } else {
                    if (fabsf(err.x) <= stop_rad && rate_ok_roll) {
                        _lvl_step_armed_roll = true;
                        _lvl_step_last_ms_roll = 0;
                    } else if (period_ms > 0 && rate_ok_roll &&
                               (_lvl_step_last_ms_roll == 0 || (now_ms - _lvl_step_last_ms_roll) >= period_ms)) {
                        // Disturbance has been triggered; keep converging using discrete "kicks"
                        _lvl_step_bias_rad.x += limit_step_rad(err.x * step_gain, _lvl_step_last_ms_roll);
                        _lvl_step_last_ms_roll = now_ms;
                    }
                }

                float roll_out_rad = angle_rad.roll + _lvl_step_bias_rad.x;
                roll_out_rad = constrain_float(roll_out_rad, roll_min_rad, roll_max_rad);
                _lvl_step_bias_rad.x = roll_out_rad - angle_rad.roll;
                _angle_bf_output_rad.x = roll_out_rad;
            } else {
                _lvl_step_bias_rad.x = 0.0f;
                _lvl_step_armed_roll = true;
                _lvl_step_last_ms_roll = 0;
                _angle_bf_output_rad.x = angle_rad.roll;
            }

            if (angle_rad.pitch_is_ef && has_pitch_control()) {
                if (_lvl_step_armed_pitch) {
                    if (fabsf(err.y) >= trig_rad && rate_ok_pitch) {
                        _lvl_step_bias_rad.y += limit_step_rad(err.y * step_gain, _lvl_step_last_ms_pitch);
                        _lvl_step_armed_pitch = false;
                        _lvl_step_last_ms_pitch = now_ms;
                    }
                } else {
                    if (fabsf(err.y) <= stop_rad && rate_ok_pitch) {
                        _lvl_step_armed_pitch = true;
                        _lvl_step_last_ms_pitch = 0;
                    } else if (period_ms > 0 && rate_ok_pitch &&
                               (_lvl_step_last_ms_pitch == 0 || (now_ms - _lvl_step_last_ms_pitch) >= period_ms)) {
                        // Disturbance has been triggered; keep converging using discrete "kicks"
                        _lvl_step_bias_rad.y += limit_step_rad(err.y * step_gain, _lvl_step_last_ms_pitch);
                        _lvl_step_last_ms_pitch = now_ms;
                    }
                }

                float pitch_out_rad = angle_rad.pitch + _lvl_step_bias_rad.y;
                pitch_out_rad = constrain_float(pitch_out_rad, pitch_min_rad, pitch_max_rad);
                _lvl_step_bias_rad.y = pitch_out_rad - angle_rad.pitch;
                _angle_bf_output_rad.y = pitch_out_rad;
            } else {
                _lvl_step_bias_rad.y = 0.0f;
                _lvl_step_armed_pitch = true;
                _lvl_step_last_ms_pitch = 0;
                _angle_bf_output_rad.y = angle_rad.pitch;
            }

            _lvl_i_out_rad = {};

            // Apply the same stabilization lead term used by the continuous controller.
            // This adds damping (rate feedback) which helps reduce overshoot/oscillation
            // when running aggressive StepOnce gains and high slew rates.
            const Vector3f &gyro = ahrs.get_gyro();
            if (!is_zero(_params.roll_stb_lead) && fabsf(ahrs.get_pitch_rad()) < M_PI/3.0f) {
                float roll_rate = gyro.x + (ahrs.sin_pitch() / ahrs.cos_pitch()) * (gyro.y * ahrs.sin_roll() + gyro.z * ahrs.cos_roll());
                _angle_bf_output_rad.x -= roll_rate * _params.roll_stb_lead;
            }
            if (!is_zero(_params.pitch_stb_lead)) {
                const float pitch_rate = ahrs.cos_pitch() * gyro.y - ahrs.sin_roll() * gyro.z;
                _angle_bf_output_rad.y -= pitch_rate * _params.pitch_stb_lead;
            }
            return;
        }
    }

    if (i > 0.0f && imax_rad > 0.0f && dt > 0.0f) {
        if (angle_rad.roll_is_ef) {
            _lvl_i_out_rad.x += err.x * i * dt;
            _lvl_i_out_rad.x = constrain_float(_lvl_i_out_rad.x, -imax_rad, imax_rad);
        } else {
            _lvl_i_out_rad.x = 0.0f;
        }

        if (angle_rad.pitch_is_ef) {
            _lvl_i_out_rad.y += err.y * i * dt;
            _lvl_i_out_rad.y = constrain_float(_lvl_i_out_rad.y, -imax_rad, imax_rad);
        } else {
            _lvl_i_out_rad.y = 0.0f;
        }
    } else {
        _lvl_i_out_rad = {};
    }

    if (angle_rad.roll_is_ef) {
        _angle_bf_output_rad.x = err.x * p + _lvl_i_out_rad.x;
    }

    if (angle_rad.pitch_is_ef) {
        _angle_bf_output_rad.y = err.y * p + _lvl_i_out_rad.y;
    }

    // lead filter
    const Vector3f &gyro = ahrs.get_gyro();

    if (!is_zero(_params.roll_stb_lead) && fabsf(ahrs.get_pitch_rad()) < M_PI/3.0f) {
        // Compute rate of change of euler roll angle
        float roll_rate = gyro.x + (ahrs.sin_pitch() / ahrs.cos_pitch()) * (gyro.y * ahrs.sin_roll() + gyro.z * ahrs.cos_roll());
        _angle_bf_output_rad.x -= roll_rate * _params.roll_stb_lead;
    }

    if (!is_zero(_params.pitch_stb_lead)) {
        // Compute rate of change of euler pitch angle
        float pitch_rate = ahrs.cos_pitch() * gyro.y - ahrs.sin_roll() * gyro.z;
        _angle_bf_output_rad.y -= pitch_rate * _params.pitch_stb_lead;
    }

    // Optional ILC feedforward overlay (continuous PI path only).
    if (_params.ilc_enable.get() != 0) {
        const int8_t axis_sel = _params.ilc_axis.get();
        const bool ilc_roll_enable = (axis_sel == 0 || axis_sel == 2);
        const bool ilc_pitch_enable = (axis_sel == 0 || axis_sel == 1);

        const float roll_min_rad = radians(_params.roll_angle_min);
        const float roll_max_rad = radians(_params.roll_angle_max);
        const float pitch_min_rad = radians(_params.pitch_angle_min);
        const float pitch_max_rad = radians(_params.pitch_angle_max);

        float roll_rate = gyro.x;
        if (fabsf(ahrs.get_pitch_rad()) < M_PI/3.0f) {
            roll_rate = gyro.x + (ahrs.sin_pitch() / ahrs.cos_pitch()) * (gyro.y * ahrs.sin_roll() + gyro.z * ahrs.cos_roll());
        }
        const float pitch_rate = ahrs.cos_pitch() * gyro.y - ahrs.sin_roll() * gyro.z;

        _angle_bf_output_rad.x = ilc_step_axis(
            _ilc_roll,
            ilc_roll_enable && angle_rad.roll_is_ef && has_roll_control(),
            err.x,
            roll_rate,
            _angle_bf_output_rad.x,
            roll_min_rad,
            roll_max_rad,
            dt,
            now_ms,
            'R'
        );

        _angle_bf_output_rad.y = ilc_step_axis(
            _ilc_pitch,
            ilc_pitch_enable && angle_rad.pitch_is_ef && has_pitch_control(),
            err.y,
            pitch_rate,
            _angle_bf_output_rad.y,
            pitch_min_rad,
            pitch_max_rad,
            dt,
            now_ms,
            'P'
        );
    } else {
        _ilc_roll.reset_runtime();
        _ilc_pitch.reset_runtime();
    }
}

// move_servo - moves servo with the given id to the specified angle.  all angles are in degrees * 10
void AP_Mount_Servo::move_servo(uint8_t function_idx, int16_t angle, int16_t angle_min, int16_t angle_max)
{
	SRV_Channels::move_servo((SRV_Channel::Function)function_idx, angle, angle_min, angle_max);
}
#endif // HAL_MOUNT_SERVO_ENABLED
