#pragma once

#include <stdint.h>

// Diagnostic snapshots only. PWM timestamps describe software writes, while
// encoder timestamps describe samples (their resolution is the control period).
struct MotorStartTiming {
    uint32_t command_id = 0;
    bool recording = false;
    int64_t command_us = 0;
    int64_t first_pwm_begin_us[2] = {-1, -1};
    int64_t first_pwm_end_us[2] = {-1, -1};
    int64_t first_encoder_us[2] = {-1, -1};
    int16_t first_pwm[2] = {};
    int16_t pwm_at_encoder[2] = {};
    int16_t last_pwm[2] = {};

    void Begin(int64_t now_us) {
        const uint32_t next_id = command_id + 1;
        *this = MotorStartTiming{};
        command_id = next_id;
        command_us = now_us;
        recording = true;
    }

    void RecordPwm(int wheel, int16_t pwm, int64_t begin_us, int64_t end_us) {
        if (!recording || wheel < 0 || wheel >= 2) return;
        last_pwm[wheel] = pwm;
        if (pwm != 0 && first_pwm_begin_us[wheel] < 0) {
            first_pwm_begin_us[wheel] = begin_us;
            first_pwm_end_us[wheel] = end_us;
            first_pwm[wheel] = pwm;
        }
    }

    void RecordEncoder(int wheel, int32_t delta, int64_t now_us) {
        if (!recording || wheel < 0 || wheel >= 2) return;
        // Ignore the pre-command sampling interval before the first drive write.
        if (delta != 0 && first_pwm_end_us[wheel] >= 0 && first_encoder_us[wheel] < 0) {
            first_encoder_us[wheel] = now_us;
            pwm_at_encoder[wheel] = last_pwm[wheel];
        }
    }

    void End() { recording = false; } // Preserve the completed measurement after stop.
};
