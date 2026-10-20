#pragma once

// Keep encoder sampling, controller dt and debug telemetry on the same period.
#if APP_MOTOR_ONLY
constexpr unsigned kMotionPeriodMs = 20;
#else
constexpr unsigned kMotionPeriodMs = 20;
#endif
constexpr float kMotionDtSeconds = kMotionPeriodMs / 1000.0f;
constexpr unsigned kMotionFrequencyHz = 1000 / kMotionPeriodMs;
