#pragma once

#include <stdint.h>
#include "pid_msg.h"

// 左电机开机默认值：按 Kp、Ki、Kd 顺序单独修改。
inline constexpr PidMsg kDefaultLeftMotorSpeedPid = {1.492f, 25.727f, 0.0f};
// 右电机开机默认值：按 Kp、Ki、Kd 顺序单独修改。
inline constexpr PidMsg kDefaultRightMotorSpeedPid = {1.492f, 49.727f, 0.0f};
constexpr unsigned kMotorPidCommandQueueLength = 8;

enum class MotorPidTarget : uint8_t { kBoth, kLeft, kRight };

// Keep PidMsg unchanged: existing network packets and saved NVS blobs use it.
struct MotorPidCommand {
    MotorPidTarget target = MotorPidTarget::kBoth;
    PidMsg gains = {};
};

struct MotorPidState {
    PidMsg left = kDefaultLeftMotorSpeedPid;
    PidMsg right = kDefaultRightMotorSpeedPid;
};
