#pragma once

// Only motion commands renew this timeout; status/PID/help do not keep wheels running.
constexpr unsigned kMotorCommandTimeoutMs = 2000;

void motor_debug_console_start();
