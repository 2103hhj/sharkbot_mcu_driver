#include "system_globals.h"
#include "board.h" 
#include "msg/motion_msg.h"
#include "msg/pid_msg.h"
#include "motion_timing.h"
#if !APP_MOTOR_ONLY
#include "msg/imu_msg.h"
#else
#include "motor_debug_console.h"
#endif
#include "freertos/task.h"
#include "esp_log.h"
#include <cmath>          

static const char *TAG = "MOTION_TASK";

void motion_task(void *p) {
    const float dt = kMotionDtSeconds;
    MotionMsg cmd_msg = {};
    MotionMsg incoming_msg = {};
    MotionMsg status_msg = {};
    PidMsg speed_msg;
    PidMsg position_msg;
    
#if !APP_MOTOR_ONLY
    ImuMsg imu_msg = {};
#else
    TickType_t last_command_tick = 0;
    bool command_active = false;
    MotorPidState motor_pid_state = {};
#endif
    TickType_t next_wake = xTaskGetTickCount();

    int current_mode = 0;
    
    while (1) {
#if !APP_MOTOR_ONLY
        xQueuePeek(q_imu_state, &imu_msg, 0);
#endif
                 
        // 1. 接收底层运动指令 (指令状态机)
        if (xQueueReceive(q_motion_cmd, &incoming_msg, 0) == pdTRUE) { 
#if APP_MOTOR_ONLY
            robot.EndMotorStartTiming();
#endif
            cmd_msg = incoming_msg;
            current_mode = cmd_msg.control_mode; // 记录最新模式
#if APP_MOTOR_ONLY
            last_command_tick = xTaskGetTickCount();
            command_active = true;
#endif

            // 处理离散单次触发模式
            if (current_mode == 0) {
                robot.Drive(cmd_msg.target_vx, cmd_msg.target_vy, cmd_msg.target_wz);
            } else if (current_mode == 1) {
                robot.MoveToPosition(cmd_msg.target_x, cmd_msg.target_y, cmd_msg.target_yaw);
            } else if (current_mode == 2) {
                g_motion_busy = true;
                robot.MoveRelative(cmd_msg.target_x, cmd_msg.target_yaw);
            } else if (current_mode == 3) {
                robot.SetMotorTargetVelocity((MotorID)cmd_msg.motor_id, cmd_msg.target_motor_v);
            } else if (current_mode == 6) {
                robot.SetAllMotorTargetsVelocity(
                    cmd_msg.target_vx,
                    cmd_msg.target_vy);
            } else if (current_mode == 7) {
                robot.SetAllMotorDuties(cmd_msg.duty_left, cmd_msg.duty_right);
                g_motion_busy = false;
            } else {
                ESP_LOGW(TAG, "Unsupported motion mode: %d", current_mode);
            }
#if APP_MOTOR_ONLY
            if ((current_mode == 6 && (cmd_msg.target_vx != 0 || cmd_msg.target_vy != 0)) ||
                (current_mode == 7 && (cmd_msg.duty_left != 0 || cmd_msg.duty_right != 0))) {
                robot.BeginMotorStartTiming();
            }
#endif
        }

        // 2. 接收 PID 指令 (速度环)
        if (xQueueReceive(q_speedpid_cmd, &speed_msg, 0) == pdTRUE) { 
            robot.SetVelocityPidGains(speed_msg.kp, speed_msg.ki, speed_msg.kd);
            g_speed_pid_state = speed_msg;
#if APP_MOTOR_ONLY
            motor_pid_state.left = motor_pid_state.right = speed_msg;
#endif
            ESP_LOGI(TAG, "Speed PID updated: kp=%.3f, ki=%.3f, kd=%.3f",
                     speed_msg.kp, speed_msg.ki, speed_msg.kd);
        }

#if APP_MOTOR_ONLY
        // A FIFO preserves consecutive left/right edits instead of overwriting
        // one wheel's pending update with the other wheel's update.
        MotorPidCommand motor_pid_cmd;
        for (unsigned i = 0; i < kMotorPidCommandQueueLength &&
                             xQueueReceive(q_motor_pid_cmd, &motor_pid_cmd, 0) == pdTRUE; ++i) {
            const PidMsg &gains = motor_pid_cmd.gains;
            if (motor_pid_cmd.target == MotorPidTarget::kBoth) {
                robot.SetVelocityPidGains(gains.kp, gains.ki, gains.kd);
                motor_pid_state.left = motor_pid_state.right = gains;
                g_speed_pid_state = gains;
            } else if (motor_pid_cmd.target == MotorPidTarget::kLeft) {
                robot.SetMotorVelocityPidGains(MotorID::kLeft, gains.kp, gains.ki, gains.kd);
                motor_pid_state.left = gains;
            } else if (motor_pid_cmd.target == MotorPidTarget::kRight) {
                robot.SetMotorVelocityPidGains(MotorID::kRight, gains.kp, gains.ki, gains.kd);
                motor_pid_state.right = gains;
            }
        }
        xQueueOverwrite(q_motor_pid_state, &motor_pid_state);
#endif
        
        // 3. 接收 PID 指令 (位置环)
        if (xQueueReceive(q_postionpid_cmd, &position_msg, 0) == pdTRUE) { 
            robot.SetPositionPidGains(position_msg.kp, position_msg.ki, position_msg.kd);
            g_position_pid_state = position_msg;
            ESP_LOGI(TAG, "Position PID updated: kp=%.3f, ki=%.3f, kd=%.3f",
                     position_msg.kp, position_msg.ki, position_msg.kd);
        }

        // 串口断开或停止发送运动指令后，不保持旧的电机目标。
#if APP_MOTOR_ONLY
        if (command_active && (xTaskGetTickCount() - last_command_tick >=
                               pdMS_TO_TICKS(kMotorCommandTimeoutMs))) {
            robot.Stop();
            g_motion_busy = false;
            command_active = false;
            ESP_LOGW(TAG, "Motion command timed out; motors stopped");
        }
#endif
        // 4. 执行物理控制循环
        if (g_emergency_stop) {
            robot.Stop();
            g_motion_busy = false;
        }
        // Stop() clears controller targets. Keep sampling encoders even during
        // emergency braking so identification feedback follows the actual wheels.
#if !APP_MOTOR_ONLY
        const float imu_yaw_rad = imu_msg.yaw * (M_PI / 180.0f);
#else
        const float imu_yaw_rad = NAN; // 无 IMU 时从编码器里程计取得航向。
#endif
        robot.Update(dt, imu_yaw_rad);
#if APP_MOTOR_ONLY
        const MotorStartTiming start_timing = robot.GetMotorStartTiming();
        xQueueOverwrite(q_motor_start_timing, &start_timing);
#endif
        if (current_mode == 2 && !robot.IsBusy()) {
            g_motion_busy = false;
        }
        
        // 5. 更新遥测状态并上报
        robot.GetVelocity(&status_msg.vx, &status_msg.vy, &status_msg.wz);
        robot.GetOdometry(&status_msg.x, &status_msg.y, &status_msg.yaw); 
        robot.GetOdometryQuaternion(&status_msg.qw,&status_msg.qx,&status_msg.qy,&status_msg.qz);

        robot.GetAllMotorVelocities(&status_msg.vel_left, &status_msg.vel_right);
        robot.GetAllMotorAngularVelocities(&status_msg.omega_left, &status_msg.omega_right);
        robot.GetAllMotorDuties(&status_msg.duty_left, &status_msg.duty_right);
        
        status_msg.control_mode = cmd_msg.control_mode;
        status_msg.source = cmd_msg.source;
        status_msg.target_vx = cmd_msg.target_vx; 
        
        xQueueOverwrite(q_motion_state, &status_msg);
        
        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(kMotionPeriodMs));
    }
}
