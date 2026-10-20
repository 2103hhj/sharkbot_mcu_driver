#ifndef BOARD_H_
#define BOARD_H_

#include "motor_driver.h"
#include "encoder_driver.h" // 引入编码器驱动
#include "motion_controller.h"
#if !APP_MOTOR_ONLY // 暂停非电机外设，保留源码以便恢复。
#include "ultrasonic_sensor.h"
#include "button_driver.h"
#include "servo_driver.h"
#include "lsm6ds3_driver.h"
#include "status_led.h"
#include "battery_adc.h"
#include "esp_log.h"

// 声明所有硬件对象为 extern
extern Lsm6ds3Imu imu;
#endif

extern At8236Motor motor_left;
extern At8236Motor motor_right;
extern QuadratureEncoder encoder_left;
extern QuadratureEncoder encoder_right;

extern MotionController robot;
#if !APP_MOTOR_ONLY
extern UltrasonicSensor ultrasonic;
extern StatusLed status_led;
extern ButtonDriver io0_button;
extern ServoDriver my_servo;
extern BatteryAdc battery_adc;
#endif

// 硬件统一初始化函数
void board_init(void);

#endif // BOARD_H_
