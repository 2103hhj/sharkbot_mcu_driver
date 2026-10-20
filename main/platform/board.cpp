#include "board.h"

#include "esp_log.h"

static const char *TAG = "BOARD_INIT";

// ================= 引脚定义 (私有) =================
// MOTOR1：左电机
const gpio_num_t kMotorLeftIn1 = GPIO_NUM_23;
const gpio_num_t kMotorLeftIn2 = GPIO_NUM_22;

// MOTOR2：右电机（GPIO12 为 ESP32 启动配置引脚，接线须符合模块启动电平要求）
const gpio_num_t kMotorRightIn1 = GPIO_NUM_12;
const gpio_num_t kMotorRightIn2 = GPIO_NUM_13;

// MOTOR1 编码器：接口第3脚、第4脚
const gpio_num_t kEncoderLeftA = GPIO_NUM_32;
const gpio_num_t kEncoderLeftB = GPIO_NUM_33;

// MOTOR2 编码器：接口第3脚、第4脚
const gpio_num_t kEncoderRightA = GPIO_NUM_25;
const gpio_num_t kEncoderRightB = GPIO_NUM_26;

#if !APP_MOTOR_ONLY
const gpio_num_t kUltrasonicTrigPin = GPIO_NUM_21;
const gpio_num_t kUltrasonicEchoPin = GPIO_NUM_47;
const gpio_num_t kStatusLedPin = GPIO_NUM_14;
const gpio_num_t kServoPin = GPIO_NUM_48;
const adc_unit_t kBatteryAdcUnit = ADC_UNIT_1;
const adc_channel_t kBatteryAdcChannel = ADC_CHANNEL_2; // GPIO3 on ESP32-S3.

// GPIO0/BOOT is a strapping/download pin. Using it as a runtime button is risky,
// but it is enabled here intentionally for the BOOT long-press recovery action.
const gpio_num_t kIo0ButtonPin = GPIO_NUM_0;
#endif

// ================= 对象实例化 =================
#if !APP_MOTOR_ONLY
Lsm6ds3Imu imu(I2C_NUM_0, GPIO_NUM_12, GPIO_NUM_13);
#endif

At8236Motor motor_left(kMotorLeftIn1, kMotorLeftIn2);
// 按当前实物确认：原右轮正 PWM 导致后退且反馈为负。
// 交换逻辑输入，使正速度命令驱动右轮前进；物理接线定义保持不变。
At8236Motor motor_right(kMotorRightIn2, kMotorRightIn1);

QuadratureEncoder encoder_left(kEncoderLeftA, kEncoderLeftB);
QuadratureEncoder encoder_right(kEncoderRightA, kEncoderRightB);

MotionController robot(motor_left, motor_right, encoder_left, encoder_right);
#if !APP_MOTOR_ONLY
UltrasonicSensor ultrasonic(kUltrasonicTrigPin, kUltrasonicEchoPin);
StatusLed status_led(kStatusLedPin);
ButtonDriver io0_button(kIo0ButtonPin, true);
ServoDriver my_servo(kServoPin);
BatteryAdc battery_adc(kBatteryAdcUnit, kBatteryAdcChannel);
#endif

// ================= 统一初始化逻辑 =================
void board_init(void) {
    ESP_LOGI(TAG, "Initializing two-wheel differential hardware...");

    motor_left.Init();
    encoder_left.Init();
    motor_right.Init();
    encoder_right.Init();

    robot.Stop(); // 上电保持停止，等待明确的运动指令。
#if !APP_MOTOR_ONLY
    ultrasonic.Init();
    status_led.Init();
    io0_button.Init();
    ESP_LOGW(TAG, "BOOT/IO0 is used as a runtime communication-mode button; holding it during reset may enter download mode.");
    my_servo.Init();

    if (imu.Init()) {
        ESP_LOGI(TAG, "IMU (LSM6DS3) initialized.");
    } else {
        ESP_LOGE(TAG, "IMU initialization failed.");
    }
#endif

    ESP_LOGI(TAG, "Two-wheel differential board ready.");
}
