#include "motion_controller.h"
#include "motor_pid_msg.h"
#include "motor_start_timing.h"
#include "unity.h"
#include <cmath>

// Replace only the hardware boundary; run the real motion controller and PID.
namespace {
int16_t pwm[2] = {};
bool braking[2] = {};
int32_t counts[2] = {};
}

At8236Motor::At8236Motor(gpio_num_t a, gpio_num_t b) : in1_pin_(a), in2_pin_(b) {}
void At8236Motor::Init() {}
void At8236Motor::SetSpeed(int16_t value) {
    pwm[in1_pin_] = value;
    braking[in1_pin_] = false;
}
void At8236Motor::Brake() { pwm[in1_pin_] = 0; braking[in1_pin_] = true; }
QuadratureEncoder::QuadratureEncoder(gpio_num_t a, gpio_num_t b) : pin_a_(a), pin_b_(b) {}
QuadratureEncoder::~QuadratureEncoder() = default;
int32_t QuadratureEncoder::GetCount() { return counts[pin_a_]; }
void QuadratureEncoder::ResetCount() { counts[pin_a_] = 0; }

static void test_open_loop_controller() {
    At8236Motor left(GPIO_NUM_0, GPIO_NUM_2), right(GPIO_NUM_1, GPIO_NUM_3);
    QuadratureEncoder enc_left(GPIO_NUM_0, GPIO_NUM_2), enc_right(GPIO_NUM_1, GPIO_NUM_3);
    MotionController robot(left, right, enc_left, enc_right);

    // A 1% input must stay small even with large PID gains and acceleration limiting.
    robot.SetVelocityPidGains(100, 100, 100);
    robot.SetMaxAcceleration(1);
    robot.SetAllMotorDuties(1, -50);
    robot.Update(0.02f, NAN);
    TEST_ASSERT_EQUAL_INT16(3, pwm[0]);
    TEST_ASSERT_EQUAL_INT16(-128, pwm[1]);
    // Feedback sign and rad/s scale: 78 counts per 20 ms = 5.235987756 rad/s.
    counts[0] = -78;
    counts[1] = -78;
    robot.Update(0.02f, NAN);
    float l, r;
    robot.GetAllMotorAngularVelocities(&l, &r);
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, 5.235987756f, l);
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, -5.235987756f, r);
    robot.GetAllMotorDuties(&l, &r);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.176470588f, l);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, -50.19607843f, r);
    robot.Update(0.02f, NAN); // No new pulses: raw feedback must immediately be zero.
    robot.GetAllMotorAngularVelocities(&l, &r);
    TEST_ASSERT_EQUAL_FLOAT(0, l);
    TEST_ASSERT_EQUAL_FLOAT(0, r);

    robot.SetAllMotorDuties(200, -200);
    robot.Update(0.02f, NAN);
    TEST_ASSERT_EQUAL_INT16(255, pwm[0]);
    TEST_ASSERT_EQUAL_INT16(-255, pwm[1]);
    robot.SetAllMotorDuties(NAN, 10); // Invalid internal callers must fail closed.
    robot.Update(0.02f, NAN);
    TEST_ASSERT_TRUE(braking[0] && braking[1]);
    robot.SetAllMotorDuties(50, 0);
    robot.Update(0.02f, NAN);
    TEST_ASSERT_EQUAL_INT16(128, pwm[0]);
    TEST_ASSERT_EQUAL_INT16(0, pwm[1]);
    TEST_ASSERT_FALSE(braking[1]); // Zero duty is coast, not active braking.
    robot.Stop();
    robot.Update(0.02f, NAN);
    TEST_ASSERT_TRUE(braking[0] && braking[1]);
    robot.GetAllMotorDuties(&l, &r);
    TEST_ASSERT_EQUAL_FLOAT(0, l);
    TEST_ASSERT_EQUAL_FLOAT(0, r);

    // Feedback must still track residual wheel motion while the stopped controller brakes.
    counts[0] = -156;
    counts[1] = 0;
    robot.Update(0.02f, NAN);
    robot.GetAllMotorAngularVelocities(&l, &r);
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, 5.235987756f, l);
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, 5.235987756f, r);
    TEST_ASSERT_TRUE(braking[0] && braking[1]);
    robot.Update(0.02f, NAN);
    robot.GetAllMotorAngularVelocities(&l, &r);
    TEST_ASSERT_EQUAL_FLOAT(0, l);
    TEST_ASSERT_EQUAL_FLOAT(0, r);

    // Returning from open loop to one-wheel velocity control must clear the other wheel.
    robot.SetAllMotorDuties(100, 100);
    robot.Update(0.02f, NAN);
    robot.SetMotorTargetVelocity(MotorID::kLeft, 50);
    robot.Update(0.02f, NAN);
    TEST_ASSERT_TRUE(pwm[0] > 0);
    TEST_ASSERT_EQUAL_INT16(0, pwm[1]);
    robot.SetAllMotorDuties(100, 100);
    robot.Drive(0, 0, 0);
    robot.Update(0.02f, NAN);
    TEST_ASSERT_TRUE(braking[0] && braking[1]);
}

static void test_sampling_period_preserves_speed_and_filter_response() {
    float velocity[2], omega[2];
    for (int sample_rate = 0; sample_rate < 2; ++sample_rate) {
        counts[0] = counts[1] = 0;
        At8236Motor left(GPIO_NUM_0, GPIO_NUM_2), right(GPIO_NUM_1, GPIO_NUM_3);
        QuadratureEncoder enc_left(GPIO_NUM_0, GPIO_NUM_2), enc_right(GPIO_NUM_1, GPIO_NUM_3);
        MotionController robot(left, right, enc_left, enc_right);
        robot.SetAllMotorDuties(80, 80);
        const int samples = sample_rate == 0 ? 1 : 2;
        const float dt = sample_rate == 0 ? 0.02f : 0.01f;
        const int counts_per_sample = sample_rate == 0 ? 78 : 39;
        for (int i = 0; i < samples; ++i) {
            counts[0] -= counts_per_sample;
            counts[1] += counts_per_sample;
            robot.Update(dt, NAN);
        }
        float right_velocity, right_omega;
        robot.GetAllMotorVelocities(&velocity[sample_rate], &right_velocity);
        robot.GetAllMotorAngularVelocities(&omega[sample_rate], &right_omega);
        TEST_ASSERT_FLOAT_WITHIN(0.00001f, 5.235987756f, omega[sample_rate]);
        TEST_ASSERT_FLOAT_WITHIN(0.00001f, omega[sample_rate], right_omega);
        TEST_ASSERT_FLOAT_WITHIN(0.001f, velocity[sample_rate], right_velocity);
        TEST_ASSERT_EQUAL_INT16(204, pwm[0]);
        TEST_ASSERT_EQUAL_INT16(204, pwm[1]);
    }
    // The same motion over 20 ms must give the same filtered speed at 50/100 Hz.
    TEST_ASSERT_FLOAT_WITHIN(0.001f, velocity[0], velocity[1]);
}

static void test_motor_pid_updates_are_independent() {
    counts[0] = counts[1] = 0;
    At8236Motor left(GPIO_NUM_0, GPIO_NUM_2), right(GPIO_NUM_1, GPIO_NUM_3);
    QuadratureEncoder enc_left(GPIO_NUM_0, GPIO_NUM_2), enc_right(GPIO_NUM_1, GPIO_NUM_3);
    MotionController robot(left, right, enc_left, enc_right);
    robot.SetVelocityPidGains(0, 0, 0);
    robot.SetMotorVelocityPidGains(MotorID::kLeft, 0.5f, 0, 0);
    robot.SetMotorVelocityPidGains(MotorID::kRight, 0, 5, 0);
    // Stay away from integer PWM boundaries so float rounding cannot mask isolation.
    const float target_mms = 10.25f * (M_PI * 65.0f) / 60.0f;
    robot.SetAllMotorTargetsVelocity(target_mms, target_mms);
    robot.Update(0.02f, NAN);
    TEST_ASSERT_EQUAL_INT16(155, pwm[0]);
    TEST_ASSERT_EQUAL_INT16(151, pwm[1]);
    robot.Update(0.02f, NAN);
    TEST_ASSERT_EQUAL_INT16(152, pwm[1]);

    // Left retuning must preserve both the right gains and its accumulated integral.
    robot.SetMotorVelocityPidGains(MotorID::kLeft, 2, 0, 0);
    robot.Update(0.02f, NAN);
    TEST_ASSERT_EQUAL_INT16(170, pwm[0]);
    TEST_ASSERT_EQUAL_INT16(153, pwm[1]);
    robot.SetMotorVelocityPidGains(MotorID::kRight, 1, 0, 0);
    robot.Update(0.02f, NAN);
    TEST_ASSERT_EQUAL_INT16(170, pwm[0]);
    TEST_ASSERT_EQUAL_INT16(160, pwm[1]);
    robot.SetMotorVelocityPidGains(static_cast<MotorID>(-1), 100, 100, 100);
    robot.SetMotorVelocityPidGains(static_cast<MotorID>(2), 100, 100, 100);
    robot.Update(0.02f, NAN);
    TEST_ASSERT_EQUAL_INT16(170, pwm[0]);
    TEST_ASSERT_EQUAL_INT16(160, pwm[1]);

    // The legacy setter must still apply a common gain to both controllers.
    robot.SetVelocityPidGains(0.5f, 0, 0);
    robot.Update(0.02f, NAN);
    TEST_ASSERT_EQUAL_INT16(155, pwm[0]);
    TEST_ASSERT_EQUAL_INT16(155, pwm[1]);
}

static void test_default_speed_pid_uses_each_wheels_config() {
    counts[0] = counts[1] = 0;
    At8236Motor left(GPIO_NUM_0, GPIO_NUM_2), right(GPIO_NUM_1, GPIO_NUM_3);
    QuadratureEncoder enc_left(GPIO_NUM_0, GPIO_NUM_2), enc_right(GPIO_NUM_1, GPIO_NUM_3);
    MotionController robot(left, right, enc_left, enc_right);
    const float target_mms = 10.25f * (M_PI * 65.0f) / 60.0f;
    robot.SetAllMotorTargetsVelocity(target_mms, target_mms);
    robot.Update(0.02f, NAN);
    const int16_t constructed_pwm[] = {pwm[0], pwm[1]};
    // Construction must match explicitly applying each side's configuration.
    // Use the real controller rather than duplicating its PID/PWM calculation.
    MotionController configured(left, right, enc_left, enc_right);
    configured.SetMotorVelocityPidGains(MotorID::kLeft, kDefaultLeftMotorSpeedPid.kp,
        kDefaultLeftMotorSpeedPid.ki, kDefaultLeftMotorSpeedPid.kd);
    configured.SetMotorVelocityPidGains(MotorID::kRight, kDefaultRightMotorSpeedPid.kp,
        kDefaultRightMotorSpeedPid.ki, kDefaultRightMotorSpeedPid.kd);
    configured.SetAllMotorTargetsVelocity(target_mms, target_mms);
    configured.Update(0.02f, NAN);
    TEST_ASSERT_EQUAL_INT16(constructed_pwm[0], pwm[0]);
    TEST_ASSERT_EQUAL_INT16(constructed_pwm[1], pwm[1]);
}

static void test_start_timing_separates_pwm_writes_from_encoder_samples() {
    MotorStartTiming timing;
    timing.Begin(1000000);
    timing.RecordEncoder(0, 1, 1000001); // Sampling before the first output is not a response.
    TEST_ASSERT_EQUAL_INT64(-1, timing.first_encoder_us[0]);
    timing.RecordPwm(0, 186, 1000010, 1000015);
    timing.RecordPwm(1, 186, 1000020, 1000025);
    timing.RecordEncoder(1, 1, 1020000);
    timing.RecordPwm(0, 201, 1020010, 1020015);
    timing.RecordPwm(0, 215, 1040010, 1040015);
    timing.RecordEncoder(0, -1, 1100000);
    // An 80 ms feedback difference must not be confused with the 10 us write difference.
    TEST_ASSERT_EQUAL_INT64(10, timing.first_pwm_begin_us[1] - timing.first_pwm_begin_us[0]);
    TEST_ASSERT_EQUAL_INT64(80000, timing.first_encoder_us[0] - timing.first_encoder_us[1]);
    TEST_ASSERT_EQUAL_INT16(186, timing.first_pwm[0]);
    TEST_ASSERT_EQUAL_INT16(215, timing.pwm_at_encoder[0]);
    timing.RecordEncoder(0, 5, 1120000);
    TEST_ASSERT_EQUAL_INT64(1100000, timing.first_encoder_us[0]);
    timing.End();
    timing.RecordPwm(0, 0, 1140000, 1140005);
    TEST_ASSERT_EQUAL_INT16(186, timing.first_pwm[0]);
    timing.Begin(2000000);
    TEST_ASSERT_EQUAL_UINT32(2, timing.command_id);
    TEST_ASSERT_EQUAL_INT64(-1, timing.first_pwm_begin_us[0]);
    TEST_ASSERT_EQUAL_INT64(-1, timing.first_encoder_us[1]);
}

extern "C" void app_main() {
    UNITY_BEGIN();
    RUN_TEST(test_open_loop_controller);
    RUN_TEST(test_sampling_period_preserves_speed_and_filter_response);
    RUN_TEST(test_motor_pid_updates_are_independent);
    RUN_TEST(test_default_speed_pid_uses_each_wheels_config);
    RUN_TEST(test_start_timing_separates_pwm_writes_from_encoder_samples);
    UNITY_END();
}
