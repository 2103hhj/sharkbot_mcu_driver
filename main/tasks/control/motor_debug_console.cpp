#include "motor_debug_console.h"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <atomic>

#include "esp_console.h"
#include "esp_log.h"
#include "freertos/task.h"
#if CONFIG_ESP_CONSOLE_UART_DEFAULT || CONFIG_ESP_CONSOLE_UART_CUSTOM
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#elif CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#elif CONFIG_ESP_CONSOLE_USB_CDC
#include "esp_vfs_cdcacm.h"
#endif
#include "msg/motion_msg.h"
#include "system_globals.h"

namespace {

std::atomic<bool> s_firewater_enabled{false};
esp_log_level_t s_text_log_level = ESP_LOG_INFO;

void set_firewater(bool enabled) {
    if (enabled) {
        esp_log_level_set("*", ESP_LOG_NONE);
        s_firewater_enabled.store(true);
    } else {
        s_firewater_enabled.store(false);
        esp_log_level_set("*", s_text_log_level);
    }
}

// Read the latest snapshot, never access the motor/encoder objects from this task.
// Serial transmission can block without delaying the higher-priority control loop.
void telemetry_task(void *) {
    TickType_t next_wake = xTaskGetTickCount();
    while (true) {
        MotionMsg state = {};
        if (s_firewater_enabled.load() && xQueuePeek(q_motion_state, &state, 0) == pdTRUE) {
            printf("speed:%.3f,%.3f\n", state.vel_left, state.vel_right);
        }
        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(20));
    }
}

int set_vofa(int argc, char **argv) {
    if (argc != 2 || (std::strcmp(argv[1], "0") != 0 && std::strcmp(argv[1], "1") != 0)) {
        if (!s_firewater_enabled.load()) puts("Usage vofa 0 (text mode) or vofa 1 (FireWater 50 Hz)");
        return 1;
    }
    const bool enabled = argv[1][0] == '1';
    set_firewater(enabled);
    if (!enabled) puts("FireWater disabled; text commands available (help)");
    return 0;
}

bool parse_number(const char *text, float min, float max, float *value) {
    char *end = nullptr;
    errno = 0;
    const float parsed = std::strtof(text, &end);
    if (end == text || *end != '\0' || errno == ERANGE || !std::isfinite(parsed) ||
        parsed < min || parsed > max) return false;
    *value = parsed;
    return true;
}

int set_motors(int argc, char **argv) {
    MotionMsg cmd = {};
    cmd.source = MOTION_SRC_UART;
    cmd.control_mode = 6;
    if (argc != 3 || !parse_number(argv[1], -500, 500, &cmd.target_vx) ||
        !parse_number(argv[2], -500, 500, &cmd.target_vy)) {
        if (!s_firewater_enabled.load()) puts("Usage motor <left_mm/s> <right_mm/s> (each -500..500)");
        return 1;
    }
    xQueueOverwrite(q_motion_cmd, &cmd);
    if (!s_firewater_enabled.load()) {
        printf("Targets left=%.2f right=%.2f mm/s; auto-stop in %u ms\n",
               cmd.target_vx, cmd.target_vy, kMotorCommandTimeoutMs);
    }
    return 0;
}

int stop_motors(int argc, char **) {
    if (argc != 1) return 1;
    MotionMsg cmd = {};
    cmd.source = MOTION_SRC_UART; // Mode 0, all-zero velocity calls robot.Stop().
    xQueueOverwrite(q_motion_cmd, &cmd);
    if (!s_firewater_enabled.load()) puts("Stop requested");
    return 0;
}

int show_status(int argc, char **) {
    if (argc != 1) return 1;
    if (s_firewater_enabled.load()) return 0; // The continuous stream already reports speed.
    MotionMsg status = {};
    if (xQueuePeek(q_motion_state, &status, 0) != pdTRUE) {
        puts("Motion feedback is not ready yet");
        return 1;
    }
    printf("Measured left=%.2f right=%.2f mm/s; vx=%.2f mm/s wz=%.3f rad/s\n"
           "Encoder odometry x=%.2f y=%.2f mm yaw=%.3f rad\n",
           status.vel_left, status.vel_right, status.vx, status.wz,
           status.x, status.y, status.yaw);
    return 0;
}

int set_pid(int argc, char **argv) {
    PidMsg pid = {};
    if (argc != 4 || !parse_number(argv[1], 0, 1000, &pid.kp) ||
        !parse_number(argv[2], 0, 1000, &pid.ki) ||
        !parse_number(argv[3], 0, 1000, &pid.kd)) {
        if (!s_firewater_enabled.load()) puts("Usage pid <kp> <ki> <kd> (each 0..1000; RAM only, resets on reboot)");
        return 1;
    }
    xQueueOverwrite(q_speedpid_cmd, &pid);
    if (!s_firewater_enabled.load()) {
        printf("Speed PID requested kp=%.3f ki=%.3f kd=%.3f (RAM only)\n",
               pid.kp, pid.ki, pid.kd);
    }
    return 0;
}

// A plain line receiver avoids terminal probing, ANSI prompts and character echo
// on the same serial connection as FireWater. Accept LF, CR or CRLF commands.
void command_task(void *) {
    char line[128] = {};
    size_t length = 0;
    bool overflow = false;
    while (true) {
        const int ch = getchar();
        if (ch == EOF) {
            clearerr(stdin);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (ch == '\r' || ch == '\n') {
            if (!overflow && length > 0) {
                line[length] = '\0';
                int result = 0;
                const esp_err_t err = esp_console_run(line, &result);
                if (!s_firewater_enabled.load() && (err != ESP_OK || result != 0)) {
                    puts("Command rejected; use help for syntax");
                }
            } else if (overflow && !s_firewater_enabled.load()) {
                puts("Command too long; discarded");
            }
            length = 0;
            overflow = false;
        } else if (!overflow && (ch == '\b' || ch == 127)) {
            if (length > 0) --length;
        } else if (!overflow && ch >= 32 && ch <= 126) {
            if (length < sizeof(line) - 1) line[length++] = static_cast<char>(ch);
            else overflow = true;
        }
    }
}

} // namespace

void motor_debug_console_start() {
    s_text_log_level = esp_log_level_get("*");
    set_firewater(true);
    fflush(stdout);
#if CONFIG_ESP_CONSOLE_UART_DEFAULT || CONFIG_ESP_CONSOLE_UART_CUSTOM
    constexpr uart_port_t port = static_cast<uart_port_t>(CONFIG_ESP_CONSOLE_UART_NUM);
    uart_config_t uart = {};
    uart.baud_rate = CONFIG_ESP_CONSOLE_UART_BAUDRATE;
    uart.data_bits = UART_DATA_8_BITS;
    uart.parity = UART_PARITY_DISABLE;
    uart.stop_bits = UART_STOP_BITS_1;
    uart.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart.source_clk = UART_SCLK_DEFAULT;
    ESP_ERROR_CHECK(uart_driver_install(port, 1024, 2048, 0, nullptr, 0));
    ESP_ERROR_CHECK(uart_param_config(port, &uart));
    ESP_ERROR_CHECK(uart_set_pin(port, CONFIG_ESP_CONSOLE_UART_TX_GPIO,
                                 CONFIG_ESP_CONSOLE_UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    uart_vfs_dev_port_set_rx_line_endings(port, ESP_LINE_ENDINGS_LF);
    uart_vfs_dev_port_set_tx_line_endings(port, ESP_LINE_ENDINGS_CRLF);
    uart_vfs_dev_use_driver(port);
#elif CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    usb_serial_jtag_driver_config_t device = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&device));
    usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_LF);
    usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);
    usb_serial_jtag_vfs_use_driver();
#elif CONFIG_ESP_CONSOLE_USB_CDC
    esp_vfs_dev_cdcacm_set_rx_line_endings(ESP_LINE_ENDINGS_LF);
    esp_vfs_dev_cdcacm_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);
#else
#error "Motor debugging requires a UART or USB console enabled in menuconfig"
#endif

    setvbuf(stdin, nullptr, _IONBF, 0);
    esp_console_config_t console = {};
    // ESP-IDF reserves one slot for nullptr. Also preserve excess arguments so
    // handlers can reject them instead of executing a silently truncated command.
    console.max_cmdline_args = 8;
    console.max_cmdline_length = 128;
    ESP_ERROR_CHECK(esp_console_init(&console));
    ESP_ERROR_CHECK(esp_console_register_help_command());
    const struct {
        const char *name;
        const char *help;
        esp_console_cmd_func_t handler;
    } commands[] = {
        {"motor", "Set left/right wheel speed in mm/s (-500..500), timeout 2s", set_motors},
        {"stop", "Stop both motors", stop_motors},
        {"status", "Show measured wheel speed and encoder odometry", show_status},
        {"pid", "Set speed PID kp ki kd (0..1000, RAM only)", set_pid},
        {"vofa", "Enable FireWater with 1 or text mode with 0", set_vofa},
    };
    for (const auto &command : commands) {
        esp_console_cmd_t config = {};
        config.command = command.name;
        config.help = command.help;
        config.func = command.handler;
        ESP_ERROR_CHECK(esp_console_cmd_register(&config));
    }
    ESP_ERROR_CHECK(xTaskCreate(command_task, "motor_commands", 4096, nullptr, 3, nullptr) == pdPASS
                        ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreate(telemetry_task, "motor_telemetry", 4096, nullptr, 2, nullptr) == pdPASS
                        ? ESP_OK : ESP_ERR_NO_MEM);
}
