idf_build_get_property(motor_only SHARKBOT_MOTOR_ONLY)
if(motor_only)
    idf_component_register(
        SRCS
            "app_main.cpp"
            "app_runtime.cpp"
            "config/pid_config.cpp"
            "core/system_globals.cpp"
            "platform/board.cpp"
            "control/motion_controller.cpp"
            "tasks/control/motion_task.cpp"
            "tasks/control/motor_debug_console.cpp"
        INCLUDE_DIRS "." "config" "core" "platform" "control" "msg" "tasks/control"
        REQUIRES motor_driver encoder_driver pid_controller nvs_flash console
                 esp_driver_uart esp_driver_usb_serial_jtag vfs
    )
    target_compile_definitions(${COMPONENT_LIB} PUBLIC APP_MOTOR_ONLY=1)
    return()
endif()

# Original full firmware: temporarily disabled by SHARKBOT_MOTOR_ONLY (default ON).
set(APP_SRCS
    "app_main.cpp"
    "app_runtime.cpp"
    "config/pid_config.cpp"
    "core/system_globals.cpp"
    "platform/board.cpp"
    "control/motion_controller.cpp"
    "tasks/control/motion_task.cpp"
    "tasks/sensors/imu_task.cpp"
    "tasks/sensors/ultrasonic_task.cpp"
    "tasks/sensors/battery_task.cpp"
    "tasks/sensors/lidar_task.cpp"
    "tasks/io/peripheral_task.cpp"
    "tasks/io/gamepad_i2c_task.cpp"
    "tasks/network/mavlink_common.cpp"
    "tasks/network/mavlink_udp_task.cpp"
    "tasks/network/mavlink_uart_task.cpp"
    "tasks/network/wifi_provision_task.cpp"
    "tasks/network/microros_task.cpp"
    "network/wifi_app.cpp"
)

idf_component_register(
    SRCS ${APP_SRCS}
    INCLUDE_DIRS
        "."
        "config"
        "core"
        "platform"
        "control"
        "msg"
        "network"
        "tasks/network"
        "../components/i2c_bus_lock"
        "../components/mavlink_lib"
    REQUIRES
        esp_http_server
        esp_wifi
        esp_netif
        esp_driver_tsens
        spi_flash
        nvs_flash
        lwip
        motor_driver
        encoder_driver
        ultrasonic_sensor
        button_driver
        servo_driver
        lsm6ds3_driver
        pid_controller
        camsense_lidar
        status_led
        battery_adc
        camera_i2c_client
        i2c_bus_lock
        mavlink_lib
        micro_ros_espidf_component
)
target_compile_definitions(${COMPONENT_LIB} PUBLIC APP_MOTOR_ONLY=0)
