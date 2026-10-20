idf_component_register(
    SRCS "test_motion_controller.cpp" "../../../main/control/motion_controller.cpp"
    INCLUDE_DIRS "../../../main/control" "../../../main/msg" "../../../components/motor_driver"
                 "../../../components/encoder_driver"
    REQUIRES unity pid_controller esp_driver_gpio esp_driver_ledc esp_driver_pcnt esp_timer
)
