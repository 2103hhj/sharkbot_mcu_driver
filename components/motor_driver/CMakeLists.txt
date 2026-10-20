idf_component_register(
    SRCS "motor_driver.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_ledc
)
