cmake_minimum_required(VERSION 3.16)

option(SHARKBOT_MOTOR_ONLY "Build only motor control and the serial debug console" ON)
if(SHARKBOT_MOTOR_ONLY)
    # Keep the original manifest and peripheral components for later restoration.
    # None of their registry/micro-ROS dependencies are needed for motor debugging.
    set(ENV{IDF_COMPONENT_MANAGER} "0")
    set(COMPONENTS main)
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(SHARKBOT_MOTOR_ONLY ${SHARKBOT_MOTOR_ONLY})
project(leap_low_v1)

if(TARGET merge-bin)
    add_custom_target(merged_bin ALL DEPENDS merge-bin)
endif()
