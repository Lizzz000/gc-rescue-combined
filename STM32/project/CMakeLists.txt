cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME project)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Algorithm/AHRS/AHRS.c
    Core/Algorithm/AHRS/MahonyAHRS.c
    Core/Algorithm/PID/pid.c
    Core/Algorithm/Chassis/motor_chassis.c
    Core/Bsp/ADC/bsp_adc.c
    Core/Bsp/Buzzer/bsp_buzzer.c
    Core/Bsp/CAN/bsp_can.c
    Core/Bsp/IMU_PWM/bsp_imu_pwm.c
    Core/Bsp/LED/bsp_led.c
    Core/Bsp/Transport_Servo/transport_servo.c
    Core/Communication/VOFA/vofa.c
    Core/Communication/Jetson/jetson_can.c
    Core/Communication/Manual/manual_can.c
    Core/Control/Chassis/chassis.c
    Core/Control/Jetson/image_tracker.c
    Core/Control/Jetson/jetson_auto.c
    Core/Control/Jetson/jetson_control.c
    Core/Devices/BMI088/BMI088driver.c
    Core/Devices/BMI088/BMI088Middleware.c
    Core/Devices/IST8310/ist8310driver.c
    Core/Devices/IST8310/ist8310driver_middleware.c
    Core/Devices/DM_Motor/dm_motor.c
    Core/Task/INS/INS_task.c
    Core/Task/CAN2/test_task.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Algorithm/AHRS
    Core/Algorithm/PID
    Core/Algorithm/Chassis
    Core/Bsp/ADC
    Core/Bsp/Buzzer
    Core/Bsp/CAN
    Core/Bsp/IMU_PWM
    Core/Bsp/LED
    Core/Bsp/Transport_Servo
    Core/Communication/VOFA
    Core/Communication/Jetson
    Core/Communication/Manual
    Core/Control/Chassis
    Core/Control/Jetson
    Core/Devices/BMI088
    Core/Devices/IST8310
    Core/Devices/DM_Motor
    Core/Task/INS
    Core/Task/CAN2
)

# Add project symbols (macros)
option(INS_USE_MAGNETOMETER "Use IST8310 for nine-axis Mahony heading" OFF)
if(INS_USE_MAGNETOMETER)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE INS_USE_MAGNETOMETER=1)
endif()

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
