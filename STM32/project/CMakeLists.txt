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
project(${CMAKE_PROJECT_NAME} LANGUAGES C ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Algorithm/AHRS/AHRS.c
    Core/Algorithm/AHRS/MahonyAHRS.c
    Core/Algorithm/PID/pid.c
    Core/Algorithm/Chassis/motor_chassis.c
    Core/Algorithm/Chassis/velocity_profile.c
    Core/Control/Chassis/chassis_odometry.c
    Core/Algorithm/Chassis/wheel_alignment.c
    Core/Algorithm/Chassis/planar_odometry.c
    Core/Algorithm/Chassis/heading_yaw_guard.c
    Core/Algorithm/Chassis/heading_eskf.c
    Core/Bsp/ADC/bsp_adc.c
    Core/Bsp/Buzzer/bsp_buzzer.c
    Core/Bsp/CAN/bsp_can.c
    Core/Bsp/IMU_PWM/bsp_imu_pwm.c
    Core/Bsp/LED/bsp_led.c
    Core/Communication/VOFA/vofa.c
    Core/Communication/Common/link_frame.c
    Core/Communication/Common/usb_link.c
    Core/Control/Chassis/chassis.c
    Core/Control/Chassis/chassis_config.c
    Core/Devices/BMI088/BMI088driver.c
    Core/Devices/BMI088/BMI088Middleware.c
    Core/Devices/IST8310/ist8310driver.c
    Core/Devices/IST8310/ist8310driver_middleware.c
    Core/Devices/DM_Motor/dm_motor.c
    Core/Task/INS/INS_task.c
    Core/Task/Chassis/chassis_task.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c
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
    Core/Communication/VOFA
    Core/Communication/Common
    Core/Control/Chassis
    Core/Devices/BMI088
    Core/Devices/IST8310
    Core/Devices/DM_Motor
    Core/Task/INS
    Core/Task/Chassis
)

# Add project symbols (macros)
# 与 hal_conf.h 的空宏一致，避免重定义告警；Cube 重新生成注释掉它时仍启用 PCD。
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE HAL_PCD_MODULE_ENABLED=)
option(CHASSIS_TUNING_ENABLED "Build explicit-command SWD chassis tuning firmware" OFF)
if(CHASSIS_TUNING_ENABLED)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE CHASSIS_TUNING_ENABLED=1 JETSON_AUTO_ENABLED=0)
endif()
option(UART6_MANUAL_ENABLED "Build dedicated USART6 chassis remote control firmware" OFF)
if(UART6_MANUAL_ENABLED)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE UART6_MANUAL_ENABLED=1)
endif()
option(JETSON_AUTO_ENABLED "Build the Jetson USB automatic transport controller" OFF)
if(JETSON_AUTO_ENABLED)
    if(CHASSIS_TUNING_ENABLED OR UART6_MANUAL_ENABLED)
        message(FATAL_ERROR "Automatic transport requires exclusive chassis control")
    endif()
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE JETSON_AUTO_ENABLED=1)
endif()
option(JETSON_AUTO_CHASSIS_TEST "Build chassis-only Jetson USB test firmware without gripper actions" OFF)
if(JETSON_AUTO_CHASSIS_TEST)
    if(JETSON_AUTO_ENABLED OR CHASSIS_TUNING_ENABLED OR UART6_MANUAL_ENABLED)
        message(FATAL_ERROR "Chassis-only test requires exclusive chassis control")
    endif()
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE JETSON_AUTO_CHASSIS_TEST=1)
endif()
if(CHASSIS_TUNING_ENABLED AND (UART6_MANUAL_ENABLED OR JETSON_AUTO_ENABLED OR JETSON_AUTO_CHASSIS_TEST))
    message(FATAL_ERROR "Chassis tuning cannot share control with USART6 or Jetson modes")
endif()
if(UART6_MANUAL_ENABLED AND (JETSON_AUTO_ENABLED OR JETSON_AUTO_CHASSIS_TEST))
    message(FATAL_ERROR "USART6 manual cannot share chassis control with Jetson modes")
endif()

# 直接使用固件的模式宏（包括已有 -DCMAKE_C_FLAGS=-DJETSON_AUTO_...）。
# 更改模式头或重新配置编译标志时，重新选择对应的控制器与业务协议源文件。
include(CheckCSourceCompiles)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS Core/Control/Chassis/chassis_mode.h)
set(CMAKE_REQUIRED_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/Core/Control/Chassis")
set(CMAKE_REQUIRED_QUIET TRUE)
set(CMAKE_TRY_COMPILE_CONFIGURATION "${CMAKE_BUILD_TYPE}")
get_target_property(CMAKE_REQUIRED_DEFINITIONS ${CMAKE_PROJECT_NAME} COMPILE_DEFINITIONS)
list(TRANSFORM CMAKE_REQUIRED_DEFINITIONS PREPEND "-D")
unset(BUILD_JETSON_CONTROLLER CACHE)
check_c_source_compiles("#include \"chassis_mode.h\"
    #if !JETSON_AUTO_CAN_ENABLED
    #error Manual controller selected
    #endif
    int main(void) { return 0; }" BUILD_JETSON_CONTROLLER)
if(BUILD_JETSON_CONTROLLER)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE Core/Communication/Jetson/jetson_link.c
        Core/Communication/Jetson/jetson_usb.c Core/Control/Jetson/jetson_auto.c)
    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE Core/Communication/Jetson Core/Control/Jetson)
else()
    add_subdirectory(../manual_test/firmware manual_test)
endif()
if(BUILD_JETSON_CONTROLLER OR CHASSIS_TUNING_ENABLED)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        Core/Control/Vision/tracked_visual_servo.c
        Core/Algorithm/Vision/adaptive_kalman.c
        Core/Algorithm/Vision/visual_servo.c)
    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE Core/Control/Vision Core/Algorithm/Vision)
endif()
if(BUILD_JETSON_CONTROLLER OR CHASSIS_TUNING_ENABLED OR UART6_MANUAL_ENABLED)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE Core/Algorithm/Chassis/point_plan.c)
endif()
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

# Keep standalone flashing images in sync with the ELF used by CLion.
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND "${CMAKE_OBJCOPY}" -O ihex
        "$<TARGET_FILE:${CMAKE_PROJECT_NAME}>" "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex"
    COMMAND "${CMAKE_OBJCOPY}" -O binary
        "$<TARGET_FILE:${CMAKE_PROJECT_NAME}>" "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin"
    BYPRODUCTS
        "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex"
        "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin"
    COMMENT "Generating HEX and BIN flashing images"
    VERBATIM
)
