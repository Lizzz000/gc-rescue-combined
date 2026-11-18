target_sources(${CMAKE_PROJECT_NAME} PRIVATE manual_link.c manual_usb.c manual_control.c)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")

if(UART6_MANUAL_ENABLED)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE uart6_manual.c)
endif()
if(CHASSIS_TUNING_ENABLED)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE live_tuning.c spin_observer.c)
endif()
