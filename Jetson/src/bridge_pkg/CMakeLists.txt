cmake_minimum_required(VERSION 3.18)
project(bridge_pkg)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(interface_pkg REQUIRED)

# 协议（仅头文件）、传输、状态各成一库；节点对象含组件注册宏，程序与组件复用这份 PIC 对象。
function(configure_bridge_ros target)
  target_include_directories(${target} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
  target_compile_features(${target} PUBLIC cxx_std_17)
  ament_target_dependencies(${target} rclcpp rclcpp_components rcl_interfaces
    std_msgs vision_msgs interface_pkg)
endfunction()

add_library(bridge_protocol INTERFACE)
target_include_directories(bridge_protocol INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(bridge_protocol INTERFACE cxx_std_17)

add_library(bridge_transport STATIC
  transport/transport.cpp
  transport/cdc/cdc.cpp)
target_link_libraries(bridge_transport PUBLIC bridge_protocol)
set_property(TARGET bridge_transport PROPERTY POSITION_INDEPENDENT_CODE ON)

add_library(bridge_state STATIC state/bridge_state.cpp state/bridge_transactions.cpp)
target_link_libraries(bridge_state PUBLIC bridge_protocol PRIVATE bridge_transport)
ament_target_dependencies(bridge_state PUBLIC interface_pkg)
set_property(TARGET bridge_state PROPERTY POSITION_INDEPENDENT_CODE ON)

add_library(bridge_node_obj OBJECT
  node/bridge_node.cpp
  node/bridge_session.cpp
  node/bridge_io.cpp
  gripper/s288_servo.cpp)
configure_bridge_ros(bridge_node_obj)
find_package(Threads REQUIRED)
target_link_libraries(bridge_node_obj bridge_transport bridge_state Threads::Threads)
set_property(TARGET bridge_node_obj PROPERTY POSITION_INDEPENDENT_CODE ON)

add_executable(bridge_runtime app/bridge_runtime.cpp)
configure_bridge_ros(bridge_runtime)
target_link_libraries(bridge_runtime bridge_node_obj)
install(TARGETS bridge_runtime DESTINATION lib/${PROJECT_NAME})

add_library(bridge_components SHARED)
configure_bridge_ros(bridge_components)
target_link_libraries(bridge_components bridge_node_obj)
rclcpp_components_register_nodes(bridge_components "BridgeNode")
install(TARGETS bridge_components LIBRARY DESTINATION lib ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

# 主机测试：CDC 设备锁与坐标周期。pty 代替 USB 串口，仅 Linux。
if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  find_package(teleop_pkg REQUIRED)
  add_executable(test_bridge_core_host
    test/host/test_main.cpp test/host/test_usb_protocol.cpp test/host/test_tx_queue.cpp
    test/host/test_bridge_state.cpp test/host/test_arrival_point.cpp test/host/test_search_request.cpp
    test/host/test_bridge_transactions.cpp)
  target_link_libraries(test_bridge_core_host PRIVATE bridge_state bridge_transport)
  ament_add_test(test_bridge_core_host COMMAND $<TARGET_FILE:test_bridge_core_host>
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO TIMEOUT 60)

  # 独立故障注入目标：编译生产代码，仅系统调用/ROS边界用桩，不链接真实设备后端。
  add_executable(test_bridge_runtime_host
    test/host/test_runtime_main.cpp test/host/test_bridge_runtime.cpp
    node/bridge_io.cpp node/bridge_session.cpp state/bridge_state.cpp state/bridge_transactions.cpp
    transport/transport.cpp)
  target_compile_features(test_bridge_runtime_host PRIVATE cxx_std_17)
  target_include_directories(test_bridge_runtime_host BEFORE PRIVATE
    test/host/runtime_stubs test/host/stubs test/host ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/..)
  ament_add_test(test_bridge_runtime_host COMMAND $<TARGET_FILE:test_bridge_runtime_host>
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO TIMEOUT 60)
  foreach(test_name cdc_write s288_timing)
    if(test_name STREQUAL "cdc_write")
      set(driver_source transport/cdc/cdc.cpp)
    else()
      set(driver_source gripper/s288_servo.cpp)
    endif()
    add_executable(test_${test_name} test/host/test_${test_name}.cpp ${driver_source})
    target_compile_features(test_${test_name} PRIVATE cxx_std_17)
    target_include_directories(test_${test_name} BEFORE PRIVATE
      test/host/posix_stubs test/host/stubs test/host ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/..)
    target_link_libraries(test_${test_name} Threads::Threads)
    ament_add_test(test_${test_name} COMMAND $<TARGET_FILE:test_${test_name}>
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO TIMEOUT 60)
  endforeach()
  add_executable(test_cdc_exclusive test/host/test_cdc_exclusive.cpp)
  target_include_directories(test_cdc_exclusive PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test/host)
  target_link_libraries(test_cdc_exclusive bridge_transport bridge_state util)
  ament_add_test(test_cdc_exclusive COMMAND $<TARGET_FILE:test_cdc_exclusive>
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO TIMEOUT 60)
  # S288 薄转发：协议实现只在 teleop_pkg/servo/s288.py
  ament_add_test(test_s288_forward
    COMMAND python3 -m unittest -v test_s288_protocol test_s288_debug
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/gripper
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO TIMEOUT 60)
endif()

ament_package()
