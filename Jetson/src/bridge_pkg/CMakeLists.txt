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
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(interface_pkg REQUIRED)

# 协议（仅头文件）、传输、状态各成一库；节点对象含组件注册宏，程序与组件复用这份 PIC 对象。
function(configure_bridge_ros target)
  target_include_directories(${target} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
  target_compile_features(${target} PUBLIC cxx_std_17)
  ament_target_dependencies(${target} rclcpp rclcpp_components rcl_interfaces
    std_msgs geometry_msgs vision_msgs interface_pkg)
endfunction()

add_library(bridge_protocol INTERFACE)
target_include_directories(bridge_protocol INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(bridge_protocol INTERFACE cxx_std_17)

add_library(bridge_transport STATIC
  transport/transport.cpp
  transport/socketcan/socketcan.cpp
  transport/damiao/damiao.cpp)
target_link_libraries(bridge_transport PUBLIC bridge_protocol)
set_property(TARGET bridge_transport PROPERTY POSITION_INDEPENDENT_CODE ON)

add_library(bridge_state STATIC state/bridge_state.cpp)
target_link_libraries(bridge_state PUBLIC bridge_protocol)
ament_target_dependencies(bridge_state PUBLIC rclcpp interface_pkg)
set_property(TARGET bridge_state PROPERTY POSITION_INDEPENDENT_CODE ON)

add_library(bridge_node_obj OBJECT
  node/bridge_node.cpp
  node/bridge_session.cpp
  node/bridge_io.cpp)
configure_bridge_ros(bridge_node_obj)
target_link_libraries(bridge_node_obj bridge_transport bridge_state)
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

ament_package()
