# 构建任务运行时。ROI、任务控制、节点与入口分目录维护。

cmake_minimum_required(VERSION 3.18)
project(mission_pkg)

# 构建类型：开发默认 RelWithDebInfo（保留优化且带调试符号）；板端生产构建用
# colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release 覆盖本默认值。
# 显式传入 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON，在本包构建目录生成 compile_commands.json。
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
    "Build type: RelWithDebInfo for development, Release for the board build" FORCE)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

# ROS 依赖集中在节点与转换库，运输控制及 ROI 核心保持可脱离 ROS 编译。
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(interface_pkg REQUIRED)
find_package(vision_msgs REQUIRED)

# ROI、任务控制为 PIC 静态库，节点为 PIC 对象；程序与组件复用同一份对象。
function(configure_node_target target)
  target_include_directories(${target} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
  target_compile_features(${target} PUBLIC cxx_std_17)
  ament_target_dependencies(${target} rclcpp std_msgs std_srvs rcl_interfaces interface_pkg vision_msgs rclcpp_components)
endfunction()
add_library(mission_roi STATIC roi/roi.cpp)
configure_node_target(mission_roi)
set_property(TARGET mission_roi PROPERTY POSITION_INDEPENDENT_CODE ON)
add_library(mission_control STATIC control/controller.cpp)
configure_node_target(mission_control)
set_property(TARGET mission_control PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(mission_control mission_roi)
add_library(mission_nodes OBJECT node/mission_node.cpp node/target_node.cpp node/safety_node.cpp)
configure_node_target(mission_nodes)
set_property(TARGET mission_nodes PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(mission_nodes mission_control mission_roi)
# 一个程序提供三个单节点角色；完整部署仍分别创建 mission/target/safety 三个进程。
add_executable(mission_runtime app/mission_runtime.cpp)
configure_node_target(mission_runtime)
target_link_libraries(mission_runtime mission_nodes mission_control mission_roi)
install(TARGETS mission_runtime DESTINATION lib/${PROJECT_NAME})
add_library(mission_components SHARED)
configure_node_target(mission_components)
target_link_libraries(mission_components mission_nodes mission_control mission_roi)
rclcpp_components_register_nodes(mission_components "MissionNode" "TargetNode" "SafetyNode")
install(TARGETS mission_components ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
ament_package()
