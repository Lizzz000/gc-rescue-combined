# 构建跟踪运行时。选择、跟踪和节点在同一份源里，OC-SORT 仍用第三方库。

cmake_minimum_required(VERSION 3.18)
project(track_pkg)

# 构建类型：开发默认 RelWithDebInfo（保留优化且带调试符号）；板端生产构建用
# colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release 覆盖本默认值。
# 显式传入 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON，在本包构建目录生成 compile_commands.json。
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
    "Build type: RelWithDebInfo for development, Release for the board build" FORCE)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

# ROS 适配依赖与 OC-SORT 所需 Eigen 分开注册，值类型核心不引入 ROS 消息库。
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(interface_pkg REQUIRED)
find_package(Eigen3 REQUIRED)
# vendored OC-SORT 已含 README 记录的本地修补；本包适配层整理不修改其算法、注释或许可证。
# 上游自带的回归测试不属于本包合同清单，不登记。
add_subdirectory(third_party/ocsort)

# 选择、跟踪和两个节点分文件编译，仍进原来的库；OC-SORT 仍链接第三方库。
function(configure_node_target target)
  target_include_directories(${target} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
  target_compile_features(${target} PUBLIC cxx_std_17)
  ament_target_dependencies(${target} rclcpp std_msgs rcl_interfaces vision_msgs interface_pkg rclcpp_components)
endfunction()
add_library(track_core STATIC tracking/tracking.cpp tracking/selector.cpp)
target_include_directories(track_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(track_core PUBLIC cxx_std_17)
ament_target_dependencies(track_core rclcpp interface_pkg)
set_property(TARGET track_core PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(track_core ocsort_core)
add_library(track_node_obj OBJECT node/track_node.cpp node/select_node.cpp)
configure_node_target(track_node_obj)
set_property(TARGET track_node_obj PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(track_node_obj track_core)
install(FILES third_party/ocsort/LICENSE DESTINATION share/${PROJECT_NAME}/licenses/ocsort)
add_executable(tracking_runtime app/main.cpp)
configure_node_target(tracking_runtime)
target_link_libraries(tracking_runtime track_node_obj)
install(TARGETS tracking_runtime DESTINATION lib/${PROJECT_NAME})
add_library(tracking_components SHARED)
configure_node_target(tracking_components)
target_link_libraries(tracking_components track_node_obj)
rclcpp_components_register_nodes(tracking_components "TrackNode" "SelectNode")
install(TARGETS tracking_components ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
ament_package()
