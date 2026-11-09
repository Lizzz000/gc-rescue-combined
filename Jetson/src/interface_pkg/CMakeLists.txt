# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
# interface_pkg 的构建脚本——生成 msg/srv 接口，并导出 app/core.hpp。
# 新增消息/服务 → 加进下方 rosidl_generate_interfaces 列表并重建本包与所有消费者
# 构建类型 → 下方的 CMAKE_BUILD_TYPE 段（默认 RelWithDebInfo，板端生产构建用 --cmake-args -DCMAKE_BUILD_TYPE=Release 覆盖）

cmake_minimum_required(VERSION 3.8)
project(interface_pkg)

# 构建类型：开发默认 RelWithDebInfo（保留优化且带调试符号）；板端生产构建用
# colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release 覆盖本默认值。
# 显式传入 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON，在本包构建目录生成 compile_commands.json。
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
    "Build type: RelWithDebInfo for development, Release for the board build" FORCE)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

# 声明构建依赖，缺失时在配置阶段直接报错。
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 新增消息/服务：在下方列表按 "msg/xxx.msg" 或 "srv/xxx.srv" 追加文件名；改完必须重建 interface_pkg 与所有消费者
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FieldObservation.msg"
  "msg/TaskState.msg"
  "msg/InferenceTiming.msg"
  "msg/DetectorStatus.msg"
  "msg/StreamStatus.msg"
  "msg/RoiCheck.msg"
  "msg/MissionStatus.msg"
  "msg/SafetyTarget.msg"
  "msg/BridgeStatus.msg"
  "srv/CheckRoi.srv"
  "srv/MissionCommand.srv"
  # 新增外部消息依赖：在 find_package 与 DEPENDENCIES 两处同时登记
  DEPENDENCIES std_msgs geometry_msgs vision_msgs
)
ament_export_dependencies(rosidl_default_runtime)
# 新增跨包复用的头文件：放入 app/ 目录即可随本行安装导出
install(DIRECTORY app/ DESTINATION include/${PROJECT_NAME}/${PROJECT_NAME}/app)
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(vision_msgs rclcpp rcl_interfaces)

ament_package()
