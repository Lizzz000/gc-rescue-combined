# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.18)
project(bringup_pkg)
find_package(ament_cmake REQUIRED)
# 基础与高级配置字段互补，由唯一装配器读取；不安装旧预设或各包覆盖层。
install(FILES config/rescue.yaml config/rescue_advanced.yaml DESTINATION share/${PROJECT_NAME}/config)
install(FILES launch/rescue.launch.py DESTINATION share/${PROJECT_NAME}/launch)
# runtime 包按目录整体安装。
install(DIRECTORY runtime DESTINATION share/${PROJECT_NAME} PATTERN "__pycache__" EXCLUDE)
ament_package()
