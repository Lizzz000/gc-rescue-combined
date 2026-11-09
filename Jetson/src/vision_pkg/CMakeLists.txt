# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.18)
project(vision_pkg LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()
# 板端始终构建真实 TensorRT 分割后端（无 OFF 开关）。
option(VISION_ENABLE_NVJPEG "Build the explicit Jetson JPEG encoding backend" ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(interface_pkg REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs videoio)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
# 相机直接使用原生 GStreamer 的采集/取消接口。
pkg_check_modules(GSTREAMER_APP REQUIRED IMPORTED_TARGET gstreamer-app-1.0)
pkg_check_modules(GSTREAMER_VIDEO REQUIRED IMPORTED_TARGET gstreamer-video-1.0)
add_subdirectory(yolo)

function(vision_core_target target)
  target_include_directories(${target} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
endfunction()
function(vision_ros_target target)
  vision_core_target(${target})
  ament_target_dependencies(${target} PUBLIC rclcpp rclcpp_components rcl_interfaces
    sensor_msgs std_msgs std_srvs vision_msgs cv_bridge interface_pkg)
  target_link_libraries(${target} PUBLIC ${OpenCV_LIBS} Threads::Threads)
endfunction()

include(cmake/libraries.cmake)

# 组件登记在节点源文件里，main 在 app/；可执行文件与插件库共用 vision_nodes 目标文件，组件库不含 main。
add_executable(vision_runtime app/vision_runtime.cpp)
vision_ros_target(vision_runtime)
target_link_libraries(vision_runtime PRIVATE vision_nodes)
add_library(vision_components SHARED)
vision_ros_target(vision_components)
target_link_libraries(vision_components PRIVATE vision_nodes)
rclcpp_components_register_nodes(vision_components "DetectNode" "StreamNode")
install(TARGETS vision_runtime DESTINATION lib/${PROJECT_NAME})
install(TARGETS vision_components LIBRARY DESTINATION lib)
install(FILES camera/neon/LICENSE-OpenCV.txt DESTINATION share/${PROJECT_NAME}/licenses)
# 模型目录仅提供类别/元数据及用户已有本机资产；不安装旧配置和旧 launch。
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/view_stream.sh DESTINATION share/${PROJECT_NAME}/scripts)

ament_package()
