# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
# v3：每个功能目录一个库（粒度偏粗），技术子目录（gstreamer/neon/opencv/jpeg/mjpeg）编进所属功能库。
# 推理库 vision_yolo / vision_yolo_headers 在 yolo/CMakeLists.txt 中定义。

function(vision_static_library target)
  add_library(${target} STATIC ${ARGN})
  set_property(TARGET ${target} PROPERTY POSITION_INDEPENDENT_CODE ON)
endfunction()

vision_static_library(vision_camera
  camera/camera.cpp
  camera/gstreamer/gstreamer.cpp
  camera/neon/i420.cpp)
vision_core_target(vision_camera)
target_link_libraries(vision_camera PUBLIC ${OpenCV_LIBS} Threads::Threads
  PRIVATE PkgConfig::GSTREAMER_APP PkgConfig::GSTREAMER_VIDEO)
target_link_libraries(vision_camera PUBLIC vision_nvmm)

vision_static_library(vision_field
  field/field.cpp
  field/color.cpp
  field/debug.cpp
  field/opencv/morphology.cpp
  field/opencv/geometry.cpp
  field/opencv/score.cpp)
vision_core_target(vision_field)
target_link_libraries(vision_field PUBLIC ${OpenCV_LIBS})

vision_static_library(vision_stream
  stream/opencv/overlay.cpp
  stream/jpeg/jpeg.cpp
  stream/mjpeg/mjpeg.cpp
  stream/mjpeg/page.cpp
  stream/mjpeg/send.cpp)
vision_ros_target(vision_stream)
target_compile_definitions(vision_stream PRIVATE RESCUE_HAS_NVJPEG=$<BOOL:${VISION_ENABLE_NVJPEG}>)
target_link_libraries(vision_stream PRIVATE PkgConfig::GSTREAMER_APP PkgConfig::GSTREAMER_VIDEO ${CMAKE_DL_LIBS})

add_library(vision_nodes OBJECT
  node/detect_node.cpp
  node/detector.cpp
  node/detect_workers.cpp
  node/stream_node.cpp)
vision_ros_target(vision_nodes)
set_property(TARGET vision_nodes PROPERTY POSITION_INDEPENDENT_CODE ON)
target_compile_definitions(vision_nodes PUBLIC VISION_ENABLE_TENSORRT=1)
target_link_libraries(vision_nodes PUBLIC vision_camera vision_field vision_stream vision_yolo_headers vision_yolo)
