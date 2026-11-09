cmake_minimum_required(VERSION 3.18)
if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
  set(CMAKE_CUDA_ARCHITECTURES 87)
endif()
project(yolo_transfer_probe LANGUAGES CXX CUDA)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
find_package(OpenCV REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(Threads REQUIRED)
find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  HINTS /usr/include/aarch64-linux-gnu /usr/include)
find_library(NVINFER_LIB nvinfer HINTS /usr/lib/aarch64-linux-gnu)
add_executable(yolo_transfer_probe
  probe.cpp yolo/yolo.cpp upstream/include/yolos/core/cuda_preprocessing.cu)
add_executable(yolo_output_capture
  capture.cpp yolo/yolo.cpp upstream/include/yolos/core/cuda_preprocessing.cu)
foreach(_target yolo_transfer_probe yolo_output_capture)
  target_include_directories(${_target} PRIVATE
    . upstream/include ${TENSORRT_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(${_target} PRIVATE
    ${NVINFER_LIB} CUDA::cudart ${OpenCV_LIBS} Threads::Threads)
  target_compile_options(${_target} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wall;-Wextra>)
endforeach()
