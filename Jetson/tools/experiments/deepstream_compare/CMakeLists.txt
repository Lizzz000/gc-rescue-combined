cmake_minimum_required(VERSION 3.18)
project(deepstream_compare LANGUAGES CXX CUDA)

# 独立实验，不接入生产 CMake，不下载源码，不构建或迁移模型。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_ARCHITECTURES 87)
set(DEEPSTREAM_ROOT "/opt/nvidia/deepstream/deepstream" CACHE PATH "Existing DeepStream SDK")
set(YOLOS_INCLUDE "" CACHE PATH "Existing project's patched YOLOs-CPP include directory")
if(NOT EXISTS "${YOLOS_INCLUDE}/yolos/core/cuda_preprocessing.cu")
  message(FATAL_ERROR "Set YOLOS_INCLUDE to an existing project build's patched vendor/include")
endif()
file(READ "${YOLOS_INCLUDE}/yolos/core/cuda_preprocessing.cu" _kernel)
string(FIND "${_kernel}" "(static_cast<float>(srcX) + 0.5f)" _center_x)
string(FIND "${_kernel}" "(static_cast<float>(srcY) + 0.5f)" _center_y)
if(_center_x LESS 0 OR _center_y LESS 0)
  message(FATAL_ERROR "Production pixel-center CUDA preprocessing patch is missing")
endif()
find_package(CUDAToolkit REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgcodecs imgproc)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED IMPORTED_TARGET gstreamer-1.0 gstreamer-app-1.0)
find_library(NVINFER nvinfer REQUIRED)
find_library(NVBUFSURFACE nvbufsurface PATHS "${DEEPSTREAM_ROOT}/lib" REQUIRED NO_DEFAULT_PATH)
find_library(NVDSMETA nvds_meta PATHS "${DEEPSTREAM_ROOT}/lib" REQUIRED NO_DEFAULT_PATH)
find_library(GSTNVDSMETA nvdsgst_meta PATHS "${DEEPSTREAM_ROOT}/lib" REQUIRED NO_DEFAULT_PATH)
add_library(no_engine_build SHARED no_engine_build.cpp)
target_link_libraries(no_engine_build PRIVATE dl)
add_executable(ds_compare runner.cpp "${YOLOS_INCLUDE}/yolos/core/cuda_preprocessing.cu")
target_include_directories(ds_compare PRIVATE
  "${YOLOS_INCLUDE}" "${DEEPSTREAM_ROOT}/sources/includes"
  "${DEEPSTREAM_ROOT}/sources/gst-plugins/gst-nvdspreprocess/include"
  "${OpenCV_INCLUDE_DIRS}")
target_link_libraries(ds_compare PRIVATE CUDA::cudart "${NVINFER}"
  "${NVBUFSURFACE}" "${NVDSMETA}" "${GSTNVDSMETA}" PkgConfig::GST
  "${OpenCV_LIBS}" dl pthread)
set_target_properties(ds_compare PROPERTIES BUILD_RPATH "${DEEPSTREAM_ROOT}/lib")
file(SHA256 "${YOLOS_INCLUDE}/yolos/core/cuda_preprocessing.cu" _cuda_sha)
file(SHA256 "${YOLOS_INCLUDE}/yolos/core/trt_session_base.hpp" _session_sha)
file(WRITE "${CMAKE_BINARY_DIR}/build-identity.txt"
  "cuda_preprocessing=${YOLOS_INCLUDE}/yolos/core/cuda_preprocessing.cu\ncuda_sha256=${_cuda_sha}\ntrt_session_sha256=${_session_sha}\ndeepstream=${DEEPSTREAM_ROOT}\n")
