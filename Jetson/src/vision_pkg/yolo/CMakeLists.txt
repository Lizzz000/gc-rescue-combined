# Local integration sources retain their original AGPL-3.0 notices.
# 编出 vision_yolo_headers（纯头文件接口）、vision_yolo（CUDA/TensorRT，含固定版本的 YOLOs-CPP-TensorRT 归档、校验和与应用层补丁）和 vision_engine_build。
# 要 CUDA 或 TensorRT 的源进 vision_yolo。
# OpenCV、sensor_msgs、std_msgs、vision_msgs 与 Threads 已由包入口找到，这里不再查找第二遍。

# 头文件接口（DTO / Letterbox / MaskWorkspace），节点依赖它。
add_library(vision_yolo_headers INTERFACE)
target_include_directories(vision_yolo_headers INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/.. ${OpenCV_INCLUDE_DIRS})
target_link_libraries(vision_yolo_headers INTERFACE ${OpenCV_LIBS})

# 本集成只接受 TensorRT 10.x，主版本不符直接失败。
# 在启用 CUDA 前确定目标架构；87 是 Orin 默认值，其他设备须显式指定实际 GPU 架构。
if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
  set(CMAKE_CUDA_ARCHITECTURES 87 CACHE STRING "Target CUDA compute architectures")
endif()
enable_language(CUDA)
find_package(CUDAToolkit REQUIRED)

find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  HINTS ${TENSORRT_DIR}/include $ENV{TENSORRT_DIR}/include
        /usr/include/aarch64-linux-gnu /usr/include)
find_library(NVINFER_LIB nvinfer
  HINTS ${TENSORRT_DIR}/lib $ENV{TENSORRT_DIR}/lib
        /usr/lib/aarch64-linux-gnu)
find_library(NVONNXPARSER_LIB nvonnxparser
  HINTS ${TENSORRT_DIR}/lib $ENV{TENSORRT_DIR}/lib
        /usr/lib/aarch64-linux-gnu)
if(NOT TENSORRT_INCLUDE_DIR OR NOT NVINFER_LIB OR NOT NVONNXPARSER_LIB)
  message(FATAL_ERROR "TensorRT headers, nvinfer and nvonnxparser are required. Set TENSORRT_DIR.")
endif()
file(READ "${TENSORRT_INCLUDE_DIR}/NvInferVersion.h" _trt_version)
if(_trt_version MATCHES "define[ \t]+NV_TENSORRT_MAJOR[ \t]+([0-9]+)")
  set(_trt_major "${CMAKE_MATCH_1}")
endif()
if(NOT _trt_major STREQUAL "10")
  message(FATAL_ERROR "This integration targets TensorRT 10.x; found '${_trt_major}'.")
endif()

# 固定归档及校验和，把下载与展开限制在构建目录，避免依赖可变分支。
# 这里准备的是推理库源码依赖；编译此目标不等于构建模型引擎，生产启动仍只读取既有 TRT。
set(_vendor "${CMAKE_CURRENT_BINARY_DIR}/vendor")
set(_archive "${_vendor}/yolos-v3.1.0.tar.gz")
set(_sha256 "0549b123bed0ecdb cda0247abf6da9636f2d6e18df059f95e538b77dfd2bd530")
string(REPLACE " " "" _sha256 "${_sha256}")
file(MAKE_DIRECTORY "${_vendor}")
# 离线构建：板端不能联网时，用 -DYOLOS_CPP_TRT_ARCHIVE=/path/to/yolos-v3.1.0.tar.gz（GitHub tag v3.1.0 的 tar.gz）指定；照样校验 SHA256，内容不符直接失败。
set(YOLOS_CPP_TRT_ARCHIVE "" CACHE FILEPATH
  "Local YOLOs-CPP-TensorRT v3.1.0 tar.gz for offline builds (SHA256 is still verified)")
set(_local_archive "${YOLOS_CPP_TRT_ARCHIVE}")
if(_local_archive AND NOT EXISTS "${_archive}")
  if(NOT EXISTS "${_local_archive}")
    message(FATAL_ERROR "YOLOS_CPP_TRT_ARCHIVE does not exist: ${_local_archive}")
  endif()
  message(STATUS "Using local YOLOs-CPP-TensorRT archive: ${_local_archive}")
  configure_file("${_local_archive}" "${_archive}" COPYONLY)
endif()
if(NOT EXISTS "${_archive}")
  message(STATUS "Downloading YOLOs-CPP-TensorRT v3.1.0 (first TensorRT build only; "
    "offline: -DYOLOS_CPP_TRT_ARCHIVE=/path/to/yolos-v3.1.0.tar.gz)")
  file(DOWNLOAD
    "https://codeload.github.com/Geekgineer/YOLOs-CPP-TensorRT/tar.gz/refs/tags/v3.1.0"
    "${_archive}" EXPECTED_HASH "SHA256=${_sha256}" TLS_VERIFY ON
    SHOW_PROGRESS)
endif()
file(SHA256 "${_archive}" _actual_sha256)
if(NOT _actual_sha256 STREQUAL _sha256)
  # 删除坏归档，下次配置可重新下载／重新复制，而不是一直卡在同一个坏文件上。
  file(REMOVE "${_archive}")
  message(FATAL_ERROR "YOLOs-CPP-TensorRT archive checksum mismatch (expected ${_sha256}, "
    "got ${_actual_sha256}); removed ${_archive}")
endif()
set(_upstream "${_vendor}/YOLOs-CPP-TensorRT-3.1.0")
if(NOT EXISTS "${_upstream}/include/yolos/yolos.hpp")
  file(ARCHIVE_EXTRACT INPUT "${_archive}" DESTINATION "${_vendor}")
endif()


# 默认保留上游全输出回传；本地两种布局无有效掩膜需求时只回传检测输出。
# 补丁仅作用于构建目录内固定版本的头文件，不改变归档或引擎。
include("${CMAKE_CURRENT_SOURCE_DIR}/patch_host_outputs.cmake")

# H13：推理只接受 NVMM 帧，经 EGLImage 映射给 CUDA，融合核直接写 TRT 输入。头文件来自 jetson_multimedia_api。
# vision_nvmm 同时给 camera（CPU 映射 NvBufSurface）和 yolo（CUDA 映射）使用。
find_path(NVBUFSURFACE_INCLUDE_DIR nvbufsurface.h
  HINTS /usr/src/jetson_multimedia_api/include /opt/nvidia/deepstream/deepstream/sources/includes)
find_library(NVBUFSURFACE_LIB nvbufsurface
  HINTS /usr/lib/aarch64-linux-gnu/nvidia /usr/lib/aarch64-linux-gnu/tegra)
find_library(EGL_LIB EGL)
if(NOT NVBUFSURFACE_INCLUDE_DIR OR NOT NVBUFSURFACE_LIB OR NOT EGL_LIB)
  message(FATAL_ERROR "TensorRT build needs nvbufsurface.h (jetson_multimedia_api), libnvbufsurface and libEGL")
endif()
add_library(vision_nvmm INTERFACE)
target_compile_definitions(vision_nvmm INTERFACE VISION_ENABLE_NVMM=1)
target_include_directories(vision_nvmm SYSTEM INTERFACE ${NVBUFSURFACE_INCLUDE_DIR})
target_link_libraries(vision_nvmm INTERFACE ${NVBUFSURFACE_LIB} ${EGL_LIB} CUDA::cuda_driver)

# 需要 CUDA 或 TensorRT 的源文件进 vision_yolo（推理侧 NVMM 前处理在 cuda/）。
add_library(vision_yolo STATIC
  cuda/nvmm_preprocess.cu yolo.cpp tensorrt/tensorrt.cpp engine/engine.cpp
  engine/engine_cache.cpp)
target_compile_features(vision_yolo PUBLIC cxx_std_17)
set_target_properties(vision_yolo PROPERTIES
  CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED ON POSITION_INDEPENDENT_CODE ON)
target_include_directories(vision_yolo SYSTEM PUBLIC
  "${_upstream}/include" ${TENSORRT_INCLUDE_DIR})
ament_target_dependencies(vision_yolo sensor_msgs std_msgs vision_msgs)
target_link_libraries(vision_yolo vision_yolo_headers vision_nvmm
  ${NVINFER_LIB} ${NVONNXPARSER_LIB} CUDA::cudart ${OpenCV_LIBS} Threads::Threads)

# 离线构建入口：只编 main，其余直接链接 vision_yolo，不重复编译源文件。
add_executable(vision_engine_build engine/engine_build_main.cpp)
target_compile_features(vision_engine_build PRIVATE cxx_std_17)
target_link_libraries(vision_engine_build vision_yolo)

install(FILES "${_upstream}/LICENSE" DESTINATION share/${PROJECT_NAME}/licenses
  RENAME YOLOs-CPP-TensorRT-LICENSE)
