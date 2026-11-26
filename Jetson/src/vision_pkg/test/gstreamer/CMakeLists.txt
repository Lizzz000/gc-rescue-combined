# Linux host regression using GStreamer base plugins and OpenCV; no camera, ROS or GPU needed.
cmake_minimum_required(VERSION 3.18)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(vision_capture_tests LANGUAGES CXX)
  enable_testing()
  find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio)
  find_package(Threads REQUIRED)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(GSTREAMER_APP REQUIRED IMPORTED_TARGET gstreamer-app-1.0)
  pkg_check_modules(GSTREAMER_VIDEO REQUIRED IMPORTED_TARGET gstreamer-video-1.0)
endif()

add_executable(test_capture_startup test_capture_startup.cpp)
target_compile_features(test_capture_startup PRIVATE cxx_std_17)
target_include_directories(test_capture_startup PRIVATE ../.. ${OpenCV_INCLUDE_DIRS})
target_link_libraries(test_capture_startup PRIVATE
  PkgConfig::GSTREAMER_APP PkgConfig::GSTREAMER_VIDEO Threads::Threads ${OpenCV_LIBS})
target_link_options(test_capture_startup PRIVATE "-Wl,--wrap=ioctl")
if(TARGET vision_camera)
  find_package(ament_cmake_test REQUIRED)
  target_link_libraries(test_capture_startup PRIVATE vision_camera)
else()
  target_sources(test_capture_startup PRIVATE
    ../../camera/gstreamer/gstreamer.cpp ../../camera/neon/i420.cpp ../../camera/v4l2_controls.cpp)
  target_compile_definitions(test_capture_startup PRIVATE VISION_ENABLE_NVMM=0)
endif()

if(COMMAND ament_add_test)
  ament_add_test(test_capture_startup COMMAND $<TARGET_FILE:test_capture_startup>
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO TIMEOUT 15)
else()
  add_test(NAME test_capture_startup COMMAND test_capture_startup)
  set_tests_properties(test_capture_startup PROPERTIES TIMEOUT 15)
endif()
