# ROS Image ownership/stride/encoding boundaries; no node, GPU or camera required.
add_executable(test_image_view test_image_view.cpp)
target_include_directories(test_image_view PRIVATE ../host ../.. ${OpenCV_INCLUDE_DIRS})
target_compile_features(test_image_view PRIVATE cxx_std_17)
ament_target_dependencies(test_image_view sensor_msgs std_msgs)
target_link_libraries(test_image_view ${OpenCV_LIBS})
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(test_image_view PRIVATE -Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake_test REQUIRED)
ament_add_test(test_image_view COMMAND $<TARGET_FILE:test_image_view>
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO TIMEOUT 30)
