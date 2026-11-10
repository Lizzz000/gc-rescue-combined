# 安全区测试（只依赖 OpenCV）。两种用法：
#   1) 板端/colcon：vision_pkg 在 BUILD_TESTING 下 add_subdirectory(test/field)，复用 vision_field 目标，
#      用 ament_add_test 登记，colcon test 运行；
#   2) PC 独立：cmake -S test/field -B build && cmake --build build && ctest --test-dir build，
#      此时用同一组源文件自建 vision_field。也可直接用 run.sh / run.ps1。
cmake_minimum_required(VERSION 3.18)
if(NOT DEFINED PROJECT_NAME)
  project(vision_field_tests LANGUAGES CXX)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
  endif()
  enable_testing()
endif()

set(FIELD_PKG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set(FIELD_REPO_SRC ${FIELD_PKG_DIR}/..)
set(FIELD_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

if(NOT TARGET vision_field)
  add_library(vision_field STATIC
    ${FIELD_PKG_DIR}/field/field.cpp
    ${FIELD_PKG_DIR}/field/color.cpp
    ${FIELD_PKG_DIR}/field/debug.cpp
    ${FIELD_PKG_DIR}/field/opencv/morphology.cpp
    ${FIELD_PKG_DIR}/field/opencv/geometry.cpp
    ${FIELD_PKG_DIR}/field/opencv/score.cpp)
  target_include_directories(vision_field PRIVATE ${FIELD_PKG_DIR} ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(vision_field PUBLIC ${OpenCV_LIBS})
endif()

# 测试与算法读同一份 yaml：构建时生成参数头
add_custom_command(
  OUTPUT ${FIELD_GEN_DIR}/field_config_values.inc
  COMMAND ${CMAKE_COMMAND} -E make_directory ${FIELD_GEN_DIR}
  COMMAND Python3::Interpreter ${CMAKE_CURRENT_SOURCE_DIR}/field_config.py ${FIELD_GEN_DIR}/field_config_values.inc
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/field_config.py
    ${FIELD_REPO_SRC}/bringup_pkg/config/rescue.yaml
    ${FIELD_REPO_SRC}/bringup_pkg/config/rescue_advanced.yaml
    ${FIELD_REPO_SRC}/bringup_pkg/runtime/schema.json
  COMMENT "生成安全区测试参数头")
add_custom_target(field_test_config DEPENDS ${FIELD_GEN_DIR}/field_config_values.inc)

# 超时：矩阵约 2–3 分钟（x86），板端留余量
set(FIELD_TEST_TIMEOUT 1200)

function(field_test name)
  add_executable(${name} ${name}.cpp)
  add_dependencies(${name} field_test_config)
  target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../host
    ${FIELD_PKG_DIR} ${FIELD_GEN_DIR} ${OpenCV_INCLUDE_DIRS})
  target_compile_definitions(${name} PRIVATE FIELD_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
  target_link_libraries(${name} PRIVATE vision_field ${OpenCV_LIBS})
  if(COMMAND ament_add_test)
    ament_add_test(${name} COMMAND $<TARGET_FILE:${name}> GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      SKIP_RETURN_CODE 77 TIMEOUT ${FIELD_TEST_TIMEOUT})
  else()
    add_test(NAME ${name} COMMAND ${name})
    set_tests_properties(${name} PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT ${FIELD_TEST_TIMEOUT})
  endif()
endfunction()

field_test(test_field_matrix)
field_test(test_field_sequence)
field_test(test_field_replay)
