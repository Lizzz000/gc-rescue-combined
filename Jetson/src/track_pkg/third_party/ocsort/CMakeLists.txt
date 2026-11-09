find_package(Eigen3 REQUIRED)
add_library(ocsort_core STATIC
  src/OCSort.cpp src/Association.cpp src/KalmanBoxTracker.cpp
  src/KalmanFilter.cpp src/Utilities.cpp src/lapjv.cpp)
target_include_directories(ocsort_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(ocsort_core PUBLIC Eigen3::Eigen)
target_compile_features(ocsort_core PUBLIC cxx_std_17)
set_target_properties(ocsort_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
