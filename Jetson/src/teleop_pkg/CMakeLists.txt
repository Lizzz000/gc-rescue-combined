cmake_minimum_required(VERSION 3.16)
project(teleop_pkg)

find_package(ament_cmake REQUIRED)

install(PROGRAMS move_timed.py DESTINATION lib/${PROJECT_NAME})
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

ament_package()
