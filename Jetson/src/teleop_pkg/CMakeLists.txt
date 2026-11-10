cmake_minimum_required(VERSION 3.16)
project(teleop_pkg)

find_package(ament_cmake REQUIRED)

# 独立手动控制程序：入口 + 链路/指令模块 + 网页上位机，按源码目录原样安装。
install(PROGRAMS manual_control.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY manual web DESTINATION lib/${PROJECT_NAME} PATTERN "__pycache__" EXCLUDE)
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(manual_command test/test_manual_command.py TIMEOUT 60)
  ament_add_pytest_test(manual_control test/test_manual_control.py TIMEOUT 120)
  # 链路验收：周期、租约、SIGKILL→STM32 制动回放（需主机 gcc 与 STM32 源码，缺失时跳过）、HTTP 错误输入
  ament_add_pytest_test(manual_link test/test_manual_link.py TIMEOUT 120)
endif()

ament_package()
