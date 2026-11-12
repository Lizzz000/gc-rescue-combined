cmake_minimum_required(VERSION 3.16)
project(teleop_pkg)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# 独立手动控制程序：入口 + 链路/指令模块 + 网页上位机，按源码目录原样安装。
install(PROGRAMS manual_control.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY manual web DESTINATION lib/${PROJECT_NAME} PATTERN "__pycache__" EXCLUDE)
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

# 唯一 S288 协议安装为可导入的 Python 包：source 工作区 setup.bash 后可 import servo.s288。
# 与 ament_python_install_package 使用同一 PYTHON_INSTALL_DIR 和 PYTHONPATH hook；
# 不生成 egg-info（import 只依赖包目录和 hook）。入口目录再留一份，供已安装的
# manual_control.py 与同目录的 manual/、web/ 一样直接运行。
_ament_cmake_python_register_environment_hook()
install(DIRECTORY servo/
  DESTINATION ${PYTHON_INSTALL_DIR}/servo
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
install(DIRECTORY servo DESTINATION lib/${PROJECT_NAME} PATTERN "__pycache__" EXCLUDE)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(manual_command test/test_manual_command.py TIMEOUT 60)
  ament_add_pytest_test(manual_control test/test_manual_control.py TIMEOUT 120)
  # 链路验收：周期、租约、SIGKILL→STM32 制动回放（需主机 gcc 与 STM32 源码，缺失时跳过）、HTTP 错误输入
  ament_add_pytest_test(manual_link test/test_manual_link.py TIMEOUT 120)
  ament_add_pytest_test(s288 test/test_s288.py TIMEOUT 60)
endif()

ament_package()
