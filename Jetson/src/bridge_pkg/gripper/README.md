# S288 夹爪调试

该工具供单个宇树 S288 舵机通过 USB 转单总线模块进行独立调试；它不接入 ROS、CAN 桥接或自动搬运状态机。协议由本目录 `s288_protocol.py` 薄转发到 `teleop_pkg/servo/s288.py`，此处不保留第二份实现。STM32 原夹爪动作尚未迁移，不能用此工具代替两端握手。

舵机必须由与 S288 匹配的独立电源供电。连接 USB 模块后，在 Jetson 上确认 `/dev/serial/by-id/` 中的设备路径，并确保其他程序没有占用此串口。程序使用 Python 3 和 `pyserial`；Jetson 上可通过系统包 `python3-serial` 安装。模块和 Linux 驱动必须支持 6 Mbps，能否工作需要板端实测。

协议从已安装的 `servo` 包导入。先编译并 source 工作区，再运行调试工具或本目录测试：

```bash
# 在 $REPO/Jetson（ROS 2 Jazzy 工作区）
source /opt/ros/jazzy/setup.bash   # 或本机等效的 RoboStack 环境
colcon build --packages-select teleop_pkg
source install/setup.bash
python3 src/bridge_pkg/gripper/s288_debug.py --device /dev/serial/by-id/实际设备名 --id 0
```

未 source 工作区时，`import servo.s288` 会失败。

串口设备名、ID、开合位置和力度均需按实物确认。启动时会发送停机命令；在交互提示符输入：

```text
status           发送停机命令并显示反馈
move 30 5        从当前反馈位置正向转动 30°，发送目标 5 秒后停机
move -30 5       从当前位置反向转动 30°
gain 1 0.05    设置输出端位置刚度 Kp 和速度刚度 Kd
stop             发送停机命令并显示反馈
quit             退出
```

也可以通过 `--kp` 和 `--kd` 设置初始增益，默认分别为 1 和 0.05。角度为舵机输出端的相对角度，正负方向与夹爪的开合关系需要实测。运动中按 Ctrl+C 请求停机；程序显示的实际转角来自反馈，未到目标时不会将目标当成实际值。到时停机并不等于夹稳物块，也不意味着维持夹紧力。通信异常时可能无法确认停机，应检查舵机和机构的实际状态。

source 工作区后，在此目录运行 `python3 -m unittest -v test_s288_protocol test_s288_debug` 可检查协议和模拟串口逻辑；这不能替代 Jetson、转换器及夹爪实测。colcon test 会跑到 `test_s288_forward`。
