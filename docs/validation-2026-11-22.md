# 2026-11-22 自动搬运离线验收

本轮验证使用正式 STM32 自动搬运状态机和场地配置。Jetson 无 ROI 投送由真实 ROS 节点和两路私有伪终端复现。两板当前未连接，最后一版固件还需烧录并完成带货实车验收。

## 核验结果

| 范围 | 结果 |
| --- | --- |
| STM32 主机回归 | 38 套 C 测试、12 个独立模块、17 项控制模式检查通过。独立 Python 用例 124 项通过，其中 8 项 ADRC 用例也由主机入口执行。 |
| 双端编解码与夹爪期限 | 162 个下行、38 个上行用例通过。S288 最长预算 11.870 s，小于 STM32 等待上限 12 s。 |
| STM32 连续自动搬运 | 正式状态机在同一实例连续完成左、右、左三趟。覆盖起点对准、180° 搜索、夹取、返航、横移、盲推、松爪、直退、反平移、整区双帧和重新建点。 |
| 返航里程计模型 | 五组输入分别跑理想反馈和假设轮滑、传感器误差，共 10/10 达到模型验收范围（2 cm、5°）。模型终点误差最大 5.14 mm、1.85°，实际车轮打滑和 IMU 偏差须实测。 |
| ARM 交叉编译 | GCC 14.3.1，7 个配置均通过。`Debug-Auto/project.bin` 大小 125,552 字节，SHA-256 为 `6ef6cd62f7211f3a9292bcec65e5571651b9221256fcc6a705679e1deb29427a`。 |
| Jetson 软件回归 | 在 x86_64 WSL / ROS 2 Jazzy 编译六个无 GPU 依赖的包。`colcon test-result` 汇总 140 项通过，含 CTest 聚合项。另直接运行 149 项 Python 检查，全部通过。 |
| Jetson 串口仿真 | 实际 Mission、Safety、Track、Select、Target、Bridge 角色完成首绿左投、后续橙右投。每趟均恢复整区与搜索，记录 0 次 ROI 检查。 |
| Jetson OpenCV 4.6.0 主机 | 分割 CPU 后处理 20,794 条检查通过。安全区 9 个套件中 5 通过、3 失败、1 跳过。矩阵、高光/黑带和几何专项合计 7 条既有失败。缺少标注的实拍回放跳过。 |

STM32 故障分支覆盖视觉丢帧与重复帧、旧编号和错侧回执、USB 断流、CAN 与里程计反馈异常。盲推期间视觉丢失仍按原时序执行，链路和底盘故障会停车。手动模式回放中，杀掉发送进程后正式 STM32 C 控制器在 247.9 ms 进入制动，回放阈值为 255 ms。Jetson 伪终端验证还覆盖首次对准时整区中心的丢失与恢复、夹取前拒绝路由、夹后丢目标仍用锁存类别、重复路由和错误编号拒绝。

视觉压力失败集中在高光压线、远视角、低对比、两厘米黑块贴线、灰度分界或运动模糊、反光与阴影叠加。本轮沿用现有识别器和模型。现场先确认整块安全区进画，夹取后记录实际放货，并计量从直退停稳到新起点的时间。 [视觉基线](../Jetson/src/vision_pkg/test/field/README.md#2026-11-17-验证结果)

## 复现

### STM32

从仓库根目录运行主机入口。`--python` 同时运行 ADRC、协议和 S288 等待预算检查。

```sh
python STM32/tests/run_native.py --python
```

里程计模型使用 C 里程计和简化返航控制器。用同一参数的五个种子复现理想与扰动结果。

```sh
out="$(mktemp -d)"
for seed in 20261117 20261118 20261119 20261120 20261121; do
  python STM32/manual_test/live_tuning/simulate_return.py --seed "$seed" --output "$out/$seed" || exit 1
done
```

在已配置 ARM GNU 工具链的终端运行全部构建预设。

```sh
cd STM32/project
for preset in Debug Debug-9axis Debug-Auto Debug-Auto-Chassis-Test Debug-UART6-Manual Debug-Tuning Release; do
  cmake --preset "$preset" && cmake --build --preset "$preset" || exit 1
done
```

### Jetson

在加载 ROS 的 Linux 主机中从仓库根目录运行。以下入口使用伪造的检测与场地消息、STM32 USB 字节流和 S288 反馈。STM32 实际动作时序由前述 C 主机回归覆盖。复现时选用空闲的本地 DDS 域。

```bash
source /opt/ros/jazzy/setup.bash
out="$(mktemp -d)"
colcon --log-base "$out/log" build --base-paths Jetson/src \
  --packages-select interface_pkg track_pkg mission_pkg bridge_pkg bringup_pkg teleop_pkg \
  --build-base "$out/build" --install-base "$out/install" --cmake-args -DBUILD_TESTING=ON
source "$out/install/setup.bash"
ROS_DOMAIN_ID=175 python3 Jetson/tools/validation/direct_transport_pty.py \
  --install "$out/install" --out "$out/direct-transport"
```

仿真目录保存 `result.json`、各节点日志、USB 报文和夹爪反馈。OpenCV 主机和工具测试入口见 [Jetson 分层验收](../Jetson/docs/README.md#分层验收)。板上部署仍使用 ROS 2 Humble、现场相机、TensorRT 引擎和实际 USB 设备。

## 下次实车

用同一版本构建两端，烧录 `Debug-Auto`，使用 `field_20261122.yaml` 和 `run:=all`。单绿按标定位置摆放，车头起步时面向完整可见的安全区。Jetson 的 V4L2 实际回读、USB 和热点稳定、实际夹持与放货、带货返航及回撤后重新记录起点，均纳入 [现场验收步骤](../STM32/docs/JETSON_USB_TEST.md#下一次现场单绿验收)。
