# Jetson 板间 USB 与三轮 CAN1 联调

更新于 2026-11-23。

板间链路是 STM32 USB OTG FS 设备，Jetson 主机打开 `link.device`。引脚 PA11 D-、PA12 D+、PA10 OTG ID，全速 12 Mbit/s，没有 ULPI，不是 480 Mbit/s。bridge 与手动程序不要同时打开同一只设备。

帧布局：SOF `A5`、版本 `01`、类型 uint8、长度 uint8、载荷、CRC-16/IBM 小端。CRC 覆盖版本、类型、长度和载荷。最大载荷 8。CRC 错则丢掉一个字节再同步。空载荷不是失联。

电机仍是 CAN1 上的达妙 MIT，使用 PD0/PD1、1 Mbit/s、RX0 优先级 6。C 板 J22/J23 并联，1 脚为 CAN_L，2 脚为 CAN_H。反馈 ID 为 0，发送 ID 为 1–3。这不是板间链路。CAN2 仅保留初始化。帧类型以 [`Jetson/docs/usb-protocol.md`](../../Jetson/docs/usb-protocol.md) 为准。

## 当前接入范围

CAN1 只收发达妙电机。反馈要求 DLC=8，帧内电机 ID 为 1、2、3。板间帧不进 CAN FIFO。

`test_task` 每 5 ms 处理一次链路快照，底盘每 10 ms 更新三轮 MIT。`Debug`、`Debug-Auto`、`Debug-Auto-Chassis-Test` 和 `Debug-UART6-Manual` 分别构建 USB 手动、USB 自动搬运、无夹爪底盘联调和 USART6 手动固件。每种构建只链接本模式的控制器与协议适配。手动和正式自动固件均在上电取得有效姿态后申请使能，等待阶段保持零速。[模式定义](../project/Core/Control/Chassis/chassis_mode.h) · [自动标定参数](../project/Core/Control/Jetson/jetson_auto_config.h)

- USB 手动只接收类型 2 速度帧，使用有效零速接管。正式自动只接收视觉和搬运握手。
- 正式自动等三轮零目标、使能反馈及里程计就绪后核对场地标定。参数齐全、当前 USB 会话收到合法类型 3/4 周期流后，才用类型 21 查询开爪。收到开爪受理和成功结果后，用类型 22 请求整区识别。自动对准、停稳并用两张新源帧复核后记录起点，再转 180°，停稳后用类型 19 切回目标识别。未标定或视觉尚未上线时保持零速。单趟底盘诊断另有显式启动开关和完成失能流程。

| 类型 | 长度 | 方向 | 载荷 |
|---:|---:|---|---|
| 2 | 7 | Jetson → STM32 | `valid`、`vx`、`vy`、`wz`。自动固件忽略，手动控制独立执行速度有效期 |
| 3 / 4 | 8 | Jetson → STM32 | `valid`、`x`、`y`、源时间低 24 位。目标中心、安全区透视中心 |
| 5 | 1 | Jetson → STM32 | `accepted`，0 或 1 |
| 6 | 3 | Jetson → STM32 | `valid`、`passed`、`destination`。只接受 000、100、111、112 |
| 7 | 6 | Jetson → STM32 | `accepted`、命令 1–6、token。3 回显搜寻编号，5 回显夹爪编号且只表示受理，6 回显首次整区模式请求编号 |
| 8 | 6 | Jetson → STM32 | `data0=1`、目的区 1 或 2、token |
| 9 | 6 | Jetson → STM32 | `kind`、`success`、`request_id:uint32`：独立夹爪终态 |
| 16 | 1 | STM32 → Jetson | `data0=2`，按已夹取类别登记目的区。`data0=1` 用于独立底盘诊断 |
| 17 / 18 / 20 | 5 | STM32 → Jetson | `data0=1` 加 token。转向完成、安全区到达、投送 |
| 19 | 5 | STM32 → Jetson | `data0=1`、非零 `request_id:uint32`，开始搜索 |
| 21 | 5 或 8 | STM32 → Jetson | `kind`、非零 `request_id:uint32`。1 夹取附最终入窗的 `source_frame:uint24`，其余 0 启动查询、2 释放、3 拒绝后张开使用 5 字节 |
| 22 | 5 | STM32 → Jetson | `data0=1`、非零 `request_id:uint32`，请求首次整区对准，命令 6 回显编号 |

类型 21 使用独立非零请求编号。类型 7 命令 5 回显编号并表示受理；类型 9（长度 6：kind、success、request_id）报告动作终态。帧表和期限以 [`Jetson/docs/usb-protocol.md`](../../Jetson/docs/usb-protocol.md) 为准，两端必须同步更新。

视觉接入复用类型 3/4。每张新图使用采集时刻的低 24 位毫秒，同一张图重发时保留原时间。板端以本机接收时间判断 250 ms 时效，并按源帧先后顺序接纳测量，支持源时钟回绕。重复或乱序旧图不会形成新到位证据。无目标时持续发布合法的无效中心。Mission 每 50 ms 发布状态；bridge 超过 500 ms 未收到状态即停发中心帧，底盘随后按通信断流处理。[观测接口](../project/Core/Communication/Common/visual_observation.h)

所有多字节字段小端。协议 `vx/vy` 是车体前/左方向的线速度字段，单位 m/s，原始值除以 1000；`wz` 是逆时针为正的车体角速度，单位 rad/s，也除以 1000。中心坐标除以 10000 得图像归一化位置，既不是距离也不是轮速。接收不裁剪中心坐标。路由 token=0 合法，夹爪 request_id 必须非零。

上行只走 USB。三个电机保持 MIT 模式，反馈 ID 0，发送 ID 1–3。

### Jetson 热烧录后无法枚举 STM32

2026-11-23 现场反复出现 `device descriptor read/64` 和 `-110/-71`。在 Jetson 上手动复位专用 Realtek USB 2.0 集线器后，STM32 和夹爪立即恢复为 `ttyACM0`、`ttyACM1`。操作前先停稳车辆，停止 ROS launch、bridge 和夹爪程序。核对实物接线与下方命令的输出。目标集线器只接尚未枚举的 STM32 和空闲夹爪，树上唯一已枚举的下游设备应为夹爪 `2e3c:7640`，相机及其他设备均在别处。确认 STM32 的 by-id 缺失、夹爪的 by-id 存在，且 `fuser` 没有报告占用进程或访问错误。

```bash
lsusb -t
lsusb -d 0bda:5489
lsusb -d 2e3c:7640
ls -l /dev/serial/by-id/
sudo fuser -v /dev/serial/by-id/usb-Artery_*
```

满足上述条件后，手动输入刚核实的 Realtek **Bus、Device** 两个编号（如 `001 009`）。下列命令核对已打开设备的 VID/PID，只向该集线器发送一次复位。待串口重新出现，用 by-id 确认两只设备身份，枚举编号可能变化。

```bash
read -rp 'Realtek USB2 hub Bus Device: ' bus dev
sudo python3 - "$bus" "$dev" <<'PY'
import fcntl, os, sys
from pathlib import Path
bus, dev = map(int, sys.argv[1:])
with open(f'/dev/bus/usb/{bus:03d}/{dev:03d}', 'r+b', buffering=0) as hub:
    if hub.read(18)[8:12] != bytes.fromhex('da 0b 89 54'):
        sys.exit('设备身份不符，已中止')
    number = os.fstat(hub.fileno()).st_rdev
    port = Path(f'/sys/dev/char/{os.major(number)}:{os.minor(number)}')
    children = [(p.read_text().strip(), p.with_name('idProduct').read_text().strip())
                for p in port.glob('*/idVendor')]
    if children != [('2e3c', '7640')]:
        sys.exit('目标下游设备与现场接线不符，已中止')
    fcntl.ioctl(hub, 0x5514, 0)  # Linux USBDEVFS_RESET
PY
ls -l /dev/serial/by-id/
```

## 手动速度联调

1. 上电保持车体静止。姿态就绪后固件自动申请使能，三台电机提交零目标并反馈正常后进入 `MANUAL_READY`。准备期限为 1000 ms，未接管时可持续零速待机。
2. USB 手动用新的有效全零类型 2 接管，之后的新非零命令才允许移动。USART6 使用 ARM 接管，相对原点仍须在实际停稳后建立。
3. 车体前、左平移的合速度上限为 **0.50 m/s**，转向上限为 **4.0 rad/s**，手动工具默认使用 **0.40 m/s、3.0 rad/s**。各分量乘 1000 编为小端有符号整数，斜向分量向零截断。正常操控、换向及松键时，底盘从已发速度平滑过渡到新目标，三轮持续使能。
4. 活动会话达到 250 ms 断流期限、断开、撤权或收到非法速度时，进入制动。三轮反馈超 30 ms、姿态失效和 CAN 错误同样触发保护。确认停转或制动达 500 ms 后请求失能。USB 断链、断流或硬件故障后，即使恢复持续的有效零速也保持失能。操作者排除故障后先发送类型 2 `valid=0` 显式撤权，再以新的 `valid=1` 全零帧重新接管。USART6 手动模式须使用新编号的 `0x40`（ARM）请求重新接管。

`manual_control_state` 可在调试器只读观察，枚举依次为 DISARMED=0、PREPARING=1、READY=2、RUNNING=3、BRAKING=4。它是任务处理阶段，不是电机执行回执；底盘在随后计算中触发保护时，状态显示最多滞后一个 5 ms 调度周期。底盘反馈仍查看 `dm_motor[3]`。不要通过修改状态变量绕过停车条件。

### 架空测试

先确认烧录的是用 `JETSON_AUTO_ENABLED=0` 重新编译的 `project/build/Debug/project.elf`。

发送端按 10 ms 周期逐次提交单帧类型 2。先保持有效零速约 1 s，确认 READY，再短时测试一个方向，随后持续零速直到停稳。首次在轮子离地时确认方向、停车和断流保护。

下表是类型 2 载荷，不是 CAN 文本。数值对应现有三轮几何，轮速为几何轮速，MIT 实际命令还乘现有电机方向符号。

| 动作 | 类型 2 载荷 | 对应速度 |
|---|---|---|
| 准备/零速 | `01 00 00 00 00 00 00` | 三轮目标 0 rad/s |
| 前进 | `01 64 00 00 00 00 00` | ID2/3 两前轮最大绝对轮速约 2.47 rad/s |
| 后退 | `01 9C FF 00 00 00 00` | ID2/3 两前轮最大绝对轮速约 2.47 rad/s |
| 左平移 | `01 00 00 64 00 00 00` | 后轮最大绝对轮速约 2.86 rad/s |
| 左转 | `01 00 00 00 00 C8 00` | 车体角速度 +0.20 rad/s，各轮约 +0.56 rad/s |
| 右转 | `01 00 00 00 00 38 FF` | 车体角速度 −0.20 rad/s，各轮约 −0.56 rad/s |
| 撤权停车 | `00 00 00 00 00 00 00` | 制动后失能，重新启动须新零速 |

随后停止发送全部类型 2，核对 250 ms 到期进入 BRAKING。失能后恢复有效非零、有效零速流，均应保持失能。再显式发送 `valid=0`、新的有效全零帧，待使能准备完成后测试起步。

当前 Jetson bridge 只周期发送类型 3、4，发帧与视觉回调分开，不增加心跳、类型或载荷字段。每 5 ms 交替发送，每种目标周期 10 ms。正常丢目标时仍发合法 `valid=0`，底盘停车等待新目标；夹爪、转向和等待回执阶段同样持续发帧。无效中心不提供到位证据，但合法类型 3/4 都续期通信时效；畸形帧、其他类型和回执不续期。

自动会话建立后，USB 离线或会话代次改变在下一控制检查（目标 5 ms）触发故障制动；最后一条合法类型 3/4 的接收时间达到 250 ms 也触发。底盘随后按控制节拍执行停车请求，250 ms 不代表物理停稳时间。制动截止只设置一次，三轮停稳或达到 500 ms 后请求失能；原姿态、CAN 和电机保护可更早失能。故障锁存至显式重新初始化，重新插线、恢复中心帧或收到迟到回执都不能自行续跑。若是 Mission 单独掉线，先经过 Jetson 的 500 ms 状态有效期，再经过 STM32 的 250 ms 周期流有效期。

三电机控制约 300 帧/s，反馈只在 CAN1，不经过 Jetson。板间周期帧在 USB 上发送。实际到帧间隔和 CAN1 端接仍需实板验证。

## 自动搬运

找物资、整区复定位和里程计返航使用逐拍重算的启动加速度。上层继续根据视觉余量或剩余位姿误差给出制动限速，底盘按 TIM2 实测步长跟随当前目标。基准导航以约 1.17 m/s² 起步，在同向速度达到 0.10 m/s 时放开到 3.5 m/s²。短目标在走完速度差后结束加速，途中更新目标从当前速度接续，同向跟随降速限值为 0.75 m/s²。目标反向时仍按 0.45 m/s² 制动至零。这些离线仿真初值仍需实车复核，配置位于 [chassis_config.h](../project/Core/Control/Chassis/chassis_config.h)。

左右横移为 0.30 m/s、0.70 s，盲推为 0.40 m/s、1.20 s，直线退出为 0.40 m/s、1.00 s。各段计时包含加速，到期提交零目标并等待停稳。正常丢目标沿用减速与停稳确认，通信或底盘故障沿用直接零目标和失能保护。

正式盲推和直退分别给减速送零、实测停稳两段各 1.5 s。三轮连续三组新反馈均不超过 0.20 rad/s 可确认停稳。若轮速估计在零速附近仍有偏置，底盘也会从送零起连续核对三轮编码器和实测陀螺。两组条件独立累计，达到其中一组即可确认停稳。

| 连续观测 | 每轮编码角最大跨度 | 每轮速度绝对值上限 | 陀螺偏航角速度绝对值上限 |
| --- | --- | --- | --- |
| 300 ms | 0.006 rad | 0.32 rad/s | 0.05 rad/s |
| 600 ms | 0.004 rad | 0.42 rad/s | 0.025 rad/s |

两组均要求三轮反馈逐帧连续、位置有效、IMU 为新鲜实测值。缺轮、跳帧或姿态过期后须重新收集证据，1.5 s 截止时间继续计时。SWD 可查看 `encoder_stop_debug` 的拒绝原因和 `accepted`，取值 1 表示 300 ms 条件通过，2 表示 600 ms 条件通过。

盲推或回撤送零后，若速度反馈抖动，而三轮角位置已连续 120 ms 稳定，可临时对三轮同时撤掉速度阻尼。从卸力前最后一帧轮角起，逐轮记录首轮反馈，三轮到齐后继续观察 600 ms。某轮先到、再回弹的位移仍计入角跨度和累计行程。此时允许的瞬时反馈峰值为 0.80 rad/s，每轮仍须满足角跨度 0.004 rad、持续超速时间与累计往返行程限制，且平均绝对速度不超过 0.42 rad/s、实测偏航角速度不超过 0.025 rad/s。三轮重新收到常规阻尼命令并反馈后才允许续行。`encoder_stop_debug.accepted=3` 表示这条路径通过。`neutral_stop_debug` 保留了卸力、恢复的时间以及各轮的位置和速度证据。

自动导航加速可通过 CMake 的 `CHASSIS_NAV_SPEED_CONTROLLER` 选择实验方法。默认 `0` 沿用逐拍加速，`1` 为有限候选 MPC，`2` 为边界层 SMC，`3` 为带扩张状态观测器的 ADRC。例如使用 `cmake --preset Debug-Auto -DCHASSIS_NAV_SPEED_CONTROLLER=1` 构建 MPC 版。三种方法从当前航向坐标系下的新轮速反馈调整加速步长，反馈未更新或轮角与陀螺持续分歧时沿用原加速上限。反向、停车和定时盲推动作继续使用各自的限制。 [算法说明](../project/Core/Algorithm/Chassis/README.md#自动导航速度实验)

### 实车标定

将物资框中心或安全区透视中心（区域两条对角线的交点）的像素坐标分别除以图像宽、高，即可在现场录帧里离线量到设定点。联调时也可读类型 3 或类型 4 的两个定点值，再除以 10000 核对。

物资窗口和整区复定位窗口按各自的实际停位标定。将车摆到可夹取位或整区起点，分别复核现有窗口及其各轴容差。机械距离按[场地补测](ODOMETRY.md#2026-11-22-自动搬运场地标定)配置。Jetson 启动前将夹爪放在张开参考位，会话记录该位置为 0°。闭合到 −50°，张开回到 0°，驱动时间为 1 s。

| 宏 | 实测和填写内容 |
|---|---|
| `JETSON_AUTO_TARGET_X`、`JETSON_AUTO_TARGET_Y`、`JETSON_AUTO_TARGET_TOL_X`、`JETSON_AUTO_TARGET_TOL_Y` | 单绿可直接夹取时，正式目标中心的 479 帧中位为 `(0.5721, 0.8681)`，两轴容差各 0.0100。 |
| `JETSON_AUTO_LOCALIZE_*` | 在整块安全区的起点标定类型 4 全区透视中心的 `X/Y/TOL_X/TOL_Y` 窗口。 |
| `JETSON_AUTO_X_TO_LEFT`、`JETSON_AUTO_Y_TO_FORWARD` | 架空分别确认横移和前后方向。按相机实际方向填 `+1` 或 `-1`。 |
| `JETSON_AUTO_RESPONSE_X_Q_PER_M/Y_Q_PER_M`、`JETSON_AUTO_APPROACH_SPEED_MPS`、`JETSON_AUTO_APPROACH_LATENCY_S` | 标定局部图像响应、平移限速和延迟；接近速度使用制动距离规划。 |
| `JETSON_AUTO_ALIGN_MAX_M` | 定时横移及整区复定位从各自起点计算的位移上限。当前场地取 0.25 m。 |
| `JETSON_AUTO_SAFETY_STRAIGHT_M` | 单次底盘联调中从视觉对准点到安全区到达点的直行距离。 |
| `JETSON_AUTO_DISTANCE_SPEED_MPS` | 独立底盘联调使用。正式搬运的盲推和放货后退按限时速度执行。 |
| `JETSON_AUTO_OUT_TURN_DEG` | 单次底盘联调按目的区转向。正式搬运固定从起点原地转 180°。 |

未标定的距离或限位保留零值。自动固件可以在此状态下观察视觉数据，保持使能零速。完成标定及开爪握手后才能执行搬运。正式搬运的物资接近和里程计返航限速 0.50 m/s，整区视觉复定位限速 0.33 m/s。常规定距、盲推和直线后退限速 0.40 m/s，返航和定角转向上限为 3.0 rad/s。定距最短请求为 10 mm，进入剩余 1.5 mm 的内层窗口后平滑减速，软件交接容差为 10 mm。定角用 2° 入窗、3° 退出的滞回。返航不设置距离上限，采用 20 s 总时限。停稳后按 2 cm、10° 复核到位，小幅偏差可在原时限内补正一次。定时横移与前后移动的实际行程和夹爪动作须在现场核验。

里程计在 STM32 本地运行，不增加 USB 帧或改变现有载荷。一次短临界区复制三轮反馈，仅当三轮接收序号都更新、反馈未超 30 ms 且 IMU 未超 20 ms 时推进积分。积分间隔最长 50 ms；断流丢失的路段不补算，恢复首组只建立基线。定距起步要求里程计有效；运行中里程计失效、连续 1500 ms 无进展或总时限到期均制动后失能，不发送正常到达结果。总时限取本段理想限速轨迹时间的 3 倍加 2000 ms。接口和复查方法见 [里程计与定距控制](ODOMETRY.md)。

### CMSIS-DAP 读取坐标

这段流程只读 RAM，不发送报文。手动速度由 Jetson `teleop_pkg` 独立封装为类型 2。

按 [CLion 下载与调试](CLION_DEBUG.md) 接 CMSIS-DAP。下载后让板持续运行，退出 CLion 调试会话再打开 OpenOCD，避免两个程序同时占用探针。使用与板内程序**完全相同**的 ELF 离线解析地址。这里的 GDB 只读 ELF，无需连接目标。

```powershell
$gdb = 'C:/Program Files/Arm/GNU Toolchain mingw-w64-x86_64-arm-none-eabi/bin/arm-none-eabi-gdb.exe'
& $gdb -q -nx -batch project/build/Debug-Auto/project.elf `
  -ex 'p/x &stage' `
  -ex 'p/x &jetson_data.target.valid' -ex 'p/x &jetson_data.target.x' `
  -ex 'p/x &jetson_data.safety.valid' -ex 'p/x &jetson_data.safety.x'

$openocd = 'D:/DevEnv/openocd-v0.12.0-i686-w64-mingw32/bin/openocd.exe'
& $openocd -f project/openocd-daplink.cfg -c 'bindto 127.0.0.1' `
  -c 'gdb_port disabled' -c 'tcl_port disabled' `
  -c 'stm32f4x.cpu configure -event examine-end {}' -c init
```

连接 OpenOCD 的本机 Telnet `127.0.0.1:4444`，执行 `stm32f4x.cpu curstate`，确认返回 `running`。下面尖括号内替换为上一步的 `0x...` 地址。也可把各命令作为同一 OpenOCD 进程的 `-c` 参数，最后加 `-c shutdown`。测量中保持 MCU 运行，避免 `halt`、断点或 `reset` 暂停控制任务。

```tcl
mdb <STATE> 1
mdb <TARGET_VALID> 1
mdh <TARGET_X> 2
mdb <SAFETY_VALID> 1
mdh <SAFETY_X> 2
```

每组 `mdh` 依次显示 `x`、`y` 两个有符号 16 位定点值。摆到物资可夹取位置或安全区可推送位置，停稳后重复读取，只记录 `valid=1` 且稳定的值。例 `0x1388 0x1d4c` 是 `5000,7500`，分别除以 10000 得到 `(0.50,0.75)`。正式复定位使用 Jetson 在 `LocalizingStart` 阶段发布的整块安全区中心。单独切换视觉模式到 `field` 仍可能读到 `valid=0`。

### 机械动作与握手

1. 首次在能看全安全区的位置静置，车头朝向安全区。USB 收到合法类型 3/4 后查询启动开爪结果（类型 21，kind 0）。开爪成功后发类型 22，待 Jetson 确认整区识别才开始自动对准。对准停稳后核对两张新图并保存起点，再原地转 180°。转向停稳后发送类型 19，待确认目标识别才开始找物块。
2. 物块接近到近处后先收敛横向误差。入窗减速、停稳并用新图复核后发类型 21（kind 1）夹取。8 字节载荷为 kind、4 字节小端请求号和 3 字节小端源帧号，源帧号来自最终确认入窗的类型 3 帧，回绕时可为零。Bridge 核对这张帧确已发出、仍新鲜、匹配唯一，且当前轨迹及类别与之连续。拒绝后用新图重新对准并换请求号。受理后按原请求号、原源帧查询。收到同号的受理回执和成功终态后发 `16/02` 登记目的区，核对类型 5、6 与 8，两个目的区须一致，类型 8 编号须等于本次抓取编号。
3. 使用里程计返回本趟保存的起点，朝向采用首次起点的安全区方向。提交零速并复核返回位姿后按原 token 发类型 17。命令 1 匹配受理后，按目的区锁航向左移或右移，目标 0.30 m/s，计时 0.70 s。横移停稳后发类型 18。
4. 命令 2 匹配受理后以 0.40 m/s 盲推 1.20 s，停稳后发送类型 21（kind 2）松爪。夹爪成功后以 0.40 m/s 直线回退 1.00 s，停稳再发类型 20。
5. 命令 4 匹配受理后，按实测横移量反向复位并恢复安全区航向，合速度上限 0.225 m/s。停稳后，用新的类型 4 整区中心做视觉精调，合速度上限 0.33 m/s。两阶段共用 10 s 与同一位移限距。停稳后连续两张新源帧入窗，记录当前位姿。继续下一趟时原地转 180°。达到设定趟数时在该位姿请求三轮失能。
6. 路由被拒、回执超时或夹爪失败时进入故障制动。故障恢复后须重新初始化并验证整区起点。

等待期间每 250 ms 重报原请求，沿用首次截止时刻。普通请求期限为 3.45 s。夹爪请求期限为 **13.5 s**，覆盖退出旧保位、动作准备、最长 10 s 运动、0.5 s 到位确认、停机重试和链路余量。启动查询首次受理前保持使能零速，首次受理后计时。类型 9 报告夹爪终态。闭合回读 Stop，现场开爪在连续三次低速输出轴反馈进入 ±2° 后轻力保位。实际夹持与释放须现场确认。匹配失败结果、结果超时或电机反馈异常进入故障停车。普通回执须在截止前的控制拍消费。

<a id="单趟构建与-swd-终态"></a>

### 指定搬运趟数与 SWD 终态

`JETSON_AUTO_DELIVERY_LIMIT` 指定本次启动的搬运趟数。`Debug-Auto` 预设显式设为 `0`，即使复用曾构建五趟的目录，重新按预设配置也恢复连续搬运。单绿验收时可在配置命令覆盖为 `1`。绿、橙各一块设为 `2`，绿色左投后继续取橙，右投并复定位后失能。以下命令在本机仓库根目录的 PowerShell 执行，ARM GNU 工具链需在 PATH 中。

```powershell
$clionBin = 'C:/Program Files/JetBrains/CLion 2026.2.3.1/bin'
$env:Path = "$clionBin/cmake/win/x64/bin;$clionBin/ninja/win/x64;$env:Path"
$deliveryLimit = 0
$deliveryBuild = "STM32/project/build/field-transport-limit-$deliveryLimit"
cmake --preset Debug-Auto -S STM32/project -B $deliveryBuild -DJETSON_AUTO_DELIVERY_LIMIT=$deliveryLimit
if ($LASTEXITCODE -eq 0) { cmake --build $deliveryBuild --parallel }
```

烧录与 SWD 地址解析均使用该目录的 `project.elf`。按上文 CMSIS-DAP 读 RAM 方法查询 `'jetson_auto.c'::stage`、`'jetson_auto.c'::start_pose`、`'chassis.c'::disable_confirmed` 和 `jetson_completed_deliveries`。双趟时，首趟送达、复定位并保存新起点后计数为 1，底盘保持使能。

| `stage`（十进制） | 达到设定趟数时的状态 |
| ---: | --- |
| 33 | `WAIT_DELIVERY_DISABLE`。整区双帧复核已通过，`start_pose` 保存新起点，底盘已请求三轮失能。末趟尚未计入。 |
| 34 | `DELIVERY_DONE`。三轮新反馈均已确认失能，计入末趟并锁定。双趟验收时计数为 2。 |
| 21 | `FAULT`。流程锁定故障。失能确认超过 500 ms，或等待期间 USB 会话断开，也会进入此状态。失败趟不计数。 |

11 月 23 日主机回归中，单趟模拟只执行取货前的首次 180° 转身，等待失能确认期间收到新图也保持停车。GDB 在进入 34 时读到的 `start_pose` 与模拟里程计一致。双趟模拟依次接受左、右目的区和独立的请求号，只执行两次取货转身。第二趟松爪失败、整区复核超时、缺一轮失能反馈和断线均保留首趟计数 1。默认模式在同一次启动中完成了 300 趟，第五趟后空场零速等待 15 秒，随后补货续跑，第 301 趟搜索已就绪。可从仓库根目录复跑：

```powershell
python STM32/tests/run_native.py --suite jetson_auto_single_delivery --suite jetson_auto_two_deliveries --suite jetson_auto_configured --suite chassis
python STM32/tests/check_jetson_codec.py --jetson-root Jetson
python STM32/tests/check_gripper_budget.py
```

### 连续双趟实车记录

2026-11-23 的 `live-multi-4` 使用 `field-two-20261123/project.bin`，SHA-256 为 `AD089E625FFAC4971345989258D7E6F27C411A48BCE930C2CB6A3AEEE18D219B`。绿色首趟取得左区许可（token 4），投送后底盘继续搜索。橙色第二趟取得右区许可（token 7）。同一次启动收到两次送达回执，停车照片确认绿块留在左区、橙块留在右区。

| 板上 tick（ms） | SWD 记录 |
| ---: | --- |
| 20170 | 阶段 32 正在首次整区定位，`start.valid=0`。 |
| 28004 | 首次读到 `start.valid=1`，保存的起点采样时间为 27970004 µs。 |
| 60383 | 首趟已复定位，`completed_deliveries=1`、`enabled=1`，阶段 17 正在为第二趟转身。 |
| 114447 | 阶段 34，`completed_deliveries=2`、`enabled=0`、`disable_confirmed=1`，三轮反馈状态均为 0。 |

本机留存的 SWD 原始记录为 `STM32/outputs/transport-20261123-board/multi-4.jsonl`。观察器摘要和停车照片分别为 `Jetson/outputs/transport-20261123-local/two-trip-summary.json` 和同目录的 `two-delivered.jpg`。这次结果对应当时光照，弱光重放和既有视觉压力失败见[自动搬运交接](../../docs/ai-handoff.md#2026-11-23-自动搬运实测)。

### 三绿两橙五趟构建

在仓库根目录打开 PowerShell，沿用上文的 CLion CMake、Ninja 和 ARM 工具链。为五趟固件单独配置构建目录，保留双趟 ELF 供已有 SWD 记录复核。

```powershell
$fiveBuild = 'STM32/project/build/field-five-20261123'
cmake --preset Debug-Auto -S STM32/project -B $fiveBuild -DJETSON_AUTO_DELIVERY_LIMIT=5
if ($LASTEXITCODE -eq 0) { cmake --build $fiveBuild --parallel }
Select-String -LiteralPath "$fiveBuild/CMakeCache.txt" -Pattern '^JETSON_AUTO_DELIVERY_LIMIT:STRING=5$'
```

按上文流程烧录并校验本目录的 `project.elf`，SWD 也使用此 ELF 解析地址。摆放三绿两橙并让第一趟能取到绿色。逐趟核对独立 token、送达回执和实物落区，绿色应有三块落左区，橙色应有两块落右区。以第五趟复定位后 `stage=34`、`jetson_completed_deliveries=5`、三轮反馈均失能且 `disable_confirmed=1` 为停车终态。

### 下一次现场单绿验收

1. 将 STM32 OTG 接到 Jetson，S288 接到另一 USB 口；调试电脑通过 DAP-Link 核对固件身份。Jetson 现场配置使用 USB 的 `/dev/serial/by-id/` 名称，枚举编号变化无需修改。
2. 将上文 `$deliveryLimit` 设为 `1`，按[指定搬运趟数与 SWD 终态](#指定搬运趟数与-swd-终态)编译、烧录并校验单趟 ELF。在 Jetson 上按[完整构建步骤](../../README.md#jetson)从同一工作树更新六个运行包，加载新的 overlay。抓取使用 8 字节请求，两端均须升级。停止上一版 ROS 进程再启动，避免同时占用相机或 USB。
3. 先用 `run:=vision` 核对相机和整区外框。停车时镜头看到完整安全区，夹爪在机械张开位，单绿放在转身后可见的取货位置。关闭预览进程后，在现场确认可以动车时只运行一次完整入口：

```bash
profile="$(ros2 pkg prefix --share bringup_pkg)/config/field_20261122.yaml"
ros2 launch bringup_pkg rescue.launch.py run:=all config:="$profile" stream:=false
```

4. 核对顺序：首次整区停稳存点 → 180° 取货 → 合爪成功 → `16/02` 登记左区 → 里程计返航 → 横移、盲推、松爪及直退 → 反平移 → 停稳、整区双帧与新起点。记录开机对准用时、从直退停稳到新起点的用时，以及初次和重记的里程计采样时间。闭合动作终态与物块是否实际留在左区均须记录。
5. 记录新起点后等待 SWD 的 `stage=34`，核对三轮已失能，再停止本次 launch。若读到 `stage=21`，保留当次日志并按故障处理。后续连续验证使用默认的 `Debug-Auto` 固件。

### 实车验收顺序

1. 车轮离地，先用手动固件核对三个电机方向、停稳和断流。检查 CAN1 端接及 USB 到帧间隔。准备一个可立即断开电机动力的开关。
2. 完成标定，编译并核对两端版本。首次上电仍架空并保持 IMU 静止，确认命令 5 受理时不移动、类型 9 成功后才转段；检查旧编号不放行。再核对搜寻方向、横移、前后方向和到位停车。
3. 先按 [里程计复核](ODOMETRY.md#实车复核)在清空的场地用低速、空载比较八向定距的估计位移与尺量结果，核对原地旋转、转向接定距以及反馈丢失后的停车。再走完整个流程：在 USB 上核对类型 16–22 的时序和 token，分别核对整区起点返航、左右定时横移、盲推、放货后退和整区重新建点，确认每次转向或退距结束后才出现对应握手。最后用实际物资确认夹稳、推入、释放和撤回。
4. 单独复测路由拒绝、旧夹取编号、丢目标、丢安全区、动作失败、受理丢失和结果丢失。确认重复查询不重复开合，等待截止不延长。盲推段单独遮挡相机，确认不再修正方向或因丢识别停车；断 USB 或轮反馈异常仍应按现有逻辑制动。
5. 分别在搜索、接近、转向、盲推、退距和等待夹爪时停止 bridge 或拔掉 USB，核对下一控制检查/250 ms 断流检查触发制动，停稳或制动 500 ms 后失能；恢复连接不得续跑。另停掉 Mission，核对 bridge 先在 500 ms 失效后停流。正常发送无效中心超过 250 ms 应原地等待，不应报通信故障。恢复前确认车已停稳，夹爪附近无人，并显式重新初始化。

## 离线验证入口

```powershell
python tests/run_native.py --temp-dir .
python -B tests/check_jetson_codec.py --temp-dir .
python -B -m unittest discover -s ../Jetson/tools/tests -v
cmake --preset Debug -S project
cmake --build project/build/Debug
cmake --preset Debug-9axis -S project
cmake --build project/build/Debug-9axis
```

主机编译器可加 `--cc` 指定。协议互验可用 `--jetson-root` 指定 Jetson 目录。这些检查在电脑端离线执行。

离线回归用于验证报文、流程和固件编译。场地参数见本页。离线结果见 [本轮验证记录](../../docs/validation-2026-11-22.md)，两块板接通后按现场步骤做带货全程验收。

现行帧表、时间量与放行规则见 [`Jetson/docs/usb-protocol.md`](../../Jetson/docs/usb-protocol.md)。
