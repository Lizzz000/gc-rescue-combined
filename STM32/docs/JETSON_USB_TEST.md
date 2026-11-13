# Jetson 板间 USB 与三轮 CAN1 联调

板间链路是 STM32 USB OTG FS 设备，Jetson 主机打开 `link.device`。引脚 PA11 D-、PA12 D+、PA10 OTG ID，全速 12 Mbit/s，没有 ULPI，不是 480 Mbit/s。bridge 与手动程序不要同时打开同一只设备。

帧布局：SOF `A5`、版本 `01`、类型 uint8、长度 uint8、载荷、CRC-16/IBM 小端。CRC 覆盖版本、类型、长度和载荷。最大载荷 8。CRC 错则丢掉一个字节再同步。空载荷不是失联。

电机仍是 CAN1 上的达妙 MIT，使用 PD0/PD1、1 Mbit/s、RX0 优先级 6。C 板 J22/J23 并联，1 脚为 CAN_L，2 脚为 CAN_H。反馈 ID 为 0，发送 ID 为 1–3。这不是板间链路。CAN2 仅保留初始化。帧类型以 [`Jetson/docs/usb-protocol.md`](../../Jetson/docs/usb-protocol.md) 为准。

## 当前接入范围

CAN1 只收发达妙电机。反馈要求 DLC=8，帧内电机 ID 为 1、2、3。板间帧不进 CAN FIFO。

`test_task` 每 5 ms 处理一次链路快照，底盘每 10 ms 更新三轮 MIT。[标定头文件](../project/Core/Control/Jetson/jetson_auto_config.h)中的 `JETSON_AUTO_ENABLED` 选择两种模式，修改后重新编译。

- `0` 为默认的手动模式，不运行自动流程；CMSIS-DAP 可读取坐标。类型 2 畸形时立即撤销上一条速度。
- `1` 为上电自动搬运。类型 1 长度 8：`alive`、`pause`、`vx/vy/wz` 三个 int16（×1000）。只有 `alive=1` 且 `pause` 为 0 或 1 才保活。普通畸形帧不刷新心跳，到最后一条合法帧满 250 ms 按断流停车。`alive=0` 且三个速度都为 0 才是任务失联，立即撤销心跳；这不是空载荷，也不是 pause。`pause=1` 时不开始下一段运动。移动由视觉中心和本地动作流程控制，夹爪由 Jetson 驱动。

| 类型 | 长度 | 方向 | 载荷 |
|---:|---:|---|---|
| 1 | 8 | Jetson → STM32 | `alive`、`pause`、`vx/vy/wz`。`alive=1` 在线；`alive=0` 且速度全 0 为失联 |
| 2 | 7 | Jetson → STM32 | `valid`、`vx`、`vy`、`wz`。自动固件忽略，手动固件忽略类型 1 |
| 3 / 4 | 8 | Jetson → STM32 | `valid`、`x`、`y`、源时间低 24 位。目标中心、安全区透视中心 |
| 5 | 1 | Jetson → STM32 | `accepted`，0 或 1 |
| 6 | 3 | Jetson → STM32 | `valid`、`passed`、`destination`。只接受 000、100、111、112 |
| 7 | 6 | Jetson → STM32 | `accepted`、命令 1–5、token。命令 3 和 5 的 token 必须是 0。命令 5 只表示受理到点 |
| 8 | 6 | Jetson → STM32 | `data0=1`、目的区 1 或 2、token |
| 16 | 1 | STM32 → Jetson | `data0=1`，到达 |
| 17 / 18 / 20 | 5 | STM32 → Jetson | `data0=1` 加 token。转向完成、安全区到达、投送 |
| 19 | 1 | STM32 → Jetson | `data0=1`，开始搜索 |
| 21 | 1 | STM32 → Jetson | `1` 夹取，`2` 释放 |

类型 21（`1` 夹取点、`2` 释放点）由类型 7 命令 5、token 0 受理。完整帧表、250 ms 链路超时、3.45 s 请求期限和放行规则以 [`Jetson/docs/usb-protocol.md`](../../Jetson/docs/usb-protocol.md) 为准。

坐标帧里源帧时间是低 24 位毫秒，同一源帧的重发不重复用于滤波或到位确认。Mission 每 50 ms 发布状态，bridge 在 500 ms 未收到状态时停止接收新握手、丢弃待发事件并撤销运动保活及有效坐标。

所有多字节字段小端。协议 `vx/vy` 是车体前/左方向的线速度字段，单位 m/s，原始值除以 1000；`wz` 是逆时针为正的车体角速度，单位 rad/s，也除以 1000。中心坐标除以 10000 得图像归一化位置，既不是距离也不是轮速。接收不裁剪中心坐标。token=0 合法。

上行只走 USB。三个电机保持 MIT 模式，反馈 ID 0，发送 ID 1–3。

## 手动速度联调

1. 上电为 `MANUAL_DISARMED`，非零速度不会自动使能。新的有效全零类型 2 才申请三段使能，进入 `MANUAL_PREPARING`。
2. 初次使能准备期间持续发送有效零速。原底盘须确认三组新鲜、已使能的三轮静止反馈，才进入 `MANUAL_READY`；准备最多 1000 ms，不由重复零帧延长。此时发非零会进入停车流程。
3. READY 后发新的非零速度进入 `MANUAL_RUNNING`。类型 2 手动速度的车体平移合速度上限为 0.50 m/s，车体角速度上限为 2.0 rad/s。超范围整帧拒绝并停车。Jetson 仍将速度乘 1000 编成有符号 16 位定点数，STM32 按解码后的合速度和角速度检查限幅。手动发送对斜向分量向零截断，贴近上限时应留出 0.001 m/s 余量，以免定点后超限。平移上限下，纯前进两后轮绝对轮速约 12.37 rad/s，纯横移前轮约 14.29 rad/s。纯转在角速度上限时各轮约 6.17 rad/s，两项同时取上限时任意方向的单轮最大绝对轮速约 20.46 rad/s。
4. 运行中收到有效零速立即清除运动目标，并重新等待三轮停车确认。即使同一任务周期内随后收到非零帧，零速事件仍会执行。停稳前到达的合法非零帧会被忽略，收到停稳后的新帧才起步。等待期间继续检查三轮反馈和速度时效。
5. 从最后一条有效速度的实际接收时刻达到 250 ms，或收到 invalid/畸形/越界速度，进入 `MANUAL_BRAKING`。三轮反馈超 30 ms、状态异常或底盘拒绝动作同样停车。中间收到的 invalid 事件会保留独立撤权代次，不能被紧接着的有效帧覆盖。
6. 制动只设一次截止，不被新帧续期；确认停车或达到 500 ms 后请求失能，FD 仍由底盘循环提交。底盘自身的姿态/CAN 保护仍可更早失能。未收到停车反馈时，失能请求不等于已证实物理停车。
7. 停车期间收到的零速不会自动重新使能。失能后须再收到新的有效零速，并重新通过准备过程；旧非零指令不会自行恢复。

PREPARING 在初次使能和运行中收到零速后保持零目标，最多等待 1000 ms。初次使能允许反馈建立，运行中停车等待沿用 READY/RUNNING 的 30 ms 反馈期限。初次使能暂忙可能消费当前零帧，发送端须持续发送零速直到 READY。

`manual_control_state` 可在调试器只读观察，枚举依次为 DISARMED=0、PREPARING=1、READY=2、RUNNING=3、BRAKING=4。它是任务处理阶段，不是电机执行回执；底盘在随后计算中触发保护时，状态显示最多滞后一个 5 ms 调度周期。底盘反馈仍查看 `dm_motor[3]`。不要通过修改状态变量绕过停车条件。

### 架空测试

先确认烧录的是用 `JETSON_AUTO_ENABLED=0` 重新编译的 `project/build/Debug/project.elf`。

发送端按 10 ms 周期逐次提交单帧类型 2。先保持有效零速约 1 s，确认 READY，再短时测试一个方向，随后持续零速直到停稳。首次在轮子离地时确认方向、停车和断流保护。

下表是类型 2 载荷，不是 CAN 文本。数值对应现有三轮几何，轮速为几何轮速，MIT 实际命令还乘现有电机方向符号。

| 动作 | 类型 2 载荷 | 对应速度 |
|---|---|---|
| 准备/零速 | `01 00 00 00 00 00 00` | 三轮目标 0 rad/s |
| 前进 | `01 64 00 00 00 00 00` | 两后轮最大绝对轮速约 2.47 rad/s |
| 后退 | `01 9C FF 00 00 00 00` | 两后轮最大绝对轮速约 2.47 rad/s |
| 左平移 | `01 00 00 64 00 00 00` | 前轮最大绝对轮速约 2.86 rad/s |
| 左转 | `01 00 00 00 00 C8 00` | 车体角速度 +0.20 rad/s，各轮约 +0.62 rad/s |
| 右转 | `01 00 00 00 00 38 FF` | 车体角速度 −0.20 rad/s，各轮约 −0.62 rad/s |
| 撤权停车 | `00 00 00 00 00 00 00` | 制动后失能，重新启动须新零速 |

随后停止发送全部类型 2，核对 250 ms 到期进入 BRAKING；恢复非零帧应仍不动，重新发送零速并停稳后才可起步。

当前 Jetson bridge 在链路就绪且任务在线时周期提交类型 1、3、4，发帧与视觉回调分开。类型 1 为心跳；任务失联时 `alive=0`。不再订阅 `/cmd_vel`，也不把自动速度写入类型 1。断流由 C 板按最后一条格式合法类型 1 的实际接收时刻执行 250 ms 超时。联调前核对 Jetson 实际运行版本，并在 USB 上确认 `alive`、`pause` 和到帧间隔。

当前 bridge 每 10 ms 重发三种周期帧，含无效坐标；有效坐标按源帧时间去重并检查有效期。

三电机控制约 300 帧/s，反馈只在 CAN1，不经过 Jetson。板间周期帧在 USB 上发送。实际到帧间隔和 CAN1 端接仍需实板验证。

## 自动搬运

### 实车标定

将物资框中心或安全区四顶点平均值的像素坐标分别除以图像宽、高，即可在现场录帧里离线量到设定点。联调时也可读类型 3 或类型 4 的两个定点值，再除以 10000 核对。

目标和安全区均使用用户指定的原始定点中心 `5000,7500`，两轴各允许 ±100。更换相机位置后需在夹取点和安全区推送起点复核这组参数。机械距离仍待标定。`gripper.move_s` 仍是待标定空值，不要填数字。

| 宏 | 实测和填写内容 |
|---|---|
| `JETSON_AUTO_TARGET_X`、`JETSON_AUTO_TARGET_Y`、`JETSON_AUTO_TARGET_TOL_X`、`JETSON_AUTO_TARGET_TOL_Y` | 把车和目标摆到可夹取位置，测类型 3 的归一化中心，按可夹取范围填写各轴允许误差。 |
| `JETSON_AUTO_SAFETY_X`、`JETSON_AUTO_SAFETY_Y`、`JETSON_AUTO_SAFETY_TOL_X`、`JETSON_AUTO_SAFETY_TOL_Y` | 把车摆到安全区的推送起点，测类型 4 的中心和到位窗口。 |
| `JETSON_AUTO_X_TO_LEFT`、`JETSON_AUTO_Y_TO_FORWARD` | 架空分别确认横移和前后方向。按相机实际方向填 `+1` 或 `-1`。 |
| `JETSON_AUTO_APPROACH_GAIN`、`JETSON_AUTO_APPROACH_SPEED_MPS`、`JETSON_AUTO_SEARCH_WZ_RAD_S` | 标定视觉跟随增益、平移限速和丢目标时的车体转向角速度。 |
| `JETSON_AUTO_DISTANCE_SPEED_MPS`、`JETSON_AUTO_ROI_BACK_M`、`JETSON_AUTO_RECOVERY_BACK_M` | 按场地测定定距限速、检查前退距和拒绝恢复退距。盲推及放货后退按限时速度执行。 |
| `JETSON_AUTO_OUT_TURN_DEG`、`JETSON_AUTO_RETURN_TURN_DEG` | 当前均为 180°。去程按目的区左转或右转，回转取相反方向。 |

未标定参数留零，继续使用 `JETSON_AUTO_ENABLED=0`。全项标定后才设为 `1`。夹爪由 Jetson 驱动，实车需目视核实夹稳和释放。盲推和放货后退分别以两后轮目标约 12.37 rad/s 的直行速度执行 1.2 s，时间含加速，随后停车并等三轮停稳；航向修正会改变各轮瞬时目标。其余定距动作以指令速度积分规划，实际行程需用车上标记或量尺复测。

### CMSIS-DAP 读取坐标

这段流程只读 RAM，不发送报文。手动速度由 Jetson `teleop_pkg` 独立封装为类型 2。

按 [CLion 下载与调试](CLION_DEBUG.md) 接 CMSIS-DAP。下载后让板持续运行，退出 CLion 调试会话再打开 OpenOCD，避免两个程序同时占用探针。使用与板内程序**完全相同**的 ELF 离线解析地址。这里的 GDB 只读 ELF，无需连接目标。

```powershell
$gdb = 'C:/Program Files/Arm/GNU Toolchain mingw-w64-x86_64-arm-none-eabi/bin/arm-none-eabi-gdb.exe'
& $gdb -q -nx -batch project/build/Debug/project.elf `
  -ex 'p/x &manual_control_state' `
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

每组 `mdh` 依次显示 `x`、`y` 两个有符号 16 位定点值。摆到物资可夹取位置或安全区可推送位置，停稳后重复读取，只记录 `valid=1` 且稳定的值。例 `0x1388 0x1d4c` 是 `5000,7500`，分别除以 10000 得到 `(0.50,0.75)`。安全区数据须等 Jetson 进入对应目的区的 `NavigatingSafety` 并识别到安全区。单独切换视觉模式到 `field` 仍可能读到 `valid=0`。

### 机械动作与握手

1. 上电静置，等待 Jetson 心跳与 IMU 就绪。使能后确认三轮停稳，再等 pause=0 且 alive=1 的类型 1（Jetson 张开夹爪期间发 pause=1），之后跟随类型 3。目标丢失时按标定角速度原地旋转搜寻。
2. 进入目标窗口并停稳后，用通过新鲜度核验的窗内坐标确认夹取点，发类型 21 载荷 `01`。类型 7 命令 5 受理后按放行规则等待，再定距后退并停稳，发类型 16 载荷 `01`。先确认类型 5 受理，再等待类型 6 通过及同侧类型 8。
3. 按授权方向转 180°并停稳，发类型 17（`01` 加小端 token）。等类型 7 命令 1 受理，再跟随类型 4。安全区中心丢失时停车等待。
4. 进入安全区窗口并停稳，连续两个通过新鲜度核验的窗内坐标包才能触发类型 18（同 token）。横纵两轴分别记录最近一次越界，同一轴在坐标有效期内再次同向越界时恢复微调。等命令 2 受理，再限时盲推。停稳后发类型 21 载荷 `02`，命令 5 受理并放行后限时后退并停稳，再发类型 20（同 token）。
5. 等命令 4 受理后反向回转 180°，停稳后发类型 19 载荷 `01`。收到命令 3 受理后重新寻找目标。回执丢失重试时，配套 Jetson 若已确认处于寻找模式，可直接重发寻找授权。
6. 到达明确被拒绝（类型 5 载荷 `00`），或到达已受理但 ROI 明确未通过（类型 6 载荷 `01 00 00`）时，Jetson 张开夹爪期间发 pause=1；放行后按恢复退距后退，再发类型 19。

握手等待期间每 250 ms 重发同一请求，机械动作只执行一次。请求期限只有一个 3.45 s（`JETSON_AUTO_REQUEST_TIMEOUT_MS`），从停稳后首次尝试提交起算，含提交等待，发送成功不重新计时；它覆盖 Jetson 的 3 s 服务预算以及调度、队列和重报余量。等待类型 7 时按当前命令及 token 匹配，保留首个受理回执。放行规则：任何回复只表示受理，下一段运动要等该回复之后第一帧合法的 alive=1、pause=0 心跳且底盘停稳；放行等待期间不重发、不计请求期限。握手超时、心跳失效或电机反馈异常时停车并失能。

### 实车验收顺序

1. 车轮离地，先用手动固件核对三个电机方向、停稳和断流。检查 CAN1 端接及 USB 到帧间隔。准备一个可立即断开电机动力的开关。
2. 完成标定，编译自动固件并核对烧录版本。首次自动上电仍架空并保持 IMU 静止，核对 Jetson 夹爪开合期间心跳 pause=1、完成后 pause=0。检查未见目标时的旋转方向，再用两种视觉中心确认横移、前后方向和到位停车。
3. 在清空的场地用低速和空载先走完整个流程。在 USB 上核对类型 16–21 的时序和 token，测量四段定距，确认每次转向或退距结束后才出现对应握手。再用实际物资确认夹稳、推入、释放和撤回。
4. 单独复测 ROI 拒绝、丢目标、丢安全区及心跳断流。恢复前确认车已停稳，夹爪附近无人。

## 离线验证入口

```powershell
python tests/run_native.py --temp-dir .
python -B tests/check_jetson_codec.py --temp-dir .
cmake --preset Debug -S project
cmake --build project/build/Debug
cmake --preset Debug-9axis -S project
cmake --build project/build/Debug-9axis
```

主机编译器可加 `--cc` 指定。协议互验可用 `--jetson-root` 指定 Jetson 目录。这些检查在电脑端离线执行。

离线回归用于验证报文、流程和固件编译。自动搬运仍待实车标定、烧录和上车验收。

现行帧表、时间量与放行规则见 [`Jetson/docs/usb-protocol.md`](../../Jetson/docs/usb-protocol.md)。
