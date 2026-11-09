# Jetson 与三轮 CAN1 联调

Jetson 与三个电机共用 CAN1，使用 PD0/PD1、1 Mbit/s、RX0 优先级 6。C 板 J22/J23 并联，1 脚为 CAN_L，2 脚为 CAN_H。CAN2 仅保留初始化。协议沿用 `C:/Users/Lenovo/OneDrive/Desktop/Jetson`。

## 当前接入范围

CAN1 硬件只将电机反馈 ID `0x000` 和 Jetson 的七个标准数据帧 ID `0x180/181/182/190/191/192/193` 送入 FIFO0。`bsp_can.c` 按 ID 分流，达妙反馈要求 DLC=8，帧内电机 ID 为 1、2、3。已知 Jetson 帧由对应协议分支处理错误长度。手动模式收到畸形 `0x180` 时立即撤销上一条速度。

`jetson_can.c` 在中断中保存最新协议快照，`test_task` 每 5 ms 处理一次，底盘每 10 ms 更新三轮 MIT。[标定头文件](../project/Core/Control/Jetson/jetson_auto_config.h)中的 `JETSON_AUTO_ENABLED` 选择两种模式，修改后重新编译。

- `0` 为默认的手动标定，`0x180` 控制底盘连续速度和车体转向，CMSIS-DAP 可调整舵机脉宽并读取坐标。
- `1` 为上电自动搬运。合法格式的周期 `0x180` 作为心跳，`valid=0` 也可保活。普通畸形帧不刷新心跳，到最后一条合法帧满 250 ms 按断流停车；空载荷 `0x180` 是任务失联标志，立即撤销心跳。移动由视觉中心和本地动作流程控制。

| 下行 ID | DLC | 载荷与用途 |
|---|---:|---|
| `0x180` | 7 | `valid:uint8, vx:int16, vy:int16, wz:int16`，手动速度或自动心跳 |
| `0x181 / 0x182` | 8 | `valid, x:int16, y:int16, source_frame:uint24`，目标及安全区中心 |
| `0x190` | 1 | 到达是否受理 |
| `0x191` | 3 | ROI 有效、是否通过、目的区 |
| `0x192` | 6 | 是否受理、命令、32 位 token |
| `0x193` | 6 | 转向授权、目的区、32 位 token |

坐标帧第 5～7 字节是源帧时间的低 24 位毫秒，同一源帧的重发不重复用于滤波或到位确认。新旧固件的坐标帧长度不兼容，必须同时部署 Jetson 与 STM32。Mission 每 50 ms 发布状态，bridge 在 500 ms 未收到状态时停止接收新握手、丢弃待发事件并撤销运动保活及有效坐标。

所有多字节字段小端。协议 `vx/vy` 是车体前/左方向的线速度字段，单位 m/s，原始值除以 1000；`wz` 是逆时针为正的车体角速度，单位 rad/s，也除以 1000。中心坐标除以 10000 得图像归一化位置，既不是距离也不是轮速。接收不裁剪中心坐标。token=0 合法。

自动模式在底盘调度后、三个 CAN1 发送邮箱均空闲时提交一条上行帧。三个电机保持 MIT 模式，速度模式的 `0x200+ID` 与部分运输 ID 重叠。

## 手动速度联调

1. 上电为 `JETSON_DISARMED`，非零速度不会自动使能。新的有效全零 `0x180` 才申请三段使能，进入 `JETSON_PREPARING`。
2. 初次使能准备期间持续发送有效零速。原底盘须确认三组新鲜、已使能的三轮静止反馈，才进入 `JETSON_READY`；准备最多 1000 ms，不由重复零帧延长。此时发非零会进入停车流程。
3. READY 后发新的非零速度进入 `JETSON_RUNNING`。`0x180` 手动速度的车体平移合速度上限为 0.50 m/s，车体角速度上限为 2.0 rad/s。超范围整帧拒绝并停车。Jetson 仍将速度乘 1000 编成有符号 16 位定点数，STM32 按解码后的合速度和角速度检查限幅。斜向分量分别四舍五入，贴近上限时应留出 0.001 m/s 余量，以免定点取整后超限。平移上限下，纯前进两后轮绝对轮速约 12.37 rad/s，纯横移前轮约 14.29 rad/s。纯转在角速度上限时各轮约 6.17 rad/s，两项同时取上限时任意方向的单轮最大绝对轮速约 20.46 rad/s。
4. 运行中收到有效零速立即清除运动目标，并重新等待三轮停车确认。即使同一任务周期内随后收到非零帧，零速事件仍会执行。停稳前到达的合法非零帧会被忽略，收到停稳后的新帧才起步。等待期间继续检查三轮反馈和速度时效。
5. 从最后一条有效速度的实际 ISR 接收时刻达到 250 ms，或收到 invalid/畸形/越界速度，进入 `JETSON_BRAKING`。三轮反馈超 30 ms、状态异常或底盘拒绝动作同样停车。中间收到的 invalid 事件会保留独立撤权代次，不能被紧接着的有效帧覆盖。
6. 制动只设一次截止，不被新帧续期；确认停车或达到 500 ms 后请求失能，FD 仍由底盘循环提交。底盘自身的姿态/CAN 保护仍可更早失能。未收到停车反馈时，失能请求不等于已证实物理停车。
7. 停车期间收到的零速不会自动重新使能。失能后须再收到新的有效零速，并重新通过准备过程；旧非零指令不会自行恢复。

PREPARING 在初次使能和运行中收到零速后保持零目标，最多等待 1000 ms。初次使能允许反馈建立，运行中停车等待沿用 READY/RUNNING 的 30 ms 反馈期限。初次使能暂忙可能消费当前零帧，发送端须持续发送零速直到 READY。

`jetson_control_state` 可在调试器只读观察，枚举依次为 DISARMED=0、PREPARING=1、READY=2、RUNNING=3、BRAKING=4。它是任务处理阶段，不是电机执行回执；底盘在随后计算中触发保护时，状态显示最多滞后一个 5 ms 调度周期。`jetson_data` 可观察接收快照；底盘反馈仍查看 `dm_motor[3]`。不要通过修改状态变量绕过停车条件。

### 架空测试

先确认烧录的是用 `JETSON_AUTO_ENABLED=0` 重新编译的 `project/build/Debug/project.elf`。

发送端按 10 ms 周期逐次提交单帧，**不要启用 USB转CAN 模块脱离主机仍会继续发送的硬件循环功能**。先保持有效零速约 1 s，确认 READY，再短时测试一个方向，随后持续零速直到停稳。首次在轮子离地时确认方向、停车和断流保护。

下表是标准 CAN 数据帧，`#` 后是数据字节，不是 USB 适配器封装。数值对应现有三轮几何，轮速为几何轮速，MIT 实际命令还乘现有电机方向符号。

| 动作 | CAN 文本 | 对应速度 |
|---|---|---|
| 准备/零速 | `180#01000000000000` | 三轮目标 0 rad/s |
| 前进 | `180#01640000000000` | 两后轮最大绝对轮速约 2.47 rad/s |
| 后退 | `180#019CFF00000000` | 两后轮最大绝对轮速约 2.47 rad/s |
| 左平移 | `180#01000064000000` | 前轮最大绝对轮速约 2.86 rad/s |
| 左转 | `180#0100000000C800` | 车体角速度 +0.20 rad/s，各轮约 +0.62 rad/s |
| 右转 | `180#010000000038FF` | 车体角速度 −0.20 rad/s，各轮约 −0.62 rad/s |
| 撤权停车 | `180#00000000000000` | 制动后失能，重新启动须新零速 |

随后停止发送全部 `0x180`，核对 250 ms 到期进入 BRAKING；恢复非零帧应仍不动，重新发送零速并停稳后才可起步。可单独发送 `181#0188134C1D010000` 检查归一化中心 `(0.5,0.75)`，它不续速度时效，也不驱动车辆。

Jetson 当前 `damiao_serial_fixed` 使用 USB 侧 921600、模块 CAN 预期 1 Mbit/s；配置中的码率只是预期值，程序不设置或验证模块实际码率。适配器同一时刻只能有一个所有者，独立测试发送程序与 Jetson bridge 不要同时打开同一个设备。

当前 Jetson bridge 在链路就绪时每 10 ms 提交 `0x180/181/182`，发帧与视觉回调分开。`/cmd_vel` 需要由外部持续发布；最后一条速度超过 0.3 s 后，bridge 改发无效 `0x180`。CAN 断流则由 C 板按最后一条有效帧的实际接收时刻执行 250 ms 超时。联调前核对 Jetson 实际运行版本，并在 CAN 侧确认帧的有效位和到帧间隔。

当前 bridge 每 10 ms 重发三种周期帧，含无效坐标；有效坐标按源帧时间去重并检查有效期。

三电机控制约 300 帧/s。若每命令返回一帧，反馈再约 300 帧/s。Jetson bridge 每 10 ms 最多做 32 次非阻塞接收尝试，无数据时提前结束，为共线流量和适配器通知留出余量。实际转发量、到帧间隔和总线端接仍需实板验证。

## 自动搬运

### 舵机接线和 CubeMX

大疆 C 板的 J16 上，C1 是 PE9（AF1，TIM1_CH1），A 是 PGND，B 是 VCC_5V_M。核对实物位号和舵机的电源、电流及信号要求。舵机独立供电时与 C 板共地，外部正电源与 J16 的 B 脚分开。PA8 已用于 I2C3 SCL。

已按用户授权用 CubeMX 生成 TIM1_CH1，保持 `JETSON_AUTO_ENABLED=0`。`tim.c/.h` 包含 `htim1`，`main.c` 在启动 RTOS 前调用 `MX_TIM1_Init()`，PE9 使用 AF1。保留既有任务的 As weak 设置与 Keep User Code。

| CubeMX 项 | 设定 |
|---|---|
| TIM1 与引脚 | Internal Clock、PWM Generation CH1、PE9 AF1 |
| 时基 | APB2 定时器 168 MHz，向上计数，PSC=167，ARR=19999，RCR=0 |
| CH1 | PWM Mode 1、高极性、Pulse=0 |
| DMA / NVIC | TIM1 不启用 DMA 和中断 |

当前工作区 APB2 定时器为 168 MHz，此配置每计数 1 μs，PWM 周期 20 ms（50 Hz）。以后改时钟时须按实际定时器频率重算 PSC。默认标定脉宽为 0，初始化不会启动 PWM。

### 实车标定

将物资框中心或安全区四顶点平均值的像素坐标分别除以图像宽、高，即可在现场录帧里离线量到设定点。联调时也可读对应 CAN 帧的两个定点值，再除以 10000 核对。

目标和安全区均使用用户指定的原始定点中心 `5000,7500`，两轴各允许 ±100。更换相机位置后需在夹取点和安全区推送起点复核这组参数。机械距离和开合端点仍待标定。

| 宏 | 实测和填写内容 |
|---|---|
| `JETSON_AUTO_TARGET_X`、`JETSON_AUTO_TARGET_Y`、`JETSON_AUTO_TARGET_TOL_X`、`JETSON_AUTO_TARGET_TOL_Y` | 把车和目标摆到可夹取位置，测 `0x181` 对应的归一化中心，按可夹取范围填写各轴允许误差。 |
| `JETSON_AUTO_SAFETY_X`、`JETSON_AUTO_SAFETY_Y`、`JETSON_AUTO_SAFETY_TOL_X`、`JETSON_AUTO_SAFETY_TOL_Y` | 把车摆到安全区的推送起点，测 `0x182` 对应的中心和到位窗口。 |
| `JETSON_AUTO_X_TO_LEFT`、`JETSON_AUTO_Y_TO_FORWARD` | 架空分别确认横移和前后方向。按相机实际方向填 `+1` 或 `-1`。 |
| `JETSON_AUTO_APPROACH_GAIN`、`JETSON_AUTO_APPROACH_SPEED_MPS`、`JETSON_AUTO_SEARCH_WZ_RAD_S` | 标定视觉跟随增益、平移限速和丢目标时的车体转向角速度。 |
| `JETSON_AUTO_DISTANCE_SPEED_MPS`、`JETSON_AUTO_ROI_BACK_M`、`JETSON_AUTO_RECOVERY_BACK_M` | 按场地测定定距限速、检查前退距和拒绝恢复退距。盲推及放货后退按限时速度执行。 |
| `JETSON_AUTO_OUT_TURN_DEG`、`JETSON_AUTO_RETURN_TURN_DEG` | 当前均为 180°。去程按目的区左转或右转，回转取相反方向。 |
| `TRANSPORT_SERVO_OPEN_US`、`TRANSPORT_SERVO_CLOSED_US`、`JETSON_AUTO_SERVO_OPEN_MS`、`JETSON_AUTO_SERVO_CLOSE_MS` | 按舵机规格及机构行程实测两端高电平脉宽（μs）和带载开合时间（ms）。先脱开连杆核对方向，再接机构检查有效行程。 |

未标定参数留零，继续使用 `JETSON_AUTO_ENABLED=0`。全项标定后才设为 `1`。舵机由脉宽和动作时间控制，实车需目视核实夹稳和释放。盲推和放货后退均以两后轮目标各 10 rad/s 的直行速度执行 1.2 s；按轮半径 0.035 m 和三轮几何换算，车体目标约为 0.404 m/s。计时含加速，随后停车并等三轮停稳；航向修正会改变各轮瞬时目标。其余定距动作以指令速度积分规划，实际行程需用车上标记或量尺复测。

### `/cmd_vel` 与 CMSIS-DAP 手动标定

这段流程用于允许实车动作时的标定。要求机构保持静止的联调只读状态和已有报文，不启动会向物理 CAN 发送周期帧的 bridge，不发布 `/cmd_vel`，也不写舵机脉宽。

烧录 `JETSON_AUTO_ENABLED=0` 的 Debug 固件，先架空检查轮子方向，脱开舵机连杆核对方向。Jetson 使用正常 `bridge_node`，保留它对 USB 转 CAN 的独占。底盘指令由一个发布器以 50 Hz 发到 `/cmd_vel`。保持全零直到 `jetson_control_state=2`（READY），短发单轴小速度，再用同一个发布器立即恢复全零并等车停稳。前、左、逆时针分别为 `linear.x`、`linear.y`、`angular.z` 的正方向。用 `linear.x=0.05` 试前进时，两后轮目标约为 1.24 rad/s。每次只改一轴，按图像中心实际移动核对方向。若中断发布，按手动速度联调的超时规则停车，恢复时重新从全零准备。

#### 运行时读写 RAM

按 [CLion 下载与调试](CLION_DEBUG.md) 接 CMSIS-DAP。下载后让板持续运行，退出 CLion 调试会话再打开 OpenOCD，避免两个程序同时占用探针。使用与板内程序**完全相同**的 ELF 离线解析地址。这里的 GDB 只读 ELF，无需连接目标。

```powershell
$gdb = 'C:/Program Files/Arm/GNU Toolchain mingw-w64-x86_64-arm-none-eabi/bin/arm-none-eabi-gdb.exe'
& $gdb -q -nx -batch project/build/Debug/project.elf `
  -ex 'p/x &jetson_control_state' -ex 'p/x &jetson_calibration_pulse_us' `
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

舵机校准只在车已实际停稳、`READY` 或 `DISARMED` 时操作。对 `volatile uint16_t jetson_calibration_pulse_us` 使用半字写 `mwh <PULSE> <脉宽μs>`，再以 `mdh <PULSE> 1` 读取。按舵机规格在固件配置的校准范围内小步调整，默认 1000–2000 μs。移动中提交的值会被丢弃，停稳后需重新写入。确认开合端点及带载动作时间，结束时写 `mwh <PULSE> 0` 关闭 PWM。调试器只写这一脉宽变量，状态和坐标只读。

### 机械动作与 CAN 握手

1. 上电静置，等待 Jetson 心跳与 IMU 就绪。使能后确认三轮停稳，张开夹爪，跟随 `0x181`。目标丢失时按标定角速度原地旋转搜寻。
2. 进入目标窗口并停稳后，用通过新鲜度核验的窗内坐标确认夹取点，再闭合夹爪并等待闭合时间。定距后退并停稳，再发 `0x200`（DLC 1，`01`）。先确认 `0x190` 受理，再等待 `0x191` 通过及同侧 `0x193` 授权。
3. 按授权方向转 180°并停稳，发 `0x203`（DLC 5，`01` 加小端 token）。等 `0x192` 命令 1 受理，再跟随新的 `0x182`。安全区中心丢失时停车等待。
4. 进入安全区窗口并停稳，连续两个通过新鲜度核验的窗内坐标包才能触发 `0x204`（DLC 5，同 token）。横纵两轴分别记录最近一次越界，同一轴在坐标有效期内再次同向越界时恢复微调。等命令 2 受理，再限时盲推。停稳后打开夹爪，等待释放时间，限时后退并停稳，再发 `0x206`（DLC 5，同 token）。
5. 等命令 4 受理后反向回转 180°，停稳后发 `0x205`（DLC 1，`01`）。收到命令 3 受理后重新寻找目标。回执丢失重试时，配套 Jetson 若已确认处于寻找模式，可直接重发寻找授权。
6. 到达明确被拒绝（`190#00`），或到达已受理但 ROI 明确未通过（`191#010000`）时，停稳后打开夹爪，按恢复退距后退，再发 `0x205`。

握手等待期间可重发同一请求，机械动作只执行一次。发送邮箱等待上限为 3 s。首次提交后按接收中断记录的到达时刻核验回执，期限为 3.45 s，覆盖 Jetson 的 3 s 服务预算以及调度、队列和一次重报余量。重报沿用首次提交期限。等待 0x192 时按当前命令及 token 匹配，保留首个受理回执的到达时刻。握手超时、心跳失效或电机反馈异常时停车并失能，保留夹爪当时的 PWM。

### 实车验收顺序

1. 车轮离地，先用手动固件核对三个电机方向、停稳和断流。检查 CAN 端接及帧间隔。准备一个可立即断开电机动力的开关。
2. 完成 TIM1 生成和标定，编译自动固件并核对烧录版本。首次自动上电仍架空并保持 IMU 静止，核对 J16 的 20 ms 周期和开合脉宽。检查未见目标时的旋转方向，再用两种视觉中心确认横移、前后方向和到位停车。
3. 在清空的场地用低速和空载先走完整个流程。CAN 侧核对五种上行帧的时序和 token，测量两段定距和两段限时动作，确认每次转向或退距结束后才出现对应握手。再用实际物资确认夹稳、推入、释放和撤回。
4. 单独复测 ROI 拒绝、丢目标、丢安全区及 CAN 断流。恢复前确认车已停稳，夹爪附近无人。

## 离线验证入口

```powershell
python tests/run_native.py --temp-dir .
python -B tests/check_jetson_codec.py --temp-dir .
cmake --preset Debug -S project
cmake --build project/build/Debug
cmake --preset Debug-9axis -S project
cmake --build project/build/Debug-9axis
```

主机编译器可加 `--cc` 指定。协议互验可用 `--jetson-root` 指定 Jetson 目录，加载当前 `tools/source/can_codec.py`（兼容旧版 `tools/can_codec.py`），再把生成的报文交给本库实际的 C 解析器。这些检查在电脑端离线执行。

离线回归用于验证报文、流程和固件编译。TIM1 已生成，自动搬运仍待实车标定、烧录和上车验收。

## 坐标时效和撤销

目标与安全区坐标分别保留无效事件的计数。无效帧即使被同一调度周期内的有效帧覆盖，也会先撤销对应的视觉控制；目标丢失按现有模式搜索，安全区丢失停车，之后才接纳新坐标。源帧时间低 24 位用于去重与跟踪，Jetson 的坐标源帧龄上限为 250 ms。到位停稳后等待 270 ms，再按状态机条件核验新鲜坐标；安全区到位需要两帧不同源帧。

MIT 控制帧拒绝非有限数、超出电机协议范围的参数，以及与使能、失能等保留指令重合的编码。拒绝发送不代表物理电机已停车，仍须检查底盘故障处理和实际反馈。

## 历史记录

[2026-11-08 联调记录](JETSON_CAN_HISTORY.md)保留当时的结果和参数，只能用于追溯；本分支的实际设备表现仍待重新验证。
