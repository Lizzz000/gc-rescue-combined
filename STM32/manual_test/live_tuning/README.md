# 三轮底盘现场调参

仅在连接实车时使用。正常 `Debug` 构建不启用 SWD 命令入口；专用构建开机失能，只接受显式、有限时长动作。正常结束按底盘斜坡减速并复核停稳，断流或 STOP 仍立即进入保护。`test_task` 仍是唯一的底盘控制调用者。

## 构建和连接

在 `STM32/project` 用现有 ARM 工具链配置独立目录：

```sh
cmake --preset Debug -B build/Live-Tuning -DCHASSIS_TUNING_ENABLED=ON
cmake --build build/Live-Tuning
```

先备份原板内 Flash，再烧录并校验 `build/Live-Tuning/project.elf`。OpenOCD 使用现有 `openocd-daplink.cfg`，Tcl 端口仅绑定 `127.0.0.1:6666`；读写 RAM 时保持 MCU 运行，不打断点、不 halt。

主机脚本只依赖 Python 标准库及 `arm-none-eabi-nm`。以下命令从仓库根目录运行，`--elf` 必须指向刚烧录的同一份 ELF；每次连接先逐字节核对全部 Flash 加载段（含 `.data` 初值），版本不匹配或初始状态非法时不写任何 RAM。每次日志路径必须不同，以免覆盖实测记录。

```sh
python STM32/manual_test/live_tuning/swd_session.py --elf STM32/project/build/Live-Tuning/project.elf --describe
python STM32/manual_test/live_tuning/swd_session.py --elf STM32/project/build/Live-Tuning/project.elf --read
python STM32/manual_test/live_tuning/swd_session.py --elf STM32/project/build/Live-Tuning/project.elf --command 2 --reg 1
```

寄存器 1 为 Kt，21/22/23 为位置、速度、力矩协议量程。必须检查三台结果与本工程的 ±12.5 rad、±40 rad/s、±1/±1/±2 N·m 一致后再比较动作。Kt 读到 0 表示采用驱动辨识的理论值，不能据此判为未配置；1.8 kg 车重不能确定电机 Kt。依据本机达妙 V1.4 手册 p10/p40 和[官方 SDK](https://github.com/dmBots/motor-sdk/blob/main/Python%E4%BE%8B%E7%A8%8B/u2can/DM_CAN.py)。本工具只读电机寄存器，不提供内部参数写入或 Flash 保存。

## 调参和短动作

每次只比较一个变量。比较 Kd 时，各组使用相同的前馈和运动设定，分别记录前进、后退和双向旋转；每段运动结束都确认停稳并失能。

```sh
python STM32/manual_test/live_tuning/swd_session.py --elf STM32/project/build/Live-Tuning/project.elf --command 3 --kd 0.22 0.27 0.22 --ff-gain 1 --friction 0 0.015 0.015
python STM32/manual_test/live_tuning/swd_session.py --elf STM32/project/build/Live-Tuning/project.elf --command 4 --vx 0.25 --duration-ms 800
python STM32/manual_test/live_tuning/swd_session.py --elf STM32/project/build/Live-Tuning/project.elf --command 4 --vx -0.25 --duration-ms 800
python STM32/manual_test/live_tuning/swd_session.py --elf STM32/project/build/Live-Tuning/project.elf --command 4 --wz 0.5 --duration-ms 800
python STM32/manual_test/live_tuning/swd_session.py --elf STM32/project/build/Live-Tuning/project.elf --command 4 --wz -0.5 --duration-ms 800
python STM32/manual_test/live_tuning/swd_session.py --elf STM32/project/build/Live-Tuning/project.elf --command 1
```

专用测试限速 0.25 m/s、0.5 rad/s，平移与旋转互斥，单段 100–1500 ms。配置限 Kd 0.05–0.5，惯性前馈比例 0–1，单轮摩擦项 0–0.03 N·m；运行中不能改配置。修改只在 RAM 中生效，复位恢复源代码默认值。

平移惯性项采用 `F=1.8*a`，按三轮几何分配为轮端力矩；例如前进加速度 0.45 m/s² 时，电机 ID1/2/3 为 `0/-0.01637/+0.01637 N·m`。摩擦项只用于平移并平滑过零，纯旋转前馈为零，总前馈限幅 ±0.05 N·m。MIT 已接受 N·m，不再乘除 Kt。显式停车、失能和故障清零前馈。

2026-11-16 落地对照后默认值为 Kd `0.20/0.25/0.20`、惯性比例 `1`、摩擦系数 `0/0.015/0.015 N·m`。三台 Kt 均读为0并保持原值。最终配置重复两组前进、后退、左转、右转，均确认停稳失能，用户确认前后与旋转平顺。测试目标为0.25 m/s、0.5 rad/s，未改变现有公开接口上限。数据见 `STM32/outputs/live_tuning_20261116/`。

2026-11-19 保持惯性和摩擦前馈，复测低速前后与双向旋转，将默认 Kd 更新为 `0.22/0.27/0.22`。同场对照数值及组合返航结果见 [底盘控制参数](../../docs/DJI_C_CUBEMX.md)。

主机每 100 ms 更新心跳，板端 300 ms 超时制动；动作仍有独立时限。启程需要原有三轮新鲜静止反馈；停车先确认三组新反馈，再失能并确认三台新的 state=0 回复。制动或失能确认超时会报错，不把软件失能当实车停车证明。异常退出发送 STOP，TCP 失败只允许重连停止，不重试运动。

只读诊断可在缺失某台失能回执后继续查询在线电机，以保留部分结果；这不放宽任何运动准入。诊断构建记录所有标准数据 CAN ID 的最近 8 帧到 `can_rx_diag`，正常构建仍只接收反馈 ID0。

JSONL 包含请求、参数回执、轮位置/速度/估计力矩、IMU、温度、目标、前馈和结果。轮速与力矩是驱动反馈，不是独立地面测速或测力。板端遥测每100 ms发布（10 Hz），主机轮询上限25 Hz；分析去重快照并使用实际采样间隔，不能用此数据判断高频振动。

验证入口：`python STM32/tests/run_native.py` 及 `python -m unittest discover -s STM32/tests -p test_swd_session.py`。实车测试后应将经过对照确认的参数写入正式源码，重新构建并校验正常手动固件；不得将未完成测试写成已标定。

偏航 ADRC 的离线对照、实车日志回放和 IMU 启动诊断见 [ADRC 初测](adrc/README.md)。

## 单独里程计测试

同一专用构建增加命令 5（有限时静态采样）与 6（前后定距闭环）。两者均先使能、确认停车、清平移原点，再等待新的有效里程计基线。命令 6 调用正式 `Chassis_Distance_Start`，按反馈位移完成；到期、反馈失效和提前停车均报错。动作之后仍须确认停车及三台新失能回执。不要并行运行多个控制脚本。

```sh
# 静态采样 1 秒；保持零速，自动失能。
python STM32/manual_test/live_tuning/swd_session.py --elf STM32/project/build/Odometry-Live-Test/project.elf --command 5 --duration-ms 1000
# 前进 5 cm，限速 0.05 m/s，独立硬时限 5 秒。
python STM32/manual_test/live_tuning/swd_session.py --elf STM32/project/build/Odometry-Live-Test/project.elf --command 6 --direction 1 --distance-m 0.05 --speed-mps 0.05 --duration-ms 5000 --timeout 8
# 短距通过且前方至少有 1.5 m 净空后，前进 1 m，限速 0.10 m/s。
python STM32/manual_test/live_tuning/swd_session.py --elf STM32/project/build/Odometry-Live-Test/project.elf --command 6 --direction 1 --distance-m 1 --speed-mps 0.10 --duration-ms 20000 --timeout 23
```

方向 1 为前进、2 为后退。测试入口限制距离 0.01–1.00 m、速度 0.02–0.10 m/s、定距硬时限 100–20000 ms；静态保持 100–1500 ms。主机定距等待时限须大于硬时限加 2 秒且不超过 25 秒，其他命令仍不超过 15 秒。原命令 4 的 1500 ms 速度动作上限保持不变。增加距离与硬时限不改变 300 ms 失联制动，也不改变正式底盘控制参数。

静态命令在零目标下采样，允许采样期间短暂丢失停车资格；期满后最多 500 ms 等待原三组新反馈停稳，再保存有效终点并失能。采样及确认期间估计漂移超过 10 mm，或反馈、IMU、里程计失效，仍会中止；不再因任一采样时刻未累计三组停车反馈而立即失败。命令 4 也须在起步前等到有效里程计，但不清零累计位置。

USART6 为 PG14/TX、PG9/RX，115200、8N1。测试构建由底盘任务独占发送 56 通道 JustFloat，通常 10 Hz；正常构建继续发送原来的三姿态通道。DAP-Link 的电脑 COM 号由 Windows 分配，不能把 USART6 等同于 COM6。串口只输出遥测，命令与心跳仍走 DAP-Link SWD。

通道 0–39 沿用原调参快照；40–47 为当前里程计 `x/y/yaw/vx/vy/wheel_wz/valid/age_ms`；48–55 为动作结束前保存的 `x/y/yaw/valid/stopped/elapsed_ms/progress_m/goal_m`。位移单位 m、航向 rad。结束后当前 `valid=0` 是失能语义，验收使用保留的终点证据和三台失能回执。主机按 ELF 符号长度核对 56 通道 ABI，旧的 40 通道 ELF 需要匹配旧版本工具。

轮编码器里程计只能验证估计值闭环；实际地面距离、打滑与航向偏移需尺量和现场观察。完成测试后恢复正常手动固件，并记录板端结果、外部测量及未完成项目。

## 随机搬运与返回原点

离线模型入口为 `simulate_return.py`，复用真实 C 里程计算法，输出固定种子的路线及独立模型真值。它不接硬件，也不是正式底盘控制器仿真；示例及已验证结果见 [里程计记录](../../docs/ODOMETRY.md)。

实车入口 `random_return_session.py` 默认仅描述计划。明确执行时先核对完整 Flash/ELF，保持一个写入锁；开头仅调用一次命令 5 建立原点，随后用命令 4 的有限时前后/旋转脉冲保留累计里程计，每段必须正常停稳并确认三台失能。出发包含 8 段前后平移、2 次小角转向及停留；返航根据累计估计位置、当前航向选前进或倒车，最后恢复初始航向。此为现场实验编排，不接视觉或实际夹爪。

```sh
python STM32/manual_test/live_tuning/random_return_session.py --describe
python STM32/manual_test/live_tuning/random_return_session.py --describe --profile accelerated
# 仅在已确认场地、起点、探针和串口连接后显式执行，日志与停止文件使用新路径。
python STM32/manual_test/live_tuning/random_return_session.py --execute --elf STM32/project/build/Odometry-Live-Test/project.elf --log STM32/outputs/random_trial/mission.jsonl --stop-file STM32/outputs/random_trial/STOP
# 已确认更长净空并要求加速时，显式选择 accelerated；默认仍为 low。
python STM32/manual_test/live_tuning/random_return_session.py --execute --profile accelerated --elf STM32/project/build/Odometry-Live-Test/project.elf --log STM32/outputs/random_accelerated/mission.jsonl --stop-file STM32/outputs/random_accelerated/STOP
```

创建 `--stop-file` 指定文件可请求停止；已存在时拒绝启动，不自动删除。两种配置均先静态采样 1000 ms，初始化后最多 60 秒、24 个动作、12 个返航脉冲；每个运动脉冲仍不超过 1500 ms。

| 配置 | 线/角速度上限 | 出发预测区域（初始车体坐标） | 运行估计围栏 | 全程线/角指令积分预算 |
| --- | --- | --- | --- | --- |
| `low`（默认） | 0.10 m/s、0.25 rad/s | x∈[0,0.50] m、\|y\|≤0.15 m | x∈[−0.10,0.65] m、\|y\|≤0.30 m | 1.2 m、π rad |
| `accelerated` | 0.20 m/s、0.50 rad/s | x∈[0,1.10] m、\|y\|≤0.25 m；至少前探 0.50 m | x∈[−0.10,1.30] m、\|y\|≤0.35 m | 3.0 m、2π rad |

`accelerated` 仍包含 8 段前后平移和 2 次转向；转向指令积分各约 25–35°，实际转角由 IMU 反馈读取。种子 20261117 的出发线积分为 1.549 m，预测最远前探 0.645 m，末点约 (0.446,−0.068) m；这是速度指令预测，不是实车测量。返航远处采用 0.15–0.20 m/s，近原点降至 0.06 m/s；转向使用 0.50 rad/s，接近目标角度时降至 0.25 rad/s。默认 `low` 的路线、限速和预算保持原值。

运行与仍使能的制动阶段检查有效里程计；每段需新有效样本，平移沿请求方向至少有 2 mm 进展。任何反馈、停车、区域或预算错误立即中止，原错误保留，不因最终 STOP 成功改写为任务成功。

终点按估计位置 2 cm、航向 5°评估，并明确记录 `real_ground_return_verified=false`。围栏也是估计值，不代替现场净空或地面位置测量；实际是否回原点须与起点标记核对。测试命令 `python -m unittest discover -s STM32/tests -p 'test_*return*.py'` 覆盖离线模拟、路线方向、预算、故障中止与 STOP 清理。

## 连续返航并保持使能

命令 7 接受 `[世界目标x, y, yaw, 线限速, 角限速, 硬时限毫秒, 0, 0]`，单位为 m、rad、m/s、rad/s。起点到目标最多 1.5 m，线限速 0.02–0.20、角限速 0.10–0.50，硬时限 100–20000 ms。周期控制使用有效轮编码器里程计与 IMU，通过 `Point_Plan` 的无障碍最短直线、最短角差及制动距离生成车体全向速度；保持累计坐标，途中不拆段失能。显式角速度接口使 `wz=0` 表示零角目标，避免进入旧接口的隐式锁航向 PID。

进入 1 cm / 4°内层范围后发零目标，最多 1500 ms 确认原三组停车反馈，最终按 2 cm / 10°验收。此窗口用于等待零速后的反馈尾迹收敛，不改变轮速门限；STOP 和故障仍沿用 500 ms 制动期限。成功进入 HOLD：三台保持使能、目标速度为零，不自动重新纠偏，正常脚本退出无需心跳。HOLD 继续检查反馈、IMU、里程计和距起步点 1.8 m 的固件估计范围；原连续返航主机默认另限1.3 m，扩大视觉实验显式使用1.8 m。显式命令 1 或故障仍制动失能。运动与到点确认期间仍有 300 ms 心跳和硬时限。

PREPARE通过后，首次速度提交若因停车资格短暂撤销而拒绝，会保持零目标并等待最多1000ms；首次接受之后再被拒绝则立即中止。等待仍检查反馈、IMU、里程计、心跳及动作总期限，不延长总时限。

`continuous_return_session.py` 是跨固件更新的单次返航入口，必须提供旧随机任务日志和更新前只读 checkpoint。它用旧冻结坐标同一时刻的航向计算车体相对原点，再映射到新固件参考系；不会重跑随机路线，也不会将新零点充当旧原点。此方式假设更新期间没有物理搬动车辆；六轴静置漂移和真实回零误差须另行记录。

```sh
python STM32/manual_test/live_tuning/continuous_return_session.py --describe --mission-log path/to/mission.jsonl --checkpoint-log path/to/checkpoint.jsonl
python STM32/manual_test/live_tuning/continuous_return_session.py --execute --mission-log path/to/mission.jsonl --checkpoint-log path/to/checkpoint.jsonl --elf path/to/exact-new.elf --log path/to/new-return.jsonl --stop-file path/to/STOP
```

本入口要求新固件未接受过命令，先逐字节核对 Flash；只发送一次命令 7。成功后只读观察保持状态，不发送失能 STOP；任何执行或保持检查失败仍执行 STOP，且不自动重试运动。明确要求保持使能的现场实验结束后，可保留该专用固件和零目标 HOLD；此时不要再烧录正常固件，否则复位会失能。

## 整轮连续随机运动

`continuous_random_session.py` 从现有命令 7 的使能零速 HOLD 开始，将当前有效里程计位姿记录为整轮原点；沿同一坐标系依次执行目标，再自动返原点。每个目标完成后保持使能，下一目标直接接续，不重置里程计，不执行中间失能或固件更新。默认只描述路线。

```sh
python STM32/manual_test/live_tuning/continuous_random_session.py --describe
python STM32/manual_test/live_tuning/continuous_random_session.py --execute --elf path/to/exact.elf --log path/to/new-mission.jsonl --stop-file path/to/STOP
```

默认 seed=20261117 为 10 个路点加原点，共 11 次命令 7，规划路程 5.948 m、最长单段 0.787 m、航向约 ±45°–60°，预计约 55 秒。线/角限速为 0.20 m/s、0.50 rad/s；全程最多 90 秒、12 个命令、累计请求距离 7 m、累计请求角度 6π。初始车体坐标中的估计围栏为 x∈[−0.10,1.30] m、|y|≤0.40 m 且半径≤1.35 m。每个计划段不超过 0.95 m，该脚本实时起点到目标仍限1 m，小于当前固件1.5 m上限。

成功返原点后只读观察零速使能状态，并以当前估计位姿检查 2 cm / 10°；失败仍停车失能，不自动重跑。规划路程、里程计估计及实际地面路程须分别记录。主机检查：`python -m unittest discover -s STM32/tests -p test_continuous_random_session.py`。

用户要求每个动作持续更久时，显式选择 `--profile long`：8 段较长复合运动再返航，航向交替为 ±90°，中间各段约转 180°，线/角速度仍为 0.20 m/s、0.50 rad/s。seed=20261117 的前 8 段距离为 0.88–0.94 m，规划总路程 7.429 m、累计转角 8π，预计 65.8 秒；最后一段返回原点约 0.185 m。此配置使用 120 秒、9 m、9π 总预算，单段 0.95 m 规划上限、原区域围栏和故障停车不变。默认 `standard` 继续使用原 90 秒、7 m、6π 预算。

```sh
python STM32/manual_test/live_tuning/continuous_random_session.py --describe --profile long
python STM32/manual_test/live_tuning/continuous_random_session.py --execute --profile long --elf path/to/exact.elf --log path/to/new-long-mission.jsonl --stop-file path/to/STOP
```

2026-11-17 编码器增量里程计版本连续完成两轮 `long`，各9段、约59.4/58.4秒，最终均保持使能和零目标。用户确认第二轮回到起点附近且动作平顺；估计毫米级偏差不是尺量精度。完整参数、前次失败与固件SHA256见 [里程计实测记录](../../docs/ODOMETRY.md)。

## 模拟视觉坐标接近与最短路径返航

`visual_return_session.py` 默认只输出计划，执行需额外安装`pyserial`并指定接在STM32 USART6上的电脑串口。命令8使用与正式视觉流程相同的Kalman和速度规划；图像坐标按真实编码器/IMU里程计经合成投影生成，加入60ms延迟、15/25q交替噪声和一次1800q异常。它验证控制代码和执行器，不验证真实相机或独立地面定位精度。

```sh
python STM32/manual_test/live_tuning/visual_return_session.py --forward 1.2 --left .6 --turn-deg 60
# 在对应前方/左侧净空已确认后执行，路径必须全新。
python STM32/manual_test/live_tuning/visual_return_session.py --execute --elf path/to/exact.elf --log path/to/mission.jsonl --stop-file path/to/STOP --serial-port COM7 --forward 1.2 --left .6 --turn-deg 60
```

执行路径为可选初始旋转→命令8视觉接近→命令7直线返航，成功保持使能。视觉段16s硬时限、19s主机期限；新坐标250ms失效、心跳300ms失效。只修改测试邮箱，禁止写滤波器、编码器或里程计状态。前方参数0.1–1.2m、侧向±0.6m、初始转角±60°；大距离投影6000q/m，小距离10000q/m，均为合成模型。

命令8期间56通道USART6遥测每40ms发布，其他命令通常100ms。低吞吐DAP读取整包可能超过发布周期，因此本工具以完整串口帧生成坐标，只通过SWD读命令/发布时间与发送控制。串口只读、DTR/RTS关闭；与当前板端时间相差超过120ms的旧USB缓存不接受，坐标源时间不前进也不能续期。日志中的`sample_seq=null`明确表示串口没有固件seqlock序号，`telemetry_tick_ms`标识新帧，不能将其冒充SWD序号。原始串口字节保存为同名`.uart.bin`。

最终版本实测、失败原因及主机/模型边界见 [本轮验证记录](../../docs/ALGORITHM_UPGRADE_20261117.md)。

## 同向30圈YAW实验

专用命令9只接受`[30,2,3,4,3,2,120000,0]`：逆时针连续30圈，每6圈依次2/3/4/3/2rad/s，沿原1.6rad/s²斜坡加减速，末段按制动距离停车。按用户指定，以展开后的AHRS YAW计圈；三轮编码器25rad解绕后的转角与gyro积分分别记录；编码器参考受轮径、中心距和打滑影响，也不是地面真值。

仅调试构建的纯旋转入口接受4rad/s；普通向量、定角转向仍限2rad/s。全过程硬时限120s、平移漂移围栏0.35m、原300ms心跳和反馈检查有效。成功保持使能零目标，故障STOP。执行前必须已安排同向连续旋转的防缠线连接。

```sh
python STM32/manual_test/live_tuning/spin_session.py
python STM32/manual_test/live_tuning/spin_session.py --execute --elf path/to/exact.elf --serial-port COM7 --log path/to/new-spin.jsonl --stop-file path/to/STOP
```

命令9的USART6以40ms周期发送64列：前56列不变，追加编码器累计rad、YAW累计rad、gyro累计rad、请求wz、阶段0–4、目标rad、YAW减编码器rad、轮速正解wz。未进入命令9时为56列；命令9后的STOP/HOLD仍为64列，复测CLI须加`--previous-spin`，复位后恢复56列。正常固件三姿态输出不变。运动循环只写DAP心跳，UART源时钟对照起步前已核验的板端时钟；不执行此前反复超时的整包/标量SWD诊断读取。终点后被动观察3秒，不继续发心跳。结果见 [YAW记录](../../docs/SPIN_YAW_20261117.md)。


同一Horco探针的HID后端已完成30圈实验，启动示例：

```powershell
openocd -f STM32/project/openocd-daplink-hid.cfg -c "bindto 127.0.0.1" -c init
```

运动日志先写本地临时目录，结束后复制到`STM32/outputs`。bulk接口曾出现心跳/读取超时，移除诊断读取、改变日志位置未完全解决；成功记录使用HID，不能据此认定所有连接问题已被根治。主机仍有独立STOP文件和固件300ms心跳期限。

测试固件的`chassis_odometry_last_us/max_us`用1MHz TIM2测量真实里程计入口（含ESKF）耗时；它包含中断抢占，最大值只是该次运行观测值，不是WCET证明。`chassis_heading_debug[6]`依次为yaw、bias、Pθθ、Pbb、YAW增益、轮偏置增益，仅在HOLD后用SWD读取。
