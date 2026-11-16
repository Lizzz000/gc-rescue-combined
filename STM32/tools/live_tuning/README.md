# 三轮底盘现场调参

仅在连接实车时使用。正常 `Debug` 构建不启用 SWD 命令入口；专用构建开机失能，只接受显式、有限时长动作。`test_task` 仍是唯一的底盘控制调用者。

## 构建和连接

在 `STM32/project` 用现有 ARM 工具链配置独立目录：

```sh
cmake --preset Debug -B build/Live-Tuning -DCHASSIS_TUNING_ENABLED=ON
cmake --build build/Live-Tuning
```

先备份原板内 Flash，再烧录并校验 `build/Live-Tuning/project.elf`。OpenOCD 使用现有 `openocd-daplink.cfg`，Tcl 端口仅绑定 `127.0.0.1:6666`；读写 RAM 时保持 MCU 运行，不打断点、不 halt。

主机脚本只依赖 Python 标准库及 `arm-none-eabi-nm`。以下命令从仓库根目录运行，`--elf` 必须指向刚烧录的同一份 ELF；每次连接先逐字节核对全部 Flash 加载段（含 `.data` 初值），版本不匹配或初始状态非法时不写任何 RAM。每次日志路径必须不同，以免覆盖实测记录。

```sh
python STM32/tools/live_tuning/swd_session.py --elf STM32/project/build/Live-Tuning/project.elf --describe
python STM32/tools/live_tuning/swd_session.py --elf STM32/project/build/Live-Tuning/project.elf --read
python STM32/tools/live_tuning/swd_session.py --elf STM32/project/build/Live-Tuning/project.elf --command 2 --reg 1
```

寄存器 1 为 Kt，21/22/23 为位置、速度、力矩协议量程。必须检查三台结果与本工程的 ±12.5 rad、±40 rad/s、±1/±1/±2 N·m 一致后再比较动作。Kt 读到 0 表示采用驱动辨识的理论值，不能据此判为未配置；1.8 kg 车重不能确定电机 Kt。依据本机达妙 V1.4 手册 p10/p40 和[官方 SDK](https://github.com/dmBots/motor-sdk/blob/main/Python%E4%BE%8B%E7%A8%8B/u2can/DM_CAN.py)。本工具只读电机寄存器，不提供内部参数写入或 Flash 保存。

## 调参和短动作

每次只比较一个变量。先记录原 Kd、零前馈的前进/后退和双向旋转，再比较 Kd，最后比较前馈；每段运动结束都确认停稳并失能。

```sh
python STM32/tools/live_tuning/swd_session.py --elf STM32/project/build/Live-Tuning/project.elf --command 3 --kd 0.20 0.25 0.20 --ff-gain 0 --friction 0 0 0
python STM32/tools/live_tuning/swd_session.py --elf STM32/project/build/Live-Tuning/project.elf --command 4 --vx 0.25 --duration-ms 800
python STM32/tools/live_tuning/swd_session.py --elf STM32/project/build/Live-Tuning/project.elf --command 4 --vx -0.25 --duration-ms 800
python STM32/tools/live_tuning/swd_session.py --elf STM32/project/build/Live-Tuning/project.elf --command 4 --wz 0.5 --duration-ms 800
python STM32/tools/live_tuning/swd_session.py --elf STM32/project/build/Live-Tuning/project.elf --command 4 --wz -0.5 --duration-ms 800
python STM32/tools/live_tuning/swd_session.py --elf STM32/project/build/Live-Tuning/project.elf --command 1
```

专用测试限速 0.25 m/s、0.5 rad/s，平移与旋转互斥，单段 100–1500 ms。配置限 Kd 0.05–0.5，惯性前馈比例 0–1，单轮摩擦项 0–0.03 N·m；运行中不能改配置。修改只在 RAM 中生效，复位恢复源代码默认值。

平移惯性项采用 `F=1.8*a`，按三轮几何分配为轮端力矩；例如前进加速度 0.45 m/s² 时，电机 ID1/2/3 为 `0/-0.01637/+0.01637 N·m`。摩擦项只用于平移并平滑过零，纯旋转前馈为零，总前馈限幅 ±0.05 N·m。MIT 已接受 N·m，不再乘除 Kt。显式停车、失能和故障清零前馈。

2026-11-16 落地对照后默认值为 Kd `0.20/0.25/0.20`、惯性比例 `1`、摩擦系数 `0/0.015/0.015 N·m`。三台 Kt 均读为0并保持原值。最终配置重复两组前进、后退、左转、右转，均确认停稳失能，用户确认前后与旋转平顺。测试目标为0.25 m/s、0.5 rad/s，未改变现有公开接口上限。数据见 `STM32/outputs/live_tuning_20261116/`。

主机每 100 ms 更新心跳，板端 300 ms 超时制动；动作仍有独立时限。启程需要原有三轮新鲜静止反馈；停车先确认三组新反馈，再失能并确认三台新的 state=0 回复。制动或失能确认超时会报错，不把软件失能当实车停车证明。异常退出发送 STOP，TCP 失败只允许重连停止，不重试运动。

只读诊断可在缺失某台失能回执后继续查询在线电机，以保留部分结果；这不放宽任何运动准入。诊断构建记录所有标准数据 CAN ID 的最近 8 帧到 `can_rx_diag`，正常构建仍只接收反馈 ID0。

JSONL 包含请求、参数回执、轮位置/速度/估计力矩、IMU、温度、目标、前馈和结果。轮速与力矩是驱动反馈，不是独立地面测速或测力。板端遥测每100 ms发布（10 Hz），主机轮询上限25 Hz；分析去重快照并使用实际采样间隔，不能用此数据判断高频振动。

验证入口：`python STM32/tests/run_native.py` 及 `python -m unittest discover -s STM32/tests -p test_swd_session.py`。实车测试后应将经过对照确认的参数写入正式源码，重新构建并校验正常手动固件；不得将未完成测试写成已标定。
