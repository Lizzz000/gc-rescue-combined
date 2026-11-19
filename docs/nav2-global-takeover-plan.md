# Nav2 全局接管修改设计（提案，待批准）

- 文档位置：`docs/nav2-global-takeover-plan.md`
- 性质：仓库工程修改**设计文档**。不实施业务改动，不新增包/目录/协议，不提供可当作现成产物的 launch 或参数文件。
- 日期：2026-11-19（Asia/Shanghai）
- 依据：当前工作树 `/workspace/gc-rescue-user-wip-20261119-215105` 的 AGENTS、README、Jetson/STM32 源码与文档；官方 ROS 2 Jazzy / Nav2 文档（见文末来源）。用户 t199u/t200u：以后由 **Jetson 使用 ROS 2 的 Nav2 及其已有神经网络** 做决策，作为后续启动全局接管的修改依据。
- 目标板卡（用户指定，现场 SKU 仍待与仓库记录核对）：**Jetson Orin Nano 8GB Super 为决策中心**；STM32F407 保持实时电机执行与反馈闭环。

凡标注「提案待批」的内容，均属于未来结构或行为变化，须单独确认后才能改仓库。本文给出有依据的推荐，不是直接实施许可。

## 1. 一句话契约

**Nav2 统管导航底盘，STM32 只做轮闭环与硬件执行；视觉搬运、对位、夹爪是任务交接，不是让 Nav2 包办夹爪，也不是用神经网络端到端开车。** 「最优解」在本文中指：在可测量的任务成功率、路径/时间/碰撞和算力约束下，用经典规划控制加现有感知，选择当时更好的可运行方案。它**不是**全局数学最优，也**不默认**强化学习或端到端驾驶更优。

## 2. 当前实际链路（已查证）

根目录 [README.md](../README.md) 与 [Jetson/docs/README.md](../Jetson/docs/README.md) 写明：面向三轮全向底盘的搬运项目。Jetson 负责视觉、目标选择、运输任务和夹爪；STM32 C 板负责姿态估计、底盘控制和电机驱动。板间为 **USB CDC**（全速 12 Mbit/s，无 ULPI），电机仍接 STM32 **CAN1**。

### 2.1 Jetson 侧现有节点（无 Nav2）

完整自动部署是 **7 个进程、8 个节点**（`bringup_pkg/runtime/core.py`，说明见 Jetson README）。角色如下，均已在源码与 `rescue.launch.py` 中存在：

| 角色 | 可执行文件 | 节点 |
| --- | --- | --- |
| vision | vision_runtime | detect_node、stream_node |
| track | tracking_runtime | track_node |
| select | tracking_runtime | select_node |
| mission | mission_runtime | mission_node |
| target | mission_runtime | target_node |
| safety | mission_runtime | safety_node |
| bridge | bridge_runtime | bridge_node |

`record_pkg`、`teleop_pkg` **不**随 `run:=all` 启动。`teleop_pkg` 是独立工具：经 USB 发送**类型 2** 手动速度，不经过 ROS `cmd_vel`。

已查包依赖（各 `package.xml`）：`rclcpp`、`std_msgs`、`vision_msgs`、`sensor_msgs`（图像）、`interface_pkg` 等。**未发现** `nav2_*`、`nav_msgs`、`tf2`/`tf2_ros`、`sensor_msgs/Imu`、`LaserScan`、`PointCloud2`、`OccupancyGrid` 的生产依赖或话题发布。`sensor_msgs` 在 `vision_pkg` 中用于 `Image` / `CompressedImage`，不是里程计或激光。

启动入口：`Jetson/src/bringup_pkg/launch/rescue.launch.py`，配置为 [rescue.yaml](../Jetson/src/bringup_pkg/config/rescue.yaml) 与 [rescue_advanced.yaml](../Jetson/src/bringup_pkg/config/rescue_advanced.yaml)。仓库根 README 与 Jetson README 的构建示例为：

```sh
source /opt/ros/humble/setup.bash
```

即**当前文档化发行版是 ROS 2 Humble**。`Jetson/src/bridge_pkg/gripper/README.md` 另写过「ROS 2 Jazzy 工作区」，与主说明不一致，列为待确认，不能据此宣称仓库已在 Jazzy 上运行。

### 2.2 现有感知与任务输入（二维，不是地图点）

- 分割：TensorRT 引擎 `package://vision_pkg/models/seg.ad2322286d06b71.trt`，标签 `classes.txt`：green / bright_blue / black / orange（配置编号 0/1/2/3）。`conf_threshold_seg: 0.5`。输出 `vision_msgs/Detection2DArray`（`/detect_node/seg/detections`），经 OC-SORT 跟踪与选择。默认 `publish_masks: false`。
- 场地：`detect` 的 `task: both` 表示具备分割与场地两套后端，**按 `active_task` 切换，不同时推理**。场地后端源码注明为 **OpenCV 紫色边界/几何**，不是第二份已配置的 `.trt` 路径。场地物理尺寸按比赛图纸配置：外框 660×360 mm、黑带宽 20 mm。`rescue_advanced.yaml` 中的 `black_min_depth` / `black_conf_depth` 是黑带色度相对下陷深度，**不是深度相机**。
- 相机：`/dev/video0`，请求 MJPG、`camera_fps: 120`，原图 1920×1080，`frame_id` 默认 `camera`。配置写明「相机实际有效帧率与算法吞吐须分别实测」。`seg_max_fps` / `field_max_fps: 100.0` 是平均上限（0 表示不再额外限速），**不是已测得的稳定帧率**。
- 安全区目标 `SafetyTarget`：`x,y` 是所选区域对角线交点再除以图像宽高的**归一化图像坐标**，无效时不可当运动目标（`interface_pkg/msg/SafetyTarget.msg`）。
- USB 类型 3/4：目标/安全区中心，`x/y:int16`（×10000）加源帧毫秒低 24 位，同样是图像平面中心，由 STM32 做视觉跟随与后续动作（[usb-protocol.md](../Jetson/docs/usb-protocol.md)）。

**结论：现有神经网络与场地几何给出的是图像平面框/中心/掩膜（掩膜默认未发布），不能直接当作 Nav2 的 `map` 目标点。** 未查到仓库内已发布的 `camera_info`、相机内参标定文件或深度/激光话题；若要用几何反投影，须先补证据（内参、外参、地面平面或深度）。

### 2.3 STM32 侧现有执行与里程计（未上送到 ROS）

- 控制任务周期目标 5 ms。公开速度接口 `Chassis_Set_Velocity(vx, vy, wz)`：车体 `vx` 前、`vy` 左，合速度不超过 **0.50 m/s**，`wz` 逆时针、绝对值不超过 **4.0 rad/s**（`chassis.h`）。正常自动运行目标为 0.40 m/s、3.0 rad/s；斜坡与定距加速度见 `CHASSIS_STRAIGHT_ACCELERATION_MPS2`（0.45）与 `CHASSIS_DISTANCE_ACCELERATION_MPS2`（0.75）。
- 里程计在板内：轮角增量 + IMU 航向 ESKF + SE(2) 积分（[ODOMETRY.md](../STM32/docs/ODOMETRY.md)）。`Chassis_Odometry_t`（`chassis_pose.h`）含 `x_m,y_m,yaw_rad,vx_mps,vy_mps,yaw_rate_rad_s,wheel_wz_rad_s,sample_time_us,valid`。世界轴由 **INS 零航向**定义，**不能直接当作相机坐标或场地图坐标**。`sample_time_us` 是 TIM2 控制拍时刻，不是 ROS 时间。结构中**没有协方差**。
- 该文档写明：「位姿当前由 STM32 本地接口读取，**未增加 USB 位姿帧或 ROS `odom` 发布**。」
- 自动固件的 USB 接收（`jetson_link.c` `Jetson_CAN_Receive_At`）处理类型 3–9 等握手与中心帧，`default` 直接忽略。协议写明：**类型 2 手动速度由自动固件忽略**，仅手动固件处理，有效期 250 ms。量化：`int16`，单位 0.001 m/s 与 0.001 rad/s，载荷 7 字节（`link_payload.h`）。
- 上行类型仅 16–21（到达、转向完成、安全到达、重找、投送完成、夹爪/点到达请求），**无 IMU、无里程计、无速度反馈帧**。最大载荷 **8 字节**。

### 2.4 一次运输如何工作（现状）

仍由 STM32 `jetson_auto` 统管底盘动作：使能、转 180°、视觉接近、夹取/ROI 握手、按里程计返航、安全区对准、锁航向限时盲推、松爪退出。Jetson `mission_pkg` + `bridge_pkg` 提供视觉中心、ROI、命令回执和夹爪终态。这是「STM32 导航 + Jetson 感知/夹爪」而不是「Nav2 导航」。

## 3. 未来目标链路（提案待批）

默认方案（推荐，待批）：

```
感知（现有 YOLO-Seg + 场地几何 + 跟踪选择）
    → 任务调度（在现有 mission 上扩展或另批新包）
        → Nav2：地图定位 + 全局规划 + 局部全向控制
            → 唯一 cmd_vel 入口（或等价体速度契约）
                → USB 速度/状态桥（须扩展协议，见第 8 节）
                    → STM32：校验、斜坡、轮逆解、CAN1 MIT、反馈闭环
    夹爪 / ROI / 对位：任务层交接，不经 Nav2 控制器
```

| 职责 | 现状 | 接管后（提案） |
| --- | --- | --- |
| 全局/局部导航、避障、路径 | STM32 视觉跟随、定距、返航、盲推 | Jetson 上 Nav2（规划 + 局部控制） |
| 体速度生成 | STM32 内部控制，或手动类型 2 | Nav2 controller 唯一输出 |
| 轮闭环、限幅、斜坡、CAN | STM32 | **保留 STM32**，不把电流/轮环放到 Linux |
| 目标检测/分割/跟踪 | Jetson 现有网络 | **保留**，作为任务目标与语义提示的来源，不替代几何避障 |
| 场地分区几何 | Jetson OpenCV Field | 可继续为任务提供安全区语义；是否纳入静态图层待批 |
| 夹爪 S288 | bridge 类型 21/9 | **仍由任务/bridge 交接**，Nav2 不包办开合 |
| 手动遥控 | teleop 类型 2，与自动固件互斥 | 与导航、对位互斥，同一时刻一个写手 |

**明确排除的默认方案：** 用神经网络直接输出 `cmd_vel` 端到端开车；把强化学习控制器当作默认可替换 Nav2 的「更优」方案；让 Nav2 Behavior Tree 直接驱动夹爪舵机。这些若要做，必须另立提案并证明优于本文的经典组合。

## 4. 经典 Nav2 与 AI 感知如何分工

Nav2（官方 Jazzy 概念，见文末）需要：

1. TF：`map → odom → base_link → 传感器架`（REP-105，Nav2 Jazzy「State Estimation / Transformations」）。
2. `nav_msgs/Odometry`：位姿在 `header.frame_id`（通常 `odom`），速度在 `child_frame_id`（通常 `base_link`），并含协方差。
3. 局部/全局 costmap 与控制器输出的速度命令。

AI 在本项目中的合理位置是**感知前端与任务目标生成**：

- 检测/分割/跟踪给出「哪一类物体、哪条轨迹、是否在 ROI」。
- 场地网络/几何给出「左右安全区在图像中的位置」。
- 任务层把**已经落到地图或 odom 中的位姿**发给 Nav2（`NavigateToPose` 或等价动作），而不是把 2D 框中心写进 `goals`。

要把 2D 框变成 `map` 目标，必须具备下列之一（当前仓库均未作为已有完整链路查到）：

| 路径 | 需要的证据 | 现状 |
| --- | --- | --- |
| 深度或立体 | 深度图/点云与相机光学中心对齐 | **未查到** 深度相机、激光雷达或点云话题 |
| 已知地面 + 标定相机 | `camera_info`、外参、地面平面假设 | 原图 `frame_id=camera`；**未查到** 内参发布或 URDF/TF |
| 已知地图标记/安全区图纸 | 场地区域在地图中的先验，再与视觉分区对应 | 仅有 660×360 mm 安全区图纸与图像归一化中心，**没有** 全场 `map` 或 SLAM 包 |

无上述证据时，不得把检测框中心、类型 3/4 的 ×10000 图像坐标，或 `SafetyTarget.x/y` 直接当作 Nav2 目标。

语义 costmap（提案）必须与几何避障分开：

- 静态语义（例如已确认的安全区边界、禁止进入的蓝色区域）可在置信与时效合格后写入静态或慢变图层。
- 动态目标（被跟踪的货物、人员，若未来有）走动态层或预测，须带时间戳、置信度、丢失超时后清除；**不能**用过期检测框长期占格。
- 几何避障（激光/深度/可靠的局部障碍层）仍是碰撞约束的主通道。仅有前置 2D 相机时，应把「无距离障碍传感器」列为**能力缺口**，用低速、已知场地、视觉停障的限定验收，而不是假装已有激光。

## 5. 控制权单入口、互斥与停稳交接（提案）

同一时刻只允许一个体速度写手进入 STM32：手动（类型 2 / UART6 手动）、Nav2 导航、视觉对位/接近（若仍保留板内视觉伺服）。取消与故障恢复**不得**回退到旧控制路径抢写。推荐状态：`手动 | 导航 | 对位 | 空闲（使能零速或已停稳）| 故障锁存`。

交接条件（须与开放决策中的停稳语义对齐，**不得在查询副作用未修时假装已安全**）：

- 交出控制前：明确停速并等待约定的「可交接」条件（见第 11 节 H3，待批）。
- 进入导航前：USB 会话有效、里程计/IMU 契约有效、Nav2 代价地图与 TF 就绪（就绪定义待批，不另堆超时保护）。
- 进入对位/夹爪：导航到达窗口后由任务层接管，Nav2 停止发布速度（或桥接层停止转发）。
- 故障：Halt / 失能后保持锁存，直到显式重新初始化；插线或帧流恢复不得自行续跑（沿用现协议精神）。

## 6. cmd_vel、里程计、TF、IMU 契约（提案）

沿用三轮全向 `vx, vy, wz`，**不套差速模型**。单位 m/s、rad/s，车体 x 前 y 左、角速度逆时针正，与 `chassis_pose.h` / 类型 2 一致。

**速度命令：**

- 官方 Nav2 Jazzy 对 `Twist` / `TwistStamped` 的默认并不完全写在同一页：Behavior Server 文档写 `enable_stamped_cmd_vel` 在 Jazzy 及更早**默认 `false`（Twist）**；部分入门指南提到 Jazzy 起更多包转向 `TwistStamped`。本仓库目前**没有** `cmd_vel` 订阅者。
- 推荐（待批，安装后以当时 Jazzy 包实测为准）：导航桥只订阅一种类型，由**唯一发布者**（Nav2 controller 或经互斥仲裁后的单一转发节点）输出。若选 `TwistStamped`，以消息头时间为准并与固件接收时间对齐策略写清；若选 `Twist`，必须另定「收到即视为当前」的年龄规则。不要两种同时生效。
- 固件限幅与斜坡保留：桥接层下发前按 0.50 m/s、4.0 rad/s 量化到 ×1000 的 int16（与类型 2 相同刻度，便于复用载荷），STM32 再执行既有斜坡。导航侧的 `vx_max/vy_max/wz_max` 与加速度限制应**不大于**固件能力，避免控制器假设瞬时达到而板端被斜坡削掉。具体数值用实测整定，不在本文写死一套未经验证的 YAML。

**里程计与 TF：**

- 必须有且仅有一个 `odom → base_link` 发布者。推荐：STM32 为位姿源，Jetson 桥翻译为 `nav_msgs/Odometry` 并发布该 TF；`map → odom` 由未来的定位/SLAM 节点独占。禁止第二套积分再发同一 TF。
- `header.frame_id=odom`，`child_frame_id=base_link`，twist 在 `base_link`。时间戳策略（板时间 vs ROS 时间、USB 延迟）待批，必须可复现。
- 协方差：板内结构目前没有。提案：先给保守、文档化的对角协方差并在实车标定前标明「未标定」；不要为通过 Nav2 启动而编造过小方差。
- IMU：BMI088/INS 在 STM32 内已用于航向与角速度，**未向 ROS 发布 `sensor_msgs/Imu`**。若未来用 `robot_localization` 再融合，须先定义唯一 IMU 发布者、坐标系、时间同步，并避免与板内 ESKF 双重积分抢 TF（开放决策）。

**局部控制器候选（官方 Jazzy，须全向）：**

据 Nav2 Jazzy「Setting Up Navigation Plugins」与 Tuning Guide：

| 插件 | 官方支持的本车相关类型 | 说明 |
| --- | --- | --- |
| DWB（默认） | Differential、**Omnidirectional** | 轨迹生成 + critics；全向需配置非零横向速度采样（如 `max_vel_y`、`vy_samples`） |
| MPPI | Differential、Ackermann、**Omnidirectional** | `motion_model: "Omni"`，`vy_max` 限制横向速度 |
| RPP / VP 等 | 官方表以差速/Ackermann/腿式为主 | **不作为本车默认候选** |

TEB 官方也写了全向，但 Jazzy 调参资料与维护重心在 DWB/MPPI。推荐短名单：**先在主机/仿真对比 DWB Omni 与 MPPI Omni**，用第 10 节的算力和跟踪误差选型，不预先宣布 MPPI 一定更好。官方写 MPPI 在「普通 Intel 四代 i5」上可到数十至 100+ Hz，**这不是 Orin Nano 上的实测，不得抄成验收指标**。

## 7. 算力、TensorRT 与导航争用（提案）

决策中心是 Jetson。现状：YOLO-Seg 走 TensorRT（可 `dla_core: -1` 用 GPU）；场地为 OpenCV；图传默认 `nvjpeg`；分割与场地**不同时推理**，处理慢时取最新帧不积压。已有 `InferenceTiming`（`gpu_ms/post_ms/inference_ms/publish_ms/total_ms`），注释写明不含采集排队与网络，且「无运行期订阅者」——适合作为以后实测的仪器，而不是现成的争用结论。

接管后 Nav2 的 planner/controller/costmap 会与推理争 CPU/GPU/内存（8GB）。原则：

- 用实测选型：记录推理 `total_ms`、controller 周期、USB 往返、从目标更新到轮指令的端到端延迟。不编帧率。
- 保持「最新帧、不堆队列」的现有策略；不盲目加模型、不叠多重滤波「保护」。
- 不把官方 x86 频率或配置里的 100/120 FPS 上限当成实车已达到的数。
- 若实测中局部控制周期无法稳定高于固件 5 ms 控制拍的合理倍数，应降规划/采样规模或降低推理分辨率，而不是加超时兜底。

## 8. USB 字段是否足够、两端必须同步

**现有字段不够支撑完整 Nav2 契约。** 依据：最大载荷 8 字节；无上行位姿/IMU；自动固件忽略类型 2；无协方差与 ROS 时间。

评估与提案（均待批，不得擅自改协议）：

| 需求 | 现有 | 缺口 |
| --- | --- | --- |
| 下发 `vx,vy,wz` | 类型 2 载荷形态已存在，但自动固件忽略 | 自动会话中的合法速度写手、与类型 3/4 周期流的互斥、租约 |
| 上报位姿+速度 | 无 | 至少需要位置、航向、vx/vy/wz、有效位、时间；8 字节不够一帧装完，须多帧或扩展长度（扩展长度会影响 CRC 与两端解析，属协议行为变化） |
| 时间同步 | 类型 3/4 仅 24 位源帧毫秒 | 控制命令与 odom 需要可对齐的时基 |
| 协方差、IMU 九轴 | 无 | 若 Nav2/定位确实需要，再单独立项，避免一帧塞满 |

两端必须同步的项（协议变更一旦批准）：帧类型与长度、量化与坐标符号、有效位语义、会话代次、停稳/故障码、命令租约与周期流关系。版本号仍为 `01` 时不能默默改载荷含义。

与本轮已查、**尚未修复且待审批**的通信问题（不可写成已修）：

- 100 ms 解析积压与 250 ms 周期流是不同层（`comm-review`），但 100 ms 过期会 `OnDisconnect` 并清周期租约，可能 Halt。
- USB 解析重同步 SOF 时戳：坏前缀被丢弃后，交付时间仍可能沿用旧前缀时刻（`results/comm2-deep-review.md` C2-1，已复现）。
- 首次发送前已入环、尚未解析的旧回执可越过序号屏障（同文件 C2-2，已复现状态迁移，不是实车夹爪发生率证明）。

在这些问题未修时做 Nav2 速度桥，会把错误时间或旧回执带进控制权切换。推荐先关闭或限定这些缺陷，再扩展协议。

## 9. 逐文件修改清单（提案待批，本轮不改）

保留：三轮几何与 `motor_chassis`、CAN1 电机、USB CDC 物理层、夹爪类型 21/9、现有视觉/跟踪/任务节点、STM32 轮环与斜坡、INS/ESKF 板内融合（行为变更另批）。

建议替换或收缩的职责（待批）：`jetson_auto` 中「全程自己规划位移/返航/盲推」在导航模式下让出给 Nav2；视觉接近是否完全退出板内伺服，还是仅在对位阶段启用，须单独决策。

可能涉及的仓库路径（**新包/新目录须先批准**，默认优先在现有 `bridge_pkg` / `mission_pkg` / `bringup_pkg` 与 `jetson_link` / `chassis` 内扩展）：

| 路径 | 未来可能的职责变化 |
| --- | --- |
| `Jetson/docs/usb-protocol.md`、`STM32/docs/JETSON_USB_TEST.md` | 同步新帧类型与互斥规则 |
| `STM32/project/Core/Communication/Jetson/jetson_link.*`、`jetson_usb.c`、`link_payload.h` | 自动模式接收速度、上报里程计 |
| `STM32/project/Core/Control/Jetson/jetson_auto.c` | 导航模式下不再自行规划路径；互斥与 Halt |
| `STM32/project/Core/Control/Chassis/chassis.*` | 对外速度入口保持单一；查询副作用未批准前不改行为 |
| `Jetson/src/bridge_pkg/` | 速度/odom 桥、唯一写手（提案，或批准后的新包） |
| `Jetson/src/mission_pkg/` | 任务目标进入 Nav2 动作，夹爪仍走现握手 |
| `Jetson/src/bringup_pkg/` | 启动角色表增加 Nav2（结构变化待批） |
| `Jetson/src/vision_pkg/`、`track_pkg` | 默认可不改模型；若输出语义层须另批接口 |
| 新 URDF / 定位 / Nav2 参数 | **整包新增，必须先批准**；本文不编造文件内容 |

## 10. 验收：任务效果而不是抽象最优

可复制命令仅在真实可运行时给出。当前仓库可运行的是 Humble 下的视觉/bridge 主机测试与 `ros2 launch bringup_pkg rescue.launch.py run:=vision|all`（见根 README；引擎须按设备准备，仓库不附带）。**Nav2 相关 launch、参数、仿真世界在本仓库中不存在**，下面只列出阶段，不假装已有命令。

| 阶段 | 目的 | 通过含义 | 命令 |
| --- | --- | --- | --- |
| 无车 / 主机 | 协议编解码、互斥状态机、量化与限幅、TF/odom 单测 | 契约测试绿，不涉及实车 | 未来在落地测试入口后补；现可沿用 `check_jetson_codec.py` 等既有主机检查，但不覆盖 Nav2 |
| 仿真 | DWB vs MPPI、目标落入、避障、互斥 | 无碰撞完成规定任务，记录路径长度与时间 | 未来示例：待批准的仿真包与官方 Nav2 Jazzy 启动文件，安装后再写准确命令 |
| 悬空（轮离地） | 指令与反馈方向、斜坡、USB 租约、不停写抢占 | 轮向与限幅符合契约，故障锁存可恢复 | 未来示例：实车悬挂规程 + 当时的 bridge/Nav2 启动 |
| 低速实车 | 已知场地、限定速度 | 任务成功率、碰撞次数、完成时间、CPU/GPU/内存、端到端延迟 | 未来示例：须含现场标定后的地图与限速参数，不在本文编造 |

验收指标建议：任务成功率、是否碰撞/刮擦、路径长度与完成时间、控制周期抖动、推理与导航的算力占用。不把「规划器代价最小」或「网络 loss 最低」当作接管完成的判据。

## 11. 本轮已查、未修且待审批的问题（不得写成已修）

这些是后续接管的前置风险，不是本文能宣布关闭的项。独立回归仍为 **36/37**（`jetson_chassis_integration` 未过），断言未放宽。

1. **`Chassis_Motion_Ready` 查询副作用（H3）**：成功查询会消费 `distance_completion_pending` 等状态。确认卡已向用户展示，**行为修复尚未批准**，不得在交接条件里把当前查询当作无副作用的纯函数。
2. **USB 解析重同步 SOF 时戳**、**首次发送前已入环的旧回执**：见第 8 节，已复现，未合入修复。
3. **100 ms 积压与 250 ms 周期流不同层**，但回调耦合会导致误 Halt。数字与策略未改。
4. **定距锁轴与调参验收轴的角状态契约仍待明确。** 文档 `ODOMETRY.md` 支持按固定目标轴投影计程，不要求每拍车体系 `vy=0`。该测试桩绕过真实 INS 入口的航向门控；**不能仅凭测试桩以及 `Heading_ESKF_Update_Observed` 中的 `check_jump=0`，就断定算法缺陷或建议重复加门限。** 独立回归表明瞬时跳角不能解释全部 integration 失败。后续应先核对「锁轴用预测 ESKF 航向、`live_tuning` 用 INS 航向做验收轴」这一契约，再分别验证世界轴投影与横向修正；不修改融合参数、不硬复位、不为报绿改期望。依据见统一改动记录第 101 行及 `results/control_heading_review.md`、`control_distance_contract_review.md`、`regression-final-2244/final-findings.md`。

## 12. 开放决策与有依据的推荐

| 编号 | 决策 | 推荐（待批，不实施） | 依据 |
| --- | --- | --- | --- |
| D1 | 发行版：继续 Humble 还是迁 Jazzy 再启 Nav2 | **以 Jazzy 为接管目标发行版**，因官方 Nav2 文档与用户指定按 Jazzy 核实；须先处理仓库主文档仍写 Humble、夹爪 README 写 Jazzy 的不一致，并做一次完整 vison/bridge 回归 | 根 README；gripper README；Nav2 Jazzy 文档 |
| D2 | 决策中心硬件 | 按用户指定的 Orin Nano 8GB Super；与 2026-11-16 记录中的「Jetson Orin Nano」是否为同一 SKU **待现场核对** | 用户 t200u；Jetson README 现场记录只写到 Orin Nano |
| D3 | 谁发 `odom→base_link` | STM32 为唯一位姿源，Jetson 只翻译发布 | 板内已有里程计；ROS 侧目前无 odom |
| D4 | 要不要 `robot_localization` 再融 IMU | **第一阶段不叠加**，避免双积分；若定位质量不足再立项 | 板内已有 ESKF；无现成 IMU 话题 |
| D5 | `Twist` 还是 `TwistStamped` | 安装 Jazzy Nav2 后读实际默认，优先与 controller 一致的一种；需要时间对齐则用 Stamped | 官方文档默认值在不同页面表述不完全相同 |
| D6 | 局部控制器 | 短名单 DWB Omni 与 MPPI Omni，**以 Orin 上与 TensorRT 同机实测选型** | Nav2 Jazzy 插件表 |
| D7 | 2D 目标如何入地图 | 先补相机标定或深度/激光，或把任务限制在「已知安全区图纸 + 视觉分区对准」；禁止直接把框当 pose | 现有 SafetyTarget / 类型 3/4 语义 |
| D8 | 障碍传感器缺口 | 承认目前无激光/深度，低速已知场地验收；若要开放空间自主，须另批传感器 | 全库检索未找到相应话题与设备配置 |
| D9 | 自动模式速度帧 | 复用类型 2 载荷形状并在自动固件增加互斥接收，或新类型；须同步协议文档 | 类型 2 已有量化；自动路径目前忽略 |
| D10 | 上行里程计编码 | 新增多帧或审慎扩展载荷长度；8 字节单帧不够完整契约 | usb-protocol 最大载荷 8 |
| D11 | 停稳交接语义 | 先批准并去掉查询副作用，再规定导航/对位用「指令回零」还是「三轮停稳」 | H3 未批准 |
| D12 | 视觉接近是否保留板内伺服 | 推荐：全局走 Nav2，近距对位可暂留任务+板内或任务+视觉伺服，二者互斥 | 现自动流程依赖类型 3/4 |
| D13 | 夹爪 | 保持任务握手，不纳入 Nav2 | 类型 21/9 已有终态契约 |

## 13. 查证来源与版本

仓库（当前工作树，2026-11-19）：

- `AGENTS.md`、`README.md`、`Jetson/docs/README.md`、`Jetson/docs/usb-protocol.md`
- `Jetson/src/bringup_pkg/launch/rescue.launch.py`、`config/rescue.yaml`、`config/rescue_advanced.yaml`
- `Jetson/src/*/package.xml`，`interface_pkg/msg/*`，`teleop_pkg/manual/command.py`，`vision_pkg` 检测与场地后端，`bridge_pkg` / `mission_pkg` 职责说明
- `STM32/docs/ODOMETRY.md`、`STM32/docs/JETSON_USB_TEST.md`、`STM32/README.md`
- `STM32/project/Core/Control/Chassis/chassis.h`、`chassis_pose.h`，`Communication/Common/link_payload.h`，`Communication/Jetson/jetson_link.c`，`Control/Jetson/jetson_auto.c`（职责与忽略类型 2）
- 本轮未修问题：`docs/optimization-changes-2026-11-19.md` 第四节；`results/comm2-deep-review.md`；`results/control_deep_findings.md`

官方文档（查阅日 2026-11-19，发行版 **Jazzy**）：

- Nav2 Transformations：<https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/transformation/setup_transforms/>
- Nav2 State Estimation（含 REP-105 的 `map→odom→base_link`）：<https://docs.nav2.org/jazzy/getting_started/navigation_concepts/state_estimation/>
- Nav2 Odometry 消息要求：<https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/odom/setup_odom_gz/>
- Nav2 插件与调参（DWB / MPPI 支持全向）：<https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/navigation_plugins/setup_navigation_plugins/>、<https://docs.nav2.org/jazzy/configuration_and_development/tuning_guide/>
- DWB：<https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/dwb_controller/>
- MPPI（`motion_model: Omni`）：<https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/>
- Behavior Server `enable_stamped_cmd_vel`（Jazzy 及更早默认 false）：<https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/configuring_behavior_server/>

本文不包含可直接启动 Nav2 的 launch 或参数；批准并落地后，应按当时安装的 Jazzy 包版本重新填写可复制命令。
