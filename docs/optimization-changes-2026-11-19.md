# 优化改动记录（2026-11-19）

现场工作副本：`/workspace/gc-rescue-user-wip-20261119-215105`，基线提交 `fc76ee3`（与当时 `origin/main` 一致）。用户未提交 WIP 已冻结于 `/workspace/gc-rescue-user-wip-20261119-215105-pristine`。本文件只记录**本轮在工作副本上实际新增或继续修改**的文件，并与上述冻结副本及 git 未跟踪文件核对。未提交、未推送、未回写用户电脑。

维护约定：由本次执行方与其上级代理统一维护，避免多人并行改写。同伴等价补丁须先独立核验再记为已完成。用户于 t196u 授权陀螺仪相关算法也可优化，但行为变化仍须另行确认、不得臆测标定值。截至本记录：阶段 B 只修正了 `read_ist8310` 的注释（先核实返回语义，实现未改），未把 ADRC 接入固件、未调标定。

本文件是本轮新增文档，下文不递归展开其正文。

## 一、继承的用户 WIP（40 个已跟踪修改，本轮不当作“已完成优化”）

冻结时相对 `fc76ee3` 的未暂存已跟踪修改共 40 个文件。除下文“本轮在 WIP 之上继续改动”的三个测试/文档文件外，其余生产与配套测试语义保持用户现场原样，本轮未改行为。

**Jetson（8）：** `Jetson/src/bridge_pkg/gripper/s288_servo.cpp`，`node/bridge_io.cpp`，`node/bridge_node.hpp`，`node/bridge_session.cpp`，`protocol/arrival_point.hpp`，以及 `test/host/` 下 `test_arrival_point.cpp`、`test_bridge_runtime.cpp`、`test_s288_timing.cpp`。

**STM32 手动/串口（8，另见下文 README 本轮增量）：** `STM32/manual_test/firmware/manual_link.c`、`manual_link.h`、`manual_usb.c`、`uart6_manual.c`，`STM32/manual_test/uart6/uart6_manual.py`、`uart6_protocol.py`、`uart6_web.py`、`uart6_web/app.js`。`STM32/manual_test/README.md` 在冻结时已有 WIP，本轮又补了预设说明。

**STM32 业务与通信（14）：** `heading_yaw_guard.c/.h`，`usb_link.c`，`usb_link_handler.h`，`usb_rx_buffer.h`，`jetson_link.c/.h`，`jetson_usb.c`，`BMI088driver.c/.h`，`BMI088reg.h` 本轮未改实现。`chassis.c/.h`、`jetson_auto.c` 在阶段 B 做了经验证的等价结构调整（数值/类型/分支不变）；`INS_task.c` 只修正注释。详见第二节第 4 款。

**STM32 既有测试（7，另见 `test_chassis.c` / `test_jetson_link.c` 本轮增量）：** `test_heading_eskf.c`，`test_jetson_auto.c`，`test_manual_control.c`，`test_sensors.c`，`test_uart6_manual.c`，`test_usb_rx_buffer.c`。

用户 WIP 中已定位、本轮只分类记录、未改行为的问题包括：USB 解析积压 100 ms 与自动周期流 250 ms 分属不同层（见 `comm-review` 与阶段 B 因果分析，不能只凭数值不同判为冲突）；`Chassis_Motion_Ready` / `Chassis_Is_Stopped` 查询有副作用且语义混用（H3）；`Service_Request` 在临界区内发送 USB（M3）；定距横向修正宏尚未标定（M4）；5 ms 任务中连续 `vTaskDelay(1ms)`（M5）。陀螺/INS 启动带宽等仍保持用户现场值。

## 二、本轮新增修改（相对冻结副本）

核对方法：对冻结树做内容比较，并计入 git 未跟踪文件。忽略 `__pycache__`。本轮实际改动如下。

行数按 `wc -l`（Asia/Shanghai）。“板端影响”指是否改变已发布固件的编译选项、链接集合或运行语义。

### 1. ADRC 主机验证（优化阶段，ADRC 不接入固件）

| 文件 | HEAD | 冻结 | 当前 | 本轮相对冻结 |
| --- | ---: | ---: | ---: | ---: |
| `STM32/manual_test/live_tuning/adrc/yaw_adrc_pretest.py` | 307 | 307 | 403 | +96 |
| `STM32/manual_test/live_tuning/adrc/README.md` | 34 | 34 | 49 | +15 |
| `STM32/tests/test_yaw_adrc_pretest.py` | （无） | （无） | 117 | 新文件 |
| `STM32/tests/native/test_chassis.c` | 2049 | 2190（已含用户 WIP） | 2299 | +109 |

- **`yaw_adrc_pretest.py`**：在已提交的初测脚本上增加训练集/留出集、`evaluate` / `metric_pairs` 与 `--verify`（相对 `results.json` 复算，容差 1e-12）。不改固件参数，不写回 `results.json`。接口增加命令行 `--verify` 与仓库路径参数；控制律数值边界与既有宏对照保持一致。测试：`python3 -m unittest` 发现器 7/7 通过；`yaw_adrc_pretest.py --verify <仓库根>` 通过。板端：无。
- **`adrc/README.md`**：补充主机回归与 `--verify` 用法。不改接口。板端：无。
- **`test_yaw_adrc_pretest.py`**：新建 unittest，复算初测指标、宏对照与一阶对象上的观测/限幅行为。板端：无。冻结树中不存在该文件，与未跟踪核对一致。
- **`test_chassis.c`**：用户 WIP 已从 2049 增至 2190。本轮只追加 `test_yaw_rate_first_order_plant`：调用真实 `Apply_Yaw_Rate_Feedback`，配以文档化的一阶对象（非实车性能）。不改 `chassis.c` 实现。断言按过零时同号不施加反馈的既有规则写就，未放宽生产阈值。测试由 `chassis` / `chassis_tuning` 套件覆盖。板端：无。

### 2. 阶段 A：测试分工、USB 测试 API 对齐、构建入口（不改业务控制）

| 文件 | HEAD | 冻结 | 当前 | 本轮相对冻结 |
| --- | ---: | ---: | ---: | ---: |
| `STM32/tests/native/test_jetson_link.c` | 479 | 497（已含用户 WIP） | 491 | −6 |
| `STM32/tests/native/test_manual_link.c` | 145 | 145 | 158 | +13 |
| `STM32/tests/native/test_usb_link_state.c` | 309 | 309 | 309 | 0（只补参数） |
| `STM32/tests/run_native.py` | 197 | 197 | 244 | +47 |
| `STM32/project/CMakeLists.txt` | 167 | 167 | 180 | +13 |
| `STM32/project/CMakePresets.json` | 77 | 77 | 99 | +22 |
| `STM32/README.md` | 52 | 52 | 52 | 0（补充说明） |
| `STM32/docs/JETSON_USB_TEST.md` | 164 | 164 | 164 | 0（补充预设名） |
| `STM32/manual_test/README.md` | 56 | 56（已含用户 WIP） | 56 | 0（再补预设名） |

- **`test_jetson_link.c`**：将 `Check_Usb_Receive_Time` 中的 `Manual_CAN_*` 调用迁出，保留 Jetson `Receive_At` 覆盖；**保留** `#include "manual_link.h"`，因为 `Check_Removed_Type` 仍使用 `MANUAL_CAN_VELOCITY_ID`。不把 `manual_link.c` 链入 `jetson_can`。测试：`jetson_can` 通过。板端：无。
- **`test_manual_link.c`**：冻结时与 HEAD 相同（不在用户 40 文件内容差之列）。新增 `Check_Receive_At`：`test_tick` 与显式 `received_ms`（含回绕）不同，断言 `Manual_CAN_Receive_At` 不改写为当前 tick。测试：`manual_can` 通过。板端：无。
- **`test_usb_link_state.c`**：按现行 `UsbRxBuffer_Push(..., received_ms)` / `UsbRxBuffer_Pop(..., uint32_t*)` 补齐入参；注入时刻使用当时 `test_tick`，空弹出传 `NULL`。不断连/代次/溢出断言，不改 100 ms 策略，不改生产实现。测试：`usb_link_state` 通过。板端：无。
- **`run_native.py`**：增加 `--suite`（可重复）、`--list-suites`、`--python`（ADRC unittest、协议互验、夹爪预算）、`--skip-native`。默认仍先跑完全部 C 套件再跑模块边界检查；失败即停，与原先一致。板端：无。
- **`CMakeLists.txt` / `CMakePresets.json`**：补齐业已存在于 `chassis_mode.h` 的 `JETSON_AUTO_CHASSIS_TEST` 选项及互斥检查；新增公开预设 `Debug-Auto-Chassis-Test`、`Debug-Tuning`（后者等同原先文档中的 `-DCHASSIS_TUNING_ENABLED=ON`）。控制器源文件选择仍通过 `JETSON_AUTO_CAN_ENABLED`，不必改链接逻辑。板端：仅在选用新预设或显式打开该选项时影响固件编译开关，默认 `Debug` 行为不变。
- **三份说明**：列出新预设与 `run_native` 新开关。不改协议或控制参数。`manual_test/README.md` 仅在用户 WIP 原文上替换构建预设表述。

### 3. 本轮文档

- `docs/optimization-changes-2026-11-19.md`：本统一改动记录（新建）。阶段 B 只追加本款与验证/遗留更新，不递归登记自身正文。

### 4. 阶段 B：行为等价集成（底盘常量集中、请求编码分离、IMU 注释）

在保存阶段 A 快照后，独立审查同伴结果（实际位于 `/workspace/gc-rescue-user-wip-20261119-215105-results/`）：`control_algorithm_constants.patch`、`control_algorithm_review.md`、`control_algorithm_change_record.md`，以及 `comm-request-encoding.patch`、`comm-review.md`、`comm-file-change-record.md`、`comm-verify.log`。确认相对用户 WIP 保留类型/数值/编码与全部分支后，先在隔离副本分别测试，再应用到共同工作副本；未强套任何不等价或不适用的补丁。未提交、未推送、未回写。

| 文件 | HEAD | 冻结/阶段A末 | 当前 | 本轮相对阶段A |
| --- | ---: | ---: | ---: | ---: |
| `STM32/project/Core/Control/Chassis/chassis.c` | 1087 | 1219 | 1217 | −2 |
| `STM32/project/Core/Control/Chassis/chassis.h` | 261 | 266 | 276 | +10 |
| `STM32/project/Core/Control/Jetson/jetson_auto.c` | 1122 | 1185 | 1197 | +12 |
| `STM32/project/Core/Task/INS/INS_task.c` | 362 | 373 | 375 | +2（仅注释） |
| `STM32/tests/native/test_jetson_auto.c` | 1665 | 1719（用户 WIP） | 1719 | 0（+6/−6，仅时钟安排） |

- **`chassis.c` / `chassis.h`**：把底盘控制里已有的文件局部宏/字面量提升为同类型、同数值的 `CHASSIS_*` 宏，公式与分支未改。预处理展开后与打补丁前逐字节相同（SHA256 `27326e7c30506c4fd8c251f11348a8a7583b986008cc4c857edf47abbb331496`）。附近未使用的字面量（如 kd `0.05f`/`0.5f`、dt 上限 `0.05f`、转向 `0.5f`）故意未收录。测试：隔离套件与合并后正式回归均未引入新失败。板端：编译开关不变，运行语义不变。
- **`jetson_auto.c`**：从 `Service_Request` 抽出纯函数 `Encode_Request`，发送、临界区和 ACK 路径未改。编码对照 1,644,775 组通过。覆盖边界：全部 16 位 id、动作类型 256 点、五组 token/request 抽样；**不是** 32 位 token 穷尽，也不是机器码等价（同伴曾放弃汇编对照）。枚举全覆盖不等于系统无 bug。板端：行为不变。
- **`INS_task.c`**：先核对 `read_ist8310` 返回语义——I2C 成功时仅当 STAT1 数据就绪位置位才返回 1，调用方只在返回 1 时把磁样本送入 `update_attitude`，否则走既有六轴。阶段 A/冻结注释写的是「1 表示 HAL 读成功，是否有新样本由 status 就绪位区分」，与实现不符。本轮只把注释改为：「1 表示 I2C 读取成功且 STAT1 数据就绪位已置位；0 表示总线读取失败，或读取成功但尚无新样本」并注明调用方空指针六轴分支不变。函数体未改。板端：无运行影响。相对 HEAD 的 `git diff` 仍含用户 WIP 的实现改动，其中只有注释两行属于本轮。
- **`test_jetson_auto.c`（已批准的测试修复，不改业务/阈值）：** 核验 `results/regression-deep-2236` 复现后纳入。`Test_Visual_Timeout_Without_Type1` 与 `Test_Periodic_Stream_Timeout` 原先在 `Begin_Target_Test()` 之后把 `test_tick` 赋为 `UINT32_MAX-120`，破坏已建立的时间边界。现改为按 wrap=0 测得的启动耗时，从 `UINT32_MAX-120-startup_elapsed` 连续启动，断言全部保留。同文件中已有 `Begin_Target_Test_At(UINT32_MAX-3250)` 的连续回绕写法。对照日志：`auto-original-probe.log` wrap=1 `vx=0` 失败；`auto-continuous-both.log` 两支 `vx=0.0019999966` 且整套通过。纳入共同副本后正式 `run_native --suite jetson_auto_configured` 通过。板端：无。

## 三、本轮验证（摘要）

- 正式入口 `python3 STM32/tests/run_native.py`：预处理 17 组模式通过。纳入测试时钟安排修正后，`jetson_auto_configured` 通过；失败即停现在发生在 `jetson_chassis_integration` @ `test_jetson_chassis_integration.c:307`（断言未放宽）。`jetson_can`、`manual_can`、`usb_link_state` 及自动/底盘等此前套件均通过。
- 继续执行诊断入口（时钟修正后）：37 个 native 套件中 **36 通过、1 失败**（仅 `jetson_chassis_integration` :307）。`check_module_boundaries` 现已通过（原先因 `:1302` 失败）。未放宽断言，未改业务行为或 `Chassis_Motion_Ready`。等价补丁隔离测试时的基线失败已见阶段 B 先前记录。
- Python：系统解释器全量 discover 117 项中 116 通过，1 项为 `test_mag_calibration` 缺 SciPy 无法导入（工具缺口）。venv（SciPy 1.18.1）下该模块 7/7 通过。ADRC unittest 7/7，`--verify` 通过。协议互验下行 155 / 上行 33 通过。UART6 网页 JS 通过。`git diff --check` 通过。夹爪预算脚本因环境仍无主机 `g++` 未跑（缺工具，不记代码失败）。
- 固件：阶段 B 从 xPack 官方发行安装 `arm-none-eabi-gcc` 14.2.1（`/workspace/tools/arm/...`），CMake 3.30.6 与 Ninja 1.12.1。七个预设均配置并链接成功（`project.elf`）：Debug RAM 40200 B / FLASH 97672 B；Debug-9axis 40200 / 99156；Debug-Auto 40640 / 117320；Debug-UART6-Manual 41472 / 104760；Debug-Auto-Chassis-Test 40488 / 109944；Debug-Tuning 41192 / 122680；Release 40152 / 56652。未做实车下载或在线验证。构建目录位于 `STM32/project/build/<preset>/`（git 忽略）。
- 阶段 B 等价核验：底盘预处理一致性（见上 SHA）；通信编码 1,644,775 组通过。覆盖边界已在第二节第 4 款写明，不把枚举全覆盖等同系统无 bug。直行与停稳的只读诊断在隔离临时副本中进行，**不算正式回归**。回绕时钟安排已纳入正式测试并经官方套件复跑。

独立回归（`results/regression-independent-2212/findings.md`）给出的编译失败、保留 `manual_link.h`、以及两处行为断言，已纳入阶段 A 修复与定位。控制/通信同伴补丁已在阶段 B 按等价条件集成，见第二节第 4 款。

## 四、遗留（待用户批准后再改行为；本轮不改实现、不放宽断言、不加保护分支）

以下为阶段 B 只读根因与最小方案，隔离诊断不算正式回归。发现的是测试夹具或层间耦合/查询副作用，不是靠新增 if、超时或回退能粉饰的表象。

1. **`test_jetson_auto.c` 回绕时钟安排（已纳入测试修复，业务比较未改）。** 根因是两处回绕场景（`Test_Visual_Timeout_Without_Type1`、`Test_Periodic_Stream_Timeout`）在启动并建立边界之后直接把 `test_tick` 赋为 `UINT32_MAX-120`，`(received_ms-center_boundary_ms)>INT32_MAX`（官方失败支 `received_minus_200=4294966975`，等效向后 321 ms），不是连续过零时的业务回绕 bug。已按 `results/regression-deep-2236`（`auto-original-probe.log` / `auto-continuous-both.log` / `clock-continuity-test-only.patch`）核验并纳入：从回绕前按实测启动耗时连续启动，全部原断言保留。正式 `jetson_auto_configured` 与模块边界检查通过。未改 `<= INT32_MAX` 比较或任何时限。

2. **`test_jetson_chassis_integration.c:307` 定距坐标契约与测试输入（尚未闭合）。** 原场景纵向已起步（`cmd[0]≈0.003700`），横向约 `-0.000608`，原 `<1e-6` 断言仍失败且未放宽。测试将航向从 0° 瞬时改为 180°/176°，同时角速度为零并标为有效观测；该桩绕过真实 INS 入口的航向门控，不能仅凭 `Heading_ESKF_Update_Observed` 中 `check_jump=0` 判定算法缺陷，也不据此建议重复添加门限。底盘锁轴约 95.8367°，下一拍估计航向约 105.1712°，世界轴转车体系可产生非零横向速度。不过，独立回归的连续一致输入对照在 176° 场景仍得到 `vy≈-1.055e-6`，故瞬时跳角不能解释全部失败，也不能直接认定修正夹具即可全过。文档 `ODOMETRY.md:97` 与业务调用支持固定目标轴投影计程，不要求每拍车体系 `vy=0`；但底盘以预测 ESKF 航向建轴，`live_tuning.c:575–579` 以 INS 航向另建验收轴，锁轴角状态的契约仍需明确。后续分别独立验证世界轴投影与横向修正，先核对契约，不修改融合参数、不硬复位、不为报绿调整期望或容差。依据：`results/control_heading_review.md`、`results/control_distance_contract_review.md`、`results/regression-final-2244/final-findings.md`；本次仅更正文档判断，未改业务或测试源码。

3. **停稳交接 / 查询副作用（H3，行为修复尚未批准，本轮未改 `Chassis_Motion_Ready`）。** `Chassis_Is_Stopped` 成功时清 `distance_completion_pending`；`Chassis_Motion_Ready` 成功时清该标志以及 `awaiting_enable_feedback`，且允许轮子仍在减速，但不接受 `HOLDING` 模式。`control_deep_findings.md` 已复现纯查询也会消费终点。上级已在群向用户展示确认卡，**目前尚未批准该行为修复**，因此未改查询实现。`JETSON_AUTO_ENABLED` 与测试宏下的语义分流、1.5 s 持有时限问题仍在。方案仍是：查询改为纯函数，状态机真正接受交接时再消费终点；不调门限、不放宽时限、不加保护。

4. **USB 会话撤销因果（与 100 ms / 250 ms 分层）。** 按 `comm-review`，`USB_RX_MAX_AGE_MS=100` 是解析积压/半帧年龄，`JETSON_AUTO_LINK_TIMEOUT_MS=250`（及中心 250 ms、手动 2 类租约 250 ms）是合法类型 3/4 周期流新鲜度，**不能只凭数字不同判冲突**。但两条规则目前被同一回调耦在一起：`UsbLink_Task` 在弹出字节或半帧超过 100 ms 时 `UsbRxBuffer_Reset`、`++link_state.generation` 并调用 `UsbLink_OnDisconnect()` → `Jetson_CAN_Invalidate_Periodic()`（把 `periodic_received` 清零）。`link_state.connected` 在这条路径上并不清零（与 `link_offline()` 的物理断开不同）。随后 `Link_Ready` 因 `!fresh || generation != link_generation` 立即 `Halt`。因此解析任务延迟或环首一个旧字节超过 100 ms，就会毁掉 250 ms 租约并故障停车；ISR 入环导致 `rx_buffer.generation` 变化时同样 `OnDisconnect`。方案 A：维持「积压过期即新会话、Halt」，把上述因果写进 `JETSON_USB_TEST.md`，并补「解析延迟 >100 ms 但周期流仍 <250 ms」的测试。方案 B（需批准）：100 ms 只重置环与解析器，不增加 `link_state.generation`、不调用 `OnDisconnect`；Halt 只留给物理断开（`connected=0`）或 250 ms 流过期。不要再加第三条「若 fresh 则不 Halt」的保护分支。数字本身未改。

5. **仍待确认、本轮未做：** M3 临界区发送、M4 横向增益标定、M5 `vTaskDelay`；陀螺/ADRC 算法优化可继续审查，但行为变更仍待确认，本轮未接入 ADRC、未调标定。主机 `g++` 仍缺，夹爪预算未跑。

## 五、本轮文件清单（相对冻结副本）

`STM32/manual_test/live_tuning/adrc/yaw_adrc_pretest.py`，`STM32/manual_test/live_tuning/adrc/README.md`，`STM32/tests/test_yaw_adrc_pretest.py`，`STM32/tests/native/test_chassis.c`，`STM32/tests/native/test_jetson_link.c`，`STM32/tests/native/test_manual_link.c`，`STM32/tests/native/test_usb_link_state.c`，`STM32/tests/run_native.py`，`STM32/project/CMakeLists.txt`，`STM32/project/CMakePresets.json`，`STM32/README.md`，`STM32/docs/JETSON_USB_TEST.md`，`STM32/manual_test/README.md`，阶段 B：`STM32/project/Core/Control/Chassis/chassis.c`，`chassis.h`，`STM32/project/Core/Control/Jetson/jetson_auto.c`，`STM32/project/Core/Task/INS/INS_task.c`（仅注释），`STM32/tests/native/test_jetson_auto.c`（仅回绕时钟安排），以及本文件 `docs/optimization-changes-2026-11-19.md`。


## 六、Nav2 全局接管设计文档（仅新增文档，未改业务）

按用户 t199u / t200u：单独编写以后由 Jetson 使用 ROS 2 Nav2 与现有神经网络做决策、STM32 负责实时电机执行的修改设计文档。本轮只新增该文档，并在此登记；未改其它源码，未提交、未推送、未回写。文档中的新包、目录、协议与行为变化均标为提案待批。第四节既有遗留（含第 101 行已更正的定距契约判断、查询副作用、USB 分层与未修通信问题）在设计文档中引用为未修项，不改写为已完成。

| 文件 | HEAD | 冻结 | 当前 | 本轮 |
| --- | ---: | ---: | ---: | ---: |
| `docs/nav2-global-takeover-plan.md` | （无） | （无） | 278 | 新文件 |

- **目的**：记录当前实际链路与未来「Nav2 统管导航底盘、夹爪/对位任务交接」的契约、USB 缺口、控制器候选与验收，供以后启动时使用。
- **接口/参数/行为**：无代码变更。板端：无。
- **不覆盖** 本文件第四节第 101 行已更正判断。

## 七、收尾深查（等价清理，未改行为；2026-11-19 23:05:44 CST）

按用户 t198u / t201u / t202u：STM32 代码完成后做深度检查，查证可确定的冗余保护、单位、默认值、参数读取与跨模块限幅/控制周期配合；本轮**不把该要求视为先前「行为变化另行确认」的撤销**。已再读最新 `results/control_distance_contract_review.md`、`results/control_heading_review.md`、`results/regression-final-2244/final-findings.md`。第四节第 101 行已更正判断保持原样，本段只追加、不改写。Nav2 设计文档已交付，本轮不再修改，也不纳入同步。

### 已实施

无新增等价源码修改。阶段 A/B 既有增量（控制常量上收、`Encode_Request` 提取、IMU 注释、官方回绕时钟测试安排、测试/构建缺口）保持不变。查无可以证明无行为差异的冗余保护可删，故不强行凑改，也未叠加新保护。

### 观察（未改源码）

1. `chassis.c` 中 `DEG_TO_RAD`（`float`）与 `CHASSIS_TURN_DEG_TO_RAD`（`double`）数值相同但类型不同，合并可能改变运算精度，本轮不合并。
2. `chassis.c` 残留字面量（如 `kd` 范围 `0.05f`–`0.5f`、前馈/加速 `dt` 封顶 `0.05f`、转向退出 `0.75f`/`0.5f` 比例）并非已上收 `CHASSIS_*` 常数的重复定义，阶段 B 已故意保留。
3. 控制周期配合：`chassis_task.c` 按 `5 ms` 唤醒；注释名义控制约 `10 ms`；任务堵住后 `dt` 封顶 `0.05 s`（约 10 个 5 ms 节拍）；电机反馈超时 `30 ms`；转向停稳确认 `20 ms`。分属不同层，数字不同不等于冗余或冲突，本轮不改。
4. USB 解析积压 `100 ms` 与周期流新鲜度 `250 ms` 的分层及同一回调耦合，仍见第四节第 4 条；方案 B 未批准，不改数字、不加第三条保护分支。
5. `JETSON_AUTO_RETURN_MAX_M` / `ALIGN_MAX_M` / `SAFETY_STRAIGHT_M` / `ROI_BACK_M` / `RECOVERY_BACK_M` 默认 `0.0f` 为待标定占位，本轮不猜测赋值。`JETSON_AUTO_ALIGN_SPEED_MPS=0.10` 与 `CHASSIS_RUN_SPEED_MPS=0.40`、`CHASSIS_DISTANCE_ACCELERATION_MPS2=0.75` 与 `CHASSIS_STRAIGHT_ACCELERATION_MPS2=0.45` 对应不同工况，不统一。
6. **仍未批准、未落地：** `Chassis_Motion_Ready` 查询副作用、USB 重同步/入环屏障（C2-1/C2-2）、INS/ESKF 建轴行为变更、接入 ADRC、改标定或协议/结构。算法升级仅作记录，不实现。

### 定距断言与回归（明确未全绿）

`test_jetson_chassis_integration.c:307` 仍失败，官方 `run_native.py` fail-fast 停在该断言。continue-on-fail 为 **36/37**，`check_module_boundaries` 通过。独立回归的连续一致输入在 176° 场景仍得到 `vy≈-1.055e-6`，不能用 0°→180° 测试桩错误掩盖这一结果，也不放宽断言或改用例以求全绿。失败用例保留，明确未通过。不得将本轮说成全部完成或全部通过。

本轮复跑（无新业务源码）：官方 fail-fast 停于 `:307`（exit 1）；continue-on-fail `36 passed, 1 failed, 37 total`；venv Python `123/123` OK；`check_jetson_codec.py` 通过；`test_uart6_web_focus.js` 通过；`git diff --check` 通过；`yaw_adrc_pretest.py --verify` 通过。主机 `g++` 系统仓库不可用，已在 box 安装并校验官方 xPack GCC 15.2.0（`xpack-gcc-15.2.0-1-linux-x64.tar.gz` SHA256 `f6dcc3815299a23b2b4372223c4ca5f1b5a0543ed097d57eb67e0753b7c484e0`），`check_gripper_budget.py --cxx` 通过：`S288 maximum budget 11870 ms <= STM32 12000 ms`。未在用户电脑安装任何工具。本轮无源码变更，未重复链接 7 个固件预设；先前 `regression-final-2244` 的链接结果不在本轮重新声明为已复验。

交付包只含相对冻结副本（pristine）的本轮 STM32 增量与本统一变更文档，不含用户原 WIP 独有文件、Jetson、`docs/nav2-global-takeover-plan.md`、`.git` 与构建产物。清单、补丁与 SHA256 见 `results` 下本轮交付目录。

## 八、算法实验（t204u，仅 results，未入固件；2026-11-19 23:22:23 CST）

按用户 t204u：以刚同步的 19 文件为交付基线，冻结于 `results/algo-baseline-20261119-231431/`（与 `stm32-delivery-20261119-230617/files` 一致，HEAD `fc76ee3`）。本轮只在 box 的 `results/algo-exp-20261119-231800/` 做独立 C 探针与短比较记录，**未改生产控制文件、未同步电脑、未提交、未推送**。不把继续优化视为已批准 Motion_Ready、USB、ADRC 或标定修改。第四节第 101 行判断保持原样。

比较见 `results/algo-exp-20261119-231800/algorithm-comparison.md`。优先候选至多两个，均属行为变化，待用户确认后才可考虑进固件：

1. 定距锁轴、世界到车体旋转与 `Hold_Yaw` 统一为预测 ESKF 航向。连续一致转到 176° 的无噪声植物上，首拍 body vy 仍为 1.160e-6（与先前 `vy≈-1.055e-6` 同量级，不是 0°→180° 桩跳角所能单独解释）；0.5 s 内 max|vy| 由 5.039e-4 降到 1.529e-4。固定种子噪声下该指标变差，不能称全面更优。
2. 上述三处统一为当时 INS 航向（与 live_tuning 建轴同源）。无噪声时 vy 与横偏接近 0；噪声下 max|vy| 升到 1.93e-3，差于基线。无实车残差数据前不宣称实车收益。

定距制动将 `sqrt(2*a*r)` 与 `point_plan` 有理化时延公式对照：现行接近律与速度上限下两组结果相同，不作为优先改动。官方套件因固件未改未重跑，仍为 36/37，不改断言。

相对 t206u「推送 main」：远端 main 仍为 `fc76ee3`，与本地 HEAD 一致，可快进、无需强推。但 19 个交付文件中有 8 个叠在用户迁移 WIP 之上（含 `chassis.c/.h`、`jetson_auto.c`、`INS_task.c` 及部分测试），整包提交会带上这些文件里的原 WIP；Jetson、其余 WIP、Nav2 计划文档按要求不纳入。提交范围依赖未拆清前**不提交、不推送**，先报请主线确认范围。

## 九、三轮审查（t205u，仅 box 追加记录；2026-11-19 23:27 CST）

按用户 t205u：算法两候选已告知且未获批准，不集成。对交付基线及实际增量做三轮不同重点审查，不等提交范围确定才审。详细分轮记录见 `results/three-review-20261119-232359/three-review.md`（依赖表 `wip8-dependency.json`，本轮日志 `round3-logs/`）。**不改第四节已有结论、不改第 101 行、不放宽 36/37 断言、不删保护、不落地未批行为修复。不提交、不推送、不再同步电脑。** 本节只追加审查结果。本文件在电脑上仍为同步时的 149 行版本，第 8、9 节均尚未同步。

- 第 1 轮（算法/数学/时序/坐标与原 WIP 耦合）：本轮已集成增量仍为常量上收、`Encode_Request` 提取、注释与回绕测试安排，未见新的控制律或坐标错误。既有定距航向源不一致维持待批。8 个含原 WIP 的文件中，`INS_task.c` 依赖未纳入的 `BMI088driver.c/.h` 与带 `dt` 的 `heading_yaw_guard`；`test_jetson_auto.c`、`test_jetson_link.c` 依赖未纳入的 `Jetson_CAN_Receive_At`（`jetson_link.c/.h`）。`chassis.c/.h` 与 `test_chassis.c` 彼此配套 `Chassis_Disable_Confirmed`，不依赖未纳入头文件。`jetson_auto.c` 业务代码未调用 `Receive_At`。`manual_test/README.md` 无编译依赖。
- 第 2 轮（结构/接口/单位默认值/文档）：结构未扩目录；待标定 0.0f 与分层超时数字未被增量改写；变更记录第 5 节已覆盖 19 个交付路径。另外两个不在上述 8 文件内的本轮测试（`test_manual_link.c`、`test_usb_link_state.c`）分别依赖未纳入的 `Manual_CAN_Receive_At` 与 WIP `UsbRxBuffer` 签名，单独对 HEAD 无法编译。
- 第 3 轮（本轮实测回归）：官方 fail-fast 仍停在 `:307`；continue-on-fail **36 passed, 1 failed, 37 total**，仅 `jetson_chassis_integration` 失败，`check_module_boundaries` 通过。venv Python 123/123、codec、UART6 脚本、`git diff --check`、夹爪 11870≤12000、ADRC `--verify` 均通过。7 预设 `--clean-first` 全部链接，体积与先前一致。若门禁要求官方套件全绿，当前工作树不能作为全绿合入；更硬的阻断仍是 8 文件原 WIP 范围未确认及其对未纳入文件的编译依赖。审查结束后保持等待。

## 十、全量审查与提交范围（t207u，2026-11-19 23:51 CST）

用户 t207u「全部审查然后全部提交」已解除先前第八、九节所记的 8 文件原 WIP 范围阻塞。按 t205u / t206u：审查后同步电脑、更新本记录、分主题提交并在可快进时推送 main。本节只追加，不改第四节结论、不改第 101 行、不放宽 36/37 断言。未批准算法候选仍不接入。不把本次审查授权理解为已批准新控制律、阈值、ADRC 入固件、查询副作用修复或 USB 方案 B。

详细分轮实证见 `results/full-commit-review-20261119-235000/full-scope-three-review.md`（本目录不纳入 git）。可引用 t205u 对 19 文件增量的三审，但新增 Jetson/驱动/协议范围已在本轮实读并重跑回归。

### 范围与分类

相对 HEAD `fc76ee3` 共 **52** 个实际修改/新增路径：纯原 WIP 32、WIP+本轮增量 8、仅本轮 9、新增 3。排除构建产物、`results/` 实验与未集成候选、缓存、工具和凭据。`docs/nav2-global-takeover-plan.md` 作为既有工程**设计文档**纳入（文首已标明提案待批），不实施其中的代码改造。

### 本轮回归（23:50–23:51 CST，不是全绿）

官方 `run_native.py` fail-fast 仍停在 `test_jetson_chassis_integration.c:307`，断言未改。continue-on-fail **36 passed, 1 failed, 37 total**，仅该套件失败，`check_module_boundaries` 通过。venv Python 123/123、codec 下行 155 / 上行 33、UART6 焦点脚本、`git diff --check`、夹爪 11870≤12000、ADRC `--verify <仓库根>` 均通过。Jetson 宿主 `run.sh`（xPack g++，无 GPU）bridge_pkg 418、BridgeRuntime 201、CDC 51、S288 timing/故障用例全部通过，不冒充板端或 ROS 运行时。7 个固件预设 `--clean-first` 全部链接，体积与哈希与 23:26 产物相同。用户已知单一官方失败并要求全部提交；提交说明必须写明 36/37，不得伪称通过。原 WIP 依赖在本 52 文件集合内闭合。未做实车烧录。

### 逐文件记录（路径、类别、函数/逻辑、参数、行数、测试）

行数为相对 HEAD 的插入/删除（未跟踪文件写新增行数）。类别：纯 WIP = 冻结副本即当前内容且相对 HEAD 已改；WIP+本轮 = 冻结已有 WIP 后又叠本轮增量；仅本轮 = 冻结等于 HEAD；新增 = HEAD 无此路径。

**Jetson（8，全部纯原 WIP）**

- `Jetson/src/bridge_pkg/gripper/s288_servo.cpp`（+5/−1）：`Session::run` 在最终 Stop 报错、超时或取消时抛错。参数影响：不改 kp/kd/行程。测试：`test_s288_timing` 新增最终故障 8 项通过。
- `Jetson/src/bridge_pkg/node/bridge_io.cpp`（+3/−1）：`sendPeriodicFrame` 在运动租约暂停期内不发周期流。配合 300 ms 暂停。
- `Jetson/src/bridge_pkg/node/bridge_node.hpp`（+2/−0）：`motion_lease_break_time_=300ms`、`motion_lease_suspended_until_`。300 ms 刻意长于 STM32 250 ms 断流。
- `Jetson/src/bridge_pkg/node/bridge_session.cpp`（+12/−1）：`openSocket` / `missionRestarted` 启动租约暂停；`handleArrivalPoint` 按阶段与租约计算 `authorized` 再调用到达点。测试：`test_bridge_runtime` 覆盖重启后不能用空中心续期、到达点授权。
- `Jetson/src/bridge_pkg/protocol/arrival_point.hpp`（+3/−2）：`onRequest(..., authorized=true)`；新动作在未授权时拒绝，已知编号仍可 Resend。测试：`test_arrival_point`。
- `Jetson/src/bridge_pkg/test/host/test_arrival_point.cpp`（+2/−1）：未授权拒绝与 Resend。
- `Jetson/src/bridge_pkg/test/host/test_bridge_runtime.cpp`（+29/−5）：到达点与周期流暂停用例；`receive` 增加 value 参数。
- `Jetson/src/bridge_pkg/test/host/test_s288_timing.cpp`（+20/−0）：最终 Stop 硬件错误或超时 → Failed。本轮宿主测试通过。板端/GPU：未测。

**STM32 通信、手动/UART6、驱动、航向（纯原 WIP，此前未纳入 19 文件清单的部分）**

- `STM32/manual_test/firmware/manual_link.c`（+6/−1）、`.h`（+1/−0）：新增 `Manual_CAN_Receive_At`，原 `Receive` 转调并填 `xTaskGetTickCount()`。闭合 `test_manual_link.c`。
- `STM32/manual_test/firmware/manual_usb.c`（+2/−2）：`UsbLink_OnFrame` 改为双参并调用 `Receive_At`。
- `STM32/manual_test/firmware/uart6_manual.c`（+16/−4）：返航最短时间拒绝（`DETAIL_RETURN_SPEED`）；状态 bit5=`Chassis_Disable_Confirmed()`；速度帧走 `Receive_At`。参数：不改 `RETURN_TIMEOUT_MS` 数值，只增加期限可行性判断。测试：`test_uart6_manual`、UART6 网页脚本。
- `STM32/manual_test/uart6/uart6_manual.py`（+5/−2）、`uart6_protocol.py`（+2/−1）、`uart6_web.py`（+2/−1）、`uart6_web/app.js`（+4/−0）：协议名 `DISARM_CONFIRMED`、`DETAIL_RETURN_SPEED` 文案、网页「待电机确认」。测试：Python 123 中 UART6 用例与 JS 焦点脚本。
- `STM32/project/Core/Algorithm/Chassis/heading_yaw_guard.c`（+5/−7）、`.h`（+3/−2）：`Update` 增加 `dt`，预测 `gyro*dt`。参数：跳点限仍 `0.087…rad`，拒绝 `dt` 越界。测试：`test_heading_eskf`、`test_sensors`。
- `STM32/project/Core/Communication/Common/usb_link.c`（+25/−3）、`usb_link_handler.h`（+1/−1）、`usb_rx_buffer.h`（+16/−2）：到达时刻、`USB_RX_MAX_AGE_MS=100`、过期 Reset+`OnDisconnect`、`OnFrame` 带起始时刻。100/250 分层耦合仍为已知风险，不在本轮改方案。测试：`test_usb_rx_buffer`（含回绕）、`test_usb_link_state`、`test_manual_control`。
- `STM32/project/Core/Communication/Jetson/jetson_link.c`（+6/−1）、`.h`（+1/−0）、`jetson_usb.c`（+2/−2）：`Jetson_CAN_Receive_At`；USB 回调传时刻。闭合自动侧测试。
- `STM32/project/Core/Devices/BMI088/BMI088driver.c`（+18/−1）、`.h`（+2/−0）、`BMI088reg.h`（+1/−0）：`BMI088_Set_Accel_Calibration_Mode`；新增 `BMI088_ACC_200_HZ`。运行仍 800 Hz。测试：`test_sensors` 寄存器与间隙场景。
- `STM32/tests/native/test_heading_eskf.c`（+11/−11）：门控调用补 `dt=0.01f`，断言集合不变。
- `STM32/tests/native/test_manual_control.c`（+2/−2）：`OnFrame` 双参。
- `STM32/tests/native/test_sensors.c`（+30/−16）：期望改为 `dt*world_z`；标定带宽与 `ROTATING_SPI_GAP`。
- `STM32/tests/native/test_uart6_manual.c`（+27/−1）：失能确认与返航拒绝相关用例。
- `STM32/tests/native/test_usb_rx_buffer.c`（+50/−32）：新签名与到达回绕。

**WIP+本轮增量（8）**

- `STM32/manual_test/README.md`（+2/−2）：文档。原 WIP 说明之上的本轮构建预设表述。无编译依赖。
- `STM32/project/Core/Control/Chassis/chassis.c`（+164/−34）、`chassis.h`（+17/−2）：原 WIP 含 `Chassis_Disable_Confirmed`、定距/航向既有逻辑；本轮增量为常量上收等等价整理（见第五节）。测试：`test_chassis` 及官方套件（36/37）。参数宏上收，数值未改作新阈值。
- `STM32/project/Core/Control/Jetson/jetson_auto.c`（+89/−14）：原 WIP 自动流程；本轮抽出 `Encode_Request`（编码 1,644,775 组对照已记录）。测试：`test_jetson_auto`。
- `STM32/project/Core/Task/INS/INS_task.c`（+17/−4）：原 WIP 调用标定切换与 5 参数航向门控；本轮对 `read_ist8310` 注释等价整理。依赖已随驱动/门控纳入而闭合。
- `STM32/tests/native/test_chassis.c`（+254/−4）、`test_jetson_auto.c`（+74/−20）、`test_jetson_link.c`（+13/−1）：原 WIP 测试叠本轮回绕时钟安排、`Receive_At` 及手动用例迁出。官方 `:307` 不在这些文件内改断言。

**仅本轮（9）+ 新增测试**

- `STM32/README.md`（+2/−2）、`STM32/docs/JETSON_USB_TEST.md`（+1/−1）：构建/测试说明。
- `STM32/manual_test/live_tuning/adrc/README.md`（+16/−1）、`yaw_adrc_pretest.py`（+137/−41）：初测脚本增加训练集/留出集与 `--verify`，**不改固件 ADRC 参数、不写入固件**。测试：`test_yaw_adrc_pretest.py`（新增 117 行）及 `--verify` 通过。
- `STM32/project/CMakeLists.txt`（+13/−0）、`CMakePresets.json`（+22/−0）：模式互斥与预设。7 预设本轮全部链接。
- `STM32/tests/native/test_manual_link.c`（+13/−0）：迁入并调用 `Manual_CAN_Receive_At`（依赖已闭合）。
- `STM32/tests/native/test_usb_link_state.c`（+6/−6）：对齐新环形缓冲签名。
- `STM32/tests/run_native.py`（+49/−2）：宿主运行器维护。

**新增工程文档**

- `docs/optimization-changes-2026-11-19.md`（本文件，追加至第十节）：统一逐文件记录。电脑上次同步为 149 行版本，同步后应包含第八至十节。
- `docs/nav2-global-takeover-plan.md`（新增 278 行）：Nav2 全局接管**设计提案**，待单独批准才能改代码；本次只作为工程文档提交，不实施。

### 已知风险（提交后仍在）

1. 官方集成测试 `:307` / 36/37，契约未闭合，不放宽断言。
2. 定距锁轴用预测 ESKF、`Hold_Yaw` 用 INS、live_tuning 另建验收轴，航向源统一候选未批准。
3. `Chassis_Motion_Ready` 查询副作用、USB 100 ms 解析过期触发 `OnDisconnect` 从而毁掉 250 ms 租约：行为修复方案未批准。
4. 部分默认距离仍为待标定 `0.0f`。ADRC 仅离线初测。未做实车与 GPU 验证。
