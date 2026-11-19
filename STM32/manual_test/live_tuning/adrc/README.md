# 偏航 ADRC 初测

2026-11-19，底盘固件基准为 `9142869`，电机 MIT Kd 为 `0.22 / 0.27 / 0.22`。

## 离线对照

在 100 Hz 的一阶偏航模型中测试 ±0.32、±0.40 rad/s 的起步、稳速、换向和停车。模型覆盖增益和时滞变化、阻力、外部扰动及测量噪声。用 12 个工况选择参数，再用 116 个独立工况评价。三组使用相同的指令斜坡、同向限幅和零速逻辑。

ADRC 用上一拍实际角速度指令预测，再用当前角速度校正二阶扩张状态观测器。初测参数为 `b0=8 s⁻¹`，控制带宽 `16 rad/s`，观测带宽 `30 rad/s`。这些是仿真搜索结果。[离散 ADRC 与输出限幅](https://arxiv.org/abs/1908.04610)

| 留出工况指标 | 当前阻尼 | 重新整定的比例阻尼 | 限幅 ADRC |
| --- | ---: | ---: | ---: |
| 稳速段平均跟踪 RMSE（rad/s） | 0.091 | 0.075 | 0.069 |
| 超速 p95（rad/s） | 0.163 | 0.131 | 0.118 |
| 平均停车时间（s） | 0.525 | 0.521 | 0.525 |
| 稳速段指令累计变动（rad/s） | 0.674 | 1.210 | 1.344 |

ADRC 在 35 个工况的跟踪误差高于当前阻尼，稳速指令累计变动约翻倍。重载欠速组受同向限幅约束，跟踪误差基本相同。停车用模型角速度进入 ±0.02 rad/s 并保持作为判据。

## 实车日志回放与板端诊断

用已有实车旋转数据进行单步预测回放，观测器输出只参与离线计算。较早的 20 Hz 数据中，四趟预测 RMSE 为 0.100–0.110 rad/s，低于保持上一拍测量的 0.111–0.127 rad/s。较新的 10 Hz 数据中，四趟预测 RMSE 为 0.203–0.326 rad/s，高于保持上一拍的 0.168–0.246 rad/s。回放只能比较预测残差，实车闭环效果尚未验证。下次整定需要同步采集 100 Hz 陀螺角速度及实际下发角速度。

IMU 未就绪时，正式配置两次上电共因加速度模长极差超限拒绝 304 个窗口。关闭加热后仍复现。临时设为 200 Hz 输出、20 Hz 带宽后，首次 48 点静止校准通过，极差为 0.072 m/s²，原门槛为 0.50 m/s²。物理干扰来源仍未定位。诊断后恢复正式 USART6 固件，并确认底盘失能。ADRC 未接入控制固件。

## 主机回归

下列命令复算已记录参数，不重新搜索，也不改写初测结果。被控对象仍是上述一阶仿真模型，指标不能当成实车表现。当前固件阻尼的无硬件闭环在 `STM32/tests/native/test_chassis.c` 的 `test_yaw_rate_first_order_plant` 中运行真实 `Apply_Yaw_Rate_Feedback`。

```sh
python STM32/manual_test/live_tuning/adrc/yaw_adrc_pretest.py . --verify
python -m unittest STM32.tests.test_yaw_adrc_pretest
```

若从仓库根目录发现模块路径不同，也可：

```sh
python -m unittest discover -s STM32/tests -p test_yaw_adrc_pretest.py
```

## 复现

从仓库根目录运行，输出文件使用新路径。脚本依赖 Python 3.9 或更新版本及标准库。完整搜索会重选参数；日常核对请用 `--verify`。

```sh
python STM32/manual_test/live_tuning/adrc/yaw_adrc_pretest.py . adrc-results.json
```

[初测结果](results.json) · [回放输入](replay_inputs.json) · [IMU 诊断计数](imu_startup_results.json)
