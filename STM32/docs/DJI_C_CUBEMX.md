# 大疆 C 板当前配置与业务接入

2026-11-05：以用户工程为基础，接入 BMI088、IST8310、教程姿态解算、RTOS 任务和 CAN1 达妙 MIT 收发（按当前接线暂时切换），USART6 向 VOFA 输出姿态角。现已按用户授权移植三轮运动解算与旧车动作，暂不接手柄，不启用 HC-08、JY901P、USB 或看门狗。USART6 TX DMA 的初始化与中断由 CubeMX 生成，原重复 USART BSP 已移除。已接入 RGB LED、板卡电压 ADC3、蜂鸣器 TIM4 PWM 接口和 BMI088 加热 TIM10 PWM 业务；蜂鸣器保持关闭。底盘使用 CAN1、1 MHz TIM2 和现有 test 任务；CAN1 使用板载收发器实际连接的 PD0/PD1，按用户授权调整为 1 Mbit/s 并生成 RX0 中断，无 DMA 或新增任务。

## 已核对的引脚

依据用户提供的 `D:/robotmaster学习/RoboMaster  开发板 C 型用户手册.pdf`：PDF 第13页（印刷第11页）UART针序与丝印说明、第19～20页（印刷第17～18页）引脚表、第18页（印刷第16页）传感器电路。与官方主板原理图 MCU/IMU 页及 `.ioc`、`main.h`、`gpio.c`、`spi.c`、`i2c.c`、`can.c`、`usart.c` 交叉核对。

| 用途 | 引脚 | 当前模式与状态 |
|---|---|---|
| BMI088 SPI1 SCK / MISO / MOSI | PB3 / PB4 / PA7 | AF5，推挽，无上下拉 |
| BMI088 加速度片选 / 陀螺仪片选 | PA4 / PB0 | 推挽输出，初始高，低有效 |
| BMI088 加速度 DRDY / 陀螺仪 DRDY | PC4 / PC5 | 普通输入，无上下拉，无 EXTI |
| BMI088 加热控制 | PF6 | TIM10_CH1，AF3，推挽、下拉、Low；200 Hz，初始化 Pulse=0，INS 任务启动并控制 |
| 蜂鸣器 | PD14 | TIM4_CH3，AF2，推挽、下拉、Low；4 kHz，初始化 Pulse=0，默认关闭，test 任务不启动其 PWM |
| 蓝 / 绿 / 红 LED | PH10 / PH11 / PH12 | 普通推挽输出、无上下拉、Low；高亮低灭，初始全部低电平 |
| 板卡电压采样 | PF10 | ADC3_IN8，模拟输入、无上下拉 |
| IST8310 I2C3 SCL / SDA | PA8 / PC9 | AF4，开漏，无内部上下拉；板载 4.7 kΩ 上拉 |
| IST8310 RSTN | PG6 | 推挽输出，初始高、上拉；低复位 |
| IST8310 DRDY | PG3 | 普通输入、上拉，无 EXTI |
| CAN1 RX / TX（当前电机） | PD0 / PD1 | AF9，推挽，无上下拉 |
| CAN2 RX / TX（保留配置） | PB5 / PB6 | AF9，推挽，无上下拉；业务不启动 |
| USART6 TX / RX | PG14 / PG9 | AF8，推挽，无上下拉，Very High；J29三针接口，当前仅发送，未启动接收 |
| HSE | PH0 / PH1 | 板载 12 MHz 晶体 |
| SWDIO / SWCLK | PA13 / PA14 | Serial Wire，释放 PB3/PB4 给 SPI |

注意 I2C3 是 **PA8=SCL、PC9=SDA**。教程部分文字存在颠倒，以上按用户手册引脚表和原理图接线确认。CAN2 的板载收发器无需额外使能 GPIO。电机和 Jetson 共用 CAN1，滤波组使用 bank 0 和 1，CAN2 的 bank 14 不启用。两路滤波分界仍为 14。

新增四项按用户手册 PDF 第8、16～18页，以及官方主板原理图 PDF 第3～5页、陀螺仪 FPC 原理图第1页核对。RGB LED 经 MOS 管驱动，因此 GPIO 高电平点亮；蜂鸣器和加热管也均为高有效。电压分压上臂 R72=200 kΩ、下臂 R83=22 kΩ，后续换算应使用 `Vboard = Vadc × (200 + 22) / 22`，倍率约10.0909，不是11。

## CubeMX 已生成设置

| 项目 | 当前值 |
|---|---|
| MCU / 固件 / 工具链 | STM32F407IGH6 / STM32Cube FW_F4 V1.28.3 / CMake、GCC |
| HSE / PLLM / PLLN / PLLP | 12 MHz / 6 / 168 / 2 |
| SYSCLK / HCLK | 168 MHz / 168 MHz |
| PCLK1 / APB1 定时器 | 42 MHz / 84 MHz |
| PCLK2 / APB2 定时器 | 84 MHz / 168 MHz |
| 电源 / Flash | Scale 1 / 5 WS |
| PLLQ | 当前4，对应84 MHz；本阶段不用USB/SDIO/RNG，日后启用前改7得到48 MHz |
| HAL 时基 | TIM6，1 ms，IRQ 优先级15 |
| RTOS 时基 | SysTick，1000 Hz；SysTick/PendSV 优先级15，SVC默认0 |
| NVIC | Priority Group 4；FreeRTOS最低优先级15，允许系统调用边界5 |
| SPI1 | 主机、双线、8 bit、MSB、Mode 3、软件NSS、/256=328.125 kHz；无DMA/IRQ |
| I2C3 | 400 kHz、Duty 2、7 bit、OwnAddress 0；无DMA/IRQ |
| TIM2 | 内部时钟，PSC=83，ARR=0xFFFFFFFF，向上，无IRQ；业务启动后1 μs计数 |
| TIM4 蜂鸣器 | 内部84 MHz，PSC=83、ARR=249，PWM Generation CH3、PWM1、高极性、Pulse=0、Fast Disable、预装载关闭；4 kHz，无DMA/IRQ |
| TIM10 加热 | 内部168 MHz，PSC=167、ARR=4999，PWM Generation CH1、PWM1、高极性、Pulse=0、Fast Disable、预装载关闭；200 Hz，无DMA/IRQ |
| ADC3 板卡电压 | PCLK2/4=21 MHz，12位、右对齐，IN8/Rank1、480周期；单通道、软件单次触发，扫描/连续/间断模式关闭，单次EOC，无DMA/IRQ |
| CAN1 / CAN2 | Normal，PSC=3、BS1=10、BS2=3、SJW=1，1 Mbit/s |
| CAN1 / CAN2 自动功能 | 自动重传与自动Bus-Off恢复开启；自动唤醒/时间触发关闭 |
| CAN1 / CAN2 NVIC | 仅 RX0，抢占优先级6、子优先级0 |
| USART6 | 已生成115200、8N1、无流控、16倍采样、TX_RX；本业务只使用TX |
| USART6 NVIC | 已生成全局中断，抢占优先级5、子优先级0；处于当前RTOS允许的范围 |
| USART6 TX DMA | DMA2 Stream7 / Channel5，Memory to Peripheral，Normal，Byte/Byte，仅Memory Increment，Low，FIFO Disable |
| DMA2 Stream7 NVIC | 抢占优先级6、子优先级0，生成HAL中断转交 |
| FreeRTOS | CMSIS-RTOS v1，heap_4，24576字节堆，最大优先级7，动态+静态分配 |
| 固定周期接口 | `INCLUDE_vTaskDelayUntil=1`，任务用 `vTaskDelayUntil` 与 `pdMS_TO_TICKS` |
| `imuTask` | 入口 `INS_task`、High、1024 words（4096字节）、Dynamic、As weak |
| `test` | 入口 `test_task`、Normal、512 words（2048字节）、Dynamic、As weak |
| 生成设置 | 按外设独立c/h，保留USER CODE，C堆0x200、主栈0x1000 |

`.ioc` 包含 `Dma.Request0=USART6_TX`、DMA2 Stream7参数及NVIC条目。CubeMX根据USART6_TX与Stream7自动选择Channel5；在 USART6 → DMA Settings 或 System Core → DMA → DMA2 中可见。若CubeMX窗口在修改前已打开本工程，需要重新打开 `project/project.ioc` 加载磁盘配置。

生成文件的职责：`dma.c` 中 `MX_DMA_Init` 开启DMA2时钟并配置IRQ；`usart.c` 中 `HAL_UART_MspInit` 初始化 `hdma_usart6_tx` 并关联 `huart6.hdmatx`；`stm32f4xx_it.c` 中 `DMA2_Stream7_IRQHandler` 转交 `HAL_DMA_IRQHandler`。`USART6_IRQHandler` 转交 `HAL_UART_IRQHandler(&huart6)`，负责Normal DMA发送后的TC收尾。`main.c` 在USART6初始化之前调用 `MX_DMA_Init`，全部外设初始化完成后才启动RTOS。

外设初始化由 CubeMX 管理：`adc.c/.h` 提供 `hadc3` 和 `MX_ADC3_Init`；`tim.c/.h` 提供 `htim4`、`htim10` 与初始化、时钟及复用引脚配置；`gpio.c` 设置三色 LED 初始低电平；`main.c` 在启动 RTOS 前调用外设初始化。ADC 启动/采样/停止、PWM 启动和占空比调节放在独立 BSP 与任务文件中。下次重新生成仍从 `.ioc` 恢复配置，通过两个强任务入口接入业务，无需手工补生成文件。

已重新打开本机 CubeMX 工程，在 Timers → TIM4/TIM10 确认 PWM 通道及 PSC/ARR，在 Analog → ADC3 确认 IN8、PCLK2/4 和12位分辨率。其他未选 ADC 通道可能因其引脚已用于 SPI、片选或 PWM 而不可选，不影响当前 PF10/IN8。

顶层 `CMakeLists.txt` 编译独立业务目录；`cmake/stm32cubemx/CMakeLists.txt` 由CubeMX维护生成源文件列表。算法、VOFA、驱动和任务不放在生成目录中；两个强任务覆盖 `freertos.c` 中的弱任务。保留 `KeepUserCode=true` 和任务 `As weak` 设置，重新生成后无需手工补回业务调用。原 `Core/Bsp/USART` 已删除，不存在第二份DMA初始化或中断。

## 教程对照与当前写法

传感器以官方教程第11、13章及配套例程为准，CAN外设流程以第14章为准，任务入口沿用第18章。达妙电机不属于教程的DJI电机，因此协议内容另按 `D:/robotmaster学习/调试助手使用说明书（达妙驱动控制协议）V1.4.pdf` 第32～37页实现。本轮按用户最新授权读取旧小车当前源码，迁移三轮计算、动作及参数；仅参考网上开源框架的组织方式，不搬旧主控初始化或整套框架。

上一版并非严格抄写：它增加了命令邮箱、保护状态机、诊断快照、传感器错误锁存和数据时效判断。这些额外层已经删除，不再保留 `Task_Motor_Request`、`Task_Check_Motors`、`Task_Stop_Motors`、`Task_Get_Diagnostics` 或 `BSP_Time_*`。相关旧接口与测试同步移除。

下表文件路径均相对于 `project/Core/`，模块目录树见仓库 README。

| 文件 / 函数 | 参考与保留内容 |
|---|---|
| `Devices/BMI088/BMI088driver.c`：`BMI088_init`、`bmi088_accel_init`、`bmi088_gyro_init` | 教程第13章配套例程的复位、读取ID、配置、回读流程；保留原有初始化错误码 |
| 同上：`BMI088_read` | 教程第183～184页，保留 `void` 返回、逐轴换算、温度符号扩展和gyro ID判断；为加热有效性增加accel ID及本次HAL读取失败检查，失败温度为NaN |
| 同上：`BMI088_write_single_reg`、`BMI088_read_single_reg`、`BMI088_read_muli_reg`及accel/gyro宏 | 第13章原寄存器访问与SPI dummy流程 |
| `Devices/BMI088/BMI088Middleware.c`：`BMI088_GPIO_init`、`BMI088_com_init` | 保留教程空接口，由CubeMX完成GPIO/SPI初始化 |
| 同上：`BMI088_delay_ms/us`、`BMI088_ACCEL_NS_L/H`、`BMI088_GYRO_NS_L/H`、`BMI088_read_write_byte` | 第13章中间层函数；适配本工程引脚标签、RTOS毫秒延时和TIM2微秒延时 |
| `Devices/IST8310/ist8310driver.c`：`ist8310_init`、`ist8310_read_over` | 教程第145～146页及第11章配套例程；恢复原status/mag结构、STAT1判断和三轴换算 |
| `Devices/IST8310/ist8310driver_middleware.c`：`ist8310_GPIO_init`、`ist8310_com_init`、`ist8310_RST_L/H`、`ist8310_delay_ms/us` | 第11章中间层接口；GPIO由CubeMX生成，延时按当前RTOS/TIM2适配 |
| 同上：`ist8310_IIC_read_single_reg`、`ist8310_IIC_write_single_reg`、`ist8310_IIC_read_muli_reg` | 第11章I2C中间层；单读/写签名保留，连读同名函数改返回uint8_t（HAL成功1、失败0），将本次HAL结果交给调用者，不加错误锁存 |
| `Bsp/CAN/bsp_can.c`：`can_filter_init` | 沿用官方 `14.CAN/bsp/boards/bsp_can.c` 的启动和 FIFO0 通知。CAN1 的 bank 0、1 使用标准数据帧列表，覆盖电机反馈和 Jetson 下行。 |
| `Devices/DM_Motor/dm_motor.c`：`HAL_CAN_RxFifo0MsgPendingCallback` | 教程第197～198页的 HAL 回调，按达妙手册第32、35页解码。每次完整更新后递增该电机的反馈序号，供底盘识别新样本 |
| 同上：`uint_to_float`、`float_to_uint`、`ctrl_motor` | 达妙协议手册第33～34页；保留换算公式和8字节MIT布局，发送返回HAL状态用于底盘处理邮箱提交失败 |
| 同上：`enable_motor`、`disable_motor` | 达妙协议手册第37页FC/FD字节表的直接封装；手册未提供函数名，这两个名称属于必要接入，不是教程原函数 |
| `Algorithm/AHRS/MahonyAHRS.c`：`MahonyAHRSupdate`、`MahonyAHRSupdateIMU`、`invSqrt` | 以第18章C源码为基础；磁场非零时参与九轴反馈；增加实测dt参数用于四元数积分及积分反馈，零磁六轴分支同步使用dt；invSqrt保留接口改为1.0f/sqrtf(x)，是本工程数值适配，非战队源码原文 |
| `Algorithm/AHRS/AHRS.c`：`AHRS_init`、`AHRS_update`、`get_angle` | 第18章 INS 接口与角度公式。初始重力对准参考 basic_framework 的 InitQuaternion，六轴初始航向为零，九轴使用倾斜补偿后的本板磁场定向。实测时间步长参考 INS_Task 和 IMU_QuaternionEKF_Update |
| `Task/INS/INS_task.c`：`Calibrate_MPU_Offset` | 参考 basic_framework/modules/imu/BMI088driver.c 同名函数的静止极差和均值判据。本车每 10 ms 采一组 BMI088，共 48 组。九轴模式还在同一窗口收集 IST8310 的新磁场样本 |
| `Task/INS/INS_task.c`：`imu_cali_slove` | 第20章标准机器人INS任务的坐标转换函数和安装矩阵；BMI先扣芯片轴零偏再旋转，顺序参考basic_framework的BMI088_Read和INS_Task；磁场适配为A*(raw-b)，参数由电脑参考PX4模型离线求解，当前单位安装矩阵已合入A；原始数据不改 |
| `Task/INS/INS_task.c`：`INS_task`；`Task/CAN2/test_task.c`：`test_task` | 沿用第18章任务入口。INS 启动温控与静止标定，完成初始姿态后按 TIM2 实测 dt 融合并发送 VOFA。test 任务以 5 ms 循环调度底盘、LED 和测压，蜂鸣器关闭，底盘默认失能 |
| `Communication/VOFA/vofa.c`：`vofa_send` | VOFA官方JustFloat帧格式；必要的发送封装，教程没有这个协议函数。使用现有HAL_UART_Transmit_DMA，不添加任务、队列或printf重定向 |
| `Algorithm/PID/pid.c`：`PID_init`、`PID_calc`、`PID_clear` | 官方第20章 `components/controller/pid.c`，裁剪为温控实际使用的位置式，保留误差、P/I/D与输出限幅；类型使用等价float |
| 同上：`PID_Step` | 旧车 `Core/Control/PID/pid.c` 连续积分、测量微分及抗饱和公式；复用现有结构，与温控使用独立实例 |
| `Algorithm/Chassis/motor_chassis.c` | 旧车 `Core/Motor/motor_chassis.c/.h` 三轮正逆解与几何；保留原函数名，2026-11-08 按用户确认的实车方向校正正逆解纵向符号 |
| `Control/Chassis/chassis.c` | 旧车 `Core/Control/chassis/straight`、`translation`、`turn` 的动作计算与参数；合并为单个周期控制模块，F407发送/计时为本工程适配 |
| `Task/INS/INS_task.c`：`INS_Get_Data` | 底盘使用的一致快照接口；世界Z角速度含义对应旧车 `Core/IMU/imu_attitude.cpp` 的 Publish，新增接口名与RTOS短临界区为适配 |
| `Bsp/IMU_PWM/bsp_imu_pwm.c`：`imu_pwm_set`；`Task/INS/INS_task.c`：`imu_temp_control` | 第18章加热 PWM 与温控函数，目标 45 ℃。本工程按 100 Hz 温控换算 Ki，采集开机零偏时也持续调节加热功率 |
| `Bsp/LED/bsp_led.c`：`bsp_led_toggle` | 官方 `2.flash_light/Src/main.c`，保留三色GPIO一起翻转，只适配引脚标签 |
| `Bsp/Buzzer/bsp_buzzer.c`：`buzzer_on`、`buzzer_off` | 官方第5章同名函数与签名，适配当前HAL宏；PSC/CCR直接写入，默认关闭；保留手动调用，发声前须先启动TIM4_CH3 PWM |
| `Bsp/ADC/bsp_adc.c`：`adcx_get_chx_value`、`get_battery_voltage` | 官方第7章单次转换和分压换算；内部helper适配为状态返回与输出参数，保留已生成的480周期配置；裁掉依赖ADC1的基准校准/芯片温度/硬件版本读取 |

官方源文件：

- [BMI088驱动](https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/13.spi_bmi088/component/devices/BMI088driver.c)、[BMI088中间层](https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/13.spi_bmi088/component/devices/BMI088Middleware.c)
- [IST8310驱动](https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/11.ist8310/bsp/boards/ist8310driver.c)、[IST8310中间层](https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/11.ist8310/bsp/boards/ist8310driver_middleware.c)
- [CAN滤波初始化](https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/14.CAN/bsp/boards/bsp_can.c)、[任务入口](https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/18.ins_task/Src/freertos.c)
- [第18章INS任务与AHRS接口](https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/18.ins_task/application/INS_task.c)、[Mahony源码](https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/18.ins_task/components/algorithm/MahonyAHRS.c)
- [第20章INS坐标转换](https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/20.standard_robot/application/INS_task.c)、[VOFA JustFloat协议](https://www.vofa.plus/docs/learning/dataengines/justfloat/)

本次姿态补充参考固定为 `HNUYueLuRM/basic_framework` 的 `1a136eb1ed2f101110348d5a5cd182272eada38e` 版本：

- [Calibrate_MPU_Offset、BMI088_Read](https://github.com/HNUYueLuRM/basic_framework/blob/1a136eb1ed2f101110348d5a5cd182272eada38e/modules/imu/BMI088driver.c)
- [InitQuaternion 的重力对准、INS_Task 的实测dt与安装修正](https://github.com/HNUYueLuRM/basic_framework/blob/1a136eb1ed2f101110348d5a5cd182272eada38e/modules/imu/ins_task.c)、[DWT_GetDeltaT 的无符号计数差](https://github.com/HNUYueLuRM/basic_framework/blob/1a136eb1ed2f101110348d5a5cd182272eada38e/bsp/dwt/bsp_dwt.c)
- [IMU_QuaternionEKF_Update 接收 dt 的方式](https://github.com/HNUYueLuRM/basic_framework/blob/1a136eb1ed2f101110348d5a5cd182272eada38e/modules/algorithm/QuaternionEKF.c)，本车将实测步长传给所选的 Mahony 算法。

参考包的 MIT 版权许可保留在 `LICENSES/basic_framework-MIT.txt`。沿用其静止标定、实测 dt 与重力初始化思路。六轴和九轴共用本工程的 Mahony，温控使用官方教程 PID 与 PWM 函数。

这仍是针对本工程的裁剪接入，不能称为整套程序逐字复制。保留的差异是：使用生成的GPIO标签；毫秒延时使用RTOS，微秒延时使用已经配置的1MHz TIM2；仅保留当前传感器量程和SPI读取分支；使用标准float/uint8_t代替等价的fp32/bool_t别名；电机CAN当前只用CAN1；将达妙手册旧版 `hcan->pTxMsg` / `HAL_CAN_Transmit` 改成当前HAL的帧头、数据数组及 `HAL_CAN_AddTxMessage`；三台电机扭矩量程分别配置。基本帧类型、ID和长度判断用于选择正确数据及防止数组越界，保留在接收回调内。

## 任务与数据观察

`INS_task` 启动 TIM2，等待教程规定的 7 ms，再初始化 BMI088 和加热 PWM。仅九轴模式另行初始化 IST8310。每 10 ms 读取一组 BMI088 并执行 45 ℃ 温控，48 组形成 0.48 s 的静止标定窗口。2026-11-05 版本先等 45±1 ℃ 连续 2 s，再静置采集 1000 组，单是这两步至少需要 12 s，另计升温时间。

开机时车体须静止。合格窗口的加速度模长峰峰值 ≤0.5 m/s²，各轴角速度峰峰值 ≤0.15 rad/s，平均加速度模长与 9.8 m/s² 相差 ≤0.5 m/s²，各轴平均角速度绝对值 ≤0.01 rad/s。陀螺零偏只存 RAM，每次启动重测。两种模式遇到晃动都重新采集整窗。九轴还要求窗口内取得 STAT1 标有新数据的磁场样本，且能求出水平磁航向。

`AHRS_init` 从窗口内的平均加速度求初始 roll、pitch。默认六轴的 yaw 从零起算，表示相对开机朝向的转角。九轴将同窗磁场按本板参数校正并作倾斜补偿，初始 yaw 以现场磁场为参考。底盘和 VOFA 始终使用同一次解算。验收时在同一车头朝向分别冷启动两版，记录上电到 VOFA 首帧用时（目标 ≤1 s）、升温期间的静止 yaw 变化以及电机通电前后 yaw 的变化。0.48 s 只计采样窗口。

2026-11-06 用 11 月 5 日两段暖态静置数据回放同一套 Mahony。两种初始化都取前 48 点估计陀螺零偏。初始 yaw 设为零时，两轮前 10 s 的 yaw 峰峰值分别为 65.01°、63.60°。改用窗口内新磁样本定向后，分别为 0.15°、0.41°。样本温度为 44.625～45.25 ℃，冷态升温中的零偏变化仍待实测。倾斜补偿公式按本车坐标适配。[NXP AN4248](https://www.nxp.com/docs/en/application-note/AN4248.pdf?WT_ASSET=Documentation&fasp=1)

随后任务每 5 ms 唤醒，BMI088 每两个周期读取一次（名义 10 ms）。六轴只使用 BMI088；九轴每 5 ms 另读 IST8310。每个有效的 BMI088 样本触发坐标转换、选定模式的 Mahony 更新与 VOFA 发送。USART6 及其 DMA 已在 RTOS 启动前完成初始化。

`dt`在相邻BMI088读取开始时用已有1MHz TIM2采样，计算 `(uint32_t)(now - last) * 1e-6f` 秒；正常32位计数回绕由无符号差值处理。标定等待不计入首个积分间隔。AHRS_update将实测dt传到Mahony九轴和零磁六轴分支的四元数积分及Ki积分反馈，不再使用固定sampleFreq。Mahony仍为Kp=0.5、Ki=0；`invSqrt`保留接口，内部由原位运算加一次牛顿近似改为 `1.0f / sqrtf(x)`，减少归一化近似误差。这是本工程数值适配，不是战队源码原文，不增加滤波或在线零偏估计。当前传感器采样、融合及发送频率安排保持不变，实际dt用于反映任务调度的时间变化；本轮不修改CubeMX外设或任务配置。

`bmi088_real_data` 保存芯片坐标下的加速度（m/s²）、角速度（rad/s）和温度（℃），九轴另将磁场（µT）存入 `ist8310_real_data`。融合角速度先减静止标定的gyro_offset，再按教程矩阵旋转，原始角速度不会被覆盖。加速度保留重力。九轴模式的磁场在 `imu_cali_slove` 中先减芯片坐标偏移 `mag_offset`，再乘 `mag_scale_factor`。此矩阵已合并离线椭球校准与当前单位安装矩阵；寄存器和原始单位换算不改，禁止在驱动中重复补偿。`ist8310_IIC_read_muli_reg`返回HAL成功1、失败0；失败当轮不调用 `ist8310_read_over` 解码旧缓冲，并清除其数据就绪位。如当轮需要融合，在补偿后将融合输入磁场置零，沿Mahony已有的六轴分支更新；成功读取后继续使用九轴。成功但STAT1无新数据时仍沿教程保留上次磁场值。没有新增超时管理、计数或状态机。

`INS_quat` 是四元数，`INS_angle[0..2]` 依次为 yaw、pitch、roll，单位 rad。底盘通过 `INS_Get_Data` 在短临界区读取同次解算的 yaw（度）、世界 Z 角速度（rad/s）和采样起点 RTOS 毫秒时刻。启动标定期间、BMI 读取失败或恢复首点返回未就绪，底盘检查样本是否在 20 ms 内。装车后需核对航向方向、静止漂移和电机通电时的磁干扰。

调试器可观察 `bmi088_real_data`、`BMI088_error`、`INS_quat`、`INS_angle`。九轴另看 `ist8310_real_data` 和 `ist8310_error`，六轴跳过磁力计初始化和读取。INS_task.c内的静态gyro_offset可查看标定值，imu_temp_pid.out可看温控计数输出，运行循环的dt为秒。BMI088结构中的status/time未使用；IST8310结构的status采用教程bit2标志（0x04），不是原始STAT1。BMI088当前HAL读取失败或芯片ID异常通过NaN温度传递；底盘只增加所需的三字段快照，没有增加诊断计数、自动重初始化或完整健康监测。

`test_task`调用一次 `can_filter_init`、`Chassis_Init` 和 `buzzer_off()`，保持蜂鸣器关闭，不启动TIM4 PWM；随后仍用5ms固定周期调用 `Chassis_Loop`，底盘内部每10ms运行一次。CAN1接收中断直接更新 `dm_motor[0..2]`，对应ID1/2/3。启动不发送FC、FD或运动目标，必须先明确调用 `Chassis_Enable`；所有底盘动作接口与循环均由这个任务调用，不从ISR或多个任务同时写控制状态。

## 板载温控、提示与电压

温控目标45℃来自官方第18章。沿用位置式PID的Kp=1600、Kd=0、输出上限4500、积分上限4400；原例程温度由800Hz加速度DRDY触发，本工程每10ms控制一次，因此Ki初值由0.2按周期换算为1.6。PWM载波仍为200Hz，控制更新100Hz，两者不同。参数只是教程迁移初值，需上板观察升温、超调与稳态波动再调节。

`imu_temp_control`从启动即计算PID，输出负值置0，PWM计数上限4500小于当前周期5000。读取失败、温度无效或温度达到50℃时清除PID并写PWM=0；50℃为本工程目标温度以上5℃的关热阈值，低于-40℃按异常低温处理。-40℃依据[Bosch BMI088工作温度范围](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)，并非新增温度目标。TIM10比较寄存器使用预装载，写入0后在下一200Hz PWM周期生效，最多再经过一个5ms载波周期。

BMI088使用本次SPI事务错误标志传递HAL失败，单字节超时参数由1000ms缩为2ms（正常单字节约24.4μs）；首个失败后跳过该次剩余HAL收发，返回NaN温度。额外核对提供温度的accel芯片ID，防止accel离线但gyro仍响应时误把全零字节解释成23℃。初始化入口和每次采样入口都清除该次错误标志，失败不会锁死后续重试。SPI或ID异常的样本不用于姿态更新或VOFA，恢复首点只重建计时基准，不用一个新角速度补算整段缺测时间。这些处理直接在现函数中完成，没有新增保护状态机、诊断任务或重初始化框架。

三色LED沿用教程一起翻转，每500ms一次，即1Hz闪烁；灯光指示低优先级任务在运行，姿态就绪由 INS_Get_Data 或首个 VOFA 帧确认。蜂鸣器默认关闭，任务不启动其PWM。保留 `buzzer_on`、`buzzer_off` 供手动使用：先调用 `HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3)`，再用 `buzzer_on(83,125)` 产生4kHz、50%占空比，调用 `buzzer_off()` 将比较值清零停止发声。

`battery_voltage`每100ms更新一次，单位V，可直接在CLion调试器中观察；启动也读取一次，失败值为NaN。`get_battery_voltage()` 使用ADC3的既有通道8、480周期配置，不覆盖为教程默认3周期。换算为 `raw × (3.3/4095) × (222/22)`；3.3V是名义参考，未启用ADC1内部基准校准，需上板对照万用表调整。该名义量程不是开发板允许的供电上限。温度、电压均不追加进VOFA，保留原16字节三float姿态帧。

来源：[LED第2章](https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/2.flash_light/Src/main.c)、[蜂鸣器第5章](https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/5.buzzer/bsp/boards/bsp_buzzer.c)、[ADC第7章](https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/7.ADC_24V_power/bsp/boards/bsp_adc.c)、[第18章温控](https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/18.ins_task/application/INS_task.c)、[第18章PWM](https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/18.ins_task/bsp/boards/bsp_imu_pwm.c)、[第20章PID](https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/20.standard_robot/components/controller/pid.c)。具体裁剪与适配同时记录在函数注释中。

## VOFA显示与接线

使用 J29 三针接口，外壳丝印 UART1，实际对应 MCU USART6：第1针 GND、第2针 TXD（PG14）、第3针 RXD（PG9）。第2针接3.3V逻辑USB-TTL模块的RX，第1针接GND；板卡单独正常供电，三针接口没有电源脚。当前只发送姿态，第3针无需接线。依据用户手册 PDF 第13页（印刷第11页）的针序与丝印说明、第19页（印刷第17页）的IO表；原理图 PDF 第5页、第7页亦一致。原先使用的四针接口方案不再用于本工程VOFA输出。

VOFA选择USB-TTL对应COM口，115200、8数据位、无校验、1停止位、无流控，协议选JustFloat。帧为三个小端float，顺序roll、pitch、yaw，单位度，随后四字节帧尾`00 00 80 7F`。每帧16字节，每10ms一次，理论1600字节/秒；8N1下单帧占用约1.39ms。前三路曲线分别命名roll、pitch、yaw。串口不混发printf文本或其他协议。

本轮临时23-float采样用于同步核对实测dt、去偏角速度、磁场与姿态，测量后删除；它不是最终VOFA协议。最终业务继续使用上述三通道DMA帧，不保留额外采样任务或诊断接口。

`vofa_send`使用静态帧，只有USART6空闲时才填充并开始DMA发送；发送未结束时保留帧内容并跳过本帧，避免DMA读取过程中覆盖。该检查用于异步缓冲区的正确使用，没有新增保护状态机。未增加独立VOFA任务，也未启动串口接收。

## 达妙收发接口

使用用户已确认并与旧车当前源码核实的参数：标准控制ID1/2/3，反馈标准ID0；位置范围±12.5 rad、速度范围±40 rad/s，扭矩范围依次为±1/±1/±2 N·m。每台电机独立保存状态、位置、速度、扭矩和两项温度。有效帧在回调中记录到达时刻 `feedback_time_ms`，完整解包后递增本机的 `feedback_seq`，无符号序号回绕到零也算新反馈。

- `enable_motor(&MOTOR_CAN, id)`：发送一次FC使能帧。
- `ctrl_motor(&MOTOR_CAN, id, pos, vel, kp, kd, torque)`：按手册打包并发送一次MIT帧，顺序和单位见头文件。
- `disable_motor(&MOTOR_CAN, id)`：发送一次FD失能帧。

这三个发送接口现在返回 `HAL_StatusTypeDef`，其参数、协议布局和直接HAL调用方式不变；HAL_OK表示进入邮箱，不代表电机已执行。MIT 控制接口会拒绝非有限数、协议范围以外的参数及与保留命令重合的编码；`float_to_uint`按手册公式转换。`Chassis_Disable`直接调用F4的 `HAL_CAN_AbortTxRequest` 撤销旧待发目标；取消异步，底盘等三个邮箱空闲再发送FD，不再保留单次转调HAL的包装函数。CAN1只由底盘控制三台电机，其他代码不同时调用底层发送。未移植旧车电机发现、参数读回、温度/堵转判断、故障状态机或输入源租期。

## 三轮底盘与动作

算法分为 `Algorithm/Chassis/motor_chassis.c` 的三轮正逆解与 `Control/Chassis/chassis.c` 的动作控制。三轮公式、PID 和原始运动参数来自旧车。2026-11-06 将平移目标的加速度由 0.30 调到 0.45 m/s²。从静止将目标升至 0.40 m/s，按配置斜率约需 0.89 s，原值约需 1.33 s。实车可在同一地面分别前进、左移 1 s，比较三轮反馈与地面位移，并检查框口能否稳定对准。设置目标后周期发送 MIT 的组织方式与控制顺序参考 [basic_framework 的 DMMotorTask](https://github.com/HNUYueLuRM/basic_framework/blob/1a136eb1ed2f101110348d5a5cd182272eada38e/modules/motor/DMmotor/dmmotor.c) 和 [RoboMaster 官方 chassis_task](https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/17.chassis_task/application/chassis_task.c)。

用户确认三台电机型号为 DMH3510。坐标保持车体x向前、y向左、wz俯视逆时针为正；轮序为前、左后、右后，对应CAN ID1/2/3。轮半径0.035m，中心距0.108m；方向全部−1，齿比全部1。以 `r=0.035`、`L=0.108`：

```text
前轮   = (vy + L*wz) / r
左后轮 = ( sqrt(3)/2*vx - vy/2 + L*wz) / r
右后轮 = (-sqrt(3)/2*vx - vy/2 + L*wz) / r
```

2026-11-08 用户确认原前进、后退与实车方向相反，现已对调两后轮公式中的纵向符号，反馈正解同步采用 `vx=(left-right)/sqrt(3)`，其中 left、right 为去除电机方向并乘轮半径后的轮缘速度。直行、定距、斜向和连续速度均由同一正逆解处理。纯前进时发给 ID2 的轮速为负、ID3 为正，纯后退反向；各轮指令单位均为 rad/s。

| 参数 | 当前值与含义 |
|---|---|
| MIT速度控制 | position=0、Kp=0、torque=0；Kd依轮序0.2/0.25/0.2，速度闭环在电机内部 |
| 轮速限幅 | ±40rad/s；超限时三轮与车体目标同比缩放，保持运动比例 |
| 定时平移及连续速度的线量参数 | 接口内部合速度上限 0.50 m/s，加速度 0.45 m/s²，斜向分量乘 1/√2 保持等速 |
| 八向定距的线量参数 | 接口内部合速度上限 0.50 m/s，对称加减速度 0.75 m/s²；纯前进对应两后轮最大绝对轮速 12.37 rad/s、角加减速度 18.56 rad/s²，从静止出发并在段末降为零速 |
| 航向PID | Kp=2、Ki=0、Kd=0.25；积分限0.2、输出限0.8rad/s |
| 连续速度 | `wz` 上限 2.0 rad/s，角加速度 1.6 rad/s²，`vx/vy/wz` 共用增量缩放比例 |
| 左右相对转角 | 默认均360°，通过相邻yaw差跨零累加实现整圈 |
| 转向规划 | 车体角速度上限 2.0 rad/s，角加减速度 4.0 rad/s²，按剩余角度制动，最后 5° 交给原 PID |
| 转向PID | Kp=2、Ki=0、Kd=0.1，积分限 0.2，输出限 2.0 rad/s，末段修偏使用原增益 |
| 单独转向完成 | 角差绝对值≤2°、实测车体角速度和待发转向目标绝对值均≤0.08rad/s，连续保持200ms；超时15000ms |
| 控制时基 | 复用5ms任务，每10ms更新；dt取1MHz TIM2无符号差，支持计数回绕 |

按当前上限换算，纯前进两后轮最大绝对目标为 12.37 rad/s，纯横移前轮为 14.29 rad/s，纯转三轮各为 6.17 rad/s。平移和转向同时达到上限时，任意方向的单轮最大绝对目标为 `(0.50 + 0.108 × 2.0) / 0.035 ≈ 20.46 rad/s`。历史版本的六轴手动固件曾烧录并通过校验；本分支和新增速度上限仍待实车验证。

旧车直行是速度＋持续时间，没有距离闭环。旧手动界面的默认值为0.4m/s、3000ms；当前接口显式传入，例如 `Chassis_Straight_Start(0.4f,3000U)`，不保留无人使用的手动默认值宏。duration_ms=0表示持续运行。航向PID使用 `PID_Step`：连续积分 `Ki*error*dt`、微分 `-Kd*world_z_rad_s`，保留旧抗饱和；温控仍调用原 `PID_calc`。平移斜坡和转角误差也保留旧源码的double中间计算后再转float，避免改变舍入顺序。

由调用 `Chassis_Loop()` 的同一任务依次执行 `Chassis_Init()`、`Chassis_Enable()`，随后在后续周期查询 `Chassis_Is_Stopped()`。普通待命启动时，等返回 1 且姿态就绪，再请求定时直行、八向平移、定距、转向或非零连续速度。动作函数返回 0 表示本次请求未被接受，调用方可在下一周期重试。直行和平移在实际接受时锁定航向。转角范围为 (0,360]°。连续速度模式内可直接更新目标。另有一项明确接续：转向剩余角绝对值 ≤5° 时，可直接请求 `Chassis_Distance_Start()`，边开始下一段平移边修正转向余角，具体行为见下文。

`Chassis_Stop()` 与 `Chassis_Set_Velocity(0,0,0)` 都清除运动目标并进入零速待命，重复发零不清除已取得的停车确认。定距、转向正常完成后保持零速待命。定时直行或平移到期先发零速 MIT，确认停车后在下一周期提交失能；等待到 500 ms 仍无确认也会失能。显式失能、转向超时和故障均不能证明三轮已停。运动中沿用 20 ms 的姿态有效期。

F407 的三个发送邮箱按组发送“零 MIT → FC → 零 MIT”使能包序，各组之间相隔一个控制周期。已使能且再次发送三轮零速 MIT 后才开始停车确认。CAN1 上三台电机须各有一帧新反馈才能组成一组，同一帧只能计一次。三轮的最新帧都须在 30 ms 内到达，各轮均为使能态（state=1）且速度绝对值 ≤0.20 rad/s。连续三组满足条件才报告停车；两组之间最长 30 ms，缺帧超时、任一轮反馈超速或状态异常即撤销确认。

`chassis_speed[0..2]` 是按各轮最近反馈正解的 vx、vy、wz，单位m/s、m/s、rad/s；先按方向和齿比还原几何轮速。三轮反馈并非同时采样，未收反馈时初值为0，应同时看 `dm_motor[].id/state`，此观察量不参与控制PID。

通常切换动作时先 Stop，再按 `Chassis_Is_Stopped()` 的结果起步。该接口只在已使能、零速待命且三轮新反馈持续满足停车条件时返回 1。转向末段直接接定距时，调用方不先调用 Stop，也不把运动中的接续称为停车确认；连续速度模式内的目标更新仍按原斜坡过渡。2026-11-05 落地快测的最后 0.4 s 内，零速度目标下 40 组三轮新反馈的最大绝对轮速为 0.068 rad/s；较早换步时，目标已回零而单轮仍达 5.05 rad/s。0.20 rad/s 门限需在比赛地面及实际负载下复核。

### 单段定距平移

普通待命启动时，车体静止、底盘已使能且姿态就绪后，由调用 `Chassis_Loop` 的同一任务请求 `Chassis_Distance_Start(direction, distance_m, speed_limit_mps)`。方向取现有八向枚举，距离用米，速度参数按车体合速度传入，单位 m/s。以 `Chassis_Distance_Start(CHASSIS_TRANSLATION_FORWARD, 0.50f, 0.50f)` 为例，两个后轮的指令由零加速至约 12.37 rad/s，角加速度幅值约 18.56 rad/s²，再减速至零，全段规划约 1.67 s。轮速换算均假设车体航向修正为零。段末持续下发零速 MIT，保留电机速度阻尼。定时平移到期按上面的反馈条件刹停、失能，调用 `Chassis_Stop()` 可主动中止动作。

按速度上限和对称加减速度计算，短段在中点从加速切换到减速，长段在两者之间插入匀速段。以接口上限运行时，分界距离约为 0.333 m。规划距离按车体速度指令的理想控制周期积分；调度抖动、电机响应和地面打滑都会影响实际行程，实际到点与停稳用时需在相同地面和负载下测量。这里只参考 WPILib 的限速、限加速度规划思路。[WPILib 梯形运动规划](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html)

《2027 年第十届中国大学生工程实践与创新能力大赛 智能+工程创新赛道竞赛命题与运行》智能救援赛项第 15–19 页图 6–8 给出 3 m × 3 m 场地，出发区边长 0.30 m。按图中几何估算，出发区中心到场地中心约 1.91 m，单段直线规划约 4.49 s。场地中心到本方物资区面向场地的围栏外缘中点约 1.15 m，单段规划约 2.97 s。两段均假定可沿当前车体的一个方向直达、静止起停、平地和定距速度上限。实际行程需按抽签的出发位、目标位置和推送深度选取，转向或分段会增加用时。出发区前有三根 10 mm 高的减速带，越带速度须实测。初赛首个普通物资须单独送入本方物资区围栏内侧，禁止抓取救援目标或将其放在车上。

2026-11-08 修正前后方向后，在同一底盘测试了三个前进加减速档位。0.20 m 规划距离下，两后轮角加减速度 14.85、18.56、22.27 rad/s² 分别在 1.20、1.08、1.08 s 确认停车；对应运动中的最大航向变化为 0.90°、1.36°、7.21°。本次将定距参数设为中档 18.56 rad/s²，对应代码中的 `CHASSIS_DISTANCE_ACCELERATION_MPS2=0.75f`。[同距离对比数据](C:/Users/Lenovo/OneDrive/Desktop/C_project/outputs/accel_straight_20261108/comparison.json)

中档随后测试规划前进 0.50 m，1.70 s 目标回零，1.74 s 确认三轮停车，之后收齐三台新的失能反馈。两后轮反馈峰值为 12.43、12.71 rad/s，运动中最大航向变化 1.58°，停车时 1.40°。每轮串口采样均为 50 Hz，时序观察分辨率约 20 ms。编码器位置解绕后估计前进 0.487 m，实际地面距离待尺量。每档与距离组合只测过一次，侧移、斜向及不同地面、负载的重复性仍需分别验证。[0.50 m 实测摘要](C:/Users/Lenovo/OneDrive/Desktop/C_project/outputs/accel_straight_20261108/run5/summary.json)

### 转向加减速与脉动诊断

转向规划保持现有 `Service_Turn`、相邻 yaw 差累加及原 PID 增益。剩余转角绝对值大于 5° 时，先换成弧度，速度目标取 `sign(remaining) * min(1.0, sqrt(2 * 4.0 * abs(remaining_rad)))`。每次用实际 `dt` 将 `turn_speed` 的变化限制在 `4.0 * dt` 内。临近终点按剩余转角减速，进入最后 5° 后由原角度 PID 修偏，仍经过同一斜坡。`Chassis_Stop()` 清零此状态。纯转向对应三轮最大绝对目标轮速均为 3.09 rad/s、角加减速度均为 12.34 rad/s²。下文保留当时上限为 2.0 rad/s 的落地记录。

真实业务源码的理想跟随测试覆盖左右 1/5/6/10/90/180/360°、10～25 ms 变周期、yaw 与计时回绕。180° 模拟至三轮停车确认为 2.340 s，最大过冲 0.983°；360° 为 3.905 s。5° 切换边界未反复退出，10°及以上有一次小幅反向修正。原 2° 到位容差保留，因此 1° 请求不要求实际走满 1°。这些时间仅来自电脑模拟，没有电机、地面与负载动态，与下面的实测记录分开解读。[真实源码回归](C:/Users/Lenovo/OneDrive/Desktop/C_project/tests/native/test_chassis.c)

此前在车体角速度上限 1.0 rad/s 下完成两次落地诊断：前进后直接左转 180°，无转向斜坡时 4.00 s 确认停车、相对 yaw 为 178.71°；独立左转加入 1.6 rad/s² 斜坡后为 4.72 s、178.36°。两次恒目标段按 MIT 字段解码均为车体 1.00348 rad/s，各轮均为 −3.09646 rad/s。无斜坡记录的恒目标内部，车体 IMU 与编码器位置差分正解的相关系数为 0.994，二者均显示快慢脉动；电机速度字段却接近恒定。加斜坡后的记录仍有同类脉动，平台内部 IMU 标准差由 0.623 降至 0.454 rad/s，但动作起点、地面接触与负载不同，且各测一次，不能归因于斜坡单一因素。[原转向数据](C:/Users/Lenovo/OneDrive/Desktop/C_project/outputs/accel_sequence_20261108/summary.json) · [斜坡前后诊断](C:/Users/Lenovo/OneDrive/Desktop/C_project/outputs/turn_ramp_20261108/summary.json)

随后架空绕过角度 PID，使用相同各轮恒目标 −3.09646 rad/s，位置差分仍有明显脉动，用户也肉眼观察到轮子顿挫。ID1/2/3 的轮速差分标准差分别为 0.757/0.991/1.086 rad/s，低于对应落地记录的 1.333/1.358/1.379 rad/s；架空车体陀螺仪绝对峰值只有 0.0207 rad/s。恒定发送目标下仍出现反馈脉动，说明上层角度 PID 改变目标与轮地负载都不是该反馈现象的必要条件。架空差分主频约为 6.75/6.75/13.49 Hz，落地三轮约为 2.60 Hz，频段不同，不能认定同一振荡机制或完全排除负载作用。架空缺少独立轮速基准，尚不能区分电机内环、轮系、编码器周期误差或速度估计。三轮均在测试结束后有新失能反馈，末次 CAN ESR 为零仅代表读取当时状态。[架空诊断数据](C:/Users/Lenovo/OneDrive/Desktop/C_project/outputs/turn_air_20261108/summary.json)

随后在相同架空动作、目标、斜坡及遥测配置下，仅将电机 MIT Kd 从 0.2/0.25/0.2 减半为 0.1/0.125/0.1。完整复测的 ID1/2/3 位置差分标准差为 0.722/0.994/1.109 rad/s，相对原值变化约 −4.6%/+0.3%/+2.1%，未出现三轮一致改善。本次不采用 Kd 减半，正式代码继续保留旧车 0.2/0.25/0.2；这项架空对比不替代新版快转规划的落地验证。[单变量试验配置](C:/Users/Lenovo/OneDrive/Desktop/C_project/outputs/turn_air_kd_20261108/run2/manifest.json) · [完整复测数据](C:/Users/Lenovo/OneDrive/Desktop/C_project/outputs/turn_air_kd_20261108/run2/serial.csv)

最后恢复原电机 Kd，使用车体角速度上限 2.0 rad/s、角加减速度 4.0 rad/s² 的新规划落地左转 180°。从进入转向阶段到三轮新反馈停车确认为 2.22 s，比此前 1.0 rad/s、1.6 rad/s² 档位的单次记录 4.72 s 缩短 2.50 s；确认时 IMU 累计相对转角为 181.77°。约 1.72 s 时累计转过 176.18°，观测到进入最后 5° 修偏区，再约 0.50 s 获得停车确认。随后 FD 释放阻尼，在约 0.2 s 内有轻微反向回落，静置初段约为 181.00°；此段发生在失能后，不是角度 PID 继续修偏。用户反馈“明显顺畅了”。当时在这个档位仅记录了一次左转，其他方向、角度及不同负载的重复性仍待验证。[新规划落地数据](C:/Users/Lenovo/OneDrive/Desktop/C_project/outputs/turn_fast_20261108/summary.json)

新规划恒目标平台约为 0.50～1.48 s，按 MIT 字段解码的车体目标为 2.0038 rad/s，各轮目标均为 −6.18315 rad/s；此时车体 IMU 的 P10/P90 为 2.032/2.271 rad/s，平台峰值 2.370 rad/s，全程峰值 2.477 rad/s。反馈超过指令上限说明实际跟随仍有动态偏差，指令限速不等于实测峰值上限。触发被任务接受后约 80 ms 首次提交非零转向帧，约 238 ms 观测到车体角速度绝对值超过 0.1 rad/s；后者只是可辨运动阈值，不是机械首次运动的精确时刻。结束后观测到三台 state=0，末次 CAN ESR 为零。本轮 INS 首次有效发布在 RTOS 时基 663 ms，未覆盖 RTOS 启动前的复位和初始化，不能当作冷上电总启动时间。

上述 2026-11-08 诊断固件的构建选项均为 `INS_USE_MAGNETOMETER=OFF`，与当前默认六轴一致，未使用九轴磁航向控制转向。临时遥测以 50 Hz 记录轮速目标、反馈位置及车体角速度；目标记录表示 HAL 成功入邮箱，不等同电机已执行。落地的位置差分与独立 IMU 相互印证，不能仅凭接近恒定的电机速度字段认定车体运行平稳。

### 修改前连续往返基线

以下记录是增加“转向末段边走边修偏”接续之前的基线。使用同一组六轴、原电机 Kd 和新转向参数，连续执行“前进 1 m → 左转 180° → 前进 1 m”；当时每段均由三轮新反馈确认停车后保持使能接下一段，最终才失能。用户确认已返回起点附近并停下。以下用时由 50 Hz 遥测的阶段切换得到，约有 20 ms 观察分辨率；平移距离由编码器位置差分及三轮正解估算，没有地面量尺结果。[往返实测摘要](C:/Users/Lenovo/OneDrive/Desktop/C_project/outputs/round_trip_20261108/summary.json) · [试验参数](C:/Users/Lenovo/OneDrive/Desktop/C_project/outputs/round_trip_20261108/run1/manifest.json)

| 阶段 | 至下一阶段用时 | 观察结果 |
|---|---:|---|
| 去程前进 1 m | 2.72 s | 编码器估计前进 0.982 m，三轮反馈确认停车后开始左转 |
| 左转 180° | 3.70 s | 转向段 IMU 累计 181.86°，末段修偏后接返程 |
| 返程前进 1 m | 2.70 s | 编码器估计前进 0.982 m，三轮反馈确认停车 |
| 首段开始至最终停车确认 | 9.12 s | phase6、bench_stopped=1；此时仍使能，之后才发送 FD |

本轮转向没有复现独立单转的 2.22 s：约 1.72 s 进入最后 5°，1.92 s 达到峰值 183.94°，随后以约 −0.13～−0.06 rad/s 的车体角速度目标慢速反向修偏，3.70 s 时累计 181.86°并进入返程。末段占 1.98 s，独立单转对应约 0.50 s；增加的用时来自到位修偏，没有另加换步等待。转向中一直保持 state=1，未发送 FD，因此这次反向回修与上面单转结束后失能释放的回落不同。

去程零目标到转向新目标约 80 ms，转向零目标到返程新目标约 60 ms。最终相对整段起始航向为 182.64°；用户的“返回起点附近”是现场观察，不能据此给出精确位置闭合误差。最终停车确认后约 20 ms 收到三台新的 state=0 反馈，末次 SWD 读取 CAN ESR=0。三段保持原电机 Kd=0.2/0.25/0.2，尚未为末段耗时修改容差、PID 或直线参数。

### 转向末段接定距

按用户要求，现有 `Chassis_Distance_Start()` 允许在 TURN 状态、剩余转角绝对值 ≤5° 时接受下一段定距请求。判断使用已累计转角加上最新 INS 样本的 yaw 差，360° 起点不会因为包角与终点相同而误接续。接受时保存原转向的最终目标航向，不把当时还差几度的朝向作为新目标；平移按原定距轨迹从零起步，航向修正沿原直行 PID。现有 `turn_speed` 保留，按实际 `dt` 和 4.0 rad/s² 逐步追上直行 PID 输出，追上后恢复原直行控制，不新增任务、接口或命令队列。

调用方在转向期间每个任务周期直接尝试 `Chassis_Distance_Start()`，返回 1 后切到下一段业务；不要先等待 `Chassis_Is_Stopped()`，也不要先调用 `Chassis_Stop()`。接续时车体可能仍在转动，初段平移与航向修正同时进行，`Chassis_Is_Stopped()` 继续返回 0。普通待命启动仍须三轮新反馈停车确认，定时平移及连续速度的接入条件不变。没有后续定距请求的单独 TURN 继续执行原 2°、0.08 rad/s、200 ms 到位条件；定距到时或显式 Stop 仍按原逻辑清零并停车，不为余角延长动作。

新的真实源码回归已覆盖左右 180/360°、最新样本进入 5°、跨圈与半圈消歧、过冲时保留原终点、仍在转动时接续、非法请求不改状态、实际控制时基与变周期角速度连续、普通及极短动作终止。理想跟随模拟中，180° 后接 1 m 在 1.87 s 接续、4.58 s 最终确认停车；这是电脑模拟，不作为下面完整往返的实车计时。

随后完成新接续逻辑的同一组“前进 1 m → 左转 180° → 前进 1 m”实测。计时均从去程阶段 phase3 开始，到最终 phase6、bench_stopped=1 的三轮新反馈停车确认结束，本次为 7.12 s，比修改前基线 9.12 s 缩短 2.00 s；直线参数和原电机 Kd 保持不变。[接续实测摘要](C:/Users/Lenovo/OneDrive/Desktop/C_project/outputs/round_trip_blend_20261108/summary.json) · [完整串口记录](C:/Users/Lenovo/OneDrive/Desktop/C_project/outputs/round_trip_blend_20261108/run1/serial.csv)

| 阶段 | 修改前停稳换段 | 本次末段接续 |
|---|---:|---:|
| 去程开始至开始转向 | 2.72 s | 2.72 s |
| 转向开始至开始返程 | 3.70 s | 1.70 s |
| 返程开始至最终停车确认 | 2.70 s | 2.70 s |
| 完整往返 | 9.12 s | 7.12 s |

接续时已转过 175.99°，距离原转向终点约差 4.01°；车体角速度为 1.609 rad/s、目标为 1.162 rad/s，确实在转动中开始返程，不是等待停稳后再起步。返程初段相对原终点的误差曾到 +3.63°，从返程约 0.58 s 起，余下运动样本保持在 ±2° 内；最终误差为 +0.25°。相对整个往返最初朝向的累计角度为 180.59°，两种参考起点不同。编码器估算去程/返程分别为 0.985/0.983 m，仍不是地面量尺距离，也不能据此宣称精确返回原点。

最终停车确认后约 20 ms 收到三台新的 state=0 反馈；本次运行后 SWD 未读取成功，没有新的 CAN ESR 数值可报告。动作期间电压最低约 23.88 V，低于 20 V 的记录在动作结束约 9.5 s 后才出现。新旧接续各完成一次完整往返，本次记录支持节省末段等待，右转及不同地面、负载下的重复性仍需另测。

## 构建与验证

2026-11-08 后续接入 Jetson 与三电机共用 CAN1，复用已有外设配置和 test_task，不修改生成文件。当前仅 `0x180` 驱动连续速度/车体转向；其余六种下行帧解析保存，运输握手上报与机构动作尚未接入。新的有效零速准备、三轮停稳门控、250 ms 命令期限及故障后新零速恢复规则见 [Jetson CAN1 联调](JETSON_CAN_TEST.md)。下面的往返固件状态为此前归档，本轮离线构建仍不代表已烧录。

2026-11-08 保存时，8 组原生测试、正常六轴 Debug 与九轴 Debug-9axis 均通过。用户确认已断开开发板并要求只保存，正常六轴固件尚未恢复烧录；此前恢复命令在 OpenOCD 初始化阶段失败，未开始写入。已保存正常六轴的 BIN、HEX、ELF 及 SHA-256，下次连接后烧录正常固件。板内最后验证的仍是本次接续往返的一次触发测试固件，上电待命、默认失能，不会自动执行往返。[固件与恢复状态](C:/Users/Lenovo/OneDrive/Desktop/C_project/outputs/round_trip_blend_20261108/final_normal_check.json) · [正常固件说明](C:/Users/Lenovo/OneDrive/Desktop/C_project/outputs/round_trip_blend_20261108/normal_firmware/说明.txt)

当前 VOFA 使用 USART6：PG14/PG9、115200/8N1、DMA2 Stream7 Channel5 Normal；DMA与USART6中断优先级分别为6/0、5/0，初始化和中断由 CubeMX 生成。本轮姿态调整保留JustFloat三通道格式及100Hz发送频率。

在仓库根目录使用已配置 Arm GNU、CMake、Ninja 的终端。比赛版默认六轴（`INS_USE_MAGNETOMETER=OFF`）。九轴预设把同一选项设为 `ON`，独立构建，分别烧录对应目录的 `project.elf` 进行对比。CubeMX 配置、RTOS 任务和 VOFA 通道均可共用。

```powershell
cmake --preset Debug -S project
cmake --build project/build/Debug
cmake --preset Debug-9axis -S project
cmake --build project/build/Debug-9axis
python tests/run_native.py
```

工程中的 CLion 运行配置 `F407-DAPLink` 和 `F407-STLINK` 都指向比赛用的 `Debug`。烧录九轴对照版时，请在下载配置中选用 `Debug-9axis` 的构建产物，并核对下载日志中的 ELF 路径为 `project/build/Debug-9axis/project.elf`。

主机测试需要原生GCC，可用 `--cc <路径>` 指定；系统临时目录不可写时加 `--temp-dir tmp`（先创建目录）。主机回归覆盖 CAN 协议、六轴与九轴的 INS 任务、真实 AHRS 算法、VOFA 帧、PID、板载 BSP 和底盘。保留原有温控、静止标定、SPI失效、VOFA及板载回归；底盘新增独立几何向量、轮序/Kd、反馈正解、三段使能、八向等幅、联合斜坡、真实dt和TIM2/RTOS回绕、三轮新反馈停车确认、定时制动后失能、双向360°跨零、转向制动包络与5°原PID切换、小角度及变周期闭环、连续200ms到位、15s超时、姿态20ms期限及异步取消。PID测试核对离散温控不变和连续积分/测量微分；INS测试包括倾斜90°时的世界Z投影与快照就绪。DMA配置与中断由生成文件和固件编译核对，不模拟已删除的独立BSP。

2026-11-05姿态数值调整验证（磁标定前）：七组主机测试全部通过，其中传感器/INS为11个场景。新增覆盖1000点完整与重采窗口、同窗加速度均值复用、重力初始化的水平/倾斜/倒置/±Xg、长时间静止数值误差、±90°附近更新后pitch有限、磁场读取失败及恢复；原有动态积分、变dt、六轴/九轴及VOFA帧检查保留。ARM Debug执行 `cmake --build project/build/Debug --clean-first`，68步全量编译链接完成，无错误或警告；FLASH 64448字节、RAM 31960字节、CCM为0。当前尚未接入指令源，链接器会裁剪未调用的动作入口，后续接入后容量会增加。

固定静置状态下分别采集修改前、候选固件各180秒，含启动等待，收到17766与17339个同步测试帧；有效板端时间跨度分别177.65与173.38秒。测试帧同步记录四元数、dt、传感器与温控数据，不用电脑串口到达间隔推断板端周期。两轮所有相邻采样均为10ms，1MHz TIM2测得dt均为10ms（1μs分辨率），I2C读取均成功，温度范围均为44.625～45.25℃。候选四元数模长范围0.999999922～1.000000137，修改前约0.99831。

以下统一排除每轮有效记录的前60秒再比较；该观察窗口不等于已证明磁航向完全收敛。角度变化取窗口首尾各10秒的均值差，峰峰值使用整个观察窗口。

| 观察量 | 修改前 | 候选固件 |
| --- | --- | --- |
| 去偏后机体系角速度均值X/Y/Z（°/s） | -0.03154 / +0.02026 / +0.00330 | -0.00451 / -0.00279 / +0.00461 |
| roll/pitch/yaw首尾均值变化（°） | -0.0430 / -0.0563 / -0.0961 | -0.0689 / -0.0823 / +0.4176 |
| roll/pitch/yaw峰峰值（°） | 0.1936 / 0.2333 / 0.4824 | 0.1795 / 0.2999 / 0.8309 |

本次修改后X/Y角速度残差较小，Z未改善，yaw波动没有减少；两轮独立采样也不足以将差别全部归因于代码。对同一份修改前数据分别重放原Mahony与精确归一化版本，原版四元数逐点复现；精确版yaw峰峰值约0.4838°，与原版0.4824°接近，说明归一化主要修正数值偏差，不能称为消除了实测航向漂移。无噪声静止向量测试中，原快速近似会使正确初姿态自行偏转，典型倾斜姿态100秒约0.528°，精确归一化消除了这项算法数值误差。11月5日版本采用1000点平均以降低零偏估计波动，尚无充分实测证明长期航向稳定性提升；磁场标定与环境磁干扰仍需另外实测，当前不凭单一静置姿态写入磁场补偿或修改Mahony增益。

测量后已删除临时23-float输出并恢复原三通道协议，最终pitch极点修正已纳入正式构建。通过Horco CMSIS-DAP v2下载到STM32F407，OpenOCD返回Verified OK并复位运行。COM7按115200/8N1连续读取45秒，收到72001字节，其中4500个完整16字节JustFloat帧、末尾1字节未完成下一帧；帧间无字节缺口，三通道均为有限角度，电脑侧观察约100.005Hz。这一轮覆盖开机磁航向收敛，只确认最终协议及数据链路，不作为稳态漂移结论。串口已关闭，可在VOFA继续连接。

本轮最终代码使用本机CubeMX 6.18.1再次生成，前后核对71个文件（Core业务及生成文件、cmake目录、顶层CMake、CMakePresets和.ioc），哈希全部一致，随后增量编译通过。GPIOG AF8、DMA2 Stream7 Channel5及两个中断的HAL转交沿用原配置；时钟、传感器、CAN及RTOS任务配置保持原值。业务入口由强函数覆盖生成的弱任务，重生成无需手动补回调用，发送缓冲位于普通SRAM。

若使用本机CubeMX命令行生成，脚本使用 `config load "工程绝对路径/project.ioc"`、`project generate`、`exit` 即可，不额外重设 `project path`。本机该额外命令曾使CMake生成器写出重复绝对路径；重新加载后直接生成已恢复单份相对路径。

上一轮按授权调整静止标定窗口、初始重力对准、四元数归一化和磁场读取结果传递，保留现有外设配置与业务划分。原有未提交改动保留，旧仓库与 Jetson 项目不修改。软件测试不替代实板轮序、转向正负、CAN交互、磁干扰与温控验收。

## 本板磁力计标定

2026-11-05用户按提示对单独板子做两轮三维翻转。参考包 `basic_framework`、`librm` 未找到磁场参数求解，电脑端明确参考 [PX4 lm_fit.cpp（30e763b）](https://github.com/PX4/PX4-Autopilot/blob/30e763b6780061d70a14894e3e8b06e6a656f9b8/src/modules/commander/lm_fit.cpp) 的先球后椭球模型，SciPy求解为本工程适配。板端保留教程 `imu_cali_slove`，只加入 `mag_offset` 与实测 `mag_scale_factor`，计算 `A*(raw-b)`，未新增任务、标定状态机、Flash存储或保护函数。

参数只由第一轮拟合，第二轮独立验证未参与拟合。第一轮相邻不同读数8850组，第二轮6466组；CSV、系数、来源、复现命令及翻转步骤见 [磁标定说明](../tools/mag_calibration/README.md)。第一轮末尾约16秒因板子复位无串口数据，前16424帧保留；第二轮120秒12000帧连续。训练集模长相对标准差22.82%→8.14%，独立集19.67%→7.14%，支持作为本板当前环境的改善候选。独立集校正均值38.98µT仍偏离拟合目标42.41µT；两轮独立拟合偏移相差2.39µT，不能称精密标定或证明yaw漂移降低。磁场模型不补偿随电流变化的场，装车后须复验。

2026-11-05 实测的固件恢复 roll/pitch/yaw 三通道，当时上电需要预热及约 10 秒静止陀螺标定。原始 `ist8310_real_data.mag` 以芯片坐标、µT 保存。IST8310 的单位安装阵已合入校准阵，改变安装方向时要重新计算组合矩阵。磁标定参数随固件保存，日常启动只需将板子放稳。

本轮正式代码的7组主机回归与5项电脑拟合测试通过，原传感器11场景保留。磁场场景新增两组独立已知补偿向量验证矩阵方向、偏移符号和只补偿一次；非磁场场景模拟磁场读取失败，以独立核对六轴gyro/重力行为。ARM Debug编译无错误或警告，FLASH 64488字节、RAM 31960字节、CCM为0。CubeMX按原.ioc再次生成后，71个配置、生成及业务文件哈希一致，增量构建无需重编。

正式固件已通过CMSIS-DAP下载并Verified OK。COM7以115200/8N1采集180秒（含开机等待），收到17645个完整16字节三角度帧，无帧间字节缺口，均为有限角度，电脑侧观察约100.005Hz；串口已关闭。排除首个有效帧起60秒后的约116.43秒观察窗口，roll/pitch/yaw首尾各10秒均值之差约+0.0501/-0.0047/+0.0097°，峰峰值约0.4988/0.5447/0.9204°。这是用户重新摆放后的本次静止观察，不能与前一轮不同姿态、不同环境数据直接当严格对照，也不能把小首尾差写成0.01°精度或消除漂移。统计见 [attitude_check.json](../tools/mag_calibration/data/2026-11-05/attitude_check.json)。

## CAN1 临时接线适配

三个电机仍为控制 ID 1/2/3、反馈 StdID 0，协议、轮序、方向、量程及 PID 均保持旧值。`bsp_can.h` 的 `MOTOR_CAN=hcan1` 与 `MOTOR_CAN_FILTER_BANK=0U` 同时控制过滤、发送、反馈筛选及底盘邮箱操作。恢复 CAN2 时同时改为 `hcan2` 和 `14U`，不要只改发送句柄。任务目录仍叫 `Task/CAN2`，实际任务入口与文件路径不变。

CAN1 位时序为 PCLK1 42 MHz / (3 × (1+10+3)) = 1 Mbit/s，普通模式，SJW 1 TQ；自动重传和自动 Bus-Off 恢复开启。CubeMX 生成 PD0/PD1、CAN1_RX0_IRQn 优先级 6/0、`CAN1_RX0_IRQHandler` 转交 HAL，无重复 BSP 初始化或中断。CAN2 保留原生成配置，但业务不 Start、不配置其 bank 14、不发电机指令。上电仍默认失能。

实板接线再次核对：官方主板原理图 PDF 第4页（图纸 Sheet 5/8）CAN1 接 MCU 的 PD0/PD1，PB8/PB9 虽可复用 CAN1，却未连接板上收发器。CAN1 收发器 U14 的 STB 接地，无 GPIO 使能；J22/J23 并联，1脚 CAN1_L、2脚 CAN1_H。已纠正工程原有 PB8/PB9 分配。

## 电机帧间等待实测适配

2026-11-05：CAN1 连续提交三轮帧时，实板曾出现 Stuff Error 和部分反馈漏收；旧车正常 MIT 同样连续发三帧，不能把这一现象归因于遗漏旧有固定间隔。三台有效反馈 StdID 均为 0，同时回复竞争是可能原因，尚不排除物理层。

本次在既有 MIT、FC、FD 三处循环中，仅在成功的前两帧后调用 `vTaskDelay(pdMS_TO_TICKS(1U))`。当前 RTOS 为 1000 Hz，这是让出到下一 tick，并非精确固定 1 ms；每组最后一帧不等待。不新增任务、队列、函数或诊断框架，PID、几何和原 10 ms 控制节拍保持不变。发送成功仍只代表进入邮箱，不等于收到电机执行确认。

验证：7 组原生测试通过。加入等待后，12 组仅 FD 的实板检查三台全部回复、ESR=0；随后 0.1 m/s、1.5 s 架空复测每台新增 154 帧，最终均为状态 0，ESR=0。首项落地直行也每台新增 154 帧、最终状态 0、ESR=0，用户确认向前行驶并停下。两次完整串口记录均为 2000 帧，姿态采样间隔 10 ms；证据在 `outputs/can1_motor_test_20261105/`。

本次使用临时单动作测试固件记录 23 通道，不作为正式业务框架。正常固件继续默认失能、VOFA 三个姿态通道。架空复测有明显 yaw 变化而陀螺积分接近零；用户已要求暂不处理偏移，继续低速落地验证，不能将九轴角度当作已验证的绝对转角精度。

后续落地左前平移已由用户确认方向正确并停下；180°请求在约5.42 s目标回零，此时yaw累计181.44°，近水平陀螺积分177.48°。三项落地测试结束均观测到三台新的state=0反馈，CAN ESR=0。转向结束等待期间yaw还会变化，不能把15 s测试窗口末值当作实际停车角度。

用户随后要求连续执行：0.5 m/s目标前进1 s、同目标左移1 s、720°转向且角速度上限1 rad/s。只在临时夹具放开对应上限，保留0.3 m/s²线加速度；两次切换各保留30 ms失能等待与30 ms使能包序，无人工逐项确认。40 s记录含3999个完整帧，采样间隔10 ms；前进/左移实际发送速度峰值分别0.2985/0.3015 m/s，编码器轮速正解峰值0.2835/0.2968 m/s，因此未达到0.5 m/s。720°阶段约12.73 s目标回零；最后非零目标后1 s，yaw累计721.78°、近水平陀螺积分734.44°，二者都不是外部角度真值。IMU角速度仍有周期起伏，未在本轮判断根因或修改算法。

连续测试最终三台接收计数均2213、状态均0，CAN ESR=0。临时五份源码、参数覆盖及测试入口已还原，正常固件编译并烧录校验通过，恢复VOFA三通道与上电默认失能；当时正式业务仍保留原0.4 m/s速度上限、0.6 rad/s转向上限和15 s转向超时。最终CubeMX再次生成核对71个配置/生成/业务文件均未变化。完整曲线与数据位于 `outputs/can1_motor_test_20261105/`。
