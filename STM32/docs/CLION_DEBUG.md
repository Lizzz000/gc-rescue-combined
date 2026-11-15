# CLion 下载与调试

在 CLion 打开本仓库的 `project/`，即包含 `CMakeLists.txt` 的目录。沿用旧 Git 项目的 OpenOCD 下载与运行方式，两套配置均下载本工程的 STM32F407IGH6 固件。

| 运行配置 | 探针 / 传输 | OpenOCD 配置 |
|---|---|---|
| `F407-DAPLink` | DAPLink / CMSIS-DAP，SWD | `project/openocd-daplink.cfg` |
| `F407-STLINK` | ST-LINK V2J24 及更新固件，dapdirect_swd | `project/openocd-stlink.cfg` |

两套入口保存在 `project/.run/`，可随 Git 保存。目标和可执行文件均为 `project`，CMake profile 为 `Debug - Debug`，构建产物为 `project/build/Debug/project.elf`。启动前自动构建；下载选“如果已更新”，复位选“初始化”，GDB/Telnet 端口为3333/4444，使用CLion捆绑的multiarch GDB。一次选择一套入口使用。

## 默认固件与文件

沿用 `C_project/project` 的默认设置：`Debug - Debug`、六轴姿态（`INS_USE_MAGNETOMETER=OFF`）、手动模式（`JETSON_AUTO_ENABLED=0`、`JETSON_AUTO_CHASSIS_TEST=0`）。在顶部运行配置中选择 `F407-DAPLink`，与原项目的默认入口一致；使用 ST-LINK 时选择 `F407-STLINK`。普通 `project` 运行入口不负责烧录。

每次链接后自动生成以下文件，均来自同一份最新 ELF：

| 文件 | 用途 |
|---|---|
| `project/build/Debug/project.elf` | CLion/OpenOCD 下载及断点调试 |
| `project/build/Debug/project.hex` | 支持 Intel HEX 的烧录工具，文件已携带地址 |
| `project/build/Debug/project.bin` | 原始二进制；烧录起始地址必须设为 `0x08000000` |

不要将 `build/verify-auto` 等测试目录的固件作为默认下载文件。自动模式仍需完成实际标定；本次未修改运动参数。

## 本机工具

- CLion工具链：`STM32`。`project/CMakePresets.json` 的 JetBrains vendor 字段已与旧项目一样指定该工具链。
- C编译器：`C:/Program Files/Arm/GNU Toolchain mingw-w64-x86_64-arm-none-eabi/bin/arm-none-eabi-gcc.exe`。
- C++编译器：同目录下的 `arm-none-eabi-g++.exe`。
- CMake：CLion内置；构建工具为其内置Ninja。
- OpenOCD：`D:/DevEnv/openocd-v0.12.0-i686-w64-mingw32/bin/openocd.exe`，已安装版本0.12.0，沿用CLion已有的全局路径设置。

工具链位置见“设置 → 构建、执行、部署 → 工具链”；OpenOCD路径见同组的“嵌入式开发”。无需额外安装工具或修改业务代码。

## 使用

1. 连接对应探针与C板SWD接口。SWDIO、SWCLK、GND对应连接；电压参考按探针接口定义接C板3.3V，板卡使用正常供电。
2. 顶部运行配置选择 `F407-DAPLink` 或 `F407-STLINK`。
3. “运行”用于构建、下载并启动；“调试”用于下载并进入GDB断点调试。
4. 需要观察连续姿态或VOFA波形时，恢复运行；断点暂停会停止采样任务，恢复后的首次dt不能代表正常连续运行的测量条件。

F407使用 `target/stm32f4x.cfg`，下载工作区由该脚本设在普通SRAM `0x20000000`，不沿用旧H723的AXI SRAM地址。两套SWD均以480kHz作为初次上板速度；同时覆盖reset-start/reset-init中的默认提速事件，使复位前后均使用480kHz。下载初始化保持复位后的HSI时钟，固件正常执行后由 `SystemClock_Config` 配置168MHz。

普通配置使用软件系统复位，不要求连接NRST。若以后需要连接复位下的芯片恢复，再根据实际NRST接线单独配置；本次只提供正常下载与调试的两套入口。

## 重新生成与验证

`.run/*.run.xml` 和两份 `openocd-*.cfg` 独立于CubeMX生成目录；生成代码不需要重新填写这两套下载配置。如果重新生成或更换电脑后CMake工具链选择变化，在CLion的 `Debug - Debug` profile确认工具链仍为 `STM32`；本机该工具链也已是默认工具链。

验证方式：实际 OpenOCD 使用 `noinit` 解析两份配置，CMake Debug 配置与 ARM 固件构建，核对 HEX/BIN 地址、向量表和 ELF 内容。`noinit` 只检查配置，不连接探针；实际下载和运行需在接好板卡后验证。

本工程的 F407 下载配置与原 `C_project/project` 保持一致。更早 H723 项目的入口仅供 IDE 格式与探针设置参考，不能迁移其芯片和 SRAM 地址。本次未修改原项目。工具用法见 [JetBrains OpenOCD文档](https://www.jetbrains.com/help/clion/openocd-support.html)。
