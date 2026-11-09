# CLion 下载与调试

在 CLion 打开本仓库的 `project/`，即包含 `CMakeLists.txt` 的目录。沿用旧 Git 项目的 OpenOCD 下载与运行方式，两套配置均下载本工程的 STM32F407IGH6 固件。

| 运行配置 | 探针 / 传输 | OpenOCD 配置 |
|---|---|---|
| `F407-DAPLink` | DAPLink / CMSIS-DAP，SWD | `project/openocd-daplink.cfg` |
| `F407-STLINK` | ST-LINK V2J24 及更新固件，dapdirect_swd | `project/openocd-stlink.cfg` |

两套入口保存在 `project/.run/`，可随 Git 保存。目标和可执行文件均为 `project`，CMake profile 为 `Debug - Debug`，构建产物为 `project/build/Debug/project.elf`。启动前自动构建；下载选“如果已更新”，复位选“初始化”，GDB/Telnet 端口为3333/4444，使用CLion捆绑的multiarch GDB。一次选择一套入口使用。

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

本次已通过：实际OpenOCD使用 `noinit` 解析两份配置（F4目标、480kHz），CMake Debug配置与ARM固件构建，以及CLion中两套运行配置的加载检查。未执行探针连接、复位或烧录，硬件通信留待上板验证。

配置格式参考旧项目的 `.run/H723-DAPLink.run.xml`、`.run/H723-STLINK.run.xml`，仅借鉴IDE入口与探针设置，未读取或迁移旧业务代码，也未修改旧仓库。工具用法见 [JetBrains OpenOCD文档](https://www.jetbrains.com/help/clion/openocd-support.html)。
