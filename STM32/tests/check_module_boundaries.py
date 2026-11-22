"""在只含声明依赖的临时目录中编译、链接纯算法及两种真实控制模式。"""
import argparse
from pathlib import Path
import shutil
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]
CORE = ROOT / "project/Core"
NATIVE = ROOT / "tests/native"
ALGORITHM = CORE / "Algorithm"
FALLBACK_CC = Path("C:/Program Files/JetBrains/CLion 2026.2.3.1/bin/mingw/bin/gcc.exe")


def build(cc, folder, units, headers=(), test=None, flags=()):
    """复制所选源码及其公共头，编译器无法从源码原目录隐式找到别的模块。"""
    folder.mkdir()
    inputs = [unit.with_suffix(suffix) for unit in units for suffix in (".c", ".h")]
    inputs.extend(headers)
    sources = [unit.with_suffix(".c").name for unit in units]
    if test:
        inputs.append(test)
        sources.append(test.name)
    else:
        # 直接链接全部 .o，独立算法中未调用的入口也不能遗留外部业务符号。
        (folder / "probe.c").write_text("int main(void) { return 0; }\n", encoding="ascii")
        sources.append("probe.c")
    for path in inputs:
        shutil.copyfile(path, folder / path.name)
    command = [cc, "-std=c11", "-Wall", "-Wextra", "-Werror", "-O0", "-I.", *flags,
               *sources, "-lm", "-o", "check.exe"]
    subprocess.run(command, cwd=folder, check=True)
    subprocess.run([str(folder / "check.exe")], cwd=folder, check=True)


def check_module_boundaries(cc, temp_dir=None):
    with tempfile.TemporaryDirectory(prefix="stm32_boundaries_", dir=temp_dir) as tmp:
        work = Path(tmp).resolve()
        # 仅给组合模块提供直接依赖，其余算法继续单独链接。
        ahrs = ALGORITHM / "AHRS/AHRS"
        mahony = ALGORITHM / "AHRS/MahonyAHRS"
        velocity_profile = ALGORITHM / "Chassis/velocity_profile"
        navigation_speed = ALGORITHM / "Chassis/navigation_speed"
        build(cc, work / "ahrs", [ahrs, mahony])
        build(cc, work / "velocity_profile", [velocity_profile, navigation_speed])
        leaves = [p.with_suffix("") for p in sorted(ALGORITHM.rglob("*.c"))
                  if p.with_suffix("") not in (ahrs, mahony, velocity_profile)]
        for unit in leaves:
            build(cc, work / unit.name, [unit])
        print(f"PASS: {2 + len(leaves)} isolated algorithm modules", flush=True)

        # 两侧各自使用真实控制器及源文件，只提供一组公共底盘/链路值接口与硬件替身。
        common_headers = [CORE / name for name in ("Control/Chassis/chassis.h",
                          "Control/Chassis/chassis_config.h", "Control/Chassis/chassis_pose.h",
                          "Communication/Common/link_payload.h")]
        platform = ["-I" + str(NATIVE / "stubs")]
        manual_firmware = ROOT / "manual_test/firmware"
        manual = [manual_firmware / name for name in ("uart6_manual", "manual_control", "manual_link")]
        manual += [CORE / "Communication/Common/link_frame", ALGORITHM / "Chassis/point_plan"]
        build(cc, work / "manual", manual, common_headers,
              NATIVE / "test_uart6_manual.c", platform)
        auto = ["Control/Jetson/jetson_auto", "Control/Vision/tracked_visual_servo",
                "Communication/Jetson/jetson_link", "Algorithm/Vision/adaptive_kalman",
                "Algorithm/Vision/visual_servo", "Algorithm/Chassis/point_plan"]
        auto_headers = ["Control/Jetson/jetson_auto_config.h", "Control/Chassis/chassis_mode.h",
                        "Control/Vision/visual_tracking_config.h", "Communication/Common/usb_link.h",
                        "Communication/Common/visual_observation.h", "Task/INS/INS_task.h",
                        "Devices/BMI088/BMI088driver.h", "Devices/IST8310/ist8310driver.h"]
        auto_sources = [CORE / name for name in auto]
        auto_interfaces = common_headers + [CORE / name for name in auto_headers]
        for name, config, mode_flags in (
                ("jetson_auto", "jetson_auto_test_config.h", []),
                ("jetson_chassis_flow", "jetson_chassis_flow_test_config.h",
                 ["-DJETSON_AUTO_ENABLED=0", "-DJETSON_AUTO_CHASSIS_TEST=1"])):
            flags = platform + mode_flags + ["-DTEST_JETSON_AUTO_CONFIGURED=1", "-include", config]
            build(cc, work / name, auto_sources, auto_interfaces + [NATIVE / config],
                  NATIVE / f"test_{name}.c", flags)
        print("PASS: manual and automatic chassis controllers each link and run independently", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", help="主机 GCC 路径")
    parser.add_argument("--temp-dir", type=Path, help="临时构建目录的父目录")
    args = parser.parse_args()
    compiler = args.cc or shutil.which("gcc") or (str(FALLBACK_CC) if FALLBACK_CC.is_file() else None)
    if not compiler:
        parser.error("请通过 --cc 指定主机 GCC")
    check_module_boundaries(compiler, args.temp_dir)
