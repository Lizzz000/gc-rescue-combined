"""编译实际业务C文件，验证底盘、CAN、手动/自动控制、传感器、姿态算法和VOFA；产物仅放临时目录。"""
from pathlib import Path
import argparse
import shutil
import subprocess
import sys
import tempfile
from check_module_boundaries import check_module_boundaries


def run_python_checks(root):
    """运行不依赖板端硬件的 Python 主机检查；不含需 NumPy/SciPy 的磁标定。"""
    python = sys.executable
    repo = root.parent
    commands = [
        [python, "-m", "unittest", "discover", "-s", str(root / "tests"),
         "-p", "test_yaw_adrc_pretest.py", "-v"],
        [python, str(root / "tests/check_jetson_codec.py"),
         "--jetson-root", str(repo / "Jetson")],
        [python, str(root / "tests/check_gripper_budget.py")],
    ]
    for command in commands:
        print("+", " ".join(command))
        subprocess.run(command, check=True, cwd=str(root))
    print("PASS: host Python checks (ADRC pretest, Jetson codec, gripper budget)")
    print("Other host unittest files can be run with: "
          "python -m unittest discover -s tests -p 'test_*.py'")


def check_control_modes(root, cc):
    """预处理全部四开关组合，确保只有一个底盘控制器可被编入固件。"""
    modes = ("CHASSIS_TUNING_ENABLED", "JETSON_AUTO_ENABLED", "JETSON_AUTO_CHASSIS_TEST", "UART6_MANUAL_ENABLED")
    cases = [None] + [tuple((bits >> i) & 1 for i in range(len(modes))) for bits in range(1 << len(modes))]
    for values in cases:
        command = [cc, "-E", "-P", "-x", "c",
                   "-I" + str(root / "project/Core/Control/Chassis")]
        if values is not None:
            command += [f"-D{name}={value}" for name, value in zip(modes, values)]
        result = subprocess.run(command + ["-"], input='#include "chassis_mode.h"\n',
                                capture_output=True, text=True)
        valid = values is None or sum(values) <= 1
        if valid:
            if result.returncode != 0:
                raise RuntimeError(f"合法控制模式 {values} 预处理失败:\n{result.stderr}")
        else:
            if values[1] and values[2]:
                expected = "Select either full transport or chassis-only test"
            elif values[0] and (values[1] or values[2]):
                expected = "Chassis tuning cannot share control"
            else:
                expected = "USART6 manual cannot share chassis control"
            if result.returncode == 0 or expected not in result.stderr:
                raise RuntimeError(f"冲突控制模式 {values} 未被正确拒绝:\n{result.stderr}")
    print(f"PASS: {len(cases)} control-mode preprocessing cases (default + all combinations)")


def main():
    """寻找主机 GCC 并依次编译、运行独立测试程序。"""
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", help="主机 GCC 路径（不能使用 arm-none-eabi-gcc）")
    parser.add_argument("--temp-dir", type=Path, help="可写临时目录的父目录；默认使用系统临时目录")
    parser.add_argument("--suite", action="append", dest="suites", metavar="NAME",
                        help="只编译并运行指定主机套件，可重复；默认全部套件")
    parser.add_argument("--list-suites", action="store_true", help="列出全部主机套件名称后退出")
    parser.add_argument("--python", action="store_true",
                        help="在 native 套件之后运行不依赖硬件的 Python 主机检查")
    parser.add_argument("--skip-native", action="store_true",
                        help="跳过 C 主机套件（仍可与 --python 一起使用）")
    args = parser.parse_args()
    fallback = Path("C:/Program Files/JetBrains/CLion 2026.2.3.1/bin/mingw/bin/gcc.exe")
    cc = args.cc or shutil.which("gcc") or (str(fallback) if fallback.exists() else None)
    if args.skip_native and not args.python and not args.list_suites:
        parser.error("使用 --skip-native 时需要同时指定 --python")
    if not args.skip_native and not cc and not args.list_suites:
        parser.error("未找到主机 GCC，请通过 --cc 指定")
    if cc and not args.list_suites and not args.skip_native:
        check_control_modes(root, cc)
    suites = {
        "adaptive_tracker": ["tests/native/test_kalman_tracking.c", "project/Core/Algorithm/Vision/adaptive_kalman.c"],
        "adaptive_kalman": ["tests/native/test_adaptive_kalman.c",
                            "project/Core/Algorithm/Vision/adaptive_kalman.c",
                            "tests/native/fixtures/image_tracker.c"],
        "jetson_can": ["tests/native/test_jetson_link.c", "project/Core/Communication/Jetson/jetson_link.c"],
        "manual_can": ["tests/native/test_manual_link.c", "manual_test/firmware/manual_link.c"],
        "manual_control": ["tests/native/test_manual_control.c", "manual_test/firmware/manual_usb.c",
                           "manual_test/firmware/manual_link.c",
                           "manual_test/firmware/manual_control.c", "project/Core/Control/Chassis/chassis.c",
                           "project/Core/Algorithm/Chassis/motor_chassis.c", "project/Core/Algorithm/PID/pid.c"],
        "uart6_manual": ["tests/native/test_uart6_manual.c", "manual_test/firmware/uart6_manual.c",
                         "manual_test/firmware/manual_control.c", "manual_test/firmware/manual_link.c",
                         "project/Core/Communication/Common/link_frame.c",
                         "project/Core/Algorithm/Chassis/point_plan.c"],
        "can": ["tests/native/test_can.c", "project/Core/Bsp/CAN/bsp_can.c", "project/Core/Devices/DM_Motor/dm_motor.c",
                "project/Core/Task/Chassis/chassis_task.c"],
        "sensors": ["tests/native/test_sensors.c", "project/Core/Devices/BMI088/BMI088driver.c",
                    "project/Core/Devices/BMI088/BMI088Middleware.c", "project/Core/Devices/IST8310/ist8310driver.c",
                    "project/Core/Devices/IST8310/ist8310driver_middleware.c", "project/Core/Task/INS/INS_task.c",
                    "project/Core/Algorithm/AHRS/AHRS.c", "project/Core/Algorithm/AHRS/MahonyAHRS.c",
                    "project/Core/Algorithm/PID/pid.c", "project/Core/Bsp/IMU_PWM/bsp_imu_pwm.c"],
        "ahrs": ["tests/native/test_ahrs.c", "project/Core/Algorithm/AHRS/AHRS.c",
                 "project/Core/Algorithm/AHRS/MahonyAHRS.c"],
        "vofa": ["tests/native/test_vofa.c", "project/Core/Communication/VOFA/vofa.c"],
        "pid": ["tests/native/test_pid.c", "project/Core/Algorithm/PID/pid.c"],
        "board": ["tests/native/test_board.c", "project/Core/Bsp/ADC/bsp_adc.c",
                  "project/Core/Bsp/LED/bsp_led.c", "project/Core/Bsp/Buzzer/bsp_buzzer.c"],
        "chassis": ["tests/native/test_chassis.c", "project/Core/Algorithm/Chassis/motor_chassis.c",
                    "project/Core/Control/Chassis/chassis.c", "project/Core/Algorithm/PID/pid.c"],
        "chassis_odometry": ["tests/native/test_chassis_odometry.c",
                             "project/Core/Control/Chassis/chassis_odometry.c",
                             "project/Core/Algorithm/Chassis/motor_chassis.c"],
        "live_tuning": ["tests/native/test_live_tuning.c", "manual_test/firmware/live_tuning.c", "project/Core/Algorithm/Vision/adaptive_kalman.c", "project/Core/Algorithm/Vision/visual_servo.c"],
        "can_diag": ["tests/native/test_can_diag.c", "project/Core/Bsp/CAN/bsp_can.c"],
    }
    suites["sensors"].append("project/Core/Algorithm/Chassis/heading_yaw_guard.c")
    suites["sensors_nine_axis"] = suites["sensors"]
    suites["chassis_tuning"] = list(suites["chassis"])
    suites["vofa_tuning"] = list(suites["vofa"])
    suites["spin_observer"] = ["tests/native/test_spin_observer.c", "manual_test/firmware/spin_observer.c", "project/Core/Algorithm/Chassis/motor_chassis.c"]
    suites["jetson_auto_configured"] = [
        "tests/native/test_jetson_auto.c", "project/Core/Communication/Jetson/jetson_link.c",
        "project/Core/Control/Jetson/jetson_auto.c", "project/Core/Algorithm/Vision/adaptive_kalman.c",
    ]
    suites["jetson_auto_unconfigured"] = suites["jetson_auto_configured"]
    suites["jetson_auto_single_delivery"] = suites["jetson_auto_configured"]
    suites["jetson_auto_two_deliveries"] = suites["jetson_auto_configured"]
    suites["jetson_auto_five_deliveries"] = suites["jetson_auto_configured"]
    suites["jetson_auto_safety_unconfigured"] = suites["jetson_auto_configured"]
    suites["jetson_auto_short_distance"] = suites["jetson_auto_configured"]
    suites["jetson_chassis_flow"] = [
        "tests/native/test_jetson_chassis_flow.c", "project/Core/Communication/Jetson/jetson_link.c",
        "project/Core/Control/Jetson/jetson_auto.c", "project/Core/Algorithm/Vision/adaptive_kalman.c",
    ]
    suites["jetson_chassis_bad_window"] = suites["jetson_chassis_flow"]
    suites["jetson_chassis_integration"] = [
        "tests/native/test_jetson_chassis_integration.c", "project/Core/Communication/Jetson/jetson_link.c",
        "project/Core/Control/Jetson/jetson_auto.c", "project/Core/Algorithm/Vision/adaptive_kalman.c",
        "project/Core/Control/Chassis/chassis.c", "project/Core/Algorithm/Chassis/motor_chassis.c",
        "project/Core/Algorithm/PID/pid.c",
    ]
    suites["can_auto"] = suites["can"]
    suites["can_tuning"] = suites["can"]
    suites["link_frame"] = [
        "tests/native/test_link_frame.c",
        "project/Core/Communication/Common/link_frame.c",
    ]
    suites["usb_rx_buffer"] = ["tests/native/test_usb_rx_buffer.c"]
    suites["rtos_startup"] = ["tests/native/test_rtos_startup.c"]
    suites["rtos_startup_uart6"] = ["tests/native/test_rtos_startup.c"]
    suites["usb_link_state"] = [
        "tests/native/test_usb_link_state.c", "project/Core/Communication/Jetson/jetson_link.c",
        "project/Core/Communication/Jetson/jetson_usb.c",
        "project/Core/Communication/Common/link_frame.c",
    ]
    suites["visual_servo"] = ["tests/native/test_visual_servo.c", "project/Core/Algorithm/Vision/visual_servo.c"]
    suites["tracked_visual_servo"] = ["tests/native/test_tracked_visual_servo.c",
                                      "project/Core/Control/Vision/tracked_visual_servo.c",
                                      "project/Core/Algorithm/Vision/adaptive_kalman.c",
                                      "project/Core/Algorithm/Vision/visual_servo.c"]
    suites["navigation_speed"] = ["tests/native/test_navigation_speed.c",
                                  "project/Core/Algorithm/Chassis/navigation_speed.c"]
    suites["point_plan"] = ["tests/native/test_point_plan.c", "project/Core/Algorithm/Chassis/point_plan.c",
                            "project/Core/Algorithm/Chassis/velocity_profile.c"]
    suites["heading_eskf"] = ["tests/native/test_heading_eskf.c", "project/Core/Algorithm/Chassis/heading_eskf.c", "project/Core/Control/Chassis/chassis_odometry.c", "project/Core/Algorithm/Chassis/motor_chassis.c"]
    suites["heading_eskf"].append("project/Core/Algorithm/Chassis/heading_yaw_guard.c")
    for name in suites:
        sources = list(suites[name])
        suites[name] = sources
        if "project/Core/Control/Jetson/jetson_auto.c" in sources and "project/Core/Algorithm/Chassis/point_plan.c" not in sources:
            sources.append("project/Core/Algorithm/Chassis/point_plan.c")
        if "manual_test/firmware/live_tuning.c" in sources:
            sources.append("project/Core/Algorithm/Chassis/point_plan.c")
            sources += ["manual_test/firmware/spin_observer.c", "project/Core/Algorithm/Chassis/motor_chassis.c"]
        if ("project/Core/Control/Jetson/jetson_auto.c" in sources or
                "manual_test/firmware/live_tuning.c" in sources):
            sources.append("project/Core/Control/Vision/tracked_visual_servo.c")
            if "project/Core/Algorithm/Vision/visual_servo.c" not in sources:
                sources.append("project/Core/Algorithm/Vision/visual_servo.c")
        if "project/Core/Control/Chassis/chassis.c" in sources:
            sources.append("project/Core/Control/Chassis/chassis_config.c")
            sources.append("project/Core/Control/Chassis/chassis_odometry.c")
            sources.append("project/Core/Algorithm/Chassis/velocity_profile.c")
        if "project/Core/Algorithm/Chassis/velocity_profile.c" in sources:
            sources.append("project/Core/Algorithm/Chassis/navigation_speed.c")
        if "project/Core/Control/Chassis/chassis_odometry.c" in sources:
            sources.extend(["project/Core/Algorithm/Chassis/wheel_alignment.c",
                            "project/Core/Algorithm/Chassis/planar_odometry.c"])
        if "project/Core/Control/Chassis/chassis_odometry.c" in sources and "project/Core/Algorithm/Chassis/heading_eskf.c" not in sources:
            sources.append("project/Core/Algorithm/Chassis/heading_eskf.c")
    if args.list_suites:
        print("\n".join(suites))
        return
    if args.suites:
        missing = [name for name in args.suites if name not in suites]
        if missing:
            known = ", ".join(suites)
            raise RuntimeError(f"未知主机套件 {missing}；可选：{known}")
        suites = {name: suites[name] for name in args.suites}
    if args.skip_native:
        if args.python:
            run_python_checks(root)
        return

    includes = ["tests/native/stubs", "tests/native/fixtures", "project/Core/Bsp/CAN",
                "project/Core/Devices/BMI088", "project/Core/Devices/IST8310",
                "project/Core/Devices/DM_Motor", "project/Core/Task/INS", "project/Core/Task/Chassis",
                "project/Core/Algorithm/AHRS", "project/Core/Communication/VOFA", "project/Core/Algorithm/PID",
                "project/Core/Bsp/IMU_PWM", "project/Core/Bsp/ADC", "project/Core/Bsp/LED", "project/Core/Bsp/Buzzer",
                "project/Core/Algorithm/Chassis", "project/Core/Control/Chassis",
                "project/Core/Algorithm/Vision", "project/Core/Control/Vision",
                "project/Core/Communication/Common", "project/Core/Communication/Jetson",
                "project/Core/Control/Jetson", "manual_test/firmware"]
    with tempfile.TemporaryDirectory(prefix="cboard_native_", dir=args.temp_dir) as build:
        for name, sources in suites.items():
            exe = str(Path(build) / (name + ".exe"))
            command = [cc, "-std=c11", "-Wall", "-Wextra", "-Werror", "-O0"]
            if name == "sensors_nine_axis":
                command.append("-DINS_USE_MAGNETOMETER=1")
            if name == "rtos_startup_uart6":
                command.append("-DUART6_MANUAL_ENABLED=1")
            if name == "can":
                command.append("-DJETSON_AUTO_ENABLED=0")
            if name == "can_auto":
                command.append("-DJETSON_AUTO_ENABLED=1")
            if name in ("can_diag", "can_tuning", "live_tuning", "chassis_tuning", "vofa_tuning"):
                command.append("-DCHASSIS_TUNING_ENABLED=1")
            if name.startswith("jetson_auto_"):
                command += ["-include", str(root / "tests/native/jetson_auto_test_config.h")]
            if name in ("jetson_auto_configured", "jetson_auto_single_delivery", "jetson_auto_two_deliveries",
                        "jetson_auto_five_deliveries"):
                command.append("-DTEST_JETSON_AUTO_CONFIGURED=1")
            if name == "jetson_auto_single_delivery":
                command.append("-DJETSON_AUTO_DELIVERY_LIMIT=1")
            if name == "jetson_auto_two_deliveries":
                command.append("-DJETSON_AUTO_DELIVERY_LIMIT=2")
            if name == "jetson_auto_five_deliveries":
                command.append("-DJETSON_AUTO_DELIVERY_LIMIT=5")
            if name == "jetson_auto_safety_unconfigured":
                command.append("-DTEST_JETSON_AUTO_SAFETY_UNCONFIGURED=1")
            if name == "jetson_auto_short_distance":
                command += ["-DTEST_JETSON_AUTO_SAFETY_UNCONFIGURED=1", "-DTEST_JETSON_AUTO_SHORT_DISTANCE=1"]
            if name.startswith("jetson_chassis_"):
                command += ["-DJETSON_AUTO_ENABLED=0", "-DJETSON_AUTO_CHASSIS_TEST=1",
                            "-DTEST_JETSON_AUTO_CONFIGURED=1", "-include",
                            str(root / "tests/native/jetson_chassis_flow_test_config.h")]
            if name == "jetson_chassis_bad_window":
                command.append("-DTEST_JETSON_CHASSIS_BAD_WINDOW=1")
            command += ["-I" + str(root / path) for path in includes]
            command += [str(root / source) for source in sources] + ["-lm", "-o", exe]
            subprocess.run(command, check=True)
            subprocess.run([exe], check=True)
    print(f"PASS: {len(suites)} native test suites")
    check_module_boundaries(cc, args.temp_dir)
    if args.python:
        run_python_checks(root)


if __name__ == "__main__":
    main()
