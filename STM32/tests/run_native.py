"""编译实际业务C文件，验证底盘、CAN、手动/自动控制、传感器、姿态算法和VOFA；产物仅放临时目录。"""
from pathlib import Path
import argparse
import shutil
import subprocess
import tempfile


def main():
    """寻找主机 GCC 并依次编译、运行独立测试程序。"""
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", help="主机 GCC 路径（不能使用 arm-none-eabi-gcc）")
    parser.add_argument("--temp-dir", type=Path, help="可写临时目录的父目录；默认使用系统临时目录")
    args = parser.parse_args()
    fallback = Path("C:/Program Files/JetBrains/CLion 2026.2.3.1/bin/mingw/bin/gcc.exe")
    cc = args.cc or shutil.which("gcc") or (str(fallback) if fallback.exists() else None)
    if not cc:
        parser.error("未找到主机 GCC，请通过 --cc 指定")
    suites = {
        "image_tracker": ["tests/native/test_image_tracker.c", "project/Core/Control/Jetson/image_tracker.c"],
        "jetson_can": ["tests/native/test_jetson_can.c", "project/Core/Communication/Jetson/jetson_can.c"],
        "manual_can": ["tests/native/test_manual_can.c", "project/Core/Communication/Manual/manual_can.c",
                       "project/Core/Communication/Jetson/jetson_can.c"],
        "manual_control": ["tests/native/test_manual_control.c", "project/Core/Communication/Jetson/jetson_can.c",
                           "project/Core/Communication/Manual/manual_can.c",
                           "project/Core/Control/Manual/manual_control.c", "project/Core/Control/Chassis/chassis.c",
                           "project/Core/Algorithm/Chassis/motor_chassis.c", "project/Core/Algorithm/PID/pid.c"],
        "can": ["tests/native/test_can.c", "project/Core/Bsp/CAN/bsp_can.c", "project/Core/Devices/DM_Motor/dm_motor.c",
                "project/Core/Task/CAN2/test_task.c"],
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
    }
    suites["sensors_nine_axis"] = suites["sensors"]
    suites["jetson_auto_configured"] = [
        "tests/native/test_jetson_auto.c", "project/Core/Communication/Jetson/jetson_can.c",
        "project/Core/Control/Jetson/jetson_auto.c", "project/Core/Control/Jetson/image_tracker.c",
    ]
    suites["jetson_auto_unconfigured"] = suites["jetson_auto_configured"]
    suites["jetson_chassis_flow"] = [
        "tests/native/test_jetson_chassis_flow.c", "project/Core/Communication/Jetson/jetson_can.c",
        "project/Core/Control/Jetson/jetson_auto.c", "project/Core/Control/Jetson/image_tracker.c",
    ]
    suites["jetson_chassis_bad_window"] = suites["jetson_chassis_flow"]
    suites["can_auto"] = suites["can"]
    suites["link_frame"] = [
        "tests/native/test_link_frame.c",
        "project/Core/Communication/Jetson/link_frame.c",
    ]
    includes = ["tests/native/stubs", "project/Core/Bsp/CAN",
                "project/Core/Devices/BMI088", "project/Core/Devices/IST8310",
                "project/Core/Devices/DM_Motor", "project/Core/Task/INS", "project/Core/Task/CAN2",
                "project/Core/Algorithm/AHRS", "project/Core/Communication/VOFA", "project/Core/Algorithm/PID",
                "project/Core/Bsp/IMU_PWM", "project/Core/Bsp/ADC", "project/Core/Bsp/LED", "project/Core/Bsp/Buzzer",
                "project/Core/Algorithm/Chassis", "project/Core/Control/Chassis",
                "project/Core/Communication/Jetson", "project/Core/Communication/Manual",
                "project/Core/Control/Jetson", "project/Core/Control/Manual"]
    with tempfile.TemporaryDirectory(prefix="cboard_native_", dir=args.temp_dir) as build:
        for name, sources in suites.items():
            exe = str(Path(build) / (name + ".exe"))
            command = [cc, "-std=c11", "-Wall", "-Wextra", "-Werror", "-O0"]
            if name == "sensors_nine_axis":
                command.append("-DINS_USE_MAGNETOMETER=1")
            if name == "can":
                command.append("-DJETSON_AUTO_ENABLED=0")
            if name == "can_auto":
                command.append("-DJETSON_AUTO_ENABLED=1")
            if name.startswith("jetson_auto_"):
                command += ["-include", str(root / "tests/native/jetson_auto_test_config.h")]
            if name == "jetson_auto_configured":
                command.append("-DTEST_JETSON_AUTO_CONFIGURED=1")
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


if __name__ == "__main__":
    main()
