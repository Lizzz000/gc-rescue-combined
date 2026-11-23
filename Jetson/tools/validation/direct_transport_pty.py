# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
"""用真实 ROS 节点和私有 USB/S288 PTY 验证两趟无 ROI 投送。"""
from __future__ import annotations

from contextlib import ExitStack
from pathlib import Path
import argparse
import json
import os
import struct
import subprocess
import sys
import time


def stop(process):
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--install", type=Path, required=True, help="已构建并 source 的 colcon install 目录")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    domain = os.environ.get("ROS_DOMAIN_ID", "")
    if not domain.isdecimal() or not 1 <= int(domain) <= 232:
        parser.error("须用 ROS_DOMAIN_ID 指定一个独立的测试域（1–232）")
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    os.environ.pop("ROS_STATIC_PEERS", None)

    from ament_index_python.packages import get_package_prefix

    install = args.install.resolve(strict=True)
    interface_prefix = Path(get_package_prefix("interface_pkg")).resolve()
    if not interface_prefix.is_relative_to(install):
        raise RuntimeError(f"interface_pkg 来源 {interface_prefix} 与指定的安装目录 {install} 不符")

    from geometry_msgs.msg import Point32
    from interface_pkg.msg import FieldObservation, MissionStatus, SafetyTarget, TaskState
    from std_srvs.srv import SetBool
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

    tools = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(tools))
    from source.usb_codec import COMMANDS, IDS
    from validation.observations import UsbProbe
    from validation.s288_probe import S288Probe
    from validation.workflow import configuration

    runtime = configuration()
    from rescue_validation_runtime.core import ROLES
    from rescue_validation_runtime.ros.launch import dump_ros_parameters

    roles = ("mission", "safety", "track", "select", "target", "bridge")
    binaries = {}
    for role in roles:
        package, executable, _ = ROLES[role]
        prefix = Path(get_package_prefix(package)).resolve()
        if not prefix.is_relative_to(install):
            raise RuntimeError(f"{package} 来源 {prefix} 与指定的安装目录 {install} 不符")
        binaries[role] = prefix / "lib" / package / executable

    source = tools.parent / "src"
    profile = source / "bringup_pkg/config/field_20261122.yaml"
    snapshot = runtime.load_configuration(profile, package_resolver=lambda name: source / name)
    args.out.mkdir(parents=True, exist_ok=False)
    params_path = args.out / "pty-parameters.yaml"
    nodes = snapshot["nodes"]
    bridge = nodes["bridge_node"]
    mission = nodes["mission_node"]

    with ExitStack() as cleanup:
        usb = UsbProbe(args.out)
        cleanup.callback(usb.close)
        gripper = S288Probe(args.out, bridge["gripper_id"])
        cleanup.callback(gripper.close)
        if not all(device.startswith("/dev/pts/") for device in (usb.device, gripper.device)):
            raise RuntimeError("USB 和 S288 必须使用本次创建的伪终端")
        bridge["serial_device"], bridge["gripper_device"] = usb.device, gripper.device
        params_path.write_text(dump_ros_parameters(snapshot["nodes"]), encoding="utf-8")
        rclpy.init()
        cleanup.callback(rclpy.try_shutdown)
        node = rclpy.create_node("direct_pickup_probe")
        cleanup.callback(node.destroy_node)
        state_qos = QoSProfile(depth=8, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        image_qos = QoSProfile(depth=4, reliability=ReliabilityPolicy.BEST_EFFORT)
        tasks = node.create_publisher(TaskState, "/detect_node/active_task", state_qos)
        fields = node.create_publisher(FieldObservation, "/detect_node/field/results", image_qos)
        detections = node.create_publisher(Detection2DArray, "/detect_node/seg/detections", image_qos)
        # 不发布 ROI 原始检测，也不给正式阶段留下任何 ROI 模拟服务。
        statuses, safety, switches = [], [], []
        node.create_subscription(MissionStatus, "/mission_node/status", statuses.append, state_qos)
        node.create_subscription(SafetyTarget, "/safety_node/target", safety.append, image_qos)
        active = "seg"
        epoch = 0
        field_visible = True
        cargo = None

        def mode(task):
            nonlocal active, epoch
            epoch += 1
            active = task
            m = TaskState()
            m.task, m.epoch, m.header.stamp = task, epoch, node.get_clock().now().to_msg()
            tasks.publish(m)

        def switch(request, reply):
            new = "field" if request.data else "seg"
            switches.append(new)
            mode(new)
            reply.success = True
            return reply

        node.create_service(SetBool, "/detect_node/set_field", switch)

        def rectangle(x0, x1):
            return [Point32(x=float(x), y=float(y)) for x, y in
                    ((x0, 300), (x1, 300), (x1, 800), (x0, 800))]

        def pulse():
            stamp = node.get_clock().now().to_msg()
            f = FieldObservation()
            f.header.stamp, f.header.frame_id = stamp, "fixture_camera"
            f.valid = f.committed = field_visible
            f.bev_width, f.bev_height = 1000, 500
            f.corners, f.left_region, f.right_region = rectangle(400, 1400), rectangle(400, 900), rectangle(900, 1400)
            f.split_line = [f.left_region[1], f.left_region[2]]
            fields.publish(f)
            a = Detection2DArray()
            a.header.stamp, a.header.frame_id = stamp, "fixture_camera"
            if active == "seg" and cargo is not None:
                d = Detection2D()
                d.id, d.header = "fixture_pickup", a.header
                d.bbox.center.position.x, d.bbox.center.position.y = 1100.0, 938.0
                d.bbox.size_x = d.bbox.size_y = 24.0
                h = ObjectHypothesisWithPose()
                h.hypothesis.class_id, h.hypothesis.score = cargo, 1.0
                d.results = [h]
                a.detections = [d]
            detections.publish(a)

        def wait(predicate, name, seconds=6):
            until = time.monotonic() + seconds
            while time.monotonic() < until:
                for role, process in processes.items():
                    if process.poll() is not None:
                        log = args.out / f"{role}.log"
                        raise RuntimeError(f"{role} 已退出（{process.returncode}），见 {log}")
                pulse()
                rclpy.spin_once(node, timeout_sec=0.014)
                usb.poll()
                gripper.check()
                if predicate():
                    return
                time.sleep(0.01)
            recent = [(m.cycle_phase, m.destination, m.roi_request_id, m.reason) for m in statuses[-6:]]
            raise TimeoutError(f"{name}: recent status={recent}, switches={switches}")

        def saw(identifier, payload, after):
            return usb.has_reply(identifier, payload.hex(), after)

        def q(command, token, accepted=True):
            return struct.pack("<BBI", int(accepted), command, token)

        def command(name, token, accepted=True):
            began = usb.command(IDS[name], token)
            wait(lambda: saw(IDS["command_reply"], q(COMMANDS[name], token, accepted), began),
                 f"{name}，请求编号 {token}")
            return began

        def action(kind, token, source_frame=None):
            began = usb.action_request(kind, token, source_frame)
            wait(lambda: saw(IDS["command_reply"], q(COMMANDS["arrival_point"], token), began),
                 f"夹爪确认动作 {kind}")
            wait(lambda: saw(IDS["action_result"], struct.pack("<BBI", kind, 1, token), began),
                 f"夹爪完成动作 {kind}")

        processes = {}
        for role, binary in binaries.items():
            output = cleanup.enter_context((args.out / f"{role}.log").open("x"))
            process = subprocess.Popen([str(binary), "--role", role, "--ros-args", "--params-file", str(params_path)],
                                       stdin=subprocess.DEVNULL, stdout=output, stderr=subprocess.STDOUT)
            processes[role] = process
            cleanup.callback(stop, process)

        mode("seg")
        wait(lambda: statuses and len(usb.records) > 10, "等待节点和 USB 启动")
        action(0, 1)
        t = usb.selected_route()
        wait(lambda: saw(IDS["arrival_ack"], b"\x00", t), "夹取前拒绝投送请求")
        command("start_align", 2)
        wait(lambda: statuses[-1].cycle_phase == MissionStatus.CYCLE_INITIAL_LOCALIZING and any(m.valid and m.side == 0 for m in safety),
             "启动转向前定位整块安全区")
        whole_center = (4688, 5093)
        field_visible = False
        start = time.monotonic()
        wait(lambda: any(row["id"] == IDS["safety"] and row["data"].startswith("00") and row["steady"] >= start
                         for row in usb.records), "安全区丢失上报无效坐标")
        field_visible = True
        start = time.monotonic()
        wait(lambda: any(row["id"] == IDS["safety"] and row["steady"] >= start
                         and row["data"].startswith("01")
                         and struct.unpack_from("<hh", bytes.fromhex(row["data"]), 1) == whole_center
                         for row in usb.records), "重新收到整块安全区坐标")
        command("start_search", 3)

        routes = []
        for label, side, grasp, release, search in [("green", 1, 4, 5, 6), ("orange", 2, 7, 8, 9)]:
            cargo = str(mission[f"{label}_class_id"])
            start = time.monotonic()
            wait(lambda: usb.target_source_since(start) is not None, f"选择 {label} 取货目标")
            action(1, grasp, usb.target_source_since(start))
            cargo = None
            start = time.monotonic()
            wait(lambda: any(row["id"] == IDS["target"] and row["data"].startswith("00") and row["steady"] >= start
                             for row in usb.records), "投送请求前已丢失实时取货目标")
            t = usb.selected_route()
            wait(lambda: saw(IDS["arrival_ack"], b"\x01", t) and saw(IDS["roi_reply"], bytes([1, 1, side]), t)
                 and saw(IDS["turn_to_safety"], struct.pack("<BBI", 1, side, grasp), t), f"{label} 投送路线")
            if statuses[-1].roi_request_id != grasp:
                raise AssertionError(f"任务状态的趟次编号与 {grasp} 不符")
            repeated = usb.selected_route()
            wait(lambda: saw(IDS["roi_reply"], bytes([1, 1, side]), repeated)
                 and saw(IDS["turn_to_safety"], struct.pack("<BBI", 1, side, grasp), repeated), "重复查询仍返回原投送路线")
            command("turn_completed", grasp + 256, accepted=False)
            command("turn_completed", grasp)
            command("safety_arrived", grasp)
            action(2, release)
            start = command("delivery_completed", grasp)
            wait(lambda: statuses[-1].cycle_phase == MissionStatus.CYCLE_LOCALIZING_START and
                 any(row["id"] == IDS["safety"] and row["data"].startswith("01") and row["steady"] >= start
                     and struct.unpack_from("<hh", bytes.fromhex(row["data"]), 1) == whole_center
                     for row in usb.records), "投送完成后重新定位整块安全区")
            command("start_search", search)
            wait(lambda: statuses[-1].cycle_phase == MissionStatus.CYCLE_SEARCHING_TARGET and active == "seg",
                 "重新对准后恢复物块搜索")
            routes.append({"color": label, "side": side, "grasp_and_route": grasp})

        if any(m.cycle_phase == MissionStatus.CYCLE_CHECKING_ROI for m in statuses):
            raise AssertionError("直接投送进入了 ROI 检查状态")
        report = {
            "configuration_sha256": snapshot["configuration_sha256"],
            "roles": list(roles),
            "routes": routes,
            "modes": switches,
            "roi_check_states": 0,
            "field_loss_recovery": True,
            "gripper_pty": gripper.device,
            "usb_pty": usb.device,
        }
        (args.out / "result.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(json.dumps(report), flush=True)


if __name__ == "__main__":
    main()
