#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Ground-truth recording client for the FAST-LIO GT system (VLN experiments).

Runs on the robot (Jetson) next to FAST-LIO. Sets the per-run metadata
parameters on the fast_lio node, optionally re-seeds the localization pose,
then triggers the recording services.

Commands:
    readpose   : print one /Odometry pose as a start_poses.csv row (calibration)
    start      : (optionally set initial pose) + set metadata + start recording
    stop       : stop recording and write the CSV

Start-pose table (start_poses.csv), one line per instruction start point:
    instruction_id,x_m,y_m,z_m,yaw_deg
    lab_01,1.203,-0.510,0.012,87.5
Lines starting with '#' are comments. If --poses-file is given (or
./start_poses.csv exists) and the instruction is found, its pose is published
to /initialpose before recording starts. Tolerance: the seed only needs to be
roughly right (~0.5 m, ~10 deg) for scan-to-map matching to converge; the
frame itself is always the reference-map frame.

Requires: rclpy (comes with ROS 2). Example:
    source /Docker_Space/fastlio-space/install/setup.bash
    python3 gt_record.py readpose --name lab_01          # during calibration
    python3 gt_record.py start --source vln --instr lab_01 --poses-file start_poses.csv
    python3 gt_record.py stop
"""

import argparse
import csv
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import Trigger


def make_param(name, type_enum, attr, value):
    v = ParameterValue()
    v.type = type_enum
    setattr(v, attr, value)
    p = Parameter()
    p.name = name
    p.value = v
    return p


def wait_for_service(node, client, name, timeout_sec=10.0):
    if not client.wait_for_service(timeout_sec=timeout_sec):
        node.get_logger().error(
            "Service '%s' not available. Is FAST-LIO running? "
            "(ros2 launch fast_lio mapping_mid360[_localization].launch.py)" % name)
        sys.exit(1)


def call_service(node, client, request):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    if future.result() is None:
        node.get_logger().error("Service call failed or timed out.")
        sys.exit(1)
    return future.result()


def load_poses(path):
    """Load a start_poses.csv table -> {instruction_id: (x, y, z, yaw_deg)}."""
    poses = {}
    with open(path, "r") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or not row[0].strip() or row[0].strip().startswith("#"):
                continue
            if row[0].strip().lower() == "instruction_id":  # header
                continue
            if len(row) < 5:
                continue
            poses[row[0].strip()] = tuple(float(v) for v in row[1:5])
    return poses


def publish_initial_pose(node, x, y, z, yaw_deg):
    """Re-seed FAST-LIO localization via /initialpose (localization mode only)."""
    msg = PoseWithCovarianceStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = "camera_init"  # FAST-LIO world frame name
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.position.z = float(z)
    yaw = math.radians(float(yaw_deg))
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)
    for _ in range(5):  # a few repeats, ~1 s total
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.2)
    node.get_logger().info(
        "Initial pose published to /initialpose: [%.3f, %.3f, %.3f, yaw %.2f deg]. "
        "Waiting 1 s for the filter to settle..." % (x, y, z, float(yaw_deg)))
    time.sleep(1.0)


def cmd_readpose(node, name):
    """Print the current /Odometry pose as one start_poses.csv row."""
    done = {"ok": False}

    def cb(msg):
        if done["ok"]:
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        print("%s,%.4f,%.4f,%.4f,%.2f" % (name, p.x, p.y, p.z, yaw))
        done["ok"] = True

    node.create_subscription(Odometry, "/Odometry", cb, 10)
    deadline = time.time() + 10.0
    while rclpy.ok() and not done["ok"] and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not done["ok"]:
        node.get_logger().error("No /Odometry received in 10 s. Is FAST-LIO running and localized?")
        sys.exit(1)


def main():
    parser = argparse.ArgumentParser(description="GT recording client (start/stop FAST-LIO path recording)")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_start = sub.add_parser("start", help="start recording one trajectory")
    p_start.add_argument("--source", default="manual",
                         choices=["manual", "vln", "baseline"],
                         help="control source label (goes into CSV name/header)")
    p_start.add_argument("--instr", default="",
                         help="instruction/episode id, e.g. lab_03 (goes into CSV name/header)")
    p_start.add_argument("--note", default="",
                         help="free-form note (goes into CSV header only)")
    p_start.add_argument("--every-n", type=int, default=None,
                         help="record a pose every N LiDAR frames (default: keep current, config default 1)")
    p_start.add_argument("--poses-file", default=None,
                         help="start_poses.csv path; if --instr is found in it, the pose is "
                              "published to /initialpose before recording (default: ./start_poses.csv if present)")
    p_start.add_argument("--pose", nargs=4, type=float, default=None, metavar=("X", "Y", "Z", "YAW_DEG"),
                         help="set the initial pose directly instead of the table")

    p_stop = sub.add_parser("stop", help="stop recording and write the CSV")

    p_read = sub.add_parser("readpose", help="print one /Odometry pose as a start_poses.csv row")
    p_read.add_argument("--name", default="instr", help="instruction_id column value for the printed row")

    for p in (p_start, p_stop, p_read):
        p.add_argument("--fastlio-node", default="/laser_mapping",
                       help="name of the fast_lio node (default: /laser_mapping)")

    args = parser.parse_args()

    rclpy.init()
    node = Node("gt_record_client")

    try:
        if args.cmd == "readpose":
            cmd_readpose(node, args.name)
            return

        if args.cmd == "start":
            # 1) optional initial pose re-seed (localization mode)
            pose = None
            if args.pose is not None:
                pose = tuple(args.pose)
            elif args.instr:
                poses_file = args.poses_file
                if poses_file is None and os.path.isfile("start_poses.csv"):
                    poses_file = "start_poses.csv"
                if poses_file is not None:
                    if not os.path.isfile(poses_file):
                        node.get_logger().error("Poses file not found: %s" % poses_file)
                        sys.exit(1)
                    table = load_poses(poses_file)
                    if args.instr in table:
                        pose = table[args.instr]
                    else:
                        node.get_logger().warn(
                            "Instruction '%s' not in %s (found: %s); starting without re-seed."
                            % (args.instr, poses_file, ", ".join(sorted(table)) or "none"))
            if pose is not None:
                publish_initial_pose(node, *pose)

            # 2) metadata parameters
            param_cli = node.create_client(SetParameters, args.fastlio_node + "/set_parameters")
            start_cli = node.create_client(Trigger, "/start_path_record")
            wait_for_service(node, param_cli, args.fastlio_node + "/set_parameters")
            wait_for_service(node, start_cli, "/start_path_record")

            req = SetParameters.Request()
            req.parameters = [
                make_param("record.control_source", ParameterType.PARAMETER_STRING,
                           "string_value", args.source),
                make_param("record.instruction_id", ParameterType.PARAMETER_STRING,
                           "string_value", args.instr),
                make_param("record.note", ParameterType.PARAMETER_STRING,
                           "string_value", args.note),
            ]
            if args.every_n is not None:
                req.parameters.append(
                    make_param("record.sample_every_n", ParameterType.PARAMETER_INTEGER,
                               "integer_value", int(args.every_n)))
            resp = call_service(node, param_cli, req)
            bad = [r.reason for r in resp.results if not r.successful]
            if bad:
                node.get_logger().error("Failed to set parameters: %s" % "; ".join(bad))
                sys.exit(1)

            res = call_service(node, start_cli, Trigger.Request())
            print(("[OK] " if res.success else "[FAIL] ") + res.message)
            sys.exit(0 if res.success else 1)

        elif args.cmd == "stop":
            stop_cli = node.create_client(Trigger, "/stop_path_record")
            wait_for_service(node, stop_cli, "/stop_path_record")
            res = call_service(node, stop_cli, Trigger.Request())
            print(("[OK] " if res.success else "[FAIL] ") + res.message)
            sys.exit(0 if res.success else 1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
