#!/usr/bin/env python3
"""Daemon-invoked read-only Nav2 planning and calibrated LiDAR snapshots. No motion."""

import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def stamp(value):
    return value.sec + value.nanosec / 1e9


def xy_yaw(p):
    q = p.rotation
    return [
        p.translation.x,
        p.translation.y,
        math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
    ]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--goal", required=True, help="JSON x/y/yaw; planning only")
    parser.add_argument("--sensor-only", action="store_true")
    args = parser.parse_args()
    goal = json.loads(args.goal)
    if set(goal) != {"x", "y", "yaw"} or not all(
        math.isfinite(v) for v in goal.values()
    ):
        raise ValueError("Finite map pose required")
    rclpy.init()
    node = Node("rosclaw_semantic_readonly_plan")
    state = {}
    buffer = Buffer()
    node._tf_listener = TransformListener(buffer, node)
    subscriptions = []
    topics = [
        ("/clock", Clock, "clock"),
        ("/amcl_pose", PoseWithCovarianceStamped, "amcl"),
        ("/scan", LaserScan, "scan"),
    ]
    if not args.sensor_only:
        topics.append(("/global_costmap/costmap", OccupancyGrid, "costmap"))
    for topic, cls, key in topics:
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        if key == "costmap":
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        subscriptions.append(
            node.create_subscription(
                cls, topic, lambda m, k=key: state.update({k: (m, time.time())}), qos
            )
        )
    deadline = time.monotonic() + 25
    first_clock = None
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if "clock" not in state:
            continue
        clock = stamp(state["clock"][0].clock)
        if first_clock is None:
            first_clock = clock
        required = {"clock", "scan", "amcl"} | (
            set() if args.sensor_only else {"costmap"}
        )
        if required <= state.keys() and clock > first_clock + 0.1:
            try:
                buffer.lookup_transform("map", "base_link", rclpy.time.Time())
                buffer.lookup_transform(
                    "map", state["scan"][0].header.frame_id, rclpy.time.Time()
                )
            except Exception:
                continue
            break
    else:
        raise TimeoutError("Fresh clock, TF, AMCL, scan and costmap not all available")

    def capture():
        now = time.time()
        clock = stamp(state["clock"][0].clock)
        base = buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        scan = state["scan"][0]
        lidar = buffer.lookup_transform("map", scan.header.frame_id, rclpy.time.Time())
        amcl = state["amcl"][0]
        if (
            abs(lidar.transform.rotation.x) > 0.02
            or abs(lidar.transform.rotation.y) > 0.02
        ):
            raise ValueError("Planar scan transform assumption not satisfied")
        # AMCL is event-driven and can retain a pose while stationary. Require a
        # received pose plus independent fresh TF/scan/clock, not invented updates.
        if amcl.header.frame_id != "map":
            raise ValueError("AMCL frame mismatch")
        for key in ["clock", "scan"] + ([] if args.sensor_only else ["costmap"]):
            if not 0 <= now - state[key][1] <= 3:
                raise ValueError("Stale received " + key)
        for value in [
            stamp(base.header.stamp),
            stamp(scan.header.stamp),
            stamp(lidar.header.stamp),
        ]:
            if not -0.2 <= clock - value <= 1.5:
                raise ValueError("Stale/future TF or scan in SIM time")
        pose = xy_yaw(base.transform)
        if (
            math.dist(pose[:2], [amcl.pose.pose.position.x, amcl.pose.pose.position.y])
            > 0.6
        ):
            raise ValueError("AMCL/TF pose disagreement")
        result = {
            "wall_time": now,
            "sim_time": clock,
            "map_frame": "map",
            "base_pose": pose,
            "tf_stamp": stamp(base.header.stamp),
            "amcl_stamp": stamp(amcl.header.stamp),
            "scan": {
                "stamp": stamp(scan.header.stamp),
                "frame": scan.header.frame_id,
                "angle_min": scan.angle_min,
                "angle_increment": scan.angle_increment,
                "range_min": scan.range_min,
                "range_max": scan.range_max,
                "ranges": [float(x) if math.isfinite(x) else None for x in scan.ranges],
                "map_sensor_xy_yaw": xy_yaw(lidar.transform),
                "tf_stamp": stamp(lidar.header.stamp),
            },
        }
        if not args.sensor_only:
            grid = state["costmap"][0]
            if (
                grid.header.frame_id != "map"
                or not -0.2 <= clock - stamp(grid.header.stamp) <= 3
            ):
                raise ValueError("Costmap frame/time mismatch")
            o = grid.info.origin
            if abs(o.orientation.z) > 1e-8 or abs(o.orientation.w - 1) > 1e-8:
                raise ValueError("Rotated costmap unsupported")
            result["costmap"] = {
                "stamp": stamp(grid.header.stamp),
                "resolution": grid.info.resolution,
                "width": grid.info.width,
                "height": grid.info.height,
                "origin": [o.position.x, o.position.y],
                "data": list(grid.data),
            }
        return result

    if args.sensor_only:
        output = capture()
    else:
        client = ActionClient(node, ComputePathToPose, "/compute_path_to_pose")
        if not client.wait_for_server(timeout_sec=5):
            raise TimeoutError("Planner action unavailable")
        message = ComputePathToPose.Goal()
        message.goal = PoseStamped()
        message.goal.header.frame_id = "map"
        message.goal.pose.position.x = float(goal["x"])
        message.goal.pose.position.y = float(goal["y"])
        message.goal.pose.orientation.z = math.sin(goal["yaw"] / 2)
        message.goal.pose.orientation.w = math.cos(goal["yaw"] / 2)
        message.planner_id = "GridBased"
        message.use_start = False
        future = client.send_goal_async(message)
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not future.done() or not future.result().accepted:
            raise RuntimeError("Planning goal not accepted")
        handle = future.result()
        future = handle.get_result_async()
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not future.done():
            handle.cancel_goal_async()
            raise TimeoutError("Read-only planner deadline")
        result = future.result()
        if result.status != 4 or result.result.error_code != 0:
            raise ValueError("Nav2 planning did not succeed")
        output = capture()
        path = result.result.path
        if path.header.frame_id != "map" or not path.poses:
            raise ValueError("Missing map-frame path")
        poses = []
        for p in path.poses:
            q = p.pose.orientation
            poses.append(
                [
                    p.pose.position.x,
                    p.pose.position.y,
                    math.atan2(
                        2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
                    ),
                ]
            )
        output.update(
            planner_status=result.status,
            planner_error_code=result.result.error_code,
            path=poses,
            path_stamp=stamp(path.header.stamp),
        )
    print(json.dumps(output, allow_nan=False))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
