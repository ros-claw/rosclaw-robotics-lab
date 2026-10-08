"""Read-only ROS evidence collection; never sends navigation or velocity commands."""

import argparse
import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output", type=Path, default=Path("/lab/reports/runtime-audit.json")
    )
    args = parser.parse_args()
    rclpy.init()
    node = Node("rosclaw_warehouse_audit")
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    discovery_deadline = time.monotonic() + 15
    graph = {}
    while time.monotonic() < discovery_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        graph = dict(node.get_topic_names_and_types())
        if {
            "/clock",
            "/chassis/odom",
            "/scan",
            "/map",
            "/front_3d_lidar/lidar_points",
        }.issubset(graph):
            break
    samples = {}
    subscriptions = []

    def receive(topic, msg):
        row = samples.setdefault(topic, {"count": 0, "first_wall_time": time.time()})
        row["count"] += 1
        row["last_wall_time"] = time.time()
        if hasattr(msg, "header"):
            row["frame_id"] = msg.header.frame_id
            row["stamp"] = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if hasattr(msg, "clock"):
            stamp = msg.clock.sec + msg.clock.nanosec / 1e9
            row.setdefault("first_sim_time", stamp)
            row["last_sim_time"] = stamp
        if hasattr(msg, "ranges"):
            row["range_count"] = len(msg.ranges)
        if hasattr(msg, "width"):
            row["width"] = msg.width

    for topic in [
        "/clock",
        "/chassis/odom",
        "/scan",
        "/front_3d_lidar/lidar_points",
        "/map",
    ]:
        if topic in graph:
            subscriptions.append(
                node.create_subscription(
                    get_message(graph[topic][0]),
                    topic,
                    lambda msg, topic=topic: receive(topic, msg),
                    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                    if topic == "/map"
                    else qos_profile_sensor_data,
                )
            )
    start = time.monotonic()
    while time.monotonic() - start < 12:
        rclpy.spin_once(node, timeout_sec=0.1)
    transforms = {}
    for source, target in [
        ("map", "odom"),
        ("odom", "base_link"),
        ("map", "base_link"),
    ]:
        try:
            tf = buffer.lookup_transform(source, target, rclpy.time.Time())
            transforms[f"{source}->{target}"] = {
                "translation": [
                    tf.transform.translation.x,
                    tf.transform.translation.y,
                    tf.transform.translation.z,
                ],
                "stamp": tf.header.stamp.sec + tf.header.stamp.nanosec / 1e9,
            }
        except Exception as exc:
            transforms[f"{source}->{target}"] = {"status": "UNKNOWN", "error": str(exc)}
    evidence = {
        "wall_time": time.time(),
        "domain": 61,
        "nodes": node.get_node_names_and_namespaces(),
        "topics": graph,
        "services": node.get_service_names_and_types(),
        "samples": samples,
        "transforms": transforms,
        "tf_frames_yaml": buffer.all_frames_as_yaml(),
    }
    args.output.write_text(json.dumps(evidence, indent=2))
    listener.unregister()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
