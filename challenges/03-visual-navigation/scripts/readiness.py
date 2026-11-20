"""Read-only DDS readiness probe, executed with the host Humble Python."""
import json
from pathlib import Path
import sys
import time

from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener


def probe(directory, timeout):
    rclpy.init()
    node = rclpy.create_node("luna_readiness")
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    samples = {}
    names = ("/amcl", "/planner_server", "/controller_server")
    lifecycle = {name: node.create_client(GetState, name + "/get_state") for name in names}
    requests = {}
    active = {}

    def remember(topic, header, **details):
        stamp = header.sec * 1_000_000_000 + header.nanosec
        record = samples.setdefault(topic, {"messages": 0, "advances": 0, "stamp_ns": 0})
        record["messages"] += 1
        if stamp > record["stamp_ns"]:
            record["advances"] += 1
        record.update(stamp_ns=stamp, **details)

    subscriptions = [
        node.create_subscription(Clock, "/clock", lambda m: remember("/clock", m.clock),
                                 qos_profile_sensor_data),
        node.create_subscription(Odometry, "/chassis/odom",
                                 lambda m: remember("/chassis/odom", m.header.stamp,
                                                    frame=m.header.frame_id,
                                                    child_frame=m.child_frame_id),
                                 qos_profile_sensor_data),
        node.create_subscription(PointCloud2, "/front_3d_lidar/lidar_points",
                                 lambda m: remember("/front_3d_lidar/lidar_points",
                                                    m.header.stamp, frame=m.header.frame_id,
                                                    points=m.height * m.width) if m.data else None,
                                 qos_profile_sensor_data),
        node.create_subscription(LaserScan, "/scan",
                                 lambda m: remember("/scan", m.header.stamp, frame=m.header.frame_id,
                                                    ranges=len(m.ranges)) if m.ranges else None,
                                 qos_profile_sensor_data),
    ]
    topics = {"/clock", "/chassis/odom", "/front_3d_lidar/lidar_points", "/scan"}
    start = time.monotonic()
    frames = {}
    ready = False

    def record():
        report = {"ready": ready, "wall_seconds": time.monotonic() - start,
                  "messages": samples, "tf_stamp_ns": frames, "lifecycle": active}
        temporary = directory / "readiness.tmp"
        temporary.write_text(json.dumps(report, indent=2) + "\n")
        temporary.replace(directory / "readiness.json")
        return report

    next_snapshot = start
    try:
        while time.monotonic() - start < timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            for name, client in lifecycle.items():
                request = requests.get(name)
                if request is not None and request.done():
                    active[name] = request.result().current_state.label
                    del requests[name]
                if name not in requests and active.get(name) != "active" and client.service_is_ready():
                    requests[name] = client.call_async(GetState.Request())
            for target, source in (("map", "base_link"), ("odom", "base_link"),
                                   ("base_link", "front_3d_lidar")):
                try:
                    transform = buffer.lookup_transform(target, source, Time())
                except TransformException:
                    continue
                stamp = transform.header.stamp
                frames[f"{target} <- {source}"] = stamp.sec * 1_000_000_000 + stamp.nanosec
            if time.monotonic() >= next_snapshot:
                record()
                next_snapshot = time.monotonic() + 2
            if (topics <= samples.keys() and all(s["advances"] >= 2 for s in samples.values())
                    and len(frames) == 3 and all(active.get(name) == "active" for name in names)):
                ready = True
                break
        print(json.dumps(record()), flush=True)
        return 0 if ready else 1
    finally:
        listener.unregister()
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(probe(Path(sys.argv[1]), float(sys.argv[2])))
