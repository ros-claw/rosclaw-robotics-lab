"""Read-only LiDAR/costmap/path evidence for the unmapped static box experiment."""

import argparse
import json
import math
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from tf2_ros import Buffer, TransformListener


def matrix(transform):
    q = transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    rotation = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
    t = transform.translation
    return rotation, np.array([t.x, t.y, t.z])


class Witness(Node):
    def __init__(self, output):
        super().__init__("rosclaw_unmapped_box_witness")
        self.output = output
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.observation_subscriptions = [
            self.create_subscription(
                PointCloud2,
                "/front_3d_lidar/lidar_points",
                self.cloud,
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                OccupancyGrid,
                "/local_costmap/costmap",
                self.costmap,
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                NavPath, "/plan", self.plan, qos_profile_sensor_data
            ),
        ]

    def record(self, event, **values):
        with self.output.open("a") as stream:
            stream.write(
                json.dumps(
                    {"wall_time": time.time(), "event": event, **values},
                    allow_nan=False,
                )
                + "\n"
            )

    def cloud(self, msg):
        try:
            t = self.tf.lookup_transform(
                "map", msg.header.frame_id, Time.from_msg(msg.header.stamp)
            ).transform
            data = point_cloud2.read_points(
                msg, field_names=["x", "y", "z"], skip_nans=True
            )
            points = np.stack([data["x"], data["y"], data["z"]], axis=-1).reshape(-1, 3)
            rot, pos = matrix(t)
            world = points @ rot.T + pos
            mask = (
                (world[:, 0] > -4.15)
                & (world[:, 0] < -2.85)
                & (world[:, 1] > 3.35)
                & (world[:, 1] < 4.65)
                & (world[:, 2] > 0.05)
                & (world[:, 2] < 1.15)
            )
            hits = world[mask]
            self.record(
                "lidar_box_observation",
                stamp=msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                frame_id=msg.header.frame_id,
                point_count=len(world),
                box_hit_count=len(hits),
                measured_box_hits_map=hits[:20].tolist(),
            )
        except Exception as exc:
            self.record("cloud_transform_unavailable", error=str(exc))

    def costmap(self, msg):
        try:
            points = np.array([[x, y, 0.5] for x in [-4, -3] for y in [3.5, 4.5]])
            if msg.header.frame_id != "map":
                rot, pos = matrix(
                    self.tf.lookup_transform(
                        msg.header.frame_id, "map", Time()
                    ).transform
                )
                points = points @ rot.T + pos
            origin = msg.info.origin
            yaw = math.atan2(
                2 * origin.orientation.w * origin.orientation.z,
                1 - 2 * origin.orientation.z**2,
            )
            xy = points[:, :2] - np.array([origin.position.x, origin.position.y])
            xy = xy @ np.array(
                [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
            )
            cells = np.floor(xy / msg.info.resolution).astype(int)
            lo = cells.min(axis=0)
            hi = cells.max(axis=0)
            array = np.array(msg.data).reshape(msg.info.height, msg.info.width)
            if (
                lo[0] < 0
                or lo[1] < 0
                or hi[0] >= msg.info.width
                or hi[1] >= msg.info.height
            ):
                return
            values = array[lo[1] : hi[1] + 1, lo[0] : hi[0] + 1]
            self.record(
                "local_costmap_box_observation",
                frame_id=msg.header.frame_id,
                stamp=msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                occupied_cells=int((values >= 65).sum()),
                total_cells=int(values.size),
                maximum_cost=int(values.max()),
            )
        except Exception as exc:
            self.record("costmap_transform_unavailable", error=str(exc))

    def plan(self, msg):
        self.record(
            "nav2_global_plan",
            frame_id=msg.header.frame_id,
            stamp=msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
            points_xy=[[p.pose.position.x, p.pose.position.y] for p in msg.poses],
        )


p = argparse.ArgumentParser()
p.add_argument("--output", type=Path, required=True)
a, ros_args = p.parse_known_args()
rclpy.init(args=ros_args)
n = Witness(a.output)
try:
    rclpy.spin(n)
finally:
    n.destroy_node()
    rclpy.shutdown()
