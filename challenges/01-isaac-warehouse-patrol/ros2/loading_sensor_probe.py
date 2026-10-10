#!/usr/bin/env python3
"""Read-only PointCloud2 audit at measurement-time TF; no motion or truth access."""

import base64
import hashlib
import argparse
import json
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry, OccupancyGrid
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformListener


def stamp(t):
    return t.sec + t.nanosec / 1e9


def rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--frames", type=int, default=3)
    p.add_argument("--output", required=True)
    a = p.parse_args()
    rclpy.init()
    n = Node("rosclaw_loading_readonly_pointcloud_audit")
    b = Buffer()
    n._tf_listener = TransformListener(b, n)
    state = {}
    subs = []
    for topic, typ, key in [
        ("/front_3d_lidar/lidar_points", PointCloud2, "cloud"),
        ("/scan", LaserScan, "scan"),
        ("/chassis/odom", Odometry, "odom"),
        ("/local_costmap/costmap", OccupancyGrid, "costmap"),
        ("/clock", Clock, "clock"),
    ]:
        subs.append(
            n.create_subscription(
                typ,
                topic,
                lambda m, k=key: state.update({k: (m, time.time())}),
                qos_profile_sensor_data,
            )
        )
    results = []
    errors = []
    last = None
    deadline = time.monotonic() + 60
    while time.monotonic() < deadline and len(results) < a.frames:
        rclpy.spin_once(n, timeout_sec=0.05)
        if not {"cloud", "clock"} <= state.keys():
            continue
        m, received = state["cloud"]
        ts = stamp(m.header.stamp)
        if ts == last:
            continue
        try:
            tf = b.lookup_transform(
                "map", m.header.frame_id, rclpy.time.Time.from_msg(m.header.stamp)
            )
            clock = stamp(state["clock"][0].clock)
            if not -0.05 <= clock - ts <= 1.5 or time.time() - received > 3:
                raise ValueError("stale/future cloud")
            if m.row_step * m.height != len(m.data) or m.point_step <= 0:
                raise ValueError("PointCloud2 buffer shape mismatch")
            points = point_cloud2.read_points_numpy(
                m, field_names=("x", "y", "z"), skip_nans=False
            )
            points = np.asarray(points, dtype=float).reshape(-1, 3)
            finite = np.isfinite(points).all(axis=1)
            valid = points[finite]
            if len(valid) == 0:
                raise ValueError("no finite XYZ")
            t = tf.transform.translation
            mat = rotation(tf.transform.rotation)
            world = valid @ mat.T + np.array([t.x, t.y, t.z])
            origin = [t.x, t.y, t.z]
            results.append(
                {
                    "stamp": ts,
                    "clock": clock,
                    "received_wall": received,
                    "frame": m.header.frame_id,
                    "tf_stamp": stamp(tf.header.stamp),
                    "map_sensor_origin": origin,
                    "map_sensor_rotation": mat.tolist(),
                    "width": m.width,
                    "height": m.height,
                    "point_step": m.point_step,
                    "row_step": m.row_step,
                    "is_bigendian": m.is_bigendian,
                    "is_dense": m.is_dense,
                    "fields": [
                        {
                            "name": f.name,
                            "offset": f.offset,
                            "datatype": f.datatype,
                            "count": f.count,
                        }
                        for f in m.fields
                    ],
                    "decoded_points": len(points),
                    "finite_points": int(finite.sum()),
                    "nonfinite_points": int((~finite).sum()),
                    "points_map": world.tolist(),
                    "raw_data_base64": base64.b64encode(bytes(m.data)).decode("ascii"),
                    "raw_data_sha256": hashlib.sha256(bytes(m.data)).hexdigest(),
                    "map_z_percentiles": np.percentile(
                        world[:, 2], [0, 1, 10, 50, 90, 99, 100]
                    ).tolist(),
                    "ground_band_count": int((np.abs(world[:, 2]) <= 0.08).sum()),
                    "near_ground_obstacle_count": int(
                        ((world[:, 2] > 0.08) & (world[:, 2] < 0.8)).sum()
                    ),
                }
            )
            last = ts
        except Exception as e:
            errors.append(str(e))
    output = {
        "schema": "rosclaw.loading_sensor_snapshot.v1",
        "status": "PASS" if len(results) == a.frames else "INCOMPLETE",
        "wall_time": time.time(),
        "frames": results,
        "errors": list(dict.fromkeys(errors)),
        "topics_seen": list(state),
        "graph": n.get_topic_names_and_types(),
        "endpoint_qos": {
            topic: [
                {
                    "node": i.node_name,
                    "reliability": str(i.qos_profile.reliability),
                    "durability": str(i.qos_profile.durability),
                }
                for i in n.get_publishers_info_by_topic(topic)
            ]
            for topic in [
                "/front_3d_lidar/lidar_points",
                "/scan",
                "/tf",
                "/chassis/odom",
                "/local_costmap/costmap",
            ]
        },
    }
    from pathlib import Path

    Path(a.output).write_text(json.dumps(output, allow_nan=False) + "\n")
    print(
        json.dumps(
            {
                "status": output["status"],
                "frames": len(results),
                "topics_seen": output["topics_seen"],
                "errors": output["errors"],
            }
        )
    )
    n.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if output["status"] == "PASS" else 1)


if __name__ == "__main__":
    main()
