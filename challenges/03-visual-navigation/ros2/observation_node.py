"""Capture paired Isaac RGB-D frames and exact-time TF on the host ROS environment."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time

from geometry_msgs.msg import TransformStamped
import numpy as np
from PIL import Image as PillowImage
import rclpy
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from navigation.camera_mount import BASE_FRAME, OPTICAL_FRAME, OPTICAL_XYZW, TOPICS, TRANSLATION_M, yaw_mount_xyzw
from navigation.contracts import CaptureEpoch, ObservationRejected
from navigation.geometry import CameraIntrinsics

QUEUE_SIZE = 12


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def frame_pixels(message, dtype, channels):
    row_width = message.width * channels * dtype.itemsize
    if message.step < row_width or len(message.data) != message.step * message.height:
        raise ObservationRejected("invalid image stride")
    if channels == 1:
        shape = (message.height, message.width)
        strides = (message.step, dtype.itemsize)
    else:
        shape = (message.height, message.width, channels)
        strides = (message.step, channels * dtype.itemsize, dtype.itemsize)
    return np.ndarray(shape, dtype, buffer=message.data, strides=strides)


def serialize_tf(transform):
    t, q = transform.transform.translation, transform.transform.rotation
    return {"target": transform.header.frame_id, "source": transform.child_frame_id,
            "stamp_ns": str(stamp_ns(transform.header.stamp)),
            "xyz_m": [t.x, t.y, t.z], "quaternion_xyzw": [q.x, q.y, q.z, q.w]}


def local_mount(yaw):
    static = TransformStamped()
    static.header.frame_id = BASE_FRAME
    static.child_frame_id = OPTICAL_FRAME
    position, rotation = static.transform.translation, static.transform.rotation
    position.x, position.y, position.z = TRANSLATION_M
    rotation.x, rotation.y, rotation.z, rotation.w = yaw_mount_xyzw(OPTICAL_XYZW, yaw)
    return static


def observation_frames(rgb, depth, calibration):
    for message in (rgb, depth, calibration):
        if message.header.frame_id != OPTICAL_FRAME:
            raise ObservationRejected("unexpected optical frame")
        if (message.width, message.height) != (rgb.width, rgb.height):
            raise ObservationRejected("unaligned image dimensions")
    if rgb.encoding not in {"rgb8", "rgba8"} or depth.encoding != "32FC1":
        raise ObservationRejected(f"unsupported encodings {rgb.encoding}/{depth.encoding}")
    if any(calibration.d):
        raise ObservationRejected("rectified pinhole calibration required")
    k = calibration.k
    intrinsics = CameraIntrinsics(rgb.width, rgb.height, k[0], k[4], k[2], k[5])
    color = frame_pixels(rgb, np.dtype("u1"), 3 if rgb.encoding == "rgb8" else 4)
    z = frame_pixels(depth, np.dtype(">f4" if depth.is_bigendian else "<f4"), 1)
    valid = np.isfinite(z) & (z > 0)
    if not valid.any():
        raise ObservationRejected("no valid positive depth pixels")
    return intrinsics, color[:, :, :3], z, valid


def save_capture(directory, run_id, identity, frames, rgb, depth, calibration, tf, local_revision, received_ns, calibration_mode):
    intrinsics, color, z, valid = frames
    prefix = f"e{identity.epoch:03d}-f{identity.sequence:06d}"
    rgb_path = directory / f"{prefix}.png"
    depth_path = directory / f"{prefix}.npy"
    PillowImage.fromarray(color).save(rgb_path)
    np.save(depth_path, z)
    u, v = rgb.width // 2, rgb.height // 2
    center = intrinsics.unproject(u, v, float(z[v, u])) if valid[v, u] else None
    return {
        "schema_version": 1,
        "source": "isaac-live-calibration" if calibration_mode else "isaac-live",
        "navigation_eligible": False,
        "session_id": run_id,
        "episode_id": run_id,
        "epoch": identity.epoch,
        "frame_seq": identity.sequence,
        "stamp_ns": str(identity.stamp_ns),
        "rgb_receive_mono_ns": str(received_ns),
        "recorded_mono_ns": str(time.monotonic_ns()),
        "rgb": {"ref": rgb_path.name, "encoding": rgb.encoding,
                "sha256": hashlib.sha256(rgb_path.read_bytes()).hexdigest()},
        "depth": {"ref": depth_path.name, "encoding": depth.encoding,
                  "sha256": hashlib.sha256(depth_path.read_bytes()).hexdigest(),
                  "semantics": "camera z in meters",
                  "valid_fraction": float(valid.mean()), "center_xyz_m": center},
        "camera": {"width": calibration.width, "height": calibration.height,
                   "optical_frame": OPTICAL_FRAME, "k": list(calibration.k),
                   "distortion_model": calibration.distortion_model, "d": list(calibration.d)},
        "transforms": tf,
        "localization_revision": local_revision,
    }


def collect(directory, run_id, wanted, timeout, calibration_mode, yaw):
    rclpy.init()
    node = rclpy.create_node("luna_observation")
    tf_buffer = Buffer(cache_time=Duration(seconds=5))
    listener = TransformListener(tf_buffer, node, qos=QoSProfile(
        depth=100, reliability=ReliabilityPolicy.BEST_EFFORT
    ))
    broadcaster = StaticTransformBroadcaster(node)
    mount = local_mount(yaw)
    broadcaster.sendTransform(mount)
    tf_buffer.set_transform_static(mount, "luna_observation")
    epoch = CaptureEpoch()
    messages = {kind: {} for kind in TOPICS}
    receipts = {kind: 0 for kind in TOPICS}
    rejected = {}
    captured = 0
    localization_revision = 0
    previous_map_odom = None
    last_tf_error = None

    def clock(msg):
        nonlocal previous_map_odom, localization_revision
        if epoch.observe_clock(stamp_ns(msg.clock)):
            for cache in messages.values():
                cache.clear()
            tf_buffer.clear()
            tf_buffer.set_transform_static(mount, "luna_observation")
            previous_map_odom = None
            localization_revision += 1

    def receive(kind, message):
        receipts[kind] += 1
        cache = messages[kind]
        stamp = stamp_ns(message.header.stamp)
        cache[stamp] = (message, time.monotonic_ns())
        if len(cache) > QUEUE_SIZE:
            del cache[min(cache)]

    node.create_subscription(Clock, "/clock", clock, qos_profile_sensor_data)
    for kind, topic in TOPICS.items():
        node.create_subscription(
            CameraInfo if kind == "info" else Image, topic,
            lambda msg, kind=kind: receive(kind, msg), qos_profile_sensor_data,
        )

    start = time.monotonic()
    try:
        with (directory / "observations.jsonl").open("x") as stream:
            while captured < wanted and time.monotonic() - start < timeout:
                rclpy.spin_once(node, timeout_sec=0.1)
                common = messages["rgb"].keys() & messages["depth"].keys() & messages["info"].keys()
                for stamp in sorted(common):
                    if not epoch.clock_ns or stamp > epoch.clock_ns:
                        continue
                    when = Time(nanoseconds=stamp)
                    try:
                        odom_camera = serialize_tf(tf_buffer.lookup_transform("odom", OPTICAL_FRAME, when))
                        map_odom = None
                        if not calibration_mode:
                            map_odom = serialize_tf(tf_buffer.lookup_transform("map", "odom", when))
                    except TransformException as exc:
                        last_tf_error = str(exc)
                        # Keep pending only while TF can still arrive for this stamp.
                        continue
                    (rgb, received_ns), (depth, _), (calibration, _) = (
                        messages[kind].pop(stamp) for kind in TOPICS
                    )
                    try:
                        frames = observation_frames(rgb, depth, calibration)
                        identity = epoch.accept_capture(stamp, epoch.epoch)
                    except ObservationRejected as exc:
                        rejected[str(exc)] = rejected.get(str(exc), 0) + 1
                        continue
                    if previous_map_odom is not None:
                        shift = math.dist(previous_map_odom["xyz_m"], map_odom["xyz_m"])
                        previous_q = previous_map_odom["quaternion_xyzw"]
                        current_q = map_odom["quaternion_xyzw"]
                        dot = abs(sum(a * b for a, b in zip(previous_q, current_q)))
                        if shift > 0.25 or 2 * math.acos(min(1.0, dot)) > math.radians(10):
                            localization_revision += 1
                    previous_map_odom = map_odom
                    observation = save_capture(directory, run_id, identity, frames, rgb, depth,
                                               calibration, [odom_camera] if map_odom is None
                                               else [odom_camera, map_odom], localization_revision,
                                               received_ns, calibration_mode)
                    observation["diagnostic_camera_yaw_deg"] = yaw
                    stream.write(json.dumps(observation, allow_nan=False) + "\n")
                    stream.flush()
                    captured += 1
                    if captured == wanted:
                        break
    finally:
        summary = {"captured": captured, "requested": wanted, "received": receipts,
                   "epoch": epoch.epoch, "rejected": rejected,
                   "wall_seconds": time.monotonic() - start, "last_tf_error": last_tf_error,
                   "tf_frames": tf_buffer.all_frames_as_yaml(),
                   "pending_exact_stamps": list(sorted(
                       messages["rgb"].keys() & messages["depth"].keys() & messages["info"].keys()
                   )),
                   "last_stamps_ns": {key: max(cache, default=None) for key, cache in messages.items()}}
        (directory / "observation_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        print(json.dumps({key: summary[key] for key in
                          ("captured", "requested", "received", "wall_seconds", "last_tf_error")}),
              flush=True)
        listener.unregister()
        node.destroy_node()
        rclpy.shutdown()
    return 0 if captured == wanted else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("run_id")
    parser.add_argument("--count", type=int, default=5)
    parser.add_argument("--timeout", type=float, default=30)
    parser.add_argument("--calibration", action="store_true")
    parser.add_argument("--diagnostic-camera-yaw", type=float, default=0.0)
    args = parser.parse_args()
    args.directory.mkdir(parents=True, exist_ok=False)
    sys.exit(collect(args.directory, args.run_id, args.count, args.timeout, args.calibration, args.diagnostic_camera_yaw))
