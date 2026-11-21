"""Deterministic single-fork exercise through the protected Nav2 action pipe."""

import argparse
import json
import math
from pathlib import Path
import sys
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import ComputePathThroughPoses, FollowPath
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from navigation.candidates import Gate, validate_corridor
from navigation.contracts import StopDwell


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class Evidence:
    def __init__(self, stream):
        self.stream = stream
        self.sequence = 0
        self.clock = None
        self.odom = None
        self.grid = None
        self.last_command = None

    def log(self, event, **payload):
        self.sequence += 1
        record = {"event": event, "event_seq": self.sequence, "source": "controller-test",
                  "sim_stamp_ns": str(self.clock) if self.clock is not None else None,
                  "host_mono_ns": str(time.monotonic_ns()), **payload}
        self.stream.write(json.dumps(record, allow_nan=False) + "\n")
        self.stream.flush()

    def remember_clock(self, message):
        self.clock = stamp_ns(message.clock)


def wait_for(node, predicate, seconds, message):
    end = time.monotonic() + seconds
    while not predicate():
        if time.monotonic() >= end:
            raise RuntimeError(message)
        rclpy.spin_once(node, timeout_sec=0.05)


def await_future(node, future, seconds, description):
    wait_for(node, future.done, seconds, description + " timed out")
    return future.result()


def pose(x, y, yaw=0.0):
    target = PoseStamped()
    target.header.frame_id = "map"
    target.pose.position.x, target.pose.position.y = float(x), float(y)
    target.pose.orientation.z = math.sin(yaw / 2)
    target.pose.orientation.w = math.cos(yaw / 2)
    return target


def occupancy_checker(grid):
    if grid.header.frame_id != "map":
        raise RuntimeError("global costmap must use the map frame")
    info = grid.info
    origin = info.origin
    if abs(origin.orientation.w) < 0.9999:
        raise RuntimeError("rotated costmap grids require coordinate conversion")
    cell = info.resolution

    def open_at(x, y, radius):
        c0 = math.floor((x - radius - origin.position.x) / cell)
        c1 = math.floor((x + radius - origin.position.x) / cell)
        r0 = math.floor((y - radius - origin.position.y) / cell)
        r1 = math.floor((y + radius - origin.position.y) / cell)
        if c0 < 0 or r0 < 0 or c1 >= info.width or r1 >= info.height:
            return False
        for row in range(r0, r1 + 1):
            for column in range(c0, c1 + 1):
                status = grid.data[row * info.width + column]
                # 99 is Nav2's already-inflated inscribed cost. Sweep against
                # lethal and unknown cells to avoid counting that inflation twice.
                if 0 <= status < 100:
                    continue
                left = origin.position.x + column * cell
                bottom = origin.position.y + row * cell
                dx = max(left - x, 0, x - left - cell)
                dy = max(bottom - y, 0, y - bottom - cell)
                if math.hypot(dx, dy) <= radius:
                    return False
        return True
    return open_at


def stopped(node, evidence):
    dwell = StopDwell()

    def settled():
        sample = evidence.odom
        if sample is None or evidence.clock is None:
            return False
        twist = sample.twist.twist
        return dwell.observe(evidence.clock, stamp_ns(sample.header.stamp),
                             math.hypot(twist.linear.x, twist.linear.y), twist.angular.z)

    try:
        wait_for(node, settled, 12.0, "fresh odometry did not verify sustained low speed")
    except RuntimeError:
        odom = evidence.odom
        command = evidence.last_command
        evidence.log("stop_unverified",
                     odometry_stamp_ns=str(stamp_ns(odom.header.stamp)) if odom else None,
                     odometry_velocity=[odom.twist.twist.linear.x, odom.twist.twist.angular.z]
                     if odom else None,
                     final_cmd_velocity=[command.linear.x, command.angular.z] if command else None)
        raise
    evidence.log("odometry_stopped", dwell_s=0.75)


def exercise(node, evidence, layout, gate_id, cancel_after):
    gates = [Gate(**data) for data in layout["gates"]]
    gate = next((item for item in gates if item.gate_id == gate_id), None)
    if gate is None or layout["frame_id"] != "map":
        raise RuntimeError("unknown map-frame gate")
    def control_chain():
        writers = node.get_publishers_info_by_topic("/luna/cmd_vel")
        return (len(writers) == 1 and writers[0].node_name == "collision_monitor"
                and node.count_subscribers("/luna/cmd_vel") == 1)

    wait_for(node, control_chain, 8.0, "final Twist source or simulator subscriber is ambiguous")
    evidence.log("final_twist_source_verified", topic="/luna/cmd_vel", node="collision_monitor")
    drive = ActionClient(node, FollowPath, "/follow_path")
    plan = ActionClient(node, ComputePathThroughPoses, "/compute_path_through_poses")
    for name, client in (("/compute_path_through_poses", plan), ("/follow_path", drive)):
        wait_for(node, client.server_is_ready, 8.0, f"{name} action server unavailable")
    node.create_subscription(Clock, "/clock", evidence.remember_clock, qos_profile_sensor_data)
    node.create_subscription(Twist, "/luna/cmd_vel",
                             lambda message: setattr(evidence, "last_command", message),
                             qos_profile_sensor_data)
    node.create_subscription(Odometry, "/chassis/odom",
                             lambda message: setattr(evidence, "odom", message),
                             qos_profile_sensor_data)
    node.create_subscription(OccupancyGrid, "/global_costmap/costmap",
                             lambda message: setattr(evidence, "grid", message),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def fresh_map():
        return evidence.grid is not None and evidence.clock is not None and (
            0 <= evidence.clock - stamp_ns(evidence.grid.header.stamp) <= 2_000_000_000)

    y = (gate.lower_y + gate.upper_y) / 2
    terminal = layout["terminal_xy"]
    goal = ComputePathThroughPoses.Goal()
    goal.planner_id = "GridBased"
    goal.goals = [pose(gate.entry_x - 1.2, y), pose(gate.exit_x + 1.1, y), pose(*terminal)]
    evidence.log("planning_requested", gate_id=gate_id, route_version=1, path_seq=1)
    compute = await_future(node, plan.send_goal_async(goal), 8.0, "planning goal response")
    if not compute.accepted:
        raise RuntimeError("Nav2 rejected ComputePathThroughPoses")
    planned = await_future(node, compute.get_result_async(), 6.0, "planning result")
    if planned.status != GoalStatus.STATUS_SUCCEEDED:
        raise RuntimeError(f"Nav2 planning ended with status {planned.status}")
    path = planned.result.path
    if path.header.frame_id != "map" or any(p.header.frame_id != "map" for p in path.poses):
        raise RuntimeError("Nav2 path frames are inconsistent")
    points = [(item.pose.position.x, item.pose.position.y) for item in path.poses]
    wait_for(node, fresh_map, 6.0, "fresh global costmap unavailable")
    params = yaml.safe_load(Path("/luna_nav/params.yaml").read_text())
    costmap = params["global_costmap"]["global_costmap"]["ros__parameters"]
    footprint = json.loads(costmap["footprint"])
    padding = float(costmap.get("footprint_padding", 0.01))
    radius = max(math.hypot(x, y) for x, y in footprint) + math.sqrt(2.0) * padding
    evidence.log("path_proposed", frame_id="map", route_version=1, path_seq=1, points=points)
    validate_corridor(points, gate, tuple(other for other in gates if other != gate),
                      radius, occupancy_checker(evidence.grid))
    evidence.log("path_validated", gate_id=gate_id, route_version=1, path_seq=1, points=points)
    command = FollowPath.Goal()
    command.path = path
    command.controller_id = "FollowPath"
    motion = await_future(node, drive.send_goal_async(command), 8.0, "FollowPath response")
    if not motion.accepted:
        raise RuntimeError("Nav2 rejected FollowPath")
    evidence.log("nav2_accepted", route_version=1, path_seq=1,
                 goal_id=bytes(motion.goal_id.uuid).hex())
    end = motion.get_result_async()
    if cancel_after is not None:
        deadline = time.monotonic() + cancel_after
        wait_for(node, lambda: end.done() or time.monotonic() >= deadline,
                 cancel_after + 1.0, "cancel scheduler")
        if end.done():
            raise RuntimeError("controller ended before the requested cancel test")
        evidence.log("cancel_requested")
        canceled = await_future(node, motion.cancel_goal_async(), 6.0, "FollowPath cancel")
        if not canceled.goals_canceling:
            raise RuntimeError("Nav2 did not acknowledge cancellation")
        evidence.log("cancel_acknowledged")
    result = await_future(node, end, 75.0 if cancel_after is None else 8.0, "FollowPath result")
    evidence.log("nav2_result", goal_status=result.status)
    expected = GoalStatus.STATUS_SUCCEEDED if cancel_after is None else GoalStatus.STATUS_CANCELED
    stopped(node, evidence)
    if result.status != expected:
        raise RuntimeError(f"unexpected FollowPath status {result.status}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("gate")
    parser.add_argument("evidence", type=Path)
    parser.add_argument("--cancel-after", type=float)
    args = parser.parse_args()
    layout = json.loads(Path("/luna_nav/public_scene_manifest.json").read_text())
    rclpy.init()
    node = rclpy.create_node("luna_controller_test")
    try:
        with args.evidence.open("x") as stream:
            evidence = Evidence(stream)
            try:
                exercise(node, evidence, layout, args.gate, args.cancel_after)
            except (ValueError, RuntimeError) as exc:
                evidence.log("test_failed", reason=str(exc))
                raise SystemExit(str(exc)) from exc
    finally:
        node.destroy_node()
        rclpy.shutdown()
