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
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from navigation.candidates import Gate, costmap_footprint_radius, validate_corridor
from navigation.contracts import StopDwell
from navigation.live_intent import verify_fresh
from navigation.path_session import PathSession, validate_handoff


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
        self.telemetry_ns = {}
        self.trace_motion = False
        self.source = "controller-test"
        self.clock_rewound = False

    def log(self, event, **payload):
        self.sequence += 1
        record = {"event": event, "event_seq": self.sequence, "source": self.source,
                  "sim_stamp_ns": str(self.clock) if self.clock is not None else None,
                  "host_mono_ns": str(time.monotonic_ns()), **payload}
        self.stream.write(json.dumps(record, allow_nan=False) + "\n")
        self.stream.flush()

    def remember_clock(self, message):
        stamp = stamp_ns(message.clock)
        if self.clock is not None and stamp < self.clock:
            self.clock_rewound = True
        self.clock = stamp


    def remember_twist(self, message, topic):
        if topic == "/luna/cmd_vel":
            self.last_command = message
        self.trace_twist("twist_command", topic, message)

    def remember_odom(self, message):
        self.odom = message
        self.trace_twist("odometry_twist", "/chassis/odom", message.twist.twist,
                         stamp_ns=str(stamp_ns(message.header.stamp)))

    def trace_twist(self, event, topic, twist, **metadata):
        now = time.monotonic_ns()
        if not self.trace_motion or now - self.telemetry_ns.get(topic, 0) < 100_000_000:
            return
        self.telemetry_ns[topic] = now
        self.log(event, topic=topic, linear_xy=[twist.linear.x, twist.linear.y],
                 angular_z=twist.angular.z, **metadata)


def wait_for(node, predicate, seconds, message, tick=None):
    end = time.monotonic() + seconds
    while True:
        if tick is not None:
            tick()
        if predicate():
            return
        if time.monotonic() >= end:
            raise RuntimeError(message)
        rclpy.spin_once(node, timeout_sec=0.05)


def await_future(node, future, seconds, description, tick=None):
    wait_for(node, future.done, seconds, description + " timed out", tick)
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


def past_gate(locate, gate, radius):
    transform = locate()
    return transform is not None and transform.translation.x > gate.exit_x + radius


def corridor_watch(evidence, locate, gate, radius):
    last_pose = time.monotonic()

    def check():
        nonlocal last_pose
        if evidence.clock_rewound:
            raise ValueError("simulation clock rewound during FollowPath")
        current = locate()
        if current is None:
            if time.monotonic() - last_pose > 1.0:
                raise RuntimeError("navigation lost current corridor pose")
            return
        last_pose = time.monotonic()
        t = current.translation
        if (gate.entry_x - radius <= t.x <= gate.exit_x + radius
                and not gate.lower_y + radius <= t.y <= gate.upper_y - radius):
            raise ValueError("vehicle footprint exited the committed gate corridor")

    return check


def replace_during_motion(node, evidence, session, motion, end, build_plan, submit, tick):
    def moving():
        if evidence.odom is None or evidence.clock is None:
            return False
        age = evidence.clock - stamp_ns(evidence.odom.header.stamp)
        twist = evidence.odom.twist.twist
        return 0 <= age <= 250_000_000 and math.hypot(twist.linear.x, twist.linear.y) > 0.12

    def wait_moving(previous):
        wait_for(node, lambda: previous.done() or moving(), 10.0,
                 "could not start a replacement while driving", tick)
        if previous.done():
            raise RuntimeError("FollowPath ended before the scheduled replacement")

    for extension in (1.18, 1.24):
        progress = time.monotonic() + 2.0
        wait_for(node, lambda: end.done() or time.monotonic() >= progress, 3.0,
                 "replacement scheduler", tick)
        wait_moving(end)
        previous_seq, previous_end = session.active, end
        if previous_seq == 1:
            held_seq = session.request()
            held_plan = build_plan(held_seq, 1.14, tick)
            evidence.log("planning_held", path_seq=held_seq, route_version=session.route_version)
        path_seq = session.request()
        path = build_plan(path_seq, extension, tick)
        if previous_end.done():
            raise RuntimeError("active path ended before replacement could be submitted")
        motion = submit(path_seq, path, handoff=True, tick=tick)
        end = motion.get_result_async()
        if previous_seq == 1:
            # Inject a late delivery of a real planning result without forwarding it to Nav2.
            submit(held_seq, held_plan)
        result = await_future(node, previous_end, 3.0, "superseded FollowPath result", tick)
        if session.finish(previous_seq):
            raise RuntimeError("superseded goal incorrectly changed the session outcome")
        evidence.log("nav2_result_ignored", path_seq=previous_seq, active_path_seq=session.active,
                     goal_status=result.status, reason="superseded")
    return motion, end


def exercise(node, evidence, layout, gate_id, cancel_after, live_observation=None,
             episode=None, replan=False, cancel_after_gate=False):
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
    for topic in ("/cmd_vel_nav", "/cmd_vel", "/luna/cmd_vel"):
        node.create_subscription(Twist, topic,
                                 lambda message, topic=topic: evidence.remember_twist(message, topic),
                                 qos_profile_sensor_data)
    node.create_subscription(Odometry, "/chassis/odom", evidence.remember_odom,
                             qos_profile_sensor_data)
    node.create_subscription(OccupancyGrid, "/global_costmap/costmap",
                             lambda message: setattr(evidence, "grid", message),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    if live_observation is not None or replan:
        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, node, qos=QoSProfile(
            depth=100, reliability=qos_profile_sensor_data.reliability))
        last_tf_error = None

        def locate():
            nonlocal last_tf_error
            if evidence.clock is None or evidence.clock < 200_000_000 or tf_listener is None:
                return None
            try:
                transform = tf_buffer.lookup_transform(
                    "map", "base_link", Time(nanoseconds=evidence.clock - 200_000_000))
            except TransformException as error:
                last_tf_error = str(error)
                return None
            return transform.transform

        def check_live():
            try:
                wait_for(node, locate, 1.8, "fresh base transform unavailable for model intent")
            except RuntimeError as error:
                raise RuntimeError(f"{error}: {last_tf_error}") from error
            transform = locate()
            t, q = transform.translation, transform.rotation
            if evidence.clock_rewound or t.x >= gate.entry_x - 0.5:
                raise ValueError("clock rewound or base is too close to the requested gate")
            verify_fresh(live_observation, episode, evidence.clock, time.monotonic_ns(),
                         (t.x, t.y, t.z), (q.x, q.y, q.z, q.w))

        if live_observation is not None:
            check_live()
            evidence.log("intent_preflight", gate_id=gate_id, episode_id=episode)

    def fresh_map():
        return evidence.grid is not None and evidence.clock is not None and (
            0 <= evidence.clock - stamp_ns(evidence.grid.header.stamp) <= 2_000_000_000)

    y = (gate.lower_y + gate.upper_y) / 2
    terminal = layout["terminal_xy"]
    params = yaml.safe_load(Path("/luna_nav/params.yaml").read_text())
    radius = costmap_footprint_radius(params)
    forbidden = tuple(other for other in gates if other != gate)
    session = PathSession()
    last_motion = None

    def build_plan(path_seq, exit_extension=1.1, tick=None):
        goal = ComputePathThroughPoses.Goal()
        goal.planner_id = "GridBased"
        goal.goals = [pose(gate.entry_x - 1.2, y), pose(gate.exit_x + exit_extension, y), pose(*terminal)]
        evidence.log("planning_requested", gate_id=gate_id, route_version=session.route_version,
                     path_seq=path_seq, waypoints=[[p.pose.position.x, p.pose.position.y] for p in goal.goals])
        compute = await_future(node, plan.send_goal_async(goal), 8.0, "planning goal response", tick)
        if not compute.accepted:
            raise RuntimeError("Nav2 rejected ComputePathThroughPoses")
        planned = await_future(node, compute.get_result_async(), 6.0, "planning result", tick)
        if planned.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"Nav2 planning ended with status {planned.status}")
        return planned.result.path

    def submit(path_seq, path, handoff=False, tick=None):
        nonlocal last_motion
        if not session.may_submit(path_seq):
            evidence.log("planning_discarded", path_seq=path_seq, route_version=session.route_version,
                         latest_path_seq=session.latest, reason="superseded")
            return None
        if path.header.frame_id != "map" or any(p.header.frame_id != "map" for p in path.poses):
            raise RuntimeError("Nav2 path frames are inconsistent")
        points = [(item.pose.position.x, item.pose.position.y) for item in path.poses]
        wait_for(node, fresh_map, 6.0, "fresh global costmap unavailable", tick)
        free = occupancy_checker(evidence.grid)
        identity = {"route_version": session.route_version, "path_seq": path_seq}
        evidence.log("path_proposed", frame_id="map", points=points, **identity)
        validate_corridor(points, gate, forbidden, radius, free)
        if handoff:
            transform = locate()
            if transform is None or transform.translation.x >= gate.entry_x - 0.5:
                raise ValueError("replacement requires a current pose before the committed gate")
            t, q = transform.translation, transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
            join = validate_handoff(points, (t.x, t.y), yaw)
            samples = max(1, math.ceil(math.dist((t.x, t.y), join) / 0.05))
            for i in range(samples + 1):
                x, y0 = (a + i / samples * (b-a) for a, b in zip((t.x, t.y), join))
                if not free(x, y0, radius + 0.025):
                    raise ValueError("replacement handoff has insufficient footprint clearance")
            evidence.log("handoff_validated", base_xy=[t.x, t.y], join_xy=join, **identity)
        evidence.log("path_validated", gate_id=gate_id, points=points, **identity)
        if live_observation is not None and not handoff:
            check_live()
            evidence.log("intent_committed", gate_id=gate_id, episode_id=episode)
        command = FollowPath.Goal()
        command.path = path
        command.controller_id = "FollowPath"
        evidence.trace_motion = True
        evidence.log("follow_path_requested", **identity)
        motion = await_future(node, drive.send_goal_async(command), 8.0, "FollowPath response", tick)
        if motion.accepted:
            last_motion = motion
        if not motion.accepted or not session.accept(path_seq):
            raise RuntimeError("Nav2 rejected FollowPath or accepted a superseded path")
        evidence.log("nav2_accepted", goal_id=bytes(motion.goal_id.uuid).hex(), **identity)
        return motion

    def cancel_active(motion, reason):
        evidence.log("cancel_requested", path_seq=session.active, reason=reason)
        reply = await_future(node, motion.cancel_goal_async(), 6.0, "FollowPath cancel")
        expected_id = bytes(motion.goal_id.uuid)
        if not any(bytes(goal.goal_id.uuid) == expected_id for goal in reply.goals_canceling):
            raise RuntimeError("Nav2 did not acknowledge cancellation of the active goal")
        evidence.log("cancel_acknowledged", path_seq=session.active, reason=reason)

    def follow():
        initial = session.request()
        motion = submit(initial, build_plan(initial))
        end = motion.get_result_async()
        tick = corridor_watch(evidence, locate, gate, radius) if replan else None
        if replan:
            motion, end = replace_during_motion(
                node, evidence, session, motion, end, build_plan, submit, tick)
        if cancel_after_gate:
            wait_for(node, lambda: end.done() or past_gate(locate, gate, radius), 55.0,
                     "the vehicle did not pass the chosen corridor before the cancel deadline", tick)
            if end.done():
                raise RuntimeError("controller ended before the requested gate exit cancel")
        if cancel_after is not None:
            deadline = time.monotonic() + cancel_after
            wait_for(node, lambda: end.done() or time.monotonic() >= deadline,
                     cancel_after + 1.0, "cancel scheduler")
            if end.done():
                raise RuntimeError("controller ended before the requested cancel test")
        should_cancel = cancel_after is not None or cancel_after_gate
        if should_cancel:
            cancel_active(motion, "controller-test")
        result = await_future(node, end, 8.0 if should_cancel else 75.0, "FollowPath result",
                              None if should_cancel else tick)
        final_path = session.active
        if not session.finish(final_path):
            raise RuntimeError("received terminal result for inactive goal")
        evidence.log("nav2_result", goal_status=result.status, path_seq=final_path)
        expected = GoalStatus.STATUS_CANCELED if should_cancel else GoalStatus.STATUS_SUCCEEDED
        stopped(node, evidence)
        if result.status != expected:
            raise RuntimeError(f"unexpected FollowPath status {result.status}")

    try:
        follow()
    except (ValueError, RuntimeError):
        if last_motion is not None and not session.terminated:
            try:
                cancel_active(last_motion, "execution_failure")
                result = await_future(node, last_motion.get_result_async(), 8.0, "recovery result")
                evidence.log("recovery_action_result", goal_status=result.status, path_seq=session.active)
                stopped(node, evidence)
            except RuntimeError as recovery:
                evidence.log("recovery_unverified", reason=str(recovery))
        raise


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("gate")
    parser.add_argument("evidence", type=Path)
    parser.add_argument("--cancel-after", type=float)
    parser.add_argument("--live-observation", type=Path)
    parser.add_argument("--episode")
    parser.add_argument("--replan", action="store_true")
    parser.add_argument("--cancel-after-gate", action="store_true")
    args = parser.parse_args()
    layout = json.loads(Path("/luna_nav/public_scene_manifest.json").read_text())
    rclpy.init()
    node = rclpy.create_node("luna_controller_test")
    try:
        with args.evidence.open("x") as stream:
            evidence = Evidence(stream)
            try:
                observation = None
                if args.live_observation:
                    observation = json.loads(args.live_observation.read_text())
                    evidence.source = "luna-live"
                exercise(node, evidence, layout, args.gate, args.cancel_after, observation,
                         args.episode, args.replan, args.cancel_after_gate)
            except (ValueError, RuntimeError) as exc:
                evidence.log("test_failed", reason=str(exc))
                raise SystemExit(str(exc)) from exc
    finally:
        node.destroy_node()
        rclpy.shutdown()
