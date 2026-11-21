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
from nav2_msgs.action import ComputePathThroughPoses, FollowPath, Spin
from nav2_msgs.srv import ClearEntireCostmap
import rclpy
from rclpy.action import ActionClient
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from navigation.adaptation import (Blockage, ROUTE_STOPPED, ROUTE_UNAVAILABLE,
                                   RouteBlocked, RouteUnavailable, alignment_turn, blocked_ahead, path_ahead)
from navigation.candidates import Gate, ValidationError, costmap_footprint_radius, validate_corridor
from navigation.contracts import StopDwell
from navigation.live_audit import review_is_current
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


def occupancy_checker(grid, frame="map"):
    if grid.header.frame_id != frame:
        raise RuntimeError(f"expected a {frame}-frame costmap")
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


def stopped(node, evidence, event="odometry_stopped"):
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
    evidence.log(event, dwell_s=0.75)


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
             episode=None, replan=False, cancel_after_gate=False, adaptive=False, route_version=1, refresh_local_map=False,
             visual_review=None):
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

    scan_ns = -1
    scan_wall = None

    def remember_scan(message):
        nonlocal scan_ns, scan_wall
        stamp = stamp_ns(message.header.stamp)
        if stamp > scan_ns and len(message.ranges) > 0:
            scan_ns, scan_wall = stamp, time.monotonic()

    def scan_fresh():
        return (scan_wall is not None and time.monotonic() - scan_wall <= 1.0
                and evidence.clock is not None and -50_000_000 <= evidence.clock - scan_ns <= 650_000_000)

    if adaptive:
        node.create_subscription(LaserScan, "/scan", remember_scan, qos_profile_sensor_data)

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
                try:
                    transform = tf_buffer.lookup_transform("map", "base_link", Time())
                except TransformException:
                    return None
                age = evidence.clock - stamp_ns(transform.header.stamp)
                if not 0 <= age <= 250_000_000:
                    return None
            return transform.transform

        def check_live():
            try:
                wait_for(node, locate, 2.8 if adaptive else 1.8,
                         "fresh base transform unavailable for model intent")
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

    local_grid = None
    local_updates = set()

    def remember_local_grid(grid):
        nonlocal local_grid
        local_grid = grid
        local_updates.add(stamp_ns(grid.header.stamp))

    def refreshed_local():
        return (local_grid is not None and evidence.clock is not None and scan_fresh()
                and scan_ns > refresh_after and sum(stamp > refresh_after for stamp in local_updates) >= 2
                and 0 <= evidence.clock - stamp_ns(local_grid.header.stamp) <= 1_000_000_000)

    if refresh_local_map:
        node.create_subscription(OccupancyGrid, "/local_costmap/costmap", remember_local_grid,
                                 QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        wait_for(node, scan_fresh, 1.5, "fresh scan required to refresh the rolling costmap")
        service = node.create_client(ClearEntireCostmap, "/local_costmap/clear_entirely_local_costmap")
        wait_for(node, service.service_is_ready, 2.0, "rolling costmap refresh service unavailable")
        await_future(node, service.call_async(ClearEntireCostmap.Request()), 2.0, "rolling costmap refresh")
        refresh_after = evidence.clock
        wait_for(node, refreshed_local, 4.0, "rolling costmap did not receive two post-refresh updates")
        evidence.log("local_costmap_refreshed", refresh_after_sim_ns=str(refresh_after),
                     local_stamp_ns=str(stamp_ns(local_grid.header.stamp)), latest_scan_ns=str(scan_ns))

    y = (gate.lower_y + gate.upper_y) / 2
    terminal = layout["terminal_xy"]
    params = yaml.safe_load(Path("/luna_nav/params.yaml").read_text())
    radius = costmap_footprint_radius(params)
    forbidden = tuple(other for other in gates if other != gate)
    session = PathSession(route_version)
    if adaptive:
        runtime_budget = Path("/luna_run/adaptive-runtime.yaml")
        budget_file = runtime_budget if runtime_budget.exists() else Path("/challenge/adaptive.yaml")
        budget = yaml.safe_load(budget_file.read_text())
        values = (budget[name] for name in ("path_lookahead_m", "blocked_dwell_sim_s",
                                            "global_map_watchdog_wall_s", "recovery_local_lookahead_m",
                                            "vision_review_lookahead_m"))
        lookahead, block_dwell, watchdog, local_lookahead, review_lookahead = map(float, values)
        if not all(math.isfinite(v) and v > 0 for v in (lookahead, block_dwell, watchdog, local_lookahead, review_lookahead)):
            raise ValueError("adaptive route budgets must be finite and positive")
        core_radius = radius * float(budget.get("vision_review_core_radius_ratio", 1.0))
        if not 0 < core_radius <= radius:
            raise ValueError("invalid vision review radius")
        blockage = Blockage(block_dwell)
    last_motion = None
    adopted_review = None
    rejected_review = None
    active_points = None
    aligned = False
    last_map_wall = time.monotonic()

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
        if planned.status == GoalStatus.STATUS_ABORTED and adaptive:
            raise RouteUnavailable(f"planner aborted for gate {gate_id}")
        if planned.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"Nav2 planning ended with status {planned.status}")
        return planned.result.path

    def check_refreshed_corridor(points):
        now_base = locate()
        if now_base is None or not refreshed_local():
            raise RuntimeError("cannot check recovered path against the current rolling costmap")
        try:
            map_to_local = tf_buffer.lookup_transform("odom", "map", Time(nanoseconds=evidence.clock - 200_000_000))
        except TransformException as error:
            raise RuntimeError("missing map-to-odom transform for recovered path") from error
        t, q = map_to_local.transform.translation, map_to_local.transform.rotation
        heading = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        cosine, sine = math.cos(heading), math.sin(heading)
        free = occupancy_checker(local_grid, "odom")
        position = (now_base.translation.x, now_base.translation.y)
        if any(not free(t.x + cosine*x - sine*y0, t.y + sine*x + cosine*y0, radius + 0.025)
               for x, y0 in path_ahead(points, position, local_lookahead)):
            raise RouteUnavailable("fresh rolling costmap blocks the recovered path")

    def submit(path_seq, path, handoff=False, tick=None):
        nonlocal last_motion, active_points
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
        try:
            validate_corridor(points, gate, forbidden, radius, free)
        except ValidationError as error:
            if adaptive and not handoff:
                raise RouteUnavailable(str(error)) from error
            raise
        if refresh_local_map:
            check_refreshed_corridor(points)
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
            if not aligned:
                check_live()
            evidence.log("intent_committed", gate_id=gate_id, episode_id=episode,
                         aligned_before_follow=aligned)
        if adaptive:
            wait_for(node, scan_fresh, 1.0, "fresh local obstacle scan unavailable")
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
        active_points = points
        evidence.log("nav2_accepted", goal_id=bytes(motion.goal_id.uuid).hex(), **identity)
        return motion

    def cancel_active(motion, reason):
        evidence.log("cancel_requested", path_seq=session.active, reason=reason)
        reply = await_future(node, motion.cancel_goal_async(), 6.0, "FollowPath cancel")
        expected_id = bytes(motion.goal_id.uuid)
        if not any(bytes(goal.goal_id.uuid) == expected_id for goal in reply.goals_canceling):
            raise RuntimeError("Nav2 did not acknowledge cancellation of the active goal")
        evidence.log("cancel_acknowledged", path_seq=session.active, reason=reason)

    def route_health():
        nonlocal last_map_wall, adopted_review, rejected_review
        if not scan_fresh():
            raise RuntimeError("live route lost its local obstacle scan")
        if time.monotonic() - last_map_wall > watchdog:
            raise RuntimeError("live route lost its global costmap")
        if not fresh_map():
            return
        current = locate()
        if current is None:
            return
        stamp = stamp_ns(evidence.grid.header.stamp)
        if blockage.stamp is not None and stamp <= blockage.stamp:
            return
        last_map_wall = time.monotonic()
        position = (current.translation.x, current.translation.y)
        free = occupancy_checker(evidence.grid)
        standard = blocked_ahead(active_points, position, lookahead, radius + 0.025, free)
        extra = None
        review_id = None
        if visual_review and visual_review.exists():
            review = json.loads(visual_review.read_text())
            q = current.rotation
            yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
            if review_is_current(review, episode, bytes(last_motion.goal_id.uuid).hex(), gate_id,
                                 evidence.clock, time.monotonic_ns(), position, yaw):
                review_id = review["observation_id"]
                extra = blocked_ahead(active_points, position, review_lookahead, core_radius, free)
                if review_id != adopted_review:
                    adopted_review = review_id
                    evidence.log("visual_review_adopted", observation_id=review_id,
                                 standard_lookahead_m=lookahead, review_lookahead_m=review_lookahead,
                                 standard_blocked=standard is not None, review_blocked=extra is not None, review_radius_m=core_radius)
            elif review["observation_id"] != rejected_review:
                rejected_review = review["observation_id"]
                evidence.log("visual_review_rejected", observation_id=rejected_review,
                             reason="inactive_goal_or_observation_outside_envelope",
                             base_drift_m=math.dist(position, review["base_xy"]),
                             yaw_drift_rad=abs(math.remainder(yaw - review["base_yaw"], math.tau)))
        blocked = standard or extra
        if blockage.observe(stamp, blocked is not None):
            evidence.log("route_blocked", gate_id=gate_id, source_grid="global_costmap",
                         costmap_stamp_ns=str(stamp), blocking_point=blocked,
                         active_path_seq=session.active, base_xy=position,
                         standard_blocked=standard is not None, visual_review=review_id)
            raise RouteBlocked("persistent obstruction in the active path envelope")

    def local_xy():
        sample = evidence.odom
        if sample is None or evidence.clock is None:
            return None
        if not 0 <= evidence.clock - stamp_ns(sample.header.stamp) <= 250_000_000:
            return None
        position = sample.pose.pose.position
        return position.x, position.y

    def align_alternate(path):
        check_live()
        wait_for(node, fresh_map, 2.0, "alignment requires a fresh global costmap")
        wait_for(node, scan_fresh, 1.0, "alignment requires a fresh local scan")
        base = locate()
        if base is None:
            raise RuntimeError("alignment lost its map pose")
        x, y0 = base.translation.x, base.translation.y
        q = base.rotation
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        points = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
        if path.header.frame_id != "map":
            raise RuntimeError("alignment requires a map-frame path")
        free = occupancy_checker(evidence.grid)
        try:
            validate_corridor(points, gate, forbidden, radius, free)
        except ValidationError as error:
            raise RouteUnavailable(str(error)) from error
        turn = alignment_turn(points, (x, y0), yaw)
        if abs(turn) < 0.40:
            return False
        if not free(x, y0, radius + 0.025):
            raise RouteUnavailable("alternate path cannot be aligned within a clear footprint")
        wait_for(node, local_xy, 1.0, "alignment requires current odometry")
        start_local = local_xy()
        spin = ActionClient(node, Spin, "/spin")
        wait_for(node, spin.server_is_ready, 2.0, "bounded alignment server unavailable")
        goal = Spin.Goal()
        goal.target_yaw = turn
        goal.time_allowance.sec = 8
        evidence.log("alignment_requested", route_version=route_version, turn_rad=turn, origin_xy=[x, y0])
        handle = await_future(node, spin.send_goal_async(goal), 2.0, "alignment acceptance")
        if not handle.accepted:
            raise RuntimeError("bounded alignment rejected")
        ended = handle.get_result_async()

        def safe_spin():
            if evidence.clock_rewound or not scan_fresh():
                raise RuntimeError("alignment lost current obstacle evidence")

        try:
            result = await_future(node, ended, 10.0, "alignment result", safe_spin)
        except RuntimeError:
            if not ended.done():
                await_future(node, handle.cancel_goal_async(), 3.0, "alignment cancel")
                await_future(node, ended, 5.0, "alignment cancellation result")
            stopped(node, evidence, event="alignment_stopped")
            raise
        stopped(node, evidence, event="alignment_stopped")
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"bounded alignment ended with status {result.status}")
        current, current_local = locate(), local_xy()
        if current is None or current_local is None:
            raise RuntimeError("bounded alignment cannot verify its final pose")
        drift = math.dist(start_local, current_local)
        if current.translation.x >= gate.entry_x - 0.5 or drift > 0.15:
            raise RuntimeError("bounded alignment left the authorized staging area")
        evidence.log("alignment_completed", route_version=route_version, turn_rad=turn,
                     odometry_translation_m=drift, current_map_xy=[current.translation.x, current.translation.y])
        return True

    def follow():
        nonlocal last_map_wall, aligned
        initial = session.request()
        path = build_plan(initial)
        if adaptive and route_version > 1:
            aligned = align_alternate(path)
            if aligned:
                initial = session.request()
                path = build_plan(initial)
        motion = submit(initial, path)
        end = motion.get_result_async()
        last_map_wall = time.monotonic()
        corridor = corridor_watch(evidence, locate, gate, radius) if replan or adaptive else None

        def adaptive_tick():
            corridor()
            route_health()

        tick = adaptive_tick if adaptive else corridor
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
        if adaptive and result.status == GoalStatus.STATUS_ABORTED:
            evidence.log("route_blocked", gate_id=gate_id, source_grid="controller_abort")
            return ROUTE_STOPPED
        if result.status != expected:
            raise RuntimeError(f"unexpected FollowPath status {result.status}")

    try:
        return follow()
    except RouteBlocked:
        cancel_active(last_motion, "route_blocked")
        result = await_future(node, last_motion.get_result_async(), 8.0, "blocked action result")
        evidence.log("nav2_result", goal_status=result.status, path_seq=session.active)
        session.finish(session.active)
        if result.status != GoalStatus.STATUS_CANCELED:
            raise RuntimeError("blocked goal did not reach canceled status")
        stopped(node, evidence)
        return ROUTE_STOPPED
    except RouteUnavailable as error:
        if last_motion is not None:
            raise
        evidence.log("route_unavailable", gate_id=gate_id, reason=str(error))
        return ROUTE_UNAVAILABLE
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
    parser.add_argument("--adaptive", action="store_true")
    parser.add_argument("--route-version", type=int, default=1)
    parser.add_argument("--refresh-local-map", action="store_true")
    parser.add_argument("--visual-review", type=Path)
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
                status = exercise(node, evidence, layout, args.gate, args.cancel_after, observation,
                                  args.episode, args.replan, args.cancel_after_gate, args.adaptive, args.route_version, args.refresh_local_map, args.visual_review)
                if status is not None:
                    raise SystemExit(status)
            except (ValueError, RuntimeError) as exc:
                evidence.log("test_failed", reason=str(exc))
                raise SystemExit(str(exc)) from exc
    finally:
        node.destroy_node()
        rclpy.shutdown()
