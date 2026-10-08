"""One-goal Nav2 integration test. Not an Agent or a patrol implementation."""

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
import yaml
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--site", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=300)
    parser.add_argument("--cancel-after", type=float)
    args = parser.parse_args()
    sites = yaml.safe_load(Path("/lab/config/inspection_sites.yaml").read_text())
    target = sites["sites"][args.site]
    stream = args.output.open("w")

    def log(event, **fields):
        row = {"event": event, "wall_time": time.time(), **fields}
        stream.write(json.dumps(row) + "\n")
        stream.flush()

    rclpy.init()
    node = Node(
        "rosclaw_single_goal_probe",
        parameter_overrides=[Parameter("use_sim_time", value=True)],
    )
    client = ActionClient(node, NavigateToPose, "/navigate_to_pose")
    if not client.wait_for_server(timeout_sec=60):
        raise RuntimeError("NavigateToPose server unavailable")
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"
    # Zero timestamp asks Nav2 for the latest transform; no fabricated sim time.
    goal.pose.pose.position.x, goal.pose.pose.position.y = (
        float(target["x"]),
        float(target["y"]),
    )
    goal.pose.pose.orientation.z = math.sin(target["yaw"] / 2)
    goal.pose.pose.orientation.w = math.cos(target["yaw"] / 2)
    log("goal_request", site=args.site, target=target)
    future = client.send_goal_async(
        goal,
        feedback_callback=lambda msg: log(
            "feedback",
            distance_remaining=msg.feedback.distance_remaining,
            recoveries=msg.feedback.number_of_recoveries,
        ),
    )
    rclpy.spin_until_future_complete(node, future, timeout_sec=15)
    if not future.done() or not future.result().accepted:
        raise RuntimeError("goal not accepted")
    handle = future.result()
    terminal = False
    try:
        log("goal_accepted", goal_uuid=[int(value) for value in handle.goal_id.uuid])
        result = handle.get_result_async()
        started = time.monotonic()
        cancelled = False
        while not result.done():
            elapsed = time.monotonic() - started
            if not cancelled and (
                elapsed > args.timeout
                or args.cancel_after is not None
                and elapsed >= args.cancel_after
            ):
                cancel = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(node, cancel, timeout_sec=5)
                log(
                    "cancel_requested",
                    acknowledged=cancel.done()
                    and bool(cancel.result().goals_canceling),
                )
                cancelled = True
                if not cancel.done() or not cancel.result().goals_canceling:
                    raise RuntimeError("cancellation was not acknowledged")
            if (
                cancelled
                and elapsed > min(args.timeout, args.cancel_after or args.timeout) + 15
            ):
                raise RuntimeError("canceled goal did not reach terminal state")
            rclpy.spin_once(node, timeout_sec=0.1)
        value = result.result()
        log(
            "goal_result",
            status=value.status,
            error_code=value.result.error_code,
            error_msg=value.result.error_msg,
        )
        terminal = True
    finally:
        if not terminal:
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel, timeout_sec=5)
            log(
                "exception_cleanup_cancel",
                acknowledged=cancel.done()
                and cancel.result() is not None
                and bool(cancel.result().goals_canceling),
            )
    node.destroy_node()
    rclpy.shutdown()
    stream.close()
    return 0 if value.status == (5 if args.cancel_after is not None else 4) else 1


if __name__ == "__main__":
    raise SystemExit(main())
