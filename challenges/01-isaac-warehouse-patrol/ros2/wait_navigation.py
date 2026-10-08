"""Bounded, read-only readiness check for the real Nav2 server."""

import time

import rclpy
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

rclpy.init()
node = Node("rosclaw_wait_navigation")
state = node.create_client(GetState, "/bt_navigator/get_state")
action = ActionClient(node, NavigateToPose, "/navigate_to_pose")
deadline = time.monotonic() + 120
ready = False
while time.monotonic() < deadline:
    if not state.wait_for_service(timeout_sec=0.5):
        continue
    future = state.call_async(GetState.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=2)
    if future.done() and future.result() is not None:
        if future.result().current_state.id == 3 and action.server_is_ready():
            ready = True
            break
    rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node()
rclpy.shutdown()
if not ready:
    raise SystemExit("Nav2 did not become ACTIVE with a ready action server in 120s")
print("Nav2 ACTIVE; NavigateToPose server READY")
