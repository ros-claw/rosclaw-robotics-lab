"""Emergency cancellation only, within the project's isolated ROS domain.

Uses DDS directly if rosbridge is unavailable. Sends no motion or navigation goal.
"""
import argparse
import json
import time

import rclpy
from action_msgs.srv import CancelGoal


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--timeout', type=float, default=5)
    a = p.parse_args()
    rclpy.init()
    node = rclpy.create_node('rosclaw_emergency_nav_cancel')
    client = node.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
    deadline = time.monotonic() + a.timeout
    result = {'acknowledged': False, 'transport': 'DDS', 'motion_command': False}
    try:
        if client.wait_for_service(timeout_sec=max(0,deadline-time.monotonic())):
            # Zero UUID and stamp are the ROS action protocol's cancel-all request.
            # Domain isolation is required: this server belongs solely to this lab.
            future = client.call_async(CancelGoal.Request())
            rclpy.spin_until_future_complete(node,future,timeout_sec=max(0,deadline-time.monotonic()))
            if future.done() and future.result() is not None:
                response = future.result()
                result.update(acknowledged=response.return_code==0, return_code=response.return_code,
                              goal_ids=[[int(v) for v in g.goal_id.uuid] for g in response.goals_canceling])
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(result))
    return 0 if result['acknowledged'] else 1


if __name__=='__main__':
    raise SystemExit(main())
