#!/usr/bin/env python3
"""SIM-only fault injector: real Nav2 ABORT while real FollowPath stays active.

An intentionally delayed action acceptance reproduces the lost goal-ack race.
No motion topics, invented result, or simulated ROS action server are used for
navigation. Real Nav2 executes the path planned by the real BT navigator.
"""
import argparse
import json
import os
from pathlib import Path
import threading
import time

import rclpy
from action_msgs.msg import GoalStatusArray
from nav2_msgs.action import FollowPath
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile


class DelayedAcceptance(Node):
    def __init__(self, output, physics):
        super().__init__('rosclaw_sim_delayed_follow_path')
        self.output, self.physics = output, physics
        self.group = ReentrantCallbackGroup()
        self.client = ActionClient(self, FollowPath, '/follow_path', callback_group=self.group)
        self.forwarded = {}
        self.current_statuses = {}
        self.log_lock = threading.Lock()
        self.server = ActionServer(self, FollowPath, '/rosclaw_fault/follow_path',
            execute_callback=self.execute, goal_callback=self.accept,
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=self.group)
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._status_subscriptions = [self.create_subscription(GoalStatusArray, name+'/_action/status',
            lambda msg, n=name: self.status(n,msg), qos, callback_group=self.group)
            for name in ('/follow_path','/navigate_to_pose')]

    def log(self, event, **data):
        with self.log_lock:
            with self.output.open('a') as stream:
                stream.write(json.dumps({'wall_time':time.time(),'event':event,**data})+'\n')

    def status(self, name, msg):
        # Retain actual action UUIDs; old terminal goals are not active goals.
        values=[{'uuid':[int(v) for v in s.goal_info.goal_id.uuid],'status':s.status} for s in msg.status_list]
        self.current_statuses[name] = {'wall_time':time.time(),'goals':values}
        snapshot = None
        try:
            snapshot=json.loads(self.physics.read_text())
        except (OSError,ValueError):
            pass
        self.log('actual_action_status',server=name,goals=values,all_servers=self.current_statuses,physics=snapshot)

    def accept(self, request):
        if not self.client.wait_for_server(timeout_sec=5):
            self.log('real_controller_unavailable')
            return GoalResponse.REJECT
        future=self.client.send_goal_async(request)
        ready=threading.Event()
        future.add_done_callback(lambda _:ready.set())
        if not ready.wait(5):
            self.log('real_controller_acceptance_timeout')
            return GoalResponse.REJECT
        handle=future.result()
        if not handle.accepted:
            self.log('real_controller_rejected')
            return GoalResponse.REJECT
        # Key by exact serialized path stamp/poses count; one fault goal only.
        self.forwarded['only_goal']=handle
        self.log('real_controller_goal_accepted',uuid=[int(v) for v in handle.goal_id.uuid],delay_seconds=2.0)
        time.sleep(2.0)
        self.log('delayed_proxy_acceptance_returned')
        return GoalResponse.ACCEPT

    async def execute(self, goal):
        handle=self.forwarded['only_goal']
        response=await handle.get_result_async()
        self.log('real_controller_result',status=response.status,uuid=[int(v) for v in handle.goal_id.uuid])
        if response.status == 4:
            goal.succeed()
        elif response.status == 5:
            # Proxy goal may never have received its own cancel, because the
            # navigator already aborted before receiving the delayed handle.
            goal.abort()
        else:
            goal.abort()
        return response.result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--physics',type=Path,required=True)
    p.add_argument('--isolated-simulation-fault-test',action='store_true',required=True)
    args=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '61':
        raise SystemExit('Fault injector requires the dedicated warehouse SIM domain 61')
    if args.output.exists():
        raise SystemExit('Refusing to overwrite fault evidence')
    rclpy.init()
    node=DelayedAcceptance(args.output,args.physics)
    executor=MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
