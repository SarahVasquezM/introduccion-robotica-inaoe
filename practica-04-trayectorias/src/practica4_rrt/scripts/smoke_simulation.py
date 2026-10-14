#!/usr/bin/env python3
"""Headless Gazebo test of dynamic, repeatable RRT action goals."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from action_msgs.msg import GoalStatus
from control_action.action import PIDcontrol
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from std_msgs.msg import Empty
import yaml


def wait_future(node, future, deadline):
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not future.done():
        raise RuntimeError('Timed out waiting for ROS action')
    return future.result()


def send_goal(node, client, values, deadline):
    goal = PIDcontrol.Goal()
    goal.xd, goal.yd, goal.zd, goal.wd, goal.tf = values
    handle = wait_future(node, client.send_goal_async(goal), deadline)
    if not handle.accepted:
        raise RuntimeError('Goal was rejected')
    return wait_future(node, handle.get_result_async(), deadline)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', default='detour')
    parser.add_argument('--timeout', type=float, default=180.0)
    parser.add_argument('--total-time', type=float)
    parser.add_argument('--tracking-kp', type=float, default=3.5)
    args = parser.parse_args()
    workspace = Path.home()/'ros2_ws'
    scenarios = yaml.safe_load(
        (workspace/'src/practica4_rrt/config/scenarios.yaml').read_text()
    )['scenarios']
    scenario = scenarios[args.scenario]
    total_time = args.total_time or scenario['total_time']
    root = workspace/'results/practica4/smoke'/args.scenario/'continuous'
    root.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = env.get('ROS_DOMAIN_ID', '74')
    env['GAZEBO_MASTER_URI'] = env.get('GAZEBO_MASTER_URI', 'http://127.0.0.1:11419')
    os.environ['ROS_DOMAIN_ID'] = env['ROS_DOMAIN_ID']
    log = (root/'launch.log').open('w')
    proc = subprocess.Popen([
        'ros2', 'launch', 'practica4_rrt', 'practica4_rrt.launch.py',
        'use_gui:=false', 'execute:=true', f'results_dir:={root}',
        f'tracking_kp:={args.tracking_kp}',
    ], env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('continuous_rrt_smoke')
    pose = [None]
    contacts = []
    monitoring = [False]
    node.create_subscription(Pose, '/simple_drone/gt_pose',
                             lambda msg: pose.__setitem__(0, msg), 10)
    node.create_subscription(
        ContactsState, '/simple_drone/bumper_states',
        lambda msg: contacts.extend(msg.states) if monitoring[0] else None, 10)
    takeoff = node.create_publisher(Empty, '/simple_drone/takeoff', 10)
    client = ActionClient(node, PIDcontrol, '/pidcontrol')
    deadline = time.monotonic()+args.timeout
    try:
        while (pose[0] is None or not client.server_is_ready() or
               takeoff.get_subscription_count() == 0):
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() > deadline or proc.poll() is not None:
                raise RuntimeError('Simulation unavailable; inspect launch.log')
        takeoff.publish(Empty())
        until = time.monotonic()+3.0
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.1)

        too_fast = send_goal(node, client, (6.0, 0.0, 2.0, 0.0, 1.0), deadline)
        print(f'Too-fast status={too_fast.status}: {too_fast.result.result}', flush=True)
        if too_fast.status != GoalStatus.STATUS_ABORTED:
            raise RuntimeError('Unsafe time was not rejected')

        prepared = send_goal(node, client, (0.0, 0.0, 2.0, 0.0, 8.0), deadline)
        print(f'Preparation status={prepared.status}: {prepared.result.result}', flush=True)
        if prepared.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError('Altitude preparation failed')

        monitoring[0] = True
        target = scenario['goal']
        answer = send_goal(
            node, client,
            (float(target[0]), float(target[1]), 2.0, 0.0, float(total_time)),
            deadline)
        print(f'Navigation status={answer.status}: {answer.result.result}', flush=True)
        if answer.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError('Navigation failed')
        if contacts:
            raise RuntimeError(f'Gazebo reported {len(contacts)} collision contacts')
        print('Gazebo collision contacts: 0', flush=True)

        returned = send_goal(node, client, (0.0, 0.0, 2.0, 0.0, float(total_time)),
                             deadline)
        print(f'Second navigation status={returned.status}: '
              f'{returned.result.result}', flush=True)
        if returned.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError('Second goal without restart failed')
        if contacts:
            raise RuntimeError(
                f'Gazebo reported {len(contacts)} collision contacts during the run'
            )
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=5)
        log.close()


if __name__ == '__main__':
    main()
