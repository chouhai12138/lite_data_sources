#!/usr/bin/env python3
"""验证 move_group 能否把「只含夹爪」的轨迹派给 remote_policy_controller。

针对的错误：
  Unable to identify any set of controllers that can actuate the specified
  joints: [ left_gripper ]  → Apparently trajectory initialization failed

仅在 use_fake_hardware:=true 时使用；目标 0.0 → 0.01 m，幅度很小。
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

ACTION = '/remote_policy_controller/follow_joint_trajectory'
JOINTS = ['left_gripper']


def main():
    rclpy.init()
    node = Node('gripper_link_tester')
    client = ActionClient(node, FollowJointTrajectory, ACTION)
    if not client.wait_for_server(timeout_sec=25.0):
        print('FAIL: action server 不存在')
        node.destroy_node()
        rclpy.shutdown()
        return 1
    print('OK: action server 在线')

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = JOINTS
    p0 = JointTrajectoryPoint(positions=[0.0], velocities=[0.0])
    p0.time_from_start = Duration(sec=0, nanosec=0)
    p1 = JointTrajectoryPoint(positions=[0.01], velocities=[0.0])
    p1.time_from_start = Duration(sec=2, nanosec=0)
    goal.trajectory.points = [p0, p1]

    fut = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=15.0)
    handle = fut.result()
    if handle is None:
        print('FAIL: 发送目标超时')
        node.destroy_node()
        rclpy.shutdown()
        return 1
    print(f'goal accepted = {handle.accepted}')
    if not handle.accepted:
        node.destroy_node()
        rclpy.shutdown()
        return 1

    rfut = handle.get_result_async()
    rclpy.spin_until_future_complete(node, rfut, timeout_sec=30.0)
    res = rfut.result()
    if res is None:
        print('WARN: 30 s 内没拿到结果（可能仍在 hold 阶段）')
    else:
        print(f'result error_code = {res.result.error_code} '
              f'({res.result.error_string!r})')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
