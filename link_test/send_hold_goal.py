#!/usr/bin/env python3
"""连接性验证：向 MoveIt 桥接节点发一条"保持当前位置"的轨迹，
观察它是否转成 MotionCommand 并到达 lite_bridge。

仅在 use_fake_hardware:=true 时使用；轨迹目标 = 当前关节位置，不产生运动。
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

JOINTS = [
    'left_shoulder_pitch', 'left_shoulder_roll', 'left_shoulder_yaw',
    'left_elbow_pitch', 'left_wrist_yaw', 'left_wrist_roll', 'left_wrist_pitch',
    'right_shoulder_pitch', 'right_shoulder_roll', 'right_shoulder_yaw',
    'right_elbow_pitch', 'right_wrist_yaw', 'right_wrist_roll', 'right_wrist_pitch',
]
ACTION = '/remote_policy_controller/follow_joint_trajectory'


class LinkTester(Node):
    def __init__(self):
        super().__init__('link_tester')
        self.positions = None
        self.create_subscription(JointState, '/joint_states', self._on_js, 10)

    def _on_js(self, msg):
        names = list(msg.name)
        if all(j in names for j in JOINTS):
            self.positions = [float(msg.position[names.index(j)]) for j in JOINTS]

    def wait_positions(self, timeout=30.0):
        import time
        t0 = time.time()
        while rclpy.ok() and self.positions is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self.positions


def main():
    rclpy.init()
    node = LinkTester()

    pos = node.wait_positions()
    if pos is None:
        node.get_logger().error('30 s 内没拿到完整 /joint_states，无法测试')
        node.destroy_node()
        rclpy.shutdown()
        return 1
    node.get_logger().info('当前关节位置: ' + ', '.join(f'{p:.3f}' for p in pos))

    client = ActionClient(node, FollowJointTrajectory, ACTION)
    if not client.wait_for_server(timeout_sec=15.0):
        node.get_logger().error(f'等不到 action server: {ACTION}')
        node.destroy_node()
        rclpy.shutdown()
        return 1
    node.get_logger().info(f'✓ action server 在线: {ACTION}')

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = JOINTS
    p0 = JointTrajectoryPoint(positions=pos, velocities=[0.0] * len(pos))
    p0.time_from_start = Duration(sec=0, nanosec=0)
    p1 = JointTrajectoryPoint(positions=pos, velocities=[0.0] * len(pos))
    p1.time_from_start = Duration(sec=2, nanosec=0)
    goal.trajectory.points = [p0, p1]

    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future, timeout_sec=10.0)
    handle = send_future.result()
    if handle is None or not handle.accepted:
        node.get_logger().error('目标被拒绝')
        node.destroy_node()
        rclpy.shutdown()
        return 1
    node.get_logger().info('✓ 目标已接受，等待执行结果...')

    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=30.0)
    res = result_future.result()
    if res is None:
        node.get_logger().warn('未在 30 s 内拿到结果（可能仍在 hold 阶段）')
    else:
        node.get_logger().info(f'✓ 轨迹执行返回 error_code={res.result.error_code}')

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
