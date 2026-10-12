#!/usr/bin/env python3
"""
Motor Monitor - console dashboard for real-time joint states.

Shows position, velocity, and torque (effort) for all 16 joints
in a live-updating terminal table.

Usage:
  ros2 run lite_moveit motor_monitor
  ros2 run lite_moveit motor_monitor --ros-args -p rate:=10.0
"""

import os
import sys
import signal
import time
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


# Joint limits from URDF (min, max in radians)
JOINT_LIMITS = {
    'left_shoulder_pitch':      (-3.1416, 0.7854),
    'left_shoulder_roll':       (-1.5708, 1.9199),
    'left_shoulder_yaw':        (-1.5708, 1.5708),
    'left_elbow_pitch':         (-2.3562, 0.0),
    'left_wrist_yaw':           (-1.5708, 1.5708),
    'left_wrist_roll':          (-0.6981, 0.6981),
    'left_wrist_pitch':         (-0.7854, 0.7854),
    'right_shoulder_pitch':     (-3.1416, 0.7854),
    'right_shoulder_roll':      (-1.9199, 1.5708),
    'right_shoulder_yaw':       (-1.5708, 1.5708),
    'right_elbow_pitch':        (0.0,     2.3562),
    'right_wrist_yaw':          (-1.5708, 1.5708),
    'right_wrist_roll':         (-0.6981, 0.6981),
    'right_wrist_pitch':        (-0.7854, 0.7854),
    'left_gripper':             (0.0,     0.047),
    'right_gripper':            (0.0,     0.047),
}

BAR_WIDTH = 20  # progress bar width in chars


def _progress_bar(value, lo, hi, width=BAR_WIDTH):
    """Render a colored progress bar — unbounded, negative or >100% is valid."""
    if hi <= lo:
        return ' ' * (width + 2)
    ratio = (value - lo) / (hi - lo)
    # 不夹紧，允许超出 0..1

    filled = int(ratio * width)
    filled = max(0, min(width, filled))  # only clamp bar visual, not ratio

    # Color by proximity to limits
    margin = min(ratio, 1.0 - ratio) if 0 <= ratio <= 1 else -1.0
    if ratio < 0 or ratio > 1.0:
        color = RED       # outside limits
    elif margin < 0.05:
        color = RED
    elif margin < 0.15:
        color = YELLOW
    else:
        color = GREEN

    if ratio < 0:
        bar_fill = ''  # all empty
    elif ratio >= 1.0:
        bar_fill = '=' * width
    elif filled < width:
        bar_fill = '=' * filled + BOLD + '>' + RESET + color + ' ' * (width - filled - 1)
    else:
        bar_fill = '=' * width
    bar = color + bar_fill + RESET

    pct = ratio * 100
    return f'[{bar}] {pct:6.1f}%'
CLEAR = '\033[2J\033[H'
HIDE_CURSOR = '\033[?25l'
SHOW_CURSOR = '\033[?25h'
BOLD = '\033[1m'
RESET = '\033[0m'
GREEN = '\033[32m'
YELLOW = '\033[33m'
RED = '\033[31m'
CYAN = '\033[36m'
MAGENTA = '\033[35m'

HEADER_COLOR = CYAN + BOLD


class MotorMonitor(Node):
    def __init__(self):
        super().__init__('motor_monitor')

        self.declare_parameter('rate', 10.0)
        self.declare_parameter('joint_state_topic', '/lite/joint_states')

        self._rate = self.get_parameter('rate').value
        self._lock = threading.Lock()

        # Latest data
        self._joint_names = []
        self._positions = []
        self._velocities = []
        self._efforts = []

        self._sub = self.create_subscription(
            JointState,
            self.get_parameter('joint_state_topic').value,
            self._callback,
            10,
        )

        self._running = True
        signal.signal(signal.SIGINT, self._on_signal)
        signal.signal(signal.SIGTERM, self._on_signal)

        self.get_logger().info(
            f'Motor monitor started, subscribing to '
            f'{self.get_parameter("joint_state_topic").value}'
        )

    def _on_signal(self, signum, frame):
        self._running = False

    def _callback(self, msg: JointState):
        with self._lock:
            self._joint_names = list(msg.name)
            self._positions = [float(p) for p in msg.position]
            self._velocities = (
                [float(v) for v in msg.velocity]
                if len(msg.velocity) == len(msg.name)
                else [0.0] * len(msg.name)
            )
            self._efforts = (
                [float(e) for e in msg.effort]
                if len(msg.effort) == len(msg.name)
                else [0.0] * len(msg.name)
            )

    def get_state(self):
        with self._lock:
            return (
                list(self._joint_names),
                list(self._positions),
                list(self._velocities),
                list(self._efforts),
            )

    def run(self):
        """Main display loop."""
        print(HIDE_CURSOR, end='', flush=True)
        try:
            while self._running and rclpy.ok():
                self._render()
                time.sleep(1.0 / self._rate)
        finally:
            print(SHOW_CURSOR, end='', flush=True)

    def _render(self):
        names, pos, vel, eff = self.get_state()
        if not names:
            sys.stdout.write(CLEAR + 'Waiting for /lite/joint_states ...\n')
            sys.stdout.flush()
            return

        # Split left / right / gripper
        left = {}
        right = {}
        grippers = {}
        for n, p, v, e in zip(names, pos, vel, eff):
            if 'gripper' in n:
                grippers[n] = (p, v, e)
            elif n.startswith('left_'):
                left[n] = (p, v, e)
            elif n.startswith('right_'):
                right[n] = (p, v, e)

        arm_order = [
            'shoulder_pitch', 'shoulder_roll', 'shoulder_yaw',
            'elbow_pitch', 'wrist_yaw', 'wrist_roll', 'wrist_pitch',
        ]

        # Build single output string, write atomically
        out = [CLEAR]
        out.append(f'{HEADER_COLOR}{" MOTOR MONITOR ":=^80}{RESET}\n')
        out.append(
            f'{HEADER_COLOR}{"Joint":24s} {"Pos(deg)":>8s} {"Progress Bar":>30s} '
            f'{"Vel":>8s} {"Torque(Nm)":>10s}{RESET}\n'
        )

        def color_torque(t):
            at = abs(t)
            if at > 5.0:
                return f'{RED}{t:10.3f}{RESET}'
            if at > 2.0:
                return f'{YELLOW}{t:10.3f}{RESET}'
            return f'{GREEN}{t:10.3f}{RESET}'

        def format_row(prefix, joint_key, data):
            full_name = prefix + joint_key
            p, v, e = data.get(full_name, (0.0, 0.0, 0.0))
            lo, hi = JOINT_LIMITS.get(full_name, (-3.14, 3.14))
            bar = _progress_bar(p, lo, hi)
            return (
                f'  {joint_key:22s} '
                f'{p*57.3:8.1f} {bar:30s} '
                f'{v:8.3f} '
                f'{color_torque(e)}\n'
            )

        out.append(f'{BOLD}  LEFT ARM{RESET}\n')
        for jk in arm_order:
            out.append(format_row('left_', jk, left))

        out.append(f'\n{BOLD}  RIGHT ARM{RESET}\n')
        for jk in arm_order:
            out.append(format_row('right_', jk, right))

        out.append(f'\n{BOLD}  GRIPPERS{RESET}\n')
        for name in sorted(grippers.keys()):
            p, v, e = grippers[name]
            lo, hi = JOINT_LIMITS.get(name, (0.0, 0.047))
            bar = _progress_bar(p, lo, hi)
            out.append(f'  {name:22s} {p*57.3:8.1f} {bar:30s} {v:8.3f} {color_torque(e)}\n')
            # Also show passive joints
            passive_name = name.replace('gripper', 'gripper_passive_joint')
            if passive_name in grippers:
                pp, pv, pe = grippers[passive_name]
                out.append(f'  {passive_name:22s} {pp*57.3:8.1f} {"":30s} {pv:8.3f} {color_torque(pe)}\n')

        total_torque = sum(abs(e) for e in eff)
        max_t_idx = max(range(len(eff)), key=lambda i: abs(eff[i])) if eff else 0
        out.append(f'\n{"-"*76}\n')
        out.append(
            f'  Total |torque|: {total_torque:6.2f} Nm  |  '
            f'Max: {names[max_t_idx]} @ {eff[max_t_idx]:.2f} Nm\n'
        )

        sys.stdout.write(''.join(out))
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = MotorMonitor()

    spinner = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spinner.start()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        spinner.join(timeout=1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
