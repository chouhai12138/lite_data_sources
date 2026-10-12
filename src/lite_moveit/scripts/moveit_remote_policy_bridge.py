#!/usr/bin/env python3
"""
MoveIt → lite_bridge 桥接节点（支持 arms / arms_grippers 两种模式）

职责：把 MoveIt 的 FollowJointTrajectory action 转成**逐周期流式**的关节目标，
发到 lite_bridge 的仲裁入口；由桥做多源仲裁 + 归一化，再转给控制器。

⚠️ 消息类型与话题（改造自 bar_msgs 时代）
------------------------------------------------
旧版发 `bar_msgs/MITCommand` → `/remote_policy_controller/command`，
那是参考工程 lite_ros2 的接管控制器，本工程**没有**这个控制器。

现在发 `robstride_msgs/MotionCommand` → `/lite_bridge/motion_command`，
与 lite_bridge 对接。两者字段对应关系：

    bar_msgs/MITCommand        robstride_msgs/MotionCommand
    ---------------------      ----------------------------
    joint_names                joint_names
    position                   position
    velocity                   velocity
    effort                     effort
    stiffness                  **kp**
    damping                    **kd**
    (无)                       **source**   ← 仲裁必需，桥据此认源
    (无)                       **mode**     ← 必须与桥的 control_mode_ 一致
    (无)                       timeout_ms

支持力矩前馈补偿: 从 identified_params.json 加载摩擦参数，
自动填充 MotionCommand.effort 通道，提升轨迹跟踪精度。

用法:
  ros2 run lite_moveit moveit_remote_policy_bridge \
    --ros-args -p stiffness:=20.0 -p damping:=2.0 \
    -p source_name:=moveit \
    -p mode:=arms \
    -p enable_torque_compensation:=true

⚠️ 使用前提（桥侧，缺一不可）
------------------------------------------------
1. 桥的 `source_names` 必须含本节点声明的 `source_name`（默认 `moveit`），
   否则命令被丢弃并打 `丢弃未知数据源 'moveit' 的命令`。
   桥的 `source_names` 默认是**空列表**（不认任何源），只能启动时给：
       ros2 launch lite_bringup lite_bringup.launch.py \
           mode:=arms source_names:=moveit
   ⚠️ 它**没有运行期接口** —— 空列表时只能重启桥。

2. 桥的协议模式必须与本节点 `control_mode` 参数**一致**，否则整帧被拒。
   `lite_bringup` 的 `initial_control_mode` 默认 `auto`，跟随 `controller:=`：
       controller=modes（默认）→ mit     本节点传 control_mode:=mit
       controller=csp          → csp     本节点传 control_mode:=csp
   ✅ **mit 与 csp 现在都可用**（CSP 通路已实装并真机验证）。
      两个参数由 `one_click.launch.py` 自动保持一致。

3. 三态模式要处于 **TRACKING**（只有它读命令）：
       ros2 service call /mode_manager/set_mode \
         robstride_msgs/srv/SetRunMode "{mode: 1}"
       ros2 service call /mode_manager/set_mode \
         robstride_msgs/srv/SetRunMode "{mode: 2}"

✅ 上述三条 `one_click.launch.py` 都已自动满足（它用同一个 launch 参数
   同时驱动"桥的白名单"与"本节点的 source 字段"，不可能配岔）：
       ros2 launch lite_moveit one_click.launch.py                # 停在 HOLD
       ros2 launch lite_moveit one_click.launch.py auto_tracking:=true
   手工起桥时才需要自己保证上面 1/2/3。

⚠️ TRACKING 之后、第一条轨迹之前，机械臂是**松的**
（tracking_controller 的 stale_command_policy=passive，且激活时清空历史
命令）；本节点的发布线程只在 `_execute_callback` 里才启动。
详见 README「关键安全事实」。
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import SingleThreadedExecutor
from control_msgs.action import FollowJointTrajectory
from robstride_msgs.msg import MotionCommand
from sensor_msgs.msg import JointState
import threading
import time

# 力矩前馈补偿（摩擦 + Pinocchio 动力学）
try:
    from torque_compensation import TorqueCompensation
    _TC_AVAILABLE = True
except ImportError:
    _TC_AVAILABLE = False

class MoveItToRemotePolicyBridge(Node):
    def __init__(self):
        super().__init__('moveit_remote_policy_bridge')
        
        # 参数
        self.declare_parameter('stiffness', 20.0)
        self.declare_parameter('damping', 2.0)
        self.declare_parameter('publish_rate', 50.0)
        # 关节集合。**默认 arms_grippers（16）是刻意选的"超集"**：
        #   · 本节点发的关节名里，桥不认识的只算警告（丢弃那几个）；
        #   · 反过来**少发**会被桥**整帧拒绝** —— lite_bridge 的
        #     CommandNormalizer 强制命令必须覆盖全部 canonical_joints
        #     （安全不变量 I5），覆盖不全直接拒帧并在日志里列出缺哪些关节。
        #     这是后来加固的：更早的版本是**静默**给未触及关节填
        #     position=0 + 兜底 Kp=20 → 那个关节真的被驱动到零位。
        #   所以：宁可多发，不可少发。
        #   launch（demo/one_click）会显式传 mode，与本机 lite_bringup 的
        #   mode 对齐；只有手工 `ros2 run` 时才用到这个默认值。
        self.declare_parameter('mode', 'arms_grippers')  # 默认16关节

        # --- 与 lite_bridge 对接的参数 ---------------------------------------
        # ⚠️ `source_name` 必须出现在桥的 `source_names` 里，否则命令被丢弃。
        #    桥的 config/bridge.yaml 里已声明过 `moveit`，故默认用它。
        self.declare_parameter('source_name', 'moveit')
        # 下发话题 —— 桥的 `~/motion_command` 展开即此。
        self.declare_parameter('command_topic', '/lite_bridge/motion_command')
        # 关节状态来源。本工程驱动发的是 /joint_states（不是 /lite/joint_states）。
        self.declare_parameter('joint_states_topic', '/joint_states')
        # 命令里的 protocol 字段，必须与桥的 control_mode_ 一致。
        #   'mit' → MotionCommand.MODE_MIT（桥发往 /tracking_controller/command）
        #   'csp' → MotionCommand.MODE_CSP（桥发往 /csp_tracking_controller/command）
        #   ✅ 两者都已实装；选哪个取决于桥的 control_mode_（必须一致）。
        self.declare_parameter('control_mode', 'mit')
        # 消息里的 timeout_ms。⚠️ 当前桥**不读**该字段（超时取自它自己的
        # source_timeouts_ms 参数），这里填只为语义完整与将来兼容。
        self.declare_parameter('command_timeout_ms', 500.0)

        self.K = self.get_parameter('stiffness').value
        self.D = self.get_parameter('damping').value
        self.dt = 1.0 / self.get_parameter('publish_rate').value
        self.mode = self.get_parameter('mode').value

        self.source_name = self.get_parameter('source_name').value
        self.command_topic = self.get_parameter('command_topic').value
        self.joint_states_topic = self.get_parameter('joint_states_topic').value
        self.command_timeout_ms = float(
            self.get_parameter('command_timeout_ms').value)

        _mode_str = str(self.get_parameter('control_mode').value).lower()
        if _mode_str not in ('mit', 'csp'):
            self.get_logger().error(
                f"control_mode 必须是 'mit' 或 'csp'，得到 '{_mode_str}'；改用 'mit'"
            )
            _mode_str = 'mit'
        self.control_mode = _mode_str
        if self.control_mode == 'csp':
            # ✅ CSP 已实装（不再告警）。但**必须**提醒启动后所处的档：
            #    CSP 组启动默认停在 mode5 CSP_MONITOR（**不使能**），
            #    须手动 6 → 7 才会跟随命令。
            self.get_logger().info(
                'control_mode=csp（CSP 通路）。⚠️ 启动后桥处于 mode5 '
                'CSP_MONITOR（不使能），要到"跟随"需分两步切档：'
                'set_mode {protocol: 255, mode: 6} 然后 {mode: 7}。'
            )


        # 力矩补偿参数
        self.declare_parameter('enable_torque_compensation', False)
        self.declare_parameter('torque_params_path', '')
        self.declare_parameter('coulomb_epsilon', 0.01)
        self.declare_parameter('torque_compensation_scale', 1.0)
        self.declare_parameter('integral_gain', 0.0)
        self.declare_parameter('integral_max', 8.0)
        self.enable_tc = self.get_parameter('enable_torque_compensation').value
        self._torque_comp = None
        self._tc_scale = self.get_parameter('torque_compensation_scale').value
        self._ki = self.get_parameter('integral_gain').value
        self._integral_max = self.get_parameter('integral_max').value

        # 积分误差累积（每个关节独立，运动期间清零）
        self._integral_error = None  # 初始化为 None，等 num_joints 确定后分配

        if self.enable_tc:
            if not _TC_AVAILABLE:
                self.get_logger().warn(
                    'torque_compensation module not found in PYTHONPATH; '
                    'torque compensation disabled. '
                    'Ensure lite_moveit/scripts is on PYTHONPATH.'
                )
                self.enable_tc = False
            else:
                tc_path = self.get_parameter('torque_params_path').value
                if tc_path == '':
                    tc_path = None  # 让 TorqueCompensation 自动查找
                coulomb_eps = self.get_parameter('coulomb_epsilon').value
                try:
                    self._torque_comp = TorqueCompensation(
                        params_path=tc_path,
                        coulomb_epsilon=coulomb_eps,
                    )
                    self.get_logger().info(
                        f'Torque compensation ENABLED '
                        f'(fit RMSE={self._torque_comp.fit_rmse_nm:.3f} Nm, '
                        f'{len(self._torque_comp.identified_joints)} joints)'
                    )
                except FileNotFoundError as e:
                    self.get_logger().warn(
                        f'identified_params.json not found: {e}. '
                        f'Torque compensation disabled. '
                        f'Place the file in lite_moveit/config/ or pass '
                        f'torque_params_path:=/path/to/identified_params.json'
                    )
                    self.enable_tc = False
        else:
            self.get_logger().info('Torque compensation DISABLED (enable_torque_compensation:=false)')
        
        # ============================================================
        # 根据模式选择关节配置
        # ============================================================
        if self.mode == 'arms_grippers':
            # 16关节：双臂 + 夹爪
            self.bar_joint_order = [
                'left_shoulder_pitch',   # 0
                'left_shoulder_roll',    # 1
                'left_shoulder_yaw',     # 2
                'left_elbow_pitch',      # 3
                'left_wrist_yaw',        # 4
                'left_wrist_roll',       # 5
                'left_wrist_pitch',      # 6
                'right_shoulder_pitch',  # 7
                'right_shoulder_roll',   # 8
                'right_shoulder_yaw',    # 9
                'right_elbow_pitch',     # 10
                'right_wrist_yaw',       # 11
                'right_wrist_roll',      # 12
                'right_wrist_pitch',     # 13
                'left_gripper',          # 14
                'right_gripper'          # 15
            ]
            self.num_joints = 16
            self.gripper_indices = [14, 15]
            self.get_logger().info('=== Mode: arms_grippers (16 joints) ===')
        else:
            # 14关节：双臂，无夹爪
            self.bar_joint_order = [
                'left_shoulder_pitch',   # 0
                'left_shoulder_roll',    # 1
                'left_shoulder_yaw',     # 2
                'left_elbow_pitch',      # 3
                'left_wrist_yaw',        # 4
                'left_wrist_roll',       # 5
                'left_wrist_pitch',      # 6
                'right_shoulder_pitch',  # 7
                'right_shoulder_roll',   # 8
                'right_shoulder_yaw',    # 9
                'right_elbow_pitch',     # 10
                'right_wrist_yaw',       # 11
                'right_wrist_roll',      # 12
                'right_wrist_pitch',     # 13
            ]
            self.num_joints = 14
            self.gripper_indices = []
            self.get_logger().info('=== Mode: arms (14 joints) ===')
        
        for i, name in enumerate(self.bar_joint_order):
            self.get_logger().info(f'  {i:2d}: {name}')
        
        # MoveIt 左臂关节 -> BAR索引
        self.moveit_to_bar_left = {
            'left_elbow_pitch': 3,
            'left_shoulder_pitch': 0,
            'left_shoulder_roll': 1,
            'left_shoulder_yaw': 2,
            'left_wrist_pitch': 6,
            'left_wrist_roll': 5,
            'left_wrist_yaw': 4,
        }
        
        # MoveIt 右臂关节 -> BAR索引
        self.moveit_to_bar_right = {
            'right_elbow_pitch': 10,
            'right_shoulder_pitch': 7,
            'right_shoulder_roll': 8,
            'right_shoulder_yaw': 9,
            'right_wrist_pitch': 13,
            'right_wrist_roll': 12,
            'right_wrist_yaw': 11,
        }
        
        # 合并映射
        self.moveit_to_bar = {**self.moveit_to_bar_left, **self.moveit_to_bar_right}

        # ⚠️ 夹爪必须**显式**补进来。
        #
        #    bar_joint_order 在 arms_grippers 下虽然含 left_gripper/right_gripper，
        #    但上面两张映射表只有 7+7 个手臂关节 —— 不补这一段，
        #    left_gripper / right_gripper 就会被当成 "Unknown joint" 每拍丢弃，
        #    而轨迹依然报 "✓ Trajectory completed successfully"（**假成功**：
        #    MoveIt/RViz 显示成功，夹爪压根没动）。
        if self.mode == 'arms_grippers':
            self.moveit_to_bar['left_gripper'] = self.gripper_indices[0]
            self.moveit_to_bar['right_gripper'] = self.gripper_indices[1]
        
        # 存储当前关节状态
        self.current_positions = {}
        self.current_velocities = {}
        self.joint_state_received = False
        
        # 订阅关节状态
        self.joint_state_sub = self.create_subscription(
            JointState,
            self.joint_states_topic,
            self._joint_state_callback,
            10
        )
        self.get_logger().info(
            f'Subscribed to {self.joint_states_topic}')

        # 发布器 —— 发到 lite_bridge 的仲裁入口
        self.pub = self.create_publisher(
            MotionCommand,
            self.command_topic,
            10
        )
        self.get_logger().info(
            f'Publishing MotionCommand → {self.command_topic} '
            f'(source={self.source_name}, mode={self.control_mode.upper()})')

        # Action服务器
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/remote_policy_controller/follow_joint_trajectory',
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            execute_callback=self._execute_callback
        )
        
        self._lock = threading.RLock()
        self._running = False
        self._publish_thread = None
        self._stop_event = threading.Event()
        self._current_cmd = None
        self._hold_mode = False
        self._trajectory = None
        self._joint_names = []
        self._start_time_ns = 0
        self._pub_counter = 0
        self._hold_snapshot = None  # snapshot of all BAR joints at trajectory start

        # 积分误差累积，运动结束后自动补偿稳态偏差
        self._integral_error = [0.0] * self.num_joints

        self.get_logger().info(f'=== Bridge Ready - {self.num_joints} joints ===')

    def _joint_state_callback(self, msg: JointState):
        with self._lock:
            for i, name in enumerate(msg.name):
                if name in self.bar_joint_order:
                    self.current_positions[name] = msg.position[i]
                    if len(msg.velocity) > i:
                        self.current_velocities[name] = msg.velocity[i]
            self.joint_state_received = True

    def _get_current_bar_state(self):
        """获取BAR格式的当前状态"""
        positions = [0.0] * self.num_joints
        velocities = [0.0] * self.num_joints
        with self._lock:
            for i, name in enumerate(self.bar_joint_order):
                if name in self.current_positions:
                    positions[i] = self.current_positions[name]
                    velocities[i] = self.current_velocities.get(name, 0.0)
        return positions, velocities

    def _map_moveit_to_bar(self, moveit_joint_names, positions, velocities):
        """将MoveIt关节映射到BAR关节，非MoveIt关节使用轨迹开始时的快照。"""
        # 优先使用快照（hold 非运动关节），没有快照时用实时值
        if self._hold_snapshot is not None:
            bar_positions = list(self._hold_snapshot)
        else:
            bar_positions, _ = self._get_current_bar_state()
        bar_velocities = [0.0] * self.num_joints  # hold joints: zero velocity
        
        moveit_values = {}
        for name, pos, vel in zip(moveit_joint_names, positions, velocities):
            moveit_values[name] = (pos, vel)
        
        # 更新所有关节
        for moveit_name, (pos, vel) in moveit_values.items():
            if moveit_name in self.moveit_to_bar:
                bar_idx = self.moveit_to_bar[moveit_name]
                bar_positions[bar_idx] = float(pos)
                bar_velocities[bar_idx] = float(vel)
            else:
                self.get_logger().warn(f'Unknown joint: {moveit_name}')
        
        # 调试日志
        self._pub_counter += 1
        if self._pub_counter % 50 == 0:
            left_pos = [bar_positions[i] for i in range(7)]
            right_pos = [bar_positions[i] for i in range(7, 14)]
            gripper_pos = [bar_positions[i] for i in self.gripper_indices] if self.gripper_indices else []
            self.get_logger().info(
                f'Left: {[f"{p:.3f}" for p in left_pos[:3]]}..., '
                f'Right: {[f"{p:.3f}" for p in right_pos[:3]]}..., '
                f'Grippers: {[f"{p:.3f}" for p in gripper_pos]}' if gripper_pos else ''
            )
        
        return self.bar_joint_order, bar_positions, bar_velocities

    def _create_motion_command(self, joint_names, positions, velocities,
                               actual_velocities=None, actual_positions=None):
        """构造发往 lite_bridge 的 MotionCommand。

        相比旧的 bar_msgs/MITCommand，多了三个桥**必需**的字段：
          · source —— 桥据此认源并做仲裁（必须在其 source_names 里）
          · mode   —— 必须与桥的 control_mode_ 一致，否则整帧被拒
          · timeout_ms —— 语义完整（桥当前不读，见文件头说明）

        并且 stiffness/damping 改名为 **kp/kd**。

        Args:
            joint_names: 关节名列表（必须与桥的 canonical_joints 同序同集合）
            positions: 期望位置 (PD 目标)
            velocities: 期望速度 (PD 阻尼项)
            actual_velocities: 实际速度, 用于摩擦前馈
            actual_positions: 实际位置, 用于重力补偿
        """
        n = len(joint_names)
        cmd = MotionCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.source = self.source_name
        cmd.mode = (MotionCommand.MODE_MIT if self.control_mode == 'mit'
                    else MotionCommand.MODE_CSP)
        cmd.timeout_ms = self.command_timeout_ms
        cmd.joint_names = joint_names
        cmd.position = positions
        cmd.velocity = velocities

        cmd.effort = [0.0] * n

        if self.enable_tc and self._torque_comp is not None:
            v_fric = actual_velocities if actual_velocities is not None else velocities
            # 1. 摩擦前馈
            raw_effort = self._torque_comp.compute_friction(joint_names, v_fric)
            # 2. 重力补偿 (使用实际位置)
            if actual_positions is not None and self._torque_comp.uses_pinocchio:
                grav = self._torque_comp.compute_gravity(joint_names, actual_positions)
            else:
                grav = [0.0] * n
            # 3. 积分修正
            for i in range(n):
                cmd.effort[i] = (
                    raw_effort[i] * self._tc_scale
                    + grav[i]
                    + self._integral_error[i]
                )

        cmd.kp = [self.K] * n
        cmd.kd = [self.D] * n

        if len(cmd.joint_names) != self.num_joints:
            self.get_logger().error(f'ERROR: joint_names length = {len(cmd.joint_names)}, expected {self.num_joints}!')

        return cmd

    def _goal_callback(self, goal_request):
        self.get_logger().info('=== GOAL RECEIVED ===')
        traj = goal_request.trajectory
        if not traj.points:
            self.get_logger().error('Empty trajectory!')
            return GoalResponse.REJECT
        
        input_joints = list(traj.joint_names)
        self.get_logger().info(f'MoveIt joints ({len(input_joints)}): {input_joints}')
        
        left_joints = [j for j in input_joints if j.startswith('left_')]
        right_joints = [j for j in input_joints if j.startswith('right_')]
        self.get_logger().info(f'Left: {len(left_joints)} joints, Right: {len(right_joints)} joints')

        # ⚠️ 含本节点认不出的关节 → **拒绝**，不能只打警告继续。
        #
        #    典型成因：mode 不一致（桥跑 arms=14，MoveIt 却规划了夹爪）。
        #    只警告的后果是轨迹"成功结束"但关节根本没动 —— MoveIt/RViz 全绿、
        #    现场毫无反应，极难排查。拒绝会让 MoveIt 明确报失败。
        unknown = [j for j in input_joints if j not in self.moveit_to_bar]
        if unknown:
            self.get_logger().error(
                f'拒绝目标：{unknown} 不在本节点的关节列表里'
                f'（当前 mode={self.mode}，共 {self.num_joints} 个关节）。'
                f'要用夹爪请用 mode:=arms_grippers 重启'
                f'（ros2 launch lite_moveit one_click.launch.py mode:=arms_grippers），'
                f'并确认 bringup 与桥的 mode 一致。')
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel requested')
        with self._lock:
            self._running = False
            self._hold_mode = False
        self._stop_publish_thread()
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        self.get_logger().info('=== EXECUTING TRAJECTORY ===')
        goal = goal_handle.request
        
        input_joint_names = list(goal.trajectory.joint_names)
        points = goal.trajectory.points
        
        trajectory = self._build_trajectory(points)
        
        if len(trajectory) < 2:
            self.get_logger().error('Invalid trajectory')
            goal_handle.abort()
            return FollowJointTrajectory.Result()
        
        total_duration = trajectory[-1]['t']
        self.get_logger().info(f'Duration: {total_duration:.3f}s, Points: {len(trajectory)}')
        
        with self._lock:
            self._joint_names = input_joint_names
            self._trajectory = trajectory
            self._start_time_ns = self.get_clock().now().nanoseconds
            self._running = True
            self._hold_mode = False
            self._stop_event.clear()
            self._pub_counter = 0
            # Snapshot current positions for joints NOT in this trajectory
            bar_pos, _ = self._get_current_bar_state()
            self._hold_snapshot = list(bar_pos)
        
        if self._publish_thread is None or not self._publish_thread.is_alive():
            self._start_publish_thread()
        
        timeout = max(total_duration + 5.0, 10.0)
        start_time = time.time()
        trajectory_completed = False
        
        while time.time() - start_time < timeout:
            with self._lock:
                if not self._running and self._hold_mode:
                    trajectory_completed = True
                    break
            time.sleep(0.05)
        
        if trajectory_completed:
            self.get_logger().info('✓ Trajectory completed, entering hold mode')
        else:
            self.get_logger().warn('Trajectory timeout, entering hold mode')
            with self._lock:
                self._running = False
                self._hold_mode = True
        
        goal_handle.succeed()
        self.get_logger().info('✓ Trajectory completed successfully')
        return FollowJointTrajectory.Result()

    def _build_trajectory(self, points):
        traj = []
        for i, pt in enumerate(points):
            t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            if i > 0 and t <= traj[-1]['t']:
                t = traj[-1]['t'] + 0.001
            traj.append({
                't': t,
                'pos': list(pt.positions) if pt.positions else [],
                'vel': list(pt.velocities) if pt.velocities else None,
            })
        
        if len(traj) == 1:
            last = traj[0]
            traj.append({
                't': last['t'] + 1.0,
                'pos': last['pos'],
                'vel': [0.0] * len(last['pos']) if last['vel'] is None else last['vel'],
            })
        
        n_joints = len(self._joint_names) if hasattr(self, '_joint_names') else 7
        for point in traj:
            if not point['pos']:
                point['pos'] = [0.0] * n_joints
            if point['vel'] is None:
                point['vel'] = [0.0] * len(point['pos'])
        
        return traj

    def _start_publish_thread(self):
        if self._publish_thread and self._publish_thread.is_alive():
            self._stop_publish_thread()
        self._publish_thread = threading.Thread(target=self._publish_loop, daemon=False)
        self._publish_thread.start()
        self.get_logger().info('Publish thread started')

    def _stop_publish_thread(self):
        self._stop_event.set()
        if self._publish_thread:
            self._publish_thread.join(timeout=3.0)
            self._publish_thread = None

    def _publish_loop(self):
        last_publish_time = time.time()
        self._integral_lock = threading.RLock()
        
        while not self._stop_event.is_set():
            with self._lock:
                if self._trajectory is None:
                    # Idle: publish keep-alive to prevent controller timeout.
                    # Use current position as target with zero stiffness/damping
                    # (passive — motor freewheeling, same as stale_command_policy).
                    #
                    # ⚠️ Kp=Kd=0 时 tracking_controller 输出 τ=0，关节是**松的**。
                    #    这样既刷新了"源还活着"（避免仲裁超时），又不施加任何力。
                    bar_q, _ = self._get_current_bar_state()
                    cmd = MotionCommand()
                    cmd.header.stamp = self.get_clock().now().to_msg()
                    cmd.source = self.source_name
                    cmd.mode = (MotionCommand.MODE_MIT if self.control_mode == 'mit'
                                else MotionCommand.MODE_CSP)
                    cmd.timeout_ms = self.command_timeout_ms
                    cmd.joint_names = self.bar_joint_order
                    cmd.position = bar_q
                    cmd.velocity = [0.0] * self.num_joints
                    cmd.effort = [0.0] * self.num_joints
                    cmd.kp = [0.0] * self.num_joints
                    cmd.kd = [0.0] * self.num_joints
                    self.pub.publish(cmd)
                    time.sleep(self.dt)
                    continue
                
                now_ns = self.get_clock().now().nanoseconds
                t = (now_ns - self._start_time_ns) / 1e9
                
                if not self._hold_mode and t < self._trajectory[-1]['t']:
                    # 轨迹执行中：清零积分（只在首次进入时）
                    with self._integral_lock:
                        self._integral_error = [0.0] * self.num_joints

                    q, qd = self._interpolate_linear(t)
                    bar_names, bar_q, bar_qd = self._map_moveit_to_bar(
                        self._joint_names, q, qd
                    )
                    _, bar_qd_actual = self._get_current_bar_state()
                    bar_q_actual, _ = self._get_current_bar_state()
                    cmd = self._create_motion_command(bar_names, bar_q, bar_qd, bar_qd_actual, bar_q_actual)
                    self.pub.publish(cmd)
                    self._current_cmd = cmd
                else:
                    if not self._hold_mode:
                        self._hold_mode = True
                        self._running = False
                        # 轨迹结束，清零积分准备重新累积
                        with self._integral_lock:
                            self._integral_error = [0.0] * self.num_joints
                        self.get_logger().info('Entering hold mode (integral learning active)')

                        last_traj = self._trajectory[-1]
                        q = last_traj['pos']
                        qd = [0.0] * len(q)
                        bar_names, bar_q, bar_qd = self._map_moveit_to_bar(
                            self._joint_names, q, qd
                        )
                        _, bar_qd_actual = self._get_current_bar_state()
                        bar_q_actual, _ = self._get_current_bar_state()
                        cmd = self._create_motion_command(bar_names, bar_q, bar_qd, bar_qd_actual, bar_q_actual)
                        self._current_cmd = cmd

                    # hold 模式：累积积分误差
                    if self._ki > 0 and self._current_cmd is not None:
                        bar_actual_positions, bar_qd_actual = self._get_current_bar_state()
                        with self._integral_lock:
                            for i in range(self.num_joints):
                                err = self._current_cmd.position[i] - bar_actual_positions[i]
                                self._integral_error[i] += err * self.dt * self._ki
                                self._integral_error[i] = max(-self._integral_max,
                                    min(self._integral_max, self._integral_error[i]))
                        cmd = self._create_motion_command(
                            self._current_cmd.joint_names,
                            self._current_cmd.position,
                            self._current_cmd.velocity,
                            bar_qd_actual,
                            bar_actual_positions
                        )
                        self._current_cmd = cmd

                    if self._current_cmd is not None:
                        self.pub.publish(self._current_cmd)
            
            current_time = time.time()
            sleep_time = self.dt - (current_time - last_publish_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
            last_publish_time = time.time()

    def _interpolate_linear(self, t):
        traj = self._trajectory
        n = len(traj)
        if n == 0:
            return [], []
        
        idx = 0
        while idx < n - 1 and t > traj[idx + 1]['t']:
            idx += 1
        
        if idx >= n - 1:
            return traj[-1]['pos'], [0.0] * len(traj[-1]['pos'])
        
        t0, t1 = traj[idx]['t'], traj[idx + 1]['t']
        dt = max(t1 - t0, 0.001)
        alpha = (t - t0) / dt
        
        p0, p1 = traj[idx]['pos'], traj[idx + 1]['pos']
        v0, v1 = traj[idx]['vel'], traj[idx + 1]['vel']
        
        q = [(1.0 - alpha) * a + alpha * b for a, b in zip(p0, p1)]
        if v0 and v1:
            qd = [(1.0 - alpha) * a + alpha * b for a, b in zip(v0, v1)]
        else:
            qd = [(b - a) / dt for a, b in zip(p0, p1)]
        
        return q, qd

    def destroy(self):
        self._stop_publish_thread()
        self._action_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MoveItToRemotePolicyBridge()
    
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()