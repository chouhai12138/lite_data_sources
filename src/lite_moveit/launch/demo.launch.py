"""
MoveIt RViz + move_group launch，可选附带桥接节点。

用法:
  # 仅 MoveIt（默认，不启动桥接节点）
  ros2 launch lite_moveit demo.launch.py

  # MoveIt + 桥接节点 + 力矩补偿（独立运行，不依赖 one_click）
  ros2 launch lite_moveit demo.launch.py start_bridge:=true

  # 独立运行 + 关闭力矩补偿
  ros2 launch lite_moveit demo.launch.py start_bridge:=true enable_torque_compensation:=false

  # 独立运行 + 自定义 K/D
  ros2 launch lite_moveit demo.launch.py start_bridge:=true stiffness:=40.0 damping:=8.0

参数:
  start_bridge         : 是否启动 moveit_remote_policy_bridge（默认 false）
  stiffness            : 桥接节点刚度 K（默认 20.0）
  damping              : 桥接节点阻尼 D（默认 2.0）
  enable_torque_compensation : 启用力矩前馈补偿（默认 true）
  torque_params_path   : identified_params.json 路径（默认自动查找）
  source_name          : 发给 lite_bridge 的源名（默认 moveit）
                         ⚠️ 必须出现在桥的 source_names 里，否则命令被丢弃
  control_mode         : 命令里的协议字段（默认 mit）
                         ⚠️ 必须与桥的 initial_control_mode 一致
                         ✅ mit / csp **都可选**（CSP 通路已实装并真机验证）
  command_topic        : 下发话题（默认 /lite_bridge/motion_command）
  mode                 : 关节集合（arms = 14 / arms_grippers = 16，默认 arms）
                         ⚠️ **必须与 lite_bringup 的 mode 逐字相同**。桥按自己的
                         canonical_joints 归一化：本节点多发夹爪关节只会刷
                         "未知关节"警告；**少发**则会被桥的完整性校验
                         （安全不变量 I5）**整帧拒绝**，日志列出缺哪些关节。
                         早期版本是静默把没被赋值的关节当成目标 0 → 那个关节
                         被驱动到零位；现在不会了。宁可多不可少，最稳的是两边一致。

⚠️ 数据要真正生效，桥侧还需满足（缺一不可）：
  1. 桥的 source_names 含 source_name（默认 moveit）
  2. 桥的 initial_control_mode 与本节点的 control_mode **一致**
     （lite_bringup 的 auto 默认跟随 controller:=：modes→mit，csp→csp）
  3. 模式处于 **TRACKING**（只有它读命令）：
       MIT 组 → mode2 TRACKING   （只能从 mode1 HOLD 进）
       CSP 组 → mode7 CSP_TRACKING（只能从 mode6 CSP_HOLD 进）
  详见 scripts/moveit_remote_policy_bridge.py 文件头。
"""

import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)


def _build_move_group(moveit_config, joint_states_topic):
    """等价于 `moveit_configs_utils.launches.generate_move_group_launch`，
    但给 move_group 加了 `joint_states` → `joint_states_topic` 的 remap。

    为什么必须 remap（不能靠参数）：
      move_group 的 PlanningSceneMonitor 把关节状态话题名写死为
      `planning_scene_monitor::PlanningSceneMonitor::DEFAULT_JOINT_STATES_TOPIC`
      （= "joint_states"），而 `moveit_cpp.hpp` 里的 `joint_state_topic` 是
      **const 成员**，`Options::load()` 根本不读同名参数 —— 改不了，只能 remap。

    为什么默认指向 `/lite_js/joint_states`：
      lite_bringup 用 `joint_state_broadcaster` 在 **/joint_states** 上只发
      实机存在的电机关节（`mode:=arms` 时是 14 个臂关节）；夹爪这些"运动学里有、
      但没有电机"的关节由 `joint_state_publisher`（lite_joint_state_completer）
      补上默认值后发到 **/lite_js/joint_states**，robot_state_publisher 也是
      订阅这一路。move_group 若仍订阅 /joint_states，就会一直刷

        The complete state of the robot is not yet known.
        Missing left_gripper, right_gripper

    改用补全后的话题即可消除。若你把 lite_bringup 的
    `complete_joint_states` 关掉，请同步把本参数设回 `/joint_states`。
    """
    ld = LaunchDescription()

    ld.add_action(DeclareBooleanLaunchArg('debug', default_value=False))
    ld.add_action(
        DeclareBooleanLaunchArg('allow_trajectory_execution', default_value=True)
    )
    ld.add_action(
        DeclareBooleanLaunchArg('publish_monitored_planning_scene', default_value=True)
    )
    # 非默认 MoveGroup capabilities（空格分隔）
    ld.add_action(
        DeclareLaunchArgument(
            'capabilities',
            default_value=moveit_config.move_group_capabilities['capabilities'],
        )
    )
    # 屏蔽这些默认 capabilities（空格分隔）
    ld.add_action(
        DeclareLaunchArgument(
            'disable_capabilities',
            default_value=moveit_config.move_group_capabilities['disable_capabilities'],
        )
    )
    # 不要把 /joint_states 里的动力学信息拷进内部监控（几乎没人依赖，默认关）
    ld.add_action(DeclareBooleanLaunchArg('monitor_dynamics', default_value=False))

    should_publish = LaunchConfiguration('publish_monitored_planning_scene')
    move_group_configuration = {
        'publish_robot_description_semantic': True,
        'allow_trajectory_execution': LaunchConfiguration('allow_trajectory_execution'),
        # 包一层参数化，让取值可以是空串
        'capabilities': ParameterValue(
            LaunchConfiguration('capabilities'), value_type=str
        ),
        'disable_capabilities': ParameterValue(
            LaunchConfiguration('disable_capabilities'), value_type=str
        ),
        'publish_planning_scene': should_publish,
        'publish_geometry_updates': should_publish,
        'publish_state_updates': should_publish,
        'publish_transforms_updates': should_publish,
        'monitor_dynamics': False,
    }

    add_debuggable_node(
        ld,
        package='moveit_ros_move_group',
        executable='move_group',
        commands_file=str(moveit_config.package_path / 'launch' / 'gdb_settings.gdb'),
        output='screen',
        parameters=[moveit_config.to_dict(), move_group_configuration],
        extra_debug_args=['--debug'],
        # OpenGL 代码可能用到 DISPLAY
        additional_env={'DISPLAY': os.environ.get('DISPLAY', '')},
        # ★ 关键：订阅补全后的关节状态话题
        remappings=[('joint_states', joint_states_topic)],
    )
    return ld


def generate_launch_description():
    # ================================================================
    # 桥接节点参数
    # ================================================================
    start_bridge_arg = DeclareLaunchArgument(
        'start_bridge', default_value='false',
        description='是否启动 moveit_remote_policy_bridge',
    )
    stiffness_arg = DeclareLaunchArgument(
        'stiffness', default_value='20.0',
        description='桥接节点刚度 K（仅 start_bridge:=true 时生效）',
    )
    damping_arg = DeclareLaunchArgument(
        'damping', default_value='2.0',
        description='桥接节点阻尼 D（仅 start_bridge:=true 时生效）',
    )
    # --- 与 lite_bridge 对接的三个参数 -------------------------------
    source_name_arg = DeclareLaunchArgument(
        'source_name', default_value='moveit',
        description='发给 lite_bridge 的源名。'
                    '⚠️ 必须出现在桥的 source_names 里，否则命令被丢弃',
    )
    control_mode_arg = DeclareLaunchArgument(
        'control_mode', default_value='mit',
        description="命令里的协议字段（'mit' / 'csp'）。"
                    '⚠️ 必须与桥的 control_mode_ 一致。'
                    "✅ 'mit' / 'csp' 都可选（CSP 已实装）。", 
    )
    command_topic_arg = DeclareLaunchArgument(
        'command_topic', default_value='/lite_bridge/motion_command',
        description='MotionCommand 下发话题',
    )
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='arms',
        description='关节集合（arms=14 / arms_grippers=16）。'
                    '⚠️ 必须与 lite_bringup 的 mode 逐字相同，'
                    '否则命令的关节列表与桥的 canonical_joints 不一致',
    )
    enable_tc_arg = DeclareLaunchArgument(
        'enable_torque_compensation', default_value='false',
        description='启用力矩前馈补偿（仅 start_bridge:=true 时生效）',
    )
    tc_scale_arg = DeclareLaunchArgument(
        'torque_compensation_scale', default_value='1.0',
        description='力矩补偿缩放因子 (>1 增强, <1 减弱)',
    )
    integral_gain_arg = DeclareLaunchArgument(
        'integral_gain', default_value='0.0',
        description='hold 模式积分增益 (0=关闭, 0.5=轻度, 2.0=激进)',
    )
    torque_params_path_arg = DeclareLaunchArgument(
        'torque_params_path', default_value='',
        description='identified_params.json 路径，留空自动查找',
    )
    # ★ 关节状态话题：move_group 与桥接节点都订阅它。
    #   lite_bringup 把"电机关节 + 无电机关节默认值"补全后发在
    #   /lite_js/joint_states；原始的 /joint_states 只有 14 个臂关节。
    joint_states_topic_arg = DeclareLaunchArgument(
        'joint_states_topic', default_value='/lite_js/joint_states',
        description='move_group / 桥接节点订阅的关节状态话题。'
                    '默认 /lite_js/joint_states = lite_bringup 用 '
                    'joint_state_publisher 补全夹爪后的整机话题（否则 move_group '
                    '会一直报 "Missing left_gripper, right_gripper"）。'
                    '⚠️ 若把 lite_bringup 的 complete_joint_states 关掉，'
                    '请设为 /joint_states。',
    )

    moveit_config = MoveItConfigsBuilder(
        "lite_000_asm", package_name="lite_moveit"
    ).to_moveit_configs()

    # 1. 静态虚拟关节 TF
    static_tfs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(moveit_config.package_path / "launch/static_virtual_joint_tfs.launch.py")
        )
    )

    # 2. Move Group（自带 joint_states remap，见 _build_move_group）
    move_group = _build_move_group(
        moveit_config, LaunchConfiguration('joint_states_topic')
    )

    # 3. 读取 SRDF
    srdf_path = moveit_config.package_path / 'config' / 'lite_000_asm.srdf'
    srdf_content = srdf_path.read_text()

    # 4. 读取 kinematics.yaml
    kinematics_path = moveit_config.package_path / 'config' / 'kinematics.yaml'
    import yaml
    kinematics_content = yaml.safe_load(kinematics_path.read_text())

    # 5. RViz（显式传递所有必要参数）
    rviz_config = str(moveit_config.package_path / "config" / "moveit.rviz")
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[
            moveit_config.to_dict(),  # 包含 robot_description, kinematics 等
            {
                'robot_description_semantic': srdf_content,
            }
        ],
    )

    # 6. 桥接节点（可选）
    bridge_node = Node(
        package='lite_moveit',
        executable='moveit_remote_policy_bridge',
        name='moveit_remote_policy_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_bridge')),
        parameters=[{
            'stiffness': LaunchConfiguration('stiffness'),
            'damping': LaunchConfiguration('damping'),
            'enable_torque_compensation': LaunchConfiguration('enable_torque_compensation'),
            'torque_compensation_scale': LaunchConfiguration('torque_compensation_scale'),
            'integral_gain': LaunchConfiguration('integral_gain'),
            'torque_params_path': LaunchConfiguration('torque_params_path'),
            # --- 与 lite_bridge 对接 ---
            'source_name': LaunchConfiguration('source_name'),
            'control_mode': LaunchConfiguration('control_mode'),
            'command_topic': LaunchConfiguration('command_topic'),
            'mode': LaunchConfiguration('mode'),
            # 与 move_group 用同一个（补全后的）关节状态话题
            'joint_states_topic': LaunchConfiguration('joint_states_topic'),
        }],
    )

    return LaunchDescription([
        start_bridge_arg,
        stiffness_arg,
        damping_arg,
        enable_tc_arg,
        tc_scale_arg,
        integral_gain_arg,
        torque_params_path_arg,
        source_name_arg,
        control_mode_arg,
        command_topic_arg,
        mode_arg,
        joint_states_topic_arg,
        static_tfs,
        move_group,
        rviz_node,
        bridge_node,
    ])
