"""
一键启动：lite_bringup（**默认参数**，只额外传桥必需的两项）→ MoveIt + 桥接节点

流程（★ 模式切换改为**手动**）
==============================
  1. `lite_bringup.launch.py`
       显式传这几个参数：
         · `source_names:=<source_name>`          ← 授权 MoveIt 数据源
         · `controller:=<controller>`             ← 控制器集合（**从 control_mode 派生**）
         · `initial_control_mode:=<control_mode>` ← 桥的协议（mit / csp）
         · `mode:=<mode>`                         ← 关节集合
       其余**全部用 lite_bringup 自己的默认值**
       （can0/can1、calibration.yaml、start_rviz=false …；
         start_mode_manager 已是**兼容占位**，不再起作用）。

       ⚠️ `controller` 与 `control_mode` **必须成对**（lite_bringup 会校验）：
            control_mode=mit → controller=modes（加载 MIT 组 0/1/2）
            control_mode=csp → controller=csp  （加载 CSP 组 5/6/7）
          本 launch 让 `controller` **默认从 `control_mode` 派生** → 不可能配岔。
          也可显式覆盖（如 `controller:=trajectory`），此时自负一致性。
       启动后默认停在 **ZERO_TORQUE（松劲）**，不会自动切模式。
  2. 延迟 `moveit_delay`（默认 10 s）→ `lite_moveit/demo.launch.py`
       `start_bridge:=true` → moveit_remote_policy_bridge
       把 FollowJointTrajectory → robstride_msgs/MotionCommand
       → `/lite_bridge/motion_command`
  3. **不再自动切 HOLD / TRACKING**。需要时手动：

       # 锁在当前位置（硬）
       ros2 service call /lite_bridge/set_mode \
           robstride_msgs/srv/SetMode "{protocol: 255, mode: 1}"
       # 使能跟随（软塌窗口：首条轨迹前会松劲，见下）
       ros2 service call /lite_bridge/set_mode \
           robstride_msgs/srv/SetMode "{protocol: 255, mode: 2}"
       # 随时变软（安全方向）
       ros2 service call /lite_bridge/set_mode \
           robstride_msgs/srv/SetMode "{protocol: 255, mode: 0}"
       # 核对（**同步**语义：ok:true 即已切换完成，回读字段是切换后的值）
       ros2 service call /lite_bridge/get_status \
           robstride_msgs/srv/GetBridgeStatus "{}"

       # 模式入口已从 mode_manager 迁到 lite_bridge（前者已被整体吸收）。
       # protocol: 255 = KEEP（沿用当前协议，推荐）。

⚠️ control_mode:=csp 与 MIT 的差别
-----------------------------------
`control_mode:=csp`（本 launch 会自动把 `controller` 也派生成 `csp`）时：

  · 启动后停在 **mode5 CSP_MONITOR**（**不使能**、真正 0 力矩、仍可读姿态）
    —— 不是 MIT 那边的 ZERO_TORQUE。此档机械臂是**自由**的。
  · 要到"跟随 MoveIt"必须**分两步**（CSP 组的 TRACKING 只能从 HOLD 进）：

        ros2 service call /lite_bridge/set_mode \
            robstride_msgs/srv/SetMode "{protocol: 255, mode: 6}"   # CSP_HOLD
        ros2 service call /lite_bridge/set_mode \
            robstride_msgs/srv/SetMode "{protocol: 255, mode: 7}"   # CSP_TRACKING

  · **CSP 没有"软塌窗口"**（与 MIT 不同）：
        MIT TRACKING（mode2）：无命令 → tracking_controller 写**全 0** → τ=0
                              → **松劲、受重力下坠**
        CSP TRACKING（mode7）：无命令 → csp_tracking_controller **持续发送
                              锁存目标** → **顶住**，不会下坠
        （锁存目标：激活时 = 激活瞬间实测位置 ⇒ 与 mode6 **同姿态**；
          收到命令后 = 最后那条命令的 position。）
    所以 CSP 下"6 → 7 之后、首条轨迹之前"机械臂**停住不动**，
    不需要"发第一条轨迹前先扶住"。仍建议吊装/有人在场，但不再是硬要求。

⚠️ 为什么这两个参数不能省
--------------------------
桥的两个默认值都指向"命令到不了电机"：
  · `initial_control_mode` 默认 `auto`，在 `controller:=modes` 下等于 mit ——
    这条其实够用，但显式写出来让"命令走哪条通路"在 launch 里可见。
  · `source_names` 默认 **空** → 桥不认任何源，**每条** MoveIt 命令被丢弃，
    并打 `丢弃未知数据源 'moveit' 的命令`（节点不崩，极易误判成链路正常）。
`source_name` 同时驱动"桥白名单"与"MoveIt 节点的 source 字段"，
两者由**同一个 launch 参数**驱动 —— 不可能配岔。

⚠️⚠️ 上电安全：TRACKING 之后的"软塌窗口"（**只影响 MIT**）
------------------------------------------------------------
`tracking_controller`（MIT，mode2）的 `stale_command_policy` 默认 `passive`：
**没有命令（或超过 100 ms 未更新）就写全 0 = 松劲**，且激活时清空历史命令。
而 MoveIt 桥接节点**只在执行轨迹时（及轨迹结束后的 hold 阶段）才发命令**：

    切到 MIT TRACKING（mode2）的那一刻起，到发出第一条 MoveIt 轨迹之前，
    机械臂是**松的**（会因自重下坠）。

⚠️ **CSP TRACKING（mode7）没有这个问题** —— 它**锁存目标并持续发送**，
   无命令时**顶住**（激活时锁存的就是 mode6 的姿态）。

所以走 MIT 时请**手动**切 TRACKING 并确保现场有人扶臂 / 已吊装；
走 CSP 时 6 → 7 是安全的（机械臂会停在 mode6 的姿态上）。

用法
====
  # 默认：起 bringup + MoveIt，停在 ZERO_TORQUE，模式手动切
  ros2 launch lite_moveit one_click.launch.py

  # 装夹爪的机型：同一个 mode 同时下发 bringup 与 MoveIt 桥接节点
  ros2 launch lite_moveit one_click.launch.py mode:=arms_grippers

  # 硬件已由别处拉起（例如要空跑 / 自定义 bringup 参数），只补 MoveIt 侧。
  # ⚠️ 此时 source_names / initial_control_mode / mode 要自己保证两边一致。
  ros2 launch lite_moveit one_click.launch.py skip_bringup:=true

⚠️ 关节集合（mode）必须两边一致
--------------------------------
`mode` 同时传给 `lite_bringup`（决定桥的 `canonical_joints` 与驱动使能哪些电机）
和 MoveIt 桥接节点（决定它发几个关节）。**少发**会被桥**整帧拒绝**：
MoveIt 发 14 而桥认 16 时，桥的完整性校验（安全不变量 I5）会拒帧并在日志里
列出缺哪些关节 —— 早期版本是静默给未触及关节填 `position=0` + `Kp=20`，
那才会让关节真的往零位跑。
本 launch 用**同一个 `mode` 参数**驱动两边 → 不可能配岔。
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ================================================================
    # 参数
    # ================================================================
    # --- 跳过 bringup（硬件已由别处拉起，只补 MoveIt 侧）---
    skip_bringup_arg = DeclareLaunchArgument(
        'skip_bringup', default_value='false',
        description='true = 不启动 lite_bringup（硬件与桥已由别处拉起），'
                    '本 launch 只起 MoveIt + 桥接节点。'
                    '⚠️ 此时 source_names / initial_control_mode / mode 由你自己保证。',
    )
    # --- 空跑（默认值 = lite_bringup 的默认值）---
    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware', default_value='false',
        description='true = mock 硬件空跑（不碰 CAN）。默认 false，与 lite_bringup '
                    '默认一致。⚠️ 想试 arms_grippers 但实机没装夹爪时用它空跑验证。',
    )

    # --- 桥对接（★ 传给桥的就这两个）---
    source_name_arg = DeclareLaunchArgument(
        'source_name', default_value='moveit',
        description='数据源名。同时用作：桥的 source_names 白名单，'
                    '以及 MoveIt 桥接节点 MotionCommand.source 字段。'
                    '由同一个参数驱动 → 不可能配岔。',
    )
    control_mode_arg = DeclareLaunchArgument(
        'control_mode', default_value='mit',
        description="桥的协议开关（'mit' / 'csp'）。传给 lite_bringup 的 "
                    'initial_control_mode；同时传给 MoveIt 桥接节点。'
                    '★ 现在 mit / csp **都可选**：本 launch 会据此自动派生 '
                    '`controller`（mit→modes，csp→csp）并透传给 lite_bringup。 '
                    '⚠️ 选 csp 时启动后停在 mode5（不使能），需手动 6→7 才跟随。',
    )

    # --- 控制器集合（★ 默认从 control_mode 派生，可显式覆盖）---
    #
    # ⚠️ lite_bringup 对 (controller, control_mode) 有**双向校验**：
    #    错配会直接 raise（响亮失败，不是静默失效）。原实现不透传 controller，
    #    于是 `control_mode:=csp` 必然撞上校验 —— 本参数就是修这个。
    #
    # 用 PythonExpression 而不是在 Python 里算：本 launch 是普通的
    # generate_launch_description()（没有 OpaqueFunction），拿不到 context
    # 去 perform()；而 PythonExpression 是**运行时**求值的 substitution。
    controller_arg = DeclareLaunchArgument(
        'controller', 
        default_value=PythonExpression([
            "'csp' if '", LaunchConfiguration('control_mode'),
            "' == 'csp' else 'modes'",
        ]),
        description='控制器集合：modes（MIT 组 0/1/2）/ csp（CSP 组 5/6/7）/ '
                    'trajectory（JTC）。**默认从 control_mode 派生**'
                    '（mit→modes，csp→csp）→ 不可能配岔。'
                    '显式覆盖时请自负与 control_mode 的一致性。',
    )

    # --- 关节集合（同时驱动 bringup 与桥接节点）---
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='arms',
        description='关节集合：arms（14，双臂）/ arms_grippers（16，含夹爪）。'
                    '★ 同一个值**同时**传给 lite_bringup（桥的 canonical_joints '
                    '与驱动使能哪些电机）和 MoveIt 桥接节点 → 两边必然一致。'
                    '⚠️ 实机没装夹爪时必须用 arms —— 否则会多导出 2 个没有反馈的 '
                    '夹爪关节，/joint_states 里永远是 0。'
                    '（注：驱动不校验使能应答，所以**不会**导致硬件 activate 失败；'
                    '那是个被订正过的说法。）',
    )
    command_topic_arg = DeclareLaunchArgument(
        'command_topic', default_value='/lite_bridge/motion_command',
        description='MotionCommand 下发话题（桥的 ~/motion_command）。',
    )
    joint_states_topic_arg = DeclareLaunchArgument(
        'joint_states_topic', default_value='/lite_js/joint_states',
        description='move_group / 桥接节点订阅的关节状态话题。默认取 lite_bringup '
                    '补全夹爪后的 /lite_js/joint_states（原始 /joint_states 只有 '
                    '14 个臂关节，move_group 会一直报 Missing left_gripper/right_gripper）。'
                    '⚠️ 若 lite_bringup 关了 complete_joint_states，请设 /joint_states。',
    )
    stiffness_arg = DeclareLaunchArgument(
        'stiffness', default_value='20.0',
        description='MoveIt 桥接节点的 MIT 刚度 Kp。'
                    '实测 Kp=20 足以顶住自重（与参考工程一致）。'
                    '⚠️ 太大（如 500）会在 1 rad 偏差下直接给满力矩。',
    )
    damping_arg = DeclareLaunchArgument(
        'damping', default_value='2.0',
        description='MoveIt 桥接节点的 MIT 阻尼 Kd（一般 0.5~2.0）。',
    )
    enable_tc_arg = DeclareLaunchArgument(
        'enable_torque_compensation', default_value='false',
        description='启用力矩前馈补偿（摩擦 + Pinocchio 动力学）。'
                    '需要 identified_params.json。',
    )
    tc_scale_arg = DeclareLaunchArgument(
        'torque_compensation_scale', default_value='1.0',
        description='力矩补偿缩放（>1 增强，<1 减弱）。',
    )
    integral_gain_arg = DeclareLaunchArgument(
        'integral_gain', default_value='0.0',
        description='积分增益（0 = 关）。用于消除稳态跟踪误差。',
    )
    torque_params_arg = DeclareLaunchArgument(
        'torque_params_path', default_value='',
        description='identified_params.json 路径。留空 = 自动查找。',
    )

    # --- 时序 ---
    moveit_delay_arg = DeclareLaunchArgument(
        'moveit_delay', default_value='10.0',
        description='lite_bringup 启动后等多少秒再拉起 MoveIt + 桥接节点。',
    )

    # ================================================================
    # 1. 硬件 + 三态控制器 + 桥（lite_bringup，默认参数）
    #    ⚠️ 只覆盖桥必需的两个参数，其余交给 lite_bringup 的默认值。
    # ================================================================
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('lite_bringup'), 'launch', 'lite_bringup.launch.py'
            ])
        ),
        launch_arguments={
            # ★ 授权数据源。空 = 桥丢弃所有上层命令。
            'source_names': LaunchConfiguration('source_name'),
            # ★ 桥的协议开关。显式写出，让"命令走哪条通路"在 launch 里可见。
            'initial_control_mode': LaunchConfiguration('control_mode'),
            # ★ 控制器集合。**默认从 control_mode 派生**（见 controller_arg）；
            #    必须与 control_mode 成对，否则 lite_bringup 直接 raise。
            'controller': LaunchConfiguration('controller'),
            # ★ 关节集合：必须与 MoveIt 桥接节点逐字一致（已由同一参数保证）。
            'mode': LaunchConfiguration('mode'),
            # 空跑（默认 false = lite_bringup 默认）
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('skip_bringup')),
    )

    # ================================================================
    # 2. MoveIt + 桥接节点
    #    （模式切换不再由本 launch 触发，全部手动）
    # ================================================================
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('lite_moveit'), 'launch', 'demo.launch.py'
            ])
        ),
        launch_arguments={
            'start_bridge': 'true',
            # --- 与桥对接 ---
            'source_name': LaunchConfiguration('source_name'),
            'control_mode': LaunchConfiguration('control_mode'),
            'command_topic': LaunchConfiguration('command_topic'),
            'joint_states_topic': LaunchConfiguration('joint_states_topic'),
            'mode': LaunchConfiguration('mode'),
            # --- 桥接节点算法参数 ---
            'stiffness': LaunchConfiguration('stiffness'),
            'damping': LaunchConfiguration('damping'),
            'enable_torque_compensation': LaunchConfiguration('enable_torque_compensation'),
            'torque_compensation_scale': LaunchConfiguration('torque_compensation_scale'),
            'integral_gain': LaunchConfiguration('integral_gain'),
            'torque_params_path': LaunchConfiguration('torque_params_path'),
        }.items(),
    )

    moveit_timer = TimerAction(
        period=LaunchConfiguration('moveit_delay'),
        actions=[
            LogInfo(msg='[one_click] 启动 MoveIt + moveit_remote_policy_bridge ...'),
            moveit_launch,
        ],
    )

    # ================================================================
    # 组装
    # ================================================================
    return LaunchDescription([
        skip_bringup_arg,
        use_fake_hardware_arg,
        source_name_arg,
        control_mode_arg,
        controller_arg,
        mode_arg,
        command_topic_arg,
        joint_states_topic_arg,
        stiffness_arg,
        damping_arg,
        enable_tc_arg,
        tc_scale_arg,
        integral_gain_arg,
        torque_params_arg,
        moveit_delay_arg,

        LogInfo(msg='========================================'),
        LogInfo(msg='[one_click] lite_bringup(默认参数) + MoveIt + 桥接节点'),
        LogInfo(msg='[one_click] 模式切换已改为**手动**（入口：/lite_bridge/set_mode）：'),
        LogInfo(msg='[one_click]   [MIT 组] 松劲  : {protocol: 255, mode: 0}'),
        LogInfo(msg='[one_click]   [MIT 组] HOLD  : {protocol: 255, mode: 1}'),
        LogInfo(msg='[one_click]   [MIT 组] 跟踪  : {protocol: 255, mode: 2}（先经 1）'),
        LogInfo(msg='[one_click]   [CSP 组] 只读  : {protocol: 255, mode: 5}（不使能）'),
        LogInfo(msg='[one_click]   [CSP 组] HOLD  : {protocol: 255, mode: 6}'),
        LogInfo(msg='[one_click]   [CSP 组] 跟踪  : {protocol: 255, mode: 7}（先经 6）'),
        LogInfo(msg='[one_click]   软塌窗口：首条轨迹前机械臂是**软**的，务必扶住/吊装'),
        LogInfo(msg='[one_click]   回读        : /lite_bridge/get_status'),
        LogInfo(msg='========================================'),

        bringup_launch,
        moveit_timer,
    ])
