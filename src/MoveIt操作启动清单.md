# Lite 双臂 + MoveIt 启动操作清单（SOP）

> 面向本工作区 `/data/lite_motor_ws`（`lite_bringup` / `lite_controller` /
> `lite_bridge` / `lite_driver` + `lite_moveit`）。
>
> ⚠️ 本清单**已改造**：旧版面向参考工程 `lite_ros2`
> （`bar_bringup_lite` + `/bar/mode/*` FSM + `topic_tools` 中继），
> 那些接口在本工作区**不存在**。对应关系：
>
> | 旧（lite_ros2）                        | 现（本工作区）                                  |
> | -------------------------------------- | ----------------------------------------------- |
> | `bar_bringup_lite real.launch.py`      | `lite_bringup lite_bringup.launch.py`            |
> | `/bar/mode/damp` `/load`               | `lite_bridge/set_mode` 的三态（ZERO_TORQUE / HOLD）|
> | `/bar/mode/start_remote`               | `~/set_mode {mode: 2}` = TRACKING                |
> | `relay /lite/joint_states /joint_states` | **不需要**，驱动直接发 `/joint_states`          |
> | `bar_msgs/MITCommand` → `/lite/mit_command` | `robstride_msgs/MotionCommand` → `/lite_bridge/motion_command` |
> | `remote_policy_controller`             | `tracking_controller`（**经** `lite_bridge` 转发）|

---

## 阶段 0：每次开机（环境 + CAN）

### 0.1 环境

```bash
export ROS_HOME=/data/lite_motor_ws/roslog   # ⚠️ 不设节点可能静默退出
source /opt/ros/jazzy/setup.bash
cd /data/lite_motor_ws && source install/setup.bash
source /data/lite_data_sources/install/setup.bash   # MoveIt 侧（lite_moveit）

# ⚠️ 若 `ros2 service call` 能发现服务但**收不到回复**（一直卡住），
#    是 Fast DDS 共享内存传输的问题 —— 两个终端都设下面之一（实测有效）：
# export FASTDDS_BUILTIN_TRANSPORTS=UDPv4      # 保留 Fast DDS，禁 SHM
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp # 或直接换 CycloneDDS
#    详见文末「`ros2 service call` 卡住 / 无响应排查」。
```

### 0.2 CAN 接口

```bash
ls /sys/class/net/            # 找 canX
for c in can0 can1; do sudo ip link set $c up type can bitrate 1000000; done
ip -br link show can0 can1    # 期望 UP
```

> ⚠️ **接口 DOWN 是最常见的"电机没反应"原因**：`write()` 直接返回
> `ENOBUFS` / `ENETDOWN`，一帧都发不出去。
> ⚠️ 适配器重新插拔会换接口名（can2/can3…），别假设左臂一定是 can0。

### 0.3 确认总线通了（`tx_failed` 必须为 0）

```bash
ros2 run lite_driver robstride_discover --iface can0   # 左臂应为 id 11~17
ros2 run lite_driver robstride_discover --iface can1   # 右臂应为 id 21~27
```

### 0.4 工作区已构建且是最新的

```bash
cd /data/lite_motor_ws && colcon build --symlink-install
```

> ⚠️ 改过 `lite_bringup` 的 launch/config 后**必须重建**，否则跑的是
> `install/` 里的旧副本 —— 曾经因此把"参数改了却不生效"误判成硬件问题。

---

## 阶段 1：一键启动（推荐）

```bash
ros2 launch lite_moveit one_click.launch.py
```

它按顺序做：

1. `lite_bringup`（**默认参数**，只额外传桥必需的
   `source_names:=moveit` + `initial_control_mode:=mit`；`controller` 用其默认
   `modes`）—— 启动默认停在 **ZERO_TORQUE（松劲）**
2. 延迟 `moveit_delay`（默认 10 s）→ MoveIt `move_group` + RViz + 桥接节点
3. **停在这里。三态切换全部手动进行**（见阶段 2）

> ⚠️ 因为不再自动切 HOLD，起来后默认是 ZERO_TORQUE —— 机械臂是**软的**。
> 确认姿态 / 放手之前，请先手动切 HOLD 锁住：
> `ros2 service call /lite_bridge/set_mode \
     robstride_msgs/srv/SetMode "{protocol: 255, mode: 1}"`

**验证**（都带 `timeout`，避免服务不在时干等）：

```bash
timeout 10 ros2 control list_controllers
# 期望 4 项：zero_torque_controller active，hold/tracking inactive，
#           joint_state_broadcaster active（切 HOLD 后 active 变成 hold_controller）

timeout 10 ros2 service call /lite_bridge/get_status robstride_msgs/srv/GetBridgeStatus "{}"
# 期望 message 是「当前模式：ZERO_TORQUE」（切 HOLD 后变 HOLD）

timeout 10 ros2 action info /remote_policy_controller/follow_joint_trajectory
# 期望 Action servers: 1 (moveit_remote_policy_bridge)
```

> ⚠️ **这些命令"卡住/没反应"= 服务不存在，也就是 bringup 没在跑**（`ros2 service call`
> 找不到服务时会无限等待，不是模式问题）。先确认进程：
>
> ```bash
> timeout 5 ros2 node list
> timeout 5 ros2 service list | grep -E "lite_bridge|controller_manager"
> ps -ef | grep -E "ros2_control_node|lite_bridge|move_group" | grep -v grep
> ```
>
> 一条都没有 → launch 已经退出/被打断。重跑并**保持 launch 终端可见**，
> 它会打印是哪个进程退出（`process has died`）。常见退出原因：
> `ROS_HOME` 没设或不可写（见阶段 0.1）、CAN 未 up、`mode` 与实机不符
> （没装夹爪却用了 `arms_grippers`）。

### 常用变体

```bash
# 装夹爪的机型（16 关节，**实机必须真的装了夹爪**）
ros2 launch lite_moveit one_click.launch.py mode:=arms_grippers

# 实机没装夹爪、只空跑验证 arms_grippers 链路
ros2 launch lite_moveit one_click.launch.py use_fake_hardware:=true mode:=arms_grippers

# 硬件已由别处拉起（要用 calibration_file / can_interface_* / controller 等），只补 MoveIt 侧
ros2 launch lite_bringup lite_bringup.launch.py use_fake_hardware:=true
ros2 launch lite_moveit one_click.launch.py skip_bringup:=true

# ⚠️ `mode` 由**同一个参数**同时下发 bringup 与 MoveIt，两边必然一致；
#    但 can_interface_* / calibration_file / controller 仍走 bringup 默认，
#    要改请用上面的 skip_bringup 两步法。
```

> **模式切不过去？** 先看 `ros2 service call /lite_bridge/get_status robstride_msgs/srv/GetBridgeStatus "{}"`
> 的 `已加载：...=是/否`，再看 `ros2 control list_controllers`。
> 若 `hold_controller` 不在列表里，说明 bringup 的 spawner 没加载成功
> （只有 `controller:=modes` 才会加载 hold/tracking）——详见排障表。

---

## 阶段 2：切到 TRACKING（使能控制）

```bash
ros2 service call /lite_bridge/set_mode \
     robstride_msgs/srv/SetMode "{protocol: 255, mode: 2}"
```

⚠️ **TRACKING 只能从 HOLD 进入**（迁移守则；`force: true` 可绕过，不推荐）。
✅ `set_mode` 返回 `ok: true` 表示 **`controller_manager` 的 STRICT 切换已经成功**，
响应里的 `actual_mode` / `controller_active` 都是**切换后**的实际值 ——
不再需要"看日志确认"或"稍后再查"。想核实时再调一次：

```bash
ros2 service call /lite_bridge/get_status robstride_msgs/srv/GetBridgeStatus "{}"
```

### ⚠️⚠️ 切过去之后：MIT 有"软塌窗口"，**CSP 没有**

**两组的行为不同，别套用。**

#### MIT TRACKING（mode2）—— 有软塌窗口

`tracking_controller` 的 `stale_command_policy` 默认是 `passive`：
**没有命令（或超过 100 ms 没更新）就写全 0 = 松劲**，
而且**激活时会清空历史命令** —— 所以配成 `hold` 也救不了"首帧之前"。

而 MoveIt 桥接节点**只在执行轨迹时（以及轨迹结束后的 hold 阶段）才发命令**：

| 时刻                          | 谁在发命令           | 机械臂状态       |
| ----------------------------- | -------------------- | ---------------- |
| 切到 TRACKING 后、首条轨迹前  | 没有人               | **松劲 → 下坠**  |
| 轨迹执行中                    | 桥接节点（带 Kp/Kd） | 跟随轨迹         |
| 轨迹结束后                    | 桥接节点（hold 目标）| 主动保持终点姿态 |
| RViz 点 Stop（cancel）        | 停止发命令           | **又松了**       |

#### CSP TRACKING（mode7）—— **没有**软塌窗口（本版修订）

`csp_tracking_controller` 改为**锁存一个目标并持续发送**它：

| 时刻                          | 写什么                      | 机械臂状态        |
| ----------------------------- | --------------------------- | ----------------- |
| 切到 mode7 后、首条轨迹前     | 锁存 = 激活瞬间实测位置     | **保持 mode6 姿态** |
| 轨迹执行中                    | 桥接节点给的 position        | 跟随轨迹          |
| 轨迹结束后 / RViz 点 Stop     | **继续写锁存值**             | **顶住，不塌**    |
| 数据源整个掉线                | **继续写锁存值**             | **顶住，不塌**    |

⚠️ 上一版 CSP 写的是"每拍实测位置"（目标≈实际 → 松劲），会在数据源掉线时塌掉
—— 那正是本版修掉的。**`stale_command_timeout_ms` 现在不再影响输出**，
只用来触发一条陈旧告警。

> 想要"真正 0 力矩"请切 **mode5**（会失能电机）；
> 想要"锁在进档姿态"请用 **mode6**。

#### 所以

- **走 MIT**：先手动停在 HOLD（硬的），确认姿态稳定、行程内无人无遮挡后，
  再手动切 TRACKING；切完让人扶住机械臂 / 立刻发第一条低速轨迹。
- **走 CSP**：6 → 7 是安全的（机械臂停在 mode6 的姿态上）。
- 变软随时可用：MIT `set_mode {mode: 0}` / CSP `set_mode {mode: 5}`（都是安全方向）。

---

### 数据源切换（人工接管 / 归还）

桥支持多个数据源（`moveit` / `teleop` / `llm` / `replay`）按优先级自动仲裁。
**人工干预**用 `/lite_bridge/set_source`：

```bash
# ① 让 teleop 接管 —— 它一动就归它，一停就自动还给 moveit（**不软塌**）
ros2 service call /lite_bridge/set_source \
    robstride_msgs/srv/SetSource "{mode: 2, source: 'teleop'}"     # 2 = HANDOVER

# ② 锁死用 moveit（它超时 → SAFE_HOLD，不让给别人）
ros2 service call /lite_bridge/set_source \
    robstride_msgs/srv/SetSource "{mode: 1, source: 'moveit'}"     # 1 = LOCK

# ③ 让大模型跑 30 秒，然后自动还回去
ros2 service call /lite_bridge/set_source \
    robstride_msgs/srv/SetSource "{mode: 3, source: 'llm', hold_ms: 30000}"  # 3 = PREEMPT

# ④ 解除干预，回自动仲裁
ros2 service call /lite_bridge/set_source \
    robstride_msgs/srv/SetSource "{mode: 0}"                       # 0 = AUTO
```

| 模式 | 值 | 受干预的源失效时 |
|---|---|---|
| `AUTO` | 0 | ——（解除干预，回自动仲裁）|
| `LOCK` | 1 | **SAFE_HOLD** —— 不让给别人，臂会**软掉** |
| `HANDOVER` | 2 | **回自动仲裁** —— 自动还给其它源 ★ |
| `PREEMPT` | 3 | 窗口内同 LOCK；`hold_ms` 到期后自动回 AUTO |

> ⚠️ **人工接管请用 `HANDOVER`，不要用 `LOCK`**。
> 用 `LOCK` 时操作员一松手，源超时 → 桥进 SAFE_HOLD → 机械臂**软掉下坠**。
> 除非你确实要"停住不动、谁也不许动"，否则都用 `HANDOVER`。

> ⚠️ 干预**优先于**优先级/独占：受干预的源只要在发命令就一直持有控制权，
> 哪怕有更高优先级且 `exclusive` 的源在抢。想让它重新参与竞争就 `AUTO`。

> 📌 旧的 `~/set_active_source` 保留为**兼容别名**，但它只等价于 `LOCK` / `AUTO`
> —— **没有 HANDOVER**。新代码请用 `~/set_source`。

> 📌 响应里同时给出 `available_sources`（全部已注册源）与 `source_online`
> （目标源现在有没有新鲜命令）—— 源名拼错时不用再查 `get_status`。

---

## 阶段 3：RViz 中执行轨迹

**RViz 中确认**：

- Fixed Frame: `world_root`
- RobotModel 显示与真实机器人姿态一致
- MotionPlanning → Planning Group: `left_arm_group` / `right_arm_group`

| 步骤 | 操作 | 注意 |
| :--- | :--- | :--- |
| 1 | 选 `left_arm_group` 或 `right_arm_group` | 不要选夹爪 group（会走另一个规划组，且夹爪未必装） |
| 2 | 拖动末端到目标位置 | 确认姿态是**绿色**（无碰撞） |
| 3 | 点击 **Plan** | 等 "Plan Successful" |
| 4 | 点击 **Execute** | 观察桥接节点日志 |
| 5 | 观察真机运动 | 要停就点 **Stop**（⚠️ 之后机械臂会松劲，见上表） |

> ⚠️ 不要点 **Plan and Execute**：分两步更容易在动之前发现问题。

**建议先用小幅度、低速**：`joint_limits.yaml` 的
`has_acceleration_limits` 与规划时间决定了运动快慢；
`stiffness` / `damping` 由 launch 参数控制（默认 20.0 / 2.0）。

**监控**：

```bash
ros2 topic hz /lite_bridge/motion_command      # 桥接节点在发吗（轨迹期间 ~50 Hz）
ros2 topic echo /lite_bridge/source_status     # 谁持有控制权、各源是否超时
ros2 topic echo /tracking_controller/command   # 桥发给控制器的全量命令
ros2 topic hz /lite_js/joint_states              # 状态反馈（MoveIt/桥 用的是这个补全后话题）
ros2 topic hz /joint_states                     # 驱动原始反馈（只有 14 个臂关节）
```

---

## 手动分步启动（一键启动出问题时用）

```bash
# 终端 1：硬件 + 三态控制器 + 桥
ros2 launch lite_bringup lite_bringup.launch.py \
    mode:=arms \
    can_interface_left:=can0 can_interface_right:=can1 \
    initial_control_mode:=mit \
    source_names:=moveit

# 终端 2：HOLD，然后 TRACKING
ros2 service call /lite_bridge/set_mode \
     robstride_msgs/srv/SetMode "{protocol: 255, mode: 1}"
ros2 service call /lite_bridge/get_status robstride_msgs/srv/GetBridgeStatus "{}"
ros2 service call /lite_bridge/set_mode \
     robstride_msgs/srv/SetMode "{protocol: 255, mode: 2}"

# 终端 3：MoveIt + 桥接节点
ros2 launch lite_moveit demo.launch.py start_bridge:=true mode:=arms
```

⚠️ **`source_names:=moveit` 和 `initial_control_mode:=mit` 不能省**：
桥的默认值是 `source_names` **空**、`initial_control_mode` **csp**，
两个默认值都会让命令**静默消失**（桥日志看起来一切正常，电机就是不动）。

如果不想改 `lite_bringup` 的启动命令，也可以在运行期手工把桥切成 MIT：

```bash
ros2 service call /lite_bridge/set_control_mode \
    robstride_msgs/srv/SetControlMode "{mode: 1}"
```

但 `source_names` **没有运行期接口**（只在启动时读一次），
所以空列表时**只能重启桥** —— 这是它最容易被忽略的坑。

---

## 关机 / 停止顺序

```
1. RViz 里点 Stop（如有轨迹在执行）
2. 先变软，避免下次上电带着奇怪的记账：
     ros2 service call /lite_bridge/set_mode \
     robstride_msgs/srv/SetMode "{protocol: 255, mode: 0}"
3. Ctrl+C 关 MoveIt（终端 3）
4. Ctrl+C 关 bringup（终端 1）—— 硬件 deactivate 会发 Disable 帧，电机进入安全态
```

---

## 故障排查速查表

| 现象 | 根因 / 排查 |
| :--- | :--- |
| `can0: not up/running` | `sudo ip link set can0 up type can bitrate 1000000` |
| 电机完全没反应，`tx_failed` 增长 | CAN 接口 DOWN，或适配器换了接口名（跑 `robstride_discover`） |
| 切模式时报"不能从 ZERO_TORQUE 直接到 TRACKING" | 迁移守则，先 `{mode: 1}` 再到 `{mode: 2}` |
| `set_mode` 返回 `ok=false` | `message` 会分类：跨协议 / 自相矛盾 / 组内守卫 / 控制器未加载。跨协议还给 `required_launch_args` |
| `set_mode` 返回 `ok=true` 但 `controller_active=false` | 记账成功但控制器没跑起来；查 `*_loaded` 字段与 controller_manager 日志 |
| 日志报 `目标控制器 'hold_controller' **未加载**，无法切换` | bringup 启动时没把三态控制器加载全。先 `ros2 control list_controllers` + `get_mode` 看 `hold_loaded`。只有 `controller:=modes`（默认）才会加载 hold/tracking；若缺，说明 spawner 失败或 bringup 不是 modes。重启 bringup 并确认日志有 `Loaded hold_controller` / `Loaded tracking_controller` 后再切模式 |
| `丢弃未知数据源 'moveit' 的命令` | 桥的 `source_names` 为空 → 重启桥并传 `source_names:=moveit` |
| 桥日志正常但电机不动 | ①`source_names` 没授权；②桥停在 `csp`；③三态不是 TRACKING |
| 命令被拒、提示"请先调用 set_control_mode" | 消息 `mode` 与桥的协议模式不一致 |
| （MIT）切 TRACKING 后机械臂立刻软下来 | **正常**（MIT 的软塌窗口）。发第一条轨迹，或回 HOLD |
| （CSP）切 mode7 后机械臂软下来 | ❌ **不该发生** —— CSP 现在锁存目标并顶住。查 `/csp_tracking_controller` 是否 active、日志有没有"拒绝激活" |
| 命令被整帧拒绝、日志列出"缺失：xxx" | `mode` 两边不一致（`lite_moveit` 比 `lite_bringup` 少）→ 桥的完整性校验（I5）拒帧。让两边一致 |
| 某个关节往零位跑 | 早期版本的桥会静默给未触及关节填 `Kp=20` + `position=0`。**现已被 I5 整帧拒绝取代**；若仍看到，说明跑的是旧桥二进制，重新 `colcon build` |
| 某关节行为怪异/不受控 | 关节状态话题里缺该关节 → 桥接节点把它当 0。查电机是否掉线；确认用的是 `/lite_js/joint_states` |
| `move_group` 一直报 `Missing left_gripper, right_gripper` | 它订阅了 `/joint_states`（只有 14 臂关节）。本包默认已 remap 到补全后的 `/lite_js/joint_states`；若你关了 lite_bringup 的 `complete_joint_states`，需用 `joint_states_topic:=/joint_states` 或打开补全 |
| `Plan` 失败 | 目标不可达 / 有碰撞 / `joint_limits.yaml` 限制 |
| `Execute` 成功但机器人不动 | `ros2 topic hz /lite_bridge/motion_command`；确认三态是 TRACKING |
| Execute 后机器人软塌 | 桥接节点是旧版本（轨迹结束后没进 hold 模式） |
| 运动太猛/冲击 | 降 `stiffness`（如 `stiffness:=8.0`），升 `damping` |
| 改了 launch/config 不生效 | 忘了 `colcon build`，跑的是 `install/` 旧副本 |
| 节点起来又莫名消失 | `ROS_HOME` 没设或不可写 → 节点静默退出。启动前 `export ROS_HOME=/data/lite_motor_ws/roslog` |
| `ros2 service call` 打印 `requester: making request` 后**一直没有 response** | 服务端点**被发现**但回复回不来。头号原因是**上一次 launch 的僵尸/孤儿进程**仍持有 DDS 端点（Ctrl+C 没清干净，`ros2_control_node` 的 CAN 线程常不响应 SIGINT）。见下方"卡住/无响应排查" |
| `ros2 service call` 打印 `waiting for service to become available...` 后卡住 | 服务真的不存在 —— 节点没起或已退出。同上排查 |

---

## `ros2 service call` 卡住 / 无响应排查

先分清两种"没反应"，处理完全不同：

| 终端输出 | 含义 |
| :--- | :--- |
| `waiting for service to become available...` 后卡住 | 服务**不存在** → 节点没起或已退出 |
| `requester: making request: ...` 后**没有 response** | 服务**被发现**了，请求发出去了，但回复回不来 → **中间件数据面不通**（见下，已定位） |

### ★ 已实测定位：Fast DDS 共享内存传输（SHM）

本机复现（同一台机器、两个 shell，节点日志一切正常）：`get_mode` 打印
`requester: making request` 后**永远没有 response**；`ros2 control list_controllers`
10 s 超时。

根因：**Fast DDS 的发现（discovery）走 UDP 多播，而请求/回复的数据面走共享内存
（SHM）**。跨 shell / 容器 / 命名空间时 SHM 不通 → "服务看得见、调用没回应"。
**与机器人代码、控制器、CAN 通通无关。**

实测对照（同机跨 shell）：

| 中间件配置 | `get_mode` | `list_controllers` |
| :--- | :--- | :--- |
| 默认（Fast DDS，含 SHM） | ✗ 无响应 | ✗ 超时 |
| `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` | ✓ 正常 | ✓ 正常 |
| `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`（Fast DDS 但禁 SHM） | ✓ 正常 | ✓ 正常 |

**修法**（两个终端都要设，且必须一致）：

```bash
# 修法 A（推荐，省事）：换 CycloneDDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 修法 B：保留 Fast DDS，只禁用共享内存传输
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

> 跑 Docker 的话，用 `--ipc=host`（或加大 `--shm-size`）也能解决同一类问题。

### 其它可能

```bash
# 1) 有没有残留/孤儿进程（Ctrl+C 后 ros2_control_node 的 CAN 线程常不响应 SIGINT）
ps -ef | grep -E "lite_bridge|ros2_control_node|move_group|rviz2|spawner" | grep -v grep

# 2) 两个终端各跑一次，**逐字对比**（RMW / DOMAIN 不一致 = 各自一个图）
env | grep -E "^(ROS|RMW)" | sort

# 3) 清干净再重启
pkill -f "ros2_control_node|lite_bridge|move_group|moveit_remote_policy_bridge|rviz2|spawner"
sleep 2
ros2 daemon stop
```

> 若 launch 那个终端已经回到 `$` 提示符，说明 launch **已经退出**了 ——
> 此时再调服务，遇到的多半是上一轮残留的幽灵端点，按上面 1)~3) 清掉即可。

---

## 一键脚本（可选）

`one_click.launch.py` 已覆盖上述阶段 1~3 的启动部分，一般不必再写脚本。
如需固定一套常用参数，写成别名即可：

```bash
# ~/.bashrc
alias lite='export ROS_HOME=/data/lite_motor_ws/roslog && \
  source /opt/ros/jazzy/setup.bash && \
  source /data/lite_motor_ws/install/setup.bash'
alias lite-up='lite && ros2 launch lite_moveit one_click.launch.py'
alias lite-hold='ros2 service call /lite_bridge/set_mode \
     robstride_msgs/srv/SetMode "{protocol: 255, mode: 1}"'
alias lite-track='ros2 service call /lite_bridge/set_mode \
     robstride_msgs/srv/SetMode "{protocol: 255, mode: 2}"'
alias lite-soft='ros2 service call /lite_bridge/set_mode \
     robstride_msgs/srv/SetMode "{protocol: 255, mode: 0}"'
```

---

## 相关文件位置

| 文件 | 路径 |
| :--- | :--- |
| 工作区 | `/data/lite_motor_ws` |
| 硬件 bringup launch | `src/lite_bringup/launch/lite_bringup.launch.py` |
| 三态控制器参数 | `src/lite_bringup/config/controllers_mit.yaml` |
| 模式入口服务定义 | `src/robstride_msgs/srv/SetMode.srv`（`/lite_bridge/set_mode`）|
| 模式回读服务定义 | `src/robstride_msgs/srv/GetBridgeStatus.srv` |
| 数据源切换服务定义 | `src/robstride_msgs/srv/SetSource.srv`（`/lite_bridge/set_source`）|
| 桥配置模板 | `src/lite_bridge/config/bridge.yaml` |
| MoveIt 包 | `/data/lite_data_sources/src/lite_moveit` |
| MoveIt 桥接节点 | `.../lite_moveit/scripts/moveit_remote_policy_bridge.py` |
| MoveIt 控制器映射 | `.../lite_moveit/config/moveit_controllers.yaml` |
| 标定文件 | `src/lite_calibration/config/calibration.yaml` |
