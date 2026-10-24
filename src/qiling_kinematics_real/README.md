# qiling_kinematics_real

Jetson NX 真机专用双臂末端位姿遥操包。使用固定
`base_link` 的 `s4_dual_arm.urdf`，通过 Pinocchio + ProxSuite 执行双臂
层级差分 IK。

## 当前真机接口

状态输入：

```text
/human_lower_state   mit_msgs/msg/MITLowState
```

状态消息中的 `joint_states.position` 固定为 26 个身体电机，真机包只读取：

```text
12..18  左臂 7 关节
19..25  右臂 7 关节
```

命令输出：

```text
/teleop/arms_mit_command   mit_msgs/msg/MITJointCommands
```

命令固定为 26 维。0..11 腿部字段使用实测位置，12..18 和 19..25 分别为双臂
的 MIT 位置/速度/增益/前馈力矩命令。O6 不进入该命令，使用独立接口。
`xr_teleop_real.launch.py` 中的 `qiling_arms_command_bridge` 将该消息转为
`/lowcmd_arms` 的 `qi/msg/LowCmd`。SDK 使用自己的上下肢融合器接管双臂；
策略有效时腿部使用策略输入。遥操不向策略的 `/human_lower_command` 发布。

## 启动

在项目根目录启动 PC Service 和 Quest 端数据发送。正式 NX 部署使用项目脚本；真机脚本自行加载 ROS 2/工作区环境，设置 Domain 21 和 CycloneDDS，并等待状态输入：

```bash
./src/scripts/start_xrobotoolkit_real.sh
./src/scripts/start_real_teleop.sh
```

下面的直接 launch 命令用于已自行准备好 ROS 和 DDS 环境的开发终端，正式部署优先使用上面的脚本：

```bash
ros2 launch qiling_kinematics_real xr_teleop_real.launch.py
```

启动后，`differential_ik_real_node` 会自动执行一次安全回 home 流程。它等待
`/human_lower_state` 中左右臂状态都有效，然后以首次实测关节位置为起点，按
以下顺序发布 26 维 MIT 命令：

```text
实测当前位置 → 过渡点 → home → 等待稳定 → 开放遥操
```

当前 home（弧度）为：

```text
left:  [0.0104905777,  0.6509880424, -0.1741435826, -1.7084382772,
        0.0738155171, -0.0963225737, -0.0936522484]
right: [-0.0749599487, -0.6475547552,  0.1073853672, -1.2964446545,
        -0.0009536889, -0.4251545072,  0.4945830405]
```

该 home 来自真机示教时的 26 维 `lowstate`：0..11 腿部数据未写入，12..18
为左臂，19..25 为右臂。启动阶段的过渡点保持不变。

过渡点为：

```text
left:  [1.0,  1.2, 0, -1.2, 0, 0, 0]
right: [1.0, -1.2, 0, -1.2, 0, 0, 0]
```

过渡点和 home 之间使用五次多项式轨迹，默认分别为 2.5 s 和 3.0 s，
到点后还需保持 0.30 s。home 阶段忽略 Quest 的目标位姿和 Grip；home 完成
后，每只手还必须先释放一次 Grip，之后才允许该侧进入遥操，防止手柄已经按住
时产生目标跳变。home 状态通过以下话题发布，`true` 表示已完成；同时发布数值状态
供录制器门控：

```text
/teleop/startup_home_complete   std_msgs/msg/Bool
/teleop/home_state               std_msgs/msg/UInt8
```

`home_state` 数值为：`0=WAITING_FOR_STATE`、`1=MOVING`、`2=SETTLING`、
`3=READY`、`4=FAULT`（包括恢复故障）、`5=HOLD_FOR_RECORDING`。
`/teleop/request_home` 与 Y+A 共用回 home 恢复流程，可在遥操、动作、
启动回 home 及故障保持阶段请求；反馈暂时失联时先记录请求，收到双臂新鲜状态后
再规划。`/teleop/request_recording_hold` 仍可单独用于锁存命令。

O6 状态节点在收到该话题为 `true` 之前强制输出零状态，因此 home 期间不会
因手柄输入改变 O6 状态。O6 仍然是独立于 26 维双臂命令的接口。

## 组合动作（默认启用）

已接入挥手、握手、抱拳和双臂回零位的关节空间动作状态机，
`config/differential_ik.yaml` 中 `gesture_enabled: true`；启动并完成 home 后
无需再手动设置参数。挥手、握手、抱拳、双臂回零位分别使用 X+A、Y+B、X+B、A+B；
Y+A 使双臂从当前命令构型平滑回 home。三个表演动作及 A+B 需要两侧 Grip/Trigger
松开；Y+A 可在 Grip/Trigger 按住时请求，松开并再次按下 Y+A 才会再次触发。
执行期间两侧遥操和 Trigger 手型输入均锁定。若起点关节在 URDF 硬限位内、
但不足 `0.12 rad` 软件余量，进场阶段先把该关节平滑移入余量；未设动作目标的
另一臂只调整这些近限位关节。动作从当前命令平滑进场，
三个表演动作结束后回 home；A+B 到达双臂关节零位后保持零位。到位并交接后
才重新接受控制。A+B 使双手张开；Y+A 可从零位回 home。手型目标以 10 Hz
重发，动作与遥操均共用此机制。离线轨迹与碰撞检查范围
见 [第一阶段说明](docs/gesture_stage1_offline.md)，状态机、手型仲裁及启用前提见
[第二阶段说明](docs/gesture_stage2_integration.md)。

## 遥操目标与回 home 恢复

Grip 接合后，IK 使用**上一帧命令构型**计算手端位姿与雅可比，并在该构型上
积分关节速度；实测关节用于跟踪误差、硬限位、预测停止和重力计算。Quest 的
相邻位姿增量经过速度、姿态超前量和关节跟踪误差限制；超出的手柄运动被丢弃，
不会在机器人后面排队形成突然追赶。位置任务含弱 home 姿态偏好，腕姿态与肘部
任务仍按现有层级求解。`tracking_stop_rad` 与 `tracking_hold_timeout_sec`
触发保护保持时，保留最后命令构型，等待 Y+A 恢复。

Y+A 和 `/teleop/request_home` 先保持最后发布的双臂命令，后台检查轨迹：
**最后命令 → 当前反馈 → home**；若直达路径碰撞检查失败，再检查
**最后命令 → 当前反馈 → 关节零位 → home**。轨迹采用关节限速、限加速度
的五次插值，运行时检查反馈跟踪，到位后平滑交接，并要求重新松开 Grip 才能
进入遥操。预检失败、反馈丢失或到位超时会保持命令并报告故障；可再次按 Y+A
重试。碰撞检查只覆盖 URDF 中的机器人本体，无法感知桌面、人员等外部障碍。
回 home 必须有双臂新鲜反馈及至少一条通过检查的路径，故障时不会盲目运动。

相关参数在 `config/differential_ik.yaml`：`target_max_position_lead_m`、
`target_max_orientation_lead_rad`、`tracking_slowdown_start_rad`、
`tracking_stop_rad`、`recovery_tracking_tolerance_rad` 等。修改参数后重启节点；
修改 C++ 后重新运行 `./src/scripts/build_xrtele.sh`。

启动文件包含：

```text
qiling_arms_command_bridge       26 维遥操 MIT 命令 → /lowcmd_arms（仅命令转换）
xrobotoolkit_pxrea_adapter_real  Quest 位姿/按键 → ROS2
xr_pose_clutch_bridge_real       Grip 离合器与相对位姿目标
differential_ik_real_node        /human_lower_state → 26 维身体命令
o6_trigger_state_node             Trigger → 独立 O6 状态位
o6_command_adapter_node           O6 状态位 → /handscmd
```

O6 内部状态话题为：

```text
/teleop/o6_trigger_state   std_msgs/msg/UInt8
```

其中 bit 0 表示左 O6 闭合，bit 1 表示右 O6 闭合。Trigger 按住时对应 bit
置位，释放后清零。`o6_command_adapter_node` 在状态变化时立即发送双手命令，
并在 IK 状态新鲜且 home 已完成时以 10 Hz 重发当前目标。动作结束后每只手
分别在其 Trigger 松开后重新使能，不要求两侧同时松开。Quest 输入超时期间，
Trigger 状态节点保持最后稳定状态，适配器因此保持最后一个手部目标位置。

`/xr/controller_joy` 的实体按键索引为：左 X=`buttons[0]`、右 A=`buttons[1]`、
左 Y=`buttons[2]`、右 B=`buttons[3]`，左右 Grip 分别为 `buttons[4]` 和
`buttons[5]`。

当前 O6 姿态与速度为：

```text
open:  [255, 104, 255, 255, 255, 255]
close: [101, 60, 0, 0, 0, 0]
speed: 200
```

当前机器已有自启的 `qi_topic_converter`，负责策略命令到 `/lowcmd` 的转换，
以及 `/lowstate` 到 `/human_lower_state` 的转换。遥操直接订阅其状态输出，
不要再启动一份默认的双向转换器。

遥操 launch 内的专用命令桥使用 `expected_motor_count=26`、
`enable_state_bridge=false`、`enable_command_bridge=true`、
`strict_command_size=true`，并把输入重映射至 `/teleop/arms_mit_command`、
输出重映射至 `/lowcmd_arms`。它与自启转换器的命令通道分开。

`src/scripts/start_real_teleop.sh` 默认加载项目的 `config/cyclonedds_nx.xml`，
当前 NX 本地文件绑定与板子直连的 `eth2`。它自行设置 Domain 21、
CycloneDDS RMW 并等待现有状态话题；启动前无需在终端手动 source/export。

## 左手灵巧手开合测试

当前 SDK 的 `HandsCmd` 固定包含左右两只手，且底层会持续使用两侧的
`positions`，所以测试节点在测试左手时，同时把右手保持在启动时的张开目标
姿态。测试节点不会自动发送命令，只有在终端输入后才发送一条命令：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run qiling_kinematics_real left_hand_test_node
```

在该节点终端输入并回车：

```text
0   左手全张开
1   左手全闭合
q   退出，不发送命令
```

默认使用 O6 的非均匀姿态，速度值为 `200`：

```text
open_position:  [255, 104, 255, 255, 255, 255]
close_position: [101,  60,   0,   0,   0,   0]
speed:          200
```

注意：虽然底层 `qi/msg/HandsCmd` 字段名仍为 `durations`，当前 SDK 会将其
转成 O6 的 `0x05` 速度指令，因此这里应设置 `speed`，不是运动持续时间。

如果现场需要微调某个关节，可以只修改参数后重复测试，不需要改代码：

```bash
ros2 run qiling_kinematics_real left_hand_test_node --ros-args \
  -p open_position:="[255, 104, 255, 255, 255, 255]" \
  -p close_position:="[101, 60, 0, 0, 0, 0]" -p speed:=200
```

该节点会向 `/handscmd` 发布 `qi/msg/HandsCmd`；不要同时运行其他灵巧手
命令发布者。由于底层当前没有真实灵巧手反馈，开合是否到位需要直接观察机械手。

## 安全说明

首次连接真机时应先只运行 PXREA 适配器，确认 Quest 位姿和按键话题正常，
再启动 IK 和底层命令转换节点。第一次执行 home 时建议：

1. 机器人周围清空，双臂前方尤其不能有桌面或人员；
2. 先降低 `command_kp` 或在底层设置限幅，并准备急停；
3. 观察日志中的 `MOVE_TO_TRANSITION`、`SETTLE_AT_TRANSITION`、
   `MOVE_TO_HOME`、`SETTLE_AT_HOME`、`COMPLETE` 状态；
4. 确认左右臂状态频率稳定、26 维命令顺序和身体 SDK 的 MIT 参数含义；
5. 确认腿部零字段不会影响底层腿部控制器，再逐步提高控制增益。

若状态在 home 过程中超时，节点进入 `FAULT` 并将臂部参考保持在当前实测位置，
不会继续向 home 运动。`startup_home_enabled` 可暂时设为 `false` 做通信检查，
但真机正式遥操前应保持为 `true`。
