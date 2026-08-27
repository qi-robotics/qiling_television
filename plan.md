# Qiling Television：Quest 3 双臂灵巧手遥操与 IK 实施计划

本文档是当前项目的实施基线。内容按照当前工作空间中的实际代码、模型、ROS 2 话题和已经确认的交互方式整理，既描述已经完成的部分，也描述接下来需要逐步实现和验证的部分。

目标是使用 Meta Quest 3 作为操作者输入，通过 XRoboToolkit / PXREA SDK 获取左右手柄的 6DoF 位姿和按键状态，经坐标系转换、抓取式离合（clutch）和目标位姿生成后，使用全 C++ 的 Pinocchio + ProxQP 差分 IK 控制 MuJoCo 中的双臂；仿真链路稳定后，再接入真实双臂机器人和 O6 灵巧手，最后按 LeRobot 数据集所需字段录制 episode，并通过独立转换脚本生成最终数据集。

---

## 1. 总体目标和边界

### 1.1 最终目标

系统最终应完成以下闭环：

~~~text
Meta Quest 3 手柄
        |
        v
XRoboToolkit PC Service / PXREA SDK
        |
        v
ROS 2 XR 输入适配器
        |
        v
手柄坐标系 -> 机器人 base_link 坐标系
        |
        v
Grip 离合、零点重置、目标位姿生成
        |
        v
双臂全 6D 位姿目标
        |
        v
C++ Pinocchio FK/Jacobian + ProxQP 差分 IK
        |
        v
双臂关节目标
        |
        +--------------------+
        |                    |
        v                    v
MuJoCo 仿真              真实机器人 ROS 2 驱动
        |                    |
        +----------+---------+
                   v
             状态同步与录制
                   |
                   v
          独立 LeRobot 转换脚本
~~~

### 1.2 当前明确的硬约束

1. 机器人为双臂人形机器人。
2. 每条手臂 7 个关节。
3. O6 灵巧手必须参与控制；O6 的控制接口已经在另一个工作空间中实现，当前约定通过 ROS 2 话题复用。
4. 机器人 SDK 对手臂控制接口最高接收频率为 50 Hz。
5. 真实机器人目前只有 ROS 2 Humble 驱动。
6. IK 必须采用全 C++ 实现。
7. Pinocchio 用于运动学、雅可比和位姿误差计算。
8. ProxQP 用于每个控制周期的差分 IK / QP 求解。
9. 第一阶段采用固定 base_link、腿部只显示不控制的仿真模式。
10. 腿部仍保留在 MuJoCo 场景和 40 维接口中，但 IK 不求解腿部，腿部命令被忽略。
11. 当前遥操输入采用 Quest 3 左右手柄。
12. 左右手柄的 Grip 按键分别作为左右手的 clutch，不使用独立 clutch 按键。
13. 当前控制方式为统一 6D 位姿控制：按住对应 Grip 后，手柄的相对平移和相对旋转同时映射到对应手腕目标。
14. 暂不采用“Grip 只控制平移、Grip + X/A 只控制姿态”的分离模式。
15. 当前阶段先不考虑 Quest 内的视频回传，Quest 侧可暂时只用于输入控制。
16. MuJoCo 当前只有头部 D435 相机；两个腕部 D405 只在机器人描述和后续真实系统规划中保留。
17. episode 录制不直接写成最终 LeRobot 格式，而是先录制必要的原始/中间数据，再使用独立脚本转换。
18. 录制只保留训练实际需要的字段，不录制不必要的底层控制细节，例如实际发送的 MIT 命令。

### 1.3 当前阶段不纳入的内容

以下内容不是第一阶段的实现目标：

- 用 PyRoki 或 CasADi + IPOPT 替换当前 C++ QP IK。
- 每帧运行非线性优化器。
- 腿部遥操。
- 头部和腰部的全身协调控制。
- Quest 内立体渲染和左右拼接视频。
- 将 MuJoCo 中的 D435、D405 完整模拟为真实 RGB-D 传感器。
- 直接把 MIT kp、kd、position、velocity、effort 命令作为训练 observation/action。

---

## 2. 代码包和职责划分

当前工作空间主要包含以下 ROS 2 包：

~~~text
src/
├── common_msgs/
│   └── mit_msgs/
├── communicate_interface/
├── qi_robot_description/
├── mujoco_simulator/
├── mujoco_d435_publisher/
└── qiling_kinematics/
~~~

### 2.1 common_msgs/mit_msgs

职责：

- 提供 MITLowState 等低状态消息。
- 提供 MITJointCommands 等关节命令消息。
- 连接 MuJoCo、差分 IK 和真实机器人控制接口。

当前重要约定：

- 仿真控制命令话题为 /human_lower_command。
- 仿真状态话题为 /human_lower_state。
- /human_lower_command 当前要求消息长度与 MuJoCo 模型的执行器数量严格一致，即 40。
- 40 维中同时包含腿部、双臂和 O6 等关节槽位。

### 2.2 qi_robot_description

职责：

- 保存机器人 URDF、网格和 MuJoCo 模型。
- 为 Pinocchio 提供计算用 URDF。
- 为 MuJoCo 提供仿真场景。

当前关键文件：

~~~text
src/qi_robot_description/
├── urdf/
│   ├── s4_dual_arm.urdf
│   ├── s4_40DOF_fullbody.urdf
│   └── s4_40DOF_fullbody_with_handeye_camera.urdf
└── new_scene/
    ├── scene_S4_40DOF_fullbody.xml
    └── S4_40DOF_fullbody.xml
~~~

模型文件的职责必须严格区分：

| 文件 | 用途 | 是否用于当前 IK |
|---|---|---|
| urdf/s4_dual_arm.urdf | 只包含双臂 14 个关节的 Pinocchio 计算模型 | 是 |
| urdf/s4_40DOF_fullbody.urdf | ROS 2 robot_state_publisher 等完整机器人描述 | 否，当前不作为 IK 模型 |
| urdf/s4_40DOF_fullbody_with_handeye_camera.urdf | 带头部和腕部相机标定关系的完整 URDF | 用于相机位姿参考 |
| new_scene/scene_S4_40DOF_fullbody.xml | MuJoCo 启动入口场景 | 是 |
| new_scene/S4_40DOF_fullbody.xml | MuJoCo 机器人主体、执行器、关键帧和相机定义 | 是 |

### 2.3 mujoco_simulator

职责：

- 加载 MuJoCo 场景。
- 发布 MuJoCo 机器人状态。
- 接收 /human_lower_command。
- 进行固定 base、冻结腿部、home 过渡和外部关节命令执行。
- 发布 /joint_states 供 Pinocchio 差分 IK 使用。
- 发布 /mujoco/qpos 供相机发布节点使用。

### 2.4 mujoco_d435_publisher

职责：

- 在 MuJoCo 中读取 d435_camera。
- 根据 /mujoco/qpos 更新机器人姿态。
- 发布仿真相机彩色图像和 CameraInfo。
- 为后续数据录制和可视化提供相机输入。

当前阶段它不是 Quest 视频传输节点。它可以帮助生成 ROS 2 图像话题，但不能自动把图像送入 Quest。Quest 视频显示属于后续 XR 视频链路，需要单独设计。

### 2.5 qiling_kinematics

职责：

- C++ Pinocchio 运动学。
- C++ ProxQP 差分 IK。
- Quest/XR 输入到机器人位姿目标的桥接。
- PXREA SDK 到 ROS 2 的适配。

当前关键源文件：

~~~text
src/qiling_kinematics/src/
├── differential_ik_node.cpp
├── xr_pose_clutch_bridge.cpp
├── xrobotoolkit_pxrea_adapter.cpp
├── pose_target_demo.cpp
└── xr_pose_demo.cpp
~~~

---

## 3. MuJoCo 仿真架构

### 3.1 启动配置

当前配置文件：

~~~text
src/mujoco_simulator/config/simulate.yaml
~~~

核心配置如下：

~~~yaml
modelPath: package://qi_robot_description/new_scene/scene_S4_40DOF_fullbody.xml
modelName: scene_S4_40DOF_fullbody
lowStateTopic: /human_lower_state
jointCommandsTopic: /human_lower_command
qposTopic: /mujoco/qpos
unPauseService: /unpause_mujoco
initPauseFlag: true
modelTableFlag: true
fixedBase: true
freezeLegs: true
startupKeyframe: teleop_start
targetKeyframe: teleop_home
homeTransitionEnabled: true
homeTransitionKeyframes: [teleop_start, teleop_elbow_lift, teleop_shoulder_rear, teleop_home]
homeTransitionDurations: [0.8, 1.0, 1.2]
homeTransitionKp: 35
homeTransitionKd: 4
homeHoldKp: 30
homeHoldKd: 4
~~~

### 3.2 MuJoCo 维度和控制范围

当前场景实际维度：

- nq = 47。
- nv = 46。
- nu = 40。
- MuJoCo 仿真步长为 0.001 秒，即 1 kHz。
- 外部 ROS 2 控制接口按 50 Hz 使用。

这三个维度不能混淆：

- nq 是广义位置数量，包含浮动基座的 7 个 qpos。
- nv 是广义速度数量，浮动基座速度为 6 个量。
- nu 是执行器数量，当前 /human_lower_command 要求 40 个控制槽。

### 3.3 固定 base_link

第一阶段固定机器人基座：

1. MuJoCo 启动后记录浮动基座的初始 qpos。
2. 每个仿真更新周期将基座 qpos 恢复为初始值。
3. 将浮动基座对应的 6 个速度置零。
4. 不对基座求 IK，不向基座施加移动控制。

这样做的目的：

- 使双臂遥操问题成为以 base_link 为参考的双臂局部控制问题。
- 避免腿部和全身动力学暂时影响双臂位姿验证。
- 让 MuJoCo、Pinocchio 和真实机器人第一版的参考坐标系保持一致。

### 3.4 冻结腿部

第一阶段腿部只显示、不控制：

1. 根据关节名称找到腿部 12 个关节。
2. 记录每个腿部关节在启动或关键帧中的 qpos。
3. 每个仿真周期恢复腿部 qpos。
4. 将腿部速度置零。
5. 忽略 /human_lower_command 中腿部对应的命令槽。
6. 腿部执行器不参与有效外部控制。

必须保留完整 40 维消息接口，不能把消息截断成只有双臂 14 维；但在第一阶段，腿部命令的内容不产生控制效果。

### 3.5 home 过渡机制

仿真不应在解除暂停后直接把双臂瞬移到最终 home。当前采用多关键帧的平滑过渡：

~~~text
teleop_start
      |
      | 0.8 s
      v
teleop_elbow_lift
      |
      | 1.0 s
      v
teleop_shoulder_rear
      |
      | 1.2 s
      v
teleop_home
~~~

过渡采用关节空间 PD：

- homeTransitionKp = 35。
- homeTransitionKd = 4。
- 目标关键帧之间进行时间插值。
- 进入最终 home 后使用 homeHoldKp = 30、homeHoldKd = 4 保持。
- 过渡阶段外部命令暂不覆盖 home 控制。
- 只有过渡完成后才进入遥操命令控制。

当前 home 关节值已经按照最新确认值修改为：

关节顺序：

~~~text
shoulder_pitch
shoulder_roll
shoulder_yaw
elbow
wrist_roll
wrist_pitch
wrist_yaw
~~~

左臂：

~~~text
-0.45,  0.30, 0.02, -1.40, 1.40, 0.0, 0.0
~~~

右臂：

~~~text
-0.45, -0.30, 0.02, -1.40, 1.40, 0.0, 0.0
~~~

当前 home 的手指槽位为 0，即手部处于打开或中性位置，具体手指方向以模型的零位定义为准。

当前 home 的设计目标：

- 双手掌心大致相对。
- 肘部向外侧偏，不向身体中线夹入。
- 肘部高度不过高。
- 双手高度高于桌面。
- 通过抬肘、后收肩部的中间关键帧避免双臂直接一字张开。
- 过渡路径不碰撞桌面。

注意：最终使用上述精确关节值后，应以 MuJoCo 实际 FK 结果为准，而不是只看关节数值。当前 FK 检查得到的大致结果是：

~~~text
左手末端位置约为 ( 0.3895,  0.3116, 1.3183)
右手末端位置约为 ( 0.3880, -0.3228, 1.3214)
左肘位置约为     ( 0.1022,  0.3328, 1.2450)
右肘位置约为     ( 0.1010, -0.3328, 1.2445)
~~~

因此后续验证时需要同时观察：

- 末端位置是否在桌面上方。
- 左右掌心法向是否相对。
- 左右肘是否向外。
- 过渡过程中是否与桌面或身体碰撞。

### 3.6 MuJoCo 命令优先级

当前仿真控制优先级应保持如下顺序：

~~~text
冻结腿部
  >
home 过渡
  >
home 保持
  >
外部 MIT 命令
  >
执行器控制范围裁切
~~~

当没有收到有效外部命令时，系统不应继续使用上一帧旧命令慢慢漂移。应满足：

- home 过渡未开始：保持启动姿态。
- home 过渡进行中：执行过渡 PD。
- home 过渡完成但没有遥操激活：保持 home。
- 遥操 clutch 松开：差分 IK 进入 hold，并将该侧目标重置为当前测量关节姿态。
- 输入超时：停止追踪目标并保持当前姿态，不能继续积分。

---

## 4. Pinocchio 双臂模型和关节映射

### 4.1 IK 使用的模型

实际 Pinocchio 控制模型固定使用：

~~~text
/home/ub/project/qiling_television/src/qi_robot_description/urdf/s4_dual_arm.urdf
~~~

该模型不是完整 40DOF 全身模型，而是专门为第一阶段双臂差分 IK 准备的 14 关节模型。

初始化时必须检查：

~~~text
model.nq == 14
model.nv == 14
~~~

如果维度不满足，应直接报错退出，禁止静默运行，因为关节索引错位会导致非常危险的控制结果。

### 4.2 双臂关节顺序

左臂：

~~~text
0 left_shoulder_pitch_joint
1 left_shoulder_roll_joint
2 left_shoulder_yaw_joint
3 left_elbow_joint
4 left_wrist_roll_joint
5 left_wrist_pitch_joint
6 left_wrist_yaw_joint
~~~

右臂：

~~~text
0 right_shoulder_pitch_joint
1 right_shoulder_roll_joint
2 right_shoulder_yaw_joint
3 right_elbow_joint
4 right_wrist_roll_joint
5 right_wrist_pitch_joint
6 right_wrist_yaw_joint
~~~

ROS 2 /joint_states 的排列顺序不能假定与上述顺序相同。当前实现必须通过 JointState.name 按关节名称建立映射，然后再填充 Pinocchio 的 q。

### 4.3 末端 frame

默认左右末端 frame：

~~~text
LH_hand_base_link
RH_hand_base_link
~~~

兼容回退 frame：

~~~text
left_wrist_yaw_link
right_wrist_yaw_link
~~~

启动时必须打印最终使用的 frame ID，并检查：

- frame 存在。
- frame 属于双臂模型。
- 左右末端不是同一个 frame。
- 当前 q 下 FK 的末端位姿是有限数。

### 4.4 Pinocchio 每周期计算

每个控制周期的计算顺序：

1. 读取最新的 /joint_states。
2. 根据关节名形成 q。
3. 调用 forwardKinematics。
4. 调用 computeJointJacobians。
5. 调用 updateFramePlacements。
6. 读取左右末端当前 SE(3) 位姿。
7. 读取左右末端 LOCAL frame Jacobian。
8. 计算当前位姿到目标位姿的 SE(3) 误差。
9. 构建两侧 QP。
10. 用上一周期解 warm-start。
11. 约束 qdot。
12. 积分得到 q_target。
13. 将双臂目标写入 40 维 MITJointCommands。

---

## 5. 当前差分 IK 方案

### 5.1 选择差分 IK 的原因

手柄输入天然是连续位姿变化，不是一次性求一个离散关节解。因此当前采用：

~~~text
目标末端位姿
    |
    v
SE(3) 位姿误差
    |
    v
期望笛卡尔速度
    |
    v
Pinocchio 雅可比
    |
    v
ProxQP 求关节速度 qdot
    |
    v
积分 q_target
    |
    v
关节 PD/MIT 命令
~~~

差分 IK 的核心形式是：

~~~text
e = log6(current_pose.inverse() * target_pose)

v_des = [position_gain * e_position,
         rotation_gain * e_rotation]

Jw = [sqrt(position_weight) * J_position,
      sqrt(rotation_weight) * J_rotation]

minimize 0.5 * ||Jw * qdot - vw_des||^2
       + 0.5 * damping * ||qdot||^2
       + posture term
~~~

其中 qdot 为 7 维关节速度。

### 5.2 当前 ProxQP 约束

当前每条手臂建立独立的 7 维 QP：

- 变量：左侧或右侧 7 个关节速度。
- 上下界：每个关节的 qdot lower / upper。
- 不使用两臂之间的耦合等式约束。
- 求解器使用上一周期结果进行 warm-start。
- 使用稠密 LDLT 线性代数路径。
- QP 失败时保持该侧目标，不发布危险的无效结果。

关节速度上界同时考虑：

1. 全局最大关节速度。
2. 当前 q 到 URDF 关节上下限之间的距离。
3. joint_limit_margin。
4. 控制周期 dt。

积分形式：

~~~text
q_target(k+1) = clamp(q_target(k) + qdot(k) * dt,
                       q_min + margin,
                       q_max - margin)
~~~

### 5.3 当前默认参数

配置文件：

~~~text
src/qiling_kinematics/config/differential_ik.yaml
~~~

当前主要参数：

~~~yaml
control_rate_hz: 50
target_frame: base_link
target_timeout_sec: 0.25
position_gain: 3.0
rotation_gain: 3.0
position_weight: 1.0
rotation_weight: 0.80
damping: 0.02
posture_weight: 0.0
joint_limit_margin: 0.08
max_joint_velocity: 1.5
max_position_error: 0.25
max_rotation_error: 1.2
command_kp: 40.0
command_kd: 2.0
qp_max_iter: 80
qp_eps_abs: 1e-5
~~~

第一阶段调参顺序：

1. 先确认坐标系和末端 frame 正确。
2. 再确认 q 与 /joint_states 一致。
3. 再确认目标不动时 q_target 不漂移。
4. 再调位置误差增益和最大速度。
5. 再调旋转误差增益和旋转速度。
6. 最后才调整阻尼、关节限位边界和 posture term。

不能在坐标系尚未验证时通过增益调参掩盖方向错误。

### 5.4 当前 IK 的重要行为约定

1. 控制模式 0 表示 hold。
2. 非 0 模式表示 active。
3. 模式 2 目前仅作为兼容值，不代表独立的姿态控制模式。
4. 当前目标是统一的 6D 位姿目标，平移和旋转同时生效。
5. clutch 松开后，该侧目标重置到当前机器人末端状态。
6. 目标超时后，全部目标重置为当前测量姿态并保持。
7. q_target 只在收到第一帧有效 JointState 时初始化，不能每帧用测量 q 覆盖。
8. 只有位姿目标持续有效且对应侧 active 时才允许积分。
9. 没有 active 目标时不能继续沿用旧 qdot 积分。

### 5.5 现有 IK 方案的局限

当前实现已经可以完成双臂位姿闭环，但仍不是完整的高质量全身控制器：

- 两条手臂相互独立，未处理双手协同约束。
- 没有碰撞约束。
- posture_weight 默认是 0，没有主动避奇异位形的 null-space 目标。
- 没有显式的末端线速度、角速度平滑器。
- 目标位姿和 q_target 的安全边界还需要在真实机器人上重新标定。
- O6 尚未接入最终命令合并链路。
- 当前 MITJointCommands 初始化为 40 个零槽，后续必须明确 O6 和腿部命令的唯一来源。

---

## 6. Quest 3 / XRoboToolkit 输入链路

### 6.1 PC Service

当前 PC 端已经安装：

~~~text
XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
~~~

服务脚本：

~~~text
/opt/apps/roboticsservice/runService.sh
~~~

启动 PC Service 后，Quest 3 客户端连接到该服务。当前已确认左右手柄能够连接并控制双臂，说明：

- Quest 3 与 PC Service 的连接可用。
- PXREA SDK 动态库可加载。
- 手柄 pose 回调可收到。
- 左右手柄输入能够进入 ROS 2 适配器。

### 6.2 PXREA SDK 适配器

源文件：

~~~text
src/qiling_kinematics/src/xrobotoolkit_pxrea_adapter.cpp
~~~

SDK 文件：

~~~text
/opt/apps/roboticsservice/SDK/include/PXREARobotSDK.h
/opt/apps/roboticsservice/SDK/x64/libPXREARobotSDK.so
~~~

当前适配器：

1. 初始化 PXREA SDK。
2. 注册 PXREADeviceStateJson 回调。
3. 解析回调中的 JSON envelope。
4. 解析 value.Controller.left 和 value.Controller.right。
5. 读取 pose、trigger、grip、primaryButton。
6. 以 90 Hz 发布 ROS 2 数据。

发布话题：

~~~text
/xr/left_controller_pose   geometry_msgs/PoseStamped
/xr/right_controller_pose  geometry_msgs/PoseStamped
/xr/controller_joy         sensor_msgs/Joy
~~~

pose frame_id：

~~~text
xr_origin
~~~

当前 Joy 数组约定：

~~~text
axes[0]    左扳机 trigger
axes[1]    右扳机 trigger
axes[2]    左握把 grip
axes[3]    右握把 grip

buttons[0] 左 primary / X
buttons[1] 右 primary / A
buttons[4] 左 Grip 数字按键
buttons[5] 右 Grip 数字按键
~~~

当前适配器尚未解析 Quest 手部骨骼或 O6 所需的手指关节信息。O6 需要另行从 SDK 或现有 O6 工作空间接入。

### 6.3 clutch 逻辑

源文件：

~~~text
src/qiling_kinematics/src/xr_pose_clutch_bridge.cpp
~~~

输入：

~~~text
/xr/left_controller_pose
/xr/right_controller_pose
/xr/controller_joy
/teleop/left_wrist_state
/teleop/right_wrist_state
~~~

输出：

~~~text
/teleop/left_wrist_target
/teleop/right_wrist_target
/teleop/left_control_mode
/teleop/right_control_mode
~~~

每侧独立使用对应手柄 Grip：

- 左 Grip 控制左臂 clutch。
- 右 Grip 控制右臂 clutch。
- 松开 Grip：该侧进入 hold。
- 再按下 Grip：记录新的 XR anchor 和 robot anchor。
- 按住 Grip：输出相对于 anchor 的 6D 位姿增量。
- X/A 当前不作为姿态/平移模式切换键。

按下瞬间的重定位逻辑：

~~~text
robot_anchor = 当前机器人末端位姿
xr_anchor    = 当前手柄位姿

之后：
relative_xr = inverse(xr_anchor) * xr_current
target      = robot_anchor * mapped(relative_xr)
~~~

这样可以避免操作者手柄当前所在位置与机器人手腕当前所在位置不一致时发生瞬移。

### 6.4 输入超时和失效安全

当前建议并应保持的超时：

- XR pose 超时：0.20 秒。
- Joy 超时：0.20 秒。
- 机器人末端状态超时：0.20 秒。
- 差分 IK 目标超时：0.25 秒。

超时处理：

1. 立刻停止该侧 active 状态。
2. 将目标冻结在最新有效机器人位姿。
3. 向 IK 发布 hold 模式。
4. 禁止继续积分旧的目标速度。
5. 恢复有效输入后，需要重新 clutch 建立 anchor。

---

## 7. 坐标系和方向映射

### 7.1 坐标系定义

当前映射的基础假设：

XRoboToolkit / XR 输入坐标：

~~~text
X：右
Y：上
Z：前 / 朝向操作者前方
~~~

机器人 base_link：

~~~text
X：前
Y：左
Z：上
~~~

当前基础轴变换矩阵：

~~~text
M =
  [ 0   0   1
   -1   0   0
    0   1   0 ]
~~~

该矩阵的行列含义必须在代码中保持一致，并在启动日志中打印。由于当前 XR 与机器人手性和轴定义存在差异，不能只用一个简单的 quaternion 乘法解决全部方向问题。

### 7.2 当前平移修正

根据已进行的手柄实测：

- Y、Z 平移方向基本正确。
- X 平移曾经相反，因此当前 X 平移需要取反。
- 左手 Y 方向需要额外取反。
- 右手 Y 方向保持当前映射。

当前桥接配置：

~~~yaml
robot_translation_axis_sign_x: -1
robot_translation_axis_sign_y: 1
robot_translation_axis_sign_z: 1
left_robot_translation_axis_sign_y: -1
right_robot_translation_axis_sign_y: 1
~~~

因此使用左右手柄时，必须分别验证，不能只验证右手后假设左手自动正确。

### 7.3 当前旋转修正

当前旋转处理不是把四元数分量逐个取反，而是：

1. 计算手柄相对于 XR anchor 的相对旋转。
2. 用基础轴变换和 robot anchor 方向构造局部 basis change。
3. 使用共轭变换把相对旋转变换到机器人参考方向。
4. 将旋转矩阵转换成轴角向量。
5. 应用旋转轴方向和统一旋转比例。
6. 与 robot anchor 组合得到目标姿态。

当前旋转参数：

~~~yaml
rotation_invert: true
rotation_scale: 0.50
max_rotation_rad: 2.50
rotation_sign_x: -1
rotation_sign_y: 1
rotation_sign_z: 1
~~~

当前旋转速度曾经偏大，因此旋转比例必须先保持在 0.50 或更低，在实测中逐步增加。旋转限幅必须保留，防止 Quest 重连、追踪跳变或 anchor 异常导致目标瞬间跳跃。

### 7.4 坐标系验收动作

每次修改映射后都必须单独执行以下动作，不允许凭整体运动感觉判断：

平移：

~~~text
左手柄向机器人 base_link +X 移动
左手柄向 base_link -X 移动
左手柄向 base_link +Y 移动
左手柄向 base_link -Y 移动
左手柄向 base_link +Z 移动
左手柄向 base_link -Z 移动
右手柄重复上述六项
~~~

旋转：

~~~text
绕 base_link X 正转和反转
绕 base_link Y 正转和反转
绕 base_link Z 正转和反转
左右手柄分别重复
~~~

每次只做一个自由度动作，其他方向尽量固定。记录：

- 手柄相对位移或旋转方向。
- target PoseStamped 的变化。
- current wrist state 的变化。
- q_target 的变化。
- 最终 MuJoCo 末端 frame 的变化。

排查顺序必须是：

~~~text
XR 原始 pose
  -> adapter 发布 pose
  -> bridge 生成 target
  -> IK 读取 target
  -> Pinocchio current / target error
  -> qdot
  -> q_target
  -> MuJoCo /joint_states
~~~

不能直接只改 IK 里的符号，因为错误可能出现在 XR 原始坐标、anchor 组合、桥接矩阵或 MuJoCo 末端 frame 定义。

---

## 8. 双臂统一位姿控制策略

### 8.1 当前选定模式

当前不再使用平移和旋转分开控制，统一采用：

~~~text
按住 Grip
    |
    v
同时控制对应手腕的 3D 平移 + 3D 旋转
~~~

原因：

- 手柄本身提供完整 6DoF 输入。
- 分开模式会造成操作者姿态意图和目标状态切换复杂。
- 之前的“仅旋转但位置仍漂移”问题更适合通过目标构造和 IK 约束修复，而不是继续增加模式分支。

### 8.2 为什么统一 6D 仍可能出现位置跟随

即使目标 PoseStamped 的位置不变，位置仍可能变化，原因可能来自：

1. 末端 frame 不在 wrist joint 的旋转中心。
2. 姿态误差导致肩、肘、腕多个关节共同运动。
3. QP 只有速度边界，没有严格位置等式约束。
4. 雅可比使用了错误的 frame reference。
5. 旋转和平移误差的权重不平衡。
6. 7 自由度冗余在无 posture 目标时存在不同解。
7. q_target 与实际 q 的同步或积分逻辑错误。

因此当前 IK 验证必须区分两类指标：

- 目标误差：current wrist 与 target wrist 的位置、旋转误差。
- 非期望方向运动：操作者只旋转时末端位置变化量，操作者只平移时末端旋转变化量。

### 8.3 后续用于减少耦合的改进顺序

先做不改变求解器结构的改进：

1. 确认 Pinocchio 使用 LOCAL / LOCAL_WORLD_ALIGNED 的一致性。
2. 确认 Pose log6 的误差定义和目标组合顺序。
3. 对目标 pose 进行有限差分测试。
4. 对每一侧单独记录数值雅可比和解析雅可比。
5. 增加 qdot 一阶低通或加速度限制。
6. 增加 posture/null-space 项，使肘部偏向舒适 home。
7. 增加姿态和平移的独立权重调节。
8. 必要时采用加权阻尼最小二乘或带软约束的 QP。

如果仍需严格保证位置和姿态在某种动作下完全不耦合，应增加任务优先级或硬等式约束；但这应在基础坐标系、frame 和 q 映射完全正确后进行。

---

## 9. O6 灵巧手控制接入方案

### 9.1 已知条件

- O6 控制已经在工作空间 /home/ub/project/qiling_grasp_ws 中存在参考实现。
- O6 控制接口通过 ROS 2 话题暴露。
- 真实机器人端已经实现 O6 控制。
- O6 必须参与最终遥操。
- 当前 MuJoCo 模型中双手每侧的手指执行器也占用 /human_lower_command 的槽位。

### 9.2 必须先确认的接口

在正式接入前，必须从 qiling_grasp_ws 和真实驱动中确认：

1. O6 状态话题名称。
2. O6 命令话题名称。
3. 消息类型。
4. 每只 O6 的关节数量。
5. 关节名称和排列顺序。
6. 角度单位，是弧度还是度。
7. 控制值含义，是位置、速度、力矩还是归一化手指开合量。
8. 消息是否需要 kp、kd、mode 或 enable 字段。
9. 最高接收频率。
10. 左右手 O6 是否使用相同的关节顺序。
11. 失联时驱动如何处理。
12. MuJoCo 中 O6 关节数量和真实 O6 接口数量是否一致。

当前需要特别注意一个已知风险：早期实现中手部曾按每侧 6 个值考虑，而当前 MuJoCo 模型的 O6 关节槽位可能包含 7 个关节，例如 thumb_ip 等。因此不能直接复制旧的 6 维数组，必须以当前模型和真实 O6 接口的 joint name 为准建立显式 mapping。

### 9.3 O6 输入来源

Quest 手柄默认只能稳定提供：

- 手柄 6DoF 位姿。
- trigger。
- grip。
- X/A 等按键。

这不足以直接生成完整灵巧手关节动作。O6 可采用的输入来源按优先级如下：

方案 A：XRoboToolkit / Quest 手部骨骼数据

- 如果 Quest 客户端和 SDK 能提供 hand tracking skeleton，则解析手指关节。
- 将人体手指关节角映射到 O6 关节。
- 需要左右手镜像、角度范围和手性校准。

方案 B：手柄 trigger / 按键映射

- trigger 映射拇指或整体抓取开合。
- 按键映射若干离散抓取模式。
- 适合先验证 O6 通信，不适合高质量模仿数据。

方案 C：复用 qiling_grasp_ws 已有抓取模式

- 在遥操桥接层发布 grasp mode。
- O6 控制节点将模式转换成 O6 关节命令。
- 适合当前已有的 O6 模式接口。

第一阶段建议先采用 B 或 C 完成 MuJoCo/真实接口打通，再根据 Quest SDK 实际是否提供 skeleton 决定是否实现 A。

### 9.4 命令合并原则

最终必须只有一个节点向 /human_lower_command 发布 40 维命令，或者明确使用一个命令 mux；不能让差分 IK 节点和 O6 节点同时直接发布同一个话题。

推荐结构：

~~~text
双臂 IK 目标 q
        \
         \
          > qiling_command_mux -> /human_lower_command
         /
O6 目标 q
        /
腿部安全保持槽
~~~

命令 mux 负责：

- 接收双臂 14 维目标。
- 接收 O6 左右手目标。
- 将腿部槽位填充为安全保持值或零值。
- 按固定全局 joint map 写入 40 维命令。
- 统一检查 finite、范围、时间戳和 freshness。
- 统一在失效时进入 hold。
- 统一记录用于调试的非训练数据。

如果暂时不新增 mux，也必须在单个最终控制节点中完成上述合并，保证 /human_lower_command 只有一个 publisher。

### 9.5 O6 与双臂控制频率

由于手臂 SDK 最高接收频率为 50 Hz：

- 差分 IK 主循环采用 50 Hz。
- O6 命令可以在 50 Hz 发送，或在单独节点内部更高频更新后由 50 Hz mux 统一输出。
- 对真实机器人不应超过 SDK 明确允许的接收频率。
- MuJoCo 内部虽然以 1 kHz 运行，但 ROS 2 外部命令仍按 50 Hz 更新。

---

## 10. 当前运行链路和指令

以下命令默认已 source ROS 2 Humble 和工作空间：

~~~bash
source /opt/ros/humble/setup.bash
source install/setup.bash
~~~

### 10.1 构建

修改 C++ 后构建：

~~~bash
colcon build --symlink-install --packages-select qiling_kinematics mujoco_simulator mujoco_d435_publisher
source install/setup.bash
~~~

如果只修改 XML、YAML 或 launch 文件，在当前 symlink-install 工作流下通常无需重新编译，但仍应重新 source 并检查 install 下的路径是否指向 source。

### 10.2 启动 MuJoCo

~~~bash
ros2 launch mujoco_simulator simulate.launch.py
~~~

默认：

- 启动时暂停。
- 模型为 scene_S4_40DOF_fullbody.xml。
- 固定 base。
- 冻结腿部。
- 解除暂停后执行多阶段 home 过渡。
- 发布 /human_lower_state。
- 发布 /joint_states。
- 发布 /mujoco/qpos。

解除暂停：

~~~bash
ros2 service call /unpause_mujoco std_srvs/srv/Trigger {}
~~~

也可以使用 MuJoCo 窗口中的播放/暂停操作；程序构造时已考虑通过 GUI 解除暂停后也触发 home 过渡。

### 10.3 单独启动 MuJoCo D435

当前仿真只有头部相机，因此单独启动：

~~~bash
ros2 launch mujoco_d435_publisher d435_camera.launch.py
~~~

主要输出：

~~~text
/camera/camera/color/image_raw
/camera/camera/color/camera_info
~~~

当前 D405 尚未在 MuJoCo 场景中发布。后续若需要腕部观测，应补充：

- MuJoCo 中的两个相机定义。
- 相机 qpos 更新。
- 左右腕部 frame。
- 两个图像话题和 CameraInfo。
- 录制器中的相机字段映射。

### 10.4 启动纯 IK 演示

~~~bash
ros2 launch qiling_kinematics differential_ik.launch.py
~~~

该启动用于验证：

- /joint_states 是否能映射到 Pinocchio。
- 目标 PoseStamped 是否能驱动双臂。
- /human_lower_command 是否符合 40 维要求。
- IK 是否在 50 Hz 稳定运行。

### 10.5 启动 XR 输入适配器

先启动 PC Service：

~~~bash
/opt/apps/roboticsservice/runService.sh
~~~

然后启动 XRoboToolkit 适配器：

~~~bash
ros2 run qiling_kinematics qiling_xrobotoolkit_pxrea_adapter
~~~

检查：

~~~bash
ros2 topic echo /xr/left_controller_pose
ros2 topic echo /xr/right_controller_pose
ros2 topic echo /xr/controller_joy
~~~

### 10.6 启动 clutch 和位姿桥接

~~~bash
ros2 run qiling_kinematics qiling_xr_pose_clutch_bridge --ros-args \
  --params-file src/qiling_kinematics/config/xr_pose_clutch_bridge.yaml
~~~

检查：

~~~bash
ros2 topic echo /teleop/left_wrist_target
ros2 topic echo /teleop/right_wrist_target
ros2 topic echo /teleop/left_control_mode
ros2 topic echo /teleop/right_control_mode
~~~

### 10.7 一键真实 XR 仿真链路

当前 real 命名的 launch 用于 XR → clutch → differential IK → MuJoCo 的闭环演示，启动前确保：

1. PC Service 已启动。
2. Quest 3 已连接。
3. MuJoCo 已运行并解除暂停。
4. MuJoCo 使用 teleop_home 完成 home 过渡。

启动：

~~~bash
ros2 launch qiling_kinematics xr_teleop_real.launch.py
~~~

如果该 launch 同时启动仿真和适配器，应先查看 launch 内容确认是否会重复启动节点；系统最终应保证每个关键话题只有预期的 publisher。

---

## 11. 分阶段实施路线

### 阶段 A：静态模型和接口基线

目标：确保所有节点使用正确模型和正确维度。

任务：

1. 确认 qi_robot_description 被正确安装。
2. 确认 MuJoCo 可以通过 package URI 加载 scene_S4_40DOF_fullbody.xml。
3. 确认 scene 文件包含正确的完整机器人 XML。
4. 检查 nq、nv、nu。
5. 检查 d435_camera 名称。
6. 检查 s4_dual_arm.urdf 的 nq、nv。
7. 检查左右末端 frame。
8. 检查所有 ROS 2 话题和消息类型。
9. 检查 /human_lower_command 的 40 维要求。

验收：

- MuJoCo 正常启动。
- 解锁后 home 过渡只执行一次。
- 腿部固定。
- base_link 固定。
- /joint_states 稳定发布。
- Pinocchio 节点可以加载模型且不报 frame/joint 错误。

### 阶段 B：home 姿态和安全过渡

目标：建立稳定、舒适、可重复的遥操起始姿态。

任务：

1. 验证最新 home 关节值已经写入 XML。
2. 检查 teleop_start、teleop_elbow_lift、teleop_shoulder_rear、teleop_home 的 qpos 长度都是 47。
3. 用 FK 检查四个关键帧的双手位置和左右肘位置。
4. 检查整个插值路径的桌面碰撞。
5. 如需要，调整中间关键帧，不改变最终 home 关节值。
6. 检查最终双手掌心是否相对。
7. 检查最终肘部是否向外而不是夹向身体。
8. 检查解除暂停后外部命令不会跳过过渡。
9. 检查暂停、继续、重启后状态一致。

验收动作：

- 连续启动 10 次，home 姿态一致。
- 连续解除暂停 10 次，没有明显瞬移。
- 过渡过程中双手不穿桌面。
- 过渡结束后没有大幅振荡。

### 阶段 C：Pinocchio FK 和状态映射

目标：证明 IK 输入的 q 和 MuJoCo 当前姿态是同一个机器人姿态。

任务：

1. 订阅 /joint_states。
2. 根据 name 建立左右臂索引。
3. 转换到 Pinocchio q。
4. 计算左右末端 FK。
5. 发布 /teleop/left_wrist_state 和 /teleop/right_wrist_state。
6. 将发布的末端状态与 MuJoCo viewer 中的末端位置对比。
7. 在 home、随机姿态和单关节变化下对比。

验收指标：

- home 时 Pinocchio 和 MuJoCo 左右末端位置误差小于预设阈值。
- 旋转误差小于预设阈值。
- 单独改变某个关节时，Pinocchio 末端变化方向和 MuJoCo 一致。
- 不存在左右手索引错位。

### 阶段 D：ProxQP 差分 IK 单元测试

目标：先不接 Quest，验证 IK 数学和数值稳定性。

任务：

1. 固定一个 q。
2. 用 FK 产生当前末端 pose。
3. 生成小幅平移目标。
4. 生成小幅旋转目标。
5. 生成同时平移和旋转目标。
6. 检查 QP 输出 qdot 的方向。
7. 检查 qdot 不超过速度边界。
8. 检查积分后的 q_target 不越过关节限位。
9. 检查目标不变时 qdot 逐渐趋近于零。
10. 检查目标超时后停止积分。
11. 人为制造 QP 失败，确认进入 hold。

必须记录：

- 每帧 q。
- 左右末端 current pose。
- 左右 target pose。
- position error。
- rotation error。
- qdot。
- q_target。
- QP status。
- solve time。

### 阶段 E：MuJoCo 位姿闭环

目标：不接 XR，使用程序生成的 PoseStamped 验证双臂位姿控制。

任务：

1. 只控制左臂。
2. 只控制右臂。
3. 双臂同时控制。
4. 发送微小平移目标。
5. 发送微小旋转目标。
6. 发送持续圆周或小范围轨迹。
7. 测试目标停止时是否停止漂移。
8. 测试目标超时是否 hold。
9. 测试目标跳变是否被限幅。

重点观察：

- 位置误差是否收敛。
- 旋转误差是否收敛。
- 只转动目标时位置是否产生非预期明显变化。
- 只平移目标时姿态是否产生非预期明显变化。
- 手腕、肘部、肩部是否有高频抖动。
- MuJoCo Control 栏的关节命令是否异常跳变。

### 阶段 F：Quest 3 原始输入验证

目标：只验证输入，不接 IK。

任务：

1. 启动 PC Service。
2. 连接 Quest 3。
3. 启动 PXREA adapter。
4. 检查左右 pose 的频率。
5. 检查 Joy 的 axes 和 buttons。
6. 检查断开、重连和 Quest 睡眠唤醒。
7. 记录原始 pose 是否出现跳变。
8. 检查 pose 四元数是否归一化和 finite。

验收：

- 左右手柄区分正确。
- 左右 pose frame_id 一致。
- Grip 按下和释放状态正确。
- 原始输入频率满足 bridge 的 freshness 要求。
- 断连时有明确超时，不保留旧 pose 继续控制。

### 阶段 G：坐标变换和 clutch 验证

目标：在接入 IK 前把方向问题完全定位在 bridge 层。

任务：

1. 启动 XR adapter。
2. 启动 pose clutch bridge。
3. 不启动 IK，直接观察 target PoseStamped。
4. 用左 Grip 建立左侧 anchor。
5. 只移动左手柄 X。
6. 只移动左手柄 Y。
7. 只移动左手柄 Z。
8. 分别绕三个轴旋转。
9. 对右侧重复。
10. 松开 Grip，确认 target 停止。
11. 重新按 Grip，确认新 anchor 生效且没有跳跃。

当前必须重点验证的修正：

- X 平移取反。
- 左手 Y 平移取反。
- 右手 Y 平移不取反。
- 三个旋转轴的方向和速度。

### 阶段 H：Quest 到 MuJoCo 双臂闭环

目标：完成第一版真实手柄遥操仿真。

节点链：

~~~text
qiling_xrobotoolkit_pxrea_adapter
        |
        +--> /xr/* pose
        +--> /xr/controller_joy
                    |
                    v
qiling_xr_pose_clutch_bridge
        |
        +--> /teleop/*_wrist_target
        +--> /teleop/*_control_mode
                    |
                    v
qiling_differential_ik
        |
        v
/human_lower_command
                    |
                    v
qiling_mujoco_simulator
~~~

任务：

1. 先只激活左臂。
2. 验证左 Grip clutch。
3. 验证左手位置和姿态。
4. 再只激活右臂。
5. 验证右 Grip clutch。
6. 最后同时激活双臂。
7. 在 home 附近做小范围运动。
8. 逐步扩大 workspace。
9. 检查输入超时。
10. 检查手柄追踪跳变。
11. 检查暂停/恢复。

验收：

- 不按 Grip 时手臂不缓慢漂移。
- 按下 Grip 的瞬间不跳变。
- 释放 Grip 后目标保持。
- 双臂方向符合定义。
- 平移和旋转都可以控制。
- 末端不会持续出现无来源运动。
- 关节命令不会大范围跳变。

### 阶段 I：O6 仿真接入

目标：先在 MuJoCo 中完成双臂和 O6 的统一命令链。

任务：

1. 读取 qiling_grasp_ws 中 O6 控制模式。
2. 确认真实 O6 ROS 2 命令消息。
3. 确认 MuJoCo O6 joint name。
4. 建立左右 O6 的显式 joint map。
5. 设计 O6 输入到手指关节的映射。
6. 第一版使用开合或抓取模式。
7. 再实现连续手指角度控制。
8. 让 O6 命令进入统一 command mux。
9. 检查双臂 IK 和 O6 不相互覆盖。
10. 检查 40 维命令每一槽的来源。

第一版可以采用：

~~~text
左 trigger  -> 左 O6 抓取开合
右 trigger  -> 右 O6 抓取开合
或：
左/右指定按键 -> 预设 O6 grasp mode
~~~

高质量数据采集阶段应优先使用连续手指状态，而不是只有几个离散模式。

### 阶段 J：真实机器人接入

目标：把经过仿真验证的双臂和 O6 控制接入真实机器人。

任务：

1. 确认真实机器人 ROS 2 Humble 环境。
2. 确认真实 /joint_states 或等效状态话题。
3. 确认真实双臂控制话题。
4. 确认 MITJointCommands 的 40 维排列。
5. 确认 arm SDK 50 Hz 上限。
6. 确认 O6 命令的频率和失联保护。
7. 确认真实 robot_base / base_link 的 TF。
8. 确认真实末端 frame 和 URDF frame 一致。
9. 在无负载、低 kp、小范围下测试。
10. 先单臂，再双臂，再接 O6。
11. 加入硬件急停和软件 deadman。
12. 设定 workspace、关节、速度和加速度限制。

真实机器人第一轮不得直接使用仿真中的全部速度和增益。必须从低速、低增益、短行程开始。

---

## 12. 数据录制与 LeRobot 转换

### 12.1 录制原则

录制器只保留后续训练需要的数据，不记录所有调试数据到最终数据集。

推荐把数据分成两层：

调试层，可选：

- 原始 XR pose。
- bridge 生成的 target pose。
- current wrist pose。
- position/rotation error。
- qdot。
- QP status。
- 节点时间戳。
- 丢帧、超时和安全状态。

训练层，必须：

- 相机观测。
- 机器人当前关节位置。
- 机器人当前关节速度，若训练任务需要。
- O6 当前关节或手部状态。
- action，即希望机器人执行的关节/末端动作表示。
- 时间戳。
- episode、frame、task 等元数据。

不建议放入训练层：

- 实际发送的 MIT kp。
- 实际发送的 MIT kd。
- MIT velocity 全部细节。
- MIT effort 全部细节。
- QP 内部矩阵。
- 所有 ROS 诊断日志。

### 12.2 Action 表示

首选 action 表示为语义化的机器人目标：

方案 1：双臂关节目标 + O6 目标

~~~text
left_arm_q_target  7
right_arm_q_target 7
left_o6_target     N
right_o6_target    N
~~~

优点：

- 与真实执行器接口接近。
- 容易复现。
- 不把底层 MIT 参数混入行为数据。
- 离线转换简单。

方案 2：双腕末端位姿 + O6 目标

~~~text
left_wrist_pose    7
right_wrist_pose   7
left_o6_target     N
right_o6_target    N
~~~

优点：

- 更接近 Quest 遥操意图。
- 适合学习末端目标。

缺点：

- 训练时还需要在线 IK。
- 数据重放时要确保 IK 版本和关节限制一致。

第一阶段建议同时保存中间语义字段，但在导出 LeRobot 时明确选定一种 action 定义，不能一会儿使用 q_target、一会儿使用实际 q。

### 12.3 观测字段

当前只有头部 D435 相机，因此第一阶段建议：

~~~text
observation.images.head
observation.state
action
timestamp
episode_index
frame_index
task
~~~

如果使用关节状态作为 observation：

~~~text
observation.state =
  left_arm_position[7]
  right_arm_position[7]
  left_o6_position[N]
  right_o6_position[N]
~~~

若后续真实机器人有两个腕部 D405，再增加：

~~~text
observation.images.left_wrist
observation.images.right_wrist
~~~

但是录制器和相机发布器必须同时检查时间戳、帧率、图像编码和相机内参。

### 12.4 录制时钟和同步

录制器应使用单一时间基准：

- ROS 2 message header stamp 作为消息时间。
- 录制器到达时间只作为接收延迟诊断。
- 以相机帧为主或以机器人状态为主，必须提前固定。
- 每一帧记录最近的机器人状态和 action。
- 如果超过同步窗口，标记无效或丢弃，不静默混配。

第一版建议：

~~~text
相机帧到达 -> 查找最近机器人 state/action -> 写入一帧
~~~

后续可以改成固定 30 Hz 或 50 Hz 的统一采样时钟。

### 12.5 独立转换脚本

录制流程：

~~~text
ROS 2 topics
    |
    v
raw episode directory
    |
    v
validate script
    |
    v
convert_to_lerobot.py
    |
    v
LeRobot dataset
~~~

转换脚本必须完成：

1. 检查每个字段长度。
2. 检查字段是否 finite。
3. 检查图像数量和状态帧数量。
4. 检查时间戳单调递增。
5. 检查 action / observation 的维度。
6. 检查 episode 是否有明确终止原因。
7. 检查任务名称和语言描述。
8. 输出转换统计。
9. 对缺失帧、丢帧和异常关节值报错或明确标记。

转换前应保存数据字典，说明：

- 每一维的 joint name。
- 单位。
- 参考坐标系。
- action 语义。
- O6 关节排列。
- 图像 topic 和相机内参。

---

## 13. 头部 D435 和后续腕部 D405

### 13.1 当前头部相机

MuJoCo 中的相机名：

~~~text
d435_camera
~~~

相机位姿已按以下 URDF 作为参考：

~~~text
src/qi_robot_description/urdf/s4_40DOF_fullbody_with_handeye_camera.urdf
~~~

当前 MuJoCo 相机近似配置：

~~~text
pos  = 0.0240925853, 0.0044939526, 0.6924775167
fovy = 58
size = 640 x 480
~~~

相机发布器的静态 TF 也必须和 MuJoCo 相机外参一致。不能只修改 XML 而不修改 camera_info 或静态 TF。

### 13.2 躯干可视化

为了避免头部相机被躯干模型遮挡，MuJoCo 当前已将躯干主要视觉网格设为透明或不可见，同时保留必要的碰撞/结构配置。后续如果需要更准确地保留碰撞：

- 可以只隐藏视觉 geom。
- 保留碰撞 geom。
- 不应为了画面清晰而删除影响机械臂碰撞的真实几何。

### 13.3 Quest 视频问题

当前 D435 publisher 只发布 ROS 2 图像。它不能自动完成：

- 把图像编码后发送到 Quest 3。
- 在 Quest 中显示平面画面。
- 实现低延迟视频回传。

如果将来需要 Quest 画面，单独增加视频链路：

~~~text
ROS 2 image
    |
    v
硬件/软件编码 H.264 或 H.265
    |
    v
局域网低延迟传输
    |
    v
Quest 客户端平面纹理显示
~~~

由于当前已经决定先不考虑视频画面传入，第一阶段不要把视频显示耦合到控制闭环中。控制链路即使没有 Quest 画面也必须可以独立运行和录制。

---

## 14. 性能、实时性和线程设计

### 14.1 频率分层

推荐频率：

~~~text
MuJoCo 内部积分             1 kHz
XRoboToolkit pose 发布       90 Hz
clutch/目标位姿更新          50 Hz
Pinocchio + ProxQP IK        50 Hz
真实手臂 ROS 2 命令          <= 50 Hz
相机发布                     30 Hz
数据集导出                   离线
~~~

### 14.2 C++ 实时路径

50 Hz 控制循环每周期预算为 20 ms。控制循环中应避免：

- 频繁动态分配大对象。
- 文件 IO。
- 大量 printf。
- 等待图像。
- 等待 XR 网络回调。
- 等待 service。
- 调用 Python。
- 运行 IPOPT 等非线性优化器。

推荐：

- 节点启动时预分配 Eigen / ProxQP 工作区。
- 回调只更新最新状态，并使用互斥锁或无锁快照。
- 控制线程固定周期运行。
- 把详细日志降频到 1 Hz 或按需开启。
- 发布控制命令前统一做 finite 和范围检查。
- 记录 solve time 的最大值、均值和 P99。

### 14.3 是否需要 C++

当前直接使用 C++ 是合理的，原因：

- 机器人控制上限为 50 Hz，但需要稳定低延迟。
- Pinocchio 和 ProxQP 都有成熟 C++ 接口。
- 可以减少 Python GC、对象分配和跨语言数据拷贝。
- 后续真实机器人接入不需要再替换 IK 主路径。

PyRoki、Python 差分 IK 或 CasADi + IPOPT 可以作为离线验证工具，但不应进入当前实时控制主链。

### 14.4 运行时监控

每个周期至少监控：

- loop period。
- solve time。
- qdot max。
- q_target 与 q 的差异。
- position error。
- rotation error。
- target age。
- joint state age。
- QP status。
- command publish count。

日志示例应降频输出，不应每帧打印：

~~~text
Cartesian error:
left position=...
left rotation=...
right position=...
right rotation=...
solve_ms=...
target_age_ms=...
~~~

---

## 15. 安全机制

### 15.1 软件安全边界

必须具备：

- 关节位置上下限。
- 关节限位 margin。
- 最大关节速度。
- 最大末端平移误差。
- 最大末端旋转误差。
- 目标变化速率限制。
- qdot 低通或加速度限制。
- XR 输入超时 hold。
- JointState 超时 hold。
- QP 失败 hold。
- 命令 finite 检查。
- command mux 唯一发布者。

### 15.2 clutch 安全

Grip 松开时：

- 不把手柄当前位置继续作为目标。
- 不使用旧 qdot。
- 目标重置到当前机器人末端状态。
- 模式变为 hold。

Grip 再按下时：

- 重新建立 anchor。
- 第一个 active 目标等于当前 robot anchor。
- 禁止跨越按下瞬间产生大位移。

### 15.3 真实机器人安全顺序

~~~text
无动力 / 仿真
    |
    v
单臂、低速度、低 kp
    |
    v
单臂完整位姿
    |
    v
双臂低速
    |
    v
加入 O6
    |
    v
小范围数据录制
    |
    v
正常速度和任务
~~~

任何出现以下情况都应立刻释放 Grip 或急停：

- 末端快速跳变。
- 关节持续向限位运动。
- q_target 与实际 q 长时间分离。
- 手柄失联但机器人仍运动。
- O6 手指持续闭合。
- 双臂或手指碰撞。

---

## 16. 调试和验收方法

### 16.1 ROS 2 图检查

启动系统后检查：

~~~bash
ros2 node list
ros2 topic list
ros2 topic info -v /human_lower_command
ros2 topic info -v /joint_states
ros2 topic hz /joint_states
ros2 topic hz /xr/left_controller_pose
ros2 topic hz /xr/right_controller_pose
~~~

重点确认：

- /human_lower_command 只有一个有效 publisher。
- /joint_states 有且只有预期的状态源。
- /teleop target 没有重复 publisher。
- XR pose 频率稳定。
- 无其他旧版 teleop 节点在后台运行。

### 16.2 手柄输入检查

~~~bash
ros2 topic echo /xr/controller_joy
ros2 topic echo /xr/left_controller_pose
ros2 topic echo /xr/right_controller_pose
~~~

检查：

- 左右 Grip 是否分别变化。
- axes[2] 和 axes[3] 是否是模拟握把值。
- buttons[4] 和 buttons[5] 是否是数字握把值。
- pose 的位置和四元数是否 finite。
- 左右手柄没有交换。

### 16.3 目标位姿检查

~~~bash
ros2 topic echo /teleop/left_wrist_target
ros2 topic echo /teleop/right_wrist_target
~~~

在松开 Grip 时，目标不应持续变化。按住 Grip 后，只进行一个轴的动作时，应只观察到对应方向的变化。

### 16.4 IK 检查

观察：

- differential IK 是否加载 s4_dual_arm.urdf。
- 左右 frame 是否正确。
- Cartesian error 是否在目标不动时收敛到接近零。
- qdot 是否在合理范围内。
- QP 是否出现失败。
- 目标超时后 q_target 是否停止积分。

### 16.5 典型问题定位

问题：不按 Grip 手臂仍缓慢移动。

排查：

1. bridge 是否发布了 mode=1。
2. bridge 是否继续发布变化的 target。
3. IK 是否在 hold 时仍积分 qdot。
4. q_target 是否每帧被错误覆盖。
5. 是否有第二个 publisher。
6. MuJoCo 是否仍在执行上一帧 MIT 命令。

问题：只旋转时位置明显移动。

排查：

1. target PoseStamped 的 position 是否真的不变。
2. robot wrist frame 是否位于预期位置。
3. Pinocchio Jacobian reference 是否正确。
4. rotation error 是否被错误写到线速度部分。
5. QP 是否存在奇异位形。
6. 是否需要 posture/null-space 或任务优先级。

问题：绕 base_link X 轴方向反。

排查：

1. XR 原始旋转方向。
2. M 基础轴变换。
3. anchor_basis_change。
4. rotation_sign_x。
5. PoseStamped 的 quaternion 组合顺序。
6. MuJoCo 和 URDF frame 的轴定义。

问题：加载后左臂不在 home 或无法控制。

排查：

1. 是否真的调用了 /unpause_mujoco。
2. homeTransition 是否完成。
3. qpos keyframe 长度是否为 47。
4. left arm 的 XML 关节顺序是否正确。
5. differential IK 是否收到对应的 JointState name。
6. q_target 是否在第一帧状态时初始化。

---

## 17. 推荐的后续实现顺序

当前建议严格按照以下顺序继续：

### 下一步 1：冻结当前双臂位姿闭环

- 不改变当前 Quest 映射。
- 不再增加平移/旋转模式。
- 增加/保留日志和诊断。
- 完成坐标和双臂闭环验收。

### 下一步 2：完善 IK 数值质量

- 增加 qdot 平滑。
- 增加 posture/null-space 目标。
- 评估任务误差和非期望方向运动。
- 检查奇异位形。
- 确定真实机器人安全参数。

### 下一步 3：接入 O6

- 先检查 qiling_grasp_ws。
- 记录 O6 topic、message、joint map。
- 先实现仿真中的 trigger 或 grasp mode。
- 再将 O6 加入唯一 command mux。

### 下一步 4：完善数据录制

- 定义 observation.state。
- 定义 action。
- 建立 raw episode 格式。
- 录制头部 D435。
- 写 validate 脚本。
- 写 LeRobot 转换脚本。

### 下一步 5：真实机器人单臂测试

- 仅左臂。
- 低速度、低增益。
- 无 O6 或只使用安全开合模式。
- 验证 clutch、超时和急停。

### 下一步 6：真实双臂和 O6

- 右臂。
- 双臂。
- O6。
- 小范围任务。
- 最终数据质量验收。

---

## 18. 当前完成项和未完成项

### 已完成或已有实现

- MuJoCo 场景路径已切换到 qi_robot_description/new_scene/scene_S4_40DOF_fullbody.xml。
- MuJoCo 支持固定 base。
- MuJoCo 支持冻结腿部。
- 保留完整 40 维 /human_lower_command 接口。
- 解锁后支持多阶段 home 过渡。
- 当前 home 关节值已更新为用户最后指定的双臂值。
- 头部 D435 的 MuJoCo 位姿已经参考带 hand-eye camera 的 URDF。
- 躯干主要视觉遮挡已处理。
- C++ Pinocchio 模型已使用 s4_dual_arm.urdf。
- C++ ProxQP 差分 IK 已建立。
- q_target 不再每帧被当前 q 错误覆盖。
- XRoboToolkit / PXREA SDK 适配器已建立。
- Quest 3 左右手柄已能连接并控制双臂。
- Grip clutch 已建立。
- 平移方向已按实测进行 X、左手 Y 的修正。
- 当前控制方式已经统一为完整 6D 位姿控制。

### 仍需完成

- 对当前统一 6D IK 做系统化误差、耦合和奇异位形验证。
- 进一步改善只旋转时的非期望位置变化。
- 加入 posture/null-space 或更合适的带软约束 QP 目标。
- 检查并确保没有后台旧节点造成重复发布。
- 明确 O6 的真实消息、关节数和映射。
- 实现 O6 仿真控制。
- 建立 40 维命令的唯一 mux。
- 完成真实机器人 50 Hz 控制链路。
- 完成头部 D435 的数据录制。
- 决定是否在后续加入两个腕部 D405。
- 完成 raw episode 到 LeRobot 的独立转换脚本。
- 在真实机器人上完成低速安全测试。

---

## 19. 设计原则

1. 先验证数据流，再调 IK；先验证坐标系，再调增益。
2. MuJoCo 和真实机器人共享同一套双臂关节名称、末端 frame 和 action 语义。
3. 仿真模型、Pinocchio URDF、真实机器人 URDF 的职责不能混用。
4. 所有左右臂关节都通过 joint name 显式映射。
5. 所有坐标变换都必须可记录、可复现、可单元测试。
6. 所有目标位姿都必须有时间戳、freshness 和跳变限制。
7. 所有控制命令都必须有唯一发布者。
8. 50 Hz 是真实手臂命令接口的硬约束。
9. MuJoCo 的 1 kHz 只代表仿真内部积分频率，不代表可以向真实 SDK 发送 1 kHz 命令。
10. O6 接入之前，必须先解决消息维度和关节映射，不能凭数组下标猜测。
11. episode 记录的是训练语义，不是底层控制器的全部内部变量。
12. 任何出现持续漂移、跳变、超限或失联运动的情况，都优先进入 hold/急停状态。

