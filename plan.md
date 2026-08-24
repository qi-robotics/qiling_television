# Quest 双臂灵巧手遥操作与 LeRobot 数据采集实施计划

## 1. 文档目的

本文档用于指导从零实现一套运行在机器人本体主机上的遥操作与数据采集系统。系统使用 Meta Quest 作为 XR 交互设备，参考 XRoboToolkit 的通信方式和 C++ 示例，通过 ROS 2 Humble 控制自研人形机器人的双臂与双侧 O6 灵巧手，并将三路相机、机器人状态和遥操作目标录制为 episode，最后离线转换为 LeRobot Dataset v3。

本计划按可独立验证的阶段组织。每个阶段必须完成验收项后，才进入下一阶段，避免 XR、IK、机器人硬件和数据采集同时调试。

## 2. 已确认的系统约束

- 操作设备：Meta Quest，优先按 XRoboToolkit 已验证的 Quest 3 路线实现。
- 机器人系统：Ubuntu 22.04、ROS 2 Humble。
- 双臂：每条手臂 7 个关节，共 14DoF。
- 灵巧手：左右各一只 O6，现有 ROS 2 控制话题和底层控制已经实现。
- 机械臂 SDK：ROS 2 接口最高接收 50 Hz 控制目标。
- 电机闭环：机器人内部已经存在高频闭环，上层不重做电机级控制。
- 遥操作目标：直接控制左右手臂末端位姿。
- IK：全 C++ 实现，使用 Pinocchio 做运动学，使用 ProxQP 做差分 IK/QP。
- 相机：头部 D435，左右腕部各一台 D405。
- 运行位置：XRoboToolkit PC Service、ROS 2、IK、机器人接口、图像服务和 episode 录制均运行在机器人本体主机；Quest 客户端运行在头显上。
- 数据：运行时录制最小必要原始 episode，离线转换为 LeRobot Dataset v3。
- 不采用 PyRoKi、PlaCo Python 或 TeleVuer 作为正式运行链路。
- 不要求保留实际发送的 MIT 命令、`kp/kd/tau`、电流、力矩等非训练字段。
- 当前目录中的 TeleVision、Isaac Gym 等内容不作为新实现的架构基础。

## 3. 最终运行链路

```text
Meta Quest
  ├── 头部 / 控制器 / 手部跟踪
  └── 机器人头部视频显示
          │
          │ XRoboToolkit 网络协议
          v
XRoboToolkit PC Service（机器人主机，C++）
          │ 最新 XR 帧：pose + tracking validity + source timestamp
          v
qiling_xr_bridge（ROS 2 C++）
          │
          v
qiling_teleop_controller（固定 50 Hz，C++）
  ├── clutch / 重定位 / 缩放
  ├── XR 坐标系到 robot_base 的变换
  ├── Pinocchio FK/Jacobian
  ├── ProxQP 双臂差分 IK
  ├── 关节、速度、工作空间和求解状态检查
  └── 生成双臂关节目标与 O6 目标
          │
          ├──> 现有手臂 ROS 2 SDK 接口（最高 50 Hz）
          └──> 现有 O6 ROS 2 控制话题

D435 + 左 D405 + 右 D405
          ├──> Quest 视频服务
          └──> Episode Recorder

机器人实际状态 + 三路 RGB + EE/O6 action + task
          │
          v
原始 episode（按 topic 白名单记录）
          │
          v
离线转换器（Conda Python 3.10）
          │
          v
LeRobot Dataset v3
```

### 3.1 频率划分

| 模块 | 目标频率 | 说明 |
|---|---:|---|
| Quest 跟踪输入 | 由设备决定，通常高于 50 Hz | 保存最新有效样本，不堆积历史命令 |
| 双臂控制与 IK | 50 Hz | 使用单调时钟，周期 20 ms |
| 手臂 SDK 目标发送 | 不超过 50 Hz | 与 SDK 上限严格一致 |
| O6 目标发送 | 由现有接口上限决定 | 可与手臂控制 tick 同步 |
| RGB 相机 | 初始 30 FPS | 最终按 USB 带宽和模型需求确定 |
| Episode 逻辑帧率 | 初始 30 FPS | 离线按时间戳对齐，不要求所有源同频 |
| 机器人内部电机闭环 | 已有高频闭环 | 不属于本项目实现范围 |

## 4. 推荐的新项目结构

```text
qiling_television/
├── README.md
├── plan.md
├── LICENSE
├── assets/
│   └── robot/
│       ├── urdf/
│       ├── meshes/
│       └── srdf/                         # 可选：碰撞对和规划组
├── configs/
│   ├── robot.yaml                        # joint/link/frame 名称及顺序
│   ├── xr.yaml                           # Quest 输入、缩放、clutch 配置
│   ├── ik.yaml                           # QP 权重、速度、阻尼和误差阈值
│   ├── safety.yaml                       # 限位、超时、工作空间和状态机
│   ├── cameras.yaml                      # serial、分辨率、FPS、topic
│   ├── o6.yaml                           # O6 映射、方向、零位和限幅
│   └── recording.yaml                    # topic 白名单和 episode 参数
├── calibration/
│   ├── xr_to_robot.yaml
│   ├── camera_extrinsics.yaml
│   └── o6_mapping.yaml
├── ros2_ws/
│   └── src/
│       ├── qiling_interfaces/            # 必要的自定义 msg/srv/action
│       ├── qiling_description/           # URDF、robot_state_publisher
│       ├── qiling_xr_bridge/             # XRoboToolkit C++ -> ROS 2
│       ├── qiling_kinematics/             # Pinocchio + ProxQP C++ 库
│       ├── qiling_teleop_controller/      # 50 Hz 控制、安全状态机
│       ├── qiling_robot_adapter/          # 现有手臂 SDK 接口适配
│       ├── qiling_o6_adapter/             # 现有 O6 topic 适配与映射
│       ├── qiling_camera/                 # RealSense 启动与视频桥接
│       ├── qiling_episode_recorder/       # episode 开始/结束和 rosbag2 白名单
│       └── qiling_bringup/                # launch、参数与生命周期编排
├── tools/
│   ├── lerobot_converter/                # 原始 episode -> LeRobot v3
│   ├── calibration/                      # XR、相机、O6 标定工具
│   ├── replay/                           # XR/状态/action 回放工具
│   └── benchmark/                        # IK、延迟、频率、丢帧测试
├── tests/
│   ├── unit/
│   ├── integration/
│   ├── recorded_inputs/
│   └── expected_outputs/
├── third_party/
│   └── README.md                         # 只记录外部依赖版本和获取方式
└── docs/
    ├── interfaces.md
    ├── coordinate_frames.md
    ├── safety.md
    ├── calibration.md
    ├── recording.md
    └── operations.md
```

原则：不直接修改 XRoboToolkit、Pinocchio、ProxQP 或机器人 SDK 的源代码。通过 adapter 封装第三方接口，并锁定可复现的 commit/tag。

## 5. 进程与线程边界

### 5.1 建议进程

1. `xrobotoolkit_pc_service`：接收 Quest 数据。
2. `qiling_xr_bridge`：将 C++ callback 数据转换为带时间戳的 ROS 2 消息。
3. `qiling_teleop_controller`：唯一的 50 Hz 双臂目标生成器。
4. `qiling_robot_adapter`：连接现有手臂 SDK 话题。
5. `qiling_o6_adapter`：连接现有 O6 控制与反馈话题。
6. 三个 RealSense 节点：按 serial 固定头部、左腕和右腕相机。
7. Quest 视频桥接进程：只负责操作者画面，不参与训练数据采样。
8. `qiling_episode_recorder`：独立进程，避免图像编码影响控制循环。
9. `robot_state_publisher`：提供机器人 TF，仅保留一个发布源。

### 5.2 控制器内部线程

- XR 接收回调只更新“最新完整样本”，不在回调中求 IK。
- ROS 2 状态回调只更新关节/O6 状态缓存。
- 独立 50 Hz 控制线程读取一次一致快照，完成目标生成、IK、安全检查和发布。
- 控制 tick 中避免动态内存分配、磁盘 I/O、日志刷屏和阻塞网络调用。
- ProxQP problem structure、Eigen 缓冲区和 Pinocchio data 在启动阶段预分配。
- 图像采集、视频编码和数据写盘不得运行在控制线程。

## 6. ROS 2 接口设计原则

具体 topic 名称以现有 SDK 为准，下面是项目内部的规范化接口。

### 6.1 XR 输入

建议定义一个完整的 `XRFrame`，一次携带同一源时刻的设备状态：

```text
header.stamp
source_timestamp_ns
head_pose
left_controller_pose
right_controller_pose
left_hand_joints[]
right_hand_joints[]
head_valid
left_valid
right_valid
buttons / triggers / clutch state
```

如果 XRoboToolkit 输出的是 JSON，只在 `qiling_xr_bridge` 中解析一次。控制器内部不使用 JSON。

建议 topic：

```text
/qiling/xr/frame
/qiling/xr/status
```

QoS：`KEEP_LAST(1)`，优先新鲜度，禁止旧 XR 数据排队进入控制器。

### 6.2 机器人状态

```text
/joint_states                         # 双臂实际关节位置，名称必须稳定
/o6/left/state                       # 现有接口或 adapter 后的统一接口
/o6/right/state
```

双臂关节不能依赖 `JointState.position[]` 的偶然排列，必须按 `name` 映射到配置文件定义的固定 14 维顺序。

### 6.3 遥操作 action

训练数据所记录的 action 是“控制器在该 tick 接受并用于求解的任务空间目标”，不是 Quest 原始姿态，也不是 MIT 底层命令：

```text
/qiling/teleop/action
  stamp
  left_ee_target_in_robot_base       # xyz + quaternion
  right_ee_target_in_robot_base      # xyz + quaternion
  left_o6_target[]
  right_o6_target[]
  valid
```

### 6.4 控制输出

```text
/qiling/arm/joint_target             # adapter 内部接口，14 维 q_target
/existing_arm_sdk_command_topic      # 现有 SDK topic
/existing_o6_left_command_topic
/existing_o6_right_command_topic
```

现有 SDK 如果要求 MIT 消息，则由 `qiling_robot_adapter` 填充所需字段。MIT 消息不进入训练 episode。

### 6.5 控制服务

```text
/qiling/teleop/arm
/qiling/teleop/disarm
/qiling/teleop/enable
/qiling/teleop/disable
/qiling/teleop/reset_fault
/qiling/episode/start
/qiling/episode/stop
/qiling/episode/discard
```

服务名称可调整，但必须区分“控制使能”和“数据录制”，不能因为开始录制而自动使能机器人。

## 7. 坐标系与遥操作语义

### 7.1 必须固定的坐标系

- `robot_base`：双臂 IK 的统一参考坐标系。
- `left_ee`、`right_ee`：URDF 中用于任务空间控制的末端 frame。
- `xr_origin`：Quest 会话原点。
- `xr_head`、`xr_left_controller`、`xr_right_controller`。
- `camera_head_color_optical_frame`。
- `camera_left_wrist_color_optical_frame`、`camera_right_wrist_color_optical_frame`。

在 `docs/coordinate_frames.md` 明确每个坐标系的：

- 右手系/左手系；
- 轴方向；
- 四元数顺序，统一为内部 Eigen/ROS 约定；
- pose 表示的是 `T_parent_child` 还是其逆；
- 长度单位，统一为米；
- 时间戳来源。

### 7.2 相对遥操作

启动遥操作时，不将 Quest 的绝对空间位置直接映射到机器人。采用相对映射：

```text
T_robot_target(t)
  = T_robot_ee_at_clutch
  * ScaleAndRotate(
      inverse(T_xr_hand_at_clutch) * T_xr_hand(t)
    )
```

要求：

- clutch 按下时冻结机器人目标并重新记录 XR/机器人锚点；
- 支持平移缩放和旋转缩放独立配置；
- 左右臂锚点独立保存，但使能状态统一管理；
- Quest tracking origin 重置后必须重新 clutch，禁止产生跳变；
- 四元数归一化，并在相邻帧保持符号连续。

## 8. C++ 差分 IK 设计

### 8.1 基本形式

QP 决策变量初始为双臂 14 维关节速度 `qdot`，必要时加入末端任务松弛变量。

每个控制 tick：

1. 从实际 `q` 做 Pinocchio FK。
2. 计算左右末端当前位姿与目标位姿之间的 SE(3) 误差。
3. 对位置误差和旋转 log-map 误差施加增益与速度限幅，得到期望末端 twist。
4. 计算左右末端 Jacobian。
5. 构建并 warm-start ProxQP。
6. 求解 `qdot`。
7. 根据 20 ms 周期积分得到 `q_target`。
8. 再次执行位置、速度、加速度和单周期步长限制。
9. 只有状态有效且 QP 通过验收，才发布目标。

基本目标函数：

```text
minimize
    w_left  * ||J_left  qdot - v_left_des ||^2
  + w_right * ||J_right qdot - v_right_des||^2
  + w_posture * ||qdot - qdot_posture||^2
  + w_smooth  * ||qdot - qdot_previous||^2
  + w_damping * ||qdot||^2
  + slack penalties
```

基本约束：

- 关节位置预测约束：`q_min + margin <= q + dt*qdot <= q_max - margin`；
- 关节速度上下限；
- 单周期加速度/速度变化限制；
- 可选工作空间线性约束；
- 后续加入自碰撞距离线性化约束。

### 8.2 拟人化动作

每条 7DoF 手臂有冗余自由度。拟人化优先通过次级目标实现：

- 肩部和肘部默认姿态 `q_nominal`；
- 肘部 frame 的位置/平面偏好；
- 远离关节限位的代价；
- 远离奇异位形的阻尼或可操作度代价；
- 左右肩部动作对称性只作为软约束，不强制对称；
- 双手共同操作物体时，可增加左右末端相对位姿软任务。

第一版不加入复杂碰撞约束。先保证单臂和双臂 QP 稳定，再加入躯干/双臂自碰撞线性化，以便定位性能和不可行问题。

### 8.3 求解失败处理

下面任一条件触发本 tick 无效：

- ProxQP 非成功状态；
- 求解时间超过配置阈值；
- 末端残差超过阈值；
- 输出包含 NaN/Inf；
- `q_target` 越界；
- 单 tick 关节变化超过上限；
- 连续状态或 XR 数据过期。

短暂单次失败可以保持上一安全目标；连续失败达到阈值后进入 `FAULT`，停止接受新的遥操作目标。不能在求解失败时继续积分上一次 `qdot`。

### 8.4 性能目标

- 控制周期：20 ms。
- QP/IK 平均耗时：目标小于 2 ms。
- QP/IK P99：目标小于 5 ms。
- 完整控制 tick P99：目标小于 10 ms。
- 30 分钟运行期间控制 deadline miss 比例：小于 0.1%。
- 运行中不得出现周期性动态分配导致的明显尖峰。

这些是项目验收目标，不是未经实测的性能承诺；最终数值必须在机器人本体主机上测量。

## 9. 安全状态机

### 9.1 状态

```text
DISCONNECTED
    -> STANDBY
    -> ARMED
    -> TELEOP
    -> STANDBY

任意活动状态 -> FAULT
硬件急停     -> ESTOP
```

- `DISCONNECTED`：缺少机器人状态、XR 或 SDK。
- `STANDBY`：只读状态，不发送活动目标。
- `ARMED`：依赖检查通过，但遥操作仍未开始。
- `TELEOP`：50 Hz 发布目标。
- `FAULT`：软件故障，需要明确复位。
- `ESTOP`：硬件急停或底层急停，软件不得自动解除。

### 9.2 使能条件

进入 `TELEOP` 前同时满足：

- 急停已释放，但不由本程序主动解除；
- 双臂状态时间戳新鲜；
- O6 状态有效；
- Quest 左右跟踪有效；
- URDF 关节顺序与反馈完全匹配；
- 当前关节在软件限位内；
- 当前 FK 与初始 target 一致，不会使能即跳动；
- 用户完成 clutch/锚点初始化；
- SDK adapter 已连接并报告可接受目标；
- IK 已完成一次无输出的 dry-run。

### 9.3 看门狗与限制

所有阈值写入 `safety.yaml`，初始建议值必须通过低速测试校准：

- XR 样本过期阈值；
- 机器人状态过期阈值；
- 最大末端线速度、角速度；
- 每关节最大速度、加速度、单 tick 步长；
- 关节软限位 margin；
- 左右手允许工作空间；
- 连续 IK 失败次数；
- SDK 发布失败次数；
- 网络断开后的保持时间和退出策略。

Quest 手势不能替代物理急停。正式硬件测试必须有机器人旁的物理急停和一名观察人员。

## 10. O6 灵巧手接入

O6 底层控制已经存在，本项目只实现 XR 手部输入到现有控制 action 的映射。

### 10.1 接口调查

接入前必须记录：

- 每只手的控制维度和反馈维度；
- topic 名称、消息类型和 QoS；
- 每个控制量的物理含义、方向、单位、零位和范围；
- 是否存在耦合、欠驱动或内部协同控制；
- 最大命令频率；
- 超时后的底层行为；
- 是否能读取实际位置/开合量。

### 10.2 第一版映射

1. 从 Quest/OpenXR 手骨架计算每根手指的归一化弯曲量。
2. 根据 O6 的实际控制空间做线性或分段线性映射。
3. 使用每只手独立的零位、方向和范围配置。
4. 增加低通、死区、速率限制和饱和。
5. tracking 无效时保持短时间，随后进入配置的安全手型。

如果 O6 是耦合手，不直接尝试逐关节复制 OpenXR 26 个关节，而是先映射到 O6 可控的低维 action。

### 10.3 验收

- 手完全张开、自然弯曲、握拳、拇指对掌等关键姿态方向正确；
- tracking 抖动不会导致 O6 高频振荡；
- 左右手镜像关系正确；
- 达到限位时无积分累积；
- 录制的 O6 action 与真正提供给现有 O6 控制接口的高层目标一致。

## 11. 相机与 Quest 视频

### 11.1 RealSense 配置

- 用 serial number 固定三台相机角色，禁止按 `/dev/video*` 顺序识别。
- 第一版仅采集训练需要的 RGB，不录深度和无关红外流。
- 初始建议 640×480 或模型目标分辨率、30 FPS。
- 分别验证每台相机，再同时运行三台相机。
- 检查三台设备是否分布在足够的 USB 控制器上。
- 记录相机 ROS header timestamp 和接收时的单调/ROS 时间关系。
- 相机外参标定结果版本化保存，不逐帧写入数据集。

### 11.2 D435 显示限制

D435 提供单路 RGB，不等于双目彩色视频。第一版 Quest 显示采用以下一种方式：

- 将头部 RGB 显示在 Quest 虚拟面板中；或
- 将同一 RGB 图像送入左右眼纹理，提供单目远程画面。

如果后续要求真实立体彩色视频，需要增加适合的双目彩色相机；该需求不阻塞双臂控制和数据采集。

### 11.3 视频链路与数据链路分离

- Quest 视频可以压缩、降分辨率或丢帧，以低延迟优先。
- Episode 必须从相机原始 ROS 图像 topic 采集，不记录 Quest 显示端二次编码画面。
- 视频编码阻塞不得影响控制线程。

## 12. Episode 最小数据规范

### 12.1 训练字段

最终 LeRobot v3 每帧只包含：

| 字段 | 内容 | 维度/形式 |
|---|---|---|
| `observation.images.head` | D435 RGB | image/video frame |
| `observation.images.left_wrist` | 左腕 D405 RGB | image/video frame |
| `observation.images.right_wrist` | 右腕 D405 RGB | image/video frame |
| `observation.state` | 双臂实际关节位置 + 左右 O6 实际状态 | `14 + N_state_left + N_state_right` |
| `action` | 左右 EE 目标位姿 + 左右 O6 目标 | `14 + N_action_left + N_action_right` |
| `task` | episode 任务文本 | string/index |

左右 EE action 均使用 `robot_base` 下的绝对 `xyz + quaternion`。四元数必须归一化并做符号连续处理。

### 12.2 明确不进入训练数据的内容

- 实际发送的 MIT 命令；
- `kp`、`kd`、`tau`；
- 电机电流、温度和诊断量；
- IK 输出关节目标；
- Quest 原始头部、控制器和手骨架 pose；
- QP Hessian、Jacobian、残差和求解器内部状态；
- 关节速度，除非后续模型明确把它作为 observation；
- 实际 EE pose，因为可由实际关节位置和 URDF 重算；
- 深度和红外图像，除非后续训练明确需要。

软件运行日志可以单独保留故障摘要，但不能混入 LeRobot feature schema。

### 12.3 原始 episode 必须保留的同步元数据

以下数据不作为模型输入，但离线转换不可缺少：

- 每条消息的源时间戳和录制时间戳；
- episode 开始、结束和 discard 状态；
- task 文本；
- 相机 serial 与 topic 的映射；
- robot/config/calibration 版本标识；
- 丢帧和时间戳回退统计。

### 12.4 原始录制 topic 白名单

```text
/camera/head/color/image_raw
/camera/left_wrist/color/image_raw
/camera/right_wrist/color/image_raw
/joint_states
/o6/left/state
/o6/right/state
/qiling/teleop/action
/qiling/episode/event
```

topic 名称最终按实际驱动修改。禁止使用“录制全部 ROS 2 topics”的方式。

### 12.5 时间对齐规则

- 机器人主机使用统一 ROS time，并同步系统时钟。
- 原始录制保留各源原始时间戳，不在写盘时强行伪造同频帧。
- 离线转换以目标数据集 FPS 建立时间轴。
- `observation.state` 和 `action` 使用最近邻或零阶保持，规则必须固定。
- 图像选择最近帧，并记录最大允许时间差；超出阈值的帧或 episode 判为无效。
- action 取“控制器该 tick 真正接受的 EE/O6 高层目标”，不取 90 Hz Quest 原始目标。
- 转换报告必须包含各源时间差分布和丢帧率。

## 13. 环境管理

### 13.1 实时运行环境

- 使用系统 ROS 2 Humble：`/opt/ros/humble`。
- C++ 工程使用 `colcon`、`ament_cmake` 和 C++17。
- Pinocchio、ProxQP、Eigen 和 XRoboToolkit PC Service 锁定版本。
- 优先通过 apt/rosdep 安装；没有合适系统包时建立独立 vendor package。
- 正式 ROS 2 控制进程不从 Conda 环境启动，避免 Python、libstdc++ 和动态库冲突。

### 13.2 离线转换环境

- 可以在机器人主机安装 Conda。
- 建立独立 Python 3.10 环境用于 LeRobot v3 转换、校验和上传。
- 不在该环境中重新安装 ROS 2 Humble。
- 转换器优先直接读取 rosbag2/原始 episode 文件，不要求在 Conda 中运行 `rclpy`。
- 锁定 LeRobot、PyTorch、视频编码依赖和数据格式版本。

### 13.3 Quest 客户端

- 从 XRoboToolkit 官方 Quest Unity Client 的已验证 Unity/Meta XR/Oculus 插件版本起步。
- Quest 应用可以在单独开发机上构建，运行时只要求头显连接机器人主机 PC Service。
- 如果设备不是 Quest 3，先做兼容性验证，不默认视为官方已验证硬件。

## 14. 分阶段实施步骤

## 阶段 0：冻结硬件与接口事实

### 任务

- 确认 Quest 具体型号和系统版本。
- 收集完整双臂 URDF、mesh、joint limit 和末端 frame。
- 收集手臂 SDK ROS 2 topic、消息定义、QoS、50 Hz 限制和超时行为。
- 收集 O6 topic、消息定义、控制/反馈维度、限位和频率。
- 记录机器人主机 CPU、GPU、内存、网卡和 USB 控制器拓扑。
- 记录三台 RealSense serial number。
- 确认物理急停工作方式。
- 建立 `docs/interfaces.md`，将未知字段列为阻塞项。

### 交付物

- `docs/interfaces.md`
- `configs/robot.yaml` 初稿
- `configs/o6.yaml` 初稿
- `configs/cameras.yaml` 初稿
- URDF 与 mesh 可被版本控制或通过明确脚本获取

### 验收门槛

- 能明确列出双臂固定 14 维关节顺序。
- 能明确指出左右末端 frame。
- 能用命令行查看手臂和 O6 的状态 topic。
- 能解释 SDK 收到命令中断后机器人会做什么。

---

## 阶段 1：建立干净的 C++/ROS 2 工程骨架

### 任务

- 创建新的 `ros2_ws/src` 包结构，不复制旧 TeleVision 运行逻辑。
- 建立 `qiling_interfaces`、`qiling_description`、`qiling_kinematics`、`qiling_teleop_controller` 和 `qiling_bringup` 空包。
- 设置 C++17、统一 warning、clang-format 和基础静态检查。
- 配置 rosdep、colcon build 和单元测试入口。
- 将 XRoboToolkit、Pinocchio、ProxQP、Eigen 的版本记录在 `third_party/README.md`。
- 建立 CI 或本机一键构建脚本，但不把 Conda 混入 ROS 构建。

### 验收门槛

- 全新 shell 中 source ROS 2 后可完整 `colcon build`。
- `colcon test` 可执行并产生测试报告。
- 启动空 bringup 不产生重复节点或 TF 发布者。

---

## 阶段 2：机器人模型和只读状态链路

### 任务

- 将 URDF 放入 `qiling_description`。
- 用 Pinocchio 和 `robot_state_publisher` 分别加载同一 URDF。
- 验证 joint 名称、顺序、零位、方向、上下限和单位。
- 实现 `qiling_robot_adapter` 的只读状态映射。
- 实现 O6 只读状态映射。
- 用 RViz 显示真实机器人关节状态。
- 建立关节状态录制样例供后续离线测试。

### 验收门槛

- 真实关节运动方向与 RViz 完全一致。
- Pinocchio FK 与 TF/RViz 中末端位姿在容差内一致。
- 缺失、重复或未知 joint name 会明确报错并阻止使能。
- 连续运行 30 分钟无状态断流或顺序漂移。

---

## 阶段 3：三相机稳定采集

### 任务

- 分别启动 D435、左右 D405，固定 serial 和 namespace。
- 只开启需要的 RGB stream。
- 再同时启动三台相机，检查 USB 带宽、CPU 和丢帧。
- 明确各相机时间戳 domain。
- 生成 10 分钟三相机录制样例。
- 标定并记录腕部相机到对应末端 frame 的外参；头部相机标定到头/基座相关 frame。

### 验收门槛

- 三路图像 topic 身份始终正确。
- 同时运行 30 分钟无设备重连。
- 每路实际 FPS 达到配置值，丢帧率处于可接受范围。
- 图像时间戳单调递增。

---

## 阶段 4：Quest 与 XRoboToolkit 最小链路

### 任务

- 构建并安装 XRoboToolkit Quest Unity Client。
- 在机器人主机运行 XRoboToolkit PC Service。
- 运行官方 C++ 示例或最小 callback 程序，输出 head/left/right pose 与 tracking validity。
- 实现 `qiling_xr_bridge`，将网络 callback 转成结构化 ROS 2 `XRFrame`。
- 统计 Quest 更新率、网络延迟、重复帧、乱序和断连行为。
- 实现头部 D435 RGB 到 Quest 的最小视频显示。
- 验证 Quest 应用暂停、摘下头显、tracking 丢失和网络断开场景。

### 验收门槛

- 连续 30 分钟收到稳定 XR 数据。
- C++ callback 不发生内存增长或线程阻塞。
- ROS 2 中只保留最新 XR frame，不形成积压。
- tracking validity 能正确反映控制器/手部丢失。
- Quest 中可持续看到 D435 画面，视频异常不影响 XR 控制数据。

---

## 阶段 5：坐标变换、clutch 与缩放

### 任务

- 写出 XRoboToolkit/OpenXR 的坐标轴定义并做实测确认。
- 实现 XR 到 `robot_base` 的固定旋转和尺度映射。
- 实现左右臂相对锚点、clutch、重新定位和 tracking origin 重置处理。
- 用 RViz marker 显示左右末端目标，不连接机器人控制。
- 对平移 X/Y/Z 和绕各轴旋转逐项做方向测试。
- 实现目标速度限制，避免手部 tracking 跳变直接形成大 pose step。

### 验收门槛

- 操作者向前/后/左/右/上/下运动时，RViz 目标方向全部正确。
- 松开并重新 clutch 后目标无跳变。
- Quest 重置边界/原点后控制器拒绝继续输出，直到重新锚定。
- 四元数连续，无 `q` 与 `-q` 导致的虚假跳变。

---

## 阶段 6：Pinocchio FK、Jacobian 和单臂差分 IK

### 任务

- 实现独立于 ROS 的 `qiling_kinematics` C++ 库。
- 加载 URDF，缓存 model/data/frame IDs 和关节映射。
- 对 FK、末端 frame 和 Jacobian 做单元测试。
- 用有限差分验证 Jacobian。
- 实现 SE(3) pose error 和 twist 限幅。
- 使用 ProxQP 实现左臂 7DoF 单臂 QP。
- 增加关节限位、速度限位、阻尼、平滑和 nominal posture。
- 对固定输入、轨迹输入、奇异点附近输入做离线 benchmark。

### 验收门槛

- FK 与参考结果一致。
- Jacobian 有限差分误差处于设定容差内。
- 随机可达目标求解稳定且不越限。
- 不可达目标通过限速/松弛平滑逼近，不产生 NaN 或大跳变。
- 单臂 IK P99 满足控制预算。

---

## 阶段 7：双臂联合 QP 与拟人化次级任务

### 任务

- 将决策变量扩展到双臂 14DoF。
- 同时加入左右末端任务。
- 使用每条手臂独立 nominal posture 和肘部偏好。
- 加入关节限位 margin、速度和加速度约束。
- 加入 QP slack 并定义不可行判定。
- 评估独立双臂求解与联合求解结果；正式接口保持联合求解能力。
- 后续按需要增加双臂/躯干碰撞距离线性化约束。
- 用记录的 XR 轨迹离线回放，统计耗时、残差和连续性。

### 验收门槛

- 两臂同时移动时均能达到可达目标且无明显相互干扰。
- 单臂静止时，另一臂运动不会使静止臂漂移。
- 肘部动作连续，不频繁翻转冗余构型。
- QP P99 和完整计算预算满足第 8.4 节目标。
- 连续失败能被稳定识别，不会输出危险目标。

---

## 阶段 8：安全控制器与虚拟机器人闭环

### 任务

- 实现完整安全状态机。
- 建立 mock robot adapter，以 50 Hz 接受 joint target 并返回模拟状态。
- 将 XR、相对目标、双臂 IK、限幅和 mock 状态闭环连接。
- 实现所有 watchdog、使能、禁用、fault 和 reset 行为。
- 记录并回放断网、状态过期、tracking 丢失、QP 失败和异常值测试。
- 确认控制器只在自己的 50 Hz tick 发布，不跟随 XR callback 直接发布。

### 验收门槛

- 10,000 次自动 fault injection 均进入预期状态。
- 所有断连都不会继续发送累积目标。
- 状态恢复后不会自动重新进入 `TELEOP`。
- mock 闭环连续运行 1 小时无 deadline 持续丢失。

---

## 阶段 9：真实手臂低速接入

### 任务

- 在 `qiling_robot_adapter` 中对接现有 ROS 2 SDK。
- 先只发送“保持当前位置”目标。
- 验证发布频率严格不超过 50 Hz。
- 先左臂、再右臂、最后双臂。
- 初始限制为小工作空间、低线速度、低角速度和小关节步长。
- 验证 SDK 超时、节点退出、ROS 2 断连和急停行为。
- 对比实际关节跟随与高层 `q_target`，但不把 MIT 命令加入训练 schema。

### 验收门槛

- 使能时无关节跳变。
- 单臂各方向动作与 Quest 一致。
- 双臂运行时不超过 SDK 50 Hz 上限。
- tracking 丢失、节点崩溃和网络断开均触发预定安全行为。
- 物理急停在全部软件状态下有效。

---

## 阶段 10：O6 遥操作接入

### 任务

- 实现 Quest 手骨架到 O6 高层 action 的映射。
- 完成左右手独立标定。
- 先在可视化或假 O6 adapter 中验证。
- 再在真实 O6 上以低速、限幅方式测试。
- 将 O6 和双臂 action 放入同一个 50 Hz 逻辑快照。
- 定义手部 tracking 丢失后的安全手型。

### 验收门槛

- 关键手型方向和幅度正确。
- O6 命令无高频振荡、突跳或越限。
- 左右手可以与双臂同时稳定运行。
- 录制 action 与 O6 adapter 接收的高层目标一致。

---

## 阶段 11：原始 Episode Recorder

### 任务

- 实现 start/stop/discard 和 task 输入。
- 使用严格 topic 白名单，不录制全部 ROS 图。
- 每个 episode 写入独立目录或 bag。
- 写入配置、标定和代码版本标识。
- 结束 episode 时生成摘要：时长、消息数、FPS、时间戳范围和丢帧。
- discard 使用可恢复或明确范围的删除方式，不影响其他 episode。
- 图像写盘压力测试与控制 deadline 同时进行。

### 验收门槛

- 录制开始/结束不改变机器人使能状态。
- 原始 episode 中仅包含白名单字段和必要元数据。
- 三路图像、state、action、O6 state 都有单调时间戳。
- 30 分钟录制不会影响 50 Hz 控制指标。
- 不完整 episode 能被检测并拒绝转换。

---

## 阶段 12：LeRobot v3 离线转换器

### 任务

- 在 Conda Python 3.10 中锁定 LeRobot v3 依赖。
- 读取原始 episode 和 metadata。
- 建立统一目标时间轴并对齐三路图像、state 和 action。
- 按固定关节/O6 顺序构建 `observation.state`。
- 将左右 `xyz + quaternion` 和 O6 target 构建为 `action`。
- 创建 task、episode、frame、timestamp 和数据集 metadata。
- 输出时间对齐报告、缺帧报告和 feature schema。
- 实现数据集加载、随机帧可视化和完整 episode 回放。
- 用官方 LeRobot API 重新打开输出数据集进行验证。

### 验收门槛

- 输出能够被目标 LeRobot 版本正常加载。
- feature 维度、dtype、图像尺寸和 FPS 与 metadata 一致。
- 任意抽取一帧都能追溯到原始 episode 时间戳。
- 可视化中三路图像、实际状态和 action 时间一致。
- 最终数据集中不存在 MIT 命令或其他明确排除字段。
- 同一输入重复转换得到相同帧数和相同数值结果。

---

## 阶段 13：整机性能、鲁棒性和操作验收

### 任务

- 同时运行 Quest、三相机、双臂、O6 和 recorder。
- 测量 XR packet age、IK 时延、控制 tick、SDK 发布频率、图像 FPS 和写盘吞吐。
- 做至少 30 分钟连续遥操作与录制。
- 做故障注入：Quest 断网、头显休眠、单相机断开、ROS 节点退出、状态过期、QP 不可行、磁盘空间不足。
- 测量视频 motion-to-photon 和操作者主观可用性。
- 检查 CPU 核心占用、内存增长、温度和网络抖动。
- 根据结果决定是否设置线程优先级、CPU affinity 或实时内核；这些优化必须基于测量，而不是默认开启。
- 编写 `docs/operations.md`：开机、标定、使能、录制、停止、急停和故障恢复。

### 最终验收

- 双臂和 O6 可由 Quest 连续稳定遥操作。
- SDK 外部命令频率不超过 50 Hz。
- 失联、越限和 IK 失败不会持续输出危险目标。
- 三路 RGB、实际状态和 EE/O6 action 可形成完整 episode。
- episode 可稳定转换并加载为 LeRobot Dataset v3。
- 运行 30 分钟不出现控制 deadline 持续丢失、明显内存泄漏或相机身份交换。
- 操作人员能按文档从冷启动完成一次录制和转换。

## 15. 测试矩阵

| 层级 | 测试内容 | 是否需要硬件 |
|---|---|---|
| Unit | 坐标变换、四元数、关节映射、限位 | 否 |
| Unit | Pinocchio FK/Jacobian、QP 约束 | 否 |
| Unit | 时间对齐、state/action 拼接 | 否 |
| Integration | 记录的 XR 输入 -> RViz target | 否 |
| Integration | XR -> IK -> mock robot | 否 |
| Integration | rosbag2 -> LeRobot v3 | 否 |
| HIL | Quest -> PC Service -> ROS 2 | Quest |
| HIL | 三台 RealSense 同时运行 | 相机 |
| HIL | 单臂低速跟随 | 单臂 |
| HIL | 双臂低速跟随 | 双臂 |
| HIL | O6 手型映射 | O6 |
| System | 双臂 + O6 + 三相机 + recorder | 全部 |
| Safety | 断网、过期、求解失败、急停 | 全部 |

## 16. 关键指标与观测方式

| 指标 | 采样位置 | 输出 |
|---|---|---|
| XR 更新频率 | `qiling_xr_bridge` | Hz、重复/乱序帧数 |
| XR 样本年龄 | 控制 tick 读取时 | mean/P95/P99/max |
| IK 耗时 | ProxQP solve 前后 | mean/P95/P99/max |
| 控制 tick 耗时 | 50 Hz 循环入口/出口 | mean/P95/P99/max |
| SDK 发布频率 | robot adapter | Hz、deadline miss |
| 相机 FPS | 各 image topic | Hz、drop count |
| 数据同步误差 | 离线 converter | 每源时间差分布 |
| 视频延迟 | 相机时间到 Quest 显示 | P50/P95 |
| 数据完整性 | 每 episode 结束 | 消息数、时长、缺失源 |

高频统计放入内存环形缓冲，按周期输出汇总，不能在每个控制 tick 打印日志。

## 17. 风险与提前决策

### 风险 1：Quest 型号不在官方验证范围

- Quest 3：按官方客户端路线。
- 其他 Quest：阶段 4 先做兼容性 spike，未通过前不进入机器人控制。

### 风险 2：D435 不是彩色立体相机

- 第一版使用单目面板或双眼复制。
- 真实双目彩色体验作为后续硬件升级，不阻塞数据采集。

### 风险 3：机器人 URDF 与 SDK joint 顺序不一致

- 所有映射按 joint name 显式配置。
- 任何缺失/重复 name 都阻止使能。

### 风险 4：QP 加入碰撞后超时或不可行

- 先完成无复杂碰撞的双臂版本。
- 碰撞约束分阶段加入，使用 slack、固定最大约束规模和 warm start。
- 任何优化都以本体主机 P99 数据为依据。

### 风险 5：ROS 2、XR、相机时间源不一致

- 原始数据保留 source stamp 和 receive stamp。
- 转换器检测回退和漂移，不静默修正异常 episode。

### 风险 6：图像录制影响控制实时性

- 录制器独立进程。
- 控制节点不做编码和磁盘 I/O。
- 必要时设置 CPU affinity、降低图像尺寸或优化存储，但不得降低控制安全检查。

### 风险 7：Conda 污染 ROS 2 C++ 运行环境

- ROS 2 runtime 与 LeRobot converter 使用独立启动入口。
- 控制链路从干净系统 shell 启动。

## 18. 每次实施迭代的固定流程

每个阶段按以下顺序执行：

1. 在文档中写清当前输入、输出、坐标系和失败行为。
2. 先写离线单元测试或 mock。
3. 实现最小功能。
4. 在无机器人运动条件下验证。
5. 收集频率、延迟和错误统计。
6. 通过阶段验收后提交代码和配置。
7. 更新 `docs/interfaces.md` 和对应操作文档。
8. 再进入下一阶段。

每次涉及真实机器人运动时：

- 先只读；
- 再保持当前位置；
- 再单关节/单方向小范围；
- 再单臂；
- 最后双臂和 O6；
- 始终保留物理急停和观察人员。

## 19. 开始实现前仍需提供的资料

这些资料不阻止建立工程骨架，但会阻止对应硬件阶段验收：

- Quest 具体型号。
- 完整双臂 URDF、mesh 和 joint limit。
- 左右 7 个关节名称及顺序。
- `robot_base`、左右 EE frame 名称。
- 手臂 SDK 的 ROS 2 message/topic/QoS 示例。
- SDK 对 MIT 字段的必填要求、默认值和 watchdog 行为。
- O6 控制和反馈消息定义、自由度、范围、单位和频率。
- 三台 RealSense serial、目标分辨率/FPS。
- 机器人主机硬件规格和 USB 拓扑。

## 20. 推荐的首个实现切片

第一次编码只完成以下闭环，不连接机器人执行：

```text
Quest
  -> XRoboToolkit PC Service C++ callback
  -> qiling_xr_bridge
  -> 坐标转换和 clutch
  -> Pinocchio + ProxQP 单臂 IK
  -> RViz / mock robot
```

该切片通过后，再增加右臂、真实 SDK、O6、相机录制和 LeRobot 转换。这样每一步都有明确输入、输出和验收依据。

## 21. 参考项目

- [XRoboToolkit](https://xr-robotics.github.io/)
- [XRoboToolkit C++ Teleop Sample](https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Cpp)
- [XRoboToolkit Quest Unity Client](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client-Quest)
- [XRoboToolkit PC Service](https://github.com/XR-Robotics/XRoboToolkit-PC-Service)
- [Unitree xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate)
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio)
- [ProxQP / proxsuite](https://github.com/Simple-Robotics/proxsuite)
- [LeRobot Dataset v3](https://huggingface.co/docs/lerobot/lerobot-dataset-v3)

