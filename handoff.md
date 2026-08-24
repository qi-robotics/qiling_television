# Vision Pro 双臂遥操作项目 Handoff

## 项目目标

从零搭建一套参考 [Unitree xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate) 的遥操作与数据采集系统：使用 Apple Vision Pro 直接控制自研双臂机器人末端，并录制 LeRobot Dataset v3 数据。

已确认的硬件与软件条件：

- Ubuntu 22.04，ROS 2 Humble
- 双臂各 7DoF
- 双侧 O6 灵巧手
- 机械臂 SDK 采用 MIT 模式：`q / dq / kp / kd / tau`
- 头部 Intel RealSense D435
- 左右腕部各一台 RealSense D405
- Vision Pro、ROS 2、IK、机器人控制和数据录制都运行在机器人本体主机
- 不考虑 Isaac Gym 仿真
- 数据格式使用 LeRobot Dataset v3

## 总体设计

参考 `xr_teleoperate` 的模块边界，将图像服务、XR 输入、机器人 IK、硬件控制和数据录制解耦：

```text
D435 / D405 相机 ──> 图像服务 ───────────────> Vision Pro
       │                                        │
       │                                        │ 手腕、手指、头部位姿
       │                                        v
       │                               XR 坐标变换与标定
       │                                        │
       │                              双臂末端目标 + O6 目标
       │                                        v
       │                              PyRoKi 双臂冗余 IK
       │                                        │ 关节目标
       │                                        v
       │                              安全过滤与轨迹插值
       │                                        │
       │                                        v
       │                         ROS 2 高频 MIT 控制器 ──> SDK
       │                                        │
       └──────────────> LeRobot v3 Recorder <───┘
```

## 推荐目录

```text
qiling_television/
├── assets/
│   └── robot/                       # URDF、meshes、碰撞模型
├── configs/
│   ├── cameras.yaml
│   ├── teleop.yaml
│   ├── ik.yaml
│   ├── safety.yaml
│   └── lerobot.yaml
├── teleop/
│   ├── televuer/                    # Vision Pro/WebXR 输入
│   ├── teleimager/                  # 三相机采集与头部图像传输
│   ├── calibration/                 # XR、机器人、相机坐标标定
│   └── hand_retargeting/            # Vision Pro 手部到 O6 的映射
├── robot_ws/src/
│   ├── qiling_description/          # ROS 2 robot_description
│   ├── qiling_interfaces/           # 双臂目标和 MIT 命令消息
│   ├── qiling_xr_bridge/            # XR 数据发布到 ROS 2
│   ├── qiling_ik/                   # PyRoKi/Pinocchio IK 后端
│   ├── qiling_control/              # 插值、安全控制、MIT 控制
│   ├── qiling_driver/               # 现有 ROS 2 SDK 驱动适配
│   ├── qiling_lerobot_recorder/     # LeRobot v3 录制
│   └── qiling_bringup/              # launch 与参数
├── scripts/
│   ├── start_teleop.sh
│   ├── start_recording.sh
│   └── check_latency.sh
└── tests/
    ├── test_transforms.py
    ├── test_ik_latency.py
    └── test_dataset_sync.py
```

## Python 与 ROS 2 环境

可以在机器人主机安装 Conda，并新建 Python 3.10 环境。`xr_teleoperate` 官方安装方案本身使用 Python 3.10；PyRoKi 的包声明也支持 Python 3.10 及以上。

建议边界如下：

- Conda Python 3.10：TeleVuer、图像客户端、PyRoKi、O6 retargeting、LeRobot。
- 系统 ROS 2 Humble：机器人驱动、C++ MIT 控制器、RealSense ROS 节点。
- 不在 Conda 中重新安装 ROS 2，只在启动时加载 `/opt/ros/humble` 和工作空间。
- 若 Conda 内 `rclpy` 出现 ABI/动态库冲突，将 `qiling_xr_bridge` 运行在系统 Python 中，使用共享内存或 ZeroMQ 接收 Conda 进程的数据。

第一阶段优先验证 Python 3.10 环境能否导入 `rclpy`、PyRoKi、Vuer、RealSense 和 LeRobot，并固定依赖版本。

## PyRoKi IK 方案

PyRoKi 适合 7DoF 手臂，因为末端 6D 位姿约束之外仍有一个冗余自由度，可以通过优化代价塑造更拟人的姿态。建议左右臂联合求解，并加入：

- 左右末端位置与姿态跟踪
- 关节限位
- 与上一帧关节解的连续性
- 关节速度与加速度平滑
- 肘部位置或肘部平面偏好
- 肩部自然姿态和默认站姿偏好
- 可操作度代价，减少奇异位形
- 左右臂、手臂与躯干的自碰撞代价

### 实时性判断

PyRoKi 可以尝试承担 30～60 Hz 的在线 IK，但不能直接承担高频 MIT 闭环。

其论文在 Franka Panda 的 IK-Beam 测试中，解析雅可比单批次约为 CPU 5.9 ms、GPU 3.6 ms，说明预编译后的纯 IK 有进入 60 Hz 预算的可能。不过这不是本机器人、双臂联合求解和完整碰撞约束下的保证。

主要风险：

- 第一次运行存在 JAX JIT 编译延迟。
- target、约束或障碍物数组形状变化会触发重新编译。
- 自动微分通常慢于解析雅可比。
- 复杂碰撞约束会增加延迟，PyRoKi 官方也说明碰撞密集场景尚未充分比较。
- Python/JAX 不提供硬实时调度保证。

使用要求：

1. 启动时完成 JIT 预热，预热成功前禁止使能机器人。
2. 固定输入 shape、目标数量和碰撞体数量，必要时预填充数组。
3. 使用上一帧关节状态作为 warm start。
4. 优先使用解析雅可比，并限制 LM 迭代次数。
5. IK 输出频率先定为 30 Hz，实测稳定后再提升到 60 Hz。
6. 设置求解超时、残差阈值和连续失败计数；失败时保持上一安全目标或进入阻尼模式。
7. 在机器人主机上测量平均、P95、P99 和最坏求解时间，验收目标建议为 P99 小于 10 ms，且运行中无 JIT 重编译。
8. 保留 Pinocchio 差分 IK 后端，作为 PyRoKi 性能不达标时的实时回退方案。

因此推荐的控制频率分层为：

```text
Vision Pro 输入             30～60 Hz
PyRoKi 双臂 IK              30～60 Hz
关节目标插值与安全过滤      200 Hz 或更高
MIT 控制器                  按 SDK 要求运行，建议使用 C++
LeRobot 数据录制            30 Hz
```

## ROS 2 接口建议

```text
/xr/head_pose
/xr/left_wrist_target
/xr/right_wrist_target
/xr/left_hand_landmarks
/xr/right_hand_landmarks

/ik/left_joint_target
/ik/right_joint_target
/o6/left_target
/o6/right_target

/robot/joint_states
/robot/mit_command
/robot/teleop_enable
/robot/emergency_stop

/camera/head/color/image_raw
/camera/left_wrist/color/image_raw
/camera/right_wrist/color/image_raw
```

底层 MIT 控制器负责把 IK 关节目标转换为连续的 `q、dq、kp、kd、tau`，并处理限位、速度限制、超时和急停。PyRoKi 节点只输出运动学目标，不直接发送电机力矩。

## LeRobot Dataset v3

建议以 30 Hz 对齐三路 RGB、机器人真实状态和实际下发动作：

```text
observation.images.head
observation.images.left_wrist
observation.images.right_wrist

observation.state
    双臂实际关节位置与速度
    双侧 O6 实际状态

observation.ee_pose
    左右末端实际位姿

action
    左右末端目标位姿，建议 xyz + quaternion
    双侧 O6 目标

action.joint_target               # 建议额外保留 IK 输出，便于诊断
timestamp
frame_index
episode_index
task
```

图像应从 RealSense 原始 ROS topic 录制，Vision Pro 中显示的压缩图像只用于操作者反馈。所有数据统一使用机器人主机 ROS time，并记录命令产生时间与实际采样时间。

## 实施顺序

1. 整理完整 URDF、joint/link 名称、限位、O6 接口和 ROS 2 SDK 驱动。
2. 验证三台 RealSense 可稳定同时运行，并确定 USB 带宽、分辨率和帧率。
3. 搭建 TeleVuer 和头部相机回传，验证 Vision Pro 位姿与图像延迟。
4. 完成 XR 坐标系到机器人 base 坐标系的标定。
5. 先离线实现 PyRoKi 双臂 IK 和拟人化代价，再进行主机实时基准测试。
6. 实现 ROS 2 安全控制器和 MIT 高频控制，先以小增益、低速度测试单臂，再测试双臂。
7. 接入 O6 手部 retargeting。
8. 实现 LeRobot v3 recorder、episode 控制和数据同步检查。
9. 完成失联、IK 失败、越界、急停和恢复流程测试。

## 仍需确认

- O6 每只手的自由度、关节名称、反馈字段与控制频率。
- ROS 2 驱动现有 topic/service/action 和 MIT 消息定义。
- 左右肩、肘、腕与末端 link 名称，以及机器人 base frame。
- MIT 控制周期及 SDK 推荐的 `kp/kd` 范围。
- 三台 RealSense 的目标分辨率、FPS，以及是否记录深度。
- 机器人主机的 CPU、GPU、CUDA 和可用 USB 控制器信息。

## 参考

- [Unitree xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate)
- [PyRoKi](https://github.com/chungmin99/pyroki)
- [PyRoKi 论文](https://arxiv.org/abs/2505.03728)
- [LeRobot Dataset v3](https://huggingface.co/docs/lerobot/lerobot-dataset-v3)
