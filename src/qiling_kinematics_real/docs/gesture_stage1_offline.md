# 动作第一阶段：离线轨迹检查

本阶段只生成和检查挥手、握手、抱拳的候选双臂轨迹。`check_gesture_trajectories.py` 不初始化 ROS，不连接机器人，也不发布任何话题。真机 IK 节点在启用动作后读取 `gesture_actions.yaml`，触发时仅做关节空间检查；网格碰撞检查由本离线工具完成。

## 关节映射

每侧七维为 `[肩俯仰, 肩横滚, 肩偏航, 肘, 腕 roll, 腕 pitch, 腕 yaw]`。左臂对应 26 电机状态的 `12..18`，右臂对应 `19..25`。动作原表的第 5 个数仍对应本模型第 5 关节和电机 16/23；不交换第 5、7 项。工具启动时会从 URDF 检查每个关节名并打印实际映射。

## 检查方法

在项目根目录运行：

```bash
python3 src/qiling_kinematics_real/scripts/check_gesture_trajectories.py
python3 src/qiling_kinematics_real/scripts/check_gesture_trajectories.py --action wave
```

默认起点来自 `config/differential_ik.yaml` 的左右 `q_rest`。如需检查另一组**已保存的** 26 电机位置，可传入逗号分隔的 26 个弧度值：

```bash
python3 src/qiling_kinematics_real/scripts/check_gesture_trajectories.py \
  --motor-positions "$MOTOR_POSITIONS_CSV"
```

工具按当前微分 IK 配置的关节速度、加速度上限给进场、摆动和回 home 段分别重新定时；全程检查 URDF 关节限位，并保留至少 `0.12 rad` 的目标余量。进场与回程为零端速的五次关节空间插值。摆动保留原动作各轴的振幅、相位和周期数，增加 `sin⁴(πu)` 包络，使摆动起止位置、速度及加速度平滑衔接。握手进场后有原动作内部的 1 秒握持等待；回程从摆动末帧直接开始，没有结束停留。每 `20 ms` 采样一次，用 Pinocchio 加 Coal 检查双臂之间、各臂非相邻连杆之间，以及双臂与 URDF 基座和固定腿部网格之间的碰撞。

输出 `PASS` 才代表该组输入通过以上**离散采样的模型检查**；`FAIL` 会指出首个越限或碰撞的动作、阶段、时间及关节/网格。缺少模型或碰撞依赖时工具会失败退出。当前模型没有根据 O6 命令更新手指形状；也不包含现场桌面、人员、头显或实时腿部位姿。更换起始构型、URDF 或周边布置后需重新验收，离线通过不能替代真机前的现场安全确认。

候选位姿及摆动参数在 `config/gesture_actions.yaml`。挥手肩俯仰中心为 `-2.70 rad`，抱拳双肘为 `-1.87 rad`；原进退场的肘部附加 `-0.3 sin(πu)` 不纳入首版候选轨迹。全局微分 IK 限速参数未修改。
