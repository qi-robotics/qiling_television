# Qiling 双臂 XR 真机遥操（Jetson NX）

当前部署目标为 Jetson NX（Ubuntu 22.04、ARM64、ROS 2 Humble）与机器人运控板网线直连。Quest 3 连接 NX 上的 XRoboToolkit PC Service；板子上原有的机器人 SDK 和 qi_topic_converter 继续自启。本项目在 NX 上运行双臂差分 IK、XR 输入适配、手部适配和专用命令桥。现场已完成双臂及手部遥操测试。

部署背景、网卡核对结果和 SDK 包校验记录见 [handoff.md](handoff.md)。节点接口细节见 [真机包说明](src/qiling_kinematics_real/README.md)。

如需 NX 开机满 60 秒后自动启动 PC Service 与真机遥操，参见 [开机自启说明](src/scripts/真机遥操开机自启.md)。启用后每次开机只要板子状态可达，双臂会自动回 home。

## 当前链路

~~~text
Quest 3 → NX：PC Service → XR 输入适配 → Grip 离合器 → 双臂 IK
                               └→ Trigger → O6 手部适配 → /handscmd
双臂 IK → /teleop/arms_mit_command → 专用命令桥 → /lowcmd_arms → 板子 SDK
板子 SDK → /lowstate → 板子自启的 qi_topic_converter → /human_lower_state → NX
~~~

状态与命令均在 ROS Domain 21。/human_lower_state 的 joint_states.position 为 26 个电机：0–11 是腿，12–18 是左臂，19–25 是右臂。NX 的专用命令桥只将遥操命令转换到 /lowcmd_arms；不要再启动一份默认状态转换器，也不要把遥操命令接到策略使用的 /lowcmd。

## 一次性准备

1. 在 NX 上安装 ROS 2 Humble、工程依赖和 XRoboToolkit ARM64 headless PC Service。已安装的服务入口为 /opt/apps/roboticsservice/runService.sh，SDK 头文件和库分别位于 /opt/apps/roboticsservice/SDK/include 与 /opt/apps/roboticsservice/SDK/arm64。换机或重装时先核对实际路径。依赖脚本：

   ~~~bash
   ./src/scripts/install_xrtele_dependencies.sh
   ~~~

2. 准备 NX 本地 DDS 文件。当前 NX 与板子通信的有线网卡为 eth2，项目使用 config/cyclonedds_nx.xml；该文件是机器本地配置，不提交到 Git。当前机器已有文件，不要覆盖。换机时若文件不存在，再从模板复制并按实际网卡名修改：

   ~~~bash
   test -f config/cyclonedds_nx.xml || cp config/cyclonedds_nx.xml.example config/cyclonedds_nx.xml
   ip -br addr
   ~~~

   Domain 21 和 NX 网卡都必须与现场网络一致。模板中的 REPLACE_WITH_NX_ETHERNET_INTERFACE 不能原样保留。Quest 连接的 NX 地址应是头显实际可达的地址，可能与连接板子的 eth2 地址不同。

3. 在 NX 本机编译。构建脚本会自行加载 ROS 环境，查找 ARM64 SDK，编译 qiling_kinematics_real 及所需接口包：

   ~~~bash
   ./src/scripts/build_xrtele.sh
   ~~~

   不复用板子或 x86 的 build/install 产物。随仓库提供的 Quest APK 位于 third_party/xrobotoolkit/quest/；安装和校验说明见 [APK 说明](third_party/xrobotoolkit/quest/README.md)。

## 每次启动：两个终端直接运行脚本

这两个脚本都可以从项目目录直接执行。启动它们之前，**无需手动 source ROS setup，也无需手动 export ROS_DOMAIN_ID、ROS_LOCALHOST_ONLY、RMW_IMPLEMENTATION 或 CYCLONEDDS_URI**。

终端 1：启动 XRoboToolkit PC Service。脚本启动后台服务后会返回提示符。

~~~bash
cd ~/liujun/qiling_television
./src/scripts/start_xrobotoolkit_real.sh
~~~

在 Quest 3 打开 XRoboToolkit App，连接 NX 的可达 IP，确认显示 WORKING，并打开 Controller Tracking 和 Send。不要打开 Switch w/ A Button。

终端 2：确认双臂周围无障碍物、硬件急停可用，启动真机遥操：

~~~bash
cd ~/liujun/qiling_television
./src/scripts/start_real_teleop.sh
~~~

真机脚本会等待 /human_lower_state，随后启动完整 launch。机器人自动执行“当前实测姿态 → 安全过渡点 → home”，完成后才开放遥操。若 30 秒内没有收到状态，脚本会退出且不启动 IK。仅需延长等待时可直接运行：

~~~bash
QILING_STATE_WAIT_SEC=60 ./src/scripts/start_real_teleop.sh
~~~

home 完成后，左右 Grip 都先完整松开一次；再按住对应 Grip 控制该侧手臂，松开则保持。左右 Trigger 分别控制左右 O6 灵巧手开合。左臂在反馈接近 URDF 限位时，只有保持的命令目标指向安全区且未越过硬限位，才允许重新握紧 Grip 后接管；其余情况进入限位恢复。

遥操终端按 Ctrl+C 停止本次 launch；PC Service 仍在后台运行。需要停止服务时执行：

~~~bash
./src/scripts/stop_xrobotoolkit_real.sh
~~~

板子上自启的 SDK、策略和 qi_topic_converter 由板子自己的服务管理。

## 环境变量究竟在哪里设置

| 项目 | start_xrobotoolkit_real.sh | start_real_teleop.sh |
| --- | --- | --- |
| ROS 2 与工作区 setup.bash | 不需要 ROS 环境 | 脚本内部自行加载 /opt/ros/humble/setup.bash 和 install/setup.bash |
| ROS_DOMAIN_ID | 不使用 | 固定为 21 |
| ROS_LOCALHOST_ONLY | 不使用 | 固定为 0 |
| RMW_IMPLEMENTATION | 不使用 | 固定为 rmw_cyclonedds_cpp |
| CYCLONEDDS_URI | 不使用 | 默认指向项目的 config/cyclonedds_nx.xml |

真机脚本也支持用 QILING_DDS_CONFIG 指定另一份 DDS 文件。如果调用它的终端**已经设置** CYCLONEDDS_URI，脚本会优先沿用该值；因此出现意外网卡或收不到状态时，先看启动时打印的“DDS 配置”路径。两个脚本在子进程里设置环境，不会改变启动它们的终端；在该终端随后手动运行 ros2 命令时，仍需准备 ROS 和 DDS 环境。

## 手动只读检查

以下命令用于另开终端手动运行 ros2 检查，**不是运行上述两个脚本的前置步骤**。在项目目录执行：

~~~bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=21
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$(pwd)/config/cyclonedds_nx.xml"
ros2 topic info -v /human_lower_state
ros2 topic info -v /lowcmd_arms
ros2 topic info -v /handscmd
~~~

/lowcmd_arms 应有板子侧 qi/msg/LowCmd 订阅者，/handscmd 应有板子侧 qi/msg/HandsCmd 订阅者。只读观察 XR 输入可用 ros2 topic hz /xr/left_controller_pose 和 /xr/right_controller_pose。PC Service 日志由启动脚本打印，默认位于 /tmp/qiling_xrobotoolkit/robotics_service.log；若设置了 XDG_RUNTIME_DIR，则位于该目录下的 qiling_xrobotoolkit/。

常见检查顺序：先看 PC Service 与 Quest 的 WORKING 状态，再看真机脚本打印的 DDS 文件路径、eth2 是否仍连接板子、/human_lower_state 是否可达。如果 home 已完成但某侧手臂仍不跟随，先完整松开再按下该侧 Grip，并检查真机节点日志中的状态和限位提示。

## 安全边界

启动真机脚本会发布 /lowcmd_arms 和 /handscmd，且会自动移动双臂到过渡点和 home。每次启动前确认工作空间、急停和板子侧状态；不要同时运行第二份真机遥操 launch 或其他相同命令话题的发布者。当前工程的 MuJoCo 仿真包和 x86 图形 Demo 不属于这套 NX 部署流程。
