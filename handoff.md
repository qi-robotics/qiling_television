# Jetson NX 双臂遥操部署交接

## 目标与当前状态

- 机器人板子与 Jetson NX 通过网线连接；机器人板子上的机器人 SDK 和 `qi_topic_converter` 已自启，ROS Domain ID 为 `21`。
- NX 已安装 ROS 2 Humble，Domain ID 也设为 `21`。本次把 XRoboToolkit PC Service 和双臂遥操程序部署到 NX；不部署仿真、数据采集或记录功能，也不在 NX 上运行腿部控制器。
- **此仓库当前不能直接在 NX 上编译和启动。**下文列出必须处理的 ARM64 库路径、x86 启动脚本和 DDS 网卡配置。本文件是部署交接说明，目前尚未验证 NX 的系统版本、网卡、安装包内容或整条遥操链路。

## 两台机器分别运行什么

| 位置 | 运行内容 | 作用 |
| --- | --- | --- |
| 机器人板子 | 已自启的机器人 SDK | 发布 `/lowstate`，接收 `/lowcmd_arms` 和 `/handscmd`；原有腿部控制维持在板子侧。 |
| 机器人板子 | 已自启的 `qi_topic_converter` | 将 `/lowstate` 转为 `/human_lower_state`。继续复用，不在 NX 上启动第二份默认状态转换器。 |
| NX | XRoboToolkit PC Service | 接收头显数据，供本机 PXREA SDK 适配节点读取。头显应连接 NX 的可达 IP。 |
| NX | `xr_teleop_real.launch.py` | 运行 XR 输入适配、双臂 IK、手部控制和**专用命令桥**；把双臂命令发往 `/lowcmd_arms`。 |

主要链路：

```text
头显 → NX：PC Service → XR 适配 → 双臂 IK → 专用命令桥 → /lowcmd_arms → 机器人板子：SDK
机器人板子：SDK → /lowstate → 已自启 qi_topic_converter → /human_lower_state → NX：双臂 IK
NX：手部命令 → /handscmd → 机器人板子：SDK
```

[src/qiling_kinematics_real/launch/xr_teleop_real.launch.py](src/qiling_kinematics_real/launch/xr_teleop_real.launch.py) 已把专用桥配置为只转换命令，输出重映射到 `/lowcmd_arms`，且不再发布一份 `/human_lower_state`。该桥仍需随遥操 launch 运行；它不是板子上已自启的默认转换器。遥操内部消息虽为 26 电机格式，本次控制目标仍是双臂；SDK 侧负责和原有腿部控制配合。

## XRoboToolkit 安装包选择

NX 是 ARM64 机器时，优先使用官方 [`XRoboToolkit-PC-Service-headless_1.0.0.0_arm64.deb`](https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.0.0/XRoboToolkit-PC-Service-headless_1.0.0.0_arm64.deb)。本项目只需要 PC Service 和 SDK，不需要 PC 端 2D/3D 图形演示。需要图形演示时再考虑完整的 [`XRoboToolkit-PC-Service_1.0.0.0_arm64.deb`](https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.0.0/XRoboToolkit-PC-Service_1.0.0.0_arm64.deb)。Windows 包以及当前板子上的 AMD64 包都不适用于 NX。

官方项目说明支持 Linux ARM64，并将 `runService.sh` 列为单独启动服务的入口：[项目说明](https://github.com/XR-Robotics/XRoboToolkit-PC-Service/blob/main/README-CN.md)。官方仓库有一条关于 headless 包运行 **2D 图形 Demo** 时缺少 QML 库的[问题报告](https://github.com/XR-Robotics/XRoboToolkit-PC-Service/issues/13)；这不能代替对服务和 SDK 的实测。包的具体文件布局尚未在 NX 上核实，以下路径必须以安装结果为准。

在 NX 上先检查环境和下载包：

```bash
uname -m                         # 预期 aarch64
. /etc/os-release; echo "$PRETTY_NAME"
ip -br -4 addr                   # 记下连接机器人板子的网卡名和 IP
dpkg-deb -f ./XRoboToolkit-PC-Service-headless_1.0.0.0_arm64.deb Architecture
sha256sum ./XRoboToolkit-PC-Service-headless_1.0.0.0_arm64.deb
```

用户提供的 headless 包预期 SHA256 为 `532c605dfa1a02b05b7c285b856c91771c78623cded30ef5b16ea371de49ed5f`。核对下载来源与哈希后安装：

```bash
sudo apt install ./XRoboToolkit-PC-Service-headless_1.0.0.0_arm64.deb
test -x /opt/apps/roboticsservice/RoboticsServiceProcess
test -x /opt/apps/roboticsservice/runService.sh
find /opt/apps/roboticsservice/SDK -name 'libPXREARobotSDK.so' -print
ldd /opt/apps/roboticsservice/RoboticsServiceProcess | grep 'not found' || true
```

确认 SDK 库实际路径，例如 `SDK/arm64`；如果包内没有所需的 ARM64 SDK 库，先核对完整 ARM64 包的内容，再决定换包。`ldd` 出现 `not found` 时，应先解决服务的运行依赖。

## 网络和 DDS：Domain 21 之外还要检查的事项

1. 在两台机器分别用 `ip -br -4 addr` 确认实际联网接口与 IP，验证双向连通。NX 与板子可以直连，但两端 IP、路由和防火墙必须允许 ROS 2 DDS 发现与数据传输。
2. 板子现用 `/home/coral/qi_deploy_igh_release/config/dds/cyclonedds_enp8s0.xml` 只绑定 `enp8s0`。如果连 NX 的网线使用板子的**另一张**网卡，现有配置可能使 NX 发现不到板子节点；需要在板子侧调整 DDS 所用接口或网络发现配置。不要按文件名猜测物理网口。
3. NX 侧配置自己的 CycloneDDS XML，绑定连接板子的实际网卡；不要复制板子的 `enp8s0` 名称。遥操进程需要 `ROS_DOMAIN_ID=21`、`ROS_LOCALHOST_ONLY=0`、`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`，并让 `CYCLONEDDS_URI` 指向 NX 自己的 XML。CycloneDDS 的[网卡配置说明](https://cyclonedds.io/docs/cyclonedds/latest/config/network_interfaces.html)可用于核对 XML。
4. 头显还必须能访问 NX 的 PC Service IP。若头显走 Wi-Fi、ROS 走有线，分别检查两条网络路径，并在头显中选择或输入 NX 可达的 IP。

同为 Domain 21 只是必要条件，不能替代网卡和跨机器发现检查。

## 当前仓库需要修改的文件

以下改动应在核实 ARM64 安装包路径及 NX 网卡后实施，然后在 NX 本机编译：

| 文件 | 当前情况 | NX 适配要求 |
| --- | --- | --- |
| [`src/scripts/build_xrtele.sh`](src/scripts/build_xrtele.sh) | 安装前检查写死 `SDK/x64/libPXREARobotSDK.so`。 | 按实际安装路径识别 ARM64 SDK。 |
| [`src/qiling_kinematics_real/CMakeLists.txt`](src/qiling_kinematics_real/CMakeLists.txt) | 查找 SDK 和运行时 RPATH 都写死 `SDK/x64`。 | 使用实际 ARM64 库路径，使 PXREA 适配节点能链接和加载该库。 |
| [`src/scripts/start_xrobotoolkit_real.sh`](src/scripts/start_xrobotoolkit_real.sh) | 强制要求 `RobotLinuxDemo.x86_64`，并把 `SDK/x64` 加入库路径。 | 改为适合 ARM64 headless 包的服务启动流程，不强制启动 x86 Unity Demo。相应核对 [`stop_xrobotoolkit_real.sh`](src/scripts/stop_xrobotoolkit_real.sh)。 |
| [`src/scripts/start_real_teleop.sh`](src/scripts/start_real_teleop.sh) | `DDS_CONFIG` 写死板子上的绝对路径，并覆盖外部 `CYCLONEDDS_URI`。 | 改为使用 NX 的 DDS 文件路径，最好允许环境变量覆盖；保留 Domain 21 和状态等待检查。 |
| [`src/scripts/install_xrtele_dependencies.sh`](src/scripts/install_xrtele_dependencies.sh) | 按 Ubuntu 22.04 的 ROS Humble apt 包安装依赖。 | 先核对 NX 的 Ubuntu 版本及这些包是否可用，再执行。Humble 已安装不代表所有编译依赖都已安装。 |

不要把板子现有的 `build/`、`install/` 目录或 AMD64 `.deb` 当成 NX 的可执行产物；在 NX 上从源码构建。此仓库的仿真包已删除，不需要为 NX 安装 MuJoCo GUI 依赖。

## 分步部署与验收

1. **确认平台和包**：完成上述 `uname -m`、系统版本、哈希及 `.deb` 架构检查；安装 headless 包，确认服务可执行文件、ARM64 SDK 库和依赖存在。
2. **确认两条网络**：验证 NX ↔ 板子的 IP 连通；确认板子现有 CycloneDDS 接口覆盖 NX 的连接路径，并配置 NX 的 DDS 网卡；确认头显能访问 NX 的 PC Service IP。
3. **适配仓库并在 NX 编译**：完成上表文件修改；安装 NX 缺失的编译依赖；在 NX 运行 `./src/scripts/build_xrtele.sh`。不要直接沿用板子上的 x86 编译结果。
4. **先验证 ROS 状态**：在 NX 的 Domain 21 和正确 DDS 环境中 `source install/setup.bash`，执行 `ros2 topic echo /human_lower_state --once`。必须拿到板子发来的完整状态；没有收到时先排查 DDS、板子上的转换器及接口类型，不启动运动遥操。
5. **验证头显链路**：在 NX 启动 PC Service，头显连接 NX；确认服务进程和 PXREA 适配节点能收到头显姿态数据。若使用 headless 包，先验证 `runService.sh` 能否单独正常启动。
6. **验证命令接收再遥操**：检查 `/lowcmd_arms` 的订阅者位于板子 SDK 侧，`/handscmd` 同样有预期订阅者；再启动 NX 上的 `start_real_teleop.sh`。该脚本收到 `/human_lower_state` 后会自动开始“当前姿态 → 过渡点 → home → 遥操”，应在双臂周围无障碍物、急停可用时进行。

本交接不要求修改机器人腿部控制流程；也不要额外启动一份默认的 `qi_topic_converter`。如果某一步失败，记录 NX 系统版本、网卡/IP、ARM64 SDK 实际路径、DDS XML、相关进程日志和上述话题的发布/订阅情况，再定位该步。
