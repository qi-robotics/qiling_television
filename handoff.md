# Jetson NX 双臂遥操部署交接

## 目标与当前状态

- 机器人板子与 Jetson NX 通过网线连接；机器人板子上的机器人 SDK 和 `qi_topic_converter` 已自启，ROS Domain ID 为 `21`。
- NX 已安装 ROS 2 Humble，Domain ID 也设为 `21`。本次把 XRoboToolkit PC Service 和双臂遥操程序部署到 NX；不部署仿真、数据采集或记录功能，也不在 NX 上运行腿部控制器。
- 2026-10-20 已在 NX 核实：`aarch64`、Ubuntu 22.04.5、ROS Domain 21；机器人 ROS 话题在 `eth2`（`192.168.123.100/24`）上可发现。已安装 XRoboToolkit ARM64 headless 包及编译依赖，本机 5 个包均构建成功。
- 下载的 headless `.deb` 为 `arm64`，SHA256 与下文预期值一致。SDK 实际路径为 `/opt/apps/roboticsservice/SDK/arm64/libPXREARobotSDK.so`；服务程序和 SDK 库均为 aarch64。已读取 `/human_lower_state` 的一帧完整 26 电机有限位置值，持续频率约 1 kHz；初次只读验收时，`/lowcmd_arms` 和 `/handscmd` 均有一个裸 DDS 订阅者、没有发布者。运行遥操后 NX 会向这两个话题发布命令。
- XR 适配器已连接 PC Service，Quest 左右控制器位姿均约 90.9 Hz，`/xr/controller_joy` 可读。此后用户已完成真机测试，确认当前双臂和手部遥操正常。左臂松开 Grip 后再次握紧受近限位恢复阻塞的问题已在项目中修复并通过现场测试；配置项为 `left_guarded_near_limit_regrip`。

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

官方项目说明支持 Linux ARM64，并将 `runService.sh` 列为单独启动服务的入口：[项目说明](https://github.com/XR-Robotics/XRoboToolkit-PC-Service/blob/main/README-CN.md)。当前 NX 已核实 headless 服务和 SDK 的安装位置；换机或换包时仍须按安装结果核对。

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
LD_LIBRARY_PATH="/opt/apps/roboticsservice:/opt/apps/roboticsservice/lib:/opt/apps/roboticsservice/SDK/arm64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
  ldd /opt/apps/roboticsservice/RoboticsServiceProcess | grep 'not found' || true
```

本机实际 SDK 库位于 `SDK/arm64`。官方源码文档另列出 `SDK/linux_aarch64/64`，构建脚本也支持这些目录；如果其他安装包布局不同，用 `PXREA_SDK_ROOT`、`PXREA_SDK_INCLUDE_DIR`、`PXREA_SDK_LIB_DIR` 显式指定。直接运行 `ldd` 会因未加载厂商库路径误报缺少打包在服务目录中的库，检查时应按 `runService.sh` 的 `LD_LIBRARY_PATH` 设置环境。按此环境检查，本机服务程序未发现缺失库。

## 网络和 DDS：Domain 21 之外还要检查的事项

1. NX 当前 `eth2=192.168.123.100/24`、`eth3=192.168.1.102/24`、`wlan0=192.168.110.84/24`。只绑定 `eth2` 的 CycloneDDS 已发现 `/lowstate`、`/human_lower_state`、`/lowcmd_arms`、`/handscmd`；只绑定 `eth3` 时未发现这些话题。因此本次机器人 ROS 链路使用 **NX 的 `eth2`**。若现场地址或接线改变，应重新核对。
2. 原交接资料记录板子的 DDS 文件为 `/home/coral/qi_deploy_igh_release/config/dds/cyclonedds_enp8s0.xml`；本次没有读取板子当前实际加载的文件。NX 的 `eth2` 已发现相关话题，因此当前无需为发现问题改动板子 DDS。不要按板子 XML 文件名猜测 NX 的网口。
3. 已创建 NX 本地 `config/cyclonedds_nx.xml` 并绑定 `eth2`，该机器配置被 Git 忽略；可提交的模板是 `config/cyclonedds_nx.xml.example`。`start_real_teleop.sh` 默认读取本地文件，也支持 `QILING_DDS_CONFIG` 或现有 `CYCLONEDDS_URI` 覆盖。遥操进程固定使用 Domain 21、`ROS_LOCALHOST_ONLY=0` 和 CycloneDDS RMW。[网卡配置说明](https://cyclonedds.io/docs/cyclonedds/latest/config/network_interfaces.html)
4. 用户已完成 Quest 到 NX 的 PC Service 连接和真机遥操测试。网络模式或地址改变时，在头显中填写当前可达的 NX 地址，不要根据机器人有线网段猜测。

同为 Domain 21 只是必要条件，不能替代网卡和跨机器发现检查。

## 已完成的仓库适配

以下为 NX 适配文件和当前运行结果：

| 文件 | 已完成的适配 | 当前结果或注意事项 |
| --- | --- | --- |
| [`src/scripts/build_xrtele.sh`](src/scripts/build_xrtele.sh) | 按架构查找 SDK 库，支持显式路径覆盖；SDK 路径通过环境变量传给真机包，避免向其余包传入无关 CMake 参数。 | NX 已完成构建。 |
| [`src/qiling_kinematics_real/CMakeLists.txt`](src/qiling_kinematics_real/CMakeLists.txt) | 使用找到的 SDK 库路径链接并设置 RPATH。 | NX 已编译并完成真机遥操测试。 |
| [`src/scripts/start_xrobotoolkit_real.sh`](src/scripts/start_xrobotoolkit_real.sh)、[`stop_xrobotoolkit_real.sh`](src/scripts/stop_xrobotoolkit_real.sh) | 通过厂商 `runService.sh` 启动服务，不再要求 x86 Unity Demo；只停止本脚本记录的服务进程。 | NX 的 PC Service 已用于真机遥操测试。 |
| [`src/scripts/start_real_teleop.sh`](src/scripts/start_real_teleop.sh) | 自行 source ROS 2/工作区环境，设置 Domain 21、CycloneDDS，并默认读取 NX DDS 文件。 | 用户已完成真机遥操测试；网卡变化时重查 DDS 文件。 |
| [`src/scripts/install_xrtele_dependencies.sh`](src/scripts/install_xrtele_dependencies.sh) | Ubuntu 22.04 ARM64 依赖已安装。 | 首次本机构建已成功；Pinocchio 引出的 Boost `python310` 提示为非致命 CMake 警告。 |

不要把板子现有的 `build/`、`install/` 目录或 AMD64 `.deb` 当成 NX 的可执行产物；在 NX 上从源码构建。此仓库的仿真包已删除，不需要为 NX 安装 MuJoCo GUI 依赖。

## 分步部署与验收

1. **确认平台和包**：本机已完成安装，服务程序、头文件和 ARM64 SDK 库存在。换包或换机时重新核对包哈希、架构和文件布局。
2. **确认两条网络**：验证 NX ↔ 板子的 IP 连通；确认板子现有 CycloneDDS 接口覆盖 NX 的连接路径，并配置 NX 的 DDS 网卡；确认头显能访问 NX 的 PC Service IP。
3. **安装依赖并在 NX 编译**：已执行安装和 `./src/scripts/build_xrtele.sh`；5 个包全部成功。不要沿用板子上的 x86 编译结果。
4. **先验证 ROS 状态**：已在 NX 收到一帧 `/human_lower_state`，`joint_states.position` 恰好有 26 个有限数值；`ros2 topic hz` 测得持续频率约 1 kHz。
5. **验证头显链路**：headless PC Service 和 PXREA 适配器已连通，左右控制器位姿各约 90.9 Hz；用户后续已完成实际 Grip/Trigger 遥操测试。
6. **真机遥操**：用户已验证当前双臂与手部控制正常。日常启动按根目录 README 分别执行 `start_xrobotoolkit_real.sh` 和 `start_real_teleop.sh`；两个脚本无需在终端预先手动 source/export。后者收到 `/human_lower_state` 后会自动开始“当前姿态 → 过渡点 → home → 遥操”，启动前仍需确认双臂周围无障碍物、急停可用。

本交接不要求修改机器人腿部控制流程；也不要额外启动一份默认的 `qi_topic_converter`。如果某一步失败，记录 NX 系统版本、网卡/IP、ARM64 SDK 实际路径、DDS XML、相关进程日志和上述话题的发布/订阅情况，再定位该步。
