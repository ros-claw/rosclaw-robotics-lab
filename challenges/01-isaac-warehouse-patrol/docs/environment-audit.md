# 环境审计（2026-10-08）

- aarch64，Ubuntu 24.04.4，NVIDIA GB10，驱动 580.159.03。
- 安装包 MD5 实测匹配 NVIDIA 6.1.0 aarch64 公布值。
- 解压到 `/home/nvidia/isaacsim`；VERSION 为 `6.1.0-rc.26+release.49347.2d230af4.gl`（官方发布 ZIP 内部版本字符串，实际启动已验证）。
- 宿主机有 Jazzy、colcon、rosdep；缺 Nav2。sudo 非交互不可用。
- 使用已有 ARM64 `rosclaw/ros-expert-jazzy:acceptance` 容器进行构建，不改宿主机驱动或 ROS 安装。
- 官方工作空间固定 commit `a9e8471ee901bc2332c1e4aca94ac580713ca3ab`（IsaacSim-6.1.0），子模块已初始化。
- carter_navigation、isaacsim_bringup、isaac_ros_navigation_goal 构建成功。构建产物必须在相同容器路径 `/work` 中使用。
- 默认资产服务仓库 USD HEAD 返回 200，2865077 字节；主场景已加载；完整引用审计见后文。
- ROSClaw CLI 1.3.0 指向本机 G12 runtime checkout；不能默认它与最新上游完全一致。
- 当前仅 GDM 桌面，无 DISPLAY；其他 Gazebo 实验正在运行，不停止它们。
- 本项目固定 ROS_DOMAIN_ID=61。启动前需继续确认此域未被其他实验使用。

## 未完成的验收

EULA 已确认；post_install 成功，兼容性结果 PASSED；预热成功并收到真实 NEW_FRAME。Streaming、场景渲染、独立 PhysX 采样、TF、LiDAR 与三目标导航均已实测，后文保留完整修复经过。Agent、三次正式巡检物理评价和宣传视频尚未实现。

## 官方依据

- [NVIDIA 发布与 MD5](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/download.html)
- [Standalone 安装](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/install_workstation.html)
- [官方固定工作空间](https://github.com/isaac-sim/IsaacSim-ros_workspaces/tree/IsaacSim-6.1.0)

完整机器可读证据：`../reports/environment.json`。不将构建成功当作导航或物理测试通过。

## 首次运行与修复记录

- Streaming App ready：238.927 秒（冷 Shader Cache）。
- 官方场景加载并开始 Play：572.050 秒（日志时间）。
- 宿主机 /clock 实际接收约 15–16 Hz；/chassis/odom 数据有效，frame_id=odom、child_frame_id=base_link。
- ARM64 容器默认 IPC 隔离时能发现 graph，但订阅超时。单侧 UDP profile 未解决；使用 --network host --ipc host 后收到 clock。默认执行脚本采用这个已测配置。
- 基础镜像缺 ros-jazzy-pointcloud-to-laserscan；已在项目新镜像补齐，当前镜像 ID 记录在 config/runtime.env。Dockerfile.local 依赖本机基础镜像，公开复现配方仍待完善。
- Nav2 官方参数与地图首次启动成功，bt_navigator active，日志确认 Managed nodes are active。尚未发送正式巡检目标。
- 资产 endTimeCode=1000、timeCodesPerSecond=60；运行时出现周期动画时间轴重置和控制器 deltaTime=0。下一次启动延长时间轴并禁用循环，待实测。
- RTX LiDAR 提示 motion BVH 未开启；下一次启动显式启用 renderer.raytracingMotion.enabled，待实测。
- 本次启动没有独立物理状态采样或截图，不能算完整官方导航基线。保留完整日志，正在带观测器重启。

## 观测器首次失败

第二轮 Nav2 再次 active；baseline_observer 异步函数因函数内 `import omni.physics.tensors` 将 omni 变成局部变量，触发 UnboundLocalError。该轮无独立物理采样或截图，不能计为有效证据。已保留 observer-failed-kit.log，改为 `from omni.physics import tensors`，补充 Ruff F/E9 检查与显式 UNKNOWN 错误文件。正在第三轮启动中复测。

观测器同时准备按物理 Tensor API 连续读取 chassis 状态、采集原始 PhysX 接触事件和真实视口帧。接触事件未完成地面分类之前，碰撞验收必须保持 UNKNOWN。这些接口仍需本机运行验证。

## Isaac 6.1 物理后端固定

完整应用的 isaacsim.physics.newton 扩展默认 auto_switch_on_startup=true；需要明确选择物理后端。观测器另报 Failed to get a valid attached USD stage id；固定 PhysX 后仍出现该错误，不能归因于 Newton。检查 6.1 当前实现发现需要传入实际 stage_id，目前已按 USD Context 显式绑定并重测。启动参数现固定 default_engine=physx、auto_switch_on_startup=false，并在观测器中核验 SimulationManager.get_active_physics_engine。该配置需要重新实测；此前运行不是 PhysX 独立状态验收通过。

接触报告 API 的包内旧文档方法名不可用；按实际 ContactReportDemo.py 改用 CreateThresholdAttr。相关失败日志全部保留。

## 三目标基线实测

2026-10-08，官方目标发送器完成三次 NavigateToPose，error_code=0，正常退出。独立 PhysX chassis 位置误差分别为 0.107、0.297、0.543 m。第三点不满足正式巡检的 0.4 m 容差，不能将三次 Nav2 成功等同于正式物理验收通过。完整基线记录在 reports/official-baseline-summary.json，原始轨迹与时间戳视口录像另存。录像是离散真实视口帧序列，保留墙钟时长，不是连续 WebRTC 屏幕录制。

ROSClaw 原生 Probe 和 rosbridge discovery 已实际采集；inspect-system --deep 成功生成 system-model.json。/scan 与 /map 最初快照未发现，延长 DDS discovery 并使用地图的 transient-local QoS 后均收到数据。TF chain 可查询，Nav2 生命周期 active，use_sim_time 为 true。

当前定位在第三点的 map/base_link 与独立物理位置相差约 0.53 m。官方 AMCL alpha1–5=0.2；正在测试仅针对理想仿真里程计的低噪声候选参数及 0.10 m Nav2 到点容差。候选文件单独保存，不修改 NVIDIA 上游源文件，未经复测不得宣称校准成功。

## 校准与一键启动结果

独立重置后的低噪声 AMCL 候选配置实测：三个物理位置误差为 0.146 / 0.174 / 0.079 m，bt_navigator 分别记录 Goal succeeded。基线精度通过 0.4 m 阈值，正式 Agent 巡检尚未运行，碰撞验收仍 UNKNOWN。评分器四项防误判单元检查和静态检查通过。

2026-10-08T08:51:32Z 一键启动重置测试成功：独立 Physics OBSERVED，Nav2 ACTIVE，NavigateToPose READY，且没有自动目标节点。USD composition audit 检出四个 Hawk ROS 摄像头循环 Payload，无 unloaded payload；非必要摄像头 ROS variant 的 session-layer 禁用方案正在复测。

2026-10-08T08:56:17Z 最终重置一键启动复测通过。关闭四个 Hawk ROS variant 后，composition errors=0、unloaded payloads=0、106 个已使用 USD layer，Physics 与 Nav2 ACTIVE/READY 正常；没有 set_navigation_goal 节点。NVIDIA 原始资产不改写，修改仅存在 session layer。冻结的最终场景审计在 reports/verified-scene-audit.json。
