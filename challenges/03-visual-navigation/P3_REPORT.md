# P3 受保护的单分叉控制测试

控制测试在一个灰盒仿真中执行真实 Nav2 规划，用实际代价地图、完整路径、配置足迹和中性门区预检。速度链经 velocity smoother 与 collision monitor。Isaac 的最终 Twist 入口只有这一位发布者。每次动作请求保留独立日志，实际位置、速度与接触来自独立 PhysX witness。

## 本机适配

- 基线用 Fast DDS 可启动。初期受保护试次的动作客户端未就绪。受保护入口采用非组合 Nav2 节点与 Cyclone DDS。回环配置增加 participant 索引范围后取得了真实规划结果与 FollowPath 回执。
- 在灰盒上将仓库预设的 0.25 m 足迹 padding 改为 0.05 m。预检仍使用矩形足迹的保守包络。标牌移到实体隔板面上以消除悬挂标牌对通道 LiDAR 投影的干扰。
- 仓库 USD 原播放范围为 1000 帧 / 60 Hz。物理采样停在约 16.65 秒，而 ROS 时钟继续。入口在开始播放前关闭循环并延长时间线，随后 PhysX 记录越过原边界。
- 本机 TF 测得 LiDAR 高度约 0.526 m。受保护灰盒的投影扫描保留 LiDAR 帧内 −0.12 m 至 +0.35 m 高度，以排除地面误投影并覆盖灰盒障碍。Humble 碰撞监控器使用 max_points。
- DWB 试次 20261121T000902Z-dabd9e25 完成规划、全路径预检和动作接受。底盘发生真实移动，之后多次触发碰撞监控，控制器中止。试次 20261121T000709Z-fd21cc13 另保存了 12 组运动期间的 RGB-D 和采集时刻 TF。

## 已验证的取消

UTC 试次 20261121T001217Z-b96b6215 使用 `python3 scripts/lab.py controller-test c24 --cancel-after 4`。在收到 FollowPath 接受后发起取消，Nav2 返回状态 5，里程计通过 0.75 秒持续低速验证。PhysX 记录 220 个动作区间样本，末段低速窗口约 0.733 秒，最高平面速度为 0.0142 m/s。记录包含地面接触，非地面接触对为零。车辆在门区前终止。

证据目录为 `/data/rosclaw/robotics_lab/p2-evidence/protected/20261121T001217Z-b96b6215`。独立结论位于 physx-evaluation.json。回归测试覆盖仿真时钟与里程计回调换序、采样新鲜度和监督进程更替的清理范围。

其他试次保留动作发现失败、路径预检拒绝、DWB abort 与早期停止判定失败。P3 完整阶段门还需要真实过门与到达、连续路径更新、ROSClaw daemon 注册和异常恢复。

## MPPI 对照

相同灰盒和地图生成了 MPPI 配置，速度上限设为 0.45 m/s，作为低速联调值。控制器、采样和评估均走同一保护链。UTC 试次 20261121T001407Z-3c4f5321 成功加载和激活 MPPI，完成真实规划、路径预检和 FollowPath 接受。随后 Nav2 因进展不足中止。PhysX 独立记录 217 个样本，位移累计约 0.119 m，未进入门区，确认中止后的低速驻留。没有非地面接触。完整路线的判定仍为失败。

复现该对照时，在 P2 的场景导出命令上加 `--controller mppi`，再使用受保护启动和相同门区命令。真实运行和完整失败记录位于 `/data/rosclaw/robotics_lab/p2-evidence/protected`。

## 速度链补测

UTC 试次 20261121T052239Z-376c1fa5 同步采样 controller、smoother、collision monitor 和 chassis odom。动作接受 3 秒后至中止前，碰撞监控输出的线速度和角速度中位数约为 0.061 m/s、0.301 rad/s，实测 odom 对应约为 0.008 m/s、0.021 rad/s。上游三个指令话题的中位数相近。该试次的主要差异位于最终 Twist 进入仿真后。FollowPath 因进展不足中止，里程计低速停车验证通过。速度对照位于 p4-evidence/mp-pipe/20261121T052239Z-376c1fa5/motion-summary.json。关节驱动与地面接触之间的具体原因仍待定位。
