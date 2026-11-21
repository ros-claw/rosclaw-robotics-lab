# Luna 视觉导航当前架构

`scripts/lab.py` 是工作站入口。每次 `start` 创建独立运行 ID 和证据目录。监督进程持有项目锁，管理一个 Isaac 子进程和一个命名的 Nav2 容器。DDS 使用专用 ROS 域。基础仓库模式采用 Fast DDS，受保护灰盒使用本机 Cyclone DDS 回环配置，后者显式增加了可发现的 participant 索引范围。

Isaac 6.2 负责场景、Nova Carter 的关节与底盘、LiDAR、RGB-D 和时钟。灰盒由 SimScene 离线导出后合入同一个 USD 世界。宿主 Humble 进程接收图像与 TF，匹配同一时间戳和光学帧，保存只读观测文件。PhysX witness 直接读取底盘刚体状态和接触对，独立于里程计与 Nav2 结果。

受保护灰盒的速度链为 DWB controller → velocity smoother → collision monitor → Isaac 的 `/luna/cmd_vel` 订阅。入口启动不会发移动目标。显式的 `controller-test` 才会在专用运行中用指定的中性门区测试 `ComputePathThroughPoses`、完整路线检查和单个 `FollowPath`。执行之前要求最终 Twist 入口只有一位发布者、仿真端只有一个订阅。动作失败时终止本次运行，运行 ID 核对保证清理范围。

当前执行器是开发验收用入口。持续的 ROSClaw session owner、可抢占更新的路径租约、Luna 请求与决策事件尚未连线。轨迹证据与可视化交付状态见各阶段报告。
