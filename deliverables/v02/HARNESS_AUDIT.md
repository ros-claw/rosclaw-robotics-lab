# 实测问题到通用能力的审计

| 问题 | 分类与现有能力 | 本轮处理 | 证据边界 |
|---|---|---|---|
| 不存在的二维 Topic / 传感器类型 | 通用 Harness；已有 Topic presence / QoS / 语义 resolver | #642 增加显式 Body required type 不匹配；原生 probe 读取插件配置 | 名称不能推断 LaserScan；未提供 required type 时不编造 |
| QoS 请求/提供不匹配 | 通用 Harness 已有 ROS_QOS_001 与原生端点采集 | 保留并回归测试 | 不等于丢包率或实际传感器质量 |
| TF、use_sim_time、时钟新鲜度 | 通用 Harness 已有 TF/时钟及 stale snapshot 检查 | 复用已有代码；暂停补测单列 | 历史快照不能包装成实时健康 |
| AMCL 异常 | 通用定位诊断 + 平台校准 | 复用 localization covariance/TF 检查；仓库仿真低噪声参数留在实验室 | 仿真校准值不能自动应用真实机器人 |
| Costmap Layer 覆盖 | 通用诊断 + 可复用修复验证步骤 | #642：按实际插件类、顺序、enabled、显式 use_maximum 及新鲜参数给出来源明确的风险警告 | 配置风险不证明实际障碍丢失；LiDAR/Costmap/PhysX 另验 |
| Lifecycle / Action readiness | 通用 Harness 已有生命周期与 action interface 检查 | 复用 wait_navigation 就绪门槛；失败启动保留 | 服务存在不等于已可执行 |
| 第一站后 TaskKernel 提前完成 | 通用多阶段验收契约 + 应用接入 | 本场景单站证据留收据，不注册为整项任务交付；最终 verify-and-remember 校验有序全部收据 | 这不是上游已实现任意自然语言多阶段目标验证的证据；不能靠词表猜用户所有要求 |
| 恢复预算 | 通用有界执行 + 场景调参 | 原有 bounded recovery 复用；#644 严格要求修复超时取消返回 CANCELED | 六项预算是本场景配置，不是普适最优值 |
| 取消发送失败 | 通用 ROS transport / SIM cleanup | #644：抛出发送错误，保留失败证据，继续尝试禁用和撤租 | dispatch / cancel response / CANCELED / 物理停止分别记录 |
| Navigator ABORT 与控制器活动 | 通用只读诊断 + 隔离仿真补测 | #646：按命名空间与原始时间记录状态矛盾，明确 ownership 未证实；补测使用实际 Nav2/FollowPath | 不依据状态 Topic 自动 cancel-all；物理停止须独立证明 |
| Hawk 循环 payload / PhysX 固定后端 | Isaac 6.1 专属适配 | session layer 禁用非必要 Hawk ROS variant，保留 LiDAR；固定 PhysX | 不修改 NVIDIA 上游资产，不称为通用 ROS 修复 |
| Native 长路径启动失败 | 通用 Native 开发体验 | #648：启动前按编码字节长度检查 Unix socket，错误提示短路径 | 不把控制 socket 迁到公开临时目录 |
| 动作 JSON 写入/读取竞争 | 本实验适配器证据发布 | 临时文件完整写入 + fsync + 同目录原子替换；动作/Memory/daemon-ready 一致处理 | 合成竞争回归不代替最终版本 15 轮实际验收 |
| 认证过期 / bridge 尚未就绪 | 模型/集成就绪 | 等待实际 TCP；隔离目录仅复用同账户有效 access token，不修改全局认证 | 不把重试和重新登录隐藏成 Agent 自主恢复 |

通用能力进入已有 Harness、probe、action client、SIM executor 或 Native CLI，不建立第二套安全 Runtime/Memory。修复建议应先有来源，再在隔离仿真验证；本轮没有给真实机器人自动写入运动参数。Runtime 本场景 `enable_firewall=False`，Memory SQLite，不能据此宣称全部安全模块或 seekdb 性能已验收。

PR 状态及合并 SHA 以最终 [交付清单](STATUS.md) 为准。主仓库新功能的单元/CI 验证与仍固定旧上游版本的仓库物理回归分开记录，不混成“新上游已完成全部物理验收”。

## 本轮新增 Provider 停滞诊断

实际箱体任务两站后发生 30 秒无首 token，SDK `aborted` 被显示为 MODEL_UNKNOWN。独立 [PR #650](https://github.com/ros-claw/rosclaw/pull/650)保留看门狗来源、明确超时码及 Provider PAUSED；复用已有错误分类/闸门，不自动重发、不改变运动取消。另有 lab 验收器遗漏 `aborted` 导致等待既有外层期限，列为后续协议修复；本轮冻结源不悄悄变更。[失败与边界](PROVIDER_FAILURE.md)。
