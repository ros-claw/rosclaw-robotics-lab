# Luna 视觉导航本地交接

代码位于 `/data/rosclaw/robotics_lab/rosclaw-robotics-lab` 的 `luna-nav-20261121` 分支。[README](README.md) 记录启动、抓帧、控制测试和停止。一次 stop 后再 start 会创建新世界和新运行 ID。

| 阶段 | 已有证据 | 待验收 |
|---|---|---|
| P0 | Isaac 6.2、Humble Nav2、时钟、TF、点云、扫描、正常停止及超时清理 | — |
| P1 | RGB-D、标定、采集时刻 TF、哈希和 10 个标靶测量 | 运行中暂停/reset、定位注入和动态外参定量检查 |
| P2 | 单分叉灰盒、地图和换牌导出。公开清单与地图逐字节一致。禁止门区回归测试通过 | 在线候选投影、执行中通道约束、L1/L3 和验收种子 |
| P3 | 受保护速度链、动作回执、PhysX 位移、取消和停车复核 | 完整物理过门与到达、连续改路和 ROSClaw 会话 |
| P4–P7 | 离线设计原型和底层运行证据 | Luna API、语义闭环、实时决策画面、回放、成片与完整验收 |

原始记录位于工作区的 p0-evidence 和 p2-evidence。各试次保留 manifest、JSON、JSONL 与原始日志。当前会话未提供 OPENAI_API_KEY，尚未进行模型网关调用。

P3 的 DWB 和 MPPI 完整路线均因进展不足或控制器中止结束。取消停车通过独立物理复核。恢复连续导航验收需要首先解决灰盒中的底盘进展问题。完整试次索引为工作区的 attempts.jsonl。
