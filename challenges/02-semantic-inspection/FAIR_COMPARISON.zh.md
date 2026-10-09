# 公平对照的可执行研究方案

状态：**方案已准备，实际对照 NOT RUN**。不能把本机巡检回归或只读位置原型当作对照成绩。正式试验前必须完成动态目标的不可变 SIM Body、daemon 执行接入及独立传感器验收。

## 运行试验：先控制相同工具，再分离 Harness 贡献

主试验比较两个 Agent 在相同模型、基础观察/候选/受控导航工具和独立评价器下的任务运行。两个系统都通过同一 rosclawd 物理边界，获得相同原始 ROS、地图、USD 与位姿输入；不提供整项任务脚本，不允许直接发轮速。工具名称不同可以映射，但参数、能力、返回证据和调用权限须等价并经审计。

若要进一步声称 ROS Expert Harness 有贡献，增加预注册的 2×2 消融，避免把框架、工具能力和模型混成一个因素：

| 条件 | Agent 编排 | 专家诊断/上下文工具 |
|---|---|---|
| A | 通用 Agent | 相同基础工具 |
| B | 通用 Agent | 基础工具 + ROS Expert Harness |
| C | ROSClaw Native | 相同基础工具 |
| D | ROSClaw Native | 基础工具 + ROS Expert Harness |

A/C 与 B/D 控制工具暴露来考察编排差异；A/B 与 C/D 考察专家工具增量。报告交互效应，不能将 D/A 差异全部称为模型优势。若某一组合尚不能按等价工具契约运行，标为未实现，不能拿另一个不受约束的实现替代。Nav2 参数、Body、地图与物理门槛四组一致，Nav2 自身的局部避障贡献不归为 Agent 自主运动控制。

## 冻结与配对

开发集 pilot 只用于确定时间/Token/恢复预算和样本规模，全部 pilot 结果另列。至少准备每任务族 30 个配对情形；正式样本量根据 pilot 的配对差异与可接受误差再冻结。仅有 30 对并不自动具备充分检验效能。

冻结模型/provider 实际版本与设置、提示词、工具契约/源码哈希、Body、候选算法、评价器、初始位姿、目标 manifest、障碍布局、随机种子、最大墙钟/Token/调用/恢复预算。模型服务在测试期变化时暂停并另开批次。先保存目标实体 manifest 的哈希，才揭示 holdout 目标；同一物理货架父子 prim 必须处于同一组。

每个情形随机化 Agent/条件执行顺序并保留种子。每次新环境重置、新 Native home、清空测试 Memory；认证配置相同且独立于任务经验。已有经验复用作为另一个消融，冻结相同训练语料并防止 holdout 泄漏。记录首次安装、缓存状态与环境就绪，不将安装耗时混入已准备环境的任务延迟。

## 验收和分母

物理成功须同时满足正确语义目标、候选/Body/新鲜路径门槛、实际到达与朝向、稳定停留、目标可见传感器证据、零非地面碰撞、完整阶段顺序与最终收据。模型文字和 Nav2 SUCCESS 都不独立决定成功。

首要指标是一次任务输入的整项成功与安全门槛；同时报告不安全候选拒绝、错误目标、超时、工具失败、恢复、人工介入、Token、调用数与墙钟。所有失败保留分母；成功样本的耗时另列，不能用成功者均值掩盖超时。配对成功差使用配对方法和不确定区间；耗时包含超时截尾并报告预算，不将未完成当作零秒。

独立评价器在试验前冻结，结果文件由观察者生成；Agent 不能修改评价器或目标真值。安全观察与取消可随时介入，但介入计入结果，不能静默补救后计为一次输入成功。

## 应用开发效率：另设研究

起点是参试者收到同一现有 ROS 机器人环境、任务 brief 和相同文档/网络权限；终点是首次独立物理验收通过。记录墙钟与有效工程时间、人工介入及理由、配置/代码 diff、依赖安装、失败修复、复用已有代码和不可完成/预算耗尽。环境安装另记。训练/熟悉 pilot 不使用 holdout 任务。

使用未参与本实现的工程师，在等价新环境中随机分组或交叉平衡经验与顺序；同一人做第二组时的学习效应须记录。成本和时间预算在揭示任务前冻结。运行对照的成功率不能替代应用开发效率结论；本次 Codex 工程实施与 ROSClaw 任务运行的分工记录不能证明任何一方更优。

## 建议逐次记录字段

`study_id, protocol_sha256, pair_id, condition, execution_order, seed, target_manifest_sha256, source_sha, tool_contract_sha256, model_version, body_hash, reset_observer_id, budget, timestamps, final_state, failed_gates, collisions, candidate_rejections, interventions, tokens, tool_calls, evidence_manifest_sha256`。

开发研究另加 `developer_experience, environment_received_at, active_work_intervals, dependencies, files_changed, diff_sha256, first_physical_acceptance_at`。最终报告须保留缺失字段及原因，不以估算填充未采集事实。
