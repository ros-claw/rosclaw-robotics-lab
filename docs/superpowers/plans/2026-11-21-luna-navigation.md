# Luna 视觉导航与决策可视化 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. 若本地没有这些技能，按本文的任务、接口和验收逐项执行即可；缺少技能不是停工理由。

**Goal:** 在 Ubuntu / Isaac Sim 6.2.0 上，以 Nova Carter、Luna 图像决策、Nav2 连续导航与 SimScene 静态场景构建一个真实物理运行、动作连贯、决策可视化且能复核的演示。

**Architecture:** SimScene 离线准备环境 USD；一个 Isaac 实例运行机器人、传感器、物理和实时采集。ROSClaw 的一个持续导航会话拥有全部 Nav2 运动请求，Luna 作为异步、无运动权限的图像决策组件。只读可视化与独立评估订阅同一事件流，显示模型输入、提议、校验、执行和实际运动。

**Tech Stack:** Isaac Sim 6.2.0 / PhysX；实际系统匹配的 ROS 2（优先 Ubuntu 24.04 + Jazzy）；Nav2；Python；OpenAI Responses API；轻量本地 HTML/CSS/JavaScript viewer；FFmpeg；可选 Rerun 导出。

**Spec:** 本文件第 1–8 节是本次已确认方向的完整落地规格，第 9–13 节是执行任务、验收和交付。配套 LUNA_NAV_DECISION_VIEW_20261121.html 是使用合成数据的交互设计原型，不是已运行导航系统。

**日期:** 2026-11-21（Asia/Shanghai）  
**交付对象:** 用户本地 Codex  
**实施位置:** 用户 Ubuntu 工作站；Isaac 路径 /data/omniverse/isaac-sim-6.2.0  
**本文件状态:** 经仓库源码与官方文档核查的实施计划。未在该工作站执行；所有频率、速度、延迟与成功率目标均不是已测结果。

## Global Constraints

1. 在 ros-claw/rosclaw-robotics-lab 新增 challenges/03-visual-navigation/。先读取本地 AGENTS.md、git status 和已存在改动；创建隔离分支/工作树，不覆盖其他 Agent 的工作。
2. 一个 Isaac 世界、一个物理时钟、一个导航会话拥有者。Luna worker 和 viewer 均无直接控制底盘权限。
3. 基础配置指向 /data/omniverse/isaac-sim-6.2.0；实际版本、CPU 架构、ROS 发行版由 P0 探测，不从目录名推断。
4. 首版使用现有 Nova Carter。RGB 给 Luna；同步深度用于本地空间落地与投影；保留 LiDAR、AMCL、基础占据地图和 Nav2。成果名称为“Luna RGB 视觉决策 + 深度/激光几何导航”，不称纯单目端到端。
5. 机器人通过真实关节驱动和物理接触运动。初始化复位可以设置出生位姿；任务运行中不得以 root transform 动画、沿轨迹传送替代底盘执行。
6. 运行中不暂停物理等待 API、录屏或 UI。离线轨迹重渲染另标 replay-render，不计作实时运行证据。
7. 中间通道没有固定停车/驻留要求；最终目标要求独立验证。当前逐站 PointExecutor 不能直接充当密集视觉路点执行器。
8. 结构化输出只提出有限意图；坐标转换、候选可达性、有效期、速度上限和完成判定由本地执行器决定。
9. 控制链只有一个最终速度入口：controller → velocity smoother → collision monitor → robot。stop/cancel 通过唯一执行者及最终限速/停止机制完成。
10. 每次尝试在启动前登记；失败、取消、API 错误、录像不完整均保留。模拟数据、真实运行、原始回放和画面重渲染须明确区分。
11. 凭据只从本机环境/现有凭据设施读取，日志、画面、Git 和交付包不含 Key。缺 Key 时先完成非 API 阶段，准确报告缺项，不以 mock 冒充模型实测。
12. 项目目标是可用 demo 与清楚的能力证据。阶段门满足后推进，不为增加测试数、文档数或架构层数无限扩张。

## Review Focus

| 高风险输入/条件 | 必须满足的行为 | 所属任务与测试 |
|---|---|---|
| reset、暂停、仿真时钟回退时仍有 API 回答或动作回调 | epoch 不符即拒绝；停止未经物理验证不报告完成 | P1/P3；test_epoch_reset_rejects_old_result |
| 模型在旧图上选择了现已封堵的通道 | 当前几何重新校验，拒绝提议；不覆盖有效旧路线 | P3/P4；test_stale_candidate_never_commits |
| 规划器能从另一侧“抄近路”到同一个路点 | 选择的门区/通道成为路径与执行约束；不能悄悄换路 | P2/P3；test_committed_gate_cannot_be_bypassed |
| Nav2 顶层动作结束，但 FollowPath 仍驱动车体 | 取消所有本会话活动运动、验证实际速度；旧回调不结束新任务 | P3；test_cancel_stops_active_follow_path |
| UI/录像变慢、回放 seek、事件迟到 | 运动不被展示阻塞；回放只显示当时已收到的信息 | P5/P6；test_replay_never_shows_future_decision |

---

# 1. 首版成果与范围

## 1.1 用户能看到的任务

“从入口出发，按照现场指示前往蓝色 B 工位，绕过货物，选择允许通行的通道，在工位前平稳停下。”

一次完整任务应包含：

- 开阔路段加速；
- 交错箱垛形成的连续弯道；
- 遮挡后重新观察；
- 至少一个由视觉标识决定的分叉；
- 窄通道减速；
- 最终目标确认与停车。

实际任务路径建议 40–60 m，场地约 20×15 m，主要静态障碍 12–20 个。它们是布局起点；若实测转弯和拍摄不合适，调整布局并记录最终值。

## 1.2 必须交付的能力

A. 本机启动、复位、运行、停止的一套入口。  
B. 真实 RGB-D 观测和图像时刻的 TF。  
C. 异步 Luna 意图生成与本地验证。  
D. 一个持续导航会话，具备通道保持、连续路径更新和有界恢复。  
E. 真实观测驱动的几何避障及有效碰撞监控。  
F. 可读的实时决策界面与离线事件回放。  
G. 可复核的物理运行与成片；分别报告运动、API、RTF、采集帧率。  
H. 成对语义实验，证明视觉变化确实改变行动。

## 1.3 阶段成功的表达

分别声明“底座可运行”“Luna 已接通”“连续导航通过”“语义对照通过”“实时展示通过”“影片完成”。任何一项未完成，整体报告明确列出，不拿视频质量代替控制验收。

---

# 2. 已核实的底座与借鉴边界

## 2.1 本次代码阅读基线

| 仓库 | 阅读提交 | 结论 |
|---|---|---|
| ros-claw/rosclaw-robotics-lab | 545830cbd2ea8fd20fb579edb113822f7d3c9841 | Isaac 6.1、Nova Carter、ROS2/Nav2、单站执行与独立证据基础；不是已完成的 Luna 视觉导航 |
| ros-claw/simscene | 3660dc1b37610d7b171934dd86ff67df01107d50 | 资产发现、USD 组合、场景测量与拍摄可借鉴；现有 robot pose_follower 主要是机械臂 |

本地代码若更新，先比较这些模块的差异，再锁定本次实施提交。不要为符合本文件而把用户的新改动回退。

## 2.2 robotics-lab 必须借鉴的具体代码

路径均相对仓库根目录。完整来源见第 12 节。

| 参考文件 | 借鉴什么 | 本次需要改变什么 |
|---|---|---|
| challenges/01-isaac-warehouse-patrol/scripts/setup.sh | 依赖安装与工作区构建组织 | 原 ARM64、6.1 分支/提交绑定改为本机验证的配置，不能全局文字替换 |
| .../isaac/asset_resolver.py | 从安装包官方区域 profile 解析资产根 | 新 challenge 不能继续硬性只接受 /Assets/Isaac/6.1 |
| .../scripts/start-sim.sh | 单 Isaac 启动、PhysX、ROS bridge | 加载新环境与传感器 hook；启动不自动发目标 |
| .../rosclaw/daemon.py | ActionGateway 注册与 transport 生命周期 | 新增视觉 session executor；沿用 Body 与动作所有权 |
| .../rosclaw/point_executor.py | goal ID、失败取消、freshness、独立停止验证 | 不复用每个中间点 2 秒驻留和逐站闭合循环 |
| .../rosclaw/prepare.py | inventory → Body/binding/contract | 新 profile 使用实际 6.2 prim、关节、传感器和 session capability |
| challenges/02-semantic-inspection/semantic_executor.py | 不可变候选、proposal 与 freshness | 候选来自在线几何，语义由真实图像决定；不读 USD 目标名称取得答案 |
| .../loading_executor.py | 路径预检、拒绝与收据 | 不搬入装卸检查专用分类逻辑 |
| .../loading_scene.py | session layer 中加入带碰撞静态箱体 | 转为新场景配方和固定种子 |
| .../isaac/baseline_observer.py | PhysX 位姿与接触独立观察 | 分离旧 Hawk 禁用逻辑、低频 PNG 与镜头跟随，不直接复用其截图循环 |
| .../isaac/camera_setup.py | 第一人称、跟随、顶视的构图方法 | 跟随镜头按渲染更新，观测相机固定在机器人，不能跟随构图改变模型视角 |
| .../config/patrol_navigation_params.yaml | topic、frame、footprint 和 DWB 基线 | 配置 MPPI 对照，测真实更新率；修正异常 min_points |
| .../ros2/cancel_navigation.py | 顶层导航和 FollowPath 的取消 | 所有会话退出路径均验证实际停下 |

必须记住四个事实：

1. 原 RGB 是 display-only；observer 会禁用部分原装相机链路。
2. 原 PointExecutor 有单动作锁和逐站驻留。
3. 原报告中约 1.135 fps 的采集与约 0.404 的 RTF 是历史运行数据，不是本机目标，也不是实时可视化实现。
4. 所读配置的 CollisionMonitor.FootprintApproach.min_points 为 6000000000，正常扫描难以触发该 approach 区域；新配置必须改成与传感器采样和区域大小匹配的值，并实测触发。这一发现针对该区域，不表示其他保护全部不存在。

## 2.3 SimScene 必须借鉴的具体代码

| 参考文件 | 借鉴内容 | 使用边界 |
|---|---|---|
| src/simscene/mcp_server.py | scene_apply、scene_measure、scene_validate、scene_export 等场景工具 | 用于准备和检查环境，不建立第二个在线机器人世界 |
| src/simscene/scene.py / usd_export.py | 米制布局、USD 引用、碰撞和视觉分离 | 静态 primitive 显式 collision:true；检查代理与传感器可见网格一致 |
| docs/asset-catalog.md | 资产选择、依赖、准备与版本记录 | 当前 GLB 验证为 6.2 RC14 等固定环境；本机另做 probe |
| skills/build-scene/SKILL.md | 测量后摆放、真实画面检查与迭代 | 本任务的道路、机器人 footprint 和可达性由导航验收决定 |
| src/simscene/capture.py / worker/video.py | 画幅、帧时刻与视频索引 | 可以复用数据记录思路 |
| worker/camera_frames.py | 图像 buffer 新鲜度问题的经验 | paused_render_barrier 不进入本次实时控制/录制循环 |
| skills/deliver-scene-video/SKILL.md | 镜头、来源和时间映射 | live 与离线重渲染分别标注 |

首版优先少量原生 USD + 基本几何碰撞体。HF 资产按单件选择，不下载整个库，不因装饰资产转换延误导航闭环。

---

# 3. 运行架构与文件组织

## 3.1 进程与所有权

| 单元 | 运行环境 | 权限与职责 |
|---|---|---|
| Isaac | 安装包自身 Python / PhysX / renderer | 唯一世界，机器人关节驱动，RGB-D/LiDAR/odom/clock，独立物理观察 |
| ROS2/Nav2 | 与本机匹配的 ROS 环境或容器 | AMCL、costmap、规划器、MPPI、smoother、collision monitor |
| 观测节点 | ROS2 Python 环境 | 订阅传感器、TF 和代价地图，生成有版本的观测与几何候选；不发运动目标 |
| ROSClaw daemon / VisualSessionExecutor | 现有项目环境 | 唯一 session owner；校验 Luna 意图；拥有 Nav2 action client；控制停止与终态 |
| Luna worker | daemon 后台异步任务优先 | 图像请求、结构化解析、错误分类；无 ROS 执行工具 |
| viewer / recorder | 独立进程 | 只读订阅，不以 UI 状态触发运动；展示慢时丢展示帧，不阻塞控制 |

观测节点与 daemon 初版只实现一种本地传输：沿用现有 rosbridge 传送轻量状态/候选引用，图像经同机只读缓存或局域 HTTP 读取。先检查当前 transport 能力；不要用 rosbridge 高频搬运原始全分辨率 RGB-D。需要模型的帧才压缩并固化字节。

若同环境可直接共享有界内存队列，可以采用这一更简单方案，并在 architecture.md 记录；不要同时维护多种观测后端。

## 3.2 建议新增文件

下表是实施目录建议，全部是待新建内容。短模块可合并，接口和职责需保持。

| 路径（相对 challenges/03-visual-navigation/） | 职责 |
|---|---|
| README.md | 启动、演示、实际状态及边界 |
| config/visual_nav.yaml | 全部有效配置入口，明确待调参数与硬约束 |
| config/runtime.env.example | 安装路径、模型 ID、Key 环境变量名，不含 Key |
| config/nav2_mppi.yaml | 新 challenge 导航参数；DWB 对照另保存 |
| scene/build_scene.py | 从 SimScene 配方生成环境与导航障碍布局 |
| scene/layouts/*.yaml | 固定布局、种子、成对语义变化 |
| isaac/scene_and_sensors.py | 场景合成、RGB-D、ROS hook、复位 |
| isaac/live_capture.py | 非阻塞展示流、录像队列与帧索引 |
| ros2/observation_node.py | 图像/深度/TF/odom/costmap 同步与快照 |
| navigation/contracts.py | ObservationBundle、Candidate、ModelIntent、DecisionEvent 等统一类型 |
| navigation/geometry.py | 投影、深度落地、footprint/corridor 检查 |
| navigation/candidates.py | 无语义答案的候选生成与拓扑记忆 |
| navigation/luna_worker.py | Responses API 及结构化解析 |
| navigation/visual_session.py | 会话状态、语义承诺、目标更新、取消 |
| navigation/nav2_adapter.py | 规划/FollowPath action 适配与 callback 关联 |
| navigation/events.py | 统一事件写入、订阅、时序与引用 |
| rosclaw/prepare_visual.py | Body/profile、实际能力与 executor 注册配置 |
| viewer/server.py + static/ | 只读本地 viewer、实时流、回放 |
| evaluator/evaluate.py | 独立物理、语义、时序、性能评价 |
| scripts/lab.py | doctor/start/run/stop/replay/evaluate/export 的统一入口 |
| tests/ | 第 9 节规定的关键回归与集成测试 |
| docs/ | architecture、接口、实际配置与结果说明 |

只对旧 daemon 增加必要的 scenario 注册和共用配置能力；可复用函数优先 import，不复制完整旧 challenge 再各自演化。

---

# 4. 数据契约：先定义，再连线

## 4.1 共用时钟与身份

所有重要数据都携带：

- session_id：一次用户任务。
- episode_id：一次独立 reset 后的尝试。
- epoch：每次 reset、时钟倒退或仿真重新初始化递增。
- event_seq：每 episode 内单调递增的本地事件序号。
- sim_time_s：源观测对应的仿真时间。
- host_mono_ns：同一主机单调时钟；不同主机必须有 clock_domain，不能相减。
- recorded_wall_utc：审计用 UTC 时间，不用于 timeout。
- route_version：只有语义通道承诺变化时递增。
- path_seq：局部/全局几何路径更新序号，不能拿它代替 route_version。
- schema_version：首版固定 1，后续变化显式升级。

候选身份/拓扑变化单独使用 candidate_revision；单纯重算同一路径不改变它。纳秒时间通过 JSON 十进制字符串传给浏览器，避免 Number 精度损失。API 网关不能取得精确 sim_step 时，sim_time_quality 标记 last_heartbeat 或 unknown；不得用最近心跳冒充精确采样时间。

ROS 时间戳用于 TF/图像同步；API 超时用 monotonic wall time。既报告观测的 sim age，也报告 wall age。不能简单以平均 RTF 换算一次具体请求的时间。

reset 后旧回答、旧动作 callback、旧图像引用全部失效。暂停时图像再接收不代表传感器又产生了新观测。

## 4.2 ObservationBundle

~~~text
ObservationBundle
  identity: session_id, episode_id, epoch, observation_id
  captured: sim_time_s, host_mono_ns, camera_frame_seq
  rgb: raw_ref, model_input_ref, model_input_sha256, width, height
  camera: optical_frame, K, distortion, calibration_version
  transform: T_odom_camera_at_capture, T_map_odom_at_capture,
             transform_timestamp, localization_revision, validity
  robot: odom_pose, odom_twist, localization_status
  geometry: costmap_version, age, sensor_source
  mission: task_text, current_stage, compact_route_history
  route_version: integer
  candidate_revision: integer
  candidates: Candidate[]
~~~

raw_ref 保存无决策叠图的 RGB。model_input_ref 保存实际送给模型的字节，允许叠加候选编号；二者分别哈希。模型回答产生后的绿线、解释、目标答案不得重新烘焙进入该次模型输入，也不要自动反馈到下一次输入。

候选标号若进入模型图像，必须在发请求前生成并锁定；请求清单声明这一处理。viewer 的显示开关、后续高亮、解释与 GT 层只改变显示，不能改变已提交字节，也不能从 viewer 截屏生成下一轮输入。

深度类型固定并实测：若使用 distance_to_image_plane，按相机 z-depth 处理；不能用同一个公式处理欧氏 range。明确米制、无效值、光学坐标轴。ROS quaternion 统一 xyzw；从 SimScene wxyz 接入时在边界转换并测试。

## 4.3 Candidate

~~~text
Candidate
  candidate_id: 本次观测内不含语义答案的 opaque ID
  persistent_gate_id: 本地几何拓扑 ID（若可关联）
  kind: corridor | frontier | terminal_region
  visible_region_px: 图像中的可见多边形/标号位置
  frame_id: map
  gate_map: 持久门线或门区几何
  corridor_map: 已验证通行区域
  proposed_via_points_map: 几何候选点
  source_geometry: 图像采集时的 odom 几何、TF、localization_revision
  validity: footprint_clearance, costmap_version, observed_extent
  progress_context: 已访问/已失败通道，但不含目标真值
~~~

- 每次通常提供 2–5 个可行候选；没有候选就返回 reobserve/stop 状态。
- 编号不得固定隐含“左边永远正确”；成对测试交换排列检查位置偏置。
- 真实隐藏目标坐标、USD prim 名称、语义 fixture 表只在评估端。
- 允许使用已声明的基础结构地图；新箱垛通过在线传感器进入地图。配置记录 prior-map 来源。
- 投影按图像时刻的 TF，并做遮挡/画面裁剪。不可把后方通道投影成前方可见路线。
- terminal_region 也先由几何模块生成，至少覆盖所有当前可见、可接近的中性停靠区；不能仅提供正确目标。允许声明公开的中性停靠区域几何，不包含颜色/字母到目标的答案映射。Luna 仍只选择已有 ID，evidence_regions 不生成导航目标。

场景导出为两份独立清单：public_scene_manifest（结构地图、几何与中性候选）和 evaluator_manifest（本轮正确目标、语义映射与 GT）。导航进程只加载前者。配对换牌/换色测试需验证：去掉请求 ID/时间戳等易变字段后，公开非图像输入、候选稳定几何 ID 与排序相同，而 RGB 与评估答案变化。

## 4.4 ModelIntent：有限意图与可展示的简短依据

建议结构：

~~~json
{
  "observation_id": "obs-0128",
  "decision": "select_candidate",
  "candidate_id": "c-2",
  "speed_profile": "cautious",
  "reason_code": "FOLLOW_VISIBLE_SIGN",
  "reason_summary": "B 工位箭头指向右侧，选择右侧通道。",
  "evidence_regions": [
    {"bbox_xyxy_norm": [0.61, 0.22, 0.83, 0.38], "label": "B 工位指示牌"}
  ]
}
~~~

decision 枚举：select_candidate、hold、reobserve、goal_reached_hint。candidate_id 允许 null。reason_summary 以简短事实性说明为目标，建议最多 80 个中文字符；evidence_regions 最多 3 个。通过 strict JSON schema 约束，再做业务验证。

不要请求、展示或编造模型内部思维链。reason_summary 是模型给出的短解释，不是正确性证据；evidence_regions 也是模型声称关注的区域，界面必须这样标注。不要画未经模型返回和验证的 attention heatmap，不提供虚构置信度百分比。

模型不能返回开放式 shell/ROS 工具调用，不决定 TF、坐标、TTL、速度上限或任务成功。goal_reached_hint 仅触发独立终点检查。

## 4.5 AcceptedIntent 与通道承诺

执行器验证后生成：

~~~text
AcceptedIntent
  request_id, observation_id, episode_id, epoch
  route_version, accepted_at_sim, accepted_at_mono
  constraint_version, global_constraint_applied, local_constraint_applied
  localization_revision
  selected_gate, allowed_corridor, stop_before_unresolved_gate
  path_horizon, speed_cap
  semantic_lease: 适用空间范围、时间/距离预算、失效条件
  validation_refs: 当前代价地图、TF、footprint 检查
~~~

route_version 只在真正改选通道/目标时变化。同一通道重复回答更新观察记录，不无条件重启导航。

失效原因至少区分：epoch_mismatch、request_superseded、observation_expired、route_changed、candidate_missing、tf_unavailable、corridor_blocked、budget_exhausted、session_stopping、schema_invalid。

## 4.6 统一事件类型

observation_ready → candidates_ready → model_request_started → model_response_received → intent_validated 或 intent_rejected → path_submitted → path_accepted → motion_observed → gate_crossed → goal_verified。

错误和生命周期事件：api_timeout、api_error、sensor_stale、local_slowdown、stop_requested、physical_stop_verified、episode_finished、viewer_gap、recording_drop。

一个 event 必须引用 observation_id/request_id/route_version/path_seq 中适用的字段。模型返回、执行器接受、Nav2 接受、物理实际动作是不同事实，不合并成一个“成功”。

---

# 5. 感知、视觉决策与连续导航实现

## 5.1 相机与目标空间落地

1. 在机器人上新增固定前向 RGB-D 相机，模型视角不受展示镜头移动影响。
2. 初始 RGB 960×540 或 1280×720；选择具体值后锁定 K 与输入缩放规则。深度与 RGB 使用对齐的相机/渲染产物。
3. 用已知平面/标尺测量验证深度单位、K、光学轴与 TF；小幅旋转机器人后投影仍正确。
4. 候选通道由局部地图、可见自由空间、已声明基础地图和访问记忆生成。标牌/颜色识别由 Luna 完成。
5. 几何模块先从当前可见结构和公开的中性停靠区生成全部 terminal_region 候选，并将候选编号投影到图像；Luna 根据标识选择 ID，再用同步深度、当前地图和附近自由地面验证接近区域。解释框不参与目标坐标生成，正确目标表仅给 evaluator。
6. 终点站立区域应在工位前，不把柜体表面深度点直接当底盘目标。
7. 观测缺深度/TF/有效图像时进入重新观察或等待，不发送猜测坐标。

坐标合同：每个点、门区、走廊和 PoseStamped 都有 frame_id。图像落地先使用采集时刻 T_odom_camera，再用同一时刻 T_map_odom 形成 map 中的持久候选；保留原始 odom 几何和转换版本。全局路径与规划请求使用 map；传给 FollowPath 的路径 frame 和 TF 由实际 Nav2 配置验证，绝不把 odom 数值直接改标签当 map。地图中的固定结构不随机器人里程计漂移而移动。

AMCL 出现显著 map→odom 修正、定位丢失或 TF 中断时，递增 localization_revision，使依赖旧定位的草案、候选和 lease 失效，受控停止后以新观测重建并重新验证。位移/转角阈值依据本机定位噪声与最窄通道余量在 P1/P3 冻结；不能沿用旧投影继续穿门，也不能用最新 TF 去冒充历史图像的采集变换。

## 5.2 模型请求

- 默认 model=gpt-6-luna，endpoint 可配置；先用官方 Responses API 做能力探测。
- 对 none/low reasoning 各采样一小批真实导航图像（建议各 10–20 次），比较语义准确性、JSON 有效率和端到端延迟。
- 缓存稳定系统提示与 schema，动态输入只包含当前任务、图像、候选和有限历史。
- 不发送完整聊天历史或视频全部帧。
- 一个在途请求、一个最新待发观测；旧待发帧直接替换，不形成 FIFO 积压。
- 测量采集→发送→完整响应→验证→执行五个时刻；TTFT 不代表可执行决策延迟。
- 处理 completed、refusal、incomplete、解析失败、429、5xx、连接失败与 timeout；不能把半段 JSON 当指令。
- 为每 episode 设置有限请求次数与墙钟时长预算；初值可用 120 次与 180 s，最终按路线长度冻结，超限记录失败。
- SDK 的自动重试和应用层重试不可叠加形成长阻塞；采用一个受会话 deadline 限制的重试策略。失败后取新图，不重发早已过时的观测。
- usage 和响应模型名按实际响应保存；缺 usage 记 unknown，不推测免费或零成本。

## 5.3 异步有效性

从图像到响应期间机器人已经移动。用采集时的 TF 还原候选，再用当前机器人位姿与地图检查：

- 原通道是否仍存在；
- 机器人是否已经越过门区；
- 当前路线是否已改选；
- 车身是否仍能进入候选；
- 回答是否超出本轮阈值与空间预算。

age 的初始调试值可以采用 wall 3 s / sim 3 s，但必须结合实际位移、转角和重新验证。若实际 p95 持续超出预算，优先提前观察、减速、缩短输出或改善输入，而不是无限放宽有效期。

API 等待时只能执行已有有效承诺范围内的路径。遇到未解决分叉，机器人在 stop_before_unresolved_gate 前减速停下。几何导航不得自行越过语义边界替模型做决定。

## 5.4 目标执行与连续路径更新

本次 MVP 固定采用：VisualSessionExecutor 内由一个 Nav2 adapter 异步编排：

1. 暂存新语义承诺，发布带 constraint_version 的约束，等待局部与全局 costmap 已采用的确认；根据该版本的门区/走廊请求 ComputePathThroughPoses（或单点时 ComputePathToPose）；
2. 用 footprint、通道承诺和当前 costmap 验证完整路径；
3. 激活对应 route_version/constraint_version，提交到同一 FollowPath controller；如提交失败，记录已承诺但尚未执行，进入有界恢复或等待，不能显示已经运动；
4. 新路径以 Nav2 支持的 preemption/update 机制接续，更新时不先 cancel 再停车；
5. 旧计算结果与旧 goal 回调按 request/path_seq 丢弃。

这是借鉴 Nav2 PipelineSequence 的规划与控制并行调度，不表示行为完全等价，也不要求为 MVP 新建 C++ BT 插件。先用小场景实测 installed Nav2 和现有 Ros2ActionClient 对 ComputePath/FollowPath 的类型、preemption、反馈与取消支持。固定 controller_id、goal_checker_id 和 progress_checker_id；最多一个规划请求在途和一个最新待提交更新。提交前检查当前机器人能连续接入新 path，不能只检查路径在计算时有效。

直接编排不再自动获得 BT Navigator 的恢复与重规划逻辑；只实现本文规定的事件/周期重规划、有界恢复与终止。不要再并行增加 NavigateThroughPoses 作为第二条运行分支。首个集成实验必须包含：行驶中连续换两次 path、释放一个迟到规划结果、随后取消，确认没有陈旧路径接管、错误终态或残留运动。

本会话不再同时启动另一条 NavigateToPose 任务。FollowPath、路径计算、恢复动作都属于同一 executor 的活动动作集合。

连续通过还需要提前交接：已经获得语义许可的中间 gate 不是当前 path 的停车终点，路径应延伸至门区后的安全交接段。在已有授权范围内，根据剩余路径距离、实测规划/提交时延和制动距离提前生成下一段，并在控制器开始末端制动前完成替换；不能等待当前 FollowPath 成功才请求下一段。尚未解决的语义边界仍是必须停车的边界。P3 专门验证一个“不需要新语义许可的中间 gate”能够无驻留、无末端停车地连续通过。

## 5.5 防止“模型选右边，规划器从左边过去”

仅放一个右侧路点不够：规划器可能绕一圈从另一边到达。

- 候选承诺包含有向门区和允许走廊。
- 规划路径必须按顺序穿过选定 gate；先前进再返回同一点不算正确通过。
- 对当前不允许穿越的分叉门，使用局部和全局均可见的导航约束/keepout 层。
- 每次约束更新携带 constraint_version，确认局部与全局 costmap 都已应用后才提交使用该版本的新路径；在等待确认期间不越过新的约束边界。实现上记录实际更新/订阅确认，不把“mask 已发布”当作“两张地图已采用”。
- 首版可通过实际 costmap 快照核验该版本 mask 在当前覆盖范围内应禁行的单元确已达到要求代价，同时核对新鲜度与 TF；保存所核验的 mask hash、地图 stamp 和结果。若采用带版本的 layer diagnostic，也须证明它在完成更新后才确认。仅收到订阅回调或看到时间戳变化不足以证明语义约束已经应用。
- 等待新约束生效时，旧路径只有在同时符合新旧边界时才能继续；否则在选择线之前受控停止。待生效版本与活动版本分开保存，迟到的确认不能激活已经撤销的版本。
- 约束只来自已经提交的任务意图与已观测几何，不从 evaluator 复制答案。
- Keepout/filter 的膨胀与 footprint 要单独验证，不能假设普通 inflation 自动把所有 filter 扩张。
- 仅验证 path 不足：执行中检查实际 footprint 是否越出承诺走廊。约束失效时停下并请求新决策，不静默改道。
- 通过 gate 的判断使用实际轨迹跨越方向与 footprint 通过条件，不能只靠“距离路点较近”删除该点。

## 5.6 局部控制与速度

先复现 DWB 基线，再做 MPPI 对照，保持机器人、传感器、地图和速度上限一致。

建议调试起点（不是最终验收事实）：

| 参数 | 起点 | 调整依据 |
|---|---:|---|
| physics | 60 Hz | 接触/车轮稳定性和本机负载 |
| RGB-D | 15–20 Hz | 实际 unique frame 频率和负载 |
| local costmap | 10–20 Hz | 障碍新鲜度与 CPU |
| controller | 20–30 Hz | p95/p99 周期耗时、deadline miss |
| smoother | 30–50 Hz | odom 新鲜度、速度跳变 |
| MPPI 时域 | 约 2–3 s | v_max×时域在地图有效前向范围内，留 footprint 余量 |
| 巡航 | 0.6–0.9 m/s | 实测制动、转弯、clearance |
| 弯道 | 0.35–0.6 m/s | 曲率、角速度及侧向加速度 |
| 窄道/终点 | 0.2–0.4 m/s | footprint、定位与制动裕量 |
| 后续直道提速 | 1.0–1.2 m/s | 前述条件都通过才增加 |

使用真实 footprint，不能通过缩小车身或关碰撞来让窄门通过。controller、smoother、collision monitor 的速度/加速度限制一致；smoother 仅限速/加速度不自动保证 jerk 有界，必要时补充受控斜坡，并保留紧急停车优先级。

几何安全预算使用 d_required = v_sim*T_local,sim + v_sim²/(2*a_brake,sim) + margin。语义路口再加 v_sim*r_max*L_API,wall，其中 r_max 为运行模式允许的仿真/墙钟比上界。正式实时模式优先限制不快于实时；不能直接把 wall 秒当 sim 秒。a_brake 用实测最差可接受制动能力，不能抄配置值当实测值。样本不足时不把少量请求的 p95 当稳定估计；显示样本数并使用保守预算，超时另计。

## 5.7 恢复与失败

进展用剩余可行路径/已通过 gate 评估，不能只看与最终目标欧氏距离。正确绕行可能暂时远离目标。

初版恢复顺序：重新观察 → 确认当前路径失效 → 标记当前边失败 → 返回最近已验证路口 → 请求新选择。倒车只在相应区域有可靠感知与 swept footprint 检查时启用。

API 超时/不可用后：只能在仍然有效、持续通过本地几何检查的已承诺路径内有限续行。建议故障确认后的总运动预算初值 1.0 m / 2.0 sim s，并有 3.0 wall s 的独立 watchdog；取最早边界，且包括制动距离和制动时间，不是走满预算后才刹车。还必须停在未决 gate、已授权走廊末端和可观测安全区域边界之前。故障发生时制动余量不足，立即制动并记失败，不能扩预算。DEGRADED/HOLD 未获得新的有效决策不能转 SUCCEEDED，即使旧 Nav2 action 随后返回成功。传感器失效直接进入本地受控停止。

超时类别必须区分：单次 API request deadline 进入 DEGRADED，按前述有限预算减速并在实际停稳后进入 HOLD；它本身不结束会话，取得新图像上的有效决策和路径后可以恢复。HOLD 仍受 mission 总墙钟预算约束。用户取消、mission 总 deadline、不可恢复的动作超时、碰撞和无法恢复的 Nav2 abort 进入终止性 STOPPING：禁止新意图，取消规划与所有活动 FollowPath/恢复动作，锁定输出停驶，独立验证实际速度。STOPPING 完成后进入 CANCELED/FAILED 等终态。仿真暂停期间只能记录 stop_pending；恢复后先停驶验证，再允许新会话。

STOPPING 先使当前 revision 失效，再发 cancel；若 cancel 时 send_goal 的 accepted 回调尚未回来，迟到 accepted 后仍须立即取消，不能重新激活运动。暂停撤销在途请求与语义 lease；恢复需要新观测与新授权。除 ROS sim timer 外，保留独立 wall watchdog 处理 /clock 停滞。

最终成功建议同时满足：正确语义工位、目标位置误差 ≤0.30 m、必要朝向达标、线速度 <0.03 m/s 且角速度 <0.05 rad/s 连续 2 sim s、零非地面接触、记录完整、没有遗留活动动作。这些是本项目建议验收值，应在首次冻结前确认并记录，不能看到结果后放宽。

评估器还必须验证规定的有向 gate 序列、零禁止 gate 穿越、无未解决语义故障。FollowPath 返回成功只完成当前几何路径，不能独立使 mission 成功。

---

# 6. 决策可视化：产品规格

## 6.1 展示的是五个可复核环节

“当时看到什么 → 当时有哪些候选 → 模型提出什么 → 执行器接受/拒绝什么 → 实际发生什么”。

不把“模型已返回”显示成“机器人已执行”；不把本地安全减速归因于 Luna。可视化不是聊天记录窗口，也不是伪造的内部思维动画。

## 6.2 两个视图，共用一套事件

### 展示模式（默认，16:9）

- 主画面约 65–70%：真实 Isaac 第三人称跟随或固定斜俯视。
- 侧栏约 30–35%：本次决策使用的冻结第一视角、候选编号、模型短说明、接受/拒绝及当前执行状态。
- 底部：简洁事件时间轴和当前速度/帧龄/RTF。
- 一个时刻只突出当前相关决策，避免大段日志遮挡机器人。

### 调试模式

增加已声明地图/局部costmap、当前规划路径与实际轨迹、传感器 age、API 时延、控制状态、完整事件详情。GT 评估层独立开关，打开时常驻“评估真值，仅用于复核”；控制器观察地图不得用这个层冒充。

可选导出 Rerun 的 RGB、投影、坐标系和多时间轴，用于工程分析。Foxglove 借鉴图像+2D/3D标注的组织方法；首版不要求另部署完整平台。

## 6.3 冻结决策帧与当前画面必须分开

主画面显示现在，侧栏显示模型实际使用的那一帧。侧栏至少标：

- “模型输入帧”；
- observation_id 与采集 sim_time；
- 回答时 age 和当前 age；
- 输入含候选标号还是 raw；
- 与当前机器人位置有多少位移/转角（调试模式）。

点一次时间轴事件，图像、候选、模型输出、校验和路径一起定位。缺原帧时显示“原始帧缺失”，不能用最近帧冒充。

## 6.4 候选与路径的视觉语义

| 对象 | 图形规则 | 数据来源 |
|---|---|---|
| 几何可行候选 | 青色虚线 + 编号 | CandidateSet |
| 模型本次提议 | 琥珀色轮廓 + “提议” | ModelIntent |
| 本地已接受走廊/路径 | 绿色实线 + revision | AcceptedIntent / Nav2 接受 |
| 估计行驶轨迹 | 白色或蓝色细线，标明“里程计估计” | odom；独立 PhysX 轨迹另标“仿真实际轨迹”，二者不混用 |
| 被拒绝提议 | 红色虚线 + 原因 | intent_rejected |
| 本地限速/停车 | 明确“本地控制器”标签 | collision monitor / session |
| 无数据/待定 | 灰色虚线 + 文字 | 数据缺失状态 |

颜色同时搭配线型、文字和图标，不能只靠颜色。模型给出的 evidence box 标“模型指出的图像区域”；投影路径标“几何投影”，两者不混用。

## 6.5 四个必须实现的界面状态

1. 请求中：显示新请求对应帧；同时显示当前仍在执行的上一条有效路线。没有上一条有效路线则等待。
2. 已采纳：分别显示执行器已接受、Nav2是否已接受、是否已观察到运动。收到 callback 之前不提前打勾。
3. 旧回答拒绝：保留模型提议、原始帧和拒绝原因；当前有效路线不被覆盖。
4. 超时减速：明确减速来自本地策略；显示剩余已批准路径/预算。不能写“Luna 决定减速”。

配套 HTML 原型用合成示意图展示这四态，不连 API，不给实际机器人命令。真实 viewer 不能自动切换 mock 数据顶替断流。

## 6.6 事件与视频同步

事件同时保存 source time 和 received time。视频索引每帧保存 capture_sim_time、capture_mono_ns、frame_seq、PTS、camera_id、width/height。

- 实时/实时回放按 received time 揭示“当时系统已经知道什么”。
- 模型提议只能在 model_response_received 之后出现。
- 在原始输入帧上回看后来决策，可以，但界面明确“事后查看该帧的决策”，不能伪装成当时实时已知道。
- replay seek 由状态 reducer 从事件重建，清除前一次选中的框线和解释。
- 暂停期间 wall 时间继续推进，sim 时间不动；两个时间轴都保留。
- 事件丢失发 gap；无法恢复时显示“记录不完整”，该次证据资格失败。

## 6.7 UI 性能与实现边界

建议用轻量本地 server + SSE/WebSocket 事件，图像独立通道；不要把每帧 JPEG base64 塞进事件 JSONL。

实时状态可 10–20 Hz 聚合发布；缩略图 5–10 fps；决策事件不丢弃。视频编码独立队列。展示帧队列满时丢旧帧并计数；关键控制/决策事件写入失效时标记 evidence_incomplete，必要时有序停止，不静默丢证据。

viewer 默认只读。没有“暂停仿真”按钮；回放暂停只暂停回放。可以保留跳转/筛选/播放功能，不提供第二个机器人控制台。

---

# 7. 场景与拍摄规格

## 7.1 三类固定布局

| 布局 | 主要结构 | 目的 |
|---|---|---|
| L1 流畅绕行 | 错位箱垛、宽弯、一个窄门 | 调速度、曲率与轨迹连续性 |
| L2 语义分叉 | 两路几何对称/可通行，标牌方向可交换，终点颜色/字母可交换 | 证明视觉改变路线与目标 |
| L3 有界恢复 | 遮挡后封路、U形或死胡同、可返回路口 | 证明能识别进展失败并恢复 |

主展示可串联 L1/L2 的特点；L3 首先用于验证，成熟后再进入宣传片。

## 7.2 布局与资产规则

- 主通道初值 2–3 m；窄道初值约 1.1–1.4 m，最终按真实 footprint、padding、转弯扫掠和定位余量测量。
- 基础地图包含墙/永久结构；用于“未知静态障碍”实验的箱垛不得提前烘焙进先验地图。
- 标牌文本/颜色交换只改外观，不改碰撞几何，便于成对因果比较。
- 标牌在决策线前有足够可读距离，满足测得的 API 延迟预算。
- 动态障碍、人群和真机迁移留给后续任务，本轮以静态障碍建立完整闭环。
- 材质采用灰白、深灰、少量蓝色设备、黄色标线；照明清晰，避免高反光地面、严重眩光和过暗标牌。
- 小物件在工作台或周边，导航通道保持视觉可解释。
- 导入视觉体与简化静态碰撞代理分别检查；不要让 invisible proxy 突出视觉边界造成“看起来没撞却撞了”。

## 7.3 摄像机与视频

初版一个主要展示视角 + 一个前向传感器；性能达标后增加第二展示视角。

- 跟随摄像机每 render callback 更新，平滑位置/朝向；不能将其变成输入相机。
- 保留固定斜俯视用于检查连续路线，减少跟随镜头掩盖左右摆动的问题。
- 实时验证录像按 wall 时间 1× 保存，保留 unique frame 与时间索引；编码 30 fps 不等于采集 30 fps。
- 导航过程不调用 SimScene 的 paused_render_barrier。
- 如离线重渲染已记录状态以改善画质，明确 replay-render，不计入闭环实时指标。复杂接触的 replay 可能不等同重新物理运行，也必须说明。
- 影片剪辑显式记录源 episode、起止时间、倍速；一个连续任务不能拼接不同运行后标为“一镜到底”。

---

# 8. 参考路线的具体选择

| 需求 | 首选借鉴 | 实施选择 |
|---|---|---|
| 机器人底座/执行所有权 | robotics-lab daemon、PointExecutor、Body | 新 session 扩展，最少修改共用入口 |
| 标定与 RGB-D | Isaac 6.2 Camera Sensors / ROS2 camera | 固定前向相机、真实缓冲区与时间戳 |
| 连续规划与跟踪 | Nav2 PipelineSequence、ComputePathThroughPoses、FollowPath | 一个 session 的非阻塞规划/跟踪管线 |
| 局部轨迹优化 | Nav2 Jazzy MPPI | 与 DWB 同条件对照，按实测选择 |
| 速度平滑与近障限制 | Nav2 Velocity Smoother、Collision Monitor | 监控位于最终速度链末端 |
| 视觉意图 | OpenAI Responses + Structured Outputs | 候选 ID + 短说明，业务校验在本地 |
| 场景 | SimScene scene/export/measure | 环境离线生成，单 Isaac 运行 |
| 决策可视化 | Foxglove Image annotations | 同帧标注、标定投影、来源区分 |
| 时间同步与工程回放 | Rerun Timelines | 采用多时间轴思想，可选导出，不做硬依赖 |

这些参考解决不同问题。不要直接引入 SimScene arm follower、另一个完整 Agent framework 或额外底盘控制器作为导航捷径。

---

# 9. 实施任务与阶段门

每个任务完成后记录：变更文件、实际运行命令、证据目录、结果、剩余问题和提交 SHA。失败先定位本阶段根因，不能改判据把失败改成通过。

## P0 — 环境、基线与可重复启动

**Files:** 新建 config/visual_nav.yaml、config/runtime.env.example、scripts/lab.py；按需窄改旧 daemon 的 scenario 注册。
**Consumes:** 本地仓库、Isaac 路径和原基线。
**Produces:** doctor() → EnvironmentReport；start() → RunHandle；stop() → StopReport。

- [ ] 读取本地指令和工作树，建立隔离分支，记录两个仓库实际 SHA。
- [ ] 探测 CPU/GPU、Ubuntu、Isaac VERSION、Python/ROS/扩展、资产根与可用模型环境变量名；日志脱敏。
- [ ] 明确容器或宿主 ROS 路线；新配置不强制 ARM64，不混用旧 6.1 workspace 与新资产。
- [ ] 启动单 Isaac + Nav2，确认 /clock、TF、odom、scan/pointcloud；启动期间不发目标。
- [ ] 验证 readiness 失败能清理本次子进程，不杀用户其他 Isaac 或 ROS 任务。
- [ ] 记录可复现启动/停止命令并提交。

**Gate:** 新入口能启动、确认就绪、干净停止；错误路径能解释；未提供 Key 不阻塞此阶段。

## P1 — 真正的 RGB-D 观测

**Files:** isaac/scene_and_sensors.py、ros2/observation_node.py、navigation/contracts.py、navigation/geometry.py。
**Consumes:** RunHandle / robot inventory。
**Produces:** capture_observation() → ObservationBundle；project_candidates(obs, candidates) → overlay metadata。

- [ ] 先写 epoch/坐标边界的必要测试；确认旧 epoch、缺 TF、NaN depth 被拒绝。
- [ ] 新增前向 RGB-D，与展示相机隔离，验证实际 unique frame 序列。
- [ ] 记录 K、深度类型、图像与 TF 的时间关系、单位和 quaternion 约定。
- [ ] 用已知几何点验证像素↔空间投影；转弯后仍一致；无效深度不生成目标。
- [ ] 验证 map/odom 坐标标签与采集时刻 TF；注入定位跳变，旧候选与 lease 必须失效。
- [ ] 暂停/恢复/reset 测试：暂停不伪造新帧；reset 后旧包无效。
- [ ] 保留关键帧与哈希，整理 sensor_contract.json，提交。

**Gate:** 图像确来自当前运行，RGB-D 与标定/TF对应；传感器缺失明确失败。

## P2 — 场景、候选与可通行约束

**Files:** scene/build_scene.py、scene/layouts/、navigation/candidates.py、geometry.py。
**Consumes:** ObservationBundle、声明的基础地图。
**Produces:** build_candidates(obs) → CandidateSet；validate_corridor(candidate, current_geometry) → ValidationResult。

- [ ] 用简单灰盒先建立 L1/L2/L3；静态障碍显式碰撞，检查 visible/collision/sensor 一致。
- [ ] 实现几何候选和候选投影；目标真值仅给 evaluator。
- [ ] 添加有向 gate、承诺走廊及被禁止门区的约束。
- [ ] 写 test_committed_gate_cannot_be_bypassed：两侧都可达同一目标时仍不得越过未选门区。
- [ ] 执行路径和实际 footprint 都检查通道承诺，验证约束更新生效时间。
- [ ] 固定开发 seed 与未查看的验收 seed 清单；记录实际生成配置，提交。

**Gate:** 候选几何有效、无语义真值泄漏、交换标牌不改变几何。

## P3 — 持续导航会话与终止

**Files:** navigation/visual_session.py、nav2_adapter.py、rosclaw/prepare_visual.py、config/nav2_mppi.yaml。
**Consumes:** CandidateSet、测试用确定性意图、Body binding。
**Produces:** VisualSessionExecutor.run(mission) → SessionResult；commit(intent) → CommitResult；cancel(reason) → StopReport。

- [ ] 先写旧 callback、重复完成、取消正在 FollowPath、规划结果迟到的测试。
- [ ] 从现有 daemon 注册新能力，确保只有一个 action owner。
- [ ] 接通本文固定的 ComputePath→路径校验→FollowPath 管线，验证真实 preemption 和 callback 行为。
- [ ] 用确定性测试意图完成至少两次途中路线更新；中间不触发固定两秒驻留。
- [ ] 路径提前延伸与交接；已授权中间 gate 通过时不因短路径终点而停车。
- [ ] 校验 path_seq 与 route_version 分离，旧 Nav2 成功回调不能结束新路径。
- [ ] DWB 与 MPPI 同条件比较；修正 collision monitor 的无效阈值，实际接近障碍验证减速/停车。
- [ ] 取消、超时、Nav2 abort、暂停后恢复分别验证无残留运动。
- [ ] 最终目标执行独立到达与低速驻留；运行测试报告并提交。

**Gate:** 一条完整路线连续执行，正常与异常退出都能证明运动终止。测试意图记录为 controller-test，不算 Luna 结果。

## P4 — Luna 接入与语义闭环

**Files:** navigation/luna_worker.py、visual_session.py、prompts/luna_navigation.txt、contracts.py。
**Consumes:** 实际 ObservationBundle、Key 与模型配置。
**Produces:** infer(obs) → ModelIntent | ModelFailure；validate_intent(intent, current) → AcceptedIntent | Rejection。

- [ ] 验证 model/gateway 的真实图像与 strict schema 支持，记录返回模型名；不支持时明确失败。
- [ ] 使用真实帧测 none/low 小样本，选择并冻结一档；记录完整响应延迟。
- [ ] 实现单在途与 latest-only，处理错误、refusal、incomplete 和超时。
- [ ] 验证模型实际收到的图像字节哈希、对应候选与原始响应都可追溯。
- [ ] 写 test_stale_candidate_never_commits 与 test_epoch_reset_rejects_old_result；模拟迟到回答，不允许污染当前任务。
- [ ] 完成一次标牌交换与一次终点视觉标识交换，检验路线/到达变化。
- [ ] 检查从未把 USD 名称/隐藏坐标提供给 Luna 或候选排序器。
- [ ] 提交 API 采样、最小闭环结果与失败记录。

**Gate:** 实际 Luna 看图改变任务相关行为；API 等待期间物理继续推进；无 Key 或错误只报告 blocked，不用 mock 代替。

## P5 — 可视化与可回放事件

**Files:** navigation/events.py、viewer/server.py、viewer/static/。
**Consumes:** 实际事件、图像引用、路径、速度。
**Produces:** /events、/frames/<id>、/episodes/<id>（具体路由在实现中固定）；只读 live/replay viewer。

- [ ] 先按配套原型实现四态，mock 只在独立 demo 模式，常驻“示意数据”。
- [ ] 接入真实事件；模型帧固定、主画面实时，两者显式区分。
- [ ] 分离 model received / intent accepted / Nav2 accepted / motion observed。
- [ ] 当前路线上叠加实际执行信息，模型建议被拒绝时不替换已接受路径。
- [ ] 实现 replay reducer 和 seek；test_replay_never_shows_future_decision。
- [ ] 断流显示 unknown/stale，不用上次速度装成实时，不自动切 mock。
- [ ] 用慢消费者验证 UI 不阻塞控制；截图人工检查文字、线条、来源与状态是否准确。
- [ ] 提交 viewer 与一份真实 episode 回放。

**Gate:** 任意一条决策能跳回真实输入、候选、响应、校验和动作；四态全部来自真实事件或显式测试事件。

## P6 — 真实录像与场景美术

**Files:** isaac/live_capture.py、scene 配方、video/export.py、viewer 展示模式。
**Consumes:** 通过 P3/P4 的系统、P5 事件。
**Produces:** raw video + frame_index + event log + edited film。

- [ ] 用独立有界队列与编码进程实现录像；不得暂停物理等待帧。
- [ ] 跟随镜头按渲染更新，与低频日志采样分离。
- [ ] 在相同场景对比无录像/单视角/展示界面开启的 RTF 和控制周期，定位真实负载。
- [ ] 按本机性能添加有限 SimScene 资产与灯光，每次验证碰撞与传感器一致性。
- [ ] 输出一段 wall-time 1× 原始录像；验证 unique frame rate、PTS 与时间索引。
- [ ] 制作展示片，记录所有剪辑段、倍速、episode 来源和断点。
- [ ] 人工观看头/中/尾及所有决策/拒绝时刻，确认画面不是提前显示未来答案。

**Gate:** 录像可解码、时序可核验、控制不依赖录屏；是否实时由实测决定。

## P7 — 冻结、对照与交付

**Files:** evaluator/evaluate.py、runs/ledger、docs/results.md、handoff README。
**Consumes:** 冻结源码、配置、scene 与预先登记的 episode 清单。
**Produces:** results.json/csv、完整实施报告、主展示/语义对照/超时停止视频和可回放包。

- [ ] 冻结源码、配置、资产清单、速度/预算、prompt/schema、模型名；后续改动另编号。
- [ ] 执行第 10 节精简验收；所有尝试进入 ledger。
- [ ] 对失败逐个给出根因归属与证据；修复后保留原失败，重新登记批次。
- [ ] 运行离线 evaluator 复核原始日志，输出指标与缺失项。
- [ ] 交付真实控制演示、语义成对比较、一次超时受控停止，以及只读回放。
- [ ] 本地提交代码，报告 branch/SHA；推送/PR 按用户本地已授权方式处理。

**Gate:** 用户可按 README 复现；状态、视频与统计相互一致；没有未说明的 mock、加速或真值控制。

---

# 10. 精简验收设计与统计口径

## 10.1 开发与验收分开

开发集可反复使用：DEV-01 开阔+S弯；DEV-02 标牌分叉；DEV-03 目标换位；DEV-04 遮挡封路/恢复。开发结果不写成泛化成功率。

首次冻结后的最小验收建议 6 个未调参 episode：

- H01/H02：同一几何、相反标牌方向的一对；
- H03/H04：同一几何、目标标识互换的一对；
- H05：新箱垛布局与窄门；
- H06：遮挡后的封路/有界恢复。

每个 episode 的 spawn、障碍、语义、timeout 与完成条件提前写入清单。六次全部报告原始计数，样本小不推断生产可靠性。首版 demo 达到 6/6 后可按需求扩展到 3 布局×5次，不在初始任务中自动无限扩量。

同样 6 个 episode 先用 oracle 正确候选验证控制层，再用 hybrid 正式运行，共 12 次正常计分。Oracle 有特权信息，单列报告。另做 3 个故障集成用例：API 超时、旧回答/epoch reset、会话失败但 FollowPath 活动。故障实验不与普通成功率混算。

## 10.2 对照设计

- DWB vs MPPI：同一组已知坐标目标、传感器、速度与地图，用来调连续运动。
- Oracle target + Nav2：给予正确目标/通道，仅作为几何运动上限参考；不宣称与视觉方法信息公平。
- Luna + 相同 Nav2：主方案。
- 无图或交换图像的受控消融：相同几何候选、任务和历史，只改变视觉信息；解释为视觉依赖检查。
- 标牌/目标视觉交换：两个分支都几何可行，观察实际路线/目标改变，是首要展示证据。

不要因 Nav2-only 没有语义目标输入而把失败解释成“传统导航能力差”。

## 10.3 硬门与优化目标分开

| 类别 | 判据 |
|---|---|
| 物理硬门 | 零非地面接触；没有 teleport；真实 footprint；接触监控完整 |
| 语义硬门 | Oracle 6/6、Hybrid 6/6；选定门区、正确目标满足预注册任务；被禁止通道不能被 planner 绕开约束；失败后另登记修复批次 |
| 生命周期硬门 | 一次 episode 一个终态；停止有物理确认；旧结果无效 |
| 证据硬门 | 输入/输出/执行关联完整；缺失关键事件不能算 PASS |
| 实时资格 | 持续运行中未暂停等模型；同时报告 RTF 分布与 wall 控制消息到达 |
| 流畅优化目标 | 减少无意义停车、角速度高频反向与速度跳变，不靠放宽碰撞门槛 |
| 展示目标 | 1080p、实际采集接近 30fps、文字可读；达不到时报告实际值 |

建议“实时展示通过”的本项目门槛：完整主任务 RTF 中位数 ≥0.95，5 秒窗口 RTF 的 p5 ≥0.85；UI/录制带来的 RTF 降幅相对同配置无展示对照 ≤10%。这是工程目标，不是行业标准；冻结前可按硬件调整并明确标签，运行后不得隐藏低 RTF。

## 10.4 指标定义

- success_rate：成功 episode / 预登记已启动 episode，缺记录不能删除分母。
- startup_success_rate：启动成功 / 全部启动尝试；与任务成功率分开报告。
- contact_count：去重接触事件数；ground 接触不计，其他均计。
- min_clearance：完整机器人 footprint 到障碍的最小距离，声明来源、采样频率与误差；有限采样不声称连续扫掠的数学证明。
- task_wall_s / task_sim_s：从用户任务被接受到终态，不含预热；终点稳定停留包括在总时间。
- moving_avg_speed：实际行驶路程 / 运动时间；同时给 total_avg_speed，避免去掉等待后显得很快。
- unexpected_stop_s：非最终驻留、非明确安全/语义等待的低速时间；类别由事件定义。
- smoothness：速度/角速度变化量与加速度、角加速度统计；统一重采样和滤波，不用噪声微分伪造 jerk 对比。
- api_latency：发送到完整可解析响应；另报 observation_to_commit 与失效拒绝率。
- semantic_correct：门区正确率与目标正确率分别算，不能只看最终距离。
- actual_capture_fps：unique frame 数/墙钟时长，同时给间隔 p95 与最大 gap；输出编码 fps 另列。
- RTF：各测量窗口 Δsim/Δwall，报告中位数、p5、最差窗口。
- intervention_count：人工导航、reset、参数热改、直接改位姿等单列；人工介入后不计自主成功。
- model_usage/cost：实际 API usage 与价格表日期/endpoint，未知不填零。

---

# 11. 给本地 Codex 的执行顺序与交付清单

## 11.1 开始工作时

1. 读取本文及配套原型；将计划放入仓库 docs/superpowers/plans/2026-11-21-luna-navigation.md。
2. 用 P0 探测表说明环境差异，最多提出真正阻断的问题；其余按本计划默认选择推进。
3. 按 P0→P1→P2/P3→P4→P5→P6→P7 执行。P5 的 mock 界面可与早期底座并行，但真实接线必须等事件契约稳定。
4. 优先给出“单分叉真实 Luna + 连续导航”的小闭环视频，再扩展资产数量。
5. 每阶段一次可复核提交，记录改动和结果。不要只写“代码已完成”，要给实际运行证据。
6. 若配方/帧率/路径更新方案需要改变，记录原因与影响；不擅自把闭环改为动画或把模型改为固定规则。

## 11.2 必交付文件

| 交付 | 内容 |
|---|---|
| README.md | 实际启动、配置、停止、复位、运行、回放命令 |
| ENVIRONMENT_REPORT.md/json | 本机版本、架构、ROS/模型探测、资产与扩展来源 |
| ARCHITECTURE.md | 实际进程、消息、运动所有权和停止链 |
| SENSOR_CONTRACT.json | topic/frame/K/深度单位/时间同步/图像来源 |
| effective_config.yaml | 本轮最终参数，含所有覆盖值 |
| run_manifest.json | SHA、场景、模型、prompt/schema、资产、seed 与模式 |
| attempts.jsonl | 全部启动与任务尝试，不覆盖 |
| events.jsonl | 每轮事件与 ID 关联 |
| model_inputs/ + model_responses/ | 真正送入的关键帧、哈希、脱敏输出与 usage |
| trajectory + contacts | 独立物理轨迹和接触；声明采样 |
| raw_video + frame_index | 原始真实时间录像及逐帧映射 |
| decision_replay | 可离线打开的回放包，缺图/缺事件明确提示 |
| RESULTS.md/json/csv | 全部通过/失败/缺失、性能与语义对照 |
| DEMO_MAIN.mp4 | 主要展示片，倍速和剪辑可追溯 |
| DEMO_SEMANTIC_PAIR.mp4 | 标牌/目标交换成对视频 |
| DEMO_TIMEOUT.mp4 | 一次本地受控减速/停止；归因准确 |
| HANDOFF.md | 本地路径、branch/SHA、已完成与未完成、复现方法 |

这些是交付内容，允许合理合并短文件。产物数量不是目标。

---

# 12. 参考资料：具体读什么、借鉴什么

以下均为作者/维护方的一手来源。仓库链接固定到本次读过的版本；在线文档在 2026-11-21 核查，执行时再确认与安装版本一致。

## A. Robotics Lab

- R01 [仓库说明](https://github.com/ros-claw/rosclaw-robotics-lab/blob/545830cbd2ea8fd20fb579edb113822f7d3c9841/README.md)：看现有物理证据、RGB display-only、旧环境和复现范围。用于界定底座能力。
- R02 [setup.sh](https://github.com/ros-claw/rosclaw-robotics-lab/blob/545830cbd2ea8fd20fb579edb113822f7d3c9841/challenges/01-isaac-warehouse-patrol/scripts/setup.sh)：读 ARM64、workspace 和 runtime pin，识别 P0 必改的环境绑定。
- R03 [asset_resolver.py](https://github.com/ros-claw/rosclaw-robotics-lab/blob/545830cbd2ea8fd20fb579edb113822f7d3c9841/challenges/01-isaac-warehouse-patrol/isaac/asset_resolver.py)：借鉴安装包区域资产解析，处理 6.1 根路径限制。
- R04 [daemon.py](https://github.com/ros-claw/rosclaw-robotics-lab/blob/545830cbd2ea8fd20fb579edb113822f7d3c9841/challenges/01-isaac-warehouse-patrol/rosclaw/daemon.py)：读 executor 注册、Body/transport 生命周期，作为 session 接入位置。
- R05 [point_executor.py](https://github.com/ros-claw/rosclaw-robotics-lab/blob/545830cbd2ea8fd20fb579edb113822f7d3c9841/challenges/01-isaac-warehouse-patrol/rosclaw/point_executor.py)：读动作锁、goal ID、cancel、freshness、独立终止；明确每站 dwell 不能照搬。
- R06 [prepare.py](https://github.com/ros-claw/rosclaw-robotics-lab/blob/545830cbd2ea8fd20fb579edb113822f7d3c9841/challenges/01-isaac-warehouse-patrol/rosclaw/prepare.py)：借鉴 inventory 编译 Body/profile，生成新视觉能力。
- R07 [semantic_executor.py](https://github.com/ros-claw/rosclaw-robotics-lab/blob/545830cbd2ea8fd20fb579edb113822f7d3c9841/challenges/02-semantic-inspection/semantic_executor.py)：借鉴候选 ID、freshness 与路径验证，注意现有语义几何的真值来源。
- R08 [loading_executor.py](https://github.com/ros-claw/rosclaw-robotics-lab/blob/545830cbd2ea8fd20fb579edb113822f7d3c9841/challenges/02-semantic-inspection/loading_executor.py)：借鉴路径 preview 和可追溯拒绝，不沿用专用检查任务。
- R09 [loading_scene.py](https://github.com/ros-claw/rosclaw-robotics-lab/blob/545830cbd2ea8fd20fb579edb113822f7d3c9841/challenges/02-semantic-inspection/loading_scene.py)：静态 fixture、碰撞体和初始化布局。
- R10 [baseline_observer.py](https://github.com/ros-claw/rosclaw-robotics-lab/blob/545830cbd2ea8fd20fb579edb113822f7d3c9841/challenges/01-isaac-warehouse-patrol/isaac/baseline_observer.py)：真实 PhysX 观察、接触、相机禁用和旧采集瓶颈。
- R11 [camera_setup.py](https://github.com/ros-claw/rosclaw-robotics-lab/blob/545830cbd2ea8fd20fb579edb113822f7d3c9841/challenges/01-isaac-warehouse-patrol/isaac/camera_setup.py)：构图与session layer，只借鉴镜头，不当作现成传感器。
- R12 [patrol_navigation_params.yaml](https://github.com/ros-claw/rosclaw-robotics-lab/blob/545830cbd2ea8fd20fb579edb113822f7d3c9841/challenges/01-isaac-warehouse-patrol/config/patrol_navigation_params.yaml)：原 DWB、footprint、topic、costmap 和异常 min_points。
- R13 [cancel_navigation.py](https://github.com/ros-claw/rosclaw-robotics-lab/blob/545830cbd2ea8fd20fb579edb113822f7d3c9841/challenges/01-isaac-warehouse-patrol/ros2/cancel_navigation.py)：多 action 取消与停止集成参考。
- R14 [最终实施报告](https://github.com/ros-claw/rosclaw-robotics-lab/blob/545830cbd2ea8fd20fb579edb113822f7d3c9841/FINAL_IMPLEMENTATION_REPORT.md)：分清采集 fps、编码 fps、RTF 与旧实验范围。

## B. SimScene

- R15 [README](https://github.com/ros-claw/simscene/blob/3660dc1b37610d7b171934dd86ff67df01107d50/README.md)：总体资产→场景→物理→视频工作流。
- R16 [mcp_server.py](https://github.com/ros-claw/simscene/blob/3660dc1b37610d7b171934dd86ff67df01107d50/src/simscene/mcp_server.py)：调用真实支持的 scene 工具，不发明现成导航 API。
- R17 [usd_export.py](https://github.com/ros-claw/simscene/blob/3660dc1b37610d7b171934dd86ff67df01107d50/src/simscene/usd_export.py)：碰撞默认值、visual_only 和 USD 组合边界。
- R18 [asset-catalog.md](https://github.com/ros-claw/simscene/blob/3660dc1b37610d7b171934dd86ff67df01107d50/docs/asset-catalog.md)：锁定资产、准备指纹、GLB runtime probe 与实际 6.2 RC 验证范围。
- R19 [build-scene](https://github.com/ros-claw/simscene/blob/3660dc1b37610d7b171934dd86ff67df01107d50/skills/build-scene/SKILL.md)：测量布局、真实图像复核。
- R20 [camera_frames.py](https://github.com/ros-claw/simscene/blob/3660dc1b37610d7b171934dd86ff67df01107d50/src/simscene/worker/camera_frames.py)：识别 paused_render_barrier，实时运行不能直接套用。
- R21 [deliver-scene-video](https://github.com/ros-claw/simscene/blob/3660dc1b37610d7b171934dd86ff67df01107d50/skills/deliver-scene-video/SKILL.md)：借鉴镜头计划、帧来源、倍速与交付检查。

## C. Isaac 与 Nav2 官方文档

- R22 [Isaac 6.2 ROS 安装](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/installation/install_ros.html)：Ubuntu/ROS/Python 环境匹配；不要混发行版库。
- R23 [Isaac 6.2 ROS Navigation](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/ros2_tutorials/robot_control/tutorial_ros2_navigation.html)：Nova Carter、地图、Nav2 启动与官方接口。
- R24 [Isaac 6.2 Camera Sensors](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/sensors/isaacsim_sensors_camera.html)：CameraSensor、rgb、distance_to_image_plane、相机频率与渲染产物。
- R25 [ROS2 时钟与频率 FAQ](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/ros2_tutorials/help/ros2_faq.html)：sim/wall、发布频率与 RTF 的区别。
- R26 [MPPI Jazzy](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/)：预测时域、motion model、critics、完整 footprint 与地图尺寸匹配。
- R27 [Nav2 Behavior Trees](https://docs.nav2.org/jazzy/getting_started/nav2_behavior_trees/)：PipelineSequence 与持续规划/执行关系。
- R28 [Navigate Through Poses](https://docs.nav2.org/jazzy/getting_started/nav2_behavior_trees/trees/nav_through_poses_recovery/)：多通道点路径更新与 passed-goal 行为；不把 waypoint 当强制停靠点。
- R29 [Velocity Smoother](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/configuring_velocity_smoother/)：速度/加速度限制、open/closed loop 与 odom 要求。
- R30 [Collision Monitor](https://docs.nav2.org/jazzy/tutorials/general_tutorials/using_collision_monitor/using_collision_monitor/)：最后一跳接线、减速/停止区域、传感器 timeout。
- R31 [Follow Dynamic Point](https://docs.nav2.org/jazzy/getting_started/nav2_behavior_trees/trees/follow_point/)：借鉴更新路径交给同一 FollowPath 的结构，不照搬其任务语义。
- R32 [Keepout Filter](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_filters/keepout_filter/)：全局/局部约束、mask 与 filter 配置；结合本文的版本生效确认和完整 footprint 校验，防止执行器绕过已选通道。

## D. 模型与可视化

- R33 [GPT-6 Luna](https://developers.openai.com/api/docs/models/gpt-6-luna)：模型 ID、图像、reasoning 与结构化输出能力；实际账号仍需验证。
- R34 [Structured Outputs](https://developers.openai.com/api/docs/guides/structured-outputs)：strict schema、additionalProperties、refusal/incomplete 处理；schema 合法不等于业务可执行。
- R35 [Responses create](https://developers.openai.com/api/reference/resources/responses/methods/create)：图像请求、text.format、输出、usage 和 API 参数。
- R36 [Foxglove Image panel](https://docs.foxglove.dev/docs/visualization/panels/image)：同帧图像、2D标注、3D投影的表达方法。
- R37 [Foxglove ImageAnnotations](https://docs.foxglove.dev/docs/sdk/schemas/image-annotations)：标注元数据与时间关联；本 viewer 可以用自己的等价轻量数据结构。
- R38 [Rerun Events and Timelines](https://rerun.io/docs/concepts/logging-and-ingestion/timelines)：多时间轴、frame index、source time 与 log time 的组织。
- R39 [Rerun detect and track objects](https://rerun.io/examples/video-image/detect_and_track_objects)：图像、框和逐帧数据关联的具体例子；本任务不因此要求引入其检测模型。

---

# 13. 最终自查

- [ ] 代码模块都能对应一个用户可见能力，而不是只有架构空壳。
- [ ] 新增文件、API、CLI 均在本机真实实现后才在 README 写“可运行”。
- [ ] 首版实际输入是 RGB-D/激光/地图的组合，报告名称准确。
- [ ] 候选生成、目标定位与 UI 都没有把评估真值暗中传给模型。
- [ ] route_version 与 path_seq、sim_time 与 wall_time、模型提议与实际执行分别记录。
- [ ] 中间路点不强制驻留，最终到达有独立证据。
- [ ] API 与录像均不暂停物理；UI 变慢不拖住控制。
- [ ] 过期回答、取消残留、planner 抄近路、reset 污染和未来事件显示都已测试。
- [ ] 每个失败保留，不把修复后成功覆盖旧失败。
- [ ] 影片、原始回放与结果表对应同一运行，倍速和离线重渲染明确。
- [ ] 最终报告清楚列出完成、未完成、实测值和复现步骤。

**本轮完成的标准：用户能启动一个真实、连续的导航任务，看见 Luna 依据图像作出的选择，看见执行器如何处理它，并能从录像和日志复核机器人实际完成了什么。**

