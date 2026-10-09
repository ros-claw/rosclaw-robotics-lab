# Challenge 02 — USD-known semantic inspection

目标：让 Agent 从一句区域描述选择 USD 语义目标，生成并验证新观察位置，复用现有 rosclawd/Nav2 受控导航。禁止增加一个写死的第五站并称作空间理解。

v0.2 交付了只读候选生成原型和执行设计。v0.3 当前已实现 daemon 内动态候选与单目标执行接口，真实任务验收正在进行。原型已读取本机真实仓库 USD 清单、官方占据地图、实际 PhysX 位姿与已编译 Body 绑定，针对开发集目标生成了安全静态候选。它没有执行新目标导航，也未完成 Challenge 02 物理验收。`authorization=false`、`execution_allowed=false` 保持显式。

`propose_observation.py` 不读取 `inspection_sites.yaml`。它使用真实货架 prim 包围盒，在边界外生成面向目标的候选；按机器人足迹外接圆、Nav2 padding、额外余量、占据/未知单元、地图边界、当前位姿的可达连通域及静态视线筛选。物理状态须新鲜、时间线播放中、接触观察完整且无碰撞。坐标、地图/配置/Body/观察者来源及 proposal hash 一起输出。

保守静态地图分析不能证明实时安全路径。执行前必须在现有 daemon 内重新验证最新地图/Costmap、TF/时钟、机器人位姿、Body intent 和 Nav2 ComputePathToPose 的扫掠足迹，再通过现有 `request_action` 派发。未知空间、动态障碍、目标检测和实际“检查完成”的判据仍需独立验证。

```bash
# 使用已安装 ROSClaw 环境（numpy/Pillow/scipy/PyYAML）；不连接运动接口
python propose_observation.py \
  --inventory ACTUAL_PHYSICS_RUN/stage-inventory.json \
  --map-yaml /path/to/carter_warehouse_navigation.yaml \
  --physics ACTUAL_PHYSICS_RUN/physics-latest.json \
  --body ACTUAL_NATIVE_RUN/body.json \
  --nav-params ../01-isaac-warehouse-patrol/config/patrol_navigation_params.yaml \
  --target '/World/.../SM_RackShelf_...' --output proposal.json
python -m unittest discover -s tests
```

语义来源是 USD 已提供的 prim 名称和几何包围盒，属于场景先验；没有声称相机发现目标。展示相机不成为 Agent 的视觉传感器。

按实际物理货架实体分组：同一货架的父/子 prim 合并到最外层 RackShelf Xform，再按实体路径 SHA256 前 8 位模 4 等于 0 固定 holdout。实际清单得到 **8 个开发实体、5 个 holdout 实体**。只对一个开发实体运行了候选生成；holdout 未运行。分组规则及目标 manifest 必须在正式实验前冻结，并向参试 Agent 隐藏 holdout 清单。当前仅为准备工作，没有泛化实验结论。早期 prim 级划分已弃用，避免同一货架父/子 prim 跨组泄漏。

完整执行设计见 [DESIGN.md](DESIGN.md)，公平对照的配对、消融、分母和开发计时方案见 [FAIR_COMPARISON.zh.md](FAIR_COMPARISON.zh.md)。实际对照尚未运行。

## 场景任务与阶段

| 阶段 | 用户指令与输入 | 验收门槛 | 状态 |
|---|---|---|---|
| 02-A 语义观察点导航 | “去本次起点西侧最近的一组开发集货架，面向货架观察，再返回实际起点。”；USD 已知货架、地图、实际位姿 | 目标语义正确；动态坐标不来自四站配置；Body/候选 ID 绑定；最新 Nav2 路径及扫掠足迹；真实到达/朝向/停留/接触；最终 Memory/TaskKernel | 实现完成，真实试跑待验收 |
| 02-B 已知区域 LiDAR 观测 | 同一任务，增加实际扫描端点落入目标区域的验证 | 02-A 全部门槛，加上已知时间和 map→LiDAR 变换、至少 3 个实际目标区域回波 | 实现完成，真实试跑待验收；不代表视觉识别或缺陷检测 |
| 02-C 留出目标与公平对照 | 冻结后揭示目标，同模型/工具/预算的配对任务 | 所有失败、拒绝和人工介入留存，分开报告运行与开发效率 | 尚未执行 |

新增 SIM Body 在用户输入前冻结开发目标集合、源文件哈希、实测起点和校验规则。Native 只读 daemon 生成的目录，通过 `proposal_id` 请求单次动作；不能提交任意 x/y 或改写四站权限。候选会过期，执行前重新读取 TF/时钟/Costmap，并对 Nav2 规划的完整路径按半地图单元平移和 0.05 rad 旋转间隔检查足迹。

## 执行与留存

在已配置且没有其他本项目仿真会话的环境中：

```bash
# ROSCLAW_SOURCE 必须是本次冻结并编译的 checkout；固定 Python 必须导入该 checkout。
export ROSCLAW_SOURCE=/path/to/pinned/rosclaw
export PYTHONPATH="$ROSCLAW_SOURCE/src${PYTHONPATH:+:$PYTHONPATH}"
export ISAAC_ROS_WS="$HOME/IsaacSim-ros_workspaces/jazzy_ws"
"$ROSCLAW_SOURCE/.venv/bin/python" run.py --output "$HOME/sim/c02a01" --isolated-simulation
# 独立重置另一个输出目录，增加真实区域 LiDAR 观测和三视角录制：
"$ROSCLAW_SOURCE/.venv/bin/python" run.py --output "$HOME/sim/c02b01" --isolated-simulation \
  --require-target-lidar --record-multiview
```

`run.py` 只做测试编排；任务由真实 Native Agent 接收一次输入，通过原有 Agentd/Operator/rosclawd 执行。每次创建独立重置，拒绝覆盖输出，保留失败并清理本项目资源。`evaluate.py` 重放规范收据、独立 PhysX 轨迹、路径足迹、语义任务和 Memory/TaskKernel。开发试跑不等于留出集或公平比较。

## 视频展示

尚无已验收的 Challenge 02 宣传视频。完成真实任务并核验后，才在这里展示对应片段、源码版本和证据；Challenge 01 的公开视频仅展示四站巡检。
