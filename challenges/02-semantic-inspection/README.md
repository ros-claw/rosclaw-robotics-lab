# Challenge 02 — USD-known semantic inspection

目标：让 Agent 从一句区域描述选择 USD 语义目标，生成并验证新观察位置，复用现有 rosclawd/Nav2 受控导航。禁止增加一个写死的第五站并称作空间理解。

当前交付是**只读候选生成原型和执行设计**。原型已读取本机真实仓库 USD 清单、官方占据地图、实际 PhysX 位姿与已编译 Body 绑定，针对开发集目标生成了安全静态候选。它没有执行新目标导航，也未完成 Challenge 02 物理验收。`authorization=false`、`execution_allowed=false` 保持显式。

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

完整执行与公平对照方案见 [DESIGN.md](DESIGN.md)。
