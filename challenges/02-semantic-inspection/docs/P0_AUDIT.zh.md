# 1010 场景、叉车碰撞与原始点云审计

本审计在新巡查导航之前完成。对象来自当前组合后的实际 USD Stage，包括 instance proxies、不可见碰撞代理及 default/render/proxy/guide purposes，不凭截图臆造资源路径。所有对照均为独立重置仿真，不计为 Native 正式任务。

## 叉齿结论

两辆叉车可见 `S_ForkliftFork` Mesh 的 CollisionAPI 禁用，但官方资产已有有效的不可见 guide Mesh `S_ForkliftFork/collision_box`。初始只读可见几何筛选漏掉了 guide purpose，因此“可见 Mesh 无碰撞”不能推断“叉齿无物理碰撞”。无需额外代理，也没有修改 NVIDIA 原始资产。

| 实体 | 实际代理世界 AABB min | max |
|---|---|---|
| Forklift/S_ForkliftFork/collision_box | (1.8542,-3.4048,0.0572) | (3.5342,-1.7610,1.0489) |
| Forklift_01/S_ForkliftFork/collision_box | (-9.5914,-3.4154,0.0572) | (-8.3283,-2.2664,1.0489) |

最终 Playing Stage 清单包含 1679 个有界 schema 行和 26 个原始 fork 相关 prim。两组各 32 个实际可见叉齿表面采样点，32/32 的 PhysX overlap 都命中对应官方代理。受控球体接触记录也分别包含两辆叉齿代理。

初次靠货架叉齿的高处落球先接触托盘/货架，是混杂对照，不接受为叉齿命中证明。低处靠货架球体最终也同时接触邻近托盘；完整碰撞路径保留，结论由表面 overlap、代理几何、真实接触交叉支撑，不虚称单一接触来源。开阔区叉齿垂直射线命中代理；靠货架垂直射线受上方货架遮挡，原记录保留。

以上为当前资产与采样点的仿真审计，不证明机器人可以穿过任意叉齿附近，也不是任意姿态全几何等价认证。巡查路径仍排除完整已知叉车包络并独立检查足迹。

## 实体清单

实际小拖车为 `/World/warehouse_with_forklifts/Warehouse_Empty_small_realtime/SM_PushcartA_02_22`，Xform AABB 约 (-7.063,12.901,0) 至 (-5.194,14.344,0.377)；其子 Mesh `SM_PushcartA_02` 启用碰撞。清单同时提取货架层、两辆叉车、托盘及其他有界设施，完整原始 Stage 审计供评价侧查阅。不根据图片声称拖车货物种类、数量或机械状态。

当前两辆叉车与最近货架的二维包围盒距离分别约 0m 与 10.6186m。机器人选择层按实际几何关系排序，不使用 Forklift_01 后缀作为答案。交换命名的单元测试验证该点。不同起点及靠近另一辆叉车的 C/D 场景验证选择不依赖机器人最近距离。

## 传感器

实际原始 PointCloud2 是 `/front_3d_lidar/lidar_points`，与 `/scan` 的 LaserScan 不同；实际里程计为 `/chassis/odom`，`/odom` 没有有效发布者。TF、clock、局部 costmap 及 QoS/endpoints 随探针记录。

正常三帧每帧约 45000 个有限 XYZ，地面约 11933 点/帧，离地 0.8m 以下约 14322 点/帧；原始字节、字段布局、SHA256 和测量时间精确 TF 均留存。缓冲预热期的 TF 外推错误也保留，只接受随后新鲜且完整的三帧。

低箱体审计在 x=-8m、y=-1.1/-0.2/0.7m 放置三个实际静态碰撞箱，宽 0.35m，高 0.15/0.35/0.70m。各三帧落在箱体实际边界、且 z≥0.10m 的点数为：

| 箱高 | 三帧实际端点数 |
|---|---|
| 0.15m | 39 / 38 / 38 |
| 0.35m | 327 / 323 / 320 |
| 0.70m | 691 / 692 / 690 |

结论仅覆盖这些姿态、距离和高度；低于 0.10m 的物体没有 clearance 承诺。平面 LaserScan 的 XY 命中不证明看到了高层货架，不证明货物缺陷检查。

## 保留的问题与修正

- 07:05:55 启动失败：BBoxCache Boost API 不接受所用 ignoreVisibility 关键字，改成实际支持的完整位置参数。
- 07:08:02 启动失败：上一启动监护进程的延迟 ERR 清理误停了新重置；当时先等待所有旧监护退出，后续脚本增加模拟器 PID 所有权比对。
- 初始地面端点覆盖不足的算法保留为 Pilot 证据，后续改为三个高度带的实际有限射线观测，没有通过放宽覆盖阈值挽救失败。
- 旧 Pilot 将区域外邻近叉车裁掉，产生不充分的 CLEAR；独立几何检查揭示后加入实际点云 halo。旧结果文件未改写，正式报告明确撤销其作为现行 CLEAR 证据的资格。

PhysX overlap_box 的半尺寸、位置和 xyzw 参数采用 [NVIDIA 官方 API](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.0/extensions/runtime/source/omni.physx/docs/api/python.html)，最终以本机实际调用和保存的命中路径验收。所有审计重置、混杂对照和启动错误见 P0 attempt ledger 与发布白名单证据。
