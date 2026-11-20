# P2 单分叉灰盒

SimScene 根据 `scene/layouts/fork.yaml` 导出固定场景。静态障碍和地面带 USD 碰撞，标牌是视觉道具。相同配方同时生成 5 cm 分辨率 Nav2 先验地图。导航进程只载入 `nav/` 中的中性门区、地图和参数。实际标牌含义和期望门区写入独立的 `evaluator_manifest.json`。

两种标牌顺序分别导出至 `/data/rosclaw/robotics_lab/p2-evidence/pair`。使用 `cmp` 验证两份地图与公开场景清单逐字节一致。评估预期由 `c24` 变为 `c71`。景物组合在 UTC 试次 `20261120T235054Z-ac267710` 进入单 Isaac 世界，真实点云、LaserScan 和 Nav2 启动检查通过。此试次的后续 RGB-D 观测因为缺少持续的 AMCL TF 被拒绝。启动证据保存在 `p2-evidence/runs`。

已加入完整路径的有向门区、禁用门区和扫掠余量校验。回归测试验证即便两条路径通往同一终点，切换到未经承诺的一侧也会被拒绝。本阶段的另一侧实拍、在线候选投影、足迹运行检查、L1/L3 和验收 seeds 仍需继续。

从 challenge 目录运行以下命令。输出目录必须为新目录。

```bash
workspace=/data/rosclaw/robotics_lab
isaac=/data/omniverse/isaac-sim-6.2.0
usd_lib=$(find "$isaac/extscache" -maxdepth 1 -type d -name 'omni.usd.libs-*' -print -quit)
env PYTHONPATH="$usd_lib:/usr/lib/python3/dist-packages" LD_LIBRARY_PATH="$usd_lib/bin" \
  "$isaac/python.sh" scene/build_scene.py "$workspace/p2-evidence/reproduction" \
  --variant original --simscene "$workspace/simscene" --nvidia "$workspace/isaac-ros-6.2"
LUNA_EVIDENCE="$workspace/p2-evidence/runs" python3 scripts/lab.py start --rgbd \
  --layout "$workspace/p2-evidence/reproduction"
```
