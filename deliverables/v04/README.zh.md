# v0.4 装卸区自主安全巡查：交付与复现索引

FINAL — actual post-merge upstream build and required live/offline acceptance passed.

正式冻结实现 `d1a1db271ae0030ea47df415d890751dcfd205fb`；ROSClaw 实际上游合并和合并后 Native 重建均为 `9862058c445c88156b9080832b5dd803c75279ae`。源码构建不是 PR 头或同树推断。上游 [PR #660](https://github.com/ros-claw/rosclaw/pull/660)，工程 [PR #6](https://github.com/ros-claw/rosclaw-robotics-lab/pull/6)。

最终冻结批次 12 次独立重置全部通过（A 3/3 · B 3/3 · C 3/3 · D 3/3）；共 27 次验证到点，最大位置误差 0.172721 米，最大朝向误差 0.251624 rad，最小稳定停留 2.017 仿真秒，零非地面有效接触。 同一最终源码的全部装卸区尝试（含校准与中断批次，旧任务兼容另计）为 20 PASS / 1 FAIL；最后批次的通过率不覆盖这些历史失败。

| Attempt | Case | Task | Result | Coverage | Views / actual new cells | Native wall (s) | Measured SIM span (s) | Model turns | Tool calls |
|---|---|---|---|---:|---|---:|---:|---:|---:|
| l04ia01 | A | PASS | OBSTRUCTED | 100.0% | 238 | 162.1 | 122.0 | 7 | 6 |
| l04ia02 | A | PASS | OBSTRUCTED | 100.0% | 238 | 173.7 | 116.8 | 7 | 6 |
| l04ia03 | A | PASS | OBSTRUCTED | 100.0% | 238 | 158.3 | 120.1 | 7 | 6 |
| l04ib01 | B | PASS | OBSTRUCTED | 81.5% | 190 / 4 | 210.6 | 137.0 | 9 | 8 |
| l04ib02 | B | PASS | OBSTRUCTED | 81.5% | 191 / 3 | 226.5 | 113.4 | 9 | 8 |
| l04ib03 | B | PASS | OBSTRUCTED | 81.5% | 192 / 2 | 235.3 | 114.7 | 9 | 8 |
| l04ic01 | C | PASS | OBSTRUCTED | 100.0% | 200 / 38 | 435.2 | 80.7 | 10 | 9 |
| l04ic02 | C | PASS | OBSTRUCTED | 100.0% | 202 / 36 | 267.6 | 108.4 | 9 | 8 |
| l04ic03 | C | PASS | OBSTRUCTED | 100.0% | 200 / 38 | 255.8 | 109.1 | 9 | 8 |
| l04id01 | D | PASS | UNKNOWN | 0.0% | none (refusal) | 85.4 | 46.3 | 6 | 5 |
| l04id02 | D | PASS | UNKNOWN | 0.0% | none (refusal) | 91.3 | 49.4 | 7 | 6 |
| l04id03 | D | PASS | UNKNOWN | 0.0% | none (refusal) | 96.2 | 56.8 | 6 | 5 |

SIM spans use actual physics samples bracketing each Native window, including model/operator waiting; individual boundary overhangs are recorded in performance-windows.json. No time interpolation.

本机 SIM 工程验收；不声称生产可靠性、独立工程师复现或优于 Codex。历史校准、三个中断批次与最终正式批次分开记账；失败不替换。最早的 ROI 裁剪 CLEAR 已撤回，原始记录与重分析并存。D 正式验证授权区域拒绝，真实返回路径拒绝失败另列。最终源码第三批 B3 也有一次真实任务 FAIL（初始转向被拒绝）；最终协议将 B 起点移至校准过的开阔区域，不改安全阈值。最后批次结果不代表全部同源码尝试 100% 成功。

## 使用入口

- [完整实施报告](../../FINAL_IMPLEMENTATION_REPORT.md)、[中文报告](IMPLEMENTATION_REPORT.zh.md)。
- [场景任务与视频](../../challenges/02-semantic-inspection/README.md)。
- [中文教程](../../challenges/02-semantic-inspection/docs/LOADING_TUTORIAL.zh.md)、[English tutorial](../../challenges/02-semantic-inspection/docs/LOADING_TUTORIAL.md)。
- [实际场景/叉齿代理/低障碍审计](../../challenges/02-semantic-inspection/docs/P0_AUDIT.zh.md)。
- [独立复现交接清单](REPRODUCTION_CHECKLIST.zh.md)：第三方新机器执行 NOT RUN。
- [冻结协议](protocol.json)、[正式结果](formal-summary.json)、[全部尝试](all-attempt-ledger.json)、[实际输入审计](single-input-audit.json)。
- [来源及实际构建绑定](runtime-binding.json)、[合并记录](upstream-merged-pr.json)、[视频来源](video-provenance.json)、[未映射箱体证明](static-map-fixture-audit.json)。

## 下载与离线重放

[公开发布](https://github.com/ros-claw/rosclaw-robotics-lab/releases/tag/loading-area-inspection-v0.4.0) 提供视频、原始白名单证据、P0 包与 SHA256。Git 中不包含 NVIDIA USD、安装器、大视频、私有 HOME、认证、操作员密钥或完整模型思考。

```bash
# 下载对应发行中的 archive 和 SHA256 后，先在本机核对哈希。
# upstream checkout 必须匹配 archive 中成功试验的真实 SHA。
/path/to/rosclaw-merged/.venv/bin/python deliverables/v04/replay_evidence.py \
  --archive loading-formal-evidence.zip \
  --output /tmp/loading-replay-new \
  --rosclaw-source /path/to/rosclaw-merged
```

重放工具拒绝路径穿越和符号链接，校验 ZIP 白名单文件哈希，并以各试验自己的冻结代码重新解码点云、检查足迹/物理到达/接触/收据/返回/Memory，以及评价侧 PhysX 场景条件。失败任务保持原状态，不制造缺失收据。离线重放只是证据一致性，不是新仿真执行。

最早 Pilot 包包含不同上游修订，不能全部用 9862058c 强行重放；按每轮 source-freeze 匹配 checkout。若只检查清单完整性，不应标为物理 replay PASS。各版本 zip 的覆盖范围有交叠，不能相加成独立尝试数；唯一分母以 all-attempt-ledger.json 的 attempt ID 为准。

## 验收门槛与能力边界

固定 0.2m 网格、≥70% 实测覆盖、0.75m 半径加格点保护、0.10–0.80m 障碍高度、最多两次观察、主动补看新增 ≥3 网格。实际导航要求 ≤0.4m 到达误差、≤0.35rad 朝向误差、≥2 仿真秒稳定停留、完整且零非地面接触、新鲜 LiDAR/TF/clock、Body/Observer/Action/Receipt/Artifact 一致，Memory 先于 TaskKernel 完成。

叉车关系来自本仓库已知设施几何，临时箱体来自实际 PointCloud2。展示 RGB 未进入识别链。当前并非任意语言任务解析、新仓库泛化、货物缺陷检测、工业安全认证或硬件测试。独立人员、新机器和公平对照由后续团队另行安排。
