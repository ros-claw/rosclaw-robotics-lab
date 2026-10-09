# Challenge 02 交付与复现

本阶段是开发集工程验收，任务、边界、全部尝试和视频见 [Challenge 02 README](../../challenges/02-semantic-inspection/README.md)。保留首轮失败和首次录制不完整；留出集、公平对照与独立工程师实测未执行。

## 下载与核对

从 [semantic-observation-v0.3.0](https://github.com/ros-claw/rosclaw-robotics-lab/releases/tag/semantic-observation-v0.3.0) 下载：

- `semantic-evidence-package.zip`：4 次开发尝试的规范收据、动作/路径/扫描/PhysX 证据、Body、配置和逐次冻结源码；包含失败。
- `rosclaw-semantic-promo-90s.mp4`、对应 JSON 与海报：c02b02 同一任务同步三视角。视频只有展示字幕，没有配音。
- `SHA256SUMS`：逐个发行文件的 SHA256；解压证据包后再核对 `manifest-sha256.json`。

源文件中原始绝对路径保留用于审计；离线评价器将白名单动作与语义证据映射至解压目录，并验证原文件哈希，不改写 Body 或配置。模型认证、操作员私钥、私有 Agent home、原始思考、完整私有账本及 NVIDIA USD/安装器不在证据包中。

## 离线重放

后三次成功任务实际使用 ROSClaw 候选 SHA `8340a693801f28c2c0703a049e6306e5bd82b147`，不是已合并 main 的集成测试。可从公开来源分支 `test/semantic-pilot-upstream-20261009` 获取该提交。

```bash
# 先核对发行文件 SHA256。使用隔离的 ROSClaw checkout 和 Python 3.12 环境。
git clone https://github.com/ros-claw/rosclaw.git rosclaw-c02-replay
cd rosclaw-c02-replay
git checkout 8340a693801f28c2c0703a049e6306e5bd82b147
python3.12 -m venv .venv
.venv/bin/python -m pip install -e . numpy scipy Pillow PyYAML
cd /path/to/rosclaw-robotics-lab
python3 challenges/02-semantic-inspection/replay_evidence.py \
  --archive /path/to/semantic-evidence-package.zip \
  --output /path/to/new-empty-replay-directory \
  --rosclaw-source /path/to/rosclaw-c02-replay
```

工具逐项核对 manifest，再以每次试验自己冻结的评价代码重放成功任务。首轮原始 FAIL 缺少完整任务收据，明确记录 `NOT_REPLAYED`，不把它升级成成功。重放结果是证据自洽检查，不是新的仿真或第三方复现。依赖安装需要网络，Python 依赖并未制作离线 wheel 包。

## 重新运行实际仿真

依照 [Challenge 01 安装教程](../../challenges/01-isaac-warehouse-patrol/docs/tutorial_zh.md) 准备 Isaac Sim 6.1 ARM64、官方在线资产、ROS workspace、Docker 和模型认证。由使用者接受 NVIDIA 许可；先确认没有本项目正在运行的仿真，再按 [Challenge 02 执行命令](../../challenges/02-semantic-inspection/README.md#执行与留存) 创建新输出目录。每次任务开始前冻结新 Body、实际起点、观察者、代码和编译产物。录制命令必须同时通过任务物理验收与 `recording-acceptance.json`。

现有环境是 DGX Spark / Ubuntu 24.04 ARM64，Isaac Sim 6.1.0 rc.26，ROS 2 Jazzy，Nav2 1.3.13，rosbridge 2.7.1。实际模型 `openai-codex/gpt-6.1-sol`。基础安装、缓存和模型网络条件影响重新运行，不把暖缓存任务时间写成从零开发时间。

## 后续实验入口

当前公开清单包含整个仓库的 USD 几何，8/5 的实体分组只作为开发阶段拆分记录。严格未知场景实验须使用未在开发与公开证据中暴露的新布局/目标，由独立评价端保管，提前冻结代码、提示词、阈值、随机种子和所有尝试分母。

任务运行对照与机器人应用开发效率对照分开进行；同模型、同预算、同基础工具和同 daemon 执行边界，不预设 ROSClaw 优于 Codex。详见 [公平对照协议](../../challenges/02-semantic-inspection/FAIR_COMPARISON.zh.md)。用户目前未安排独立工程师及另一台机器，第三方实测状态仍为 **NOT RUN / 待安排**。
