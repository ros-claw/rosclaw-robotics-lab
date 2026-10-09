# 独立工程师复现交接包

状态：**第三方实测待安排**。用户已确认暂未指定工程师和另一台机器。本包用于交接；本机回归、离线证据重放和自动化测试不能代替独立复现。

建议条件：未参与实现的工程师；另一台支持 Isaac Sim 6.1 的 ARM64 NVIDIA GPU 主机。首轮目标是复现同架构基线；其他平台另外记录，不默认兼容。记录机器是否全新、已有缓存/依赖/模型配置，分别计时。

1. 工程师自行获取 NVIDIA 官方 Isaac Sim 6.1 ARM64 安装包，阅读并接受其 EULA。不要从本项目复制 NVIDIA USD、安装器或资产缓存。
2. 从正式 Release 下载源码/教程/证据校验清单，记录实际 Git SHA。按公开教程构建固定 ROS workspace 和 ROSClaw checkout；Docker 基础镜像有 digest，但 apt 结果可能随软件源变化，记录实际包版本与本地 image ID。
3. 配置自己的正常模型认证。不要复制开发者的 `.rosclaw`、`.codex`、Native `home/` 或操作员密钥。
4. 先运行统一诊断入口，再 headless 启动一套新环境。确认专用 ROS 域和端口未占用。不要停止其他项目的容器。
5. 使用短结果路径（例如 `$HOME/sim/repro01`，位于 Git 外），只输入一次标准巡检任务。收集失败和必要人工介入，不删除失败后重试来形成“首次成功”。
6. 运行独立评价器并使用白名单打包器导出证据。检查 Body、源代码和收据哈希；每站物理到达/停留/LiDAR/接触以及最终 Memory/TaskKernel 都须满足。
7. 填写 [REPORT_TEMPLATE.md](REPORT_TEMPLATE.md)，逐项记录步骤、耗时和问题。只有实际完成后，才把报告状态改成“独立复现通过”。

统一入口（在 `challenges/01-isaac-warehouse-patrol`）：

```bash
python3 scripts/lab.py doctor
python3 scripts/lab.py start headless patrol
# 另一终端：./scripts/start-agent-observers.sh
python3 scripts/lab.py task "$HOME/sim/repro01" "$PHYSICS_DIR" "$TASK" entry shelf aisle home
"$ROSCLAW_SOURCE/.venv/bin/python" evaluator/run_report.py \
  --directory "$HOME/sim/repro01" --output reports/external-reproduction.json
python3 scripts/lab.py stop
```

`PHYSICS_DIR` 取启动打印的实际绝对路径；`TASK` 使用教程的标准任务。`ROSCLAW_SOURCE` 按 runtime.env 的实际配置导出。统一入口不隐式发目标；`task` 通过真实 Native/rosclawd 流程执行。15 轮工程回归可用 `lab.py regression "$HOME/sim/r15"`，`lab.py results "$HOME/sim/r15"` 查看当前或最终摘要；当前平台路径长度须通过预检。

离线重放只说明发布证据自洽，不说明在此机器上重新运行了仿真。交接包不含认证、私有运行目录、NVIDIA 资产或 Piper 声音模型。
