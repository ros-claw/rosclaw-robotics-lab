# P1 RGB-D 观测记录

前向 RGB-D 来自 Nova Carter 底盘上的独立相机。RGB、深度、CameraInfo 使用同一 render product。观测端按 stamp 严格关联，再查询采集时刻的 TF。深度缺失、无有效像素、重复 stamp 或无法取得 TF 的样本无法作为正常采集输出。当前采集工具只保存证据，无运动权限。

- 正常仓库场景 UTC 试次 `20261120T234015Z-15c0200d` 的 `rgbd-observer` 保存了五组 960×540 RGB-D 与对应的 `odom ← camera`、`map ← odom`。原始帧和深度数组有 SHA-256。现场相机画面中可辨认货架、箱垛和地面。
- 标定试次 `20261120T234415Z-a32b41c5` 的 `rgbd-7e951eba` 保存五组标定帧。两块独立编排的标靶分别位于光学轴深度 1.0 m、1.6 m，且偏离主点。十次测量均通过 1 cm 检查，最大三维偏差约 0.313 mm。`calibration-results.json` 保留读数与像素。
- 早期采样缺少 `map ← odom` 时以非零状态退出。本机正常场景之后取得了该 TF，标定场景缺少它的尝试全部保留。标定帧只测相机几何，记录为 `isaac-live-calibration`。
- 单元测试覆盖时钟回退后旧 epoch 拒绝、时钟停住时拒绝重复图像、无效深度及 z-depth 往返投影。实际转弯、暂停/恢复、定位注入和运行中 reset 的联调验收仍待完成。

证据根目录为 `/data/rosclaw/robotics_lab/p0-evidence`。校验命令如下，目录参数使用 `observe` 打印的实际路径。

```bash
python3 scripts/lab.py start --rgbd
python3 scripts/lab.py observe --count 5 --timeout 45
python3 scripts/lab.py stop
```

用 `start --calibration` 启动靶标验证，采样后运行 `python3 ros2/verify_calibration.py <取证目录>`。下一次启动获得独立 episode。采样端依赖宿主 Humble 的 rclpy、NumPy、Pillow。

灰盒动作试次 20261121T000709Z-fd21cc13 的 rgbd-0e21a2d3 保存 12 组底盘运动期间的同步 RGB-D 和实时 TF。该试次揭示了原始 USD 播放范围回绕，已在保护控制入口关闭循环，详见 [P3 报告](P3_REPORT.md)。
