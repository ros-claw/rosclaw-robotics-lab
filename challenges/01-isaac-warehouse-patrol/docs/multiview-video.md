# 一句话巡检：同步三视角演示与教程

本演示提交一次中文任务，由真实 ROSClaw Native Agent 发现 ROS 能力、
逐站提出导航动作、检查执行结果，最后完成独立物理验收和 Memory 保存。
本轮 SIM 动作按测试策略自动审批；导航由真实 Nav2、实际 RTX LiDAR 和
局部代价地图执行。展示相机并不是 Agent 的视觉输入。

## 画面

输出 2560×1440。主画面为 1920×1080 原始第三人称相机的缩小排版，
右上为机器人前向视角，右下为跟随机器人移动、12 米宽的顶视相机。
两个辅助相机实际捕获 1280×720，再缩小为 960×540。没有通过放大低分辨率
原图来冒充细节。三相机在同一次捕获中提交，每一组核对实际 renderer
frame ID 一致；SWH frame ID 不可用。物理轨迹为独立 PhysX 采样。

左下显示从实际 SDK session 提取的可见 toolCall 名称及站点，
开始阶段重放真实终端可见输出。原始思考、认证文件和完整私有 session 不发布。
讲解字幕和中文合成旁白属于后期说明，不冒充 Agent 输出。

## 时间

宣传版 180 秒，开头和收尾保留正常墙钟时间，中间导航按画面标注压缩；
完整教程保留任务墙钟时间。两版均含 10 秒真实场景片头定格和
12 秒实际结束画面验收卡。画面根据真实时间戳 PNG 和终端事件重建，
捕获间隔内重复最近的真实帧，无运动插值。它不是连续屏幕录像，
仿真时间与墙钟时间不同，均明确标注。

## 制作

```bash
# 先运行 headless patrol 环境；三视角配置必须在启动前指定
export ROSCLAW_CAMERA_VIEW=follow
export ROSCLAW_CAMERA_RESOLUTION=1920x1080
export ROSCLAW_MULTIVIEW=1
export ROSCLAW_CAPTURE_SECONDS=1800
export ROSCLAW_OBSTACLE_TEST=1
export ROSCLAW_REQUIRE_OBSTACLE_EVIDENCE=1
./scripts/demo.sh headless patrol
./scripts/start-agent-observers.sh
# 同时启动 obstacle_witness.py，再通过 run-native-acceptance.sh 提交唯一任务
# 等待独立 run_report.py 输出 PASS 后渲染
python visualization/render_multiview.py --directory RUN --report REPORT_JSON \
  --map OFFICIAL_MAP_PNG --output MOVIE.mp4 --ffmpeg FFMPEG --mode promo
python visualization/narrate_multiview.py --video MOVIE.mp4 --report REPORT_JSON \
  --output MOVIE_ZH.mp4 --ffmpeg FFMPEG --piper-model VOICE_ONNX
```

渲染环境需要 Pillow、pyte、PyYAML 和 FFmpeg。最终成片的中文旁白采用
本地 Piper `zh_CN-huayan-medium`，模型 SHA256 记录在视频 JSON 中。
[语音引擎与安装说明](https://github.com/OHF-Voice/piper1-gpl)、
[声音模型卡](https://huggingface.co/rhasspy/piper-voices/blob/main/zh/zh_CN/huayan/medium/MODEL_CARD)。
引擎只在制作机器运行，不随视频打包；模型卡将训练数据许可列为 Unknown，
本交付不声称该声音已经完成商业授权审查。旁白属于 AI 合成的后期讲解。
可选在线 edge-tts 在制作时出现空音频，重试后仍有片段失败，最终未用于成片。

字幕 SRT 独立保存并嵌入 MP4。相机源代码与绘制脚本可审阅；最终清单
记录视频哈希、任务报告哈希、时间压缩和同步性检查结果。未通过完整
物理验收的录制不会作为成功宣传片。
