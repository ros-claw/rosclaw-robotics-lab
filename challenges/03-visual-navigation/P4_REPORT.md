# P4 Luna 视觉接口与换牌对照

已用项目凭据实测 OpenAI Responses API 的 `gpt-6-luna`、真实 PNG 输入、严格 JSON Schema 和本地二次校验。模型只接收任务、固定候选及照片。独立评估程序另外读取标牌真值。探测器没有 ROS 控制权限。

| 真实 Isaac 输入 | 模型结果 | 端到端响应 |
| --- | --- | --- |
| 空白墙面 | `reobserve`，候选为 null | 3.08 秒 |
| 蓝色 B 在右侧 | `select_candidate c24`，符合预期 | 2.93 秒 |
| 蓝色 B 换到左侧 | `select_candidate c71`，符合预期 | 2.67 秒 |

两次换牌试次保留了相同的公开场景清单、Nav2 地图、任务和中性候选。发送的帧字节和 SHA-256 均已留存，独立程序复验验收通过。左右样本来自单独启动的真实 Isaac 世界，各使用相同的诊断相机偏转 64°，以补偿当前灰盒底盘静置后偏离分叉的朝向。其静态 TF 使用同一旋转。此模式拒绝控制测试。

根目录为 `/data/rosclaw/robotics_lab/p4-evidence`。包含两轮 Isaac 记录、实际视觉输入、请求清单、解析结果和 `semantic-pair.json`。初始视觉请求的 HTTP 401 和后续成功均已记录。

从 challenge 目录运行下面的命令。使用本机 `OPENAI_API_KEY`，输出目录需要是新目录。

```bash
python3 scripts/lab.py luna-probe CAPTURE_DIR PUBLIC_SCENE_MANIFEST OUTPUT_DIR --frame 1
python3 evaluator/semantic_probe.py ORIGINAL_OUTPUT SWAPPED_OUTPUT ORIGINAL_LAYOUT SWAPPED_LAYOUT
```

这组证据覆盖可见标牌改变模型选择。底盘对 Luna 意图的实时承诺、过门、模型时限与回放仍待联调。

同一对帧另以 low reasoning 各复测一次，仍选择右侧 c24、左侧 c71。响应耗时分别为 3.67 和 3.31 秒。本轮耗时数据仅覆盖这些样本，实时模型时限尚未冻结。
