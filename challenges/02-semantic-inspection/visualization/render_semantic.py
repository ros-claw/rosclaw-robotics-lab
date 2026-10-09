"""Render one verified C02 mission from synchronized real screenshots and receipts."""

import argparse
import bisect
from datetime import datetime
import hashlib
import json
from pathlib import Path
import subprocess

from PIL import Image, ImageDraw, ImageFont

FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
W, H, FPS, SECONDS = 2560, 1440, 8, 90


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--attempt", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--ffmpeg", required=True)
    a = p.parse_args()
    report = json.loads((a.attempt / "acceptance.json").read_text())
    coverage = json.loads((a.attempt / "recording-acceptance.json").read_text())
    run = json.loads((a.attempt / "result.json").read_text())
    if (
        report["status"] != "PASS"
        or coverage["status"] != "PASS"
        or not report["require_target_lidar"]
    ):
        raise ValueError("Complete physical, inspection and recording gates required")
    root = Path(run["physics_directory"]) / "baseline-frames"
    frames = [
        json.loads(s) for s in (root / "timestamps.jsonl").read_text().splitlines()
    ]
    stamps = [r["wall_time"] for r in frames]
    start, end = coverage["native_wall_time"]
    visits = report["visits"]
    task = json.loads((a.attempt / "native/task-kernel.json").read_text())
    task_finished = datetime.fromisoformat(task["updated_at"]).timestamp()
    inspection = next((a.attempt / "native/semantic-evidence").glob("*inspection.json"))
    evidence = json.loads(inspection.read_text())
    # Persist actual verified result, never a count from a different trial.
    hit_count = evidence["verification"]["hit_count"]
    fonts = {n: ImageFont.truetype(FONT, n) for n in (24, 28, 32, 36, 40, 48, 54)}
    a.output.parent.mkdir(parents=True, exist_ok=True)
    command = [
        a.ffmpeg,
        "-y",
        "-loglevel",
        "error",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "rgb24",
        "-s",
        f"{W}x{H}",
        "-r",
        str(FPS),
        "-i",
        "-",
        "-an",
        "-c:v",
        "libx264",
        "-threads",
        "4",
        "-preset",
        "fast",
        "-crf",
        "20",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(a.output),
    ]
    process = subprocess.Popen(command, stdin=subprocess.PIPE)
    last_index, sources = None, {}
    sampled = []

    def text(draw, xy, value, size=32, color="#d9e4f0"):
        draw.text(xy, value, font=fonts[size], fill=color)

    for index in range(FPS * SECONDS):
        # Six-second opening and eight-second evidence hold; continuous source time in between.
        fraction = min(1, max(0, (index / FPS - 6) / (SECONDS - 14)))
        wall = start + (end - start) * fraction
        fi = max(0, min(len(frames) - 1, bisect.bisect_right(stamps, wall) - 1))
        row = frames[fi]
        if fi != last_index:
            sources = {
                name: Image.open(root / view["frame"]).convert("RGB")
                for name, view in row["views"].items()
            }
            last_index = fi
        image = Image.new("RGB", (W, H), "#0c1421")
        draw = ImageDraw.Draw(image)
        text(draw, (24, 18), "ROSClaw  ·  一句话完成语义观察与返回", 54, "#ffffff")
        text(
            draw,
            (26, 88),
            "CHALLENGE 02-A + 02-B  |  Isaac Sim + ROS 2 / Nav2  |  本次独立重置："
            + a.attempt.name,
            28,
            "#70d8cf",
        )
        for name, box, label in [
            ("follow", (24, 150, 1504, 846), "第三人称 · 跟随机器人"),
            ("robot", (1552, 150, 984, 554), "机器人前向 · 展示相机"),
            ("top-close", (1552, 744, 984, 554), "近顶视 · 路径与周围空间"),
        ]:
            x, y, w, h = box
            image.paste(sources[name].resize((w, h), Image.Resampling.LANCZOS), (x, y))
            draw.rectangle((x, y, x + w, y + 48), fill="#122236")
            text(draw, (x + 14, y + 3), label, 28, "#ffffff")
        draw.rounded_rectangle((24, 1018, 1528, 1298), 18, fill="#172539")
        if wall < visits[0]["started_wall_time"]:
            title = "01  一次任务输入 → 读取货架语义与候选观察位置"
            detail = "“检查起点西侧最近的开发集货架，面向货架停留，"
            detail2 = "然后返回本次实际起点，保存验证报告和记忆。”"
        elif wall < visits[0]["finished_wall_time"]:
            title = "02  受控导航 → 实时路径与完整足迹验证"
            detail = "USD 已知目标 → 动态观察位置 → Body 绑定候选 ID"
            detail2 = "实际 Nav2 执行；独立 PhysX 核验到达、朝向与停留。"
        elif wall < visits[1]["started_wall_time"]:
            title = "03  到达后观测 → 实际 LiDAR + TF + 停止状态"
            detail = f"已知货架区域验收通过：{hit_count} 个实际回波。"
            detail2 = "观测按同一时间绑定；不代表视觉识别或缺陷检测。"
        elif wall < visits[1]["finished_wall_time"]:
            title = "04  返回任务开始时的实测位置"
            detail = "机器人逐目标请求动作，由 rosclawd / Nav2 执行。"
            detail2 = "没有写死“第五站”；不是固定四站巡检录像。"
        elif wall < task_finished:
            title = "05  返回后验证整项任务 → 保存报告与记忆"
            detail = "逐项重放动作收据、轨迹、朝向、停留和 LiDAR 证据。"
            detail2 = (
                "等待成功 Memory 和 TaskKernel 结束；返回到点尚不等于整项任务完成。"
            )
        else:
            title = "05  完整任务 PASS → 验证报告 / Memory / TaskKernel"
            detail = "货架误差 %.3f m · 返回误差 %.3f m · 非地面碰撞 0" % tuple(
                v["verification"]["position_error_m"] for v in visits
            )
            detail2 = f"区域回波 {hit_count} · 实际稳定停留 ≥2 仿真秒 · 无人工重发"
        text(draw, (48, 1032), title, 40, "#70d8cf")
        text(draw, (48, 1100), detail, 36)
        text(draw, (48, 1157), detail2, 36)
        text(
            draw,
            (48, 1231),
            "证据：规范动作收据 + 独立 PhysX + 真实扫描；完整任务通过后才写成功记忆。",
            24,
        )
        text(
            draw,
            (24, 1320),
            f"原始墙钟 +{wall - start:6.1f}s / {end - start:.1f}s  ·  仿真 {row['sim_time']:.2f}s  ·  同步渲染帧 {row['views']['follow']['render_frame']}",
            28,
        )
        text(
            draw,
            (24, 1370),
            f"中段连续压缩 {(end - start) / (SECONDS - 14):.2f}×墙钟；开头/结尾定格。8 fps 编码，重复真实帧，无运动插值。开发集试验，非泛化结论。",
            24,
            "#a4b4c9",
        )
        process.stdin.write(image.tobytes())
        sampled.append(
            {
                "output_frame": index,
                "source_frame": row["frame"],
                "source_wall_time": row["wall_time"],
                "render_frame": row["views"]["follow"]["render_frame"],
            }
        )
        if index in (FPS * 40, FPS * (SECONDS - 1)):
            image.save(
                a.output.parent
                / ("semantic-poster.png" if index == FPS * 40 else "semantic-final.png")
            )
    process.stdin.close()
    if process.wait() != 0:
        raise RuntimeError("Video encoder failed")
    metadata = {
        "attempt": a.attempt.name,
        "duration_seconds": SECONDS,
        "encoded_fps": FPS,
        "source_commit": report["source_commit"],
        "rosclaw_commit": report["rosclaw_commit"],
        "source_wall_time": [start, end],
        "continuous_compression": (end - start) / (SECONDS - 14),
        "intro_hold_seconds": 6,
        "outro_hold_seconds": 8,
        "interpolation": False,
        "inspection_hit_count": hit_count,
        "recording_acceptance": coverage,
        "physical_acceptance_sha256": hashlib.sha256(
            (a.attempt / "acceptance.json").read_bytes()
        ).hexdigest(),
        "sha256": hashlib.sha256(a.output.read_bytes()).hexdigest(),
        "frame_provenance": sampled,
        "captions": "Editorial explanation grounded in actual mission receipts; not a verbatim Agent transcript.",
    }
    a.output.with_suffix(".json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n"
    )


if __name__ == "__main__":
    main()
