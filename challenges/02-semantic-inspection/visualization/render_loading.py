"""Render verified loading inspection from actual synchronized frames and cloud receipts."""

import argparse
import bisect
import hashlib
import json
from pathlib import Path
import subprocess
import numpy as np
from PIL import Image, ImageDraw, ImageFont

FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
W, H, FPS, SECONDS = 2560, 1440, 8, 90


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--attempt", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--ffmpeg", required=True)
    a = p.parse_args()
    read = lambda p: json.loads(p.read_text())
    report, run = read(a.attempt / "acceptance.json"), read(a.attempt / "result.json")
    recording = read(a.attempt / "recording-acceptance.json")
    if (
        report["status"] != "PASS"
        or run["status"] != "PASS"
        or recording["status"] != "PASS"
    ):
        raise ValueError("Physical, independent-case and recording gates must all pass")
    root = Path(run["physics_directory"]) / "baseline-frames"
    frames = [
        json.loads(s) for s in (root / "timestamps.jsonl").read_text().splitlines()
    ]
    stamps = [r["wall_time"] for r in frames]
    start, end = recording["native_wall_time"]
    visits = report["visits"]
    clouds = []
    for f in (a.attempt / "native/semantic-evidence").glob("*cloud.json"):
        d = read(f)
        clouds.append((d["independent_physics"]["wall_time"], d, f))
    clouds.sort(key=lambda item: item[0])
    trajectory = []
    for f in (a.attempt / "native/actions").glob("*.json"):
        trajectory.extend(read(f).get("trajectory", []))
    trajectory.sort(key=lambda r: r["wall_time"])
    fonts = {n: ImageFont.truetype(FONT, n) for n in (24, 28, 32, 36, 40, 48, 54)}

    def text(draw, xy, value, size=32, color="#d9e4f0"):
        draw.text(xy, value, font=fonts[size], fill=color)

    def wrap(draw, value, xy, width, size=28, max_lines=4, color="#d9e4f0"):
        line, lines = "", []
        for char in value:
            if char == "\n" or draw.textlength(line + char, font=fonts[size]) > width:
                lines.append(line)
                line = "" if char == "\n" else char
            else:
                line += char
        if line:
            lines.append(line)
        for i, line in enumerate(lines[:max_lines]):
            text(draw, (xy[0], xy[1] + i * (size + 12)), line, size, color)

    a.output.parent.mkdir(parents=True, exist_ok=True)
    process = subprocess.Popen(
        [
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
        ],
        stdin=subprocess.PIPE,
    )
    last_index, sources, selected = None, {}, {}
    for index in range(FPS * SECONDS):
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
        selected[fi] = row
        image = Image.new("RGB", (W, H), "#0c1421")
        draw = ImageDraw.Draw(image)
        text(draw, (24, 16), "ROSClaw · 仓库装卸区自主安全巡查", 54, "#ffffff")
        text(
            draw,
            (26, 88),
            f"Isaac Sim + ROS 2 / Nav2  |  单次自然语言任务输入  |  独立重置 {a.attempt.name}",
            28,
            "#70d8cf",
        )
        for name, box, title in [
            ("follow", (24, 148, 1468, 826), "第三人称 · 实际机器人运动"),
            ("robot", (1516, 148, 1020, 574), "机器人前向 · 展示相机"),
            ("top-close", (1516, 758, 1020, 574), "近顶视 · 机器人与真实场景"),
        ]:
            x, y, w, h = box
            image.paste(sources[name].resize((w, h), Image.Resampling.LANCZOS), (x, y))
            draw.rectangle((x, y, x + w, y + 45), fill="#122236")
            text(draw, (x + 12, y + 3), title, 28, "#ffffff")
        draw.rounded_rectangle((24, 1000, 1492, 1332), 18, fill="#172539")
        measured = [item for item in clouds if item[0] <= wall]
        if measured:
            _, cloud, _ = measured[-1]
            summary = cloud["summary"]
            reg = summary["region"]
            lo = np.array(reg["min"]) - 1.1
            hi = np.array(reg["max"]) + 1.1

            def xy(point):
                u = (np.array(point) - lo) / (hi - lo)
                return (40 + int(u[0] * 400), 1310 - int(u[1] * 270))

            draw.rectangle((40, 1038, 440, 1310), fill="#0b121e")
            r = summary["thresholds"]["resolution_m"]
            for field, color in [
                ("free_cells", "#205e62"),
                ("occupied_cells", "#f1ae48"),
            ]:
                for cell in summary[field]:
                    low = np.array(reg["min"]) + np.array(cell) * r
                    high = np.minimum(low + r, reg["max"])
                    x0, y1 = xy(low)
                    x1, y0 = xy(high)
                    draw.rectangle((x0, y0, x1, y1), fill=color)
            points = np.array(cloud["snapshot"]["frames"][-1]["points_map"])
            keep = (
                (points[:, 2] >= 0.1)
                & (points[:, 2] <= 0.8)
                & np.all(points[:, :2] >= lo, axis=1)
                & np.all(points[:, :2] <= hi, axis=1)
            )
            for point in points[keep, :2][::2]:
                x, y = xy(point)
                draw.point((x, y), fill="#ffe59b")
            x0, y1 = xy(reg["min"])
            x1, y0 = xy(reg["max"])
            draw.rectangle((x0, y0, x1, y1), outline="#b3f9ef", width=3)
            text(draw, (42, 1006), "实际点云 + 已观测区域", 24, "#ffffff")
            title = f"实测 {summary['result']}  ·  覆盖 {summary['coverage_ratio']:.1%}"
            detail = f"已完成 {len(measured)} 次观察；本次新增 {summary['new_observed_cells']} 个有效网格。"
            detail2 = f"额外障碍：{'检测到' if summary['extra_obstacle_detected'] else '未检测到'}；边界也按完整足迹检查。"
        else:
            text(draw, (42, 1070), "等待实际观测", 32, "#849ab2")
            title = "一次任务输入 → 理解目标并选择安全观察点"
            detail = "“帮我检查一下货架前的装卸区域。”"
            detail2 = "重点查看货架旁叉车；自主选点，证据不足则补充观察。"
        text(draw, (472, 1014), title, 36, "#ffffff")
        wrap(draw, detail, (474, 1070), 990, 28, 2)
        wrap(draw, detail2, (474, 1150), 990, 28, 2)
        done = sum(v["finished_wall_time"] <= wall for v in visits)
        phase = "受控导航 / 等待实际反馈"
        if done and done < len(visits):
            phase = "读取实测反馈 → 有限次补充观察 / 返回实际起点"
        if done == len(visits):
            phase = "已安全返回；校验收据 → 保存 Memory → 结束任务"
        text(draw, (474, 1242), phase, 28, "#70d8cf")
        if fraction == 1:
            text(
                draw,
                (42, 1350),
                f"独立验收 PASS  |  {len(report['observation_views'])} 次观察 + 返回  |  零非地面接触  |  Memory → TaskKernel SUCCEEDED",
                32,
                "#8ce5ac",
            )
        else:
            text(
                draw,
                (24, 1352),
                "检查范围：离地 0.10–0.80m；未知不冒充畅通。RGB 三视角用于展示，未进入识别链。",
                28,
                "#b3c5d7",
            )
        speed = (end - start) / (SECONDS - 14)
        text(
            draw,
            (24, 1400),
            f"SIMULATION  |  来源墙钟 {wall - start:.1f}/{end - start:.1f}s  |  中段墙钟加速 ×{speed:.2f}  |  输出 {FPS}fps  |  无插帧",
            24,
            "#829ab1",
        )
        process.stdin.write(image.tobytes())
    process.stdin.close()
    if process.wait():
        raise RuntimeError("ffmpeg failed")
    gaps = np.diff(stamps)
    sim_ratio = (frames[-1]["sim_time"] - frames[0]["sim_time"]) / (
        stamps[-1] - stamps[0]
    )
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    provenance = {
        "attempt": a.attempt.name,
        "physical_acceptance_sha256": sha(a.attempt / "acceptance.json"),
        "recording_index_sha256": sha(root / "timestamps.jsonl"),
        "output_sha256": sha(a.output),
        "output_seconds": SECONDS,
        "encoded_fps": FPS,
        "capture_median_fps": float(1 / np.median(gaps)),
        "capture_max_gap_seconds": float(gaps.max()),
        "simulation_to_wall_ratio": sim_ratio,
        "wall_speedup_middle": speed,
        "sim_speedup_middle": sim_ratio * speed,
        "opening_hold_seconds": 6,
        "ending_hold_seconds": 8,
        "interpolation": False,
        "scope": "Actual synchronized rendered views; actual raw measured cloud and measured masks held until next observation. No RGB/VLM inference, no Ground Truth shown as perception. No raw model reasoning.",
        "cloud_sources": [
            {"file": str(f.relative_to(a.attempt)), "sha256": sha(f)}
            for _, _, f in clouds
        ],
        "selected_frame_sources": [
            {
                "index": i,
                "wall_time": r["wall_time"],
                "views": {
                    name: {"file": v["frame"], "sha256": sha(root / v["frame"])}
                    for name, v in r["views"].items()
                },
            }
            for i, r in selected.items()
        ],
    }
    a.output.with_suffix(".provenance.json").write_text(
        json.dumps(provenance, indent=2) + "\n"
    )
    print(
        json.dumps(
            {
                k: v
                for k, v in provenance.items()
                if k not in ["selected_frame_sources", "cloud_sources"]
            }
        )
    )


if __name__ == "__main__":
    main()
