"""One real Native mission: synchronized cameras, visible PTY and physical proof.

Editorial captions are separate from actual terminal output. No private thinking
is exported; no synthetic scene, invented tool calls or motion interpolation.
"""

import argparse
import base64
import bisect
import codecs
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import subprocess

from PIL import Image, ImageDraw, ImageFont
import pyte
import yaml

FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
MONO = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"
W, H = 2560, 1440
NAMES = {"entry": "入口", "shelf": "货架区", "aisle": "通道", "home": "Home"}


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def wrap(text, font, width):
    lines, current = [], ""
    for char in text:
        if char == "\n" or font.getlength(current + char) > width:
            lines.append(current)
            current = "" if char == "\n" else char
        else:
            current += char
    if current:
        lines.append(current)
    return lines


def visible_tools(root):
    source = max((root / "home/agent/sessions").glob("*.jsonl"), key=lambda p: p.stat().st_mtime)
    result = []
    for row in rows(source):
        message = row.get("message", {})
        if message.get("role") != "assistant":
            continue
        # Entry timestamp is when the SDK persisted the emitted message; the
        # message timestamp can instead mark inference start several seconds ago.
        wall = datetime.fromisoformat(row["timestamp"].replace("Z", "+00:00")).timestamp()
        for item in message.get("content", []):
            if item.get("type") != "toolCall":
                continue
            args = item.get("arguments", {})
            result.append({"wall_time": wall, "name": item["name"], "arguments": {"site_id": args["site_id"]} if "site_id" in args else {}})
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--directory", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--map", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ffmpeg", required=True)
    parser.add_argument("--mode", choices=["promo", "tutorial"], default="promo")
    args = parser.parse_args()
    root = args.directory
    settings = json.loads((root / "home/agent/settings.json").read_text())
    if settings.get("hideThinkingBlock") is not True:
        raise ValueError("Private thinking must be hidden in the actual recorded TUI")
    report = json.loads(args.report.read_text())
    if report["status"] != "PASS":
        raise ValueError("Only a physically verified complete Native mission can be promoted")
    if (report.get("obstacle_verification") or {}).get("status") != "PASS":
        raise ValueError("This obstacle tutorial requires the independent obstacle contract")
    config = json.loads((root / "execution_config.json").read_text())
    physics = Path(config["physics_directory"])
    frames = rows(physics / "baseline-frames/timestamps.jsonl")
    for row in frames:
        views = row.get("views", {})
        if set(views) != {"follow", "robot", "top-close"}:
            raise ValueError("Three real camera views required")
        ids = [view["render_frame"] for view in views.values()]
        if None in ids or len(set(ids)) != 1:
            raise ValueError("Camera images must have the same actual renderer frame ID")
    samples = rows(physics / "physics-trajectory.jsonl")
    pty_events = rows(root / "native-pty.events.jsonl")
    tools = visible_tools(root)
    if len(tools) != report["tool_call_count"]:
        raise ValueError("Visible SDK tool call count must match Native acceptance")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    (args.output.parent / "multiview-visible-tools.json").write_text(json.dumps(tools, ensure_ascii=False, indent=2) + "\n")
    visits = sorted(report["visits"], key=lambda item: item["started_wall_time"])
    start, end = pty_events[0]["wall_time"], pty_events[-1]["wall_time"]
    if frames[0]["wall_time"] > start or frames[-1]["wall_time"] < end:
        raise ValueError("All three camera streams must cover the entire Native interval")
    closed = datetime.fromisoformat(report["task_kernel"]["updated_at"]).timestamp()
    if not visits[-1]["finished_wall_time"] < closed <= end:
        raise ValueError("Final task completion must follow physical arrival")
    ft = [row["wall_time"] for row in frames]
    st = [row["wall_time"] for row in samples]
    intro, outro = 10.0, 12.0
    fps = 12 if args.mode == "promo" else 2
    run_duration = 158.0 if args.mode == "promo" else end - start
    duration = intro + run_duration + outro
    first = min(45.0, visits[0]["started_wall_time"] - start + 8)
    last = min(30.0, end - closed + 12)
    if args.mode == "promo":
        middle = run_duration - first - last
        speed = (end - start - first - last) / middle
        segments = [(first, start, 1.0), (middle, start + first, speed), (last, end - last, 1.0)]
    else:
        segments = [(run_duration, start, 1.0)]

    def source_time(movie_time):
        offset = min(run_duration, max(0, movie_time - intro))
        for length, wall_start, compression in segments:
            if offset <= length:
                return wall_start + offset * compression, compression
            offset -= length
        return end, 1.0

    font = ImageFont.truetype(FONT, 28)
    small = ImageFont.truetype(FONT, 22)
    title = ImageFont.truetype(FONT, 42)
    large = ImageFont.truetype(FONT, 38)
    mono = ImageFont.truetype(MONO, 24)
    terminal_cjk = ImageFont.truetype(FONT, 24)
    map_font = ImageFont.truetype(FONT, 16)
    occupancy = Image.open(args.map).convert("RGB")
    map_w, map_h = occupancy.size
    info = yaml.safe_load(args.map.with_suffix(".yaml").read_text())
    occupancy.thumbnail((410, 250))
    map_x, map_y = 1110 + (430 - occupancy.width) // 2, 1030

    def point(x, y):
        return (map_x + (x - info["origin"][0]) / info["resolution"] * occupancy.width / map_w,
                map_y + (map_h - (y - info["origin"][1]) / info["resolution"]) * occupancy.height / map_h)

    screen = pyte.Screen(80, 24)
    stream = pyte.Stream(screen)
    decoder = codecs.getincrementaldecoder("utf-8")("replace")
    event_index = 0
    cached_index, pictures = None, {}
    count = math.ceil(duration * fps)
    encoder = subprocess.Popen([
        args.ffmpeg, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
        "-s", f"{W}x{H}", "-r", str(fps), "-i", "-", "-an", "-c:v", "libx264",
        "-preset", "fast", "-crf", "19", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(args.output),
    ], stdin=subprocess.PIPE)
    # The drawing loop below uses only recorded public text, actual camera PNGs
    # and independent measurements. Editorial explanations are labelled.
    try:
        for index in range(count):
            movie_time = index / fps
            wall, compression = source_time(movie_time)
            fi = max(0, bisect.bisect_right(ft, wall) - 1)
            si = max(0, bisect.bisect_right(st, wall) - 1)
            sample = samples[si]
            frame = frames[fi]
            while event_index < len(pty_events) and pty_events[event_index]["wall_time"] <= wall:
                stream.feed(decoder.decode(base64.b64decode(pty_events[event_index]["data"])))
                event_index += 1
            if fi != cached_index:
                for name, size in [("follow", (1536, 864)), ("robot", (960, 540)), ("top-close", (960, 540))]:
                    with Image.open(physics / "baseline-frames" / frame["views"][name]["frame"]) as source:
                        pictures[name] = source.convert("RGB").resize(size, Image.Resampling.LANCZOS)
                cached_index = fi
            canvas = Image.new("RGB", (W, H), "#0c1522")
            canvas.paste(pictures["follow"], (16, 120))
            canvas.paste(pictures["robot"], (1580, 120))
            canvas.paste(pictures["top-close"], (1580, 700))
            d = ImageDraw.Draw(canvas)
            completed = [v for v in visits if v["finished_wall_time"] <= wall and v["success"]]
            current = next((v for v in visits if v["started_wall_time"] <= wall < v["finished_wall_time"]), None)
            if wall < visits[0]["started_wall_time"]:
                chapter, explanation = "01 一句话下达任务", "讲解：真实 Native Agent 接入 ROS、发现能力并提出动作"
            elif wall < visits[-1]["finished_wall_time"]:
                chapter = "02 自主巡检与返回"
                explanation = "讲解：ROSClaw 编排与核验，Nav2 根据实际激光执行导航"
            elif wall < closed:
                chapter, explanation = "03 检查结果与保存记忆", "讲解：到点之后仍需完成整项物理验收与 Memory 收据"
            else:
                chapter, explanation = "04 任务完成与证据", "讲解：TaskKernel 在最终成功记忆收据之后完成"
            d.text((24, 12), "ROSClaw｜一句话，让机器人完成仓库巡检", font=title, fill="white")
            d.text((24, 70), f"{chapter}  ·  {explanation}", font=small, fill="#acd3ed")
            d.text((1880, 26), "同一任务 · 三视角同步", font=font, fill="#70d9f5")
            d.text((1810, 75), "DGX Spark / Isaac Sim 6.1 / ROS 2 Jazzy", font=small, fill="#bdcad8")
            d.rectangle((16, 120, 1552, 164), fill="#081321")
            d.text((32, 124), "第三人称｜真实机器人运动", font=font, fill="white")
            d.rectangle((1580, 120, 2540, 164), fill="#081321")
            d.text((1596, 124), "机器人前向视角｜展示相机", font=font, fill="white")
            d.text((1596, 664), "跟随顶视｜12 米取景，观察周围障碍", font=font, fill="white")
            if intro <= movie_time < intro + run_duration and (wall < visits[0]["started_wall_time"] or wall >= visits[-1]["finished_wall_time"]):
                d.rectangle((16, 120, 1552, 984), fill="#09111c")
                d.text((32, 126), ("真实 ROSClaw 终端｜结果与记忆" if wall >= visits[-1]["finished_wall_time"] else "真实 ROSClaw 终端｜本轮可见输出重放"), font=font, fill="#70d9f5")
                for line in range(24):
                    for column, char in screen.buffer[line].items():
                        if char.data and char.data != " ":
                            d.text((42 + column * 18, 180 + line * 31), char.data,
                                   font=terminal_cjk if any(ord(c) > 127 for c in char.data) else mono,
                                   fill="#e3ecf7")
            d.rounded_rectangle((16, 1002, 1090, 1275), radius=12, fill="#132339")
            d.text((34, 1010), "真实 Agent 工具调用｜从实际 SDK 记录提取", font=font, fill="#70d9f5")
            visible = [event for event in tools if event["wall_time"] <= wall][-3:]
            for line, event in enumerate(visible):
                site = event["arguments"].get("site_id")
                text = event["name"] + ("  → " + NAMES.get(site, site) if site else "")
                d.text((34, 1058 + line * 43), text, font=font, fill="white")
            if not visible:
                d.text((34, 1060), "等待实际工具调用；此处不会预填任务计划。", font=font, fill="#acbdd0")
            active = "Nav2 正在执行：" + NAMES[current["site_id"]] if current else "Agent 检查状态与结果"
            if wall >= closed:
                active = "Memory: SUCCESS    TaskKernel: SUCCEEDED"
            d.text((34, 1226), active, font=font, fill="#86eda9" if wall >= closed else "#ffcd75")
            d.rounded_rectangle((1102, 1002, 1552, 1275), radius=12, fill="#132339")
            canvas.paste(occupancy, (map_x, map_y))
            d = ImageDraw.Draw(canvas)
            trace = [point(*row["physics_transforms_xyzw"][0][:2]) for row in samples[:si + 1:4]]
            if len(trace) > 1:
                d.line(trace, fill="#1585ff", width=3)
            for name, site in config["sites"].items():
                x, y = point(site["x"], site["y"])
                d.ellipse((x - 4, y - 4, x + 4, y + 4), fill="#f76b24")
                d.text((x + 5, y - 6), NAMES[name], font=map_font, fill="#db4018")
            x, y = point(*sample["physics_transforms_xyzw"][0][:2])
            d.ellipse((x - 5, y - 5, x + 5, y + 5), fill="#00b85d")
            d.text((1116, 1006), "独立 PhysX 实际轨迹", font=small, fill="white")
            frame_id = frame["views"]["follow"]["render_frame"]
            d.text((1596, 1250), f"三相机捕获同步帧：{frame_id}  ·  非独立轮次拼接", font=small, fill="#a9c4db")
            for step, visit in enumerate(visits):
                x = 24 + step * 390
                passed = visit in completed
                selected = current is visit
                color = "#1b5a41" if passed else "#664823" if selected else "#1a2b40"
                d.rounded_rectangle((x, 1292, x + 372, 1350), radius=9, fill=color)
                text = f"{step + 1} {NAMES[visit['site_id']]}" + (f"  ✓ {visit['position_error_m']:.3f} m" if passed else "  执行中" if selected else "  待验收")
                d.text((x + 12, 1300), text, font=font, fill="white")
            d.text((1610, 1300), f"已验收 {len(completed)}/4  ·  非地面碰撞 {sample['collision_count']}", font=font, fill="#86eda9")
            elapsed = wall - start
            pace = f"导航段 {compression:.2f}× 墙钟压缩" if compression != 1 else "1× 真实墙钟时间"
            if movie_time < intro:
                pace = "片头：本轮真实场景定格"
            elif movie_time >= intro + run_duration:
                pace = "片尾：本轮真实结束画面定格"
            d.text((24, 1370), f"任务墙钟 {elapsed:.1f}s  |  SIM {sample['sim_time']:.2f}s  |  {pace}  |  实际仿真帧 + 可见终端重放", font=small, fill="#b7c9dc")
            d.text((1700, 1370), "ROSClaw Robotics Lab · SIM 验证", font=small, fill="#70d9f5")
            if movie_time < intro:
                panel = Image.new("RGB", (1450, 500), "#10243a")
                pd = ImageDraw.Draw(panel)
                pd.text((36, 24), "本轮实际提交的唯一任务指令", font=title, fill="#70d9f5")
                for line, text in enumerate(wrap(config["task"], large, 1370)):
                    pd.text((36, 115 + line * 58), text, font=large, fill="white")
                pd.text((36, 410), "真实 Native Agent · 逐站提案 · 本轮 SIM 动作按测试策略审批", font=font, fill="#b8d1e7")
                canvas.paste(panel, (58, 310))
            if movie_time >= intro + run_duration:
                panel = Image.new("RGB", (1450, 720), "#10243a")
                pd = ImageDraw.Draw(panel)
                pd.text((36, 25), "本轮任务完成｜独立物理验收 PASS", font=title, fill="#86eda9")
                maximum = max(v["position_error_m"] for v in visits)
                pd.text((36, 102), f"4/4 站点到达并返回 Home   最大位置误差 {maximum:.3f} m", font=large, fill="white")
                pd.text((36, 165), "零有效非地面碰撞 · 未映射箱体证据通过", font=large, fill="white")
                for step, visit in enumerate(visits):
                    pd.text((36, 240 + step * 48), f"{NAMES[visit['site_id']]}：{visit['position_error_m']:.3f} m｜稳定停留 {visit['stable_dwell_sim_seconds']:.2f} SIM 秒", font=font, fill="#d6e5f3")
                pd.text((36, 452), "Memory: SUCCESS     TaskKernel: SUCCEEDED", font=large, fill="#86eda9")
                pd.text((36, 520), "独立 PhysX + 真实 LiDAR + Nav2 Action + 执行收据", font=font, fill="#70d9f5")
                pd.text((36, 572), "视频为真实帧与终端记录重建；展示相机不作为 Agent 视觉输入。", font=small, fill="#bccfe1")
                pd.text((36, 638), "github.com/ros-claw/rosclaw-robotics-lab", font=font, fill="white")
                canvas.paste(panel, (58, 200))
            if index == int(duration * fps * 0.35):
                canvas.save(args.output.with_suffix(".png"))
            encoder.stdin.write(canvas.tobytes())
            if index % 120 == 0:
                print(json.dumps({"frame": index, "total": count, "mode": args.mode}), flush=True)
    finally:
        encoder.stdin.close()
        code = encoder.wait()
    if code:
        raise RuntimeError(f"Encoding failed: {code}")
    args.output.with_suffix(".json").write_text(json.dumps({
        "run_id": root.name, "mode": args.mode, "duration_seconds": count / fps,
        "resolution": [W, H], "fps": fps, "native_wall_interval": [start, end],
        "intro_hold_seconds": intro, "outro_hold_seconds": outro,
        "timeline_segments": [{"video_seconds": length, "source_wall_start": ws, "wall_time_compression": c} for length, ws, c in segments],
        "synchronization": "All three actual capture renderer frame IDs match in every recorded group; SWH IDs unavailable",
        "source": "one physically verified real Native mission; three viewport PNG streams; visible PTY replay; independent PhysX",
        "no_motion_interpolation": True, "visible_private_thinking": False, "audio": False,
        "acceptance_report_sha256": hashlib.sha256(args.report.read_bytes()).hexdigest(),
        "renderer_source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    }, ensure_ascii=False, indent=2) + "\n")


if __name__ == "__main__":
    main()
