"""Caption a single 1× live recording using event and camera wall timestamps."""

import argparse
import json
from pathlib import Path
import subprocess


def read_rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def srt_stamp(ns):
    seconds, millis = divmod(max(ns, 0) // 1_000_000, 1000)
    minutes, sec = divmod(seconds, 60)
    hours, minute = divmod(minutes, 60)
    return f"{hours:02d}:{minute:02d}:{sec:02d},{millis:03d}"


def luna_labels(start, model_dir, events_path):
    reply = json.loads((model_dir / "response.json").read_text())
    request = int(reply["request_host_mono_ns"])
    received = int(reply["received_host_mono_ns"])
    labels = [(start, "Isaac Sim · 独立展示相机 · 墙钟 1×"),
              (request, "Luna 正在读取车载图像 · 车辆驻车")]
    if reply.get("intent", {}).get("decision") == "select_candidate":
        gate = reply["intent"]["candidate_id"]
        labels.append((received, f"Luna 提议 {gate} · {reply['intent']['reason_summary']}"))
    else:
        labels.append((received, "Luna 请求超时 · 保持驻车" if reply.get("failure") in
                       {"transport_curl_28", "transport_deadline"} else "本次视觉请求结束 · 保持驻车"))
    if events_path:
        events = read_rows(events_path)
        by_name = {}
        for event in events:
            by_name.setdefault(event["event"], event)
        for event, label in (
                ("intent_committed", "完整路径和门区已校验 · 提交 FollowPath"),
                ("nav2_accepted", "Nav2 已接受 · 跟踪校验后的路径"),
                ("odometry_stopped", "里程计确认低速驻留 · PhysX 独立验收见回放")):
            if event in by_name:
                labels.append((int(by_name[event]["host_mono_ns"]), label))
        for update in (e for e in events if e["event"] == "nav2_accepted" and e["path_seq"] > 1):
            labels.append((int(update["host_mono_ns"]),
                           f"Nav2 接续路径 {update['path_seq']} · 保持已承诺通道 {gate}"))
        verification = model_dir / "physx-evaluation.json"
        if verification.exists():
            result = json.loads(verification.read_text())
            if result["pass"] and "verified_host_mono_ns" in result:
                labels.append((int(result["verified_host_mono_ns"]),
                               f"PhysX 证实穿过 {gate} 并停稳 · 非地面接触 0"))
    return labels


def handoff_labels(start, events_path):
    labels = [(start, "连续交接控制试验 · Isaac Sim 独立相机 · 1×")]
    for event in read_rows(events_path):
        kind = event["event"]
        stamp = int(event["host_mono_ns"])
        if kind == "nav2_accepted":
            seq = event["path_seq"]
            text = "Nav2 接受初始路径 1" if seq == 1 else f"Nav2 接受更新路径 {seq} · 保持同一通道"
        elif kind == "cancel_acknowledged":
            text = "Nav2 确认取消活动路径"
        elif kind == "odometry_stopped":
            text = "里程计确认低速驻留 · 实际过门与停车由 PhysX 复核"
        else:
            continue
        labels.append((stamp, text))
    return labels


def concise(text):
    return " ".join(text.split())[:48]


def picture_windows(pictures, start):
    windows = []
    pictures = sorted(pictures)
    for i, (when, path) in enumerate(pictures):
        later = pictures[i+1][0] if i+1 < len(pictures) else when + 4_000_000_000
        lo = max(0, (when - start) / 1e9)
        hi = (min(later, when + 4_000_000_000) - start) / 1e9
        if hi > lo:
            windows.append((lo, hi, path))
    return windows


def mission_labels(start, mission, audit):
    session = read_rows(mission)
    labels = [(start, "Luna 动态仓库 · Isaac Sim 实况 / 墙钟 1×")]
    pictures = []
    requests = {e["observation_id"]: Path(e["model_dir"]) for e in session
                if e["event"] == "request"}
    flows = {Path(e["controller"]) for e in session if "controller" in e}
    for event in session:
        when = int(event["host_mono_ns"])
        kind = event["event"]
        if kind == "request":
            pictures.append((when, Path(event["model_dir"]) / "model-input.png"))
            labels.append((when, "Luna 读取车载相机 · 车辆驻车"))
        elif kind == "model_returned" and event["decision"] == "select_candidate":
            labels.append((when, f"Luna 提议通道 {event['candidate_id']} · {concise(event['reason_summary'])}"))
        elif kind == "physical_stop_verified":
            labels.append((when, f"PhysX 确认停车 · 通道 {event['gate']} 暂不可通行"))
        elif kind == "route_completed":
            folder = list(requests.values())[-1]
            result = json.loads((folder / "physx-evaluation.json").read_text())
            if result["pass"]:
                labels.append((when, f"PhysX 验收通过 · {event['gate']} · 终点误差 "
                               f"{result['physical_distance_to_terminal_m']:.2f} m · 非地面接触 0"))
        elif kind == "hold":
            labels.append((when, "任务进入 HOLD · 车辆驻车"))
    for path in flows:
        for event in read_rows(path):
            kind = event["event"]
            if kind == "nav2_accepted":
                text = f"Nav2 接受已校验路径 · 路线版本 {event['route_version']}"
            elif kind == "route_blocked":
                text = ("视觉复核扩大检查 · 代价地图确认封堵" if event.get("visual_review")
                        and not event.get("standard_blocked") else "本地代价地图确认封堵 · 执行停车")
            elif kind == "alignment_completed":
                text = "车辆完成原地对正 · 重新规划替代路线"
            elif kind == "odometry_stopped":
                text = "车辆完成低速驻留 · 等待独立物理验收"
            else:
                continue
            labels.append((int(event["host_mono_ns"]), text))
    if audit:
        shown_review = False
        for event in read_rows(audit):
            if event["event"] != "audit_returned" or not event["current_goal"]:
                continue
            when = int(event["host_mono_ns"])
            folder = Path(event["model_dir"])
            reply = json.loads((folder / "response.json").read_text())
            term = {"review": "建议本地复核", "continue": "继续观察", "reobserve": "重新观察"}[event["decision"]]
            labels.append((when, f"行驶中 Luna · {term} · {reply['elapsed_seconds']:.2f} s"))
            if event["decision"] == "review" and not shown_review:
                shown_review = True
                pictures.append((when, folder / "model-input.png"))
    return labels, picture_windows(pictures, start)


def captions(capture, model_dir, events_path, mission=None, audit=None):
    frames = read_rows(capture / "frames.jsonl")
    start = int(frames[0]["host_mono_ns"])
    end = int(frames[-1]["host_mono_ns"])
    if mission:
        labels, pictures = mission_labels(start, mission, audit)
    else:
        labels = (luna_labels(start, model_dir, events_path) if model_dir
                  else handoff_labels(start, events_path))
        pictures = []
    final = next(((when, label) for when, label in labels if label.startswith("PhysX 验收通过")), None)
    freeze = bool(mission and final and final[0] <= end)
    if freeze:
        labels.append((end, "终点画面定格 · " + final[1]))
        end += 2_500_000_000
    labels.sort()
    # Group near-simultaneous circuit events without showing a later fact early.
    output = []
    for i, (when, caption) in enumerate(labels):
        following = labels[i + 1][0] if i + 1 < len(labels) else end
        if following - when < 250_000_000:
            continue
        cutoff = min(following, when + 3_500_000_000) if caption.startswith("行驶中 Luna") else following
        output.append(f"{len(output)+1}\n{srt_stamp(when-start)} --> "
                      f"{srt_stamp(cutoff-start)}\n{caption}\n")
        if cutoff < following - 250_000_000:
            output.append(f"{len(output)+1}\n{srt_stamp(cutoff-start)} --> "
                          f"{srt_stamp(following-start)}\nNav2 跟踪当前路径 · 本地避障持续运行\n")
    return "\n".join(output), pictures, freeze


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("model_dir", type=Path, nargs="?")
    parser.add_argument("--events", type=Path)
    parser.add_argument("--mission", type=Path)
    parser.add_argument("--audit", type=Path)
    args = parser.parse_args()
    if not (args.model_dir or args.events or args.mission):
        parser.error("Provide a mission log, model directory, or controller event stream")
    caption, pictures, freeze = captions(args.capture, args.model_dir, args.events, args.mission, args.audit)
    with (args.capture / "live.srt").open("x") as stream:
        stream.write(caption)
    command = ["ffmpeg", "-nostdin", "-n", "-hide_banner", "-loglevel", "error",
               "-i", "isaac-live-1x.mp4"]
    subtitle = ("subtitles=live.srt:force_style="
                "'FontName=Noto Sans CJK SC,FontSize=18,Outline=2,MarginV=15'")
    if freeze:
        subtitle = "tpad=stop_mode=clone:stop_duration=2.5," + subtitle
    if pictures:
        filters = []
        prior = "[0:v]"
        for index, (lo, hi, image) in enumerate(pictures, 1):
            command += ["-i", str(image.resolve())]
            filters.append(f"[{index}:v]scale=248:-2,pad=252:ih+4:2:2:color=white,"
                           "drawtext=font='Noto Sans CJK SC':text='车载模型输入':x=8:y=h-th-8:"
                           f"fontsize=13:fontcolor=white:box=1:boxcolor=black@0.75[p{index}]")
            filters.append(f"{prior}[p{index}]overlay=W-w-16:16:"
                           f"eof_action=repeat:enable='gte(t,{lo:.3f})*lt(t,{hi:.3f})'[o{index}]")
            prior = f"[o{index}]"
        filters.append(prior + subtitle + "[out]")
        command += ["-filter_complex_threads", "2", "-filter_complex", ";".join(filters), "-map", "[out]"]
    else:
        command += ["-vf", subtitle]
    command += ["-fps_mode", "vfr", "-enc_time_base", "1:1000",
                "-c:v", "libx264", "-threads", "2", "-crf", "18",
                "-movflags", "+faststart", "isaac-live-captioned.mp4"]
    subprocess.run(command, cwd=args.capture, check=True)



if __name__ == "__main__":
    main()
