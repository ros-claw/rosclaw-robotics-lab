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


def captions(capture, model_dir, events_path):
    frames = read_rows(capture / "frames.jsonl")
    start = int(frames[0]["host_mono_ns"])
    end = int(frames[-1]["host_mono_ns"])
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
        by_name = {event["event"]: event for event in events}
        for event, label in (
                ("intent_committed", "完整路径和门区已校验 · 提交 FollowPath"),
                ("nav2_accepted", "Nav2 已接受 · 跟踪校验后的路径"),
                ("odometry_stopped", "里程计确认低速驻留 · PhysX 独立验收见回放")):
            if event in by_name:
                labels.append((int(by_name[event]["host_mono_ns"]), label))
        verification = model_dir / "physx-evaluation.json"
        if verification.exists():
            result = json.loads(verification.read_text())
            if result["pass"] and "verified_host_mono_ns" in result:
                labels.append((int(result["verified_host_mono_ns"]),
                               f"PhysX 证实穿过 {gate} 并停稳 · 非地面接触 0"))
    labels.sort()
    # Group near-simultaneous circuit events without showing a later fact early.
    output = []
    for i, (when, caption) in enumerate(labels):
        following = labels[i + 1][0] if i + 1 < len(labels) else end
        if following - when < 250_000_000:
            continue
        output.append(f"{len(output)+1}\n{srt_stamp(when-start)} --> "
                      f"{srt_stamp(following-start)}\n{caption}\n")
    return "\n".join(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("model_dir", type=Path)
    parser.add_argument("--events", type=Path)
    args = parser.parse_args()
    caption = captions(args.capture, args.model_dir, args.events)
    with (args.capture / "live.srt").open("x") as stream:
        stream.write(caption)
    subprocess.run(["ffmpeg", "-nostdin", "-n", "-hide_banner", "-loglevel", "error",
                    "-i", "isaac-live-1x.mp4", "-vf",
                    "subtitles=live.srt:force_style='FontName=Noto Sans CJK SC,FontSize=19,Outline=2,MarginV=15'",
                    "-fps_mode", "vfr", "-c:v", "libx264", "-threads", "2", "-crf", "18",
                    "-movflags", "+faststart", "isaac-live-captioned.mp4"],
                   cwd=args.capture, check=True)


if __name__ == "__main__":
    main()
