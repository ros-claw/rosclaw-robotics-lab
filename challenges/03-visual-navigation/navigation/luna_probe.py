"""Probe Luna vision on a frozen Isaac RGB frame without any motion authority."""

import base64
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

DECISIONS = ["select_candidate", "hold", "reobserve"]
AUDIT_DECISIONS = ["continue", "review", "reobserve"]
TASK = "寻找蓝色 B 工位的标志。选择标志所在侧对应的通道。"
INSTRUCTIONS = (
    "你只提供视觉建议，不能驱动机器人。根据实际 RGB 图像和中性候选判断。"
    "隔板上并列的标牌按相对位置指示同侧通道。左牌指示左侧，右牌指示右侧。"
    "只要能辨认两块标牌及蓝色 B 在其中的位置，就可根据此规则选择，无需看到通道内部。"
    "只有清楚看见蓝色 B 标牌并能将其定位到某一侧时，才能选择该侧候选。"
    "证据不足时返回 reobserve，并将 candidate_id 设为 null。"
    "原因只写 80 个字以内的可见事实。evidence_regions 是图像归一化坐标的标牌框，最多三个。"
)


class ProbeError(ValueError):
    pass


def schema(observation_id, candidates, decisions=DECISIONS):
    return {
        "type": "object", "additionalProperties": False,
        "required": ["observation_id", "decision", "candidate_id", "reason_summary", "evidence_regions"],
        "properties": {
            "observation_id": {"type": "string", "enum": [observation_id]},
            "decision": {"type": "string", "enum": decisions},
            "candidate_id": {"type": ["string", "null"], "enum": [*candidates, None]},
            "reason_summary": {"type": "string"},
            "evidence_regions": {"type": "array", "items": {
                "type": "object", "additionalProperties": False,
                "required": ["bbox_xyxy_norm", "label"],
                "properties": {
                    "bbox_xyxy_norm": {"type": "array", "items": {"type": "number"}},
                    "label": {"type": "string"},
                },
            }},
        },
    }


def parse_intent(response, observation_id, candidates, decisions=DECISIONS):
    status = response.get("status")
    if status != "completed":
        raise ProbeError("response_incomplete" if status == "incomplete" else "response_not_completed")
    parts = [part for item in response.get("output", [])
             if item.get("type") == "message" and item.get("role") == "assistant"
             for part in item.get("content", [])]
    if any(part.get("type") == "refusal" for part in parts):
        raise ProbeError("model_refusal")
    texts = [part.get("text") for part in parts if part.get("type") == "output_text"]
    try:
        intent = json.loads(texts[0]) if len(texts) == 1 else None
    except (TypeError, ValueError):
        raise ProbeError("intent_json_invalid") from None
    if not isinstance(intent, dict) or set(intent) != set(schema(observation_id, [])["required"]):
        raise ProbeError("intent_fields_invalid")
    if intent["observation_id"] != observation_id:
        raise ProbeError("observation_id_mismatch")
    decision, candidate = intent["decision"], intent["candidate_id"]
    if decision not in decisions:
        raise ProbeError("decision_invalid")
    if decision == "select_candidate":
        if not isinstance(candidate, str) or candidate not in candidates:
            raise ProbeError("candidate_unknown")
    elif candidate is not None:
        raise ProbeError("candidate_unexpected")
    reason, regions = intent["reason_summary"], intent["evidence_regions"]
    if not isinstance(reason, str) or not 0 < len(reason) <= 80:
        raise ProbeError("reason_invalid")
    if not isinstance(regions, list) or len(regions) > 3:
        raise ProbeError("evidence_invalid")
    for region in regions:
        if (not isinstance(region, dict) or set(region) != {"label", "bbox_xyxy_norm"}
                or not isinstance(region["label"], str) or not region["label"].strip()):
            raise ProbeError("evidence_invalid")
        box = region["bbox_xyxy_norm"]
        if (not isinstance(box, list) or len(box) != 4
                or any(type(v) not in (int, float) or not 0 <= v <= 1 for v in box)
                or not (box[0] < box[2] and box[1] < box[3])):
            raise ProbeError("evidence_bounds_invalid")
    return intent


def load_frame(directory, frame_seq):
    records = [json.loads(line) for line in (directory / "observations.jsonl").read_text().splitlines()]
    matches = [item for item in records if item["frame_seq"] == frame_seq]
    if len(matches) != 1 or matches[0]["source"] != "isaac-live":
        raise ProbeError("expected_unique_live_frame")
    observation = matches[0]
    path = directory / observation["rgb"]["ref"]
    if path.resolve().parent != directory.resolve() or path.suffix != ".png":
        raise ProbeError("invalid_frame_reference")
    image = path.read_bytes()
    if hashlib.sha256(image).hexdigest() != observation["rgb"]["sha256"]:
        raise ProbeError("frame_hash_mismatch")
    return observation, image


def call_responses(payload, key, timeout):
    if any(c not in "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_" for c in key):
        raise ProbeError("invalid_credential_format")
    # Config and body are delivered through stdin. The command line stores no secret.
    config = ('header = "Authorization: Bearer ' + key + '"\n'
              'header = "Content-Type: application/json"\n')
    env = {name: value for name, value in os.environ.items() if name not in {
        "OPENAI_API_KEY", "SSLKEYLOGFILE",
    }}
    with os.fdopen(os.memfd_create("luna-request"), "w+b") as body_stream:
        body_stream.write(json.dumps(payload, ensure_ascii=False).encode("utf-8"))
        body_stream.seek(0)
        fd = body_stream.fileno()
        proc = subprocess.run(
            ["curl", "-q", "--silent", "--show-error", "--proto", "=https",
             "--max-time", str(timeout), "--config", "-", "--write-out", "\n%{http_code}",
             "--data-binary", f"@/proc/self/fd/{fd}", "https://api.openai.com/v1/responses"],
            input=config, text=True, capture_output=True, timeout=timeout + 3, env=env,
            pass_fds=(fd,),
        )
    if proc.returncode:
        raise ProbeError(f"transport_curl_{proc.returncode}")
    body, status = proc.stdout.rsplit("\n", 1)
    try:
        response = json.loads(body)
    except ValueError:
        raise ProbeError(f"invalid_response_http_{status}") from None
    if not isinstance(response, dict):
        raise ProbeError(f"invalid_response_http_{status}")
    if status != "200":
        error = response.get("error") or {}
        code = error.get("code")
        message = str(error.get("message", "")).lower()
        if "missing scopes" in message or "insufficient permissions" in message:
            raise ProbeError(f"http_{status}_insufficient_permissions")
        code = code if code in {"model_not_found", "invalid_api_key", "insufficient_quota",
                               "rate_limit_exceeded", "invalid_json_schema"} else "other"
        raise ProbeError(f"http_{status}_{code}")
    return response


def observation_id(observation):
    return f'{observation["episode_id"]}:{observation["epoch"]}:{observation["stamp_ns"]}'


def probe(directory, frame_seq, layout_path, destination, model, timeout, key, reasoning_effort="none",
          task=None, excluded=(), audit_gate=None):
    observation, image = load_frame(directory, frame_seq)
    layout = json.loads(layout_path.read_text())
    candidates = {gate["gate_id"]: "面向分叉时的左侧" if gate["lower_y"] > 0 else "面向分叉时的右侧"
                  for gate in layout["gates"]}
    labels = layout.get("candidate_labels", {})
    candidates = {gate: labels.get(gate, side) for gate, side in candidates.items() if gate not in excluded}
    if not candidates:
        raise ProbeError("no_eligible_candidate")
    instructions = INSTRUCTIONS if task is None else (
        "你只提供视觉建议。候选可供提出，最终仍需本地规划和安全校验。根据图像和任务选择明确允许的候选。"
        "看不清时返回 reobserve，无法满足任务时返回 hold。写出简短可见依据。")
    task = task or TASK
    decisions = DECISIONS
    if audit_gate is not None:
        direction = candidates.get(audit_gate, "已承诺通道方向")
        instructions = (
            "这是行驶中车载相机的图像。当前路线可向画面左侧或右侧绕行，并非沿画面中心直行。"
            "结合已承诺通道所属侧检查画面；该侧边缘也可能包括接下来要走的路径。"
            "若该侧附近可见大型直立挡板、移动物体或明显占用，请返回 review，让本地系统检查更远的路线。"
            "图像看起来正常时返回 continue，图像不可用时返回 reobserve。candidate_id 必须为 null。"
            "仅说明看得见的依据。你不能确认通过、选择另一条路线或控制车辆。")
        task = f"已承诺候选 {audit_gate} 对应{direction}；复核该路线方向是否出现可见的潜在遮挡。"
        candidates = {}
        decisions = AUDIT_DECISIONS
    image_id = observation_id(observation)
    payload = {
        "model": model, "store": False, "reasoning": {"effort": reasoning_effort},
        "instructions": instructions,
        "input": [{"role": "user", "content": [{
            "type": "input_text", "text": json.dumps({
                "task": task, "observation_id": image_id, "candidate_sides": candidates,
            }, ensure_ascii=False),
        }, {"type": "input_image", "detail": "high",
            "image_url": "data:image/png;base64," + base64.b64encode(image).decode("ascii")}]}],
        "text": {"format": {"type": "json_schema", "name": "luna_visual_intent", "strict": True,
                            "schema": schema(image_id, candidates, decisions)}},
        "max_output_tokens": 600,
    }
    destination.mkdir(parents=True, exist_ok=False)
    (destination / "model-input.png").write_bytes(image)
    manifest = {"source": "isaac-live-frozen-probe", "motion_authority": False,
                "model_requested": model, "purpose": "motion_audit" if audit_gate else "route_decision",
                "reasoning_effort": reasoning_effort, "input_sha256": observation["rgb"]["sha256"],
                "observation_id": image_id, "observation_stamp_ns": observation["stamp_ns"],
                "diagnostic_camera_yaw_deg": observation.get("diagnostic_camera_yaw_deg", 0.0),
                "source_capture": str(directory.resolve()), "candidate_sides": candidates,
                "task": task, "instructions": instructions, "schema": payload["text"]["format"]}
    (destination / "request.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
    start = time.monotonic()
    start_ns = time.monotonic_ns()
    result = {}
    try:
        response = call_responses(payload, key, timeout)
        result = {"response_id": response.get("id"), "model": response.get("model"),
                  "status": response.get("status"), "usage": response.get("usage")}
        (destination / "api-response.json").write_text(
            json.dumps(response, ensure_ascii=False, indent=2) + "\n")
        intent = parse_intent(response, image_id, candidates, decisions)
        result["intent"] = intent
        return intent
    except subprocess.TimeoutExpired:
        result["failure"] = "transport_deadline"
        raise ProbeError(result["failure"]) from None
    except ProbeError as exc:
        result["failure"] = str(exc)
        raise
    finally:
        result["request_host_mono_ns"] = str(start_ns)
        result["received_host_mono_ns"] = str(time.monotonic_ns())
        result["elapsed_seconds"] = time.monotonic() - start
        (destination / "response.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
