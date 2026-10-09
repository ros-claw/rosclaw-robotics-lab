"""Add labelled Chinese editorial narration to a verified multiview movie."""
import argparse
import array
import asyncio
import json
import shutil
import sys
import hashlib
from pathlib import Path
import subprocess
import wave
import edge_tts


def stamp(seconds):
    ms = round(seconds * 1000)
    return f"{ms//3600000:02}:{ms//60000%60:02}:{ms//1000%60:02},{ms%1000:03}"


async def main():
    p = argparse.ArgumentParser()
    p.add_argument('--video', type=Path, required=True)
    p.add_argument('--report', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--ffmpeg', required=True)
    p.add_argument('--piper-model', type=Path)
    a = p.parse_args()
    meta = json.loads(a.video.with_suffix('.json').read_text())
    report = json.loads(a.report.read_text())
    if report['status'] != 'PASS':
        raise ValueError('Verified mission required')
    def movie_time(wall):
        t = meta['intro_hold_seconds']
        for seg in meta['timeline_segments']:
            end = seg['source_wall_start'] + seg['video_seconds'] * seg['wall_time_compression']
            if wall <= end:
                return t + (wall-seg['source_wall_start']) / seg['wall_time_compression']
            t += seg['video_seconds']
        return t
    visits = sorted(report['visits'], key=lambda v:v['started_wall_time'])
    cues = [
        (0.8, '一句话，让机器人完成仓库巡检。这是同一轮真实仿真中的三视角演示。'),
        (10.5, '我们只提交一次任务：检查状态，依次巡检入口、货架区和通道，根据实际激光绕开箱体，最后返回 Home，并保存报告和记忆。'),
        (movie_time(visits[0]['started_wall_time']) + 2, '机器人开始前往入口。ROSClaw 负责发现能力、提出动作和检查结果，Nav2 负责执行导航。本轮仿真动作按测试策略自动审批。'),
        (movie_time(visits[1]['started_wall_time']) + 1, '接下来前往货架区。左侧观察机器人运动，右上查看前向画面，右下查看周边障碍。三路画面对应同一个渲染帧。'),
        (movie_time(visits[2]['started_wall_time']) + 1, '现在巡检仓库通道。箱体没有写进静态地图，导航使用真实激光和局部代价地图。展示相机本身不作为智能体的视觉输入。'),
        (movie_time(visits[3]['started_wall_time']) + 1, '巡检结束，机器人返回 Home。每一站都要求实际到达并稳定停留，工具说成功还不够，独立物理测量也必须通过。'),
        (meta['duration_seconds']-11.5, f"本轮四个站点全部通过，最大位置误差 {max(v['position_error_m'] for v in visits):.3f} 米，没有有效非地面碰撞。成功记忆保存后，整项任务才完成。"),
    ]
    cues.sort()
    voice_name = 'piper:zh_CN-huayan-medium' if a.piper_model else 'zh-CN-XiaoxiaoNeural'
    extension = '.wav' if a.piper_model else '.mp3'
    work = a.output.parent / (a.output.stem + ('-narration-local' if a.piper_model else '-narration'))
    work.mkdir(parents=True, exist_ok=True)
    rate = 24000
    audio = array.array('h', [0]) * int(meta['duration_seconds']*rate)
    subtitles = []
    records = []
    for i, (start, text) in enumerate(cues):
        clip = work/(f'{i:02}'+extension)
        # Reuse exactly matching editorial audio from the other movie version.
        if not clip.exists() or clip.stat().st_size == 0:
            for source in a.output.parent.glob('*.json'):
                prior = json.loads(source.read_text())
                if prior.get('voice') != voice_name:
                    continue
                for j, cue in enumerate(prior.get('narration_cues', [])):
                    candidate = a.output.parent/prior.get('narration_asset_directory', source.stem+'-narration')/(f'{j:02}'+extension)
                    if cue['text'] == text and candidate.exists() and candidate.stat().st_size:
                        shutil.copyfile(candidate, clip)
                        break
                if clip.exists() and clip.stat().st_size:
                    break
        if a.piper_model and (not clip.exists() or clip.stat().st_size == 0):
            spoken = text.replace('ROSClaw', '罗斯克劳').replace('Nav2', '导航栈').replace('Home', '起点')
            input_path = work/f'{i:02}.txt';input_path.write_text(spoken)
            subprocess.run([sys.executable, '-m', 'piper', '-m', str(a.piper_model),
                '-i', str(input_path), '-f', str(clip), '--length-scale', '0.95'], check=True)
        if not clip.exists() or clip.stat().st_size == 0:
            for attempt in range(4):
                try:
                    await asyncio.wait_for(edge_tts.Communicate(text,
                        voice='zh-CN-XiaoxiaoNeural').save(str(clip)), timeout=30)
                    if clip.stat().st_size == 0:
                        raise RuntimeError('Empty narration clip')
                    break
                except Exception:
                    clip.unlink(missing_ok=True)
                    if attempt == 3:
                        raise
                    await asyncio.sleep(2 ** attempt)

        raw = subprocess.check_output([a.ffmpeg,'-v','error','-i',str(clip),'-f','s16le','-ac','1','-ar',str(rate),'-'])
        duration = len(raw)/2/rate
        limit = (cues[i+1][0] if i+1<len(cues) else meta['duration_seconds']) - start - 0.2
        if limit <= 0:
            raise ValueError('Overlapping editorial cues')
        if duration > limit:
            raw = subprocess.check_output([a.ffmpeg,'-v','error','-i',str(clip),'-af',f'atempo={duration/limit:.6f}','-f','s16le','-ac','1','-ar',str(rate),'-'])
        samples = array.array('h'); samples.frombytes(raw)
        offset = int(start*rate)
        finish = min(len(audio), offset+len(samples))
        audio[offset:finish] = samples[:finish-offset]
        end = start + (finish-offset)/rate
        subtitles.append(f'{i+1}\n{stamp(start)} --> {stamp(end)}\n【后期讲解】{text}\n')
        records.append({'start':start,'end':end,'text':text})
        print(json.dumps({'narration_clip':i,'seconds':end-start}),flush=True)
    wav = work/'editorial.wav'
    with wave.open(str(wav),'wb') as out:
        out.setnchannels(1);out.setsampwidth(2);out.setframerate(rate);out.writeframes(audio.tobytes())
    srt = a.output.with_suffix('.srt');srt.write_text('\n'.join(subtitles))
    subprocess.run([a.ffmpeg,'-y','-v','error','-i',str(a.video),'-i',str(wav),'-i',str(srt),'-map','0:v:0','-map','1:a:0','-map','2:0','-c:v','copy','-c:a','aac','-b:a','160k','-c:s','mov_text','-metadata:s:s:0','language=zho','-metadata:s:a:0','title=Chinese editorial narration','-movflags','+faststart','-t',str(meta['duration_seconds']),str(a.output)],check=True)
    meta.update(audio=True,narration='AI generated Chinese editorial explanation; not Agent output',voice=voice_name,narration_cues=records,narration_asset_directory=work.name)
    if a.piper_model:
        meta['voice_model_sha256'] = hashlib.sha256(a.piper_model.read_bytes()).hexdigest()
    a.output.with_suffix('.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2)+'\n')


if __name__ == '__main__':
    asyncio.run(main())
