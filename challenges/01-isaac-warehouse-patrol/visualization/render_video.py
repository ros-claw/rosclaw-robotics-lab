"""Timestamp-aligned real viewport, terminal replay and independent physical HUD.

No generated scene or invented Agent text. Promo compresses time explicitly;
workflow preserves the full recorded wall interval at 2 frames per second.
"""
import argparse
import base64
import bisect
import codecs
from datetime import datetime
import json
import hashlib
import math
import subprocess
from pathlib import Path

import pyte
import yaml
from PIL import Image, ImageDraw, ImageFont

FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
MONO = '/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf'


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--directory', type=Path, required=True)
    p.add_argument('--map', type=Path, required=True)
    p.add_argument('--ffmpeg', required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--mode', choices=['promo', 'workflow'], default='promo')
    a = p.parse_args()
    config = json.loads((a.directory / 'execution_config.json').read_text())
    settings = json.loads((a.directory/'home/agent/settings.json').read_text())
    if settings.get('hideThinkingBlock') is not True:
        raise ValueError('Do not publish terminal recordings with visible private thinking')
    physics = Path(config['physics_directory'])
    frames = rows(physics / 'baseline-frames/timestamps.jsonl')
    frames = [r for r in frames if (physics/'baseline-frames'/r['frame']).is_file()]
    samples = rows(physics / 'physics-trajectory.jsonl')
    events = rows(a.directory / 'native-pty.events.jsonl')
    visits = [json.loads(path.read_text()) for path in (a.directory/'actions').glob('*.json') if path.name != 'patrol.verification.json']
    final = json.loads((a.directory/'actions/patrol.verification.json').read_text())
    box = json.loads((physics/'scene-audit.json').read_text()).get('unmapped_test_obstacle')
    witnesses = rows(physics/'path-witness.jsonl') if (physics/'path-witness.jsonl').exists() else []
    witness_index = 0
    box_hits, occupied_cells = 0, 0
    if final.get('status') not in ('PASS', None) or final.get('memory_outcome') != 'success':
        raise ValueError('Only independently completed Memory missions are promotional sources')
    task_kernel = json.loads((a.directory/'task-kernel.json').read_text())
    task_closed = datetime.fromisoformat(task_kernel['updated_at']).timestamp()
    start, end = events[0]['wall_time'], events[-1]['wall_time']
    if frames[-1]['wall_time'] < end - 2:
        raise ValueError('Viewport capture does not cover the full workflow')
    fps = 12 if a.mode == 'promo' else 2
    duration = 80 if a.mode == 'promo' else end-start
    count = math.ceil(duration*fps)
    factor = (end-start)/duration
    ft, st = [x['wall_time'] for x in frames], [x['wall_time'] for x in samples]
    screen, decoder = pyte.Screen(80, 24), codecs.getincrementaldecoder('utf-8')('replace')
    stream = pyte.Stream(screen)
    event_index = 0
    map_info = yaml.safe_load(a.map.with_suffix('.yaml').read_text())
    occupancy = Image.open(a.map).convert('RGB')
    map_width, map_height = occupancy.size
    occupancy.thumbnail((600, 320))
    map_x, map_y = 1280+(640-occupancy.width)//2, 130
    def point(x, y):
        u = (x-map_info['origin'][0])/map_info['resolution']
        v = map_height - (y-map_info['origin'][1])/map_info['resolution']
        return (map_x+u*occupancy.width/map_width, map_y+v*occupancy.height/map_height)
    title_font = ImageFont.truetype(FONT, 30)
    body_font = ImageFont.truetype(FONT, 22)
    small_font = ImageFont.truetype(FONT, 18)
    mono = ImageFont.truetype(MONO, 12)
    cjk_mono = ImageFont.truetype(FONT, 12)
    a.output.parent.mkdir(parents=True, exist_ok=True)
    encoder = subprocess.Popen([a.ffmpeg, '-y', '-loglevel', 'error', '-f','rawvideo','-pix_fmt','rgb24','-s','1920x1080','-r',str(fps),'-i','-','-an','-c:v','libx264','-preset','fast','-crf','21','-pix_fmt','yuv420p','-movflags','+faststart',str(a.output)], stdin=subprocess.PIPE)
    current_frame = None
    viewport = None
    for i in range(count):
        t = start + (end-start)*i/max(1,count-1)
        fi = max(0, bisect.bisect_right(ft,t)-1)
        si = max(0, bisect.bisect_right(st,t)-1)
        sample = samples[si]
        while event_index < len(events) and events[event_index]['wall_time'] <= t:
            stream.feed(decoder.decode(base64.b64decode(events[event_index]['data'])))
            event_index += 1
        if current_frame != fi:
            viewport = Image.open(physics/'baseline-frames'/frames[fi]['frame']).convert('RGB').resize((1280,720))
            current_frame = fi
        canvas = Image.new('RGB',(1920,1080),(12,18,28))
        canvas.paste(viewport,(0,100))
        if frames[fi].get('display_view') == 'top':
            fp = samples[max(0,bisect.bisect_right(st,frames[fi]['wall_time'])-1)]['physics_transforms_xyzw'][0]
            cx, cy = 640-(fp[1]-1)*1280/48, 360-fp[0]*1280/48
            closeup=viewport.crop((int(cx-90),int(cy-68),int(cx+90),int(cy+68))).resize((360,272))
            canvas.paste(closeup,(895,515))
        canvas.paste(occupancy,(map_x,map_y))
        d = ImageDraw.Draw(canvas)
        d.text((30,22),'ROSClaw × Isaac Sim | 一句话，让机器人巡检仓库',font=title_font,fill='white')
        speed = f'{factor:.1f}× wall-time compression' if a.mode=='promo' else '1× wall time · continuous recorded interval'
        d.text((1100,35),speed,font=small_font,fill='#55ddff')
        d.text((30,66),'DGX Spark · Isaac Sim 6.1 · Nova Carter · ROS 2 Jazzy · SIM evidence',font=small_font,fill='#b2c0cf')
        if frames[fi].get('display_view') == 'top':
            d.rectangle((893,513,1256,788),outline='#55ddff',width=2)
            d.text((895,487),'Actual robot viewport crop / 机器人局部',font=small_font,fill='#55ddff')
        d.text((1300,105),'Measured map + independent PhysX trajectory',font=small_font,fill='white')
        trace = [point(*s['physics_transforms_xyzw'][0][:2]) for s in samples[:si+1:4]]
        if len(trace)>1:
            d.line(trace,fill='#15a3ff',width=3)
        if box:
            while witness_index < len(witnesses) and witnesses[witness_index]['wall_time'] <= t:
                row = witnesses[witness_index]
                box_hits=max(box_hits,row.get('box_hit_count',0))
                occupied_cells=max(occupied_cells,row.get('occupied_cells',0))
                witness_index+=1
            corners=[point(-4,3.5),point(-3,4.5)]
            d.rectangle((min(c[0] for c in corners),min(c[1] for c in corners),max(c[0] for c in corners),max(c[1] for c in corners)),fill='#ed4b40')
            d.text((1300,435),f'Unmapped box: LiDAR hits {box_hits} · local occupied {occupied_cells}',font=small_font,fill='#ff9d30')
        for name, site in config['sites'].items():
            x,y=point(site['x'],site['y']);d.ellipse((x-4,y-4,x+4,y+4),fill='#ff9d30');d.text((x+6,y-8),name,font=small_font,fill='#f05800')
        x,y=point(*sample['physics_transforms_xyzw'][0][:2]);d.ellipse((x-6,y-6,x+6,y+6),fill='#20cf65')
        d.text((1300,460),'Actual Native Agent terminal replay',font=small_font,fill='#55ddff')
        for row in range(24):
            for col, char in screen.buffer[row].items():
                if char.data and char.data != " ":
                    d.text((1300+col*7.2,495+row*13),char.data,
                           font=cjk_mono if any(ord(c)>127 for c in char.data) else mono,
                           fill='#dce5ed')
        d.text((30,845),'用户指令 / User task',font=body_font,fill='#55ddff')
        task=config['task']
        for j in range(0,len(task),55):
            d.text((30,882+(j//55)*30),task[j:j+55],font=body_font,fill='white')
        completed = [v for v in visits if v.get('verification',{}).get('success') and v.get('finished_wall_time',math.inf)<=t]
        summary=' | '.join(f"{v['site_id']}: {v['verification']['position_error_m']:.3f}m PASS" for v in sorted(completed,key=lambda v:v['finished_wall_time'])) or 'Checking readiness / executing · final acceptance pending'
        d.text((30,975),summary,font=body_font,fill='#6df3a2')
        if t >= task_closed:
            d.text((1320,840),'Memory: SUCCESS',font=title_font,fill='#6df3a2')
            d.text((1320,885),'TaskKernel: SUCCEEDED',font=body_font,fill='#6df3a2')
        d.text((30,1020),f"Wall elapsed {t-start:.1f}s | SIM {sample['sim_time']:.2f}s | non-floor contacts {sample['collision_count']} | viewport snapshots + recorded PTY",font=small_font,fill='#b2c0cf')
        encoder.stdin.write(canvas.tobytes())
        if i%120==0:
            print(json.dumps({'frame':i,'total':count}),flush=True)
    encoder.stdin.close()
    if encoder.wait():
        raise RuntimeError('ffmpeg encoding failed')
    a.output.with_suffix('.json').write_text(json.dumps({'run_id':a.directory.name,'mode':a.mode,'duration_seconds':duration,'wall_interval':[start,end],'wall_time_compression':factor,'fps':fps,'source':'actual Isaac viewport PNGs, exact PTY events, independent PhysX','audio':False,'visible_private_thinking':False,'renderer_source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()},indent=2)+'\n')


if __name__=='__main__':
    main()
