"""Relocate media paths in a temporary workspace; never modify original evidence."""
import argparse,hashlib,json,shutil,subprocess,tempfile
from pathlib import Path


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--attempt',type=Path,required=True)
    p.add_argument('--frames-directory',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--ffmpeg',required=True)
    p.add_argument('--renderer',type=Path,default=Path(__file__).resolve().parents[2]/'challenges/02-semantic-inspection/visualization/render_loading.py')
    a=p.parse_args();original=a.attempt.resolve();frames=a.frames_directory.resolve()
    if not (frames/'timestamps.jsonl').is_file():raise ValueError('Extract all three source archives into the same frames directory')
    provenance=json.loads((frames/'video-provenance.json').read_text())
    def digest(path):
        with path.open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()
    if digest(frames/'timestamps.jsonl')!=provenance['recording_index_sha256'] or digest(original/'acceptance.json')!=provenance['physical_acceptance_sha256']:
        raise ValueError('Camera index and physical acceptance do not match video provenance')
    for row in provenance['selected_frame_sources']:
        for view in row['views'].values():
            f=(frames/view['file']).resolve()
            if not f.is_relative_to(frames) or digest(f)!=view['sha256']:
                raise ValueError('Selected source frame differs from provenance')
    if a.output.exists():raise ValueError('Existing output refused')
    with tempfile.TemporaryDirectory(prefix='rosclaw-media-replay-') as tmp:
        work=Path(tmp)/original.name;work.mkdir()
        for name in ('acceptance.json','recording-acceptance.json'):
            shutil.copyfile(original/name,work/name)
        result=json.loads((original/'result.json').read_text());result['physics_directory']=str(work/'physics')
        (work/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        (work/'physics').mkdir();(work/'physics/baseline-frames').symlink_to(frames,target_is_directory=True)
        (work/'native').symlink_to(original/'native',target_is_directory=True)
        subprocess.run([__import__('sys').executable,str(a.renderer.resolve()),'--attempt',str(work),'--output',str(a.output.resolve()),'--ffmpeg',a.ffmpeg],check=True)
    print('Original task evidence was preserved; only temporary media lookup paths were relocated.')


if __name__=='__main__':
    main()
