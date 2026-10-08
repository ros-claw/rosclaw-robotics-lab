"""Whitelist public evidence; never archive private runtime homes or sessions."""
import argparse
import hashlib
import json
import zipfile
from pathlib import Path


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--directory',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    config=json.loads((a.directory/'execution_config.json').read_text())
    physics=Path(config['physics_directory'])
    allowed=['execution_config.json','body.json','canonical-receipts-final.json',
             'sdk-usage.json','source-freeze.json','task-kernel.json']
    files=[(a.directory/name,name) for name in allowed if (a.directory/name).is_file()]
    files += [(path,str(path.relative_to(a.directory))) for path in (a.directory/'actions').glob('*.json')]
    files += [(path,str(path.relative_to(a.directory))) for path in (a.directory/'frozen-source').rglob('*') if path.is_file()]
    for name in ['physics-trajectory.jsonl','physics-contacts.jsonl','scene-audit.json','stage-inventory.json','path-witness.jsonl']:
        if (physics/name).is_file():
            files.append((physics/name,'independent/'+name))
    snapshots=[]
    for path,name in files:
        payload=path.read_bytes()
        if name.endswith('.jsonl') and payload and not payload.endswith(b'\n'):
            payload=payload.rsplit(b'\n',1)[0]+b'\n' if b'\n' in payload else b''
        snapshots.append((path,name,payload))
    manifest={name:hashlib.sha256(payload).hexdigest() for path,name,payload in snapshots}
    with zipfile.ZipFile(a.output,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for path,name,payload in snapshots:
            if path.is_symlink() or 'auth' in name.lower() or 'operator-identity' in name:
                raise ValueError('Unexpected private or linked file in whitelist')
            z.writestr(name,payload)
        z.writestr('public-evidence-manifest.json',json.dumps(manifest,indent=2)+'\n')
    print(json.dumps({'file_count':len(files),'archive':str(a.output),'sha256':hashlib.sha256(a.output.read_bytes()).hexdigest()}))


if __name__=='__main__':
    main()
