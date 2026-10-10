import hashlib,json,subprocess
from pathlib import Path
b=Path('/home/nvidia/sim/rosclaw-robotics-lab/.runtime/v04');lab=b.parents[1];up=lab.parent/'rosclaw-v04-merged';protocol=json.loads((b/'formal4/protocol.json').read_text());expected=protocol['upstream_commit'];assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=up,text=True).strip()==expected
rows=[]
for t in protocol['trials']:
 root=lab.parent/t['id'];f=json.loads((root/'native/source-freeze.json').read_text());checks=[]
 assert f['git_sha']==protocol['lab_commit'] and not f['working_tree_dirty'] and f['native_build_stamp']['commit']==expected and f['rosclaw_upstream']['git_sha']==expected
 for key,prefix in [('source_hashes','challenges/01-isaac-warehouse-patrol'),('semantic_source_hashes','challenges/02-semantic-inspection')]:
  for name,h in f[key].items():
   p=lab/prefix/name;actual=hashlib.sha256(p.read_bytes()).hexdigest();checks.append({'file':str(p.relative_to(lab)),'expected':h,'actual':actual,'matches':h==actual})
 for name,h in f['native_build_sha256'].items():
  actual=hashlib.sha256((up/name).read_bytes()).hexdigest();checks.append({'file':'rosclaw/'+name,'expected':h,'actual':actual,'matches':h==actual})
 if not all(c['matches'] for c in checks):raise ValueError('Delivery runtime differs from physically tested frozen code')
 rows.append({'attempt':t['id'],'source_freeze_sha256':hashlib.sha256((root/'native/source-freeze.json').read_bytes()).hexdigest(),'files_checked':len(checks),'body_hash':json.loads((root/'native/execution_config.json').read_text())['body_snapshot_hash'],'checks':checks})
out={'status':'PASS','upstream_pr':'https://github.com/ros-claw/rosclaw/pull/660','actual_merge_sha':expected,'actual_native_build_sha':json.loads((up/'packages/rosclaw-agent/dist/build-stamp.json').read_text())['commit'],'frozen_lab_sha':protocol['lab_commit'],'delivery_head_at_check':subprocess.check_output(['git','rev-parse','HEAD'],cwd=lab,text=True).strip(),'scope':'Delivery executable file hashes compared to every actual final frozen trial, with actual upstream merge/build SHA. Later documentation commits may differ. This does not replace the live trials.','attempts':rows};(b/'runtime-binding.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps({'status':'PASS','attempts':len(rows),'files_per_attempt':rows[0]['files_checked']}))
