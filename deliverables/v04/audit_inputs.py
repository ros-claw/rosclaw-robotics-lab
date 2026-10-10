"""Read private journals locally; export only allowed-path verdict, tools and public task."""
import argparse,hashlib,json
from datetime import datetime
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--attempts',type=Path,nargs='+',required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();rows=[]
for attempt in a.attempts:
 root=attempt/'native';r=json.loads((attempt/'result.json').read_text());cfg=root/'execution_config.json';task=json.loads(cfg.read_text())['task'] if cfg.exists() else None;users=[];tools=[];files=[];hashes=[]
 for s in sorted((root/'home/agent/sessions').glob('*.jsonl')):
  hashes.append(hashlib.sha256(s.read_bytes()).hexdigest())
  for line in s.read_text().splitlines():
   row=json.loads(line);m=row.get('message',{});c=m.get('content',[])
   if m.get('role')=='user':
    value=c if isinstance(c,str) else ''.join(x.get('text','') for x in c if isinstance(x,dict) and x.get('type')=='text')
    if value!=task:raise ValueError('Unexpected private task; refuse export')
    users.append({'timestamp':row['timestamp'],'text':task})
   if m.get('role')=='assistant' and isinstance(c,list):
    for x in c:
     if x.get('type')=='toolCall':
      tools.append({'timestamp':row['timestamp'],'name':x['name']})
      if x['name'] in ('read','grep','exec','bash','write','edit'):
       args=x.get('arguments') or {};expected=(str(args.get('path','')).endswith('/packages/rosclaw-agent/dist/skills/rosclaw-embodied/SKILL.md') and set(args)=={'path'} and x['name']=='read')
       report_name={'l04c01':'patrol.verification.json','l04oldc02':'mission.verification.json'}.get(attempt.name)
       report=root/'actions'/report_name if report_name else None
       receipt_read=False;receipt_proof=None
       if report and report.is_file() and not report.is_symlink() and args.get('path')==str(report.resolve()) and x['name'] in ('read','grep'):
        report_data=json.loads(report.read_text());access_time=datetime.fromisoformat(row['timestamp'].replace('Z','+00:00')).timestamp()
        permitted_keys={'path','limit','offset'} if x['name']=='read' else {'path','pattern','limit','context','ignoreCase','literal'}
        receipt_read=(set(args)<=permitted_keys and report_data.get('success') is True and report_data.get('memory_outcome')=='success' and access_time>=report.stat().st_mtime)
        receipt_proof={'sha256':hashlib.sha256(report.read_bytes()).hexdigest(),'report_written_wall_time':report.stat().st_mtime,'access_wall_time':access_time,'memory_success':report_data.get('memory_outcome')=='success'}
       files.append({'tool':x['name'],'official_embodied_skill_only':expected,'verified_compatibility_report_only':receipt_read,'allowed_file_access':expected or receipt_read,'report_proof':receipt_proof})
 status=('PASS' if len(users)==1 and all(x['allowed_file_access'] for x in files) else 'NOT_STARTED' if not users and not hashes and r['status']!='PASS' else 'FAIL')
 rows.append({'attempt':attempt.name,'original_status':r['status'],'status':status,'task_input_count':len(users),'task_inputs':users,'tool_calls':tools,'file_access_checks':files,'private_journal_sha256':hashes})
out={'status':'PASS' if all(r['status'] in ('PASS','NOT_STARTED') for r in rows) else 'FAIL','scope':'Local sanitized SDK audit; not external attestation or malicious-access isolation. SIM approvals are separate. Startup with zero SDK input is NOT_STARTED, never task PASS. Loading tasks allow only the official embodied skill. Legacy compatibility additionally permits read/grep of their exact successful Memory verification report after its creation, with file hash and timestamps. No fixture or PhysX truth access allowed. No assistant prose/thinking or arbitrary tool arguments exported.','attempts':rows};a.output.write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n');print(json.dumps({'status':out['status'],'counts':{s:sum(r['status']==s for r in rows) for s in ('PASS','NOT_STARTED','FAIL')}}))
