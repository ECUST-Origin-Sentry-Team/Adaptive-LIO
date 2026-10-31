#!/usr/bin/env python3
from pathlib import Path
import json,tarfile,hashlib,shutil
root=Path('/workspace/Adaptive-LIO/benchmark/low_delay_compare')
bench=Path('/workspace/bench')
out=root/'evidence'/'runs';out.mkdir(parents=True,exist_ok=True)
inputs=[(p.name,p) for p in sorted((bench/'low_delay_runs').iterdir()) if p.is_dir() and (p/'run.json').exists()]
inputs += [('smoke_low',bench/'low_delay_smoke')]
inputs += [('fault_'+v,bench/'low_delay_fault'/v) for v in ['low','fixed']]
manifest=[]
for name,run in inputs:
 if not (run/'run.json').exists():raise RuntimeError('missing run '+str(run))
 dest=out/(name+'.tar.gz')
 with tarfile.open(dest,'w:gz',compresslevel=6) as tar:
  for f in sorted(run.rglob('*')):
   if f.is_file() and 'ros_logs' not in f.relative_to(run).parts:
    tar.add(f,arcname=str(Path(name)/f.relative_to(run)),recursive=False)
 manifest.append({'archive':str(dest.relative_to(root)),'bytes':dest.stat().st_size,'sha256':hashlib.sha256(dest.read_bytes()).hexdigest(),'source':str(run)})
for name in ['summary.json','acceptance.json','order.json','gap_audit.json','worst_delivery_audit.json']:
 shutil.copy2(bench/'low_delay_runs'/name,root/'evidence'/name)
shutil.copy2(bench/'low_delay_fault/fault_comparison.json',root/'evidence/fault_comparison.json')
(root/'evidence/archive_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps({'archives':len(manifest),'bytes':sum(x['bytes'] for x in manifest)}))
