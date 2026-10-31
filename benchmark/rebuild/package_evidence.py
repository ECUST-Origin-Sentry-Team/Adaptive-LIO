#!/usr/bin/env python3
"""Persist compact complete run evidence; original point-cloud datasets stay external."""
from pathlib import Path
import tarfile, hashlib, json, shutil
root=Path('/workspace/Adaptive-LIO/benchmark/rebuild')
bench=Path('/workspace/bench')
out=root/'evidence'/'runs';out.mkdir(parents=True,exist_ok=True)
inputs=sorted((bench/'rebuild_runs').glob('*'))
inputs += [bench/'rebuild_smoke',bench/'rebuild_smoke2',bench/'rebuild_fault']
manifest=[]
for run in inputs:
 if not run.is_dir() or not (run/'run.json').exists():continue
 dest=out/(run.name+'.tar.gz')
 with tarfile.open(dest,'w:gz',compresslevel=6) as tar:
  for f in sorted(run.rglob('*')):
   if not f.is_file() or 'ros_logs' in f.relative_to(run).parts:continue
   tar.add(f,arcname=str(Path(run.name)/f.relative_to(run)),recursive=False)
 manifest.append({'archive':str(dest.relative_to(root)),'bytes':dest.stat().st_size,'sha256':hashlib.sha256(dest.read_bytes()).hexdigest(),'source':str(run),'includes':['ROS2 recording','node/player/recorder logs','traces','trajectories','run metadata','metrics when available']})
for name in ['summary.json','acceptance.json','order.json']:
 shutil.copy2(bench/'rebuild_runs'/name,root/'evidence'/name)
(root/'evidence'/'archive_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps({'archives':len(manifest),'total_bytes':sum(x['bytes'] for x in manifest)}))
