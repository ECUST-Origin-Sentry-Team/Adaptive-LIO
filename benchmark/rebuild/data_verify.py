#!/usr/bin/env python3
import pathlib,collections,hashlib,json,importlib.metadata
from rosbags.rosbag2 import Reader
root=pathlib.Path('/workspace/datasets')
def inspect(path):
 h=collections.defaultdict(hashlib.sha256);counts=collections.Counter();backward=0;last=None
 with Reader(path) as r:
  for c,t,raw in r.messages():
   h[c.topic].update(raw);counts[c.topic]+=1
   if last is not None and t<last:backward+=1
   last=t
 return {'counts':dict(counts),'cdr_sha256':{k:v.hexdigest() for k,v in h.items()},'backward_storage_timestamps':backward,'files_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in path.iterdir() if p.is_file()}}
paths=[root/'flatwall/flatwall_05_ros2',root/'glim_cross_sensor/livox_ros2',root/'glim_cross_sensor/livox_retimed',root/'flatwall/flatwall_05_fault_gap']
result={'rosbags_version':importlib.metadata.version('rosbags'),'bags':{str(p):inspect(p) for p in paths}}
a=result['bags'][str(paths[1])];b=result['bags'][str(paths[2])]
assert a['counts']==b['counts'];assert a['cdr_sha256']==b['cdr_sha256']
result['retimed_payload_and_per_topic_order_equal']=True
assert result['bags'][str(paths[0])]['counts']=={'/livox/imu':2861,'/livox/lidar':141}
assert result['bags'][str(paths[1])]['counts']=={'/livox/imu':12611,'/livox/lidar':618}
result['source_expected_counts_match']=True
out=root/'validation.json';out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
