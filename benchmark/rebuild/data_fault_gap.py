#!/usr/bin/env python3
"""Drop exactly 15 consecutive IMUs around first stamp +4s; preserve all others."""
import pathlib,hashlib,json
from rosbags.rosbag2 import Reader,Writer
from rosbags.typesys import Stores,get_typestore
src=pathlib.Path('/workspace/datasets/flatwall/flatwall_05_ros2');dst=pathlib.Path('/workspace/datasets/flatwall/flatwall_05_fault_gap')
ts=get_typestore(Stores.ROS2_JAZZY)
with Reader(src) as r, Writer(dst,version=9) as w:
 cmap={c.id:w.add_connection(c.topic,c.msgtype,msgdef=c.msgdef.data,rihs01=c.digest,offered_qos_profiles=c.ext.offered_qos_profiles) for c in r.connections}
 first=None;removed=[];kept=[];counts={};last=None;gaps=[]
 for c,t,raw in r.messages():
  if c.topic=='/livox/imu':
   msg=ts.deserialize_cdr(raw,c.msgtype);s=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
   if first is None:first=s
   if s>=first+4_000_000_000 and len(removed)<15:removed.append(s);continue
   if last is not None:gaps.append(s-last)
   last=s
  w.write(cmap[c.id],t,raw);counts[c.topic]=counts.get(c.topic,0)+1
manifest={'source':str(src),'output':str(dst),'removed_imu':len(removed),'removed_header_ns':removed,'counts':counts,'maximum_remaining_imu_header_gap_s':max(gaps)*1e-9,'output_db_sha256':hashlib.sha256((dst/(dst.name+'.db3')).read_bytes()).hexdigest()}
dst.with_suffix('.manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');print(json.dumps(manifest,indent=2))
