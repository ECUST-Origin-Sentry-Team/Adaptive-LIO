#!/usr/bin/env python3
"""Convert public Livox ROS1 fixtures, preserving payload/header times.
Only ROS serialization and Livox package name change. Retimed derivative changes
storage timestamps to message header timestamps; all CDR message bytes identical.
"""
import argparse,hashlib,json,pathlib,collections,tarfile
import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores,get_typestore,get_types_from_msg
p=argparse.ArgumentParser();p.add_argument('source',type=pathlib.Path);p.add_argument('output',type=pathlib.Path);p.add_argument('--retimed',type=pathlib.Path);a=p.parse_args()
src=get_typestore(Stores.ROS1_NOETIC);dst=get_typestore(Stores.ROS2_JAZZY)
for name in ['CustomPoint','CustomMsg']:
 text=(pathlib.Path('/workspace/Adaptive-LIO/src/livox_ros_driver2/msg')/(name+'.msg')).read_text()
 dst.register(get_types_from_msg(text,'livox_ros_driver2/msg/'+name))
rows=[];stamp_series=collections.defaultdict(list);acc=[];pc=[];record_delay=collections.defaultdict(list)
with Reader(a.source) as r:
 connections=[c for c in r.connections if c.topic in ['/livox/lidar','/livox/imu']]
 for c in connections:src.register(get_types_from_msg(c.msgdef.data,c.msgtype))
 all_connections=[{'topic':c.topic,'type':c.msgtype,'count':c.msgcount} for c in r.connections]
 for c,t,raw in r.messages(connections):
  msg=src.deserialize_ros1(raw,c.msgtype);s=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
  stamp_series[c.topic].append(s);record_delay[c.topic].append((t-s)*1e-9)
  typ=c.msgtype.replace('livox_ros_driver/msg/','livox_ros_driver2/msg/')
  cdr=bytes(dst.serialize_cdr(msg,typ))
  # Decode destination schema once per message: validates driver2 wire compatibility.
  check=dst.deserialize_cdr(cdr,typ)
  assert check.header.stamp==msg.header.stamp
  if c.topic.endswith('imu'):acc.append([msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z])
  else:pc.append(msg.point_num)
  rows.append((c.topic,typ,t,s,cdr))
def write(path,retime):
 with Writer(path,version=9) as w:
  cmap={topic:w.add_connection(topic,typ,typestore=dst) for topic,typ in dict((x[0],x[1]) for x in rows).items()}
  for topic,typ,t,s,raw in sorted(rows,key=lambda row:row[3 if retime else 2]):w.write(cmap[topic],s if retime else t,raw)
def hfile(path,algorithm):
 h=hashlib.new(algorithm)
 with path.open('rb') as f:
  while b:=f.read(1024**2):h.update(b)
 return h.hexdigest()
write(a.output,False)
if a.retimed:write(a.retimed,True)
metrics={}
for topic,stamps in stamp_series.items():
 st=np.array(stamps,dtype=np.int64);d=np.diff(st)*1e-9
 h=hashlib.sha256()
 for row in rows:
  if row[0]==topic:h.update(row[4])
 metrics[topic]={'count':len(st),'header_first_ns':int(st[0]),'header_last_ns':int(st[-1]),'max_header_gap_s':float(max(d)),'duplicate_headers':int(sum(d==0)),'backward_headers':int(sum(d<0)),'record_minus_header_min_s':min(record_delay[topic]),'record_minus_header_max_s':max(record_delay[topic]),'ordered_cdr_sha256':h.hexdigest()}
manifest={'source':str(a.source),'source_md5':hfile(a.source,'md5'),'source_sha256':hfile(a.source,'sha256'),'source_bytes':a.source.stat().st_size,'source_connections':all_connections,'output':str(a.output),'retimed':str(a.retimed) if a.retimed else None,'retiming':'storage timestamp = actual message header; payload unchanged; globally sorted' if a.retimed else None,'topics':metrics,'acceleration_norm_median_raw':float(np.median(np.linalg.norm(np.array(acc),axis=1))),'lidar_point_count_range':[min(pc),max(pc)],'output_sha256':hfile(a.output/(a.output.name+'.db3'),'sha256')}
a.output.with_suffix('.manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');print(json.dumps(manifest,indent=2))
