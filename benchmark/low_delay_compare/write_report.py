#!/usr/bin/env python3
from pathlib import Path
import json,csv
root=Path('/workspace/Adaptive-LIO/benchmark/low_delay_compare')
r=json.loads(Path('/workspace/bench/low_delay_runs/summary.json').read_text())
a=json.loads(Path('/workspace/bench/low_delay_runs/acceptance.json').read_text())
f=json.loads(Path('/workspace/bench/low_delay_fault/fault_comparison.json').read_text())
rows=['# 当前修复版与 Low-Delay-Time 的输出对比（2026-10-31）','',
'本轮重新进行12次成对回放（3条件×2版本×2重复），另对两版分别运行真实IMU缺样测试。固定版二进制保持上一轮发布的0362c6ef…不变；老版使用实际Low-Delay-Time分支24e6ca3，仅做编译兼容和可关闭的观测日志。详见 [协议与复现](README.md)、[源码语义审查](SEMANTICS.md)。','',
'## 原始输入、输出覆盖与实际交付','',
'下表每行是一轮实测，传感器间隔按去重后的时间覆盖计算，重复和倒退另行保留；接收间隔来自真实录包时间。旧版较高的消息频率与新版目标50Hz不同，不能仅按频率判定完整预测质量。','',
'|运行|消息数|唯一时间戳频率Hz|最大传感器间隔ms|最大接收间隔ms|重复戳|倒退|位置不变比例|','|---|---:|---:|---:|---:|---:|---:|---:|']
for m in r['metrics']:
 rows.append(f"|{m['run']}|{m['odom_count']}|{m['odom_unique_sensor_mean_rate_hz']:.2f}|{m['odom_unique_sensor_intervals_s']['max']*1000:.3f}|{m['odom_receipt_intervals_s']['max']*1000:.3f}|{m['odom_duplicate_stamps_total']}|{m['odom_sensor_intervals_s']['negative']}|{m['odom_unchanged_position_transition_fraction']*100:.2f}%|")
rows+=['','位置不变按相邻位置差≤1e-12m统计，是描述性指标。老版代码只在IMU到来时传播姿态，位置保持最新LiDAR结果，不能把这些重复位置简单称为配准失败。原始序列含实际运动，GT路径长度在summary中保存。','',
'## 字段与健康状态','',
'|运行|输入回调完整|全零位姿协方差比例|全零速度比例|健康状态话题|','|---|---|---:|---:|---|']
for m in r['metrics']:
 rows.append(f"|{m['run']}|{m.get('imu_callback_matches_source')}|{m['odom_pose_covariance_all_zero_fraction']*100:.1f}%|{m['odom_twist_values_all_zero_fraction']*100:.1f}%|{m['health_status_available']}|")
rows+=['','老版协方差和twist未填，默认零值不代表真实零不确定性或静止。新版提供模型传播的估计量，但本轮没有验证协方差是否经过误差标定。','',
'## 共同真值时刻的平移APE','',
'各条件取四轮共同覆盖的GT点，最近时间匹配≤30ms，SE3对齐、不估尺度。老版重复时间戳采用最后收到的位置。两个分支内层算法也不同，不能把精度差异全部归因于输出层。','',
'|条件|共同/全部GT|运行|RMSE(m)|','|---|---:|---|---:|']
for case,d in r['common_gt_ape'].items():
 for name,x in d['runs'].items():rows.append(f"|{case}|{d['common_count']}/{d['gt_count']}|{name}|{x['rmse_m']:.6f}|")
rows+=['','## 真实缺样测试','',f"从FlatWall05删除15条IMU，形成{f['gap_s']*1000:.3f}ms时间戳缺口。",'',
'|版本|收到FAULT状态|时间戳达到缺口之后的odom数|FAULT状态后收到odom数|','|---|---|---:|---:|']
for v,d in f['runs'].items():rows.append(f"|{v}|{d['fault_status_received']}|{d['odom_stamps_at_or_after_gap']}|{d['odom_received_after_fault_status']}|")
rows+=['','新版本门限为50ms，老版外层门限为200ms；此次80ms缺样的不同反应包含明确的策略差异。老版继续发布不能证明缺失测量被正确恢复，新版FAULT后的停止属于显式故障处理。','',
'## 边界与证据','',
f"- 自动实验完整性和新版约束验收：{a['pass']}；真实缺样响应验收：{f['fixed_fault_acceptance']}。",'- 老版只在请求退出之后出现预期的未回收线程abort(-6)，已独立记录；没有把它混作回放期间空洞。',
'- 原始IMU时间连续性、节点输入回调间隔、节点publish间隔和录包接收间隔分别统计。源输入完整仍可能有交付停顿。',
'- 新版的连续性不代表任意负载下硬实时保证；两次重复和两个短公开序列也不能证明普遍精度优越。',
'- 详细逐次统计见 [summary.json](evidence/summary.json)，CSV见 [comparison.csv](evidence/comparison.csv)，录包与追踪见 [运行压缩包](evidence/runs)。所有结果来自本轮，未借用上一次对ljb基线的数字。','']
(root/'RESULTS.md').write_text('\n'.join(rows))
(root/'evidence').mkdir(exist_ok=True)
fields=['run','odom_count','unique_hz','sensor_max_s','receipt_max_s','duplicates','backwards','held_position_fraction','imu_callback_complete','pose_cov_all_zero_fraction','twist_all_zero_fraction','health_status']
with (root/'evidence/comparison.csv').open('w',newline='') as out:
 w=csv.DictWriter(out,fieldnames=fields,lineterminator='\n');w.writeheader()
 for m in r['metrics']:w.writerow(dict(run=m['run'],odom_count=m['odom_count'],unique_hz=m['odom_unique_sensor_mean_rate_hz'],sensor_max_s=m['odom_unique_sensor_intervals_s']['max'],receipt_max_s=m['odom_receipt_intervals_s']['max'],duplicates=m['odom_duplicate_stamps_total'],backwards=m['odom_sensor_intervals_s']['negative'],held_position_fraction=m['odom_unchanged_position_transition_fraction'],imu_callback_complete=m.get('imu_callback_matches_source'),pose_cov_all_zero_fraction=m['odom_pose_covariance_all_zero_fraction'],twist_all_zero_fraction=m['odom_twist_values_all_zero_fraction'],health_status=m['health_status_available']))
p=root/'RESULTS.md'
text=p.read_text(); first,rest=text.split('\n',1)
preface='''

## 结论先行

**新版在完整状态预测、时间戳规范性和故障可见性方面更完善；本轮没有证明它比Low-Delay-Time空洞更少、更低延迟或定位更准。**

- 两版各6次正常回放都没有超过0.5秒的传感器覆盖空洞或接收空洞，源IMU与节点回调全部完整。
- 新版无重复时间戳；老版每次FlatWall128个、每次GLIM605个重复戳。老版高频更新姿态，位置保持最近LiDAR结果，速度/协方差字段未填写。
- 老版约201–204Hz，新版实测约45Hz（目标50Hz）。新版是更完整的预测输出，不是更高的原始消息频率。
- 两版仍有约0.1秒级的接收间隔：新版最大126.299ms、老版最大124.808ms。不存在严格20ms墙钟交付保证。
- **本轮GLIM老版位置APE约0.085–0.095m，新版约0.213–0.215m，老版更准。** 两分支内层算法不同，不能把这个差距归因于本次外层修复，也不能宣称全面优于老版。

## 初始化间隔审计

老版六次运行的最大unique间隔全部是第一个发布间隔。原始最大值保留在下表；另外排除首间隔后的描述性统计如下，不能把初始化交接误称为稳态停发：

|条件|旧版排除首间隔后最大值|新版最大值|
|---|---:|---:|
|FlatWall05|5.131ms|25.054ms|
|GLIM原始|5.098ms|25.087ms|
|GLIM重排时间|5.098ms|25.087ms|

详情：[gap_audit.json](evidence/gap_audit.json)。这项额外审计没有改变0.5秒验收门限或原始最大间隔统计。
'''
p.write_text(first+preface+rest)
with p.open('a') as out:
 out.write('\n旧版H追踪在三次运行各出现一次，仅为初始化附近2.709–3.730ms的历史起点钳制，不能称为本轮观察到>2秒的迟到后验。旧版T/G/S计数全为零，正常回放没有触发旧版校正年龄拒发或大dt重置。\n')
 out.write('\n旧版FlatWall两次APE为1.177m与0.597m，波动明显。thread_local随机采样器状态可能受回调线程分配影响，Ceres并行浮点行为也可能参与；尚未做控制变量消融，不能宣称已确定原因。\n')
 out.write('\n真实缺样中旧版跨缺口后继续输出2131条消息且无FAULT；新版明确FAULT，缺口之后及FAULT状态之后均无输出。这符合不同阈值与健康策略，不能把旧版继续发消息当作输入没有丢失。\n')
with p.open('a') as out:
 out.write('''

## 约0.1秒交付间隔的定位

对新版每种输入条件的最坏publish墙钟间隔进行追踪：

|条件|publish间隔|首个符合发布间隔的IMU入队距前次publish|该IMU入队后至publish|
|---|---:|---:|---:|
|FlatWall|113.529ms|113.395ms|0.134ms|
|GLIM原始|116.531ms|110.665ms|5.865ms|
|GLIM重排|106.654ms|106.604ms|0.050ms|

这些事件没有发现更早符合频率条件的IMU在队列内长期滞留，主要受输入到达、回调调度和采样节奏影响。不同指标的最大值不一定来自同一事件，不能把最大接收间隔减去最大发布间隔解释为DDS延迟。完整证据：[worst_delivery_audit.json](evidence/worst_delivery_audit.json)。因此“没有>0.5秒大空洞”不等于“严格每20ms墙钟交付”。
''')
