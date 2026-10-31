#!/usr/bin/env python3
"""Render measured results; never imports the lost experiment's numbers."""
import json
from pathlib import Path
root=Path('/workspace/Adaptive-LIO/benchmark/rebuild')
runs=Path('/workspace/bench/rebuild_runs')
r=json.loads((runs/'summary.json').read_text())
a=json.loads((runs/'acceptance.json').read_text())
rows=['# 连续里程计输出修复：重建与验证（2026-10-31）','',
'本轮从 `ljb` 的 `97a3b0e` 重新实现、构建和实测，不沿用已丢失版本的结果。修复与证据保存在 `fix/continuous-odom-rebuild`。', '',
'## 修复范围','',
'见 [问题清单](ISSUES.md) 与 [接口说明](../../src/Adaptive-LIO/docs/continuous_output.md)。核心是校正陈旧时继续发布有效 IMU 预测并标记退化、拒绝过期后验而不停止预测、逐样本传播与最多4样本分批、独立 LiDAR 后验与健康状态、输入执行器隔离及时间追踪。本轮另外通过 GDB 定位并修复了退出阶段全局 ROS 发布器析构崩溃。', '',
'## 真实 bag 成对测试','',
'对照是保留旧0.5秒发布门限和批处理逻辑的受控97a基线；两版使用相同输入接收、确定性采样、观测导出和退出修复。不是丢失的旧1秒二进制。每种输入重复两次，交替运行次序，共12次；均1倍速。', '',
'| 输入 / 次序 | 版本 | odom条数 | 最大传感器间隔(s) | 最大接收间隔(s) | >0.5s传感器间隔数 | 末尾缺口(s) |',
'|---|---|---:|---:|---:|---:|---:|']
for m in r['metrics']:
 n=m['run'];v='修复' if '_fixed_' in n else '受控基线'
 s=m.get('odom_sensor_intervals_s',{});w=m.get('odom_receipt_intervals_s',{})
 rows.append(f"| {n} | {v} | {m['odom_count']} | {s.get('max',0):.6f} | {w.get('max',0):.6f} | {s.get('over_0_5',0)} | {m.get('odom_tail_to_last_imu_s',0):.6f} |")
rows += ['', '自动验收结果：`'+str(a['pass'])+'`。各项判据与逐次结果见 [acceptance.json](evidence/acceptance.json)，完整统计见 [summary.json](evidence/summary.json)。', '',
'## 定位精度：共同真值时刻的平移 APE','',
'最近时间匹配≤30ms，取所有对比轨迹共同覆盖的真值时刻，分别作刚体SE3对齐，不估计尺度。不是全序列覆盖率，也不是姿态误差。', '',
'| 输入 | 共同GT / 总GT | 运行 | 平移RMSE(m) |','|---|---:|---|---:|']
for case,d in r['common_gt_ape'].items():
 for run,x in d['runs'].items(): rows.append(f"| {case} | {d['common_count']} / {d['gt_count']} | {run} | {x['rmse_m']:.6f} |")
rows += ['', '## 故障注入与边界','',
'- 核心与真实ROS适配器分别注入0.75、1.5、2.5秒后验延迟；验证过期拒绝、新鲜校正恢复、真实时间戳、协方差传播与回放一致性。',
'- 同源核心探针对照0.5/1秒门限：1秒门限在持续1.5秒延迟下仍拒发800/1001次检查；修复版拒发0次。这是核心检查次数，不是ROS消息数量。',
'- 4秒IMU积压逐样本处理；真实缺样与非有限输入进入FAULT；没有新IMU时不生成伪造里程计。',
'- 7项针对里程计的CTest通过；没有声称全仓库风格检查或所有其他模块测试通过。',
'- 原始FlatWall/GLIM bag校验值与发布者一致。GLIM重排版仅修改存储时间、消息内容完全一致；无法证明与丢失的旧派生包逐字节相同。',
'- GT来源、构建环境、二进制哈希、复现命令见 [README](README.md) 与 evidence。', '',
'## 结论的适用范围','',
'传感器时间覆盖、节点墙钟发布间隔、录包接收间隔必须分开解释。积压消息回补可以消除传感器轨迹空洞，却仍有交付延迟；本方案不提供硬实时保证。纯前端里程计可以使用此方案，但缺少新LiDAR约束时仍会漂移，DEGRADED及协方差必须被使用方正确处理。',
'']
(root/'RESULTS.md').write_text('\n'.join(rows))
