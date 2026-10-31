#!/usr/bin/env python3
"""Compare reaction to identical real missing-IMU bag, not output count as health."""
import argparse
import json
from pathlib import Path
import numpy as np
from analyze import analyze, bag_data, source_imu, trace_data

parser = argparse.ArgumentParser()
parser.add_argument('--runs', default='/workspace/bench/low_delay_fault')
parser.add_argument('--source', default='/workspace/datasets/flatwall/flatwall_05_fault_gap')
args = parser.parse_args()
root = Path(args.runs)
source = source_imu(Path(args.source))
index = int(np.argmax(np.diff(source)))
fault_stamp = source[index + 1]
result = {'source_imu_count': len(source), 'gap_left_stamp': float(source[index]),
          'gap_right_stamp': float(fault_stamp), 'gap_s': float(source[index + 1] - source[index]),
          'runs': {}}
for version in ['low', 'fixed']:
    run = root / version
    metrics = analyze(run, Path(args.source))
    data = bag_data(run / 'recording')
    faults = [time for time, state in data['status'] if state['state'] == 'FAULT']
    first_fault = min(faults) if faults else None
    trace = trace_data(run / 'output_trace.tsv')
    item = {'metrics': metrics, 'fault_status_received': bool(faults),
            'odom_stamps_at_or_after_gap': int(sum(row[0] >= fault_stamp for row in data['odom'])),
            'odom_received_after_fault_status': sum(row[8] > first_fault for row in data['odom']) if faults else None,
            'published_stamps_at_or_after_gap': int(np.sum(trace['P'][:, 0] >= fault_stamp)) if 'P' in trace else None,
            'legacy_gap_trace_count': len(trace.get('G', [])),
            'interpretation': 'Continuing output in legacy is not proof of healthy full-state IMU propagation; fixed intentionally enters FAULT.'}
    result['runs'][version] = item
fixed = result['runs']['fixed']
result['fixed_fault_acceptance'] = fixed['fault_status_received'] and fixed['odom_stamps_at_or_after_gap'] == 0 and fixed['odom_received_after_fault_status'] == 0
(root / 'fault_comparison.json').write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps(result, indent=2))
raise SystemExit(0 if result['fixed_fault_acceptance'] else 1)
