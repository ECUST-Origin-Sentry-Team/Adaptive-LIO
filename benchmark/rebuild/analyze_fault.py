#!/usr/bin/env python3
"""Inspect a real missing-IMU bag replay independently of nominal acceptance."""
import argparse
import json
from pathlib import Path
from analyze import bag_data, source_imu, stats
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument('--run', required=True)
parser.add_argument('--source', default='/workspace/datasets/flatwall/flatwall_05_fault_gap')
args = parser.parse_args()
run = Path(args.run)
data = bag_data(run / 'recording')
source = source_imu(Path(args.source))
faults = [time for time, state in data['status'] if state['state'] == 'FAULT']
first_fault = min(faults) if faults else None
late = sum(row[8] > first_fault for row in data['odom']) if first_fault is not None else None
result = {'source_imu_count': len(source), 'source_sensor_intervals_s': stats(np.diff(source)),
          'received_fault_status': bool(faults), 'first_fault_receipt_time': first_fault,
          'odom_count': len(data['odom']), 'odom_received_after_first_fault_status': late,
          'processes': json.loads((run / 'run.json').read_text())}
result['pass'] = bool(faults) and late == 0 and len(data['odom']) > 0
(run / 'fault_validation.json').write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps(result, indent=2))
raise SystemExit(0 if result['pass'] else 1)
