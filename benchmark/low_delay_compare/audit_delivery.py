#!/usr/bin/env python3
"""Attribute largest measured publication pause using existing monotonic traces.

The decomposition describes each observed event, not a hard real-time bound.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from analyze import trace_data

parser = argparse.ArgumentParser()
parser.add_argument('--runs', default='/workspace/bench/low_delay_runs')
args = parser.parse_args()
root = Path(args.runs)
summary = json.loads((root / 'summary.json').read_text())
report = {'clock_semantics': 'P/I/C use the same monotonic node clock. Recorder timestamps use wall clock and their maxima are descriptive, not directly subtracted from monotonic timestamps.',
          'interpretation_limit': 'A small queue-to-publish duration for the current sample does not prove the whole interval was upstream: inspect the first rate-eligible sample arrival too.',
          'cases': {}}
for case in ['flatwall05', 'glim_original', 'glim_retimed']:
    rows = [row for row in summary['metrics'] if row['run'].startswith(case + '_fixed_')]
    row = max(rows, key=lambda r: r['publish_wall_intervals_s']['max'])
    traces = trace_data(root / row['run'] / 'output_trace.tsv')
    pub = traces['P']
    index = int(np.argmax(np.diff(pub[:, -1])))
    prior, current = pub[index], pub[index + 1]
    incoming = traces['I']
    matches = incoming[incoming[:, 0] == current[0]]
    current_enqueue = matches[-1, -1] if len(matches) else None
    eligible = incoming[(incoming[:, 0] >= prior[0] + .02 - 1e-6) & (incoming[:, 0] <= current[0])]
    earliest = eligible[np.argmin(eligible[:, -1])] if len(eligible) else None
    callback = traces.get('C', np.empty((0, 3)))
    callbacks_current = callback[callback[:, 0] == current[0]] if len(callback) else []
    callback_time = callbacks_current[-1, -1] if len(callbacks_current) else None
    in_interval = incoming[(incoming[:, -1] > prior[-1]) & (incoming[:, -1] <= current[-1])]
    report['cases'][case] = {
        'run': row['run'], 'publication_interval_index_zero_based': index,
        'previous_sensor_stamp': float(prior[0]), 'current_sensor_stamp': float(current[0]),
        'sensor_interval_s': float(current[0] - prior[0]),
        'previous_publish_steady_s': float(prior[-1]), 'current_publish_steady_s': float(current[-1]),
        'publish_wall_gap_s': float(current[-1] - prior[-1]),
        'current_sample_enqueue_steady_s': None if current_enqueue is None else float(current_enqueue),
        'current_sample_callback_steady_s': None if callback_time is None else float(callback_time),
        'current_sample_enqueue_after_previous_publish_s': None if current_enqueue is None else float(current_enqueue - prior[-1]),
        'current_sample_queue_to_publish_s': None if current_enqueue is None else float(current[-1] - current_enqueue),
        'earliest_eligible_sensor_stamp': None if earliest is None else float(earliest[0]),
        'earliest_eligible_enqueue_steady_s': None if earliest is None else float(earliest[-1]),
        'earliest_eligible_arrival_after_previous_publish_s': None if earliest is None else float(earliest[-1] - prior[-1]),
        'earliest_eligible_arrival_to_current_publish_s': None if earliest is None else float(current[-1] - earliest[-1]),
        'imu_samples_enqueued_during_interval': len(in_interval),
        'run_max_recorded_imu_receipt_interval_s': row['recorded_imu_delivery_intervals_s']['max'],
        'run_max_imu_callback_wall_interval_s': row['imu_callback_delivery_intervals_s']['max'],
        'run_max_odom_receipt_interval_s': row['odom_receipt_intervals_s']['max'],
        'case_max_recorded_imu_receipt_interval_s': max(r['recorded_imu_delivery_intervals_s']['max'] for r in rows),
        'case_max_imu_callback_wall_interval_s': max(r['imu_callback_delivery_intervals_s']['max'] for r in rows),
        'case_max_odom_receipt_interval_s': max(r['odom_receipt_intervals_s']['max'] for r in rows),
    }
(root / 'worst_delivery_audit.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(report, indent=2))
