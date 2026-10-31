#!/usr/bin/env python3
"""Locate observed gaps; descriptive startup split does not change acceptance."""
import argparse
import json
from pathlib import Path
import numpy as np
from analyze import trajectory

parser = argparse.ArgumentParser()
parser.add_argument('--runs', default='/workspace/bench/low_delay_runs')
args = parser.parse_args()
root = Path(args.runs)
summary = json.loads((root / 'summary.json').read_text())
report = {'definition': 'Gap index zero denotes the first interval between sorted unique odom timestamps. Excluding that first interval is descriptive only; original acceptance and maximum-gap metrics remain unchanged.', 'runs': {}, 'cases': {}}
for row in summary['metrics']:
    raw = trajectory(root / row['run'] / 'odom.tum')
    stamps = np.unique(raw[:, 0])
    gaps = np.diff(stamps)
    index = int(np.argmax(gaps))
    item = {'version': row['processes']['version'], 'first_unique_stamp': float(stamps[0]),
            'second_unique_stamp': float(stamps[1]), 'first_unique_interval_s': float(gaps[0]),
            'max_unique_interval_s': float(gaps[index]), 'max_unique_interval_index_zero_based': index,
            'max_gap_left_stamp': float(stamps[index]), 'max_gap_right_stamp': float(stamps[index + 1]),
            'max_gap_is_first_interval': index == 0,
            'max_unique_interval_excluding_first_s': float(np.max(gaps[1:])) if len(gaps) > 1 else None,
            'max_receipt_interval_s': row['odom_receipt_intervals_s']['max'],
            'mean_unique_sensor_rate_hz': row['odom_unique_sensor_mean_rate_hz'],
            'median_unique_sensor_period_s': row['odom_unique_sensor_intervals_s']['median']}
    report['runs'][row['run']] = item
    case = row['run'].rsplit('_', 2)[0]
    report['cases'].setdefault(case, {}).setdefault(item['version'], []).append(item)
for case, versions in report['cases'].items():
    for version, rows in versions.items():
        versions[version] = {'max_receipt_interval_s': max(r['max_receipt_interval_s'] for r in rows),
                             'mean_unique_sensor_rate_hz_range': [min(r['mean_unique_sensor_rate_hz'] for r in rows), max(r['mean_unique_sensor_rate_hz'] for r in rows)],
                             'max_unique_interval_excluding_first_s': max(r['max_unique_interval_excluding_first_s'] for r in rows),
                             'all_max_gaps_are_first_interval': all(r['max_gap_is_first_interval'] for r in rows)}
(root / 'gap_audit.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(report, indent=2))
