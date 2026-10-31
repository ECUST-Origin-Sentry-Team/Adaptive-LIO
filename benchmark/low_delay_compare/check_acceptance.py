#!/usr/bin/env python3
"""Validate experiment integrity and fixed-version guarantees separately from old behavior."""
import argparse
import json
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('--runs', default='/workspace/bench/low_delay_runs')
args = parser.parse_args()
root = Path(args.runs)
report = json.loads((root / 'summary.json').read_text())
metrics = report['metrics']
fixed = [row for row in metrics if row['processes']['version'] == 'fixed']
old = [row for row in metrics if row['processes']['version'] == 'low']
checks = {'twelve_formal_runs': len(metrics) == 12, 'six_per_version': len(fixed) == len(old) == 6}
by_run = {}
for row in metrics:
    process = row['processes']
    tests = {
        'complete_recorded_imu': row.get('recorded_imu_matches_source') is True,
        'successful_playback_and_recording': process.get('player_exit') == process.get('recorder_exit') == 0,
        'node_survived_until_requested_stop': process.get('node_exited_before_requested_stop') is False,
        'node_exit_classified': process.get('node_exit') == 0 or process.get('expected_legacy_shutdown_abort') is True,
        'nonempty_odometry': row.get('odom_count', 0) > 0,
    }
    if process['version'] == 'fixed':
        intervals = row.get('odom_sensor_intervals_s', {})
        tests.update({
            'complete_imu_callbacks': row.get('imu_callback_matches_source') is True,
            'finite_odometry': row.get('odom_all_finite') is True,
            'monotonic_unique_stamps': intervals.get('negative') == intervals.get('zero') == row.get('odom_duplicate_stamps_total') == 0,
            'no_sensor_holes_over_half_second': intervals.get('over_0_5') == 0,
            'prediction_reaches_input_tail': -1e-6 <= row.get('odom_tail_to_last_imu_s', 999) <= .025 + 1e-6,
            'all_publications_received': row.get('published_vs_received_count_equal') is True and row.get('published_vs_received_stamp_max_abs_s', 1) < 1e-6,
            'no_fault_status': row.get('status_counts', {}).get('FAULT', 0) == 0,
            'health_status_available': row.get('health_status_available') is True,
            'clean_shutdown': process.get('node_exit') == 0,
        })
    by_run[row['run']] = tests
checks['all_integrity_and_fixed_guarantees'] = all(all(tests.values()) for tests in by_run.values())
result = {'pass': all(checks.values()), 'checks': checks, 'by_run': by_run,
          'scope': 'Old holes, callback loss, duplicates, held positions and empty covariance are measured outcomes, not hidden or excluded. Old post-SIGINT abort is explicitly retained. No expectation of inner algorithm equivalence or hard wall-time guarantee.'}
(root / 'acceptance.json').write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps(result, indent=2))
raise SystemExit(0 if result['pass'] else 1)
