#!/usr/bin/env python3
"""Machine-readable acceptance for reconstructed fixed build; never hides failures."""
import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--runs', default='/workspace/bench/rebuild_runs')
    args = parser.parse_args()
    root = Path(args.runs)
    report = json.loads((root / 'summary.json').read_text())
    fixed = [m for m in report['metrics'] if '_fixed_' in m['run']]
    checks = {'fixed_run_count_six': len(fixed) == 6}
    by_run = {}
    for row in fixed:
        interval = row.get('odom_sensor_intervals_s', {})
        tests = {
            'complete_imu_callbacks': row.get('imu_callback_matches_source') is True,
            'complete_recorded_imu': row.get('recorded_imu_matches_source') is True,
            'finite_odometry': row.get('odom_all_finite') is True,
            'nonempty_odometry': row.get('odom_count', 0) > 0,
            'prediction_reaches_input_tail': -1e-6 <= row.get('odom_tail_to_last_imu_s', 999) <= .025 + 1e-6,
            'no_sensor_holes_over_half_second': interval.get('over_0_5') == 0,
            'no_duplicate_or_backward_stamp': interval.get('negative') == 0 and interval.get('zero') == 0,
            'all_publications_received': row.get('published_vs_received_count_equal') is True and row.get('published_vs_received_stamp_max_abs_s', 1) < 1e-6,
            'no_fault_status': row.get('status_counts', {}).get('FAULT', 0) == 0,
            'status_received': sum(row.get('status_counts', {}).values()) > 0,
            'lidar_posterior_complete': row.get('lidar_count', 0) == row.get('posterior_count', -1) > 0,
            'clean_process_exit': all(row['processes'].get(k) == 0 for k in ('node_exit', 'player_exit', 'recorder_exit')),
        }
        by_run[row['run']] = tests
    checks['all_fixed_run_checks'] = bool(by_run) and all(all(tests.values()) for tests in by_run.values())
    checks['all_inner_posteriors_exactly_equal'] = bool(report['posterior_comparison']) and all(
        row['exactly_equal'] for case in report['posterior_comparison'].values() for row in case.values())
    result = {'checks': checks, 'by_run': by_run,
              'pass': all(checks.values()),
              'scope': 'Sensor-time continuity, input/output integrity, inner posterior invariance. Wall-time delivery stalls reported separately; no claim of hard real time or improved accuracy.'}
    (root / 'acceptance.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['pass'] else 1)


if __name__ == '__main__':
    main()
