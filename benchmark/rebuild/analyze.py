#!/usr/bin/env python3
"""Analyze bag timestamps without conflating sensor continuity with delivery latency.

Dependencies: numpy, rosbags (pip). Output JSON is the evidence source; no old
benchmark numbers are imported. APE uses common GT points and rigid SE(3) alignment.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import numpy as np
from rosbags.highlevel import AnyReader


def stamp(msg):
    return msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec


def stats(values):
    values = np.asarray(values, dtype=float)
    return {'count': int(len(values)), 'max': float(np.max(values)) if len(values) else None,
            'p99': float(np.percentile(values, 99)) if len(values) else None,
            'over_0_5': int(np.sum(values > .5)), 'negative': int(np.sum(values < 0)),
            'zero': int(np.sum(values == 0))}


def trajectory(path):
    if not path.exists() or path.stat().st_size == 0:
        return np.empty((0, 8))
    return np.atleast_2d(np.loadtxt(path))


def bag_data(path):
    data = {'odom': [], 'lidar': [], 'imu': [], 'status': []}
    with AnyReader([path]) as reader:
        for conn, receipt, raw in reader.messages():
            if conn.topic not in ['/odom', '/odom/lidar', '/odom/status', '/livox/imu']:
                continue
            msg = reader.deserialize(raw, conn.msgtype)
            if conn.topic == '/odom/status':
                data['status'].append((receipt * 1e-9, json.loads(msg.data)))
            elif conn.topic == '/livox/imu':
                data['imu'].append([stamp(msg), receipt * 1e-9])
            else:
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                row = [stamp(msg), p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                       receipt * 1e-9, *msg.pose.covariance, *msg.twist.covariance]
                data['odom' if conn.topic == '/odom' else 'lidar'].append(row)
    return data


def source_imu(path):
    stamps = []
    with AnyReader([path]) as reader:
        for conn, _, raw in reader.messages(connections=[c for c in reader.connections if c.topic == '/livox/imu']):
            stamps.append(stamp(reader.deserialize(raw, conn.msgtype)))
    return np.asarray(stamps)


def trace_data(path):
    events = {}
    if not path.exists():
        return events
    for line in path.read_text().splitlines():
        cols = line.replace(',', ' ').split()
        if len(cols) != 4:
            continue
        try:
            row = [float(x) for x in cols[1:]]
        except ValueError:
            continue
        events.setdefault(cols[0], []).append(row)
    return {event: np.asarray(rows) for event, rows in events.items()}


def analyze(run, source):
    data = bag_data(run / 'recording')
    result = {'run': run.name, 'processes': json.loads((run / 'run.json').read_text())}
    odom = np.asarray(data['odom'])
    lidar = np.asarray(data['lidar'])
    imu = np.asarray(data['imu'])
    expected = source_imu(source)
    result['source_imu_count'] = len(expected)
    result['recorded_imu_count'] = len(imu)
    result['recorded_imu_matches_source'] = bool(len(imu) == len(expected) and np.array_equal(imu[:, 0], expected))
    result['source_imu_intervals_s'] = stats(np.diff(expected))
    callback_path = run / 'imu_trace.tsv'
    if callback_path.exists() and callback_path.stat().st_size:
        callbacks = np.atleast_2d(np.loadtxt(callback_path))
        result['imu_callback_count'] = len(callbacks)
        result['imu_callback_matches_source'] = bool(len(callbacks) == len(expected) and np.array_equal(callbacks[:, 0], expected))
        result['imu_callback_decisions'] = dict(Counter(str(int(x)) for x in callbacks[:, 1]))
        result['imu_callback_delivery_intervals_s'] = stats(np.diff(callbacks[:, 3]))
    result['recorded_imu_delivery_intervals_s'] = stats(np.diff(imu[:, 1])) if len(imu) else None
    for key, values in [('odom', odom), ('lidar', lidar)]:
        result[key + '_count'] = len(values)
        if not len(values):
            continue
        result[key + '_all_finite'] = bool(np.all(np.isfinite(values)))
        result[key + '_sensor_intervals_s'] = stats(np.diff(values[:, 0]))
        result[key + '_receipt_intervals_s'] = stats(np.diff(values[:, 8]))
        result[key + '_sensor_span_s'] = float(values[-1, 0] - values[0, 0])
        result[key + '_pose_cov_trace_range'] = [float(np.min(np.sum(values[:, 9:45:7], axis=1))),
                                                float(np.max(np.sum(values[:, 9:45:7], axis=1)))]
        np.savetxt(run / (key + '.tum'), values[:, :8], fmt='%.17g')
    posterior = trajectory(run / 'posterior.tum')
    result['posterior_count'] = len(posterior)
    if len(lidar) and len(posterior) == len(lidar):
        result['lidar_vs_posterior_max_abs'] = np.max(np.abs(lidar[:, :8] - posterior), axis=0).tolist()
    statuses = data['status']
    result['status_counts'] = dict(Counter(row[1]['state'] for row in statuses))
    result['status_input_stale_count'] = sum(bool(row[1].get('input_stale')) for row in statuses)
    trace = trace_data(run / 'output_trace.tsv')
    result['trace_counts'] = {key: len(rows) for key, rows in trace.items()}
    if 'P' in trace:
        pub = trace['P']
        result['publish_wall_intervals_s'] = stats(np.diff(pub[:, -1]))
        result['publish_sensor_intervals_s'] = stats(np.diff(pub[:, 0]))
        result['max_correction_age_s'] = float(np.max(pub[:, 1]))
        result['stale_prediction_sensor_duration_s'] = float(np.sum(np.diff(pub[:, 0]) * (pub[:-1, 1] > .5)))
        result['published_vs_received_count_equal'] = len(pub) == len(odom)
        if len(pub) == len(odom):
            result['published_vs_received_stamp_max_abs_s'] = float(np.max(np.abs(pub[:, 0] - odom[:, 0])))
        if 'D' in trace and len(trace['D']) == len(pub):
            result['publish_call_duration_s'] = stats(trace['D'][:, -1] - pub[:, -1])
        if 'R' in trace and 'E' in trace and len(trace['R']) == len(trace['E']):
            result['replay_duration_s'] = stats(trace['E'][:, -1] - trace['R'][:, -1])
        if 'I' in trace:
            incoming = {row[0]: row[-1] for row in trace['I']}
            delays = [row[-1] - incoming[row[0]] for row in pub if row[0] in incoming]
            result['published_imu_enqueue_to_publish_s'] = stats(delays)
            audit = []
            for i in np.where(np.diff(pub[:, -1]) > .5)[0]:
                prior, current = pub[i], pub[i + 1]
                enqueued = incoming.get(current[0])
                eligible = trace['I'][(trace['I'][:, 0] >= prior[0] + .02 - 1e-6) &
                                      (trace['I'][:, 0] <= current[0])]
                audit.append({'previous_sensor_time': float(prior[0]), 'next_sensor_time': float(current[0]),
                              'publish_wall_gap_s': float(current[-1] - prior[-1]),
                              'next_sample_arrived_after_previous_publish_s': None if enqueued is None else float(enqueued - prior[-1]),
                              'next_sample_enqueue_to_publish_s': None if enqueued is None else float(current[-1] - enqueued),
                              'first_rate_eligible_arrived_after_previous_publish_s': None if not len(eligible) else float(np.min(eligible[:, -1]) - prior[-1])})
            result['wall_gap_audit'] = audit
    (run / 'metrics.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    return result


def nearest_indices(gt_times, estimate_times):
    right = np.clip(np.searchsorted(estimate_times, gt_times), 0, len(estimate_times) - 1)
    left = np.maximum(right - 1, 0)
    return np.where(np.abs(estimate_times[left] - gt_times) < np.abs(estimate_times[right] - gt_times), left, right)


def aligned_rmse(estimate, truth):
    ec, tc = estimate - estimate.mean(axis=0), truth - truth.mean(axis=0)
    u, _, vt = np.linalg.svd(ec.T @ tc)
    reflection = np.eye(3)
    reflection[-1, -1] = np.linalg.det(u @ vt)
    aligned = ec @ (u @ reflection @ vt) + truth.mean(axis=0)
    error = np.linalg.norm(aligned - truth, axis=1)
    return {'rmse_m': float(np.sqrt(np.mean(error ** 2))), 'max_m': float(np.max(error))}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', default='/workspace/bench/rebuild_runs')
    args = ap.parse_args()
    root = Path(args.runs)
    cases = {
        'flatwall05': ('/workspace/datasets/flatwall/flatwall_05_ros2', '/workspace/datasets/flatwall/gt/flatwall_05.txt'),
        'glim_original': ('/workspace/datasets/glim_cross_sensor/livox_ros2', '/workspace/datasets/glim_cross_sensor/gt/livox.txt'),
        'glim_retimed': ('/workspace/datasets/glim_cross_sensor/livox_retimed', '/workspace/datasets/glim_cross_sensor/gt/livox.txt'),
    }
    report = {'metrics': [], 'posterior_comparison': {}, 'common_gt_ape': {}}
    for case, (source, gt_path) in cases.items():
        runs = sorted(p for p in root.glob(case + '_*') if (p / 'run.json').exists())
        for run in runs:
            report['metrics'].append(analyze(run, Path(source)))
        if not runs:
            continue
        ref = trajectory(runs[0] / 'posterior.tum')
        comparison = {}
        for run in runs:
            other = trajectory(run / 'posterior.tum')
            same_shape = other.shape == ref.shape
            comparison[run.name] = {'reference': runs[0].name, 'same_shape': same_shape,
                                    'exactly_equal': same_shape and bool(np.array_equal(ref, other)),
                                    'max_abs': float(np.max(np.abs(ref - other))) if same_shape and ref.size else None}
        report['posterior_comparison'][case] = comparison
        gt = trajectory(Path(gt_path))
        estimates = [trajectory(run / 'odom.tum') for run in runs]
        if not len(gt) or any(not len(e) for e in estimates):
            continue
        common = np.ones(len(gt), dtype=bool)
        indices = []
        for estimate in estimates:
            ix = nearest_indices(gt[:, 0], estimate[:, 0])
            common &= np.abs(gt[:, 0] - estimate[ix, 0]) <= .03
            indices.append(ix)
        ape = {'gt_count': len(gt), 'common_count': int(np.sum(common)), 'association_limit_s': .03,
               'alignment': 'rigid SE3, no scale; common GT nearest timestamp intersection', 'runs': {}}
        if np.sum(common) >= 3:
            for run, estimate, ix in zip(runs, estimates, indices):
                ape['runs'][run.name] = aligned_rmse(estimate[ix[common], 1:4], gt[common, 1:4])
        report['common_gt_ape'][case] = ape
    (root / 'summary.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    for row in report['metrics']:
        print(row['run'], 'odom', row['odom_count'], 'sensor', row.get('odom_sensor_intervals_s'),
              'receipt', row.get('odom_receipt_intervals_s'))


if __name__ == '__main__':
    main()
