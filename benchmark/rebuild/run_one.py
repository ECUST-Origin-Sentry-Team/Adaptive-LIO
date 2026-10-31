#!/usr/bin/env python3
"""Run inside sourced ROS 2 container. Record wall-delivery and sensor-time evidence."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def stop(proc):
    if proc.poll() is not None:
        return proc.returncode
    os.killpg(proc.pid, signal.SIGINT)
    try:
        return proc.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()
        return -999


def main():
    ap = argparse.ArgumentParser()
    for name in ('binary', 'bag', 'config', 'output'):
        ap.add_argument('--' + name, required=True)
    ap.add_argument('--domain', type=int, default=81)
    ap.add_argument('--drain', type=float, default=15)
    args = ap.parse_args()
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=False)
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain),
               ADAPTIVE_LIO_OUTPUT_TRACE=str(out / 'output_trace.tsv'),
               ADAPTIVE_LIO_IMU_TRACE=str(out / 'imu_trace.tsv'))
    handles = []
    processes = []

    def launch(name, command):
        handle = (out / (name + '.log')).open('w')
        handles.append(handle)
        proc = subprocess.Popen(command, env=env, stdout=handle,
                                stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(proc)
        return proc

    result = dict(vars(args), started_unix=time.time())
    for key in ('binary', 'config'):
        result[key + '_sha256'] = hashlib.sha256(Path(getattr(args, key)).read_bytes()).hexdigest()
    result['playback_rate'] = 1
    result['player_delay_s'] = 3
    result['recorder_storage'] = 'sqlite3'
    result['qos_overrides_sha256'] = hashlib.sha256(Path(__file__).with_name('replay_qos.yaml').read_bytes()).hexdigest()
    try:
        node = launch('node', [args.binary, '--config_file', args.config,
                              '--trajectory_file', str(out / 'posterior.tum')])
        recorder = launch('recorder', ['ros2', 'bag', 'record', '-s', 'sqlite3',
                                      '-o', str(out / 'recording'), '/odom',
                                      '/odom/lidar', '/odom/status', '/livox/imu'])
        time.sleep(3)
        if node.poll() is not None or recorder.poll() is not None:
            raise RuntimeError('node or recorder exited before playback')
        player = launch('player', ['ros2', 'bag', 'play', args.bag, '--rate', '1',
                                  '--delay', '3', '--clock',
                                  '--qos-profile-overrides-path',
                                  str(Path(__file__).with_name('replay_qos.yaml'))])
        result['player_exit'] = player.wait(timeout=300)
        time.sleep(args.drain)
        result['node_exit'] = stop(node)
        time.sleep(1)
        result['recorder_exit'] = stop(recorder)
    except Exception as error:
        result['error'] = str(error)
        raise
    finally:
        for proc in reversed(processes):
            stop(proc)
        for handle in handles:
            handle.close()
        result['finished_unix'] = time.time()
        (out / 'run.json').write_text(json.dumps(result, indent=2) + '\n')
    if any(result.get(key, 1) != 0 for key in ('player_exit', 'node_exit', 'recorder_exit')):
        raise RuntimeError('nonzero process result: ' + json.dumps(result))


if __name__ == '__main__':
    main()
