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
    ap.add_argument('--domain', type=int, default=83)
    ap.add_argument('--version', choices=['fixed', 'low'], required=True)
    ap.add_argument('--drain', type=float, default=15)
    args = ap.parse_args()
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=False)
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain), ROS_LOG_DIR=str(out / 'ros_logs'),
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
    (out / 'config.snapshot.yaml').write_bytes(Path(args.config).read_bytes())
    (out / 'replay_qos.snapshot.yaml').write_bytes(Path(__file__).with_name('replay_qos.yaml').read_bytes())
    result['playback_rate'] = 1
    result['player_delay_s'] = 3
    result['recorder_storage'] = 'sqlite3'
    result['qos_overrides_sha256'] = hashlib.sha256(Path(__file__).with_name('replay_qos.yaml').read_bytes()).hexdigest()
    try:
        node_command = [args.binary, '--config_file', args.config,
                        '--trajectory_file', str(out / 'posterior.tum')]
        node = launch('node', node_command)
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
        result['node_exited_before_requested_stop'] = node.poll() is not None
        result['node_exit'] = stop(node)
        result['expected_legacy_shutdown_abort'] = (args.version == 'low' and result['node_exit'] == -6 and not result['node_exited_before_requested_stop'] and result['player_exit'] == 0)
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
    if result.get('player_exit') != 0 or result.get('recorder_exit') != 0 or (result.get('node_exit') != 0 and not result.get('expected_legacy_shutdown_abort')):
        raise RuntimeError('nonzero process result: ' + json.dumps(result))


if __name__ == '__main__':
    main()
