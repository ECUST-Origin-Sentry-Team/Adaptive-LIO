#!/usr/bin/env python3
"""Replay the same physical IMU-gap derivative through both actual binaries."""
import argparse
import os
from pathlib import Path
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument('--output', default='/workspace/bench/low_delay_fault')
parser.add_argument('--container', default='adaptive-lio-run')
args = parser.parse_args()
root = Path(__file__).resolve().parent
out = Path(args.output)
out.mkdir(parents=True, exist_ok=True)
for version in ['low', 'fixed']:
    print('FAULT RUN', version, flush=True)
    config = root / ('flatwall_low.yaml' if version == 'low' else 'flatwall.yaml')
    command = ['docker', 'exec', '--user', f'{os.getuid()}:{os.getgid()}', args.container,
               'bash', '-lc', 'source /opt/ros/jazzy/setup.bash && '
               'source /workspace/install-deps/setup.bash && exec python3 "$@"', 'runner',
               str(root / 'run_one.py'), '--version', version,
               '--binary', f'/workspace/install-{version}/adaptive_lio/lib/adaptive_lio/adaptive_lio',
               '--config', str(config), '--bag', '/workspace/datasets/flatwall/flatwall_05_fault_gap',
               '--output', str(out / version)]
    subprocess.run(command, check=True)
