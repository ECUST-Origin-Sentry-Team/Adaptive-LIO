#!/usr/bin/env python3
"""Host-side paired runner. Build/tests must be idle during formal runs."""
import argparse
import json
import os
from pathlib import Path
import subprocess

ROOT = Path('/workspace/Adaptive-LIO/benchmark/rebuild')
CASES = {
    'flatwall05': ('/workspace/datasets/flatwall/flatwall_05_ros2', 'flatwall.yaml'),
    'glim_original': ('/workspace/datasets/glim_cross_sensor/livox_ros2', 'glim.yaml'),
    'glim_retimed': ('/workspace/datasets/glim_cross_sensor/livox_retimed', 'glim.yaml'),
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--output', default='/workspace/bench/rebuild_runs')
    ap.add_argument('--container', default='adaptive-lio-run')
    ap.add_argument('--case', choices=list(CASES))
    ap.add_argument('--repeat', choices=['a', 'b'])
    args = ap.parse_args()
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    order = []
    for case, (bag, config) in CASES.items():
        if args.case and case != args.case:
            continue
        for repeat, versions in [('a', ['fixed', 'baseline']), ('b', ['baseline', 'fixed'])]:
            if args.repeat and repeat != args.repeat:
                continue
            for version in versions:
                run = out / f'{case}_{version}_{repeat}'
                if (run / 'run.json').exists():
                    raise RuntimeError(f'refusing to overwrite {run}')
                binary = f'/workspace/install-{version}/adaptive_lio/lib/adaptive_lio/adaptive_lio'
                command = ['docker', 'exec', '--user', f'{os.getuid()}:{os.getgid()}', args.container, 'bash', '-lc',
                           'source /opt/ros/jazzy/setup.bash && '
                           'source /workspace/install-deps/setup.bash && '
                           'exec python3 "$@"', 'runner', str(ROOT / 'run_one.py'),
                           '--binary', binary, '--bag', bag, '--config', str(ROOT / config),
                           '--output', str(run)]
                print('RUN', case, version, repeat, flush=True)
                subprocess.run(command, check=True)
                order.append({'case': case, 'version': version, 'repeat': repeat, 'path': str(run)})
                (out / 'order.json').write_text(json.dumps(order, indent=2) + '\n')


if __name__ == '__main__':
    main()
