# Rebuild and validation protocol

Date: 2026-10-31. Upstream base: `97a3b0edd27dbf03b9577f27c0e1ae2eff08b581` (`ljb`).
This work reconstructs lost local fixes; it does not reuse earlier claims as measured results.

## Reproduction

The scripts use `/workspace` inside and outside Docker. Container name is `adaptive-lio-run`.
Use `ros:jazzy-ros-base` (tested image digest `sha256:8eed0a4669bbf43ee331878a4c39bd8ebddea8e3312a58b4fae8edb14526b617`), bind `/workspace`, and provide the environment's combined CA bundle read-only at `/run/host-ca.pem`. Preserve configured Docker proxy defaults.

1. Clone this repository to `/workspace/Adaptive-LIO`; checkout the repair branch.
2. Create baseline worktree and apply only the observer/control patch:
   ```sh
   git -C /workspace/Adaptive-LIO worktree add --detach /workspace/baseline 97a3b0edd27dbf03b9577f27c0e1ae2eff08b581
   git -C /workspace/baseline apply /workspace/Adaptive-LIO/benchmark/rebuild/baseline_observer.patch
   ```
3. Inside container run `bash /workspace/Adaptive-LIO/benchmark/rebuild/install_dependencies.sh`, then `bash /workspace/Adaptive-LIO/benchmark/rebuild/build.sh`.
4. On host run `bash benchmark/rebuild/data_fetch.sh` (Python venv support required). Source links, checksums and conversions are in `data_sources.json` and `evidence`.
5. Inside container source `/opt/ros/jazzy/setup.bash` and `/workspace/install-deps/setup.bash`, then run:
   ```sh
   ctest --test-dir /workspace/build-fixed/adaptive_lio -R '^adaptive_lio_' --output-on-failure
   bash /workspace/Adaptive-LIO/benchmark/rebuild/run_regression_probe.sh
   ```
6. When all build/test activity is idle, on host run `python3 benchmark/rebuild/run_suite.py`.
7. Analyze using `/workspace/bench/data_venv/bin/python benchmark/rebuild/analyze.py`, then `check_acceptance.py`. Failure is reported, not silently excluded. The acceptance file covers sensor-time behavior; receiver wall stalls remain a separately reported result.

`run_one.py` runs **inside** the sourced ROS container and also supports the fault derivative bag. `analyze_fault.py` verifies FAULT reception/no subsequent received odometry.

## Controlled comparison

The baseline contains original97a predictor and output worker (0.5s gate). Both versions receive identical deterministic per-frame sampling, reliable KEEP_ALL IMU on a dedicated executor, LiDAR posterior observer and optional TUM/input traces. This controls delivery/sampling effects; it is explicitly not a pristine97a binary or the lost previous1s binary. The exact baseline patch is versioned. Separate same-source core probes compare0.5s and1s gates against the repair.

Formal runs: three input conditions (FlatWall05, GLIM original storage times, GLIM header-retimed), two repeats, each fixed+baseline. Repeat a order fixed→baseline, repeat b baseline→fixed. Playback1x, player delay3s, drain15s. Runs are sequential and do not overlap compilation/tests. Executable and config hashes are captured per run. Tracing enabled, hence reported latency includes diagnostic overhead.

## Interpretation

- No duplicate/backward output timestamps; complete input callbacks; no missing publications; finite poses/covariances; sensor-time gaps; explicit health states.
- Also report initial/final timestamp coverage, not only interior max gaps. A permanently stopped output can have excellent interior intervals.
- Wall-clock gaps and queue residence measure responsiveness. Continuous sensor coverage may be delivered as a late burst; it is not a hard real-time guarantee.
- GT evaluation uses nearest associations within30ms and the intersection shared by all compared runs, rigidSE3 translation alignment with no scale. Report total/shared counts. This is position APE; no claim of orientation accuracy or calibrated covariance.
- FlatWall GT is fiducial-derived. GLIM GT is batch optimization against a survey map, not independent motion capture.
- GLIM retimed payloads and per-topic order are verified identical to original; only storage schedule changes. This newly generated derivative cannot be asserted byte-identical to the lost earlier derivative.
- `/odom/lidar` includes committed prediction-only frames if registration is rejected. Review `lio_quality` logs before interpreting every pose as an accepted LiDAR match.
- Pure frontend operation is supported. Without fresh LiDAR constraints, IMU drift grows; DEGRADED is a meaningful warning, not a promise of unchanged accuracy.
