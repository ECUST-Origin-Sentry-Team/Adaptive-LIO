# Current continuous output vs Low-Delay-Time

Date: 2026-10-31. This is a new paired experiment; previous ljb-baseline numbers are not reused.

## Versions

- Fixed: runtime source commit16185f2576ed70d3d5b7fa7f98e8e423a5cb8c51, published source/evidence checkpoint3483a8a; binary SHA2560362c6ef5c78635cb3d2355a8042ff1bc97736d180856064fe05d72a410bed0a. Runtime unchanged for this comparison.
- Legacy: actual case-sensitive branch `Low-Delay-Time`, commit24e6ca33b5878d97c5515b4d3350ce790399bb8d. Binary50de9bc73f5466f30723a6e04c591c0a8985a4cfff7ac3376ab28bb304be593f.
- Legacy patch only excludes unused obsolete Ceres LocalParameterization source/includes and adds opt-in observers. Active solver, queues, QoS, prediction, publication timestamps and shutdown behavior are preserved. See legacy_compat_observers.patch and legacy_provenance.json.

## Comparison and interpretation

Use the same public FlatWall05, GLIM Livox original and header-retimed bags described in ../rebuild/data_sources.json. Record IMU, odom, optional lidar posterior and health topics at1x playback. Two repeats per condition, alternating fixed→low and low→fixed; no concurrent builds/tests/analysis during formal runs. Twelve fresh nominal runs, then one true80ms missing-IMU derivative run per version.

Both versions share sensor units (acceleration scale9.80665), identity calibration and common supported config values. Legacy-only required fields retain original legacy YAML values (motion mode, ICP selector, degenerate window, beta/saturation fields and map_options); unused auxiliary topic placeholder permits old unconditional subscription. This is a comparison of delivered versions, not an ablation isolating only outer publication code. The inner estimators and random sampling also differ, so APE differences cannot be attributed solely to output changes.

Metrics distinguish:

1. Source IMU timestamp continuity, receiver input completeness and node callback completeness.
2. Raw odom repeated/backward stamps versus sorted unique sensor-time coverage and tail cessation.
3. Node publish wall intervals versus recorder receive wall intervals; trace includes instrumentation overhead.
4. Native frequency (old roughlyIMUrate, fixed target50Hz); higher message rate alone is not fuller prediction.
5. Consecutive identical positions (<=1e-12m) with motion/GT context, twist/covariance population and health availability.
6. Common-GT translation APE using last received pose at duplicate timestamps, nearest associations<=30ms, SE3 no scale. Report shared GT coverage; no statistical accuracy superiority claim from two repeats.
7. Real missing-IMU behavior: fixed50ms gap policy faults, legacy200ms policy may continue through80ms missing input. Continued publication is not proof of intact measurements.

Legacy's unjoined measurement thread causes abort(-6) after requested SIGINT. The runner accepts this **only after successful playback and the requested shutdown**, marks it explicitly, and does not count it as a running odom gap. Unexpected earlier exit remains an experiment failure. Old observers flush before this known shutdown abort. A drained recording can be complete despite the exit failure.

## Reproduction

Use the ROS2Jazzy container/dependencies and public fixtures from ../rebuild/README.md. Add the old worktree:

```sh
git -C /workspace/Adaptive-LIO worktree add --detach /workspace/low-delay 24e6ca33b5878d97c5515b4d3350ce790399bb8d
git -C /workspace/low-delay apply /workspace/Adaptive-LIO/benchmark/low_delay_compare/legacy_compat_observers.patch
```

Inside sourced ROS container, source `/workspace/install-deps/setup.bash`, then:

```sh
colcon --log-base /workspace/bench/log-low build --base-paths /workspace/low-delay --build-base /workspace/build-low --install-base /workspace/install-low --packages-select adaptive_lio --parallel-workers 1 --cmake-args -DBUILD_TESTING=OFF
```

Host: `python3 benchmark/low_delay_compare/run_suite.py`; use a fresh output directory for every rerun. Each run records executable/config/QoS hashes and full config snapshots. Analyze with the rosbags/numpy venv using `analyze.py`, `check_acceptance.py`; fault comparisons use `analyze_fault.py`. Source audit and exact behavior citations: [SEMANTICS.md](SEMANTICS.md).

Normal data and fault injection answer different questions. A pause in delivery may be followed by a complete burst: evaluate receipt intervals as well as sensor timestamp coverage. These experiments cannot guarantee hard real-time behavior on arbitrary hardware or load.

After the12nominal runs, run `python3 benchmark/low_delay_compare/run_fault_pair.py` on the host. Once all playback has stopped, use the data venv to run `analyze.py`, `check_acceptance.py`, `analyze_fault.py`, `audit_gap_locations.py`, and `audit_delivery.py` in this folder. Then `write_report.py` and `package_evidence.py` reproduce the human-readable report and complete run archives. The added gap audits are descriptive; they do not change the original0.5s threshold or remove first intervals from original statistics.

Results: [RESULTS.md](RESULTS.md). Complete recorded output, input IMU observations, traces, config snapshots, trajectories and logs are stored in `evidence/runs`; source point-cloud bags are recovered through ../rebuild/data_fetch.sh rather than duplicated in Git.
