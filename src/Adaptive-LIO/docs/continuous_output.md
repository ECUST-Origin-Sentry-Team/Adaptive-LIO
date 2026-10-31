# Continuous IMU odometry output

This reconstruction starts from `ljb` 97a3b0e. It is a standalone frontend odometer; no pose graph or loop closure is required for this repair.

## Topics

- `/odom`: predicted pose/twist/covariance at a real processed IMU stamp, target50Hz sensor time; strict stamp monotonicity. Recovery can change pose; continuity does not mean a correction-free/smoothed trajectory.
- `/odom/lidar`: committed inner pose with pose covariance at measurement time. Twist is unset and must not be consumed as a measurement. Includes prediction-only committed frames following rejected registration; use `lio_quality` logs to distinguish accepted matches.
- `/odom/status`: reliable/transient-local JSON String, approximately10Hz with immediate health transitions. `state` is UNINITIALIZED, FRESH, DEGRADED or FAULT. Fields: `stamp` (last published odometry stamp), `correction_age_s` (current predictor age, null if invalid), `input_stale`, `history_miss`, `last_publish_sensor_interval_s`.

Correction age is sensor prediction time minus last accepted LiDAR correction time. Age >0.5s marks DEGRADED without suppressing otherwise valid IMU output. ESKF covariance propagates with model noise; it is not a calibrated accuracy guarantee. No IMU consumed by the worker for >0.5s steady time marks stale input; this flag does not measure source freshness while a backlog is being consumed; no new odometry is fabricated. A confirmed input timestamp gap >50ms, nonfinite data, invalid propagation or application queue overflow latches FAULT. Restart the entire node after a fault; never silently reset only one chain.

Reliable KEEP_ALL IMU input uses a dedicated executor. Ensure compatible input publisher QoS. Middleware storage is not a real-time scheduling guarantee; the application queue is bounded20000 samples and faults on overflow.

Worker consumes <=4 IMUs before checking a new posterior. Each sample is appended AND propagated immediately, then checked for sensor-time publication eligibility. A delayed posterior is applied at its actual measurement time and replayed across retained IMU intervals. A posterior earlier than the2s retained window is rejected and counted, without stopping healthy prediction or clamping its timestamp. Pending posteriors must preserve sequence, anchor-time and accepted-correction-time ordering.

## Tracing and evaluation

`ADAPTIVE_LIO_OUTPUT_TRACE=/absolute/file.txt` enables buffered diagnostic trace. Columns: event, sensor_stamp, extra, steady_clock_seconds.

- C: IMU callback entry; I: processed IMU enqueue (extra queue length)
- U: posterior enqueue (extra last accepted LiDAR time)
- R/E: actual ready-posterior replay start/end (extra previous/new accepted time), including a posterior that initially waited for IMU coverage
- P/D: `/odom` publish call entry/return (extra correction age)

Output trace is opt-in and flushes on Stop; abrupt termination may lose the buffered tail. It adds locking/I/O overhead; use identical settings for comparisons where possible. A publication timestamp gap, publication wall gap and receiver wall gap answer different questions. A valid sensor-time sequence delivered in a burst is not a hard real-time guarantee.

`--trajectory_file /absolute/posterior.tum` exports fullprecision committed inner states. Evaluate this trajectory separately from `/odom`. APE requires ground truth, stated association tolerance, SE3 alignment without scale, and coverage. Do not infer accuracy improvement from uninterrupted publication.

The compatibility member `counters.timeout` counts stale publishability checks, NOT rejected messages. Status checks may increment it too; derive degraded duration from age and timestamps, not this counter.

See `benchmark/rebuild` at repository root for reproducible configs, baseline observer patch, tests and measured results. No shadow replay filter is implemented; profiling and deployment deadlines determine whether that additional architecture is needed.
