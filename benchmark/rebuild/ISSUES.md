# Output continuity reconstruction — 2026-10-31

Base: ECUST-Origin-Sentry-Team/Adaptive-LIO `ljb`, 97a3b0edd27dbf03b9577f27c0e1ae2eff08b581.
This is a reconstruction, not the lost binary or patch. Previous numerical results are not evidence for this build.

## Issues and corrections

1. Correction age >0.5s suppressed healthy IMU prediction. Continue valid propagation and mark DEGRADED; retain strict invalid-input faults. Increasing the gate to 1s alone cannot solve stalled correction or burst processing.
2. Expired posterior permanently faulted the predictor. Reject only that correction, retain valid state, recover when a fresh posterior arrives. Never clamp the posterior measurement time to a different history entry.
3. Append-all before Advance could evict the running propagation anchor. Consume Append+Advance per sample; bound worker batches to four; check new posterior between batches.
4. One output after a whole batch omitted intermediate sensor times. Check sensor-time publication interval per sample (target50Hz). Never synthesize stamps or repeat old poses with new stamps.
5. Shared executor and shallow input queue could turn scheduling backlog into apparent sensor gaps. Isolate IMU executor; reliable KEEP_ALL (bounded application queue still faults on overflow). Actual publisher QoS must be compatible.
6. Distinguish correction stale, no newly delivered input, and confirmed timestamp gap. Silence is DEGRADED/input_stale; confirmed >50ms gap or nonfinite input is latched FAULT. Restart whole estimator after FAULT.
7. Add independent committed LiDAR pose topic and optional fullprecision TUM export; keep matching/inner estimator unchanged. Rejected-registration predicted commits are still commits, inspect quality logs.
8. Record callback ingress, IMU enqueue, posterior enqueue, replay and publish boundaries with steady time. Separate sensor coverage, node wall publication and receiver intervals.
9. Verify delayed replay state AND covariance against timely application; test pending correction ordering. Do not infer accuracy improvement from continuity alone.
10. Lost workspace artifacts: preserve source, scripts, source manifest and small measured result summaries in GitHub branch. Generated data/logs can be reconstructed; no claim local path alone is durable backup.

## Acceptance

- Independent core tests: posterior delays0.75/1.5/2.5s, beyond-window rejection/recovery, off-grid timestamp replay equivalence, >history-duration burst, covariance growth, reorder rejection, NaN and real-gap fault.
- Actual ROS adapter tests plus source-bag replay: strictly increasing real timestamps; no stale-age output suppression; explicit health transitions.
- Paired actual-bag runs: same configs/input; two repeats alternating order; all IMU/frame counts; common-support GT alignment if available; report sample coverage.
- Never describe receiver stalls as fixed merely because sensor timestamp gaps disappear. Exclude drain silence from in-data degradation duration.
- Assess measured replay duration before deciding whether a separate shadow filter is warranted. Current design does not promise hard real-time bounds.
