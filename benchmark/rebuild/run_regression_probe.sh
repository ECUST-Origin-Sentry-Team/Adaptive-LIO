#!/usr/bin/env bash
# Run inside the ROS build container after Eigen is installed.
set -euo pipefail
fixed_root=${1:-/workspace/Adaptive-LIO}
baseline_root=${2:-/workspace/baseline}
output_dir=${3:-/workspace/bench/regression_probe}
mkdir -p "$output_dir"
for version in baseline fixed; do
    source_root=$fixed_root
    if [[ "$version" == baseline ]]; then source_root=$baseline_root; fi
    g++ -std=c++17 -O1 \
        -I"$source_root/src/Adaptive-LIO/src" \
        -I"$source_root/src/Adaptive-LIO/thirdparty" \
        -I/usr/include/eigen3 \
        "$fixed_root/benchmark/rebuild/regression_probe.cpp" \
        -o "$output_dir/$version"
    for limit in 0.5 1.0; do
        "$output_dir/$version" "$limit" > "$output_dir/${version}_${limit}.json"
    done
done
