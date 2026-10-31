#!/usr/bin/env bash
# Restore public fixtures. Expected layout /workspace/Adaptive-LIO and /workspace/bench.
set -euo pipefail
mkdir -p /workspace/datasets/flatwall /workspace/datasets/glim_cross_sensor
fetch() {
  local url="$1" destination="$2"
  test -s "$destination" || curl -fL --retry 4 "$url" -o "$destination"
}
fetch https://zenodo.org/records/7641866/files/flatwall_05.bag /workspace/datasets/flatwall/flatwall_05.bag
fetch https://zenodo.org/records/7641866/files/gt.tar.gz /workspace/datasets/flatwall/gt.tar.gz
fetch https://zenodo.org/records/6864654/files/livox.bag /workspace/datasets/glim_cross_sensor/livox.bag
fetch https://zenodo.org/records/6864654/files/gt.tar.gz /workspace/datasets/glim_cross_sensor/gt.tar.gz
md5sum -c <<'EOF'
15b7c46d0a018d0dbaf6bb8a1b30a828  /workspace/datasets/flatwall/flatwall_05.bag
fb17a6d4e0bc1765e276f0dcca8a5063  /workspace/datasets/glim_cross_sensor/livox.bag
EOF
tar xzf /workspace/datasets/flatwall/gt.tar.gz -C /workspace/datasets/flatwall
tar xzf /workspace/datasets/glim_cross_sensor/gt.tar.gz -C /workspace/datasets/glim_cross_sensor
python3 -m venv /workspace/bench/data_venv
/workspace/bench/data_venv/bin/pip install rosbags==0.11.9 numpy
script_dir="$(cd "$(dirname "$0")" && pwd)"
test -d /workspace/datasets/flatwall/flatwall_05_ros2 || /workspace/bench/data_venv/bin/python "$script_dir/data_convert.py" /workspace/datasets/flatwall/flatwall_05.bag /workspace/datasets/flatwall/flatwall_05_ros2
test -d /workspace/datasets/glim_cross_sensor/livox_ros2 || /workspace/bench/data_venv/bin/python "$script_dir/data_convert.py" /workspace/datasets/glim_cross_sensor/livox.bag /workspace/datasets/glim_cross_sensor/livox_ros2 --retimed /workspace/datasets/glim_cross_sensor/livox_retimed
test -d /workspace/datasets/flatwall/flatwall_05_fault_gap || /workspace/bench/data_venv/bin/python "$script_dir/data_fault_gap.py"
/workspace/bench/data_venv/bin/python "$script_dir/data_verify.py"
