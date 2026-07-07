#!/usr/bin/env python3
"""Generate dummy SemanticKITTI .label files for a custom ERASOR2 sequence.

ERASOR2's SemanticKITTI dataloader expects labels/000000.label even for custom
sequences.  For self-recorded data without semantic ground truth, zero-filled
uint32 labels with the same point count as each velodyne/*.bin are sufficient
for map generation.
"""
from pathlib import Path
import argparse
import numpy as np


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("sequence_dir", help="Path to dataset/sequences/XX")
    args = parser.parse_args()

    seq_dir = Path(args.sequence_dir).expanduser().resolve()
    velodyne_dir = seq_dir / "velodyne"
    label_dir = seq_dir / "labels"
    label_dir.mkdir(parents=True, exist_ok=True)

    bin_files = sorted(velodyne_dir.glob("*.bin"))
    if not bin_files:
        raise SystemExit(f"No .bin files found in {velodyne_dir}")

    for bin_path in bin_files:
        size = bin_path.stat().st_size
        if size % 16 != 0:
            print(f"[WARN] {bin_path.name}: size {size} is not divisible by 16")
        n_points = size // 16
        labels = np.zeros(n_points, dtype=np.uint32)
        out_path = label_dir / f"{bin_path.stem}.label"
        labels.tofile(out_path)
        print(f"{bin_path.name} -> {out_path.name}, points={n_points}")

    print(f"Done. Generated {len(bin_files)} files in {label_dir}")


if __name__ == "__main__":
    main()
