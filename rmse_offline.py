#!/usr/bin/env python3
"""
RMSE Calculator — tính RMSE từng file trong thư mục Calib/
Tên file format: <ID>_<T|A>_<groundtruth_mm>.txt

Usage:
    python3 calc_rmse.py <calib_dir>

Example:
    python3 calc_rmse.py Calib/
"""

import sys
import re
import math
import os

VALUE_RE = re.compile(r':\s*(\d+)\s*$')
FILE_RE  = re.compile(r'^(\w+)_([TA])_(\d+)\.(?:txt|csv)$')

def compute_rmse(values, gt):
    if not values:
        return 0.0
    return math.sqrt(sum((v - gt) ** 2 for v in values) / len(values))

def process_file(filepath, gt):
    values = []
    with open(filepath, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            m = VALUE_RE.search(line.rstrip())
            if m:
                values.append(int(m.group(1)))
    return values

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 calc_rmse.py <calib_dir>")
        sys.exit(1)

    calib_dir = sys.argv[1].rstrip("/")
    if not os.path.isdir(calib_dir):
        print(f"[ERR] Directory not found: {calib_dir}")
        sys.exit(1)

    files = sorted(f for f in os.listdir(calib_dir) if FILE_RE.match(f))
    if not files:
        print(f"[ERR] No matching files found in {calib_dir}")
        sys.exit(1)

    # Header
    print(f"\n{'─'*65}")
    print(f"  {'File':<28} {'N':>5}  {'GT(mm)':>7}  {'Mean':>7}  {'Bias':>7}  {'RMSE':>7}")
    print(f"{'─'*65}")

    all_values = []
    all_gt     = None

    for fname in files:
        m  = FILE_RE.match(fname)
        fid, ftype, gt_str = m.group(1), m.group(2), m.group(3)
        gt = float(gt_str)

        values = process_file(os.path.join(calib_dir, fname), gt)
        if not values:
            print(f"  {fname:<28} {'N/A':>5}  {gt:>7.0f}  {'—':>7}  {'—':>7}  {'—':>7}")
            continue

        rmse = compute_rmse(values, gt)
        mean = sum(values) / len(values)
        bias = mean - gt

        label = f"{fid} ({'Tag  ' if ftype=='T' else 'Anchor'})"
        print(f"  {label:<28} {len(values):>5}  {gt:>7.0f}  {mean:>7.1f}  {bias:>+7.1f}  {rmse:>7.2f}")

        all_values.extend(values)
        if all_gt is None:
            all_gt = gt

    # Summary tổng
    if all_values and all_gt is not None:
        total_rmse = compute_rmse(all_values, all_gt)
        total_mean = sum(all_values) / len(all_values)
        total_bias = total_mean - all_gt
        print(f"{'─'*65}")
        print(f"  {'TOTAL':<28} {len(all_values):>5}  {all_gt:>7.0f}  {total_mean:>7.1f}  {total_bias:>+7.1f}  {total_rmse:>7.2f}")
    print(f"{'─'*65}\n")

if __name__ == "__main__":
    main()
