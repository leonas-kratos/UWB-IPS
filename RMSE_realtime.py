#!/usr/bin/env python3
"""
Serial RMSE Monitor — đọc /dev/ttyACM0, parse giá trị range, hiển thị RMSE realtime.
Không lưu file.

Usage:
    python3 serial_rmse.py <ground_truth_mm> [max_lines]

Examples:
    python3 serial_rmse.py 6500
    python3 serial_rmse.py 6500 1000
"""

import sys
import re
import math
import signal
import serial
import time
from collections import deque

VALUE_RE = re.compile(r':\s*(\d+)\s*$')

def compute_rmse(values, gt):
    if not values:
        return 0.0
    return math.sqrt(sum((v - gt) ** 2 for v in values) / len(values))

def main():
    if len(sys.argv) < 2 or len(sys.argv) > 3:
        print("Usage: python3 serial_rmse.py <ground_truth_mm> [max_lines]")
        sys.exit(1)

    ground_truth = float(sys.argv[1])
    max_lines    = int(sys.argv[2]) if len(sys.argv) == 3 else None
    port         = "/dev/ttyACM0"
    baudrate     = 921600

    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"[OK] {port} @ {baudrate} baud | GT={ground_truth:.0f}mm"
              + (f" | max={max_lines}" if max_lines else "") + "\n")
    except serial.SerialException as e:
        print(f"[ERR] Cannot open {port}: {e}")
        sys.exit(1)

    line_count   = 0
    values       = []
    start_time   = time.time()
    recent_times = deque()

    def on_exit(sig, frame):
        elapsed = time.time() - start_time
        rmse    = compute_rmse(values, ground_truth)
        mean    = sum(values) / len(values) if values else 0
        print(f"\n{'='*50}")
        print(f"  Lines   : {line_count}  |  {elapsed:.1f}s")
        print(f"  Mean    : {mean:.1f} mm  (bias {mean-ground_truth:+.1f} mm)")
        print(f"  RMSE    : {rmse:.2f} mm")
        print(f"{'='*50}")
        ser.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, on_exit)

    while True:
        try:
            raw = ser.readline()
            if not raw:
                continue

            line  = raw.decode("utf-8", errors="replace").rstrip()
            now   = time.time()
            m     = VALUE_RE.search(line)
            if not m:
                continue

            value = int(m.group(1))
            error = value - ground_truth
            values.append(value)
            line_count += 1

            # Tốc độ tức thời sliding window 2s
            recent_times.append(now)
            while recent_times and now - recent_times[0] > 2.0:
                recent_times.popleft()
            speed = len(recent_times) / min(now - start_time, 2.0) if line_count > 1 else 0.0

            rmse     = compute_rmse(values, ground_truth)
            progress = f"{line_count}/{max_lines}" if max_lines else str(line_count)
            print(f"({progress} | {speed:.1f}ln/s) range={value}mm  err={error:+.0f}mm  RMSE={rmse:.2f}mm")

            if max_lines and line_count >= max_lines:
                on_exit(None, None)

        except Exception as e:
            print(f"[WARN] {e}")

if __name__ == "__main__":
    main()
