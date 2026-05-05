#!/usr/bin/env python3
"""
Serial Logger — đọc /dev/ttyACM0 @ 921600 baud và lưu ra file
Usage:
    python3 serial_logger.py <output.txt|output.csv> [max_lines]

    max_lines : số dòng tối đa muốn lưu (bỏ qua = không giới hạn)

Examples:
    python3 serial_logger.py data.csv          # lưu đến khi Ctrl+C
    python3 serial_logger.py data.csv 1000     # lưu đủ 1000 dòng rồi tự dừng
    Ctrl+C để dừng sớm bất cứ lúc nào
"""

import sys
import signal
import serial
import time
from datetime import datetime
from collections import deque

def main():
    if len(sys.argv) < 2 or len(sys.argv) > 3:
        print("Usage: python3 serial_logger.py <output.txt|output.csv> [max_lines]")
        sys.exit(1)

    output_file = sys.argv[1]
    max_lines   = int(sys.argv[2]) if len(sys.argv) == 3 else None
    port        = "/dev/ttyACM0"
    baudrate    = 921600

    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"[OK] Opened {port} @ {baudrate} baud")
        print(f"[OK] Logging to : {output_file}")
        if max_lines:
            print(f"[OK] Max lines  : {max_lines}")
        else:
            print(f"[OK] Max lines  : unlimited (Ctrl+C to stop)")
        print()
    except serial.SerialException as e:
        print(f"[ERR] Cannot open {port}: {e}")
        sys.exit(1)

    is_csv     = output_file.lower().endswith(".csv")
    line_count = 0
    start_time = time.time()

    # Sliding window 2 giây để tính tốc độ tức thời
    recent_times = deque()

    def print_status(ts, line, speed):
        progress = f"{line_count}/{max_lines}" if max_lines else str(line_count)
        print(f"[{ts}] ({progress} lines | {speed:.1f} ln/s) {line}")

    def on_exit(sig, frame):
        elapsed = time.time() - start_time
        avg     = line_count / elapsed if elapsed > 0 else 0
        print(f"\n[OK] Stopped after {elapsed:.1f}s — {line_count} lines saved ({avg:.1f} ln/s avg)")
        ser.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, on_exit)

    with open(output_file, "w", encoding="utf-8") as f:
        if is_csv:
            f.write("timestamp,data\n")

        while True:
            try:
                raw = ser.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="replace").rstrip()
                now  = time.time()
                ts   = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

                if is_csv:
                    safe = line.replace('"', '""')
                    f.write(f'{ts},"{safe}"\n')
                else:
                    f.write(f"[{ts}] {line}\n")

                f.flush()
                line_count += 1

                # Tính tốc độ tức thời (sliding window 2s)
                recent_times.append(now)
                while recent_times and now - recent_times[0] > 2.0:
                    recent_times.popleft()
                speed = len(recent_times) / min(now - start_time, 2.0) if line_count > 1 else 0.0

                print_status(ts, line, speed)

                # Dừng nếu đạt max_lines
                if max_lines and line_count >= max_lines:
                    elapsed = time.time() - start_time
                    avg     = line_count / elapsed if elapsed > 0 else 0
                    print(f"\n[OK] Done — {line_count} lines in {elapsed:.1f}s ({avg:.1f} ln/s avg)")
                    ser.close()
                    sys.exit(0)

            except Exception as e:
                print(f"[WARN] Read error: {e}")

if __name__ == "__main__":
    main()
