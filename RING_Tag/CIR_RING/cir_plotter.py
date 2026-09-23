#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Chương trình hiển thị và vẽ đồ thị CIR (Channel Impulse Response) của 4 Anchor UWB,
giải mã dữ liệu HEX sang giá trị Magnitude (uint16 LE) và tính toán tốc độ lấy mẫu (Sample Rate / Frequency) của Tag.

Tác giả: UWB-IPS Project
"""

import sys
import os
import time
import re
import struct
import argparse
import threading
from collections import deque
from queue import Queue, Empty

import numpy as np
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg
import serial


# ==============================================================================
# CẤU HÌNH MẶC ĐỊNH
# ==============================================================================
DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUDRATE = 921600
NUM_ANCHORS = 4
MAX_CIR_SAMPLES = 47

# Bảng màu cho 4 Anchor (A1, A2, A3, A4)
ANCHOR_COLORS = [
    {"curve": (0, 220, 255), "fill": (0, 220, 255, 45), "fp": (255, 80, 80)},    # Xanh dương sáng (Cyan)
    {"curve": (80, 255, 120), "fill": (80, 255, 120, 45), "fp": (255, 200, 50)},   # Xanh lá neon
    {"curve": (255, 180, 50), "fill": (255, 180, 50, 45), "fp": (255, 80, 200)},   # Cam vàng
    {"curve": (210, 110, 255), "fill": (210, 110, 255, 45), "fp": (80, 240, 255)}, # Tím mộng mơ
]


# ==============================================================================
# HÀM GIẢI MÃ HEX SANG MAGNITUDE UINT16 (LITTLE ENDIAN)
# ==============================================================================
def decode_cir_hex(hex_str: str) -> np.ndarray:
    """
    Giải mã chuỗi HEX thành mảng giá trị uint16 (Little Endian).
    Ví dụ: '80002300' -> byte [0x80, 0x00, 0x23, 0x00] -> [128, 35]
    """
    hex_str = hex_str.strip()
    if not hex_str:
        return np.array([], dtype=np.uint16)
    try:
        raw_bytes = bytes.fromhex(hex_str)
        # Mỗi mẫu 2 bytes (uint16 Little Endian)
        num_samples = len(raw_bytes) // 2
        magnitudes = np.frombuffer(raw_bytes[:num_samples * 2], dtype='<u2')
        return magnitudes
    except ValueError as e:
        print(f"[CẢNH BÁO] Lỗi giải mã hex: {e}")
        return np.array([], dtype=np.uint16)


# ==============================================================================
# THREAD ĐỌC SERIAL
# ==============================================================================
class SerialReaderThread(QtCore.QThread):
    packet_received = QtCore.pyqtSignal(dict)
    connection_status = QtCore.pyqtSignal(bool, str)

    def __init__(self, port, baudrate, demo_mode=False):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.demo_mode = demo_mode
        self.running = True
        self.ser = None

        # Regex phát hiện dòng tiêu đề CIR và dòng MAG
        # [CIR] tag=0x0005 anch=0x1003 fp=751.56 ns=47 N=756 sN=32 A1=6836 A2=7162 A3=5700 G=1976 PC=120
        self.cir_hdr_re = re.compile(
            r'\[CIR\]\s+tag=(0x[0-9a-fA-F]+)\s+anch=(0x[0-9a-fA-F]+)\s+fp=([0-9.]+)\s+ns=(\d+)'
            r'(?:\s+N=(\d+))?(?:\s+sN=(\d+))?(?:\s+A1=(\d+))?(?:\s+A2=(\d+))?(?:\s+A3=(\d+))?'
            r'(?:\s+G=(\d+))?(?:\s+PC=(\d+))?'
        )
        self.mag_re = re.compile(r'MAG:\s*([0-9a-fA-F]+)')

    def run(self):
        if self.demo_mode:
            self.connection_status.emit(True, "Chế độ mô phỏng (Demo Mode)")
            self._run_demo()
            return

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.connection_status.emit(True, f"Đã kết nối {self.port} @ {self.baudrate}")
        except Exception as e:
            self.connection_status.emit(False, f"Lỗi mở {self.port}: {e}")
            return

        pending_hdr = None

        while self.running:
            try:
                line_bytes = self.ser.readline()
                if not line_bytes:
                    continue

                line = line_bytes.decode('utf-8', errors='replace').strip()
                if not line:
                    continue

                # Kiểm tra xem có phải dòng [CIR]
                hdr_match = self.cir_hdr_re.search(line)
                if hdr_match:
                    pending_hdr = {
                        "tag_id": hdr_match.group(1).upper(),
                        "anchor_id": hdr_match.group(2).upper(),
                        "first_path": float(hdr_match.group(3)),
                        "num_samples": int(hdr_match.group(4)),
                        "max_noise": int(hdr_match.group(5) or 0),
                        "std_noise": int(hdr_match.group(6) or 0),
                        "a1": int(hdr_match.group(7) or 0),
                        "a2": int(hdr_match.group(8) or 0),
                        "a3": int(hdr_match.group(9) or 0),
                        "growth": int(hdr_match.group(10) or 0),
                        "preamble_cnt": int(hdr_match.group(11) or 0),
                        "recv_time": time.time(),
                    }
                    continue

                # Kiểm tra xem có phải dòng MAG:
                mag_match = self.mag_re.search(line)
                if mag_match and pending_hdr is not None:
                    hex_str = mag_match.group(1)
                    magnitudes = decode_cir_hex(hex_str)

                    packet = dict(pending_hdr)
                    packet["raw_hex"] = hex_str
                    packet["magnitudes"] = magnitudes
                    pending_hdr = None

                    self.packet_received.emit(packet)

            except Exception as e:
                print(f"[LỖI SERIAL] {e}")
                time.sleep(0.01)

        if self.ser and self.ser.is_open:
            self.ser.close()

    def _run_demo(self):
        """Sinh dữ liệu giả lập nếu không có phần cứng để kiểm tra giao diện"""
        anchor_ids = ["0X1001", "0X1002", "0X1003", "0X1004"]
        idx = 0
        while self.running:
            anch = anchor_ids[idx % 4]
            idx += 1
            ns = 47
            fp = 745.0 + np.random.uniform(-3, 3)

            # Tạo dạng sóng CIR DW1000 mô phỏng: nhiễu nền -> sườn dốc first path -> peak -> suy hao
            x = np.arange(ns)
            fp_peak = 23 + np.random.uniform(-1, 1)
            signal = np.exp(-((x - fp_peak) / 3.0)**2) * np.random.uniform(5000, 8000)
            noise = np.random.uniform(20, 100, size=ns)
            mags = np.clip(signal + noise, 0, 65535).astype(np.uint16)

            # Chuyển thành chuỗi HEX
            hex_str = mags.tobytes().hex().upper()

            packet = {
                "tag_id": "0x0005",
                "anchor_id": anch,
                "first_path": fp,
                "num_samples": ns,
                "max_noise": int(np.random.uniform(500, 1500)),
                "std_noise": int(np.random.uniform(30, 80)),
                "a1": int(np.random.uniform(3000, 7500)),
                "a2": int(np.random.uniform(5000, 7500)),
                "a3": int(np.random.uniform(4000, 6000)),
                "growth": 1980,
                "preamble_cnt": 120,
                "recv_time": time.time(),
                "raw_hex": hex_str,
                "magnitudes": mags
            }
            self.packet_received.emit(packet)
            time.sleep(0.025) # ~40 Hz tổng = 10 Hz mỗi anchor

    def stop(self):
        self.running = False
        self.wait(1000)


# ==============================================================================
# GIAO DIỆN CHÍNH (PYQT5 + PYQTGRAPH)
# ==============================================================================
class CIRMainWindow(QtWidgets.QMainWindow):
    def __init__(self, port, baudrate, demo_mode=False):
        super().__init__()
        self.setWindowTitle("UWB CIR Analyzer & Tag Sampling Rate Monitor - 4 Anchors")
        self.resize(1360, 880)

        # Cấu hình màu nền tối chuẩn khoa học
        self.setStyleSheet("""
            QMainWindow {
                background-color: #0F111A;
            }
            QGroupBox {
                border: 1px solid #282C3E;
                border-radius: 8px;
                margin-top: 10px;
                font-weight: bold;
                font-size: 13px;
                color: #82AAFF;
                padding-top: 15px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 14px;
                padding: 0 5px 0 5px;
            }
            QLabel {
                color: #A6ACCD;
                font-size: 12px;
            }
            QPlainTextEdit {
                background-color: #090B10;
                color: #C3E88D;
                border: 1px solid #1E2233;
                border-radius: 6px;
                font-family: 'Consolas', 'Courier New', monospace;
                font-size: 11px;
            }
        """)

        # Trạng thái dữ liệu 4 anchor
        # known_anchors khởi đầu rỗng — tự động học từ dữ liệu thực tế
        self.known_anchors = []
        self.anchor_to_idx = {}

        # Tính toán tốc độ lấy mẫu
        self.tag_timestamps = deque()  # lưu timestamp của tất cả các gói CIR của Tag
        self.anchor_timestamps = {}  # tạo động khi gặp anchor mới
        self.last_tag_time = None
        self.last_tag_delta_ms = 0.0
        self.total_packets_count = 0
        self.current_tag_id = "Chưa rõ"

        # Dữ liệu giải mã gần nhất
        self.latest_data = {}

        self.init_ui()

        # Khởi động luồng đọc Serial
        self.serial_thread = SerialReaderThread(port, baudrate, demo_mode=demo_mode)
        self.serial_thread.packet_received.connect(self.on_packet_received)
        self.serial_thread.connection_status.connect(self.on_connection_status)
        self.serial_thread.start()

        # Timer cập nhật đồ thị (60 FPS mượt mà)
        self.plot_timer = QtCore.QTimer()
        self.plot_timer.timeout.connect(self.update_plots_and_metrics)
        self.plot_timer.start(16)

    def init_ui(self):
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QtWidgets.QVBoxLayout(central_widget)
        main_layout.setContentsMargins(12, 12, 12, 12)
        main_layout.setSpacing(10)

        # ----------------------------------------------------------------------
        # 1. THANH TRẠNG THÁI & CHỈ SỐ LẤY MẪU (METRICS DASHBOARD)
        # ----------------------------------------------------------------------
        dashboard_box = QtWidgets.QGroupBox("THÔNG SỐ HOẠT ĐỘNG & TỐC ĐỘ LẤY MẪU CỦA TAG")
        dash_layout = QtWidgets.QHBoxLayout(dashboard_box)
        dash_layout.setSpacing(20)

        # Thẻ 1: Trạng thái Serial & Tag ID
        card1 = QtWidgets.QFrame()
        card1.setStyleSheet("background-color: #171A26; border-radius: 6px; padding: 6px;")
        c1_layout = QtWidgets.QVBoxLayout(card1)
        self.lbl_status = QtWidgets.QLabel("Đang kết nối...")
        self.lbl_status.setStyleSheet("color: #FFCB6B; font-weight: bold; font-size: 13px;")
        self.lbl_tag_id = QtWidgets.QLabel("Tag ID: --")
        self.lbl_tag_id.setStyleSheet("color: #89DDFF; font-size: 14px; font-weight: bold;")
        c1_layout.addWidget(self.lbl_status)
        c1_layout.addWidget(self.lbl_tag_id)
        dash_layout.addWidget(card1, 2)

        # Thẻ 2: Tốc độ lấy mẫu tổng của Tag (Hz)
        card2 = QtWidgets.QFrame()
        card2.setStyleSheet("background-color: #171A26; border-radius: 6px; padding: 6px;")
        c2_layout = QtWidgets.QVBoxLayout(card2)
        lbl_rate_title = QtWidgets.QLabel("TỐC ĐỘ LẤY MẪU TAG (TỔNG)")
        lbl_rate_title.setStyleSheet("color: #717CB4; font-size: 11px; font-weight: bold;")
        self.lbl_tag_hz = QtWidgets.QLabel("0.0 Hz")
        self.lbl_tag_hz.setStyleSheet("color: #C3E88D; font-size: 22px; font-weight: bold;")
        self.lbl_delta_t = QtWidgets.QLabel("Khoảng cách mẫu Δt: -- ms")
        self.lbl_delta_t.setStyleSheet("color: #82AAFF; font-size: 11px;")
        c2_layout.addWidget(lbl_rate_title)
        c2_layout.addWidget(self.lbl_tag_hz)
        c2_layout.addWidget(self.lbl_delta_t)
        dash_layout.addWidget(card2, 2)

        # Thẻ 3: Tốc độ chu kỳ quét đủ 4 Anchor (Full Cycle Rate)
        card3 = QtWidgets.QFrame()
        card3.setStyleSheet("background-color: #171A26; border-radius: 6px; padding: 6px;")
        c3_layout = QtWidgets.QVBoxLayout(card3)
        lbl_cycle_title = QtWidgets.QLabel("CHU KỲ QUÉT ĐỦ 4 ANCHOR")
        lbl_cycle_title.setStyleSheet("color: #717CB4; font-size: 11px; font-weight: bold;")
        self.lbl_cycle_hz = QtWidgets.QLabel("0.0 Hz (vòng/s)")
        self.lbl_cycle_hz.setStyleSheet("color: #FF5370; font-size: 22px; font-weight: bold;")
        self.lbl_total_pkts = QtWidgets.QLabel("Tổng gói CIR: 0")
        self.lbl_total_pkts.setStyleSheet("color: #A6ACCD; font-size: 11px;")
        c3_layout.addWidget(lbl_cycle_title)
        c3_layout.addWidget(self.lbl_cycle_hz)
        c3_layout.addWidget(self.lbl_total_pkts)
        dash_layout.addWidget(card3, 2)

        main_layout.addWidget(dashboard_box, 0)

        # ----------------------------------------------------------------------
        # 2. KHU VỰC ĐỒ THỊ CIR CỦA 4 ANCHOR (LƯỚI 2x2)
        # ----------------------------------------------------------------------
        plots_group = QtWidgets.QGroupBox("ĐỒ THỊ ĐÁP ỨNG KÊNH CIR (CHANNEL IMPULSE RESPONSE) CỦA 4 ANCHOR")
        plots_layout = QtWidgets.QGridLayout(plots_group)
        plots_layout.setSpacing(12)

        self.plot_widgets = []
        self.curves = []
        self.fp_lines = []
        self.peak_points = []
        self.anchor_info_labels = []

        positions = [(0, 0), (0, 1), (1, 0), (1, 1)]

        for i in range(NUM_ANCHORS):
            color = ANCHOR_COLORS[i]
            pw = pg.PlotWidget()
            pw.setBackground("#0D0F17")
            pw.showGrid(x=True, y=True, alpha=0.25)
            pw.setLabel('bottom', 'Sample Index (Mẫu)', color='#717CB4', size='11pt')
            pw.setLabel('left', 'Biên độ Magnitude', color='#717CB4', size='11pt')
            pw.setYRange(0, 8500)
            pw.setXRange(0, MAX_CIR_SAMPLES)

            # Đường cong CIR
            pen = pg.mkPen(color=color["curve"], width=2.2)
            brush = pg.mkBrush(color=color["fill"])
            curve = pw.plot([], [], pen=pen, fillLevel=0, brush=brush)

            # Vạch chỉ First Path (FP)
            fp_pen = pg.mkPen(color=color["fp"], width=1.8, style=QtCore.Qt.DashLine)
            fp_line = pg.InfiniteLine(angle=90, movable=False, pen=fp_pen)
            fp_line.setVisible(False)
            pw.addItem(fp_line)

            # Điểm đánh dấu Peak
            peak_scatter = pw.plot([], [], pen=None, symbol='star', symbolSize=14,
                                   symbolBrush=pg.mkBrush(255, 255, 100),
                                   symbolPen=pg.mkPen('w', width=1))

            # Label thông tin chi tiết của từng Anchor
            info_label = QtWidgets.QLabel(f"Anchor {i+1}: Đang chờ dữ liệu...")
            info_label.setStyleSheet("color: #EEFFFF; font-size: 12px; font-weight: bold; background: #141724; padding: 4px; border-radius: 4px;")

            container = QtWidgets.QVBoxLayout()
            container.addWidget(info_label)
            container.addWidget(pw)

            sub_widget = QtWidgets.QWidget()
            sub_widget.setLayout(container)

            r, c = positions[i]
            plots_layout.addWidget(sub_widget, r, c)

            self.plot_widgets.append(pw)
            self.curves.append(curve)
            self.fp_lines.append(fp_line)
            self.peak_points.append(peak_scatter)
            self.anchor_info_labels.append(info_label)

        main_layout.addWidget(plots_group, 5)

        # ----------------------------------------------------------------------
        # 3. KHU VỰC HIỂN THỊ HEX VÀ DỮ LIỆU ĐÃ GIẢI MÃ
        # ----------------------------------------------------------------------
        hex_group = QtWidgets.QGroupBox("DỮ LIỆU HEX VÀ KẾT QUẢ GIẢI MÃ MỚI NHẤT")
        hex_layout = QtWidgets.QHBoxLayout(hex_group)
        hex_layout.setSpacing(10)

        # Khung Hex thô
        v_hex = QtWidgets.QVBoxLayout()
        lbl_raw_title = QtWidgets.QLabel("Chuỗi HEX thô (2 byte LE / 1 mẫu Magnitude):")
        lbl_raw_title.setStyleSheet("color: #FFCB6B; font-weight: bold;")
        self.txt_raw_hex = QtWidgets.QPlainTextEdit()
        self.txt_raw_hex.setReadOnly(True)
        self.txt_raw_hex.setMaximumHeight(90)
        v_hex.addWidget(lbl_raw_title)
        v_hex.addWidget(self.txt_raw_hex)
        hex_layout.addLayout(v_hex, 1)

        # Khung giá trị đã giải mã
        v_dec = QtWidgets.QVBoxLayout()
        lbl_dec_title = QtWidgets.QLabel("Mảng giá trị Magnitude đã giải mã (uint16 decimal):")
        lbl_dec_title.setStyleSheet("color: #C3E88D; font-weight: bold;")
        self.txt_decoded = QtWidgets.QPlainTextEdit()
        self.txt_decoded.setReadOnly(True)
        self.txt_decoded.setMaximumHeight(90)
        v_dec.addWidget(lbl_dec_title)
        v_dec.addWidget(self.txt_decoded)
        hex_layout.addLayout(v_dec, 1)

        main_layout.addWidget(hex_group, 1)

    # --------------------------------------------------------------------------
    # XỬ LÝ SỰ KIỆN TỪ SERIAL READER THREAD
    # --------------------------------------------------------------------------
    @QtCore.pyqtSlot(bool, str)
    def on_connection_status(self, success, message):
        color = "#C3E88D" if success else "#FF5370"
        self.lbl_status.setText(f"Cổng: {message}")
        self.lbl_status.setStyleSheet(f"color: {color}; font-weight: bold; font-size: 13px;")

    @QtCore.pyqtSlot(dict)
    def on_packet_received(self, pkt):
        now = pkt["recv_time"]
        self.total_packets_count += 1
        self.current_tag_id = pkt["tag_id"]

        # Tính khoảng thời gian Δt giữa 2 gói liên tiếp
        if self.last_tag_time is not None:
            dt_ms = (now - self.last_tag_time) * 1000.0
            self.last_tag_delta_ms = dt_ms
        self.last_tag_time = now

        # Thêm vào sliding window để tính Hz (cửa sổ 2 giây)
        self.tag_timestamps.append(now)
        while self.tag_timestamps and now - self.tag_timestamps[0] > 2.0:
            self.tag_timestamps.popleft()

        # Xác định Anchor — normalize ID về chữ thường để tránh lệch key
        anch_id = pkt["anchor_id"].upper()
        pkt["anchor_id"] = anch_id  # đảm bảo packet cũng dùng key chuẩn

        if anch_id not in self.anchor_to_idx:
            if len(self.known_anchors) < NUM_ANCHORS:
                # Slot mới còn trống: thêm anchor vào
                idx = len(self.known_anchors)
                self.known_anchors.append(anch_id)
                self.anchor_to_idx[anch_id] = idx
                self.anchor_timestamps[anch_id] = deque()
                print(f"[INFO] Anchor mới: {anch_id} → slot {idx}")
            else:
                # Đã đủ 4 anchor, bỏ qua anchor lạ (không ghi đè slot 0 nữa)
                print(f"[WARN] Anchor {anch_id} không có trong {self.known_anchors}, bỏ qua")
                return
        else:
            idx = self.anchor_to_idx[anch_id]

        # Đảm bảo anchor_timestamps tồn tại (phòng trường hợp thiếu)
        if anch_id not in self.anchor_timestamps:
            self.anchor_timestamps[anch_id] = deque()

        # Cập nhật timestamp của từng anchor
        atimes = self.anchor_timestamps[anch_id]
        atimes.append(now)
        while atimes and now - atimes[0] > 2.0:
            atimes.popleft()

        # Lưu dữ liệu mới nhất
        self.latest_data[idx] = pkt

    # --------------------------------------------------------------------------
    # CẬP NHẬT ĐỒ THỊ VÀ METRICS ĐỊNH KỲ
    # --------------------------------------------------------------------------
    def update_plots_and_metrics(self):
        now = time.time()

        # 1. Tính tốc độ lấy mẫu tổng của Tag (Hz)
        tag_hz = 0.0
        if len(self.tag_timestamps) > 1:
            duration = now - self.tag_timestamps[0]
            if duration > 0.05:
                tag_hz = (len(self.tag_timestamps) - 1) / duration

        # 2. Tính tốc độ quét đủ 4 Anchor (Full Cycle Rate)
        # Giả sử 1 vòng gồm 4 anchor -> Cycle Rate = Total Hz / 4
        cycle_hz = tag_hz / 4.0 if NUM_ANCHORS > 0 else 0.0

        # Cập nhật hiển thị lên Dashboard
        self.lbl_tag_id.setText(f"Tag ID: {self.current_tag_id}")
        self.lbl_tag_hz.setText(f"{tag_hz:.1f} Hz (mẫu/s)")
        self.lbl_delta_t.setText(f"Khoảng cách mẫu Δt: {self.last_tag_delta_ms:.1f} ms")
        self.lbl_cycle_hz.setText(f"{cycle_hz:.2f} Hz (vòng/s)")
        self.lbl_total_pkts.setText(f"Tổng gói CIR: {self.total_packets_count}")

        # Cập nhật từng đồ thị Anchor
        for idx in range(NUM_ANCHORS):
            if idx in self.latest_data:
                pkt = self.latest_data[idx]
                mags = pkt["magnitudes"]
                if len(mags) == 0:
                    continue

                x = np.arange(len(mags))
                y = mags

                # Vẽ đường CIR
                self.curves[idx].setData(x, y)

                # Tìm Peak
                peak_idx = int(np.argmax(y))
                peak_val = int(y[peak_idx])
                self.peak_points[idx].setData([peak_idx], [peak_val])

                # Đường First Path (FP)
                # Trong C firmware: start = (fp_int > 23) ? (fp_int - 23) : 0;
                # Vị trí First Path tương đối trong 47 mẫu là (fp - start)
                fp_raw = pkt["first_path"]
                fp_int = int(fp_raw)
                start_sample = (fp_int - 23) if fp_int > 23 else 0
                rel_fp = fp_raw - start_sample

                if 0 <= rel_fp <= len(mags):
                    self.fp_lines[idx].setValue(rel_fp)
                    self.fp_lines[idx].setVisible(True)

                # Tính tần số riêng của Anchor này
                anch_id = pkt["anchor_id"]
                anch_hz = 0.0
                if anch_id in self.anchor_timestamps:
                    atimes = self.anchor_timestamps[anch_id]
                    if len(atimes) > 1:
                        dur = now - atimes[0]
                        if dur > 0.05:
                            anch_hz = (len(atimes) - 1) / dur

                # Cập nhật label chi tiết
                self.anchor_info_labels[idx].setText(
                    f"Anchor {idx+1} [{pkt['anchor_id']}] — Tốc độ: {anch_hz:.1f} Hz | "
                    f"Đỉnh Peak: {peak_val} (mẫu #{peak_idx}) | "
                    f"FirstPath: {fp_raw:.2f} | "
                    f"Nhiễu: N={pkt['max_noise']}, sN={pkt['std_noise']} | "
                    f"Amp: [{pkt['a1']}, {pkt['a2']}, {pkt['a3']}]"
                )

        # Cập nhật bảng hiển thị HEX gần nhất
        if self.latest_data:
            newest_idx = max(self.latest_data.keys(), key=lambda k: self.latest_data[k]["recv_time"])
            newest_pkt = self.latest_data[newest_idx]

            hex_text = f"[{newest_pkt['anchor_id']}] (Độ dài: {len(newest_pkt['raw_hex'])} hex chars = {len(newest_pkt['raw_hex'])//2} bytes):\n{newest_pkt['raw_hex']}"
            self.txt_raw_hex.setPlainText(hex_text)

            mags_str = ", ".join(str(v) for v in newest_pkt['magnitudes'])
            dec_text = f"[{newest_pkt['anchor_id']}] ({len(newest_pkt['magnitudes'])} samples uint16 LE):\n[{mags_str}]"
            self.txt_decoded.setPlainText(dec_text)

    def closeEvent(self, event):
        self.serial_thread.stop()
        event.accept()


# ==============================================================================
# HÀM MAIN
# ==============================================================================
def main():
    parser = argparse.ArgumentParser(description="UWB CIR Real-Time Plotter & Sampling Rate Monitor")
    parser.add_argument("--port", type=str, default=DEFAULT_PORT, help=f"Cổng Serial (mặc định: {DEFAULT_PORT})")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUDRATE, help=f"Baudrate (mặc định: {DEFAULT_BAUDRATE})")
    parser.add_argument("--demo", action="store_true", help="Chạy chế độ mô phỏng dữ liệu (không cần phần cứng)")
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")

    # Kiểm tra cổng serial nếu không bật demo
    if not args.demo and not os.path.exists(args.port):
        print(f"[THÔNG BÁO] Không tìm thấy cổng {args.port}. Chuyển sang chế độ Demo mô phỏng...")
        args.demo = True

    window = CIRMainWindow(port=args.port, baudrate=args.baud, demo_mode=args.demo)
    window.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()