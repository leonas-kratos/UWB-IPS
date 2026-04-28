"""
CIR Real-time Plotter - 4 Anchors (C-Accelerated, single file)
Format serial: 0x1001,<distance_mm>,<cir_0>,...,<cir_99>

Compile C:  make cir
Chạy demo:  python cir_realtime.py --demo
Chạy thật:  python cir_realtime.py --port /dev/ttyACM0 --baud 921600
"""

import sys
import os
import ctypes
import argparse
import time
import math
import random
import threading

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QHBoxLayout, QGridLayout, QLabel
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
import pyqtgraph as pg

# ==================== LOAD C LIBRARY ====================
def _load_lib():
    base = os.path.dirname(os.path.abspath(__file__))
    for name in ['cir_processor.so', 'cir_processor.dll', 'cir_processor.dylib']:
        path = os.path.join(base, name)
        if os.path.exists(path):
            return ctypes.CDLL(path)
    raise FileNotFoundError(
        "Không tìm thấy cir_processor.so — chạy: make cir"
    )

_lib = _load_lib()

CIR_SAMPLES  = 100
HISTORY_LEN  = 200
_IntCIR  = ctypes.c_int * CIR_SAMPLES
_IntHist = ctypes.c_int * HISTORY_LEN

# Khai báo hàm C
_lib.cir_init.restype         = None;  _lib.cir_init.argtypes         = []
_lib.cir_reset.restype        = None;  _lib.cir_reset.argtypes        = []
_lib.cir_update.restype       = ctypes.c_int
_lib.cir_update.argtypes      = [ctypes.c_int, ctypes.c_int,
                                  ctypes.POINTER(ctypes.c_int), ctypes.c_double]
_lib.cir_get_cir.restype      = ctypes.c_int
_lib.cir_get_cir.argtypes     = [ctypes.c_int, ctypes.POINTER(ctypes.c_int)]
_lib.cir_get_dist_hist.restype  = ctypes.c_int
_lib.cir_get_dist_hist.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_int)]
_lib.cir_get_stats.restype    = ctypes.c_int
_lib.cir_get_stats.argtypes   = [ctypes.c_int,
                                  ctypes.POINTER(ctypes.c_int),
                                  ctypes.POINTER(ctypes.c_int),
                                  ctypes.POINTER(ctypes.c_int),
                                  ctypes.POINTER(ctypes.c_int),
                                  ctypes.POINTER(ctypes.c_double),
                                  ctypes.POINTER(ctypes.c_int)]
_lib.cir_mark_dead.restype    = None;  _lib.cir_mark_dead.argtypes    = [ctypes.c_int]
_lib.cir_init()

def c_update(aid, dist, cir_list, ts):
    arr = (_IntCIR)(*cir_list[:CIR_SAMPLES])
    _lib.cir_update(aid, dist, arr, ctypes.c_double(ts))

def c_get_cir(aid):
    buf = _IntCIR(); _lib.cir_get_cir(aid, buf); return list(buf)

def c_get_hist(aid):
    buf = _IntHist(); _lib.cir_get_dist_hist(aid, buf); return list(buf)

def c_get_stats(aid):
    d=ctypes.c_int(); fp=ctypes.c_int(); pk=ctypes.c_int()
    fc=ctypes.c_int(); fs=ctypes.c_double(); al=ctypes.c_int()
    _lib.cir_get_stats(aid, ctypes.byref(d), ctypes.byref(fp),
                       ctypes.byref(pk), ctypes.byref(fc),
                       ctypes.byref(fs), ctypes.byref(al))
    return {'distance': d.value, 'fp_index': fp.value, 'cir_peak': pk.value,
            'frame_count': fc.value, 'fps': fs.value, 'alive': bool(al.value)}

# ==================== CẤU HÌNH ====================
ANCHOR_IDS    = [0x1001, 0x1002, 0x1003, 0x1004]
REFRESH_MS    = 40
ALIVE_TIMEOUT = 1.5
COL_DIST      = 1
COL_CIR       = 2

COLORS = {
    0x1001: (100, 200, 255),
    0x1002: (100, 255, 150),
    0x1003: (255, 180,  80),
    0x1004: (255, 100, 100),
}
NAMES = {a: f"0x{a:04X}" for a in ANCHOR_IDS}

_last_ts = {aid: 0.0 for aid in ANCHOR_IDS}
_lock    = threading.Lock()

# ==================== PARSE ====================
def parse_line(line: str):
    parts = line.strip().split(',')
    if len(parts) < COL_CIR + CIR_SAMPLES:
        return
    try:
        s   = parts[0].strip()
        aid = int(s, 16) if 'x' in s.lower() else int(s)
        if aid not in ANCHOR_IDS:
            return
        dist    = int(parts[COL_DIST])
        cir_raw = [max(0, int(v)) for v in parts[COL_CIR:COL_CIR + CIR_SAMPLES]]
        while len(cir_raw) < CIR_SAMPLES:
            cir_raw.append(0)
        ts = time.time()
        c_update(aid, dist, cir_raw, ts)
        with _lock:
            _last_ts[aid] = ts
    except (ValueError, IndexError):
        pass

# ==================== THREADS ====================
class DemoThread(QThread):
    line_ready = pyqtSignal(str)

    def run(self):
        params = {
            0x1001: (1200, 1200, 8),
            0x1002: (900,  1500, 10),
            0x1003: (1500, 800,  7),
            0x1004: (800,  2000, 9),
        }
        while True:
            for aid in ANCHOR_IDS:
                amp, d0, fp = params[aid]
                dist = d0 + int(random.gauss(0, 20))
                cir  = [max(0, int(
                    amp * math.exp(-0.5 * ((i - fp) / 2.5) ** 2)
                    + amp * 0.3 * math.exp(-0.5 * ((i - fp - 10) / 3.0) ** 2)
                    + random.gauss(0, amp * 0.06)
                )) for i in range(CIR_SAMPLES)]
                self.line_ready.emit(f"0x{aid:04X},{dist}," + ','.join(str(v) for v in cir))
                time.sleep(0.01)
            time.sleep(0.03)


class SerialThread(QThread):
    line_ready = pyqtSignal(str)

    def __init__(self, port, baud):
        super().__init__()
        self.port     = port
        self.baud     = baud
        self._running = True

    def run(self):
        import serial
        try:
            ser = serial.Serial(self.port, self.baud, timeout=1)
            ser.reset_input_buffer()
            print(f"✓ Serial: {self.port} @ {self.baud}")
            while self._running:
                if not ser.is_open:
                    break
                raw = ser.readline()
                if raw:
                    self.line_ready.emit(raw.decode('utf-8', errors='replace'))
        except Exception as e:
            print(f"✗ Serial: {e}")

    def stop(self):
        self._running = False

# ==================== MAIN WINDOW ====================
class MainWindow(QMainWindow):
    def __init__(self, demo, port, baud):
        super().__init__()
        self.setWindowTitle("CIR Real-time — 4 Anchors (C-Accelerated)")
        self.resize(1400, 820)
        self.setStyleSheet("background:#12122a; color:#e0e0e0;")
        self._build_ui()

        self.thread = DemoThread() if demo else SerialThread(port, baud)
        self.thread.line_ready.connect(parse_line)
        self.thread.start()

        self._t0 = time.time(); self._frames = 0
        self.timer = QTimer()
        self.timer.timeout.connect(self._refresh)
        self.timer.start(REFRESH_MS)

    def _build_ui(self):
        root = QWidget()
        vbox = QVBoxLayout(root)
        vbox.setContentsMargins(6, 6, 6, 6)
        vbox.setSpacing(4)
        self.setCentralWidget(root)

        # Top bar
        top = QHBoxLayout()
        top.addWidget(QLabel("<b>CIR Real-time Monitor — 4 Anchors</b>"))
        top.addStretch()
        self.lbl_fps = QLabel("FPS: --")
        self.lbl_fps.setStyleSheet("color:#666; font-size:11px;")
        top.addWidget(self.lbl_fps)
        vbox.addLayout(top)

        # 2×2 CIR plots
        grid_w = QWidget()
        grid   = QGridLayout(grid_w)
        grid.setSpacing(4)
        vbox.addWidget(grid_w, stretch=4)

        self.curves     = {}
        self.fp_lines   = {}
        self.info_items = {}
        x_cir = list(range(CIR_SAMPLES))

        for i, aid in enumerate(ANCHOR_IDS):
            row, col = divmod(i, 2)
            c = COLORS[aid]
            pw = pg.PlotWidget()
            pw.setBackground('#1a1a2e')
            pw.setTitle(
                f"<span style='color:rgb({c[0]},{c[1]},{c[2]});font-size:13pt'>"
                f"<b>Anchor {NAMES[aid]}</b></span>"
            )
            pw.getAxis('left').setLabel('Amplitude')
            pw.getAxis('bottom').setLabel('CIR sample')
            pw.setXRange(0, CIR_SAMPLES - 1, padding=0.02)
            pw.setYRange(0, 9000, padding=0.04)
            pw.showGrid(x=False, y=True, alpha=0.15)
            pw.getAxis('left').setTextPen('w')
            pw.getAxis('bottom').setTextPen('w')

            curve = pw.plot(x=x_cir, y=[0] * CIR_SAMPLES,
                            pen=pg.mkPen(color=c, width=2),
                            fillLevel=0,
                            brush=pg.mkBrush(c[0], c[1], c[2], 40))

            fp_line = pg.InfiniteLine(pos=8, angle=90,
                                      pen=pg.mkPen('#ff4444', width=1,
                                                   style=Qt.DashLine))
            pw.addItem(fp_line)

            info = pg.TextItem(text='waiting...', color=(200, 200, 200), anchor=(0, 0))
            info.setPos(1, 8500)
            pw.addItem(info)

            grid.addWidget(pw, row, col)
            self.curves[aid]     = curve
            self.fp_lines[aid]   = fp_line
            self.info_items[aid] = info

        # Sparkline bar
        spark_w = QWidget()
        spark_h = QHBoxLayout(spark_w)
        spark_h.setSpacing(6)
        vbox.addWidget(spark_w, stretch=1)

        self.spark_curves = {}
        self.dist_labels  = {}

        for aid in ANCHOR_IDS:
            c = COLORS[aid]
            box = QWidget()
            bv  = QVBoxLayout(box)
            bv.setContentsMargins(2, 2, 2, 2)
            bv.setSpacing(1)

            lbl = QLabel(f"{NAMES[aid]}: ---")
            lbl.setStyleSheet(
                f"color:rgb({c[0]},{c[1]},{c[2]}); font-size:11px; font-weight:bold;"
            )
            bv.addWidget(lbl)

            sp = pg.PlotWidget()
            sp.setBackground('#0d0d1f')
            sp.setMaximumHeight(55)
            sp.hideAxis('left'); sp.hideAxis('bottom')
            sp.setXRange(0, HISTORY_LEN, padding=0)
            sp.setMouseEnabled(x=False, y=False)
            sc = sp.plot(x=list(range(HISTORY_LEN)), y=[0] * HISTORY_LEN,
                         pen=pg.mkPen(color=c, width=1))
            bv.addWidget(sp)
            spark_h.addWidget(box)

            self.spark_curves[aid] = sc
            self.dist_labels[aid]  = lbl

    def _refresh(self):
        now = time.time()
        x_cir  = list(range(CIR_SAMPLES))
        x_hist = list(range(HISTORY_LEN))

        for aid in ANCHOR_IDS:
            with _lock:
                last = _last_ts[aid]
            if last > 0 and (now - last) > ALIVE_TIMEOUT:
                _lib.cir_mark_dead(aid)

            self.curves[aid].setData(x=x_cir, y=c_get_cir(aid))

            st = c_get_stats(aid)
            self.fp_lines[aid].setValue(st['fp_index'])

            dot = "●" if st['alive'] else "○"
            self.info_items[aid].setText(
                f"{dot}  {st['distance']:6d} mm   "
                f"frames: {st['frame_count']}   "
                f"fps: {st['fps']:.1f}   "
                f"peak: {st['cir_peak']}"
            )

            self.spark_curves[aid].setData(x=x_hist, y=c_get_hist(aid))
            self.dist_labels[aid].setText(f"{NAMES[aid]}:  {st['distance']:6d} mm")

        self._frames += 1
        if self._frames % 20 == 0:
            self.lbl_fps.setText(f"FPS: {self._frames / max(now - self._t0, 1e-6):.0f}")

    def closeEvent(self, event):
        self.timer.stop()
        if hasattr(self.thread, 'stop'):
            self.thread.stop()
        self.thread.quit()
        self.thread.wait(500)
        _lib.cir_reset()
        event.accept()

# ==================== MAIN ====================
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='/dev/ttyACM0')
    parser.add_argument('--baud', default=921600, type=int)
    parser.add_argument('--demo', action='store_true')
    args = parser.parse_args()

    pg.setConfigOptions(antialias=False)
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    win = MainWindow(demo=args.demo, port=args.port, baud=args.baud)
    win.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
