"""
CIR Real-time Plotter - 4 Anchors
Format: 0x1001,<distance_mm>,<cir_0>,...,<cir_49>

pip install pyqtgraph PyQt5 pyserial
Demo:   python cir_4anchor_realtime.py --demo
Serial: python cir_4anchor_realtime.py --port /dev/ttyACM0 --baud 115200
"""

import sys, argparse, collections, time, math, random, threading
import numpy as np

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget,
                              QVBoxLayout, QHBoxLayout, QGridLayout,
                              QLabel, QPushButton)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
import pyqtgraph as pg

# ── CONFIG ──────────────────────────────────────────────
ANCHOR_IDS  = [0x1001, 0x1002, 0x1003, 0x1004]
CIR_SAMPLES = 50
HISTORY_LEN = 100
COL_DIST    = 1
COL_CIR     = 2
REFRESH_MS  = 40

COLORS = {
    0x1001: (100, 200, 255),
    0x1002: (100, 255, 150),
    0x1003: (255, 180,  80),
    0x1004: (255, 100, 100),
}
NAMES = {a: f"0x{a:04X}" for a in ANCHOR_IDS}

# ── THREAD-SAFE STATE ───────────────────────────────────
_lock = threading.Lock()

class AnchorState:
    def __init__(self):
        self.cir         = [0] * CIR_SAMPLES
        self.dist_hist   = collections.deque([0]*HISTORY_LEN, maxlen=HISTORY_LEN)
        self.distance    = 0
        self.frame_count = 0
        self.last_ts     = 0.0

anchors = {aid: AnchorState() for aid in ANCHOR_IDS}

def parse_line(line: str):
    parts = line.strip().split(',')
    if len(parts) < COL_CIR + CIR_SAMPLES:
        return
    try:
        s   = parts[0].strip()
        aid = int(s, 16) if 'x' in s.lower() else int(s)
        if aid not in anchors:
            return
        dist = int(parts[COL_DIST])
        cir  = [max(0, int(v)) for v in parts[COL_CIR:COL_CIR + CIR_SAMPLES]]
        with _lock:
            st = anchors[aid]
            st.distance = dist
            st.cir      = cir
            st.dist_hist.append(dist)
            st.frame_count += 1
            st.last_ts = time.time()
    except (ValueError, IndexError):
        pass

# ── THREADS ─────────────────────────────────────────────
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
                cir  = []
                for i in range(CIR_SAMPLES):
                    v = int(amp   * math.exp(-0.5*((i-fp)/2.5)**2)
                          + amp*0.3 * math.exp(-0.5*((i-fp-10)/3.0)**2)
                          + random.gauss(0, amp*0.06))
                    cir.append(max(0, v))
                row = f"0x{aid:04X},{dist}," + ','.join(str(v) for v in cir)
                self.line_ready.emit(row)
                time.sleep(0.01)
            time.sleep(0.03)


class SerialThread(QThread):
    line_ready = pyqtSignal(str)

    def __init__(self, port, baud):
        super().__init__()
        self.port = port
        self.baud = baud

    def run(self):
        import serial
        try:
            ser = serial.Serial(self.port, self.baud, timeout=1)
            ser.reset_input_buffer()
            while True:
                raw = ser.readline()
                if raw:
                    self.line_ready.emit(raw.decode('utf-8', errors='replace'))
        except Exception as e:
            print(f"[Serial] {e}")

# ── MAIN WINDOW ─────────────────────────────────────────
class MainWindow(QMainWindow):
    def __init__(self, demo, port, baud):
        super().__init__()
        self.setWindowTitle("CIR Real-time — 4 Anchors")
        self.resize(1400, 900)
        self.setStyleSheet("background:#12122a; color:#e0e0e0;")

        # FFT on/off state per anchor
        self.fft_enabled = {aid: False for aid in ANCHOR_IDS}

        self._build_ui()

        self.thread = DemoThread() if demo else SerialThread(port, baud)
        self.thread.line_ready.connect(parse_line)
        self.thread.start()

        self._t0     = time.time()
        self._frames = 0
        self.timer   = QTimer()
        self.timer.timeout.connect(self._refresh)
        self.timer.start(REFRESH_MS)

    # ── BUILD ──────────────────────────────────────────
    def _build_ui(self):
        root = QWidget()
        vbox = QVBoxLayout(root)
        vbox.setContentsMargins(6, 6, 6, 6)
        vbox.setSpacing(4)
        self.setCentralWidget(root)

        # top bar
        top = QHBoxLayout()
        t   = QLabel("CIR Real-time Monitor — 4 Anchors")
        t.setStyleSheet("font-size:14px; font-weight:bold;")
        top.addWidget(t)
        top.addStretch()
        self.lbl_fps = QLabel("FPS: --")
        self.lbl_fps.setStyleSheet("color:#666; font-size:11px;")
        top.addWidget(self.lbl_fps)
        vbox.addLayout(top)

        # 2×2 grid of plot panels
        grid_w = QWidget()
        grid   = QGridLayout(grid_w)
        grid.setSpacing(4)
        vbox.addWidget(grid_w, stretch=4)

        self.curves      = {}
        self.fft_curves  = {}
        self.fp_lines    = {}
        self.info_items  = {}
        self.plot_widgets = {}
        self.fft_plot_widgets = {}
        self.fft_buttons = {}
        self.panel_stacks = {}   # {aid: QWidget container with vbox}

        x_cir = list(range(CIR_SAMPLES))

        for i, aid in enumerate(ANCHOR_IDS):
            row, col = divmod(i, 2)
            c = COLORS[aid]

            # ── outer container for this cell ──────────
            cell_w = QWidget()
            cell_v = QVBoxLayout(cell_w)
            cell_v.setContentsMargins(0, 0, 0, 0)
            cell_v.setSpacing(2)

            # ── header row: title + FFT button ─────────
            hdr = QHBoxLayout()
            title_lbl = QLabel(f"Anchor {NAMES[aid]}")
            title_lbl.setStyleSheet(
                f"color:rgb({c[0]},{c[1]},{c[2]}); font-size:13pt; font-weight:bold;"
            )
            hdr.addWidget(title_lbl)
            hdr.addStretch()

            fft_btn = QPushButton("FFT OFF")
            fft_btn.setCheckable(True)
            fft_btn.setChecked(False)
            fft_btn.setFixedWidth(80)
            fft_btn.setStyleSheet(
                "QPushButton {"
                "  background:#1e1e40; color:#888; border:1px solid #444;"
                "  border-radius:4px; padding:2px 6px; font-size:11px;"
                "}"
                "QPushButton:checked {"
                f"  background:rgb({c[0]//3},{c[1]//3},{c[2]//3});"
                f"  color:rgb({c[0]},{c[1]},{c[2]});"
                "  border:1px solid rgba(200,200,200,0.4);"
                "}"
            )
            fft_btn.toggled.connect(lambda checked, a=aid: self._toggle_fft(a, checked))
            hdr.addWidget(fft_btn)
            cell_v.addLayout(hdr)

            # ── CIR plot ───────────────────────────────
            pw = pg.PlotWidget()
            pw.setBackground('#1a1a2e')
            pw.getAxis('left').setLabel('Amplitude')
            pw.getAxis('bottom').setLabel('CIR sample')
            pw.setXRange(0, CIR_SAMPLES - 1, padding=0.02)
            pw.setYRange(0, 9000, padding=0.04)
            pw.showGrid(x=False, y=True, alpha=0.15)
            pw.getAxis('left').setTextPen('w')
            pw.getAxis('bottom').setTextPen('w')

            curve = pw.plot(
                x=x_cir, y=[0]*CIR_SAMPLES,
                pen=pg.mkPen(color=c, width=2),
                fillLevel=0,
                brush=pg.mkBrush(c[0], c[1], c[2], 40),
            )

            fp_line = pg.InfiniteLine(
                pos=8, angle=90,
                pen=pg.mkPen('#ff4444', width=1, style=Qt.DashLine)
            )
            pw.addItem(fp_line)

            info = pg.TextItem(text='waiting...', color=(200,200,200), anchor=(0,0))
            info.setPos(1, 8500)
            pw.addItem(info)

            cell_v.addWidget(pw, stretch=3)

            # ── FFT plot (hidden by default) ───────────
            fft_pw = pg.PlotWidget()
            fft_pw.setBackground('#0d0d22')
            fft_pw.getAxis('left').setLabel('|FFT|')
            fft_pw.getAxis('bottom').setLabel('Frequency bin')
            fft_pw.showGrid(x=False, y=True, alpha=0.15)
            fft_pw.getAxis('left').setTextPen('w')
            fft_pw.getAxis('bottom').setTextPen('w')

            # brighter / thinner color for FFT line
            fc = tuple(min(255, v + 60) for v in c)
            fft_curve = fft_pw.plot(
                pen=pg.mkPen(color=fc, width=1.5),
                fillLevel=0,
                brush=pg.mkBrush(fc[0], fc[1], fc[2], 30),
            )

            fft_pw.hide()
            cell_v.addWidget(fft_pw, stretch=2)

            grid.addWidget(cell_w, row, col)

            self.curves[aid]           = curve
            self.fft_curves[aid]       = fft_curve
            self.fp_lines[aid]         = fp_line
            self.info_items[aid]       = info
            self.plot_widgets[aid]     = pw
            self.fft_plot_widgets[aid] = fft_pw
            self.fft_buttons[aid]      = fft_btn

        # ── sparkline bar (distance history) ──────────
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
                f"color:rgb{c}; font-size:11px; font-weight:bold;"
            )
            bv.addWidget(lbl)

            sp = pg.PlotWidget()
            sp.setBackground('#0d0d1f')
            sp.setMaximumHeight(55)
            sp.hideAxis('left')
            sp.hideAxis('bottom')
            sp.setXRange(0, HISTORY_LEN, padding=0)
            sp.setMouseEnabled(x=False, y=False)

            sc = sp.plot(
                x=list(range(HISTORY_LEN)), y=[0]*HISTORY_LEN,
                pen=pg.mkPen(color=c, width=1),
            )
            bv.addWidget(sp)
            spark_h.addWidget(box)

            self.spark_curves[aid] = sc
            self.dist_labels[aid]  = lbl

    # ── TOGGLE FFT ─────────────────────────────────────
    def _toggle_fft(self, aid, enabled):
        self.fft_enabled[aid] = enabled
        self.fft_buttons[aid].setText("FFT ON" if enabled else "FFT OFF")
        fft_pw = self.fft_plot_widgets[aid]
        if enabled:
            fft_pw.show()
        else:
            fft_pw.hide()

    # ── REFRESH ────────────────────────────────────────
    def _refresh(self):
        with _lock:
            snapshot = {
                aid: (st.cir[:], list(st.dist_hist),
                      st.distance, st.frame_count, st.last_ts)
                for aid, st in anchors.items()
            }

        x_cir  = list(range(CIR_SAMPLES))
        x_hist = list(range(HISTORY_LEN))
        n_fft  = CIR_SAMPLES // 2      # positive frequencies only

        for aid in ANCHOR_IDS:
            cir, dist_hist, dist, fc, ts = snapshot[aid]

            # CIR waveform
            self.curves[aid].setData(x=x_cir, y=cir)

            # first-path marker
            fp_idx = cir[:20].index(max(cir[:20])) if any(cir[:20]) else 0
            self.fp_lines[aid].setValue(fp_idx)

            # info text
            alive = time.time() - ts < 1.5
            dot   = "●" if alive else "○"
            self.info_items[aid].setText(
                f"{dot}  {dist:6d} mm   frames: {fc}"
            )

            # FFT (only compute when panel is visible)
            if self.fft_enabled[aid]:
                arr      = np.array(cir, dtype=np.float64)
                arr     -= arr.mean()                       # remove DC
                arr     *= np.hanning(len(arr))             # window
                spectrum = np.abs(np.fft.rfft(arr))[:n_fft]
                x_fft    = list(range(len(spectrum)))
                self.fft_curves[aid].setData(x=x_fft, y=spectrum.tolist())

            # sparkline
            self.spark_curves[aid].setData(x=x_hist, y=dist_hist)
            self.dist_labels[aid].setText(
                f"{NAMES[aid]}:  {dist:6d} mm"
            )

        # FPS
        self._frames += 1
        if self._frames % 20 == 0:
            fps = self._frames / max(time.time() - self._t0, 1e-6)
            self.lbl_fps.setText(f"FPS: {fps:.0f}")


# ── ENTRY ───────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port',  default='/dev/ttyACM0')
    parser.add_argument('--baud',  default=921600, type=int)
    parser.add_argument('--demo',  action='store_true')
    args = parser.parse_args()

    pg.setConfigOptions(antialias=False)
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    win = MainWindow(demo=args.demo, port=args.port, baud=args.baud)
    win.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
