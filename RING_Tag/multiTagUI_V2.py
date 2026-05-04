import sys
import serial
import time
import threading
import re
import argparse
import numpy as np
from queue import Queue
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
from filterpy.kalman import ExtendedKalmanFilter, UnscentedKalmanFilter, MerweScaledSigmaPoints
from filterpy.kalman import KalmanFilter

ANCHORS = [
    (0, 0),
    (4807, 0),
    (4807, 4063),
]

SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 921600

COLOR_PALETTE = [
    (255, 80,  80),
    (80,  200, 255),
    (80,  255, 140),
    (255, 200, 50),
    (220, 100, 255),
    (255, 160, 50),
    (50,  255, 220),
    (255, 80,  180),
]

ALGORITHM_NAMES = {
    None:  'LSE Only (no filter)',
    'kf':  'Kalman Filter (KF)',
    'ekf': 'Extended Kalman Filter (EKF)',
    'ukf': 'Unscented Kalman Filter (UKF)',
    'pf':  'Particle Filter (PF)',
}


# ──────────────────────────────────────────────
#  Trilateration LSE
# ──────────────────────────────────────────────

def lse_trilateration(distances):
    if len(distances) < 3:
        return np.array([np.nan, np.nan])
    x1, y1 = ANCHORS[0]
    d1 = distances[0]
    A, b = [], []
    for i in range(1, 3):
        xi, yi = ANCHORS[i]
        di = distances[i]
        A.append([xi - x1, yi - y1])
        b.append(0.5 * (xi**2 + yi**2 - di**2 - (x1**2 + y1**2 - d1**2)))
    try:
        pos, _, _, _ = np.linalg.lstsq(np.array(A, float), np.array(b, float), rcond=None)
        return pos
    except Exception as e:
        print("LSE error:", e)
        return np.array([np.nan, np.nan])


# ──────────────────────────────────────────────
#  Measurement model chung (cho EKF / UKF)
# ──────────────────────────────────────────────

def hx(state):
    return np.array([
        np.sqrt((state[0] - ax)**2 + (state[1] - ay)**2)
        for ax, ay in ANCHORS
    ])

def H_jacobian(state):
    H = np.zeros((3, 4))
    for i, (ax, ay) in enumerate(ANCHORS):
        dx = state[0] - ax
        dy = state[1] - ay
        dist = np.sqrt(dx**2 + dy**2)
        if dist > 0.001:
            H[i, 0] = dx / dist
            H[i, 1] = dy / dist
    return H


# ──────────────────────────────────────────────
#  LSE Tracker (pass-through, không filter)
# ──────────────────────────────────────────────

class LSETracker:
    def __init__(self):
        self.initialized = True   # luôn sẵn sàng

    def initialize(self, pos):
        pass   # không cần state

    def update(self, distances, dt=0.1):
        return lse_trilateration(distances)


# ──────────────────────────────────────────────
#  KF
# ──────────────────────────────────────────────

class KFTracker:
    def __init__(self):
        self.kf = KalmanFilter(dim_x=4, dim_z=2)
        self.kf.F = np.eye(4)
        self.kf.F[0, 2] = 1.0
        self.kf.F[1, 3] = 1.0
        self.kf.H = np.array([[1, 0, 0, 0],
                               [0, 1, 0, 0]], dtype=float)
        self.kf.P = np.eye(4) * 500
        self.kf.Q = np.eye(4) * 0.5
        self.kf.R = np.eye(2) * 200
        self.initialized = False

    def initialize(self, pos):
        self.kf.x = np.array([pos[0], pos[1], 0.0, 0.0])
        self.initialized = True

    def update(self, distances, dt=0.1):
        if not self.initialized:
            return np.array([np.nan, np.nan])
        self.kf.F[0, 2] = dt
        self.kf.F[1, 3] = dt
        self.kf.predict()
        z = lse_trilateration(distances)
        if not np.isnan(z).any():
            self.kf.update(z)
        return self.kf.x[:2]


# ──────────────────────────────────────────────
#  EKF
# ──────────────────────────────────────────────

class EKFTracker:
    def __init__(self):
        self.ekf = ExtendedKalmanFilter(dim_x=4, dim_z=3)
        self.ekf.F = np.eye(4)
        self.ekf.F[0, 2] = 1.0
        self.ekf.F[1, 3] = 1.0
        self.ekf.P = np.eye(4) * 100
        self.ekf.Q = np.eye(4) * 0.1
        self.ekf.R = np.eye(3) * 50
        self.ekf.H = np.zeros((3, 4))
        self.initialized = False

    def initialize(self, pos):
        self.ekf.x = np.array([pos[0], pos[1], 0.0, 0.0])
        self.initialized = True

    def update(self, distances, dt=0.1):
        if not self.initialized:
            return np.array([np.nan, np.nan])
        self.ekf.F[0, 2] = dt
        self.ekf.F[1, 3] = dt
        self.ekf.predict()
        self.ekf.H = H_jacobian(self.ekf.x)
        self.ekf.update(np.array(distances[:3], float), H_jacobian, hx)
        return self.ekf.x[:2]


# ──────────────────────────────────────────────
#  UKF
# ──────────────────────────────────────────────

def fx_ukf(x, dt):
    F = np.eye(4)
    F[0, 2] = dt
    F[1, 3] = dt
    return F @ x

class UKFTracker:
    def __init__(self):
        points = MerweScaledSigmaPoints(n=4, alpha=0.1, beta=2.0, kappa=1.0)
        self.ukf = UnscentedKalmanFilter(
            dim_x=4, dim_z=3, dt=0.1,
            fx=fx_ukf, hx=hx, points=points
        )
        self.ukf.P = np.eye(4) * 100
        self.ukf.Q = np.eye(4) * 0.1
        self.ukf.R = np.eye(3) * 50
        self.initialized = False

    def initialize(self, pos):
        self.ukf.x = np.array([pos[0], pos[1], 0.0, 0.0])
        self.initialized = True

    def update(self, distances, dt=0.1):
        if not self.initialized:
            return np.array([np.nan, np.nan])
        self.ukf.predict(dt=dt)
        self.ukf.update(np.array(distances[:3], float))
        return self.ukf.x[:2]


# ──────────────────────────────────────────────
#  Particle Filter
# ──────────────────────────────────────────────

class PFTracker:
    def __init__(self, n_particles=300):
        self.N = n_particles
        self.process_std = 80.0
        self.obs_std     = 120.0
        self.particles   = None
        self.weights     = None
        self.initialized = False

    def initialize(self, pos):
        self.particles = np.zeros((self.N, 4))
        self.particles[:, 0] = pos[0] + np.random.randn(self.N) * 200
        self.particles[:, 1] = pos[1] + np.random.randn(self.N) * 200
        self.particles[:, 2] = np.random.randn(self.N) * 50
        self.particles[:, 3] = np.random.randn(self.N) * 50
        self.weights = np.ones(self.N) / self.N
        self.initialized = True

    def update(self, distances, dt=0.1):
        if not self.initialized:
            return np.array([np.nan, np.nan])
        # Predict
        self.particles[:, 0] += self.particles[:, 2] * dt + np.random.randn(self.N) * self.process_std
        self.particles[:, 1] += self.particles[:, 3] * dt + np.random.randn(self.N) * self.process_std
        self.particles[:, 2] += np.random.randn(self.N) * (self.process_std * 0.3)
        self.particles[:, 3] += np.random.randn(self.N) * (self.process_std * 0.3)
        # Update weights
        z = np.array(distances[:3], float)
        for i, (ax, ay) in enumerate(ANCHORS):
            pred_d = np.sqrt((self.particles[:, 0] - ax)**2 +
                             (self.particles[:, 1] - ay)**2)
            self.weights *= np.exp(-0.5 * ((pred_d - z[i]) / self.obs_std)**2)
        total = self.weights.sum()
        if total < 1e-300:
            self.weights = np.ones(self.N) / self.N
        else:
            self.weights /= total
        # Resample nếu Neff thấp
        if 1.0 / np.sum(self.weights**2) < self.N / 2:
            positions = (np.arange(self.N) + np.random.uniform()) / self.N
            cumsum = np.cumsum(self.weights)
            i, j, indexes = 0, 0, np.zeros(self.N, int)
            while i < self.N:
                if positions[i] < cumsum[j]:
                    indexes[i] = j; i += 1
                else:
                    j += 1
            self.particles = self.particles[indexes]
            self.weights = np.ones(self.N) / self.N
        return np.array([
            np.average(self.particles[:, 0], weights=self.weights),
            np.average(self.particles[:, 1], weights=self.weights),
        ])


# ──────────────────────────────────────────────
#  Factory
# ──────────────────────────────────────────────

def make_tracker(algorithm):
    """algorithm=None → LSE thuần."""
    if algorithm is None:
        return LSETracker()
    algo = algorithm.lower()
    if algo == 'kf':  return KFTracker()
    if algo == 'ekf': return EKFTracker()
    if algo == 'ukf': return UKFTracker()
    if algo == 'pf':  return PFTracker()
    raise ValueError(f"Thuật toán không hợp lệ: '{algorithm}'. Chọn: kf / ekf / ukf / pf")


# ──────────────────────────────────────────────
#  Main GUI
# ──────────────────────────────────────────────

class MultiTagTracker(QtWidgets.QMainWindow):
    new_tag_signal = QtCore.pyqtSignal(str)

    def __init__(self, algorithm):
        super().__init__()
        self.algorithm  = algorithm                          # None hoặc str
        self.algo_label = ALGORITHM_NAMES.get(algorithm, algorithm.upper() if algorithm else '')

        self.data_queue = Queue()
        self.tag_state  = {}
        self.tag_plots  = {}
        self.tag_lock   = threading.Lock()
        self._color_idx = 0

        self.line_pattern = re.compile(
            r'(0x[0-9a-fA-F]{4})\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)'
        )
        self.fps_time = time.time()

        self.new_tag_signal.connect(self._add_tag_to_plot)
        self.init_serial()
        self.init_ui()
        self.start_serial_thread()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(15)

    # ── Serial ────────────────────────────────
    def init_serial(self):
        try:
            self.ser = serial.Serial(port=SERIAL_PORT, baudrate=BAUDRATE, timeout=0.1)
            print(f"[OK] Serial {SERIAL_PORT} @ {BAUDRATE}")
        except Exception as e:
            print(f"[ERR] Không mở được serial: {e}")
            sys.exit(1)

    def start_serial_thread(self):
        threading.Thread(target=self._serial_reader, daemon=True).start()

    def _serial_reader(self):
        while True:
            try:
                line = self.ser.readline().decode('utf-8', errors='replace').strip()
                if line:
                    self.data_queue.put(line)
            except Exception as e:
                print("Serial error:", e)

    # ── UI ────────────────────────────────────
    def init_ui(self):
        self.plot_widget = pg.PlotWidget()
        self.setCentralWidget(self.plot_widget)
        self.plot_widget.setXRange(-1000, 6500)
        self.plot_widget.setYRange(-1000, 6500)
        self.plot_widget.setBackground('#111118')
        self.plot_widget.showGrid(x=True, y=True, alpha=0.2)

        ax_list, ay_list = zip(*ANCHORS)
        self.plot_widget.plot(
            list(ax_list), list(ay_list),
            pen=None, symbol='t', symbolSize=18,
            symbolBrush=pg.mkBrush(100, 180, 255),
            symbolPen=pg.mkPen('w', width=1),
        )
        for i, (ax, ay) in enumerate(ANCHORS):
            t = pg.TextItem(text=f"A{i}", color=(150, 200, 255), anchor=(0.5, 1.5))
            t.setPos(ax, ay)
            self.plot_widget.addItem(t)

        badge = pg.TextItem(
            text=f"Algorithm: {self.algo_label}",
            color=(220, 220, 80), anchor=(0, 0)
        )
        badge.setPos(-900, 6200)
        self.plot_widget.addItem(badge)

    # ── Tag management ────────────────────────
    def _next_color(self):
        c = COLOR_PALETTE[self._color_idx % len(COLOR_PALETTE)]
        self._color_idx += 1
        return c

    def _register_tag(self, tag_id):
        with self.tag_lock:
            if tag_id in self.tag_state:
                return
            color = self._next_color()
            self.tag_state[tag_id] = {
                'tracker':     make_tracker(self.algorithm),
                'initialized': False,
                'trail':       [],
                'last_time':   time.time(),
                'color':       color,
            }
        print(f"[NEW TAG] {tag_id}  algorithm={self.algo_label}")
        self.new_tag_signal.emit(tag_id)

    @QtCore.pyqtSlot(str)
    def _add_tag_to_plot(self, tag_id):
        with self.tag_lock:
            if tag_id in self.tag_plots:
                return
            color = self.tag_state[tag_id]['color']
        brush   = pg.mkBrush(*color, 220)
        pen     = pg.mkPen(*color, width=1.5)
        scatter = self.plot_widget.plot(
            [], [], pen=None, symbol='o',
            symbolSize=14, symbolBrush=brush,
            symbolPen=pg.mkPen('w', width=1)
        )
        trail = self.plot_widget.plot([], [], pen=pen)
        label = pg.TextItem(text=tag_id, color=color, anchor=(0.5, -0.8))
        self.plot_widget.addItem(label)
        with self.tag_lock:
            self.tag_plots[tag_id] = {'scatter': scatter, 'trail': trail, 'label': label}

    # ── Main update loop ──────────────────────
    def update_plot(self):
        current_time = time.time()
        fps = 1.0 / max(current_time - self.fps_time, 1e-6)
        self.fps_time = current_time
        updated_tags  = set()

        while not self.data_queue.empty():
            raw     = self.data_queue.get()
            matches = self.line_pattern.findall(raw)
            for match in matches:
                tag_id    = '0x' + match[0][2:].upper()
                distances = [int(match[1]), int(match[2]), int(match[3])]

                if tag_id not in self.tag_state:
                    self._register_tag(tag_id)
                    continue

                state   = self.tag_state[tag_id]
                tracker = state['tracker']

                dt = current_time - state['last_time']
                state['last_time'] = current_time
                if dt <= 0 or dt > 5:
                    dt = 0.1

                # LSETracker.initialized luôn True nên skip init block
                if not state['initialized']:
                    init_pos = lse_trilateration(distances)
                    if np.isnan(init_pos).any():
                        continue
                    tracker.initialize(init_pos)
                    state['initialized'] = True
                    filtered_pos = init_pos
                else:
                    filtered_pos = tracker.update(distances, dt)

                if np.isnan(filtered_pos).any():
                    continue

                state['trail'].append(filtered_pos.copy())
                if len(state['trail']) > 500:
                    state['trail'] = state['trail'][-500:]

                algo_str = self.algorithm.upper() if self.algorithm else 'LSE'
                print(f"[{tag_id}|{algo_str}] X={filtered_pos[0]:.1f}  Y={filtered_pos[1]:.1f}  d={distances}")
                updated_tags.add(tag_id)

                with self.tag_lock:
                    plots = self.tag_plots.get(tag_id)
                if plots:
                    plots['scatter'].setData([filtered_pos[0]], [filtered_pos[1]])
                    plots['label'].setPos(filtered_pos[0], filtered_pos[1])

        for tag_id in updated_tags:
            with self.tag_lock:
                plots = self.tag_plots.get(tag_id)
            if not plots:
                continue
            trail = self.tag_state[tag_id]['trail']
            if len(trail) >= 2:
                arr = np.array(trail)
                plots['trail'].setData(arr[:, 0], arr[:, 1])

        algo_str = self.algorithm.upper() if self.algorithm else 'LSE'
        self.setWindowTitle(
            f"Multi-Tag UWB Tracking  [{algo_str}]"
            f"  —  FPS: {fps:.1f}  |  Tags: {len(self.tag_state)}"
        )


# ──────────────────────────────────────────────
#  Entry point
# ──────────────────────────────────────────────

def parse_args():
    parser = argparse.ArgumentParser(
        description="Multi-Tag UWB Tracker — mặc định LSE, dùng -a để chọn filter.",
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument(
        '-a', '--algorithm',
        choices=['kf', 'ekf', 'ukf', 'pf'],
        default=None,          # ← None = LSE thuần
        metavar='ALGO',
        help=(
            "Filter algorithm (mặc định: không dùng, chỉ LSE):\n"
            "  kf  – Kalman Filter\n"
            "  ekf – Extended Kalman Filter\n"
            "  ukf – Unscented Kalman Filter\n"
            "  pf  – Particle Filter\n"
        )
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    label = ALGORITHM_NAMES[args.algorithm]
    print(f"[CONFIG] Algorithm: {label}")

    app = QtWidgets.QApplication(sys.argv)
    tracker = MultiTagTracker(algorithm=args.algorithm)
    tracker.show()
    sys.exit(app.exec_())
