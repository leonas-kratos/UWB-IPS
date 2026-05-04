import sys
import serial
import time
import threading
import re
import numpy as np
from queue import Queue
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
from filterpy.kalman import ExtendedKalmanFilter

# 3 anchors (vì mỗi tag chỉ đo 3 khoảng cách)
ANCHORS = [
    (0, 0),
    (4807, 0),
    (4807, 4063),
]

# 5 Tags
TAG_IDS = ['0x567B', '0x5678', '0x5679', '0x567A', '0x567C']

TAG_COLORS = {
    '0x567B': (255, 80,  80),   # đỏ
    '0x5678': (80,  200, 255),  # xanh dương
    '0x5679': (80,  255, 140),  # xanh lá
    '0x567A': (255, 200, 50),   # vàng
    '0x567C': (220, 100, 255),  # tím
}

SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 921600

# ----- EKF cho từng tag -----

class EKFPositionTracker:
    def __init__(self):
        self.dim_x = 4   # [x, y, vx, vy]
        self.dim_z = 3   # [d1, d2, d3]
        self.ekf = ExtendedKalmanFilter(dim_x=self.dim_x, dim_z=self.dim_z)

        self.ekf.F = np.eye(self.dim_x)
        self.ekf.F[0, 2] = 1.0
        self.ekf.F[1, 3] = 1.0

        self.ekf.P = np.eye(self.dim_x) * 100
        self.ekf.Q = np.eye(self.dim_x) * 0.1
        self.ekf.R = np.eye(self.dim_z) * 50
        self.ekf.H = np.zeros((self.dim_z, self.dim_x))

        self.initialized = False
        self.dt = 0.1

    def H_jacobian(self, x):
        H = np.zeros((self.dim_z, self.dim_x))
        for i, anchor in enumerate(ANCHORS):
            dx = x[0] - anchor[0]
            dy = x[1] - anchor[1]
            dist = np.sqrt(dx**2 + dy**2)
            if dist > 0.001:
                H[i, 0] = dx / dist
                H[i, 1] = dy / dist
        return H

    def hx(self, x):
        distances = []
        for anchor in ANCHORS:
            dx = x[0] - anchor[0]
            dy = x[1] - anchor[1]
            distances.append(np.sqrt(dx**2 + dy**2))
        return np.array(distances)

    def initialize(self, initial_pos):
        self.ekf.x = np.array([initial_pos[0], initial_pos[1], 0.0, 0.0])
        self.initialized = True

    def update(self, distances, dt=0.1):
        if not self.initialized:
            return np.array([np.nan, np.nan])
        self.dt = dt
        self.ekf.F[0, 2] = dt
        self.ekf.F[1, 3] = dt
        self.ekf.predict()
        z = np.array(distances[:3])
        self.ekf.H = self.H_jacobian(self.ekf.x)
        self.ekf.update(z, self.H_jacobian, self.hx)
        return self.ekf.x[:2]


def lse_trilateration_3(distances):
    """LSE với 3 anchor, 3 khoảng cách."""
    if len(distances) < 3:
        return np.array([np.nan, np.nan])
    x1, y1 = ANCHORS[0]
    d1 = distances[0]
    A = []
    b = []
    for i in range(1, 3):
        xi, yi = ANCHORS[i]
        di = distances[i]
        A.append([xi - x1, yi - y1])
        b.append(0.5 * (xi**2 + yi**2 - di**2 - (x1**2 + y1**2 - d1**2)))
    try:
        A = np.array(A, dtype=float)
        b = np.array(b, dtype=float)
        pos, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        return pos
    except Exception as e:
        print("LSE error:", e)
        return np.array([np.nan, np.nan])


# ----- Main GUI -----

class MultiTagTracker(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Multi-Tag UWB Tracking with EKF")
        self.setGeometry(100, 100, 900, 700)

        # Queue và state mỗi tag
        self.data_queue = Queue()
        self.tag_state = {
            tag: {
                'ekf': EKFPositionTracker(),
                'initialized': False,
                'trail': [],
                'last_time': time.time(),
            }
            for tag in TAG_IDS
        }

        # Regex: 0x567B,938,910,1257
        self.line_pattern = re.compile(
            r'(0x[0-9a-fA-F]{4})\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)'
        )

        self.fps_time = time.time()

        self.init_serial()
        self.init_ui()
        self.start_serial_thread()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(15)

    def init_serial(self):
        try:
            self.ser = serial.Serial(
                port=SERIAL_PORT,
                baudrate=BAUDRATE,
                timeout=0.1
            )
            print(f"Mở cổng serial {SERIAL_PORT} thành công.")
        except Exception as e:
            print("Lỗi mở serial:", e)
            sys.exit(1)

    def init_ui(self):
        self.plot_widget = pg.PlotWidget()
        self.setCentralWidget(self.plot_widget)

        self.plot_widget.setXRange(-1000, 6500)
        self.plot_widget.setYRange(-1000, 6500)
        self.plot_widget.setTitle("Multi-Tag UWB Tracking - EKF Real-Time")
        self.plot_widget.setBackground('#111118')
        self.plot_widget.showGrid(x=True, y=True, alpha=0.2)

        # Vẽ anchors
        anchor_x, anchor_y = zip(*ANCHORS)
        self.plot_widget.plot(
            list(anchor_x), list(anchor_y),
            pen=None, symbol='t', symbolSize=18,
            symbolBrush=pg.mkBrush(100, 180, 255),
            symbolPen=pg.mkPen('w', width=1),
            name="Anchors"
        )
        for i, (ax, ay) in enumerate(ANCHORS):
            txt = pg.TextItem(text=f"A{i}", color=(150, 200, 255), anchor=(0.5, 1.5))
            txt.setPos(ax, ay)
            self.plot_widget.addItem(txt)

        # Vẽ zone (tuỳ chỉnh nếu cần)
        self.plot_widget.plot(
            [1200, 1200, 4800, 4800, 1200],
            [600, 2710, 2710, 600, 600],
            pen=pg.mkPen((80, 255, 120, 80), width=1.5)
        )

        # Scatter và trail cho mỗi tag
        self.tag_plots = {}
        for tag in TAG_IDS:
            color = TAG_COLORS[tag]
            brush = pg.mkBrush(*color, 220)
            pen   = pg.mkPen(*color, width=1.5)
            scatter = self.plot_widget.plot(
                [], [], pen=None, symbol='o',
                symbolSize=14, symbolBrush=brush,
                symbolPen=pg.mkPen('w', width=1)
            )
            trail = self.plot_widget.plot([], [], pen=pen)
            label = pg.TextItem(text=tag, color=color, anchor=(0.5, -0.8))
            self.plot_widget.addItem(label)
            self.tag_plots[tag] = {
                'scatter': scatter,
                'trail': trail,
                'label': label,
            }

        # Legend đơn giản
        legend = self.plot_widget.addLegend(offset=(10, 10))

    def start_serial_thread(self):
        threading.Thread(target=self.serial_reader, daemon=True).start()

    def serial_reader(self):
        """Đọc từng dòng UART, mỗi dòng là 1 tag."""
        while True:
            try:
                line = self.ser.readline().decode('utf-8', errors='replace').strip()
                if line:
                    self.data_queue.put(line)
            except Exception as e:
                print("Serial error:", e)

    def update_plot(self):
        current_time = time.time()
        fps = 1.0 / max(current_time - self.fps_time, 1e-6)
        self.fps_time = current_time

        updated_tags = set()

        while not self.data_queue.empty():
            raw = self.data_queue.get()
            # Tìm tất cả match trong dòng (có thể nhiều tag ghép 1 dòng)
            matches = self.line_pattern.findall(raw)
            for match in matches:
                tag_id = match[0].lower()
                # Chuẩn hoá: "0x567b" -> "0x567B"
                tag_id = '0x' + tag_id[2:].upper()
                if tag_id not in TAG_IDS:
                    continue
                distances = [int(match[1]), int(match[2]), int(match[3])]
                state = self.tag_state[tag_id]

                dt = current_time - state['last_time']
                state['last_time'] = current_time
                if dt <= 0 or dt > 5:
                    dt = 0.1

                # Khởi tạo EKF lần đầu
                if not state['initialized']:
                    init_pos = lse_trilateration_3(distances)
                    if not np.isnan(init_pos).any():
                        state['ekf'].initialize(init_pos)
                        state['initialized'] = True
                        filtered_pos = init_pos
                    else:
                        continue
                else:
                    filtered_pos = state['ekf'].update(distances, dt)

                if np.isnan(filtered_pos).any():
                    continue

                # Cập nhật trail
                state['trail'].append(filtered_pos.copy())
                if len(state['trail']) > 500:
                    state['trail'] = state['trail'][-500:]

                print(f"[{tag_id}] X: {filtered_pos[0]:.1f}  Y: {filtered_pos[1]:.1f}  | d: {distances}")
                updated_tags.add(tag_id)

                # Cập nhật scatter & label
                plots = self.tag_plots[tag_id]
                plots['scatter'].setData([filtered_pos[0]], [filtered_pos[1]])
                plots['label'].setPos(filtered_pos[0], filtered_pos[1])

        # Cập nhật trail curve
        for tag_id in updated_tags:
            trail = self.tag_state[tag_id]['trail']
            if len(trail) >= 2:
                arr = np.array(trail)
                self.tag_plots[tag_id]['trail'].setData(arr[:, 0], arr[:, 1])

        self.setWindowTitle(f"Multi-Tag UWB Tracking with EKF  —  FPS: {fps:.1f}")


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    tracker = MultiTagTracker()
    tracker.show()
    sys.exit(app.exec_())
