"""
uwb_ui.py  —  Chạy trên MÁY UI  (1 tag, có AI LOS/NLOS panel)
===============================================================
Nhận JSON từ tag_backend.py qua TCP, vẽ map realtime + hiện
trạng thái LOS/NLOS của từng anchor.

Format JSON nhận vào:
    {"tag":"TAG1","x":1234.0,"y":5678.0,
     "anchors":[
       {"id":"1001","dist":1500.0,"los":1,"prob_nlos":0.08},
       ...
     ]}

Cài đặt:
    pip install PyQt5 pyqtgraph

Chạy:
    python uwb_ui.py
"""

import sys, json, socket, threading
from queue import Queue, Empty
from collections import deque
import numpy as np

from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg

# ══════════════════════════════════════════
#  CHỈNH Ở ĐÂY
# ══════════════════════════════════════════
TAG_SOURCE = {"host": "100.102.252.35", "port": 9001}
TAG_ID     = "TAG1"
# ══════════════════════════════════════════

ANCHORS = [
    (0,     0    ),
    (0,     4400 ),
    (14800, 4400 ),
    (14800, 0    ),
]
ANCHOR_IDS = ["1001", "1002", "1003", "1004"]
TRAIL_LEN  = 600

TAG_COLOR = (255, 160, 40)   # amber


# ═══════════════════════════════════════════════════════════════════════════════
#  TCP RECEIVER
# ═══════════════════════════════════════════════════════════════════════════════
class TagReceiver:
    def __init__(self, host, port, queue: Queue):
        self.host = host; self.port = port; self.queue = queue
        self.running = False

    def start(self):
        self.running = True
        threading.Thread(target=self._run, daemon=True).start()

    def stop(self):
        self.running = False

    def _run(self):
        import time
        while self.running:
            try:
                self.queue.put({"event": "connecting"})
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5)
                sock.connect((self.host, self.port))
                sock.settimeout(None)
                self.queue.put({"event": "connected"})
                buf = ""
                while self.running:
                    data = sock.recv(4096).decode("utf-8", errors="replace")
                    if not data: break
                    buf += data
                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        line = line.strip()
                        if not line: continue
                        try:
                            self.queue.put(json.loads(line))
                        except Exception:
                            pass
            except Exception as e:
                self.queue.put({"event": "disconnected", "msg": str(e)})
            if self.running:
                time.sleep(3)


# ═══════════════════════════════════════════════════════════════════════════════
#  ANCHOR STATUS CARD  (widget)
# ═══════════════════════════════════════════════════════════════════════════════
class AnchorCard(QtWidgets.QFrame):
    def __init__(self, anchor_id: str, parent=None):
        super().__init__(parent)
        self.anchor_id = anchor_id
        self._build()

    def _build(self):
        self.setFixedHeight(80)
        self.setStyleSheet("""
            QFrame {
                background: #0d1117;
                border: 1px solid #21262d;
                border-radius: 6px;
            }
        """)
        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(10, 7, 10, 7)
        root.setSpacing(4)

        # Header row
        hdr = QtWidgets.QHBoxLayout()
        self._id_lbl = QtWidgets.QLabel(f"A{self.anchor_id}")
        self._id_lbl.setStyleSheet(
            "color:#58a6ff;font-weight:bold;font-size:12px;"
            "font-family:Consolas;border:none;background:transparent;")
        self._status_lbl = QtWidgets.QLabel("LOS ✔")
        self._status_lbl.setStyleSheet(
            "color:#3fb950;font-size:11px;font-family:Consolas;"
            "border:none;background:transparent;")
        hdr.addWidget(self._id_lbl)
        hdr.addStretch()
        hdr.addWidget(self._status_lbl)
        root.addLayout(hdr)

        # Progress bar
        self._bar = QtWidgets.QProgressBar()
        self._bar.setRange(0, 100)
        self._bar.setValue(0)
        self._bar.setTextVisible(False)
        self._bar.setFixedHeight(6)
        self._bar.setStyleSheet(self._bar_css(False))
        root.addWidget(self._bar)

        # Dist + prob
        self._info_lbl = QtWidgets.QLabel("dist: —  |  p(NLOS): —")
        self._info_lbl.setStyleSheet(
            "color:#484f58;font-size:10px;font-family:Consolas;"
            "border:none;background:transparent;")
        root.addWidget(self._info_lbl)

    def _bar_css(self, nlos: bool) -> str:
        color = "#f85149" if nlos else "#3fb950"
        return (f"QProgressBar{{background:#21262d;border-radius:3px;border:none;}}"
                f"QProgressBar::chunk{{background:{color};border-radius:3px;}}")

    def update_data(self, los: int, prob_nlos: float, dist: float):
        nlos = (los == 0)
        self._bar.setValue(int(prob_nlos * 100))
        self._bar.setStyleSheet(self._bar_css(nlos))
        if nlos:
            self._status_lbl.setText("NLOS ✖")
            self._status_lbl.setStyleSheet(
                "color:#f85149;font-size:11px;font-family:Consolas;"
                "border:none;background:transparent;")
            self.setStyleSheet("""
                QFrame { background:#0d1117; border:1px solid #f8514944;
                          border-radius:6px; }
            """)
        else:
            self._status_lbl.setText("LOS  ✔")
            self._status_lbl.setStyleSheet(
                "color:#3fb950;font-size:11px;font-family:Consolas;"
                "border:none;background:transparent;")
            self.setStyleSheet("""
                QFrame { background:#0d1117; border:1px solid #21262d;
                          border-radius:6px; }
            """)
        self._info_lbl.setText(f"dist: {dist:.0f} mm  |  p(NLOS): {prob_nlos:.2f}")


# ═══════════════════════════════════════════════════════════════════════════════
#  MAIN WINDOW
# ═══════════════════════════════════════════════════════════════════════════════
class UWBWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(f"UWB AI Tracker — {TAG_ID}")
        self.setMinimumSize(1200, 760)

        self.queue    = Queue()
        self.receiver = TagReceiver(TAG_SOURCE["host"], TAG_SOURCE["port"], self.queue)
        self.trail    = deque(maxlen=TRAIL_LEN)
        self._pt_cnt  = 0
        self._nlos_history = deque(maxlen=200)  # list of [0/1] per anchor
        self._nlos_circles = []  # pyqtgraph items

        self._build_ui()
        self.receiver.start()

        self._timer = QtCore.QTimer()
        self._timer.timeout.connect(self._poll)
        self._timer.start(16)

    # ── Build UI ──────────────────────────────────────────────────────────────
    def _build_ui(self):
        pg.setConfigOptions(antialias=True)
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QHBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # ── Map ────────────────────────────────────────────────────────────
        self.plot = pg.PlotWidget()
        self.plot.setBackground("#0d1117")
        self.plot.showGrid(x=True, y=True, alpha=0.10)
        self.plot.setLabel("left",   "Y (mm)", color="#8b949e")
        self.plot.setLabel("bottom", "X (mm)", color="#8b949e")
        self.plot.getAxis("left").setPen(pg.mkPen("#21262d"))
        self.plot.getAxis("bottom").setPen(pg.mkPen("#21262d"))
        for ax in (self.plot.getAxis("left"), self.plot.getAxis("bottom")):
            ax.setTextPen(pg.mkPen("#484f58"))
        self.plot.setXRange(-1000, 16000, padding=0)
        self.plot.setYRange(-1000, 5500,  padding=0)
        self.plot.setAspectLocked(True)
        self.plot.setMouseEnabled(x=True, y=True)

        # Room boundary
        bx = [0, 0, 14800, 14800, 0]
        by = [0, 4400, 4400, 0, 0]
        self.plot.addItem(pg.PlotCurveItem(
            x=bx, y=by,
            pen=pg.mkPen("#30363d", width=1.5,
                         style=QtCore.Qt.DashLine)))

        # Anchors
        for i, (x, y) in enumerate(ANCHORS):
            self.plot.addItem(pg.ScatterPlotItem(
                x=[x], y=[y], size=18, symbol="s",
                pen=pg.mkPen("#1f6feb", width=2),
                brush=pg.mkBrush(31, 111, 235, 60)))
            lbl = pg.TextItem(f"  A{ANCHOR_IDS[i]}\n  ({x},{y})",
                              color="#58a6ff", anchor=(0, 1))
            lbl.setFont(QtGui.QFont("Consolas", 8))
            lbl.setPos(x, y)
            self.plot.addItem(lbl)

        # NLOS distance circles (one per anchor, initially hidden)
        for _ in range(4):
            c = pg.PlotCurveItem(
                pen=pg.mkPen("#f85149", width=1,
                             style=QtCore.Qt.DotLine))
            self.plot.addItem(c)
            self._nlos_circles.append(c)

        # Trail
        r, g, b = TAG_COLOR
        self._trail_item = pg.PlotCurveItem(
            pen=pg.mkPen((r,g,b,160), width=2), antialias=True)
        self._dot_item = pg.ScatterPlotItem(
            size=22, symbol="o",
            pen=pg.mkPen((r,g,b), width=2),
            brush=pg.mkBrush(r, g, b, 210))
        self._lbl_item = pg.TextItem("", color=(r,g,b), anchor=(0.5, 2.0),
                                     fill=pg.mkBrush(0,0,0,140))
        self._lbl_item.setFont(QtGui.QFont("Consolas", 9, QtGui.QFont.Bold))
        self.plot.addItem(self._trail_item)
        self.plot.addItem(self._dot_item)
        self.plot.addItem(self._lbl_item)

        root.addWidget(self.plot, stretch=1)

        # ── Sidebar ────────────────────────────────────────────────────────
        sb = QtWidgets.QWidget()
        sb.setFixedWidth(230)
        sb.setStyleSheet("background:#161b22;")
        vb = QtWidgets.QVBoxLayout(sb)
        vb.setContentsMargins(12, 14, 12, 12)
        vb.setSpacing(10)

        # Connection card
        conn_card = QtWidgets.QFrame()
        conn_card.setStyleSheet(
            f"QFrame{{border:1px solid #{r:02x}{g:02x}{b:02x}55;"
            "border-radius:6px;background:#0d1117;}}")
        cc = QtWidgets.QVBoxLayout(conn_card)
        cc.setContentsMargins(10, 8, 10, 8); cc.setSpacing(4)
        row = QtWidgets.QHBoxLayout()
        name_lbl = QtWidgets.QLabel(TAG_ID)
        name_lbl.setStyleSheet(
            f"color:rgb({r},{g},{b});font-weight:bold;font-size:13px;"
            "font-family:Consolas;border:none;background:transparent;")
        self._status_dot = QtWidgets.QLabel("●")
        self._status_dot.setStyleSheet(
            "color:#484f58;font-size:13px;border:none;background:transparent;")
        row.addWidget(name_lbl); row.addStretch(); row.addWidget(self._status_dot)
        cc.addLayout(row)
        ip_lbl = QtWidgets.QLabel(f"{TAG_SOURCE['host']}:{TAG_SOURCE['port']}")
        ip_lbl.setStyleSheet(
            "color:#484f58;font-size:10px;font-family:Consolas;"
            "border:none;background:transparent;")
        cc.addWidget(ip_lbl)
        self._coord_lbl = QtWidgets.QLabel("X: —\nY: —")
        self._coord_lbl.setStyleSheet(
            "color:#8b949e;font-size:11px;font-family:Consolas;"
            "border:none;background:transparent;")
        cc.addWidget(self._coord_lbl)
        vb.addWidget(conn_card)

        # Section: AI LOS/NLOS Monitor
        vb.addWidget(self._sec("🤖  AI LOS/NLOS"))
        self._anchor_cards: dict[str, AnchorCard] = {}
        for aid in ANCHOR_IDS:
            card = AnchorCard(aid)
            vb.addWidget(card)
            self._anchor_cards[aid] = card

        vb.addWidget(self._sep())

        # NLOS stats
        vb.addWidget(self._sec("NLOS RATE  (last 200)"))
        self._nlos_stat_lbl = QtWidgets.QLabel("")
        self._nlos_stat_lbl.setStyleSheet(
            "color:#8b949e;font-size:10px;font-family:Consolas;"
            "background:transparent;")
        vb.addWidget(self._nlos_stat_lbl)

        vb.addWidget(self._sep())
        vb.addWidget(self._sec("CONTROLS"))

        btn_clear = QtWidgets.QPushButton("Clear Trail")
        btn_clear.clicked.connect(self._clear_trail)
        btn_clear.setStyleSheet(self._bstyle())
        vb.addWidget(btn_clear)

        vb.addWidget(self._sep())
        self._stats_lbl = QtWidgets.QLabel("")
        self._stats_lbl.setStyleSheet(
            "color:#484f58;font-size:10px;font-family:Consolas;"
            "background:transparent;")
        vb.addWidget(self._stats_lbl)
        vb.addStretch()

        root.addWidget(sb)
        self.setStyleSheet("QMainWindow{background:#0d1117;}")

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _sec(self, t):
        l = QtWidgets.QLabel(t)
        l.setStyleSheet(
            "color:#484f58;font-size:10px;font-family:Consolas;"
            "letter-spacing:1px;background:transparent;")
        return l

    def _sep(self):
        f = QtWidgets.QFrame()
        f.setFrameShape(QtWidgets.QFrame.HLine)
        f.setStyleSheet("color:#21262d;"); return f

    def _bstyle(self):
        return ("QPushButton{background:#21262d;color:#8b949e;"
                "border:1px solid #30363d;border-radius:4px;padding:5px;"
                "font-family:Consolas;font-size:11px;}"
                "QPushButton:hover{background:#30363d;color:#c9d1d9;}")

    def _clear_trail(self):
        self.trail.clear()
        self._trail_item.setData(x=[], y=[])

    # ── NLOS circles ──────────────────────────────────────────────────────────
    def _update_nlos_circles(self, anchors):
        for i, a in enumerate(anchors):
            if a["los"] == 0:  # NLOS
                ax, ay = ANCHORS[i]
                r = a["dist"]
                theta = np.linspace(0, 2*np.pi, 80)
                self._nlos_circles[i].setData(
                    x=ax + r*np.cos(theta),
                    y=ay + r*np.sin(theta))
            else:
                self._nlos_circles[i].setData(x=[], y=[])

    # ── Poll queue ────────────────────────────────────────────────────────────
    def _poll(self):
        updated = False

        while True:
            try:
                msg = self.queue.get_nowait()
            except Empty:
                break

            # Connection events
            if "event" in msg:
                ev = msg["event"]
                if ev == "connected":
                    self._status_dot.setStyleSheet(
                        "color:#3fb950;font-size:13px;border:none;background:transparent;")
                elif ev == "connecting":
                    self._status_dot.setStyleSheet(
                        "color:#f0883e;font-size:13px;border:none;background:transparent;")
                elif ev == "disconnected":
                    self._status_dot.setStyleSheet(
                        "color:#f85149;font-size:13px;border:none;background:transparent;")
                continue

            # Position + anchor data
            x = msg.get("x"); y = msg.get("y")
            anchors = msg.get("anchors", [])
            if x is None or y is None: continue

            self.trail.append((x, y))
            self._pt_cnt += 1
            updated = True
            self._coord_lbl.setText(f"X: {x:7.0f} mm\nY: {y:7.0f} mm")

            # Update anchor cards
            nlos_row = []
            for a in anchors:
                aid = a.get("id")
                if aid in self._anchor_cards:
                    self._anchor_cards[aid].update_data(
                        los      = a.get("los", 1),
                        prob_nlos= a.get("prob_nlos", 0.0),
                        dist     = a.get("dist", 0.0)
                    )
                nlos_row.append(0 if a.get("los", 1) == 0 else 0)

            # Track NLOS history
            if len(anchors) == 4:
                self._nlos_history.append([
                    1 if a.get("los", 1) == 0 else 0
                    for a in anchors
                ])
                self._update_nlos_circles(anchors)

        if updated and self.trail:
            trail = list(self.trail)
            tx = [p[0] for p in trail]
            ty = [p[1] for p in trail]
            self._trail_item.setData(x=tx, y=ty)
            self._dot_item.setData(x=[tx[-1]], y=[ty[-1]])
            self._lbl_item.setText(f"{TAG_ID}\n({tx[-1]:.0f}, {ty[-1]:.0f})")
            self._lbl_item.setPos(tx[-1], ty[-1])

        # NLOS rate stats
        if self._nlos_history:
            n = len(self._nlos_history)
            rates = [sum(h[i] for h in self._nlos_history)/n*100
                     for i in range(4)]
            self._nlos_stat_lbl.setText(
                "\n".join(f"  A{ANCHOR_IDS[i]}: {rates[i]:.0f}%"
                          for i in range(4)))

        self._stats_lbl.setText(f"Points: {self._pt_cnt}")

    def closeEvent(self, event):
        self.receiver.stop()
        event.accept()


# ══════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    pg.setConfigOption("background", "#0d1117")
    pg.setConfigOption("foreground", "#8b949e")

    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    pal = QtGui.QPalette()
    pal.setColor(QtGui.QPalette.Window,     QtGui.QColor(13,  17,  23))
    pal.setColor(QtGui.QPalette.WindowText, QtGui.QColor(201, 209, 217))
    pal.setColor(QtGui.QPalette.Base,       QtGui.QColor(22,  27,  34))
    pal.setColor(QtGui.QPalette.Button,     QtGui.QColor(33,  38,  45))
    pal.setColor(QtGui.QPalette.ButtonText, QtGui.QColor(201, 209, 217))
    app.setPalette(pal)

    win = UWBWindow()
    win.show()
    sys.exit(app.exec_())
