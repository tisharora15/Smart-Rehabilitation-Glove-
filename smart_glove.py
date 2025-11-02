
import sys, json, time, sqlite3
from collections import deque
import threading
from queue import Queue
import paho.mqtt.client as mqtt
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
from pyqtgraph import DateAxisItem
from datetime import datetime
import numpy as np
import os

MQTT_BROKER = 'broker.emqx.io'         # EMQX public broker
MQTT_PORT = 1883
TOPIC = 'glove/glove_01/sensors'       # topic to subscribe to
MAX_POINTS = 2000                      # rolling buffer length

DB_PATH = 'glove_readings.db'          # SQLite file (created in current folder)


data_lock = threading.Lock()
db_queue = Queue(maxsize=1000)  # queue of items to write to DB


def init_db():
  
    try:
        conn = sqlite3.connect(DB_PATH)
        c = conn.cursor()
        c.execute('''
            CREATE TABLE IF NOT EXISTS readings (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                ts INTEGER NOT NULL,
                device TEXT,
                topic TEXT,
                payload TEXT
            )
        ''')
        conn.commit()
    except Exception as e:
        print("init_db error:", e)
    finally:
        try: conn.close()
        except: pass

def db_writer_thread():
    
    try:
        conn = sqlite3.connect(DB_PATH, timeout=5.0, check_same_thread=False)
        try:
            # improve concurrency for readers/writer
            conn.execute('PRAGMA journal_mode=WAL;')
        except Exception:
            pass
        c = conn.cursor()
        while True:
            item = db_queue.get()  # blocks
            if item is None:
                break
            ts_ms, device, topic, payload_raw = item
            try:
                c.execute('INSERT INTO readings (ts, device, topic, payload) VALUES (?,?,?,?)',
                          (int(ts_ms), device, topic, payload_raw))
                conn.commit()
            except Exception as e:
                # log and continue
                print("DB write error:", e)
        conn.close()
    except Exception as e:
        print("DB writer fatal error:", e)

class DataSource:
    def __init__(self):
        self.ts = deque(maxlen=MAX_POINTS)
        self.flex = deque(maxlen=MAX_POINTS)
        self.fsr = deque(maxlen=MAX_POINTS)
        self.accel = deque(maxlen=MAX_POINTS)
        self.yaw = deque(maxlen=MAX_POINTS)
        self.latest_ts = None
        self.latest_flex = None
        self.latest_fsr = None
        self.latest_accel = None
        self.latest_yaw = None

data = DataSource()


def on_connect(client, userdata, flags, rc):
    print("MQTT connected rc=", rc)
    if rc == 0:
        client.subscribe(TOPIC)
        print(f"Subscribed to {TOPIC}")
    else:
        print("Connection failed with rc=", rc)

def on_disconnect(client, userdata, rc):
    print("MQTT disconnected rc=", rc)

def on_message(client, userdata, msg):
    payload_raw = msg.payload.decode('utf-8', errors='ignore')
    try:
        j = json.loads(payload_raw)
    except:
        j = None

    now = time.time()

    if j:
        flex = j.get('flex_raw', j.get('flex', None))
        fsr = j.get('fsr_raw', j.get('fsr', None))
        accel = j.get('accel_mag', None)
        yaw = j.get('yaw_rate_dps', j.get('yaw', None))
        device = j.get('device_id', 'unknown')
    else:
        flex = None; fsr = None; accel = None; yaw = None
        device = 'unknown'

    # thread-safe update of in-memory buffers
    with data_lock:
        data.ts.append(now)
        data.flex.append(flex)
        data.fsr.append(fsr)
        data.accel.append(accel)
        data.yaw.append(yaw)
        data.latest_ts = now
        data.latest_flex = flex
        data.latest_fsr = fsr
        data.latest_accel = accel
        data.latest_yaw = yaw

    # enqueue DB write (non-blocking if queue has space)
    ts_ms = int(now * 1000)
    try:
        db_queue.put_nowait((ts_ms, device, msg.topic, payload_raw))
    except Exception:
        # if DB queue full, skip write (don't block MQTT thread)
        print("DB queue full ? skipping DB write for this message")

def mqtt_thread():
    
    client_id = f"pi-subscriber-{int(time.time()) % 100000}"
    client = mqtt.Client(client_id=client_id, clean_session=True)
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_disconnect = on_disconnect

    # Set a will message (optional) to signal device offline
    # client.will_set("glove/status", payload=f"{client_id} offline", qos=0, retain=False)

    while True:
        try:
            print(f"Attempting MQTT connect to {MQTT_BROKER}:{MQTT_PORT} ...")
            client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
            # Start network loop in this thread and block here until connection lost
            client.loop_forever(retry_first_connection=False)
        except Exception as e:
            print("MQTT connect/loop error:", e)
            # backoff before retrying
            time.sleep(5)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Smart Glove Live ? PyQtGraph")
        self.resize(1000, 700)

        cw = QtWidgets.QWidget()
        self.setCentralWidget(cw)
        vlayout = QtWidgets.QVBoxLayout()
        cw.setLayout(vlayout)

        # Readout panel
        readout_layout = QtWidgets.QHBoxLayout()
        vlayout.addLayout(readout_layout)
        self.time_label = QtWidgets.QLabel("Time: --:--:--"); self.time_label.setStyleSheet("font: 14pt;")
        readout_layout.addWidget(self.time_label); readout_layout.addSpacing(20)

        def make_readout(name, color):
            w = QtWidgets.QWidget(); h = QtWidgets.QHBoxLayout(); w.setLayout(h)
            box = QtWidgets.QLabel(); box.setFixedSize(16,16); box.setStyleSheet(f"background-color: {color}; border: 1px solid #000;")
            label = QtWidgets.QLabel(f"{name}: --"); label.setStyleSheet("font: 12pt;")
            h.addWidget(box); h.addSpacing(6); h.addWidget(label)
            return w, label

        self.flex_widget, self.flex_label = make_readout("Flex", "#f1c40f")
        self.fsr_widget, self.fsr_label   = make_readout("FSR",  "#e74c3c")
        self.acc_widget, self.acc_label   = make_readout("Accel", "#1abc9c")
        self.yaw_widget, self.yaw_label   = make_readout("Yaw",  "#9b59b6")
        readout_layout.addWidget(self.flex_widget); readout_layout.addWidget(self.fsr_widget)
        readout_layout.addWidget(self.acc_widget); readout_layout.addWidget(self.yaw_widget)
        readout_layout.addStretch()

        # --- separate DateAxisItem for each plot ---
        date_axis1 = DateAxisItem(orientation='bottom')
        date_axis2 = DateAxisItem(orientation='bottom')

        self.plotw1 = pg.PlotWidget(axisItems={'bottom': date_axis1}, title="Flex & FSR")
        self.plotw2 = pg.PlotWidget(axisItems={'bottom': date_axis2}, title="Accel & Yaw")
        self.plotw1.showGrid(x=True, y=True, alpha=0.3); self.plotw2.showGrid(x=True, y=True, alpha=0.3)
        self.plotw1.setLabel('left', 'Analog'); self.plotw2.setLabel('left', 'IMU')
        vlayout.addWidget(self.plotw1, 2); vlayout.addWidget(self.plotw2, 2)

        # Curves and legend
        self.curve_flex = self.plotw1.plot(pen=pg.mkPen('#f1c40f', width=2), name='Flex')
        self.curve_fsr  = self.plotw1.plot(pen=pg.mkPen('#e74c3c', width=2), name='FSR')
        self.curve_acc  = self.plotw2.plot(pen=pg.mkPen('#1abc9c', width=2), name='Accel')
        self.curve_yaw  = self.plotw2.plot(pen=pg.mkPen('#9b59b6', width=2), name='Yaw')
        self.legend1 = self.plotw1.addLegend(offset=(10,10)); self.legend2 = self.plotw2.addLegend(offset=(10,10))
        self.legend1.addItem(self.curve_flex, 'Flex'); self.legend1.addItem(self.curve_fsr, 'FSR')
        self.legend2.addItem(self.curve_acc, 'Accel'); self.legend2.addItem(self.curve_yaw, 'Yaw')

        # Crosshair and mouse
        self.vLine = pg.InfiniteLine(angle=90, movable=False, pen=pg.mkPen('w', style=QtCore.Qt.DashLine))
        self.hLine = pg.InfiniteLine(angle=0, movable=False, pen=pg.mkPen('w', style=QtCore.Qt.DashLine))
        self.plotw1.addItem(self.vLine, ignoreBounds=True); self.plotw1.addItem(self.hLine, ignoreBounds=True)
        self.plotw1.scene().sigMouseMoved.connect(self.onMouseMoved)

        # Timer
        self.timer = QtCore.QTimer(); self.timer.timeout.connect(self.update); self.timer.start(100)

    def update(self):
        # Copy current buffers under lock to ensure consistent snapshot
        with data_lock:
            if len(data.ts) == 0:
                return
            x_raw = list(data.ts)
            flex_raw = list(data.flex)
            fsr_raw = list(data.fsr)
            acc_raw = list(data.accel)
            yaw_raw = list(data.yaw)
            latest_ts = data.latest_ts
            latest_f = data.latest_flex
            latest_p = data.latest_fsr
            latest_a = data.latest_accel
            latest_y = data.latest_yaw

        # Convert timestamp (seconds) to floats acceptable to DateAxisItem (unix time)
        # pyqtgraph DateAxisItem expects floats (seconds since epoch); we already use time.time()
        x = np.array(x_raw, dtype=float)

        # Replace None with nan so numpy.isfinite works and pyqtgraph won't choke
        def to_numpy_list(lst):
            return np.array([float(v) if (v is not None) else np.nan for v in lst], dtype=float)

        try:
            flex = to_numpy_list(flex_raw)
            fsr = to_numpy_list(fsr_raw)
            acc = to_numpy_list(acc_raw)
            yaw = to_numpy_list(yaw_raw)
        except Exception as e:
            # If conversion fails, skip update to avoid crashing the UI
            print("Data conversion error:", e)
            return

        # Update curves
        try:
            self.curve_flex.setData(x, flex)
            self.curve_fsr.setData(x, fsr)
            self.curve_acc.setData(x, acc)
            self.curve_yaw.setData(x, yaw)
            self.plotw1.enableAutoRange('y', True)
            self.plotw2.enableAutoRange('y', True)
        except Exception as e:
            # guard against plotting errors
            print("Plot update error:", e)

        # update readouts
        ts = latest_ts
        ts_str = datetime.fromtimestamp(ts).strftime('%H:%M:%S') if ts else "--:--:--"
        self.time_label.setText(f"Time: {ts_str}")
        def fmt(v): return f"{v:.2f}" if (v is not None and not (isinstance(v, float) and np.isnan(v))) else "--"
        self.flex_label.setText(f"Flex: {fmt(latest_f)}")
        self.fsr_label.setText( f"FSR:  {fmt(latest_p)}")
        self.acc_label.setText( f"Accel:{fmt(latest_a)}")
        self.yaw_label.setText( f"Yaw:  {fmt(latest_y)}")

    def onMouseMoved(self, evt):
        pos = evt
        vb = self.plotw1.getViewBox()
        if self.plotw1.sceneBoundingRect().contains(pos):
            mousePoint = vb.mapSceneToView(pos)
            x = mousePoint.x(); y = mousePoint.y()
            self.vLine.setPos(x); self.hLine.setPos(y)

# ---------------- main ----------------
if __name__ == '__main__':
    init_db()

    # start DB writer thread
    t_db = threading.Thread(target=db_writer_thread, daemon=True)
    t_db.start()

    # start mqtt thread
    t_mqtt = threading.Thread(target=mqtt_thread, daemon=True)
    t_mqtt.start()

    # start GUI (main thread)
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow(); w.show()
    try:
        sys.exit(app.exec_())
    finally:
        # cleanup: stop DB writer gracefully
        try:
            db_queue.put_nowait(None)
        except:
            pass
