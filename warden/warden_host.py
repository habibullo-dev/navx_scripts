#!/usr/bin/env python3
"""
WARDEN host - integrated inspection + monitoring + dashboard.

Runs on the Raspberry Pi (the robot's brain). It:
  * talks to the ESP32 sensor hub over serial (ENV / THERMAL / RF / SERVO / LED / STATUS)
  * reads the C920 camera
  * ALWAYS-ON in the background: polls environment + thermal + RF, and runs YOLO object
    detection on the camera (the "room audit")
  * ON DEMAND: runs an optical hidden-camera inspection sweep (servo tilt + LED flash +
    differential glint detection + multi-angle persistence), triggered by the dashboard
    SCAN button or the X button on the ROS /joy gamepad
  * fuses optical + thermal + RF into a confidence score
  * serves a live web dashboard at http://<pi-ip>:8000

THIS IS A FIRST INTEGRATION VERSION. It has not been run on your hardware. Read CONFIG,
install deps, and bring it up in stages. The camera-exposure handling and the optical
thresholds are the parts most likely to need tuning - same as on the bench.
"""

import time
import json
import threading
from dataclasses import dataclass, field, asdict

import serial                 # pip install pyserial
import cv2                     # sudo apt install python3-opencv   (easier on a Pi)
import numpy as np
from flask import Flask, jsonify, render_template_string  # pip install flask

# Optional / heavy - guarded so the app still runs without them
try:
    from ultralytics import YOLO          # pip install ultralytics  (pulls torch - heavy on a Pi)
    _HAVE_YOLO = True
except Exception:
    _HAVE_YOLO = False

try:
    import pygame                         # pip install pygame  (only for USE_JOYSTICK pygame path)
    _HAVE_PYGAME = True
except Exception:
    _HAVE_PYGAME = False


# ============================== CONFIG ==============================
class CFG:
    # --- ESP32 serial ---
    SERIAL_PORT = "/dev/ttyUSB0"   # check on the Pi with: ls /dev/ttyUSB*
    BAUD = 115200

    # --- camera ---
    CAMERA_INDEX = 0               # C920 on the Pi - try 0, then 1, 2...
    FRAME_W = 1280
    FRAME_H = 720

    # --- web dashboard ---
    SERVER_HOST = "0.0.0.0"        # 0.0.0.0 = reachable from your laptop's browser
    SERVER_PORT = 8000

    # --- always-on background sensors ---
    SENSOR_PERIOD = 2.0            # seconds between ENV/THERMAL/RF polls

    # --- YOLO object detection (AI room audit) ---
    # NOTE: this reads the camera CONTINUOUSLY. Set USE_YOLO = False if you want the
    # camera idle except during an X-triggered inspection sweep.
    USE_YOLO = True                # set False if the Pi is too slow / want camera idle
    YOLO_MODEL = "yolov8n.pt"      # 'n' = nano, lightest; downloads on first run
    YOLO_PERIOD = 1.5              # seconds between detections (Pi is slow)
    YOLO_CONF = 0.45

    # --- joystick trigger ---
    # You already run a ROS joy_node for driving. Trigger the scan from the ROS /joy
    # topic, NOT pygame - two readers on one gamepad device conflict. Leave the pygame
    # path (USE_JOYSTICK) OFF while ROS joy_node is running.
    USE_JOYSTICK = False           # pygame path: leave OFF while ROS joy_node runs
    USE_ROS_JOY = True             # subscribe to ROS /joy for the X-button trigger
    JOY_TOPIC = "/joy"
    JOY_SCAN_BUTTON = 2            # X index - VERIFY with: ros2 topic echo /joy

    # --- optical inspection sweep ---
    SERVO_ANGLES = [30, 60, 90, 120, 150]   # tilt steps the head sweeps through
    SERVO_SETTLE = 0.4             # seconds to let the head stop shaking
    LED_WARMUP = 0.15              # seconds after LED ON before the photo
    LOCK_EXPOSURE = True           # lock exposure so OFF/ON frames are comparable
    MANUAL_EXPOSURE_MODE = 0.25    # V4L2 "manual" - some cams want 1 instead (tune it)
    DIFF_THRESH = 60               # how much brighter a pixel must get to count as a glint
    STRONG_BONUS = 20              # a glint this much above DIFF_THRESH is a "strong" hit
    MIN_AREA = 4
    MAX_AREA = 600

    # --- layer fusion weights (optical is primary) ---
    W_OPTICAL = 0.70
    W_THERMAL = 0.15
    W_RF = 0.15
    SUSPICIOUS_AT = 0.50           # confidence >= this -> flag as suspicious


# ============================== SHARED STATE ==============================
@dataclass
class State:
    status: str = "idle"                         # idle | scanning
    env: dict = field(default_factory=dict)      # {t,h,p,gas}
    thermal: dict = field(default_factory=dict)  # {min,max,mean,grid}
    rf: dict = field(default_factory=dict)       # {n,strong,top}
    objects: list = field(default_factory=list)  # YOLO: [{label,count}]
    last_inspection: dict = field(default_factory=dict)
    updated: float = 0.0


STATE = State()
STATE_LOCK = threading.Lock()      # protects STATE
SERIAL_LOCK = threading.Lock()     # only one thread talks to the ESP32 at a time
CAMERA_LOCK = threading.Lock()     # only one thread reads the camera at a time
SCAN_REQUEST = threading.Event()   # set this to ask for an inspection


# ============================== ESP32 ==============================
class ESP32:
    """Thin wrapper over the serial link to the sensor hub."""

    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=2)
        time.sleep(2.0)                # ESP32 reboots when the port opens - wait it out
        self.ser.reset_input_buffer()

    def cmd(self, text, expect_json=False):
        with SERIAL_LOCK:
            self.ser.reset_input_buffer()
            self.ser.write((text + "\n").encode())
            line = self.ser.readline().decode(errors="ignore").strip()
        if expect_json:
            try:
                return json.loads(line)
            except Exception:
                return {}
        return line


# ============================== ALWAYS-ON SENSORS ==============================
def sensor_loop(esp):
    """Poll environment, thermal and RF forever, in the background."""
    while True:
        try:
            env = esp.cmd("ENV", expect_json=True)
            thermal = esp.cmd("THERMAL", expect_json=True)
            rf = esp.cmd("RF", expect_json=True)
            with STATE_LOCK:
                if env:
                    STATE.env = env
                if thermal:
                    STATE.thermal = thermal
                if rf:
                    STATE.rf = rf
                STATE.updated = time.time()
        except Exception as ex:
            print("[sensors] error:", ex)
        time.sleep(CFG.SENSOR_PERIOD)


# ============================== YOLO (room audit) ==============================
def yolo_loop(cap):
    """Continuously identify objects in view (towels, clutter, etc.)."""
    if not CFG.USE_YOLO:
        return
    if not _HAVE_YOLO:
        print("[yolo] ultralytics not installed - object detection disabled")
        return
    model = YOLO(CFG.YOLO_MODEL)
    print("[yolo] model loaded")
    while True:
        # don't fight the inspection for the camera
        if STATE.status == "scanning":
            time.sleep(0.3)
            continue
        with CAMERA_LOCK:
            ok, frame = cap.read()
        if not ok:
            time.sleep(0.5)
            continue
        try:
            res = model(frame, conf=CFG.YOLO_CONF, verbose=False)[0]
            counts = {}
            for b in res.boxes:
                label = model.names[int(b.cls)]
                counts[label] = counts.get(label, 0) + 1
            with STATE_LOCK:
                STATE.objects = [{"label": k, "count": v} for k, v in sorted(counts.items())]
        except Exception as ex:
            print("[yolo] error:", ex)
        time.sleep(CFG.YOLO_PERIOD)


# ============================== OPTICAL DETECTION ==============================
def find_glints(off, on):
    """Differential retroreflection: pixels that brighten a lot when the LED turns on."""
    g_off = cv2.cvtColor(off, cv2.COLOR_BGR2GRAY)
    g_on = cv2.cvtColor(on, cv2.COLOR_BGR2GRAY)
    diff = cv2.subtract(g_on, g_off)
    _, mask = cv2.threshold(diff, CFG.DIFF_THRESH, 255, cv2.THRESH_BINARY)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    glints = []
    for c in cnts:
        area = cv2.contourArea(c)
        if CFG.MIN_AREA <= area <= CFG.MAX_AREA:
            m = cv2.moments(c)
            if m["m00"] > 0:
                cx, cy = int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"])
                glints.append({"x": cx, "y": cy, "area": float(area),
                               "bright": int(diff[cy, cx])})
    return glints


def capture_pair(esp, cap):
    """Grab one LED-OFF and one LED-ON frame with matched exposure."""
    if CFG.LOCK_EXPOSURE:
        # camera/driver specific - if your OFF/ON frames look auto-adjusted, tune this
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, CFG.MANUAL_EXPOSURE_MODE)

    esp.cmd("LED OFF")
    time.sleep(0.2)
    for _ in range(3):            # flush buffered frames so we get a fresh one
        cap.read()
    ok1, off = cap.read()

    esp.cmd("LED ON")
    time.sleep(CFG.LED_WARMUP)
    for _ in range(2):
        cap.read()
    ok2, on = cap.read()

    esp.cmd("LED OFF")
    return (off if ok1 else None), (on if ok2 else None)


def run_inspection(esp, cap):
    """One full sweep: tilt through angles, flash + capture, score, fuse."""
    with STATE_LOCK:
        STATE.status = "scanning"
    esp.cmd("STATUS 0 0 255")     # blue = scanning

    per_angle = []
    with CAMERA_LOCK:             # hold the camera for the whole sweep
        for ang in CFG.SERVO_ANGLES:
            esp.cmd(f"SERVO {ang}")
            time.sleep(CFG.SERVO_SETTLE)
            off, on = capture_pair(esp, cap)
            glints = find_glints(off, on) if (off is not None and on is not None) else []
            strong = [g for g in glints if g["bright"] >= CFG.DIFF_THRESH + CFG.STRONG_BONUS]
            per_angle.append({
                "angle": ang,
                "glints": len(glints),
                "strong": len(strong),
                "max_bright": max((g["bright"] for g in glints), default=0),
            })
        esp.cmd("SERVO 90")       # recentre the head

    # --- persistence (v1 heuristic) ---
    # A real lens, lit coaxially, glints at MOST tilt angles; a mirror/screw flashes at
    # one angle only. So: optical confidence = fraction of angles that showed a strong
    # glint. NOTE: a stronger version would track the same WORLD point across angles
    # (compensating for the image shift as the head tilts) - that's the next refinement.
    angles_with_strong = sum(1 for a in per_angle if a["strong"] > 0)
    optical_score = min(1.0, angles_with_strong / max(1, len(CFG.SERVO_ANGLES)))

    # --- corroboration layers ---
    with STATE_LOCK:
        thermal, rf = STATE.thermal, STATE.rf
    thermal_hot = 1.0 if thermal and (thermal.get("max", 0) - thermal.get("mean", 0) > 3) else 0.0
    rf_present = 1.0 if rf and rf.get("strong", 0) > 0 else 0.0

    confidence = (CFG.W_OPTICAL * optical_score
                  + CFG.W_THERMAL * thermal_hot
                  + CFG.W_RF * rf_present)
    verdict = "SUSPICIOUS" if confidence >= CFG.SUSPICIOUS_AT else "clear"

    result = {
        "time": time.strftime("%H:%M:%S"),
        "verdict": verdict,
        "confidence": round(confidence, 2),
        "optical_score": round(optical_score, 2),
        "angles_with_strong": angles_with_strong,
        "thermal_hot": bool(thermal_hot),
        "rf_present": bool(rf_present),
        "per_angle": per_angle,
    }
    with STATE_LOCK:
        STATE.last_inspection = result
        STATE.status = "idle"
    esp.cmd("STATUS 255 0 0" if verdict == "SUSPICIOUS" else "STATUS 0 255 0")
    print("[inspect]", verdict, "confidence", result["confidence"])
    return result


def inspection_worker(esp, cap):
    """Waits for a scan request (button), then runs one inspection."""
    while True:
        SCAN_REQUEST.wait()
        SCAN_REQUEST.clear()
        if STATE.status == "scanning":
            continue
        try:
            run_inspection(esp, cap)
        except Exception as ex:
            print("[inspect] error:", ex)
            with STATE_LOCK:
                STATE.status = "idle"


# ============================== JOYSTICK (pygame - legacy/standalone) ==============================
def joystick_loop():
    """pygame path: opens the gamepad device directly. Do NOT use this while a ROS
    joy_node is running on the same pad - use ros_joy_loop() instead."""
    if not CFG.USE_JOYSTICK:
        return
    if not _HAVE_PYGAME:
        print("[joy] pygame not installed - joystick trigger disabled")
        return
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("[joy] no joystick found")
        return
    js = pygame.joystick.Joystick(0)
    js.init()
    print("[joy] using", js.get_name())
    while True:
        for e in pygame.event.get():
            if e.type == pygame.JOYBUTTONDOWN and e.button == CFG.JOY_SCAN_BUTTON:
                print("[joy] scan button pressed")
                SCAN_REQUEST.set()
        time.sleep(0.05)


# ============================== ROS /joy TRIGGER ==============================
def ros_joy_loop():
    """Start a scan on the X button, read from the ROS /joy topic.

    Reuses the existing joy_node (the one ROS teleop already runs), so it does NOT
    open /dev/input/js0 directly and won't fight ROS for the gamepad.
    """
    if not CFG.USE_ROS_JOY:
        return
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Joy
    except Exception as ex:
        print("[joy] rclpy/sensor_msgs unavailable - ROS joy trigger off:", ex)
        return

    class JoyTrigger(Node):
        def __init__(self):
            super().__init__('warden_joy_trigger')
            self.prev = 0
            self.create_subscription(Joy, CFG.JOY_TOPIC, self.cb, 10)

        def cb(self, msg):
            b = CFG.JOY_SCAN_BUTTON
            if b < len(msg.buttons):
                cur = msg.buttons[b]
                if cur == 1 and self.prev == 0:        # rising edge only
                    print("[joy] X pressed -> scan")
                    SCAN_REQUEST.set()
                self.prev = cur

    rclpy.init()
    node = JoyTrigger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ============================== DASHBOARD ==============================
app = Flask(__name__)

DASHBOARD = """<!doctype html>
<html><head><meta charset="utf-8"><title>WARDEN</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  body{font-family:system-ui,sans-serif;background:#15161a;color:#e6e6e6;margin:0;padding:16px}
  h1{font-size:20px;margin:0 0 4px} .sub{color:#999;font-size:13px;margin-bottom:14px}
  .row{display:flex;flex-wrap:wrap;gap:12px}
  .card{background:#22242b;border:1px solid #33353d;border-radius:10px;padding:14px;min-width:210px;flex:1}
  .card h2{font-size:13px;color:#9aa;margin:0 0 8px;text-transform:uppercase;letter-spacing:.5px}
  .big{font-size:22px}
  .kv{display:flex;justify-content:space-between;font-size:14px;padding:2px 0}
  .badge{display:inline-block;padding:4px 10px;border-radius:20px;font-size:13px;font-weight:600}
  .idle{background:#2b3a4a;color:#8fc} .scanning{background:#3a3a14;color:#fe8}
  button{background:#2d6cdf;color:#fff;border:0;border-radius:8px;padding:10px 18px;font-size:15px;cursor:pointer}
  button:active{background:#1f55b8}
  .muted{color:#888;font-size:13px}
</style></head><body>
<h1>WARDEN inspection dashboard</h1>
<div class="sub">multilayer: optical (primary) + thermal + RF, with AI room audit</div>

<div style="margin-bottom:14px">
  Status: <span id="status" class="badge idle">idle</span>
  &nbsp;&nbsp;<button onclick="scan()">RUN SCAN</button>
  <span id="upd" class="muted"></span>
</div>

<div class="row">
  <div class="card"><h2>Last inspection</h2>
    <div id="verdict" class="big">--</div>
    <div class="kv"><span>confidence</span><b id="conf">--</b></div>
    <div class="kv"><span>optical</span><span id="opt">--</span></div>
    <div class="kv"><span>thermal hot</span><span id="th">--</span></div>
    <div class="kv"><span>RF present</span><span id="rf2">--</span></div>
    <div class="muted" id="itime"></div>
  </div>
  <div class="card"><h2>Environment</h2>
    <div class="kv"><span>temp</span><span id="t">--</span></div>
    <div class="kv"><span>humidity</span><span id="h">--</span></div>
    <div class="kv"><span>pressure</span><span id="p">--</span></div>
    <div class="kv"><span>air (gas)</span><span id="gas">--</span></div>
  </div>
  <div class="card"><h2>Thermal</h2>
    <div class="kv"><span>min</span><span id="tmin">--</span></div>
    <div class="kv"><span>max</span><span id="tmax">--</span></div>
    <div class="kv"><span>mean</span><span id="tmean">--</span></div>
  </div>
  <div class="card"><h2>RF (Wi-Fi)</h2>
    <div class="kv"><span>devices</span><span id="rfn">--</span></div>
    <div class="kv"><span>strong</span><span id="rfs">--</span></div>
  </div>
  <div class="card"><h2>Room audit (AI)</h2>
    <div id="objs" class="muted">--</div>
  </div>
</div>

<script>
async function scan(){ await fetch('/scan',{method:'POST'}); }
function set(id,v){ document.getElementById(id).textContent = v; }
async function tick(){
  try{
    const d = await (await fetch('/data')).json();
    const st = document.getElementById('status');
    st.textContent = d.status; st.className = 'badge ' + d.status;
    set('upd', d.updated ? '  updated ' + new Date(d.updated*1000).toLocaleTimeString() : '');

    const e = d.env||{};
    set('t', e.t!=null? e.t+' C':'--'); set('h', e.h!=null? e.h+' %':'--');
    set('p', e.p!=null? e.p+' hPa':'--'); set('gas', e.gas!=null? e.gas:'--');

    const th = d.thermal||{};
    set('tmin', th.min!=null? th.min+' C':'--'); set('tmax', th.max!=null? th.max+' C':'--');
    set('tmean', th.mean!=null? th.mean+' C':'--');

    const rf = d.rf||{};
    set('rfn', rf.n!=null? rf.n:'--'); set('rfs', rf.strong!=null? rf.strong:'--');

    const objs = d.objects||[];
    document.getElementById('objs').textContent =
      objs.length? objs.map(o=>o.label+' x'+o.count).join(', ') : 'nothing detected';

    const ins = d.last_inspection||{};
    set('verdict', ins.verdict||'--');
    set('conf', ins.confidence!=null? ins.confidence:'--');
    set('opt', ins.optical_score!=null? ins.optical_score:'--');
    set('th', ins.thermal_hot? 'yes':'no'); set('rf2', ins.rf_present? 'yes':'no');
    set('itime', ins.time? 'at '+ins.time : '');
  }catch(err){ /* ignore transient fetch errors */ }
}
setInterval(tick, 1000); tick();
</script>
</body></html>"""


@app.route("/")
def index():
    return render_template_string(DASHBOARD)


@app.route("/data")
def data():
    with STATE_LOCK:
        return jsonify(asdict(STATE))


@app.route("/scan", methods=["POST"])
def scan():
    SCAN_REQUEST.set()
    return jsonify({"ok": True})


# ============================== MAIN ==============================
def main():
    print("WARDEN host starting...")
    esp = ESP32(CFG.SERIAL_PORT, CFG.BAUD)
    print("ESP32:", esp.cmd("PING"))     # expect PONG

    cap = cv2.VideoCapture(CFG.CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CFG.FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CFG.FRAME_H)
    if not cap.isOpened():
        print("WARNING: camera did not open at index", CFG.CAMERA_INDEX)

    threading.Thread(target=sensor_loop, args=(esp,), daemon=True).start()
    threading.Thread(target=yolo_loop, args=(cap,), daemon=True).start()
    threading.Thread(target=joystick_loop, daemon=True).start()        # pygame path (off by default)
    threading.Thread(target=ros_joy_loop, daemon=True).start()         # ROS /joy X-button trigger
    threading.Thread(target=inspection_worker, args=(esp, cap), daemon=True).start()

    print(f"Dashboard: http://<this-pi-ip>:{CFG.SERVER_PORT}")
    app.run(host=CFG.SERVER_HOST, port=CFG.SERVER_PORT, threaded=True)


if __name__ == "__main__":
    main()