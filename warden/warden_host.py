#!/usr/bin/env python3
"""
WARDEN host - integrated inspection + monitoring + dashboard.

Runs on the Raspberry Pi (the robot's brain). It:
  * talks to the ESP32 sensor hub over serial (ENV / THERMAL / RF / SERVO / LED / STATUS)
  * reads the C920 camera
  * drives the robot base from the joystick (left stick -> /cmd_vel)
  * ON DEMAND (X button / dashboard): runs an optical hidden-camera inspection sweep
    (servo tilt + LED flash + differential glint detection), saving an annotated photo
    per tilt angle
  * fuses optical + thermal + RF into a confidence score
  * serves a live web dashboard at http://<pi-ip>:8000

Run order (no lidar driver, to avoid the CP2102 port clash with the ESP32):
    ros2 launch ~/drive_only.launch.py     # motors (turtlebot3_node on OpenCR/ttyACM0)
    ros2 run joy joy_node                  # gamepad -> /joy
    python3 warden_host.py                 # ESP32 + camera + driving + dashboard
"""

import os
import time
import json
import threading
from dataclasses import dataclass, field, asdict

import serial                 # pip install pyserial
import cv2                     # sudo apt install python3-opencv   (easier on a Pi)
import numpy as np
from flask import Flask, jsonify, render_template_string, send_from_directory

# Optional / heavy - guarded so the app still runs without them
try:
    from ultralytics import YOLO          # pip install ultralytics  (pulls torch - heavy on a Pi)
    _HAVE_YOLO = True
except Exception:
    _HAVE_YOLO = False

try:
    import pygame                         # only for the legacy USE_JOYSTICK pygame path
    _HAVE_PYGAME = True
except Exception:
    _HAVE_PYGAME = False


# ============================== CONFIG ==============================
class CFG:
    # --- ESP32 serial ---
    SERIAL_PORT = "/dev/ttyUSB0"   # the ESP32's CP2102 (verify: ls -l /dev/serial/by-id/)
    BAUD = 115200

    # --- camera ---
    CAMERA_INDEX = 0               # C920 on the Pi - try 0, then 1, 2...
    FRAME_W = 1280
    FRAME_H = 720

    # --- web dashboard ---
    SERVER_HOST = "0.0.0.0"        # 0.0.0.0 = reachable from your laptop's browser
    SERVER_PORT = 8000

    # captured inspection photos are written here and served at /captures/<name>
    CAPTURE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "captures")

    # --- always-on background sensors ---
    SENSOR_PERIOD = 2.0            # seconds between ENV/THERMAL/RF polls

    # --- YOLO object detection (AI room audit) ---
    # Reads the camera CONTINUOUSLY. Leave False unless you've sorted torch on the Pi
    # (the pip wheel SIGILLs on this board).
    USE_YOLO = False
    YOLO_MODEL = "yolov8n.pt"
    YOLO_PERIOD = 1.5
    YOLO_CONF = 0.45

    # --- joystick: DRIVE + SCAN (single ROS node off the /joy topic) ---
    USE_JOYSTICK = False           # legacy pygame path: leave OFF while ROS joy_node runs
    USE_ROS_JOY = True
    JOY_TOPIC = "/joy"
    DRIVE_TOPIC = "/cmd_vel"

    JOY_SCAN_BUTTON = 0            # X on your pad (verified via ros2 topic echo /joy)

    # left stick -> drive. VERIFY indices with: ros2 topic echo /joy
    AXIS_LINEAR = 1               # stick up/down  -> forward/back
    AXIS_ANGULAR = 0              # stick left/right -> turn
    DRIVE_DEADZONE = 0.10
    BASE_LINEAR = 0.18            # m/s at full stick (Burger max ~0.22)
    BASE_ANGULAR = 1.20           # rad/s at full stick (Burger max ~2.84)

    # --- optical inspection sweep ---
    SERVO_ANGLES = [30, 60, 90, 120, 150]   # tilt steps the head sweeps through
    SERVO_SETTLE = 0.4
    LED_WARMUP = 0.15
    LOCK_EXPOSURE = True
    MANUAL_EXPOSURE_MODE = 0.25
    DIFF_THRESH = 60
    STRONG_BONUS = 20
    MIN_AREA = 4
    MAX_AREA = 600

    # --- layer fusion weights (optical is primary) ---
    W_OPTICAL = 0.70
    W_THERMAL = 0.15
    W_RF = 0.15
    SUSPICIOUS_AT = 0.50


# ============================== SHARED STATE ==============================
@dataclass
class State:
    status: str = "idle"                         # idle | scanning
    driving: bool = False                        # base currently commanded to move
    env: dict = field(default_factory=dict)
    thermal: dict = field(default_factory=dict)
    rf: dict = field(default_factory=dict)
    objects: list = field(default_factory=list)
    last_inspection: dict = field(default_factory=dict)
    updated: float = 0.0


STATE = State()
STATE_LOCK = threading.Lock()
SERIAL_LOCK = threading.Lock()
CAMERA_LOCK = threading.Lock()
SCAN_REQUEST = threading.Event()


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
    if not CFG.USE_YOLO:
        return
    if not _HAVE_YOLO:
        print("[yolo] ultralytics not installed - object detection disabled")
        return
    model = YOLO(CFG.YOLO_MODEL)
    print("[yolo] model loaded")
    while True:
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


def annotate_and_save(frame, glints, angle):
    """Draw detected glints on the frame and save it for the dashboard. Returns filename."""
    img = frame.copy()
    for g in glints:
        strong = g["bright"] >= CFG.DIFF_THRESH + CFG.STRONG_BONUS
        color = (0, 0, 255) if strong else (0, 200, 255)   # red = strong, amber = weak
        cv2.circle(img, (g["x"], g["y"]), 12, color, 2)
    cv2.putText(img, f"tilt {angle} deg", (16, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
    fname = f"angle_{angle}.jpg"
    try:
        cv2.imwrite(os.path.join(CFG.CAPTURE_DIR, fname), img,
                    [int(cv2.IMWRITE_JPEG_QUALITY), 70])
    except Exception as ex:
        print("[inspect] save error:", ex)
        return ""
    return fname


def capture_pair(esp, cap):
    """Grab one LED-OFF and one LED-ON frame with matched exposure."""
    if CFG.LOCK_EXPOSURE:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, CFG.MANUAL_EXPOSURE_MODE)

    esp.cmd("LED OFF")
    time.sleep(0.2)
    for _ in range(3):
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
    """One full sweep: tilt through angles, flash + capture, save photo, score, fuse."""
    with STATE_LOCK:
        STATE.status = "scanning"
    esp.cmd("STATUS 0 0 255")     # blue = scanning

    per_angle = []
    with CAMERA_LOCK:
        for ang in CFG.SERVO_ANGLES:
            esp.cmd(f"SERVO {ang}")
            time.sleep(CFG.SERVO_SETTLE)
            off, on = capture_pair(esp, cap)
            glints = find_glints(off, on) if (off is not None and on is not None) else []
            strong = [g for g in glints if g["bright"] >= CFG.DIFF_THRESH + CFG.STRONG_BONUS]
            img = annotate_and_save(on, glints, ang) if on is not None else ""
            per_angle.append({
                "angle": ang,
                "glints": len(glints),
                "strong": len(strong),
                "max_bright": max((g["bright"] for g in glints), default=0),
                "img": img,
            })
        esp.cmd("SERVO 90")       # recentre the head

    angles_with_strong = sum(1 for a in per_angle if a["strong"] > 0)
    optical_score = min(1.0, angles_with_strong / max(1, len(CFG.SERVO_ANGLES)))

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
        STATE.updated = time.time()
    esp.cmd("STATUS 255 0 0" if verdict == "SUSPICIOUS" else "STATUS 0 255 0")
    print("[inspect]", verdict, "confidence", result["confidence"])
    return result


def inspection_worker(esp, cap):
    """Waits for a scan request (X button / dashboard), then runs one inspection."""
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


# ============================== JOYSTICK (pygame - legacy) ==============================
def joystick_loop():
    """Legacy pygame path. Do NOT use while a ROS joy_node owns the gamepad."""
    if not CFG.USE_JOYSTICK:
        return
    if not _HAVE_PYGAME:
        print("[joy] pygame not installed - legacy joystick disabled")
        return
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("[joy] no joystick found")
        return
    js = pygame.joystick.Joystick(0)
    js.init()
    while True:
        for e in pygame.event.get():
            if e.type == pygame.JOYBUTTONDOWN and e.button == CFG.JOY_SCAN_BUTTON:
                SCAN_REQUEST.set()
        time.sleep(0.05)


# ============================== ROS /joy: DRIVE + SCAN ==============================
def ros_joy_loop():
    """Single ROS node: left stick -> /cmd_vel (drive), X button -> scan request.

    Reuses the running joy_node, so it never opens the gamepad device directly.
    Driving is suppressed while an optical scan is in progress.
    """
    if not CFG.USE_ROS_JOY:
        return
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Joy
        from geometry_msgs.msg import Twist
    except Exception as ex:
        print("[joy] rclpy/msgs unavailable - ROS joy disabled:", ex)
        return

    class JoyDriveScan(Node):
        def __init__(self):
            super().__init__('warden_joy')
            self.create_subscription(Joy, CFG.JOY_TOPIC, self.joy_cb, 10)
            self.cmd_pub = self.create_publisher(Twist, CFG.DRIVE_TOPIC, 10)
            self.axes = []
            self.prev_scan = 0
            self.create_timer(0.05, self.drive_cb)        # 20 Hz

        def joy_cb(self, msg):
            self.axes = list(msg.axes)
            b = CFG.JOY_SCAN_BUTTON
            if b < len(msg.buttons):
                cur = msg.buttons[b]
                if cur == 1 and self.prev_scan == 0:       # rising edge
                    print("[joy] X -> scan")
                    SCAN_REQUEST.set()
                self.prev_scan = cur

        def _dz(self, v):
            return 0.0 if abs(v) < CFG.DRIVE_DEADZONE else v

        def drive_cb(self):
            t = Twist()
            with STATE_LOCK:
                scanning = STATE.status == "scanning"
            moving = False
            if not scanning and len(self.axes) > max(CFG.AXIS_LINEAR, CFG.AXIS_ANGULAR):
                lin = self._dz(self.axes[CFG.AXIS_LINEAR])
                ang = self._dz(self.axes[CFG.AXIS_ANGULAR])
                t.linear.x = CFG.BASE_LINEAR * lin
                t.angular.z = CFG.BASE_ANGULAR * ang
                moving = (lin != 0.0 or ang != 0.0)
            self.cmd_pub.publish(t)
            with STATE_LOCK:
                STATE.driving = moving

    rclpy.init()
    node = JoyDriveScan()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ============================== DASHBOARD ==============================
app = Flask(__name__)

DASHBOARD = """<!doctype html>
<html lang="en"><head><meta charset="utf-8"><title>WARDEN</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  :root{
    --bg:#0a0c10; --panel:#12161d; --panel2:#161b24; --line:#232a35;
    --ink:#e8edf4; --muted:#7d8896; --dim:#586473;
    --amber:#ffb020; --green:#34d27b; --red:#ff4d5e; --blue:#3d9bff;
    --mono:"SF Mono","JetBrains Mono","Fira Code",ui-monospace,"Cascadia Code",Menlo,Consolas,monospace;
    --sans:ui-sans-serif,-apple-system,"Segoe UI",Roboto,sans-serif;
  }
  *{box-sizing:border-box}
  body{margin:0;background:
      radial-gradient(1200px 600px at 80% -10%, #14202b 0%, transparent 60%),
      radial-gradient(900px 500px at -10% 110%, #1a1320 0%, transparent 55%),
      var(--bg);
    color:var(--ink);font-family:var(--sans);
    background-attachment:fixed;min-height:100vh;
    -webkit-font-smoothing:antialiased}
  .wrap{max-width:1180px;margin:0 auto;padding:22px 20px 60px}
  .topbar{display:flex;align-items:center;justify-content:space-between;
    border-bottom:1px solid var(--line);padding-bottom:14px;margin-bottom:22px}
  .brand{font-family:var(--mono);font-weight:700;font-size:20px;letter-spacing:6px}
  .brand .dot{color:var(--amber)}
  .brand small{display:block;font-size:10px;letter-spacing:3px;color:var(--dim);
    font-weight:500;margin-top:3px}
  .pills{display:flex;gap:8px;align-items:center}
  .pill{font-family:var(--mono);font-size:11px;letter-spacing:1.5px;text-transform:uppercase;
    padding:6px 12px;border-radius:6px;border:1px solid var(--line);color:var(--muted);
    background:var(--panel)}
  .pill.on{color:var(--bg);border-color:transparent}
  .pill.idle.on{background:var(--blue)} .pill.scanning.on{background:var(--amber)}
  .pill.drive.on{background:var(--green)}

  /* verdict banner */
  .verdict{position:relative;overflow:hidden;border:1px solid var(--line);
    border-radius:14px;background:linear-gradient(180deg,var(--panel2),var(--panel));
    padding:22px 24px;margin-bottom:20px;display:flex;align-items:center;
    justify-content:space-between;gap:18px}
  .verdict::before{content:"";position:absolute;inset:0;width:6px;background:var(--dim)}
  .verdict.clear::before{background:var(--green)}
  .verdict.SUSPICIOUS::before{background:var(--red)}
  .vmain{display:flex;flex-direction:column;gap:4px;padding-left:8px}
  .vlabel{font-family:var(--mono);font-size:11px;letter-spacing:2px;color:var(--muted);
    text-transform:uppercase}
  .vtext{font-family:var(--mono);font-size:34px;font-weight:700;letter-spacing:1px}
  .vtext.clear{color:var(--green)} .vtext.SUSPICIOUS{color:var(--red)} .vtext.none{color:var(--dim)}
  .vmeta{font-size:12px;color:var(--muted);font-family:var(--mono)}
  .conf{text-align:right;font-family:var(--mono)}
  .conf b{font-size:40px;font-weight:700;display:block;line-height:1}
  .conf span{font-size:11px;letter-spacing:2px;color:var(--muted);text-transform:uppercase}

  .grid{display:grid;grid-template-columns:1.4fr 1fr;gap:16px}
  @media(max-width:820px){.grid{grid-template-columns:1fr}}
  .card{background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:16px}
  .card h2{font-family:var(--mono);font-size:11px;letter-spacing:2px;color:var(--muted);
    text-transform:uppercase;margin:0 0 14px;display:flex;justify-content:space-between}
  .card h2 .hint{color:var(--dim);letter-spacing:1px}

  /* tilt gauge */
  .gauge{display:flex;align-items:center;gap:18px;flex-wrap:wrap}
  .gauge svg{flex:0 0 auto}
  .legend{font-family:var(--mono);font-size:11px;color:var(--muted);line-height:1.9}
  .legend .sw{display:inline-block;width:10px;height:10px;border-radius:2px;margin-right:6px;
    vertical-align:middle}

  /* photo gallery */
  .shots{display:grid;grid-template-columns:repeat(auto-fill,minmax(150px,1fr));gap:10px}
  .shot{border:1px solid var(--line);border-radius:8px;overflow:hidden;background:#000;position:relative}
  .shot img{display:block;width:100%;height:auto;aspect-ratio:16/9;object-fit:cover}
  .shot .tag{position:absolute;top:6px;left:6px;font-family:var(--mono);font-size:10px;
    padding:2px 7px;border-radius:4px;background:rgba(0,0,0,.65);color:var(--ink);letter-spacing:1px}
  .shot .hit{position:absolute;bottom:6px;right:6px;font-family:var(--mono);font-size:10px;
    padding:2px 7px;border-radius:4px;letter-spacing:.5px}
  .shot .hit.s{background:var(--red);color:#fff} .shot .hit.n{background:var(--panel2);color:var(--muted)}
  .empty{color:var(--dim);font-family:var(--mono);font-size:12px;padding:14px 2px}

  /* telemetry */
  .kv{display:flex;justify-content:space-between;font-size:13px;padding:5px 0;
    border-bottom:1px dashed var(--line)}
  .kv:last-child{border-bottom:0}
  .kv span{color:var(--muted)} .kv b{font-family:var(--mono);font-weight:600}
  .stack{display:flex;flex-direction:column;gap:16px}
  .foot{margin-top:22px;font-family:var(--mono);font-size:11px;color:var(--dim);
    display:flex;justify-content:space-between;letter-spacing:1px}
</style></head><body>
<div class="wrap">
  <div class="topbar">
    <div class="brand">WARDEN<span class="dot">.</span><small>OPTICAL // THERMAL // RF</small></div>
    <div class="pills">
      <div id="p-status" class="pill idle on">idle</div>
      <div id="p-drive" class="pill drive">drive</div>
      <button onclick="scan()" class="pill" style="cursor:pointer;color:var(--amber);border-color:var(--amber)">▶ run scan</button>
    </div>
  </div>

  <div id="verdict" class="verdict">
    <div class="vmain">
      <div class="vlabel">last inspection</div>
      <div id="vtext" class="vtext none">— — —</div>
      <div id="vmeta" class="vmeta">no scan yet</div>
    </div>
    <div class="conf"><b id="conf">--</b><span>confidence</span></div>
  </div>

  <div class="grid">
    <div class="stack">
      <div class="card">
        <h2>camera sweep <span class="hint" id="sweep-hint">head tilt</span></h2>
        <div class="gauge">
          <svg id="arc" width="220" height="130" viewBox="0 0 220 130"></svg>
          <div class="legend">
            <div><span class="sw" style="background:var(--red)"></span>strong glint (lens-like)</div>
            <div><span class="sw" style="background:var(--amber)"></span>weak / single reflection</div>
            <div><span class="sw" style="background:var(--dim)"></span>angle swept, clear</div>
          </div>
        </div>
      </div>
      <div class="card">
        <h2>captured frames <span class="hint">red rings = glints</span></h2>
        <div id="shots" class="shots"></div>
      </div>
    </div>

    <div class="stack">
      <div class="card"><h2>environment</h2>
        <div class="kv"><span>temperature</span><b id="t">--</b></div>
        <div class="kv"><span>humidity</span><b id="h">--</b></div>
        <div class="kv"><span>pressure</span><b id="p">--</b></div>
        <div class="kv"><span>air (gas)</span><b id="gas">--</b></div>
      </div>
      <div class="card"><h2>thermal</h2>
        <div class="kv"><span>min</span><b id="tmin">--</b></div>
        <div class="kv"><span>max</span><b id="tmax">--</b></div>
        <div class="kv"><span>mean</span><b id="tmean">--</b></div>
        <div class="kv"><span>hot spot</span><b id="thot">--</b></div>
      </div>
      <div class="card"><h2>rf (wi-fi)</h2>
        <div class="kv"><span>devices</span><b id="rfn">--</b></div>
        <div class="kv"><span>strong</span><b id="rfs">--</b></div>
      </div>
      <div class="card"><h2>room audit</h2>
        <div id="objs" class="empty">disabled</div>
      </div>
    </div>
  </div>

  <div class="foot"><span id="upd">awaiting telemetry…</span><span>WARDEN host · :8000</span></div>
</div>

<script>
const ANGLES = [30,60,90,120,150];
async function scan(){ try{ await fetch('/scan',{method:'POST'}); }catch(e){} }
function set(id,v){ const el=document.getElementById(id); if(el) el.textContent=v; }

function drawArc(perAngle){
  // semicircular gauge, ticks at each sweep angle, colored by glint strength
  const svg=document.getElementById('arc'); svg.innerHTML='';
  const cx=110, cy=120, r=92;
  const NS='http://www.w3.org/2000/svg';
  const base=document.createElementNS(NS,'path');
  const p=a=>{const rad=Math.PI*(1-a/180);return [cx+r*Math.cos(rad), cy-r*Math.sin(rad)];};
  const [x0,y0]=p(0), [x1,y1]=p(180);
  base.setAttribute('d',`M ${x0} ${y0} A ${r} ${r} 0 0 1 ${x1} ${y1}`);
  base.setAttribute('fill','none'); base.setAttribute('stroke','#232a35'); base.setAttribute('stroke-width','3');
  svg.appendChild(base);
  const byAngle={}; (perAngle||[]).forEach(a=>byAngle[a.angle]=a);
  ANGLES.forEach(ang=>{
    const [x,y]=p(ang); const d=byAngle[ang];
    const dot=document.createElementNS(NS,'circle');
    dot.setAttribute('cx',x); dot.setAttribute('cy',y); dot.setAttribute('r','7');
    let c='#586473';
    if(d){ if(d.strong>0) c='#ff4d5e'; else if(d.glints>0) c='#ffb020'; }
    dot.setAttribute('fill',c); dot.setAttribute('stroke','#0a0c10'); dot.setAttribute('stroke-width','2');
    svg.appendChild(dot);
    const lbl=document.createElementNS(NS,'text');
    const [lx,ly]=p(ang);
    lbl.setAttribute('x',lx); lbl.setAttribute('y',ang===90?ly-14:ly-12);
    lbl.setAttribute('fill','#7d8896'); lbl.setAttribute('font-size','10');
    lbl.setAttribute('font-family','monospace'); lbl.setAttribute('text-anchor','middle');
    lbl.textContent=ang+'°'; svg.appendChild(lbl);
  });
}

function drawShots(perAngle, t){
  const box=document.getElementById('shots');
  if(!perAngle || !perAngle.length){ box.innerHTML='<div class="empty">no captures yet — run a scan</div>'; return; }
  box.innerHTML='';
  perAngle.forEach(a=>{
    const cell=document.createElement('div'); cell.className='shot';
    if(a.img){
      const im=document.createElement('img');
      im.src='/captures/'+a.img+'?t='+t; im.alt='tilt '+a.angle;
      cell.appendChild(im);
    }
    const tag=document.createElement('div'); tag.className='tag'; tag.textContent=a.angle+'°';
    const hit=document.createElement('div');
    hit.className='hit '+(a.strong>0?'s':'n');
    hit.textContent=(a.strong>0? a.strong+' strong' : a.glints+' glint');
    cell.appendChild(tag); cell.appendChild(hit); box.appendChild(cell);
  });
}

async function tick(){
  try{
    const d=await (await fetch('/data')).json();
    // status pills
    const ps=document.getElementById('p-status');
    ps.textContent=d.status; ps.className='pill '+d.status+' on';
    document.getElementById('p-drive').className='pill drive'+(d.driving?' on':'');

    const e=d.env||{};
    set('t', e.t!=null? e.t+' °C':'--'); set('h', e.h!=null? e.h+' %':'--');
    set('p', e.p!=null? e.p+' hPa':'--'); set('gas', e.gas!=null? e.gas:'--');
    const th=d.thermal||{};
    set('tmin', th.min!=null? th.min+' °C':'--'); set('tmax', th.max!=null? th.max+' °C':'--');
    set('tmean', th.mean!=null? th.mean+' °C':'--');
    const rf=d.rf||{};
    set('rfn', rf.n!=null? rf.n:'--'); set('rfs', rf.strong!=null? rf.strong:'--');

    const objs=d.objects||[];
    document.getElementById('objs').textContent =
      objs.length? objs.map(o=>o.label+' ×'+o.count).join(', ') : 'disabled';

    const ins=d.last_inspection||{};
    const v=ins.verdict||null;
    const vt=document.getElementById('vtext');
    vt.textContent = v? (v==='SUSPICIOUS'?'SUSPICIOUS':'CLEAR') : '— — —';
    vt.className='vtext '+(v||'none');
    document.getElementById('verdict').className='verdict '+(v||'');
    set('conf', ins.confidence!=null? ins.confidence:'--');
    set('thot', ins.thermal_hot? 'yes':'no');
    document.getElementById('vmeta').textContent = ins.time?
      ('optical '+ins.optical_score+' · '+ins.angles_with_strong+'/'+ANGLES.length+' angles · '+ins.time)
      : 'no scan yet';

    drawArc(ins.per_angle);
    drawShots(ins.per_angle, Math.floor((d.updated||0)));
    set('upd', d.updated? 'updated '+new Date(d.updated*1000).toLocaleTimeString() : 'awaiting telemetry…');
  }catch(err){ /* transient */ }
}
drawArc([]); drawShots([],0);
setInterval(tick,1000); tick();
</script>
</body></html>"""


@app.route("/")
def index():
    return render_template_string(DASHBOARD)


@app.route("/data")
def data():
    with STATE_LOCK:
        return jsonify(asdict(STATE))


@app.route("/captures/<path:fname>")
def captures(fname):
    return send_from_directory(CFG.CAPTURE_DIR, fname)


@app.route("/scan", methods=["POST"])
def scan():
    SCAN_REQUEST.set()
    return jsonify({"ok": True})


# ============================== MAIN ==============================
def main():
    print("WARDEN host starting...")
    os.makedirs(CFG.CAPTURE_DIR, exist_ok=True)

    esp = ESP32(CFG.SERIAL_PORT, CFG.BAUD)
    print("ESP32:", esp.cmd("PING"))     # expect PONG

    cap = cv2.VideoCapture(CFG.CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CFG.FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CFG.FRAME_H)
    if not cap.isOpened():
        print("WARNING: camera did not open at index", CFG.CAMERA_INDEX)

    threading.Thread(target=sensor_loop, args=(esp,), daemon=True).start()
    threading.Thread(target=yolo_loop, args=(cap,), daemon=True).start()
    threading.Thread(target=joystick_loop, daemon=True).start()         # legacy pygame (off)
    threading.Thread(target=ros_joy_loop, daemon=True).start()          # drive + scan
    threading.Thread(target=inspection_worker, args=(esp, cap), daemon=True).start()

    print(f"Dashboard: http://<this-pi-ip>:{CFG.SERVER_PORT}")
    app.run(host=CFG.SERVER_HOST, port=CFG.SERVER_PORT, threaded=True)


if __name__ == "__main__":
    main()