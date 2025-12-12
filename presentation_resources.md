# NavX Project: Technical Demo Resources

This document compiles every high-impact code section, mathematical formula, and system feature from your codebase. Use these specific blocks in your presentation slides to demonstrate technical depth.

---

## 🏗️ 1. Visual Engineering & UI/UX
**Goal:** Prove you can build professional, futuristic interfaces (Cyberpunk/Glassmorphism style).

### A. The "Glassmorphism" Effect
**File:** `camera.html` (CSS)
**Why:** Demonstrates modern CSS3 mastery.
```css
/* Real-time blur for "Frosted Glass" look */
.modal-overlay {
    background: rgba(0, 0, 0, 0.85);
    backdrop-filter: blur(5px);
}

/* Glowing Neon Borders */
.modal-content::before {
    content: "";
    position: absolute;
    background: linear-gradient(45deg, var(--primary-color), transparent, var(--primary-color));
    opacity: 0.3;
}
```

### B. Procedural Vector Map
**File:** `camera.html` (SVG/JS)
**Why:** Shows you generate visuals with code, not just static images.
```javascript
// Dynamic Agent Animation on SVG Map
function updateAgentPosition(status) {
    // Interpolate position along the path based on progress
    const path = document.getElementById(routes[activeMode].id);
    const len = path.getTotalLength();
    const pt = path.getPointAtLength(progress * len);

    // Calculate rotation tangent to path
    const nextPt = path.getPointAtLength(Math.min((progress + 0.01) * len, len));
    const angle = Math.atan2(nextPt.y - pt.y, nextPt.x - pt.x) * (180 / Math.PI);
    
    agent.setAttribute("transform", `translate(${pt.x}, ${pt.y}) rotate(${angle})`);
}
```

---

## 🧠 2. Artificial Intelligence & Optimization
**Goal:** Show that you prioritize performance and real-time constraints.

### A. Adaptive Frame Rate Algorithm
**File:** `detection_server.py`
**Why:** **Crucial Highlight.** Shows engineering optimization. You balance latency vs. CPU load dynamically.
```python
# Adaptive Skip Logic: Balances Latency (infer_ms) vs. Fluidity
# If inference is slow (>50ms), we skip more frames to maintain real-time UI.
if infer_ms > 50 and effective_skip < MAX_FRAME_SKIP:
    effective_skip += 1  # Reduce load
elif infer_ms < 30 and effective_skip > MIN_FRAME_SKIP:
    effective_skip -= 1  # Increase smoothness
```

### B. Efficient Data Packaging
**File:** `detection_server.py`
**Why:** Shows good "Network Engineering".
```python
# Packing multiple data streams (AI, Battery, Nav) into one packet
# reduces WebSocket overhead logic.
payload = {
    "timestamp": time.time(),
    "objects": detections,        # YOLOv8 Results
    "infer_ms": round(infer_ms, 2),
    "battery": latest_battery_pct,
    "nav_status": latest_nav_status
}
await websocket.send(json.dumps(payload))
```

---

## 📊 3. Sensor Fusion & Data Science
**Goal:** Show mathematical understanding of sensor data.

### A. Road Quality Variance Formula
**File:** `road_quality_node.py`
**Why:** Moving beyond simple thresholds to statistical analysis.
**The Formula:**
$$ \sigma^2 = \frac{\sum (x - \mu)^2}{N} $$

**The Code:**
```python
# We calculate Statistical Variance to determine road "Roughness"
# This avoids false positives from single bumps.
ax_mean = sum(self.ax_window) / len(self.ax_window)

# Generator expression for efficiency
var_ax = sum((a - ax_mean) ** 2 for a in self.ax_window) / len(self.ax_window)
var_ay = sum((a - ay_mean) ** 2 for a in self.ay_window) / len(self.ay_window)

total_roughness_score = var_ax + var_ay
```

### B. Geofencing & Distance Math
**File:** `waypoint_notifier.py`
**Why:** Core robotics spatial logic.
```python
# Euclidean Distance Calculation
def distance(a, b):
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

# Geofence Trigger
if d_dest <= self.reach_threshold and not self.dest_reached:
    self.dest_reached = True
    self.get_logger().info(f"🎯 Destination reached! ({d_dest:.3f} m)")
```

---

## 🎮 4. System Logic & Interaction
**Goal:** Show personality and system management.

### A. The "Celebration Spin"
**File:** `nexius_joy_teleop.py`
**Why:** Adds character/personality to the robot. Great demo closer.
```python
# Feature: Press 'X' to trigger a 360-degree victory spin
if idx == 3: # X Button
    total_angle = 3.0 * 2.0 * math.pi  # 3 Full Rotations
    self.celebration_time_left = total_angle / self.celebration_speed
    self.get_logger().info('Celebration SPIN! (3x360°)')
```

### B. Smart Power Management
**File:** `smart_camera.py`
**Why:** Shows attention to battery life (Green Engineering).
```python
# Logic: Automatically kill camera process if robot is still for 12s
def check_camera_state(self):
    should_be_on = (time.time() - self.last_move_time) < TIMEOUT_SECONDS
    
    if should_be_on and self.camera_process is None:
        self.start_camera() # Spawns 'mjpg_streamer'
    elif not should_be_on and self.camera_process:
        self.stop_camera()  # Kills process group to save battery
```

---

## 🚀 Summary Checklist for Presentation

1.  **Intro:** Show the **UI** (Section 1A) to grab attention.
2.  **The "Brains":** Show **Adaptive AI** (Section 2A) to prove it's smart.
3.  **The "Senses":** Show **Road Quality** (Section 3A) to show it interacts with the world.
4.  **The "Soul":** End with the **Celebration Spin** (Section 4A) video clip + code.
