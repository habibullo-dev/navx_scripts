# NAVX Scripts

This folder contains ROS2 nodes and a lightweight web demo used to showcase the NAVX robot vision and navigation dashboard.

Two ways to run this project:

1) Demo Mode (no ROS, no GPU required)
- Runs a tiny WebSocket server that streams synthetic detections, battery, and navigation status
- Open `camera.html` to see the dashboard animate and update in real-time

2) Full System (ROS2 + Camera + YOLO)
- Uses `detection_server.py` to run YOLOv8 on camera frames and bridge ROS2 topics to the dashboard
- Requires a Linux system with ROS2 installed and a working camera stream (e.g. `mjpg_streamer`)

## 1) Demo Mode (recommended for quick submission)

Prerequisites:
- Python 3.9+

Install dependencies and run the mock server:

```
python -m venv .venv
.venv\Scripts\Activate.ps1
pip install -r requirements-demo.txt
python mock_server.py
```

Open the dashboard:
- Open `camera.html` in your browser (double-click or use a local server). It connects to `ws://localhost:8765` by default.

Tip (Windows): you can use `run_demo.ps1` to automate the steps above.

## 2) Full System (ROS2 + YOLO + Camera)

Requirements:
- Linux with ROS2 (e.g. Humble) installed and sourced
- Python environment with: `opencv-python`, `torch`, `ultralytics`, `websockets`
- A camera stream available over HTTP/MJPEG (e.g. `mjpg_streamer` on the robot)
- ROS2 topics:
  - `/battery_state` (`sensor_msgs/BatteryState`)
  - `/nav_status` (`std_msgs/String` JSON payload)

Install Python deps (example):
```
pip install opencv-python websockets ultralytics torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
```

Run the server:
```
python detection_server.py
```

Then open `camera.html` in your browser. The page will show:
- Live camera stream (if reachable via MJPEG URL)
- YOLO detections drawn as boxes
- Battery level and connection status
- Map overlay and navigation mode selection

Notes:
- Edit `PI_STREAM_URL` and `WEBSOCKET_PORT` in `detection_server.py` to match your network.
- `rclpy` and ROS message packages are installed by ROS2 and are not provided via `pip`. Ensure your ROS2 environment is sourced before running the server.

## Repository Layout (simplified)

- `camera.html` — Vision + navigation dashboard UI
- `camera.styles.css` — Minimal CSS for the dashboard (variables, map styles)
- `mock_server.py` — Demo WebSocket server (no ROS/GPU required)
- `requirements-demo.txt` — Dependencies for the demo server
- `detection_server.py` — YOLO + ROS2 bridge streaming to the dashboard
- `smart_camera.py` — ROS2 node that auto-starts/stops MJPEG stream based on movement
- `waypoint_notifier.py` — ROS2 node that publishes home/destination reach events
- `debug_battery.py` — Helper for debugging battery topic
- `yolov8n.pt` — YOLOv8 Nano model weights

## Troubleshooting

- If `camera.html` shows DISCONNECTED, ensure the server (mock or detection) is running on port 8765.
- If the live feed shows SIGNAL LOST, verify the `IMG` source URL (MJPEG) or ignore it in demo mode.
- On Windows, ROS2 nodes will not run unless you have a compatible ROS2 setup. Use Demo Mode instead.
- If you use a firewall, allow inbound connections to port 8765 for local testing.
