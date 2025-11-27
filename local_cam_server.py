import cv2
import time
import threading
import json
import asyncio
import websockets
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from ultralytics import YOLO

# --- CONFIGURATION ---
HTTP_PORT = 8080
WEBSOCKET_PORT = 8765
CONF_THRESHOLD = 0.4
TARGET_WIDTH = 320

# --- GLOBAL STATE ---
# Shared between threads
class SharedState:
    def __init__(self):
        self.frame_bytes = None
        self.detections = []
        self.lock = threading.Lock()
        self.running = True

state = SharedState()

# --- MJPEG HTTP HANDLER ---
class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while state.running:
                    with state.lock:
                        if state.frame_bytes is None:
                            continue
                        data = state.frame_bytes
                    
                    self.wfile.write(b'--frame\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(data))
                    self.end_headers()
                    self.wfile.write(data)
                    self.wfile.write(b'\r\n')
                    time.sleep(0.03) # Limit to ~30 FPS streaming
            except Exception as e:
                pass
        else:
            self.send_response(404)
            self.end_headers()

def run_http_server():
    server = ThreadingHTTPServer(('0.0.0.0', HTTP_PORT), MJPEGHandler)
    print(f"HTTP MJPEG Server started on port {HTTP_PORT}")
    server.serve_forever()

# --- CAMERA & INFERENCE LOOP ---
# --- CAMERA & INFERENCE LOOP ---
def run_camera_loop():
    print("Loading YOLO model...")
    model = None
    try:
        from ultralytics import YOLO
        model = YOLO("yolov8n.pt")
    except ImportError:
        print("Warning: 'ultralytics' not found. Object detection disabled.")

    cap = cv2.VideoCapture(0)
    
    use_dummy = False
    if not cap.isOpened():
        print("Error: Could not open webcam. Switching to DUMMY MODE (Test Pattern).")
        use_dummy = True
    else:
        print("Camera started.")
    
    import numpy as np

    while state.running:
        if use_dummy:
            # Generate a moving test pattern
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            t = time.time()
            cv2.circle(frame, (int(320 + 100 * np.sin(t)), int(240 + 100 * np.cos(t))), 50, (0, 255, 0), -1)
            cv2.putText(frame, "NO CAMERA - DUMMY MODE", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            if model is None:
                cv2.putText(frame, "NO YOLO - DETECTION DISABLED", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        else:
            ret, frame = cap.read()
            if not ret:
                print("Failed to read frame")
                time.sleep(1)
                continue

        # Resize for consistency
        h, w = frame.shape[:2]
        if w != TARGET_WIDTH:
            new_h = int(h * (TARGET_WIDTH / w))
            frame = cv2.resize(frame, (TARGET_WIDTH, new_h))

        # Inference
        infer_ms = 0
        current_detections = []
        
        if model:
            t0 = time.perf_counter()
            results = model(frame, verbose=False, stream=True)
            infer_ms = (time.perf_counter() - t0) * 1000.0

            for r in results:
                for box in r.boxes:
                    conf = float(box.conf[0])
                    if conf < CONF_THRESHOLD:
                        continue
                    x1, y1, x2, y2 = box.xyxyn[0].tolist()
                    cls = int(box.cls[0])
                    label = model.names.get(cls, str(cls))
                    current_detections.append({
                        "label": label,
                        "conf": round(conf, 2),
                        "bbox": [x1, y1, x2, y2]
                    })

        # Encode for MJPEG
        _, jpg = cv2.imencode('.jpg', frame)
        
        # Update State
        with state.lock:
            state.frame_bytes = jpg.tobytes()
            state.detections = {
                "objects": current_detections,
                "infer_ms": round(infer_ms, 1),
                "width": TARGET_WIDTH,
                "height": frame.shape[0],
                "timestamp": time.time()
            }
        
        time.sleep(0.01)

    if not use_dummy:
        cap.release()

# --- WEBSOCKET HANDLER ---
async def ws_handler(websocket):
    print("WS Client connected")
    try:
        while state.running:
            with state.lock:
                data = state.detections
            
            if data:
                await websocket.send(json.dumps(data))
            
            await asyncio.sleep(0.05) # 20 Hz updates
    except websockets.exceptions.ConnectionClosed:
        pass

async def run_ws_server():
    print(f"WebSocket Server started on port {WEBSOCKET_PORT}")
    async with websockets.serve(ws_handler, "0.0.0.0", WEBSOCKET_PORT):
        await asyncio.Future()

# --- MAIN ---
if __name__ == "__main__":
    # Start HTTP Server in Thread
    t_http = threading.Thread(target=run_http_server, daemon=True)
    t_http.start()

    # Start Camera Loop in Thread
    t_cam = threading.Thread(target=run_camera_loop, daemon=True)
    t_cam.start()

    # Run WebSocket Server in Main Async Loop
    try:
        asyncio.run(run_ws_server())
    except KeyboardInterrupt:
        state.running = False
        print("Stopping...")
