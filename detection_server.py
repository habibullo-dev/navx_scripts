import cv2
import asyncio
import websockets
import json
import time
from ultralytics import YOLO

# --- CONFIGURATION ---
# The IP of your Raspberry Pi (from your HTML snippet)
PI_STREAM_URL = "http://172.20.10.10:8080/?action=stream"
# Port for the dashboard to listen to for AI data
WEBSOCKET_PORT = 8765
# How many frames to skip (Process 1 out of every N frames)
FRAME_SKIP = 3 

# Load the lightweight YOLOv8 Nano model
# It will download 'yolov8n.pt' automatically on first run (~6MB)
print("Loading YOLOv8n model...")
model = YOLO("yolov8n.pt") 

async def detection_handler(websocket):
    print("Client connected to AI stream!")
    
    cap = cv2.VideoCapture(PI_STREAM_URL)
    
    if not cap.isOpened():
        print(f"Error: Could not open stream at {PI_STREAM_URL}")
        await websocket.send(json.dumps({"error": "Stream unavailable"}))
        return

    frame_count = 0
    
    try:
        while cap.isOpened():
            success, frame = cap.read()
            if not success:
                break

            frame_count += 1
            
            # Skip frames to save CPU (Actionable 'lightweight' trick)
            if frame_count % FRAME_SKIP != 0:
                await asyncio.sleep(0.01) # Yield control slightly
                continue

            # --- AI PROCESSING ---
            # Resize for speed (optional, YOLO handles this, but smaller is faster)
            # frame_small = cv2.resize(frame, (640, 480)) 
            
            # Run inference
            # stream=True makes it faster/lighter
            results = model(frame, stream=True, verbose=False)

            detections = []
            
            # Parse results
            for r in results:
                boxes = r.boxes
                for box in boxes:
                    # Get box coordinates (normalized 0-1 is best for web scaling)
                    x1, y1, x2, y2 = box.xyxyn[0].tolist() 
                    conf = float(box.conf[0])
                    cls = int(box.cls[0])
                    label = model.names[cls]

                    # Filter: Only send common traffic items to keep JSON light
                    # (Remove this if check to see EVERYTHING)
                    if conf > 0.4: 
                        detections.append({
                            "label": label,
                            "conf": round(conf, 2),
                            "bbox": [x1, y1, x2, y2] # Normalized coordinates
                        })

            # Send data to dashboard
            payload = json.dumps({"timestamp": time.time(), "objects": detections})
            await websocket.send(payload)
            
            # Tiny sleep to let the event loop breathe
            await asyncio.sleep(0.01)

    except websockets.exceptions.ConnectionClosed:
        print("Client disconnected")
    finally:
        cap.release()

async def main():
    print(f"Starting AI WebSocket server on port {WEBSOCKET_PORT}...")
    print(f"Reading stream from: {PI_STREAM_URL}")
    async with websockets.serve(detection_handler, "localhost", WEBSOCKET_PORT):
        await asyncio.Future()  # Run forever

if __name__ == "__main__":
    asyncio.run(main())