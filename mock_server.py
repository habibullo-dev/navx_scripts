import asyncio
import json
import math
import random
import time

import websockets

PORT = 8765

# Synthetic labels
LABELS = ["person", "car", "bicycle", "dog", "traffic light", "bench"]

class DemoState:
    def __init__(self):
        self.battery = 72
        self.battery_dir = -1
        self.progress = 0.0
        self.mode = "fastest"
        self.last_mode_change = time.time()
        # Define a fixed route of length 10m between home and destination
        self.home = {"x": 0.2, "y": 7.8}
        self.dest = {"x": 7.8, "y": 0.2}
        self.speed_mps = 0.6  # movement speed along route for demo

    def step(self, dt: float):
        # Battery oscillates between 15 and 85
        self.battery += self.battery_dir * 0.05
        if self.battery <= 15:
            self.battery = 15
            self.battery_dir = 1
        elif self.battery >= 85:
            self.battery = 85
            self.battery_dir = -1

        # Move progress 0..1, reverse at ends
        self.progress += (self.speed_mps * dt) / 10.0
        if self.progress > 1.0:
            self.progress = 1.0
            self.speed_mps *= -1
        elif self.progress < 0.0:
            self.progress = 0.0
            self.speed_mps *= -1

    def current_pose(self):
        # Interpolate linearly between home and dest
        x = self.home["x"] + (self.dest["x"] - self.home["x"]) * self.progress
        y = self.home["y"] + (self.dest["y"] - self.home["y"]) * self.progress
        # Compute yaw from path direction
        dx = (self.dest["x"] - self.home["x"]) * (1 if self.speed_mps >= 0 else -1)
        dy = (self.dest["y"] - self.home["y"]) * (1 if self.speed_mps >= 0 else -1)
        yaw = math.atan2(dy, dx)
        return {"x": x, "y": y, "yaw": yaw}

    def nav_status(self):
        cp = self.current_pose()
        home_reached = self.progress <= 0.001
        dest_reached = self.progress >= 0.999
        return {
            "current_pose": cp,
            "home": {"x": self.home["x"], "y": self.home["y"]},
            "destination": {"x": self.dest["x"], "y": self.dest["y"]},
            "home_reached": home_reached,
            "dest_reached": dest_reached,
        }


def random_detections(n=3):
    objs = []
    for _ in range(n):
        x1, y1 = random.random() * 0.7, random.random() * 0.7
        w, h = random.random() * 0.25 + 0.05, random.random() * 0.25 + 0.05
        x2, y2 = min(x1 + w, 0.98), min(y1 + h, 0.98)
        objs.append({
            "label": random.choice(LABELS),
            "conf": round(random.uniform(0.5, 0.98), 3),
            "bbox": [round(x1, 3), round(y1, 3), round(x2, 3), round(y2, 3)]
        })
    return objs


async def handler(websocket):
    print("Client connected")
    state = DemoState()

    last = time.perf_counter()
    try:
        while True:
            now = time.perf_counter()
            dt = now - last
            last = now

            state.step(dt)

            payload = {
                "timestamp": time.time(),
                "objects": random_detections(random.randint(1, 5)),
                "infer_ms": round(random.uniform(12, 42), 2),
                "battery": int(state.battery),
                "nav_status": state.nav_status(),
            }

            await websocket.send(json.dumps(payload))
            # ~30 FPS message rate feels smooth for demo
            await asyncio.sleep(1/30)
    except websockets.exceptions.ConnectionClosed:
        print("Client disconnected")


async def main():
    print(f"Starting Mock WebSocket server on ws://localhost:{PORT}")
    async with websockets.serve(handler, "0.0.0.0", PORT):
        await asyncio.Future()  # run forever


if __name__ == "__main__":
    asyncio.run(main())
