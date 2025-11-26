#!/usr/bin/env python3
import threading
import time

import cv2
from cv2 import IMWRITE_JPEG_QUALITY
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import Response

app = FastAPI()

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('navx_cam_ws_subscriber')
        self.bridge = CvBridge()
        self.latest_jpeg = None
        self.last_update = 0.0

        self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info('NavX WS camera subscriber started on /image_raw')

    def image_callback(self, msg: Image):
        try:
            # convert to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # encode to JPEG (lower quality => smaller, faster)
            ok, jpeg = cv2.imencode('.jpg', cv_image, [int(IMWRITE_JPEG_QUALITY), 65])
            if ok:
                self.latest_jpeg = jpeg.tobytes()
                self.last_update = time.time()
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")

ros_node: CameraSubscriber | None = None

@app.get("/navx/cam/frame")
def get_single_frame():
    """Debug endpoint: returns one JPEG frame."""
    if ros_node is None or ros_node.latest_jpeg is None:
        return Response(content=b"No frame", media_type="text/plain", status_code=503)
    return Response(content=ros_node.latest_jpeg, media_type="image/jpeg")

@app.websocket("/ws/cam")
async def websocket_cam(websocket: WebSocket):
    """WebSocket endpoint streaming latest JPEG frames."""
    await websocket.accept()
    try:
        # target ~15 FPS → 0.066s interval
        send_interval = 1.0 / 15.0
        last_sent_time = 0.0

        while True:
            now = time.time()
            if ros_node is not None and ros_node.latest_jpeg is not None:
                # send only if we have a new-ish frame
                if now - last_sent_time >= send_interval:
                    await websocket.send_bytes(ros_node.latest_jpeg)
                    last_sent_time = now
            # small sleep to avoid busy loop
            await asyncio_sleep(0.01)
    except WebSocketDisconnect:
        # client closed – just exit the loop
        return

# simple wrapper because FastAPI WS handler is async
import asyncio
async def asyncio_sleep(sec: float):
    await asyncio.sleep(sec)

def start_ros2():
    global ros_node
    rclpy.init()
    ros_node = CameraSubscriber()
    rclpy.spin(ros_node)
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    # start ROS2 node in a background thread
    t = threading.Thread(target=start_ros2, daemon=True)
    t.start()

    import uvicorn
    # 0.0.0.0 so robot & other PCs can reach it
    uvicorn.run(app, host="0.0.0.0", port=8000)
