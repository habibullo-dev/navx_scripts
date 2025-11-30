#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import subprocess
import time
import os
import signal

# Configuration
CAMERA_CMD = 'mjpg_streamer -i "input_uvc.so -d /dev/video0 -r 640x480 -f 30" -o "output_http.so -w ./www -p 8080"'
TIMEOUT_SECONDS = 5.0  # Time to wait after stopping before turning off camera

class SmartCamera(Node):
    def __init__(self):
        super().__init__('smart_camera')
        
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        
        self.last_move_time = 0.0
        self.camera_process = None
        self.is_moving = False
        
        # Check status periodically
        self.timer = self.create_timer(0.5, self.check_camera_state)
        
        self.get_logger().info('Smart Camera Node Started')
        self.get_logger().info(f'Camera will activate on movement and deactivate after {TIMEOUT_SECONDS}s of stillness')

    def cmd_vel_callback(self, msg):
        # Check if there is any movement command
        if abs(msg.linear.x) > 0.001 or abs(msg.angular.z) > 0.001:
            self.last_move_time = time.time()
            self.is_moving = True
        else:
            # If we receive explicit 0 velocity, we are not "moving" in terms of command,
            # but we still rely on the timeout to actually stop the camera.
            # We don't set is_moving = False here immediately because we want the timeout logic to handle it.
            pass

    def check_camera_state(self):
        current_time = time.time()
        time_since_move = current_time - self.last_move_time
        
        # Determine if camera should be on
        # It should be on if we moved recently (within timeout)
        should_be_on = time_since_move < TIMEOUT_SECONDS
        
        if should_be_on:
            if self.camera_process is None:
                self.start_camera()
        else:
            if self.camera_process is not None:
                self.stop_camera()

    def start_camera(self):
        self.get_logger().info('Movement detected: Starting Camera...')
        try:
            # using shell=True to handle the complex command string with quotes
            self.camera_process = subprocess.Popen(CAMERA_CMD, shell=True, preexec_fn=os.setsid)
        except Exception as e:
            self.get_logger().error(f'Failed to start camera: {e}')

    def stop_camera(self):
        self.get_logger().info(f'No movement for {TIMEOUT_SECONDS}s: Stopping Camera...')
        if self.camera_process:
            try:
                # Kill the process group to ensure mjpg_streamer and any child processes die
                os.killpg(os.getpgid(self.camera_process.pid), signal.SIGTERM)
                self.camera_process.wait(timeout=1)
            except Exception as e:
                self.get_logger().error(f'Error stopping camera: {e}')
            
            self.camera_process = None

    def destroy_node(self):
        self.stop_camera()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SmartCamera()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
