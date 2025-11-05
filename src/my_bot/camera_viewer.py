#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import subprocess
import time

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        
        # Check if camera topic exists
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.frame_count = 0
        self.start_time = time.time()
        
        self.get_logger().info("🎥 Camera Viewer Started")
        self.get_logger().info("📷 Monitoring /camera/image_raw")
        self.get_logger().info("💡 Tip: Use 'ros2 run rqt_image_view rqt_image_view' to see the camera feed visually")
        
    def image_callback(self, msg):
        self.frame_count += 1
        
        if self.frame_count % 10 == 0:  # Print every 10th frame
            elapsed = time.time() - self.start_time
            fps = self.frame_count / elapsed if elapsed > 0 else 0
            
            self.get_logger().info(f"📸 Frame {self.frame_count}: {msg.width}x{msg.height}, {fps:.1f} FPS")

def main(args=None):
    rclpy.init(args=args)
    
    node = CameraViewer()
    
    try:
        print("🎥 Camera Viewer Started!")
        print("📺 To see the actual camera feed, run in another terminal:")
        print("   ros2 run rqt_image_view rqt_image_view")
        print("   Then select '/camera/image_raw' topic")
        print()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 Camera viewer stopped")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()