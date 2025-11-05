#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraTest(Node):
    def __init__(self):
        super().__init__('camera_test')
        
        self.bridge = CvBridge()
        
        # Subscribe to camera feed
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.get_logger().info("🎥 Camera test node started")
        self.get_logger().info("📷 Listening for camera images on /camera/image_raw")
        
    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Display image info
            height, width, channels = cv_image.shape
            self.get_logger().info(f"📸 Received image: {width}x{height} pixels")
            
            # Show the image
            cv2.imshow("Camera Feed", cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"❌ Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    node = CameraTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 Camera test stopped")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()