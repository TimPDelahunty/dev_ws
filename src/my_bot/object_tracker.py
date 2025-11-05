#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class ObjectTracker(Node):
    def __init__(self):
        super().__init__('object_tracker')
        
        self.bridge = CvBridge()
        
        # Camera subscription
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_raw', 
            self.image_callback,
            10
        )
        
        # Joint command publishers (same as keyboard control)
        self.pan_pub = self.create_publisher(Float64, '/pan_platform_joint/command', 10)
        self.tilt_pub = self.create_publisher(Float64, '/tilt_platform_joint/command', 10)
        
        # Current joint positions
        self.pan_position = 3.14  # Center
        self.tilt_position = 0.0  # Center
        
        # Movement parameters
        self.tracking_step = 0.1  # Smaller steps for smooth tracking
        self.pan_min, self.pan_max = 0.0, 6.28
        self.tilt_min, self.tilt_max = -0.78, 1.57
        
        # Object detection parameters
        self.target_color_lower = np.array([0, 100, 100])    # Orange/Red lower bound (HSV)
        self.target_color_upper = np.array([20, 255, 255])   # Orange/Red upper bound (HSV)
        
        # Image center for tracking reference
        self.image_center_x = 320  # Will be updated with actual image size
        self.image_center_y = 240
        
        # Tracking deadzone (pixels) - no movement if object is in center
        self.deadzone = 50
        
        self.get_logger().info("🎯 Object Tracker Started")
        self.get_logger().info("🔍 Looking for orange/red objects")
        self.get_logger().info("📷 Camera feed: /camera/image_raw")
        
    def send_joint_commands(self):
        """Send current positions to joints"""
        pan_msg = Float64()
        tilt_msg = Float64()
        pan_msg.data = self.pan_position
        tilt_msg.data = self.tilt_position
        
        self.pan_pub.publish(pan_msg)
        self.tilt_pub.publish(tilt_msg)
        
    def move_to_track_object(self, object_x, object_y):
        """Move camera to center object in frame"""
        # Calculate error from center
        error_x = object_x - self.image_center_x
        error_y = object_y - self.image_center_y
        
        # Check if object is within deadzone
        if abs(error_x) < self.deadzone and abs(error_y) < self.deadzone:
            return  # No movement needed
        
        # Calculate movement needed (proportional control)
        pan_delta = 0
        tilt_delta = 0
        
        if abs(error_x) > self.deadzone:
            # Pan movement (left/right)
            pan_delta = -self.tracking_step if error_x > 0 else self.tracking_step
            
        if abs(error_y) > self.deadzone:
            # Tilt movement (up/down) 
            tilt_delta = self.tracking_step if error_y > 0 else -self.tracking_step
        
        # Apply movement with limits
        new_pan = self.pan_position + pan_delta
        new_tilt = self.tilt_position + tilt_delta
        
        self.pan_position = max(self.pan_min, min(self.pan_max, new_pan))
        self.tilt_position = max(self.tilt_min, min(self.tilt_max, new_tilt))
        
        # Send commands
        self.send_joint_commands()
        
        self.get_logger().info(f"🎯 Tracking: Object at ({object_x},{object_y}) -> Pan: {math.degrees(self.pan_position):.1f}°, Tilt: {math.degrees(self.tilt_position):.1f}°")
        
    def detect_object(self, cv_image):
        """Detect orange/red objects in image"""
        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Create mask for target color
        mask = cv2.inRange(hsv, self.target_color_lower, self.target_color_upper)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Find largest contour (biggest object)
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Get bounding box
            x, y, w, h = cv2.boundingRect(largest_contour)
            
            # Calculate center of object
            object_center_x = x + w // 2
            object_center_y = y + h // 2
            
            # Draw detection on image
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(cv_image, (object_center_x, object_center_y), 10, (0, 255, 0), -1)
            cv2.putText(cv_image, f"Target ({object_center_x},{object_center_y})", 
                       (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            return object_center_x, object_center_y
        
        return None, None
        
    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Update image dimensions
            height, width = cv_image.shape[:2]
            self.image_center_x = width // 2
            self.image_center_y = height // 2
            
            # Detect objects
            object_x, object_y = self.detect_object(cv_image)
            
            # Draw crosshairs at center
            cv2.line(cv_image, (self.image_center_x - 20, self.image_center_y), 
                    (self.image_center_x + 20, self.image_center_y), (255, 255, 255), 2)
            cv2.line(cv_image, (self.image_center_x, self.image_center_y - 20), 
                    (self.image_center_x, self.image_center_y + 20), (255, 255, 255), 2)
            
            # Draw deadzone
            cv2.rectangle(cv_image, 
                         (self.image_center_x - self.deadzone, self.image_center_y - self.deadzone),
                         (self.image_center_x + self.deadzone, self.image_center_y + self.deadzone),
                         (255, 255, 0), 1)
            
            # Track object if detected
            if object_x is not None and object_y is not None:
                self.move_to_track_object(object_x, object_y)
            
            # Display image
            cv2.imshow("Object Tracker", cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"❌ Error in image processing: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    node = ObjectTracker()
    
    try:
        print("🎯 Object Tracker Started!")
        print("📍 The camera will automatically track orange/red objects")
        print("🎮 Use Ctrl+C to stop")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 Object tracker stopped")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()