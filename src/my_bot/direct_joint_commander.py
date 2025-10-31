#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math
import time

class DirectJointCommander(Node):
    def __init__(self):
        super().__init__('direct_joint_commander')
        
        # Publishers for individual joint commands (what the bridge expects)
        self.pan_publisher = self.create_publisher(
            Float64, 
            '/pan_platform_joint/command', 
            10
        )
        
        self.tilt_publisher = self.create_publisher(
            Float64, 
            '/tilt_platform_joint/command', 
            10
        )
        
        # Timer to send commands every 2 seconds
        self.timer = self.create_timer(2.0, self.move_robot)
        self.move_count = 0
        
        print("🎯 Direct Joint Commander Started!")
        print("Publishing Float64 commands directly to bridge topics")
        print("Press Ctrl+C to stop")

    def move_robot(self):
        # Create Float64 messages
        pan_msg = Float64()
        tilt_msg = Float64()
        
        # Create simple back-and-forth movement
        if self.move_count % 4 == 0:
            pan_msg.data = 1.0
            tilt_msg.data = 0.5
            print(f"🔄 Move {self.move_count + 1}: Pan=1.0 rad, Tilt=0.5 rad")
        elif self.move_count % 4 == 1:
            pan_msg.data = 2.0
            tilt_msg.data = -0.3
            print(f"🔄 Move {self.move_count + 1}: Pan=2.0 rad, Tilt=-0.3 rad")
        elif self.move_count % 4 == 2:
            pan_msg.data = 4.0
            tilt_msg.data = 0.8
            print(f"🔄 Move {self.move_count + 1}: Pan=4.0 rad, Tilt=0.8 rad")
        else:
            pan_msg.data = 3.14
            tilt_msg.data = 0.0
            print(f"🔄 Move {self.move_count + 1}: Pan=3.14 rad (center), Tilt=0.0 rad")
        
        # Publish the commands
        self.pan_publisher.publish(pan_msg)
        self.tilt_publisher.publish(tilt_msg)
        
        print(f"📤 Float64 commands sent to bridge topics!")
        print(f"   → /pan_platform_joint/command: {pan_msg.data}")
        print(f"   → /tilt_platform_joint/command: {tilt_msg.data}")
        
        self.move_count += 1

def main(args=None):
    rclpy.init(args=args)
    
    try:
        commander = DirectJointCommander()
        
        print("🚀 Starting direct joint commanding...")
        print("👀 Watch your Gazebo simulation window!")
        print("   - This bypasses trajectory controller")
        print("   - Sends Float64 commands directly to bridge")
        
        rclpy.spin(commander)
        
    except KeyboardInterrupt:
        print("\n🛑 Direct joint commander stopped")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()