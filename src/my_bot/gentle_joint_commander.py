#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math
import time

class GentleJointCommander(Node):
    def __init__(self):
        super().__init__('gentle_joint_commander')
        
        # Publishers for individual joint commands
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
        
        # Timer to send commands every 3 seconds (slower)
        self.timer = self.create_timer(3.0, self.move_robot)
        self.move_count = 0
        
        print("🕊️  Gentle Joint Commander Started!")
        print("Using slower, smaller movements to avoid chaos")
        print("Press Ctrl+C to stop")

    def move_robot(self):
        # Create Float64 messages
        pan_msg = Float64()
        tilt_msg = Float64()
        
        # Create gentle, smaller movements
        if self.move_count % 4 == 0:
            pan_msg.data = 3.14  # Center
            tilt_msg.data = 0.0  # Center
            print(f"🔄 Move {self.move_count + 1}: CENTER - Pan=3.14 rad, Tilt=0.0 rad")
        elif self.move_count % 4 == 1:
            pan_msg.data = 3.64  # +0.5 rad from center
            tilt_msg.data = 0.3   # Small tilt up
            print(f"🔄 Move {self.move_count + 1}: RIGHT/UP - Pan=3.64 rad, Tilt=0.3 rad")
        elif self.move_count % 4 == 2:
            pan_msg.data = 2.64  # -0.5 rad from center  
            tilt_msg.data = -0.3  # Small tilt down
            print(f"🔄 Move {self.move_count + 1}: LEFT/DOWN - Pan=2.64 rad, Tilt=-0.3 rad")
        else:
            pan_msg.data = 3.14  # Back to center
            tilt_msg.data = 0.5   # Moderate tilt up
            print(f"🔄 Move {self.move_count + 1}: CENTER/UP - Pan=3.14 rad, Tilt=0.5 rad")
        
        # Publish the commands
        self.pan_publisher.publish(pan_msg)
        self.tilt_publisher.publish(tilt_msg)
        
        print(f"📤 Gentle commands sent!")
        print(f"   → Pan: {pan_msg.data:.2f} rad ({math.degrees(pan_msg.data):.1f}°)")
        print(f"   → Tilt: {tilt_msg.data:.2f} rad ({math.degrees(tilt_msg.data):.1f}°)")
        print()
        
        self.move_count += 1

def main(args=None):
    rclpy.init(args=args)
    
    try:
        commander = GentleJointCommander()
        
        print("🚀 Starting gentle joint commanding...")
        print("👀 Robot should move smoothly now!")
        print("   - Slower timing (3 seconds between moves)")
        print("   - Smaller movements (±0.5 rad max)")
        print("   - Lower control gains")
        
        rclpy.spin(commander)
        
    except KeyboardInterrupt:
        print("\n🛑 Gentle joint commander stopped")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()