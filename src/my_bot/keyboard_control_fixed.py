#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        
        # Publishers for individual joint commands (same as working gentle commander)
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
        
        # Current joint positions (tracking)
        self.pan_position = 3.14  # Start at center (180°)
        self.tilt_position = 0.0  # Start at center (0°)
        
        # Movement step size (radians) - using safe values from gentle commander
        self.step = 0.2
        
        # Joint limits (from your URDF)
        self.pan_min = 0.0
        self.pan_max = 6.28  # 360 degrees
        self.tilt_min = -0.78  # -45 degrees
        self.tilt_max = 1.57   # 90 degrees
        
        print("🎮 === Pan-Tilt Camera Keyboard Control ===")
        print("Using Direct Float64 Commands (Working Method)")
        print("Controls:")
        print("  W/S: Tilt Up/Down")
        print("  A/D: Pan Left/Right")
        print("  R: Reset to center")
        print("  Q: Quit")
        print("  Step size: 0.2 radians (11.5°)")
        print("==========================================")
        
        # Send initial center position
        self.send_current_position()

    def send_current_position(self):
        """Send current position to robot"""
        # Create Float64 messages
        pan_msg = Float64()
        tilt_msg = Float64()
        
        pan_msg.data = self.pan_position
        tilt_msg.data = self.tilt_position
        
        # Publish commands
        self.pan_publisher.publish(pan_msg)
        self.tilt_publisher.publish(tilt_msg)
        
        # Print current positions
        pan_degrees = math.degrees(self.pan_position)
        tilt_degrees = math.degrees(self.tilt_position)
        print(f"📤 Position: Pan={pan_degrees:.1f}° ({self.pan_position:.2f} rad), Tilt={tilt_degrees:.1f}° ({self.tilt_position:.2f} rad)")

    def move_joints(self, pan_delta=0.0, tilt_delta=0.0):
        """Update positions with limits and send commands"""
        # Update positions
        new_pan = self.pan_position + pan_delta
        new_tilt = self.tilt_position + tilt_delta
        
        # Apply joint limits
        self.pan_position = max(self.pan_min, min(self.pan_max, new_pan))
        self.tilt_position = max(self.tilt_min, min(self.tilt_max, new_tilt))
        
        # Check if hit limits
        if new_pan != self.pan_position:
            print(f"⚠️  Pan limit reached!")
        if new_tilt != self.tilt_position:
            print(f"⚠️  Tilt limit reached!")
        
        # Send updated position
        self.send_current_position()

    def reset_position(self):
        """Reset to center position"""
        self.pan_position = 3.14  # Center position for pan
        self.tilt_position = 0.0  # Center position for tilt
        self.send_current_position()
        print("🔄 Reset to center position")

    def handle_key(self, key_input):
        key = key_input.lower().strip()
        
        if key == 'w':  # Tilt up
            print("⬆️  Tilt UP")
            self.move_joints(tilt_delta=self.step)
        elif key == 's':  # Tilt down
            print("⬇️  Tilt DOWN")
            self.move_joints(tilt_delta=-self.step)
        elif key == 'a':  # Pan left
            print("⬅️  Pan LEFT")
            self.move_joints(pan_delta=self.step)
        elif key == 'd':  # Pan right
            print("➡️  Pan RIGHT")
            self.move_joints(pan_delta=-self.step)
        elif key == 'r':  # Reset
            print("🔄 RESET")
            self.reset_position()
        elif key == 'q':  # Quit
            print("👋 QUIT")
            return False
        else:
            print(f"❓ Unknown key: '{key}' - Use W/A/S/D/R/Q")
        
        return True

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControl()

    try:
        print("🚀 Keyboard control ready!")
        print("💡 Tip: Robot should move immediately when you press keys")
        
        while rclpy.ok():
            key = input("\nEnter command (W/A/S/D/R/Q): ")
            if not node.handle_key(key):
                break
                
    except KeyboardInterrupt:
        print("\n🛑 Keyboard control stopped")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()