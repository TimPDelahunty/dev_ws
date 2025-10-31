#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
import time

class AutoMover(Node):
    def __init__(self):
        super().__init__('auto_mover')
        
        # Publisher for trajectory controller
        self.publisher = self.create_publisher(
            JointTrajectory, 
            '/pan_tilt_joint_trajectory_controller/joint_trajectory', 
            10
        )
        
        # Timer to send commands every 2 seconds
        self.timer = self.create_timer(2.0, self.move_robot)
        self.move_count = 0
        
        print("🤖 Auto Mover Started - Robot should move automatically!")
        print("Press Ctrl+C to stop")

    def move_robot(self):
        # Create trajectory message
        trajectory_msg = JointTrajectory()
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        trajectory_msg.header.frame_id = ''
        trajectory_msg.joint_names = ['pan_platform_joint', 'tilt_platform_joint']
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        
        # Create simple back-and-forth movement
        if self.move_count % 4 == 0:
            # Move to position 1
            pan_pos = 1.0
            tilt_pos = 0.5
            print(f"🔄 Move {self.move_count + 1}: Pan=1.0 rad, Tilt=0.5 rad")
        elif self.move_count % 4 == 1:
            # Move to position 2
            pan_pos = 2.0
            tilt_pos = -0.3
            print(f"🔄 Move {self.move_count + 1}: Pan=2.0 rad, Tilt=-0.3 rad")
        elif self.move_count % 4 == 2:
            # Move to position 3
            pan_pos = 4.0
            tilt_pos = 0.8
            print(f"🔄 Move {self.move_count + 1}: Pan=4.0 rad, Tilt=0.8 rad")
        else:
            # Return to center
            pan_pos = 3.14
            tilt_pos = 0.0
            print(f"🔄 Move {self.move_count + 1}: Pan=3.14 rad (center), Tilt=0.0 rad")
        
        point.positions = [pan_pos, tilt_pos]
        point.velocities = [0.0, 0.0]
        point.time_from_start = rclpy.duration.Duration(seconds=1.5).to_msg()
        
        trajectory_msg.points = [point]
        
        # Publish the trajectory
        self.publisher.publish(trajectory_msg)
        
        print(f"📤 Command sent! Watch Gazebo for movement...")
        self.move_count += 1

def main(args=None):
    rclpy.init(args=args)
    
    try:
        auto_mover = AutoMover()
        
        print("🚀 Starting automatic robot movement...")
        print("👀 Watch your Gazebo simulation window!")
        print("   - Blue platform should rotate (pan)")
        print("   - Orange platform should tilt up/down")
        print("   - Black camera should move with platforms")
        
        rclpy.spin(auto_mover)
        
    except KeyboardInterrupt:
        print("\n🛑 Auto mover stopped by user")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()