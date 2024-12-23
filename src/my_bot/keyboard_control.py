import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher = self.create_publisher(JointState, '/pan_tilt_joint_controller/commands', 10)
        self.joint_states = JointState()
        self.joint_states.name = ['pan_joint', 'tilt_joint']
        self.joint_states.position = [0.0, 0.0]  # Initial positions
        self.step = 0.1  # Movement increment

    def move_joint(self, pan_delta=0, tilt_delta=0):
        self.joint_states.position[0] += pan_delta
        self.joint_states.position[1] += tilt_delta
        self.joint_states.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.joint_states)

    def key_callback(self, key_input):
        if key_input == 'w':  # Tilt up
            self.move_joint(tilt_delta=self.step)
        elif key_input == 's':  # Tilt down
            self.move_joint(tilt_delta=-self.step)
        elif key_input == 'a':  # Pan left
            self.move_joint(pan_delta=self.step)
        elif key_input == 'd':  # Pan right
            self.move_joint(pan_delta=-self.step)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControl()

    try:
        while True:
            key = input("Enter 'w/a/s/d' to move or 'q' to quit: ")
            if key == 'q':
                break
            node.key_callback(key)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
