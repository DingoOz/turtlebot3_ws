import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class XboxTeleopNode(Node):
    def __init__(self):
        super().__init__('xbox_teleop_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        self.subscription  # prevent unused variable warning

    def joy_callback(self, joy_msg):
        twist = Twist()
        
        # Check if Axis 5 (right trigger) is fully pressed (-1.0)
        dead_man_switch = joy_msg.axes[5] == -1.0
        
        if dead_man_switch:
            # Left stick Y-axis for linear velocity
            twist.linear.x = joy_msg.axes[1] * 0.5  # Scale down to 0.5 m/s max
            # Left stick X-axis for angular velocity
            twist.angular.z = joy_msg.axes[0] * 1.5  # Scale to 1.5 rad/s max
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # Display Xbox controller axes data
        self.get_logger().info('Xbox Controller Axes:')
        for i, axis in enumerate(joy_msg.axes):
            self.get_logger().info(f'  Axis {i}: {axis:.2f}')
        
        # Display Twist output
        self.get_logger().info('Twist Output:')
        self.get_logger().info(f'  Linear X: {twist.linear.x:.2f} m/s')
        self.get_logger().info(f'  Angular Z: {twist.angular.z:.2f} rad/s')
        self.get_logger().info(f'  Dead Man\'s Switch: {"Engaged" if dead_man_switch else "Disengaged"}')
        
        self.get_logger().info('------------------------')
        
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    xbox_teleop_node = XboxTeleopNode()
    rclpy.spin(xbox_teleop_node)
    xbox_teleop_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
