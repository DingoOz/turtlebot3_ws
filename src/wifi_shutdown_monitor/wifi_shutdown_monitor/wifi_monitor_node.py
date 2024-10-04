import rclpy
from rclpy.node import Node
import subprocess
import time
import os

class WifiMonitorNode(Node):
    def __init__(self):
        super().__init__('wifi_monitor_node')
        self.ssid = "TurtleNet"
        self.timeout = 300  # 5 minutes in seconds
        self.last_connected_time = time.time()
        
        self.timer = self.create_timer(10, self.check_wifi_connection)
        self.get_logger().info('WiFi Monitor Node has started.')

    def check_wifi_connection(self):
        if self.is_connected_to_ssid():
            self.last_connected_time = time.time()
            self.get_logger().info(f'Connected to {self.ssid}')
        else:
            time_since_last_connection = time.time() - self.last_connected_time
            self.get_logger().warn(f'Not connected to {self.ssid}. Time since last connection: {time_since_last_connection:.2f} seconds')
            
            if time_since_last_connection > self.timeout:
                self.get_logger().error(f'WiFi connection to {self.ssid} lost for more than {self.timeout} seconds. Shutting down...')
                self.shutdown_robot()

    def is_connected_to_ssid(self):
        try:
            result = subprocess.run(['iwgetid', '-r'], capture_output=True, text=True)
            return result.stdout.strip() == self.ssid
        except subprocess.CalledProcessError:
            return False

    def shutdown_robot(self):
        self.get_logger().info('Initiating shutdown sequence...')
        # Add any cleanup or safe shutdown procedures here
        # For example, you might want to stop all motors, save any important data, etc.
        
        # Finally, shut down the system
        os.system('sudo shutdown now')

def main(args=None):
    rclpy.init(args=args)
    wifi_monitor = WifiMonitorNode()
    rclpy.spin(wifi_monitor)
    wifi_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
