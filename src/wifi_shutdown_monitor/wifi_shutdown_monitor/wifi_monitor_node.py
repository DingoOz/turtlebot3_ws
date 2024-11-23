"""WiFi connection monitor for TurtleBot3.

This module monitors the WiFi connection to a specific SSID and initiates
a safe shutdown if the connection is lost for an extended period.
"""

import os
import subprocess
import time

import rclpy
from rclpy.node import Node


class WifiMonitorNode(Node):
    """Monitor WiFi connection and handle connection loss.

    This node periodically checks the connection to a specific WiFi network
    and initiates a safe shutdown if the connection is lost for too long.
    """

    def __init__(self) -> None:
        """Initialize the WiFi monitor node.

        Sets up the monitoring parameters and creates a timer for periodic checks.
        """
        super().__init__("wifi_monitor_node")
        self.ssid = "TurtleNet"
        self.timeout = 300  # 5 minutes in seconds
        self.last_connected_time = time.time()
        self.timer = self.create_timer(10, self.check_wifi_connection)
        self.get_logger().info("WiFi Monitor Node has started.")

    def check_wifi_connection(self) -> None:
        """Check WiFi connection status and handle disconnections.

        Verifies connection to the specified SSID and initiates shutdown
        if connection has been lost for longer than the timeout period.
        """
        if self.is_connected_to_ssid():
            self.last_connected_time = time.time()
            self.get_logger().info(f"Connected to {self.ssid}")
        else:
            time_since_last_connection = time.time() - self.last_connected_time
            self.get_logger().warn(
                f"Not connected to {self.ssid}. Time since last connection: "
                f"{time_since_last_connection:.2f} seconds"
            )
            if time_since_last_connection > self.timeout:
                self.get_logger().error(
                    f"WiFi connection to {self.ssid} lost for more than "
                    f"{self.timeout} seconds. Shutting down..."
                )
                self.shutdown_robot()

    def is_connected_to_ssid(self) -> bool:
        """Check if connected to the specified SSID.

        Returns:
            bool: True if connected to the target SSID, False otherwise.
        """
        try:
            result = subprocess.run(
                ["iwgetid", "-r"],
                capture_output=True,
                text=True,
                check=False
            )
            return result.stdout.strip() == self.ssid
        except subprocess.CalledProcessError:
            return False

    def shutdown_robot(self) -> None:
        """Initiate a safe shutdown sequence for the robot.

        Logs the shutdown event and executes the system shutdown command.
        Any cleanup procedures should be added here before shutdown.
        """
        self.get_logger().info("Initiating shutdown sequence...")
        # Add any cleanup or safe shutdown procedures here
        # For example, you might want to stop all motors, save any important data, etc.
        os.system("sudo shutdown now")


def main(args=None) -> None:
    """Run the WiFi monitor node.

    Args:
        args: Command line arguments (optional).
    """
    rclpy.init(args=args)
    wifi_monitor = WifiMonitorNode()
    rclpy.spin(wifi_monitor)
    wifi_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    
