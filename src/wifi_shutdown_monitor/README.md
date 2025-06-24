# WiFi Shutdown Monitor

A ROS2 package for TurtleBot3 that monitors WiFi connectivity and performs a safe shutdown when the connection to a specified network is lost for an extended period.

## Overview

This package provides a safety mechanism for autonomous robots operating in WiFi-dependent environments. When the robot loses connection to its designated WiFi network for more than a configurable timeout period, it will automatically initiate a safe shutdown to prevent uncontrolled behaviour or potential damage.

## Features

- **Continuous WiFi monitoring**: Periodically checks connection to a specified SSID
- **Configurable timeout**: Customisable disconnection timeout before shutdown
- **Safe shutdown sequence**: Graceful system shutdown with logging
- **ROS2 integration**: Built as a proper ROS2 node with standard logging

## Hardware Requirements

- TurtleBot3 with Raspberry Pi 4
- Ubuntu 22.04
- ROS2 Jazzy
- WiFi adapter (built-in or USB)

## Installation

1. Navigate to your ROS2 workspace source directory:
   ```bash
   cd ~/your_ros2_ws/src
   ```

2. Copy the `wifi_shutdown_monitor` package to your workspace

3. Install dependencies:
   ```bash
   cd ~/your_ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. Build the package:
   ```bash
   colcon build --packages-select wifi_shutdown_monitor
   ```

5. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Configuration

The WiFi monitor can be configured by modifying the parameters in `wifi_monitor_node.py`:

- **`ssid`**: The target WiFi network name (default: "TurtleNet")
- **`timeout`**: Disconnection timeout in seconds (default: 300 seconds / 5 minutes)
- **Check interval**: WiFi connection check frequency (default: 10 seconds)

### Customising Parameters

Edit the `__init__` method in `wifi_shutdown_monitor/wifi_monitor_node.py`:

```python
self.ssid = "YourNetworkName"      # Change to your WiFi network
self.timeout = 180                 # Change timeout (in seconds)
```

## Usage

### Running the Node

Start the WiFi monitor node:

```bash
ros2 run wifi_shutdown_monitor wifi_monitor
```

### Using the Launch File

Alternatively, use the provided launch file:

```bash
ros2 launch wifi_shutdown_monitor wifi_monitor.launch.py
```

### Integration with TurtleBot3 Launch

To automatically start the WiFi monitor with your TurtleBot3, add it to your main launch file:

```python
# Add to your existing launch file
Node(
    package='wifi_shutdown_monitor',
    executable='wifi_monitor',
    name='wifi_monitor_node',
    output='screen',
),
```

## Behaviour

1. **Normal Operation**: The node checks WiFi connectivity every 10 seconds
2. **Connection Lost**: When disconnected, it logs warnings with time since last connection
3. **Timeout Exceeded**: After the configured timeout period, it initiates shutdown
4. **Shutdown Process**: Logs the shutdown event and executes `sudo shutdown now`

## Logging

The node provides comprehensive logging:

- **Info logs**: Successful connections and node startup
- **Warning logs**: Connection loss with time tracking  
- **Error logs**: Timeout exceeded and shutdown initiation

## Security Considerations

The shutdown command requires sudo privileges. Ensure your robot user has passwordless sudo access for the shutdown command:

```bash
# Add to /etc/sudoers (use visudo)
your_username ALL=(ALL) NOPASSWD: /sbin/shutdown
```

## Troubleshooting

### Common Issues

1. **Permission denied for shutdown**:
   - Configure passwordless sudo for shutdown command
   - Verify user has appropriate privileges

2. **iwgetid command not found**:
   ```bash
   sudo apt install wireless-tools
   ```

3. **Node fails to detect WiFi**:
   - Verify WiFi adapter is recognised: `iwconfig`
   - Check network interface status: `ip link show`

4. **Incorrect SSID detection**:
   - Manually test: `iwgetid -r`
   - Ensure the robot is connected to the expected network

### Testing

Test the WiFi detection manually:
```bash
# Check current SSID
iwgetid -r

# Test the monitoring logic
ros2 run wifi_shutdown_monitor wifi_monitor
```

## Safety Notes

- **Test thoroughly** in a safe environment before deployment
- Consider implementing **additional safety checks** before shutdown
- **Backup important data** regularly as shutdown is immediate
- **Monitor logs** to understand connection patterns

## Customisation

### Adding Pre-shutdown Procedures

Modify the `shutdown_robot()` method to add custom cleanup:

```python
def shutdown_robot(self) -> None:
    """Initiate a safe shutdown sequence for the robot."""
    self.get_logger().info("Initiating shutdown sequence...")
    
    # Add your custom cleanup here:
    # - Stop all motors
    # - Save sensor data
    # - Send status messages
    # - Close file handles
    
    os.system("sudo shutdown now")
```

### Integration with Other Nodes

The monitor can be extended to:
- Publish status messages to other nodes
- Subscribe to robot state for conditional shutdown
- Coordinate with navigation or mission planning systems

## License

TODO: Add appropriate license information

## Contributing

When contributing to this package:
- Maintain Australian spelling in comments
- Follow ROS2 Python coding standards
- Test changes thoroughly in simulation first
- Update this README for any new features

## Support

For issues related to:
- **TurtleBot3 setup**: Refer to official TurtleBot3 documentation
- **ROS2 Jazzy**: Check ROS2 community forums
- **WiFi configuration**: Consult Ubuntu networking guides