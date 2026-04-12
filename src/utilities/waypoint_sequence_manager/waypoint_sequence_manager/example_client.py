#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import json

class WaypointSequenceExampleClient(Node):
    """
    Example client demonstrating how to use the waypoint sequence manager.
    This shows how to programmatically send waypoint sequences and control commands.
    """
    
    def __init__(self):
        super().__init__('waypoint_sequence_example_client')
        
        # Publishers
        self.sequence_pub = self.create_publisher(String, '/new_waypoint_sequence', 10)
        self.control_pub = self.create_publisher(String, '/waypoint_sequence_control', 10)
        
        # Subscriber for status
        self.status_sub = self.create_subscription(
            String, '/waypoint_sequence_status', self.status_callback, 10)
        
        self.current_status = "UNKNOWN"
        
        self.get_logger().info('Waypoint Sequence Example Client initialized')
        
    def status_callback(self, msg):
        """Handle status updates from waypoint sequence manager"""
        self.current_status = msg.data
        self.get_logger().info(f'Status update: {msg.data}')
        
    def send_factory_waypoints(self):
        """Send a typical factory waypoint sequence"""
        waypoints = [
            {"x": 2.0, "y": 1.0, "z": 0.0},   # Station 1
            {"x": 5.0, "y": 1.0, "z": 0.0},   # Station 2
            {"x": 5.0, "y": 4.0, "z": 0.0},   # Station 3
            {"x": 2.0, "y": 4.0, "z": 0.0},   # Station 4
        ]
        
        sequence_data = {
            "waypoints": waypoints
        }
        
        msg = String()
        msg.data = json.dumps(sequence_data)
        self.sequence_pub.publish(msg)
        
        self.get_logger().info(f'Sent waypoint sequence with {len(waypoints)} points')
        
    def send_control_command(self, command):
        """Send control command to waypoint sequence manager"""
        msg = String()
        msg.data = command
        self.control_pub.publish(msg)
        self.get_logger().info(f'Sent control command: {command}')
        
    def run_example_sequence(self):
        """Run a complete example sequence"""
        self.get_logger().info('Starting waypoint sequence example...')
        
        # Send waypoints
        time.sleep(1.0)  # Give time for setup
        self.send_factory_waypoints()
        
        # Wait for sequence to be loaded
        time.sleep(2.0)
        
        # Start the sequence
        self.send_control_command('START')
        
        # Let it run for a while
        time.sleep(10.0)
        
        # Pause
        self.send_control_command('PAUSE')
        time.sleep(3.0)
        
        # Resume
        self.send_control_command('RESUME')
        time.sleep(5.0)
        
        # Stop
        self.send_control_command('STOP')
        
        self.get_logger().info('Example sequence completed')

def main(args=None):
    rclpy.init(args=args)
    
    client = WaypointSequenceExampleClient()
    
    # Run the example after a short delay
    timer = client.create_timer(2.0, client.run_example_sequence)
    timer.cancel()  # One-shot timer
    
    try:
        # Run example after 2 seconds
        client.create_timer(2.0, lambda: [client.run_example_sequence(), client.destroy_node()])
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()