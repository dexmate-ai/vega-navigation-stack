#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Bool, String
from joy_command_mapper.msg import RobotCommand
from visualization_msgs.msg import MarkerArray, Marker
import math
import json
import time
import signal
import sys

class WaypointSequenceManager(Node):
    def __init__(self):
        super().__init__('waypoint_sequence_manager')
        
        # Parameters
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('sequence_topic', '/waypoint_sequence')
        self.declare_parameter('new_sequence_topic', '/new_waypoint_sequence')
        self.declare_parameter('sequence_control_topic', '/waypoint_sequence_control')
        
        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        sequence_topic = self.get_parameter('sequence_topic').get_parameter_value().string_value
        new_sequence_topic = self.get_parameter('new_sequence_topic').get_parameter_value().string_value
        sequence_control_topic = self.get_parameter('sequence_control_topic').get_parameter_value().string_value
        
        # State variables
        self.waypoint_sequence = []
        self.current_waypoint_index = 0
        self.robot_position = None
        self.is_active = False
        self.sequence_id = 0
        
        # Publishers
        self.waypoint_pub = self.create_publisher(PointStamped, '/way_point', 10)
        self.robot_cmd_pub = self.create_publisher(RobotCommand, '/robot_command', 10)
        self.status_pub = self.create_publisher(String, '/waypoint_sequence_status', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint_sequence_markers', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/state_estimation', self.odom_callback, 10)
        self.new_sequence_sub = self.create_subscription(
            String, new_sequence_topic, self.new_sequence_callback, 10)
        self.control_sub = self.create_subscription(
            String, sequence_control_topic, self.control_callback, 10)
        
        # Timer for waypoint management
        self.timer = self.create_timer(0.5, self.waypoint_management_callback)
        
        self.get_logger().info('Waypoint Sequence Manager initialized')
        self.publish_status("IDLE")
        
    def odom_callback(self, msg):
        """Update robot position from odometry"""
        self.robot_position = msg.pose.pose.position
        
    def new_sequence_callback(self, msg):
        """Handle new waypoint sequence"""
        try:
            data = json.loads(msg.data)
            waypoints = data.get('waypoints', [])
            
            if waypoints:
                self.waypoint_sequence = []
                for wp in waypoints:
                    point = PointStamped()
                    point.header.frame_id = 'map'
                    point.header.stamp = self.get_clock().now().to_msg()
                    point.point.x = wp['x']
                    point.point.y = wp['y']
                    point.point.z = wp['z']
                    self.waypoint_sequence.append(point)
                
                self.current_waypoint_index = 0
                self.sequence_id += 1
                self.publish_waypoint_markers()
                self.get_logger().info(f'New waypoint sequence loaded: {len(self.waypoint_sequence)} waypoints')
                self.publish_status("SEQUENCE_LOADED")
            else:
                self.get_logger().warn('Received empty waypoint sequence')
                
        except Exception as e:
            self.get_logger().error(f'Failed to parse waypoint sequence: {str(e)}')
            
    def control_callback(self, msg):
        """Handle control commands"""
        command = msg.data.upper()
        
        if command == "START":
            if self.waypoint_sequence:
                self.is_active = True
                self.current_waypoint_index = 0
                self.publish_current_waypoint()
                self.get_logger().info('Waypoint sequence started')
                self.publish_status("ACTIVE")
            else:
                self.get_logger().warn('No waypoint sequence loaded')
                self.publish_status("NO_SEQUENCE")
                
        elif command == "STOP":
            self.is_active = False
            self.get_logger().info('Waypoint sequence stopped')
            self.publish_status("STOPPED")
            
        elif command == "PAUSE":
            self.is_active = False
            self.get_logger().info('Waypoint sequence paused')
            self.publish_status("PAUSED")
            
        elif command == "RESUME":
            if self.waypoint_sequence:
                self.is_active = True
                self.publish_current_waypoint()
                self.get_logger().info('Waypoint sequence resumed')
                self.publish_status("ACTIVE")
            else:
                self.get_logger().warn('No waypoint sequence to resume')
                self.publish_status("NO_SEQUENCE")
                
        elif command == "RESET":
            self.current_waypoint_index = 0
            self.is_active = False
            self.get_logger().info('Waypoint sequence reset')
            self.publish_status("RESET")
            
        elif command == "NEXT":
            if self.waypoint_sequence and self.current_waypoint_index < len(self.waypoint_sequence) - 1:
                self.current_waypoint_index += 1
                if self.is_active:
                    self.publish_current_waypoint()
                self.get_logger().info(f'Advanced to waypoint {self.current_waypoint_index + 1}')
                self.publish_status("ADVANCED")
            else:
                self.get_logger().warn('Cannot advance waypoint')
    
    def waypoint_management_callback(self):
        """Main waypoint management logic"""
        if not self.is_active or not self.waypoint_sequence or self.robot_position is None:
            return
            
        current_waypoint = self.waypoint_sequence[self.current_waypoint_index]
        
        # Calculate distance to current waypoint
        distance = math.sqrt(
            (self.robot_position.x - current_waypoint.point.x) ** 2 +
            (self.robot_position.y - current_waypoint.point.y) ** 2
        )
        
        # Check if we reached the current waypoint
        if distance < self.goal_tolerance:
            self.get_logger().info(f'Reached waypoint {self.current_waypoint_index + 1}/{len(self.waypoint_sequence)}')
            
            # Move to next waypoint or loop back to start
            self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoint_sequence)
            
            # Publish next waypoint
            self.publish_current_waypoint()
            
            if self.current_waypoint_index == 0:
                self.get_logger().info('Completed full sequence, looping back to start')
                self.publish_status("SEQUENCE_COMPLETED")
            
            # Update markers
            self.publish_waypoint_markers()
    
    def publish_current_waypoint(self):
        """Publish current waypoint and issue robot command trigger"""
        if self.waypoint_sequence and self.current_waypoint_index < len(self.waypoint_sequence) and self.robot_position is not None:
            # Publish robot command equivalent to previous joystick trigger
            command_stamp = self.get_clock().now().to_msg()
            robot_cmd = RobotCommand()
            robot_cmd.header.stamp = command_stamp
            robot_cmd.header.frame_id = "waypoint_sequence_manager"
            robot_cmd.source = "waypoint_sequence_manager"
            robot_cmd.vx = 1.0
            robot_cmd.vy = 0.0
            robot_cmd.wz = 0.0
            robot_cmd.autonomy_mode = True
            robot_cmd.manual_mode = False
            robot_cmd.obstacle_check_enabled = True
            robot_cmd.clear_cloud = False
            robot_cmd.emergency_stop = False
            robot_cmd.deadman_switch = 1.0
            self.robot_cmd_pub.publish(robot_cmd)

            # Create waypoint with robot's current Z (same as waypoint_rviz_plugin)
            waypoint = PointStamped()
            waypoint.header.frame_id = "map"
            waypoint.header.stamp = command_stamp

            # Use stored X,Y but robot's current Z to avoid vertical movement
            stored_waypoint = self.waypoint_sequence[self.current_waypoint_index]
            waypoint.point.x = stored_waypoint.point.x
            waypoint.point.y = stored_waypoint.point.y
            waypoint.point.z = self.robot_position.z  # Use robot's current Z position

            self.waypoint_pub.publish(waypoint)

            # Small delay and publish waypoint again (same as waypoint_rviz_plugin)
            time.sleep(0.01)
            self.waypoint_pub.publish(waypoint)

            self.get_logger().info(f'Published waypoint {self.current_waypoint_index + 1}/{len(self.waypoint_sequence)}: '
                                 f'({waypoint.point.x:.2f}, {waypoint.point.y:.2f}, {waypoint.point.z:.2f}) [Z from robot current position]')
    
    def publish_status(self, status):
        """Publish current status"""
        status_msg = String()
        status_msg.data = f"{status}|{len(self.waypoint_sequence)}|{self.current_waypoint_index + 1}|{self.sequence_id}"
        self.status_pub.publish(status_msg)
    
    def publish_waypoint_markers(self):
        """Publish visualization markers for waypoints"""
        marker_array = MarkerArray()
        
        # Clear existing markers
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
        # Add markers for each waypoint
        for i, waypoint in enumerate(self.waypoint_sequence):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'waypoint_sequence'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position = waypoint.point
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            
            # Color coding: current waypoint in red, others in blue
            if i == self.current_waypoint_index:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            marker.color.a = 0.8
            
            marker_array.markers.append(marker)
            
            # Add text marker with waypoint number
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = 'waypoint_numbers'
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position = waypoint.point
            text_marker.pose.position.z += 0.5  # Offset text above the sphere
            text_marker.pose.orientation.w = 1.0
            
            text_marker.scale.z = 0.3
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            text_marker.text = str(i + 1)
            marker_array.markers.append(text_marker)
        
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    
    waypoint_sequence_manager = WaypointSequenceManager()
    
    def signal_handler(sig, frame):
        waypoint_sequence_manager.get_logger().info('Shutting down waypoint sequence manager...')
        waypoint_sequence_manager.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    
    # Register signal handlers for clean shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        rclpy.spin(waypoint_sequence_manager)
    except KeyboardInterrupt:
        waypoint_sequence_manager.get_logger().info('Keyboard interrupt received')
    except Exception as e:
        waypoint_sequence_manager.get_logger().error(f'Unexpected error: {str(e)}')
    finally:
        # Ensure cleanup happens
        try:
            waypoint_sequence_manager.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
