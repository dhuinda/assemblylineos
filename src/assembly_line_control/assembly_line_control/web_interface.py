#!/usr/bin/env python3

"""
Web Interface for Assembly Line OS

This serves the web-based user interface. The actual ROS communication
happens through rosbridge, which lets the browser talk directly to ROS 2
over WebSocket connections.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import json
import os
import threading
from ament_index_python.packages import get_package_share_directory
from flask import Flask, render_template
from flask_cors import CORS


class MotorCommandPublisher(Node):
    """Publishes commands to motors and relays (legacy - rosbridge handles this now)"""
    
    def __init__(self):
        super().__init__('motor_command_publisher')
        
        # Set up publishers for motor commands
        self.motor1_pub = self.create_publisher(Int32, 'motor1/command', 10)
        self.motor2_pub = self.create_publisher(Int32, 'motor2/command', 10)
        
        # Set up publisher for relay commands
        self.relay_pub = self.create_publisher(String, 'relay/command', 10)
        
        # Set up publisher for executing entire sequences
        self.sequence_pub = self.create_publisher(String, 'sequence/execute', 10)
        
        self.get_logger().info('Motor command publisher node started')
    
    def publish_motor_command(self, motor_id, steps):
        """Publish a motor command"""
        msg = Int32()
        msg.data = int(steps)
        
        if motor_id == 1:
            self.motor1_pub.publish(msg)
        elif motor_id == 2:
            self.motor2_pub.publish(msg)
        
        self.get_logger().info(f'Published motor {motor_id} command: {steps} steps')
    
    def publish_relay_command(self, relay_id, state):
        """Publish a relay command (on/off)"""
        msg = String()
        msg.data = json.dumps({
            'relay_id': int(relay_id),
            'state': str(state)  # 'on' or 'off'
        })
        self.relay_pub.publish(msg)
        self.get_logger().info(f'Published relay {relay_id} command: {state}')
    
    def publish_sequence(self, sequence):
        """Publish a sequence of commands"""
        msg = String()
        msg.data = json.dumps(sequence)
        self.sequence_pub.publish(msg)
        self.get_logger().info(f'Published sequence with {len(sequence)} commands')


# Global ROS node instance (kept for backward compatibility)
ros_node = None
ros_thread = None


def init_ros():
    """Start up ROS in a background thread"""
    global ros_node, ros_thread
    
    if not rclpy.ok():
        rclpy.init()
    
    ros_node = MotorCommandPublisher()
    
    def spin_ros():
        try:
            rclpy.spin(ros_node)
        except KeyboardInterrupt:
            pass
        finally:
            if ros_node:
                ros_node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    
    ros_thread = threading.Thread(target=spin_ros, daemon=True)
    ros_thread.start()


# Find where the HTML templates and static files (CSS, JS) are located
try:
    package_share_directory = get_package_share_directory('assembly_line_control')
    template_dir = os.path.join(package_share_directory, 'templates')
    static_dir = os.path.join(package_share_directory, 'static')
except Exception as e:
    # If the package isn't installed yet, use relative paths from this file
    template_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'templates')
    static_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'static')
    template_dir = os.path.abspath(template_dir)
    static_dir = os.path.abspath(static_dir)

# Set up Flask to serve the web interface (rosbridge handles all the ROS stuff)
app = Flask(__name__, 
            template_folder=template_dir,
            static_folder=static_dir)
CORS(app)


@app.route('/')
def index():
    """Serve the main web interface"""
    return render_template('index.html')


def main():
    """Start the web server"""
    # Initialize ROS (though rosbridge does most of the work)
    init_ros()
    
    try:
        # Start the web server
        print("Starting web interface on http://localhost:1111")
        print("Using ROS Bridge for ROS 2 communication")
        print("Make sure rosbridge_server is running on ws://localhost:9090")
        print(f"Templates: {template_dir}")
        print(f"Static: {static_dir}")
        app.run(host='0.0.0.0', port=1111, debug=False, use_reloader=False)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        # Clean up when we're done
        if ros_node:
            ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

