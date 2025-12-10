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
import socket
from ament_index_python.packages import get_package_share_directory
from flask import Flask, render_template, request, jsonify
from flask_cors import CORS
from pathlib import Path


class MotorCommandPublisher(Node):
    """Publishes commands to motors and relays (legacy - rosbridge handles this now)"""
    
    def __init__(self):
        super().__init__('motor_command_publisher')
        
        # Declare parameters for ROS Bridge configuration
        self.declare_parameter('rosbridge_host', '')
        self.declare_parameter('rosbridge_port', 9090)
        
        # Set up publishers for motor commands
        self.motor1_pub = self.create_publisher(Int32, 'motor1/command', 10)
        self.motor2_pub = self.create_publisher(Int32, 'motor2/command', 10)
        
        # Set up publisher for relay commands
        self.relay_pub = self.create_publisher(String, 'relay/command', 10)
        
        # Set up publisher for executing entire sequences
        self.sequence_pub = self.create_publisher(String, 'sequence/execute', 10)
        
        # Set up publisher for Arduino reconnect requests
        self.reconnect_pub = self.create_publisher(String, 'arduino/reconnect', 10)
        
        self.get_logger().info('Motor command publisher node started')
    
    def get_rosbridge_config(self):
        """Get ROS Bridge host and port from parameters"""
        try:
            rosbridge_host_param = self.get_parameter('rosbridge_host')
            rosbridge_host = rosbridge_host_param.get_parameter_value().string_value or ''
        except Exception:
            rosbridge_host = ''
        
        try:
            rosbridge_port_param = self.get_parameter('rosbridge_port')
            port_value = rosbridge_port_param.get_parameter_value()
            
            # Try integer first
            if hasattr(port_value, 'integer_value'):
                try:
                    rosbridge_port = port_value.integer_value
                    if rosbridge_port:
                        return rosbridge_host, rosbridge_port
                except (AttributeError, TypeError):
                    pass
            
            # Try string (LaunchConfiguration might pass as string)
            if hasattr(port_value, 'string_value'):
                try:
                    port_str = port_value.string_value
                    if port_str:
                        rosbridge_port = int(port_str)
                        return rosbridge_host, rosbridge_port
                except (ValueError, TypeError, AttributeError):
                    pass
            
            # Default
            rosbridge_port = 9090
        except Exception:
            rosbridge_port = 9090
            
        return rosbridge_host, rosbridge_port
    
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

# Global ROS Bridge configuration
rosbridge_host = None
rosbridge_port = 9090


def init_ros():
    """Start up ROS in a background thread"""
    global ros_node, ros_thread, rosbridge_host, rosbridge_port
    
    if not rclpy.ok():
        rclpy.init()
    
    ros_node = MotorCommandPublisher()
    
    # Read ROS Bridge configuration from parameters after a short delay
    # to ensure parameters are fully loaded
    def read_params_after_init():
        import time
        time.sleep(0.1)  # Brief delay for parameters to be available
        global rosbridge_host, rosbridge_port
        try:
            host_param, port_param = ros_node.get_rosbridge_config()
            if host_param:
                rosbridge_host = host_param
            if port_param:
                rosbridge_port = port_param
        except Exception as e:
            # Parameters might not be available yet, that's okay
            pass
    
    # Read params in background
    threading.Thread(target=read_params_after_init, daemon=True).start()
    
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


def get_rosbridge_url():
    """
    Determine the ROS Bridge WebSocket URL.
    
    This function determines the correct ROS Bridge URL based on:
    1. ROS parameters (if available)
    2. Environment variables (ROS_BRIDGE_HOST, ROS_BRIDGE_PORT)
    3. Request host (for remote access)
    4. Default to localhost
    
    Returns:
        str: WebSocket URL for ROS Bridge (e.g., 'ws://192.168.1.100:9090')
    """
    global rosbridge_host, rosbridge_port
    
    # Try to get host from environment variable
    env_host = os.environ.get('ROS_BRIDGE_HOST', '')
    env_port = os.environ.get('ROS_BRIDGE_PORT', '')
    
    # Use ROS parameters if available (they override env vars)
    host = rosbridge_host if rosbridge_host else env_host
    # Port: use ROS param if it exists, otherwise env var, otherwise default
    if rosbridge_port is not None:
        port = rosbridge_port
    elif env_port:
        try:
            port = int(env_port)
        except ValueError:
            port = 9090
    else:
        port = 9090
    
    # If no explicit host is configured, determine from request
    if not host:
        try:
            # Get the host from the current request
            request_host = request.host.split(':')[0]  # Remove port if present
            # If it's not localhost/127.0.0.1, use it
            if request_host not in ['localhost', '127.0.0.1']:
                host = request_host
            else:
                # For localhost access, try to get the actual server IP
                # This handles the case where server is on a remote machine
                # accessed via its IP address
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                try:
                    # Connect to a remote address (doesn't actually send data)
                    s.connect(('8.8.8.8', 80))
                    host = s.getsockname()[0]
                except Exception:
                    host = 'localhost'
                finally:
                    s.close()
        except Exception:
            # If we can't determine host, default to localhost
            host = 'localhost'
    
    # Build the WebSocket URL
    protocol = 'ws'
    if os.environ.get('ROS_BRIDGE_SECURE', '').lower() in ('true', '1', 'yes'):
        protocol = 'wss'
    
    return f'{protocol}://{host}:{port}'


@app.route('/')
def index():
    """Serve the main web interface"""
    rosbridge_url = get_rosbridge_url()
    return render_template('index.html', rosbridge_url=rosbridge_url)


def get_settings_path():
    """Get the path to the pin configuration file."""
    # Use ~/.assembly_line_os directory for configuration
    config_dir = Path.home() / '.assembly_line_os'
    config_dir.mkdir(parents=True, exist_ok=True)
    return config_dir / 'pin_config.json'


def get_default_settings():
    """Get default pin configuration."""
    return {
        "motors": [
            {"id": 1, "step_pin": 2, "dir_pin": 3},
            {"id": 2, "step_pin": 5, "dir_pin": 6}
        ],
        "relays": [
            {"id": 1, "pin": 54},  # A0 on Arduino Giga
            {"id": 2, "pin": 55},  # A1
            {"id": 3, "pin": 56},  # A2
            {"id": 4, "pin": 57}   # A3
        ],
        "custom": []
    }


@app.route('/api/settings', methods=['GET'])
def get_settings():
    """Get the current pin configuration."""
    settings_path = get_settings_path()
    
    try:
        if settings_path.exists():
            with open(settings_path, 'r') as f:
                config = json.load(f)
            return jsonify(config)
        else:
            # Return defaults if no config file exists
            return jsonify(get_default_settings())
    except Exception as e:
        print(f"Error reading settings: {e}")
        return jsonify(get_default_settings())


@app.route('/api/settings', methods=['POST'])
def save_settings():
    """Save the pin configuration."""
    settings_path = get_settings_path()
    
    try:
        config = request.get_json()
        
        if not config:
            return jsonify({"error": "No configuration provided"}), 400
        
        # Validate required fields
        if 'motors' not in config or 'relays' not in config:
            return jsonify({"error": "Invalid configuration: missing motors or relays"}), 400
        
        # Save to file
        with open(settings_path, 'w') as f:
            json.dump(config, f, indent=2)
        
        print(f"Pin configuration saved to {settings_path}")
        
        # Notify ROS node about config change (if available)
        if ros_node:
            try:
                # Publish config update to a topic for arduino_controller to pick up
                msg = String()
                msg.data = json.dumps({"type": "config_update", "config": config})
                if hasattr(ros_node, 'sequence_pub'):
                    ros_node.sequence_pub.publish(msg)
            except Exception as e:
                print(f"Warning: Could not notify ROS about config update: {e}")
        
        return jsonify({"success": True, "message": "Settings saved successfully"})
    
    except json.JSONDecodeError as e:
        return jsonify({"error": f"Invalid JSON: {e}"}), 400
    except Exception as e:
        print(f"Error saving settings: {e}")
        return jsonify({"error": str(e)}), 500


@app.route('/api/arduino/reconnect', methods=['POST'])
def reconnect_arduino():
    """Trigger Arduino reconnection to apply new pin settings."""
    try:
        if ros_node and hasattr(ros_node, 'reconnect_pub'):
            msg = String()
            msg.data = 'reconnect'
            ros_node.reconnect_pub.publish(msg)
            print("Arduino reconnect request sent via ROS")
            return jsonify({"success": True, "message": "Reconnect request sent"})
        else:
            return jsonify({"error": "ROS node not available"}), 503
    except Exception as e:
        print(f"Error sending reconnect request: {e}")
        return jsonify({"error": str(e)}), 500


def main():
    """Start the web server"""
    global rosbridge_host, rosbridge_port
    
    # Try to get ROS Bridge port from environment variable if not set via ROS
    if not rosbridge_port or rosbridge_port == 9090:
        env_port = os.environ.get('ROS_BRIDGE_PORT')
        if env_port:
            try:
                rosbridge_port = int(env_port)
            except ValueError:
                pass
    
    # Initialize ROS (though rosbridge does most of the work)
    init_ros()
    
    # After init_ros, try to read parameters again if ROS is available
    if ros_node:
        try:
            host_param, port_param = ros_node.get_rosbridge_config()
            if host_param:
                rosbridge_host = host_param
            if port_param and port_param != 9090:
                rosbridge_port = port_param
        except Exception:
            pass
    
    try:
        # Start the web server
        print("Starting web interface on http://0.0.0.0:1111")
        print("Using ROS Bridge for ROS 2 communication")
        print(f"ROS Bridge URL will be determined dynamically based on connection")
        print(f"Default port: {rosbridge_port}")
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

