#!/usr/bin/env python3

"""
Relay Controller for Assembly Line OS

This node keeps track of whether each relay is on or off. When it receives
commands from the web interface, it sends them to the Arduino over USB serial
to actually switch the relays.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import serial
import serial.tools.list_ports
import time


class RelayController(Node):
    """Keeps track of relay states and sends commands to the Arduino"""
    
    def __init__(self):
        super().__init__('relay_controller')
        
        # Track whether each relay is on or off
        self.relay_states = {}
        
        # Start with all relays turned off
        for relay_id in range(1, 5):
            self.relay_states[relay_id] = 'off'
        
        # Set up serial connection to talk to the Arduino
        self.serial_port = None
        self.serial_baud = 115200
        self.serial_timeout = 1.0
        
        # Let users specify the serial port, or we'll try to find it automatically
        self.declare_parameter('serial_port', '')
        self.declare_parameter('serial_baud', 115200)
        
        self.init_serial_connection()
        
        # Listen for commands to switch relays
        self.relay_command_sub = self.create_subscription(
            String, 'relay/command', 
            self.relay_command_callback, 10)
        
        # Publish updates about relay status so the web interface can show current state
        self.relay1_status_pub = self.create_publisher(
            String, 'relay1/status', 10)
        self.relay2_status_pub = self.create_publisher(
            String, 'relay2/status', 10)
        self.relay3_status_pub = self.create_publisher(
            String, 'relay3/status', 10)
        self.relay4_status_pub = self.create_publisher(
            String, 'relay4/status', 10)
        
        # Update relay status once per second
        self.status_timer = self.create_timer(1.0, self.publish_all_status)
        
        self.get_logger().info('Relay controller node started')
    
    def init_serial_connection(self):
        """Connect to the Arduino over USB serial"""
        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('serial_baud').get_parameter_value().integer_value
        
        if baud:
            self.serial_baud = baud
        
        # If no port was specified, try to find the Arduino automatically
        if not port:
            port = self.find_arduino_port()
        
        if port:
            try:
                self.serial_port = serial.Serial(
                    port=port,
                    baudrate=self.serial_baud,
                    timeout=self.serial_timeout,
                    write_timeout=self.serial_timeout
                )
                # Wait a moment for the Arduino to finish resetting when we first connect
                time.sleep(0.5)
                self.get_logger().info(f'Connected to Arduino on {port} at {self.serial_baud} baud')
            except serial.SerialException as e:
                self.get_logger().error(f'Failed to open serial port {port}: {e}')
                self.serial_port = None
        else:
            self.get_logger().warn('No Arduino found. Relay commands will not be sent to hardware.')
    
    def find_arduino_port(self):
        """Look through USB ports to find the Arduino"""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            # Check if the port description mentions common Arduino-related terms
            if any(keyword in port.description.lower() for keyword in ['arduino', 'ch340', 'ch341', 'cp210', 'ftdi']):
                return port.device
            # Also check the USB vendor ID (many Arduino boards use these IDs)
            if port.vid and port.pid:
                arduino_vids = [0x2341, 0x2A03, 0x1A86, 0x10C4, 0x0403]
                if port.vid in arduino_vids:
                    return port.device
        return None
    
    def send_relay_command(self, relay_id, state):
        """Send a command to the Arduino to switch a relay"""
        if not self.serial_port or not self.serial_port.is_open:
            return
        
        try:
            # Format the command as JSON and send it over serial
            command = {
                'type': 'relay',
                'relay_id': relay_id,
                'state': state
            }
            command_str = json.dumps(command) + '\n'
            self.serial_port.write(command_str.encode('utf-8'))
            self.serial_port.flush()
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')
            self.serial_port = None
    
    def relay_command_callback(self, msg):
        """Called when we receive a command to switch a relay"""
        try:
            command = json.loads(msg.data)
            relay_id = int(command.get('relay_id'))
            state = command.get('state', 'off').lower()
            
            # Validate relay_id
            if relay_id < 1 or relay_id > 4:
                self.get_logger().warn(f'Invalid relay_id: {relay_id}')
                return
            
            # Validate state
            if state not in ['on', 'off']:
                self.get_logger().warn(f'Invalid relay state: {state}')
                return
            
            # Remember the new state and tell the Arduino to switch
            self.relay_states[relay_id] = state
            self.send_relay_command(relay_id, state)
            
            self.get_logger().info(f'Relay {relay_id} set to {state.upper()}')
            
            # Immediately send an update so the web interface knows the relay changed
            self.publish_relay_status(relay_id)
            
        except json.JSONDecodeError:
            self.get_logger().error(f'Failed to parse relay command: {msg.data}')
        except (KeyError, ValueError) as e:
            self.get_logger().error(f'Invalid relay command format: {e}')
    
    def publish_relay_status(self, relay_id):
        """Send the current state of a relay to anyone who's listening"""
        state = self.relay_states.get(relay_id, 'off')
        
        status = {
            'relay_id': relay_id,
            'state': state
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status)
        
        if relay_id == 1:
            self.relay1_status_pub.publish(status_msg)
        elif relay_id == 2:
            self.relay2_status_pub.publish(status_msg)
        elif relay_id == 3:
            self.relay3_status_pub.publish(status_msg)
        elif relay_id == 4:
            self.relay4_status_pub.publish(status_msg)
    
    def publish_all_status(self):
        """Send status updates for all relays"""
        for relay_id in range(1, 5):
            self.publish_relay_status(relay_id)


def main(args=None):
    rclpy.init(args=args)
    relay_controller = RelayController()
    
    try:
        rclpy.spin(relay_controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up: close the serial connection when we're done
        if relay_controller.serial_port and relay_controller.serial_port.is_open:
            relay_controller.serial_port.close()
            relay_controller.get_logger().info('Serial port closed')
        relay_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

