#!/usr/bin/env python3

"""
Motor Controller for Assembly Line OS

This node keeps track of how many steps each motor still needs to complete,
and how fast they're moving. When it receives commands from the web interface,
it sends them to the Arduino over USB serial to actually move the motors.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, String
import json
import time
import serial
import serial.tools.list_ports


class MotorController(Node):
    """Keeps track of motor positions and sends commands to the Arduino"""
    
    def __init__(self):
        super().__init__('motor_controller')
        
        # We track each motor's state: how many steps left, how fast it's going, and if it's moving
        self.motor_states = {}
        
        # Set up both motors with default values
        for motor_id in range(1, 3):
            self.motor_states[motor_id] = {
                'steps_remaining': 0,
                'speed': 100.0,  # Default speed: 100 steps per second
                'is_moving': False,
                'last_update_time': time.time()
            }
        
        # Set up serial connection to talk to the Arduino
        self.serial_port = None
        self.serial_baud = 115200
        self.serial_timeout = 1.0
        
        # Let users specify the serial port, or we'll try to find it automatically
        self.declare_parameter('serial_port', '')
        self.declare_parameter('serial_baud', 115200)
        
        self.init_serial_connection()
        
        # Listen for commands to move the motors
        self.motor1_sub = self.create_subscription(
            Int32, 'motor1/command', 
            lambda msg: self.motor_command_callback(1, msg), 10)
        self.motor2_sub = self.create_subscription(
            Int32, 'motor2/command', 
            lambda msg: self.motor_command_callback(2, msg), 10)
        
        # Listen for speed change commands
        self.motor1_speed_sub = self.create_subscription(
            Float32, 'motor1/speed', 
            lambda msg: self.motor_speed_callback(1, msg), 10)
        self.motor2_speed_sub = self.create_subscription(
            Float32, 'motor2/speed', 
            lambda msg: self.motor_speed_callback(2, msg), 10)
        
        # Publish updates about motor status so the web interface can show progress
        self.motor1_status_pub = self.create_publisher(
            String, 'motor1/status', 10)
        self.motor2_status_pub = self.create_publisher(
            String, 'motor2/status', 10)
        
        # Update motor positions 20 times per second (smooth enough for real-time updates)
        self.update_timer = self.create_timer(0.05, self.update_motor_states)
        
        self.get_logger().info('Motor controller node started')
    
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
                time.sleep(2)
                self.get_logger().info(f'Connected to Arduino on {port} at {self.serial_baud} baud')
            except serial.SerialException as e:
                self.get_logger().error(f'Failed to open serial port {port}: {e}')
                self.serial_port = None
        else:
            self.get_logger().warn('No Arduino found. Motor commands will not be sent to hardware.')
    
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
    
    def send_motor_command(self, motor_id, steps, speed):
        """Send a command to the Arduino to move a motor"""
        if not self.serial_port or not self.serial_port.is_open:
            return
        
        try:
            # Format the command as JSON and send it over serial
            command = {
                'type': 'motor',
                'motor_id': motor_id,
                'steps': int(steps),
                'speed': float(speed)
            }
            command_str = json.dumps(command) + '\n'
            self.serial_port.write(command_str.encode('utf-8'))
            self.serial_port.flush()
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')
            self.serial_port = None
    
    def motor_command_callback(self, motor_id, msg):
        """Called when we receive a command to move a motor"""
        steps = msg.data
        state = self.motor_states[motor_id]
        
        # Add these steps to the queue (positive moves forward, negative moves backward)
        state['steps_remaining'] += steps
        state['is_moving'] = True
        state['last_update_time'] = time.time()
        
        # Tell the Arduino to start moving
        self.send_motor_command(motor_id, steps, state['speed'])
        
        self.get_logger().info(
            f'Motor {motor_id} command: {steps} steps '
            f'(total remaining: {state["steps_remaining"]})'
        )
    
    def motor_speed_callback(self, motor_id, msg):
        """Called when we receive a command to change motor speed"""
        # Keep speed in a reasonable range (1-200 steps per second)
        speed = max(1.0, min(200.0, msg.data))
        state = self.motor_states[motor_id]
        state['speed'] = speed
        
        # Send the speed update to Arduino (0 steps means just update speed, don't move)
        self.send_motor_command(motor_id, 0, speed)
        
        self.get_logger().info(f'Motor {motor_id} speed set to {speed} steps/second')
    
    def update_motor_states(self):
        """Update our estimate of how many steps each motor has left"""
        current_time = time.time()
        
        for motor_id, state in self.motor_states.items():
            if state['is_moving'] and state['steps_remaining'] != 0:
                # Figure out how much time has passed since we last checked
                elapsed = current_time - state['last_update_time']
                
                # Calculate how many steps the motor should have completed by now
                steps_to_subtract = abs(state['speed'] * elapsed)
                
                # Decrease the remaining steps (handling both forward and backward movement)
                if state['steps_remaining'] > 0:
                    state['steps_remaining'] = max(0, state['steps_remaining'] - steps_to_subtract)
                else:
                    state['steps_remaining'] = min(0, state['steps_remaining'] + steps_to_subtract)
                
                # If we're close enough to zero, consider the motor done
                if abs(state['steps_remaining']) < 0.5:
                    state['steps_remaining'] = 0
                    state['is_moving'] = False
                    self.get_logger().info(f'Motor {motor_id} finished moving')
                
                state['last_update_time'] = current_time
            
            # Send status update to the web interface
            self.publish_motor_status(motor_id)
    
    def publish_motor_status(self, motor_id):
        """Send the current motor status to anyone who's listening"""
        state = self.motor_states[motor_id]
        
        status = {
            'motor_id': motor_id,
            'steps_remaining': int(state['steps_remaining']),
            'speed': state['speed'],
            'is_moving': state['is_moving']
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status)
        
        if motor_id == 1:
            self.motor1_status_pub.publish(status_msg)
        elif motor_id == 2:
            self.motor2_status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    
    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up: close the serial connection when we're done
        if motor_controller.serial_port and motor_controller.serial_port.is_open:
            motor_controller.serial_port.close()
            motor_controller.get_logger().info('Serial port closed')
        motor_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

