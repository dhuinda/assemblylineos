#!/usr/bin/env python3

"""
Unified Arduino Controller for Assembly Line OS

This node handles ALL communication with the Arduino over a single serial connection.
It controls both motors and relays, avoiding the port conflict that occurs when
motor_controller and relay_controller each try to open their own connection.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, String
import json
import time
import serial
import serial.tools.list_ports
import threading


class ArduinoController(Node):
    """Single controller for all Arduino communication (motors + relays)"""
    
    def __init__(self):
        super().__init__('arduino_controller')
        
        # Motor state tracking
        self.motor_states = {}
        for motor_id in range(1, 3):
            self.motor_states[motor_id] = {
                'steps_remaining': 0,
                'steps_total': 0,  # Total steps for current movement
                'speed': 100.0,
                'is_moving': False,
                'start_time': 0,  # When movement started
                'expected_duration': 0  # Expected duration in seconds
            }
        
        # Relay state tracking
        self.relay_states = {}
        for relay_id in range(1, 5):
            self.relay_states[relay_id] = 'off'
        
        # Serial connection
        self.serial_port = None
        self.serial_baud = 115200
        self.serial_timeout = 0.1
        self.serial_lock = threading.Lock()  # Thread-safe serial access
        
        # Parameters
        self.declare_parameter('serial_port', '')
        self.declare_parameter('serial_baud', 115200)
        
        # Initialize serial connection
        self.init_serial_connection()
        
        # Start background thread to read Arduino responses
        self.running = True
        self.response_thread = threading.Thread(target=self._read_responses, daemon=True)
        self.response_thread.start()
        
        # === MOTOR SUBSCRIPTIONS ===
        self.motor1_sub = self.create_subscription(
            Int32, 'motor1/command', 
            lambda msg: self.motor_command_callback(1, msg), 10)
        self.motor2_sub = self.create_subscription(
            Int32, 'motor2/command', 
            lambda msg: self.motor_command_callback(2, msg), 10)
        
        self.motor1_speed_sub = self.create_subscription(
            Float32, 'motor1/speed', 
            lambda msg: self.motor_speed_callback(1, msg), 10)
        self.motor2_speed_sub = self.create_subscription(
            Float32, 'motor2/speed', 
            lambda msg: self.motor_speed_callback(2, msg), 10)
        
        # === RELAY SUBSCRIPTIONS ===
        self.relay_command_sub = self.create_subscription(
            String, 'relay/command', 
            self.relay_command_callback, 10)
        
        # === E-STOP SUBSCRIPTION ===
        self.estop_sub = self.create_subscription(
            String, 'estop', 
            self.estop_callback, 10)
        
        # === PUBLISHERS ===
        self.motor1_status_pub = self.create_publisher(String, 'motor1/status', 10)
        self.motor2_status_pub = self.create_publisher(String, 'motor2/status', 10)
        self.relay1_status_pub = self.create_publisher(String, 'relay1/status', 10)
        self.relay2_status_pub = self.create_publisher(String, 'relay2/status', 10)
        self.relay3_status_pub = self.create_publisher(String, 'relay3/status', 10)
        self.relay4_status_pub = self.create_publisher(String, 'relay4/status', 10)
        
        # Timers - 10Hz for motor status is enough for smooth display without jitter
        self.motor_update_timer = self.create_timer(0.1, self.update_motor_states)
        self.relay_status_timer = self.create_timer(1.0, self.publish_all_relay_status)
        
        self.get_logger().info('Arduino controller started (unified motor + relay control)')
    
    def init_serial_connection(self):
        """Connect to the Arduino over USB serial"""
        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('serial_baud').get_parameter_value().integer_value
        
        if baud:
            self.serial_baud = baud
        
        if not port:
            port = self.find_arduino_port()
        
        if port:
            try:
                self.serial_port = serial.Serial(
                    port=port,
                    baudrate=self.serial_baud,
                    timeout=self.serial_timeout,
                    write_timeout=1.0
                )
                time.sleep(2)  # Wait for Arduino to reset
                self.get_logger().info(f'Connected to Arduino on {port} at {self.serial_baud} baud')
            except serial.SerialException as e:
                self.get_logger().error(f'Failed to open serial port {port}: {e}')
                self.serial_port = None
        else:
            self.get_logger().warn('No Arduino found. Commands will not be sent to hardware.')
    
    def find_arduino_port(self):
        """Look through USB ports to find the Arduino"""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if any(keyword in port.description.lower() for keyword in 
                   ['arduino', 'ch340', 'ch341', 'cp210', 'ftdi', 'usb serial']):
                return port.device
            if port.vid and port.pid:
                arduino_vids = [0x2341, 0x2A03, 0x1A86, 0x10C4, 0x0403]
                if port.vid in arduino_vids:
                    return port.device
        return None
    
    def _read_responses(self):
        """Background thread to read Arduino responses"""
        buffer = ""
        while self.running and self.serial_port and self.serial_port.is_open:
            try:
                with self.serial_lock:
                    if self.serial_port.in_waiting > 0:
                        data = self.serial_port.read(self.serial_port.in_waiting).decode('utf-8', errors='ignore')
                        buffer += data
                
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    if line:
                        # Log Arduino responses at info level so they're visible
                        self.get_logger().info(f'[Arduino] {line}')
                
                time.sleep(0.01)
            except Exception as e:
                if self.running:
                    self.get_logger().error(f'Error reading serial: {e}')
                break
    
    def send_command(self, command: dict):
        """Send a JSON command to the Arduino (thread-safe)"""
        if not self.serial_port or not self.serial_port.is_open:
            self.get_logger().warn('Serial port not open, command not sent')
            return False
        
        try:
            with self.serial_lock:
                command_str = json.dumps(command) + '\n'
                self.serial_port.write(command_str.encode('utf-8'))
                self.serial_port.flush()
            return True
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')
            return False
    
    # === MOTOR CONTROL ===
    
    def motor_command_callback(self, motor_id, msg):
        """Handle motor movement commands"""
        steps = msg.data
        state = self.motor_states[motor_id]
        current_time = time.time()
        
        if steps == 0:
            # Stop command - immediately halt the motor
            state['steps_remaining'] = 0
            state['steps_total'] = 0
            state['is_moving'] = False
            state['start_time'] = 0
            state['expected_duration'] = 0
            self.get_logger().info(f'Motor {motor_id}: STOP command received')
        else:
            # New movement command - reset tracking
            state['steps_remaining'] = abs(steps)
            state['steps_total'] = abs(steps)
            state['is_moving'] = True
            state['start_time'] = current_time
            # Calculate expected duration based on current speed
            if state['speed'] > 0:
                state['expected_duration'] = abs(steps) / state['speed']
            else:
                state['expected_duration'] = 0
        
        command = {
            'type': 'motor',
            'motor_id': motor_id,
            'steps': int(steps),
            'speed': float(state['speed'])
        }
        
        if self.send_command(command):
            if steps == 0:
                self.get_logger().info(f'Motor {motor_id}: STOPPED')
            else:
                self.get_logger().info(f'Motor {motor_id}: {steps} steps at {state["speed"]} steps/sec (est. {state["expected_duration"]:.2f}s)')
    
    def motor_speed_callback(self, motor_id, msg):
        """Handle motor speed changes"""
        speed = max(1.0, min(6500.0, msg.data))
        state = self.motor_states[motor_id]
        old_speed = state['speed']
        state['speed'] = speed
        
        # If motor is currently moving, recalculate expected duration
        if state['is_moving'] and state['steps_remaining'] > 0 and speed > 0:
            # Calculate remaining duration based on remaining steps and new speed
            remaining_steps = state['steps_remaining']
            state['expected_duration'] = remaining_steps / speed
            state['start_time'] = time.time()  # Reset start time
            state['steps_total'] = remaining_steps  # Reset total to remaining
        
        command = {
            'type': 'motor',
            'motor_id': motor_id,
            'steps': 0,
            'speed': float(speed)
        }
        
        self.send_command(command)
        self.get_logger().info(f'Motor {motor_id} speed set to {speed} steps/sec')
    
    def update_motor_states(self):
        """Update motor position estimates and publish status"""
        current_time = time.time()
        
        for motor_id, state in self.motor_states.items():
            if state['is_moving'] and state['steps_total'] > 0:
                # Calculate progress based on elapsed time since start
                elapsed = current_time - state['start_time']
                
                if state['expected_duration'] > 0:
                    # Calculate remaining steps based on progress through expected duration
                    progress = min(1.0, elapsed / state['expected_duration'])
                    state['steps_remaining'] = max(0, state['steps_total'] * (1.0 - progress))
                    
                    # Check if movement is complete
                    if progress >= 1.0:
                        state['steps_remaining'] = 0
                        state['is_moving'] = False
                        state['steps_total'] = 0
                else:
                    # No expected duration, mark as complete
                    state['steps_remaining'] = 0
                    state['is_moving'] = False
            
            self.publish_motor_status(motor_id)
    
    def publish_motor_status(self, motor_id):
        """Publish motor status"""
        state = self.motor_states[motor_id]
        status = {
            'motor_id': motor_id,
            'steps_remaining': round(state['steps_remaining']),  # Round to avoid floating point noise
            'steps_total': round(state['steps_total']),
            'speed': state['speed'],
            'is_moving': state['is_moving']
        }
        
        msg = String()
        msg.data = json.dumps(status)
        
        if motor_id == 1:
            self.motor1_status_pub.publish(msg)
        elif motor_id == 2:
            self.motor2_status_pub.publish(msg)
    
    # === RELAY CONTROL ===
    
    def relay_command_callback(self, msg):
        """Handle relay commands"""
        try:
            command_data = json.loads(msg.data)
            relay_id = int(command_data.get('relay_id'))
            state = command_data.get('state', 'off').lower()
            
            if relay_id < 1 or relay_id > 4:
                self.get_logger().warn(f'Invalid relay_id: {relay_id}')
                return
            
            if state not in ['on', 'off']:
                self.get_logger().warn(f'Invalid relay state: {state}')
                return
            
            self.relay_states[relay_id] = state
            
            command = {
                'type': 'relay',
                'relay_id': relay_id,
                'state': state
            }
            
            if self.send_command(command):
                self.get_logger().info(f'Relay {relay_id} set to {state.upper()}')
            
            self.publish_relay_status(relay_id)
            
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            self.get_logger().error(f'Invalid relay command: {e}')
    
    def publish_relay_status(self, relay_id):
        """Publish relay status"""
        state = self.relay_states.get(relay_id, 'off')
        status = {'relay_id': relay_id, 'state': state}
        
        msg = String()
        msg.data = json.dumps(status)
        
        pubs = {1: self.relay1_status_pub, 2: self.relay2_status_pub, 
                3: self.relay3_status_pub, 4: self.relay4_status_pub}
        if relay_id in pubs:
            pubs[relay_id].publish(msg)
    
    def publish_all_relay_status(self):
        """Publish status for all relays"""
        for relay_id in range(1, 5):
            self.publish_relay_status(relay_id)
    
    # === E-STOP CONTROL ===
    
    def estop_callback(self, msg):
        """Handle emergency stop command"""
        self.get_logger().warn('E-STOP: Emergency stop activated!')
        
        # Stop all motors locally
        for motor_id in range(1, 3):
            state = self.motor_states[motor_id]
            state['steps_remaining'] = 0
            state['steps_total'] = 0
            state['is_moving'] = False
            state['start_time'] = 0
            state['expected_duration'] = 0
        
        # Turn off all relays locally
        for relay_id in range(1, 5):
            self.relay_states[relay_id] = 'off'
        
        # Send E-Stop command to Arduino
        command = {'type': 'estop'}
        if self.send_command(command):
            self.get_logger().warn('E-STOP: Command sent to Arduino')
        
        # Publish updated statuses
        for motor_id in range(1, 3):
            self.publish_motor_status(motor_id)
        for relay_id in range(1, 5):
            self.publish_relay_status(relay_id)
        
        self.get_logger().warn('E-STOP: All systems stopped')
    
    def destroy_node(self):
        """Clean up on shutdown"""
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info('Serial port closed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    controller = ArduinoController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
