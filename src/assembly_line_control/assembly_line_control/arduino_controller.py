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
        self.connected = False
        self.connection_attempts = 0
        self.max_connection_attempts = 3
        self.reconnect_delay = 5.0  # seconds between reconnection attempts
        self.last_successful_port = None
        
        # Connection status stability - prevent flickering
        self._last_published_connected = None  # Track last published state
        self._disconnect_count = 0  # Count consecutive disconnect indicators
        self._disconnect_threshold = 2  # Need this many consecutive failures to mark disconnected
        
        # Parameters
        self.declare_parameter('serial_port', '')
        self.declare_parameter('serial_baud', 115200)
        
        # === PUBLISHERS (create first so they're available during init) ===
        self.motor1_status_pub = self.create_publisher(String, 'motor1/status', 10)
        self.motor2_status_pub = self.create_publisher(String, 'motor2/status', 10)
        self.relay1_status_pub = self.create_publisher(String, 'relay1/status', 10)
        self.relay2_status_pub = self.create_publisher(String, 'relay2/status', 10)
        self.relay3_status_pub = self.create_publisher(String, 'relay3/status', 10)
        self.relay4_status_pub = self.create_publisher(String, 'relay4/status', 10)
        self.connection_status_pub = self.create_publisher(String, 'arduino/status', 10)
        
        # Initialize serial connection (after publishers are created)
        self.init_serial_connection()
        
        # Start background thread to read Arduino responses
        self.running = True
        self.response_thread = threading.Thread(target=self._read_responses, daemon=True)
        self.response_thread.start()
        
        # Start connection monitor thread
        self.connection_monitor_thread = threading.Thread(target=self._monitor_connection, daemon=True)
        self.connection_monitor_thread.start()
        
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
        
        # Timers - 10Hz for motor status is enough for smooth display without jitter
        self.motor_update_timer = self.create_timer(0.1, self.update_motor_states)
        self.relay_status_timer = self.create_timer(1.0, self.publish_all_relay_status)
        self.connection_status_timer = self.create_timer(5.0, self.publish_connection_status)
        
        self.get_logger().info('Arduino controller started (unified motor + relay control)')
    
    def init_serial_connection(self):
        """Connect to the Arduino over USB serial with robust port discovery"""
        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('serial_baud').get_parameter_value().integer_value
        
        if baud:
            self.serial_baud = baud
        
        # If a specific port is provided, try it first
        if port:
            if self._try_connect_port(port):
                return
            self.get_logger().warn(f'Specified port {port} failed, searching for Arduino...')
        
        # Try the last successful port first (if we're reconnecting)
        if self.last_successful_port:
            self.get_logger().info(f'Trying last successful port: {self.last_successful_port}')
            if self._try_connect_port(self.last_successful_port):
                return
        
        # Search all available ports
        candidate_ports = self.find_arduino_ports()
        
        if not candidate_ports:
            self.get_logger().warn('No potential Arduino ports found. Will retry...')
            self.connected = False
            return
        
        self.get_logger().info(f'Found {len(candidate_ports)} potential Arduino port(s): {[p[0] for p in candidate_ports]}')
        
        # Try each candidate port
        for port_device, port_info in candidate_ports:
            self.get_logger().info(f'Trying port {port_device} ({port_info})...')
            if self._try_connect_port(port_device):
                return
        
        self.get_logger().warn('Could not connect to any Arduino. Will retry...')
        self.connected = False
    
    def _try_connect_port(self, port):
        """Try to connect to a specific port - if port opens, we're connected"""
        try:
            # Close existing connection if any
            if self.serial_port and self.serial_port.is_open:
                try:
                    self.serial_port.close()
                except:
                    pass
            
            # Open new connection
            self.serial_port = serial.Serial(
                port=port,
                baudrate=self.serial_baud,
                timeout=self.serial_timeout,
                write_timeout=1.0
            )
            
            # Wait for Arduino to reset after connection
            time.sleep(2.0)
            
            # Clear any startup messages
            self.serial_port.reset_input_buffer()
            
            # If we got here, port opened successfully - we're connected
            self.connected = True
            self.last_successful_port = port
            self.connection_attempts = 0
            self._disconnect_count = 0
            self.get_logger().info(f'✓ Connected to Arduino on {port} at {self.serial_baud} baud')
            
            # Immediately publish connection status
            self._publish_connected_status()
            return True
                
        except serial.SerialException as e:
            self.get_logger().warn(f'Could not open port {port}: {e}')
            self.serial_port = None
            return False
        except Exception as e:
            self.get_logger().error(f'Unexpected error connecting to {port}: {e}')
            self.serial_port = None
            return False
    
    def find_arduino_ports(self):
        """Find all potential Arduino ports, sorted by likelihood"""
        ports = serial.tools.list_ports.comports()
        candidates = []
        
        # Known Arduino vendor IDs
        arduino_vids = {
            0x2341: 'Arduino',
            0x2A03: 'Arduino',
            0x1A86: 'CH340 (Arduino clone)',
            0x10C4: 'CP210x (Arduino clone)',
            0x0403: 'FTDI',
            0x239A: 'Adafruit',
            0x1B4F: 'SparkFun',
        }
        
        # Keywords that suggest an Arduino
        arduino_keywords = ['arduino', 'ch340', 'ch341', 'cp210', 'cp2102', 'ftdi', 
                          'usb serial', 'usb-serial', 'acm', 'usbmodem', 'usbserial',
                          'giga', 'mega', 'uno', 'nano', 'leonardo', 'due']
        
        for port in ports:
            score = 0
            info_parts = []
            
            # Check vendor ID
            if port.vid:
                if port.vid in arduino_vids:
                    score += 10
                    info_parts.append(arduino_vids[port.vid])
            
            # Check description for keywords
            desc_lower = (port.description or '').lower()
            for keyword in arduino_keywords:
                if keyword in desc_lower:
                    score += 5
                    break
            
            # Check device name patterns
            device_lower = port.device.lower()
            if 'usbmodem' in device_lower or 'acm' in device_lower:
                score += 3
            if 'ttyusb' in device_lower or 'ttyacm' in device_lower:
                score += 3
            if 'cu.usb' in device_lower:
                score += 2
            
            # Add port info
            if port.description:
                info_parts.append(port.description)
            if port.vid and port.pid:
                info_parts.append(f'VID:PID={port.vid:04X}:{port.pid:04X}')
            
            # Only include ports with some indicators they might be Arduino
            if score > 0 or 'usb' in device_lower:
                candidates.append((port.device, ', '.join(info_parts) if info_parts else 'Unknown', score))
        
        # Sort by score (highest first)
        candidates.sort(key=lambda x: x[2], reverse=True)
        
        # Return (device, info) tuples
        return [(c[0], c[1]) for c in candidates]
    
    def _monitor_connection(self):
        """Background thread to monitor and restore connection"""
        while self.running:
            try:
                time.sleep(self.reconnect_delay)
                
                # Only try to reconnect if self.connected is explicitly False
                # Don't check serial_port.is_open as it can be flaky
                if not self.connected:
                    if self.connection_attempts < self.max_connection_attempts:
                        self.connection_attempts += 1
                        self.get_logger().info(f'Attempting to reconnect to Arduino (attempt {self.connection_attempts}/{self.max_connection_attempts})...')
                        self.init_serial_connection()
                    elif self.connection_attempts == self.max_connection_attempts:
                        self.connection_attempts += 1  # Prevent repeated logging
                        self.get_logger().warn('Max reconnection attempts reached. Will keep trying in background...')
                    else:
                        # Keep trying silently
                        self.init_serial_connection()
                else:
                    # Connection is good, reset attempt counter
                    self.connection_attempts = 0
                    
            except Exception as e:
                self.get_logger().error(f'Error in connection monitor: {e}')
    
    def _read_responses(self):
        """Background thread to read Arduino responses"""
        buffer = ""
        consecutive_errors = 0
        max_consecutive_errors = 5  # Only mark disconnected after multiple failures
        
        while self.running:
            try:
                # Check if we have a valid connection
                if not self.serial_port or not self.serial_port.is_open:
                    time.sleep(0.5)
                    continue
                
                with self.serial_lock:
                    if self.serial_port and self.serial_port.is_open and self.serial_port.in_waiting > 0:
                        data = self.serial_port.read(self.serial_port.in_waiting).decode('utf-8', errors='ignore')
                        buffer += data
                        consecutive_errors = 0  # Reset on successful read
                
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    if line:
                        # Log Arduino responses at info level so they're visible
                        self.get_logger().info(f'[Arduino] {line}')
                        consecutive_errors = 0  # Reset on valid data
                
                time.sleep(0.01)
            except serial.SerialException as e:
                if self.running:
                    consecutive_errors += 1
                    if consecutive_errors >= max_consecutive_errors:
                        self.get_logger().error(f'Serial connection lost after {consecutive_errors} errors: {e}')
                        self.connected = False
                    # Don't break - let the connection monitor handle reconnection
                time.sleep(0.5)
            except Exception as e:
                if self.running:
                    self.get_logger().error(f'Error reading serial: {e}')
                time.sleep(0.1)
    
    def send_command(self, command: dict):
        """Send a JSON command to the Arduino (thread-safe)"""
        if not self.connected or not self.serial_port or not self.serial_port.is_open:
            # Only log occasionally to avoid spam
            if not hasattr(self, '_last_disconnect_log') or time.time() - self._last_disconnect_log > 5:
                self.get_logger().warn('Arduino not connected, command not sent')
                self._last_disconnect_log = time.time()
            return False
        
        try:
            with self.serial_lock:
                command_str = json.dumps(command) + '\n'
                self.serial_port.write(command_str.encode('utf-8'))
                self.serial_port.flush()
            return True
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')
            # Don't set connected=False here - let read thread handle disconnect detection
            return False
        except Exception as e:
            self.get_logger().error(f'Error sending command: {e}')
            return False
    
    # === MOTOR CONTROL ===
    
    def motor_command_callback(self, motor_id, msg):
        """Handle motor movement commands"""
        steps = msg.data
        state = self.motor_states[motor_id]
        current_time = time.time()
        
        if steps == 0:
            # Explicit stop command - halt the motor
            state['steps_remaining'] = 0
            state['steps_total'] = 0
            state['is_moving'] = False
            state['start_time'] = 0
            state['expected_duration'] = 0
            
            # Send explicit stop command to Arduino
            command = {
                'type': 'motor',
                'motor_id': motor_id,
                'steps': 0,
                'speed': float(state['speed']),
                'stop': True  # Explicit stop flag
            }
            
            if self.send_command(command):
                self.get_logger().info(f'Motor {motor_id}: STOPPED')
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
    
    def _publish_connected_status(self):
        """Immediately publish connected status (called right after successful connection)"""
        self._last_published_connected = True
        status = {
            'connected': True,
            'port': self.last_successful_port,
            'baud': self.serial_baud
        }
        msg = String()
        msg.data = json.dumps(status)
        self.connection_status_pub.publish(msg)
        self.get_logger().info(f'Arduino status: connected on {self.last_successful_port}')
    
    def publish_connection_status(self):
        """Publish Arduino connection status periodically so new clients get it"""
        # Simple logic: if we have a successful port and connected flag is true, we're connected
        is_connected = self.connected and self.last_successful_port is not None
        
        # Always publish current status so new browser connections receive it
        status = {
            'connected': is_connected,
            'port': self.last_successful_port if is_connected else None,
            'baud': self.serial_baud if is_connected else None
        }
        msg = String()
        msg.data = json.dumps(status)
        self.connection_status_pub.publish(msg)
        
        # Only log if state changed
        if self._last_published_connected != is_connected:
            self._last_published_connected = is_connected
            if is_connected:
                self.get_logger().info(f'Arduino status: connected on {self.last_successful_port}')
            else:
                self.get_logger().warn('Arduino status: disconnected')
    
    def destroy_node(self):
        """Clean up on shutdown"""
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
                self.get_logger().info('Serial port closed')
            except:
                pass
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
