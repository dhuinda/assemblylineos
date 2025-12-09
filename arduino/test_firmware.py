#!/usr/bin/env python3
"""
Arduino Firmware Test Script

A simple test application to test the Arduino firmware without running
the full ROS 2 application. This script connects to the Arduino via
serial and allows you to send commands interactively.

Usage:
    python3 test_firmware.py [--port PORT] [--baud BAUDRATE]
    
    Or run without arguments and it will try to auto-detect the Arduino.
"""

import serial
import serial.tools.list_ports
import json
import time
import sys
import argparse
import threading
from typing import Optional


class ArduinoTester:
    """Test interface for Arduino firmware"""
    
    def __init__(self, port: Optional[str] = None, baud: int = 115200, verbose: bool = True):
        self.port = port
        self.baud = baud
        self.serial_conn = None
        self.running = False
        self.response_thread = None
        self.verbose = verbose
        
    def find_arduino_port(self) -> Optional[str]:
        """Auto-detect Arduino port"""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            # Check port description
            if any(keyword in port.description.lower() for keyword in 
                   ['arduino', 'ch340', 'ch341', 'cp210', 'ftdi', 'usb serial']):
                return port.device
            # Check USB vendor IDs
            if port.vid and port.pid:
                arduino_vids = [0x2341, 0x2A03, 0x1A86, 0x10C4, 0x0403]
                if port.vid in arduino_vids:
                    return port.device
        return None
    
    def connect(self) -> bool:
        """Connect to Arduino"""
        if not self.port:
            self.port = self.find_arduino_port()
        
        if not self.port:
            print("ERROR: No Arduino port found. Please specify with --port")
            return False
        
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=0.1,  # Short timeout for non-blocking reads
                write_timeout=1.0
            )
            # Wait for Arduino to reset
            time.sleep(2)
            print(f"✓ Connected to Arduino on {self.port} at {self.baud} baud")
            
            # Start response listener
            self.running = True
            self.response_thread = threading.Thread(target=self._read_responses, daemon=True)
            self.response_thread.start()
            
            # Read and display initial messages from Arduino
            print("\nReading initial Arduino messages...")
            time.sleep(0.5)
            initial_timeout = time.time() + 2.0  # Wait up to 2 seconds for initial messages
            while time.time() < initial_timeout:
                if self.serial_conn.in_waiting:
                    line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        print(f"[Arduino] {line}")
                else:
                    time.sleep(0.1)
            print("Ready to send commands.\n")
            
            return True
        except serial.SerialException as e:
            print(f"ERROR: Failed to connect to {self.port}: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from Arduino"""
        self.running = False
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("\n✓ Disconnected from Arduino")
    
    def _read_responses(self):
        """Background thread to read Arduino responses"""
        buffer = ""
        while self.running and self.serial_conn and self.serial_conn.is_open:
            try:
                # Read all available bytes
                if self.serial_conn.in_waiting > 0:
                    data = self.serial_conn.read(self.serial_conn.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data
                    
                    # Process complete lines
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line:
                            if self.verbose:
                                print(f"[Arduino] {line}")
                else:
                    # Small delay to avoid busy-waiting
                    time.sleep(0.01)
            except serial.SerialException as e:
                if self.running:
                    print(f"[Error reading serial] {e}")
                break
            except Exception as e:
                if self.running:
                    print(f"[Error reading response] {e}")
                break
    
    def send_command(self, command: dict, wait_for_response: bool = True) -> bool:
        """Send a JSON command to Arduino"""
        if not self.serial_conn or not self.serial_conn.is_open:
            print("ERROR: Not connected to Arduino")
            return False
        
        try:
            command_str = json.dumps(command) + '\n'
            self.serial_conn.write(command_str.encode('utf-8'))
            self.serial_conn.flush()
            
            # Give Arduino time to process and respond
            if wait_for_response:
                time.sleep(0.1)
            
            return True
        except serial.SerialException as e:
            print(f"ERROR: Failed to send command: {e}")
            return False
    
    def test_motor(self, motor_id: int, steps: int, speed: float = 100.0):
        """Test a motor command"""
        if motor_id < 1 or motor_id > 2:
            print(f"ERROR: motor_id must be 1 or 2 (got {motor_id})")
            return
        
        command = {
            'type': 'motor',
            'motor_id': motor_id,
            'steps': steps,
            'speed': speed
        }
        print(f"Sending: Motor {motor_id}, {steps} steps at {speed} steps/sec")
        self.send_command(command)
    
    def test_relay(self, relay_id: int, state: str):
        """Test a relay command"""
        if relay_id < 1 or relay_id > 4:
            print(f"ERROR: relay_id must be 1-4 (got {relay_id})")
            return
        
        state = state.lower()
        if state not in ['on', 'off']:
            print(f"ERROR: state must be 'on' or 'off' (got {state})")
            return
        
        command = {
            'type': 'relay',
            'relay_id': relay_id,
            'state': state
        }
        print(f"Sending: Relay {relay_id} -> {state.upper()}")
        self.send_command(command)
    
    def wait_for_response(self, timeout: float = 1.0) -> str:
        """Wait for and return the next response from Arduino"""
        if not self.serial_conn or not self.serial_conn.is_open:
            return ""
        
        start_time = time.time()
        buffer = ""
        
        while time.time() - start_time < timeout:
            if self.serial_conn.in_waiting > 0:
                data = self.serial_conn.read(self.serial_conn.in_waiting).decode('utf-8', errors='ignore')
                buffer += data
                
                if '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    return line.strip()
            time.sleep(0.01)
        
        return ""


def print_menu():
    """Print the interactive menu"""
    print("\n" + "="*60)
    print("Arduino Firmware Test Menu")
    print("="*60)
    print("1. Test Motor 1")
    print("2. Test Motor 2")
    print("3. Test Relay 1")
    print("4. Test Relay 2")
    print("5. Test Relay 3")
    print("6. Test Relay 4")
    print("7. Run Test Sequence (all motors and relays)")
    print("8. Custom Command")
    print("0. Exit")
    print("="*60)


def interactive_mode(tester: ArduinoTester):
    """Run interactive menu mode"""
    print("\nEntering interactive mode. Type 'help' for commands or use menu numbers.")
    print("Type 'menu' to show the menu again.\n")
    
    while True:
        try:
            choice = input("\n> ").strip().lower()
            
            if choice == '0' or choice == 'exit' or choice == 'quit':
                break
            elif choice == 'menu' or choice == 'help':
                print_menu()
            elif choice == '1':
                steps = int(input("Enter steps for Motor 1: "))
                speed = float(input("Enter speed (default 100): ") or "100")
                tester.test_motor(1, steps, speed)
            elif choice == '2':
                steps = int(input("Enter steps for Motor 2: "))
                speed = float(input("Enter speed (default 100): ") or "100")
                tester.test_motor(2, steps, speed)
            elif choice == '3':
                state = input("Enter state (on/off) for Relay 1: ").strip()
                tester.test_relay(1, state)
            elif choice == '4':
                state = input("Enter state (on/off) for Relay 2: ").strip()
                tester.test_relay(2, state)
            elif choice == '5':
                state = input("Enter state (on/off) for Relay 3: ").strip()
                tester.test_relay(3, state)
            elif choice == '6':
                state = input("Enter state (on/off) for Relay 4: ").strip()
                tester.test_relay(4, state)
            elif choice == '7':
                run_test_sequence(tester)
            elif choice == '8':
                custom_command(tester)
            else:
                print("Invalid choice. Type 'menu' for options.")
        
        except ValueError as e:
            print(f"ERROR: Invalid input - {e}")
        except KeyboardInterrupt:
            print("\n\nInterrupted by user")
            break
        except Exception as e:
            print(f"ERROR: {e}")


def run_test_sequence(tester: ArduinoTester):
    """Run a comprehensive test sequence"""
    print("\n" + "="*60)
    print("Running Test Sequence")
    print("="*60)
    
    # Test motors
    print("\n--- Testing Motors ---")
    for motor_id in [1, 2]:
        print(f"\nTesting Motor {motor_id}...")
        tester.test_motor(motor_id, 100, 50.0)
        time.sleep(2)
        tester.test_motor(motor_id, -50, 50.0)
        time.sleep(2)
    
    # Test relays
    print("\n--- Testing Relays ---")
    for relay_id in [1, 2, 3, 4]:
        print(f"\nTesting Relay {relay_id}...")
        tester.test_relay(relay_id, 'on')
        time.sleep(0.5)
        tester.test_relay(relay_id, 'off')
        time.sleep(0.5)
    
    print("\n" + "="*60)
    print("Test sequence complete!")
    print("="*60)


def custom_command(tester: ArduinoTester):
    """Send a custom JSON command"""
    print("\nEnter custom JSON command (or 'cancel' to abort):")
    print("Example: {\"type\":\"motor\",\"motor_id\":1,\"steps\":100,\"speed\":50.0}")
    print("         {\"type\":\"relay\",\"relay_id\":1,\"state\":\"on\"}")
    
    cmd_str = input("> ").strip()
    if cmd_str.lower() == 'cancel':
        return
    
    try:
        command = json.loads(cmd_str)
        print(f"Sending custom command: {json.dumps(command)}")
        tester.send_command(command)
    except json.JSONDecodeError as e:
        print(f"ERROR: Invalid JSON - {e}")
    except Exception as e:
        print(f"ERROR: {e}")


def main():
    parser = argparse.ArgumentParser(
        description='Test Arduino firmware without running the full ROS 2 application'
    )
    parser.add_argument(
        '--port', '-p',
        type=str,
        default=None,
        help='Serial port (e.g., /dev/ttyUSB0 or COM3). Auto-detects if not specified.'
    )
    parser.add_argument(
        '--baud', '-b',
        type=int,
        default=115200,
        help='Baud rate (default: 115200)'
    )
    parser.add_argument(
        '--list-ports',
        action='store_true',
        help='List available serial ports and exit'
    )
    parser.add_argument(
        '--test-sequence',
        action='store_true',
        help='Run test sequence and exit (non-interactive)'
    )
    parser.add_argument(
        '--motor',
        type=int,
        metavar='ID',
        help='Test motor (1-2) with default values'
    )
    parser.add_argument(
        '--relay',
        type=int,
        metavar='ID',
        help='Test relay (1-4) with default values'
    )
    parser.add_argument(
        '--steps',
        type=int,
        default=100,
        help='Number of steps for motor (default: 100)'
    )
    parser.add_argument(
        '--speed',
        type=float,
        default=100.0,
        help='Speed in steps/second (default: 100.0)'
    )
    parser.add_argument(
        '--relay-state',
        type=str,
        choices=['on', 'off'],
        help='Relay state (on/off)'
    )
    parser.add_argument(
        '--quiet', '-q',
        action='store_true',
        help='Suppress Arduino response messages (responses still captured)'
    )
    
    args = parser.parse_args()
    
    # List ports if requested
    if args.list_ports:
        print("Available serial ports:")
        ports = serial.tools.list_ports.comports()
        for port in ports:
            print(f"  {port.device} - {port.description}")
            if port.vid and port.pid:
                print(f"    VID: 0x{port.vid:04X}, PID: 0x{port.pid:04X}")
        return
    
    # Create tester
    tester = ArduinoTester(port=args.port, baud=args.baud, verbose=not args.quiet)
    
    # Connect
    if not tester.connect():
        sys.exit(1)
    
    try:
        # Command-line mode
        if args.test_sequence:
            run_test_sequence(tester)
        elif args.motor:
            tester.test_motor(args.motor, args.steps, args.speed)
            time.sleep(2)
        elif args.relay:
            if not args.relay_state:
                print("ERROR: --relay-state required when using --relay")
                sys.exit(1)
            tester.test_relay(args.relay, args.relay_state)
            time.sleep(1)
        else:
            # Interactive mode
            print_menu()
            interactive_mode(tester)
    
    finally:
        tester.disconnect()


if __name__ == '__main__':
    main()
