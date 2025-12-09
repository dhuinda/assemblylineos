# Arduino Test Scripts

This folder contains test scripts for the assembly line control system.

## Quick Motor Test

### test_motor1_simple.ino

A very simple standalone Arduino sketch to test Motor 1 hardware.

**What it does:**
- Continuously spins Motor 1 at a constant speed
- No serial communication required
- No Python scripts needed
- Perfect for verifying hardware connections

**Usage:**
1. Open `test_motor1_simple.ino` in Arduino IDE
2. Select your Arduino board and port
3. Upload the sketch
4. Motor 1 should start spinning immediately
5. Open Serial Monitor (115200 baud) to see status messages

**Configuration:**
Edit these constants in the sketch to customize behavior:
- `STEP_INTERVAL` - Speed (lower = faster, higher = slower)
- `DIRECTION` - Change to `LOW` to reverse direction
- Pin numbers if your wiring is different

**Troubleshooting:**
- **Motor not moving**: Check enable pin logic (try changing `LOW` to `HIGH` on line 40)
- **Wrong direction**: Change `DIRECTION` from `HIGH` to `LOW` (or vice versa)
- **Too fast/slow**: Adjust `STEP_INTERVAL` value

---

## Full System Test

### test_firmware.py

Python script to test the full Arduino firmware with all motors and relays.

**Usage:**
```bash
# Auto-detect Arduino and run interactive mode
python3 test_firmware.py

# Specify port
python3 test_firmware.py --port /dev/ttyUSB0

# List available ports
python3 test_firmware.py --list-ports

# Quick test motor 1
python3 test_firmware.py --motor 1 --steps 100

# Run full test sequence
python3 test_firmware.py --test-sequence
```

See the main README for more details.

---

## Main Firmware

### assembly_line_control/assembly_line_control.ino

The main Arduino firmware that integrates with the ROS 2 control system.

This is what you'll use in production - it receives commands from the Raspberry Pi and controls both motors and relays.
