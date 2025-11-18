# Assembly Line Control - Arduino Giga Firmware

This Arduino sketch controls 4 stepper motors and 4 relays based on commands received from a Raspberry Pi via USB Serial. Uses custom stepper motor control (no external libraries required).

## Hardware Requirements

- **Arduino Giga** (or compatible board)
- 4 Stepper motor drivers (e.g., A4988, DRV8825, TMC2208)
- 4 Relays (or relay modules)
- Appropriate power supplies for motors and relays

## Pin Configuration

### Stepper Motors
- **Motor 1**: STEP=Pin 2, DIR=Pin 3, ENABLE=Pin 4
- **Motor 2**: STEP=Pin 5, DIR=Pin 6, ENABLE=Pin 7
- **Motor 3**: STEP=Pin 8, DIR=Pin 9, ENABLE=Pin 10
- **Motor 4**: STEP=Pin 11, DIR=Pin 12, ENABLE=Pin 13

### Relays
- **Relay 1**: Pin A0
- **Relay 2**: Pin A1
- **Relay 3**: Pin A2
- **Relay 4**: Pin A3

**Note**: Adjust pin numbers in the code to match your hardware configuration.

## Library Requirements

**No external libraries required!** This firmware uses only standard Arduino libraries.

## Communication Protocol

The Arduino receives JSON commands via Serial at 115200 baud:

### Motor Command
```json
{"type":"motor","motor_id":1,"steps":100,"speed":50.0}
```
- `motor_id`: 1-4
- `steps`: Number of steps to move (positive = forward, negative = backward)
- `speed`: Steps per second (1.0 - 200.0)

### Relay Command
```json
{"type":"relay","relay_id":1,"state":"on"}
```
- `relay_id`: 1-4
- `state`: "on" or "off"

## Installation

1. Install the AccelStepper library via Arduino IDE Library Manager
2. Open `assembly_line_control.ino` in Arduino IDE
3. Adjust pin numbers if needed to match your hardware
4. Select your Arduino board and port
5. Upload the sketch

## Testing

After uploading, open the Serial Monitor (115200 baud) to see status messages. You can test by sending commands manually:

```
{"type":"motor","motor_id":1,"steps":100,"speed":50.0}
{"type":"relay","relay_id":1,"state":"on"}
```

## Troubleshooting

- **Motors not moving**: Check enable pin logic (some drivers use HIGH to enable, others use LOW)
- **Wrong direction**: Swap STEP and DIR pins or invert direction in code
- **Serial not working**: Verify baud rate is 115200 and USB cable is connected
- **Relays not switching**: Check relay module logic (some are active LOW, others active HIGH)

