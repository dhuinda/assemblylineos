# Arduino Setup Instructions

## Quick Start

1. **Upload Firmware**
   - Open `assembly_line_control.ino` in Arduino IDE
   - Select your board: Tools → Board → Arduino Giga
   - Select port: Tools → Port → (your Arduino Giga port)
   - Click Upload

2. **Verify Connection**
   - Open Serial Monitor (Tools → Serial Monitor)
   - Set baud rate to 115200
   - You should see: "Arduino Giga Assembly Line Control Ready"

## Hardware Connections

### Stepper Motors
Connect each stepper motor driver to the Arduino:

| Motor | STEP Pin | DIR Pin | ENABLE Pin |
|-------|----------|---------|------------|
| 1     | 2        | 3       | 4          |
| 2     | 5        | 6       | 7          |
| 3     | 8        | 9       | 10         |
| 4     | 11       | 12      | 13         |

**Note**: Adjust these pin numbers in the code if your hardware uses different pins.

### Relays
Connect relay modules to analog pins (used as digital outputs):

| Relay | Pin |
|-------|-----|
| 1     | A0  |
| 2     | A1  |
| 3     | A2  |
| 4     | A3  |

## Testing

Test the Arduino independently before connecting to Raspberry Pi:

1. Open Serial Monitor (115200 baud)
2. Send motor command:
   ```
   {"type":"motor","motor_id":1,"steps":100,"speed":50.0}
   ```
3. Send relay command:
   ```
   {"type":"relay","relay_id":1,"state":"on"}
   ```

You should see "OK" responses and the hardware should respond accordingly.

## Troubleshooting

- **Port already in use**: Make sure no other program (including Arduino IDE Serial Monitor) has the port open
- **Motors not moving**: 
  - Check enable pin logic (some drivers need HIGH, others LOW)
  - Verify motor driver power supply
  - Check STEP/DIR pin connections
- **Relays not switching**: 
  - Verify relay module power supply
  - Check if relay module is active HIGH or LOW (adjust code if needed)

