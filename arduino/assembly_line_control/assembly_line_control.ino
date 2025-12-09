/*
 * Assembly Line Control - Arduino Giga Firmware
 * 
 * Receives commands from Raspberry Pi via USB Serial
 * Controls 2 stepper motors (powering the assembly line) and 4 relays
 * 
 * Hardware Requirements:
 * - Arduino Giga
 * - 2 Stepper motor drivers (e.g., A4988, DRV8825, TMC2208)
 * - 4 Relays (or relay modules)
 * 
 * Pin Configuration for Arduino Giga:
 * Motor 1: STEP=2, DIR=3
 * Motor 2: STEP=5, DIR=6
 * 
 * Relay 1: Pin A0
 * Relay 2: Pin A1
 * Relay 3: Pin A2
 * Relay 4: Pin A3
 * 
 * Note: Adjust pin numbers based on your hardware configuration
 * Note: ENABLE pins not used - configure drivers to be always enabled
 * 
 * Motor commands (motor_id 1-2) are received from the control system
 * and control the 2 stepper motors that power the assembly line.
 */

// Motor pin definitions (2 stepper motors)
// Format: {STEP_PIN, DIR_PIN}
const int MOTOR_PINS[2][2] = {
  {2, 3},   // Motor 1: STEP=2, DIR=3
  {5, 6}    // Motor 2: STEP=5, DIR=6
};

// Relay pin definitions
const int RELAY_PINS[4] = {A0, A1, A2, A3};

// Motor state structure
struct MotorState {
  long steps_remaining;      // Steps left to move (can be negative)
  unsigned long step_interval; // Microseconds between steps (calculated from speed)
  unsigned long last_step_time; // Last time a step was taken
  bool is_moving;            // Whether motor is currently moving
  bool direction;            // Current direction (true = forward, false = backward)
};

// Motor states array (2 stepper motors)
MotorState motors[2];

// Motor speeds (steps per second) - default values
float motor_speeds[2] = {100.0, 100.0};

// Relay states
bool relay_states[4] = {false, false, false, false};

// Serial communication
String serialBuffer = "";

// Minimum step interval in microseconds (for maximum speed protection)
const unsigned long MIN_STEP_INTERVAL = 50; // 2000 us = 500 steps/sec max

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect (only needed for native USB)
  }
  delay(1000); // Give time for serial to stabilize
  Serial.println("Arduino Giga Assembly Line Control Ready");
  Serial.println("Configuration: 2 stepper motors, 4 relays");
  
  // Initialize stepper motors (2 motors powering the assembly line)
  for (int i = 0; i < 2; i++) {
    // Set pins as outputs
    pinMode(MOTOR_PINS[i][0], OUTPUT); // STEP
    pinMode(MOTOR_PINS[i][1], OUTPUT); // DIR
    
    // Initialize pins
    digitalWrite(MOTOR_PINS[i][0], LOW); // STEP starts LOW
    digitalWrite(MOTOR_PINS[i][1], LOW); // DIR starts LOW
    
    // Initialize motor state
    motors[i].steps_remaining = 0;
    motors[i].is_moving = false;
    motors[i].direction = true;
    motors[i].last_step_time = 0;
    motors[i].step_interval = calculateStepInterval(motor_speeds[i]);
  }
  
  // Initialize relay pins
  for (int i = 0; i < 4; i++) {
    pinMode(RELAY_PINS[i], OUTPUT);
    digitalWrite(RELAY_PINS[i], LOW); // Start with relays off
  }
  
  // Setup built-in LED for status indication
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // LED on = ready
  
  // Clear serial buffer
  serialBuffer.reserve(256);
  
  Serial.println("Initialization complete");
  Serial.println("Waiting for commands...");
}

void loop() {
  // Read serial commands (non-blocking)
  readSerialCommands();
  
  // Update all motors (non-blocking)
  updateMotors();
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char inChar = Serial.read();
    
    if (inChar == '\n') {
      // Process complete command
      processCommand(serialBuffer);
      serialBuffer = "";
    } else {
      serialBuffer += inChar;
    }
  }
}

void processCommand(String command) {
  // Parse JSON-like command
  // Expected format: {"type":"motor","motor_id":1,"steps":100,"speed":50.0}
  // or with spaces: {"type": "motor", "motor_id": 1, "steps": 100, "speed": 50.0}
  
  command.trim();
  
  // Debug: Echo received command
  Serial.print("Received: ");
  Serial.println(command);
  
  // Check if it's an E-STOP command (highest priority)
  if (command.indexOf("\"type\":\"estop\"") >= 0 || command.indexOf("\"type\": \"estop\"") >= 0) {
    processEStopCommand();
  }
  // Check if it's a motor command (handle both with and without spaces)
  else if (command.indexOf("\"type\":\"motor\"") >= 0 || command.indexOf("\"type\": \"motor\"") >= 0) {
    processMotorCommand(command);
  }
  // Check if it's a relay command (handle both with and without spaces)
  else if (command.indexOf("\"type\":\"relay\"") >= 0 || command.indexOf("\"type\": \"relay\"") >= 0) {
    processRelayCommand(command);
  }
  else {
    Serial.println("ERROR: Unknown command type");
  }
}

void processEStopCommand() {
  Serial.println("E-STOP: Emergency stop activated!");
  
  // Stop all motors immediately
  for (int i = 0; i < 2; i++) {
    motors[i].steps_remaining = 0;
    motors[i].is_moving = false;
  }
  
  // Turn off all relays
  for (int i = 0; i < 4; i++) {
    relay_states[i] = false;
    digitalWrite(RELAY_PINS[i], LOW);
  }
  
  // Visual feedback - set LED to LOW to indicate E-STOP (non-blocking)
  digitalWrite(LED_BUILTIN, LOW);
  
  Serial.println("E-STOP: All motors stopped, all relays off");
}

void processMotorCommand(String command) {
  // Extract motor_id (only 1-2 are valid, matching the control system)
  int motor_id = extractInt(command, "motor_id");
  if (motor_id < 1 || motor_id > 2) {
    Serial.print("ERROR: Invalid motor_id: ");
    Serial.print(motor_id);
    Serial.println(" (valid range: 1-2)");
    return;
  }
  
  int motor_index = motor_id - 1; // Convert to 0-based index
  
  // Extract steps
  int steps = extractInt(command, "steps");
  
  // Extract speed (if provided)
  float speed = extractFloat(command, "speed");
  if (speed > 0) {
    motor_speeds[motor_index] = constrain(speed, 1.0, 6500);
    motors[motor_index].step_interval = calculateStepInterval(motor_speeds[motor_index]);
  }
  
  // Check for explicit stop flag (stop:true in JSON forces motor to stop)
  bool explicit_stop = (command.indexOf("\"stop\":true") >= 0 || command.indexOf("\"stop\": true") >= 0);
  
  // Update motor state
  if (explicit_stop) {
    // Explicit stop command - always stop the motor
    motors[motor_index].steps_remaining = 0;
    motors[motor_index].is_moving = false;
    digitalWrite(MOTOR_PINS[motor_index][0], LOW);
    
    Serial.print("STOP: Motor ");
    Serial.print(motor_id);
    Serial.println(" stopped (explicit)");
  } else if (steps == 0) {
    // steps=0 without explicit stop = speed-only update
    // Don't stop the motor, just update speed (already done above)
    Serial.print("SPEED UPDATE: Motor ");
    Serial.print(motor_id);
    Serial.print(" speed=");
    Serial.println(motor_speeds[motor_index]);
  } else {
    motors[motor_index].steps_remaining += steps;
    motors[motor_index].is_moving = true;
    motors[motor_index].direction = (steps > 0);
    motors[motor_index].last_step_time = micros();
    
    // Set direction pin
    digitalWrite(MOTOR_PINS[motor_index][1], motors[motor_index].direction ? HIGH : LOW);
    
    // Visual feedback - set LED HIGH when motor starts (non-blocking)
    digitalWrite(LED_BUILTIN, HIGH);
  }
  
  // Send acknowledgment
  Serial.print("OK: Motor ");
  Serial.print(motor_id);
  Serial.print(" steps=");
  Serial.print(steps);
  Serial.print(" total_remaining=");
  Serial.print(motors[motor_index].steps_remaining);
  Serial.print(" speed=");
  Serial.println(motor_speeds[motor_index]);
}

void processRelayCommand(String command) {
  // Extract relay_id
  int relay_id = extractInt(command, "relay_id");
  if (relay_id < 1 || relay_id > 4) {
    Serial.print("ERROR: Invalid relay_id: ");
    Serial.println(relay_id);
    return;
  }
  
  int relay_index = relay_id - 1; // Convert to 0-based index
  
  // Extract state
  String state = extractString(command, "state");
  state.toLowerCase();
  
  if (state == "on") {
    digitalWrite(RELAY_PINS[relay_index], HIGH);
    relay_states[relay_index] = true;
    Serial.print("OK: Relay ");
    Serial.print(relay_id);
    Serial.println(" ON");
  } else if (state == "off") {
    digitalWrite(RELAY_PINS[relay_index], LOW);
    relay_states[relay_index] = false;
    Serial.print("OK: Relay ");
    Serial.print(relay_id);
    Serial.println(" OFF");
  } else {
    Serial.print("ERROR: Invalid relay state: ");
    Serial.println(state);
  }
}

void updateMotors() {
  unsigned long current_time = micros();
  
  // Update stepper motors (2 motors powering the assembly line)
  // Each motor runs independently, enabling true parallel execution
  for (int i = 0; i < 2; i++) {
    if (motors[i].is_moving && motors[i].steps_remaining != 0) {
      // Update direction pin based on current steps_remaining sign
      // This ensures direction is correct even if new commands change the sign
      bool new_direction = (motors[i].steps_remaining > 0);
      if (motors[i].direction != new_direction) {
        motors[i].direction = new_direction;
        digitalWrite(MOTOR_PINS[i][1], motors[i].direction ? HIGH : LOW);
      }
      
      // Check if it's time to take a step
      if ((current_time - motors[i].last_step_time) >= motors[i].step_interval) {
        // Take a step
        stepMotor(i);
        
        // Update state
        if (motors[i].steps_remaining > 0) {
          motors[i].steps_remaining--;
        } else {
          motors[i].steps_remaining++;
        }
        
        motors[i].last_step_time = current_time;
        
        // Check if motor has finished
        if (motors[i].steps_remaining == 0) {
          motors[i].is_moving = false;
          // Keep STEP pin LOW when not moving
          digitalWrite(MOTOR_PINS[i][0], LOW);
        }
      }
    }
  }
}

void stepMotor(int motor_index) {
  // Generate a step pulse for a stepper motor
  // Most stepper drivers need a rising edge on STEP pin
  digitalWrite(MOTOR_PINS[motor_index][0], HIGH);
  delayMicroseconds(2); // Minimum pulse width (usually 1-2 microseconds)
  digitalWrite(MOTOR_PINS[motor_index][0], LOW);
}

unsigned long calculateStepInterval(float speed_steps_per_sec) {
  // Convert steps per second to microseconds between steps
  if (speed_steps_per_sec <= 0) {
    speed_steps_per_sec = 1.0; // Minimum speed
  }
  
  unsigned long interval = (unsigned long)(1000000.0 / speed_steps_per_sec);
  
  // Enforce minimum interval for safety
  if (interval < MIN_STEP_INTERVAL) {
    interval = MIN_STEP_INTERVAL;
  }
  
  return interval;
}

// Helper function to extract integer value from JSON string
int extractInt(String json, String key) {
  String searchKey = "\"" + key + "\":";
  int keyIndex = json.indexOf(searchKey);
  if (keyIndex < 0) {
    return 0;
  }
  
  int valueStart = keyIndex + searchKey.length();
  int valueEnd = json.indexOf(',', valueStart);
  if (valueEnd < 0) {
    valueEnd = json.indexOf('}', valueStart);
  }
  if (valueEnd < 0) {
    return 0;
  }
  
  String valueStr = json.substring(valueStart, valueEnd);
  valueStr.trim();
  return valueStr.toInt();
}

// Helper function to extract float value from JSON string
float extractFloat(String json, String key) {
  String searchKey = "\"" + key + "\":";
  int keyIndex = json.indexOf(searchKey);
  if (keyIndex < 0) {
    return 0.0;
  }
  
  int valueStart = keyIndex + searchKey.length();
  int valueEnd = json.indexOf(',', valueStart);
  if (valueEnd < 0) {
    valueEnd = json.indexOf('}', valueStart);
  }
  if (valueEnd < 0) {
    return 0.0;
  }
  
  String valueStr = json.substring(valueStart, valueEnd);
  valueStr.trim();
  return valueStr.toFloat();
}

// Helper function to extract string value from JSON string
// Handles both "key":"value" and "key": "value" (with space after colon)
String extractString(String json, String key) {
  // Try without space first: "key":"value"
  String searchKey = "\"" + key + "\":\"";
  int keyIndex = json.indexOf(searchKey);
  
  // If not found, try with space: "key": "value"
  if (keyIndex < 0) {
    searchKey = "\"" + key + "\": \"";
    keyIndex = json.indexOf(searchKey);
  }
  
  if (keyIndex < 0) {
    return "";
  }
  
  int valueStart = keyIndex + searchKey.length();
  int valueEnd = json.indexOf('"', valueStart);
  if (valueEnd < 0) {
    return "";
  }
  
  return json.substring(valueStart, valueEnd);
}
