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

#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// Motor pin definitions (2 stepper motors) - can be reconfigured via serial
// Format: {STEP_PIN, DIR_PIN}
int MOTOR_PINS[2][2] = {
  {2, 3},   // Motor 1: STEP=2, DIR=3 (default)
  {5, 6}    // Motor 2: STEP=5, DIR=6 (default)
};

// Relay pin definitions - can be reconfigured via serial
int RELAY_PINS[4] = {A0, A1, A2, A3};  // Default: A0-A3 (pins 54-57)

// Potentiometer for variable motor speed (analog 0-1023, reported over serial)
#define POT_PIN A6
#define POT_REPORT_INTERVAL_MS 80
unsigned long lastPotReportTime = 0;

// Motor status: report steps_remaining and speed so host can show accurate time remaining
#define MOTOR_STATUS_INTERVAL_MS 100   // When any motor is moving
#define MOTOR_STATUS_IDLE_INTERVAL_MS 500  // When all motors idle
unsigned long lastMotorStatusTime = 0;

// Relay logic: typical boards are active-LOW (LOW = relay ON, HIGH = relay OFF).
#define RELAY_OFF_LEVEL HIGH
#define RELAY_ON_LEVEL  LOW

// Custom pins - for user-defined I/O
#define MAX_CUSTOM_PINS 16
struct CustomPin {
  char name[32];
  int pin;
  int mode;  // 0=INPUT, 1=OUTPUT, 2=INPUT_PULLUP
  bool configured;
};
CustomPin customPins[MAX_CUSTOM_PINS];
int numCustomPins = 0;

// Motor state structure
struct MotorState {
  long steps_remaining;
  unsigned long step_interval;
  unsigned long last_step_time;
  bool is_moving;
  bool direction;
};

MotorState motors[2];

float motor_speeds[2] = {100.0, 100.0};

bool motor_invert_direction[2] = {false, false};

bool relay_states[4] = {false, false, false, false};

#define SERIAL_BUFFER_MAX 512
char serialBuffer[SERIAL_BUFFER_MAX];
size_t serialBufferIndex = 0;

// Max bytes read toward one line per loop() — keeps motion/telemetry fair
#define SERIAL_READ_BUDGET 96

#define DEBUG_SERIAL 0

const unsigned long MIN_STEP_INTERVAL = 200;

// Catch-up stepping (per motor per loop)
#define MAX_STEPS_PER_MOTOR_LOOP 24
// If we fall behind more than this many step intervals, snap schedule to avoid a huge burst
#define MAX_STEP_LAG_INTERVALS 8

// Reject clearly invalid configured pins (board-dependent; Giga uses many numbers)
#define PIN_NUMBER_MAX 255

// TX headroom before emitting telemetry (skip cycle if buffer is tight).
// Keep these small: on some Arduino cores availableForWrite() can top out near 63.
#define SERIAL_TX_HEADROOM_TELEM 24
// Short analog JSON line ~45 bytes; actual line-size check below is authoritative.
#define SERIAL_TX_HEADROOM_ANALOG 16
#define SERIAL_TX_HEADROOM_VERBOSE 8

// Output line buffer for JSON telemetry
#define JSON_LINE_BUF 192
char jsonLineBuf[JSON_LINE_BUF];

void readSerialCommands(void);
void processCommandBuf(char* command);
void processEStopCommand(void);
void processConfigCommandBuf(char* command);
void processMotorCommandBuf(char* command);
void processRelayCommandBuf(char* command);
void updateMotors(void);
void stepMotor(int motor_index);
unsigned long calculateStepInterval(float speed_steps_per_sec);

static void trimBufferInPlace(char* s) {
  if (!s || !*s) return;
  char* p = s;
  while (*p && isspace((unsigned char)*p)) p++;
  if (p != s) {
    memmove(s, p, strlen(p) + 1);
  }
  size_t L = strlen(s);
  while (L > 0 && isspace((unsigned char)s[L - 1])) L--;
  s[L] = '\0';
}

static bool serialCanWrite(int minFree) {
  int n = Serial.availableForWrite();
  if (n < 0) return true;
  return n >= minFree;
}

static void serialPrintLnPriority(const char* msg) {
  Serial.println(msg);
}

// Find "\"key\":" then first non-space after colon
static const char* jsonValuePtr(const char* json, const char* key) {
  if (!json || !key) return nullptr;
  char pattern[40];
  snprintf(pattern, sizeof(pattern), "\"%s\":", key);
  const char* p = strstr(json, pattern);
  if (!p) return nullptr;
  p += strlen(pattern);
  while (*p == ' ' || *p == '\t') p++;
  return p;
}

static int extractIntJson(const char* json, const char* key) {
  const char* p = jsonValuePtr(json, key);
  if (!p) return 0;
  return (int)strtol(p, nullptr, 10);
}

static long extractLongJson(const char* json, const char* key) {
  const char* p = jsonValuePtr(json, key);
  if (!p) return 0;
  return strtol(p, nullptr, 10);
}

static float extractFloatJson(const char* json, const char* key) {
  const char* p = jsonValuePtr(json, key);
  if (!p) return 0.0f;
  return strtof(p, nullptr);
}

// Copy quoted string value at p (p must point to opening ")
static int extractQuotedStringAt(const char* p, char* out, size_t outSz) {
  if (!p || *p != '"' || !out || outSz == 0) return -1;
  p++;
  size_t i = 0;
  while (*p && *p != '"' && i < outSz - 1) {
    out[i++] = *p++;
  }
  out[i] = '\0';
  if (*p != '"') {
    out[0] = '\0';
    return -1;
  }
  return (int)i;
}

static int extractStringJson(const char* json, const char* key, char* out, size_t outSz) {
  const char* p = jsonValuePtr(json, key);
  if (!p || *p != '"') return -1;
  return extractQuotedStringAt(p, out, outSz);
}

static bool jsonHasLiteral(const char* json, const char* lit) {
  return json && strstr(json, lit) != nullptr;
}

static bool pinsValidMotor(int stepPin, int dirPin) {
  if (stepPin <= 0 || dirPin <= 0) return false;
  if (stepPin > PIN_NUMBER_MAX || dirPin > PIN_NUMBER_MAX) return false;
  if (stepPin == dirPin) return false;
  return true;
}

static bool pinValidGeneral(int pin) {
  if (pin < 0 || pin > PIN_NUMBER_MAX) return false;
  return true;
}

// Find "\"id\":<n>" or "\"id\": <n>" in [start,end), ensuring it is not id 10/12/etc.
static const char* findIdInArray(const char* start, const char* end, int id) {
  if (!start || !end || start >= end) return nullptr;
  char pat[24];
  snprintf(pat, sizeof(pat), "\"id\":%d", id);
  const char* p = start;
  while (p < end && (p = strstr(p, pat)) != nullptr) {
    if (p >= end) return nullptr;
    const char* after = p + strlen(pat);
    if (after < end && isdigit((unsigned char)*after)) {
      p++;
      continue;
    }
    return p;
  }
  snprintf(pat, sizeof(pat), "\"id\": %d", id);
  p = start;
  while (p < end && (p = strstr(p, pat)) != nullptr) {
    if (p >= end) return nullptr;
    const char* after = p + strlen(pat);
    if (after < end && isdigit((unsigned char)*after)) {
      p++;
      continue;
    }
    return p;
  }
  return nullptr;
}

// Find last '{' at or before pos within [start, pos]
static const char* findObjectStart(const char* start, const char* pos) {
  const char* p = pos;
  while (p >= start) {
    if (*p == '{') return p;
    p--;
  }
  return nullptr;
}

// Pointer range [objStart, objEnd] inclusive end at closing brace
static bool motorObjHasInvertTrue(const char* objStart, const char* objEnd) {
  if (!objStart || !objEnd || objEnd < objStart) return false;
  size_t n = (size_t)(objEnd - objStart + 1);
  if (n >= SERIAL_BUFFER_MAX) n = SERIAL_BUFFER_MAX - 1;
  char scratch[128];
  if (n >= sizeof(scratch)) n = sizeof(scratch) - 1;
  memcpy(scratch, objStart, n);
  scratch[n] = '\0';
  return strstr(scratch, "\"invert_direction\":true") != nullptr
    || strstr(scratch, "\"invert_direction\": true") != nullptr;
}

void setup() {
  Serial.begin(115200);
  unsigned long serialWaitStart = millis();
  while (!Serial && (millis() - serialWaitStart < 3000)) {
    delay(1);
  }
  delay(200);

  Serial.println("Arduino Giga Assembly Line Control Ready");
  Serial.println("Configuration: 2 stepper motors, 4 relays");

  for (int i = 0; i < 2; i++) {
    pinMode(MOTOR_PINS[i][0], OUTPUT);
    pinMode(MOTOR_PINS[i][1], OUTPUT);
    digitalWrite(MOTOR_PINS[i][0], LOW);
    digitalWrite(MOTOR_PINS[i][1], LOW);

    motors[i].steps_remaining = 0;
    motors[i].is_moving = false;
    motors[i].direction = true;
    motors[i].last_step_time = 0;
    motors[i].step_interval = calculateStepInterval(motor_speeds[i]);
  }

  for (int i = 0; i < 4; i++) {
    pinMode(RELAY_PINS[i], OUTPUT);
    digitalWrite(RELAY_PINS[i], RELAY_OFF_LEVEL);
    relay_states[i] = false;
  }

  pinMode(POT_PIN, INPUT);

  for (int i = 0; i < MAX_CUSTOM_PINS; i++) {
    customPins[i].name[0] = '\0';
    customPins[i].pin = -1;
    customPins[i].mode = 0;
    customPins[i].configured = false;
  }
  numCustomPins = 0;

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  serialBufferIndex = 0;

  Serial.println("Initialization complete");
  Serial.println("Waiting for commands...");
}

void loop() {
  updateMotors();
  readSerialCommands();

  unsigned long now = millis();

  // Potentiometer reporting — runs FIRST so it isn't starved by motor_status.
  // Uses Serial.print() (which may briefly block if the TX buffer is full but
  // will always complete once the host drains bytes).  The old approach gated
  // on serialCanWrite(~49) which silently dropped every write when the Giga's
  // USB-CDC availableForWrite() returned 0 during the initial handshake or
  // under momentary back-pressure.
  if (now - lastPotReportTime >= POT_REPORT_INTERVAL_MS) {
    lastPotReportTime = now;
    if (Serial) {
      int val = analogRead(POT_PIN);
      Serial.print("{\"type\":\"analog\",\"pin\":\"pot\",\"value\":");
      Serial.print(val);
      Serial.println("}");
    }
  }

  // Motor status reporting — same fix: write directly via Serial.print()
  // instead of gating on serialCanWrite() which can starve output on USB-CDC.
  bool anyMoving = motors[0].is_moving || motors[1].is_moving;
  unsigned long statusInterval = anyMoving ? MOTOR_STATUS_INTERVAL_MS : MOTOR_STATUS_IDLE_INTERVAL_MS;
  if (now - lastMotorStatusTime >= statusInterval) {
    lastMotorStatusTime = now;
    if (Serial) {
      for (int i = 0; i < 2; i++) {
        Serial.print("{\"type\":\"motor_status\",\"motor_id\":");
        Serial.print(i + 1);
        Serial.print(",\"steps_remaining\":");
        Serial.print(motors[i].steps_remaining);
        Serial.print(",\"speed\":");
        Serial.print(motor_speeds[i]);
        Serial.print(",\"is_moving\":");
        Serial.print(motors[i].is_moving ? "true" : "false");
        Serial.println("}");
      }
    }
  }
}

void readSerialCommands() {
  int budget = SERIAL_READ_BUDGET;
  while (Serial.available() > 0 && budget-- > 0) {
    char inChar = (char)Serial.read();

    if (inChar == '\n' || inChar == '\r') {
      if (serialBufferIndex > 0) {
        serialBuffer[serialBufferIndex] = '\0';
        processCommandBuf(serialBuffer);
        serialBufferIndex = 0;
      }
      return;
    }

    if (serialBufferIndex < (SERIAL_BUFFER_MAX - 1)) {
      serialBuffer[serialBufferIndex++] = inChar;
    } else {
      serialBufferIndex = 0;
      while (Serial.available() > 0 && budget-- > 0) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') return;
      }
      return;
    }
  }
}

void processCommandBuf(char* command) {
  trimBufferInPlace(command);
  if (!command[0]) return;

#if DEBUG_SERIAL
  if (Serial) {
    Serial.print("Received: ");
    Serial.println(command);
  }
#endif

  if (strstr(command, "\"type\":\"estop\"") || strstr(command, "\"type\": \"estop\"")) {
    processEStopCommand();
  } else if (strstr(command, "\"type\":\"config\"") || strstr(command, "\"type\": \"config\"")) {
    processConfigCommandBuf(command);
  } else if (strstr(command, "\"type\":\"motor\"") || strstr(command, "\"type\": \"motor\"")) {
    processMotorCommandBuf(command);
  } else if (strstr(command, "\"type\":\"relay\"") || strstr(command, "\"type\": \"relay\"")) {
    processRelayCommandBuf(command);
  } else {
    serialPrintLnPriority("ERROR: Unknown command type");
  }
}

void processEStopCommand() {
  serialPrintLnPriority("E-STOP: Emergency stop activated!");

  for (int i = 0; i < 2; i++) {
    motors[i].steps_remaining = 0;
    motors[i].is_moving = false;
  }

  for (int i = 0; i < 4; i++) {
    relay_states[i] = false;
    digitalWrite(RELAY_PINS[i], RELAY_OFF_LEVEL);
  }

  digitalWrite(LED_BUILTIN, LOW);

  serialPrintLnPriority("E-STOP: All motors stopped, all relays off");
}

void processConfigCommandBuf(char* command) {
  serialPrintLnPriority("CONFIG: Processing pin configuration...");

  for (int i = 0; i < 2; i++) {
    motors[i].steps_remaining = 0;
    motors[i].is_moving = false;
    digitalWrite(MOTOR_PINS[i][0], LOW);
  }

  const char* motorsKey = strstr(command, "\"motors\"");
  if (motorsKey) {
    const char* arrayStart = strchr(motorsKey, '[');
    const char* arrayEnd = arrayStart ? strchr(arrayStart, ']') : nullptr;
    if (arrayStart && arrayEnd && arrayEnd > arrayStart) {
      for (int motorId = 1; motorId <= 2; motorId++) {
        const char* idPos = findIdInArray(arrayStart, arrayEnd, motorId);
        if (!idPos || idPos >= arrayEnd) continue;

        const char* objStart = findObjectStart(arrayStart, idPos);
        if (!objStart || objStart >= idPos) continue;
        const char* objEnd = strchr(idPos, '}');
        if (!objEnd || objEnd > arrayEnd) continue;

        size_t objLen = (size_t)(objEnd - objStart + 1);
        if (objLen >= sizeof(jsonLineBuf)) objLen = sizeof(jsonLineBuf) - 1;
        memcpy(jsonLineBuf, objStart, objLen);
        jsonLineBuf[objLen] = '\0';

        int stepPin = extractIntJson(jsonLineBuf, "step_pin");
        int dirPin = extractIntJson(jsonLineBuf, "dir_pin");

        if (!pinsValidMotor(stepPin, dirPin)) {
          if (Serial) {
            Serial.print("ERROR: Invalid motor pins step=");
            Serial.print(stepPin);
            Serial.print(" dir=");
            Serial.println(dirPin);
          }
          continue;
        }

        int motorIndex = motorId - 1;
        MOTOR_PINS[motorIndex][0] = stepPin;
        MOTOR_PINS[motorIndex][1] = dirPin;
        motor_invert_direction[motorIndex] = motorObjHasInvertTrue(objStart, objEnd);

        pinMode(MOTOR_PINS[motorIndex][0], OUTPUT);
        pinMode(MOTOR_PINS[motorIndex][1], OUTPUT);
        digitalWrite(MOTOR_PINS[motorIndex][0], LOW);
        digitalWrite(MOTOR_PINS[motorIndex][1], LOW);
        motors[motorIndex].step_interval = calculateStepInterval(motor_speeds[motorIndex]);

        if (Serial) {
          Serial.print("CONFIG: Motor ");
          Serial.print(motorId);
          Serial.print(" -> STEP=");
          Serial.print(stepPin);
          Serial.print(", DIR=");
          Serial.print(dirPin);
          Serial.print(", invert=");
          Serial.println(motor_invert_direction[motorIndex] ? 1 : 0);
        }
      }
    }
  }

  const char* relaysKey = strstr(command, "\"relays\"");
  if (relaysKey) {
    const char* arrayStart = strchr(relaysKey, '[');
    const char* arrayEnd = arrayStart ? strchr(arrayStart, ']') : nullptr;
    if (arrayStart && arrayEnd && arrayEnd > arrayStart) {
      for (int relayId = 1; relayId <= 4; relayId++) {
        const char* idPos = findIdInArray(arrayStart, arrayEnd, relayId);
        if (!idPos || idPos >= arrayEnd) continue;

        const char* objStart = findObjectStart(arrayStart, idPos);
        if (!objStart || objStart >= idPos) continue;
        const char* objEnd = strchr(idPos, '}');
        if (!objEnd || objEnd > arrayEnd) continue;

        size_t objLen = (size_t)(objEnd - objStart + 1);
        if (objLen >= sizeof(jsonLineBuf)) objLen = sizeof(jsonLineBuf) - 1;
        memcpy(jsonLineBuf, objStart, objLen);
        jsonLineBuf[objLen] = '\0';

        int pin = extractIntJson(jsonLineBuf, "pin");
        if (!pinValidGeneral(pin)) {
          if (Serial) {
            Serial.print("ERROR: Invalid relay pin ");
            Serial.println(pin);
          }
          continue;
        }

        int relayIndex = relayId - 1;
        RELAY_PINS[relayIndex] = pin;
        pinMode(RELAY_PINS[relayIndex], OUTPUT);
        digitalWrite(RELAY_PINS[relayIndex], relay_states[relayIndex] ? RELAY_ON_LEVEL : RELAY_OFF_LEVEL);

        if (Serial) {
          Serial.print("CONFIG: Relay ");
          Serial.print(relayId);
          Serial.print(" -> Pin=");
          Serial.println(pin);
        }
      }
    }
  }

  const char* customKey = strstr(command, "\"custom\"");
  if (customKey) {
    const char* arrayStart = strchr(customKey, '[');
    const char* arrayEnd = arrayStart ? strchr(arrayStart, ']') : nullptr;
    if (arrayStart && arrayEnd && arrayEnd > arrayStart) {
      const char* innerBegin = arrayStart + 1;
      numCustomPins = 0;
      for (int i = 0; i < MAX_CUSTOM_PINS; i++) {
        customPins[i].configured = false;
      }

      const char* scan = innerBegin;
      while (numCustomPins < MAX_CUSTOM_PINS && scan < arrayEnd) {
        const char* objStart = strchr(scan, '{');
        if (!objStart || objStart >= arrayEnd) break;
        const char* objEnd = strchr(objStart, '}');
        if (!objEnd || objEnd > arrayEnd) break;

        size_t objLen = (size_t)(objEnd - objStart + 1);
        if (objLen >= sizeof(jsonLineBuf)) objLen = sizeof(jsonLineBuf) - 1;
        memcpy(jsonLineBuf, objStart, objLen);
        jsonLineBuf[objLen] = '\0';

        char nameBuf[32];
        if (extractStringJson(jsonLineBuf, "name", nameBuf, sizeof(nameBuf)) < 0) {
          scan = objEnd + 1;
          continue;
        }

        int pin = extractIntJson(jsonLineBuf, "pin");
        if (!pinValidGeneral(pin)) {
          scan = objEnd + 1;
          continue;
        }

        char modeBuf[20];
        if (extractStringJson(jsonLineBuf, "mode", modeBuf, sizeof(modeBuf)) < 0) {
          modeBuf[0] = '\0';
        }

        strncpy(customPins[numCustomPins].name, nameBuf, sizeof(customPins[numCustomPins].name) - 1);
        customPins[numCustomPins].name[sizeof(customPins[numCustomPins].name) - 1] = '\0';
        customPins[numCustomPins].pin = pin;

        if (strcmp(modeBuf, "output") == 0) {
          customPins[numCustomPins].mode = 1;
          pinMode(pin, OUTPUT);
          digitalWrite(pin, LOW);
        } else if (strcmp(modeBuf, "input_pullup") == 0) {
          customPins[numCustomPins].mode = 2;
          pinMode(pin, INPUT_PULLUP);
        } else {
          customPins[numCustomPins].mode = 0;
          pinMode(pin, INPUT);
        }

        customPins[numCustomPins].configured = true;

        if (Serial) {
          Serial.print("CONFIG: Custom pin '");
          Serial.print(customPins[numCustomPins].name);
          Serial.print("' -> Pin=");
          Serial.print(pin);
          Serial.print(", Mode=");
          Serial.println(modeBuf[0] ? modeBuf : "input");
        }

        numCustomPins++;
        scan = objEnd + 1;
      }
    }
  }

  serialPrintLnPriority("CONFIG: Pin configuration applied successfully");
  if (Serial) {
    Serial.print("CONFIG: ");
    Serial.print(numCustomPins);
    Serial.println(" custom pins configured");
  }
}

void processMotorCommandBuf(char* command) {
  int motor_id = extractIntJson(command, "motor_id");
  if (motor_id < 1 || motor_id > 2) {
    if (Serial) {
      Serial.print("ERROR: Invalid motor_id: ");
      Serial.print(motor_id);
      Serial.println(" (valid range: 1-2)");
    }
    return;
  }

  int motor_index = motor_id - 1;

  long steps = extractLongJson(command, "steps");

  float speed = extractFloatJson(command, "speed");
  if (speed > 0) {
    motor_speeds[motor_index] = constrain(speed, 1.0f, 5000.0f);
    motors[motor_index].step_interval = calculateStepInterval(motor_speeds[motor_index]);
  }

  bool explicit_stop = jsonHasLiteral(command, "\"stop\":true") || jsonHasLiteral(command, "\"stop\": true");

  if (explicit_stop) {
    motors[motor_index].steps_remaining = 0;
    motors[motor_index].is_moving = false;
    digitalWrite(MOTOR_PINS[motor_index][0], LOW);

    if (Serial) {
      Serial.print("STOP: Motor ");
      Serial.print(motor_id);
      Serial.println(" stopped (explicit)");
    }
  } else if (steps == 0) {
    if (Serial) {
      Serial.print("SPEED UPDATE: Motor ");
      Serial.print(motor_id);
      Serial.print(" speed=");
      Serial.println(motor_speeds[motor_index]);
    }
  } else {
    motors[motor_index].steps_remaining += steps;
    motors[motor_index].is_moving = true;
    motors[motor_index].direction = (steps > 0);
    motors[motor_index].last_step_time = micros();

    bool effective_dir = motors[motor_index].direction ^ motor_invert_direction[motor_index];
    digitalWrite(MOTOR_PINS[motor_index][1], effective_dir ? HIGH : LOW);

    digitalWrite(LED_BUILTIN, HIGH);
  }

  if (Serial) {
    Serial.print("OK: Motor ");
    Serial.print(motor_id);
    Serial.print(" steps=");
    Serial.print(steps);
    Serial.print(" total_remaining=");
    Serial.print(motors[motor_index].steps_remaining);
    Serial.print(" speed=");
    Serial.println(motor_speeds[motor_index]);
  }
}

static void strToLowerAscii(char* s) {
  for (; *s; s++) {
    if (*s >= 'A' && *s <= 'Z') *s = (char)(*s - 'A' + 'a');
  }
}

void processRelayCommandBuf(char* command) {
  int relay_id = extractIntJson(command, "relay_id");
  if (relay_id < 1 || relay_id > 4) {
    if (Serial) {
      Serial.print("ERROR: Invalid relay_id: ");
      Serial.println(relay_id);
    }
    return;
  }

  int relay_index = relay_id - 1;

  char stateBuf[12];
  if (extractStringJson(command, "state", stateBuf, sizeof(stateBuf)) < 0) {
    if (Serial) {
      Serial.println("ERROR: Missing relay state");
    }
    return;
  }
  strToLowerAscii(stateBuf);

  if (strcmp(stateBuf, "on") == 0) {
    digitalWrite(RELAY_PINS[relay_index], RELAY_ON_LEVEL);
    relay_states[relay_index] = true;
    if (Serial) {
      Serial.print("OK: Relay ");
      Serial.print(relay_id);
      Serial.println(" ON");
    }
  } else if (strcmp(stateBuf, "off") == 0) {
    digitalWrite(RELAY_PINS[relay_index], RELAY_OFF_LEVEL);
    relay_states[relay_index] = false;
    if (Serial) {
      Serial.print("OK: Relay ");
      Serial.print(relay_id);
      Serial.println(" OFF");
    }
  } else {
    if (Serial) {
      Serial.print("ERROR: Invalid relay state: ");
      Serial.println(stateBuf);
    }
  }
}

void updateMotors() {
  static int rrStart = 0;
  for (int pass = 0; pass < 2; pass++) {
    int i = (rrStart + pass) & 1;
    if (!motors[i].is_moving || motors[i].steps_remaining == 0) {
      continue;
    }

    int stepsThisLoop = 0;
    while (stepsThisLoop < MAX_STEPS_PER_MOTOR_LOOP
           && motors[i].is_moving
           && motors[i].steps_remaining != 0) {
      unsigned long current_time = micros();

      unsigned long overdue = current_time - motors[i].last_step_time;
      unsigned long maxLag = motors[i].step_interval * (unsigned long)MAX_STEP_LAG_INTERVALS;
      if (overdue > maxLag) {
        motors[i].last_step_time = current_time - maxLag;
      }

      if ((micros() - motors[i].last_step_time) < motors[i].step_interval) {
        break;
      }

      current_time = micros();

      bool new_direction = (motors[i].steps_remaining > 0);
      if (motors[i].direction != new_direction) {
        motors[i].direction = new_direction;
        bool effective_dir = motors[i].direction ^ motor_invert_direction[i];
        digitalWrite(MOTOR_PINS[i][1], effective_dir ? HIGH : LOW);
      }

      stepMotor(i);

      if (motors[i].steps_remaining > 0) {
        motors[i].steps_remaining--;
      } else {
        motors[i].steps_remaining++;
      }

      // Use wall-clock pacing (no pulse bursts to "catch up"), which keeps
      // both motors on the same scheduling behavior and avoids grinding bursts.
      motors[i].last_step_time = current_time;

      if (motors[i].steps_remaining == 0) {
        motors[i].is_moving = false;
        digitalWrite(MOTOR_PINS[i][0], LOW);
      }

      stepsThisLoop++;
    }
  }
  rrStart ^= 1;
}

#define STEP_PULSE_WIDTH_US 2

void stepMotor(int motor_index) {
  digitalWrite(MOTOR_PINS[motor_index][0], HIGH);
  delayMicroseconds(STEP_PULSE_WIDTH_US);
  digitalWrite(MOTOR_PINS[motor_index][0], LOW);
}

unsigned long calculateStepInterval(float speed_steps_per_sec) {
  if (speed_steps_per_sec <= 0) {
    speed_steps_per_sec = 1.0f;
  }

  unsigned long interval = (unsigned long)(1000000.0f / speed_steps_per_sec);

  if (interval < MIN_STEP_INTERVAL) {
    interval = MIN_STEP_INTERVAL;
  }

  return interval;
}
