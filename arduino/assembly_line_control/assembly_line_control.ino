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
#include <mbed.h>

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
// Compact telemetry cadence:
// one packet contains pot + both motor states to reduce serial overhead.
#define TELEMETRY_MOVING_INTERVAL_MS 60
#define TELEMETRY_IDLE_INTERVAL_MS 180
unsigned long lastTelemetryTime = 0;

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
  volatile long steps_remaining;
  volatile uint16_t step_ticks;
  volatile uint16_t tick_countdown;
  volatile bool is_moving;
  volatile bool direction;
};

MotorState motors[2];

float motor_speeds[2] = {100.0, 100.0};

bool motor_invert_direction[2] = {false, false};

bool relay_states[4] = {false, false, false, false};

#define SERIAL_BUFFER_MAX 512
char serialBuffer[SERIAL_BUFFER_MAX];
size_t serialBufferIndex = 0;

// Max bytes read toward one line per loop() — keeps motion/telemetry fair
#define SERIAL_READ_BUDGET 48

#define DEBUG_SERIAL 0

const unsigned long MIN_STEP_INTERVAL = 200;
const uint32_t MOTOR_TIMER_TICK_US = 50;
mbed::Ticker motorStepTicker;

// Reject clearly invalid configured pins (board-dependent; Giga uses many numbers)
#define PIN_NUMBER_MAX 255

// Output line buffer for JSON telemetry
#define JSON_LINE_BUF 192
char jsonLineBuf[JSON_LINE_BUF];

// Non-blocking serial TX queue: telemetry is enqueued, then drained in small chunks.
// This avoids long Serial.print() blocking pauses that cause motor stutter.
#define SERIAL_TX_QUEUE_SIZE 4096
#define SERIAL_TX_DRAIN_BUDGET 96   // Max bytes written from queue per loop()
char serialTxQueue[SERIAL_TX_QUEUE_SIZE];
size_t serialTxHead = 0;  // Next write position
size_t serialTxTail = 0;  // Next read position
bool serialWasConnected = false;
unsigned long serialLastReadyAnnounceMs = 0;
const unsigned long READY_ANNOUNCE_INTERVAL_MS = 1000;

void readSerialCommands(void);
void processCommandBuf(char* command);
void processEStopCommand(void);
void processConfigCommandBuf(char* command);
void processMotorCommandBuf(char* command);
void processRelayCommandBuf(char* command);
void updateMotors(void);
void stepMotor(int motor_index);
unsigned long calculateStepInterval(float speed_steps_per_sec);
uint16_t calculateStepTicks(float speed_steps_per_sec);
void motorStepTickerISR(void);
void drainSerialTxQueue(void);
void emitCompactTelemetry(unsigned long nowMs);
void clearSerialTxQueue(void);
void flushSerialInputBytes(void);
void handleSerialDisconnectSafety(void);
void updateSerialLinkState(void);

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

static bool serialTxEnqueueLine(const char* line) {
  if (!line) return false;
  size_t n = strlen(line);
  if (n == 0) return false;
  return serialTxEnqueue(line, n);
}

void clearSerialTxQueue(void) {
  serialTxHead = 0;
  serialTxTail = 0;
}

void flushSerialInputBytes(void) {
  while (Serial.available() > 0) {
    (void)Serial.read();
  }
}

static size_t serialTxUsed(void) {
  if (serialTxHead >= serialTxTail) return serialTxHead - serialTxTail;
  return SERIAL_TX_QUEUE_SIZE - (serialTxTail - serialTxHead);
}

static size_t serialTxFree(void) {
  // Keep one slot empty so head == tail means "empty".
  return (SERIAL_TX_QUEUE_SIZE - 1) - serialTxUsed();
}

static bool serialTxEnqueue(const char* data, size_t len) {
  if (!data || len == 0) return true;
  if (len > serialTxFree()) return false;
  for (size_t i = 0; i < len; i++) {
    serialTxQueue[serialTxHead] = data[i];
    serialTxHead++;
    if (serialTxHead >= SERIAL_TX_QUEUE_SIZE) serialTxHead = 0;
  }
  return true;
}

void drainSerialTxQueue(void) {
  if (!Serial) return;
  int canWrite = Serial.availableForWrite();
  size_t budget = 0;
  if (canWrite > 0) {
    budget = (size_t)canWrite;
    if (budget > (size_t)SERIAL_TX_DRAIN_BUDGET) budget = (size_t)SERIAL_TX_DRAIN_BUDGET;
  } else {
    // Some USB CDC stacks intermittently report 0 writable bytes even while
    // writes still succeed. Keep a tiny fallback so host-side telemetry/probes
    // do not stall and trigger false "Arduino down" reports.
    budget = 16;
  }

  while (budget > 0 && serialTxTail != serialTxHead) {
    size_t contiguous = (serialTxHead > serialTxTail)
      ? (serialTxHead - serialTxTail)
      : (SERIAL_TX_QUEUE_SIZE - serialTxTail);
    if (contiguous > budget) contiguous = budget;

    size_t written = Serial.write((const uint8_t*)&serialTxQueue[serialTxTail], contiguous);
    if (written == 0) break;

    serialTxTail += written;
    if (serialTxTail >= SERIAL_TX_QUEUE_SIZE) serialTxTail = 0;
    budget -= written;
  }
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

  for (int i = 0; i < 2; i++) {
    pinMode(MOTOR_PINS[i][0], OUTPUT);
    pinMode(MOTOR_PINS[i][1], OUTPUT);
    digitalWrite(MOTOR_PINS[i][0], LOW);
    digitalWrite(MOTOR_PINS[i][1], LOW);

    motors[i].steps_remaining = 0;
    motors[i].is_moving = false;
    motors[i].direction = true;
    motors[i].step_ticks = calculateStepTicks(motor_speeds[i]);
    motors[i].tick_countdown = motors[i].step_ticks;
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
  serialWasConnected = (bool)Serial;
  serialLastReadyAnnounceMs = 0;
  if (serialWasConnected) {
    serialTxEnqueueLine("{\"type\":\"ready\"}\n");
  }

  motorStepTicker.attach_us(motorStepTickerISR, MOTOR_TIMER_TICK_US);
}

void loop() {
  updateSerialLinkState();

  readSerialCommands();

  unsigned long now = millis();
  emitCompactTelemetry(now);

  // Non-blocking drain of queued telemetry bytes.
  drainSerialTxQueue();
}

void handleSerialDisconnectSafety(void) {
  noInterrupts();
  for (int i = 0; i < 2; i++) {
    motors[i].steps_remaining = 0;
    motors[i].is_moving = false;
    motors[i].tick_countdown = motors[i].step_ticks;
  }
  interrupts();

  for (int i = 0; i < 2; i++) {
    digitalWrite(MOTOR_PINS[i][0], LOW);
  }

  for (int i = 0; i < 4; i++) {
    relay_states[i] = false;
    digitalWrite(RELAY_PINS[i], RELAY_OFF_LEVEL);
  }

  digitalWrite(LED_BUILTIN, LOW);
}

void updateSerialLinkState(void) {
  bool serialConnected = (bool)Serial;
  unsigned long nowMs = millis();

  if (serialConnected != serialWasConnected) {
    serialWasConnected = serialConnected;
    serialBufferIndex = 0;
    flushSerialInputBytes();
    clearSerialTxQueue();

    if (serialConnected) {
      serialTxEnqueueLine("{\"type\":\"ready\",\"event\":\"reconnect\"}\n");
      serialLastReadyAnnounceMs = nowMs;
    } else {
      handleSerialDisconnectSafety();
    }
    return;
  }

  if (!serialConnected) return;

  // Periodic ready beacon while connected in case host missed initial packet.
  if (nowMs - serialLastReadyAnnounceMs >= READY_ANNOUNCE_INTERVAL_MS) {
    serialTxEnqueueLine("{\"type\":\"ready\"}\n");
    serialLastReadyAnnounceMs = nowMs;
  }
}

void emitCompactTelemetry(unsigned long nowMs) {
  bool anyMoving;
  long m1Remaining;
  long m2Remaining;
  bool m1Moving;
  bool m2Moving;
  noInterrupts();
  m1Remaining = motors[0].steps_remaining;
  m2Remaining = motors[1].steps_remaining;
  m1Moving = motors[0].is_moving;
  m2Moving = motors[1].is_moving;
  interrupts();
  anyMoving = m1Moving || m2Moving;
  unsigned long intervalMs = anyMoving ? TELEMETRY_MOVING_INTERVAL_MS : TELEMETRY_IDLE_INTERVAL_MS;
  if (nowMs - lastTelemetryTime < intervalMs) return;
  lastTelemetryTime = nowMs;

  int pot = analogRead(POT_PIN);
  unsigned int s1 = (unsigned int)(motor_speeds[0] + 0.5f);
  unsigned int s2 = (unsigned int)(motor_speeds[1] + 0.5f);
  int n = snprintf(
    jsonLineBuf,
    sizeof(jsonLineBuf),
    "T,%d,%ld,%u,%d,%ld,%u,%d\n",
    pot,
    m1Remaining,
    s1,
    m1Moving ? 1 : 0,
    m2Remaining,
    s2,
    m2Moving ? 1 : 0
  );
  if (n <= 0 || n >= (int)sizeof(jsonLineBuf)) return;

  // Drop telemetry frame if queue is tight; next frame arrives soon.
  if (serialTxFree() < (size_t)n) return;
  (void)serialTxEnqueue(jsonLineBuf, (size_t)n);
}

void readSerialCommands() {
  if (!Serial) return;
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
    serialTxEnqueueLine("{\"type\":\"error\",\"code\":\"unknown_command\"}\n");
  }
}

void processEStopCommand() {
  noInterrupts();
  for (int i = 0; i < 2; i++) {
    motors[i].steps_remaining = 0;
    motors[i].is_moving = false;
    motors[i].tick_countdown = motors[i].step_ticks;
  }
  interrupts();

  for (int i = 0; i < 4; i++) {
    relay_states[i] = false;
    digitalWrite(RELAY_PINS[i], RELAY_OFF_LEVEL);
  }

  digitalWrite(LED_BUILTIN, LOW);
  serialTxEnqueueLine("{\"type\":\"estop_ack\"}\n");
}

void processConfigCommandBuf(char* command) {
  noInterrupts();
  for (int i = 0; i < 2; i++) {
    motors[i].steps_remaining = 0;
    motors[i].is_moving = false;
    motors[i].tick_countdown = motors[i].step_ticks;
  }
  interrupts();

  for (int i = 0; i < 2; i++) {
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

        if (!pinsValidMotor(stepPin, dirPin)) continue;

        int motorIndex = motorId - 1;
        MOTOR_PINS[motorIndex][0] = stepPin;
        MOTOR_PINS[motorIndex][1] = dirPin;
        motor_invert_direction[motorIndex] = motorObjHasInvertTrue(objStart, objEnd);

        pinMode(MOTOR_PINS[motorIndex][0], OUTPUT);
        pinMode(MOTOR_PINS[motorIndex][1], OUTPUT);
        digitalWrite(MOTOR_PINS[motorIndex][0], LOW);
        digitalWrite(MOTOR_PINS[motorIndex][1], LOW);
        noInterrupts();
        motors[motorIndex].step_ticks = calculateStepTicks(motor_speeds[motorIndex]);
        motors[motorIndex].tick_countdown = motors[motorIndex].step_ticks;
        interrupts();

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
        if (!pinValidGeneral(pin)) continue;

        int relayIndex = relayId - 1;
        RELAY_PINS[relayIndex] = pin;
        pinMode(RELAY_PINS[relayIndex], OUTPUT);
        digitalWrite(RELAY_PINS[relayIndex], relay_states[relayIndex] ? RELAY_ON_LEVEL : RELAY_OFF_LEVEL);

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

        numCustomPins++;
        scan = objEnd + 1;
      }
    }
  }
  serialTxEnqueueLine("{\"type\":\"config_ack\"}\n");
}

void processMotorCommandBuf(char* command) {
  int motor_id = extractIntJson(command, "motor_id");
  if (motor_id < 1 || motor_id > 2) {
    serialTxEnqueueLine("{\"type\":\"error\",\"code\":\"invalid_motor_id\"}\n");
    return;
  }

  int motor_index = motor_id - 1;

  long steps = extractLongJson(command, "steps");

  float speed = extractFloatJson(command, "speed");
  if (speed > 0) {
    motor_speeds[motor_index] = constrain(speed, 1.0f, 5000.0f);
    noInterrupts();
    motors[motor_index].step_ticks = calculateStepTicks(motor_speeds[motor_index]);
    if (motors[motor_index].tick_countdown > motors[motor_index].step_ticks) {
      motors[motor_index].tick_countdown = motors[motor_index].step_ticks;
    }
    interrupts();
  }

  bool explicit_stop = jsonHasLiteral(command, "\"stop\":true") || jsonHasLiteral(command, "\"stop\": true");

  if (explicit_stop) {
    noInterrupts();
    motors[motor_index].steps_remaining = 0;
    motors[motor_index].is_moving = false;
    motors[motor_index].tick_countdown = motors[motor_index].step_ticks;
    interrupts();
    digitalWrite(MOTOR_PINS[motor_index][0], LOW);

  } else if (steps != 0) {
    noInterrupts();
    motors[motor_index].steps_remaining += steps;
    bool moving = (motors[motor_index].steps_remaining != 0);
    motors[motor_index].is_moving = moving;
    if (moving) {
      motors[motor_index].direction = (motors[motor_index].steps_remaining > 0);
      motors[motor_index].tick_countdown = 1;
    }
    bool effective_dir = motors[motor_index].direction ^ motor_invert_direction[motor_index];
    interrupts();

    digitalWrite(MOTOR_PINS[motor_index][1], effective_dir ? HIGH : LOW);

    digitalWrite(LED_BUILTIN, HIGH);
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
    serialTxEnqueueLine("{\"type\":\"error\",\"code\":\"invalid_relay_id\"}\n");
    return;
  }

  int relay_index = relay_id - 1;

  char stateBuf[12];
  if (extractStringJson(command, "state", stateBuf, sizeof(stateBuf)) < 0) {
    serialTxEnqueueLine("{\"type\":\"error\",\"code\":\"missing_relay_state\"}\n");
    return;
  }
  strToLowerAscii(stateBuf);

  if (strcmp(stateBuf, "on") == 0) {
    digitalWrite(RELAY_PINS[relay_index], RELAY_ON_LEVEL);
    relay_states[relay_index] = true;
  } else if (strcmp(stateBuf, "off") == 0) {
    digitalWrite(RELAY_PINS[relay_index], RELAY_OFF_LEVEL);
    relay_states[relay_index] = false;
  } else {
    serialTxEnqueueLine("{\"type\":\"error\",\"code\":\"invalid_relay_state\"}\n");
  }
}

void updateMotors() {
  // Timer ISR now drives stepping cadence; keep this function for compatibility.
}

#define STEP_PULSE_WIDTH_US 4

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

uint16_t calculateStepTicks(float speed_steps_per_sec) {
  unsigned long intervalUs = calculateStepInterval(speed_steps_per_sec);
  uint16_t ticks = (uint16_t)((intervalUs + (MOTOR_TIMER_TICK_US / 2)) / MOTOR_TIMER_TICK_US);
  if (ticks == 0) ticks = 1;
  return ticks;
}

void motorStepTickerISR(void) {
  for (int i = 0; i < 2; i++) {
    if (!motors[i].is_moving) continue;
    long remaining = motors[i].steps_remaining;
    if (remaining == 0) {
      motors[i].is_moving = false;
      continue;
    }

    if (motors[i].tick_countdown > 1) {
      motors[i].tick_countdown--;
      continue;
    }

    bool newDirection = (remaining > 0);
    if (motors[i].direction != newDirection) {
      motors[i].direction = newDirection;
      bool effectiveDir = motors[i].direction ^ motor_invert_direction[i];
      digitalWrite(MOTOR_PINS[i][1], effectiveDir ? HIGH : LOW);
    }

    digitalWrite(MOTOR_PINS[i][0], HIGH);
    delayMicroseconds(STEP_PULSE_WIDTH_US);
    digitalWrite(MOTOR_PINS[i][0], LOW);

    if (remaining > 0) remaining--;
    else remaining++;
    motors[i].steps_remaining = remaining;

    if (remaining == 0) {
      motors[i].is_moving = false;
      motors[i].tick_countdown = motors[i].step_ticks;
    } else {
      motors[i].tick_countdown = motors[i].step_ticks;
    }
  }
}
