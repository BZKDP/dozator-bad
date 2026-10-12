/*
 * Gate12 dispenser control for ESP32 + PCA9685 + five positional servos.
 *
 * Wiring (PCA9685 default I2C address 0x40):
 *   ESP32 3V3    -> PCA9685 VCC (logic only)
 *   ESP32 GND    -> PCA9685 GND
 *   ESP32 GPIO21 -> PCA9685 SDA
 *   ESP32 GPIO22 -> PCA9685 SCL
 *   external regulated 5 V -> PCA9685 V+ and GND screw terminal
 *   servos 1..5 -> PCA9685 channels 0..4
 *
 * The ESP32 is powered only through its USB connector. The firmware intentionally
 * generates no servo pulse and causes no movement on boot.
 *
 * Serial protocol, 115200:
 *   PING
 *   CH <servo>
 *   INFO
 *   INFO ALL
 *   SET <us>
 *   SET <servo> <us>
 *   STEP <delta_us>
 *   JOG <-1|0|1>
 *   JOG <servo> <-1|0|1>
 *   LIM <min_us> <max_us>
 *   LIM <servo> <min_us> <max_us>
 *   RATE <us_per_sec>
 *   RATE <servo> <us_per_sec>
 *   HOLD <0|1>
 *   HOLD <servo> <0|1>
 *   POWERMAX <ms>
 *   POWERMAX <servo> <ms>
 *   CYCLE <closed_us> <eject_us> <hold_ms>
 *   CYCLE <servo> <closed_us> <eject_us> <hold_ms>
 *   STOP
 *   STOP <servo>
 *   DETACH
 *   DETACH <servo>
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

const long BAUD = 115200;

const uint8_t NUM_SERVOS = 5;
const uint8_t PCA_CHANNELS[NUM_SERVOS] = {0, 1, 2, 3, 4};
const uint8_t PCA_ADDRESS = 0x40;
const uint8_t I2C_SDA_PIN = 21;
const uint8_t I2C_SCL_PIN = 22;
const float SERVO_FREQUENCY_HZ = 50.0f;

const int HARD_MIN_US = 700;
const int HARD_MAX_US = 2300;
const int CENTER_US = 1500;
// Deliberately narrow first-run window. Widen per channel only during calibration.
const int DEFAULT_SAFE_MIN_US = 1450;
const int DEFAULT_SAFE_MAX_US = 1550;
// Equal defaults intentionally block a dispense cycle until this mechanism is calibrated.
const int DEFAULT_CLOSED_US = 1500;
const int DEFAULT_EJECT_US = 1500;
const uint16_t AUTO_DETACH_MS = 250;
const unsigned long DEFAULT_POWER_MAX_MS = 3000;
const unsigned long POWER_MAX_LIMIT_MS = 600000UL;

Adafruit_PWMServoDriver pwm(PCA_ADDRESS);

bool servoAttached[NUM_SERVOS];
bool jogActive[NUM_SERVOS];
bool holdTorque[NUM_SERVOS];
bool cycleActive[NUM_SERVOS];

int currentUs[NUM_SERVOS];
int targetUs[NUM_SERVOS];
int safeMinUs[NUM_SERVOS];
int safeMaxUs[NUM_SERVOS];
int slewUsPerSec[NUM_SERVOS];
int8_t jogDir[NUM_SERVOS];

uint8_t cyclePhase[NUM_SERVOS];
int cycleClosedUs[NUM_SERVOS];
int cycleEjectUs[NUM_SERVOS];
unsigned long cycleHoldUntil[NUM_SERVOS];
unsigned long cycleHoldMs[NUM_SERVOS];

unsigned long lastMotionUs[NUM_SERVOS];
uint32_t motionRemainder[NUM_SERVOS];
unsigned long detachAtMs[NUM_SERVOS];
unsigned long attachedAtMs[NUM_SERVOS];
unsigned long powerMaxMs[NUM_SERVOS];

uint8_t activeServo = 0;
String buf;

bool validServo(int servo) {
  return servo >= 0 && servo < NUM_SERVOS;
}

int clampHard(int us) {
  return constrain(us, HARD_MIN_US, HARD_MAX_US);
}

int clampSafe(uint8_t servo, int us) {
  return constrain(us, safeMinUs[servo], safeMaxUs[servo]);
}

unsigned long clampPowerMax(int ms) {
  if (ms <= 0) {
    return 0;
  }
  if ((unsigned long)ms > POWER_MAX_LIMIT_MS) {
    return POWER_MAX_LIMIT_MS;
  }
  return (unsigned long)ms;
}

bool movingNow(uint8_t servo) {
  return jogActive[servo] || currentUs[servo] != targetUs[servo] || cycleActive[servo];
}

void cancelDetach(uint8_t servo) {
  detachAtMs[servo] = 0;
}

void detachServo(uint8_t servo) {
  if (servoAttached[servo]) {
    // PCA9685 special FULL OFF value: stop sending pulses to this channel.
    pwm.setPWM(PCA_CHANNELS[servo], 0, 4096);
    servoAttached[servo] = false;
  }
  detachAtMs[servo] = 0;
  attachedAtMs[servo] = 0;
}

void scheduleDetach(uint8_t servo) {
  if (!holdTorque[servo] && servoAttached[servo] && !movingNow(servo)) {
    detachAtMs[servo] = millis() + AUTO_DETACH_MS;
  }
}

void attachAt(uint8_t servo, int us) {
  if (!servoAttached[servo]) {
    servoAttached[servo] = true;
    attachedAtMs[servo] = millis();
  }
  currentUs[servo] = clampSafe(servo, us);
  targetUs[servo] = currentUs[servo];
  pwm.writeMicroseconds(PCA_CHANNELS[servo], currentUs[servo]);
  lastMotionUs[servo] = micros();
  motionRemainder[servo] = 0;
  cancelDetach(servo);
}

void beginMoveTo(uint8_t servo, int us) {
  int next = clampSafe(servo, us);
  if (!servoAttached[servo]) {
    attachAt(servo, currentUs[servo]);
  }
  jogActive[servo] = false;
  jogDir[servo] = 0;
  targetUs[servo] = next;
  lastMotionUs[servo] = micros();
  motionRemainder[servo] = 0;
  cancelDetach(servo);
}

void stopMotion(uint8_t servo) {
  updateMotion(servo);
  cycleActive[servo] = false;
  cyclePhase[servo] = 0;
  jogActive[servo] = false;
  jogDir[servo] = 0;
  targetUs[servo] = currentUs[servo];
  if (servoAttached[servo]) {
    pwm.writeMicroseconds(PCA_CHANNELS[servo], currentUs[servo]);
  }
  scheduleDetach(servo);
}

void beginJog(uint8_t servo, int dir) {
  dir = (dir > 0) ? 1 : (dir < 0 ? -1 : 0);
  if (dir == 0) {
    stopMotion(servo);
    return;
  }
  if (!servoAttached[servo]) {
    attachAt(servo, currentUs[servo]);
  }
  cycleActive[servo] = false;
  cyclePhase[servo] = 0;
  jogActive[servo] = true;
  jogDir[servo] = (int8_t)dir;
  targetUs[servo] = currentUs[servo];
  lastMotionUs[servo] = micros();
  motionRemainder[servo] = 0;
  cancelDetach(servo);
}

void updateAutoDetach(uint8_t servo) {
  if (detachAtMs[servo] == 0 || holdTorque[servo] || movingNow(servo)) {
    return;
  }
  if ((long)(millis() - detachAtMs[servo]) >= 0) {
    detachServo(servo);
  }
}

void updatePowerWatchdog(uint8_t servo) {
  if (!servoAttached[servo] || powerMaxMs[servo] == 0 || attachedAtMs[servo] == 0) {
    return;
  }
  if (millis() - attachedAtMs[servo] >= powerMaxMs[servo]) {
    cycleActive[servo] = false;
    cyclePhase[servo] = 0;
    jogActive[servo] = false;
    jogDir[servo] = 0;
    targetUs[servo] = currentUs[servo];
    detachServo(servo);
    Serial.printf("SAFE DETACH %u power_timeout\n", servo);
  }
}

void updateMotion(uint8_t servo) {
  if (!servoAttached[servo] || !movingNow(servo)) {
    lastMotionUs[servo] = micros();
    motionRemainder[servo] = 0;
    return;
  }

  unsigned long now = micros();
  unsigned long dt = now - lastMotionUs[servo];
  if (dt < 1000) {
    return;
  }
  lastMotionUs[servo] = now;

  uint64_t movement = (uint64_t)slewUsPerSec[servo] * (uint64_t)dt + motionRemainder[servo];
  int maxStep = (int)(movement / 1000000ULL);
  motionRemainder[servo] = (uint32_t)(movement % 1000000ULL);
  if (maxStep < 1) {
    return;
  }

  if (jogActive[servo]) {
    int next = currentUs[servo] + (jogDir[servo] > 0 ? maxStep : -maxStep);
    currentUs[servo] = clampSafe(servo, next);
    targetUs[servo] = currentUs[servo];
    if ((jogDir[servo] < 0 && currentUs[servo] <= safeMinUs[servo]) ||
        (jogDir[servo] > 0 && currentUs[servo] >= safeMaxUs[servo])) {
      jogActive[servo] = false;
      jogDir[servo] = 0;
      scheduleDetach(servo);
    }
  } else {
    int delta = targetUs[servo] - currentUs[servo];
    bool reached = false;
    if (abs(delta) <= maxStep) {
      currentUs[servo] = targetUs[servo];
      reached = true;
    } else {
      currentUs[servo] += (delta > 0) ? maxStep : -maxStep;
    }
    if (reached) {
      scheduleDetach(servo);
    }
  }

  pwm.writeMicroseconds(PCA_CHANNELS[servo], currentUs[servo]);
}

void beginCycle(uint8_t servo, int closedUs, int ejectUs, unsigned long holdMs) {
  cycleClosedUs[servo] = clampSafe(servo, closedUs);
  cycleEjectUs[servo] = clampSafe(servo, ejectUs);
  cycleHoldMs[servo] = constrain(holdMs, 50UL, 5000UL);

  if (!servoAttached[servo]) {
    attachAt(servo, currentUs[servo]);
  }

  cycleActive[servo] = true;
  cyclePhase[servo] = 1;
  beginMoveTo(servo, cycleClosedUs[servo]);
}

void updateCycle(uint8_t servo) {
  if (!cycleActive[servo]) {
    return;
  }

  if (cyclePhase[servo] == 1 && currentUs[servo] == targetUs[servo]) {
    beginMoveTo(servo, cycleEjectUs[servo]);
    cyclePhase[servo] = 2;
    return;
  }

  if (cyclePhase[servo] == 2 && currentUs[servo] == targetUs[servo]) {
    cycleHoldUntil[servo] = millis() + cycleHoldMs[servo];
    cyclePhase[servo] = 3;
    return;
  }

  if (cyclePhase[servo] == 3 && (long)(millis() - cycleHoldUntil[servo]) >= 0) {
    beginMoveTo(servo, cycleClosedUs[servo]);
    cyclePhase[servo] = 4;
    return;
  }

  if (cyclePhase[servo] == 4 && currentUs[servo] == targetUs[servo]) {
    cycleActive[servo] = false;
    cyclePhase[servo] = 0;
    scheduleDetach(servo);
    Serial.printf("DONE %u\n", servo);
  }
}

void printInfo(uint8_t servo) {
  unsigned long poweredMs = (servoAttached[servo] && attachedAtMs[servo] != 0) ? millis() - attachedAtMs[servo] : 0;
  Serial.printf(
    "INFO servo=%u pca_ch=%u attached=%d pos=%d target=%d min=%d max=%d rate=%d jog=%d hold=%d cycle=%d pmax=%lu power=%lu\n",
    servo,
    PCA_CHANNELS[servo],
    servoAttached[servo] ? 1 : 0,
    currentUs[servo],
    targetUs[servo],
    safeMinUs[servo],
    safeMaxUs[servo],
    slewUsPerSec[servo],
    jogDir[servo],
    holdTorque[servo] ? 1 : 0,
    cycleActive[servo] ? 1 : 0,
    powerMaxMs[servo],
    poweredMs
  );
}

bool parseOneInt(const String& line, const char* fmt, int& a) {
  return sscanf(line.c_str(), fmt, &a) == 1;
}

bool parseTwoInts(const String& line, const char* fmt, int& a, int& b) {
  return sscanf(line.c_str(), fmt, &a, &b) == 2;
}

bool parseThreeInts(const String& line, const char* fmt, int& a, int& b, int& c) {
  return sscanf(line.c_str(), fmt, &a, &b, &c) == 3;
}

bool parseFourInts(const String& line, const char* fmt, int& a, int& b, int& c, int& d) {
  return sscanf(line.c_str(), fmt, &a, &b, &c, &d) == 4;
}

void printErrServo() {
  Serial.println("ERR servo");
}

void handle(String line) {
  line.trim();
  if (line.length() == 0) {
    return;
  }

  String cmd = line;
  cmd.toUpperCase();

  if (cmd == "PING") {
    Serial.println("PONG");
    return;
  }

  if (cmd == "INFO") {
    printInfo(activeServo);
    return;
  }

  if (cmd == "INFO ALL") {
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
      printInfo(i);
    }
    return;
  }

  int a = 0;
  int b = 0;
  int c = 0;
  int d = 0;

  if (parseOneInt(cmd, "CH %d", a)) {
    if (!validServo(a)) {
      printErrServo();
      return;
    }
    activeServo = (uint8_t)a;
    Serial.printf("OK CH %u\n", activeServo);
    return;
  }

  if (cmd == "STOP") {
    stopMotion(activeServo);
    Serial.printf("OK STOP %u %d\n", activeServo, currentUs[activeServo]);
    return;
  }

  if (parseOneInt(cmd, "STOP %d", a)) {
    if (!validServo(a)) {
      printErrServo();
      return;
    }
    stopMotion((uint8_t)a);
    Serial.printf("OK STOP %d %d\n", a, currentUs[a]);
    return;
  }

  if (cmd == "DETACH") {
    stopMotion(activeServo);
    detachServo(activeServo);
    Serial.printf("OK DETACH %u\n", activeServo);
    return;
  }

  if (parseOneInt(cmd, "DETACH %d", a)) {
    if (!validServo(a)) {
      printErrServo();
      return;
    }
    stopMotion((uint8_t)a);
    detachServo((uint8_t)a);
    Serial.printf("OK DETACH %d\n", a);
    return;
  }

  if (parseTwoInts(cmd, "SET %d %d", a, b)) {
    if (!validServo(a)) {
      printErrServo();
      return;
    }
    beginMoveTo((uint8_t)a, b);
    Serial.printf("OK SET %d %d\n", a, targetUs[a]);
    return;
  }

  if (parseOneInt(cmd, "SET %d", a)) {
    beginMoveTo(activeServo, a);
    Serial.printf("OK SET %u %d\n", activeServo, targetUs[activeServo]);
    return;
  }

  if (parseTwoInts(cmd, "STEP %d %d", a, b)) {
    if (!validServo(a)) {
      printErrServo();
      return;
    }
    beginMoveTo((uint8_t)a, targetUs[a] + b);
    Serial.printf("OK SET %d %d\n", a, targetUs[a]);
    return;
  }

  if (parseOneInt(cmd, "STEP %d", a)) {
    beginMoveTo(activeServo, targetUs[activeServo] + a);
    Serial.printf("OK SET %u %d\n", activeServo, targetUs[activeServo]);
    return;
  }

  if (parseTwoInts(cmd, "JOG %d %d", a, b)) {
    if (!validServo(a)) {
      printErrServo();
      return;
    }
    beginJog((uint8_t)a, b);
    Serial.printf("OK JOG %d %d %d\n", a, jogDir[a], currentUs[a]);
    return;
  }

  if (parseOneInt(cmd, "JOG %d", a)) {
    beginJog(activeServo, a);
    Serial.printf("OK JOG %u %d %d\n", activeServo, jogDir[activeServo], currentUs[activeServo]);
    return;
  }

  if (parseThreeInts(cmd, "LIM %d %d %d", a, b, c)) {
    if (!validServo(a)) {
      printErrServo();
      return;
    }
    b = clampHard(b);
    c = clampHard(c);
    if (b >= c || c - b < 20) {
      Serial.println("ERR bad_limits");
      return;
    }
    safeMinUs[a] = b;
    safeMaxUs[a] = c;
    currentUs[a] = clampSafe((uint8_t)a, currentUs[a]);
    targetUs[a] = clampSafe((uint8_t)a, targetUs[a]);
    if (servoAttached[a]) {
      pwm.writeMicroseconds(PCA_CHANNELS[a], currentUs[a]);
    }
    Serial.printf("OK LIM %d %d %d\n", a, safeMinUs[a], safeMaxUs[a]);
    return;
  }

  if (parseTwoInts(cmd, "LIM %d %d", a, b)) {
    a = clampHard(a);
    b = clampHard(b);
    if (a >= b || b - a < 20) {
      Serial.println("ERR bad_limits");
      return;
    }
    safeMinUs[activeServo] = a;
    safeMaxUs[activeServo] = b;
    currentUs[activeServo] = clampSafe(activeServo, currentUs[activeServo]);
    targetUs[activeServo] = clampSafe(activeServo, targetUs[activeServo]);
    if (servoAttached[activeServo]) {
      pwm.writeMicroseconds(PCA_CHANNELS[activeServo], currentUs[activeServo]);
    }
    Serial.printf("OK LIM %u %d %d\n", activeServo, safeMinUs[activeServo], safeMaxUs[activeServo]);
    return;
  }

  if (parseTwoInts(cmd, "RATE %d %d", a, b)) {
    if (!validServo(a)) {
      printErrServo();
      return;
    }
    slewUsPerSec[a] = constrain(b, 5, 2000);
    Serial.printf("OK RATE %d %d\n", a, slewUsPerSec[a]);
    return;
  }

  if (parseOneInt(cmd, "RATE %d", a)) {
    slewUsPerSec[activeServo] = constrain(a, 5, 2000);
    Serial.printf("OK RATE %u %d\n", activeServo, slewUsPerSec[activeServo]);
    return;
  }

  if (parseTwoInts(cmd, "HOLD %d %d", a, b)) {
    if (!validServo(a)) {
      printErrServo();
      return;
    }
    holdTorque[a] = b != 0;
    if (holdTorque[a]) {
      cancelDetach((uint8_t)a);
    } else {
      scheduleDetach((uint8_t)a);
    }
    Serial.printf("OK HOLD %d %d\n", a, holdTorque[a] ? 1 : 0);
    return;
  }

  if (parseOneInt(cmd, "HOLD %d", a)) {
    holdTorque[activeServo] = a != 0;
    if (holdTorque[activeServo]) {
      cancelDetach(activeServo);
    } else {
      scheduleDetach(activeServo);
    }
    Serial.printf("OK HOLD %u %d\n", activeServo, holdTorque[activeServo] ? 1 : 0);
    return;
  }

  if (parseTwoInts(cmd, "POWERMAX %d %d", a, b)) {
    if (!validServo(a)) {
      printErrServo();
      return;
    }
    powerMaxMs[a] = clampPowerMax(b);
    Serial.printf("OK POWERMAX %d %lu\n", a, powerMaxMs[a]);
    return;
  }

  if (parseOneInt(cmd, "POWERMAX %d", a)) {
    powerMaxMs[activeServo] = clampPowerMax(a);
    Serial.printf("OK POWERMAX %u %lu\n", activeServo, powerMaxMs[activeServo]);
    return;
  }

  if (parseFourInts(cmd, "CYCLE %d %d %d %d", a, b, c, d)) {
    if (!validServo(a)) {
      printErrServo();
      return;
    }
    beginCycle((uint8_t)a, b, c, (unsigned long)d);
    Serial.printf("OK CYCLE %d %d %d %d\n", a, cycleClosedUs[a], cycleEjectUs[a], (int)cycleHoldMs[a]);
    return;
  }

  if (parseThreeInts(cmd, "CYCLE %d %d %d", a, b, c)) {
    beginCycle(activeServo, a, b, (unsigned long)c);
    Serial.printf("OK CYCLE %u %d %d %d\n", activeServo, cycleClosedUs[activeServo], cycleEjectUs[activeServo], (int)cycleHoldMs[activeServo]);
    return;
  }

  Serial.println("ERR unknown");
}

void setup() {
  Serial.begin(BAUD);
  delay(300);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if (!pwm.begin()) {
    Serial.println("FATAL PCA9685 not_found");
    while (true) {
      delay(1000);
    }
  }
  pwm.setPWMFreq(SERVO_FREQUENCY_HZ);
  delay(10);
  // Disable every output before accepting commands: absolutely no boot movement.
  for (uint8_t channel = 0; channel < 16; channel++) {
    pwm.setPWM(channel, 0, 4096);
  }

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    servoAttached[i] = false;
    jogActive[i] = false;
    holdTorque[i] = false;
    cycleActive[i] = false;
    currentUs[i] = CENTER_US;
    targetUs[i] = CENTER_US;
    safeMinUs[i] = DEFAULT_SAFE_MIN_US;
    safeMaxUs[i] = DEFAULT_SAFE_MAX_US;
    slewUsPerSec[i] = 500;
    jogDir[i] = 0;
    cyclePhase[i] = 0;
    cycleClosedUs[i] = DEFAULT_CLOSED_US;
    cycleEjectUs[i] = DEFAULT_EJECT_US;
    cycleHoldUntil[i] = 0;
    cycleHoldMs[i] = 600;
    lastMotionUs[i] = micros();
    motionRemainder[i] = 0;
    detachAtMs[i] = 0;
    attachedAtMs[i] = 0;
    powerMaxMs[i] = DEFAULT_POWER_MAX_MS;
  }

  Serial.printf("READY gate12_pca9685_safe servos=%u address=0x%02X no_auto_move\n", NUM_SERVOS, PCA_ADDRESS);
  printInfo(activeServo);
}

void loop() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n') {
      handle(buf);
      buf = "";
    } else if (ch != '\r') {
      buf += ch;
      if (buf.length() > 120) {
        buf = "";
        Serial.println("ERR line_too_long");
      }
    }
  }

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    updateMotion(i);
    updateCycle(i);
    updatePowerWatchdog(i);
    updateAutoDetach(i);
  }
}
