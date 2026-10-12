/*
 * Gate12 safe servo control for Arduino Mega + MG90S/SG90.
 *
 * This is the Arduino Mega version of the ESP32 safe controller.
 * It uses the standard Servo library and the same USB-Serial protocol as the
 * PC slider UI.
 *
 * Wiring:
 *   MG90S signal orange/yellow -> Mega D9
 *   MG90S red                  -> 5V servo supply
 *   MG90S brown/black          -> GND servo supply and Mega GND
 *   Mega USB                   -> computer
 *
 * Serial protocol, 115200:
 *   PING
 *   INFO
 *   SET <us>              e.g. SET 1500
 *   STEP <delta_us>       e.g. STEP -10
 *   LIM <min_us> <max_us> e.g. LIM 1400 1600
 *   RATE <us_per_sec>     e.g. RATE 300
 *   JOG <-1|0|1>          hold-to-move mode, 0 stops
 *   HOLD <0|1>            0=auto-detach after stop, 1=keep servo powered
 *   CYCLE <home_us> <drop_us> <hold_ms>
 *   STOP
 *   DETACH
 */

#include <Servo.h>

const int SERVO_PIN = 9;
const long BAUD = 115200;

const int HARD_MIN_US = 700;
const int HARD_MAX_US = 2300;

int safeMinUs = 1450;
int safeMaxUs = 1550;

int currentUs = 1500;
int targetUs = 1500;
int slewUsPerSec = 300;

Servo gate;
bool servoAttached = false;
String buf;

unsigned long lastMotionUs = 0;
uint32_t motionRemainder = 0;
bool jogActive = false;
int8_t jogDir = 0;
bool holdTorque = false;
const uint16_t AUTO_DETACH_MS = 250;
unsigned long detachAtMs = 0;

bool cycleActive = false;
uint8_t cyclePhase = 0;
int cycleHomeUs = 1500;
int cycleDropUs = 1550;
unsigned long cycleHoldUntil = 0;
unsigned long cycleHoldMs = 450;

int clampHard(int us) {
  return constrain(us, HARD_MIN_US, HARD_MAX_US);
}

int clampSafe(int us) {
  return constrain(us, safeMinUs, safeMaxUs);
}

bool movingNow() {
  return jogActive || currentUs != targetUs;
}

void cancelDetach() {
  detachAtMs = 0;
}

void scheduleDetach() {
  if (!holdTorque && servoAttached && !movingNow()) {
    detachAtMs = millis() + AUTO_DETACH_MS;
  }
}

void attachAt(int us) {
  if (!servoAttached) {
    gate.attach(SERVO_PIN, HARD_MIN_US, HARD_MAX_US);
    servoAttached = true;
  }
  currentUs = clampSafe(us);
  targetUs = currentUs;
  gate.writeMicroseconds(currentUs);
  lastMotionUs = micros();
  motionRemainder = 0;
  cancelDetach();
}

void beginMoveTo(int us) {
  int next = clampSafe(us);
  if (!servoAttached) {
    attachAt(currentUs);
  }
  jogActive = false;
  jogDir = 0;
  targetUs = next;
  lastMotionUs = micros();
  motionRemainder = 0;
  cancelDetach();
}

void stopMotion() {
  updateMotion();
  cycleActive = false;
  cyclePhase = 0;
  jogActive = false;
  jogDir = 0;
  targetUs = currentUs;
  if (servoAttached) {
    gate.writeMicroseconds(currentUs);
  }
  scheduleDetach();
}

void beginJog(int dir) {
  dir = (dir > 0) ? 1 : (dir < 0 ? -1 : 0);
  if (dir == 0) {
    stopMotion();
    return;
  }
  if (!servoAttached) {
    attachAt(currentUs);
  }
  cycleActive = false;
  cyclePhase = 0;
  jogActive = true;
  jogDir = (int8_t)dir;
  targetUs = currentUs;
  lastMotionUs = micros();
  motionRemainder = 0;
  cancelDetach();
}

void updateAutoDetach() {
  if (detachAtMs == 0 || holdTorque || movingNow()) {
    return;
  }
  if ((long)(millis() - detachAtMs) >= 0) {
    if (servoAttached) {
      gate.detach();
      servoAttached = false;
    }
    detachAtMs = 0;
  }
}

void updateMotion() {
  if (!servoAttached || !movingNow()) {
    lastMotionUs = micros();
    motionRemainder = 0;
    return;
  }

  unsigned long now = micros();
  unsigned long dt = now - lastMotionUs;
  if (dt < 1000) {
    return;
  }
  lastMotionUs = now;

  uint64_t movement = (uint64_t)slewUsPerSec * (uint64_t)dt + motionRemainder;
  int maxStep = (int)(movement / 1000000ULL);
  motionRemainder = (uint32_t)(movement % 1000000ULL);
  if (maxStep < 1) {
    return;
  }

  if (jogActive) {
    int next = currentUs + (jogDir > 0 ? maxStep : -maxStep);
    currentUs = clampSafe(next);
    targetUs = currentUs;
    if ((jogDir < 0 && currentUs <= safeMinUs) || (jogDir > 0 && currentUs >= safeMaxUs)) {
      jogActive = false;
      jogDir = 0;
      scheduleDetach();
    }
  } else {
    int delta = targetUs - currentUs;
    bool reached = false;
    if (abs(delta) <= maxStep) {
      currentUs = targetUs;
      reached = true;
    } else {
      currentUs += (delta > 0) ? maxStep : -maxStep;
    }
    if (reached) {
      scheduleDetach();
    }
  }
  gate.writeMicroseconds(currentUs);
}

void updateCycle() {
  if (!cycleActive) {
    return;
  }

  if (cyclePhase == 1 && currentUs == targetUs) {
    beginMoveTo(cycleDropUs);
    cyclePhase = 2;
    return;
  }

  if (cyclePhase == 2 && currentUs == targetUs) {
    cycleHoldUntil = millis() + cycleHoldMs;
    cyclePhase = 3;
    return;
  }

  if (cyclePhase == 3 && (long)(millis() - cycleHoldUntil) >= 0) {
    beginMoveTo(cycleHomeUs);
    cyclePhase = 4;
    return;
  }

  if (cyclePhase == 4 && currentUs == targetUs) {
    cycleActive = false;
    cyclePhase = 0;
    Serial.println(F("DONE"));
  }
}

void printInfo() {
  Serial.print(F("INFO board=mega pin="));
  Serial.print(SERVO_PIN);
  Serial.print(F(" attached="));
  Serial.print(servoAttached ? 1 : 0);
  Serial.print(F(" pos="));
  Serial.print(currentUs);
  Serial.print(F(" target="));
  Serial.print(targetUs);
  Serial.print(F(" min="));
  Serial.print(safeMinUs);
  Serial.print(F(" max="));
  Serial.print(safeMaxUs);
  Serial.print(F(" rate="));
  Serial.print(slewUsPerSec);
  Serial.print(F(" jog="));
  Serial.print(jogDir);
  Serial.print(F(" hold="));
  Serial.print(holdTorque ? 1 : 0);
  Serial.print(F(" cycle="));
  Serial.println(cycleActive ? 1 : 0);
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

void handleLine(String line) {
  line.trim();
  if (line.length() == 0) {
    return;
  }

  String cmd = line;
  cmd.toUpperCase();

  if (cmd == "PING") {
    Serial.println(F("PONG"));
    return;
  }

  if (cmd == "INFO") {
    printInfo();
    return;
  }

  if (cmd == "STOP") {
    stopMotion();
    Serial.print(F("OK STOP "));
    Serial.println(currentUs);
    return;
  }

  if (cmd == "DETACH") {
    stopMotion();
    if (servoAttached) {
      gate.detach();
      servoAttached = false;
    }
    Serial.println(F("OK DETACH"));
    return;
  }

  int a = 0;
  int b = 0;
  int c = 0;

  if (parseOneInt(cmd, "SET %d", a)) {
    cycleActive = false;
    beginMoveTo(a);
    Serial.print(F("OK SET "));
    Serial.println(targetUs);
    return;
  }

  if (parseOneInt(cmd, "JOG %d", a)) {
    beginJog(a);
    Serial.print(F("OK JOG "));
    Serial.print(jogDir);
    Serial.print(' ');
    Serial.println(currentUs);
    return;
  }

  if (parseOneInt(cmd, "STEP %d", a)) {
    cycleActive = false;
    beginMoveTo(targetUs + a);
    Serial.print(F("OK SET "));
    Serial.println(targetUs);
    return;
  }

  if (parseTwoInts(cmd, "LIM %d %d", a, b)) {
    a = clampHard(a);
    b = clampHard(b);
    if (a >= b || b - a < 20) {
      Serial.println(F("ERR bad_limits"));
      return;
    }
    safeMinUs = a;
    safeMaxUs = b;
    targetUs = clampSafe(targetUs);
    currentUs = clampSafe(currentUs);
    if (servoAttached) {
      gate.writeMicroseconds(currentUs);
    }
    Serial.print(F("OK LIM "));
    Serial.print(safeMinUs);
    Serial.print(' ');
    Serial.println(safeMaxUs);
    return;
  }

  if (parseOneInt(cmd, "RATE %d", a)) {
    slewUsPerSec = constrain(a, 5, 2000);
    Serial.print(F("OK RATE "));
    Serial.println(slewUsPerSec);
    return;
  }

  if (parseOneInt(cmd, "HOLD %d", a)) {
    holdTorque = a != 0;
    if (holdTorque) {
      cancelDetach();
    } else {
      scheduleDetach();
    }
    Serial.print(F("OK HOLD "));
    Serial.println(holdTorque ? 1 : 0);
    return;
  }

  if (parseThreeInts(cmd, "CYCLE %d %d %d", a, b, c)) {
    cycleHomeUs = clampSafe(a);
    cycleDropUs = clampSafe(b);
    cycleHoldMs = constrain(c, 50, 5000);

    if (!servoAttached) {
      attachAt(cycleHomeUs);
    }

    cycleActive = true;
    cyclePhase = 1;
    beginMoveTo(cycleHomeUs);
    Serial.print(F("OK CYCLE "));
    Serial.print(cycleHomeUs);
    Serial.print(' ');
    Serial.print(cycleDropUs);
    Serial.print(' ');
    Serial.println(cycleHoldMs);
    return;
  }

  Serial.println(F("ERR unknown"));
}

void setup() {
  Serial.begin(BAUD);
  delay(300);
  Serial.println(F("READY gate12_mega_safe no_auto_move"));
  printInfo();
}

void loop() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n') {
      handleLine(buf);
      buf = "";
    } else if (ch != '\r') {
      buf += ch;
      if (buf.length() > 96) {
        buf = "";
        Serial.println(F("ERR line_too_long"));
      }
    }
  }

  updateMotion();
  updateCycle();
  updateAutoDetach();
}
