/*
 * BX04 v2 — НЕБЛОКИРУЮЩИЙ тест головки дозатора. ESP32 + SG90/MG90S, сигнал GPIO13.
 * Движение плавное конечным автоматом: порт читается всегда, новая команда
 * мгновенно перебивает текущее движение. Если в логе сам собой появляется
 * "BX04 READY" — плата ресетится от просадки питания сервы: нужен внешний 5В.
 *
 * Команды (Serial 115200):
 *   h / e     -> плавно в HOME / EJECT
 *   t         -> цикл выдачи (HOME->EJECT->пауза->HOME), n -> 5 циклов
 *   0..180    -> плавно на угол
 *   !<a>      -> МГНОВЕННО на угол (без плавности, напр. !90)
 *   H<a> E<a> -> задать HOME/EJECT, w -> сохранить
 *   v<d>      -> скорость град/с (10..300), напр. v120
 *   x         -> стоп на месте
 *   ?         -> статус
 */
#include <ESP32Servo.h>
#include <Preferences.h>

const int SERVO_PIN = 13;
const int SERVO_PIN2 = 26;   // дубль: серва может быть на D13 или D26
Preferences prefs;
Servo srv, srv2;
String buf;

int HOME_A = 14, EJECT_A = 140;
int cur = 14, target = 14;
int speedDps = 70;                 // град/с (холостой ход)
int powerDps = 25;                 // силовая зона (перехват, первые 45% хода)
bool dith = true;                  // раскачка в силовой зоне
int fw = 0;                        // счётчик шагов для раскачки
unsigned long lastStep = 0, phaseT = 0;
int cycle = 0, cyclesLeft = 0;     // 0 нет, 1 ->EJECT, 2 пауза, 3 ->HOME

void setTarget(int a) { target = constrain(a, 0, 180); }

void show() {
  Serial.printf("HOME=%d EJECT=%d ход=%d тек=%d цель=%d скор=%d\n",
                HOME_A, EJECT_A, EJECT_A - HOME_A, cur, target, speedDps);
}

void setup() {
  Serial.begin(115200);
  prefs.begin("bx04", false);
  HOME_A = prefs.getInt("home", HOME_A);
  EJECT_A = prefs.getInt("eject", EJECT_A);
  srv.setPeriodHertz(50);
  srv.attach(SERVO_PIN, 500, 2400);
  srv2.setPeriodHertz(50);
  srv2.attach(SERVO_PIN2, 500, 2400);
  cur = target = HOME_A;
  srv.write(cur); srv2.write(cur);
  Serial.println("BX04 READY v2 (D13+D26)");
  show();
}

void handle(String line) {
  line.trim();
  if (!line.length()) return;
  char k = line[0];
  if (k == 'h') { cycle = 0; setTarget(HOME_A); Serial.println("-> HOME"); return; }
  if (k == 'e') { cycle = 0; setTarget(EJECT_A); Serial.println("-> EJECT"); return; }
  if (k == 't') { cycle = 1; cyclesLeft = 1; setTarget(EJECT_A); Serial.println("ЦИКЛ"); return; }
  if (k == 'n') { cycle = 1; cyclesLeft = 5; setTarget(EJECT_A); Serial.println("5 ЦИКЛОВ"); return; }
  if (k == 'x') { cycle = 0; target = cur; Serial.printf("СТОП на %d\n", cur); return; }
  if (k == 'H') { HOME_A = constrain(line.substring(1).toInt(), 0, 180); show(); return; }
  if (k == 'E') { EJECT_A = constrain(line.substring(1).toInt(), 0, 180); show(); return; }
  if (k == 'w') { prefs.putInt("home", HOME_A); prefs.putInt("eject", EJECT_A); Serial.println("СОХРАНЕНО"); return; }
  if (k == 'v') { speedDps = constrain(line.substring(1).toInt(), 10, 300); show(); return; }
  if (k == 'p') { powerDps = constrain(line.substring(1).toInt(), 8, 100); Serial.printf("power=%d\n", powerDps); return; }
  if (k == 'd') { dith = line.substring(1).toInt() != 0; Serial.printf("раскачка=%d\n", dith); return; }
  if (k == '!') { cycle = 0; int a = constrain(line.substring(1).toInt(), 0, 180); cur = a; target = a; srv.write(a); srv2.write(a); Serial.printf("JUMP %d\n", a); return; }
  if (k == '?') { show(); return; }
  if (isDigit(k)) { cycle = 0; setTarget(line.toInt()); Serial.printf("-> %d\n", target); }
}

void loop() {
  // порт читается ВСЕГДА
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { handle(buf); buf = ""; }
    else buf += c;
  }
  // движение. Силовая зона (первые 45% хода в сторону EJECT):
  // КРУПНЫЕ шаги 7° — у аналоговой сервы момент ~ ошибке позиции, 1°-шаги душили её;
  // каждый 4-й шаг: замах назад 5° и удар вперёд 12° (импульс, как гайковёрт).
  unsigned long now = millis();
  int span = abs(EJECT_A - HOME_A);
  bool toEject = (EJECT_A > HOME_A) ? (target > cur) : (target < cur);
  bool power = toEject && (abs(cur - HOME_A) < span * 45 / 100);
  if (cur != target && now - lastStep >= (unsigned long)(power ? 150 : 1000 / speedDps)) {
    lastStep = now;
    int dir = (target > cur) ? 1 : -1;
    if (power) {
      fw++;
      int stp = 7;
      if (dith && (fw % 4 == 0)) {                       // замах-удар
        srv.write(constrain(cur - dir * 5, 0, 180));
        srv2.write(constrain(cur - dir * 5, 0, 180));
        delay(90);
        stp = 12;
      }
      cur = constrain(cur + dir * min(stp, abs(target - cur)), 0, 180);
    } else {
      cur = constrain(cur + dir, 0, 180);
    }
    srv.write(cur); srv2.write(cur);
    if (cur == target) { Serial.printf("angle = %d\n", cur); phaseT = now; fw = 0; }
  }
  // автомат цикла выдачи
  if (cycle && cur == target) {
    if (cycle == 1) { cycle = 2; phaseT = now; }
    else if (cycle == 2 && now - phaseT >= 400) { cycle = 3; setTarget(HOME_A); }
    else if (cycle == 3 && cur == HOME_A) {
      cyclesLeft--;
      if (cyclesLeft > 0) { cycle = 1; setTarget(EJECT_A); Serial.printf("осталось %d\n", cyclesLeft); }
      else { cycle = 0; Serial.println("ЦИКЛ ГОТОВ"); }
    }
  }
}
