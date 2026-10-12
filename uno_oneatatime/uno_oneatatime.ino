/*
 * Arduino UNO — два серво-шлагбаума ПО ОЧЕРЕДИ (питание на одном за раз).
 * В каждый момент запитан (attached) только ОДИН серво — второй detach (ток не тянет),
 * поэтому слабого БП хватает, просадок/браунаута нет.
 *
 * G1 -> D9, G2 -> D10.  0° = опущено, 90° = поднято.
 * Цикл по кругу: G2 поднять->опустить(0), затем G1 поднять->опустить(0).
 */

#include <Servo.h>

const int PIN_G1 = 9, PIN_G2 = 10;
int CLOSED = 0, OPEN = 90;

Servo s;   // один объект — переключаем между пинами

void moveGate(int pin, const char *name){
  Serial.print(name); Serial.println(F(" ^"));
  s.attach(pin);
  s.write(OPEN);   delay(2000);   // поднят 2 сек
  Serial.print(name); Serial.println(F(" v"));
  s.write(CLOSED); delay(900);    // вернуться в 0
  s.detach();                     // снять питание с этого серво
}

void setup(){
  Serial.begin(115200);
  // оба в 0 один раз при старте
  s.attach(PIN_G1); s.write(CLOSED); delay(400); s.detach();
  s.attach(PIN_G2); s.write(CLOSED); delay(400); s.detach();
  Serial.println(F("READY one-at-a-time (G2 -> G1)"));
}

void loop(){
  moveGate(PIN_G2, "G2");   // сначала придержка
  delay(600);
  moveGate(PIN_G1, "G1");   // потом главный
  delay(2500);              // пауза перед следующим кругом
}
