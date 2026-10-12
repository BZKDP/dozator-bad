/*
 * ЭТАЛОННЫЙ ТЕСТ СЕРВЫ — по официальному примеру Sweep библиотеки ESP32Servo.
 * НИКАКИХ команд не нужно: серва сама плавно машет 20<->160 бесконечно,
 * сигнал идёт СРАЗУ на D13 и D26. Смотри на серву. В Serial — лог углов.
 */
#include <ESP32Servo.h>

Servo s1, s2;

void setup() {
  Serial.begin(115200);
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  s1.setPeriodHertz(50);
  s2.setPeriodHertz(50);
  s1.attach(13, 500, 2400);
  s2.attach(26, 500, 2400);
  Serial.println("SWEEP TEST: 20<->160 на D13+D26, бесконечно");
}

void loop() {
  for (int a = 20; a <= 160; a++) { s1.write(a); s2.write(a); delay(15); }
  Serial.println("-> 160");
  delay(400);
  for (int a = 160; a >= 20; a--) { s1.write(a); s2.write(a); delay(15); }
  Serial.println("-> 20");
  delay(400);
}
