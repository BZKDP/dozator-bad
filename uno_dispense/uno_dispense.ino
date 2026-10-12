/*
 * Arduino UNO — тест ДВУХ серво-шлагбаумов (выдача 1 банки).
 * Библиотека Servo встроена в ядро AVR, ставить ничего не нужно.
 *
 * Подключение:
 *   Серво1 (G1 главный): сигнал -> D9,  + -> 5V, - -> GND
 *   Серво2 (G2 придержка): сигнал -> D10, + -> 5V, - -> GND
 *   Два "+" сводятся в один 5V (макетка/Y-сплиттер). Маленькие SG90 ок от 5V Uno,
 *   для MG996R/нагрузки — внешний БП 5-6В, минус БП к GND Uno.
 *
 * Калибровка (как на ESP32): 0° = ОПУЩЕНО (закрыто), 90° = ПОДНЯТО (открыто).
 *
 * Команды в Serial (115200):
 *   1o / 1c   серво1 поднять/опустить
 *   2o / 2c   серво2 поднять/опустить
 *   1 <deg> / 2 <deg>   серво на угол (напр. "1 90")
 *   d   ВЫДАТЬ 1 БАНКУ (G2^ G2v G1^ G1v)
 *   h   домой (оба в 0)
 */

#include <Servo.h>

const int PIN_G1 = 9;
const int PIN_G2 = 10;

int CLOSED = 0;    // опущено
int OPEN   = 90;   // поднято

const int T_LIFT  = 350;
const int T_ROLL  = 500;
const int T_SHIFT = 450;

Servo g1, g2;
String buf;

void demoCycle(){
  Serial.println(F("cycle start"));
  g2.write(OPEN);   delay(2000);   // G2 поднят 2 сек — банка входит в буфер
  g2.write(CLOSED); delay(600);    // G2 опущен — держит столб
  g1.write(OPEN);   delay(2000);   // G1 поднят 2 сек — банка из буфера к фасовщику
  g1.write(CLOSED); delay(600);    // G1 опущен
  Serial.println(F("cycle done"));
}

void setCmd(Servo &s, String r){
  r.trim();
  if(r=="o"){ s.write(OPEN); Serial.println(F("open(90)")); }
  else if(r=="c"){ s.write(CLOSED); Serial.println(F("closed(0)")); }
  else { int a=constrain(r.toInt(),0,180); s.write(a); Serial.print(F("angle=")); Serial.println(a); }
}

void handle(String line){
  line.trim();
  if(!line.length()) return;
  char k=line[0];
  if(k=='d'){ demoCycle(); return; }
  if(k=='h'){ g1.write(CLOSED); g2.write(CLOSED); Serial.println(F("home 0/0")); return; }
  if(k=='1'){ setCmd(g1, line.substring(1)); return; }
  if(k=='2'){ setCmd(g2, line.substring(1)); return; }
  Serial.println(F("? 1o 1c 2o 2c | 1 <deg> | 2 <deg> | d | h"));
}

void setup(){
  Serial.begin(115200);
  g1.attach(PIN_G1); g2.attach(PIN_G2);
  g1.write(CLOSED); g2.write(CLOSED);   // оба в 0
  delay(1000);
  Serial.println(F("READY UNO. Цикл повторяется сам каждые ~3с. Serial: 1o/1c 2o/2c d h"));
}

void loop(){
  demoCycle();                          // авто-повтор цикла (RESET не нужен)
  unsigned long t = millis();           // пауза 3с между циклами, слушаем serial
  while (millis() - t < 3000) {
    while (Serial.available()) {
      char ch=(char)Serial.read();
      if(ch=='\n'){ handle(buf); buf=""; }
      else if(ch!='\r') buf+=ch;
    }
  }
}
