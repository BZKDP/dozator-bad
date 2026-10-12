/*
 * Тест ДВУХ серво-шлагбаумов на ESP32 — выдача 1 банки (escapement).
 * Библиотека: ESP32Servo.
 *
 * Подключение:
 *   Серво1 (G1 главный, к фасовщику): сигнал -> GPIO13, + -> 5V/VIN, - -> GND
 *   Серво2 (G2 придержка):            сигнал -> GPIO14, + -> 5V/VIN, - -> GND
 *   (для нагрузки питать серво от внешнего БП 5-6В, земля общая с ESP32)
 *
 * Калибровка (как у первого серво):
 *   0°  = ОПУЩЕНО (закрыто)
 *   90° = ПОДНЯТО (открыто)
 *
 * Команды в Serial (115200):
 *   1o / 1c   -> серво1 поднять/опустить
 *   2o / 2c   -> серво2 поднять/опустить
 *   1 <deg>   -> серво1 на угол (напр. "1 90")
 *   2 <deg>   -> серво2 на угол
 *   d         -> ВЫДАТЬ 1 БАНКУ (полный цикл: G2↑ G2↓ G1↑ G1↓)
 *   h         -> домой (оба в 0)
 */

#include <ESP32Servo.h>

const int PIN_G1 = 13;   // главный (к фасовщику)
const int PIN_G2 = 14;   // придержка

int CLOSED = 0;          // опущено
int OPEN   = 90;         // поднято

// тайминги цикла (мс) — подстрой под механику
const int T_LIFT  = 350;  // ход заслонки
const int T_ROLL  = 500;  // банка прокатывается
const int T_SHIFT = 450;  // столб опускается

Servo g1, g2;
String buf;

void dispense(){
  Serial.println("dispense: 1 банка");
  // 1. G2 поднять — нижняя банка из колонны въезжает на участок
  g2.write(OPEN);  delay(T_LIFT + T_ROLL);
  g2.write(CLOSED);delay(T_SHIFT);          // G2 вниз — столб опустился на шаг
  // 2. G1 поднять — банка уходит на дорожку к фасовщику
  g1.write(OPEN);  delay(T_LIFT + T_ROLL);
  g1.write(CLOSED);delay(T_LIFT);           // G1 вниз
  Serial.println("done");
}

void setServoCmd(Servo &s, String rest){
  rest.trim();
  if(rest=="o"){ s.write(OPEN);  Serial.println("open(90)"); }
  else if(rest=="c"){ s.write(CLOSED); Serial.println("closed(0)"); }
  else { int a=constrain(rest.toInt(),0,180); s.write(a); Serial.printf("angle=%d\n",a); }
}

void handle(String line){
  line.trim();
  if(line.length()==0) return;
  char k=line[0];
  if(k=='d'){ dispense(); return; }
  if(k=='h'){ g1.write(CLOSED); g2.write(CLOSED); Serial.println("home 0/0"); return; }
  if(k=='1'){ setServoCmd(g1, line.substring(1)); return; }
  if(k=='2'){ setServoCmd(g2, line.substring(1)); return; }
  Serial.println("? 1o 1c 2o 2c | 1 <deg> | 2 <deg> | d | h");
}

void setup(){
  Serial.begin(115200);
  delay(300);
  g1.setPeriodHertz(50); g1.attach(PIN_G1, 500, 2400);
  g2.setPeriodHertz(50); g2.attach(PIN_G2, 500, 2400);
  g1.write(CLOSED);                 // оба в 0 — прицепить рычаги
  g2.write(CLOSED);
  Serial.println("READY 2 servo (G1=13, G2=14). Оба в 0. Команды: 1o 2o d h ...");
}

void loop(){
  while(Serial.available()){
    char ch=(char)Serial.read();
    if(ch=='\n'){ handle(buf); buf=""; }
    else if(ch!='\r') buf+=ch;
  }
}
