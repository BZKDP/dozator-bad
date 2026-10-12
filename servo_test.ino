/*
 * Тест ОДНОГО серво на ESP32 — подбор положений шлагбаума.
 * Библиотека: "ESP32Servo" (Arduino IDE → Управление библиотеками → найти ESP32Servo → Install).
 *
 * Подключение:
 *   сигнал серво (оранж/жёлтый) -> GPIO13
 *   + (красный)                 -> 5V/VIN  (один SG90; для MG996R — внешний БП 5-6В!)
 *   - (коричн/чёрный)           -> GND     (земля БП = земля ESP32)
 *
 * Команды в Serial Monitor (115200, "Both NL & CR" или "No line ending" — обе ок):
 *   число 0..180   -> повернуть серво на этот угол (градусы)
 *   o              -> открыть (поднять шлагбаум) = угол OPEN
 *   c              -> закрыть (опустить)         = угол CLOSED
 *   t              -> цикл: закрыть -> открыть -> закрыть (имитация выдачи)
 *   s              -> «свипнуть» 0..180..0 (проверить весь ход)
 *   u<us>          -> подать импульс в микросекундах, напр. u1500 (точная калибровка)
 */

#include <ESP32Servo.h>

const int SERVO_PIN = 13;

// ПОДБЕРИ под свою механику шлагбаума (углы в градусах):
int CLOSED = 0;     // опущен — перекрывает лоток
int OPEN   = 90;    // поднят — пропускает банку

Servo gate;
String buf;

void goAngle(int a){
  a = constrain(a, 0, 180);
  gate.write(a);
  Serial.printf("angle = %d\n", a);
}

void setup(){
  Serial.begin(115200);
  delay(300);
  gate.setPeriodHertz(50);              // 50 Гц для аналоговых серво
  gate.attach(SERVO_PIN, 500, 2400);    // мин/макс импульс (мкс) — норм для SG90/MG90S/MG996R
  gate.write(CLOSED);                   // старт: закрыто
  Serial.println("READY. Команды: 0..180 | o | c | t | s | u<us>");
}

void handle(String line){
  line.trim();
  if(line.length()==0) return;

  char k = line[0];
  if(k=='o'){ goAngle(OPEN);  return; }
  if(k=='c'){ goAngle(CLOSED);return; }
  if(k=='s'){                                  // свип всего хода
    for(int a=0;a<=180;a+=5){ gate.write(a); delay(20); }
    for(int a=180;a>=0;a-=5){ gate.write(a); delay(20); }
    Serial.println("sweep done"); return;
  }
  if(k=='t'){                                  // имитация выдачи: закрыть->открыть->закрыть
    goAngle(CLOSED); delay(400);
    goAngle(OPEN);   delay(600);
    goAngle(CLOSED);
    Serial.println("cycle done"); return;
  }
  if(k=='u'){                                  // прямой импульс в мкс
    int us = line.substring(1).toInt();
    if(us>=400 && us<=2600){ gate.writeMicroseconds(us); Serial.printf("us = %d\n", us); }
    else Serial.println("us 400..2600");
    return;
  }
  // иначе пробуем как угол
  int a = line.toInt();
  goAngle(a);
}

void loop(){
  while(Serial.available()){
    char ch=(char)Serial.read();
    if(ch=='\n'){ handle(buf); buf=""; }
    else if(ch!='\r') buf+=ch;
  }
}
