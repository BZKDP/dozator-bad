/*
 * Arduino UNO — РУЧНОЕ управление двумя серво по USB (из окна на ПК).
 * Питание "по очереди": активен (attached) только тот серво, которым сейчас рулишь,
 * второй detach (ток не тянет) — слабого БП хватает.
 *
 * G1 -> D9, G2 -> D10.  0° = опущено, 90° = поднято.
 * Команды (из приложения / Serial): 1o 1c 2o 2c h
 */

#include <Servo.h>

const int PIN_G1 = 9, PIN_G2 = 10;
int CLOSED = 0, OPEN = 90;
Servo g1, g2;

void useG1(){ if(g2.attached()) g2.detach(); if(!g1.attached()) g1.attach(PIN_G1); }
void useG2(){ if(g1.attached()) g1.detach(); if(!g2.attached()) g2.attach(PIN_G2); }

String buf;
void handle(String s){
  s.trim();
  if(s=="1o"){ useG1(); g1.write(OPEN);   Serial.println(F("G1 up")); }
  else if(s=="1c"){ useG1(); g1.write(CLOSED); Serial.println(F("G1 down")); }
  else if(s=="2o"){ useG2(); g2.write(OPEN);   Serial.println(F("G2 up")); }
  else if(s=="2c"){ useG2(); g2.write(CLOSED); Serial.println(F("G2 down")); }
  else if(s=="h"){ useG1(); g1.write(CLOSED); delay(700); useG2(); g2.write(CLOSED); Serial.println(F("home")); }
  else if(s.length()) Serial.println(F("? 1o 1c 2o 2c h"));
}

void setup(){
  Serial.begin(115200);
  g1.attach(PIN_G1); g1.write(CLOSED); delay(500); g1.detach();  // в 0 и обесточить
  Serial.println(F("READY manual (1o 1c 2o 2c h)"));
}

void loop(){
  while(Serial.available()){
    char ch=(char)Serial.read();
    if(ch=='\n'){ handle(buf); buf=""; }
    else if(ch!='\r') buf+=ch;
  }
}
