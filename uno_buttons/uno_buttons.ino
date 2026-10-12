/*
 * Arduino UNO — две КНОПКИ управляют двумя серво-шлагбаумами.
 *   Кнопка1 (D2) -> G1 (серво D9):  нажал = поднять, ещё раз = опустить (toggle)
 *   Кнопка2 (D3) -> G2 (серво D10): так же
 * Кнопки: одна нога в пин, вторая в GND (внутр. подтяжка INPUT_PULLUP, нажато = LOW).
 *
 * Калибровка: 0° = опущено (закрыто), 90° = поднято (открыто).
 * Через Serial тоже можно: 1o/1c, 2o/2c, 1 <deg>, 2 <deg>, h.
 */

#include <Servo.h>

const int PIN_G1 = 9, PIN_G2 = 10;
const int BTN1 = 2, BTN2 = 3;

int CLOSED = 0;     // опущено
int OPEN   = 90;    // поднято

Servo g1, g2;
bool s1up = false, s2up = false;

// дебаунс на кнопку
struct Btn { int pin; int prev; int stable; unsigned long t; };
Btn b1 = {BTN1, HIGH, HIGH, 0};
Btn b2 = {BTN2, HIGH, HIGH, 0};
const unsigned long DEB = 40;

bool pressed(Btn &b){           // true один раз в момент нажатия
  int rd = digitalRead(b.pin);
  if (rd != b.prev) { b.t = millis(); b.prev = rd; }
  if ((millis() - b.t) > DEB && rd != b.stable) {
    b.stable = rd;
    if (b.stable == LOW) return true;   // фронт нажатия
  }
  return false;
}

String buf;
void setCmd(Servo &s, bool &up, String r){
  r.trim();
  if(r=="o"){ s.write(OPEN); up=true; }
  else if(r=="c"){ s.write(CLOSED); up=false; }
  else { int a=constrain(r.toInt(),0,180); s.write(a); up=(a>=45); }
}
void handle(String line){
  line.trim(); if(!line.length()) return;
  char k=line[0];
  if(k=='h'){ g1.write(CLOSED); g2.write(CLOSED); s1up=s2up=false; return; }
  if(k=='1'){ setCmd(g1,s1up,line.substring(1)); return; }
  if(k=='2'){ setCmd(g2,s2up,line.substring(1)); return; }
}

void setup(){
  Serial.begin(115200);
  pinMode(BTN1, INPUT_PULLUP);
  pinMode(BTN2, INPUT_PULLUP);
  g1.attach(PIN_G1); g2.attach(PIN_G2);
  g1.write(CLOSED); g2.write(CLOSED);
  Serial.println(F("READY: BTN1(D2)->G1, BTN2(D3)->G2 (нажатие = вверх/вниз)"));
}

void loop(){
  if (pressed(b1)) { s1up = !s1up; g1.write(s1up ? OPEN : CLOSED); Serial.println(s1up?F("G1 up"):F("G1 down")); }
  if (pressed(b2)) { s2up = !s2up; g2.write(s2up ? OPEN : CLOSED); Serial.println(s2up?F("G2 up"):F("G2 down")); }

  while (Serial.available()) {
    char ch=(char)Serial.read();
    if(ch=='\n'){ handle(buf); buf=""; }
    else if(ch!='\r') buf+=ch;
  }
}
