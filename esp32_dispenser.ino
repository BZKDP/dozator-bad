/*
 * Дозатор БАД — прошивка ESP32 + PCA9685
 * Escapement-логика: два серво-шлагбаума на канал, выдача по одной банке.
 *
 * Библиотека: Adafruit PWM Servo Driver Library (поставить в Arduino IDE).
 *
 * Протокол по USB-Serial (115200):
 *   "D <канал> <кол-во>\n"   выдать N банок из канала
 *   "ORDER 2 0 1\n"          выдать сразу по списку (берберин 2, мемон 0, B6 1)
 *   "CAL <канал> <gate0|1> <us>\n"  тест: подать угол в мкс (для калибровки)
 *   "PING\n"                 -> "PONG"
 * Ответы: "OK", "DONE", "ERR ..."
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ----- конфигурация -----
const uint8_t  NUM_CHANNELS = 3;          // позиций (масштабируется)
const uint8_t  PCA_ADDR     = 0x40;       // адрес платы (несколько -> массив)
const uint16_t SERVO_FREQ   = 50;         // Гц для серво

// канал n: G1(выход)=2n, G2(придержка)=2n+1
// углы в МИКРОСЕКУНДАХ импульса; подбери на месте калибровкой
struct GateCal { uint16_t closed_us; uint16_t open_us; };
GateCal G1cal[NUM_CHANNELS] = { {1500, 900}, {1500, 900}, {1500, 900} }; // выход
GateCal G2cal[NUM_CHANNELS] = { {1500, 2100}, {1500, 2100}, {1500, 2100} }; // придержка

// тайминги цикла (мс) — подстрой под угол наклона/скорость скатывания
const uint16_t T_GATE   = 250;  // время хода серво
const uint16_t T_ROLL   = 450;  // пока банка выкатывается из канала
const uint16_t T_SHIFT  = 400;  // пока колонна сдвигается на шаг
const uint16_t T_SETTLE = 150;  // пауза между банками

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA_ADDR);

// перевод микросекунд импульса в «тики» PCA9685 (12 бит @ 50 Гц)
uint16_t usToTicks(uint16_t us){
  // период 20000 мкс = 4096 тиков
  return (uint16_t)((uint32_t)us * 4096 / 20000);
}
void setServoUs(uint8_t ch, uint16_t us){
  pwm.setPWM(ch, 0, usToTicks(us));
}

uint8_t g1ch(uint8_t c){ return 2*c;   }
uint8_t g2ch(uint8_t c){ return 2*c+1; }

void gateG1(uint8_t c, bool open){ setServoUs(g1ch(c), open? G1cal[c].open_us : G1cal[c].closed_us); }
void gateG2(uint8_t c, bool open){ setServoUs(g2ch(c), open? G2cal[c].open_us : G2cal[c].closed_us); }

// ----- выдать одну банку из канала -----
void dispenseOne(uint8_t c){
  gateG1(c, true);  delay(T_GATE);   // 1. открыть выход
  delay(T_ROLL);                     //    банка выкатывается
  gateG1(c, false); delay(T_GATE);   // 2. закрыть выход
  gateG2(c, true);  delay(T_GATE);   // 3. открыть придержку
  delay(T_SHIFT);                    //    колонна сдвигается
  gateG2(c, false); delay(T_GATE);   // 4. закрыть придержку — изолирована 1 банка
  delay(T_SETTLE);
}

void dispenseN(uint8_t c, uint16_t n){
  for(uint16_t i=0;i<n;i++) dispenseOne(c);
}

void homeAll(){
  for(uint8_t c=0;c<NUM_CHANNELS;c++){ gateG1(c,false); gateG2(c,false); }
}

// ----- разбор команд -----
String buf;
void handle(String line){
  line.trim();
  if(line.length()==0) return;
  if(line=="PING"){ Serial.println("PONG"); return; }

  if(line.startsWith("D ")){
    int s1=line.indexOf(' ');
    int s2=line.indexOf(' ', s1+1);
    int c=line.substring(s1+1, s2).toInt();
    int n=line.substring(s2+1).toInt();
    if(c<0||c>=NUM_CHANNELS){ Serial.println("ERR channel"); return; }
    Serial.println("OK");
    dispenseN(c, n);
    Serial.println("DONE");
    return;
  }

  if(line.startsWith("ORDER")){
    // ORDER n0 n1 n2 ...
    int idx=line.indexOf(' '); int ch=0;
    Serial.println("OK");
    while(idx>0 && ch<NUM_CHANNELS){
      int nxt=line.indexOf(' ', idx+1);
      String tok = (nxt>0)? line.substring(idx+1,nxt) : line.substring(idx+1);
      int n=tok.toInt();
      if(n>0) dispenseN(ch, n);
      ch++; idx=nxt;
    }
    Serial.println("DONE");
    return;
  }

  if(line.startsWith("CAL ")){
    // CAL channel gate us   (gate: 0=G1, 1=G2)
    int a=line.indexOf(' ');
    int b=line.indexOf(' ',a+1);
    int d=line.indexOf(' ',b+1);
    int c=line.substring(a+1,b).toInt();
    int g=line.substring(b+1,d).toInt();
    int us=line.substring(d+1).toInt();
    if(c<0||c>=NUM_CHANNELS){ Serial.println("ERR channel"); return; }
    setServoUs(g? g2ch(c):g1ch(c), us);
    Serial.println("OK");
    return;
  }

  Serial.println("ERR unknown");
}

void setup(){
  Serial.begin(115200);
  Wire.begin(21, 22);           // SDA=21, SCL=22
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(200);
  homeAll();                    // все шлагбаумы в «закрыто»
  Serial.println("READY");
}

void loop(){
  while(Serial.available()){
    char ch=(char)Serial.read();
    if(ch=='\n'){ handle(buf); buf=""; }
    else if(ch!='\r') buf+=ch;
  }
}
