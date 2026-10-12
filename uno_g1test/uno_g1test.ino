/* Диагностика: крутим ТОЛЬКО серво1 (D9), 0<->90, медленно. Серво2 не трогаем. */
#include <Servo.h>
Servo g1;
void setup(){
  Serial.begin(115200);
  g1.attach(9);
  Serial.println(F("G1-only test (D9): 0<->90"));
}
void loop(){
  Serial.println(F("G1 -> 0"));
  g1.write(0);  delay(1200);
  Serial.println(F("G1 -> 90"));
  g1.write(90); delay(1200);
}
