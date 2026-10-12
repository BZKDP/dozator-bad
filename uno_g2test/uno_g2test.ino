/* Диагностика: крутим ТОЛЬКО серво2 (D10), 0<->90, медленно. Серво1 не трогаем. */
#include <Servo.h>
Servo g2;
void setup(){
  Serial.begin(115200);
  g2.attach(10);
  Serial.println(F("G2-only test (D10): 0<->90"));
}
void loop(){
  Serial.println(F("G2 -> 0"));
  g2.write(0);  delay(1200);
  Serial.println(F("G2 -> 90"));
  g2.write(90); delay(1200);
}
