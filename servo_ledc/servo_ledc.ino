/*
 * ГОЛЫЙ ТЕСТ БЕЗ БИБЛИОТЕК: нативный LEDC ядра ESP32 (ядро 3.x API).
 * Серва должна вставать в 3 ЧЁТКИЕ позиции с паузой 1.2 с:
 * ЦЕНТР (1500 мкс) -> ЛЕВО (1000) -> ЦЕНТР -> ПРАВО (2000) -> ...
 * Сигнал на D13 и D26. Если серва не встаёт в 3 разных положения — она неисправна.
 */
const int P1 = 13, P2 = 26;

void pulse(int us) {
  uint32_t duty = (uint32_t)us * 16383 / 20000;   // 50 Гц, 14 бит
  ledcWrite(P1, duty);
  ledcWrite(P2, duty);
  Serial.printf("импульс %d мкс (duty %u)\n", us, duty);
}

void setup() {
  Serial.begin(115200);
  ledcAttach(P1, 50, 14);
  ledcAttach(P2, 50, 14);
  Serial.println("LEDC NATIVE TEST: центр/лево/центр/право, пауза 1.2 c");
}

void loop() {
  pulse(1500); delay(1200);
  pulse(1000); delay(1200);
  pulse(1500); delay(1200);
  pulse(2000); delay(1200);
}
