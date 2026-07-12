#include "buzzer.h"

 uint32_t buzz_off = 0;

void beep_init(void) { pinMode(BUZZER_GPIO, OUTPUT); }
void beep_off(void) { digitalWrite(BUZZER_GPIO, LOW); }
void beep_on(uint32_t duration) {
  analogWrite(BUZZER_GPIO, 127);
  buzz_off = millis() + duration;
}