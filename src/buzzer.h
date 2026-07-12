#pragma once

#ifndef BUZZER_H
#define BUZZER_H

#define BUZZER_GPIO 19

#include <Arduino.h>

extern uint32_t buzz_off;

void beep_init(void);
void beep_off(void);
void beep_on(uint32_t duration);

#endif // BUZZER_H