#ifndef __BUZZER_H
#define __BUZZER_H

#include "main.h"
#include "delay.h"
#define Buzzer    GPIO_PIN_4 
void Buzzer_Init();
void Buzzer_On();
void Buzzer_Off();
#endif