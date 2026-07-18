#ifndef INC_BLDC_LED_H_
#define INC_BLDC_LED_H_

#include "bldc_motor.h"



void LED_Init(void);

void LED_Task10ms(void);

void LED_SysTick(void);

void LED_SignalCAN(void);



#endif
