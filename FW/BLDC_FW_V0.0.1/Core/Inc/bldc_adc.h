#ifndef INC_BLDC_ADC_H_
#define INC_BLDC_ADC_H_

#include "bldc_motor.h"


extern float g_fJoyX;
extern float g_fJoyY;



void ADC_CalibrateOffset(sMotor *psM);

void ADC_SampleCurrents(sMotor *psM);

void ADC_Task10ms(void);



#endif
