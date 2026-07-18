#ifndef INC_BLDC_MOTOR_H_
#define INC_BLDC_MOTOR_H_

#include "stm32g4xx_hal.h"
#include "bldc_config.h"


//  Motor Struct
typedef struct
{
    // HAL Handles
    TIM_HandleTypeDef  *psTimPwm;       // TIM1 (M1) / TIM8 (M0)
    TIM_HandleTypeDef  *psTimHall;      // TIM3 (M0) / TIM4 (M1)
    ADC_HandleTypeDef  *psAdcUW;        // ADC1 (M0) / ADC3 (M1) : IU, IW
    ADC_HandleTypeDef  *psAdcV;         // ADC2 (M0) / ADC4 (M1) : IV

    // PWM Channel Mapping
    // M0: U=CH3, V=CH4, W=CH1 / M1: U=CH3, V=CH2, W=CH1
    uint32_t            nChU;
    uint32_t            nChV;
    uint32_t            nChW;

    // Hall Sensor
    uint8_t             nHallSum;           // HU*4 + HV*2 + HW  (1~6)
    volatile float      fRpm;              // Instantaneous RPM (updated in Hall ISR)
    volatile uint32_t   nHallHoldCnt;      // Stall detection counter (reset in Hall ISR)

    // Current Measurement
    float               fIu, fIv, fIw;
    float               fIu_LPF, fIv_LPF, fIw_LPF;
    uint32_t            nIu_Offset;
    uint32_t            nIv_Offset;
    uint32_t            nIw_Offset;
    float               fFi;               // Current LPF coefficient

    // Speed
    float               fMotorSpeedRpm;    // LPF-filtered speed [RPM]
    float               fFt;              // Speed LPF coefficient

    // Voltage, Temperature
    float               fVdc;             // Bus voltage [V]
    float               fTemp;            // MOSFET temperature [degC]

    // Control
    uint32_t            nVoltageRef;       // Final PWM duty (0 ~ CNT_MAX)
    float               fRpmRef;          // Speed reference [RPM] — set via Motor_SetRpmRef()
    float               fThrottleRamp;    // Ramped open-loop reference

    // PI
    float               fKp;
    float               fKi;
    float               fRpmErr;
    float               fPterm;
    float               fIterm;
    uint8_t             nSpdFlg;          // 0: open-loop  1: PI closed-loop
    uint8_t             nRestartRamp;     // 1: E-stop 해제 후 소프트스타트 중

    // Protection
    uint8_t             nFltFlg;          // Fault code (BLDC_FLT_xxx)
    uint16_t            nFltCnt;          // Overcurrent counter

    // Config
    uint8_t             nIdx;             // Motor index (0 or 1)
    uint8_t             nMotorDir;        // BLDC_DIR_CW / BLDC_DIR_CCW
    uint8_t             nEnabled;         // PWM output active flag

} sMotor;


//  Global Motor Array
extern sMotor g_sMotor[BLDC_MOTOR_COUNT];


//  Init
void Motor_Init(uint8_t nIdx);

//  PWM Control
void Motor_Enable(sMotor *psM);
void Motor_Disable(sMotor *psM);

//  Commutation
void Motor_SetPhases(sMotor *psM, int8_t nA, int8_t nB, int8_t nC);
void Motor_UpdateCommutation(sMotor *psM);

//  Control Loop / Task
void Motor_ControlLoop(sMotor *psM);   // TIM1/TIM8 Update ISR (20kHz)
void Motor_Task1ms(sMotor *psM);       // PI speed controller (SysTick scheduler)

//  Setters
void Motor_SetRpmRef(uint8_t nIdx, float fRpmRef);
void Motor_SetDir(uint8_t nIdx, uint8_t nDir);
void Motor_SetSpdFlg(uint8_t nIdx, uint8_t nFlg);
void Motor_ClearFault(uint8_t nIdx);

//  Utilities
void  Motor_LPF(float fInput, float fFx, float *pfOutput);
void  Motor_Ramp(float fTarget, float *pfOutput, float fSlope);
float Motor_CalcLpfCoeff(float fFc, float fTs);



#endif
