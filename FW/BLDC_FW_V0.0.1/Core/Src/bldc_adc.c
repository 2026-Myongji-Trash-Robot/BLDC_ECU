#include "bldc_adc.h"

extern ADC_HandleTypeDef hadc5;


float g_fJoyX = 0.0f;
float g_fJoyY = 0.0f;


static float _ADC_RawToAmpere(uint32_t nRaw, uint32_t nOffset)
{
    float fV = ((float)nRaw - (float)nOffset) * (BLDC_ADC_VREF / BLDC_ADC_FS);
    return fV / BLDC_OPAMP_GAIN;
}


void ADC_CalibrateOffset(sMotor *psM)
{
    uint32_t nSumU = 0U;
    uint32_t nSumV = 0U;
    uint32_t nSumW = 0U;

    for (uint8_t i = 0U; i < BLDC_CURRENT_CAL_CNT; i++)
    {
        /* IU, IW — ADC1(M0) / ADC3(M1) */
        HAL_ADC_Start(psM->psAdcUW);
        HAL_ADC_PollForConversion(psM->psAdcUW, 2U);
        nSumU += HAL_ADC_GetValue(psM->psAdcUW);
        HAL_ADC_PollForConversion(psM->psAdcUW, 2U);
        nSumW += HAL_ADC_GetValue(psM->psAdcUW);
        HAL_ADC_Stop(psM->psAdcUW);

        /* IV — ADC2(M0) / ADC4(M1) */
        HAL_ADC_Start(psM->psAdcV);
        HAL_ADC_PollForConversion(psM->psAdcV, 2U);
        nSumV += HAL_ADC_GetValue(psM->psAdcV);
        HAL_ADC_Stop(psM->psAdcV);

        HAL_Delay(1U);
    }

    psM->nIu_Offset = nSumU / BLDC_CURRENT_CAL_CNT;
    psM->nIv_Offset = nSumV / BLDC_CURRENT_CAL_CNT;
    psM->nIw_Offset = nSumW / BLDC_CURRENT_CAL_CNT;
}


void ADC_SampleCurrents(sMotor *psM)
{
    uint32_t nRawU, nRawV, nRawW;

    /* IU, IW */
    HAL_ADC_Start(psM->psAdcUW);
    HAL_ADC_PollForConversion(psM->psAdcUW, 1U);
    nRawU = HAL_ADC_GetValue(psM->psAdcUW);
    HAL_ADC_PollForConversion(psM->psAdcUW, 1U);
    nRawW = HAL_ADC_GetValue(psM->psAdcUW);
    HAL_ADC_Stop(psM->psAdcUW);

    /* IV */
    HAL_ADC_Start(psM->psAdcV);
    HAL_ADC_PollForConversion(psM->psAdcV, 1U);
    nRawV = HAL_ADC_GetValue(psM->psAdcV);
    HAL_ADC_Stop(psM->psAdcV);

    /* convert to [A] */
    psM->fIu = _ADC_RawToAmpere(nRawU, psM->nIu_Offset);
    psM->fIv = _ADC_RawToAmpere(nRawV, psM->nIv_Offset);
    psM->fIw = _ADC_RawToAmpere(nRawW, psM->nIw_Offset);

    /* LPF — fFi computed in Motor_Init() */
    Motor_LPF(psM->fIu, psM->fFi, &psM->fIu_LPF);
    Motor_LPF(psM->fIv, psM->fFi, &psM->fIv_LPF);
    Motor_LPF(psM->fIw, psM->fFi, &psM->fIw_LPF);
}


static uint32_t _ADC5_ReadChannel(uint32_t nChannel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel      = nChannel;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset       = 0U;
    HAL_ADC_ConfigChannel(&hadc5, &sConfig);

    HAL_ADC_Start(&hadc5);
    HAL_ADC_PollForConversion(&hadc5, 2U);
    uint32_t nVal = HAL_ADC_GetValue(&hadc5);
    HAL_ADC_Stop(&hadc5);
    return nVal;
}


/* Joystick channels: long sampling time for high-impedance source.
 * First conversion discarded — sampling capacitor settles after channel switch.
 * Result is average of 4 subsequent conversions. */
static uint32_t _ADC5_ReadJoy(uint32_t nChannel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel      = nChannel;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset       = 0U;
    HAL_ADC_ConfigChannel(&hadc5, &sConfig);

    /* discard first sample — settling after channel switch */
    HAL_ADC_Start(&hadc5);
    HAL_ADC_PollForConversion(&hadc5, 2U);
    (void)HAL_ADC_GetValue(&hadc5);
    HAL_ADC_Stop(&hadc5);

    /* average 4 samples */
    uint32_t nSum = 0U;
    for (uint8_t i = 0U; i < 4U; i++)
    {
        HAL_ADC_Start(&hadc5);
        HAL_ADC_PollForConversion(&hadc5, 2U);
        nSum += HAL_ADC_GetValue(&hadc5);
        HAL_ADC_Stop(&hadc5);
    }
    return nSum / 4U;
}


void ADC_Task10ms(void)
{
    uint32_t nM0Ntc, nM0Vdc, nM1Ntc, nM1Vdc, nJoyX, nJoyY;
    float fV;

    nM0Ntc = _ADC5_ReadChannel(ADC_CHANNEL_1);    /* PA8  CH1  — M0 NTC */
    nM0Vdc = _ADC5_ReadChannel(ADC_CHANNEL_2);    /* PA9  CH2  — M0 36V */
    nM1Vdc = _ADC5_ReadChannel(ADC_CHANNEL_12);   /* PD8  CH12 — M1 36V */
    nM1Ntc = _ADC5_ReadChannel(ADC_CHANNEL_13);   /* PD9  CH13 — M1 NTC */
    nJoyX  = _ADC5_ReadJoy(ADC_CHANNEL_8);        /* PD11 CH8  — JOY X (channel-swapped) */
    nJoyY  = _ADC5_ReadJoy(ADC_CHANNEL_7);        /* PD10 CH7  — JOY Y (channel-swapped) */

    /* M0 temperature */
#if (BLDC_HW_NTC)
    fV = (float)nM0Ntc * (BLDC_ADC_VREF / BLDC_ADC_FS);
    g_sMotor[0].fTemp = ((BLDC_NTC_A3 * fV + BLDC_NTC_A2) * fV + BLDC_NTC_A1) * fV + BLDC_NTC_A0;
#endif

    /* M0 bus voltage */
    fV = (float)nM0Vdc * (BLDC_ADC_VREF / BLDC_ADC_FS);
    g_sMotor[0].fVdc = fV / BLDC_VDIV_RATIO;

    /* M1 temperature */
#if (BLDC_HW_NTC)
    fV = (float)nM1Ntc * (BLDC_ADC_VREF / BLDC_ADC_FS);
    g_sMotor[1].fTemp = ((BLDC_NTC_A3 * fV + BLDC_NTC_A2) * fV + BLDC_NTC_A1) * fV + BLDC_NTC_A0;
#endif

    /* M1 bus voltage */
    fV = (float)nM1Vdc * (BLDC_ADC_VREF / BLDC_ADC_FS);
    g_sMotor[1].fVdc = fV / BLDC_VDIV_RATIO;

    /* Joystick normalization — asymmetric range: 5V joystick on 3.3V ADC.
     * Both halves mapped independently to [-1.0, +1.0]. */
    const float fCenter = (float)BLDC_JOY_ADC_CENTER;
    float fJoyXRaw, fJoyYRaw;

    fJoyXRaw = ((float)nJoyX >= fCenter)
               ? ((float)nJoyX - fCenter) / (4095.0f - fCenter)
               : ((float)nJoyX - fCenter) / fCenter;

    fJoyYRaw = ((float)nJoyY >= fCenter)
               ? ((float)nJoyY - fCenter) / (4095.0f - fCenter)
               : ((float)nJoyY - fCenter) / fCenter;

    if (fJoyXRaw >  1.0f) { fJoyXRaw =  1.0f; }
    if (fJoyXRaw < -1.0f) { fJoyXRaw = -1.0f; }
    if (fJoyYRaw >  1.0f) { fJoyYRaw =  1.0f; }
    if (fJoyYRaw < -1.0f) { fJoyYRaw = -1.0f; }

    /* axis inversion */
#if (BLDC_JOY_INVERT_X)
    fJoyXRaw = -fJoyXRaw;
#endif
#if (BLDC_JOY_INVERT_Y)
    fJoyYRaw = -fJoyYRaw;
#endif

    /* LPF — noise suppression (α≈0.2, fc≈3Hz @ 10ms) */
    g_fJoyX += 0.2f * (fJoyXRaw - g_fJoyX);
    g_fJoyY += 0.2f * (fJoyYRaw - g_fJoyY);

    /* deadband */
    if (g_fJoyX > -BLDC_JOY_DEADBAND && g_fJoyX < BLDC_JOY_DEADBAND) { g_fJoyX = 0.0f; }
    if (g_fJoyY > -BLDC_JOY_DEADBAND && g_fJoyY < BLDC_JOY_DEADBAND) { g_fJoyY = 0.0f; }

    /* M0 protection */
    /* undervoltage: latch fault only when powered (>1V); clear via Motor_ClearFault */
    if (g_sMotor[0].fVdc > 1.0f && g_sMotor[0].fVdc < BLDC_VDC_MIN)
    {
        if (g_sMotor[0].nFltFlg == BLDC_FLT_NONE)
        {
            g_sMotor[0].nFltFlg = BLDC_FLT_UNDERVOLT;
        }
    }
    /* overheat: latch fault — no auto-recovery, operator must clear manually */
#if (BLDC_HW_NTC)
    if (g_sMotor[0].fTemp > BLDC_TEMP_FAULT)
    {
        g_sMotor[0].nFltFlg = BLDC_FLT_OVERHEAT;
    }
#endif

    /* M1 protection */
    if (g_sMotor[1].fVdc > 1.0f && g_sMotor[1].fVdc < BLDC_VDC_MIN)
    {
        if (g_sMotor[1].nFltFlg == BLDC_FLT_NONE)
        {
            g_sMotor[1].nFltFlg = BLDC_FLT_UNDERVOLT;
        }
    }
#if (BLDC_HW_NTC)
    if (g_sMotor[1].fTemp > BLDC_TEMP_FAULT)
    {
        g_sMotor[1].nFltFlg = BLDC_FLT_OVERHEAT;
    }
#endif
}
