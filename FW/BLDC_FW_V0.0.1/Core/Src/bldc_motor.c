#include <string.h>
#include <math.h>
#include "bldc_motor.h"
#include "bldc_adc.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern ADC_HandleTypeDef hadc4;

sMotor g_sMotor[BLDC_MOTOR_COUNT];

/* TIM_CHANNEL_x >> 2 = index (0~3)
 * CCER mask per channel: (CC1E | CC1NE) << (index * 4) */
static void _Motor_MaskChannel(sMotor *psM, uint32_t nCh)
{
    uint32_t nIdx = nCh >> 2U;
    psM->psTimPwm->Instance->CCER &=
        ~((TIM_CCER_CC1E | TIM_CCER_CC1NE) << (nIdx * 4U));
}

static void _Motor_UnmaskChannel(sMotor *psM, uint32_t nCh)
{
    uint32_t nIdx = nCh >> 2U;
    psM->psTimPwm->Instance->CCER |=
        (TIM_CCER_CC1E | TIM_CCER_CC1NE) << (nIdx * 4U);
}

float Motor_CalcLpfCoeff(float fFc, float fTs)
{
    float fW = 2.0f * PI * fFc * fTs;
    return fW / (1.0f + fW);
}

void Motor_LPF(float fInput, float fFx, float *pfOutput)
{
    *pfOutput = (1.0f - fFx) * (*pfOutput) + fFx * fInput;
}

void Motor_Ramp(float fTarget, float *pfOutput, float fSlope)
{
    if (*pfOutput < fTarget)
    {
        *pfOutput += fSlope;
        if (*pfOutput > fTarget) { *pfOutput = fTarget; }
    }
    else if (*pfOutput > fTarget)
    {
        *pfOutput -= fSlope;
        if (*pfOutput < fTarget) { *pfOutput = fTarget; }
    }
}

void Motor_Init(uint8_t nIdx)
{
    sMotor *psM = &g_sMotor[nIdx];

    memset(psM, 0, sizeof(sMotor));

    psM->nIdx      = nIdx;
    psM->nMotorDir = (nIdx == 0U) ? BLDC_M0_DEFAULT_DIR : BLDC_M1_DEFAULT_DIR;
    psM->nSpdFlg   = (BLDC_PI_KP > 0.0f) ? 1U : 0U;
    psM->fKp       = BLDC_PI_KP;
    psM->fKi       = BLDC_PI_KI;

    psM->fFi = Motor_CalcLpfCoeff(BLDC_LPF_FC_CURRENT, BLDC_TSAMP);
    psM->fFt = Motor_CalcLpfCoeff(BLDC_LPF_FC_SPEED,   BLDC_TSAMP);

    if (nIdx == 0U)
    {
        psM->psTimPwm  = &htim8;
        psM->psTimHall = &htim3;
        psM->psAdcUW   = &hadc1;
        psM->psAdcV    = &hadc2;
        psM->nChU      = M0_TIM_CH_U;
        psM->nChV      = M0_TIM_CH_V;
        psM->nChW      = M0_TIM_CH_W;
    }
    else
    {
        psM->psTimPwm  = &htim1;
        psM->psTimHall = &htim4;
        psM->psAdcUW   = &hadc3;
        psM->psAdcV    = &hadc4;
        psM->nChU      = M1_TIM_CH_U;
        psM->nChV      = M1_TIM_CH_V;
        psM->nChW      = M1_TIM_CH_W;
    }

    HAL_TIM_PWM_Start(psM->psTimPwm,    psM->nChU);
    HAL_TIMEx_PWMN_Start(psM->psTimPwm, psM->nChU);
    HAL_TIM_PWM_Start(psM->psTimPwm,    psM->nChV);
    HAL_TIMEx_PWMN_Start(psM->psTimPwm, psM->nChV);
    HAL_TIM_PWM_Start(psM->psTimPwm,    psM->nChW);
    HAL_TIMEx_PWMN_Start(psM->psTimPwm, psM->nChW);

    __HAL_TIM_ENABLE_IT(psM->psTimPwm, TIM_IT_UPDATE);

    /* BKIN circuit: 10kΩ pull-up to 3.3V, NC e-stop switch to GND.
     * Switch closed (normal) : BKIN=LOW  → brake inactive
     * Switch open  (e-stop)  : BKIN=HIGH → brake fires → motor stops
     * Wire break             : pull-up → HIGH → brake fires (fail-safe)
     * MOE auto-restored by AOE when BKIN returns LOW. */
    psM->psTimPwm->Instance->BDTR |= TIM_BDTR_BKE;

    _Motor_MaskChannel(psM, psM->nChU);
    _Motor_MaskChannel(psM, psM->nChV);
    _Motor_MaskChannel(psM, psM->nChW);

    HAL_TIMEx_HallSensor_Start_IT(psM->psTimHall);
}

void Motor_Enable(sMotor *psM)
{
    psM->psTimPwm->Instance->BDTR |= TIM_BDTR_MOE;
    psM->nEnabled = 1U;
}

void Motor_Disable(sMotor *psM)
{
    psM->psTimPwm->Instance->BDTR &= ~TIM_BDTR_MOE;
    _Motor_MaskChannel(psM, psM->nChU);
    _Motor_MaskChannel(psM, psM->nChV);
    _Motor_MaskChannel(psM, psM->nChW);
    __HAL_TIM_SET_COMPARE(psM->psTimPwm, psM->nChU, 0U);
    __HAL_TIM_SET_COMPARE(psM->psTimPwm, psM->nChV, 0U);
    __HAL_TIM_SET_COMPARE(psM->psTimPwm, psM->nChW, 0U);
    psM->nEnabled      = 0U;
    psM->nVoltageRef   = 0U;
    psM->fThrottleRamp = 0.0f;
}

/* nA=U  nB=V  nC=W    1=PWM  -1=100% ON  0=float */
void Motor_SetPhases(sMotor *psM, int8_t nA, int8_t nB, int8_t nC)
{
    if (psM->nVoltageRef == 0U || psM->nEnabled == 0U)
    {
        _Motor_MaskChannel(psM, psM->nChU);
        _Motor_MaskChannel(psM, psM->nChV);
        _Motor_MaskChannel(psM, psM->nChW);
        return;
    }

    uint32_t nDuty = psM->nVoltageRef;

    if      (nA ==  1) { __HAL_TIM_SET_COMPARE(psM->psTimPwm, psM->nChU, nDuty); _Motor_UnmaskChannel(psM, psM->nChU); }
    else if (nA == -1) { __HAL_TIM_SET_COMPARE(psM->psTimPwm, psM->nChU, 0U);    _Motor_UnmaskChannel(psM, psM->nChU); }
    else               { __HAL_TIM_SET_COMPARE(psM->psTimPwm, psM->nChU, 0U);    _Motor_MaskChannel(psM,   psM->nChU); }

    if      (nB ==  1) { __HAL_TIM_SET_COMPARE(psM->psTimPwm, psM->nChV, nDuty); _Motor_UnmaskChannel(psM, psM->nChV); }
    else if (nB == -1) { __HAL_TIM_SET_COMPARE(psM->psTimPwm, psM->nChV, 0U);    _Motor_UnmaskChannel(psM, psM->nChV); }
    else               { __HAL_TIM_SET_COMPARE(psM->psTimPwm, psM->nChV, 0U);    _Motor_MaskChannel(psM,   psM->nChV); }

    if      (nC ==  1) { __HAL_TIM_SET_COMPARE(psM->psTimPwm, psM->nChW, nDuty); _Motor_UnmaskChannel(psM, psM->nChW); }
    else if (nC == -1) { __HAL_TIM_SET_COMPARE(psM->psTimPwm, psM->nChW, 0U);    _Motor_UnmaskChannel(psM, psM->nChW); }
    else               { __HAL_TIM_SET_COMPARE(psM->psTimPwm, psM->nChW, 0U);    _Motor_MaskChannel(psM,   psM->nChW); }
}

static void _Motor_PhasesFwd(sMotor *psM)
{
    switch (psM->nHallSum)
    {
        case 5: Motor_SetPhases(psM,  0, -1,  1); break;
        case 3: Motor_SetPhases(psM, -1,  1,  0); break;
        case 1: Motor_SetPhases(psM, -1,  0,  1); break;
        case 6: Motor_SetPhases(psM,  1,  0, -1); break;
        case 4: Motor_SetPhases(psM,  1, -1,  0); break;
        case 2: Motor_SetPhases(psM,  0,  1, -1); break;
        default: Motor_SetPhases(psM, 0,  0,  0); break;
    }
}

static void _Motor_PhasesRev(sMotor *psM)
{
    switch (psM->nHallSum)
    {
        case 5: Motor_SetPhases(psM,  0,  1, -1); break;
        case 3: Motor_SetPhases(psM,  1, -1,  0); break;
        case 1: Motor_SetPhases(psM,  1,  0, -1); break;
        case 6: Motor_SetPhases(psM, -1,  0,  1); break;
        case 4: Motor_SetPhases(psM, -1,  1,  0); break;
        case 2: Motor_SetPhases(psM,  0, -1,  1); break;
        default: Motor_SetPhases(psM,  0,  0,  0); break;
    }
}

void Motor_UpdateCommutation(sMotor *psM)
{
    if (psM->nMotorDir == BLDC_DIR_CW)
    {
        _Motor_PhasesFwd(psM);
    }
    else
    {
        _Motor_PhasesRev(psM);
    }
}

void Motor_ControlLoop(sMotor *psM)
{
    /* E-stop guard: hardware clears MOE when BKIN fires.
     * Reset ramp/stall/ref so re-enable starts cleanly from zero. */
    if ((psM->psTimPwm->Instance->BDTR & TIM_BDTR_MOE) == 0U)
    {
        psM->fThrottleRamp = 0.0f;
        psM->nHallHoldCnt  = 0U;
        psM->nVoltageRef   = 0U;
        psM->fIterm        = 0.0f;
        psM->nRestartRamp  = 1U;
        Motor_SetPhases(psM, 0, 0, 0);
        return;
    }

    /* Current sampling is done in the 1ms task (bldc_mgr.c) — HAL polling
     * is unsafe inside ISR. Use pre-computed fIu_LPF/fIv_LPF/fIw_LPF here. */

    /* Hall stall detection — nHallHoldCnt reset to 0 on each Hall edge */
    psM->nHallHoldCnt++;
    if (psM->nHallHoldCnt > 20000U)   /* 20000 × 50µs = 1 s */
    {
        psM->fRpm         = 0.0f;
        psM->nHallHoldCnt = 0U;

        /* Magnetic lock-up prevention: if the rotor holds a fixed Hall state,
         * torque drops to zero at the equilibrium point.  Force-advance to the
         * next commutation step every 1 s to escape the dead zone.
         * CW sequence: 5→3→1→6→4→2→5   CCW: 5→2→4→6→1→3→5 */
        if (psM->fRpmRef > 0.0f && psM->nEnabled != 0U)
        {
            static const uint8_t s_nNextCW[7]  = {0U, 6U, 5U, 1U, 2U, 3U, 4U};
            static const uint8_t s_nNextCCW[7] = {0U, 3U, 4U, 5U, 6U, 2U, 1U};
            uint8_t nCur = psM->nHallSum;
            if (nCur >= 1U && nCur <= 6U)
            {
                psM->nHallSum = (psM->nMotorDir == BLDC_DIR_CW)
                                ? s_nNextCW[nCur] : s_nNextCCW[nCur];
            }
        }
    }

#if (BLDC_HW_CURRENT_SENSING)
    if (psM->nEnabled != 0U)
    {
        float fIu   = fabsf(psM->fIu_LPF);
        float fIv   = fabsf(psM->fIv_LPF);
        float fIw   = fabsf(psM->fIw_LPF);
        float fIMax = (fIu > fIv) ? fIu : fIv;
        if (fIw > fIMax) { fIMax = fIw; }

        if (fIMax > BLDC_OC_LEVEL)
        {
            psM->nFltCnt++;
            if (psM->nFltCnt >= (uint16_t)BLDC_OC_TRIP_COUNT)
            {
                psM->nFltFlg     = BLDC_FLT_OVERCURRENT;
                psM->nVoltageRef = 0U;
            }
        }
        else if (fIMax < BLDC_OC_CLEAR_LEVEL)
        {
            psM->nFltCnt = 0U;
        }
    }
#endif

    Motor_LPF(psM->fRpm, psM->fFt, &psM->fMotorSpeedRpm);

    if (psM->nSpdFlg == 0U)
    {
        /* Open-loop: linear RPM→duty scale. Tune after measurement. */
        float fTarget = (psM->fRpmRef / BLDC_MAX_RPM_OPENLOOP) * (float)BLDC_CNT_MAX;
        if (fTarget > (float)BLDC_MAX_DUTY_CNT) { fTarget = (float)BLDC_MAX_DUTY_CNT; }
        if (fTarget < 0.0f)                     { fTarget = 0.0f; }

        Motor_Ramp(fTarget, &psM->fThrottleRamp, BLDC_RAMP_SLOPE);
        psM->nVoltageRef = (uint32_t)psM->fThrottleRamp;
    }
    else if (psM->nRestartRamp != 0U)
    {
        /* PI 모드 E-stop 해제 직후: 개루프와 동일한 램프로 소프트스타트.
         * MAX_DUTY까지 램프 → 첫 홀 엣지 수신 시 nRestartRamp=0 → 정상 PI 복귀. */
        Motor_Ramp((float)BLDC_MAX_DUTY_CNT, &psM->fThrottleRamp, BLDC_RAMP_SLOPE);
        psM->nVoltageRef = (uint32_t)psM->fThrottleRamp;
    }
    /* nSpdFlg == 1 && nRestartRamp == 0: nVoltageRef updated in Motor_Task1ms */

    if (psM->nFltFlg == BLDC_FLT_NONE)
    {
        Motor_UpdateCommutation(psM);
    }
    else
    {
        Motor_Disable(psM);
        /* Fault on one motor → stop the other: both share the 36V bus.
         * Null guard required: psTimPwm=NULL when ACTIVE_MOTOR_COUNT=1. */
        uint8_t nOther = (psM->nIdx == 0U) ? 1U : 0U;
        if (g_sMotor[nOther].psTimPwm != NULL)
        {
            Motor_Disable(&g_sMotor[nOther]);
            if (g_sMotor[nOther].nFltFlg == BLDC_FLT_NONE)
            {
                g_sMotor[nOther].nFltFlg = psM->nFltFlg;
            }
        }
    }
}

void Motor_Task1ms(sMotor *psM)
{
    if (psM->nSpdFlg != 1U)
    {
        psM->fPterm = 0.0f;
        psM->fIterm = 0.0f;
        return;
    }

    psM->fRpmErr  = psM->fRpmRef - psM->fMotorSpeedRpm;
    psM->fPterm   = psM->fKp * psM->fRpmErr;
    psM->fIterm  += psM->fKi * psM->fRpmErr * 0.001f;

    float fPIOut  = psM->fPterm + psM->fIterm;

    /* Anti-windup — back-calculation */
    if (fPIOut > (float)BLDC_MAX_DUTY_CNT)
    {
        fPIOut      = (float)BLDC_MAX_DUTY_CNT;
        psM->fIterm = fPIOut - psM->fPterm;
    }
    if (fPIOut < 0.0f)
    {
        fPIOut      = 0.0f;
        psM->fIterm = 0.0f;
    }

    psM->nVoltageRef = (uint32_t)fPIOut;
}

void Motor_SetRpmRef(uint8_t nIdx, float fRpmRef)
{
    if (nIdx >= BLDC_MOTOR_COUNT) { return; }
    g_sMotor[nIdx].fRpmRef = fRpmRef;
}

void Motor_SetDir(uint8_t nIdx, uint8_t nDir)
{
    if (nIdx >= BLDC_MOTOR_COUNT) { return; }
    g_sMotor[nIdx].nMotorDir = nDir;
}

void Motor_SetSpdFlg(uint8_t nIdx, uint8_t nFlg)
{
    if (nIdx >= BLDC_MOTOR_COUNT) { return; }
    g_sMotor[nIdx].nSpdFlg = nFlg;
}

void Motor_ClearFault(uint8_t nIdx)
{
    if (nIdx >= BLDC_MOTOR_COUNT) { return; }

    sMotor *psM = &g_sMotor[nIdx];

    psM->nFltFlg       = BLDC_FLT_NONE;
    psM->nFltCnt       = 0U;
    psM->fThrottleRamp = 0.0f;

    Motor_Enable(psM);
}
