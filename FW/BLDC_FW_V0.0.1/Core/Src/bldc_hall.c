#include "bldc_hall.h"
#include "main.h"

/* DEBUG — reset to 0, rotate one full revolution, read value: count/6 = pole pairs.
 * Remove after verification. */
volatile uint32_t g_dbgHallEdgeCnt = 0U;


/* TIM3(M0): PC8=HU  PC7=HV  PC6=HW
 * TIM4(M1): PD14=HU PD13=HV PD12=HW */
void Hall_ReadState(sMotor *psM)
{
    uint8_t nHU, nHV, nHW;

    if (psM->nIdx == 0U)
    {
        nHU = (uint8_t)HAL_GPIO_ReadPin(M0_HU_T33_GPIO_Port, M0_HU_T33_Pin);
        nHV = (uint8_t)HAL_GPIO_ReadPin(M0_HV_T32_GPIO_Port, M0_HV_T32_Pin);
        nHW = (uint8_t)HAL_GPIO_ReadPin(M0_HW_T31_GPIO_Port, M0_HW_T31_Pin);
    }
    else
    {
        nHU = (uint8_t)HAL_GPIO_ReadPin(M1_HU_T43_GPIO_Port, M1_HU_T43_Pin);
        nHV = (uint8_t)HAL_GPIO_ReadPin(M1_HV_T42_GPIO_Port, M1_HV_T42_Pin);
        nHW = (uint8_t)HAL_GPIO_ReadPin(M1_HW_T41_GPIO_Port, M1_HW_T41_Pin);
    }

    psM->nHallSum = (nHU << 2U) | (nHV << 1U) | nHW;
}

/* Hall sensor interface mode fires on every TI1 (XOR) edge.
 * TIM3 → M0    TIM4 → M1
 * RPM = 60 × HALL_TIM_CLK_HZ / (HALL_EDGES_PER_REV × CCR1) */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    sMotor *psM = NULL;

    if (htim->Instance == TIM3)
    {
        psM = &g_sMotor[0];
    }
    else if (htim->Instance == TIM4)
    {
        psM = &g_sMotor[1];
    }
    else
    {
        return;
    }

    if (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_1)
    {
        return;
    }

    psM->nHallHoldCnt  = 0U;
    psM->nRestartRamp  = 0U;
    g_dbgHallEdgeCnt++;

    /* CCR1 = tick count between previous and current edge */
    uint32_t nCcr = htim->Instance->CCR1;
    if (nCcr > 0U)
    {
        psM->fRpm = (60.0f * (float)BLDC_HALL_TIM_CLK_HZ)
                    / ((float)BLDC_HALL_EDGES_PER_REV * (float)nCcr);
    }

    Hall_ReadState(psM);
    Motor_UpdateCommutation(psM);
}
