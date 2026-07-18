#include "bldc_can.h"
#include "bldc_motor.h"
#include "bldc_led.h"

static FDCAN_HandleTypeDef *s_phFdcan      = NULL;
static volatile uint32_t    s_nLastRxTick  = 0U;
static volatile uint8_t     s_nBusOffFlag  = 0U;   /* set in Bus-Off ISR, cleared in CAN_Task100ms */

#define CAN_RX_TIMEOUT_MS   500U    /* RX watchdog timeout — 2~3x ROS2 publish interval */


static int16_t _CAN_FloatToInt16(float fVal, float fLsb, float fMin, float fMax)
{
    float fScaled = fVal / fLsb;
    if (fScaled > fMax) { fScaled = fMax; }
    if (fScaled < fMin) { fScaled = fMin; }
    return (int16_t)fScaled;
}


void CAN_Init(FDCAN_HandleTypeDef *phFdcan)
{
    s_phFdcan = phFdcan;

    FDCAN_FilterTypeDef sFilter;
    sFilter.IdType       = FDCAN_STANDARD_ID;
    sFilter.FilterType   = FDCAN_FILTER_MASK;
    sFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilter.FilterID2    = 0x7FFU;  /* exact match */

    /* CMD filter */
    sFilter.FilterIndex = 0U;
    sFilter.FilterID1   = CAN_ID_CMD;
    HAL_FDCAN_ConfigFilter(phFdcan, &sFilter);

    /* reject non-matching frames */
    HAL_FDCAN_ConfigGlobalFilter(phFdcan,
                                 FDCAN_REJECT,
                                 FDCAN_REJECT,
                                 FDCAN_FILTER_REMOTE,
                                 FDCAN_FILTER_REMOTE);

    HAL_FDCAN_ActivateNotification(phFdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U);
    HAL_FDCAN_ActivateNotification(phFdcan, FDCAN_IT_BUS_OFF, 0U);

    HAL_FDCAN_Start(phFdcan);
}


static void _CAN_SendStatus(uint32_t nId, sMotor *psM)
{
    FDCAN_TxHeaderTypeDef sTxHeader;
    uint8_t aData[8];

    /* B[0:1] RPM — negative = reverse */
    float fRpmSigned = (psM->nMotorDir == BLDC_DIR_CW) ? psM->fMotorSpeedRpm : -psM->fMotorSpeedRpm;
    int16_t nRpm = _CAN_FloatToInt16(fRpmSigned,
                                      CAN_STATUS_RPM_LSB, -32768.0f, 32767.0f);

    /* B[2:3] max current (abs) */
    float fIu  = psM->fIu_LPF < 0.0f ? -psM->fIu_LPF : psM->fIu_LPF;
    float fIv  = psM->fIv_LPF < 0.0f ? -psM->fIv_LPF : psM->fIv_LPF;
    float fIw  = psM->fIw_LPF < 0.0f ? -psM->fIw_LPF : psM->fIw_LPF;
    float fIMax = (fIu > fIv) ? fIu : fIv;
    if (fIw > fIMax) { fIMax = fIw; }
    int16_t nCurr = _CAN_FloatToInt16(fIMax,
                                       CAN_STATUS_CURR_LSB, 0.0f, 32767.0f);

    /* B[4] temperature — clamp 0~100°C */
    float fTemp = psM->fTemp;
    if (fTemp > 100.0f) { fTemp = 100.0f; }
    if (fTemp <   0.0f) { fTemp =   0.0f; }

    /* B[6] status flags */
    uint8_t nFlags = 0U;
    if (psM->nEnabled)                  { nFlags |= CAN_STATUS_FL_ENABLED;  }
    if (psM->nSpdFlg)                   { nFlags |= CAN_STATUS_FL_PI;       }
    if (psM->fMotorSpeedRpm > 1.0f)     { nFlags |= CAN_STATUS_FL_SPINNING; }

    aData[0] = (uint8_t)( nRpm        & 0xFF);
    aData[1] = (uint8_t)((nRpm >> 8)  & 0xFF);
    aData[2] = (uint8_t)( nCurr       & 0xFF);
    aData[3] = (uint8_t)((nCurr >> 8) & 0xFF);
    aData[4] = (uint8_t)(int8_t)fTemp;
    aData[5] = psM->nFltFlg;
    aData[6] = nFlags;
    aData[7] = 0x00U;

    sTxHeader.Identifier          = nId;
    sTxHeader.IdType              = FDCAN_STANDARD_ID;
    sTxHeader.TxFrameType         = FDCAN_DATA_FRAME;
    sTxHeader.DataLength          = FDCAN_DLC_BYTES_8;
    sTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    sTxHeader.BitRateSwitch       = FDCAN_BRS_OFF;
    sTxHeader.FDFormat            = FDCAN_CLASSIC_CAN;
    sTxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    sTxHeader.MessageMarker       = 0U;

    HAL_FDCAN_AddMessageToTxFifoQ(s_phFdcan, &sTxHeader, aData);
}


void CAN_Task100ms(void)
{
    /* [1] Bus-Off recovery — flag set in ISR, restart here */
    if (s_nBusOffFlag != 0U)
    {
        s_nBusOffFlag = 0U;
        HAL_FDCAN_Stop(s_phFdcan);
        HAL_FDCAN_Start(s_phFdcan);
        HAL_FDCAN_ActivateNotification(s_phFdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U);
        HAL_FDCAN_ActivateNotification(s_phFdcan, FDCAN_IT_BUS_OFF, 0U);
        return;
    }

    /* [2] RX watchdog */
    if ((HAL_GetTick() - s_nLastRxTick) > CAN_RX_TIMEOUT_MS)
    {
        Motor_SetRpmRef(0U, 0.0f);
        Motor_SetRpmRef(1U, 0.0f);
    }

    /* [3] status TX */
    _CAN_SendStatus(CAN_ID_STATUS_M0, &g_sMotor[0]);
    _CAN_SendStatus(CAN_ID_STATUS_M1, &g_sMotor[1]);
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
    {
        return;
    }

    FDCAN_RxHeaderTypeDef sRxHeader;
    uint8_t aData[8];

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &sRxHeader, aData) != HAL_OK)
    {
        return;
    }

    /* manual mode: ignore CAN commands */
    if (g_nOperMode != BLDC_MODE_CAN)
    {
        return;
    }

    s_nLastRxTick = HAL_GetTick();
    LED_SignalCAN();

    switch (sRxHeader.Identifier)
    {
        case CAN_ID_CMD:
        {
            if (sRxHeader.DataLength < FDCAN_DLC_BYTES_8)
            {
                break;
            }

            /* B[0:1] M0 RPM / B[2:3] M1 RPM — negative = reverse */
            int16_t nM0Raw = (int16_t)((uint16_t)aData[0] | ((uint16_t)aData[1] << 8U));
            int16_t nM1Raw = (int16_t)((uint16_t)aData[2] | ((uint16_t)aData[3] << 8U));

            Motor_SetDir(0U, nM0Raw >= 0 ? BLDC_M0_DEFAULT_DIR : !BLDC_M0_DEFAULT_DIR);
            Motor_SetDir(1U, nM1Raw >= 0 ? BLDC_M1_DEFAULT_DIR : !BLDC_M1_DEFAULT_DIR);

            float fM0Rpm = (float)(nM0Raw < 0 ? -nM0Raw : nM0Raw) * CAN_CMD_RPM_LSB;
            float fM1Rpm = (float)(nM1Raw < 0 ? -nM1Raw : nM1Raw) * CAN_CMD_RPM_LSB;

            Motor_SetRpmRef(0U, fM0Rpm);
            Motor_SetRpmRef(1U, fM1Rpm);

            /* B[4] control mode */
            uint8_t nMode = aData[4];

            /* null guard: psTimPwm=NULL when ACTIVE_MOTOR_COUNT=1 */
            if (nMode == CAN_CMD_MODE_RUN)
            {
                if (g_sMotor[0].psTimPwm != NULL) { Motor_Enable(&g_sMotor[0]); }
                if (g_sMotor[1].psTimPwm != NULL) { Motor_Enable(&g_sMotor[1]); }
            }
            else
            {
                if (g_sMotor[0].psTimPwm != NULL) { Motor_Disable(&g_sMotor[0]); }
                if (g_sMotor[1].psTimPwm != NULL) { Motor_Disable(&g_sMotor[1]); }
            }

            /* B[5] fault clear */
            uint8_t nFltClr = aData[5];
            if (nFltClr & CAN_FLTCLR_ALL)
            {
                if (g_sMotor[0].psTimPwm != NULL) { Motor_ClearFault(0U); }
                if (g_sMotor[1].psTimPwm != NULL) { Motor_ClearFault(1U); }
            }
            else
            {
                if ((nFltClr & CAN_FLTCLR_M0) && g_sMotor[0].psTimPwm != NULL) { Motor_ClearFault(0U); }
                if ((nFltClr & CAN_FLTCLR_M1) && g_sMotor[1].psTimPwm != NULL) { Motor_ClearFault(1U); }
            }

            break;
        }

        default:
            break;
    }
}


void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) == 0U)
    {
        return;
    }

    /* stop motors immediately */
    Motor_SetRpmRef(0U, 0.0f);
    Motor_SetRpmRef(1U, 0.0f);

    /* FDCAN restart deferred to CAN_Task100ms */
    s_nBusOffFlag = 1U;
}
