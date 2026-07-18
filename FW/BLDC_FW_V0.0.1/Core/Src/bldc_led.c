#include "bldc_led.h"
#include "main.h"

/* Sink-driven LEDs: GPIO LOW = ON */
#define LED_ON(port, pin)     HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET)
#define LED_OFF(port, pin)    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET)
#define LED_TOGGLE(port, pin) HAL_GPIO_TogglePin(port, pin)

#define LED_RUN_RPM_THRESHOLD   5.0f

static uint8_t s_nCanBlinkCnt = 0U;


void LED_Init(void)
{
    LED_OFF(M0_LED_36V_GPIO_Port,   M0_LED_36V_Pin);
    LED_OFF(M0_LED_RUN_GPIO_Port,   M0_LED_RUN_Pin);
    LED_OFF(M0_LED_ERR_GPIO_Port,   M0_LED_ERR_Pin);
    LED_OFF(M1_LED_36V_GPIO_Port,   M1_LED_36V_Pin);
    LED_OFF(M1_LED_RUN_GPIO_Port,   M1_LED_RUN_Pin);
    LED_OFF(M1_LED_ERR_GPIO_Port,   M1_LED_ERR_Pin);
    LED_OFF(CORE_LED_SYS_GPIO_Port, CORE_LED_SYS_Pin);
    LED_OFF(CORE_LED_CAN_GPIO_Port, CORE_LED_CAN_Pin);
}


/* Called from TIM8 Update ISR (20 kHz). Toggles SYS LED every 500 ms.
 * Blink stops when MOE=0 (e-stop active) so dark SYS signals HW brake. */
void LED_SysTick(void)
{
    static uint16_t s_nCnt = 0U;
    if (++s_nCnt < 10000U) { return; }
    s_nCnt = 0U;

    if ((g_sMotor[0].psTimPwm->Instance->BDTR & TIM_BDTR_MOE) != 0U)
    {
        LED_TOGGLE(CORE_LED_SYS_GPIO_Port, CORE_LED_SYS_Pin);
    }
    else
    {
        LED_OFF(CORE_LED_SYS_GPIO_Port, CORE_LED_SYS_Pin);
    }
}


/* Called on CAN RX — holds CAN LED on for 5 × 10 ms = 50 ms */
void LED_SignalCAN(void)
{
    s_nCanBlinkCnt = 5U;
}


void LED_Task10ms(void)
{
    for (uint8_t i = 0U; i < BLDC_ACTIVE_MOTOR_COUNT; i++)
    {
        GPIO_TypeDef *p36VPort, *pRunPort, *pErrPort;
        uint16_t      n36VPin,   nRunPin,   nErrPin;

        if (i == 0U)
        {
            p36VPort = M0_LED_36V_GPIO_Port; n36VPin = M0_LED_36V_Pin;
            pRunPort = M0_LED_RUN_GPIO_Port; nRunPin = M0_LED_RUN_Pin;
            pErrPort = M0_LED_ERR_GPIO_Port; nErrPin = M0_LED_ERR_Pin;
        }
        else
        {
            p36VPort = M1_LED_36V_GPIO_Port; n36VPin = M1_LED_36V_Pin;
            pRunPort = M1_LED_RUN_GPIO_Port; nRunPin = M1_LED_RUN_Pin;
            pErrPort = M1_LED_ERR_GPIO_Port; nErrPin = M1_LED_ERR_Pin;
        }

        if (g_sMotor[i].fVdc > BLDC_VDC_MIN)
        {
            LED_ON(p36VPort, n36VPin);
        }
        else
        {
            LED_OFF(p36VPort, n36VPin);
        }

        /* Use fMotorSpeedRpm (LPF) not fRpm — fRpm zeroed on stall kick */
        if (g_sMotor[i].nEnabled && g_sMotor[i].fMotorSpeedRpm > LED_RUN_RPM_THRESHOLD)
        {
            LED_ON(pRunPort, nRunPin);
        }
        else
        {
            LED_OFF(pRunPort, nRunPin);
        }

        if (g_sMotor[i].nFltFlg != BLDC_FLT_NONE)
        {
            LED_ON(pErrPort, nErrPin);
        }
        else
        {
            LED_OFF(pErrPort, nErrPin);
        }
    }

    if (s_nCanBlinkCnt > 0U)
    {
        s_nCanBlinkCnt--;
        LED_ON(CORE_LED_CAN_GPIO_Port, CORE_LED_CAN_Pin);
    }
    else
    {
        LED_OFF(CORE_LED_CAN_GPIO_Port, CORE_LED_CAN_Pin);
    }
}
