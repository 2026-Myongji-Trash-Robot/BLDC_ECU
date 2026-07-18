#include "bldc_mgr.h"
#include "bldc_motor.h"
#include "bldc_hall.h"
#include "bldc_adc.h"
#include "bldc_can.h"
#include "bldc_led.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
extern FDCAN_HandleTypeDef hfdcan1;

uint8_t g_nOperMode = BLDC_MODE_CAN;


void BLDC_Init(void)
{
    for (uint8_t i = 0U; i < BLDC_ACTIVE_MOTOR_COUNT; i++)
    {
        Motor_Init(i);
        ADC_CalibrateOffset(&g_sMotor[i]);
        Hall_ReadState(&g_sMotor[i]);
        Motor_UpdateCommutation(&g_sMotor[i]);

        /* Hall state outside 1~6: wiring fault or disconnected sensor.
         * Set fault and skip Enable to avoid commutating from an invalid state. */
        if (g_sMotor[i].nHallSum < 1U || g_sMotor[i].nHallSum > 6U)
        {
            g_sMotor[i].nFltFlg = BLDC_FLT_HALL;
            continue;
        }

        Motor_Enable(&g_sMotor[i]);
    }

    CAN_Init(&hfdcan1);

    LED_Init();

    /* MODE_CAN_EN: 3.3V → 10kΩ pull-up → DIP → GND
     * Switch OFF (open) : HIGH (pull-up) → CAN mode
     * Switch ON (closed): LOW  (GND)     → MANUAL mode */
    if (HAL_GPIO_ReadPin(MODE_CAN_EN_GPIO_Port, MODE_CAN_EN_Pin) == GPIO_PIN_SET)
    {
        g_nOperMode = BLDC_MODE_CAN;
    }
    else
    {
        g_nOperMode = BLDC_MODE_MANUAL;
    }
}


/* Software scheduler — call from while(1).
 *  1 ms : current sampling; PI controller (CAN) or RPM setpoint (MANUAL)
 * 10 ms : ADC5 NTC/bus-voltage, undervoltage+overheat protection, LEDs
 * 100 ms: CAN status TX + RX watchdog (CAN mode only) */
void BLDC_Run(void)
{
    static uint32_t s_nPrevTick = 0U;
    static uint8_t  s_n1msCnt  = 0U;
    static uint8_t  s_n10msCnt = 0U;

    uint32_t nNow = HAL_GetTick();

    if (nNow == s_nPrevTick)
    {
        return;
    }
    s_nPrevTick = nNow;

    /* 1 ms task */

    /* HAL ADC polling is unsafe inside ISR — sample currents here in the
     * main loop. Updated fIu_LPF/fIv_LPF/fIw_LPF are read by OC protection. */
#if (BLDC_HW_CURRENT_SENSING)
    for (uint8_t i = 0U; i < BLDC_ACTIVE_MOTOR_COUNT; i++)
    {
        if (g_sMotor[i].nEnabled != 0U)
        {
            ADC_SampleCurrents(&g_sMotor[i]);
        }
    }
#endif

    if (g_nOperMode == BLDC_MODE_CAN)
    {
        for (uint8_t i = 0U; i < BLDC_ACTIVE_MOTOR_COUNT; i++)
        {
            Motor_Task1ms(&g_sMotor[i]);
        }
    }
    else
    {
#if (BLDC_JOY_ENABLE)
        /* Discrete 4-zone control: threshold comparison only — immune to noise.
         * Y-axis priority: forward overrides any lateral input.
         * Differential steering: both wheels forward, inner/outer RPM differ. */
        if (g_fJoyY > BLDC_JOY_ZONE_THR)
        {
            /* forward */
            Motor_SetDir(0U, BLDC_M0_DEFAULT_DIR);
            Motor_SetDir(1U, BLDC_M1_DEFAULT_DIR);
            Motor_SetRpmRef(0U, BLDC_JOY_FWD_RPM);
            Motor_SetRpmRef(1U, BLDC_JOY_FWD_RPM);
        }
        else if (g_fJoyX > BLDC_JOY_ZONE_THR)
        {
            /* turn right — M0(left, outer) / M1(right, inner) */
            Motor_SetDir(0U, BLDC_M0_DEFAULT_DIR);
            Motor_SetDir(1U, BLDC_M1_DEFAULT_DIR);
            Motor_SetRpmRef(0U, BLDC_JOY_TURN_OUTER_RPM);
            Motor_SetRpmRef(1U, BLDC_JOY_TURN_INNER_RPM);
        }
        else if (g_fJoyX < -BLDC_JOY_ZONE_THR)
        {
            /* turn left — M0(left, inner) / M1(right, outer) */
            Motor_SetDir(0U, BLDC_M0_DEFAULT_DIR);
            Motor_SetDir(1U, BLDC_M1_DEFAULT_DIR);
            Motor_SetRpmRef(0U, BLDC_JOY_TURN_INNER_RPM);
            Motor_SetRpmRef(1U, BLDC_JOY_TURN_OUTER_RPM);
        }
        else
        {
            /* stop */
            Motor_SetRpmRef(0U, 0.0f);
            Motor_SetRpmRef(1U, 0.0f);
        }
#else
        /* MANUAL mode: fixed RPM */
        for (uint8_t i = 0U; i < BLDC_ACTIVE_MOTOR_COUNT; i++)
        {
            Motor_SetRpmRef(i, BLDC_NONCAN_FIXED_RPM);
            Motor_Task1ms(&g_sMotor[i]);
        }
#endif
    }

    s_n1msCnt++;

    /* 10 ms task */
    if (s_n1msCnt >= 10U)
    {
        ADC_Task10ms();
        LED_Task10ms();
        s_n1msCnt = 0U;
        s_n10msCnt++;
    }

    /* 100 ms task */
    if (s_n10msCnt >= 10U)
    {
        if (g_nOperMode == BLDC_MODE_CAN)
        {
            CAN_Task100ms();
        }
        s_n10msCnt = 0U;
    }
}


/* TIM8(M0) / TIM1(M1) Update ISR → 20 kHz control loop.
 * LED_SysTick called only from TIM8 → SYS LED 500 ms heartbeat. */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM8)
    {
        Motor_ControlLoop(&g_sMotor[0]);
        LED_SysTick();
    }
    else if (htim->Instance == TIM1)
    {
        Motor_ControlLoop(&g_sMotor[1]);
    }
}
