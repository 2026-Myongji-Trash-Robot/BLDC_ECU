#ifndef INC_BLDC_CONFIG_H_
#define INC_BLDC_CONFIG_H_

#include "main.h"


//--------------------------------------------------------------------------------
// MOTOR Config
//--------------------------------------------------------------------------------

#define BLDC_VOLTAGE_PROFILE        36		// Voltage system: 12 or 36

#if (BLDC_VOLTAGE_PROFILE == 36)
  #define BLDC_NOMINAL_VOLTAGE      36.0f
  #define BLDC_VDC_MIN              28.0f   // undervoltage trip (~88%)
  #define BLDC_JOY_MAX_RPM          300.0f  // manual mode max RPM
  #define BLDC_OC_LEVEL             20.0f   // overcurrent trip [A]  — verify with motor
  #define BLDC_OC_CLEAR_LEVEL       15.0f   // overcurrent hysteresis clear [A]
  #define BLDC_MOTOR_POLE_PAIRS     10      // measured: 60 edges/rev ÷ 6
  #define BLDC_MAX_RPM_OPENLOOP     5775.0f  // no-load max RPM — measured
  #define BLDC_PI_KP                0.0f    // PI proportional gain — tune after measurement
  #define BLDC_PI_KI                0.0f    // PI integral gain — tune after measurement

#elif (BLDC_VOLTAGE_PROFILE == 12)

  #define BLDC_NOMINAL_VOLTAGE      12.0f
  #define BLDC_VDC_MIN              10.5f   // undervoltage trip (~88%)
  #define BLDC_JOY_MAX_RPM          100.0f  // manual mode max RPM  — tune after Kv measurement
  #define BLDC_OC_LEVEL              3.0f   // overcurrent trip [A]  — no-load ~0.33A
  #define BLDC_OC_CLEAR_LEVEL        2.0f   // overcurrent hysteresis clear [A]
  #define BLDC_MOTOR_POLE_PAIRS      4      // 12V test motor
  #define BLDC_MAX_RPM_OPENLOOP     16000.0f // no-load max RPM — estimated
  #define BLDC_PI_KP                2.0f    // PI proportional gain — tuned 12V
  #define BLDC_PI_KI                2.0f    // PI integral gain — tuned 12V

#else
  #error "BLDC_VOLTAGE_PROFILE must be 12 or 36"
#endif

// Active motor count: 0 = no HW / 1 = M0 only / 2 = M0 + M1
#define BLDC_ACTIVE_MOTOR_COUNT     1
#define BLDC_MOTOR_COUNT            2       // total slots (fixed)

// Default rotation direction per motor
#define BLDC_M0_DEFAULT_DIR         BLDC_DIR_CW
#define BLDC_M1_DEFAULT_DIR         BLDC_DIR_CCW

// 0: fixed RPM / 1: joystick differential steering
#define BLDC_JOY_ENABLE             0

//--------------------------------------------------------------------------------
// HW FLAGS
//--------------------------------------------------------------------------------

#define BLDC_HW_CURRENT_SENSING     1       // 0: disabled / 1: shunt+opamp connected
#define BLDC_HW_NTC                 1       // 0: disabled / 1: NTC connected

//--------------------------------------------------------------------------------
// TUNING
//--------------------------------------------------------------------------------

// Manual mode fixed RPM (dipswitch ON, BLDC_JOY_ENABLE=0 only)
#define BLDC_NONCAN_FIXED_RPM       500.0f

// Open-loop ramp slope [CNT/ISR-tick] — 0→CNT_MAX in ~1.4s at 20kHz
#define BLDC_RAMP_SLOPE             0.15f

// LPF cutoff frequencies [Hz]
// Note: current LPF Fs=1kHz (1ms task) → actual fc ~25Hz regardless of value below
#define BLDC_LPF_FC_CURRENT         500.0f
#define BLDC_LPF_FC_SPEED           5.0f

// NTC polynomial: T = a3*V^3 + a2*V^2 + a1*V + a0
// Coefficients based on F767 reference — re-measure for NTCS0603E3103FLT
#define BLDC_NTC_A3                 (-11.489f)
#define BLDC_NTC_A2                 ( 63.236f)
#define BLDC_NTC_A1                 (-149.02f)
#define BLDC_NTC_A0                 ( 181.97f)

//--------------------------------------------------------------------------------
// PROTECTION THRESHOLDS
//--------------------------------------------------------------------------------

// Maximum PWM duty ratio [0.0 ~ 1.0]
// Limits CCR to reduce inductive voltage spike at MOSFET turn-off
// BSC014N06NS Vds=60V on 36V bus — spike headroom ≈ 24V
// Raise only after verifying no overvoltage with oscilloscope
#define BLDC_MAX_DUTY_RATIO         0.70f
#define BLDC_MAX_DUTY_CNT           ((uint32_t)((float)BLDC_CNT_MAX * BLDC_MAX_DUTY_RATIO))

// Overcurrent: trip after OC_LEVEL sustained for OC_TRIP_COUNT ISR ticks
#define BLDC_OC_TRIP_COUNT          200     // ticks @ 20kHz = 10ms

// Temperature [degC]
#define BLDC_TEMP_FAULT             100.0f  // overheat trip
#define BLDC_TEMP_RECOVER            90.0f  // overheat hysteresis clear

// BLDC_VDC_MIN, BLDC_OC_LEVEL, BLDC_OC_CLEAR_LEVEL defined in MOTOR Config above

//--------------------------------------------------------------------------------
// ADC CONSTANTS
//--------------------------------------------------------------------------------

// Current sensing (ADC1/2 for M0, ADC3/4 for M1)
#define BLDC_ADC_VREF               3.3f
#define BLDC_ADC_FS                 4095.0f
#define BLDC_OFFSET_VOLT            1.65f   // OPAMP output at zero current (Vref/2)
#define BLDC_OPAMP_GAIN             0.044f  // [V/A]
#define BLDC_CURRENT_CAL_CNT        10      // offset calibration sample count

// Bus voltage divider (ADC5)
// R_top=100kΩ, R_bot=8.2kΩ → ratio measured: 0.8113V / 11.5V
#define BLDC_VDIV_RATIO             0.07055f

//--------------------------------------------------------------------------------
// CHANNEL MAPPING
//--------------------------------------------------------------------------------

// Motor 0 — TIM8 (PWM) / TIM3 (Hall)
#define M0_TIM_CH_U                 TIM_CHANNEL_3
#define M0_TIM_CH_V                 TIM_CHANNEL_4
#define M0_TIM_CH_W                 TIM_CHANNEL_1

// Motor 1 — TIM1 (PWM) / TIM4 (Hall)
#define M1_TIM_CH_U                 TIM_CHANNEL_3
#define M1_TIM_CH_V                 TIM_CHANNEL_2
#define M1_TIM_CH_W                 TIM_CHANNEL_1

//--------------------------------------------------------------------------------
// SYSTEM CONSTANTS
//--------------------------------------------------------------------------------

// Clock & PWM
#define BLDC_SYSCLK_HZ              170000000UL // 170MHz
#define BLDC_PWM_FREQ_HZ            20000UL     // 20kHz center-aligned
#define BLDC_CNT_MAX                4249UL      // ARR = 170M / 20k / 2 - 1
#define BLDC_HALL_TIM_CLK_HZ        10000000UL  // TIM3/4: 170MHz / (PSC+1=17) ≈ 10MHz
#define BLDC_TSAMP                  0.00005f    // 50us (1/20kHz)

// Derived
#define BLDC_HALL_EDGES_PER_REV     (BLDC_MOTOR_POLE_PAIRS * 6.0f) // pole-pairs × 6 edges

// Operating mode
#define BLDC_MODE_CAN               1U
#define BLDC_MODE_MANUAL            0U
extern uint8_t g_nOperMode;

// Direction
#define BLDC_DIR_CW                 1
#define BLDC_DIR_CCW                0

// Fault codes
#define BLDC_FLT_NONE               0
#define BLDC_FLT_OVERCURRENT        1
#define BLDC_FLT_OVERHEAT           2
#define BLDC_FLT_UNDERVOLT          3
#define BLDC_FLT_BKIN               4   // hardware break input triggered
#define BLDC_FLT_HALL               5   // invalid Hall state at init (outside 1~6)

// Math
#ifndef PI
#define PI                          3.14159f
#endif

//--------------------------------------------------------------------------------
// JOYSTICK — Manual mode
//--------------------------------------------------------------------------------

// BLDC_JOY_MAX_RPM defined in MOTOR Config above
#define BLDC_JOY_DEADBAND           0.05f
// zone threshold — must exceed max noise amplitude
#define BLDC_JOY_ZONE_THR           0.5f

// per-zone RPM
#define BLDC_JOY_FWD_RPM            150.0f  // forward
#define BLDC_JOY_TURN_INNER_RPM      30.0f  // inner wheel during turn
#define BLDC_JOY_TURN_OUTER_RPM     100.0f  // outer wheel during turn

// axis inversion: 1=invert / 0=normal — joystick mounted upside-down
#define BLDC_JOY_INVERT_Y           1
#define BLDC_JOY_INVERT_X           1

// joystick neutral position raw ADC (measured: ~2900)
#define BLDC_JOY_ADC_CENTER         2900U



#endif
