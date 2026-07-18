#ifndef INC_BLDC_CAN_H_
#define INC_BLDC_CAN_H_

#include "stm32g4xx_hal.h"
#include "bldc_config.h"


//  RX Frame IDs  (ROS2 → BLDC)
#define CAN_ID_CMD              0x200U

//  TX Frame IDs  (BLDC → ROS2)
#define CAN_ID_STATUS_M0        0x210U
#define CAN_ID_STATUS_M1        0x211U

//--------------------------------------------------------------------------------
//  CAN_ID_CMD  (DLC = 8)
//
//  Byte[0:1]  M0 RPM ref   int16 LE  LSB = 1 RPM  (-3000 ~ +3000, negative = reverse)
//  Byte[2:3]  M1 RPM ref   int16 LE  LSB = 1 RPM  (-3000 ~ +3000, negative = reverse)
//  Byte[4]    Control mode  uint8     0=IDLE / 1=RUN / 2=FREE / 3=EMERGENCY
//  Byte[5]    Fault clear   uint8     bit0=M0 / bit1=M1 / bit2=all
//  Byte[6:7]  reserved
//--------------------------------------------------------------------------------
#define CAN_CMD_RPM_LSB         1.0f    /* RPM/LSB */

#define CAN_CMD_MODE_IDLE       0U
#define CAN_CMD_MODE_RUN        1U
#define CAN_CMD_MODE_FREE       2U
#define CAN_CMD_MODE_EMERGENCY  3U

#define CAN_FLTCLR_M0          (1U << 0)
#define CAN_FLTCLR_M1          (1U << 1)
#define CAN_FLTCLR_ALL         (1U << 2)

//--------------------------------------------------------------------------------
//  CAN_ID_STATUS_M0/M1  (DLC = 8)
//
//  Byte[0:1]  Actual RPM    int16 LE  LSB = 1 RPM    (-3000 ~ +3000, negative = reverse)
//  Byte[2:3]  Max current   int16 LE  LSB = 0.01 A   (max(|Iu|,|Iv|,|Iw|) LPF)
//  Byte[4]    Temperature   int8      LSB = 1 °C     (clamp 0~100)
//  Byte[5]    Fault code    uint8     BLDC_FLT_xxx   (0=none ~ 5=hall)
//  Byte[6]    Status flags  uint8     bit0=enabled  bit1=PI  bit2=spinning
//  Byte[7]    reserved      0x00
//--------------------------------------------------------------------------------
#define CAN_STATUS_RPM_LSB      1.0f    /* RPM/LSB */
#define CAN_STATUS_CURR_LSB     0.01f   /* A/LSB */
#define CAN_STATUS_TEMP_LSB     1.0f    /* °C/LSB */

#define CAN_STATUS_FL_ENABLED   (1U << 0)
#define CAN_STATUS_FL_PI        (1U << 1)
#define CAN_STATUS_FL_SPINNING  (1U << 2)



void CAN_Init(FDCAN_HandleTypeDef *phFdcan);
void CAN_Task100ms(void);



#endif
