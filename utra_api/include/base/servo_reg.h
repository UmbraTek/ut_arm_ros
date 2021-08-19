/* Copyright 2020 UmbraTek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#ifndef __SERVO_REG_H__
#define __SERVO_REG_H__

#include <stdint.h>

class SERVO_REG {
 public:
  SERVO_REG(void) {}
  const uint8_t UUID[5] = {0x01, 0, 12, ' ', ' '};
  const uint8_t SW_VERSION[5] = {0x02, 0, 12, ' ', ' '};
  const uint8_t HW_VERSION[5] = {0x03, 0, 12, ' ', ' '};
  const uint8_t MULTI_VERSION[5] = {0x04, 0, 12, ' ', ' '};
  const uint8_t MECH_RATIO[5] = {0x05, 0, 4, 4, 0};
  const uint8_t COM_ID[5] = {0x08, ' ', ' ', 1, 0};
  const uint8_t COM_BAUD[5] = {0x09, ' ', ' ', 4, 0};
  const uint8_t RESET_ERR[5] = {0x0C, ' ', ' ', 1, 0};
  const uint8_t RESET_DRIVER[5] = {0x0D, ' ', ' ', 1, 0};
  const uint8_t ERASE_PARM[5] = {0x0E, ' ', ' ', 1, 0};
  const uint8_t SAVED_PARM[5] = {0x0F, ' ', ' ', 1, 0};

  const uint8_t ELEC_RATIO[5] = {0x10, 0, 4, 4, 0};
  const uint8_t MOTION_DIR[5] = {0x11, 0, 1, 1, 0};
  const uint8_t TEMP_LIMIT[5] = {0x18, 0, 2, 2, 0};
  const uint8_t VOLT_LIMIT[5] = {0x19, 0, 2, 2, 0};
  const uint8_t CURR_LIMIT[5] = {0x1A, 0, 4, 4, 0};
  const uint8_t BRAKE_PWM[5] = {0x1F, 0, 1, 1, 0};

  const uint8_t MOTION_MODE[5] = {0x20, 0, 1, 1, 0};
  const uint8_t MOTION_ENABLE[5] = {0x21, 0, 1, 1, 0};
  const uint8_t BRAKE_ENABLE[5] = {0x22, 0, 1, 1, 0};
  const uint8_t TEMP_DRIVER[5] = {0x28, 0, 4, ' ', ' '};
  const uint8_t TEMP_MOTO[5] = {0x29, 0, 4, ' ', ' '};
  const uint8_t BUS_VOLT[5] = {0x2A, 0, 4, ' ', ' '};
  const uint8_t BUS_CURR[5] = {0x2B, 0, 4, ' ', ' '};
  const uint8_t MULTI_VOLT[5] = {0x2C, 0, 4, ' ', ' '};
  const uint8_t ERROR_CODE[5] = {0x2F, 0, 1, 0, 0};

  const uint8_t POS_TARGET[5] = {0x30, 0, 4, 4, 0};
  const uint8_t POS_CURRENT[5] = {0x31, 0, 4, 0, 0};
  const uint8_t POS_LIMIT_MIN[5] = {0x32, 0, 4, 4, 0};
  const uint8_t POS_LIMIT_MAX[5] = {0x33, 0, 4, 4, 0};
  const uint8_t POS_LIMIT_DIFF[5] = {0x34, 0, 4, 4, 0};
  const uint8_t POS_PIDP[5] = {0x35, 0, 4, 4, 0};
  const uint8_t POS_SMOOTH_CYC[5] = {0x36, 0, 1, 1, 0};
  const uint8_t POS_CAL_ZERO[5] = {0x3F, ' ', ' ', 1, 0};

  const uint8_t VEL_TARGET[5] = {0x40, 0, 4, 4, 0};
  const uint8_t VEL_CURRENT[5] = {0x41, 0, 4, 0, 0};
  const uint8_t VEL_LIMIT_MIN[5] = {0x42, 0, 4, 4, 0};
  const uint8_t VEL_LIMIT_MAX[5] = {0x43, 0, 4, 4, 0};
  const uint8_t VEL_LIMIT_DIFF[5] = {0x44, 0, 4, 4, 0};
  const uint8_t VEL_PIDP[5] = {0x45, 0, 4, 4, 0};
  const uint8_t VEL_PIDI[5] = {0x46, 0, 4, 4, 0};
  const uint8_t VEL_SMOOTH_CYC[5] = {0x47, 0, 1, 1, 0};

  const uint8_t TAU_TARGET[5] = {0x50, 0, 4, 4, 0};
  const uint8_t TAU_CURRENT[5] = {0x51, 0, 4, 0, 0};
  const uint8_t TAU_LIMIT_MIN[5] = {0x52, 0, 4, 4, 0};
  const uint8_t TAU_LIMIT_MAX[5] = {0x53, 0, 4, 4, 0};
  const uint8_t TAU_LIMIT_DIFF[5] = {0x54, 0, 4, 4, 0};
  const uint8_t TAU_PIDP[5] = {0x55, 0, 4, 4, 0};
  const uint8_t TAU_PIDI[5] = {0x56, 0, 4, 4, 0};
  const uint8_t TAU_SMOOTH_CYC[5] = {0x57, 0, 1, 1, 0};

  uint8_t CPOS_TARGET[5] = {0x60, ' ', ' ', 0, ' '};
  const uint8_t SPOSTAU_CURRENT[5] = {0x68, 0, 8 + 1, ' ', ' '};
  const uint8_t CPOSTAU_CURRENT[5] = {0x69, 2, 8 + 1, ' ', ' '};
};

#endif
