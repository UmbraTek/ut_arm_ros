/* Copyright 2020 UmbraTek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#ifndef __ARM_REG_H__
#define __ARM_REG_H__

#include <stdint.h>

class RS485_LINE {
 public:
  static const uint8_t SERVO = 92;
  static const uint8_t TGPIO = 2;
  static const uint8_t CGPIO = 3;
};

class ARM_REG {
 public:
  ARM_REG(uint8_t axis) : AXIS(axis) {}

  const uint8_t AXIS = 0;

  const uint8_t UUID[5] = {0x01, 0, 17, ' ', ' '};
  const uint8_t SW_VERSION[5] = {0x02, 0, 20, ' ', ' '};
  const uint8_t HW_VERSION[5] = {0x03, 0, 20, ' ', ' '};
  const uint8_t UBOT_AXIS[5] = {0x04, 0, 1, ' ', ' '};
  const uint8_t SYS_SHUTDOWN[5] = {0x0B, ' ', ' ', 1, 0};
  const uint8_t RESET_ERR[5] = {0x0C, ' ', ' ', 1, 0};
  const uint8_t SYS_REBOOT[5] = {0x0D, ' ', ' ', 1, 0};
  const uint8_t ERASE_PARM[5] = {0x0E, ' ', ' ', 1, 0};
  const uint8_t SAVED_PARM[5] = {0x0F, ' ', ' ', 1, 0};

  const uint8_t MOTION_MDOE[5] = {0x20, 0, 1, 1, 0};
  const uint8_t MOTION_ENABLE[5] = {0x21, 0, 4, 2, 0};
  const uint8_t BRAKE_ENABLE[5] = {0x22, 0, 4, 2, 0};
  const uint8_t ERROR_CODE[5] = {0x23, 0, 2, ' ', ' '};
  const uint8_t SERVO_MSG[5] = {0x24, 0, (uint8_t)(AXIS * 2), ' ', ' '};
  const uint8_t MOTION_STATUS[5] = {0x25, 0, 1, 1, 0};
  const uint8_t CMD_NUM[5] = {0x26, 0, 4, 4, 0};

  const uint8_t MOVET_LINE[5] = {0x30, ' ', ' ', 36, 4};
  const uint8_t MOVET_LINEB[5] = {0x31, ' ', ' ', 40, 4};
  const uint8_t MOVET_CIRCLE[5] = {0x32, ' ', ' ', 64, 4};
  const uint8_t MOVET_P2P[5] = {0x33, ' ', ' ', ' ', ' '};
  const uint8_t MOVET_P2PB[5] = {0x34, ' ', ' ', ' ', ' '};
  const uint8_t MOVEJ_LINE[5] = {0x35, ' ', ' ', ' ', ' '};
  const uint8_t MOVEJ_LINEB[5] = {0x36, ' ', ' ', ' ', ' '};
  const uint8_t MOVEJ_CIRCLE[5] = {0x37, ' ', ' ', ' ', ' '};
  const uint8_t MOVEJ_P2P[5] = {0x38, ' ', ' ', (uint8_t)((AXIS + 3) * 4), 4};
  const uint8_t MoveJ_P2PB[5] = {0x39, ' ', ' ', ' ', ' '};
  const uint8_t MOVEJ_HOME[5] = {0x3A, ' ', ' ', 12, 4};
  const uint8_t MOVE_SLEEP[5] = {0x3B, ' ', ' ', 4, 4};
  const uint8_t MOVE_SERVOJ[5] = {0x3C, ' ', ' ', (uint8_t)((AXIS + 3) * 4), 4};
  uint8_t MOVES_JOINT[5] = {0x3D, ' ', ' ', 0x55, 4};
  const uint8_t PLAN_SLEEP[5] = {0x3F, ' ', ' ', 4, 4};

  const uint8_t TCP_JERK[5] = {0x40, 0, 4, 4, 4};
  const uint8_t TCP_MAXACC[5] = {0x41, 0, 4, 4, 4};
  const uint8_t JOINT_JERK[5] = {0x42, 0, 4, 4, 4};
  const uint8_t JOINT_MAXACC[5] = {0x43, 0, 4, 4, 4};
  const uint8_t TCP_OFFSET[5] = {0x44, 0, 24, 24, 0};
  const uint8_t LOAD_PARAM[5] = {0x45, 0, 16, 16, 0};
  const uint8_t GRAVITY_DIR[5] = {0x46, 0, 12, 12, 0};
  const uint8_t COLLIS_SENS[5] = {0x47, 0, 1, 1, 0};
  const uint8_t TEACH_SENS[5] = {0x48, 0, 1, 1, 0};

  const uint8_t TCP_POS_CURR[5] = {0x50, 0, 24, ' ', ' '};
  const uint8_t JOINT_POS_CURR[5] = {0x51, 0, (uint8_t)(AXIS * 4), ' ', ' '};
  const uint8_t CAL_IK[5] = {0x52, 24, (uint8_t)(AXIS * 4), ' ', ' '};
  const uint8_t CAL_FK[5] = {0x53, (uint8_t)(AXIS * 4), 24, ' ', ' '};
  const uint8_t IS_JOINT_LIMIT[5] = {0x54, (uint8_t)(AXIS * 4), 1, ' ', ' '};
  const uint8_t IS_TCP_LIMIT[5] = {0x55, 24, 1, ' ', ' '};

  const uint8_t UTRC_INT8_NOW[5] = {0x60, 3, 2, 4, 1};
  const uint8_t UTRC_INT32_NOW[5] = {0x61, 3, 8, 7, 1};
  const uint8_t UTRC_FP32_NOW[5] = {0x62, 3, 8, 7, 1};
  uint8_t UTRC_INT8N_NOW[5] = {0x63, 4, 0x55, 0x55, 1};

  const uint8_t UTRC_INT8_QUE[5] = {0x64, ' ', ' ', 4, 0};
  const uint8_t UTRC_INT32_QUE[5] = {0x65, ' ', ' ', 7, 0};
  const uint8_t UTRC_FP32_QUE[5] = {0x66, ' ', ' ', 7, 0};
  uint8_t UTRC_INT8N_QUE[5] = {0x67, ' ', ' ', 0x55, 0};

  uint8_t PASS_RS485_NOW[5] = {0x68, ' ', ' ', 0x55, 0x55};
  uint8_t PASS_RS485_QUE[5] = {0x69, ' ', ' ', 0x55, 0};

  const uint8_t UTRC_U8FP32_NOW[5] = {0x6A, 4, 8, 8, 1};
  uint8_t UTRC_FP32N_NOW[5] = {0x6B, 4, 0x55, 0x55, 1};
  const uint8_t GPIO_IN[5] = {0x6E, 2, 0x55, ' ', ' '};
  const uint8_t GPIO_OU[5] = {0x6F, 2, 0x55, ' ', ' '};

  const uint8_t FRICTION[5] = {0x70, 2, 16, 2 + 16, 0};
};

#endif
