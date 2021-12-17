/* Copyright 2021 UmbraTek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#ifndef __UTRA_FLXIV_API_H__
#define __UTRA_FLXIV_API_H__

#include "base/arm_api_base.h"
#include "base/flxiv_reg.h"
#include "base/servo_reg.h"
#include "common/socket_tcp.h"

class UtraFlxiVApi {
 public:
  UtraFlxiVApi(ArmApiBase *utra_api, uint8_t id = 101);
  ~UtraFlxiVApi(void);

  int get_uuid(char uuid[24]);
  int get_sw_version(char version[12]);
  int get_hw_version(char version[24]);
  int reset_err(void);
  int restart_driver(void);
  int erase_parm(void);
  int saved_parm(void);

  int get_temp_limit(int *min, int *max);
  int set_temp_limit(uint8_t min, uint8_t max, bool now = true);
  int get_volt_limit(int *min, int *max);
  int set_volt_limit(uint8_t min, uint8_t max, bool now = true);

  int set_motion_mode(uint8_t value, bool now = true);
  int get_motion_mode(uint8_t *value);
  int set_motion_enable(uint8_t value, bool now = true);
  int get_motion_enable(uint8_t *value);
  int get_temp_motor(float *value);
  int get_temp_driver(float *value);
  int get_bus_volt(float *value);
  int get_error_code(uint8_t *value);

  int get_senser(float *value);

 private:
  ArmApiBase *utra_api_;
  uint8_t id_;
  uint8_t line_;
  SERVO_REG reg_;
  FLXIV_REG flxiv_reg_;
};

#endif
