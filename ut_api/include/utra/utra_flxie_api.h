/* Copyright 2021 UmbraTek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#ifndef __UTRA_FLXIE_API_H__
#define __UTRA_FLXIE_API_H__

#include "base/arm_api_base.h"
#include "base/flxie_reg.h"
#include "base/servo_reg.h"
#include "common/socket_tcp.h"

class UtraFlxiE2Api {
 public:
  UtraFlxiE2Api(ArmApiBase *utra_api, uint8_t id = 101);
  ~UtraFlxiE2Api(void);

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
  int get_curr_limit(float *value);
  int set_curr_limit(float value, bool now = true);

  int set_motion_mode(uint8_t value, bool now = true);
  int get_motion_mode(uint8_t *value);
  int set_motion_enable(uint8_t value, bool now = true);
  int get_motion_enable(uint8_t *value);
  int set_unlock_function(uint8_t value);
  int get_temp_motor(float *value);
  int get_temp_driver(float *value);
  int get_bus_volt(float *value);
  int get_bus_curr(float *value);
  int get_error_code(uint8_t *value);

  int get_pos_target(float *value);
  int set_pos_target(float value, bool now = true);
  int get_pos_current(float *value);
  int get_pos_pidp(float *value);
  int set_pos_pidp(float value, bool now = true);
  int get_pos_smooth_cyc(uint8_t *value);
  int set_pos_smooth_cyc(uint8_t value, bool now = true);
  int get_pos_adrc_param(int i, float *value);
  int set_pos_adrc_param(int i, float value);
  int pos_cal_zero(void);

  int get_vel_limit_min(float *value);
  int set_vel_limit_min(float value, bool now = true);
  int get_vel_limit_max(float *value);
  int set_vel_limit_max(float value, bool now = true);

  int get_tau_target(float *value);
  int set_tau_target(float value, bool now = true);
  int get_tau_current(float *value);
  int get_tau_limit_min(float *value);
  int set_tau_limit_min(float value, bool now = true);
  int get_tau_limit_max(float *value);
  int set_tau_limit_max(float value, bool now = true);
  int get_tau_pidp(float *value);
  int set_tau_pidp(float value, bool now = true);
  int get_tau_pidi(float *value);
  int set_tau_pidi(float value, bool now = true);
  int get_tau_adrc_param(int i, float *value);
  int set_tau_adrc_param(int i, float value);
  int get_tau_smooth_cyc(uint8_t *value);
  int set_tau_smooth_cyc(uint8_t value, bool now = true);

  int get_senser(float *value);

 private:
  ArmApiBase *utra_api_;
  uint8_t id_;
  uint8_t line_;
  SERVO_REG reg_;
  FLXIE_REG flxie_reg_;
};

#endif
