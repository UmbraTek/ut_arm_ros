/* Copyright 2021 UmbraTek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#include "utra/utra_flxie_api.h"
#include "base/arm_reg.h"

UtraFlxiE2Api::UtraFlxiE2Api(ArmApiBase *utra_api, uint8_t id) {
  utra_api_ = utra_api;
  id_ = id;
  line_ = RS485_LINE::TGPIO;
}

UtraFlxiE2Api::~UtraFlxiE2Api(void) {}

/************************************************************
 *                     Basic Api
 ************************************************************/
int UtraFlxiE2Api::get_uuid(char uuid[24]) {
  uint8_t temp[12];
  int ret = utra_api_->get_utrc_int8n_now(line_, id_, reg_.UUID[0], temp, reg_.UUID[2]);
  for (int i = 0; i < 12; ++i) sprintf(&uuid[i * 2], "%02x", temp[i]);
  return ret;
}

int UtraFlxiE2Api::get_sw_version(char version[12]) {
  uint8_t temp[12];
  int ret = utra_api_->get_utrc_int8n_now(line_, id_, reg_.SW_VERSION[0], temp, reg_.SW_VERSION[2]);
  for (int i = 0; i < 12; ++i) sprintf(&version[i], "%c", temp[i]);
  return ret;
}

int UtraFlxiE2Api::get_hw_version(char version[24]) {
  uint8_t temp[12];
  int ret = utra_api_->get_utrc_int8n_now(line_, id_, reg_.HW_VERSION[0], temp, reg_.HW_VERSION[2]);
  for (int i = 0; i < 12; ++i) sprintf(&version[i * 2], "%02x", temp[i]);
  return ret;
}

int UtraFlxiE2Api::reset_err(void) {
  return utra_api_->set_utrc_int8_now(line_, id_, reg_.RESET_ERR[0], reg_.RESET_ERR[0]);  //
}

int UtraFlxiE2Api::restart_driver(void) {
  return utra_api_->set_utrc_int8_now(line_, id_, reg_.RESET_DRIVER[0], reg_.RESET_DRIVER[0]);
}

int UtraFlxiE2Api::erase_parm(void) {
  return utra_api_->set_utrc_int8_now(line_, id_, reg_.ERASE_PARM[0], reg_.ERASE_PARM[0]);  //
}

int UtraFlxiE2Api::saved_parm(void) {
  return utra_api_->set_utrc_int8_now(line_, id_, reg_.SAVED_PARM[0], reg_.SAVED_PARM[0]);  //
}

/************************************************************
 *                     State Api
 ************************************************************/
int UtraFlxiE2Api::get_temp_limit(int *min, int *max) {
  uint8_t rx_data[2];
  int ret = utra_api_->get_utrc_int8n_now(line_, id_, reg_.TEMP_LIMIT[0], rx_data, reg_.TEMP_LIMIT[2]);
  *min = (int8_t)rx_data[0];
  *max = (int8_t)rx_data[1];
  return ret;
}

int UtraFlxiE2Api::set_temp_limit(uint8_t min, uint8_t max, bool now) {
  uint8_t tx_data[2] = {min, max};

  if (now)
    return utra_api_->set_utrc_int8n_now(line_, id_, reg_.TEMP_LIMIT[0], tx_data, reg_.TEMP_LIMIT[3]);
  else
    return utra_api_->set_utrc_int8n_que(line_, id_, reg_.TEMP_LIMIT[0], tx_data, reg_.TEMP_LIMIT[3]);
}

int UtraFlxiE2Api::get_volt_limit(int *min, int *max) {
  uint8_t rx_data[2];
  int ret = utra_api_->get_utrc_int8n_now(line_, id_, reg_.VOLT_LIMIT[0], rx_data, reg_.VOLT_LIMIT[2]);
  *min = (int8_t)rx_data[0];
  *max = (int8_t)rx_data[1];
  return ret;
}

int UtraFlxiE2Api::set_volt_limit(uint8_t min, uint8_t max, bool now) {
  uint8_t tx_data[2] = {min, max};
  if (now)
    return utra_api_->set_utrc_int8n_now(line_, id_, reg_.VOLT_LIMIT[0], tx_data, reg_.VOLT_LIMIT[3]);
  else
    return utra_api_->set_utrc_int8n_que(line_, id_, reg_.VOLT_LIMIT[0], tx_data, reg_.VOLT_LIMIT[3]);
}

int UtraFlxiE2Api::get_curr_limit(float *value) {
  return utra_api_->get_utrc_float_now(line_, id_, reg_.CURR_LIMIT[0], value);  //
}

int UtraFlxiE2Api::set_curr_limit(float value, bool now) {
  if (now)
    return utra_api_->set_utrc_float_now(line_, id_, value, reg_.CURR_LIMIT[0]);
  else
    return utra_api_->set_utrc_float_que(line_, id_, value, reg_.CURR_LIMIT[0]);
}

/************************************************************
 *                     Control Api
 ************************************************************/
int UtraFlxiE2Api::set_motion_mode(uint8_t value, bool now) {
  if (now)
    return utra_api_->set_utrc_int8_now(line_, id_, reg_.MOTION_MODE[0], value);
  else
    return utra_api_->set_utrc_int8_que(line_, id_, reg_.MOTION_MODE[0], value);
}

int UtraFlxiE2Api::get_motion_mode(uint8_t *value) {
  return utra_api_->get_utrc_int8_now(line_, id_, reg_.MOTION_MODE[0], value);
}

int UtraFlxiE2Api::set_motion_enable(uint8_t value, bool now) {
  if (now)
    return utra_api_->set_utrc_int8_now(line_, id_, reg_.MOTION_ENABLE[0], value);
  else
    return utra_api_->set_utrc_int8_que(line_, id_, reg_.MOTION_ENABLE[0], value);
}

int UtraFlxiE2Api::get_motion_enable(uint8_t *value) {
  return utra_api_->get_utrc_int8_now(line_, id_, reg_.MOTION_ENABLE[0], value);
}

int UtraFlxiE2Api::set_unlock_function(uint8_t value) {
  return utra_api_->set_utrc_int8_now(line_, id_, flxie_reg_.UNLOCK_FUN[0], value);
}

int UtraFlxiE2Api::get_temp_motor(float *value) {
  return utra_api_->get_utrc_float_now(line_, id_, reg_.TEMP_MOTOR[0], value);  //
}

int UtraFlxiE2Api::get_temp_driver(float *value) {
  return utra_api_->get_utrc_float_now(line_, id_, reg_.TEMP_DRIVER[0], value);
}

int UtraFlxiE2Api::get_bus_volt(float *value) {
  return utra_api_->get_utrc_float_now(line_, id_, reg_.BUS_VOLT[0], value);  //
}

int UtraFlxiE2Api::get_bus_curr(float *value) {
  return utra_api_->get_utrc_float_now(line_, id_, reg_.BUS_CURR[0], value);  //
}

int UtraFlxiE2Api::get_error_code(uint8_t *value) {
  return utra_api_->get_utrc_int8_now(line_, id_, reg_.ERROR_CODE[0], value);
}

/************************************************************
 *                    Position Api
 ************************************************************/
int UtraFlxiE2Api::get_pos_target(float *value) {
  return utra_api_->get_utrc_float_now(line_, id_, reg_.POS_TARGET[0], value);  //
}

int UtraFlxiE2Api::set_pos_target(float value, bool now) {
  if (now)
    return utra_api_->set_utrc_float_now(line_, id_, reg_.POS_TARGET[0], value);
  else
    return utra_api_->set_utrc_float_que(line_, id_, reg_.POS_TARGET[0], value);
}

int UtraFlxiE2Api::get_pos_current(float *value) {
  return utra_api_->get_utrc_float_now(line_, id_, reg_.POS_CURRENT[0], value);
}

int UtraFlxiE2Api::get_pos_pidp(float *value) {
  return utra_api_->get_utrc_float_now(line_, id_, reg_.POS_PIDP[0], value);  //
}

int UtraFlxiE2Api::set_pos_pidp(float value, bool now) {
  if (now)
    return utra_api_->set_utrc_float_now(line_, id_, reg_.POS_PIDP[0], value);
  else
    return utra_api_->set_utrc_float_que(line_, id_, reg_.POS_PIDP[0], value);
}

int UtraFlxiE2Api::get_pos_smooth_cyc(uint8_t *value) {
  return utra_api_->get_utrc_int8_now(line_, id_, reg_.POS_SMOOTH_CYC[0], value);
}

int UtraFlxiE2Api::set_pos_smooth_cyc(uint8_t value, bool now) {
  if (now)
    return utra_api_->set_utrc_int8_now(line_, id_, reg_.POS_SMOOTH_CYC[0], value);
  else
    return utra_api_->set_utrc_int8_que(line_, id_, reg_.POS_SMOOTH_CYC[0], value);
}

int UtraFlxiE2Api::get_pos_adrc_param(int i, float *value) {
  return utra_api_->get_utrc_u8float_now(line_, id_, reg_.POS_ADRC_PARAM[0], i, value);  //
}

int UtraFlxiE2Api::set_pos_adrc_param(int i, float value) {
  return utra_api_->set_utrc_u8float_now(line_, id_, reg_.POS_ADRC_PARAM[0], i, value);
}

int UtraFlxiE2Api::pos_cal_zero(void) {
  return utra_api_->set_utrc_int8_now(line_, id_, reg_.POS_CAL_ZERO[0], reg_.POS_CAL_ZERO[0]);
}

/************************************************************
 *                    Speed Api
 ************************************************************/
int UtraFlxiE2Api::get_vel_limit_min(float *value) {
  return utra_api_->get_utrc_float_now(line_, id_, reg_.VEL_LIMIT_MIN[0], value);
}

int UtraFlxiE2Api::set_vel_limit_min(float value, bool now) {
  if (now)
    return utra_api_->set_utrc_float_now(line_, id_, reg_.VEL_LIMIT_MIN[0], value);
  else
    return utra_api_->set_utrc_float_que(line_, id_, reg_.VEL_LIMIT_MIN[0], value);
}

int UtraFlxiE2Api::get_vel_limit_max(float *value) {
  return utra_api_->get_utrc_float_now(line_, id_, reg_.VEL_LIMIT_MAX[0], value);
}

int UtraFlxiE2Api::set_vel_limit_max(float value, bool now) {
  if (now)
    return utra_api_->set_utrc_float_now(line_, id_, reg_.VEL_LIMIT_MAX[0], value);
  else
    return utra_api_->set_utrc_float_que(line_, id_, reg_.VEL_LIMIT_MAX[0], value);
}

/************************************************************
 *                    torque Api
 ************************************************************/

int UtraFlxiE2Api::get_tau_target(float *value) {
  return utra_api_->get_utrc_float_now(line_, id_, reg_.TAU_TARGET[0], value);  //
}

int UtraFlxiE2Api::set_tau_target(float value, bool now) {
  if (now)
    return utra_api_->set_utrc_float_now(line_, id_, reg_.TAU_TARGET[0], value);
  else
    return utra_api_->set_utrc_float_que(line_, id_, reg_.TAU_TARGET[0], value);
}

int UtraFlxiE2Api::get_tau_current(float *value) {
  return utra_api_->get_utrc_float_now(line_, id_, reg_.TAU_CURRENT[0], value);
}

int UtraFlxiE2Api::get_tau_limit_min(float *value) {
  return utra_api_->get_utrc_float_now(line_, id_, reg_.TAU_LIMIT_MIN[0], value);
}

int UtraFlxiE2Api::set_tau_limit_min(float value, bool now) {
  if (now)
    return utra_api_->set_utrc_float_now(line_, id_, reg_.TAU_LIMIT_MIN[0], value);
  else
    return utra_api_->set_utrc_float_que(line_, id_, reg_.TAU_LIMIT_MIN[0], value);
}

int UtraFlxiE2Api::get_tau_limit_max(float *value) {
  return utra_api_->get_utrc_float_now(line_, id_, reg_.TAU_LIMIT_MAX[0], value);
}

int UtraFlxiE2Api::set_tau_limit_max(float value, bool now) {
  if (now)
    return utra_api_->set_utrc_float_now(line_, id_, reg_.TAU_LIMIT_MAX[0], value);
  else
    return utra_api_->set_utrc_float_que(line_, id_, reg_.TAU_LIMIT_MAX[0], value);
}

int UtraFlxiE2Api::get_tau_pidp(float *value) {
  return utra_api_->get_utrc_float_now(line_, id_, reg_.TAU_PIDP[0], value);  //
}

int UtraFlxiE2Api::set_tau_pidp(float value, bool now) {
  if (now)
    return utra_api_->set_utrc_float_now(line_, id_, reg_.TAU_PIDP[0], value);
  else
    return utra_api_->set_utrc_float_que(line_, id_, reg_.TAU_PIDP[0], value);
}

int UtraFlxiE2Api::get_tau_pidi(float *value) {
  return utra_api_->get_utrc_float_now(line_, id_, reg_.TAU_PIDI[0], value);  //
}

int UtraFlxiE2Api::set_tau_pidi(float value, bool now) {
  if (now)
    return utra_api_->set_utrc_float_now(line_, id_, reg_.TAU_PIDI[0], value);
  else
    return utra_api_->set_utrc_float_que(line_, id_, reg_.TAU_PIDI[0], value);
}

int UtraFlxiE2Api::get_tau_adrc_param(int i, float *value) {
  return utra_api_->get_utrc_u8float_now(line_, id_, reg_.TAU_ADRC_PARAM[0], i, value);  //
}

int UtraFlxiE2Api::set_tau_adrc_param(int i, float value) {
  return utra_api_->set_utrc_u8float_now(line_, id_, reg_.TAU_ADRC_PARAM[0], i, value);
}

int UtraFlxiE2Api::get_tau_smooth_cyc(uint8_t *value) {
  return utra_api_->get_utrc_int8_now(line_, id_, reg_.TAU_SMOOTH_CYC[0], value);
}

int UtraFlxiE2Api::set_tau_smooth_cyc(uint8_t value, bool now) {
  if (now)
    return utra_api_->set_utrc_int8_now(line_, id_, reg_.TAU_SMOOTH_CYC[0], value);
  else
    return utra_api_->set_utrc_int8_que(line_, id_, reg_.TAU_SMOOTH_CYC[0], value);
}

/************************************************************
 *                      Senser Api
 ************************************************************/
int UtraFlxiE2Api::get_senser(float *value) {
  return utra_api_->get_utrc_nfloat_now(line_, id_, flxie_reg_.SENSER1[0], 4, value);  //
}