/* Copyright 2021 UmbraTek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#include "utra/utra_flxiv_api.h"
#include "base/arm_reg.h"

UtraFlxiVApi::UtraFlxiVApi(ArmApiBase *utra_api, uint8_t id) {
  utra_api_ = utra_api;
  id_ = id;
  line_ = RS485_LINE::TGPIO;
}

UtraFlxiVApi::~UtraFlxiVApi(void) {}

/************************************************************
 *                     Basic Api
 ************************************************************/
int UtraFlxiVApi::get_uuid(char uuid[24]) {
  uint8_t temp[12];
  int ret = utra_api_->get_utrc_int8n_now(line_, id_, reg_.UUID[0], temp, reg_.UUID[2]);
  for (int i = 0; i < 12; ++i) sprintf(&uuid[i * 2], "%02x", temp[i]);
  return ret;
}

int UtraFlxiVApi::get_sw_version(char version[12]) {
  uint8_t temp[12];
  int ret = utra_api_->get_utrc_int8n_now(line_, id_, reg_.SW_VERSION[0], temp, reg_.SW_VERSION[2]);
  for (int i = 0; i < 12; ++i) sprintf(&version[i], "%c", temp[i]);
  return ret;
}

int UtraFlxiVApi::get_hw_version(char version[24]) {
  uint8_t temp[12];
  int ret = utra_api_->get_utrc_int8n_now(line_, id_, reg_.HW_VERSION[0], temp, reg_.HW_VERSION[2]);
  for (int i = 0; i < 12; ++i) sprintf(&version[i * 2], "%02x", temp[i]);
  return ret;
}

int UtraFlxiVApi::reset_err(void) {
  return utra_api_->set_utrc_int8_now(line_, id_, reg_.RESET_ERR[0], reg_.RESET_ERR[0]);  //
}

int UtraFlxiVApi::restart_driver(void) {
  return utra_api_->set_utrc_int8_now(line_, id_, reg_.RESET_DRIVER[0], reg_.RESET_DRIVER[0]);
}

int UtraFlxiVApi::erase_parm(void) {
  return utra_api_->set_utrc_int8_now(line_, id_, reg_.ERASE_PARM[0], reg_.ERASE_PARM[0]);  //
}

int UtraFlxiVApi::saved_parm(void) {
  return utra_api_->set_utrc_int8_now(line_, id_, reg_.SAVED_PARM[0], reg_.SAVED_PARM[0]);  //
}

/************************************************************
 *                     State Api
 ************************************************************/
int UtraFlxiVApi::get_temp_limit(int *min, int *max) {
  uint8_t rx_data[2];
  int ret = utra_api_->get_utrc_int8n_now(line_, id_, reg_.TEMP_LIMIT[0], rx_data, reg_.TEMP_LIMIT[2]);
  *min = (int8_t)rx_data[0];
  *max = (int8_t)rx_data[1];
  return ret;
}

int UtraFlxiVApi::set_temp_limit(uint8_t min, uint8_t max, bool now) {
  uint8_t tx_data[2] = {min, max};

  if (now)
    return utra_api_->set_utrc_int8n_now(line_, id_, reg_.TEMP_LIMIT[0], tx_data, reg_.TEMP_LIMIT[3]);
  else
    return utra_api_->set_utrc_int8n_que(line_, id_, reg_.TEMP_LIMIT[0], tx_data, reg_.TEMP_LIMIT[3]);
}

int UtraFlxiVApi::get_volt_limit(int *min, int *max) {
  uint8_t rx_data[2];
  int ret = utra_api_->get_utrc_int8n_now(line_, id_, reg_.VOLT_LIMIT[0], rx_data, reg_.VOLT_LIMIT[2]);
  *min = (int8_t)rx_data[0];
  *max = (int8_t)rx_data[1];
  return ret;
}

int UtraFlxiVApi::set_volt_limit(uint8_t min, uint8_t max, bool now) {
  uint8_t tx_data[2] = {min, max};
  if (now)
    return utra_api_->set_utrc_int8n_now(line_, id_, reg_.VOLT_LIMIT[0], tx_data, reg_.VOLT_LIMIT[3]);
  else
    return utra_api_->set_utrc_int8n_que(line_, id_, reg_.VOLT_LIMIT[0], tx_data, reg_.VOLT_LIMIT[3]);
}

/************************************************************
 *                     Control Api
 ************************************************************/
int UtraFlxiVApi::set_motion_mode(uint8_t value, bool now) {
  if (now)
    return utra_api_->set_utrc_int8_now(line_, id_, reg_.MOTION_MODE[0], value);
  else
    return utra_api_->set_utrc_int8_que(line_, id_, reg_.MOTION_MODE[0], value);
}

int UtraFlxiVApi::get_motion_mode(uint8_t *value) {
  return utra_api_->get_utrc_int8_now(line_, id_, reg_.MOTION_MODE[0], value);
}

int UtraFlxiVApi::set_motion_enable(uint8_t value, bool now) {
  if (now)
    return utra_api_->set_utrc_int8_now(line_, id_, reg_.MOTION_ENABLE[0], value);
  else
    return utra_api_->set_utrc_int8_que(line_, id_, reg_.MOTION_ENABLE[0], value);
}

int UtraFlxiVApi::get_motion_enable(uint8_t *value) {
  return utra_api_->get_utrc_int8_now(line_, id_, reg_.MOTION_ENABLE[0], value);
}

int UtraFlxiVApi::get_temp_motor(float *value) {
  return utra_api_->get_utrc_float_now(line_, id_, reg_.TEMP_MOTOR[0], value);  //
}

int UtraFlxiVApi::get_temp_driver(float *value) {
  return utra_api_->get_utrc_float_now(line_, id_, reg_.TEMP_DRIVER[0], value);
}

int UtraFlxiVApi::get_bus_volt(float *value) {
  return utra_api_->get_utrc_float_now(line_, id_, reg_.BUS_VOLT[0], value);  //
}

int UtraFlxiVApi::get_error_code(uint8_t *value) {
  return utra_api_->get_utrc_int8_now(line_, id_, reg_.ERROR_CODE[0], value);  //
}

/************************************************************
 *                      Senser Api
 ************************************************************/
int UtraFlxiVApi::get_senser(float *value) {
  return utra_api_->get_utrc_nfloat_now(line_, id_, flxiv_reg_.SENSER1[0], 4, value);  //
}