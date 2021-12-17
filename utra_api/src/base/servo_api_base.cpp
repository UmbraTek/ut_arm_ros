/* Copyright 2020 UmbraTek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#include "base/servo_api_base.h"
#include "common/hex_data.h"

ServoApiBase::ServoApiBase(void) { pthread_mutex_init(&mutex_, NULL); }

ServoApiBase::ServoApiBase(uint8_t bus_type, Socket* socket_fp, uint8_t servo_id) {
  pthread_mutex_init(&mutex_, NULL);
  servoinit(bus_type, socket_fp, servo_id);
}

ServoApiBase::~ServoApiBase(void) {
  if (utrc_client_ != NULL) delete utrc_client_;
}

void ServoApiBase::servoinit(uint8_t bus_type, Socket* socket_fp, uint8_t servo_id) {
  bus_type_ = bus_type;
  socket_fp_ = socket_fp;
  id_ = servo_id;

  if (bus_type_ == BUS_TYPE::UTRC) {
    utrc_client_ = new UtrcClient(socket_fp_);

    utrc_tx_.master_id = 0xAA;
    utrc_tx_.slave_id = servo_id;
    utrc_tx_.state = 0x00;
  } else if (bus_type_ == BUS_TYPE::CANBUS) {
  }
}

void ServoApiBase::send(uint8_t rw, uint8_t cmd, uint8_t cmd_data_len, uint8_t* cmd_data) {
  utrc_tx_.rw = rw;
  utrc_tx_.cmd = cmd;
  utrc_tx_.len = cmd_data_len + 1;
  if (cmd_data_len > 0) memcpy(utrc_tx_.data, cmd_data, cmd_data_len);
  utrc_client_->send(&utrc_tx_);
}

int ServoApiBase::pend(uint8_t rx_len, float timeout_s) {  //
  return utrc_client_->pend(&utrc_tx_, rx_len, timeout_s, &utrc_rx_);
}

int ServoApiBase::sendpend(int id, uint8_t rw, const uint8_t cmd[5], uint8_t* tx_data, float timeout_s) {
  int tx_len = 0, rx_len = 0;
  if (rw == SERVO_RW::R) {
    tx_len = cmd[1];
    rx_len = cmd[2];
  } else if (rw == SERVO_RW::W) {
    tx_len = cmd[3];
    rx_len = cmd[4];
  }

  pthread_mutex_lock(&mutex_);
  if (bus_type_ == BUS_TYPE::UTRC) {
    id_ = id;
    utrc_tx_.slave_id = id_;
  } else if (bus_type_ == BUS_TYPE::CANBUS) {
  }
  send(rw, cmd[0], tx_len, tx_data);
  int ret = pend(rx_len, timeout_s);
  pthread_mutex_unlock(&mutex_);
  return ret;
}

int ServoApiBase::get_reg_int8(int id, uint8_t* value, const uint8_t reg[5]) {
  int ret = sendpend(id, SERVO_RW::R, reg, NULL);
  memcpy(value, &utrc_rx_.data[0], reg[2]);
  return ret;
}

int ServoApiBase::set_reg_int8(int id, uint8_t* value, const uint8_t reg[5]) {
  return sendpend(id, SERVO_RW::W, reg, value);  //
}

int ServoApiBase::get_reg_int32(int id, int* value, const uint8_t reg[5]) {
  int ret = sendpend(id, SERVO_RW::R, reg, NULL);
  *value = HexData::hex_to_int32_big(&utrc_rx_.data[0]);
  return ret;
}

int ServoApiBase::set_reg_int32(int id, int value, const uint8_t reg[5]) {
  uint8_t data[4];
  HexData::int32_to_hex_big(value, &data[0]);
  int ret = sendpend(id, SERVO_RW::W, reg, data);
  return ret;
}

int ServoApiBase::get_reg_fp32(int id, float* value, const uint8_t reg[5]) {
  int ret = sendpend(id, SERVO_RW::R, reg, NULL);
  // int32_t value_int = HexData::hex_to_int32_big(&utrc_rx_.data[0]);
  // *value = SERVO_INT_TO_FP(value_int);
  HexData::hex_to_fp32_big(&utrc_rx_.data[0], value, 1);
  return ret;
}

int ServoApiBase::set_reg_fp32(int id, float value, const uint8_t reg[5]) {
  uint8_t data[4];
  // int32_t value_int = SERVO_FP_TO_INT(value);
  // HexData::int32_to_hex_big(value_int, data);
  HexData::fp32_to_hex_big(value, data);
  int ret = sendpend(id, SERVO_RW::W, reg, data);
  return ret;
}

int ServoApiBase::set_reg_int8(int id, uint8_t value, const uint8_t reg[5]) { return set_reg_int8(id, &value, reg); }

int ServoApiBase::get_reg_int8(int id, uint8_t* value1, uint8_t* value2, const uint8_t reg[5]) {
  uint8_t data[2] = {0, 0};
  int ret = get_reg_int8(id, data, reg);
  *value1 = data[0];
  *value2 = data[1];
  return ret;
}
int ServoApiBase::get_reg_int8(int id, int8_t* value1, int8_t* value2, const uint8_t reg[5]) {
  uint8_t data[2] = {0, 0};
  int ret = get_reg_int8(id, data, reg);
  *value1 = data[0];
  *value2 = data[1];
  return ret;
}
int ServoApiBase::set_reg_int8(int id, uint8_t value1, uint8_t value2, const uint8_t reg[5]) {
  printf("==>5\n");
  uint8_t data[2] = {value1, value2};
  return set_reg_int8(id, data, reg);
}

int ServoApiBase::get_uuid_(int id, char uuid[24]) {
  uint8_t temp[12];
  int ret = get_reg_int8(id, temp, reg_.UUID);
  for (int i = 0; i < 12; ++i) sprintf(&uuid[i * 2], "%02x", temp[i]);
  return ret;
}
int ServoApiBase::get_sw_version_(int id, char version[12]) {
  uint8_t temp[12];
  int ret = get_reg_int8(id, temp, reg_.SW_VERSION);
  for (int i = 0; i < 12; ++i) sprintf(&version[i], "%c", temp[i]);
  return ret;
}
int ServoApiBase::get_hw_version_(int id, char version[24]) {
  uint8_t temp[12];
  int ret = get_reg_int8(id, temp, reg_.HW_VERSION);
  for (int i = 0; i < 12; ++i) sprintf(&version[i * 2], "%02x", temp[i]);
  return ret;
}
int ServoApiBase::get_multi_version_(int id, char version[12]) {
  uint8_t temp[12];
  int ret = get_reg_int8(id, temp, reg_.MULTI_VERSION);
  int temp2 = HexData::hex_to_int32_big(temp);
  version[0] = '0';
  version[1] = '0';
  version[2] = '0';
  sprintf(&version[3], "%d", temp2);
  return ret;
}
int ServoApiBase::get_mech_ratio_(int id, float* ratio) { return get_reg_fp32(id, ratio, reg_.MECH_RATIO); }
int ServoApiBase::set_mech_ratio_(int id, float ratio) { return set_reg_fp32(id, ratio, reg_.MECH_RATIO); }
int ServoApiBase::set_com_id_(int id, int set_id) { return set_reg_int8(id, set_id, reg_.COM_ID); }
int ServoApiBase::set_com_baud_(int id, int baud) { return set_reg_int32(id, baud, reg_.COM_BAUD); }
int ServoApiBase::reset_err_(int id) { return set_reg_int8(id, reg_.RESET_ERR[0], reg_.RESET_ERR); }
int ServoApiBase::restart_driver_(int id) { return set_reg_int8(id, reg_.RESET_DRIVER[0], reg_.RESET_DRIVER); }
int ServoApiBase::erase_parm_(int id) { return set_reg_int8(id, reg_.ERASE_PARM[0], reg_.ERASE_PARM); }
int ServoApiBase::saved_parm_(int id) { return set_reg_int8(id, reg_.SAVED_PARM[0], reg_.SAVED_PARM); }

int ServoApiBase::get_elec_ratio_(int id, float* ratio) { return get_reg_fp32(id, ratio, reg_.ELEC_RATIO); }
int ServoApiBase::set_elec_ratio_(int id, float ratio) { return set_reg_fp32(id, ratio, reg_.ELEC_RATIO); }
int ServoApiBase::get_motion_dir_(int id, uint8_t* dir) { return get_reg_int8(id, dir, reg_.MOTION_DIR); }
int ServoApiBase::set_motion_dir_(int id, uint8_t dir) { return set_reg_int8(id, dir, reg_.MOTION_DIR); }
int ServoApiBase::get_temp_limit_(int id, int8_t* min, int8_t* max) { return get_reg_int8(id, min, max, reg_.TEMP_LIMIT); }
int ServoApiBase::set_temp_limit_(int id, int8_t min, int8_t max) { return set_reg_int8(id, min, max, reg_.TEMP_LIMIT); }
int ServoApiBase::get_volt_limit_(int id, uint8_t* min, uint8_t* max) { return get_reg_int8(id, min, max, reg_.VOLT_LIMIT); }
int ServoApiBase::set_volt_limit_(int id, uint8_t min, uint8_t max) { return set_reg_int8(id, min, max, reg_.VOLT_LIMIT); }
int ServoApiBase::get_curr_limit_(int id, float* curr) { return get_reg_fp32(id, curr, reg_.CURR_LIMIT); }
int ServoApiBase::set_curr_limit_(int id, float curr) { return set_reg_fp32(id, curr, reg_.CURR_LIMIT); }

// motion model
int ServoApiBase::get_motion_mode_(int id, uint8_t* mode) { return get_reg_int8(id, mode, reg_.MOTION_MODE); }
int ServoApiBase::set_motion_mode_(int id, uint8_t mode) { return set_reg_int8(id, mode, reg_.MOTION_MODE); }

// motion able
int ServoApiBase::get_motion_enable_(int id, uint8_t* able) { return get_reg_int8(id, able, reg_.MOTION_ENABLE); }
int ServoApiBase::set_motion_enable_(int id, uint8_t able) { return set_reg_int8(id, able, reg_.MOTION_ENABLE); }

// brake able
int ServoApiBase::get_brake_enable_(int id, uint8_t* able) { return get_reg_int8(id, able, reg_.BRAKE_ENABLE); }
int ServoApiBase::set_brake_enable_(int id, uint8_t able) { return set_reg_int8(id, able, reg_.BRAKE_ENABLE); }

int ServoApiBase::get_temp_driver_(int id, float* temp) { return get_reg_fp32(id, temp, reg_.TEMP_DRIVER); }
int ServoApiBase::get_temp_motor_(int id, float* temp) { return get_reg_fp32(id, temp, reg_.TEMP_MOTOR); }
int ServoApiBase::get_bus_volt_(int id, float* volt) { return get_reg_fp32(id, volt, reg_.BUS_VOLT); }
int ServoApiBase::get_bus_curr_(int id, float* curr) { return get_reg_fp32(id, curr, reg_.BUS_CURR); }
int ServoApiBase::get_multi_volt_(int id, float* volt) { return get_reg_fp32(id, volt, reg_.MULTI_VOLT); }
int ServoApiBase::get_error_code_(int id, uint8_t* errcode) { return get_reg_int8(id, errcode, reg_.ERROR_CODE); }

// pos
int ServoApiBase::get_pos_target_(int id, float* pos) { return get_reg_fp32(id, pos, reg_.POS_TARGET); }
int ServoApiBase::set_pos_target_(int id, float pos) { return set_reg_fp32(id, pos, reg_.POS_TARGET); }
int ServoApiBase::get_pos_current_(int id, float* pos) { return get_reg_fp32(id, pos, reg_.POS_CURRENT); }
int ServoApiBase::get_pos_limit_min_(int id, float* pos) { return get_reg_fp32(id, pos, reg_.POS_LIMIT_MIN); }
int ServoApiBase::set_pos_limit_min_(int id, float pos) { return set_reg_fp32(id, pos, reg_.POS_LIMIT_MIN); }
int ServoApiBase::get_pos_limit_max_(int id, float* pos) { return get_reg_fp32(id, pos, reg_.POS_LIMIT_MAX); }
int ServoApiBase::set_pos_limit_max_(int id, float pos) { return set_reg_fp32(id, pos, reg_.POS_LIMIT_MAX); }
int ServoApiBase::get_pos_limit_diff_(int id, float* pos) { return get_reg_fp32(id, pos, reg_.POS_LIMIT_DIFF); }
int ServoApiBase::set_pos_limit_diff_(int id, float pos) { return set_reg_fp32(id, pos, reg_.POS_LIMIT_DIFF); }
int ServoApiBase::get_pos_pidp_(int id, float* p) { return get_reg_fp32(id, p, reg_.POS_PIDP); }
int ServoApiBase::set_pos_pidp_(int id, float p) { return set_reg_fp32(id, p, reg_.POS_PIDP); }
int ServoApiBase::get_pos_smooth_cyc_(int id, uint8_t* cyc) { return get_reg_int8(id, cyc, reg_.POS_SMOOTH_CYC); }
int ServoApiBase::set_pos_smooth_cyc_(int id, uint8_t cyc) { return set_reg_int8(id, cyc, reg_.POS_SMOOTH_CYC); }
int ServoApiBase::get_pos_adrc_param_(int id, uint8_t i, float* param) {
  uint8_t data[1] = {i};
  int ret = sendpend(id, SERVO_RW::R, reg_.POS_ADRC_PARAM, data);
  HexData::hex_to_fp32_big(&utrc_rx_.data[0], param, 1);
  return ret;
}
int ServoApiBase::set_pos_adrc_param_(int id, uint8_t i, float param) {
  uint8_t data[4];
  HexData::fp32_to_hex_big(param, data);
  uint8_t txdata[5] = {i, data[0], data[1], data[2], data[3]};
  int ret = sendpend(id, SERVO_RW::W, reg_.POS_ADRC_PARAM, txdata);
  return ret;
}
int ServoApiBase::pos_cal_zero_(int id) { return set_reg_int8(id, reg_.POS_CAL_ZERO[0], reg_.POS_CAL_ZERO); }

// vel
int ServoApiBase::get_vel_target_(int id, float* vel) { return get_reg_fp32(id, vel, reg_.VEL_TARGET); }
int ServoApiBase::set_vel_target_(int id, float vel) { return set_reg_fp32(id, vel, reg_.VEL_TARGET); }
int ServoApiBase::get_vel_current_(int id, float* vel) { return get_reg_fp32(id, vel, reg_.VEL_CURRENT); }
int ServoApiBase::get_vel_limit_min_(int id, float* vel) { return get_reg_fp32(id, vel, reg_.VEL_LIMIT_MIN); }
int ServoApiBase::set_vel_limit_min_(int id, float vel) { return set_reg_fp32(id, vel, reg_.VEL_LIMIT_MIN); }
int ServoApiBase::get_vel_limit_max_(int id, float* vel) { return get_reg_fp32(id, vel, reg_.VEL_LIMIT_MAX); }
int ServoApiBase::set_vel_limit_max_(int id, float vel) { return set_reg_fp32(id, vel, reg_.VEL_LIMIT_MAX); }
int ServoApiBase::get_vel_limit_diff_(int id, float* vel) { return get_reg_fp32(id, vel, reg_.VEL_LIMIT_DIFF); }
int ServoApiBase::set_vel_limit_diff_(int id, float vel) { return set_reg_fp32(id, vel, reg_.VEL_LIMIT_DIFF); }
int ServoApiBase::get_vel_pidp_(int id, float* p) { return get_reg_fp32(id, p, reg_.VEL_PIDP); }
int ServoApiBase::set_vel_pidp_(int id, float p) { return set_reg_fp32(id, p, reg_.VEL_PIDP); }
int ServoApiBase::get_vel_pidi_(int id, float* i) { return get_reg_fp32(id, i, reg_.VEL_PIDI); }
int ServoApiBase::set_vel_pidi_(int id, float i) { return set_reg_fp32(id, i, reg_.VEL_PIDI); }
int ServoApiBase::get_vel_smooth_cyc_(int id, uint8_t* cyc) { return get_reg_int8(id, cyc, reg_.VEL_SMOOTH_CYC); }
int ServoApiBase::set_vel_smooth_cyc_(int id, uint8_t cyc) { return set_reg_int8(id, cyc, reg_.VEL_SMOOTH_CYC); }
int ServoApiBase::get_vel_adrc_param_(int id, uint8_t i, float* param) {
  uint8_t data[1] = {i};
  int ret = sendpend(id, SERVO_RW::R, reg_.VEL_ADRC_PARAM, data);
  HexData::hex_to_fp32_big(&utrc_rx_.data[0], param, 1);
  return ret;
}
int ServoApiBase::set_vel_adrc_param_(int id, uint8_t i, float param) {
  uint8_t data[4];
  HexData::fp32_to_hex_big(param, data);
  uint8_t txdata[5] = {i, data[0], data[1], data[2], data[3]};
  int ret = sendpend(id, SERVO_RW::W, reg_.VEL_ADRC_PARAM, txdata);
  return ret;
}

// tau
int ServoApiBase::get_tau_target_(int id, float* tau) { return get_reg_fp32(id, tau, reg_.TAU_TARGET); }
int ServoApiBase::set_tau_target_(int id, float tau) { return set_reg_fp32(id, tau, reg_.TAU_TARGET); }
int ServoApiBase::get_tau_current_(int id, float* tau) { return get_reg_fp32(id, tau, reg_.TAU_CURRENT); }
int ServoApiBase::get_tau_limit_min_(int id, float* tau) { return get_reg_fp32(id, tau, reg_.TAU_LIMIT_MIN); }
int ServoApiBase::set_tau_limit_min_(int id, float tau) { return set_reg_fp32(id, tau, reg_.TAU_LIMIT_MIN); }
int ServoApiBase::get_tau_limit_max_(int id, float* tau) { return get_reg_fp32(id, tau, reg_.TAU_LIMIT_MAX); }
int ServoApiBase::set_tau_limit_max_(int id, float tau) { return set_reg_fp32(id, tau, reg_.TAU_LIMIT_MAX); }
int ServoApiBase::get_tau_limit_diff_(int id, float* tau) { return get_reg_fp32(id, tau, reg_.TAU_LIMIT_DIFF); }
int ServoApiBase::set_tau_limit_diff_(int id, float tau) { return set_reg_fp32(id, tau, reg_.TAU_LIMIT_DIFF); }
int ServoApiBase::get_tau_pidp_(int id, float* p) { return get_reg_fp32(id, p, reg_.TAU_PIDP); }
int ServoApiBase::set_tau_pidp_(int id, float p) { return set_reg_fp32(id, p, reg_.TAU_PIDP); }
int ServoApiBase::get_tau_pidi_(int id, float* i) { return get_reg_fp32(id, i, reg_.TAU_PIDI); }
int ServoApiBase::set_tau_pidi_(int id, float i) { return set_reg_fp32(id, i, reg_.TAU_PIDI); }
int ServoApiBase::get_tau_smooth_cyc_(int id, uint8_t* cyc) { return get_reg_int8(id, cyc, reg_.TAU_SMOOTH_CYC); }
int ServoApiBase::set_tau_smooth_cyc_(int id, uint8_t cyc) { return set_reg_int8(id, cyc, reg_.TAU_SMOOTH_CYC); }
int ServoApiBase::get_tau_adrc_param_(int id, uint8_t i, float* param) {
  uint8_t data[1] = {i};
  int ret = sendpend(id, SERVO_RW::R, reg_.TAU_ADRC_PARAM, data);
  HexData::hex_to_fp32_big(&utrc_rx_.data[0], param, 1);
  return ret;
}
int ServoApiBase::set_tau_adrc_param_(int id, uint8_t i, float param) {
  uint8_t data[4];
  HexData::fp32_to_hex_big(param, data);
  uint8_t txdata[5] = {i, data[0], data[1], data[2], data[3]};
  int ret = sendpend(id, SERVO_RW::W, reg_.TAU_ADRC_PARAM, txdata);
  return ret;
}

int ServoApiBase::set_cpos_target_(uint8_t sid, uint8_t eid, float* pos) {
  int num = eid - sid + 1;
  int32_t temp[num];
  for (int i = 0; i < num; i++) temp[i] = SERVO_FP_TO_INT(pos[i]);

  uint8_t data[4 * num + 2];
  data[0] = sid;
  data[1] = eid;
  HexData::int32_to_hex_big(temp, &data[2], num);

  pthread_mutex_lock(&mutex_);
  id_ = 0x55;
  utrc_tx_.slave_id = id_;
  reg_.CPOS_TARGET[3] = 2 + 4 * num;
  send(SERVO_RW::W, reg_.CPOS_TARGET[0], num * 4 + 2, data);
  pthread_mutex_unlock(&mutex_);

  return 0;
}

int ServoApiBase::get_spostau_current_(int id, int* num, float* pos, float* tau) {
  int ret = sendpend(id, SERVO_RW::R, reg_.SPOSTAU_CURRENT, NULL);
  *num = utrc_rx_.data[0];
  HexData::hex_to_fp32_big(&utrc_rx_.data[1], pos, 1);
  HexData::hex_to_fp32_big(&utrc_rx_.data[5], tau, 1);
  // int32_t pos_int = HexData::hex_to_int32_big(&utrc_rx_.data[1]);
  // int32_t tau_int = HexData::hex_to_int32_big(&utrc_rx_.data[5]);

  // *pos = SERVO_INT_TO_FP(pos_int);
  // *tau = SERVO_INT_TO_FP(tau_int);
  return ret;
}

int ServoApiBase::get_cpostau_current_(uint8_t sid, uint8_t eid, int* num, float* pos, float* tau, int* ret) {
  if (bus_type_ != BUS_TYPE::UTRC) return -77;

  int temp = 0;
  int id = 0x55;
  uint8_t rw = SERVO_RW::R;
  const uint8_t* cmd = reg_.CPOSTAU_CURRENT;
  uint8_t tx_data[2] = {sid, eid};
  float timeout_s = 0.005;
  int tx_len = cmd[1];
  int rx_len = cmd[2];

  pthread_mutex_lock(&mutex_);
  id_ = id;
  utrc_tx_.slave_id = id_;
  send(rw, cmd[0], tx_len, tx_data);

  for (int i = 0; i < eid - sid + 1; i++) {
    ret[i] = pend(rx_len, timeout_s);
    if (utrc_rx_.master_id != i + 1) ret[i] = UTRC_ERROR::TIMEOUT;

    num[i] = utrc_rx_.data[0];
    int32_t pos_int = HexData::hex_to_int32_big(&utrc_rx_.data[1]);
    int32_t tau_int = HexData::hex_to_int32_big(&utrc_rx_.data[5]);
    pos[i] = SERVO_INT_TO_FP(pos_int);
    tau[i] = SERVO_INT_TO_FP(tau_int);
    temp += ret[i];
  }
  pthread_mutex_unlock(&mutex_);

  return temp;
}
