/* Copyright 2020 UmbraTek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#include "base/arm_api_base.h"
#include <unistd.h>
#include "common/hex_data.h"

ArmApiBase::ArmApiBase(void) { pthread_mutex_init(&mutex_, NULL); }

ArmApiBase::ArmApiBase(Socket* socket_fp) {
  pthread_mutex_init(&mutex_, NULL);
  arminit(socket_fp);
}

ArmApiBase::~ArmApiBase(void) {
  is_error_ = true;
  delete socket_fp_;
  delete reg_;
  delete utrc_client_;
}

void ArmApiBase::arminit(Socket* socket_fp) {
  socket_fp_ = socket_fp;
  utrc_client_ = new UtrcClient(socket_fp_);

  utrc_tx_.master_id = 0xAA;
  utrc_tx_.slave_id = 0x55;
  utrc_tx_.state = 0x00;

  axis_ = 6;
  reg_ = new ARM_REG(axis_);
  int ret = get_axis(&axis_);
  if (ret == 0 || ret == UTRC_ERROR::STATE) {
    delete reg_;
    reg_ = new ARM_REG(axis_);
  } else {
    is_error_ = true;
    printf("[ArmApiBa] Error: get_axis: %d\n", ret);
  }
}

void ArmApiBase::send(uint8_t rw, uint8_t cmd, uint8_t cmd_data_len, uint8_t* cmd_data) {
  if (is_error_) return;
  utrc_tx_.rw = rw;
  utrc_tx_.cmd = cmd;
  utrc_tx_.len = cmd_data_len + 1;
  if (cmd_data_len > 0) memcpy(utrc_tx_.data, cmd_data, cmd_data_len);
  utrc_client_->send(&utrc_tx_);
}

int ArmApiBase::pend(uint8_t rx_len, float timeout_s) {  //
  if (is_error_) return -999;
  return utrc_client_->pend(&utrc_tx_, rx_len, timeout_s, &utrc_rx_);
}

int ArmApiBase::sendpend(uint8_t rw, const uint8_t cmd[5], uint8_t* tx_data, float timeout_s) {
  int tx_len = 0, rx_len = 0;
  if (rw == ARM_RW::R) {
    tx_len = cmd[1];
    rx_len = cmd[2];
  } else if (rw == ARM_RW::W) {
    tx_len = cmd[3];
    rx_len = cmd[4];
  }

  pthread_mutex_lock(&mutex_);
  send(rw, cmd[0], tx_len, tx_data);
  int ret = pend(rx_len, timeout_s);
  pthread_mutex_unlock(&mutex_);
  return ret;
}

bool ArmApiBase::is_error(void) { return is_error_; }

int ArmApiBase::get_reg_int8(uint8_t* value, const uint8_t reg[5]) {
  int ret = sendpend(ARM_RW::R, reg, NULL);
  memcpy(value, &utrc_rx_.data[0], reg[2]);
  return ret;
}

int ArmApiBase::set_reg_int8(uint8_t* value, const uint8_t reg[5]) {
  return sendpend(ARM_RW::W, reg, value);  //
}

int ArmApiBase::get_reg_int32(int* value, const uint8_t reg[5], int n) {
  int ret = sendpend(ARM_RW::R, reg, NULL);
  HexData::hex_to_int32_big(&utrc_rx_.data[0], value, n);
  return ret;
}

int ArmApiBase::set_reg_int32(int* value, const uint8_t reg[5], int n) {
  uint8_t data[4];
  HexData::int32_to_hex_big(value, &data[0], n);
  int ret = sendpend(ARM_RW::W, reg, data);
  return ret;
}

int ArmApiBase::get_reg_fp32(float* value, const uint8_t reg[5], int n) {
  int ret = sendpend(ARM_RW::R, reg, NULL);
  HexData::hex_to_fp32_big(&utrc_rx_.data[0], value, n);
  return ret;
}

int ArmApiBase::set_reg_fp32(float* value, const uint8_t reg[5], int n) {
  uint8_t data[4 * n];
  HexData::fp32_to_hex_big(value, data, n);
  int ret = sendpend(ARM_RW::W, reg, data);
  return ret;
}

int ArmApiBase::get_axis(uint8_t* axis) { return get_reg_int8(axis, reg_->UBOT_AXIS); }
int ArmApiBase::get_uuid(uint8_t uuid[17]) { return get_reg_int8(uuid, reg_->UUID); }
int ArmApiBase::get_sw_version(uint8_t version[20]) { return get_reg_int8(version, reg_->SW_VERSION); }
int ArmApiBase::get_hw_version(uint8_t version[20]) { return get_reg_int8(version, reg_->HW_VERSION); }
int ArmApiBase::shutdown_system(void) { return set_reg_int8((uint8_t*)&reg_->SYS_SHUTDOWN[0], reg_->SYS_SHUTDOWN); }
int ArmApiBase::reset_err(void) { return set_reg_int8((uint8_t*)&reg_->RESET_ERR[0], reg_->RESET_ERR); }
int ArmApiBase::reboot_system(void) { return set_reg_int8((uint8_t*)&reg_->SYS_REBOOT[0], reg_->SYS_REBOOT); }
int ArmApiBase::erase_parm(void) { return set_reg_int8((uint8_t*)&reg_->ERASE_PARM[0], reg_->ERASE_PARM); }
int ArmApiBase::saved_parm(void) { return set_reg_int8((uint8_t*)&reg_->SAVED_PARM[0], reg_->SAVED_PARM); }

int ArmApiBase::get_motion_mode(uint8_t* mode) { return get_reg_int8(mode, reg_->MOTION_MDOE); }
int ArmApiBase::set_motion_mode(uint8_t mode) { return set_reg_int8(&mode, reg_->MOTION_MDOE); }
int ArmApiBase::get_motion_enable(int* able) { return get_reg_int32(able, reg_->MOTION_ENABLE); }
int ArmApiBase::set_motion_enable(uint8_t axis, uint8_t en) {
  uint8_t data[2] = {axis, en};
  return set_reg_int8(data, reg_->MOTION_ENABLE);
}
int ArmApiBase::get_brake_enable(int* able) { return get_reg_int32(able, reg_->BRAKE_ENABLE); }
int ArmApiBase::set_brake_enable(uint8_t axis, uint8_t en) {
  uint8_t data[2] = {axis, en};
  return set_reg_int8(data, reg_->BRAKE_ENABLE);
}
int ArmApiBase::get_error_code(uint8_t* code) { return get_reg_int8(code, reg_->ERROR_CODE); }
int ArmApiBase::get_servo_msg(uint8_t* msg) { return get_reg_int8(msg, reg_->SERVO_MSG); }
int ArmApiBase::get_motion_status(uint8_t* status) { return get_reg_int8(status, reg_->MOTION_STATUS); }
int ArmApiBase::set_motion_status(uint8_t status) { return set_reg_int8(&status, reg_->MOTION_STATUS); }
int ArmApiBase::get_cmd_num(int* num) { return get_reg_int32(num, reg_->CMD_NUM); }
int ArmApiBase::set_cmd_num(int num) { return set_reg_int32(&num, reg_->CMD_NUM); }

int ArmApiBase::moveto_cartesian_line(float* mvpose, float mvvelo, float mvacc, float mvtime) {
  float data[6 + 3];
  memcpy(data, mvpose, 6 * 4);
  data[6] = mvvelo;
  data[7] = mvacc;
  data[8] = mvtime;
  return set_reg_fp32(data, reg_->MOVET_LINE, 9);
}

int ArmApiBase::moveto_cartesian_lineb(float* mvpose, float mvvelo, float mvacc, float mvtime, float mvradii) {
  float data[6 + 4];
  memcpy(data, mvpose, 6 * 4);
  data[6] = mvvelo;
  data[7] = mvacc;
  data[8] = mvtime;
  data[9] = mvradii;
  return set_reg_fp32(data, reg_->MOVET_LINEB, 10);
}

int ArmApiBase::moveto_cartesian_p2p(void) { return 0; }
int ArmApiBase::moveto_cartesian_p2pb(void) { return 0; }

int ArmApiBase::moveto_cartesian_circle(float* pose1, float* pose2, float mvvelo, float mvacc, float mvtime, float percent) {
  float data[16];
  memcpy(data, pose1, 6 * 4);
  memcpy(&data[6], pose2, 6 * 4);
  data[12] = mvvelo;
  data[13] = mvacc;
  data[14] = mvtime;
  data[15] = percent;
  return set_reg_fp32(data, reg_->MOVET_CIRCLE, 16);
}

int ArmApiBase::moveto_joint_line(void) { return 0; }
int ArmApiBase::moveto_joint_lineb(void) { return 0; }

int ArmApiBase::moveto_joint_p2p(float* mvjoint, float mvvelo, float mvacc, float mvtime) {
  float data[axis_ + 3];
  memcpy(data, mvjoint, axis_ * 4);
  data[axis_] = mvvelo;
  data[axis_ + 1] = mvacc;
  data[axis_ + 2] = mvtime;
  return set_reg_fp32(data, reg_->MOVEJ_P2P, axis_ + 3);
}

int ArmApiBase::moveto_joint_circle(void) { return 0; }

int ArmApiBase::moveto_home_p2p(float mvvelo, float mvacc, float mvtime) {
  float data[3] = {mvvelo, mvacc, mvtime};
  return set_reg_fp32(data, reg_->MOVEJ_HOME, 3);
}

int ArmApiBase::moveto_servoj(float* mvjoint, float mvvelo, float mvacc, float mvtime) {
  float data[axis_ + 3];
  memcpy(data, mvjoint, axis_ * 4);
  data[axis_] = mvvelo;
  data[axis_ + 1] = mvacc;
  data[axis_ + 2] = mvtime;
  return set_reg_fp32(data, reg_->MOVE_SERVOJ, axis_ + 3);
}

int ArmApiBase::moveto_servo_joint(int frames_num, float* mvjoint, float* mvtime) {
  int data_len = frames_num * (axis_ + 1);
  float txdata[data_len];

  for (int i = 0; i < frames_num; i++) {
    for (int j = 0; j < axis_; j++) {
      txdata[i * (axis_ + 1) + j] = mvjoint[i * axis_ + j];
    }
    txdata[i * (axis_ + 1) + axis_] = mvtime[i];
  }

  uint8_t data[4 * frames_num + 4];
  HexData::int32_to_hex_big(&frames_num, &data[0], 1);
  HexData::fp32_to_hex_big(txdata, &data[4], data_len);
  reg_->MOVES_JOINT[3] = (data_len + 1) * 4;
  int ret = sendpend(ARM_RW::W, reg_->MOVES_JOINT, data);
  return ret;
}

int ArmApiBase::move_sleep(float time) { return set_reg_fp32(&time, reg_->MOVE_SLEEP); }
int ArmApiBase::plan_sleep(float time) { return set_reg_fp32(&time, reg_->PLAN_SLEEP); }

int ArmApiBase::get_tcp_jerk(float* jerk) { return get_reg_fp32(jerk, reg_->TCP_JERK); }
int ArmApiBase::set_tcp_jerk(float jerk) { return set_reg_fp32(&jerk, reg_->TCP_JERK); }

int ArmApiBase::get_tcp_maxacc(float* acc) { return get_reg_fp32(acc, reg_->TCP_MAXACC); }
int ArmApiBase::set_tcp_maxacc(float acc) { return set_reg_fp32(&acc, reg_->TCP_MAXACC); }

int ArmApiBase::get_joint_jerk(float* jerk) { return get_reg_fp32(jerk, reg_->JOINT_JERK); }
int ArmApiBase::set_joint_jerk(float jerk) { return set_reg_fp32(&jerk, reg_->JOINT_JERK); }

int ArmApiBase::get_joint_maxacc(float* acc) { return get_reg_fp32(acc, reg_->JOINT_MAXACC); }
int ArmApiBase::set_joint_maxacc(float acc) { return set_reg_fp32(&acc, reg_->JOINT_MAXACC); }

int ArmApiBase::get_tcp_offset(float* offset) { return get_reg_fp32(offset, reg_->TCP_OFFSET, 6); }
int ArmApiBase::set_tcp_offset(float* offset) { return set_reg_fp32(offset, reg_->TCP_OFFSET, 6); }

int ArmApiBase::get_tcp_load(float* load) { return get_reg_fp32(load, reg_->LOAD_PARAM, 4); }
int ArmApiBase::set_tcp_load(float* load) { return set_reg_fp32(load, reg_->LOAD_PARAM, 4); }

int ArmApiBase::get_gravity_dir(float* dir) { return get_reg_fp32(dir, reg_->GRAVITY_DIR, 3); }
int ArmApiBase::set_gravity_dir(float* dir) { return set_reg_fp32(dir, reg_->GRAVITY_DIR, 3); }

int ArmApiBase::get_collis_sens(uint8_t* sens) { return get_reg_int8(sens, reg_->COLLIS_SENS); }
int ArmApiBase::set_collis_sens(uint8_t sens) { return set_reg_int8(&sens, reg_->COLLIS_SENS); }

int ArmApiBase::get_teach_sens(uint8_t* sens) { return get_reg_int8(sens, reg_->TEACH_SENS); }
int ArmApiBase::set_teach_sens(uint8_t sens) { return set_reg_int8(&sens, reg_->TEACH_SENS); }

int ArmApiBase::get_tcp_target_pos(float* pos) { return get_reg_fp32(pos, reg_->TCP_POS_CURR, 6); }
int ArmApiBase::get_tcp_actual_pos(float* pos) { return 0; }
int ArmApiBase::get_joint_target_pos(float* pos) { return get_reg_fp32(pos, reg_->JOINT_POS_CURR, axis_); }
int ArmApiBase::get_joint_actual_pos(float* pos) { return 0; }

int ArmApiBase::get_ik(void) { return 0; }
int ArmApiBase::get_fk(void) { return 0; }
int ArmApiBase::is_joint_limit(void) { return 0; }
int ArmApiBase::is_tcp_limit(void) { return 0; }

int ArmApiBase::get_friction(uint8_t axis, float* fri) {
  uint8_t tx_data[2] = {reg_->FRICTION[0], axis};
  int ret = sendpend(ARM_RW::R, reg_->FRICTION, tx_data);
  HexData::hex_to_fp32_big(&utrc_rx_.data[0], fri, 4);
  return ret;
}
int ArmApiBase::set_friction(uint8_t axis, float* fri) {
  uint8_t data[4 * 4 + 2] = {reg_->FRICTION[0], axis};
  HexData::fp32_to_hex_big(fri, &data[2], 4);
  int ret = sendpend(ARM_RW::W, reg_->FRICTION, data);
  return ret;
}

int ArmApiBase::get_utrc_int8_now(uint8_t line, uint8_t id, uint8_t reg, uint8_t* value) {
  uint8_t tx_data[3] = {line, id, reg};
  int ret = sendpend(ARM_RW::R, reg_->UTRC_INT8_NOW, tx_data);
  if (ret == 0 || ret == UTRC_ERROR::STATE) ret = utrc_rx_.data[0];
  *value = utrc_rx_.data[1];
  return ret;
}

int ArmApiBase::set_utrc_int8_now(uint8_t line, uint8_t id, uint8_t reg, uint8_t value) {
  uint8_t tx_data[4] = {line, id, reg, value};
  int ret = sendpend(ARM_RW::W, reg_->UTRC_INT8_NOW, tx_data);
  if (ret == 0 || ret == UTRC_ERROR::STATE) ret = utrc_rx_.data[0];
  return ret;
}

int ArmApiBase::get_utrc_int32_now(uint8_t line, uint8_t id, uint8_t reg, int32_t* value) {
  uint8_t tx_data[3] = {line, id, reg};
  int32_t rx_data[2];
  int ret = sendpend(ARM_RW::R, reg_->UTRC_INT8_NOW, tx_data);
  HexData::hex_to_int32_big(&utrc_rx_.data[0], rx_data, 2);
  if (ret == 0 || ret == UTRC_ERROR::STATE) ret = rx_data[0];
  *value = rx_data[1];
  return ret;
}

int ArmApiBase::set_utrc_int32_now(uint8_t line, uint8_t id, uint8_t reg, int32_t value) {
  uint8_t tx_data[7] = {line, id, reg};
  HexData::int32_to_hex_big(&value, &tx_data[3], 1);
  int ret = sendpend(ARM_RW::W, reg_->UTRC_INT8_NOW, tx_data);
  if (ret == 0 || ret == UTRC_ERROR::STATE) ret = utrc_rx_.data[0];
  return ret;
}

int ArmApiBase::get_utrc_float_now(uint8_t line, uint8_t id, uint8_t reg, float* value) {
  uint8_t tx_data[3] = {line, id, reg};
  float rx_data[2];
  int ret = sendpend(ARM_RW::R, reg_->UTRC_FP32_NOW, tx_data);
  HexData::hex_to_fp32_big(&utrc_rx_.data[0], rx_data, 2);
  if (ret == 0 || ret == UTRC_ERROR::STATE) ret = rx_data[0];
  *value = rx_data[1];
  return ret;
}

int ArmApiBase::set_utrc_float_now(uint8_t line, uint8_t id, uint8_t reg, float value) {
  uint8_t tx_data[7] = {line, id, reg};
  HexData::fp32_to_hex_big(&value, &tx_data[3], 1);
  int ret = sendpend(ARM_RW::W, reg_->UTRC_FP32_NOW, tx_data);
  if (ret == 0 || ret == UTRC_ERROR::STATE) ret = utrc_rx_.data[0];
  return ret;
}

int ArmApiBase::get_utrc_int8n_now(uint8_t line, uint8_t id, uint8_t reg, uint8_t* data, uint8_t rx_len) {
  uint8_t tx_data[4] = {line, id, reg, rx_len};
  reg_->UTRC_INT8N_NOW[2] = rx_len + 1;
  int ret = sendpend(ARM_RW::R, reg_->UTRC_INT8N_NOW, tx_data);
  if (ret == 0 || ret == UTRC_ERROR::STATE) ret = utrc_rx_.data[0];
  memcpy(data, &utrc_rx_.data[1], rx_len);
  return ret;
}

int ArmApiBase::set_utrc_int8n_now(uint8_t line, uint8_t id, uint8_t reg, uint8_t* data, uint8_t tx_len) {
  uint8_t tx_data[4 + tx_len] = {line, id, reg, tx_len};
  memcpy(&tx_data[4], data, tx_len);
  reg_->UTRC_INT8N_NOW[3] = tx_len + 4;

  int ret = sendpend(ARM_RW::W, reg_->UTRC_INT8N_NOW, tx_data);
  if (ret == 0 || ret == UTRC_ERROR::STATE) ret = utrc_rx_.data[0];
  return ret;
}

int ArmApiBase::set_utrc_int8_que(uint8_t line, uint8_t id, uint8_t reg, uint8_t value) {
  uint8_t tx_data[4] = {line, id, reg, value};
  int ret = sendpend(ARM_RW::W, reg_->UTRC_INT8_QUE, tx_data);
  if (ret == 0 || ret == UTRC_ERROR::STATE) ret = 0;
  return ret;
}
int ArmApiBase::set_utrc_int32_que(uint8_t line, uint8_t id, uint8_t reg, int32_t value) {
  uint8_t tx_data[7] = {line, id, reg};
  HexData::int32_to_hex_big(&value, &tx_data[3], 1);
  int ret = sendpend(ARM_RW::W, reg_->UTRC_INT32_QUE, tx_data);
  if (ret == 0 || ret == UTRC_ERROR::STATE) ret = 0;
  return ret;
}

int ArmApiBase::set_utrc_float_que(uint8_t line, uint8_t id, uint8_t reg, float value) {
  uint8_t tx_data[7] = {line, id, reg};
  HexData::fp32_to_hex_big(&value, &tx_data[3], 1);
  int ret = sendpend(ARM_RW::W, reg_->UTRC_FP32_QUE, tx_data);
  if (ret == 0 || ret == UTRC_ERROR::STATE) ret = 0;
  return ret;
}

int ArmApiBase::set_utrc_int8n_que(uint8_t line, uint8_t id, uint8_t reg, uint8_t* data, uint8_t tx_len) {
  uint8_t tx_data[4 + tx_len] = {line, id, reg, tx_len};
  memcpy(&tx_data[4], data, tx_len);
  reg_->UTRC_INT8N_QUE[3] = tx_len + 4;

  int ret = sendpend(ARM_RW::W, reg_->UTRC_INT8N_QUE, tx_data);
  if (ret == 0 || ret == UTRC_ERROR::STATE) ret = 0;
  return ret;
}

int ArmApiBase::set_pass_rs485_now(uint8_t line, uint8_t timeout_ms, uint8_t tx_len, uint8_t rx_len, uint8_t* tx_data,
                                   uint8_t* rx_data) {
  if (tx_len > 125 || rx_len > 125) return -3;

  uint8_t temp[4 + tx_len] = {line, timeout_ms, tx_len, rx_len};
  memcpy(&temp[4], tx_data, tx_len);
  reg_->PASS_RS485_NOW[3] = tx_len + 4;
  reg_->PASS_RS485_NOW[4] = rx_len + 2;

  int ret = sendpend(ARM_RW::W, reg_->PASS_RS485_NOW, temp);
  if (ret == 0 || ret == UTRC_ERROR::STATE) ret = utrc_rx_.data[0];
  memcpy(rx_data, &utrc_rx_.data[1], rx_len);
  return ret;
}

int ArmApiBase::set_pass_rs485_que(uint8_t line, uint8_t tx_len, uint8_t* tx_data) {
  if (tx_len > 125) return -3;

  uint8_t temp[2 + tx_len] = {line, tx_len};
  memcpy(&temp[2], tx_data, tx_len);
  reg_->PASS_RS485_QUE[3] = tx_len + 2;

  int ret = sendpend(ARM_RW::W, reg_->PASS_RS485_QUE, temp);
  if (ret == 0 || ret == UTRC_ERROR::STATE) ret = 0;

  return ret;
}

int ArmApiBase::get_utrc_u8float_now(uint8_t line, uint8_t id, uint8_t reg, uint8_t num, float* value) {
  uint8_t tx_data[4] = {line, id, reg, num};
  float rx_data[2];
  int ret = sendpend(ARM_RW::R, reg_->UTRC_U8FP32_NOW, tx_data);
  HexData::hex_to_fp32_big(&utrc_rx_.data[0], rx_data, 2);
  if (ret == 0 || ret == UTRC_ERROR::STATE) ret = rx_data[0];
  *value = rx_data[1];
  return ret;
}

int ArmApiBase::set_utrc_u8float_now(uint8_t line, uint8_t id, uint8_t reg, uint8_t num, float value) {
  uint8_t tx_data[8] = {line, id, reg, num};
  HexData::fp32_to_hex_big(&value, &tx_data[4], 1);
  int ret = sendpend(ARM_RW::W, reg_->UTRC_U8FP32_NOW, tx_data);
  if (ret == 0 || ret == UTRC_ERROR::STATE) ret = utrc_rx_.data[0];
  return ret;
}

int ArmApiBase::get_gpio_in(uint8_t line, uint8_t id, int32_t* fun, int32_t* digit, int* adc_num, float* adc_value) {
  uint8_t tx_data[2] = {line, id};
  int ret = sendpend(ARM_RW::R, reg_->GPIO_IN, tx_data);
  if (ret == 0 || ret == UTRC_ERROR::STATE) ret = utrc_rx_.data[0];
  HexData::hex_to_int32_big(&utrc_rx_.data[1], fun, 1);
  HexData::hex_to_int32_big(&utrc_rx_.data[5], digit, 1);
  *adc_num = utrc_rx_.data[9];
  HexData::hex_to_fp32_big(&utrc_rx_.data[10], adc_value, *adc_num);
  return ret;
}
int ArmApiBase::get_tgpio_in(int32_t* fun, int32_t* digit, int* adc_num, float* adc_value) {
  return get_gpio_in(2, 1, fun, digit, adc_num, adc_value);
}
int ArmApiBase::get_cgpio_in(int32_t* fun, int32_t* digit, int* adc_num, float* adc_value) {
  return get_gpio_in(3, 1, fun, digit, adc_num, adc_value);
}

int ArmApiBase::get_gpio_out(uint8_t line, uint8_t id, int32_t* fun, int32_t* digit, int* adc_num, float* adc_value) {
  uint8_t tx_data[2] = {line, id};
  int ret = sendpend(ARM_RW::R, reg_->GPIO_OU, tx_data);
  if (ret == 0 || ret == UTRC_ERROR::STATE) ret = utrc_rx_.data[0];
  HexData::hex_to_int32_big(&utrc_rx_.data[1], fun, 1);
  HexData::hex_to_int32_big(&utrc_rx_.data[5], digit, 1);
  *adc_num = utrc_rx_.data[9];
  HexData::hex_to_fp32_big(&utrc_rx_.data[10], adc_value, *adc_num);
  return ret;
}

int ArmApiBase::get_tgpio_out(int32_t* fun, int32_t* digit, int* adc_num, float* adc_value) {
  return get_gpio_out(2, 1, fun, digit, adc_num, adc_value);
}
int ArmApiBase::get_cgpio_out(int32_t* fun, int32_t* digit, int* adc_num, float* adc_value) {
  return get_gpio_out(3, 1, fun, digit, adc_num, adc_value);
}

int ArmApiBase::set_tgpio_digit_out(int value) { return set_utrc_int32_now(2, 1, 0x13, value); }
int ArmApiBase::set_cgpio_digit_out(int value) { return set_utrc_int32_now(3, 1, 0x13, value); }

int ArmApiBase::get_cgpio_uuid(uint8_t* value) { return get_utrc_int8n_now(3, 1, 0x01, value, 12); }
int ArmApiBase::get_cgpio_sw_version(uint8_t* value) { return get_utrc_int8n_now(3, 1, 0x02, value, 12); }
int ArmApiBase::get_cgpio_hw_version(uint8_t* value) { return get_utrc_int8n_now(3, 1, 0x03, value, 12); }

int ArmApiBase::get_tgpio_uuid(uint8_t* value) { return get_utrc_int8n_now(2, 1, 0x01, value, 12); }
int ArmApiBase::get_tgpio_sw_version(uint8_t* value) { return get_utrc_int8n_now(2, 1, 0x02, value, 12); }
int ArmApiBase::get_tgpio_hw_version(uint8_t* value) { return get_utrc_int8n_now(2, 1, 0x03, value, 12); }
