/* Copyright 2020 UmbraTek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#ifndef __SERVO_API_BASE_H__
#define __SERVO_API_BASE_H__

#include "base/servo_reg.h"
#include "common/utrc_t.h"

class ServoApiBase {
 public:
  ServoApiBase(void);
  ServoApiBase(uint8_t bus_type, Socket* socket_fp, uint8_t servo_id);
  ~ServoApiBase(void);
  void servoinit(uint8_t bus_type, Socket* socket_fp, uint8_t servo_id);

  int get_uuid_(int id, char uuid[24]);
  int get_sw_version_(int id, char version[12]);
  int get_hw_version_(int id, char version[24]);
  int get_multi_version_(int id, char version[12]);
  int get_mech_ratio_(int id, float* ratio);
  int set_mech_ratio_(int id, float ratio);
  int set_com_id_(int id, int set_id);
  int set_com_baud_(int id, int baud);
  int reset_err_(int id);
  int restart_driver_(int id);
  int erase_parm_(int id);
  int saved_parm_(int id);

  int get_elec_ratio_(int id, float* ratio);
  int set_elec_ratio_(int id, float ratio);
  int get_motion_dir_(int id, uint8_t* dir);
  int set_motion_dir_(int id, uint8_t dir);
  int get_temp_limit_(int id, int8_t* min, int8_t* max);
  int set_temp_limit_(int id, int8_t min, int8_t max);
  int get_volt_limit_(int id, uint8_t* min, uint8_t* max);
  int set_volt_limit_(int id, uint8_t min, uint8_t max);
  int get_curr_limit_(int id, float* curr);
  int set_curr_limit_(int id, float curr);

  int get_motion_mode_(int id, uint8_t* mode);
  int set_motion_mode_(int id, uint8_t mode);
  int get_motion_enable_(int id, uint8_t* able);
  int set_motion_enable_(int id, uint8_t able);
  int get_brake_enable_(int id, uint8_t* able);
  int set_brake_enable_(int id, uint8_t able);

  int get_temp_driver_(int id, float* temp);
  int get_temp_motor_(int id, float* temp);
  int get_bus_volt_(int id, float* volt);
  int get_bus_curr_(int id, float* curr);
  int get_multi_volt_(int id, float* volt);
  int get_error_code_(int id, uint8_t* errcode);

  int get_pos_target_(int id, float* pos);
  int set_pos_target_(int id, float pos);
  int get_pos_current_(int id, float* pos);
  int get_pos_limit_min_(int id, float* pos);
  int set_pos_limit_min_(int id, float pos);
  int get_pos_limit_max_(int id, float* pos);
  int set_pos_limit_max_(int id, float pos);
  int get_pos_limit_diff_(int id, float* pos);
  int set_pos_limit_diff_(int id, float pos);
  int get_pos_pidp_(int id, float* p);
  int set_pos_pidp_(int id, float p);
  int get_pos_smooth_cyc_(int id, uint8_t* cyc);
  int set_pos_smooth_cyc_(int id, uint8_t cyc);
  int get_pos_adrc_param_(int id, uint8_t i, float* param);
  int set_pos_adrc_param_(int id, uint8_t i, float param);
  int pos_cal_zero_(int id);

  int get_vel_target_(int id, float* vel);
  int set_vel_target_(int id, float vel);
  int get_vel_current_(int id, float* vel);
  int get_vel_limit_min_(int id, float* vel);
  int set_vel_limit_min_(int id, float vel);
  int get_vel_limit_max_(int id, float* vel);
  int set_vel_limit_max_(int id, float vel);
  int get_vel_limit_diff_(int id, float* vel);
  int set_vel_limit_diff_(int id, float vel);
  int get_vel_pidp_(int id, float* p);
  int set_vel_pidp_(int id, float p);
  int get_vel_pidi_(int id, float* i);
  int set_vel_pidi_(int id, float i);
  int get_vel_smooth_cyc_(int id, uint8_t* cyc);
  int set_vel_smooth_cyc_(int id, uint8_t cyc);
  int get_vel_adrc_param_(int id, uint8_t i, float* param);
  int set_vel_adrc_param_(int id, uint8_t i, float param);

  int get_tau_target_(int id, float* tau);
  int set_tau_target_(int id, float tau);
  int get_tau_current_(int id, float* tau);
  int get_tau_limit_min_(int id, float* tau);
  int set_tau_limit_min_(int id, float tau);
  int get_tau_limit_max_(int id, float* tau);
  int set_tau_limit_max_(int id, float tau);
  int get_tau_limit_diff_(int id, float* tau);
  int set_tau_limit_diff_(int id, float tau);
  int get_tau_pidp_(int id, float* p);
  int set_tau_pidp_(int id, float p);
  int get_tau_pidi_(int id, float* i);
  int set_tau_pidi_(int id, float i);
  int get_tau_smooth_cyc_(int id, uint8_t* cyc);
  int set_tau_smooth_cyc_(int id, uint8_t cyc);
  int get_tau_adrc_param_(int id, uint8_t i, float* param);
  int set_tau_adrc_param_(int id, uint8_t i, float param);

  int set_cpos_target_(uint8_t sid, uint8_t eid, float* pos);
  int get_spostau_current_(int id, int* num, float* pos, float* tau);
  int get_cpostau_current_(uint8_t sid, uint8_t eid, int* num, float* pos, float* tau, int* ret);

 protected:
  uint8_t id_;
  SERVO_REG reg_;

 private:
  uint8_t bus_type_;
  Socket* socket_fp_ = NULL;
  UtrcClient* utrc_client_ = NULL;
  utrc_t utrc_tx_;
  utrc_t utrc_rx_;
  pthread_mutex_t mutex_;

  void send(uint8_t rw, uint8_t cmd, uint8_t cmd_data_len, uint8_t* cmd_data);
  int pend(uint8_t rx_len, float timeout_s = 0.001);
  int sendpend(int id, uint8_t rw, const uint8_t cmd[5], uint8_t* tx_data, float timeout_s = 0.02);

  int get_reg_int8(int id, uint8_t* value, const uint8_t reg[5]);
  int set_reg_int8(int id, uint8_t* value, const uint8_t reg[5]);
  int get_reg_int32(int id, int* value, const uint8_t reg[5]);
  int set_reg_int32(int id, int value, const uint8_t reg[5]);
  int get_reg_fp32(int id, float* value, const uint8_t reg[5]);
  int set_reg_fp32(int id, float value, const uint8_t reg[5]);

  int get_reg_int8(int id, int8_t* value1, int8_t* value2, const uint8_t reg[5]);
  int set_reg_int8(int id, uint8_t value, const uint8_t reg[5]);
  int get_reg_int8(int id, uint8_t* value1, uint8_t* value2, const uint8_t reg[5]);
  int set_reg_int8(int id, uint8_t value1, uint8_t value2, const uint8_t reg[5]);
};

class SERVO_RW {
 public:
  const static uint8_t R = 0;
  const static uint8_t W = 1;
};

class BUS_TYPE {
 public:
  const static uint8_t CANBUS = 1;
  const static uint8_t UTRC = 2;
};

#define SERVO_FP_TO_INT(a) ((a)*100000)
#define SERVO_INT_TO_FP(a) (float(a) * 0.00001)

#endif
