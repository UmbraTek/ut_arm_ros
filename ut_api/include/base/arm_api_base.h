/* Copyright 2020 UmbraTek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#ifndef __ARM_API_BASE_H__
#define __ARM_API_BASE_H__

#include "base/arm_reg.h"
#include "common/utrc_t.h"

class ArmApiBase {
 public:
  ArmApiBase(void);
  ArmApiBase(Socket* socket_fp);
  ~ArmApiBase(void);

  void arminit(Socket* socket_fp);
  bool is_error(void);
  int get_axis(uint8_t* axis);

  int get_uuid(uint8_t uuid[17]);
  int get_sw_version(uint8_t version[20]);
  int get_hw_version(uint8_t version[20]);
  int get_sys_autorun(uint8_t* autorun);
  int set_sys_autorun(uint8_t autorun);
  int shutdown_system(void);
  int reset_err(void);
  int reboot_system(void);
  int erase_parm(void);
  int saved_parm(void);

  int get_motion_mode(uint8_t* mode);
  int set_motion_mode(uint8_t mode);
  int into_motion_mode_pos(void);
  int into_motion_mode_teach(void);
  int get_motion_enable(int* able);
  int set_motion_enable(uint8_t axis, uint8_t en);
  int into_motion_enable(void);
  int into_motion_disable(void);
  int get_brake_enable(int* able);
  int set_brake_enable(uint8_t axis, uint8_t en);
  int get_error_code(uint8_t* code);
  int get_servo_msg(uint8_t* msg);
  int get_motion_status(uint8_t* status);
  int set_motion_status(uint8_t status);
  int motion_status_into_stop(void);
  int motion_status_into_ready(void);
  int motion_status_into_pause(void);
  int get_cmd_num(int* num);
  int set_cmd_num(int num);

  int moveto_cartesian_line(float* mvpose, float mvvelo, float mvacc, float mvtime);
  int moveto_cartesian_lineb(float* mvpose, float mvvelo, float mvacc, float mvtime, float mvradii);
  int moveto_cartesian_p2p(float* mvpose, float mvvelo, float mvacc, float mvtime);
  int moveto_cartesian_p2pb(void);
  int moveto_cartesian_circle(float* pose1, float* pose2, float mvvelo, float mvacc, float mvtime, float percent);
  int moveto_joint_line(float* mvjoint, float mvvelo, float mvacc, float mvtime);
  int moveto_joint_lineb(float* mvjoint, float mvvelo, float mvacc, float mvtime, float mvradii);
  int moveto_joint_p2p(float* mvjoint, float mvvelo, float mvacc, float mvtime);
  int moveto_joint_circle(float* mvjoint1, float* mvjoint2, float mvvelo, float mvacc, float mvtime, float percent);
  int moveto_home_p2p(float mvvelo, float mvacc, float mvtime);
  int moveto_servo_joint(int frames_num, float* mvjoint, float* mvtime);
  int moveto_joint_servo(int frames_num, float* mvjoint, float* mvtime);
  int moveto_cartesian_servo(int frames_num, float* mvpose, float* mvtime);
  int move_sleep(float time);
  int plan_sleep(float time);

  int get_tcp_jerk(float* jerk);
  int set_tcp_jerk(float jerk);
  int get_tcp_maxacc(float* acc);
  int set_tcp_maxacc(float acc);
  int get_joint_jerk(float* jerk);
  int set_joint_jerk(float jerk);
  int get_joint_maxacc(float* acc);
  int set_joint_maxacc(float acc);
  int get_tcp_offset(float* offset);
  int set_tcp_offset(float* offset);
  int get_tcp_load(float* load);
  int set_tcp_load(float* load);
  int get_gravity_dir(float* dir);
  int set_gravity_dir(float* dir);
  int get_collis_sens(uint8_t* sens);
  int set_collis_sens(uint8_t sens);
  int get_teach_sens(uint8_t* sens);
  int set_teach_sens(uint8_t sens);
  int get_limit_fun(int* fun);
  int set_limit_fun(int fun);
  int set_limit_angle_enable(int en);
  int set_limit_geometry_enable(int en);
  int get_tcp_target_pos(float* pos);
  int get_tcp_actual_pos(float* pos);
  int get_joint_target_pos(float* pos);
  int get_joint_actual_pos(float* pos);
  int get_ik(float* pose, float* qnear, float* joints);
  int get_fk(float* joints, float* pose);
  int is_joint_limit(void);
  int is_tcp_limit(void);

  int get_friction(uint8_t axis, float* value);
  int set_friction(uint8_t axis, float* fri);

  int get_utrc_int8_now(uint8_t line, uint8_t id, uint8_t reg, uint8_t* value);
  int set_utrc_int8_now(uint8_t line, uint8_t id, uint8_t reg, uint8_t value);
  int get_utrc_int32_now(uint8_t line, uint8_t id, uint8_t reg, int32_t* value);
  int set_utrc_int32_now(uint8_t line, uint8_t id, uint8_t reg, int32_t value);
  int get_utrc_float_now(uint8_t line, uint8_t id, uint8_t reg, float* value);
  int set_utrc_float_now(uint8_t line, uint8_t id, uint8_t reg, float value);
  int get_utrc_int8n_now(uint8_t line, uint8_t id, uint8_t reg, uint8_t* data, uint8_t rx_len);
  int set_utrc_int8n_now(uint8_t line, uint8_t id, uint8_t reg, uint8_t* data, uint8_t tx_len);

  int set_utrc_int8_que(uint8_t line, uint8_t id, uint8_t reg, uint8_t value);
  int set_utrc_int32_que(uint8_t line, uint8_t id, uint8_t reg, int32_t value);
  int set_utrc_float_que(uint8_t line, uint8_t id, uint8_t reg, float value);
  int set_utrc_int8n_que(uint8_t line, uint8_t id, uint8_t reg, uint8_t* data, uint8_t tx_len);

  int set_pass_rs485_now(uint8_t line, uint8_t timeout_ms, uint8_t tx_len, uint8_t rx_len, uint8_t* tx_data, uint8_t* rx_data);
  int set_pass_rs485_que(uint8_t line, uint8_t tx_len, uint8_t* tx_data);

  int get_utrc_u8float_now(uint8_t line, uint8_t id, uint8_t reg, uint8_t num, float* value);
  int set_utrc_u8float_now(uint8_t line, uint8_t id, uint8_t reg, uint8_t num, float value);
  int get_utrc_nfloat_now(uint8_t line, uint8_t id, uint8_t reg, uint8_t num, float* value);

  int get_gpio_in(uint8_t line, uint8_t id, int32_t* fun, int32_t* digit, int* adc_num, float* adc_value);
  int get_gpio_out(uint8_t line, uint8_t id, int32_t* fun, int32_t* digit, int* adc_num, float* adc_value);

  int get_tgpio_in(int32_t* fun, int32_t* digit, int* adc_num, float* adc_value);
  int get_tgpio_out(int32_t* fun, int32_t* digit, int* adc_num, float* adc_value);
  int set_tgpio_digit_out(int value);
  int get_tgpio_uuid(uint8_t* value);
  int get_tgpio_sw_version(uint8_t* value);
  int get_tgpio_hw_version(uint8_t* value);

  int get_cgpio_in(int32_t* fun, int32_t* digit, int* adc_num, float* adc_value);
  int get_cgpio_out(int32_t* fun, int32_t* digit, int* adc_num, float* adc_value);
  int set_cgpio_digit_out(int value);
  int get_cgpio_uuid(uint8_t* value);
  int get_cgpio_sw_version(uint8_t* value);
  int get_cgpio_hw_version(uint8_t* value);

 protected:
  ARM_REG* reg_;

 private:
  utrc_t utrc_tx_;
  utrc_t utrc_rx_;
  Socket* socket_fp_ = NULL;
  UtrcClient* utrc_client_ = NULL;

  uint8_t axis_ = 0;
  int is_error_ = false;
  pthread_mutex_t mutex_;

  void send(uint8_t rw, uint8_t cmd, uint8_t cmd_data_len, uint8_t* cmd_data);
  int pend(uint8_t rx_len, float timeout_s = 0.001);
  int sendpend(uint8_t rw, const uint8_t cmd[5], uint8_t* tx_data, float timeout_s = 0.1);

  int get_reg_int8(uint8_t* value, const uint8_t reg[5]);
  int set_reg_int8(uint8_t* value, const uint8_t reg[5]);
  int get_reg_int32(int* value, const uint8_t reg[5], int n = 1);
  int set_reg_int32(int* value, const uint8_t reg[5], int n = 1);
  int get_reg_fp32(float* value, const uint8_t reg[5], int n = 1);
  int set_reg_fp32(float* value, const uint8_t reg[5], int n = 1);
  int get_reg_fp32_fp32(const uint8_t reg[5], float* txdate, int tx_n, float* rxdate, int rx_n);
  int get_reg_int8_fp32(const uint8_t reg[5], float* txdate, int tx_n, float* rxdate, int rx_n);
};

class ARM_RW {
 public:
  const static uint8_t R = 0;
  const static uint8_t W = 1;
};

#endif
