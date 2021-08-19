/* Copyright 2020 UmbraTek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#ifndef __ARM_REPORT_CONFIG_H__
#define __ARM_REPORT_CONFIG_H__

#include "base/arm_reg.h"
#include "common/periodic_rt.h"
#include "common/socket.h"

#pragma pack(1)
typedef struct _report_conf_t {
  uint16_t len;
  float trs_maxacc;
  float trs_jerk;
  float rot_maxacc;
  float rot_jerk;
  float p2p_maxacc;
  float p2p_jerk;
  float tcp_offset[6];
  float tcp_load[4];
  float gravity_dir[3];
  uint8_t colli_sens;
  uint8_t teach_sens;
} arm_report_config_t;
#pragma pack()

class ArmReportConfig {
 public:
  ArmReportConfig(void);
  ArmReportConfig(Socket* socket_fp);
  ~ArmReportConfig(void);
  void arminit(Socket* socket_fp);

  void close(void);
  bool is_error(void);
  bool is_update(void);
  void print_data(void);
  void print_data(arm_report_config_t* rx_data);
  int get_data(arm_report_config_t* rx_data);

 private:
  bool is_error_ = false;
  Socket* socket_fp_ = NULL;

  int rxcnt_ = 0;
  int frame_len = 0;
  bool is_update_ = 0;
  serial_stream_t rxdata_;
  int report_flag_ = 0;
  arm_report_config_t report_config_[3];
  RtPeriodicMemberFun<ArmReportConfig>* recv_task_ = NULL;

  void flush_data(uint8_t* rx_data, int len);
  void recv_proc(void);
};

#endif
