/* Copyright 2020 UmbraTek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#ifndef __ARM_REPORT_STATUS_H__
#define __ARM_REPORT_STATUS_H__

#include "base/arm_reg.h"
#include "common/periodic_rt.h"
#include "common/socket.h"

#pragma pack(1)
typedef struct _report_status_t {
  uint16_t len;
  uint8_t axis;
  uint8_t motion_status;
  uint8_t motion_mode;
  uint32_t mt_brake;
  uint32_t mt_able;
  uint8_t err_code;
  uint8_t war_code;
  uint16_t cmd_num;
  float joint[32];
  float pose[6];
  float tau[32];
} arm_report_status_t;
#pragma pack()

class ArmReportStatus {
 public:
  ArmReportStatus(void);
  ArmReportStatus(Socket* socket_fp, int axis);
  ~ArmReportStatus(void);
  void arminit(Socket* socket_fp, int axis);

  void close(void);
  bool is_error(void);
  bool is_update(void);
  void print_data(void);
  void print_data(arm_report_status_t* rx_data);
  int get_data(arm_report_status_t* rx_data);

 private:
  uint8_t axis_ = 0;
  bool is_error_ = false;
  Socket* socket_fp_ = NULL;

  int rxcnt_ = 0;
  int frame_len = 0;
  bool is_update_ = 0;
  serial_stream_t rxdata_;
  int report_flag_ = 0;
  arm_report_status_t report_status_[3];
  RtPeriodicMemberFun<ArmReportStatus>* recv_task_ = NULL;

  void flush_data(uint8_t* rx_data, int len);
  void recv_proc(void);
};

#endif
