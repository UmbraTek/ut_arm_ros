/* Copyright 2020 UmbraTek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#ifndef __UTRA_REPORT_STATUS_H__
#define __UTRA_REPORT_STATUS_H__

#include "base/arm_report_status.h"
#include "common/socket_tcp.h"

class UtraReportStatus10Hz : public ArmReportStatus {
 public:
  UtraReportStatus10Hz(char *ip, int axis);
  ~UtraReportStatus10Hz(void);

 private:
  SocketTcp *socket_tcp_ = NULL;
};

class UtraReportStatus100Hz : public ArmReportStatus {
 public:
  UtraReportStatus100Hz(char *ip, int axis);
  ~UtraReportStatus100Hz(void);

 private:
  SocketTcp *socket_tcp_ = NULL;
};

#endif
