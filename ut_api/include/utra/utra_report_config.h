/* Copyright 2020 UmbraTek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#ifndef __UTRA_REPORT_CONFIG_H__
#define __UTRA_REPORT_CONFIG_H__

#include "base/arm_report_config.h"
#include "common/socket_tcp.h"

class UtraReportConfig10Hz : public ArmReportConfig {
 public:
  UtraReportConfig10Hz(char *ip);
  ~UtraReportConfig10Hz(void);

 private:
  SocketTcp *socket_tcp_ = NULL;
};

#endif
