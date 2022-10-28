/* Copyright 2020 UmbraTek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#include "utra/utra_report_config.h"

UtraReportConfig10Hz::UtraReportConfig10Hz(char* ip) {
  socket_tcp_ = new SocketTcp(ip, 30003, 16, NULL, 4096, 45);
  if (socket_tcp_->is_error()) {
    printf("[UtraReCo] Error: socket_file open failed, %s\n", ip);
    return;
  }
  arminit(socket_tcp_);
  sleep(1);
}

UtraReportConfig10Hz::~UtraReportConfig10Hz(void) {
  if (socket_tcp_ != NULL) {
    socket_tcp_->close_port();
    delete socket_tcp_;
  }
}
