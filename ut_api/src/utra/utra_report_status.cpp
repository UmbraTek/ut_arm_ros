/* Copyright 2020 UmbraTek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#include "utra/utra_report_status.h"

UtraReportStatus10Hz::UtraReportStatus10Hz(char* ip, int axis) {
  socket_tcp_ = new SocketTcp(ip, 30001, 16, NULL, 4096, 45);
  if (socket_tcp_->is_error()) {
    printf("[UtraReSt] Error: socket_file open failed, %s\n", ip);
    return;
  }
  arminit(socket_tcp_, axis);
  sleep(1);
}

UtraReportStatus10Hz::~UtraReportStatus10Hz(void) {
  if (socket_tcp_ != NULL) {
    socket_tcp_->close_port();
    delete socket_tcp_;
  }
}

UtraReportStatus100Hz::UtraReportStatus100Hz(char* ip, int axis) {
  socket_tcp_ = new SocketTcp(ip, 30002, 1600, NULL, 40960, 45);
  if (socket_tcp_->is_error()) {
    printf("[UtraReSt] Error: socket_file open failed, %s\n", ip);
    return;
  }
  arminit(socket_tcp_, axis);
  sleep(1);
}

UtraReportStatus100Hz::~UtraReportStatus100Hz(void) {
  if (socket_tcp_ != NULL) {
    socket_tcp_->close_port();
    delete socket_tcp_;
  }
}