/* Copyright 2020 UmbraTek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#include "utra/utra_api_tcp.h"

UtraApiTcp::UtraApiTcp(char* ip) {
  utrc_decode_ = new UtrcDecode(0xAA, 0x55, 128);

  socket_tcp_ = new SocketTcp(ip, 502, 16, utrc_decode_, 128, 45);
  if (socket_tcp_->is_error()) {
    printf("[UtraApiT] Error: socket_file open failed, %s\n", ip);
    return;
  }

  arminit(socket_tcp_);
  sleep(1);
}

UtraApiTcp::~UtraApiTcp(void) {
  if (socket_tcp_ != NULL) {
    socket_tcp_->close_port();
    delete socket_tcp_;
  }
  if (utrc_decode_ != NULL) delete utrc_decode_;
}