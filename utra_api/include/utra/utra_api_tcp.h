/* Copyright 2020 UmbraTek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#ifndef __UTRA_API_SERIAL_H__
#define __UTRA_API_SERIAL_H__

#include "base/arm_api_base.h"
#include "common/socket_tcp.h"

class UtraApiTcp : public ArmApiBase {
 public:
  UtraApiTcp(char *ip);
  ~UtraApiTcp(void);

 private:
  UtrcDecode *utrc_decode_ = NULL;
  SocketTcp *socket_tcp_ = NULL;
};

#endif
