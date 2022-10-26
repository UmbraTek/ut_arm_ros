/* Copyright 2021 Umbratek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#ifndef __COMMON_PORT_SERIAL_H__
#define __COMMON_PORT_SERIAL_H__

#include <pthread.h>
#include "periodic_rt.h"
#include "socket.h"

class SocketSerial : public Socket {
 public:
  SocketSerial(const char* port, int baud, int rxque_max, SerialDecode* decode, int rxlen_max, int priority);
  ~SocketSerial(void);
  bool is_error(void);
  void close_port(void);
  void flush(bool is_decode = true);
  void flush(int slave_id, int master_id, int rxlen_max);

  int write_frame(serial_stream_t* data);
  int read_frame(serial_stream_t* data, float timeout_s = 0);

 private:
  int fp_;
  int is_error_;
  int rxlen_max_;
  bool is_decode_ = true;
  SerialDecode* decode_ = NULL;
  serial_stream_t rx_stream_;
  BlockDeque<serial_stream_t>* rx_que_ = NULL;
  RtPeriodicMemberFun<SocketSerial>* recv_task_ = NULL;

  int init_serial(const char* port, int baud);
  void recv_proc(void);
};

#endif
