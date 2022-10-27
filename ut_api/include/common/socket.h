/* Copyright 2021 Umbratek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#ifndef __COMMON_SOCKET_H__
#define __COMMON_SOCKET_H__

#include <string.h>

#include "deque_block.h"

#define SERIAL_DATA_MAX 2560
typedef struct {
  int len;
  unsigned char data[SERIAL_DATA_MAX];
} serial_stream_t;

class SerialDecode {
 public:
  SerialDecode(void) {}
  virtual ~SerialDecode(void) {}

  virtual void flush(void) = 0;
  virtual void flush(int params1, int params2, int params3) = 0;
  virtual serial_stream_t* parse_put(unsigned char* data, int len, BlockDeque<serial_stream_t>* rx_que) = 0;
  virtual serial_stream_t* parse_put_fast(unsigned char* data, int len, BlockDeque<serial_stream_t>* rx_que) = 0;
};

class Socket {
 public:
  Socket(void) {}
  virtual ~Socket(void) {}
  virtual bool is_error(void) = 0;
  virtual void close_port(void) = 0;
  virtual void flush(bool is_decode = true) = 0;
  virtual void flush(int params1, int params2, int params3) = 0;

  virtual int write_frame(serial_stream_t* data) = 0;
  virtual int read_frame(serial_stream_t* data, float timeout_s = 0) = 0;
};

#endif
