/* Copyright 2021 Umbratek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#include "common/socket_tcp.h"
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include "common/linuxcvl.h"

SocketTcp::SocketTcp(char *ip, int port, int rxque_max, SerialDecode *decode, int rxlen_max, int priority) {
  fp_ = LinuxCvl::socket_init((char *)" ", 0, 0);
  if (fp_ == -1) {
    is_error_ = true;
    return;
  }

  int ret = LinuxCvl::socket_connect_server(&fp_, ip, port);
  if (ret != 0) {
    printf("[SockeTcp] Error tcp socket failed, ret = %d ip:%s port:%d\n", ret, ip, port);
    is_error_ = true;
    return;
  }
  is_error_ = false;
  rx_que_ = new BlockDeque<serial_stream_t>(rxque_max);
  decode_ = decode;
  rxlen_max_ = rxlen_max;
  if (decode_ == NULL)
    is_decode_ = false;
  else
    is_decode_ = true;

  flush();
  recv_task_ = new RtPeriodicMemberFun<SocketTcp>(0, "recv_task", 1024 * 1024, priority, &SocketTcp::recv_proc, this);
  recv_task_->start();
}

SocketTcp::~SocketTcp(void) {
  is_error_ = true;
  delete recv_task_;
  delete rx_que_;
}

bool SocketTcp::is_error(void) { return is_error_; }

void SocketTcp::close_port(void) {
  is_error_ = true;
  close(fp_);
}

void SocketTcp::flush(bool is_decode) {
  is_decode_ = is_decode;
  rx_que_->flush();
  if (decode_ != NULL && is_decode_) decode_->flush();
}

void SocketTcp::flush(int slave_id, int master_id, int rxlen_max) {
  is_decode_ = true;
  rx_que_->flush();
  if (decode_ != NULL) decode_->flush(slave_id, master_id, rxlen_max);
}

int SocketTcp::write_frame(serial_stream_t *data) {
  if (is_error_) return -1;
  return LinuxCvl::socket_send_data(fp_, data->data, data->len);
}

int SocketTcp::read_frame(serial_stream_t *data, float timeout_s) {
  if (is_error_) return -1;
  return rx_que_->pop(data, timeout_s);
}

void SocketTcp::recv_proc(void) {
  unsigned char ch[rxlen_max_];
  int ret;
  while (is_error_ == false) {
    bzero(ch, rxlen_max_);
    ret = recv(fp_, ch, rxlen_max_, 0);
    if (ret <= 0) {
      close(fp_);
      printf("[SockeTcp] recv_proc exit\n");
      pthread_exit(0);
      return;
    }

    if (decode_ != NULL && is_decode_) {
      decode_->parse_put(ch, ret, rx_que_);
    } else {
      serial_stream_.len = ret;
      memcpy(serial_stream_.data, ch, serial_stream_.len);
      rx_que_->push_back(&serial_stream_);
    }
  }
}