/* Copyright 2021 Umbratek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#include "common/socket_serial.h"
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/shm.h>
#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>
#include "common/linuxcvl.h"
#include "common/print.h"

SocketSerial::SocketSerial(const char *port, int baud, int rxque_max, SerialDecode *decode, int rxlen_max,
                           int priority) {
  int ret = init_serial(port, baud);
  if (ret != 0) {
    printf("[SockeSer] Error serial init failed, ret = %d\n", ret);
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
  recv_task_ =
      new RtPeriodicMemberFun<SocketSerial>(0, "recv_task", 1024 * 1024, priority, &SocketSerial::recv_proc, this);
  recv_task_->start();
}

SocketSerial::~SocketSerial(void) {
  is_error_ = true;
  close(fp_);
  recv_task_->stop();
  delete recv_task_;
  delete rx_que_;
}

bool SocketSerial::is_error(void) { return is_error_; }

void SocketSerial::close_port(void) {
  is_error_ = true;
  close(fp_);
}

void SocketSerial::flush(bool is_decode) {
  is_decode_ = is_decode;
  rx_que_->flush();
  if (decode_ != NULL && is_decode_) decode_->flush();
}

void SocketSerial::flush(int slave_id, int master_id, int rxlen_max) {
  is_decode_ = true;
  rx_que_->flush();
  if (decode_ != NULL) decode_->flush(slave_id, master_id, rxlen_max);
}

int SocketSerial::write_frame(serial_stream_t *data) {
  if (is_error_) return -1;
  if (write(fp_, data->data, data->len) != data->len) return -1;
  return 0;
}

int SocketSerial::read_frame(serial_stream_t *data, float timeout_s) {
  if (is_error_) return -1;
  return rx_que_->pop(data, timeout_s);
}

void SocketSerial::recv_proc(void) {
  unsigned char ch[rxlen_max_];
  int ret;
  while (is_error_ == false) {
    ret = read(fp_, ch, rxlen_max_);
    if (is_error_) return;
    if (ret >= 0) {
      if (decode_ != NULL && is_decode_) {
        decode_->parse_put(ch, ret, rx_que_);
      } else {
        serial_stream_.len = ret;
        memcpy(serial_stream_.data, ch, serial_stream_.len);
        rx_que_->push_back(&serial_stream_);
      }
    }
  }
}

int SocketSerial::init_serial(const char *port, int baud) {
  fp_ = open((const char *)port, O_RDWR | O_NOCTTY | O_NDELAY);
  if (-1 == fp_) return -1;

  int flags = fcntl(fp_, F_GETFL, 0);
  flags &= ~O_NONBLOCK;
  if (fcntl(fp_, F_SETFL, flags) < 0) {
    printf("[SockeSer] fcntl failed.\n");
    return -1;
  }

  speed_t speed = B115200;
  switch (baud) {
    case 110:
      speed = B110;
      break;
    case 300:
      speed = B300;
      break;
    case 600:
      speed = B600;
      break;
    case 1200:
      speed = B1200;
      break;
    case 2400:
      speed = B2400;
      break;
    case 4800:
      speed = B4800;
      break;
    case 9600:
      speed = B9600;
      break;
    case 19200:
      speed = B19200;
      break;
    case 38400:
      speed = B38400;
      break;
    case 57600:
      speed = B57600;
      break;
    case 115200:
      speed = B115200;
      break;
    case 230400:
      speed = B230400;
      break;
    case 460800:
      speed = B460800;
      break;
    case 921600:
      speed = B921600;
      break;
    case 1000000:
      speed = B1000000;
      break;
    case 2000000:
      speed = B2000000;
      break;
  }

  struct termios options;
  struct termios oldtio;
  bzero(&options, sizeof(options));
  bzero(&oldtio, sizeof(oldtio));

  int ret = tcgetattr(fp_, &oldtio);
  if (ret != 0) return -2;

  options.c_cflag |= CLOCAL | CREAD;
  options.c_cflag &= ~CSIZE;

  //  set tty speed
  ret = cfsetispeed(&options, speed);
  if (ret != 0) return -3;
  ret = cfsetospeed(&options, speed);
  if (ret != 0) return -4;

  //  data bits 8
  options.c_cflag |= CS8;

  // parity N
  options.c_cflag &= ~PARENB;  // parity = N , Clear parity enable
  options.c_iflag &= ~INPCK;   // parity = N , Enable parity checking

  // stop bits=1
  options.c_cflag &= ~CSTOPB;

  int hardflow = 0;
  if (hardflow) {
    options.c_cflag |= CRTSCTS;
  } else {
    options.c_cflag &= ~CRTSCTS;
  }

  options.c_cc[VTIME] = 100;  // Time-out value (tenths of a second) [!ICANON].
  options.c_cc[VMIN] = 1;     // Minimum number of bytes read at once [!ICANON].

  // options.c_iflag &= ~(INLCR | ICRNL);         //不要回车和换行转换
  // options.c_iflag &= ~(IXON | IXOFF | IXANY);  //不要软件流控制
  // options.c_oflag &= ~OPOST;
  // options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  //原始模式
  // options.c_cflag &= ~CRTSCTS;

  tcflush(fp_, TCIFLUSH);
  ret = tcsetattr(fp_, TCSANOW, &options);
  if (ret != 0) return -5;
  return 0;
}

// g++ linux/port/serial.cpp common/crc16.cpp linux/thread.cc -o serial123 -I./ -lpthread
/*
int main(int argc, char *argv[]) {
  SocketSerial serial_port("/dev/ttyUSB0", 115200, 20, 32);  //

  serial_stream_t rx_data;
  while (true) {
    int ret = serial_port.read_frame(&rx_data);
    // int ret;
    printf("rx:%d, ret:%d\n", rx_data.len, ret);
    for (int i = 0; i < rx_data.len; i++) printf("0x%x ", rx_data.data[i]);
    printf("\n");

    serial_port.write_frame(&rx_data);
  }
}*/
