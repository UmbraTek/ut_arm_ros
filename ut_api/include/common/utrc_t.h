/* Copyright 2020 UmbraTek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#ifndef __COMMON_UTRC_T_H__
#define __COMMON_UTRC_T_H__

#include <stddef.h>
#include <stdio.h>
#include "crc16.h"
#include "hex_data.h"
#include "print.h"
#include "socket.h"


class UTRC_ERROR {
 public:
  UTRC_ERROR(void){};
  ~UTRC_ERROR(void){};

  static const int M_ID = -1;
  static const int S_ID = -2;
  static const int TIMEOUT = -3;
  static const int STATE = -4;
  static const int LEN = -5;
  static const int RW = -6;
  static const int CMD = -7;
  static const int CRC = -8;
  static const int CONNECT = -9;
  static const int LEN_MIN = -10;
};

// 优先级
class UTRC_PR {
 public:
  static const uint8_t HIGHT = 0x0A;
  static const uint8_t MID = 0x0B;
  static const uint8_t LOW = 0x0C;
  static const uint8_t NONE = 0x0B;
};

class UTRC_RW {
 public:
  static const uint8_t OR = 0x00;
  static const uint8_t OW = 0x01;
  static const uint8_t RW = 0x02;

  const static uint8_t R = 0x00;
  const static uint8_t W = 0x01;
};

typedef struct {
  uint8_t master_id;
  uint8_t slave_id;
  uint8_t state : 1;
  uint8_t len : 7;
  uint8_t rw : 1;
  uint8_t cmd : 7;

  uint8_t data[128];
  uint16_t crc;

  int8_t intf_type;
  int intf_fp;

  int pack(serial_stream_t* serial_stream) {
    serial_stream->data[0] = master_id;
    serial_stream->data[1] = slave_id;
    serial_stream->data[2] = ((state & 0x01) << 7) + (len & 0x7F);
    serial_stream->data[3] = ((rw & 0x01) << 7) + (cmd & 0x7F);

    memcpy(&serial_stream->data[4], data, len - 1);

    crc = Crc::modbus(serial_stream->data, len + 3);
    serial_stream->data[len + 3] = crc & 0xFF;
    serial_stream->data[len + 4] = (crc >> 8) & 0xFF;

    serial_stream->len = len + 5;
    return 0;
  }

  int unpack(serial_stream_t* serial_stream) {
    len = serial_stream->data[2] & 0x7F;
    if (serial_stream->len != len + 5) {
      printf("[UtrcTyp] Error 1:%d %d\n", serial_stream->len, len);
      return UTRC_ERROR::LEN;
    }

    master_id = serial_stream->data[0];
    slave_id = serial_stream->data[1];
    state = (serial_stream->data[2] & 0xFF) >> 7;
    rw = (serial_stream->data[3] & 0xFF) >> 7;
    cmd = serial_stream->data[3] & 0x7F;
    memcpy(data, &serial_stream->data[4], len - 1);

    return 0;
  }

  void print_pack(char* srt) {
    printf("\n%s utrc pack\n", srt);
    printf("  m_id : 0x%x\n", master_id);
    printf("  s_id : 0x%x\n", slave_id);
    printf("  state: %d\n", state);
    printf("  len  : %d\n", len);
    printf("  rw   : %d\n", rw);
    printf("  cmd  : 0x%x\n", cmd);
    Print::hex("  data : ", data, len - 1);
    printf("  crc  : 0x%x\n", crc);
    printf("  type : %d\n", intf_type);
  }
} utrc_t;

class UtrcClient {
 public:
  UtrcClient(Socket* port_fp) {
    pthread_mutex_init(&mutex_, NULL);
    port_fp_ = port_fp;
    if (port_fp_ != NULL) port_fp_->flush();
  }

  ~UtrcClient(void) {}

  int connect_device(uint32_t baud = 0xFFFFFFFF) {
    utrc_t tx_utrc;
    tx_utrc.master_id = 0xAA;
    tx_utrc.slave_id = 0x55;
    tx_utrc.state = 0;
    tx_utrc.len = 0x08;
    tx_utrc.rw = 0;
    tx_utrc.cmd = 0x7F;
    tx_utrc.data[0] = 0x7F;
    tx_utrc.data[1] = 0x7F;
    tx_utrc.data[2] = 0x7F;
    tx_utrc.data[3] = 0x7F;
    tx_utrc.data[4] = 0x7F;
    tx_utrc.data[5] = 0x7F;
    tx_utrc.data[6] = 0x7F;
    HexData::int32_to_hex_big(baud, &tx_utrc.data[0]);

    send(&tx_utrc);
    utrc_t rx_utrc;
    int ret = pend(&tx_utrc, 1, 1, &rx_utrc);
    if (ret != 0) return ret;
    return 0;
  }

  void send(utrc_t* tx_utrc) {
    tx_utrc->pack(&tx_stream_);

    pthread_mutex_lock(&mutex_);
    port_fp_->flush(tx_utrc->slave_id, tx_utrc->master_id, 64);
    port_fp_->write_frame(&tx_stream_);
    pthread_mutex_unlock(&mutex_);
  }

  int pend(utrc_t* tx_utrc, int r_len, float timeout_s, utrc_t* rx_utrc) {
    (void)r_len;
    int ret = UTRC_ERROR::TIMEOUT;

    pthread_mutex_lock(&mutex_);
    int temp = port_fp_->read_frame(&rx_stream_, timeout_s);
    pthread_mutex_unlock(&mutex_);
    if (temp == -1 || rx_stream_.len < 6) {
      // printf("[UxBusCli] Error 1: rx_stream_.len:%d %d\n", rx_stream_.len, temp);
      return ret;
    }

    ret = rx_utrc->unpack(&rx_stream_);
    if (ret != 0)
      return ret;
    else if (rx_utrc->master_id != tx_utrc->slave_id && tx_utrc->slave_id != 0x55) {
      printf("[UtrcCli] Error: UTRC_ERROR.M_ID: %d %d\n", rx_utrc->master_id, tx_utrc->slave_id);
      return UTRC_ERROR::M_ID;
    } else if (rx_utrc->slave_id != tx_utrc->master_id) {
      printf("[UtrcCli] Error: UTRC_ERROR.S_ID: %d %d\n", rx_utrc->slave_id, tx_utrc->master_id);
      return UTRC_ERROR::S_ID;
    } else if (rx_utrc->state != 0) {
      return UTRC_ERROR::STATE;
    } else if ((rx_utrc->len != r_len + 1) && (r_len != 0x55)) {
      printf("[UtrcCli] Error: UTRC_ERROR.LEN: %d %d\n", rx_utrc->len, r_len);
      return UTRC_ERROR::LEN;
    } else if (rx_utrc->rw != tx_utrc->rw) {
      printf("[UtrcCli] Error: UTRC_ERROR.RW: %d %d\n", rx_utrc->rw, tx_utrc->rw);
      return UTRC_ERROR::RW;
    } else if (rx_utrc->cmd != tx_utrc->cmd) {
      printf("[UtrcCli] Error: UTRC_ERROR.CMD: %d %d\n", rx_utrc->cmd, tx_utrc->cmd);
      return UTRC_ERROR::CMD;
    }
    return 0;
  }

 private:
  Socket* port_fp_ = NULL;
  serial_stream_t rx_stream_;
  serial_stream_t tx_stream_;
  pthread_mutex_t mutex_;
};

class UtrcDecode : public SerialDecode {
 public:
  UtrcDecode(int master_id, int slave_id, int data_maxlen) { flush(master_id, slave_id, data_maxlen); }
  ~UtrcDecode(void) {}

  void flush(void) {
    rx_data_idx_ = 0;
    rx_state_ = UTRC_START_FROMID;
  }

  void flush(int master_id, int slave_id, int data_maxlen) {
    rx_data_idx_ = 0;
    master_id_ = master_id;
    slave_id_ = slave_id;
    data_maxlen_ = (data_maxlen > SERIAL_DATA_MAX ? SERIAL_DATA_MAX : data_maxlen);
    rx_state_ = UTRC_START_FROMID;
    // rx_data_continue_ = false;
  }

  serial_stream_t* parse_put(unsigned char* data, int len, BlockDeque<serial_stream_t>* rx_que) {
    for (int i = 0; i < len; i++) {
      ch = data[i];
      // printf("[utrc_t] ---state = %d, ch = 0x%x 0x%x 0x%x\n", rx_state_, ch, master_id_, slave_id_);
      switch (rx_state_) {
        case UTRC_START_FROMID:
          if (master_id_ == ch || 0x55 == master_id_) {
            rx_buf_[0] = ch;
            rx_state_ = UTRC_START_TOOID;
          }
          break;

        case UTRC_START_TOOID:
          if (slave_id_ == ch) {
            rx_buf_[1] = ch;
            rx_state_ = UTRC_STATE_LENGTH;
          } else {
            rx_state_ = UTRC_START_FROMID;
          }
          break;

        case UTRC_STATE_LENGTH:
          rx_length_ = ch & 0x7F;
          if (0 < rx_length_ && rx_length_ < (data_maxlen_ - 5)) {
            rx_buf_[2] = ch;
            rx_length_ = rx_length_;
            rx_data_idx_ = 3;
            rx_state_ = UTRC_STATE_DATA;
          } else {
            rx_state_ = UTRC_START_FROMID;
          }
          break;

        case UTRC_STATE_DATA:
          if (rx_data_idx_ < rx_length_ + 3) {
            rx_buf_[rx_data_idx_++] = ch;
            if (rx_data_idx_ == rx_length_ + 3) {
              rx_state_ = UTRC_STATE_CRC1;
            }
          } else {
            rx_state_ = UTRC_START_FROMID;
          }
          break;

        case UTRC_STATE_CRC1:
          rx_buf_[rx_length_ + 3] = ch;
          rx_state_ = UTRC_STATE_CRC2;
          break;

        case UTRC_STATE_CRC2:
          rx_buf_[rx_length_ + 4] = ch;
          rx_state_ = UTRC_START_FROMID;

          crc1_ = Crc::modbus(rx_buf_, rx_length_ + 3);
          crc2_ = (rx_buf_[rx_length_ + 4] << 8) + rx_buf_[rx_length_ + 3];
          if (crc1_ == crc2_) {
            // printf("[utrc_t ] crc1_ ok\n");
            rx_data.len = rx_length_ + 4 + 1;
            memcpy(rx_data.data, rx_buf_, rx_data.len);
            if (rx_que != NULL) {
              rx_que->push_back(&rx_data);
              // printf("[utrc_t ] push_back ed\n");
            } else {
              return &rx_data;
            }
            // Print::hex("[utrc_t ] parse_put rx_data: ", rx_data.data, rx_data.len);
            // printf("[utrc_t ] parse_put rx_que.size:%ld %ld\n", rx_que->size(), (long)rx_que);
          }
          break;

        default:
          rx_state_ = UTRC_START_FROMID;
          break;
      }
    }
    return NULL;
  }

  serial_stream_t* parse_put_fast(unsigned char* data, int len, BlockDeque<serial_stream_t>* rx_que) {
    return parse_put(data, len, rx_que);
  }

 private:
  int data_maxlen_;
  unsigned char master_id_;
  unsigned char slave_id_;

  int rx_state_;
  typedef enum _UTRC_RECV_STATE {
    UTRC_START_FROMID = 0,
    UTRC_START_TOOID = 1,
    UTRC_STATE_LENGTH = 2,
    UTRC_STATE_DATA = 3,
    UTRC_STATE_CRC1 = 4,
    UTRC_STATE_CRC2 = 5,
  } UTRC_RECV_STATE;

  int rx_length_;
  int rx_data_idx_;
  unsigned char rx_buf_[128];
  serial_stream_t rx_data;

  unsigned char ch;
  uint16_t crc1_, crc2_;
};

#endif
