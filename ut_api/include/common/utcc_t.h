/* Copyright 2020 UmbraTek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#ifndef __COMMON_UTCC_T_H__
#define __COMMON_UTCC_T_H__

#include <stddef.h>
#include <stdio.h>
#include "crc16.h"
#include "hex_data.h"
#include "print.h"
#include "socket.h"

class UTCC_ERROR {
 public:
  UTCC_ERROR(void){};
  ~UTCC_ERROR(void){};

  static const int HEAD = -1;
  static const int ID = -2;
  static const int TIMEOUT = -3;
  static const int STATE = -4;
  static const int LEN = -5;
  static const int RW = -6;
  static const int CMD = -7;
  static const int CRC = -8;
  static const int CONNECT = -9;
  static const int LEN_MIN = -10;
};

class UTCC_RW {
 public:
  static const uint8_t OR = 0x00;
  static const uint8_t OW = 0x01;
  static const uint8_t RW = 0x02;

  const static uint8_t R = 0x00;
  const static uint8_t W = 0x01;
};

typedef struct {
  uint8_t head = 0xAA;
  uint16_t id;
  uint8_t state : 1;
  uint8_t len : 7;
  uint8_t rw : 1;
  uint8_t cmd : 7;

  uint8_t data[128];
  uint16_t crc;

  int8_t intf_type;
  int intf_fp;

  int pack(serial_stream_t* serial_stream) {
    serial_stream->data[0] = head;
    HexData::int16_to_hex_big(id, &serial_stream->data[1]);
    serial_stream->data[3] = ((state & 0x01) << 7) + (len & 0x7F);
    serial_stream->data[4] = ((rw & 0x01) << 7) + (cmd & 0x7F);

    memcpy(&serial_stream->data[5], data, len - 1);

    crc = Crc::modbus(serial_stream->data, len + 4);
    serial_stream->data[len + 4] = crc & 0xFF;
    serial_stream->data[len + 5] = (crc >> 8) & 0xFF;

    serial_stream->len = len + 6;
    return 0;
  }

  int unpack(serial_stream_t* serial_stream) {
    // Print::hex("[UtccTyp] : ", serial_stream->data, serial_stream->len);

    if (serial_stream->len < 7) {
      printf("[UtccTyp] Error UTCC_ERROR.LEN: %d\n", serial_stream->len);
      return UTCC_ERROR::LEN;
    }

    len = serial_stream->data[3] & 0x7F;
    if (serial_stream->len != len + 6) {
      printf("[UtccTyp] Error UTCC_ERROR.LEN: %d %d\n", serial_stream->len, len);
      return UTCC_ERROR::LEN;
    }

    if (head != serial_stream->data[0]) {
      printf("[UtccTyp] Error: UTCC_ERROR.HEAD: %d %d\n", head, serial_stream->data[0]);
      return UTCC_ERROR::HEAD;
    }

    id = HexData::hex_to_int16_big(&serial_stream->data[1]);
    state = (id & 0x80) >> 7;
    id = id & 0x7F;
    rw = (serial_stream->data[4] & 0xFF) >> 7;
    cmd = serial_stream->data[4] & 0x7F;
    memcpy(data, &serial_stream->data[5], len - 1);

    crc = Crc::modbus(serial_stream->data, len + 4);
    uint16_t crc2 = serial_stream->data[len + 4] + (serial_stream->data[len + 5] << 8);
    if (crc != crc2) {
      printf("[UtccTyp] Error: UTCC_ERROR.CRC: %d %d\n", crc, crc2);
      return UTCC_ERROR::CRC;
    }

    // print_pack("[UtccTyp] ");

    return 0;
  }

  void print_pack(char* srt) {
    printf("\n%s utcc pack\n", srt);
    printf("  m_id : 0x%x\n", head);
    printf("  s_id : 0x%x\n", id);
    printf("  state: %d\n", state);
    printf("  len  : %d\n", len);
    printf("  rw   : %d\n", rw);
    printf("  cmd  : 0x%x\n", cmd);
    Print::hex("  data : ", data, len - 1);
    printf("  crc  : 0x%x\n", crc);
    printf("  type : %d\n", intf_type);
  }
} utcc_t;

class UtccClient {
 public:
  UtccClient(Socket* port_fp) {
    pthread_mutex_init(&mutex_, NULL);
    port_fp_ = port_fp;
    if (port_fp_ != NULL) port_fp_->flush();
  }

  ~UtccClient(void) {}

  int connect_device(uint32_t baud = 0xFFFFFFFF) {
    utcc_t tx_utcc;
    tx_utcc.head = 0xAA;
    tx_utcc.id = 0x0055;
    tx_utcc.state = 0;
    tx_utcc.len = 0x08;
    tx_utcc.rw = 0;
    tx_utcc.cmd = 0x7F;
    tx_utcc.data[0] = 0x7F;
    tx_utcc.data[1] = 0x7F;
    tx_utcc.data[2] = 0x7F;
    tx_utcc.data[3] = 0x7F;
    tx_utcc.data[4] = 0x7F;
    tx_utcc.data[5] = 0x7F;
    tx_utcc.data[6] = 0x7F;
    HexData::int32_to_hex_big(baud, &tx_utcc.data[0]);

    send(&tx_utcc);
    utcc_t rx_utcc;
    int ret = pend(&tx_utcc, 1, 1, &rx_utcc);
    if (ret != 0) return ret;
    return 0;
  }

  void send(utcc_t* tx_utcc) {
    tx_utcc->pack(&tx_stream_);

    pthread_mutex_lock(&mutex_);
    port_fp_->flush();
    port_fp_->write_frame(&tx_stream_);
    pthread_mutex_unlock(&mutex_);
  }

  int pend_(utcc_t* tx_utcc, float timeout_s, utcc_t* rx_utcc) {
    int ret = UTCC_ERROR::TIMEOUT;

    pthread_mutex_lock(&mutex_);
    int temp = port_fp_->read_frame(&rx_stream_, timeout_s);
    pthread_mutex_unlock(&mutex_);
    if (temp == -1 || rx_stream_.len < 6) {
      // printf("[UxBusCli] Error 1: rx_stream_.len:%d %d\n", rx_stream_.len, temp);
      return ret;
    }

    ret = rx_utcc->unpack(&rx_stream_);
    if (ret != 0)
      return ret;
    else if (rx_utcc->id != tx_utcc->id) {
      printf("[UtccCli] Error: UTCC_ERROR.ID: %d %d\n", rx_utcc->id, tx_utcc->id);
      return UTCC_ERROR::ID;
    } else if (rx_utcc->state != 0) {
      return UTCC_ERROR::STATE;
    } else if (rx_utcc->rw != tx_utcc->rw) {
      printf("[UtccCli] Error: UTCC_ERROR.RW: %d %d\n", rx_utcc->rw, tx_utcc->rw);
      return UTCC_ERROR::RW;
    } else if (rx_utcc->cmd != tx_utcc->cmd) {
      printf("[UtccCli] Error: UTCC_ERROR.CMD: %d %d\n", rx_utcc->cmd, tx_utcc->cmd);
      return UTCC_ERROR::CMD;
    }
    return 0;
  }

  int pend(utcc_t* tx_utcc, int rx_len, float timeout_s, utcc_t* rx_utcc) {
    utcc_t rx_utcc1;
    rx_utcc1.head = 0;
    utcc_t rx_utcc2;
    while (1) {
      int ret = pend_(tx_utcc, timeout_s, &rx_utcc2);
      if (ret != 0 && ret != UTCC_ERROR::STATE) return ret;

      if (rx_utcc1.head == 0) {
        memcpy(&rx_utcc1, &rx_utcc2, sizeof(utcc_t));
      } else {
        memcpy(&rx_utcc1.data[rx_utcc1.len - 1], &rx_utcc2.data[0], rx_utcc2.len - 1);
        rx_utcc1.len = rx_utcc1.len + rx_utcc2.len - 1;
      }

      if (rx_utcc1.len == rx_len + 1) {
        memcpy(rx_utcc, &rx_utcc1, sizeof(utcc_t));
        // Print::hex("[UtccCli] pend: ", rx_utcc1.data, rx_utcc1.len - 1);
        return ret;
      }
    }
  }

 private:
  Socket* port_fp_ = NULL;
  serial_stream_t rx_stream_;
  serial_stream_t tx_stream_;
  pthread_mutex_t mutex_;
};

class UtccDecode : public SerialDecode {
 public:
  UtccDecode(int head, int slave_id, int data_maxlen) { flush(head, slave_id, data_maxlen); }
  ~UtccDecode(void) {}

  void flush(void) {
    rx_data_idx_ = 0;
    rx_state_ = UTCC_START_FROMID;
  }

  void flush(int head, int slave_id, int data_maxlen) {
    rx_data_idx_ = 0;
    master_id_ = head;
    slave_id_ = slave_id;
    data_maxlen_ = (data_maxlen > SERIAL_DATA_MAX ? SERIAL_DATA_MAX : data_maxlen);
    rx_state_ = UTCC_START_FROMID;
    // rx_data_continue_ = false;
  }

  serial_stream_t* parse_put(unsigned char* data, int len, BlockDeque<serial_stream_t>* rx_que) {
    for (int i = 0; i < len; i++) {
      ch = data[i];
      // printf("[utcc_t] ---state = %d, ch = 0x%x 0x%x 0x%x\n", rx_state_, ch, master_id_, slave_id_);
      switch (rx_state_) {
        case UTCC_START_FROMID:
          if (master_id_ == ch || 0x55 == master_id_) {
            rx_buf_[0] = ch;
            rx_state_ = UTCC_START_TOOID1;
          }
          break;

        case UTCC_START_TOOID1:
          rx_buf_[1] = ch;
          rx_state_ = UTCC_START_TOOID2;
          break;

        case UTCC_START_TOOID2:
          rx_buf_[2] = ch;
          rx_state_ = UTCC_STATE_LENGTH;
          break;

        case UTCC_STATE_LENGTH:
          rx_length_ = ch & 0x7F;
          if (0 < rx_length_ && rx_length_ < (data_maxlen_ - 5)) {
            rx_buf_[3] = ch;
            rx_length_ = rx_length_;
            rx_data_idx_ = 4;
            rx_state_ = UTCC_STATE_DATA;
          } else {
            rx_state_ = UTCC_START_FROMID;
          }
          break;

        case UTCC_STATE_DATA:
          if (rx_data_idx_ < rx_length_ + 4) {
            rx_buf_[rx_data_idx_++] = ch;
            if (rx_data_idx_ == rx_length_ + 4) {
              rx_state_ = UTCC_STATE_CRC1;
            }
          } else {
            rx_state_ = UTCC_START_FROMID;
          }
          break;

        case UTCC_STATE_CRC1:
          rx_buf_[rx_length_ + 4] = ch;
          rx_state_ = UTCC_STATE_CRC2;
          break;

        case UTCC_STATE_CRC2:
          rx_buf_[rx_length_ + 5] = ch;
          rx_state_ = UTCC_START_FROMID;

          crc1_ = Crc::modbus(rx_buf_, rx_length_ + 4);
          crc2_ = (rx_buf_[rx_length_ + 5] << 8) + rx_buf_[rx_length_ + 4];
          if (crc1_ == crc2_) {
            // printf("[utcc_t ] crc1_ ok\n");
            rx_data.len = rx_length_ + 5 + 1;
            memcpy(rx_data.data, rx_buf_, rx_data.len);
            if (rx_que != NULL) {
              rx_que->push_back(&rx_data);
              // printf("[utcc_t ] push_back ed\n");
            } else {
              return &rx_data;
            }
            // Print::hex("[utcc_t ] parse_put rx_data: ", rx_data.data, rx_data.len);
            // printf("[utcc_t ] parse_put rx_que.size:%ld %ld\n", rx_que->size(), (long)rx_que);
          }
          break;

        default:
          rx_state_ = UTCC_START_FROMID;
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
  typedef enum _UTCC_RECV_STATE {
    UTCC_START_FROMID = 0,
    UTCC_START_TOOID1 = 1,
    UTCC_START_TOOID2 = 2,
    UTCC_STATE_LENGTH = 3,
    UTCC_STATE_DATA = 4,
    UTCC_STATE_CRC1 = 5,
    UTCC_STATE_CRC2 = 6,
  } UTCC_RECV_STATE;

  int rx_length_;
  int rx_data_idx_;
  unsigned char rx_buf_[128];
  serial_stream_t rx_data;

  unsigned char ch;
  uint16_t crc1_, crc2_;
};

#endif
