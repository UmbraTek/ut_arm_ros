/* Copyright 2021 Umbratek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#ifndef __COMMON_HEX_DATA_H__
#define __COMMON_HEX_DATA_H__

#include <stdint.h>
#include <stdio.h>

class HexData {
 public:
  static int bit_set1(int a, int n) { return (a | (0x01 << n)); }

  static int bit_set0(int a, int n) { return (a & (~(0x01 << n))); }

  static int bit_get(int a, int n) { return (a & (0x01 << n)); }

  static int8_t hex_to_int8_big(uint8_t *a) { return a[0]; }
  static void hex_to_int8_big(uint8_t *a, int8_t *b, int n) {
    for (int i = 0; i < n; i++) b[i] = hex_to_int8_big(&a[i]);
  }

  static int16_t hex_to_int16_big(uint8_t *a) { return (a[0] << 8) + a[1]; }
  static void hex_to_int16_big(uint8_t *a, int16_t *b, int n) {
    for (int i = 0; i < n; i++) b[i] = hex_to_int16_big(&a[i * 2]);
  }

  static uint16_t hex_to_uint16_big(uint8_t *a) { return (a[0] << 8) + a[1]; }
  static void hex_to_int16_big(uint8_t *a, uint16_t *b, int n) {
    for (int i = 0; i < n; i++) b[i] = hex_to_uint16_big(&a[i * 2]);
  }

  static int32_t hex_to_int32_big(uint8_t *a) { return ((a[0] << 24) + (a[1] << 16) + (a[2] << 8) + a[3]); }
  static void hex_to_int32_big(uint8_t *a, int32_t *b, int n) {
    for (int i = 0; i < n; i++) b[i] = hex_to_int32_big(&a[i * 4]);
  }

  static float hex_to_fp32_big(uint8_t a[4]) {
    union _fp32hex {
      float dataf;
      uint32_t datai;
    } fp32hex;
    fp32hex.datai = hex_to_int32_big(a);
    return (float)fp32hex.dataf;
  }
  static void hex_to_fp32_big(uint8_t *a, float *b, int n) {
    for (int i = 0; i < n; ++i) b[i] = hex_to_fp32_big(&a[i * 4]);
  }

  static void int16_to_hex_big(int a, uint8_t *b) {
    b[0] = (uint8_t)(a >> 8);
    b[1] = (uint8_t)a;
  }
  static void int16_to_hex_big(int16_t *a, uint8_t *b, int n) {
    for (int i = 0; i < n; i++) int16_to_hex_big(a[i], &b[i * 2]);
  }

  static void int32_to_hex_big(int a, uint8_t *b) {
    b[0] = (uint8_t)(a >> 24);
    b[1] = (uint8_t)(a >> 16);
    b[2] = (uint8_t)(a >> 8);
    b[3] = (uint8_t)a;
  }
  static void int32_to_hex_big(int32_t *a, uint8_t *b, int n) {
    for (int i = 0; i < n; i++) int32_to_hex_big(a[i], &b[i * 4]);
  }

  static void fp32_to_hex_big(float dataf, uint8_t datahex[4]) {
    union _fp32hex {
      float dataf;
      uint32_t datai;
    } fp32hex;
    fp32hex.dataf = dataf;
    int32_to_hex_big(fp32hex.datai, datahex);
  }
  static void fp32_to_hex_big(float *dataf, uint8_t *datahex, int n) {
    for (int i = 0; i < n; ++i) fp32_to_hex_big(dataf[i], &datahex[i * 4]);
  }

  static float hex_to_fp32(uint8_t datahex[4]) {
    union _fp32hex {
      float dataf;
      uint8_t datahex[4];
    } fp32hex;
    fp32hex.datahex[0] = datahex[0];
    fp32hex.datahex[1] = datahex[1];
    fp32hex.datahex[2] = datahex[2];
    fp32hex.datahex[3] = datahex[3];
    return (float)fp32hex.dataf;
  }

  static void hex_to_fp32(uint8_t *datahex, float *dataf, int n) {
    for (int i = 0; i < n; ++i) {
      dataf[i] = hex_to_fp32(&datahex[i * 4]);
    }
  }

  static void hex_to_str(uint8_t *hex, char *str, int len) {
    for (int i = 0; i < len; ++i) sprintf(&str[i * 3], "%02x ", hex[i]);
  }
};

#endif
