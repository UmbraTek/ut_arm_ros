/* Copyright 2020 UmbraTek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#ifndef __FLXIV_REG_H__
#define __FLXIV_REG_H__

#include <stdint.h>

class FLXIV_REG {
 public:
  FLXIV_REG(void) {}
  const uint8_t SENSER1[5] = {0x60, 0, 16, ' ', ' '};
};

#endif
