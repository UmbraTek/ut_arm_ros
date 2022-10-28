/* Copyright 2020 UmbraTek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#ifndef __FLXIE_REG_H__
#define __FLXIE_REG_H__

#include <stdint.h>

class FLXIE_REG {
 public:
  FLXIE_REG(void) {}
  const uint8_t UNLOCK_FUN[5] = {0x22, 0, 1, 1, 0};
  const uint8_t SENSER1[5] = {0x60, 0, 16, ' ', ' '};
};

#endif
