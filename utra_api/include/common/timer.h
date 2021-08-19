/* Copyright 2021 Umbratek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/

#ifndef __COMMON_TIME_H__
#define __COMMON_TIME_H__

#include <assert.h>
#include <stdint.h>
#include <time.h>

class Timer {
 public:
  /**
   * Timer for measuring time elapsed with clock_monotonic
   * @method Timer
   */
  Timer(void) { start(); }

  void start() { clock_gettime(CLOCK_MONOTONIC, &time_start_); }

  double get_seconds() { return (double)get_ns() / 1.e9; }
  double get_ms(void) { return (double)get_ns() / 1.e6; }
  int64_t get_ns(void) {
    clock_gettime(CLOCK_MONOTONIC, &time_now_);
    return (int64_t)(time_now_.tv_nsec - time_start_.tv_nsec) + 1000000000 * (time_now_.tv_sec - time_start_.tv_sec);
  }

 private:
  struct timespec time_start_;
  struct timespec time_now_;
};

#endif  // __COMMON_TIME_H__
