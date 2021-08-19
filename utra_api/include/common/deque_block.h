/* Copyright 2021 Umbratek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#ifndef __COMMON_DEQUE_BLOCK_H__
#define __COMMON_DEQUE_BLOCK_H__

#pragma once

#include <condition_variable>
#include <deque>
#include <mutex>
#include "linuxcvl.h"
#include "queue_memcpy.h"

template <class T>
class BlockDeque {
 public:
  BlockDeque(int max_size = 1) {
    ts_.tv_sec = 0;
    ts_.tv_nsec = 1000;
    que_ = new QueueMemcpy<T>(max_size);
  }

  ~BlockDeque(void) { delete que_; }

  int flush(void) { return que_->clear(); }

  int clear(void) { return flush(); }

  int push(T *data, bool is_block = true) {
    std::unique_lock<std::mutex> lock(mutex_);  // 初始化时是默认上锁的

    if (que_->is_full()) {
      if (is_block) {
        while (que_->is_full()) m_not_full_.wait(lock);
      } else {
        // que_->pop(&temp_data_);
        return -1;
      }
    }

    int ret = que_->push_back(data);
    m_not_empty_.notify_all();
    return ret;
  }

  int push_back(T *data, bool is_block = true) { return push(data, is_block); }

  int pop(T *data, float timeout_s = 0) {
    // 当前有数据，直接获取
    if (!que_->empty()) {
      que_->pop(data);
      m_not_full_.notify_all();
      return 0;
    }

    // 当前没数据，有限时间阻塞等待
    long timeout_us = (int)(timeout_s * 1000000);
    if (timeout_us > 0) {
      time_start_us_ = LinuxCvl::get_us();
      while (true) {
        if (!que_->empty()) {
          que_->pop(data);
          m_not_full_.notify_all();
          return 0;
        }

        time_curr_us_ = LinuxCvl::get_us();
        if ((time_curr_us_ - time_start_us_) > timeout_us) return -1;
        nanosleep(&ts_, NULL);
      }
    }

    // 当前没数据，无时间限制阻塞等待
    std::unique_lock<std::mutex> lock(mutex_);
    while (que_->empty()) {
      m_not_empty_.wait(lock);
    }

    int ret = que_->pop(data);
    m_not_full_.notify_all();
    return ret;
  }

  int begin(T *data, float timeout_s = 0) { return pop(data, timeout_s); }

  size_t size() { return que_->size(); }

  bool is_full() { return que_->is_full(); }

  bool empty() { return que_->empty(); }
  bool is_empty() { return que_->empty(); }

  int get(T *data) { return que_->get(data); }

 private:
  QueueMemcpy<T> *que_;
  T temp_data_;
  struct timespec ts_;
  signed long long time_start_us_;
  signed long long time_curr_us_;

  std::mutex mutex_;
  std::condition_variable m_not_empty_;
  std::condition_variable m_not_full_;
};

#endif
