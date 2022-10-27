/* Copyright 2021 Umbratek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#ifndef __COMMON_QUEUE_MEMCPY_H__
#define __COMMON_QUEUE_MEMCPY_H__

#include <pthread.h>

template <class T>
class QueueMemcpy {
 public:
  QueueMemcpy(long max_size = 1024) {
    max_size_ = max_size;
    buf_ = new T[max_size_];
    cp_size_ = sizeof(T);

    pthread_mutex_init(&mutex_, NULL);
    flush();
  }

  ~QueueMemcpy(void) { delete[] buf_; }

  int flush(void) {
    pthread_mutex_lock(&mutex_);
    cnt_ = 0;
    head_ = 0;
    tail_ = 0;
    pthread_mutex_unlock(&mutex_);
    return 0;
  }

  int clear(void) { return flush(); }

  int push(T *data) {
    pthread_mutex_lock(&mutex_);
    if (max_size_ <= cnt_) {
      pthread_mutex_unlock(&mutex_);
      return -1;
    }
    if (max_size_ <= head_) head_ = 0;

    memcpy(&buf_[head_], data, cp_size_);
    head_++;
    cnt_++;
    pthread_mutex_unlock(&mutex_);
    return 0;
  }

  int push_back(T *data) { return push(data); }

  int pop(T *data) {
    pthread_mutex_lock(&mutex_);
    if (0 >= cnt_) {
      pthread_mutex_unlock(&mutex_);
      return -1;
    }
    if (max_size_ <= tail_) tail_ = 0;

    memcpy(data, &buf_[tail_], cp_size_);
    tail_++;
    cnt_--;
    pthread_mutex_unlock(&mutex_);
    return 0;
  }

  int begin(T *data) { return pop(data); }

  long size(void) { return cnt_; }

  bool is_full(void) { return (max_size_ <= cnt_) ? true : false; }

  int empty(void) { return cnt_ <= 0 ? true : false; }

  /**
   * 队列数据不出队，获取一个节点的数据
   * @method get
   * @param  data             [获取节点数据的指针]
   * @return                  [0成功，否则失败]
   */

  int get(T *data) {
    pthread_mutex_lock(&mutex_);
    if (0 >= cnt_) {
      pthread_mutex_unlock(&mutex_);
      return -1;
    }
    if (max_size_ <= tail_) tail_ = 0;

    memcpy(data, &buf_[tail_], cp_size_);
    pthread_mutex_unlock(&mutex_);

    return 0;
  }

 protected:
 private:
  long max_size_;
  size_t cp_size_;
  T *buf_;

  long cnt_;
  long head_;
  long tail_;

  pthread_mutex_t mutex_;
};

#endif
