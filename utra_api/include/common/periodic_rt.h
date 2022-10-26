/* Copyright 2021 Umbratek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#ifndef __COMMON_PERIODIC_RT_H__
#define __COMMON_PERIODIC_RT_H__

#include <math.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <vector>
// #ifdef linux
#include <sys/prctl.h>
#include <sys/timerfd.h>
// #endif

#include "timer.h"

class RtTask {
 public:
  /**
   * A single periodic task which will call run() at the given frequency
   * @method RtTask
   * @param  period_s   [周期定时时间，单位：秒]
   * @param  name       [线程名字]
   * @param  stack_size [线程堆栈大小]
   * @param  priority   [线程优先级：１－５０]
   */

  RtTask(float period_s, std::string name, long stack_size, int priority)
      : period_s_(period_s), name_(name), stack_size_(stack_size), priority_(priority) {
    clear_max();
  }

  virtual ~RtTask(void) { stop(); }

  void start(void) {
    if (is_running_) return;

    is_running_ = true;
    thread_id_ = new std::thread(&RtTask::loop_function, this);
  }

  void stop(void) {
    if (!is_running_) return;
    is_running_ = false;
    // delete thread_id_;
    // thread_id_->join();
  }

  void print_status(void) {
    if (!is_running_) return;
    printf("|%-20s|%6.4f|%6.4f|%6.4f\n", name_.c_str(), max_runtime_, period_s_, max_period_);
  }

  void clear_max(void) {
    max_period_ = 0;
    max_runtime_ = 0;
  }

  bool is_slow(void) {
    return max_period_ > period_s_ * 1.01f || max_runtime_ > period_s_;  //
  }

  float get_period(void) { return period_s_; }
  float get_max_period(void) { return max_period_; }
  float get_max_runtime(void) { return max_runtime_; }

  virtual void run() = 0;

 private:
  volatile bool is_running_ = false;  // 函数周期运行的标志变量
  float last_runtime_ = 0;            // 临时变量1
  float last_period_time_ = 0;        // 临时变量2
  float max_period_ = 0;              // 线程最大周期调度时间
  float max_runtime_ = 0;             // s函数最大运行时间

  float period_s_;          // 函数周期运行时间
  std::string name_;        // 线程名字
  long stack_size_;         // 线程堆栈大小
  int priority_;            // 线程优先级
  std::thread* thread_id_;  // 线程创建返回id

  // 设置线程堆栈大小
  void stack_prefault(long size) {
    //volatile char stack[size];
    //memset(const_cast<char*>(stack), 0, size);
    //if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
    //  printf("[Rt  Task] [Error] %s mlockall failed.\n", name_.c_str());
    //}
  }

  // 设置线程优先级
  void setup_scheduler(int priority) {
    //struct sched_param params;
    //params.sched_priority = priority;
    //if (sched_setscheduler(0, SCHED_FIFO, &params) == -1) {
    //  printf("[Rt  Task] [Error] %s setup_scheduler failed.\n", name_.c_str());
    //}
  }

  // 周期性循环运行的函数
  void loop_function(void) {
    prctl(PR_SET_NAME, name_.c_str());
    stack_prefault(stack_size_);
    setup_scheduler(priority_);

    // timerFd 定时器
    int seconds = (int)period_s_;
    int nanoseconds = (int)(1e9 * std::fmod(period_s_, 1.f));
    itimerspec timerSpec;
    timerSpec.it_value.tv_sec = seconds;
    timerSpec.it_value.tv_nsec = nanoseconds;
    timerSpec.it_interval.tv_sec = seconds;
    timerSpec.it_interval.tv_nsec = nanoseconds;
    int timerFd = timerfd_create(CLOCK_MONOTONIC, 0);
    timerfd_settime(timerFd, 0, &timerSpec, nullptr);

    Timer t;
    unsigned long long missed = 0;
    printf("[Rt  Task] RtTask Start %s (%d s, %d ns)\n", name_.c_str(), seconds, nanoseconds);
    t.start();

    bool is_loop = ((seconds > 0) || (nanoseconds > 10000)) ? true : false;  // 如果时间小于10us只运行一次func
    do {
      // last_period_time_ = (float)t.get_seconds();
      // t.start();
      run();
      // last_runtime_ = (float)t.get_seconds();

      int m = read(timerFd, &missed, sizeof(missed));
      (void)m;

      // max_period_ = std::max(max_period_, last_period_time_);
      // max_runtime_ = std::max(max_runtime_, last_runtime_);
    } while (is_running_ && is_loop);
    printf("[Rt  Task] RtTask %s has stopped!\n", name_.c_str());
  }
};

// A collection of periodic tasks which can be monitored together
// 线程任务集合管理器
class RtTaskManager {
 public:
  RtTaskManager(void){};
  ~RtTaskManager(void) {}

  void add_task(RtTask* task) { tasks_.push_back(task); }

  void print_status(void) {
    printf("\n-------------------------TASKS--------------------------\n");
    printf("|%-20s|%-6s|%-6s|%-6s\n", "name", "rt-max", "T-des", "T-max");
    printf("----------------------------------------------------------\n");
    for (auto& task : tasks_) task->print_status();
  }

  void print_status_of_slowtasks(void) {
    for (auto& task : tasks_) {
      if (task->is_slow()) task->print_status();
    }
  }

  void stop_all(void) {
    for (auto& task : tasks_) task->stop();
  }

 private:
  std::vector<RtTask*> tasks_;
};

// 单独函数的周期线程任务
class RtPeriodicFun : public RtTask {
 public:
  RtPeriodicFun(float period_s, std::string name, long stack_size, int priority, void (*function)())
      : RtTask(period_s, name, stack_size, priority), _function(function) {}
  ~RtPeriodicFun(void) { stop(); }

  void run(void) { _function(); }

 private:
  void (*_function)() = nullptr;
};

// 类成员函数的周期线程任务
template <typename T>
class RtPeriodicMemberFun : public RtTask {
 public:
  RtPeriodicMemberFun(float period_s, std::string name, long stack_size, int priority, void (T::*function)(), T* obj)
      : RtTask(period_s, name, stack_size, priority), _function(function), _obj(obj) {}
  ~RtPeriodicMemberFun(void) { stop(); }

  void run(void) { (_obj->*_function)(); }

 private:
  void (T::*_function)();
  T* _obj;
};

#endif
