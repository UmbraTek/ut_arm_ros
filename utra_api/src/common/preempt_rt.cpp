/* Copyright 2021 Umbratek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#include "common/linuxcvl.h"

#include <sched.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

static void stack_prefault(unsigned long long stack_max) {
  unsigned char dummy[stack_max];
  memset(dummy, 0, stack_max);
}

/**
 * 初始化本线程为实时线程
 * @method LinuxCvl::rt_init
 * @param  priority       [优先级，正常为0，最高52]
 * @param  stack_max      [本线程的堆栈大小，单位bit]
 */

void LinuxCvl::rt_init(int priority, unsigned long long stack_max) {
  struct sched_param param;

  param.sched_priority = priority;
  if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
    printf(__FILE__ "  Error: sched_setscheduler failed\n");
    _exit(-1);
  }

  if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
    printf(__FILE__ "  Error: mlockall failed\n");
    _exit(-2);
  }
  stack_prefault(stack_max); /* pre-fault our stack */
}

/**
 * 第二种创建实时线程的方式，创建线程＋直接实时属性初始化
 * @method LinuxCvl::rt_task_create
 * @param  task                     [description]
 * @param  stk_size                 [description]
 * @param  prio                     [description]
 * @return                          [description]
 */

int LinuxCvl::rt_task_create(rt_task_t *task, long stk_size, long prio) {
  int ret;
  ret = pthread_attr_init(&task->attr);
  ret = pthread_attr_setdetachstate(&task->attr, PTHREAD_CREATE_JOINABLE);  // 线程分离属性
  ret = pthread_attr_setschedpolicy(&task->attr, SCHED_FIFO);               //线程调度策略,允许被高优先级抢占
  ret = pthread_attr_setstacksize(&task->attr, stk_size);                   //线程栈大小

  struct sched_param sched_para;
  sched_para.sched_priority = prio;
  ret = pthread_attr_setschedparam(&task->attr, &sched_para);

  return ret;
}

/**
 * 开启创建的实时线程
 * @method LinuxCvl::rt_task_start
 * @param  task                         [实时线程的结构体变量]
 * @param  name                         [实时线程名字]
 * @param  thread                       [实时线程执行函数]
 * @param  arg                          [传入参数]
 * @return                              [description]
 */
int LinuxCvl::rt_task_start(rt_task_t *task, char *name, void *(*thread)(void *), void *arg) {
  pthread_create(&task->tid, &task->attr, thread, arg);
  pthread_setname_np(task->tid, name);
  return 0;
}

/**
 * 杀死实时线程
 * @method LinuxCvl::rt_task_delete
 * @param  task                     [实时线程的结构体变量]
 */

void LinuxCvl::rt_task_delete(rt_task_t *task) {
  pthread_kill(task->tid, 0);
  pthread_join(task->tid, NULL);
}

/**
 * 时间单位转换函数
 * @method LinuxCvl::tv2us
 * @param  tv              [description]
 * @return                 [description]
 */

signed long long LinuxCvl::tv2us(const struct timespec *tv) { return (tv->tv_sec * NSEC_PER_SEC + tv->tv_nsec) * 0.001; }

signed long long LinuxCvl::tv2ns(const struct timespec *tv) {
  return (tv->tv_sec * NSEC_PER_SEC + tv->tv_nsec);  //
}

void LinuxCvl::us2tv(signed long long us, struct timespec *tv) {
  signed long long u = us * 1000;
  tv->tv_nsec = u % NSEC_PER_SEC;
  tv->tv_sec = u / NSEC_PER_SEC;
}

void LinuxCvl::ns2tv(signed long long ns, struct timespec *tv) {
  tv->tv_nsec = ns % NSEC_PER_SEC;
  tv->tv_sec = ns / NSEC_PER_SEC;
}

/**
 * 获取系统开机运行的总时间，单位us
 * @method LinuxCvl::get_us
 * @return                  [当前系统时间]
 */

signed long long LinuxCvl::get_us(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return tv2us(&t);
}

/**
 * 获取系统开机运行的总时间，单位ns
 * @method LinuxCvl::get_ns
 * @return                  [description]
 */

signed long long LinuxCvl::get_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return tv2ns(&t);
}

/**
 * 将系统开机运行时间转换成字符串
 * @method LinuxCvl::ns_to_str
 * @param  ns                  [系统运行总时间]
 * @param  str                 [字符串]
 */

void LinuxCvl::ns_to_str(signed long long ns, char *str) {
  signed long long h, m, s, ms, us;
  us = ns / 1000;
  ms = us / 1000;
  s = ms / 1000;
  m = s / 60;
  h = m / 60;
  sprintf((char *)str, "%5lldh%2lldm%2llds:%3lldms:%3lldus:%3lldns", h, m % 60, s % 60, ms % 1000, us % 1000, ns % 1000);
}

/**
 * 在timespec时间类型变量中添加时间
 * @method LinuxCvl::tv_add
 * @param  t                [description]
 * @param  us               [description]
 */

void LinuxCvl::tv_add(struct timespec *t, signed long long us) {
  t->tv_nsec += us * 1000;
  while (t->tv_nsec >= NSEC_PER_SEC) {
    t->tv_nsec -= NSEC_PER_SEC;
    t->tv_sec++;
  }
}

/**
 * 实时线程的sleep函数
 * @method LinuxCvl::rt_sleep_us
 * @param  time               [sleep时间，单位us]
 */

void LinuxCvl::rt_sleep_us(signed long long time) {
  int ret;
  signed long long time_lgoal;
  struct timespec time_sgoal;

  time_lgoal = get_us() + time;
  us2tv(time_lgoal, &time_sgoal);
  ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &time_sgoal, NULL);
  if (0 != ret) printf(__FILE__ "  Error: clock_nanosleep failed\n");
}

void LinuxCvl::rt_sleep_ns(signed long long time) {
  int ret;
  signed long long time_lgoal;
  struct timespec time_sgoal;

  time_lgoal = get_ns() + time;
  ns2tv(time_lgoal, &time_sgoal);
  ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &time_sgoal, NULL);
  if (0 != ret) printf(__FILE__ "  Error: clock_nanosleep failed\n");
}

void LinuxCvl::rt_sleep_until(long long wakeup_time) {
  timespec t;
  ns2tv(wakeup_time, &t);
  clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
}
