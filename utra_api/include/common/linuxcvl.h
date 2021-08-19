/* Copyright 2021 Umbratek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#ifndef __COMMON_LINUX_CVL_H__
#define __COMMON_LINUX_CVL_H__

#include <pthread.h>
typedef void *(*fun_point_t)(void *);

class LinuxCvl {
 public:
  LinuxCvl(void){};
  ~LinuxCvl(void){};

  static const int MAC_SIZE = 18;
  static const int IP_SIZE = 16;

  // network
  static int get_local_mac(char *eth_inf, char *mac);
  static int get_local_ip(char *eth_inf, char *ip);
  static int socket_init(int port, char *net_name, int is_server);
  static int socket_init(char *local_ip, int port, int is_server);
  static int socket_wait_forclient(int sockfd);
  static int socket_connect_server(int *socket, char server_ip[], int server_port);
  static int socket_send_data(int client_fp, unsigned char *data, int len);
  static int socket_is_connect(int client_fp);

  // preempt
  static void rt_init(int priority, unsigned long long stack_max);

  typedef struct _rt_task_t {
    pthread_t tid;
    pthread_attr_t attr;
  } rt_task_t;
  static int rt_task_create(rt_task_t *task, long stk_size, long prio);
  static int rt_task_start(rt_task_t *task, char *name, void *(*thread)(void *), void *arg);
  static void rt_task_delete(rt_task_t *task);
  static signed long long tv2us(const struct timespec *tv);
  static signed long long tv2ns(const struct timespec *tv);
  static void us2tv(signed long long us, struct timespec *tv);
  static void ns2tv(signed long long ns, struct timespec *tv);
  static signed long long get_us(void);
  static signed long long get_ns(void);
  static void ns_to_str(signed long long ns, char *str);
  static void tv_add(struct timespec *t, signed long long us);
  static void rt_sleep_us(signed long long time);
  static void rt_sleep_ns(signed long long time);
  static void rt_sleep_until(long long wakeup_time);

  // thread
  static pthread_t thread_init(fun_point_t fun_point, void *arg);
  static void thread_delete(pthread_t id);

 protected:
 private:
  static const long long NSEC_PER_SEC = 1000000000;
};

#endif
