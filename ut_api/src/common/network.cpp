/* Copyright 2017 Umbratek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#include "common/linuxcvl.h"

#include <arpa/inet.h>
#include <errno.h>
#include <net/if.h>
#include <netinet/tcp.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

/**
 * 通过网卡名获取mac地址
 * @method LinuxCvl::get_local_mac
 * @param  eth_inf              [本地网卡名]
 * @param  mac                  [获取的mac]
 * @return                      [0正常，否则失败]
 */

int LinuxCvl::get_local_mac(char *eth_inf, char *mac) {
  struct ifreq ifr;
  int sd;

  bzero(&ifr, sizeof(struct ifreq));
  if ((sd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    printf("[LinuxCvl] Error: get %s mac address socket creat\n", eth_inf);
    return -1;
  }

  strncpy(ifr.ifr_name, eth_inf, sizeof(ifr.ifr_name) - 1);
  if (ioctl(sd, SIOCGIFHWADDR, &ifr) < 0) {
    printf("[LinuxCvl] Error: get %s mac address\n", eth_inf);
    close(sd);
    return -1;
  }

  snprintf(mac, MAC_SIZE, "%02x:%02x:%02x:%02x:%02x:%02x", (unsigned char)ifr.ifr_hwaddr.sa_data[0],
           (unsigned char)ifr.ifr_hwaddr.sa_data[1], (unsigned char)ifr.ifr_hwaddr.sa_data[2],
           (unsigned char)ifr.ifr_hwaddr.sa_data[3], (unsigned char)ifr.ifr_hwaddr.sa_data[4],
           (unsigned char)ifr.ifr_hwaddr.sa_data[5]);

  close(sd);

  return 0;
}

/**
 * 通过本地网卡名获取本地ip
 * @method LinuxCvl::get_local_ip
 * @param  eth_inf             [本地网卡名]
 * @param  ip                  [获取的ip]
 * @return                     [0正常，否则失败]
 */

int LinuxCvl::get_local_ip(char *eth_inf, char *ip) {
  struct sockaddr_in sin;
  struct ifreq ifr;

  int sd = socket(AF_INET, SOCK_DGRAM, 0);
  if (-1 == sd) {
    printf("[LinuxCvl] Error: socket return: %s\n", strerror(errno));
    return -1;
  }

  strncpy(ifr.ifr_name, eth_inf, IFNAMSIZ);
  ifr.ifr_name[IFNAMSIZ - 1] = 0;

  // if error: No such device
  if (ioctl(sd, SIOCGIFADDR, &ifr) < 0) {
    printf("[LinuxCvl] Error: get_local_ip, net_name:%s ioctl return: %s\n", eth_inf, strerror(errno));
    close(sd);
    return -1;
  }

  memcpy(&sin, &ifr.ifr_addr, sizeof(sin));
  snprintf(ip, IP_SIZE, "%s", inet_ntoa(sin.sin_addr));

  close(sd);
  return 0;
}

#define PERRNO(ret, db_flg, str)     \
  {                                  \
    if (-1 == ret) {                 \
      printf("%s%s\n", db_flg, str); \
      return -1;                     \
    }                                \
  }

/**
 * 通过本地ip地址获取socket并初始化
 * @method LinuxCvl::socket_init
 * @param  local_ip           [本地ip地址]
 * @param  port               [socket绑定的端口]
 * @param  is_server          [1初始化为服务器，0为客户端]
 * @return                    [-1失败，否则返回socket]
 */

int LinuxCvl::socket_init(char *local_ip, int port, int is_server) {
  int sockfd = socket(AF_INET, SOCK_STREAM, 0);
  PERRNO(sockfd, "[LinuxCvl]", " Error: socket");

  int on = 1;
  int keepAlive = 1;     // Turn on keepalive attribute
  int keepIdle = 1;      // If there is no data in n seconds, probe
  int keepInterval = 1;  // Detection interval,5 seconds
  int keepCount = 3;     // 3 detection attempts
  struct timeval timeout = {2, 0};

  int ret = setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (void *)&on, sizeof(on));
  PERRNO(ret, "[LinuxCvl]", " Error: setsockopt");
  ret = setsockopt(sockfd, SOL_SOCKET, SO_KEEPALIVE, (void *)&keepAlive, sizeof(keepAlive));
  PERRNO(ret, "[LinuxCvl]", " Error: setsockopt");
  ret = setsockopt(sockfd, SOL_TCP, TCP_KEEPIDLE, (void *)&keepIdle, sizeof(keepIdle));
  PERRNO(ret, "[LinuxCvl]", " Error: setsockopt");
  ret = setsockopt(sockfd, SOL_TCP, TCP_KEEPINTVL, (void *)&keepInterval, sizeof(keepInterval));
  PERRNO(ret, "[LinuxCvl]", " Error: setsockopt");
  ret = setsockopt(sockfd, SOL_TCP, TCP_KEEPCNT, (void *)&keepCount, sizeof(keepCount));
  PERRNO(ret, "[LinuxCvl]", " Error: setsockopt");
  ret = setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout, sizeof(struct timeval));
  PERRNO(ret, "[LinuxCvl]", " Error: setsockopt");

  struct sockaddr_in local_addr;
  local_addr.sin_family = AF_INET;
  local_addr.sin_port = htons(port);
  local_addr.sin_addr.s_addr = inet_addr(local_ip);
  ret = bind(sockfd, (struct sockaddr *)&local_addr, sizeof(local_addr));
  PERRNO(ret, "[LinuxCvl]", " Error: socket_init bind");

  if (is_server) {
    ret = listen(sockfd, 10);
    PERRNO(ret, __FILE__, "  Error: listen");
  }
  return sockfd;
}

/**
 * 通过本地网卡名字获取socket并初始化
 * @method LinuxCvl::socket_init
 * @param  port               [socket绑定的端口]
 * @param  net_name           [本地网卡名字]
 * @param  is_server          [1初始化为服务器，0为客户端]
 * @return                    [-1失败，否则返回socket]
 */

int LinuxCvl::socket_init(int port, char *net_name, int is_server) {
  char local_ip[16] = {0};
  int ret = get_local_ip(net_name, local_ip);
  if (-1 == ret) printf("[LinuxCvl] Error: get_local_ip, net_name: %s\n", net_name);

  int sockfd = socket_init(local_ip, port, is_server);
  return sockfd;
}

/**
 * 服务器使用socket等待客户端连接
 * @method LinuxCvl::socket_wait_forclient
 * @param  sockfd                       [socket]
 * @return                              [-1失败，其他正常返回客户端的fp]
 */

int LinuxCvl::socket_wait_forclient(int sockfd) {
  struct sockaddr_in cliaddr;
  socklen_t addrlen = sizeof(cliaddr);
  int clifd = accept(sockfd, (struct sockaddr *)&cliaddr, &addrlen);
  PERRNO(clifd, "[LinuxCvl]", " Error: accept");
  return clifd;
}

/**
 * 客户端使用socket连接到服务器
 * @method LinuxCvl::socket_connect_server
 * @param  socket                       [socket]
 * @param  server_ip                    [服务器ip]
 * @param  server_port                  [服务器端口]
 * @return                              [0正常，否则失败]
 */

int LinuxCvl::socket_connect_server(int *socket, char server_ip[], int server_port) {
  struct sockaddr_in server_addr;
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(server_port);
  inet_aton(server_ip, &server_addr.sin_addr);
  int ret = connect(*socket, (struct sockaddr *)&server_addr, sizeof(server_addr));
  PERRNO(ret, "[LinuxCvl]", " Error: socket_connect_server");
  return 0;
}

/**
 * 服务器给客户端发送数据
 * @method LinuxCvl::socket_send_data
 * @param  client_fp               [客户端fp]
 * @param  data                    [发送的数据]
 * @param  len                     [数据长度]
 * @return                         [-1失败，否则正常]
 */

int LinuxCvl::socket_send_data(int client_fp, unsigned char *data, int len) {
  int ret = send(client_fp, (void *)data, len, 0);
  if (ret != len) {
    printf("[LinuxCvl] Error: socket_send_data\n");
    return -1;
  }
  return 0;
}

/**
 * 服务器判断客户端是否依然在线，测试TCP连接
 * @method LinuxCvl::socket_is_connect
 * @param  client_fp                [客户端的fp]
 * @return                          [1连接，0断开]
 */

int LinuxCvl::socket_is_connect(int client_fp) {
  if (client_fp <= 0) return 0;

  struct tcp_info info;
  int len = sizeof(info);
  getsockopt(client_fp, IPPROTO_TCP, TCP_INFO, &info, (socklen_t *)&len);
  if ((info.tcpi_state == TCP_ESTABLISHED))
    return 1;
  else
    return 0;
}
