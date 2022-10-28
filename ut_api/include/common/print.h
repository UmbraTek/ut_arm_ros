/* Copyright 2021 Umbratek Inc. All Rights Reserved.
 *
 * Licensed
 *
 * Author: Jimy Zhang <jimy.zhang@umbratek.com> <jimy92@163.com>
 ============================================================================*/
#ifndef __COMMON_PRINT_H__
#define __COMMON_PRINT_H__

#include <stdarg.h>
#include <stdio.h>

class Print {
 public:
  Print(void) {}
  ~Print(void) {}

  int PRINTF_NUM_MAX = 128;

  static void nvect(const char *str, double vect[], int n) {
    printf("%s", str);
    for (int i = 0; i < n; ++i) printf("%0.3f ", vect[i]);
    printf("\n");
  }

  static void nvect(const char *str, char vect[], int n) {
    printf("%s", str);
    for (int i = 0; i < n; ++i) printf("%d ", vect[i]);
    printf("\n");
  }

  static void nvect(const char *str, unsigned char vect[], int n) {
    printf("%s", str);
    for (int i = 0; i < n; ++i) printf("%d ", vect[i]);
    printf("\n");
  }

  static void nvect(const char *str, signed char vect[], int n) {
    printf("%s", str);
    for (int i = 0; i < n; ++i) printf("%d ", vect[i]);
    printf("\n");
  }

  static void nvect(const char *str, unsigned short vect[], int n) {
    printf("%s", str);
    for (int i = 0; i < n; ++i) printf("%d ", vect[i]);
    printf("\n");
  }

  static void nvect(const char *str, unsigned int vect[], int n) {
    printf("%s", str);
    for (int i = 0; i < n; ++i) printf("%d ", vect[i]);
    printf("\n");
  }

  static void nvect(const char *str, int vect[], int n) {
    printf("%s", str);
    for (int i = 0; i < n; ++i) printf("%d ", vect[i]);
    printf("\n");
  }

  static void nvect(const char *str, float vect[], int n) {
    printf("%s", str);
    for (int i = 0; i < n; ++i) printf("%.3f ", vect[i]);
    printf("\n");
  }

  static void hex(const char *str, unsigned char *hex, int len) {
    char buf[len * 3 + 1] = {'\0'};
    for (int i = 0; i < len; ++i) sprintf((char *)&buf[i * 3], "%02x ", hex[i]);

    printf("%s %s\n", str, buf);
  }

  static void mat4x4(char *str, float mat[4][4]) {
    printf("%s:\n", str);
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; j++) {
        printf("%f ", mat[i][j]);
      }
      printf("\n");
    }
  }

  static void nmat(float *A, int m, int n) {
    for (int i = 0; i < m; ++i) {
      for (int j = 0; j < n; j++) {
        printf("%f ", A[i * n + j]);
      }
      printf("\n");
    }
    printf("\n");
  }

  static void nvect1106(FILE *fp, const char *str, float vect[], int n) {
    fprintf(fp, "%s", str);
    for (int i = 0; i < n; ++i) fprintf(fp, "%11.6f ", vect[i]);
  }
};

#ifndef COLOUR_NONE
#define COLOUR_NONE "\033[0m"
#define COLOUR_RED "\033[0;31m"
#define COLOUR_RED_LIGHT "\033[1;31m"
#define COLOUR_GREEN "\033[0;32m"
#define COLOUR_GREEN_LIGHT "\033[1;32m"
#define COLOUR_BLUE "\033[0;34m"
#define COLOUR_BLUE_LIGHT "\033[1;34m"
#define COLOUR_YELLOW "\033[1;33m"
#define COLOUR_GRAY "\033[0;37m"
#define COLOUR_WHITE "\033[1;37m"
#endif

#endif
