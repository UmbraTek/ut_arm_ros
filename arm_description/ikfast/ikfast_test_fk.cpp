#define IKFAST_HAS_LIBRARY
#include "/usr/local/lib/python2.7/dist-packages/openravepy/_openravepy_0_53/ikfast.h"
using namespace ikfast;

IKFAST_API void ComputeFk(const IkReal* j, IkReal* eetrans, IkReal* eerot);

#include <stdio.h>
#include <stdlib.h>

#define sq(x) ((x) * (x))
#define SIGN(x) ((x >= 0.0) ? 1 : -1)

void rot_to_euler(double mat[3][3], double arr[3]) {
  arr[1] = atan2(-mat[2][0], sqrt(sq(mat[2][1]) + sq(mat[2][2])));
  arr[0] = atan2(mat[2][1] * SIGN(cos(arr[4])), mat[2][2] * SIGN(cos(arr[4])));
  arr[2] = atan2(mat[1][0] * SIGN(cos(arr[4])), mat[0][0] * SIGN(cos(arr[4])));
}

// ./ik_fast_utra6_550_robot 0 0 0 0 0 0
int main(int argc, char** argv) {
  IkReal j[7];
  if (argc == GetNumJoints() + 1) {
    j[0] = atof(argv[1]) * M_PI / 180.0;
    j[1] = atof(argv[2]) * M_PI / 180.0;
    j[2] = atof(argv[3]) * M_PI / 180.0;
    j[3] = atof(argv[4]) * M_PI / 180.0;
    j[4] = atof(argv[5]) * M_PI / 180.0;
    j[5] = atof(argv[6]) * M_PI / 180.0;
    if (GetNumJoints() == 7) j[6] = atof(argv[7]) * M_PI / 180.0;
    for (int i = 0; i < 7; i++) printf("%f ", j[i]);
    printf("\n");
  } else {
    j[0] = 0;
    j[1] = 0;
    j[2] = 0;
    j[3] = 0;
    j[4] = 0;
    j[5] = 0;
    j[6] = 0;
    for (int i = 0; i < 6; i++) printf("%f ", j[i]);
    printf("\n");
  }

  IkReal eetrans[3];
  IkReal eerot[9];
  ComputeFk(j, eetrans, eerot);

  IkReal arr[6];
  double mat[3][3] = {eerot[0], eerot[1], eerot[2], eerot[3], eerot[4], eerot[5], eerot[6], eerot[7], eerot[8]};
  rot_to_euler(mat, arr);

  for (int i = 0; i < 7; i++) printf("%f ", j[i]);
  printf("\n");
  for (int i = 0; i < 3; i++) printf("%f ", eetrans[i] * 1000.0);
  for (int i = 0; i < 3; i++) printf("%f ", arr[i] * 180.0 / M_PI);
  printf("\n");
  for (int i = 0; i < 9; i++) printf("%f ", eerot[i]);
  printf("\n");
  return 0;
}
