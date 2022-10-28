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
  IkReal q[7];
  srand((unsigned)time(NULL));
  // float n = ((float)rand() / (float)(RAND_MAX)-0.5) * 2.0 * 160.0 * M_PI / 180.0;
  // printf("%f\n", n);

  q[0] = ((float)rand() / (float)(RAND_MAX)-0.5) * 2.0 * 160.0 * M_PI / 180.0;
  q[1] = ((float)rand() / (float)(RAND_MAX)-0.5) * 2.0 * 160.0 * M_PI / 180.0;
  q[2] = ((float)rand() / (float)(RAND_MAX)-0.5) * 2.0 * 160.0 * M_PI / 180.0;
  q[3] = ((float)rand() / (float)(RAND_MAX)-0.5) * 2.0 * 160.0 * M_PI / 180.0;
  q[4] = ((float)rand() / (float)(RAND_MAX)-0.5) * 2.0 * 160.0 * M_PI / 180.0;
  q[5] = ((float)rand() / (float)(RAND_MAX)-0.5) * 2.0 * 160.0 * M_PI / 180.0;
  q[6] = ((float)rand() / (float)(RAND_MAX)-0.5) * 2.0 * 160.0 * M_PI / 180.0;
  printf("q: ");
  for (int i = 0; i < 7; i++) printf("%10.6f ", q[i]);
  printf("\n");

  IkReal eetrans[3];
  IkReal eerot[9];
  ComputeFk(q, eetrans, eerot);
  printf("T: ");
  for (int i = 0; i < 3; i++) printf("%10.6f ", eetrans[i] * 1000.0);
  printf("\n");

  printf("R: ");
  for (int i = 0; i < 9; i++) printf("%10.6f ", eerot[i]);
  printf("\n");

  bool bSuccess = false;
  IkSolutionList<IkReal> solutions;
  if (GetNumJoints() == 6) bSuccess = ComputeIk(eetrans, eerot, NULL, solutions);
  // else if (GetNumJoints() == 7)
  //  bSuccess = ComputeIk(eetrans, eerot, &q[2], solutions);

  if (!bSuccess) {
    fprintf(stderr, "Failed to get ik solution\n");
    return -1;
  }

  printf("Found %d ik solutions:\n", (int)solutions.GetNumSolutions());
  std::vector<IkReal> solvalues(GetNumJoints());

  bool is_solution = false;
  /*
  for (std::size_t i = 0; i < solutions.GetNumSolutions(); ++i) {
    const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
    // printf("sol%d (free=%d): ", (int)i, (int)sol.GetFree().size());
    std::vector<IkReal> vsolfree(sol.GetFree().size());
    sol.GetSolution(&solvalues[0], vsolfree.size() > 0 ? &vsolfree[0] : NULL);

    bool is_solution = true;
    for (std::size_t j = 0; j < solvalues.size(); ++j) {
      if (fabs(q[j] - solvalues[j]) > 0.001) is_solution = false;
    }

    if (is_solution) {
      for (std::size_t j = 0; j < solvalues.size(); ++j) printf("%.15f, ", solvalues[j]);
      printf("\n");
      continue;
    }
  }*/

  if (!is_solution) {
    printf("is_solution faile\n");
    for (std::size_t i = 0; i < solutions.GetNumSolutions(); ++i) {
      const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
      std::vector<IkReal> vsolfree(sol.GetFree().size());
      sol.GetSolution(&solvalues[0], vsolfree.size() > 0 ? &vsolfree[0] : NULL);
      for (std::size_t j = 0; j < solvalues.size(); ++j) printf("%.15f, ", solvalues[j]);
      printf("\n");
    }
  }
}
/*
int main(int argc, char** argv) {
  if (argc != 12 + GetNumFreeParameters() + 1) {
    printf(
        "\nUsage: ./ik r00 r01 r02 t0 r10 r11 r12 t1 r20 r21 r22 t2 free0 ...\n\n"
        "Returns the ik solutions given the transformation of the end effector specified by\n"
        "a 3x3 rotation R (rXX), and a 3x1 translation (tX).\n"
        "There are %d free parameters that have to be specified.\n\n",
        GetNumFreeParameters());
    return 1;
  }

  IkSolutionList<IkReal> solutions;
  std::vector<IkReal> vfree(GetNumFreeParameters());
  IkReal eerot[9], eetrans[3];
  eerot[0] = atof(argv[1]);
  eerot[1] = atof(argv[2]);
  eerot[2] = atof(argv[3]);
  eetrans[0] = atof(argv[4]);
  eerot[3] = atof(argv[5]);
  eerot[4] = atof(argv[6]);
  eerot[5] = atof(argv[7]);
  eetrans[1] = atof(argv[8]);
  eerot[6] = atof(argv[9]);
  eerot[7] = atof(argv[10]);
  eerot[8] = atof(argv[11]);
  eetrans[2] = atof(argv[12]);
  for (std::size_t i = 0; i < vfree.size(); ++i) vfree[i] = atof(argv[13 + i]);
  bool bSuccess = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);

  if (!bSuccess) {
    fprintf(stderr, "Failed to get ik solution\n");
    return -1;
  }

  printf("Found %d ik solutions:\n", (int)solutions.GetNumSolutions());
  std::vector<IkReal> solvalues(GetNumJoints());
  for (std::size_t i = 0; i < solutions.GetNumSolutions(); ++i) {
    const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
    printf("sol%d (free=%d): ", (int)i, (int)sol.GetFree().size());
    std::vector<IkReal> vsolfree(sol.GetFree().size());
    sol.GetSolution(&solvalues[0], vsolfree.size() > 0 ? &vsolfree[0] : NULL);
    for (std::size_t j = 0; j < solvalues.size(); ++j) printf("%.15f, ", solvalues[j]);
    printf("\n");
  }
  return 0;
}*/
