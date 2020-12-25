/* Produced by CVXGEN, 2020-12-10 23:07:19 -0500.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: matrix_support.c. */
/* Description: Support functions for matrix multiplication and vector filling. */
#include "com_solver.h"
void com_multbymA(double *lhs, double *rhs) {
}
void com_multbymAT(double *lhs, double *rhs) {
  lhs[0] = 0;
  lhs[1] = 0;
}
void com_multbymG(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(com_params.NeJc[0])-rhs[1]*(com_params.NeJc[6]);
  lhs[1] = -rhs[0]*(com_params.NeJc[1])-rhs[1]*(com_params.NeJc[7]);
  lhs[2] = -rhs[0]*(com_params.NeJc[2])-rhs[1]*(com_params.NeJc[8]);
  lhs[3] = -rhs[0]*(com_params.NeJc[3])-rhs[1]*(com_params.NeJc[9]);
  lhs[4] = -rhs[0]*(com_params.NeJc[4])-rhs[1]*(com_params.NeJc[10]);
  lhs[5] = -rhs[0]*(com_params.NeJc[5])-rhs[1]*(com_params.NeJc[11]);
}
void com_multbymGT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(com_params.NeJc[0])-rhs[1]*(com_params.NeJc[1])-rhs[2]*(com_params.NeJc[2])-rhs[3]*(com_params.NeJc[3])-rhs[4]*(com_params.NeJc[4])-rhs[5]*(com_params.NeJc[5]);
  lhs[1] = -rhs[0]*(com_params.NeJc[6])-rhs[1]*(com_params.NeJc[7])-rhs[2]*(com_params.NeJc[8])-rhs[3]*(com_params.NeJc[9])-rhs[4]*(com_params.NeJc[10])-rhs[5]*(com_params.NeJc[11]);
}
void com_multbyP(double *lhs, double *rhs) {
  /* TODO use the fact that P is symmetric? */
  /* TODO check doubling / half factor etc. */
  lhs[0] = rhs[0]*(2*com_params.Q[0])+rhs[1]*(2*com_params.Q[2]);
  lhs[1] = rhs[0]*(2*com_params.Q[1])+rhs[1]*(2*com_params.Q[3]);
}
void com_fillq(void) {
  com_work.q[0] = com_params.P[0];
  com_work.q[1] = com_params.P[1];
}
void com_fillh(void) {
  com_work.h[0] = com_params.deXc[0];
  com_work.h[1] = com_params.deXc[1];
  com_work.h[2] = com_params.deXc[2];
  com_work.h[3] = com_params.deXc[3];
  com_work.h[4] = com_params.deXc[4];
  com_work.h[5] = com_params.deXc[5];
}
void com_fillb(void) {
}
void com_pre_ops(void) {
}
