/* Produced by CVXGEN, 2020-10-04 06:20:49 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: matrix_support.c. */
/* Description: Support functions for matrix multiplication and vector filling. */
#include "CoMclp_solver.h"
void CoMclp_multbymA(double *lhs, double *rhs) {
}
void CoMclp_multbymAT(double *lhs, double *rhs) {
  lhs[0] = 0;
  lhs[1] = 0;
}
void CoMclp_multbymG(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(CoMclp_params.NeJc[0])-rhs[1]*(CoMclp_params.NeJc[6]);
  lhs[1] = -rhs[0]*(CoMclp_params.NeJc[1])-rhs[1]*(CoMclp_params.NeJc[7]);
  lhs[2] = -rhs[0]*(CoMclp_params.NeJc[2])-rhs[1]*(CoMclp_params.NeJc[8]);
  lhs[3] = -rhs[0]*(CoMclp_params.NeJc[3])-rhs[1]*(CoMclp_params.NeJc[9]);
  lhs[4] = -rhs[0]*(CoMclp_params.NeJc[4])-rhs[1]*(CoMclp_params.NeJc[10]);
  lhs[5] = -rhs[0]*(CoMclp_params.NeJc[5])-rhs[1]*(CoMclp_params.NeJc[11]);
}
void CoMclp_multbymGT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(CoMclp_params.NeJc[0])-rhs[1]*(CoMclp_params.NeJc[1])-rhs[2]*(CoMclp_params.NeJc[2])-rhs[3]*(CoMclp_params.NeJc[3])-rhs[4]*(CoMclp_params.NeJc[4])-rhs[5]*(CoMclp_params.NeJc[5]);
  lhs[1] = -rhs[0]*(CoMclp_params.NeJc[6])-rhs[1]*(CoMclp_params.NeJc[7])-rhs[2]*(CoMclp_params.NeJc[8])-rhs[3]*(CoMclp_params.NeJc[9])-rhs[4]*(CoMclp_params.NeJc[10])-rhs[5]*(CoMclp_params.NeJc[11]);
}
void CoMclp_multbyP(double *lhs, double *rhs) {
  /* TODO use the fact that P is symmetric? */
  /* TODO check doubling / half factor etc. */
  lhs[0] = rhs[0]*(2*CoMclp_params.Q[0])+rhs[1]*(2*CoMclp_params.Q[2]);
  lhs[1] = rhs[0]*(2*CoMclp_params.Q[1])+rhs[1]*(2*CoMclp_params.Q[3]);
}
void CoMclp_fillq(void) {
  CoMclp_work.q[0] = CoMclp_params.P[0];
  CoMclp_work.q[1] = CoMclp_params.P[1];
}
void CoMclp_fillh(void) {
  CoMclp_work.h[0] = CoMclp_params.deXc[0];
  CoMclp_work.h[1] = CoMclp_params.deXc[1];
  CoMclp_work.h[2] = CoMclp_params.deXc[2];
  CoMclp_work.h[3] = CoMclp_params.deXc[3];
  CoMclp_work.h[4] = CoMclp_params.deXc[4];
  CoMclp_work.h[5] = CoMclp_params.deXc[5];
}
void CoMclp_fillb(void) {
}
void CoMclp_pre_ops(void) {
}
