/* Produced by CVXGEN, 2020-12-10 23:07:19 -0500.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: matrix_support.c. */
/* Description: Support functions for matrix multiplication and vector filling. */
#include "cop_solver.h"
void cop_multbymA(double *lhs, double *rhs) {
}
void cop_multbymAT(double *lhs, double *rhs) {
  lhs[0] = 0;
  lhs[1] = 0;
}
void cop_multbymG(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(cop_params.NeJc[0])-rhs[1]*(cop_params.NeJc[6]);
  lhs[1] = -rhs[0]*(cop_params.NeJc[1])-rhs[1]*(cop_params.NeJc[7]);
  lhs[2] = -rhs[0]*(cop_params.NeJc[2])-rhs[1]*(cop_params.NeJc[8]);
  lhs[3] = -rhs[0]*(cop_params.NeJc[3])-rhs[1]*(cop_params.NeJc[9]);
  lhs[4] = -rhs[0]*(cop_params.NeJc[4])-rhs[1]*(cop_params.NeJc[10]);
  lhs[5] = -rhs[0]*(cop_params.NeJc[5])-rhs[1]*(cop_params.NeJc[11]);
}
void cop_multbymGT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(cop_params.NeJc[0])-rhs[1]*(cop_params.NeJc[1])-rhs[2]*(cop_params.NeJc[2])-rhs[3]*(cop_params.NeJc[3])-rhs[4]*(cop_params.NeJc[4])-rhs[5]*(cop_params.NeJc[5]);
  lhs[1] = -rhs[0]*(cop_params.NeJc[6])-rhs[1]*(cop_params.NeJc[7])-rhs[2]*(cop_params.NeJc[8])-rhs[3]*(cop_params.NeJc[9])-rhs[4]*(cop_params.NeJc[10])-rhs[5]*(cop_params.NeJc[11]);
}
void cop_multbyP(double *lhs, double *rhs) {
  /* TODO use the fact that P is symmetric? */
  /* TODO check doubling / half factor etc. */
  lhs[0] = rhs[0]*(2*cop_params.Q[0])+rhs[1]*(2*cop_params.Q[2]);
  lhs[1] = rhs[0]*(2*cop_params.Q[1])+rhs[1]*(2*cop_params.Q[3]);
}
void cop_fillq(void) {
  cop_work.q[0] = cop_params.P[0];
  cop_work.q[1] = cop_params.P[1];
}
void cop_fillh(void) {
  cop_work.h[0] = cop_params.deXc[0];
  cop_work.h[1] = cop_params.deXc[1];
  cop_work.h[2] = cop_params.deXc[2];
  cop_work.h[3] = cop_params.deXc[3];
  cop_work.h[4] = cop_params.deXc[4];
  cop_work.h[5] = cop_params.deXc[5];
}
void cop_fillb(void) {
}
void cop_pre_ops(void) {
}
