/* Produced by CVXGEN, 2020-10-04 06:20:49 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: ldl.c. */
/* Description: Basic test harness for solver.c. */
#include "CoMclp_solver.h"
/* Be sure to place ldl_solve first, so storage schemes are defined by it. */
void CoMclp_ldl_CoMclp_solve(double *target, double *var) {
  int i;
  /* Find var = (L*diag(CoMclp_work.d)*L') \ target, then unpermute. */
  /* Answer goes into var. */
  /* Forward substitution. */
  /* Include permutation as we retrieve from target. Use v so we can unpermute */
  /* later. */
  CoMclp_work.v[0] = target[2];
  CoMclp_work.v[1] = target[3];
  CoMclp_work.v[2] = target[4];
  CoMclp_work.v[3] = target[5];
  CoMclp_work.v[4] = target[6];
  CoMclp_work.v[5] = target[7];
  CoMclp_work.v[6] = target[8]-CoMclp_work.L[0]*CoMclp_work.v[0];
  CoMclp_work.v[7] = target[9]-CoMclp_work.L[1]*CoMclp_work.v[1];
  CoMclp_work.v[8] = target[10]-CoMclp_work.L[2]*CoMclp_work.v[2];
  CoMclp_work.v[9] = target[11]-CoMclp_work.L[3]*CoMclp_work.v[3];
  CoMclp_work.v[10] = target[12]-CoMclp_work.L[4]*CoMclp_work.v[4];
  CoMclp_work.v[11] = target[0]-CoMclp_work.L[5]*CoMclp_work.v[6]-CoMclp_work.L[6]*CoMclp_work.v[7]-CoMclp_work.L[7]*CoMclp_work.v[8]-CoMclp_work.L[8]*CoMclp_work.v[9]-CoMclp_work.L[9]*CoMclp_work.v[10];
  CoMclp_work.v[12] = target[1]-CoMclp_work.L[10]*CoMclp_work.v[6]-CoMclp_work.L[11]*CoMclp_work.v[7]-CoMclp_work.L[12]*CoMclp_work.v[8]-CoMclp_work.L[13]*CoMclp_work.v[9]-CoMclp_work.L[14]*CoMclp_work.v[10]-CoMclp_work.L[15]*CoMclp_work.v[11];
  CoMclp_work.v[13] = target[13]-CoMclp_work.L[16]*CoMclp_work.v[5]-CoMclp_work.L[17]*CoMclp_work.v[11]-CoMclp_work.L[18]*CoMclp_work.v[12];
  /* Diagonal scaling. Assume correctness of CoMclp_work.d_inv. */
  for (i = 0; i < 14; i++)
    CoMclp_work.v[i] *= CoMclp_work.d_inv[i];
  /* Back substitution */
  CoMclp_work.v[12] -= CoMclp_work.L[18]*CoMclp_work.v[13];
  CoMclp_work.v[11] -= CoMclp_work.L[15]*CoMclp_work.v[12]+CoMclp_work.L[17]*CoMclp_work.v[13];
  CoMclp_work.v[10] -= CoMclp_work.L[9]*CoMclp_work.v[11]+CoMclp_work.L[14]*CoMclp_work.v[12];
  CoMclp_work.v[9] -= CoMclp_work.L[8]*CoMclp_work.v[11]+CoMclp_work.L[13]*CoMclp_work.v[12];
  CoMclp_work.v[8] -= CoMclp_work.L[7]*CoMclp_work.v[11]+CoMclp_work.L[12]*CoMclp_work.v[12];
  CoMclp_work.v[7] -= CoMclp_work.L[6]*CoMclp_work.v[11]+CoMclp_work.L[11]*CoMclp_work.v[12];
  CoMclp_work.v[6] -= CoMclp_work.L[5]*CoMclp_work.v[11]+CoMclp_work.L[10]*CoMclp_work.v[12];
  CoMclp_work.v[5] -= CoMclp_work.L[16]*CoMclp_work.v[13];
  CoMclp_work.v[4] -= CoMclp_work.L[4]*CoMclp_work.v[10];
  CoMclp_work.v[3] -= CoMclp_work.L[3]*CoMclp_work.v[9];
  CoMclp_work.v[2] -= CoMclp_work.L[2]*CoMclp_work.v[8];
  CoMclp_work.v[1] -= CoMclp_work.L[1]*CoMclp_work.v[7];
  CoMclp_work.v[0] -= CoMclp_work.L[0]*CoMclp_work.v[6];
  /* Unpermute the result, from v to var. */
  var[0] = CoMclp_work.v[11];
  var[1] = CoMclp_work.v[12];
  var[2] = CoMclp_work.v[0];
  var[3] = CoMclp_work.v[1];
  var[4] = CoMclp_work.v[2];
  var[5] = CoMclp_work.v[3];
  var[6] = CoMclp_work.v[4];
  var[7] = CoMclp_work.v[5];
  var[8] = CoMclp_work.v[6];
  var[9] = CoMclp_work.v[7];
  var[10] = CoMclp_work.v[8];
  var[11] = CoMclp_work.v[9];
  var[12] = CoMclp_work.v[10];
  var[13] = CoMclp_work.v[13];
#ifndef ZERO_LIBRARY_MODE
  if (CoMclp_settings.debug) {
    printf("Squared norm for solution is %.8g.\n", CoMclp_check_residual(target, var));
  }
#endif
}
void CoMclp_ldl_factor(void) {
  CoMclp_work.d[0] = CoMclp_work.KKT[0];
  if (CoMclp_work.d[0] < 0)
    CoMclp_work.d[0] = CoMclp_settings.kkt_reg;
  else
    CoMclp_work.d[0] += CoMclp_settings.kkt_reg;
  CoMclp_work.d_inv[0] = 1/CoMclp_work.d[0];
  CoMclp_work.L[0] = CoMclp_work.KKT[1]*CoMclp_work.d_inv[0];
  CoMclp_work.v[1] = CoMclp_work.KKT[2];
  CoMclp_work.d[1] = CoMclp_work.v[1];
  if (CoMclp_work.d[1] < 0)
    CoMclp_work.d[1] = CoMclp_settings.kkt_reg;
  else
    CoMclp_work.d[1] += CoMclp_settings.kkt_reg;
  CoMclp_work.d_inv[1] = 1/CoMclp_work.d[1];
  CoMclp_work.L[1] = (CoMclp_work.KKT[3])*CoMclp_work.d_inv[1];
  CoMclp_work.v[2] = CoMclp_work.KKT[4];
  CoMclp_work.d[2] = CoMclp_work.v[2];
  if (CoMclp_work.d[2] < 0)
    CoMclp_work.d[2] = CoMclp_settings.kkt_reg;
  else
    CoMclp_work.d[2] += CoMclp_settings.kkt_reg;
  CoMclp_work.d_inv[2] = 1/CoMclp_work.d[2];
  CoMclp_work.L[2] = (CoMclp_work.KKT[5])*CoMclp_work.d_inv[2];
  CoMclp_work.v[3] = CoMclp_work.KKT[6];
  CoMclp_work.d[3] = CoMclp_work.v[3];
  if (CoMclp_work.d[3] < 0)
    CoMclp_work.d[3] = CoMclp_settings.kkt_reg;
  else
    CoMclp_work.d[3] += CoMclp_settings.kkt_reg;
  CoMclp_work.d_inv[3] = 1/CoMclp_work.d[3];
  CoMclp_work.L[3] = (CoMclp_work.KKT[7])*CoMclp_work.d_inv[3];
  CoMclp_work.v[4] = CoMclp_work.KKT[8];
  CoMclp_work.d[4] = CoMclp_work.v[4];
  if (CoMclp_work.d[4] < 0)
    CoMclp_work.d[4] = CoMclp_settings.kkt_reg;
  else
    CoMclp_work.d[4] += CoMclp_settings.kkt_reg;
  CoMclp_work.d_inv[4] = 1/CoMclp_work.d[4];
  CoMclp_work.L[4] = (CoMclp_work.KKT[9])*CoMclp_work.d_inv[4];
  CoMclp_work.v[5] = CoMclp_work.KKT[10];
  CoMclp_work.d[5] = CoMclp_work.v[5];
  if (CoMclp_work.d[5] < 0)
    CoMclp_work.d[5] = CoMclp_settings.kkt_reg;
  else
    CoMclp_work.d[5] += CoMclp_settings.kkt_reg;
  CoMclp_work.d_inv[5] = 1/CoMclp_work.d[5];
  CoMclp_work.L[16] = (CoMclp_work.KKT[11])*CoMclp_work.d_inv[5];
  CoMclp_work.v[0] = CoMclp_work.L[0]*CoMclp_work.d[0];
  CoMclp_work.v[6] = CoMclp_work.KKT[12]-CoMclp_work.L[0]*CoMclp_work.v[0];
  CoMclp_work.d[6] = CoMclp_work.v[6];
  if (CoMclp_work.d[6] > 0)
    CoMclp_work.d[6] = -CoMclp_settings.kkt_reg;
  else
    CoMclp_work.d[6] -= CoMclp_settings.kkt_reg;
  CoMclp_work.d_inv[6] = 1/CoMclp_work.d[6];
  CoMclp_work.L[5] = (CoMclp_work.KKT[13])*CoMclp_work.d_inv[6];
  CoMclp_work.L[10] = (CoMclp_work.KKT[14])*CoMclp_work.d_inv[6];
  CoMclp_work.v[1] = CoMclp_work.L[1]*CoMclp_work.d[1];
  CoMclp_work.v[7] = CoMclp_work.KKT[15]-CoMclp_work.L[1]*CoMclp_work.v[1];
  CoMclp_work.d[7] = CoMclp_work.v[7];
  if (CoMclp_work.d[7] > 0)
    CoMclp_work.d[7] = -CoMclp_settings.kkt_reg;
  else
    CoMclp_work.d[7] -= CoMclp_settings.kkt_reg;
  CoMclp_work.d_inv[7] = 1/CoMclp_work.d[7];
  CoMclp_work.L[6] = (CoMclp_work.KKT[16])*CoMclp_work.d_inv[7];
  CoMclp_work.L[11] = (CoMclp_work.KKT[17])*CoMclp_work.d_inv[7];
  CoMclp_work.v[2] = CoMclp_work.L[2]*CoMclp_work.d[2];
  CoMclp_work.v[8] = CoMclp_work.KKT[18]-CoMclp_work.L[2]*CoMclp_work.v[2];
  CoMclp_work.d[8] = CoMclp_work.v[8];
  if (CoMclp_work.d[8] > 0)
    CoMclp_work.d[8] = -CoMclp_settings.kkt_reg;
  else
    CoMclp_work.d[8] -= CoMclp_settings.kkt_reg;
  CoMclp_work.d_inv[8] = 1/CoMclp_work.d[8];
  CoMclp_work.L[7] = (CoMclp_work.KKT[19])*CoMclp_work.d_inv[8];
  CoMclp_work.L[12] = (CoMclp_work.KKT[20])*CoMclp_work.d_inv[8];
  CoMclp_work.v[3] = CoMclp_work.L[3]*CoMclp_work.d[3];
  CoMclp_work.v[9] = CoMclp_work.KKT[21]-CoMclp_work.L[3]*CoMclp_work.v[3];
  CoMclp_work.d[9] = CoMclp_work.v[9];
  if (CoMclp_work.d[9] > 0)
    CoMclp_work.d[9] = -CoMclp_settings.kkt_reg;
  else
    CoMclp_work.d[9] -= CoMclp_settings.kkt_reg;
  CoMclp_work.d_inv[9] = 1/CoMclp_work.d[9];
  CoMclp_work.L[8] = (CoMclp_work.KKT[22])*CoMclp_work.d_inv[9];
  CoMclp_work.L[13] = (CoMclp_work.KKT[23])*CoMclp_work.d_inv[9];
  CoMclp_work.v[4] = CoMclp_work.L[4]*CoMclp_work.d[4];
  CoMclp_work.v[10] = CoMclp_work.KKT[24]-CoMclp_work.L[4]*CoMclp_work.v[4];
  CoMclp_work.d[10] = CoMclp_work.v[10];
  if (CoMclp_work.d[10] > 0)
    CoMclp_work.d[10] = -CoMclp_settings.kkt_reg;
  else
    CoMclp_work.d[10] -= CoMclp_settings.kkt_reg;
  CoMclp_work.d_inv[10] = 1/CoMclp_work.d[10];
  CoMclp_work.L[9] = (CoMclp_work.KKT[25])*CoMclp_work.d_inv[10];
  CoMclp_work.L[14] = (CoMclp_work.KKT[26])*CoMclp_work.d_inv[10];
  CoMclp_work.v[6] = CoMclp_work.L[5]*CoMclp_work.d[6];
  CoMclp_work.v[7] = CoMclp_work.L[6]*CoMclp_work.d[7];
  CoMclp_work.v[8] = CoMclp_work.L[7]*CoMclp_work.d[8];
  CoMclp_work.v[9] = CoMclp_work.L[8]*CoMclp_work.d[9];
  CoMclp_work.v[10] = CoMclp_work.L[9]*CoMclp_work.d[10];
  CoMclp_work.v[11] = CoMclp_work.KKT[27]-CoMclp_work.L[5]*CoMclp_work.v[6]-CoMclp_work.L[6]*CoMclp_work.v[7]-CoMclp_work.L[7]*CoMclp_work.v[8]-CoMclp_work.L[8]*CoMclp_work.v[9]-CoMclp_work.L[9]*CoMclp_work.v[10];
  CoMclp_work.d[11] = CoMclp_work.v[11];
  if (CoMclp_work.d[11] < 0)
    CoMclp_work.d[11] = CoMclp_settings.kkt_reg;
  else
    CoMclp_work.d[11] += CoMclp_settings.kkt_reg;
  CoMclp_work.d_inv[11] = 1/CoMclp_work.d[11];
  CoMclp_work.L[15] = (CoMclp_work.KKT[28]-CoMclp_work.L[10]*CoMclp_work.v[6]-CoMclp_work.L[11]*CoMclp_work.v[7]-CoMclp_work.L[12]*CoMclp_work.v[8]-CoMclp_work.L[13]*CoMclp_work.v[9]-CoMclp_work.L[14]*CoMclp_work.v[10])*CoMclp_work.d_inv[11];
  CoMclp_work.L[17] = (CoMclp_work.KKT[29])*CoMclp_work.d_inv[11];
  CoMclp_work.v[6] = CoMclp_work.L[10]*CoMclp_work.d[6];
  CoMclp_work.v[7] = CoMclp_work.L[11]*CoMclp_work.d[7];
  CoMclp_work.v[8] = CoMclp_work.L[12]*CoMclp_work.d[8];
  CoMclp_work.v[9] = CoMclp_work.L[13]*CoMclp_work.d[9];
  CoMclp_work.v[10] = CoMclp_work.L[14]*CoMclp_work.d[10];
  CoMclp_work.v[11] = CoMclp_work.L[15]*CoMclp_work.d[11];
  CoMclp_work.v[12] = CoMclp_work.KKT[30]-CoMclp_work.L[10]*CoMclp_work.v[6]-CoMclp_work.L[11]*CoMclp_work.v[7]-CoMclp_work.L[12]*CoMclp_work.v[8]-CoMclp_work.L[13]*CoMclp_work.v[9]-CoMclp_work.L[14]*CoMclp_work.v[10]-CoMclp_work.L[15]*CoMclp_work.v[11];
  CoMclp_work.d[12] = CoMclp_work.v[12];
  if (CoMclp_work.d[12] < 0)
    CoMclp_work.d[12] = CoMclp_settings.kkt_reg;
  else
    CoMclp_work.d[12] += CoMclp_settings.kkt_reg;
  CoMclp_work.d_inv[12] = 1/CoMclp_work.d[12];
  CoMclp_work.L[18] = (CoMclp_work.KKT[31]-CoMclp_work.L[17]*CoMclp_work.v[11])*CoMclp_work.d_inv[12];
  CoMclp_work.v[5] = CoMclp_work.L[16]*CoMclp_work.d[5];
  CoMclp_work.v[11] = CoMclp_work.L[17]*CoMclp_work.d[11];
  CoMclp_work.v[12] = CoMclp_work.L[18]*CoMclp_work.d[12];
  CoMclp_work.v[13] = CoMclp_work.KKT[32]-CoMclp_work.L[16]*CoMclp_work.v[5]-CoMclp_work.L[17]*CoMclp_work.v[11]-CoMclp_work.L[18]*CoMclp_work.v[12];
  CoMclp_work.d[13] = CoMclp_work.v[13];
  if (CoMclp_work.d[13] > 0)
    CoMclp_work.d[13] = -CoMclp_settings.kkt_reg;
  else
    CoMclp_work.d[13] -= CoMclp_settings.kkt_reg;
  CoMclp_work.d_inv[13] = 1/CoMclp_work.d[13];
#ifndef ZERO_LIBRARY_MODE
  if (CoMclp_settings.debug) {
    printf("Squared Frobenius for factorization is %.8g.\n", CoMclp_check_factorization());
  }
#endif
}
double CoMclp_check_factorization(void) {
  /* Returns the squared Frobenius norm of A - L*D*L'. */
  double temp, residual;
  /* Only check the lower triangle. */
  residual = 0;
  temp = CoMclp_work.KKT[27]-1*CoMclp_work.d[11]*1-CoMclp_work.L[5]*CoMclp_work.d[6]*CoMclp_work.L[5]-CoMclp_work.L[6]*CoMclp_work.d[7]*CoMclp_work.L[6]-CoMclp_work.L[7]*CoMclp_work.d[8]*CoMclp_work.L[7]-CoMclp_work.L[8]*CoMclp_work.d[9]*CoMclp_work.L[8]-CoMclp_work.L[9]*CoMclp_work.d[10]*CoMclp_work.L[9];
  residual += temp*temp;
  temp = CoMclp_work.KKT[28]-CoMclp_work.L[15]*CoMclp_work.d[11]*1-CoMclp_work.L[10]*CoMclp_work.d[6]*CoMclp_work.L[5]-CoMclp_work.L[11]*CoMclp_work.d[7]*CoMclp_work.L[6]-CoMclp_work.L[12]*CoMclp_work.d[8]*CoMclp_work.L[7]-CoMclp_work.L[13]*CoMclp_work.d[9]*CoMclp_work.L[8]-CoMclp_work.L[14]*CoMclp_work.d[10]*CoMclp_work.L[9];
  residual += temp*temp;
  temp = CoMclp_work.KKT[30]-CoMclp_work.L[15]*CoMclp_work.d[11]*CoMclp_work.L[15]-1*CoMclp_work.d[12]*1-CoMclp_work.L[10]*CoMclp_work.d[6]*CoMclp_work.L[10]-CoMclp_work.L[11]*CoMclp_work.d[7]*CoMclp_work.L[11]-CoMclp_work.L[12]*CoMclp_work.d[8]*CoMclp_work.L[12]-CoMclp_work.L[13]*CoMclp_work.d[9]*CoMclp_work.L[13]-CoMclp_work.L[14]*CoMclp_work.d[10]*CoMclp_work.L[14];
  residual += temp*temp;
  temp = CoMclp_work.KKT[0]-1*CoMclp_work.d[0]*1;
  residual += temp*temp;
  temp = CoMclp_work.KKT[2]-1*CoMclp_work.d[1]*1;
  residual += temp*temp;
  temp = CoMclp_work.KKT[4]-1*CoMclp_work.d[2]*1;
  residual += temp*temp;
  temp = CoMclp_work.KKT[6]-1*CoMclp_work.d[3]*1;
  residual += temp*temp;
  temp = CoMclp_work.KKT[8]-1*CoMclp_work.d[4]*1;
  residual += temp*temp;
  temp = CoMclp_work.KKT[10]-1*CoMclp_work.d[5]*1;
  residual += temp*temp;
  temp = CoMclp_work.KKT[1]-CoMclp_work.L[0]*CoMclp_work.d[0]*1;
  residual += temp*temp;
  temp = CoMclp_work.KKT[3]-CoMclp_work.L[1]*CoMclp_work.d[1]*1;
  residual += temp*temp;
  temp = CoMclp_work.KKT[5]-CoMclp_work.L[2]*CoMclp_work.d[2]*1;
  residual += temp*temp;
  temp = CoMclp_work.KKT[7]-CoMclp_work.L[3]*CoMclp_work.d[3]*1;
  residual += temp*temp;
  temp = CoMclp_work.KKT[9]-CoMclp_work.L[4]*CoMclp_work.d[4]*1;
  residual += temp*temp;
  temp = CoMclp_work.KKT[11]-CoMclp_work.L[16]*CoMclp_work.d[5]*1;
  residual += temp*temp;
  temp = CoMclp_work.KKT[12]-CoMclp_work.L[0]*CoMclp_work.d[0]*CoMclp_work.L[0]-1*CoMclp_work.d[6]*1;
  residual += temp*temp;
  temp = CoMclp_work.KKT[15]-CoMclp_work.L[1]*CoMclp_work.d[1]*CoMclp_work.L[1]-1*CoMclp_work.d[7]*1;
  residual += temp*temp;
  temp = CoMclp_work.KKT[18]-CoMclp_work.L[2]*CoMclp_work.d[2]*CoMclp_work.L[2]-1*CoMclp_work.d[8]*1;
  residual += temp*temp;
  temp = CoMclp_work.KKT[21]-CoMclp_work.L[3]*CoMclp_work.d[3]*CoMclp_work.L[3]-1*CoMclp_work.d[9]*1;
  residual += temp*temp;
  temp = CoMclp_work.KKT[24]-CoMclp_work.L[4]*CoMclp_work.d[4]*CoMclp_work.L[4]-1*CoMclp_work.d[10]*1;
  residual += temp*temp;
  temp = CoMclp_work.KKT[32]-CoMclp_work.L[16]*CoMclp_work.d[5]*CoMclp_work.L[16]-1*CoMclp_work.d[13]*1-CoMclp_work.L[17]*CoMclp_work.d[11]*CoMclp_work.L[17]-CoMclp_work.L[18]*CoMclp_work.d[12]*CoMclp_work.L[18];
  residual += temp*temp;
  temp = CoMclp_work.KKT[13]-1*CoMclp_work.d[6]*CoMclp_work.L[5];
  residual += temp*temp;
  temp = CoMclp_work.KKT[14]-1*CoMclp_work.d[6]*CoMclp_work.L[10];
  residual += temp*temp;
  temp = CoMclp_work.KKT[16]-1*CoMclp_work.d[7]*CoMclp_work.L[6];
  residual += temp*temp;
  temp = CoMclp_work.KKT[17]-1*CoMclp_work.d[7]*CoMclp_work.L[11];
  residual += temp*temp;
  temp = CoMclp_work.KKT[19]-1*CoMclp_work.d[8]*CoMclp_work.L[7];
  residual += temp*temp;
  temp = CoMclp_work.KKT[20]-1*CoMclp_work.d[8]*CoMclp_work.L[12];
  residual += temp*temp;
  temp = CoMclp_work.KKT[22]-1*CoMclp_work.d[9]*CoMclp_work.L[8];
  residual += temp*temp;
  temp = CoMclp_work.KKT[23]-1*CoMclp_work.d[9]*CoMclp_work.L[13];
  residual += temp*temp;
  temp = CoMclp_work.KKT[25]-1*CoMclp_work.d[10]*CoMclp_work.L[9];
  residual += temp*temp;
  temp = CoMclp_work.KKT[26]-1*CoMclp_work.d[10]*CoMclp_work.L[14];
  residual += temp*temp;
  temp = CoMclp_work.KKT[29]-CoMclp_work.L[17]*CoMclp_work.d[11]*1;
  residual += temp*temp;
  temp = CoMclp_work.KKT[31]-CoMclp_work.L[17]*CoMclp_work.d[11]*CoMclp_work.L[15]-CoMclp_work.L[18]*CoMclp_work.d[12]*1;
  residual += temp*temp;
  return residual;
}
void CoMclp_matrix_multiply(double *result, double *source) {
  /* Finds result = A*source. */
  result[0] = CoMclp_work.KKT[27]*source[0]+CoMclp_work.KKT[28]*source[1]+CoMclp_work.KKT[13]*source[8]+CoMclp_work.KKT[16]*source[9]+CoMclp_work.KKT[19]*source[10]+CoMclp_work.KKT[22]*source[11]+CoMclp_work.KKT[25]*source[12]+CoMclp_work.KKT[29]*source[13];
  result[1] = CoMclp_work.KKT[28]*source[0]+CoMclp_work.KKT[30]*source[1]+CoMclp_work.KKT[14]*source[8]+CoMclp_work.KKT[17]*source[9]+CoMclp_work.KKT[20]*source[10]+CoMclp_work.KKT[23]*source[11]+CoMclp_work.KKT[26]*source[12]+CoMclp_work.KKT[31]*source[13];
  result[2] = CoMclp_work.KKT[0]*source[2]+CoMclp_work.KKT[1]*source[8];
  result[3] = CoMclp_work.KKT[2]*source[3]+CoMclp_work.KKT[3]*source[9];
  result[4] = CoMclp_work.KKT[4]*source[4]+CoMclp_work.KKT[5]*source[10];
  result[5] = CoMclp_work.KKT[6]*source[5]+CoMclp_work.KKT[7]*source[11];
  result[6] = CoMclp_work.KKT[8]*source[6]+CoMclp_work.KKT[9]*source[12];
  result[7] = CoMclp_work.KKT[10]*source[7]+CoMclp_work.KKT[11]*source[13];
  result[8] = CoMclp_work.KKT[1]*source[2]+CoMclp_work.KKT[12]*source[8]+CoMclp_work.KKT[13]*source[0]+CoMclp_work.KKT[14]*source[1];
  result[9] = CoMclp_work.KKT[3]*source[3]+CoMclp_work.KKT[15]*source[9]+CoMclp_work.KKT[16]*source[0]+CoMclp_work.KKT[17]*source[1];
  result[10] = CoMclp_work.KKT[5]*source[4]+CoMclp_work.KKT[18]*source[10]+CoMclp_work.KKT[19]*source[0]+CoMclp_work.KKT[20]*source[1];
  result[11] = CoMclp_work.KKT[7]*source[5]+CoMclp_work.KKT[21]*source[11]+CoMclp_work.KKT[22]*source[0]+CoMclp_work.KKT[23]*source[1];
  result[12] = CoMclp_work.KKT[9]*source[6]+CoMclp_work.KKT[24]*source[12]+CoMclp_work.KKT[25]*source[0]+CoMclp_work.KKT[26]*source[1];
  result[13] = CoMclp_work.KKT[11]*source[7]+CoMclp_work.KKT[32]*source[13]+CoMclp_work.KKT[29]*source[0]+CoMclp_work.KKT[31]*source[1];
}
double CoMclp_check_residual(double *target, double *multiplicand) {
  /* Returns the squared 2-norm of lhs - A*rhs. */
  /* Reuses v to find the residual. */
  int i;
  double residual;
  residual = 0;
  CoMclp_matrix_multiply(CoMclp_work.v, multiplicand);
  for (i = 0; i < 2; i++) {
    residual += (target[i] - CoMclp_work.v[i])*(target[i] - CoMclp_work.v[i]);
  }
  return residual;
}
void CoMclp_fill_KKT(void) {
  CoMclp_work.KKT[27] = 2*CoMclp_params.Q[0];
  CoMclp_work.KKT[28] = 2*CoMclp_params.Q[2];
  CoMclp_work.KKT[30] = 2*CoMclp_params.Q[3];
  CoMclp_work.KKT[0] = CoMclp_work.s_inv_z[0];
  CoMclp_work.KKT[2] = CoMclp_work.s_inv_z[1];
  CoMclp_work.KKT[4] = CoMclp_work.s_inv_z[2];
  CoMclp_work.KKT[6] = CoMclp_work.s_inv_z[3];
  CoMclp_work.KKT[8] = CoMclp_work.s_inv_z[4];
  CoMclp_work.KKT[10] = CoMclp_work.s_inv_z[5];
  CoMclp_work.KKT[1] = 1;
  CoMclp_work.KKT[3] = 1;
  CoMclp_work.KKT[5] = 1;
  CoMclp_work.KKT[7] = 1;
  CoMclp_work.KKT[9] = 1;
  CoMclp_work.KKT[11] = 1;
  CoMclp_work.KKT[12] = CoMclp_work.block_33[0];
  CoMclp_work.KKT[15] = CoMclp_work.block_33[0];
  CoMclp_work.KKT[18] = CoMclp_work.block_33[0];
  CoMclp_work.KKT[21] = CoMclp_work.block_33[0];
  CoMclp_work.KKT[24] = CoMclp_work.block_33[0];
  CoMclp_work.KKT[32] = CoMclp_work.block_33[0];
  CoMclp_work.KKT[13] = CoMclp_params.NeJc[0];
  CoMclp_work.KKT[14] = CoMclp_params.NeJc[6];
  CoMclp_work.KKT[16] = CoMclp_params.NeJc[1];
  CoMclp_work.KKT[17] = CoMclp_params.NeJc[7];
  CoMclp_work.KKT[19] = CoMclp_params.NeJc[2];
  CoMclp_work.KKT[20] = CoMclp_params.NeJc[8];
  CoMclp_work.KKT[22] = CoMclp_params.NeJc[3];
  CoMclp_work.KKT[23] = CoMclp_params.NeJc[9];
  CoMclp_work.KKT[25] = CoMclp_params.NeJc[4];
  CoMclp_work.KKT[26] = CoMclp_params.NeJc[10];
  CoMclp_work.KKT[29] = CoMclp_params.NeJc[5];
  CoMclp_work.KKT[31] = CoMclp_params.NeJc[11];
}
