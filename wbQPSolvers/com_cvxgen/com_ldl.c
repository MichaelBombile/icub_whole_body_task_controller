/* Produced by CVXGEN, 2020-12-10 23:07:19 -0500.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: ldl.c. */
/* Description: Basic test harness for solver.c. */
#include "com_solver.h"
/* Be sure to place ldl_solve first, so storage schemes are defined by it. */
void com_ldl_com_solve(double *target, double *var) {
  int i;
  /* Find var = (L*diag(com_work.d)*L') \ target, then unpermute. */
  /* Answer goes into var. */
  /* Forward substitution. */
  /* Include permutation as we retrieve from target. Use v so we can unpermute */
  /* later. */
  com_work.v[0] = target[2];
  com_work.v[1] = target[3];
  com_work.v[2] = target[4];
  com_work.v[3] = target[5];
  com_work.v[4] = target[6];
  com_work.v[5] = target[7];
  com_work.v[6] = target[8]-com_work.L[0]*com_work.v[0];
  com_work.v[7] = target[9]-com_work.L[1]*com_work.v[1];
  com_work.v[8] = target[10]-com_work.L[2]*com_work.v[2];
  com_work.v[9] = target[11]-com_work.L[3]*com_work.v[3];
  com_work.v[10] = target[12]-com_work.L[4]*com_work.v[4];
  com_work.v[11] = target[0]-com_work.L[5]*com_work.v[6]-com_work.L[6]*com_work.v[7]-com_work.L[7]*com_work.v[8]-com_work.L[8]*com_work.v[9]-com_work.L[9]*com_work.v[10];
  com_work.v[12] = target[1]-com_work.L[10]*com_work.v[6]-com_work.L[11]*com_work.v[7]-com_work.L[12]*com_work.v[8]-com_work.L[13]*com_work.v[9]-com_work.L[14]*com_work.v[10]-com_work.L[15]*com_work.v[11];
  com_work.v[13] = target[13]-com_work.L[16]*com_work.v[5]-com_work.L[17]*com_work.v[11]-com_work.L[18]*com_work.v[12];
  /* Diagonal scaling. Assume correctness of com_work.d_inv. */
  for (i = 0; i < 14; i++)
    com_work.v[i] *= com_work.d_inv[i];
  /* Back substitution */
  com_work.v[12] -= com_work.L[18]*com_work.v[13];
  com_work.v[11] -= com_work.L[15]*com_work.v[12]+com_work.L[17]*com_work.v[13];
  com_work.v[10] -= com_work.L[9]*com_work.v[11]+com_work.L[14]*com_work.v[12];
  com_work.v[9] -= com_work.L[8]*com_work.v[11]+com_work.L[13]*com_work.v[12];
  com_work.v[8] -= com_work.L[7]*com_work.v[11]+com_work.L[12]*com_work.v[12];
  com_work.v[7] -= com_work.L[6]*com_work.v[11]+com_work.L[11]*com_work.v[12];
  com_work.v[6] -= com_work.L[5]*com_work.v[11]+com_work.L[10]*com_work.v[12];
  com_work.v[5] -= com_work.L[16]*com_work.v[13];
  com_work.v[4] -= com_work.L[4]*com_work.v[10];
  com_work.v[3] -= com_work.L[3]*com_work.v[9];
  com_work.v[2] -= com_work.L[2]*com_work.v[8];
  com_work.v[1] -= com_work.L[1]*com_work.v[7];
  com_work.v[0] -= com_work.L[0]*com_work.v[6];
  /* Unpermute the result, from v to var. */
  var[0] = com_work.v[11];
  var[1] = com_work.v[12];
  var[2] = com_work.v[0];
  var[3] = com_work.v[1];
  var[4] = com_work.v[2];
  var[5] = com_work.v[3];
  var[6] = com_work.v[4];
  var[7] = com_work.v[5];
  var[8] = com_work.v[6];
  var[9] = com_work.v[7];
  var[10] = com_work.v[8];
  var[11] = com_work.v[9];
  var[12] = com_work.v[10];
  var[13] = com_work.v[13];
#ifndef ZERO_LIBRARY_MODE
  if (com_settings.debug) {
    printf("Squared norm for solution is %.8g.\n", com_check_residual(target, var));
  }
#endif
}
void com_ldl_factor(void) {
  com_work.d[0] = com_work.KKT[0];
  if (com_work.d[0] < 0)
    com_work.d[0] = com_settings.kkt_reg;
  else
    com_work.d[0] += com_settings.kkt_reg;
  com_work.d_inv[0] = 1/com_work.d[0];
  com_work.L[0] = com_work.KKT[1]*com_work.d_inv[0];
  com_work.v[1] = com_work.KKT[2];
  com_work.d[1] = com_work.v[1];
  if (com_work.d[1] < 0)
    com_work.d[1] = com_settings.kkt_reg;
  else
    com_work.d[1] += com_settings.kkt_reg;
  com_work.d_inv[1] = 1/com_work.d[1];
  com_work.L[1] = (com_work.KKT[3])*com_work.d_inv[1];
  com_work.v[2] = com_work.KKT[4];
  com_work.d[2] = com_work.v[2];
  if (com_work.d[2] < 0)
    com_work.d[2] = com_settings.kkt_reg;
  else
    com_work.d[2] += com_settings.kkt_reg;
  com_work.d_inv[2] = 1/com_work.d[2];
  com_work.L[2] = (com_work.KKT[5])*com_work.d_inv[2];
  com_work.v[3] = com_work.KKT[6];
  com_work.d[3] = com_work.v[3];
  if (com_work.d[3] < 0)
    com_work.d[3] = com_settings.kkt_reg;
  else
    com_work.d[3] += com_settings.kkt_reg;
  com_work.d_inv[3] = 1/com_work.d[3];
  com_work.L[3] = (com_work.KKT[7])*com_work.d_inv[3];
  com_work.v[4] = com_work.KKT[8];
  com_work.d[4] = com_work.v[4];
  if (com_work.d[4] < 0)
    com_work.d[4] = com_settings.kkt_reg;
  else
    com_work.d[4] += com_settings.kkt_reg;
  com_work.d_inv[4] = 1/com_work.d[4];
  com_work.L[4] = (com_work.KKT[9])*com_work.d_inv[4];
  com_work.v[5] = com_work.KKT[10];
  com_work.d[5] = com_work.v[5];
  if (com_work.d[5] < 0)
    com_work.d[5] = com_settings.kkt_reg;
  else
    com_work.d[5] += com_settings.kkt_reg;
  com_work.d_inv[5] = 1/com_work.d[5];
  com_work.L[16] = (com_work.KKT[11])*com_work.d_inv[5];
  com_work.v[0] = com_work.L[0]*com_work.d[0];
  com_work.v[6] = com_work.KKT[12]-com_work.L[0]*com_work.v[0];
  com_work.d[6] = com_work.v[6];
  if (com_work.d[6] > 0)
    com_work.d[6] = -com_settings.kkt_reg;
  else
    com_work.d[6] -= com_settings.kkt_reg;
  com_work.d_inv[6] = 1/com_work.d[6];
  com_work.L[5] = (com_work.KKT[13])*com_work.d_inv[6];
  com_work.L[10] = (com_work.KKT[14])*com_work.d_inv[6];
  com_work.v[1] = com_work.L[1]*com_work.d[1];
  com_work.v[7] = com_work.KKT[15]-com_work.L[1]*com_work.v[1];
  com_work.d[7] = com_work.v[7];
  if (com_work.d[7] > 0)
    com_work.d[7] = -com_settings.kkt_reg;
  else
    com_work.d[7] -= com_settings.kkt_reg;
  com_work.d_inv[7] = 1/com_work.d[7];
  com_work.L[6] = (com_work.KKT[16])*com_work.d_inv[7];
  com_work.L[11] = (com_work.KKT[17])*com_work.d_inv[7];
  com_work.v[2] = com_work.L[2]*com_work.d[2];
  com_work.v[8] = com_work.KKT[18]-com_work.L[2]*com_work.v[2];
  com_work.d[8] = com_work.v[8];
  if (com_work.d[8] > 0)
    com_work.d[8] = -com_settings.kkt_reg;
  else
    com_work.d[8] -= com_settings.kkt_reg;
  com_work.d_inv[8] = 1/com_work.d[8];
  com_work.L[7] = (com_work.KKT[19])*com_work.d_inv[8];
  com_work.L[12] = (com_work.KKT[20])*com_work.d_inv[8];
  com_work.v[3] = com_work.L[3]*com_work.d[3];
  com_work.v[9] = com_work.KKT[21]-com_work.L[3]*com_work.v[3];
  com_work.d[9] = com_work.v[9];
  if (com_work.d[9] > 0)
    com_work.d[9] = -com_settings.kkt_reg;
  else
    com_work.d[9] -= com_settings.kkt_reg;
  com_work.d_inv[9] = 1/com_work.d[9];
  com_work.L[8] = (com_work.KKT[22])*com_work.d_inv[9];
  com_work.L[13] = (com_work.KKT[23])*com_work.d_inv[9];
  com_work.v[4] = com_work.L[4]*com_work.d[4];
  com_work.v[10] = com_work.KKT[24]-com_work.L[4]*com_work.v[4];
  com_work.d[10] = com_work.v[10];
  if (com_work.d[10] > 0)
    com_work.d[10] = -com_settings.kkt_reg;
  else
    com_work.d[10] -= com_settings.kkt_reg;
  com_work.d_inv[10] = 1/com_work.d[10];
  com_work.L[9] = (com_work.KKT[25])*com_work.d_inv[10];
  com_work.L[14] = (com_work.KKT[26])*com_work.d_inv[10];
  com_work.v[6] = com_work.L[5]*com_work.d[6];
  com_work.v[7] = com_work.L[6]*com_work.d[7];
  com_work.v[8] = com_work.L[7]*com_work.d[8];
  com_work.v[9] = com_work.L[8]*com_work.d[9];
  com_work.v[10] = com_work.L[9]*com_work.d[10];
  com_work.v[11] = com_work.KKT[27]-com_work.L[5]*com_work.v[6]-com_work.L[6]*com_work.v[7]-com_work.L[7]*com_work.v[8]-com_work.L[8]*com_work.v[9]-com_work.L[9]*com_work.v[10];
  com_work.d[11] = com_work.v[11];
  if (com_work.d[11] < 0)
    com_work.d[11] = com_settings.kkt_reg;
  else
    com_work.d[11] += com_settings.kkt_reg;
  com_work.d_inv[11] = 1/com_work.d[11];
  com_work.L[15] = (com_work.KKT[28]-com_work.L[10]*com_work.v[6]-com_work.L[11]*com_work.v[7]-com_work.L[12]*com_work.v[8]-com_work.L[13]*com_work.v[9]-com_work.L[14]*com_work.v[10])*com_work.d_inv[11];
  com_work.L[17] = (com_work.KKT[29])*com_work.d_inv[11];
  com_work.v[6] = com_work.L[10]*com_work.d[6];
  com_work.v[7] = com_work.L[11]*com_work.d[7];
  com_work.v[8] = com_work.L[12]*com_work.d[8];
  com_work.v[9] = com_work.L[13]*com_work.d[9];
  com_work.v[10] = com_work.L[14]*com_work.d[10];
  com_work.v[11] = com_work.L[15]*com_work.d[11];
  com_work.v[12] = com_work.KKT[30]-com_work.L[10]*com_work.v[6]-com_work.L[11]*com_work.v[7]-com_work.L[12]*com_work.v[8]-com_work.L[13]*com_work.v[9]-com_work.L[14]*com_work.v[10]-com_work.L[15]*com_work.v[11];
  com_work.d[12] = com_work.v[12];
  if (com_work.d[12] < 0)
    com_work.d[12] = com_settings.kkt_reg;
  else
    com_work.d[12] += com_settings.kkt_reg;
  com_work.d_inv[12] = 1/com_work.d[12];
  com_work.L[18] = (com_work.KKT[31]-com_work.L[17]*com_work.v[11])*com_work.d_inv[12];
  com_work.v[5] = com_work.L[16]*com_work.d[5];
  com_work.v[11] = com_work.L[17]*com_work.d[11];
  com_work.v[12] = com_work.L[18]*com_work.d[12];
  com_work.v[13] = com_work.KKT[32]-com_work.L[16]*com_work.v[5]-com_work.L[17]*com_work.v[11]-com_work.L[18]*com_work.v[12];
  com_work.d[13] = com_work.v[13];
  if (com_work.d[13] > 0)
    com_work.d[13] = -com_settings.kkt_reg;
  else
    com_work.d[13] -= com_settings.kkt_reg;
  com_work.d_inv[13] = 1/com_work.d[13];
#ifndef ZERO_LIBRARY_MODE
  if (com_settings.debug) {
    printf("Squared Frobenius for factorization is %.8g.\n", com_check_factorization());
  }
#endif
}
double com_check_factorization(void) {
  /* Returns the squared Frobenius norm of A - L*D*L'. */
  double temp, residual;
  /* Only check the lower triangle. */
  residual = 0;
  temp = com_work.KKT[27]-1*com_work.d[11]*1-com_work.L[5]*com_work.d[6]*com_work.L[5]-com_work.L[6]*com_work.d[7]*com_work.L[6]-com_work.L[7]*com_work.d[8]*com_work.L[7]-com_work.L[8]*com_work.d[9]*com_work.L[8]-com_work.L[9]*com_work.d[10]*com_work.L[9];
  residual += temp*temp;
  temp = com_work.KKT[28]-com_work.L[15]*com_work.d[11]*1-com_work.L[10]*com_work.d[6]*com_work.L[5]-com_work.L[11]*com_work.d[7]*com_work.L[6]-com_work.L[12]*com_work.d[8]*com_work.L[7]-com_work.L[13]*com_work.d[9]*com_work.L[8]-com_work.L[14]*com_work.d[10]*com_work.L[9];
  residual += temp*temp;
  temp = com_work.KKT[30]-com_work.L[15]*com_work.d[11]*com_work.L[15]-1*com_work.d[12]*1-com_work.L[10]*com_work.d[6]*com_work.L[10]-com_work.L[11]*com_work.d[7]*com_work.L[11]-com_work.L[12]*com_work.d[8]*com_work.L[12]-com_work.L[13]*com_work.d[9]*com_work.L[13]-com_work.L[14]*com_work.d[10]*com_work.L[14];
  residual += temp*temp;
  temp = com_work.KKT[0]-1*com_work.d[0]*1;
  residual += temp*temp;
  temp = com_work.KKT[2]-1*com_work.d[1]*1;
  residual += temp*temp;
  temp = com_work.KKT[4]-1*com_work.d[2]*1;
  residual += temp*temp;
  temp = com_work.KKT[6]-1*com_work.d[3]*1;
  residual += temp*temp;
  temp = com_work.KKT[8]-1*com_work.d[4]*1;
  residual += temp*temp;
  temp = com_work.KKT[10]-1*com_work.d[5]*1;
  residual += temp*temp;
  temp = com_work.KKT[1]-com_work.L[0]*com_work.d[0]*1;
  residual += temp*temp;
  temp = com_work.KKT[3]-com_work.L[1]*com_work.d[1]*1;
  residual += temp*temp;
  temp = com_work.KKT[5]-com_work.L[2]*com_work.d[2]*1;
  residual += temp*temp;
  temp = com_work.KKT[7]-com_work.L[3]*com_work.d[3]*1;
  residual += temp*temp;
  temp = com_work.KKT[9]-com_work.L[4]*com_work.d[4]*1;
  residual += temp*temp;
  temp = com_work.KKT[11]-com_work.L[16]*com_work.d[5]*1;
  residual += temp*temp;
  temp = com_work.KKT[12]-com_work.L[0]*com_work.d[0]*com_work.L[0]-1*com_work.d[6]*1;
  residual += temp*temp;
  temp = com_work.KKT[15]-com_work.L[1]*com_work.d[1]*com_work.L[1]-1*com_work.d[7]*1;
  residual += temp*temp;
  temp = com_work.KKT[18]-com_work.L[2]*com_work.d[2]*com_work.L[2]-1*com_work.d[8]*1;
  residual += temp*temp;
  temp = com_work.KKT[21]-com_work.L[3]*com_work.d[3]*com_work.L[3]-1*com_work.d[9]*1;
  residual += temp*temp;
  temp = com_work.KKT[24]-com_work.L[4]*com_work.d[4]*com_work.L[4]-1*com_work.d[10]*1;
  residual += temp*temp;
  temp = com_work.KKT[32]-com_work.L[16]*com_work.d[5]*com_work.L[16]-1*com_work.d[13]*1-com_work.L[17]*com_work.d[11]*com_work.L[17]-com_work.L[18]*com_work.d[12]*com_work.L[18];
  residual += temp*temp;
  temp = com_work.KKT[13]-1*com_work.d[6]*com_work.L[5];
  residual += temp*temp;
  temp = com_work.KKT[14]-1*com_work.d[6]*com_work.L[10];
  residual += temp*temp;
  temp = com_work.KKT[16]-1*com_work.d[7]*com_work.L[6];
  residual += temp*temp;
  temp = com_work.KKT[17]-1*com_work.d[7]*com_work.L[11];
  residual += temp*temp;
  temp = com_work.KKT[19]-1*com_work.d[8]*com_work.L[7];
  residual += temp*temp;
  temp = com_work.KKT[20]-1*com_work.d[8]*com_work.L[12];
  residual += temp*temp;
  temp = com_work.KKT[22]-1*com_work.d[9]*com_work.L[8];
  residual += temp*temp;
  temp = com_work.KKT[23]-1*com_work.d[9]*com_work.L[13];
  residual += temp*temp;
  temp = com_work.KKT[25]-1*com_work.d[10]*com_work.L[9];
  residual += temp*temp;
  temp = com_work.KKT[26]-1*com_work.d[10]*com_work.L[14];
  residual += temp*temp;
  temp = com_work.KKT[29]-com_work.L[17]*com_work.d[11]*1;
  residual += temp*temp;
  temp = com_work.KKT[31]-com_work.L[17]*com_work.d[11]*com_work.L[15]-com_work.L[18]*com_work.d[12]*1;
  residual += temp*temp;
  return residual;
}
void com_matrix_multiply(double *result, double *source) {
  /* Finds result = A*source. */
  result[0] = com_work.KKT[27]*source[0]+com_work.KKT[28]*source[1]+com_work.KKT[13]*source[8]+com_work.KKT[16]*source[9]+com_work.KKT[19]*source[10]+com_work.KKT[22]*source[11]+com_work.KKT[25]*source[12]+com_work.KKT[29]*source[13];
  result[1] = com_work.KKT[28]*source[0]+com_work.KKT[30]*source[1]+com_work.KKT[14]*source[8]+com_work.KKT[17]*source[9]+com_work.KKT[20]*source[10]+com_work.KKT[23]*source[11]+com_work.KKT[26]*source[12]+com_work.KKT[31]*source[13];
  result[2] = com_work.KKT[0]*source[2]+com_work.KKT[1]*source[8];
  result[3] = com_work.KKT[2]*source[3]+com_work.KKT[3]*source[9];
  result[4] = com_work.KKT[4]*source[4]+com_work.KKT[5]*source[10];
  result[5] = com_work.KKT[6]*source[5]+com_work.KKT[7]*source[11];
  result[6] = com_work.KKT[8]*source[6]+com_work.KKT[9]*source[12];
  result[7] = com_work.KKT[10]*source[7]+com_work.KKT[11]*source[13];
  result[8] = com_work.KKT[1]*source[2]+com_work.KKT[12]*source[8]+com_work.KKT[13]*source[0]+com_work.KKT[14]*source[1];
  result[9] = com_work.KKT[3]*source[3]+com_work.KKT[15]*source[9]+com_work.KKT[16]*source[0]+com_work.KKT[17]*source[1];
  result[10] = com_work.KKT[5]*source[4]+com_work.KKT[18]*source[10]+com_work.KKT[19]*source[0]+com_work.KKT[20]*source[1];
  result[11] = com_work.KKT[7]*source[5]+com_work.KKT[21]*source[11]+com_work.KKT[22]*source[0]+com_work.KKT[23]*source[1];
  result[12] = com_work.KKT[9]*source[6]+com_work.KKT[24]*source[12]+com_work.KKT[25]*source[0]+com_work.KKT[26]*source[1];
  result[13] = com_work.KKT[11]*source[7]+com_work.KKT[32]*source[13]+com_work.KKT[29]*source[0]+com_work.KKT[31]*source[1];
}
double com_check_residual(double *target, double *multiplicand) {
  /* Returns the squared 2-norm of lhs - A*rhs. */
  /* Reuses v to find the residual. */
  int i;
  double residual;
  residual = 0;
  com_matrix_multiply(com_work.v, multiplicand);
  for (i = 0; i < 2; i++) {
    residual += (target[i] - com_work.v[i])*(target[i] - com_work.v[i]);
  }
  return residual;
}
void com_fill_KKT(void) {
  com_work.KKT[27] = 2*com_params.Q[0];
  com_work.KKT[28] = 2*com_params.Q[2];
  com_work.KKT[30] = 2*com_params.Q[3];
  com_work.KKT[0] = com_work.s_inv_z[0];
  com_work.KKT[2] = com_work.s_inv_z[1];
  com_work.KKT[4] = com_work.s_inv_z[2];
  com_work.KKT[6] = com_work.s_inv_z[3];
  com_work.KKT[8] = com_work.s_inv_z[4];
  com_work.KKT[10] = com_work.s_inv_z[5];
  com_work.KKT[1] = 1;
  com_work.KKT[3] = 1;
  com_work.KKT[5] = 1;
  com_work.KKT[7] = 1;
  com_work.KKT[9] = 1;
  com_work.KKT[11] = 1;
  com_work.KKT[12] = com_work.block_33[0];
  com_work.KKT[15] = com_work.block_33[0];
  com_work.KKT[18] = com_work.block_33[0];
  com_work.KKT[21] = com_work.block_33[0];
  com_work.KKT[24] = com_work.block_33[0];
  com_work.KKT[32] = com_work.block_33[0];
  com_work.KKT[13] = com_params.NeJc[0];
  com_work.KKT[14] = com_params.NeJc[6];
  com_work.KKT[16] = com_params.NeJc[1];
  com_work.KKT[17] = com_params.NeJc[7];
  com_work.KKT[19] = com_params.NeJc[2];
  com_work.KKT[20] = com_params.NeJc[8];
  com_work.KKT[22] = com_params.NeJc[3];
  com_work.KKT[23] = com_params.NeJc[9];
  com_work.KKT[25] = com_params.NeJc[4];
  com_work.KKT[26] = com_params.NeJc[10];
  com_work.KKT[29] = com_params.NeJc[5];
  com_work.KKT[31] = com_params.NeJc[11];
}
