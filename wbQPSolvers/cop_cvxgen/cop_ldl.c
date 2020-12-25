/* Produced by CVXGEN, 2020-12-10 23:07:19 -0500.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: ldl.c. */
/* Description: Basic test harness for solver.c. */
#include "cop_solver.h"
/* Be sure to place ldl_solve first, so storage schemes are defined by it. */
void cop_ldl_cop_solve(double *target, double *var) {
  int i;
  /* Find var = (L*diag(cop_work.d)*L') \ target, then unpermute. */
  /* Answer goes into var. */
  /* Forward substitution. */
  /* Include permutation as we retrieve from target. Use v so we can unpermute */
  /* later. */
  cop_work.v[0] = target[2];
  cop_work.v[1] = target[3];
  cop_work.v[2] = target[4];
  cop_work.v[3] = target[5];
  cop_work.v[4] = target[6];
  cop_work.v[5] = target[7];
  cop_work.v[6] = target[8]-cop_work.L[0]*cop_work.v[0];
  cop_work.v[7] = target[9]-cop_work.L[1]*cop_work.v[1];
  cop_work.v[8] = target[10]-cop_work.L[2]*cop_work.v[2];
  cop_work.v[9] = target[11]-cop_work.L[3]*cop_work.v[3];
  cop_work.v[10] = target[12]-cop_work.L[4]*cop_work.v[4];
  cop_work.v[11] = target[0]-cop_work.L[5]*cop_work.v[6]-cop_work.L[6]*cop_work.v[7]-cop_work.L[7]*cop_work.v[8]-cop_work.L[8]*cop_work.v[9]-cop_work.L[9]*cop_work.v[10];
  cop_work.v[12] = target[1]-cop_work.L[10]*cop_work.v[6]-cop_work.L[11]*cop_work.v[7]-cop_work.L[12]*cop_work.v[8]-cop_work.L[13]*cop_work.v[9]-cop_work.L[14]*cop_work.v[10]-cop_work.L[15]*cop_work.v[11];
  cop_work.v[13] = target[13]-cop_work.L[16]*cop_work.v[5]-cop_work.L[17]*cop_work.v[11]-cop_work.L[18]*cop_work.v[12];
  /* Diagonal scaling. Assume correctness of cop_work.d_inv. */
  for (i = 0; i < 14; i++)
    cop_work.v[i] *= cop_work.d_inv[i];
  /* Back substitution */
  cop_work.v[12] -= cop_work.L[18]*cop_work.v[13];
  cop_work.v[11] -= cop_work.L[15]*cop_work.v[12]+cop_work.L[17]*cop_work.v[13];
  cop_work.v[10] -= cop_work.L[9]*cop_work.v[11]+cop_work.L[14]*cop_work.v[12];
  cop_work.v[9] -= cop_work.L[8]*cop_work.v[11]+cop_work.L[13]*cop_work.v[12];
  cop_work.v[8] -= cop_work.L[7]*cop_work.v[11]+cop_work.L[12]*cop_work.v[12];
  cop_work.v[7] -= cop_work.L[6]*cop_work.v[11]+cop_work.L[11]*cop_work.v[12];
  cop_work.v[6] -= cop_work.L[5]*cop_work.v[11]+cop_work.L[10]*cop_work.v[12];
  cop_work.v[5] -= cop_work.L[16]*cop_work.v[13];
  cop_work.v[4] -= cop_work.L[4]*cop_work.v[10];
  cop_work.v[3] -= cop_work.L[3]*cop_work.v[9];
  cop_work.v[2] -= cop_work.L[2]*cop_work.v[8];
  cop_work.v[1] -= cop_work.L[1]*cop_work.v[7];
  cop_work.v[0] -= cop_work.L[0]*cop_work.v[6];
  /* Unpermute the result, from v to var. */
  var[0] = cop_work.v[11];
  var[1] = cop_work.v[12];
  var[2] = cop_work.v[0];
  var[3] = cop_work.v[1];
  var[4] = cop_work.v[2];
  var[5] = cop_work.v[3];
  var[6] = cop_work.v[4];
  var[7] = cop_work.v[5];
  var[8] = cop_work.v[6];
  var[9] = cop_work.v[7];
  var[10] = cop_work.v[8];
  var[11] = cop_work.v[9];
  var[12] = cop_work.v[10];
  var[13] = cop_work.v[13];
#ifndef ZERO_LIBRARY_MODE
  if (cop_settings.debug) {
    printf("Squared norm for solution is %.8g.\n", cop_check_residual(target, var));
  }
#endif
}
void cop_ldl_factor(void) {
  cop_work.d[0] = cop_work.KKT[0];
  if (cop_work.d[0] < 0)
    cop_work.d[0] = cop_settings.kkt_reg;
  else
    cop_work.d[0] += cop_settings.kkt_reg;
  cop_work.d_inv[0] = 1/cop_work.d[0];
  cop_work.L[0] = cop_work.KKT[1]*cop_work.d_inv[0];
  cop_work.v[1] = cop_work.KKT[2];
  cop_work.d[1] = cop_work.v[1];
  if (cop_work.d[1] < 0)
    cop_work.d[1] = cop_settings.kkt_reg;
  else
    cop_work.d[1] += cop_settings.kkt_reg;
  cop_work.d_inv[1] = 1/cop_work.d[1];
  cop_work.L[1] = (cop_work.KKT[3])*cop_work.d_inv[1];
  cop_work.v[2] = cop_work.KKT[4];
  cop_work.d[2] = cop_work.v[2];
  if (cop_work.d[2] < 0)
    cop_work.d[2] = cop_settings.kkt_reg;
  else
    cop_work.d[2] += cop_settings.kkt_reg;
  cop_work.d_inv[2] = 1/cop_work.d[2];
  cop_work.L[2] = (cop_work.KKT[5])*cop_work.d_inv[2];
  cop_work.v[3] = cop_work.KKT[6];
  cop_work.d[3] = cop_work.v[3];
  if (cop_work.d[3] < 0)
    cop_work.d[3] = cop_settings.kkt_reg;
  else
    cop_work.d[3] += cop_settings.kkt_reg;
  cop_work.d_inv[3] = 1/cop_work.d[3];
  cop_work.L[3] = (cop_work.KKT[7])*cop_work.d_inv[3];
  cop_work.v[4] = cop_work.KKT[8];
  cop_work.d[4] = cop_work.v[4];
  if (cop_work.d[4] < 0)
    cop_work.d[4] = cop_settings.kkt_reg;
  else
    cop_work.d[4] += cop_settings.kkt_reg;
  cop_work.d_inv[4] = 1/cop_work.d[4];
  cop_work.L[4] = (cop_work.KKT[9])*cop_work.d_inv[4];
  cop_work.v[5] = cop_work.KKT[10];
  cop_work.d[5] = cop_work.v[5];
  if (cop_work.d[5] < 0)
    cop_work.d[5] = cop_settings.kkt_reg;
  else
    cop_work.d[5] += cop_settings.kkt_reg;
  cop_work.d_inv[5] = 1/cop_work.d[5];
  cop_work.L[16] = (cop_work.KKT[11])*cop_work.d_inv[5];
  cop_work.v[0] = cop_work.L[0]*cop_work.d[0];
  cop_work.v[6] = cop_work.KKT[12]-cop_work.L[0]*cop_work.v[0];
  cop_work.d[6] = cop_work.v[6];
  if (cop_work.d[6] > 0)
    cop_work.d[6] = -cop_settings.kkt_reg;
  else
    cop_work.d[6] -= cop_settings.kkt_reg;
  cop_work.d_inv[6] = 1/cop_work.d[6];
  cop_work.L[5] = (cop_work.KKT[13])*cop_work.d_inv[6];
  cop_work.L[10] = (cop_work.KKT[14])*cop_work.d_inv[6];
  cop_work.v[1] = cop_work.L[1]*cop_work.d[1];
  cop_work.v[7] = cop_work.KKT[15]-cop_work.L[1]*cop_work.v[1];
  cop_work.d[7] = cop_work.v[7];
  if (cop_work.d[7] > 0)
    cop_work.d[7] = -cop_settings.kkt_reg;
  else
    cop_work.d[7] -= cop_settings.kkt_reg;
  cop_work.d_inv[7] = 1/cop_work.d[7];
  cop_work.L[6] = (cop_work.KKT[16])*cop_work.d_inv[7];
  cop_work.L[11] = (cop_work.KKT[17])*cop_work.d_inv[7];
  cop_work.v[2] = cop_work.L[2]*cop_work.d[2];
  cop_work.v[8] = cop_work.KKT[18]-cop_work.L[2]*cop_work.v[2];
  cop_work.d[8] = cop_work.v[8];
  if (cop_work.d[8] > 0)
    cop_work.d[8] = -cop_settings.kkt_reg;
  else
    cop_work.d[8] -= cop_settings.kkt_reg;
  cop_work.d_inv[8] = 1/cop_work.d[8];
  cop_work.L[7] = (cop_work.KKT[19])*cop_work.d_inv[8];
  cop_work.L[12] = (cop_work.KKT[20])*cop_work.d_inv[8];
  cop_work.v[3] = cop_work.L[3]*cop_work.d[3];
  cop_work.v[9] = cop_work.KKT[21]-cop_work.L[3]*cop_work.v[3];
  cop_work.d[9] = cop_work.v[9];
  if (cop_work.d[9] > 0)
    cop_work.d[9] = -cop_settings.kkt_reg;
  else
    cop_work.d[9] -= cop_settings.kkt_reg;
  cop_work.d_inv[9] = 1/cop_work.d[9];
  cop_work.L[8] = (cop_work.KKT[22])*cop_work.d_inv[9];
  cop_work.L[13] = (cop_work.KKT[23])*cop_work.d_inv[9];
  cop_work.v[4] = cop_work.L[4]*cop_work.d[4];
  cop_work.v[10] = cop_work.KKT[24]-cop_work.L[4]*cop_work.v[4];
  cop_work.d[10] = cop_work.v[10];
  if (cop_work.d[10] > 0)
    cop_work.d[10] = -cop_settings.kkt_reg;
  else
    cop_work.d[10] -= cop_settings.kkt_reg;
  cop_work.d_inv[10] = 1/cop_work.d[10];
  cop_work.L[9] = (cop_work.KKT[25])*cop_work.d_inv[10];
  cop_work.L[14] = (cop_work.KKT[26])*cop_work.d_inv[10];
  cop_work.v[6] = cop_work.L[5]*cop_work.d[6];
  cop_work.v[7] = cop_work.L[6]*cop_work.d[7];
  cop_work.v[8] = cop_work.L[7]*cop_work.d[8];
  cop_work.v[9] = cop_work.L[8]*cop_work.d[9];
  cop_work.v[10] = cop_work.L[9]*cop_work.d[10];
  cop_work.v[11] = cop_work.KKT[27]-cop_work.L[5]*cop_work.v[6]-cop_work.L[6]*cop_work.v[7]-cop_work.L[7]*cop_work.v[8]-cop_work.L[8]*cop_work.v[9]-cop_work.L[9]*cop_work.v[10];
  cop_work.d[11] = cop_work.v[11];
  if (cop_work.d[11] < 0)
    cop_work.d[11] = cop_settings.kkt_reg;
  else
    cop_work.d[11] += cop_settings.kkt_reg;
  cop_work.d_inv[11] = 1/cop_work.d[11];
  cop_work.L[15] = (cop_work.KKT[28]-cop_work.L[10]*cop_work.v[6]-cop_work.L[11]*cop_work.v[7]-cop_work.L[12]*cop_work.v[8]-cop_work.L[13]*cop_work.v[9]-cop_work.L[14]*cop_work.v[10])*cop_work.d_inv[11];
  cop_work.L[17] = (cop_work.KKT[29])*cop_work.d_inv[11];
  cop_work.v[6] = cop_work.L[10]*cop_work.d[6];
  cop_work.v[7] = cop_work.L[11]*cop_work.d[7];
  cop_work.v[8] = cop_work.L[12]*cop_work.d[8];
  cop_work.v[9] = cop_work.L[13]*cop_work.d[9];
  cop_work.v[10] = cop_work.L[14]*cop_work.d[10];
  cop_work.v[11] = cop_work.L[15]*cop_work.d[11];
  cop_work.v[12] = cop_work.KKT[30]-cop_work.L[10]*cop_work.v[6]-cop_work.L[11]*cop_work.v[7]-cop_work.L[12]*cop_work.v[8]-cop_work.L[13]*cop_work.v[9]-cop_work.L[14]*cop_work.v[10]-cop_work.L[15]*cop_work.v[11];
  cop_work.d[12] = cop_work.v[12];
  if (cop_work.d[12] < 0)
    cop_work.d[12] = cop_settings.kkt_reg;
  else
    cop_work.d[12] += cop_settings.kkt_reg;
  cop_work.d_inv[12] = 1/cop_work.d[12];
  cop_work.L[18] = (cop_work.KKT[31]-cop_work.L[17]*cop_work.v[11])*cop_work.d_inv[12];
  cop_work.v[5] = cop_work.L[16]*cop_work.d[5];
  cop_work.v[11] = cop_work.L[17]*cop_work.d[11];
  cop_work.v[12] = cop_work.L[18]*cop_work.d[12];
  cop_work.v[13] = cop_work.KKT[32]-cop_work.L[16]*cop_work.v[5]-cop_work.L[17]*cop_work.v[11]-cop_work.L[18]*cop_work.v[12];
  cop_work.d[13] = cop_work.v[13];
  if (cop_work.d[13] > 0)
    cop_work.d[13] = -cop_settings.kkt_reg;
  else
    cop_work.d[13] -= cop_settings.kkt_reg;
  cop_work.d_inv[13] = 1/cop_work.d[13];
#ifndef ZERO_LIBRARY_MODE
  if (cop_settings.debug) {
    printf("Squared Frobenius for factorization is %.8g.\n", cop_check_factorization());
  }
#endif
}
double cop_check_factorization(void) {
  /* Returns the squared Frobenius norm of A - L*D*L'. */
  double temp, residual;
  /* Only check the lower triangle. */
  residual = 0;
  temp = cop_work.KKT[27]-1*cop_work.d[11]*1-cop_work.L[5]*cop_work.d[6]*cop_work.L[5]-cop_work.L[6]*cop_work.d[7]*cop_work.L[6]-cop_work.L[7]*cop_work.d[8]*cop_work.L[7]-cop_work.L[8]*cop_work.d[9]*cop_work.L[8]-cop_work.L[9]*cop_work.d[10]*cop_work.L[9];
  residual += temp*temp;
  temp = cop_work.KKT[28]-cop_work.L[15]*cop_work.d[11]*1-cop_work.L[10]*cop_work.d[6]*cop_work.L[5]-cop_work.L[11]*cop_work.d[7]*cop_work.L[6]-cop_work.L[12]*cop_work.d[8]*cop_work.L[7]-cop_work.L[13]*cop_work.d[9]*cop_work.L[8]-cop_work.L[14]*cop_work.d[10]*cop_work.L[9];
  residual += temp*temp;
  temp = cop_work.KKT[30]-cop_work.L[15]*cop_work.d[11]*cop_work.L[15]-1*cop_work.d[12]*1-cop_work.L[10]*cop_work.d[6]*cop_work.L[10]-cop_work.L[11]*cop_work.d[7]*cop_work.L[11]-cop_work.L[12]*cop_work.d[8]*cop_work.L[12]-cop_work.L[13]*cop_work.d[9]*cop_work.L[13]-cop_work.L[14]*cop_work.d[10]*cop_work.L[14];
  residual += temp*temp;
  temp = cop_work.KKT[0]-1*cop_work.d[0]*1;
  residual += temp*temp;
  temp = cop_work.KKT[2]-1*cop_work.d[1]*1;
  residual += temp*temp;
  temp = cop_work.KKT[4]-1*cop_work.d[2]*1;
  residual += temp*temp;
  temp = cop_work.KKT[6]-1*cop_work.d[3]*1;
  residual += temp*temp;
  temp = cop_work.KKT[8]-1*cop_work.d[4]*1;
  residual += temp*temp;
  temp = cop_work.KKT[10]-1*cop_work.d[5]*1;
  residual += temp*temp;
  temp = cop_work.KKT[1]-cop_work.L[0]*cop_work.d[0]*1;
  residual += temp*temp;
  temp = cop_work.KKT[3]-cop_work.L[1]*cop_work.d[1]*1;
  residual += temp*temp;
  temp = cop_work.KKT[5]-cop_work.L[2]*cop_work.d[2]*1;
  residual += temp*temp;
  temp = cop_work.KKT[7]-cop_work.L[3]*cop_work.d[3]*1;
  residual += temp*temp;
  temp = cop_work.KKT[9]-cop_work.L[4]*cop_work.d[4]*1;
  residual += temp*temp;
  temp = cop_work.KKT[11]-cop_work.L[16]*cop_work.d[5]*1;
  residual += temp*temp;
  temp = cop_work.KKT[12]-cop_work.L[0]*cop_work.d[0]*cop_work.L[0]-1*cop_work.d[6]*1;
  residual += temp*temp;
  temp = cop_work.KKT[15]-cop_work.L[1]*cop_work.d[1]*cop_work.L[1]-1*cop_work.d[7]*1;
  residual += temp*temp;
  temp = cop_work.KKT[18]-cop_work.L[2]*cop_work.d[2]*cop_work.L[2]-1*cop_work.d[8]*1;
  residual += temp*temp;
  temp = cop_work.KKT[21]-cop_work.L[3]*cop_work.d[3]*cop_work.L[3]-1*cop_work.d[9]*1;
  residual += temp*temp;
  temp = cop_work.KKT[24]-cop_work.L[4]*cop_work.d[4]*cop_work.L[4]-1*cop_work.d[10]*1;
  residual += temp*temp;
  temp = cop_work.KKT[32]-cop_work.L[16]*cop_work.d[5]*cop_work.L[16]-1*cop_work.d[13]*1-cop_work.L[17]*cop_work.d[11]*cop_work.L[17]-cop_work.L[18]*cop_work.d[12]*cop_work.L[18];
  residual += temp*temp;
  temp = cop_work.KKT[13]-1*cop_work.d[6]*cop_work.L[5];
  residual += temp*temp;
  temp = cop_work.KKT[14]-1*cop_work.d[6]*cop_work.L[10];
  residual += temp*temp;
  temp = cop_work.KKT[16]-1*cop_work.d[7]*cop_work.L[6];
  residual += temp*temp;
  temp = cop_work.KKT[17]-1*cop_work.d[7]*cop_work.L[11];
  residual += temp*temp;
  temp = cop_work.KKT[19]-1*cop_work.d[8]*cop_work.L[7];
  residual += temp*temp;
  temp = cop_work.KKT[20]-1*cop_work.d[8]*cop_work.L[12];
  residual += temp*temp;
  temp = cop_work.KKT[22]-1*cop_work.d[9]*cop_work.L[8];
  residual += temp*temp;
  temp = cop_work.KKT[23]-1*cop_work.d[9]*cop_work.L[13];
  residual += temp*temp;
  temp = cop_work.KKT[25]-1*cop_work.d[10]*cop_work.L[9];
  residual += temp*temp;
  temp = cop_work.KKT[26]-1*cop_work.d[10]*cop_work.L[14];
  residual += temp*temp;
  temp = cop_work.KKT[29]-cop_work.L[17]*cop_work.d[11]*1;
  residual += temp*temp;
  temp = cop_work.KKT[31]-cop_work.L[17]*cop_work.d[11]*cop_work.L[15]-cop_work.L[18]*cop_work.d[12]*1;
  residual += temp*temp;
  return residual;
}
void cop_matrix_multiply(double *result, double *source) {
  /* Finds result = A*source. */
  result[0] = cop_work.KKT[27]*source[0]+cop_work.KKT[28]*source[1]+cop_work.KKT[13]*source[8]+cop_work.KKT[16]*source[9]+cop_work.KKT[19]*source[10]+cop_work.KKT[22]*source[11]+cop_work.KKT[25]*source[12]+cop_work.KKT[29]*source[13];
  result[1] = cop_work.KKT[28]*source[0]+cop_work.KKT[30]*source[1]+cop_work.KKT[14]*source[8]+cop_work.KKT[17]*source[9]+cop_work.KKT[20]*source[10]+cop_work.KKT[23]*source[11]+cop_work.KKT[26]*source[12]+cop_work.KKT[31]*source[13];
  result[2] = cop_work.KKT[0]*source[2]+cop_work.KKT[1]*source[8];
  result[3] = cop_work.KKT[2]*source[3]+cop_work.KKT[3]*source[9];
  result[4] = cop_work.KKT[4]*source[4]+cop_work.KKT[5]*source[10];
  result[5] = cop_work.KKT[6]*source[5]+cop_work.KKT[7]*source[11];
  result[6] = cop_work.KKT[8]*source[6]+cop_work.KKT[9]*source[12];
  result[7] = cop_work.KKT[10]*source[7]+cop_work.KKT[11]*source[13];
  result[8] = cop_work.KKT[1]*source[2]+cop_work.KKT[12]*source[8]+cop_work.KKT[13]*source[0]+cop_work.KKT[14]*source[1];
  result[9] = cop_work.KKT[3]*source[3]+cop_work.KKT[15]*source[9]+cop_work.KKT[16]*source[0]+cop_work.KKT[17]*source[1];
  result[10] = cop_work.KKT[5]*source[4]+cop_work.KKT[18]*source[10]+cop_work.KKT[19]*source[0]+cop_work.KKT[20]*source[1];
  result[11] = cop_work.KKT[7]*source[5]+cop_work.KKT[21]*source[11]+cop_work.KKT[22]*source[0]+cop_work.KKT[23]*source[1];
  result[12] = cop_work.KKT[9]*source[6]+cop_work.KKT[24]*source[12]+cop_work.KKT[25]*source[0]+cop_work.KKT[26]*source[1];
  result[13] = cop_work.KKT[11]*source[7]+cop_work.KKT[32]*source[13]+cop_work.KKT[29]*source[0]+cop_work.KKT[31]*source[1];
}
double cop_check_residual(double *target, double *multiplicand) {
  /* Returns the squared 2-norm of lhs - A*rhs. */
  /* Reuses v to find the residual. */
  int i;
  double residual;
  residual = 0;
  cop_matrix_multiply(cop_work.v, multiplicand);
  for (i = 0; i < 2; i++) {
    residual += (target[i] - cop_work.v[i])*(target[i] - cop_work.v[i]);
  }
  return residual;
}
void cop_fill_KKT(void) {
  cop_work.KKT[27] = 2*cop_params.Q[0];
  cop_work.KKT[28] = 2*cop_params.Q[2];
  cop_work.KKT[30] = 2*cop_params.Q[3];
  cop_work.KKT[0] = cop_work.s_inv_z[0];
  cop_work.KKT[2] = cop_work.s_inv_z[1];
  cop_work.KKT[4] = cop_work.s_inv_z[2];
  cop_work.KKT[6] = cop_work.s_inv_z[3];
  cop_work.KKT[8] = cop_work.s_inv_z[4];
  cop_work.KKT[10] = cop_work.s_inv_z[5];
  cop_work.KKT[1] = 1;
  cop_work.KKT[3] = 1;
  cop_work.KKT[5] = 1;
  cop_work.KKT[7] = 1;
  cop_work.KKT[9] = 1;
  cop_work.KKT[11] = 1;
  cop_work.KKT[12] = cop_work.block_33[0];
  cop_work.KKT[15] = cop_work.block_33[0];
  cop_work.KKT[18] = cop_work.block_33[0];
  cop_work.KKT[21] = cop_work.block_33[0];
  cop_work.KKT[24] = cop_work.block_33[0];
  cop_work.KKT[32] = cop_work.block_33[0];
  cop_work.KKT[13] = cop_params.NeJc[0];
  cop_work.KKT[14] = cop_params.NeJc[6];
  cop_work.KKT[16] = cop_params.NeJc[1];
  cop_work.KKT[17] = cop_params.NeJc[7];
  cop_work.KKT[19] = cop_params.NeJc[2];
  cop_work.KKT[20] = cop_params.NeJc[8];
  cop_work.KKT[22] = cop_params.NeJc[3];
  cop_work.KKT[23] = cop_params.NeJc[9];
  cop_work.KKT[25] = cop_params.NeJc[4];
  cop_work.KKT[26] = cop_params.NeJc[10];
  cop_work.KKT[29] = cop_params.NeJc[5];
  cop_work.KKT[31] = cop_params.NeJc[11];
}
