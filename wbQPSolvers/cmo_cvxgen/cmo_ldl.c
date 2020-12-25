/* Produced by CVXGEN, 2019-04-21 11:08:24 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: ldl.c. */
/* Description: Basic test harness for solver.c. */
#include "cmo_solver.h"
/* Be sure to place ldl_solve first, so storage schemes are defined by it. */
void cmo_ldl_cmo_solve(double *target, double *var) {
  int i;
  /* Find var = (L*diag(cmo_work.d)*L') \ target, then unpermute. */
  /* Answer goes into var. */
  /* Forward substitution. */
  /* Include permutation as we retrieve from target. Use v so we can unpermute */
  /* later. */
  cmo_work.v[0] = target[26];
  cmo_work.v[1] = target[27];
  cmo_work.v[2] = target[28];
  cmo_work.v[3] = target[29];
  cmo_work.v[4] = target[30];
  cmo_work.v[5] = target[31];
  cmo_work.v[6] = target[32];
  cmo_work.v[7] = target[33];
  cmo_work.v[8] = target[34];
  cmo_work.v[9] = target[35];
  cmo_work.v[10] = target[36];
  cmo_work.v[11] = target[37];
  cmo_work.v[12] = target[38];
  cmo_work.v[13] = target[39];
  cmo_work.v[14] = target[40];
  cmo_work.v[15] = target[41];
  cmo_work.v[16] = target[42];
  cmo_work.v[17] = target[43];
  cmo_work.v[18] = target[44];
  cmo_work.v[19] = target[45];
  cmo_work.v[20] = target[46];
  cmo_work.v[21] = target[47];
  cmo_work.v[22] = target[48];
  cmo_work.v[23] = target[49];
  cmo_work.v[24] = target[0];
  cmo_work.v[25] = target[1];
  cmo_work.v[26] = target[2];
  cmo_work.v[27] = target[3];
  cmo_work.v[28] = target[4];
  cmo_work.v[29] = target[5];
  cmo_work.v[30] = target[6];
  cmo_work.v[31] = target[7];
  cmo_work.v[32] = target[8];
  cmo_work.v[33] = target[9];
  cmo_work.v[34] = target[10];
  cmo_work.v[35] = target[11];
  cmo_work.v[36] = target[12];
  cmo_work.v[37] = target[13];
  cmo_work.v[38] = target[50]-cmo_work.L[0]*cmo_work.v[0]-cmo_work.L[1]*cmo_work.v[26];
  cmo_work.v[39] = target[62]-cmo_work.L[2]*cmo_work.v[12]-cmo_work.L[3]*cmo_work.v[26]-cmo_work.L[4]*cmo_work.v[38];
  cmo_work.v[40] = target[14]-cmo_work.L[5]*cmo_work.v[38]-cmo_work.L[6]*cmo_work.v[39];
  cmo_work.v[41] = target[51]-cmo_work.L[7]*cmo_work.v[1]-cmo_work.L[8]*cmo_work.v[27];
  cmo_work.v[42] = target[63]-cmo_work.L[9]*cmo_work.v[13]-cmo_work.L[10]*cmo_work.v[27]-cmo_work.L[11]*cmo_work.v[41];
  cmo_work.v[43] = target[15]-cmo_work.L[12]*cmo_work.v[41]-cmo_work.L[13]*cmo_work.v[42];
  cmo_work.v[44] = target[52]-cmo_work.L[14]*cmo_work.v[2]-cmo_work.L[15]*cmo_work.v[28];
  cmo_work.v[45] = target[64]-cmo_work.L[16]*cmo_work.v[14]-cmo_work.L[17]*cmo_work.v[28]-cmo_work.L[18]*cmo_work.v[44];
  cmo_work.v[46] = target[16]-cmo_work.L[19]*cmo_work.v[44]-cmo_work.L[20]*cmo_work.v[45];
  cmo_work.v[47] = target[53]-cmo_work.L[21]*cmo_work.v[3]-cmo_work.L[22]*cmo_work.v[29];
  cmo_work.v[48] = target[65]-cmo_work.L[23]*cmo_work.v[15]-cmo_work.L[24]*cmo_work.v[29]-cmo_work.L[25]*cmo_work.v[47];
  cmo_work.v[49] = target[17]-cmo_work.L[26]*cmo_work.v[47]-cmo_work.L[27]*cmo_work.v[48];
  cmo_work.v[50] = target[54]-cmo_work.L[28]*cmo_work.v[4]-cmo_work.L[29]*cmo_work.v[30];
  cmo_work.v[51] = target[66]-cmo_work.L[30]*cmo_work.v[16]-cmo_work.L[31]*cmo_work.v[30]-cmo_work.L[32]*cmo_work.v[50];
  cmo_work.v[52] = target[18]-cmo_work.L[33]*cmo_work.v[50]-cmo_work.L[34]*cmo_work.v[51];
  cmo_work.v[53] = target[55]-cmo_work.L[35]*cmo_work.v[5]-cmo_work.L[36]*cmo_work.v[31];
  cmo_work.v[54] = target[67]-cmo_work.L[37]*cmo_work.v[17]-cmo_work.L[38]*cmo_work.v[31]-cmo_work.L[39]*cmo_work.v[53];
  cmo_work.v[55] = target[19]-cmo_work.L[40]*cmo_work.v[53]-cmo_work.L[41]*cmo_work.v[54];
  cmo_work.v[56] = target[56]-cmo_work.L[42]*cmo_work.v[6]-cmo_work.L[43]*cmo_work.v[32];
  cmo_work.v[57] = target[68]-cmo_work.L[44]*cmo_work.v[18]-cmo_work.L[45]*cmo_work.v[32]-cmo_work.L[46]*cmo_work.v[56];
  cmo_work.v[58] = target[20]-cmo_work.L[47]*cmo_work.v[56]-cmo_work.L[48]*cmo_work.v[57];
  cmo_work.v[59] = target[57]-cmo_work.L[49]*cmo_work.v[7]-cmo_work.L[50]*cmo_work.v[33];
  cmo_work.v[60] = target[69]-cmo_work.L[51]*cmo_work.v[19]-cmo_work.L[52]*cmo_work.v[33]-cmo_work.L[53]*cmo_work.v[59];
  cmo_work.v[61] = target[21]-cmo_work.L[54]*cmo_work.v[59]-cmo_work.L[55]*cmo_work.v[60];
  cmo_work.v[62] = target[58]-cmo_work.L[56]*cmo_work.v[8]-cmo_work.L[57]*cmo_work.v[34];
  cmo_work.v[63] = target[70]-cmo_work.L[58]*cmo_work.v[20]-cmo_work.L[59]*cmo_work.v[34]-cmo_work.L[60]*cmo_work.v[62];
  cmo_work.v[64] = target[22]-cmo_work.L[61]*cmo_work.v[62]-cmo_work.L[62]*cmo_work.v[63];
  cmo_work.v[65] = target[59]-cmo_work.L[63]*cmo_work.v[9]-cmo_work.L[64]*cmo_work.v[35];
  cmo_work.v[66] = target[71]-cmo_work.L[65]*cmo_work.v[21]-cmo_work.L[66]*cmo_work.v[35]-cmo_work.L[67]*cmo_work.v[65];
  cmo_work.v[67] = target[23]-cmo_work.L[68]*cmo_work.v[65]-cmo_work.L[69]*cmo_work.v[66];
  cmo_work.v[68] = target[60]-cmo_work.L[70]*cmo_work.v[10]-cmo_work.L[71]*cmo_work.v[36];
  cmo_work.v[69] = target[72]-cmo_work.L[72]*cmo_work.v[22]-cmo_work.L[73]*cmo_work.v[36]-cmo_work.L[74]*cmo_work.v[68];
  cmo_work.v[70] = target[24]-cmo_work.L[75]*cmo_work.v[68]-cmo_work.L[76]*cmo_work.v[69];
  cmo_work.v[71] = target[61]-cmo_work.L[77]*cmo_work.v[11]-cmo_work.L[78]*cmo_work.v[37];
  cmo_work.v[72] = target[73]-cmo_work.L[79]*cmo_work.v[23]-cmo_work.L[80]*cmo_work.v[37]-cmo_work.L[81]*cmo_work.v[71];
  cmo_work.v[73] = target[25]-cmo_work.L[82]*cmo_work.v[71]-cmo_work.L[83]*cmo_work.v[72];
  cmo_work.v[74] = target[74]-cmo_work.L[84]*cmo_work.v[24]-cmo_work.L[85]*cmo_work.v[40]-cmo_work.L[86]*cmo_work.v[43]-cmo_work.L[87]*cmo_work.v[46]-cmo_work.L[88]*cmo_work.v[49]-cmo_work.L[89]*cmo_work.v[52]-cmo_work.L[90]*cmo_work.v[55]-cmo_work.L[91]*cmo_work.v[58]-cmo_work.L[92]*cmo_work.v[61]-cmo_work.L[93]*cmo_work.v[64]-cmo_work.L[94]*cmo_work.v[67]-cmo_work.L[95]*cmo_work.v[70]-cmo_work.L[96]*cmo_work.v[73];
  cmo_work.v[75] = target[75]-cmo_work.L[97]*cmo_work.v[25]-cmo_work.L[98]*cmo_work.v[40]-cmo_work.L[99]*cmo_work.v[43]-cmo_work.L[100]*cmo_work.v[46]-cmo_work.L[101]*cmo_work.v[49]-cmo_work.L[102]*cmo_work.v[52]-cmo_work.L[103]*cmo_work.v[55]-cmo_work.L[104]*cmo_work.v[58]-cmo_work.L[105]*cmo_work.v[61]-cmo_work.L[106]*cmo_work.v[64]-cmo_work.L[107]*cmo_work.v[67]-cmo_work.L[108]*cmo_work.v[70]-cmo_work.L[109]*cmo_work.v[73]-cmo_work.L[110]*cmo_work.v[74];
  /* Diagonal scaling. Assume correctness of cmo_work.d_inv. */
  for (i = 0; i < 76; i++)
    cmo_work.v[i] *= cmo_work.d_inv[i];
  /* Back substitution */
  cmo_work.v[74] -= cmo_work.L[110]*cmo_work.v[75];
  cmo_work.v[73] -= cmo_work.L[96]*cmo_work.v[74]+cmo_work.L[109]*cmo_work.v[75];
  cmo_work.v[72] -= cmo_work.L[83]*cmo_work.v[73];
  cmo_work.v[71] -= cmo_work.L[81]*cmo_work.v[72]+cmo_work.L[82]*cmo_work.v[73];
  cmo_work.v[70] -= cmo_work.L[95]*cmo_work.v[74]+cmo_work.L[108]*cmo_work.v[75];
  cmo_work.v[69] -= cmo_work.L[76]*cmo_work.v[70];
  cmo_work.v[68] -= cmo_work.L[74]*cmo_work.v[69]+cmo_work.L[75]*cmo_work.v[70];
  cmo_work.v[67] -= cmo_work.L[94]*cmo_work.v[74]+cmo_work.L[107]*cmo_work.v[75];
  cmo_work.v[66] -= cmo_work.L[69]*cmo_work.v[67];
  cmo_work.v[65] -= cmo_work.L[67]*cmo_work.v[66]+cmo_work.L[68]*cmo_work.v[67];
  cmo_work.v[64] -= cmo_work.L[93]*cmo_work.v[74]+cmo_work.L[106]*cmo_work.v[75];
  cmo_work.v[63] -= cmo_work.L[62]*cmo_work.v[64];
  cmo_work.v[62] -= cmo_work.L[60]*cmo_work.v[63]+cmo_work.L[61]*cmo_work.v[64];
  cmo_work.v[61] -= cmo_work.L[92]*cmo_work.v[74]+cmo_work.L[105]*cmo_work.v[75];
  cmo_work.v[60] -= cmo_work.L[55]*cmo_work.v[61];
  cmo_work.v[59] -= cmo_work.L[53]*cmo_work.v[60]+cmo_work.L[54]*cmo_work.v[61];
  cmo_work.v[58] -= cmo_work.L[91]*cmo_work.v[74]+cmo_work.L[104]*cmo_work.v[75];
  cmo_work.v[57] -= cmo_work.L[48]*cmo_work.v[58];
  cmo_work.v[56] -= cmo_work.L[46]*cmo_work.v[57]+cmo_work.L[47]*cmo_work.v[58];
  cmo_work.v[55] -= cmo_work.L[90]*cmo_work.v[74]+cmo_work.L[103]*cmo_work.v[75];
  cmo_work.v[54] -= cmo_work.L[41]*cmo_work.v[55];
  cmo_work.v[53] -= cmo_work.L[39]*cmo_work.v[54]+cmo_work.L[40]*cmo_work.v[55];
  cmo_work.v[52] -= cmo_work.L[89]*cmo_work.v[74]+cmo_work.L[102]*cmo_work.v[75];
  cmo_work.v[51] -= cmo_work.L[34]*cmo_work.v[52];
  cmo_work.v[50] -= cmo_work.L[32]*cmo_work.v[51]+cmo_work.L[33]*cmo_work.v[52];
  cmo_work.v[49] -= cmo_work.L[88]*cmo_work.v[74]+cmo_work.L[101]*cmo_work.v[75];
  cmo_work.v[48] -= cmo_work.L[27]*cmo_work.v[49];
  cmo_work.v[47] -= cmo_work.L[25]*cmo_work.v[48]+cmo_work.L[26]*cmo_work.v[49];
  cmo_work.v[46] -= cmo_work.L[87]*cmo_work.v[74]+cmo_work.L[100]*cmo_work.v[75];
  cmo_work.v[45] -= cmo_work.L[20]*cmo_work.v[46];
  cmo_work.v[44] -= cmo_work.L[18]*cmo_work.v[45]+cmo_work.L[19]*cmo_work.v[46];
  cmo_work.v[43] -= cmo_work.L[86]*cmo_work.v[74]+cmo_work.L[99]*cmo_work.v[75];
  cmo_work.v[42] -= cmo_work.L[13]*cmo_work.v[43];
  cmo_work.v[41] -= cmo_work.L[11]*cmo_work.v[42]+cmo_work.L[12]*cmo_work.v[43];
  cmo_work.v[40] -= cmo_work.L[85]*cmo_work.v[74]+cmo_work.L[98]*cmo_work.v[75];
  cmo_work.v[39] -= cmo_work.L[6]*cmo_work.v[40];
  cmo_work.v[38] -= cmo_work.L[4]*cmo_work.v[39]+cmo_work.L[5]*cmo_work.v[40];
  cmo_work.v[37] -= cmo_work.L[78]*cmo_work.v[71]+cmo_work.L[80]*cmo_work.v[72];
  cmo_work.v[36] -= cmo_work.L[71]*cmo_work.v[68]+cmo_work.L[73]*cmo_work.v[69];
  cmo_work.v[35] -= cmo_work.L[64]*cmo_work.v[65]+cmo_work.L[66]*cmo_work.v[66];
  cmo_work.v[34] -= cmo_work.L[57]*cmo_work.v[62]+cmo_work.L[59]*cmo_work.v[63];
  cmo_work.v[33] -= cmo_work.L[50]*cmo_work.v[59]+cmo_work.L[52]*cmo_work.v[60];
  cmo_work.v[32] -= cmo_work.L[43]*cmo_work.v[56]+cmo_work.L[45]*cmo_work.v[57];
  cmo_work.v[31] -= cmo_work.L[36]*cmo_work.v[53]+cmo_work.L[38]*cmo_work.v[54];
  cmo_work.v[30] -= cmo_work.L[29]*cmo_work.v[50]+cmo_work.L[31]*cmo_work.v[51];
  cmo_work.v[29] -= cmo_work.L[22]*cmo_work.v[47]+cmo_work.L[24]*cmo_work.v[48];
  cmo_work.v[28] -= cmo_work.L[15]*cmo_work.v[44]+cmo_work.L[17]*cmo_work.v[45];
  cmo_work.v[27] -= cmo_work.L[8]*cmo_work.v[41]+cmo_work.L[10]*cmo_work.v[42];
  cmo_work.v[26] -= cmo_work.L[1]*cmo_work.v[38]+cmo_work.L[3]*cmo_work.v[39];
  cmo_work.v[25] -= cmo_work.L[97]*cmo_work.v[75];
  cmo_work.v[24] -= cmo_work.L[84]*cmo_work.v[74];
  cmo_work.v[23] -= cmo_work.L[79]*cmo_work.v[72];
  cmo_work.v[22] -= cmo_work.L[72]*cmo_work.v[69];
  cmo_work.v[21] -= cmo_work.L[65]*cmo_work.v[66];
  cmo_work.v[20] -= cmo_work.L[58]*cmo_work.v[63];
  cmo_work.v[19] -= cmo_work.L[51]*cmo_work.v[60];
  cmo_work.v[18] -= cmo_work.L[44]*cmo_work.v[57];
  cmo_work.v[17] -= cmo_work.L[37]*cmo_work.v[54];
  cmo_work.v[16] -= cmo_work.L[30]*cmo_work.v[51];
  cmo_work.v[15] -= cmo_work.L[23]*cmo_work.v[48];
  cmo_work.v[14] -= cmo_work.L[16]*cmo_work.v[45];
  cmo_work.v[13] -= cmo_work.L[9]*cmo_work.v[42];
  cmo_work.v[12] -= cmo_work.L[2]*cmo_work.v[39];
  cmo_work.v[11] -= cmo_work.L[77]*cmo_work.v[71];
  cmo_work.v[10] -= cmo_work.L[70]*cmo_work.v[68];
  cmo_work.v[9] -= cmo_work.L[63]*cmo_work.v[65];
  cmo_work.v[8] -= cmo_work.L[56]*cmo_work.v[62];
  cmo_work.v[7] -= cmo_work.L[49]*cmo_work.v[59];
  cmo_work.v[6] -= cmo_work.L[42]*cmo_work.v[56];
  cmo_work.v[5] -= cmo_work.L[35]*cmo_work.v[53];
  cmo_work.v[4] -= cmo_work.L[28]*cmo_work.v[50];
  cmo_work.v[3] -= cmo_work.L[21]*cmo_work.v[47];
  cmo_work.v[2] -= cmo_work.L[14]*cmo_work.v[44];
  cmo_work.v[1] -= cmo_work.L[7]*cmo_work.v[41];
  cmo_work.v[0] -= cmo_work.L[0]*cmo_work.v[38];
  /* Unpermute the result, from v to var. */
  var[0] = cmo_work.v[24];
  var[1] = cmo_work.v[25];
  var[2] = cmo_work.v[26];
  var[3] = cmo_work.v[27];
  var[4] = cmo_work.v[28];
  var[5] = cmo_work.v[29];
  var[6] = cmo_work.v[30];
  var[7] = cmo_work.v[31];
  var[8] = cmo_work.v[32];
  var[9] = cmo_work.v[33];
  var[10] = cmo_work.v[34];
  var[11] = cmo_work.v[35];
  var[12] = cmo_work.v[36];
  var[13] = cmo_work.v[37];
  var[14] = cmo_work.v[40];
  var[15] = cmo_work.v[43];
  var[16] = cmo_work.v[46];
  var[17] = cmo_work.v[49];
  var[18] = cmo_work.v[52];
  var[19] = cmo_work.v[55];
  var[20] = cmo_work.v[58];
  var[21] = cmo_work.v[61];
  var[22] = cmo_work.v[64];
  var[23] = cmo_work.v[67];
  var[24] = cmo_work.v[70];
  var[25] = cmo_work.v[73];
  var[26] = cmo_work.v[0];
  var[27] = cmo_work.v[1];
  var[28] = cmo_work.v[2];
  var[29] = cmo_work.v[3];
  var[30] = cmo_work.v[4];
  var[31] = cmo_work.v[5];
  var[32] = cmo_work.v[6];
  var[33] = cmo_work.v[7];
  var[34] = cmo_work.v[8];
  var[35] = cmo_work.v[9];
  var[36] = cmo_work.v[10];
  var[37] = cmo_work.v[11];
  var[38] = cmo_work.v[12];
  var[39] = cmo_work.v[13];
  var[40] = cmo_work.v[14];
  var[41] = cmo_work.v[15];
  var[42] = cmo_work.v[16];
  var[43] = cmo_work.v[17];
  var[44] = cmo_work.v[18];
  var[45] = cmo_work.v[19];
  var[46] = cmo_work.v[20];
  var[47] = cmo_work.v[21];
  var[48] = cmo_work.v[22];
  var[49] = cmo_work.v[23];
  var[50] = cmo_work.v[38];
  var[51] = cmo_work.v[41];
  var[52] = cmo_work.v[44];
  var[53] = cmo_work.v[47];
  var[54] = cmo_work.v[50];
  var[55] = cmo_work.v[53];
  var[56] = cmo_work.v[56];
  var[57] = cmo_work.v[59];
  var[58] = cmo_work.v[62];
  var[59] = cmo_work.v[65];
  var[60] = cmo_work.v[68];
  var[61] = cmo_work.v[71];
  var[62] = cmo_work.v[39];
  var[63] = cmo_work.v[42];
  var[64] = cmo_work.v[45];
  var[65] = cmo_work.v[48];
  var[66] = cmo_work.v[51];
  var[67] = cmo_work.v[54];
  var[68] = cmo_work.v[57];
  var[69] = cmo_work.v[60];
  var[70] = cmo_work.v[63];
  var[71] = cmo_work.v[66];
  var[72] = cmo_work.v[69];
  var[73] = cmo_work.v[72];
  var[74] = cmo_work.v[74];
  var[75] = cmo_work.v[75];
#ifndef ZERO_LIBRARY_MODE
  if (cmo_settings.debug) {
    printf("Squared norm for solution is %.8g.\n", cmo_check_residual(target, var));
  }
#endif
}
void cmo_ldl_factor(void) {
  cmo_work.d[0] = cmo_work.KKT[0];
  if (cmo_work.d[0] < 0)
    cmo_work.d[0] = cmo_settings.kkt_reg;
  else
    cmo_work.d[0] += cmo_settings.kkt_reg;
  cmo_work.d_inv[0] = 1/cmo_work.d[0];
  cmo_work.L[0] = cmo_work.KKT[1]*cmo_work.d_inv[0];
  cmo_work.v[1] = cmo_work.KKT[2];
  cmo_work.d[1] = cmo_work.v[1];
  if (cmo_work.d[1] < 0)
    cmo_work.d[1] = cmo_settings.kkt_reg;
  else
    cmo_work.d[1] += cmo_settings.kkt_reg;
  cmo_work.d_inv[1] = 1/cmo_work.d[1];
  cmo_work.L[7] = (cmo_work.KKT[3])*cmo_work.d_inv[1];
  cmo_work.v[2] = cmo_work.KKT[4];
  cmo_work.d[2] = cmo_work.v[2];
  if (cmo_work.d[2] < 0)
    cmo_work.d[2] = cmo_settings.kkt_reg;
  else
    cmo_work.d[2] += cmo_settings.kkt_reg;
  cmo_work.d_inv[2] = 1/cmo_work.d[2];
  cmo_work.L[14] = (cmo_work.KKT[5])*cmo_work.d_inv[2];
  cmo_work.v[3] = cmo_work.KKT[6];
  cmo_work.d[3] = cmo_work.v[3];
  if (cmo_work.d[3] < 0)
    cmo_work.d[3] = cmo_settings.kkt_reg;
  else
    cmo_work.d[3] += cmo_settings.kkt_reg;
  cmo_work.d_inv[3] = 1/cmo_work.d[3];
  cmo_work.L[21] = (cmo_work.KKT[7])*cmo_work.d_inv[3];
  cmo_work.v[4] = cmo_work.KKT[8];
  cmo_work.d[4] = cmo_work.v[4];
  if (cmo_work.d[4] < 0)
    cmo_work.d[4] = cmo_settings.kkt_reg;
  else
    cmo_work.d[4] += cmo_settings.kkt_reg;
  cmo_work.d_inv[4] = 1/cmo_work.d[4];
  cmo_work.L[28] = (cmo_work.KKT[9])*cmo_work.d_inv[4];
  cmo_work.v[5] = cmo_work.KKT[10];
  cmo_work.d[5] = cmo_work.v[5];
  if (cmo_work.d[5] < 0)
    cmo_work.d[5] = cmo_settings.kkt_reg;
  else
    cmo_work.d[5] += cmo_settings.kkt_reg;
  cmo_work.d_inv[5] = 1/cmo_work.d[5];
  cmo_work.L[35] = (cmo_work.KKT[11])*cmo_work.d_inv[5];
  cmo_work.v[6] = cmo_work.KKT[12];
  cmo_work.d[6] = cmo_work.v[6];
  if (cmo_work.d[6] < 0)
    cmo_work.d[6] = cmo_settings.kkt_reg;
  else
    cmo_work.d[6] += cmo_settings.kkt_reg;
  cmo_work.d_inv[6] = 1/cmo_work.d[6];
  cmo_work.L[42] = (cmo_work.KKT[13])*cmo_work.d_inv[6];
  cmo_work.v[7] = cmo_work.KKT[14];
  cmo_work.d[7] = cmo_work.v[7];
  if (cmo_work.d[7] < 0)
    cmo_work.d[7] = cmo_settings.kkt_reg;
  else
    cmo_work.d[7] += cmo_settings.kkt_reg;
  cmo_work.d_inv[7] = 1/cmo_work.d[7];
  cmo_work.L[49] = (cmo_work.KKT[15])*cmo_work.d_inv[7];
  cmo_work.v[8] = cmo_work.KKT[16];
  cmo_work.d[8] = cmo_work.v[8];
  if (cmo_work.d[8] < 0)
    cmo_work.d[8] = cmo_settings.kkt_reg;
  else
    cmo_work.d[8] += cmo_settings.kkt_reg;
  cmo_work.d_inv[8] = 1/cmo_work.d[8];
  cmo_work.L[56] = (cmo_work.KKT[17])*cmo_work.d_inv[8];
  cmo_work.v[9] = cmo_work.KKT[18];
  cmo_work.d[9] = cmo_work.v[9];
  if (cmo_work.d[9] < 0)
    cmo_work.d[9] = cmo_settings.kkt_reg;
  else
    cmo_work.d[9] += cmo_settings.kkt_reg;
  cmo_work.d_inv[9] = 1/cmo_work.d[9];
  cmo_work.L[63] = (cmo_work.KKT[19])*cmo_work.d_inv[9];
  cmo_work.v[10] = cmo_work.KKT[20];
  cmo_work.d[10] = cmo_work.v[10];
  if (cmo_work.d[10] < 0)
    cmo_work.d[10] = cmo_settings.kkt_reg;
  else
    cmo_work.d[10] += cmo_settings.kkt_reg;
  cmo_work.d_inv[10] = 1/cmo_work.d[10];
  cmo_work.L[70] = (cmo_work.KKT[21])*cmo_work.d_inv[10];
  cmo_work.v[11] = cmo_work.KKT[22];
  cmo_work.d[11] = cmo_work.v[11];
  if (cmo_work.d[11] < 0)
    cmo_work.d[11] = cmo_settings.kkt_reg;
  else
    cmo_work.d[11] += cmo_settings.kkt_reg;
  cmo_work.d_inv[11] = 1/cmo_work.d[11];
  cmo_work.L[77] = (cmo_work.KKT[23])*cmo_work.d_inv[11];
  cmo_work.v[12] = cmo_work.KKT[24];
  cmo_work.d[12] = cmo_work.v[12];
  if (cmo_work.d[12] < 0)
    cmo_work.d[12] = cmo_settings.kkt_reg;
  else
    cmo_work.d[12] += cmo_settings.kkt_reg;
  cmo_work.d_inv[12] = 1/cmo_work.d[12];
  cmo_work.L[2] = (cmo_work.KKT[25])*cmo_work.d_inv[12];
  cmo_work.v[13] = cmo_work.KKT[26];
  cmo_work.d[13] = cmo_work.v[13];
  if (cmo_work.d[13] < 0)
    cmo_work.d[13] = cmo_settings.kkt_reg;
  else
    cmo_work.d[13] += cmo_settings.kkt_reg;
  cmo_work.d_inv[13] = 1/cmo_work.d[13];
  cmo_work.L[9] = (cmo_work.KKT[27])*cmo_work.d_inv[13];
  cmo_work.v[14] = cmo_work.KKT[28];
  cmo_work.d[14] = cmo_work.v[14];
  if (cmo_work.d[14] < 0)
    cmo_work.d[14] = cmo_settings.kkt_reg;
  else
    cmo_work.d[14] += cmo_settings.kkt_reg;
  cmo_work.d_inv[14] = 1/cmo_work.d[14];
  cmo_work.L[16] = (cmo_work.KKT[29])*cmo_work.d_inv[14];
  cmo_work.v[15] = cmo_work.KKT[30];
  cmo_work.d[15] = cmo_work.v[15];
  if (cmo_work.d[15] < 0)
    cmo_work.d[15] = cmo_settings.kkt_reg;
  else
    cmo_work.d[15] += cmo_settings.kkt_reg;
  cmo_work.d_inv[15] = 1/cmo_work.d[15];
  cmo_work.L[23] = (cmo_work.KKT[31])*cmo_work.d_inv[15];
  cmo_work.v[16] = cmo_work.KKT[32];
  cmo_work.d[16] = cmo_work.v[16];
  if (cmo_work.d[16] < 0)
    cmo_work.d[16] = cmo_settings.kkt_reg;
  else
    cmo_work.d[16] += cmo_settings.kkt_reg;
  cmo_work.d_inv[16] = 1/cmo_work.d[16];
  cmo_work.L[30] = (cmo_work.KKT[33])*cmo_work.d_inv[16];
  cmo_work.v[17] = cmo_work.KKT[34];
  cmo_work.d[17] = cmo_work.v[17];
  if (cmo_work.d[17] < 0)
    cmo_work.d[17] = cmo_settings.kkt_reg;
  else
    cmo_work.d[17] += cmo_settings.kkt_reg;
  cmo_work.d_inv[17] = 1/cmo_work.d[17];
  cmo_work.L[37] = (cmo_work.KKT[35])*cmo_work.d_inv[17];
  cmo_work.v[18] = cmo_work.KKT[36];
  cmo_work.d[18] = cmo_work.v[18];
  if (cmo_work.d[18] < 0)
    cmo_work.d[18] = cmo_settings.kkt_reg;
  else
    cmo_work.d[18] += cmo_settings.kkt_reg;
  cmo_work.d_inv[18] = 1/cmo_work.d[18];
  cmo_work.L[44] = (cmo_work.KKT[37])*cmo_work.d_inv[18];
  cmo_work.v[19] = cmo_work.KKT[38];
  cmo_work.d[19] = cmo_work.v[19];
  if (cmo_work.d[19] < 0)
    cmo_work.d[19] = cmo_settings.kkt_reg;
  else
    cmo_work.d[19] += cmo_settings.kkt_reg;
  cmo_work.d_inv[19] = 1/cmo_work.d[19];
  cmo_work.L[51] = (cmo_work.KKT[39])*cmo_work.d_inv[19];
  cmo_work.v[20] = cmo_work.KKT[40];
  cmo_work.d[20] = cmo_work.v[20];
  if (cmo_work.d[20] < 0)
    cmo_work.d[20] = cmo_settings.kkt_reg;
  else
    cmo_work.d[20] += cmo_settings.kkt_reg;
  cmo_work.d_inv[20] = 1/cmo_work.d[20];
  cmo_work.L[58] = (cmo_work.KKT[41])*cmo_work.d_inv[20];
  cmo_work.v[21] = cmo_work.KKT[42];
  cmo_work.d[21] = cmo_work.v[21];
  if (cmo_work.d[21] < 0)
    cmo_work.d[21] = cmo_settings.kkt_reg;
  else
    cmo_work.d[21] += cmo_settings.kkt_reg;
  cmo_work.d_inv[21] = 1/cmo_work.d[21];
  cmo_work.L[65] = (cmo_work.KKT[43])*cmo_work.d_inv[21];
  cmo_work.v[22] = cmo_work.KKT[44];
  cmo_work.d[22] = cmo_work.v[22];
  if (cmo_work.d[22] < 0)
    cmo_work.d[22] = cmo_settings.kkt_reg;
  else
    cmo_work.d[22] += cmo_settings.kkt_reg;
  cmo_work.d_inv[22] = 1/cmo_work.d[22];
  cmo_work.L[72] = (cmo_work.KKT[45])*cmo_work.d_inv[22];
  cmo_work.v[23] = cmo_work.KKT[46];
  cmo_work.d[23] = cmo_work.v[23];
  if (cmo_work.d[23] < 0)
    cmo_work.d[23] = cmo_settings.kkt_reg;
  else
    cmo_work.d[23] += cmo_settings.kkt_reg;
  cmo_work.d_inv[23] = 1/cmo_work.d[23];
  cmo_work.L[79] = (cmo_work.KKT[47])*cmo_work.d_inv[23];
  cmo_work.v[24] = cmo_work.KKT[48];
  cmo_work.d[24] = cmo_work.v[24];
  if (cmo_work.d[24] < 0)
    cmo_work.d[24] = cmo_settings.kkt_reg;
  else
    cmo_work.d[24] += cmo_settings.kkt_reg;
  cmo_work.d_inv[24] = 1/cmo_work.d[24];
  cmo_work.L[84] = (cmo_work.KKT[49])*cmo_work.d_inv[24];
  cmo_work.v[25] = cmo_work.KKT[50];
  cmo_work.d[25] = cmo_work.v[25];
  if (cmo_work.d[25] < 0)
    cmo_work.d[25] = cmo_settings.kkt_reg;
  else
    cmo_work.d[25] += cmo_settings.kkt_reg;
  cmo_work.d_inv[25] = 1/cmo_work.d[25];
  cmo_work.L[97] = (cmo_work.KKT[51])*cmo_work.d_inv[25];
  cmo_work.v[26] = 0;
  cmo_work.d[26] = cmo_work.v[26];
  if (cmo_work.d[26] < 0)
    cmo_work.d[26] = cmo_settings.kkt_reg;
  else
    cmo_work.d[26] += cmo_settings.kkt_reg;
  cmo_work.d_inv[26] = 1/cmo_work.d[26];
  cmo_work.L[1] = (cmo_work.KKT[52])*cmo_work.d_inv[26];
  cmo_work.L[3] = (cmo_work.KKT[53])*cmo_work.d_inv[26];
  cmo_work.v[27] = 0;
  cmo_work.d[27] = cmo_work.v[27];
  if (cmo_work.d[27] < 0)
    cmo_work.d[27] = cmo_settings.kkt_reg;
  else
    cmo_work.d[27] += cmo_settings.kkt_reg;
  cmo_work.d_inv[27] = 1/cmo_work.d[27];
  cmo_work.L[8] = (cmo_work.KKT[54])*cmo_work.d_inv[27];
  cmo_work.L[10] = (cmo_work.KKT[55])*cmo_work.d_inv[27];
  cmo_work.v[28] = 0;
  cmo_work.d[28] = cmo_work.v[28];
  if (cmo_work.d[28] < 0)
    cmo_work.d[28] = cmo_settings.kkt_reg;
  else
    cmo_work.d[28] += cmo_settings.kkt_reg;
  cmo_work.d_inv[28] = 1/cmo_work.d[28];
  cmo_work.L[15] = (cmo_work.KKT[56])*cmo_work.d_inv[28];
  cmo_work.L[17] = (cmo_work.KKT[57])*cmo_work.d_inv[28];
  cmo_work.v[29] = 0;
  cmo_work.d[29] = cmo_work.v[29];
  if (cmo_work.d[29] < 0)
    cmo_work.d[29] = cmo_settings.kkt_reg;
  else
    cmo_work.d[29] += cmo_settings.kkt_reg;
  cmo_work.d_inv[29] = 1/cmo_work.d[29];
  cmo_work.L[22] = (cmo_work.KKT[58])*cmo_work.d_inv[29];
  cmo_work.L[24] = (cmo_work.KKT[59])*cmo_work.d_inv[29];
  cmo_work.v[30] = 0;
  cmo_work.d[30] = cmo_work.v[30];
  if (cmo_work.d[30] < 0)
    cmo_work.d[30] = cmo_settings.kkt_reg;
  else
    cmo_work.d[30] += cmo_settings.kkt_reg;
  cmo_work.d_inv[30] = 1/cmo_work.d[30];
  cmo_work.L[29] = (cmo_work.KKT[60])*cmo_work.d_inv[30];
  cmo_work.L[31] = (cmo_work.KKT[61])*cmo_work.d_inv[30];
  cmo_work.v[31] = 0;
  cmo_work.d[31] = cmo_work.v[31];
  if (cmo_work.d[31] < 0)
    cmo_work.d[31] = cmo_settings.kkt_reg;
  else
    cmo_work.d[31] += cmo_settings.kkt_reg;
  cmo_work.d_inv[31] = 1/cmo_work.d[31];
  cmo_work.L[36] = (cmo_work.KKT[62])*cmo_work.d_inv[31];
  cmo_work.L[38] = (cmo_work.KKT[63])*cmo_work.d_inv[31];
  cmo_work.v[32] = 0;
  cmo_work.d[32] = cmo_work.v[32];
  if (cmo_work.d[32] < 0)
    cmo_work.d[32] = cmo_settings.kkt_reg;
  else
    cmo_work.d[32] += cmo_settings.kkt_reg;
  cmo_work.d_inv[32] = 1/cmo_work.d[32];
  cmo_work.L[43] = (cmo_work.KKT[64])*cmo_work.d_inv[32];
  cmo_work.L[45] = (cmo_work.KKT[65])*cmo_work.d_inv[32];
  cmo_work.v[33] = 0;
  cmo_work.d[33] = cmo_work.v[33];
  if (cmo_work.d[33] < 0)
    cmo_work.d[33] = cmo_settings.kkt_reg;
  else
    cmo_work.d[33] += cmo_settings.kkt_reg;
  cmo_work.d_inv[33] = 1/cmo_work.d[33];
  cmo_work.L[50] = (cmo_work.KKT[66])*cmo_work.d_inv[33];
  cmo_work.L[52] = (cmo_work.KKT[67])*cmo_work.d_inv[33];
  cmo_work.v[34] = 0;
  cmo_work.d[34] = cmo_work.v[34];
  if (cmo_work.d[34] < 0)
    cmo_work.d[34] = cmo_settings.kkt_reg;
  else
    cmo_work.d[34] += cmo_settings.kkt_reg;
  cmo_work.d_inv[34] = 1/cmo_work.d[34];
  cmo_work.L[57] = (cmo_work.KKT[68])*cmo_work.d_inv[34];
  cmo_work.L[59] = (cmo_work.KKT[69])*cmo_work.d_inv[34];
  cmo_work.v[35] = 0;
  cmo_work.d[35] = cmo_work.v[35];
  if (cmo_work.d[35] < 0)
    cmo_work.d[35] = cmo_settings.kkt_reg;
  else
    cmo_work.d[35] += cmo_settings.kkt_reg;
  cmo_work.d_inv[35] = 1/cmo_work.d[35];
  cmo_work.L[64] = (cmo_work.KKT[70])*cmo_work.d_inv[35];
  cmo_work.L[66] = (cmo_work.KKT[71])*cmo_work.d_inv[35];
  cmo_work.v[36] = 0;
  cmo_work.d[36] = cmo_work.v[36];
  if (cmo_work.d[36] < 0)
    cmo_work.d[36] = cmo_settings.kkt_reg;
  else
    cmo_work.d[36] += cmo_settings.kkt_reg;
  cmo_work.d_inv[36] = 1/cmo_work.d[36];
  cmo_work.L[71] = (cmo_work.KKT[72])*cmo_work.d_inv[36];
  cmo_work.L[73] = (cmo_work.KKT[73])*cmo_work.d_inv[36];
  cmo_work.v[37] = 0;
  cmo_work.d[37] = cmo_work.v[37];
  if (cmo_work.d[37] < 0)
    cmo_work.d[37] = cmo_settings.kkt_reg;
  else
    cmo_work.d[37] += cmo_settings.kkt_reg;
  cmo_work.d_inv[37] = 1/cmo_work.d[37];
  cmo_work.L[78] = (cmo_work.KKT[74])*cmo_work.d_inv[37];
  cmo_work.L[80] = (cmo_work.KKT[75])*cmo_work.d_inv[37];
  cmo_work.v[0] = cmo_work.L[0]*cmo_work.d[0];
  cmo_work.v[26] = cmo_work.L[1]*cmo_work.d[26];
  cmo_work.v[38] = cmo_work.KKT[76]-cmo_work.L[0]*cmo_work.v[0]-cmo_work.L[1]*cmo_work.v[26];
  cmo_work.d[38] = cmo_work.v[38];
  if (cmo_work.d[38] > 0)
    cmo_work.d[38] = -cmo_settings.kkt_reg;
  else
    cmo_work.d[38] -= cmo_settings.kkt_reg;
  cmo_work.d_inv[38] = 1/cmo_work.d[38];
  cmo_work.L[4] = (-cmo_work.L[3]*cmo_work.v[26])*cmo_work.d_inv[38];
  cmo_work.L[5] = (cmo_work.KKT[77])*cmo_work.d_inv[38];
  cmo_work.v[12] = cmo_work.L[2]*cmo_work.d[12];
  cmo_work.v[26] = cmo_work.L[3]*cmo_work.d[26];
  cmo_work.v[38] = cmo_work.L[4]*cmo_work.d[38];
  cmo_work.v[39] = cmo_work.KKT[78]-cmo_work.L[2]*cmo_work.v[12]-cmo_work.L[3]*cmo_work.v[26]-cmo_work.L[4]*cmo_work.v[38];
  cmo_work.d[39] = cmo_work.v[39];
  if (cmo_work.d[39] > 0)
    cmo_work.d[39] = -cmo_settings.kkt_reg;
  else
    cmo_work.d[39] -= cmo_settings.kkt_reg;
  cmo_work.d_inv[39] = 1/cmo_work.d[39];
  cmo_work.L[6] = (cmo_work.KKT[79]-cmo_work.L[5]*cmo_work.v[38])*cmo_work.d_inv[39];
  cmo_work.v[38] = cmo_work.L[5]*cmo_work.d[38];
  cmo_work.v[39] = cmo_work.L[6]*cmo_work.d[39];
  cmo_work.v[40] = 0-cmo_work.L[5]*cmo_work.v[38]-cmo_work.L[6]*cmo_work.v[39];
  cmo_work.d[40] = cmo_work.v[40];
  if (cmo_work.d[40] < 0)
    cmo_work.d[40] = cmo_settings.kkt_reg;
  else
    cmo_work.d[40] += cmo_settings.kkt_reg;
  cmo_work.d_inv[40] = 1/cmo_work.d[40];
  cmo_work.L[85] = (cmo_work.KKT[80])*cmo_work.d_inv[40];
  cmo_work.L[98] = (cmo_work.KKT[81])*cmo_work.d_inv[40];
  cmo_work.v[1] = cmo_work.L[7]*cmo_work.d[1];
  cmo_work.v[27] = cmo_work.L[8]*cmo_work.d[27];
  cmo_work.v[41] = cmo_work.KKT[82]-cmo_work.L[7]*cmo_work.v[1]-cmo_work.L[8]*cmo_work.v[27];
  cmo_work.d[41] = cmo_work.v[41];
  if (cmo_work.d[41] > 0)
    cmo_work.d[41] = -cmo_settings.kkt_reg;
  else
    cmo_work.d[41] -= cmo_settings.kkt_reg;
  cmo_work.d_inv[41] = 1/cmo_work.d[41];
  cmo_work.L[11] = (-cmo_work.L[10]*cmo_work.v[27])*cmo_work.d_inv[41];
  cmo_work.L[12] = (cmo_work.KKT[83])*cmo_work.d_inv[41];
  cmo_work.v[13] = cmo_work.L[9]*cmo_work.d[13];
  cmo_work.v[27] = cmo_work.L[10]*cmo_work.d[27];
  cmo_work.v[41] = cmo_work.L[11]*cmo_work.d[41];
  cmo_work.v[42] = cmo_work.KKT[84]-cmo_work.L[9]*cmo_work.v[13]-cmo_work.L[10]*cmo_work.v[27]-cmo_work.L[11]*cmo_work.v[41];
  cmo_work.d[42] = cmo_work.v[42];
  if (cmo_work.d[42] > 0)
    cmo_work.d[42] = -cmo_settings.kkt_reg;
  else
    cmo_work.d[42] -= cmo_settings.kkt_reg;
  cmo_work.d_inv[42] = 1/cmo_work.d[42];
  cmo_work.L[13] = (cmo_work.KKT[85]-cmo_work.L[12]*cmo_work.v[41])*cmo_work.d_inv[42];
  cmo_work.v[41] = cmo_work.L[12]*cmo_work.d[41];
  cmo_work.v[42] = cmo_work.L[13]*cmo_work.d[42];
  cmo_work.v[43] = 0-cmo_work.L[12]*cmo_work.v[41]-cmo_work.L[13]*cmo_work.v[42];
  cmo_work.d[43] = cmo_work.v[43];
  if (cmo_work.d[43] < 0)
    cmo_work.d[43] = cmo_settings.kkt_reg;
  else
    cmo_work.d[43] += cmo_settings.kkt_reg;
  cmo_work.d_inv[43] = 1/cmo_work.d[43];
  cmo_work.L[86] = (cmo_work.KKT[86])*cmo_work.d_inv[43];
  cmo_work.L[99] = (cmo_work.KKT[87])*cmo_work.d_inv[43];
  cmo_work.v[2] = cmo_work.L[14]*cmo_work.d[2];
  cmo_work.v[28] = cmo_work.L[15]*cmo_work.d[28];
  cmo_work.v[44] = cmo_work.KKT[88]-cmo_work.L[14]*cmo_work.v[2]-cmo_work.L[15]*cmo_work.v[28];
  cmo_work.d[44] = cmo_work.v[44];
  if (cmo_work.d[44] > 0)
    cmo_work.d[44] = -cmo_settings.kkt_reg;
  else
    cmo_work.d[44] -= cmo_settings.kkt_reg;
  cmo_work.d_inv[44] = 1/cmo_work.d[44];
  cmo_work.L[18] = (-cmo_work.L[17]*cmo_work.v[28])*cmo_work.d_inv[44];
  cmo_work.L[19] = (cmo_work.KKT[89])*cmo_work.d_inv[44];
  cmo_work.v[14] = cmo_work.L[16]*cmo_work.d[14];
  cmo_work.v[28] = cmo_work.L[17]*cmo_work.d[28];
  cmo_work.v[44] = cmo_work.L[18]*cmo_work.d[44];
  cmo_work.v[45] = cmo_work.KKT[90]-cmo_work.L[16]*cmo_work.v[14]-cmo_work.L[17]*cmo_work.v[28]-cmo_work.L[18]*cmo_work.v[44];
  cmo_work.d[45] = cmo_work.v[45];
  if (cmo_work.d[45] > 0)
    cmo_work.d[45] = -cmo_settings.kkt_reg;
  else
    cmo_work.d[45] -= cmo_settings.kkt_reg;
  cmo_work.d_inv[45] = 1/cmo_work.d[45];
  cmo_work.L[20] = (cmo_work.KKT[91]-cmo_work.L[19]*cmo_work.v[44])*cmo_work.d_inv[45];
  cmo_work.v[44] = cmo_work.L[19]*cmo_work.d[44];
  cmo_work.v[45] = cmo_work.L[20]*cmo_work.d[45];
  cmo_work.v[46] = 0-cmo_work.L[19]*cmo_work.v[44]-cmo_work.L[20]*cmo_work.v[45];
  cmo_work.d[46] = cmo_work.v[46];
  if (cmo_work.d[46] < 0)
    cmo_work.d[46] = cmo_settings.kkt_reg;
  else
    cmo_work.d[46] += cmo_settings.kkt_reg;
  cmo_work.d_inv[46] = 1/cmo_work.d[46];
  cmo_work.L[87] = (cmo_work.KKT[92])*cmo_work.d_inv[46];
  cmo_work.L[100] = (cmo_work.KKT[93])*cmo_work.d_inv[46];
  cmo_work.v[3] = cmo_work.L[21]*cmo_work.d[3];
  cmo_work.v[29] = cmo_work.L[22]*cmo_work.d[29];
  cmo_work.v[47] = cmo_work.KKT[94]-cmo_work.L[21]*cmo_work.v[3]-cmo_work.L[22]*cmo_work.v[29];
  cmo_work.d[47] = cmo_work.v[47];
  if (cmo_work.d[47] > 0)
    cmo_work.d[47] = -cmo_settings.kkt_reg;
  else
    cmo_work.d[47] -= cmo_settings.kkt_reg;
  cmo_work.d_inv[47] = 1/cmo_work.d[47];
  cmo_work.L[25] = (-cmo_work.L[24]*cmo_work.v[29])*cmo_work.d_inv[47];
  cmo_work.L[26] = (cmo_work.KKT[95])*cmo_work.d_inv[47];
  cmo_work.v[15] = cmo_work.L[23]*cmo_work.d[15];
  cmo_work.v[29] = cmo_work.L[24]*cmo_work.d[29];
  cmo_work.v[47] = cmo_work.L[25]*cmo_work.d[47];
  cmo_work.v[48] = cmo_work.KKT[96]-cmo_work.L[23]*cmo_work.v[15]-cmo_work.L[24]*cmo_work.v[29]-cmo_work.L[25]*cmo_work.v[47];
  cmo_work.d[48] = cmo_work.v[48];
  if (cmo_work.d[48] > 0)
    cmo_work.d[48] = -cmo_settings.kkt_reg;
  else
    cmo_work.d[48] -= cmo_settings.kkt_reg;
  cmo_work.d_inv[48] = 1/cmo_work.d[48];
  cmo_work.L[27] = (cmo_work.KKT[97]-cmo_work.L[26]*cmo_work.v[47])*cmo_work.d_inv[48];
  cmo_work.v[47] = cmo_work.L[26]*cmo_work.d[47];
  cmo_work.v[48] = cmo_work.L[27]*cmo_work.d[48];
  cmo_work.v[49] = 0-cmo_work.L[26]*cmo_work.v[47]-cmo_work.L[27]*cmo_work.v[48];
  cmo_work.d[49] = cmo_work.v[49];
  if (cmo_work.d[49] < 0)
    cmo_work.d[49] = cmo_settings.kkt_reg;
  else
    cmo_work.d[49] += cmo_settings.kkt_reg;
  cmo_work.d_inv[49] = 1/cmo_work.d[49];
  cmo_work.L[88] = (cmo_work.KKT[98])*cmo_work.d_inv[49];
  cmo_work.L[101] = (cmo_work.KKT[99])*cmo_work.d_inv[49];
  cmo_work.v[4] = cmo_work.L[28]*cmo_work.d[4];
  cmo_work.v[30] = cmo_work.L[29]*cmo_work.d[30];
  cmo_work.v[50] = cmo_work.KKT[100]-cmo_work.L[28]*cmo_work.v[4]-cmo_work.L[29]*cmo_work.v[30];
  cmo_work.d[50] = cmo_work.v[50];
  if (cmo_work.d[50] > 0)
    cmo_work.d[50] = -cmo_settings.kkt_reg;
  else
    cmo_work.d[50] -= cmo_settings.kkt_reg;
  cmo_work.d_inv[50] = 1/cmo_work.d[50];
  cmo_work.L[32] = (-cmo_work.L[31]*cmo_work.v[30])*cmo_work.d_inv[50];
  cmo_work.L[33] = (cmo_work.KKT[101])*cmo_work.d_inv[50];
  cmo_work.v[16] = cmo_work.L[30]*cmo_work.d[16];
  cmo_work.v[30] = cmo_work.L[31]*cmo_work.d[30];
  cmo_work.v[50] = cmo_work.L[32]*cmo_work.d[50];
  cmo_work.v[51] = cmo_work.KKT[102]-cmo_work.L[30]*cmo_work.v[16]-cmo_work.L[31]*cmo_work.v[30]-cmo_work.L[32]*cmo_work.v[50];
  cmo_work.d[51] = cmo_work.v[51];
  if (cmo_work.d[51] > 0)
    cmo_work.d[51] = -cmo_settings.kkt_reg;
  else
    cmo_work.d[51] -= cmo_settings.kkt_reg;
  cmo_work.d_inv[51] = 1/cmo_work.d[51];
  cmo_work.L[34] = (cmo_work.KKT[103]-cmo_work.L[33]*cmo_work.v[50])*cmo_work.d_inv[51];
  cmo_work.v[50] = cmo_work.L[33]*cmo_work.d[50];
  cmo_work.v[51] = cmo_work.L[34]*cmo_work.d[51];
  cmo_work.v[52] = 0-cmo_work.L[33]*cmo_work.v[50]-cmo_work.L[34]*cmo_work.v[51];
  cmo_work.d[52] = cmo_work.v[52];
  if (cmo_work.d[52] < 0)
    cmo_work.d[52] = cmo_settings.kkt_reg;
  else
    cmo_work.d[52] += cmo_settings.kkt_reg;
  cmo_work.d_inv[52] = 1/cmo_work.d[52];
  cmo_work.L[89] = (cmo_work.KKT[104])*cmo_work.d_inv[52];
  cmo_work.L[102] = (cmo_work.KKT[105])*cmo_work.d_inv[52];
  cmo_work.v[5] = cmo_work.L[35]*cmo_work.d[5];
  cmo_work.v[31] = cmo_work.L[36]*cmo_work.d[31];
  cmo_work.v[53] = cmo_work.KKT[106]-cmo_work.L[35]*cmo_work.v[5]-cmo_work.L[36]*cmo_work.v[31];
  cmo_work.d[53] = cmo_work.v[53];
  if (cmo_work.d[53] > 0)
    cmo_work.d[53] = -cmo_settings.kkt_reg;
  else
    cmo_work.d[53] -= cmo_settings.kkt_reg;
  cmo_work.d_inv[53] = 1/cmo_work.d[53];
  cmo_work.L[39] = (-cmo_work.L[38]*cmo_work.v[31])*cmo_work.d_inv[53];
  cmo_work.L[40] = (cmo_work.KKT[107])*cmo_work.d_inv[53];
  cmo_work.v[17] = cmo_work.L[37]*cmo_work.d[17];
  cmo_work.v[31] = cmo_work.L[38]*cmo_work.d[31];
  cmo_work.v[53] = cmo_work.L[39]*cmo_work.d[53];
  cmo_work.v[54] = cmo_work.KKT[108]-cmo_work.L[37]*cmo_work.v[17]-cmo_work.L[38]*cmo_work.v[31]-cmo_work.L[39]*cmo_work.v[53];
  cmo_work.d[54] = cmo_work.v[54];
  if (cmo_work.d[54] > 0)
    cmo_work.d[54] = -cmo_settings.kkt_reg;
  else
    cmo_work.d[54] -= cmo_settings.kkt_reg;
  cmo_work.d_inv[54] = 1/cmo_work.d[54];
  cmo_work.L[41] = (cmo_work.KKT[109]-cmo_work.L[40]*cmo_work.v[53])*cmo_work.d_inv[54];
  cmo_work.v[53] = cmo_work.L[40]*cmo_work.d[53];
  cmo_work.v[54] = cmo_work.L[41]*cmo_work.d[54];
  cmo_work.v[55] = 0-cmo_work.L[40]*cmo_work.v[53]-cmo_work.L[41]*cmo_work.v[54];
  cmo_work.d[55] = cmo_work.v[55];
  if (cmo_work.d[55] < 0)
    cmo_work.d[55] = cmo_settings.kkt_reg;
  else
    cmo_work.d[55] += cmo_settings.kkt_reg;
  cmo_work.d_inv[55] = 1/cmo_work.d[55];
  cmo_work.L[90] = (cmo_work.KKT[110])*cmo_work.d_inv[55];
  cmo_work.L[103] = (cmo_work.KKT[111])*cmo_work.d_inv[55];
  cmo_work.v[6] = cmo_work.L[42]*cmo_work.d[6];
  cmo_work.v[32] = cmo_work.L[43]*cmo_work.d[32];
  cmo_work.v[56] = cmo_work.KKT[112]-cmo_work.L[42]*cmo_work.v[6]-cmo_work.L[43]*cmo_work.v[32];
  cmo_work.d[56] = cmo_work.v[56];
  if (cmo_work.d[56] > 0)
    cmo_work.d[56] = -cmo_settings.kkt_reg;
  else
    cmo_work.d[56] -= cmo_settings.kkt_reg;
  cmo_work.d_inv[56] = 1/cmo_work.d[56];
  cmo_work.L[46] = (-cmo_work.L[45]*cmo_work.v[32])*cmo_work.d_inv[56];
  cmo_work.L[47] = (cmo_work.KKT[113])*cmo_work.d_inv[56];
  cmo_work.v[18] = cmo_work.L[44]*cmo_work.d[18];
  cmo_work.v[32] = cmo_work.L[45]*cmo_work.d[32];
  cmo_work.v[56] = cmo_work.L[46]*cmo_work.d[56];
  cmo_work.v[57] = cmo_work.KKT[114]-cmo_work.L[44]*cmo_work.v[18]-cmo_work.L[45]*cmo_work.v[32]-cmo_work.L[46]*cmo_work.v[56];
  cmo_work.d[57] = cmo_work.v[57];
  if (cmo_work.d[57] > 0)
    cmo_work.d[57] = -cmo_settings.kkt_reg;
  else
    cmo_work.d[57] -= cmo_settings.kkt_reg;
  cmo_work.d_inv[57] = 1/cmo_work.d[57];
  cmo_work.L[48] = (cmo_work.KKT[115]-cmo_work.L[47]*cmo_work.v[56])*cmo_work.d_inv[57];
  cmo_work.v[56] = cmo_work.L[47]*cmo_work.d[56];
  cmo_work.v[57] = cmo_work.L[48]*cmo_work.d[57];
  cmo_work.v[58] = 0-cmo_work.L[47]*cmo_work.v[56]-cmo_work.L[48]*cmo_work.v[57];
  cmo_work.d[58] = cmo_work.v[58];
  if (cmo_work.d[58] < 0)
    cmo_work.d[58] = cmo_settings.kkt_reg;
  else
    cmo_work.d[58] += cmo_settings.kkt_reg;
  cmo_work.d_inv[58] = 1/cmo_work.d[58];
  cmo_work.L[91] = (cmo_work.KKT[116])*cmo_work.d_inv[58];
  cmo_work.L[104] = (cmo_work.KKT[117])*cmo_work.d_inv[58];
  cmo_work.v[7] = cmo_work.L[49]*cmo_work.d[7];
  cmo_work.v[33] = cmo_work.L[50]*cmo_work.d[33];
  cmo_work.v[59] = cmo_work.KKT[118]-cmo_work.L[49]*cmo_work.v[7]-cmo_work.L[50]*cmo_work.v[33];
  cmo_work.d[59] = cmo_work.v[59];
  if (cmo_work.d[59] > 0)
    cmo_work.d[59] = -cmo_settings.kkt_reg;
  else
    cmo_work.d[59] -= cmo_settings.kkt_reg;
  cmo_work.d_inv[59] = 1/cmo_work.d[59];
  cmo_work.L[53] = (-cmo_work.L[52]*cmo_work.v[33])*cmo_work.d_inv[59];
  cmo_work.L[54] = (cmo_work.KKT[119])*cmo_work.d_inv[59];
  cmo_work.v[19] = cmo_work.L[51]*cmo_work.d[19];
  cmo_work.v[33] = cmo_work.L[52]*cmo_work.d[33];
  cmo_work.v[59] = cmo_work.L[53]*cmo_work.d[59];
  cmo_work.v[60] = cmo_work.KKT[120]-cmo_work.L[51]*cmo_work.v[19]-cmo_work.L[52]*cmo_work.v[33]-cmo_work.L[53]*cmo_work.v[59];
  cmo_work.d[60] = cmo_work.v[60];
  if (cmo_work.d[60] > 0)
    cmo_work.d[60] = -cmo_settings.kkt_reg;
  else
    cmo_work.d[60] -= cmo_settings.kkt_reg;
  cmo_work.d_inv[60] = 1/cmo_work.d[60];
  cmo_work.L[55] = (cmo_work.KKT[121]-cmo_work.L[54]*cmo_work.v[59])*cmo_work.d_inv[60];
  cmo_work.v[59] = cmo_work.L[54]*cmo_work.d[59];
  cmo_work.v[60] = cmo_work.L[55]*cmo_work.d[60];
  cmo_work.v[61] = 0-cmo_work.L[54]*cmo_work.v[59]-cmo_work.L[55]*cmo_work.v[60];
  cmo_work.d[61] = cmo_work.v[61];
  if (cmo_work.d[61] < 0)
    cmo_work.d[61] = cmo_settings.kkt_reg;
  else
    cmo_work.d[61] += cmo_settings.kkt_reg;
  cmo_work.d_inv[61] = 1/cmo_work.d[61];
  cmo_work.L[92] = (cmo_work.KKT[122])*cmo_work.d_inv[61];
  cmo_work.L[105] = (cmo_work.KKT[123])*cmo_work.d_inv[61];
  cmo_work.v[8] = cmo_work.L[56]*cmo_work.d[8];
  cmo_work.v[34] = cmo_work.L[57]*cmo_work.d[34];
  cmo_work.v[62] = cmo_work.KKT[124]-cmo_work.L[56]*cmo_work.v[8]-cmo_work.L[57]*cmo_work.v[34];
  cmo_work.d[62] = cmo_work.v[62];
  if (cmo_work.d[62] > 0)
    cmo_work.d[62] = -cmo_settings.kkt_reg;
  else
    cmo_work.d[62] -= cmo_settings.kkt_reg;
  cmo_work.d_inv[62] = 1/cmo_work.d[62];
  cmo_work.L[60] = (-cmo_work.L[59]*cmo_work.v[34])*cmo_work.d_inv[62];
  cmo_work.L[61] = (cmo_work.KKT[125])*cmo_work.d_inv[62];
  cmo_work.v[20] = cmo_work.L[58]*cmo_work.d[20];
  cmo_work.v[34] = cmo_work.L[59]*cmo_work.d[34];
  cmo_work.v[62] = cmo_work.L[60]*cmo_work.d[62];
  cmo_work.v[63] = cmo_work.KKT[126]-cmo_work.L[58]*cmo_work.v[20]-cmo_work.L[59]*cmo_work.v[34]-cmo_work.L[60]*cmo_work.v[62];
  cmo_work.d[63] = cmo_work.v[63];
  if (cmo_work.d[63] > 0)
    cmo_work.d[63] = -cmo_settings.kkt_reg;
  else
    cmo_work.d[63] -= cmo_settings.kkt_reg;
  cmo_work.d_inv[63] = 1/cmo_work.d[63];
  cmo_work.L[62] = (cmo_work.KKT[127]-cmo_work.L[61]*cmo_work.v[62])*cmo_work.d_inv[63];
  cmo_work.v[62] = cmo_work.L[61]*cmo_work.d[62];
  cmo_work.v[63] = cmo_work.L[62]*cmo_work.d[63];
  cmo_work.v[64] = 0-cmo_work.L[61]*cmo_work.v[62]-cmo_work.L[62]*cmo_work.v[63];
  cmo_work.d[64] = cmo_work.v[64];
  if (cmo_work.d[64] < 0)
    cmo_work.d[64] = cmo_settings.kkt_reg;
  else
    cmo_work.d[64] += cmo_settings.kkt_reg;
  cmo_work.d_inv[64] = 1/cmo_work.d[64];
  cmo_work.L[93] = (cmo_work.KKT[128])*cmo_work.d_inv[64];
  cmo_work.L[106] = (cmo_work.KKT[129])*cmo_work.d_inv[64];
  cmo_work.v[9] = cmo_work.L[63]*cmo_work.d[9];
  cmo_work.v[35] = cmo_work.L[64]*cmo_work.d[35];
  cmo_work.v[65] = cmo_work.KKT[130]-cmo_work.L[63]*cmo_work.v[9]-cmo_work.L[64]*cmo_work.v[35];
  cmo_work.d[65] = cmo_work.v[65];
  if (cmo_work.d[65] > 0)
    cmo_work.d[65] = -cmo_settings.kkt_reg;
  else
    cmo_work.d[65] -= cmo_settings.kkt_reg;
  cmo_work.d_inv[65] = 1/cmo_work.d[65];
  cmo_work.L[67] = (-cmo_work.L[66]*cmo_work.v[35])*cmo_work.d_inv[65];
  cmo_work.L[68] = (cmo_work.KKT[131])*cmo_work.d_inv[65];
  cmo_work.v[21] = cmo_work.L[65]*cmo_work.d[21];
  cmo_work.v[35] = cmo_work.L[66]*cmo_work.d[35];
  cmo_work.v[65] = cmo_work.L[67]*cmo_work.d[65];
  cmo_work.v[66] = cmo_work.KKT[132]-cmo_work.L[65]*cmo_work.v[21]-cmo_work.L[66]*cmo_work.v[35]-cmo_work.L[67]*cmo_work.v[65];
  cmo_work.d[66] = cmo_work.v[66];
  if (cmo_work.d[66] > 0)
    cmo_work.d[66] = -cmo_settings.kkt_reg;
  else
    cmo_work.d[66] -= cmo_settings.kkt_reg;
  cmo_work.d_inv[66] = 1/cmo_work.d[66];
  cmo_work.L[69] = (cmo_work.KKT[133]-cmo_work.L[68]*cmo_work.v[65])*cmo_work.d_inv[66];
  cmo_work.v[65] = cmo_work.L[68]*cmo_work.d[65];
  cmo_work.v[66] = cmo_work.L[69]*cmo_work.d[66];
  cmo_work.v[67] = 0-cmo_work.L[68]*cmo_work.v[65]-cmo_work.L[69]*cmo_work.v[66];
  cmo_work.d[67] = cmo_work.v[67];
  if (cmo_work.d[67] < 0)
    cmo_work.d[67] = cmo_settings.kkt_reg;
  else
    cmo_work.d[67] += cmo_settings.kkt_reg;
  cmo_work.d_inv[67] = 1/cmo_work.d[67];
  cmo_work.L[94] = (cmo_work.KKT[134])*cmo_work.d_inv[67];
  cmo_work.L[107] = (cmo_work.KKT[135])*cmo_work.d_inv[67];
  cmo_work.v[10] = cmo_work.L[70]*cmo_work.d[10];
  cmo_work.v[36] = cmo_work.L[71]*cmo_work.d[36];
  cmo_work.v[68] = cmo_work.KKT[136]-cmo_work.L[70]*cmo_work.v[10]-cmo_work.L[71]*cmo_work.v[36];
  cmo_work.d[68] = cmo_work.v[68];
  if (cmo_work.d[68] > 0)
    cmo_work.d[68] = -cmo_settings.kkt_reg;
  else
    cmo_work.d[68] -= cmo_settings.kkt_reg;
  cmo_work.d_inv[68] = 1/cmo_work.d[68];
  cmo_work.L[74] = (-cmo_work.L[73]*cmo_work.v[36])*cmo_work.d_inv[68];
  cmo_work.L[75] = (cmo_work.KKT[137])*cmo_work.d_inv[68];
  cmo_work.v[22] = cmo_work.L[72]*cmo_work.d[22];
  cmo_work.v[36] = cmo_work.L[73]*cmo_work.d[36];
  cmo_work.v[68] = cmo_work.L[74]*cmo_work.d[68];
  cmo_work.v[69] = cmo_work.KKT[138]-cmo_work.L[72]*cmo_work.v[22]-cmo_work.L[73]*cmo_work.v[36]-cmo_work.L[74]*cmo_work.v[68];
  cmo_work.d[69] = cmo_work.v[69];
  if (cmo_work.d[69] > 0)
    cmo_work.d[69] = -cmo_settings.kkt_reg;
  else
    cmo_work.d[69] -= cmo_settings.kkt_reg;
  cmo_work.d_inv[69] = 1/cmo_work.d[69];
  cmo_work.L[76] = (cmo_work.KKT[139]-cmo_work.L[75]*cmo_work.v[68])*cmo_work.d_inv[69];
  cmo_work.v[68] = cmo_work.L[75]*cmo_work.d[68];
  cmo_work.v[69] = cmo_work.L[76]*cmo_work.d[69];
  cmo_work.v[70] = 0-cmo_work.L[75]*cmo_work.v[68]-cmo_work.L[76]*cmo_work.v[69];
  cmo_work.d[70] = cmo_work.v[70];
  if (cmo_work.d[70] < 0)
    cmo_work.d[70] = cmo_settings.kkt_reg;
  else
    cmo_work.d[70] += cmo_settings.kkt_reg;
  cmo_work.d_inv[70] = 1/cmo_work.d[70];
  cmo_work.L[95] = (cmo_work.KKT[140])*cmo_work.d_inv[70];
  cmo_work.L[108] = (cmo_work.KKT[141])*cmo_work.d_inv[70];
  cmo_work.v[11] = cmo_work.L[77]*cmo_work.d[11];
  cmo_work.v[37] = cmo_work.L[78]*cmo_work.d[37];
  cmo_work.v[71] = cmo_work.KKT[142]-cmo_work.L[77]*cmo_work.v[11]-cmo_work.L[78]*cmo_work.v[37];
  cmo_work.d[71] = cmo_work.v[71];
  if (cmo_work.d[71] > 0)
    cmo_work.d[71] = -cmo_settings.kkt_reg;
  else
    cmo_work.d[71] -= cmo_settings.kkt_reg;
  cmo_work.d_inv[71] = 1/cmo_work.d[71];
  cmo_work.L[81] = (-cmo_work.L[80]*cmo_work.v[37])*cmo_work.d_inv[71];
  cmo_work.L[82] = (cmo_work.KKT[143])*cmo_work.d_inv[71];
  cmo_work.v[23] = cmo_work.L[79]*cmo_work.d[23];
  cmo_work.v[37] = cmo_work.L[80]*cmo_work.d[37];
  cmo_work.v[71] = cmo_work.L[81]*cmo_work.d[71];
  cmo_work.v[72] = cmo_work.KKT[144]-cmo_work.L[79]*cmo_work.v[23]-cmo_work.L[80]*cmo_work.v[37]-cmo_work.L[81]*cmo_work.v[71];
  cmo_work.d[72] = cmo_work.v[72];
  if (cmo_work.d[72] > 0)
    cmo_work.d[72] = -cmo_settings.kkt_reg;
  else
    cmo_work.d[72] -= cmo_settings.kkt_reg;
  cmo_work.d_inv[72] = 1/cmo_work.d[72];
  cmo_work.L[83] = (cmo_work.KKT[145]-cmo_work.L[82]*cmo_work.v[71])*cmo_work.d_inv[72];
  cmo_work.v[71] = cmo_work.L[82]*cmo_work.d[71];
  cmo_work.v[72] = cmo_work.L[83]*cmo_work.d[72];
  cmo_work.v[73] = 0-cmo_work.L[82]*cmo_work.v[71]-cmo_work.L[83]*cmo_work.v[72];
  cmo_work.d[73] = cmo_work.v[73];
  if (cmo_work.d[73] < 0)
    cmo_work.d[73] = cmo_settings.kkt_reg;
  else
    cmo_work.d[73] += cmo_settings.kkt_reg;
  cmo_work.d_inv[73] = 1/cmo_work.d[73];
  cmo_work.L[96] = (cmo_work.KKT[146])*cmo_work.d_inv[73];
  cmo_work.L[109] = (cmo_work.KKT[147])*cmo_work.d_inv[73];
  cmo_work.v[24] = cmo_work.L[84]*cmo_work.d[24];
  cmo_work.v[40] = cmo_work.L[85]*cmo_work.d[40];
  cmo_work.v[43] = cmo_work.L[86]*cmo_work.d[43];
  cmo_work.v[46] = cmo_work.L[87]*cmo_work.d[46];
  cmo_work.v[49] = cmo_work.L[88]*cmo_work.d[49];
  cmo_work.v[52] = cmo_work.L[89]*cmo_work.d[52];
  cmo_work.v[55] = cmo_work.L[90]*cmo_work.d[55];
  cmo_work.v[58] = cmo_work.L[91]*cmo_work.d[58];
  cmo_work.v[61] = cmo_work.L[92]*cmo_work.d[61];
  cmo_work.v[64] = cmo_work.L[93]*cmo_work.d[64];
  cmo_work.v[67] = cmo_work.L[94]*cmo_work.d[67];
  cmo_work.v[70] = cmo_work.L[95]*cmo_work.d[70];
  cmo_work.v[73] = cmo_work.L[96]*cmo_work.d[73];
  cmo_work.v[74] = 0-cmo_work.L[84]*cmo_work.v[24]-cmo_work.L[85]*cmo_work.v[40]-cmo_work.L[86]*cmo_work.v[43]-cmo_work.L[87]*cmo_work.v[46]-cmo_work.L[88]*cmo_work.v[49]-cmo_work.L[89]*cmo_work.v[52]-cmo_work.L[90]*cmo_work.v[55]-cmo_work.L[91]*cmo_work.v[58]-cmo_work.L[92]*cmo_work.v[61]-cmo_work.L[93]*cmo_work.v[64]-cmo_work.L[94]*cmo_work.v[67]-cmo_work.L[95]*cmo_work.v[70]-cmo_work.L[96]*cmo_work.v[73];
  cmo_work.d[74] = cmo_work.v[74];
  if (cmo_work.d[74] > 0)
    cmo_work.d[74] = -cmo_settings.kkt_reg;
  else
    cmo_work.d[74] -= cmo_settings.kkt_reg;
  cmo_work.d_inv[74] = 1/cmo_work.d[74];
  cmo_work.L[110] = (-cmo_work.L[98]*cmo_work.v[40]-cmo_work.L[99]*cmo_work.v[43]-cmo_work.L[100]*cmo_work.v[46]-cmo_work.L[101]*cmo_work.v[49]-cmo_work.L[102]*cmo_work.v[52]-cmo_work.L[103]*cmo_work.v[55]-cmo_work.L[104]*cmo_work.v[58]-cmo_work.L[105]*cmo_work.v[61]-cmo_work.L[106]*cmo_work.v[64]-cmo_work.L[107]*cmo_work.v[67]-cmo_work.L[108]*cmo_work.v[70]-cmo_work.L[109]*cmo_work.v[73])*cmo_work.d_inv[74];
  cmo_work.v[25] = cmo_work.L[97]*cmo_work.d[25];
  cmo_work.v[40] = cmo_work.L[98]*cmo_work.d[40];
  cmo_work.v[43] = cmo_work.L[99]*cmo_work.d[43];
  cmo_work.v[46] = cmo_work.L[100]*cmo_work.d[46];
  cmo_work.v[49] = cmo_work.L[101]*cmo_work.d[49];
  cmo_work.v[52] = cmo_work.L[102]*cmo_work.d[52];
  cmo_work.v[55] = cmo_work.L[103]*cmo_work.d[55];
  cmo_work.v[58] = cmo_work.L[104]*cmo_work.d[58];
  cmo_work.v[61] = cmo_work.L[105]*cmo_work.d[61];
  cmo_work.v[64] = cmo_work.L[106]*cmo_work.d[64];
  cmo_work.v[67] = cmo_work.L[107]*cmo_work.d[67];
  cmo_work.v[70] = cmo_work.L[108]*cmo_work.d[70];
  cmo_work.v[73] = cmo_work.L[109]*cmo_work.d[73];
  cmo_work.v[74] = cmo_work.L[110]*cmo_work.d[74];
  cmo_work.v[75] = 0-cmo_work.L[97]*cmo_work.v[25]-cmo_work.L[98]*cmo_work.v[40]-cmo_work.L[99]*cmo_work.v[43]-cmo_work.L[100]*cmo_work.v[46]-cmo_work.L[101]*cmo_work.v[49]-cmo_work.L[102]*cmo_work.v[52]-cmo_work.L[103]*cmo_work.v[55]-cmo_work.L[104]*cmo_work.v[58]-cmo_work.L[105]*cmo_work.v[61]-cmo_work.L[106]*cmo_work.v[64]-cmo_work.L[107]*cmo_work.v[67]-cmo_work.L[108]*cmo_work.v[70]-cmo_work.L[109]*cmo_work.v[73]-cmo_work.L[110]*cmo_work.v[74];
  cmo_work.d[75] = cmo_work.v[75];
  if (cmo_work.d[75] > 0)
    cmo_work.d[75] = -cmo_settings.kkt_reg;
  else
    cmo_work.d[75] -= cmo_settings.kkt_reg;
  cmo_work.d_inv[75] = 1/cmo_work.d[75];
#ifndef ZERO_LIBRARY_MODE
  if (cmo_settings.debug) {
    printf("Squared Frobenius for factorization is %.8g.\n", cmo_check_factorization());
  }
#endif
}
double cmo_check_factorization(void) {
  /* Returns the squared Frobenius norm of A - L*D*L'. */
  double temp, residual;
  /* Only check the lower triangle. */
  residual = 0;
  temp = cmo_work.KKT[48]-1*cmo_work.d[24]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[50]-1*cmo_work.d[25]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[0]-1*cmo_work.d[0]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[2]-1*cmo_work.d[1]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[4]-1*cmo_work.d[2]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[6]-1*cmo_work.d[3]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[8]-1*cmo_work.d[4]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[10]-1*cmo_work.d[5]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[12]-1*cmo_work.d[6]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[14]-1*cmo_work.d[7]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[16]-1*cmo_work.d[8]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[18]-1*cmo_work.d[9]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[20]-1*cmo_work.d[10]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[22]-1*cmo_work.d[11]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[24]-1*cmo_work.d[12]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[26]-1*cmo_work.d[13]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[28]-1*cmo_work.d[14]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[30]-1*cmo_work.d[15]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[32]-1*cmo_work.d[16]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[34]-1*cmo_work.d[17]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[36]-1*cmo_work.d[18]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[38]-1*cmo_work.d[19]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[40]-1*cmo_work.d[20]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[42]-1*cmo_work.d[21]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[44]-1*cmo_work.d[22]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[46]-1*cmo_work.d[23]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[1]-cmo_work.L[0]*cmo_work.d[0]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[3]-cmo_work.L[7]*cmo_work.d[1]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[5]-cmo_work.L[14]*cmo_work.d[2]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[7]-cmo_work.L[21]*cmo_work.d[3]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[9]-cmo_work.L[28]*cmo_work.d[4]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[11]-cmo_work.L[35]*cmo_work.d[5]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[13]-cmo_work.L[42]*cmo_work.d[6]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[15]-cmo_work.L[49]*cmo_work.d[7]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[17]-cmo_work.L[56]*cmo_work.d[8]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[19]-cmo_work.L[63]*cmo_work.d[9]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[21]-cmo_work.L[70]*cmo_work.d[10]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[23]-cmo_work.L[77]*cmo_work.d[11]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[25]-cmo_work.L[2]*cmo_work.d[12]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[27]-cmo_work.L[9]*cmo_work.d[13]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[29]-cmo_work.L[16]*cmo_work.d[14]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[31]-cmo_work.L[23]*cmo_work.d[15]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[33]-cmo_work.L[30]*cmo_work.d[16]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[35]-cmo_work.L[37]*cmo_work.d[17]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[37]-cmo_work.L[44]*cmo_work.d[18]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[39]-cmo_work.L[51]*cmo_work.d[19]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[41]-cmo_work.L[58]*cmo_work.d[20]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[43]-cmo_work.L[65]*cmo_work.d[21]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[45]-cmo_work.L[72]*cmo_work.d[22]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[47]-cmo_work.L[79]*cmo_work.d[23]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[76]-cmo_work.L[0]*cmo_work.d[0]*cmo_work.L[0]-1*cmo_work.d[38]*1-cmo_work.L[1]*cmo_work.d[26]*cmo_work.L[1];
  residual += temp*temp;
  temp = cmo_work.KKT[82]-cmo_work.L[7]*cmo_work.d[1]*cmo_work.L[7]-1*cmo_work.d[41]*1-cmo_work.L[8]*cmo_work.d[27]*cmo_work.L[8];
  residual += temp*temp;
  temp = cmo_work.KKT[88]-cmo_work.L[14]*cmo_work.d[2]*cmo_work.L[14]-1*cmo_work.d[44]*1-cmo_work.L[15]*cmo_work.d[28]*cmo_work.L[15];
  residual += temp*temp;
  temp = cmo_work.KKT[94]-cmo_work.L[21]*cmo_work.d[3]*cmo_work.L[21]-1*cmo_work.d[47]*1-cmo_work.L[22]*cmo_work.d[29]*cmo_work.L[22];
  residual += temp*temp;
  temp = cmo_work.KKT[100]-cmo_work.L[28]*cmo_work.d[4]*cmo_work.L[28]-1*cmo_work.d[50]*1-cmo_work.L[29]*cmo_work.d[30]*cmo_work.L[29];
  residual += temp*temp;
  temp = cmo_work.KKT[106]-cmo_work.L[35]*cmo_work.d[5]*cmo_work.L[35]-1*cmo_work.d[53]*1-cmo_work.L[36]*cmo_work.d[31]*cmo_work.L[36];
  residual += temp*temp;
  temp = cmo_work.KKT[112]-cmo_work.L[42]*cmo_work.d[6]*cmo_work.L[42]-1*cmo_work.d[56]*1-cmo_work.L[43]*cmo_work.d[32]*cmo_work.L[43];
  residual += temp*temp;
  temp = cmo_work.KKT[118]-cmo_work.L[49]*cmo_work.d[7]*cmo_work.L[49]-1*cmo_work.d[59]*1-cmo_work.L[50]*cmo_work.d[33]*cmo_work.L[50];
  residual += temp*temp;
  temp = cmo_work.KKT[124]-cmo_work.L[56]*cmo_work.d[8]*cmo_work.L[56]-1*cmo_work.d[62]*1-cmo_work.L[57]*cmo_work.d[34]*cmo_work.L[57];
  residual += temp*temp;
  temp = cmo_work.KKT[130]-cmo_work.L[63]*cmo_work.d[9]*cmo_work.L[63]-1*cmo_work.d[65]*1-cmo_work.L[64]*cmo_work.d[35]*cmo_work.L[64];
  residual += temp*temp;
  temp = cmo_work.KKT[136]-cmo_work.L[70]*cmo_work.d[10]*cmo_work.L[70]-1*cmo_work.d[68]*1-cmo_work.L[71]*cmo_work.d[36]*cmo_work.L[71];
  residual += temp*temp;
  temp = cmo_work.KKT[142]-cmo_work.L[77]*cmo_work.d[11]*cmo_work.L[77]-1*cmo_work.d[71]*1-cmo_work.L[78]*cmo_work.d[37]*cmo_work.L[78];
  residual += temp*temp;
  temp = cmo_work.KKT[78]-cmo_work.L[2]*cmo_work.d[12]*cmo_work.L[2]-1*cmo_work.d[39]*1-cmo_work.L[3]*cmo_work.d[26]*cmo_work.L[3]-cmo_work.L[4]*cmo_work.d[38]*cmo_work.L[4];
  residual += temp*temp;
  temp = cmo_work.KKT[84]-cmo_work.L[9]*cmo_work.d[13]*cmo_work.L[9]-1*cmo_work.d[42]*1-cmo_work.L[10]*cmo_work.d[27]*cmo_work.L[10]-cmo_work.L[11]*cmo_work.d[41]*cmo_work.L[11];
  residual += temp*temp;
  temp = cmo_work.KKT[90]-cmo_work.L[16]*cmo_work.d[14]*cmo_work.L[16]-1*cmo_work.d[45]*1-cmo_work.L[17]*cmo_work.d[28]*cmo_work.L[17]-cmo_work.L[18]*cmo_work.d[44]*cmo_work.L[18];
  residual += temp*temp;
  temp = cmo_work.KKT[96]-cmo_work.L[23]*cmo_work.d[15]*cmo_work.L[23]-1*cmo_work.d[48]*1-cmo_work.L[24]*cmo_work.d[29]*cmo_work.L[24]-cmo_work.L[25]*cmo_work.d[47]*cmo_work.L[25];
  residual += temp*temp;
  temp = cmo_work.KKT[102]-cmo_work.L[30]*cmo_work.d[16]*cmo_work.L[30]-1*cmo_work.d[51]*1-cmo_work.L[31]*cmo_work.d[30]*cmo_work.L[31]-cmo_work.L[32]*cmo_work.d[50]*cmo_work.L[32];
  residual += temp*temp;
  temp = cmo_work.KKT[108]-cmo_work.L[37]*cmo_work.d[17]*cmo_work.L[37]-1*cmo_work.d[54]*1-cmo_work.L[38]*cmo_work.d[31]*cmo_work.L[38]-cmo_work.L[39]*cmo_work.d[53]*cmo_work.L[39];
  residual += temp*temp;
  temp = cmo_work.KKT[114]-cmo_work.L[44]*cmo_work.d[18]*cmo_work.L[44]-1*cmo_work.d[57]*1-cmo_work.L[45]*cmo_work.d[32]*cmo_work.L[45]-cmo_work.L[46]*cmo_work.d[56]*cmo_work.L[46];
  residual += temp*temp;
  temp = cmo_work.KKT[120]-cmo_work.L[51]*cmo_work.d[19]*cmo_work.L[51]-1*cmo_work.d[60]*1-cmo_work.L[52]*cmo_work.d[33]*cmo_work.L[52]-cmo_work.L[53]*cmo_work.d[59]*cmo_work.L[53];
  residual += temp*temp;
  temp = cmo_work.KKT[126]-cmo_work.L[58]*cmo_work.d[20]*cmo_work.L[58]-1*cmo_work.d[63]*1-cmo_work.L[59]*cmo_work.d[34]*cmo_work.L[59]-cmo_work.L[60]*cmo_work.d[62]*cmo_work.L[60];
  residual += temp*temp;
  temp = cmo_work.KKT[132]-cmo_work.L[65]*cmo_work.d[21]*cmo_work.L[65]-1*cmo_work.d[66]*1-cmo_work.L[66]*cmo_work.d[35]*cmo_work.L[66]-cmo_work.L[67]*cmo_work.d[65]*cmo_work.L[67];
  residual += temp*temp;
  temp = cmo_work.KKT[138]-cmo_work.L[72]*cmo_work.d[22]*cmo_work.L[72]-1*cmo_work.d[69]*1-cmo_work.L[73]*cmo_work.d[36]*cmo_work.L[73]-cmo_work.L[74]*cmo_work.d[68]*cmo_work.L[74];
  residual += temp*temp;
  temp = cmo_work.KKT[144]-cmo_work.L[79]*cmo_work.d[23]*cmo_work.L[79]-1*cmo_work.d[72]*1-cmo_work.L[80]*cmo_work.d[37]*cmo_work.L[80]-cmo_work.L[81]*cmo_work.d[71]*cmo_work.L[81];
  residual += temp*temp;
  temp = cmo_work.KKT[52]-cmo_work.L[1]*cmo_work.d[26]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[54]-cmo_work.L[8]*cmo_work.d[27]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[56]-cmo_work.L[15]*cmo_work.d[28]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[58]-cmo_work.L[22]*cmo_work.d[29]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[60]-cmo_work.L[29]*cmo_work.d[30]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[62]-cmo_work.L[36]*cmo_work.d[31]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[64]-cmo_work.L[43]*cmo_work.d[32]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[66]-cmo_work.L[50]*cmo_work.d[33]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[68]-cmo_work.L[57]*cmo_work.d[34]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[70]-cmo_work.L[64]*cmo_work.d[35]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[72]-cmo_work.L[71]*cmo_work.d[36]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[74]-cmo_work.L[78]*cmo_work.d[37]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[77]-1*cmo_work.d[38]*cmo_work.L[5];
  residual += temp*temp;
  temp = cmo_work.KKT[83]-1*cmo_work.d[41]*cmo_work.L[12];
  residual += temp*temp;
  temp = cmo_work.KKT[89]-1*cmo_work.d[44]*cmo_work.L[19];
  residual += temp*temp;
  temp = cmo_work.KKT[95]-1*cmo_work.d[47]*cmo_work.L[26];
  residual += temp*temp;
  temp = cmo_work.KKT[101]-1*cmo_work.d[50]*cmo_work.L[33];
  residual += temp*temp;
  temp = cmo_work.KKT[107]-1*cmo_work.d[53]*cmo_work.L[40];
  residual += temp*temp;
  temp = cmo_work.KKT[113]-1*cmo_work.d[56]*cmo_work.L[47];
  residual += temp*temp;
  temp = cmo_work.KKT[119]-1*cmo_work.d[59]*cmo_work.L[54];
  residual += temp*temp;
  temp = cmo_work.KKT[125]-1*cmo_work.d[62]*cmo_work.L[61];
  residual += temp*temp;
  temp = cmo_work.KKT[131]-1*cmo_work.d[65]*cmo_work.L[68];
  residual += temp*temp;
  temp = cmo_work.KKT[137]-1*cmo_work.d[68]*cmo_work.L[75];
  residual += temp*temp;
  temp = cmo_work.KKT[143]-1*cmo_work.d[71]*cmo_work.L[82];
  residual += temp*temp;
  temp = cmo_work.KKT[53]-cmo_work.L[3]*cmo_work.d[26]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[55]-cmo_work.L[10]*cmo_work.d[27]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[57]-cmo_work.L[17]*cmo_work.d[28]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[59]-cmo_work.L[24]*cmo_work.d[29]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[61]-cmo_work.L[31]*cmo_work.d[30]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[63]-cmo_work.L[38]*cmo_work.d[31]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[65]-cmo_work.L[45]*cmo_work.d[32]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[67]-cmo_work.L[52]*cmo_work.d[33]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[69]-cmo_work.L[59]*cmo_work.d[34]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[71]-cmo_work.L[66]*cmo_work.d[35]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[73]-cmo_work.L[73]*cmo_work.d[36]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[75]-cmo_work.L[80]*cmo_work.d[37]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[79]-1*cmo_work.d[39]*cmo_work.L[6]-cmo_work.L[4]*cmo_work.d[38]*cmo_work.L[5];
  residual += temp*temp;
  temp = cmo_work.KKT[85]-1*cmo_work.d[42]*cmo_work.L[13]-cmo_work.L[11]*cmo_work.d[41]*cmo_work.L[12];
  residual += temp*temp;
  temp = cmo_work.KKT[91]-1*cmo_work.d[45]*cmo_work.L[20]-cmo_work.L[18]*cmo_work.d[44]*cmo_work.L[19];
  residual += temp*temp;
  temp = cmo_work.KKT[97]-1*cmo_work.d[48]*cmo_work.L[27]-cmo_work.L[25]*cmo_work.d[47]*cmo_work.L[26];
  residual += temp*temp;
  temp = cmo_work.KKT[103]-1*cmo_work.d[51]*cmo_work.L[34]-cmo_work.L[32]*cmo_work.d[50]*cmo_work.L[33];
  residual += temp*temp;
  temp = cmo_work.KKT[109]-1*cmo_work.d[54]*cmo_work.L[41]-cmo_work.L[39]*cmo_work.d[53]*cmo_work.L[40];
  residual += temp*temp;
  temp = cmo_work.KKT[115]-1*cmo_work.d[57]*cmo_work.L[48]-cmo_work.L[46]*cmo_work.d[56]*cmo_work.L[47];
  residual += temp*temp;
  temp = cmo_work.KKT[121]-1*cmo_work.d[60]*cmo_work.L[55]-cmo_work.L[53]*cmo_work.d[59]*cmo_work.L[54];
  residual += temp*temp;
  temp = cmo_work.KKT[127]-1*cmo_work.d[63]*cmo_work.L[62]-cmo_work.L[60]*cmo_work.d[62]*cmo_work.L[61];
  residual += temp*temp;
  temp = cmo_work.KKT[133]-1*cmo_work.d[66]*cmo_work.L[69]-cmo_work.L[67]*cmo_work.d[65]*cmo_work.L[68];
  residual += temp*temp;
  temp = cmo_work.KKT[139]-1*cmo_work.d[69]*cmo_work.L[76]-cmo_work.L[74]*cmo_work.d[68]*cmo_work.L[75];
  residual += temp*temp;
  temp = cmo_work.KKT[145]-1*cmo_work.d[72]*cmo_work.L[83]-cmo_work.L[81]*cmo_work.d[71]*cmo_work.L[82];
  residual += temp*temp;
  temp = cmo_work.KKT[49]-cmo_work.L[84]*cmo_work.d[24]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[80]-cmo_work.L[85]*cmo_work.d[40]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[86]-cmo_work.L[86]*cmo_work.d[43]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[92]-cmo_work.L[87]*cmo_work.d[46]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[98]-cmo_work.L[88]*cmo_work.d[49]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[104]-cmo_work.L[89]*cmo_work.d[52]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[110]-cmo_work.L[90]*cmo_work.d[55]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[116]-cmo_work.L[91]*cmo_work.d[58]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[122]-cmo_work.L[92]*cmo_work.d[61]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[128]-cmo_work.L[93]*cmo_work.d[64]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[134]-cmo_work.L[94]*cmo_work.d[67]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[140]-cmo_work.L[95]*cmo_work.d[70]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[146]-cmo_work.L[96]*cmo_work.d[73]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[51]-cmo_work.L[97]*cmo_work.d[25]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[81]-cmo_work.L[98]*cmo_work.d[40]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[87]-cmo_work.L[99]*cmo_work.d[43]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[93]-cmo_work.L[100]*cmo_work.d[46]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[99]-cmo_work.L[101]*cmo_work.d[49]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[105]-cmo_work.L[102]*cmo_work.d[52]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[111]-cmo_work.L[103]*cmo_work.d[55]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[117]-cmo_work.L[104]*cmo_work.d[58]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[123]-cmo_work.L[105]*cmo_work.d[61]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[129]-cmo_work.L[106]*cmo_work.d[64]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[135]-cmo_work.L[107]*cmo_work.d[67]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[141]-cmo_work.L[108]*cmo_work.d[70]*1;
  residual += temp*temp;
  temp = cmo_work.KKT[147]-cmo_work.L[109]*cmo_work.d[73]*1;
  residual += temp*temp;
  return residual;
}
void cmo_matrix_multiply(double *result, double *source) {
  /* Finds result = A*source. */
  result[0] = cmo_work.KKT[48]*source[0]+cmo_work.KKT[49]*source[74];
  result[1] = cmo_work.KKT[50]*source[1]+cmo_work.KKT[51]*source[75];
  result[2] = cmo_work.KKT[52]*source[50]+cmo_work.KKT[53]*source[62];
  result[3] = cmo_work.KKT[54]*source[51]+cmo_work.KKT[55]*source[63];
  result[4] = cmo_work.KKT[56]*source[52]+cmo_work.KKT[57]*source[64];
  result[5] = cmo_work.KKT[58]*source[53]+cmo_work.KKT[59]*source[65];
  result[6] = cmo_work.KKT[60]*source[54]+cmo_work.KKT[61]*source[66];
  result[7] = cmo_work.KKT[62]*source[55]+cmo_work.KKT[63]*source[67];
  result[8] = cmo_work.KKT[64]*source[56]+cmo_work.KKT[65]*source[68];
  result[9] = cmo_work.KKT[66]*source[57]+cmo_work.KKT[67]*source[69];
  result[10] = cmo_work.KKT[68]*source[58]+cmo_work.KKT[69]*source[70];
  result[11] = cmo_work.KKT[70]*source[59]+cmo_work.KKT[71]*source[71];
  result[12] = cmo_work.KKT[72]*source[60]+cmo_work.KKT[73]*source[72];
  result[13] = cmo_work.KKT[74]*source[61]+cmo_work.KKT[75]*source[73];
  result[14] = cmo_work.KKT[77]*source[50]+cmo_work.KKT[79]*source[62]+cmo_work.KKT[80]*source[74]+cmo_work.KKT[81]*source[75];
  result[15] = cmo_work.KKT[83]*source[51]+cmo_work.KKT[85]*source[63]+cmo_work.KKT[86]*source[74]+cmo_work.KKT[87]*source[75];
  result[16] = cmo_work.KKT[89]*source[52]+cmo_work.KKT[91]*source[64]+cmo_work.KKT[92]*source[74]+cmo_work.KKT[93]*source[75];
  result[17] = cmo_work.KKT[95]*source[53]+cmo_work.KKT[97]*source[65]+cmo_work.KKT[98]*source[74]+cmo_work.KKT[99]*source[75];
  result[18] = cmo_work.KKT[101]*source[54]+cmo_work.KKT[103]*source[66]+cmo_work.KKT[104]*source[74]+cmo_work.KKT[105]*source[75];
  result[19] = cmo_work.KKT[107]*source[55]+cmo_work.KKT[109]*source[67]+cmo_work.KKT[110]*source[74]+cmo_work.KKT[111]*source[75];
  result[20] = cmo_work.KKT[113]*source[56]+cmo_work.KKT[115]*source[68]+cmo_work.KKT[116]*source[74]+cmo_work.KKT[117]*source[75];
  result[21] = cmo_work.KKT[119]*source[57]+cmo_work.KKT[121]*source[69]+cmo_work.KKT[122]*source[74]+cmo_work.KKT[123]*source[75];
  result[22] = cmo_work.KKT[125]*source[58]+cmo_work.KKT[127]*source[70]+cmo_work.KKT[128]*source[74]+cmo_work.KKT[129]*source[75];
  result[23] = cmo_work.KKT[131]*source[59]+cmo_work.KKT[133]*source[71]+cmo_work.KKT[134]*source[74]+cmo_work.KKT[135]*source[75];
  result[24] = cmo_work.KKT[137]*source[60]+cmo_work.KKT[139]*source[72]+cmo_work.KKT[140]*source[74]+cmo_work.KKT[141]*source[75];
  result[25] = cmo_work.KKT[143]*source[61]+cmo_work.KKT[145]*source[73]+cmo_work.KKT[146]*source[74]+cmo_work.KKT[147]*source[75];
  result[26] = cmo_work.KKT[0]*source[26]+cmo_work.KKT[1]*source[50];
  result[27] = cmo_work.KKT[2]*source[27]+cmo_work.KKT[3]*source[51];
  result[28] = cmo_work.KKT[4]*source[28]+cmo_work.KKT[5]*source[52];
  result[29] = cmo_work.KKT[6]*source[29]+cmo_work.KKT[7]*source[53];
  result[30] = cmo_work.KKT[8]*source[30]+cmo_work.KKT[9]*source[54];
  result[31] = cmo_work.KKT[10]*source[31]+cmo_work.KKT[11]*source[55];
  result[32] = cmo_work.KKT[12]*source[32]+cmo_work.KKT[13]*source[56];
  result[33] = cmo_work.KKT[14]*source[33]+cmo_work.KKT[15]*source[57];
  result[34] = cmo_work.KKT[16]*source[34]+cmo_work.KKT[17]*source[58];
  result[35] = cmo_work.KKT[18]*source[35]+cmo_work.KKT[19]*source[59];
  result[36] = cmo_work.KKT[20]*source[36]+cmo_work.KKT[21]*source[60];
  result[37] = cmo_work.KKT[22]*source[37]+cmo_work.KKT[23]*source[61];
  result[38] = cmo_work.KKT[24]*source[38]+cmo_work.KKT[25]*source[62];
  result[39] = cmo_work.KKT[26]*source[39]+cmo_work.KKT[27]*source[63];
  result[40] = cmo_work.KKT[28]*source[40]+cmo_work.KKT[29]*source[64];
  result[41] = cmo_work.KKT[30]*source[41]+cmo_work.KKT[31]*source[65];
  result[42] = cmo_work.KKT[32]*source[42]+cmo_work.KKT[33]*source[66];
  result[43] = cmo_work.KKT[34]*source[43]+cmo_work.KKT[35]*source[67];
  result[44] = cmo_work.KKT[36]*source[44]+cmo_work.KKT[37]*source[68];
  result[45] = cmo_work.KKT[38]*source[45]+cmo_work.KKT[39]*source[69];
  result[46] = cmo_work.KKT[40]*source[46]+cmo_work.KKT[41]*source[70];
  result[47] = cmo_work.KKT[42]*source[47]+cmo_work.KKT[43]*source[71];
  result[48] = cmo_work.KKT[44]*source[48]+cmo_work.KKT[45]*source[72];
  result[49] = cmo_work.KKT[46]*source[49]+cmo_work.KKT[47]*source[73];
  result[50] = cmo_work.KKT[1]*source[26]+cmo_work.KKT[76]*source[50]+cmo_work.KKT[52]*source[2]+cmo_work.KKT[77]*source[14];
  result[51] = cmo_work.KKT[3]*source[27]+cmo_work.KKT[82]*source[51]+cmo_work.KKT[54]*source[3]+cmo_work.KKT[83]*source[15];
  result[52] = cmo_work.KKT[5]*source[28]+cmo_work.KKT[88]*source[52]+cmo_work.KKT[56]*source[4]+cmo_work.KKT[89]*source[16];
  result[53] = cmo_work.KKT[7]*source[29]+cmo_work.KKT[94]*source[53]+cmo_work.KKT[58]*source[5]+cmo_work.KKT[95]*source[17];
  result[54] = cmo_work.KKT[9]*source[30]+cmo_work.KKT[100]*source[54]+cmo_work.KKT[60]*source[6]+cmo_work.KKT[101]*source[18];
  result[55] = cmo_work.KKT[11]*source[31]+cmo_work.KKT[106]*source[55]+cmo_work.KKT[62]*source[7]+cmo_work.KKT[107]*source[19];
  result[56] = cmo_work.KKT[13]*source[32]+cmo_work.KKT[112]*source[56]+cmo_work.KKT[64]*source[8]+cmo_work.KKT[113]*source[20];
  result[57] = cmo_work.KKT[15]*source[33]+cmo_work.KKT[118]*source[57]+cmo_work.KKT[66]*source[9]+cmo_work.KKT[119]*source[21];
  result[58] = cmo_work.KKT[17]*source[34]+cmo_work.KKT[124]*source[58]+cmo_work.KKT[68]*source[10]+cmo_work.KKT[125]*source[22];
  result[59] = cmo_work.KKT[19]*source[35]+cmo_work.KKT[130]*source[59]+cmo_work.KKT[70]*source[11]+cmo_work.KKT[131]*source[23];
  result[60] = cmo_work.KKT[21]*source[36]+cmo_work.KKT[136]*source[60]+cmo_work.KKT[72]*source[12]+cmo_work.KKT[137]*source[24];
  result[61] = cmo_work.KKT[23]*source[37]+cmo_work.KKT[142]*source[61]+cmo_work.KKT[74]*source[13]+cmo_work.KKT[143]*source[25];
  result[62] = cmo_work.KKT[25]*source[38]+cmo_work.KKT[78]*source[62]+cmo_work.KKT[53]*source[2]+cmo_work.KKT[79]*source[14];
  result[63] = cmo_work.KKT[27]*source[39]+cmo_work.KKT[84]*source[63]+cmo_work.KKT[55]*source[3]+cmo_work.KKT[85]*source[15];
  result[64] = cmo_work.KKT[29]*source[40]+cmo_work.KKT[90]*source[64]+cmo_work.KKT[57]*source[4]+cmo_work.KKT[91]*source[16];
  result[65] = cmo_work.KKT[31]*source[41]+cmo_work.KKT[96]*source[65]+cmo_work.KKT[59]*source[5]+cmo_work.KKT[97]*source[17];
  result[66] = cmo_work.KKT[33]*source[42]+cmo_work.KKT[102]*source[66]+cmo_work.KKT[61]*source[6]+cmo_work.KKT[103]*source[18];
  result[67] = cmo_work.KKT[35]*source[43]+cmo_work.KKT[108]*source[67]+cmo_work.KKT[63]*source[7]+cmo_work.KKT[109]*source[19];
  result[68] = cmo_work.KKT[37]*source[44]+cmo_work.KKT[114]*source[68]+cmo_work.KKT[65]*source[8]+cmo_work.KKT[115]*source[20];
  result[69] = cmo_work.KKT[39]*source[45]+cmo_work.KKT[120]*source[69]+cmo_work.KKT[67]*source[9]+cmo_work.KKT[121]*source[21];
  result[70] = cmo_work.KKT[41]*source[46]+cmo_work.KKT[126]*source[70]+cmo_work.KKT[69]*source[10]+cmo_work.KKT[127]*source[22];
  result[71] = cmo_work.KKT[43]*source[47]+cmo_work.KKT[132]*source[71]+cmo_work.KKT[71]*source[11]+cmo_work.KKT[133]*source[23];
  result[72] = cmo_work.KKT[45]*source[48]+cmo_work.KKT[138]*source[72]+cmo_work.KKT[73]*source[12]+cmo_work.KKT[139]*source[24];
  result[73] = cmo_work.KKT[47]*source[49]+cmo_work.KKT[144]*source[73]+cmo_work.KKT[75]*source[13]+cmo_work.KKT[145]*source[25];
  result[74] = cmo_work.KKT[49]*source[0]+cmo_work.KKT[80]*source[14]+cmo_work.KKT[86]*source[15]+cmo_work.KKT[92]*source[16]+cmo_work.KKT[98]*source[17]+cmo_work.KKT[104]*source[18]+cmo_work.KKT[110]*source[19]+cmo_work.KKT[116]*source[20]+cmo_work.KKT[122]*source[21]+cmo_work.KKT[128]*source[22]+cmo_work.KKT[134]*source[23]+cmo_work.KKT[140]*source[24]+cmo_work.KKT[146]*source[25];
  result[75] = cmo_work.KKT[51]*source[1]+cmo_work.KKT[81]*source[14]+cmo_work.KKT[87]*source[15]+cmo_work.KKT[93]*source[16]+cmo_work.KKT[99]*source[17]+cmo_work.KKT[105]*source[18]+cmo_work.KKT[111]*source[19]+cmo_work.KKT[117]*source[20]+cmo_work.KKT[123]*source[21]+cmo_work.KKT[129]*source[22]+cmo_work.KKT[135]*source[23]+cmo_work.KKT[141]*source[24]+cmo_work.KKT[147]*source[25];
}
double cmo_check_residual(double *target, double *multiplicand) {
  /* Returns the squared 2-norm of lhs - A*rhs. */
  /* Reuses v to find the residual. */
  int i;
  double residual;
  residual = 0;
  cmo_matrix_multiply(cmo_work.v, multiplicand);
  for (i = 0; i < 26; i++) {
    residual += (target[i] - cmo_work.v[i])*(target[i] - cmo_work.v[i]);
  }
  return residual;
}
void cmo_fill_KKT(void) {
  cmo_work.KKT[48] = 2;
  cmo_work.KKT[50] = 2;
  cmo_work.KKT[0] = cmo_work.s_inv_z[0];
  cmo_work.KKT[2] = cmo_work.s_inv_z[1];
  cmo_work.KKT[4] = cmo_work.s_inv_z[2];
  cmo_work.KKT[6] = cmo_work.s_inv_z[3];
  cmo_work.KKT[8] = cmo_work.s_inv_z[4];
  cmo_work.KKT[10] = cmo_work.s_inv_z[5];
  cmo_work.KKT[12] = cmo_work.s_inv_z[6];
  cmo_work.KKT[14] = cmo_work.s_inv_z[7];
  cmo_work.KKT[16] = cmo_work.s_inv_z[8];
  cmo_work.KKT[18] = cmo_work.s_inv_z[9];
  cmo_work.KKT[20] = cmo_work.s_inv_z[10];
  cmo_work.KKT[22] = cmo_work.s_inv_z[11];
  cmo_work.KKT[24] = cmo_work.s_inv_z[12];
  cmo_work.KKT[26] = cmo_work.s_inv_z[13];
  cmo_work.KKT[28] = cmo_work.s_inv_z[14];
  cmo_work.KKT[30] = cmo_work.s_inv_z[15];
  cmo_work.KKT[32] = cmo_work.s_inv_z[16];
  cmo_work.KKT[34] = cmo_work.s_inv_z[17];
  cmo_work.KKT[36] = cmo_work.s_inv_z[18];
  cmo_work.KKT[38] = cmo_work.s_inv_z[19];
  cmo_work.KKT[40] = cmo_work.s_inv_z[20];
  cmo_work.KKT[42] = cmo_work.s_inv_z[21];
  cmo_work.KKT[44] = cmo_work.s_inv_z[22];
  cmo_work.KKT[46] = cmo_work.s_inv_z[23];
  cmo_work.KKT[1] = 1;
  cmo_work.KKT[3] = 1;
  cmo_work.KKT[5] = 1;
  cmo_work.KKT[7] = 1;
  cmo_work.KKT[9] = 1;
  cmo_work.KKT[11] = 1;
  cmo_work.KKT[13] = 1;
  cmo_work.KKT[15] = 1;
  cmo_work.KKT[17] = 1;
  cmo_work.KKT[19] = 1;
  cmo_work.KKT[21] = 1;
  cmo_work.KKT[23] = 1;
  cmo_work.KKT[25] = 1;
  cmo_work.KKT[27] = 1;
  cmo_work.KKT[29] = 1;
  cmo_work.KKT[31] = 1;
  cmo_work.KKT[33] = 1;
  cmo_work.KKT[35] = 1;
  cmo_work.KKT[37] = 1;
  cmo_work.KKT[39] = 1;
  cmo_work.KKT[41] = 1;
  cmo_work.KKT[43] = 1;
  cmo_work.KKT[45] = 1;
  cmo_work.KKT[47] = 1;
  cmo_work.KKT[76] = cmo_work.block_33[0];
  cmo_work.KKT[82] = cmo_work.block_33[0];
  cmo_work.KKT[88] = cmo_work.block_33[0];
  cmo_work.KKT[94] = cmo_work.block_33[0];
  cmo_work.KKT[100] = cmo_work.block_33[0];
  cmo_work.KKT[106] = cmo_work.block_33[0];
  cmo_work.KKT[112] = cmo_work.block_33[0];
  cmo_work.KKT[118] = cmo_work.block_33[0];
  cmo_work.KKT[124] = cmo_work.block_33[0];
  cmo_work.KKT[130] = cmo_work.block_33[0];
  cmo_work.KKT[136] = cmo_work.block_33[0];
  cmo_work.KKT[142] = cmo_work.block_33[0];
  cmo_work.KKT[78] = cmo_work.block_33[0];
  cmo_work.KKT[84] = cmo_work.block_33[0];
  cmo_work.KKT[90] = cmo_work.block_33[0];
  cmo_work.KKT[96] = cmo_work.block_33[0];
  cmo_work.KKT[102] = cmo_work.block_33[0];
  cmo_work.KKT[108] = cmo_work.block_33[0];
  cmo_work.KKT[114] = cmo_work.block_33[0];
  cmo_work.KKT[120] = cmo_work.block_33[0];
  cmo_work.KKT[126] = cmo_work.block_33[0];
  cmo_work.KKT[132] = cmo_work.block_33[0];
  cmo_work.KKT[138] = cmo_work.block_33[0];
  cmo_work.KKT[144] = cmo_work.block_33[0];
  cmo_work.KKT[52] = -1;
  cmo_work.KKT[54] = -1;
  cmo_work.KKT[56] = -1;
  cmo_work.KKT[58] = -1;
  cmo_work.KKT[60] = -1;
  cmo_work.KKT[62] = -1;
  cmo_work.KKT[64] = -1;
  cmo_work.KKT[66] = -1;
  cmo_work.KKT[68] = -1;
  cmo_work.KKT[70] = -1;
  cmo_work.KKT[72] = -1;
  cmo_work.KKT[74] = -1;
  cmo_work.KKT[77] = 1;
  cmo_work.KKT[83] = 1;
  cmo_work.KKT[89] = 1;
  cmo_work.KKT[95] = 1;
  cmo_work.KKT[101] = 1;
  cmo_work.KKT[107] = 1;
  cmo_work.KKT[113] = 1;
  cmo_work.KKT[119] = 1;
  cmo_work.KKT[125] = 1;
  cmo_work.KKT[131] = 1;
  cmo_work.KKT[137] = 1;
  cmo_work.KKT[143] = 1;
  cmo_work.KKT[53] = -1;
  cmo_work.KKT[55] = -1;
  cmo_work.KKT[57] = -1;
  cmo_work.KKT[59] = -1;
  cmo_work.KKT[61] = -1;
  cmo_work.KKT[63] = -1;
  cmo_work.KKT[65] = -1;
  cmo_work.KKT[67] = -1;
  cmo_work.KKT[69] = -1;
  cmo_work.KKT[71] = -1;
  cmo_work.KKT[73] = -1;
  cmo_work.KKT[75] = -1;
  cmo_work.KKT[79] = -1;
  cmo_work.KKT[85] = -1;
  cmo_work.KKT[91] = -1;
  cmo_work.KKT[97] = -1;
  cmo_work.KKT[103] = -1;
  cmo_work.KKT[109] = -1;
  cmo_work.KKT[115] = -1;
  cmo_work.KKT[121] = -1;
  cmo_work.KKT[127] = -1;
  cmo_work.KKT[133] = -1;
  cmo_work.KKT[139] = -1;
  cmo_work.KKT[145] = -1;
  cmo_work.KKT[49] = -1;
  cmo_work.KKT[80] = cmo_params.Qam[0]*cmo_params.Jgm[0]+cmo_params.Qam[1]*cmo_params.Jgm[1]+cmo_params.Qam[2]*cmo_params.Jgm[2]+cmo_params.Qam[3]*cmo_params.Jgm[3]+cmo_params.Qam[4]*cmo_params.Jgm[4]+cmo_params.Qam[5]*cmo_params.Jgm[5];
  cmo_work.KKT[86] = cmo_params.Qam[0]*cmo_params.Jgm[6]+cmo_params.Qam[1]*cmo_params.Jgm[7]+cmo_params.Qam[2]*cmo_params.Jgm[8]+cmo_params.Qam[3]*cmo_params.Jgm[9]+cmo_params.Qam[4]*cmo_params.Jgm[10]+cmo_params.Qam[5]*cmo_params.Jgm[11];
  cmo_work.KKT[92] = cmo_params.Qam[0]*cmo_params.Jgm[12]+cmo_params.Qam[1]*cmo_params.Jgm[13]+cmo_params.Qam[2]*cmo_params.Jgm[14]+cmo_params.Qam[3]*cmo_params.Jgm[15]+cmo_params.Qam[4]*cmo_params.Jgm[16]+cmo_params.Qam[5]*cmo_params.Jgm[17];
  cmo_work.KKT[98] = cmo_params.Qam[0]*cmo_params.Jgm[18]+cmo_params.Qam[1]*cmo_params.Jgm[19]+cmo_params.Qam[2]*cmo_params.Jgm[20]+cmo_params.Qam[3]*cmo_params.Jgm[21]+cmo_params.Qam[4]*cmo_params.Jgm[22]+cmo_params.Qam[5]*cmo_params.Jgm[23];
  cmo_work.KKT[104] = cmo_params.Qam[0]*cmo_params.Jgm[24]+cmo_params.Qam[1]*cmo_params.Jgm[25]+cmo_params.Qam[2]*cmo_params.Jgm[26]+cmo_params.Qam[3]*cmo_params.Jgm[27]+cmo_params.Qam[4]*cmo_params.Jgm[28]+cmo_params.Qam[5]*cmo_params.Jgm[29];
  cmo_work.KKT[110] = cmo_params.Qam[0]*cmo_params.Jgm[30]+cmo_params.Qam[1]*cmo_params.Jgm[31]+cmo_params.Qam[2]*cmo_params.Jgm[32]+cmo_params.Qam[3]*cmo_params.Jgm[33]+cmo_params.Qam[4]*cmo_params.Jgm[34]+cmo_params.Qam[5]*cmo_params.Jgm[35];
  cmo_work.KKT[116] = cmo_params.Qam[0]*cmo_params.Jgm[36]+cmo_params.Qam[1]*cmo_params.Jgm[37]+cmo_params.Qam[2]*cmo_params.Jgm[38]+cmo_params.Qam[3]*cmo_params.Jgm[39]+cmo_params.Qam[4]*cmo_params.Jgm[40]+cmo_params.Qam[5]*cmo_params.Jgm[41];
  cmo_work.KKT[122] = cmo_params.Qam[0]*cmo_params.Jgm[42]+cmo_params.Qam[1]*cmo_params.Jgm[43]+cmo_params.Qam[2]*cmo_params.Jgm[44]+cmo_params.Qam[3]*cmo_params.Jgm[45]+cmo_params.Qam[4]*cmo_params.Jgm[46]+cmo_params.Qam[5]*cmo_params.Jgm[47];
  cmo_work.KKT[128] = cmo_params.Qam[0]*cmo_params.Jgm[48]+cmo_params.Qam[1]*cmo_params.Jgm[49]+cmo_params.Qam[2]*cmo_params.Jgm[50]+cmo_params.Qam[3]*cmo_params.Jgm[51]+cmo_params.Qam[4]*cmo_params.Jgm[52]+cmo_params.Qam[5]*cmo_params.Jgm[53];
  cmo_work.KKT[134] = cmo_params.Qam[0]*cmo_params.Jgm[54]+cmo_params.Qam[1]*cmo_params.Jgm[55]+cmo_params.Qam[2]*cmo_params.Jgm[56]+cmo_params.Qam[3]*cmo_params.Jgm[57]+cmo_params.Qam[4]*cmo_params.Jgm[58]+cmo_params.Qam[5]*cmo_params.Jgm[59];
  cmo_work.KKT[140] = cmo_params.Qam[0]*cmo_params.Jgm[60]+cmo_params.Qam[1]*cmo_params.Jgm[61]+cmo_params.Qam[2]*cmo_params.Jgm[62]+cmo_params.Qam[3]*cmo_params.Jgm[63]+cmo_params.Qam[4]*cmo_params.Jgm[64]+cmo_params.Qam[5]*cmo_params.Jgm[65];
  cmo_work.KKT[146] = cmo_params.Qam[0]*cmo_params.Jgm[66]+cmo_params.Qam[1]*cmo_params.Jgm[67]+cmo_params.Qam[2]*cmo_params.Jgm[68]+cmo_params.Qam[3]*cmo_params.Jgm[69]+cmo_params.Qam[4]*cmo_params.Jgm[70]+cmo_params.Qam[5]*cmo_params.Jgm[71];
  cmo_work.KKT[51] = -1;
  cmo_work.KKT[81] = cmo_params.Qrm[0]*cmo_params.Jgc[0]+cmo_params.Qrm[1]*cmo_params.Jgc[1]+cmo_params.Qrm[2]*cmo_params.Jgc[2]+cmo_params.Qrm[3]*cmo_params.Jgc[3]+cmo_params.Qrm[4]*cmo_params.Jgc[4]+cmo_params.Qrm[5]*cmo_params.Jgc[5]+cmo_params.Qrm[6]*cmo_params.Jgc[6]+cmo_params.Qrm[7]*cmo_params.Jgc[7]+cmo_params.Qrm[8]*cmo_params.Jgc[8]+cmo_params.Qrm[9]*cmo_params.Jgc[9]+cmo_params.Qrm[10]*cmo_params.Jgc[10]+cmo_params.Qrm[11]*cmo_params.Jgc[11];
  cmo_work.KKT[87] = cmo_params.Qrm[0]*cmo_params.Jgc[12]+cmo_params.Qrm[1]*cmo_params.Jgc[13]+cmo_params.Qrm[2]*cmo_params.Jgc[14]+cmo_params.Qrm[3]*cmo_params.Jgc[15]+cmo_params.Qrm[4]*cmo_params.Jgc[16]+cmo_params.Qrm[5]*cmo_params.Jgc[17]+cmo_params.Qrm[6]*cmo_params.Jgc[18]+cmo_params.Qrm[7]*cmo_params.Jgc[19]+cmo_params.Qrm[8]*cmo_params.Jgc[20]+cmo_params.Qrm[9]*cmo_params.Jgc[21]+cmo_params.Qrm[10]*cmo_params.Jgc[22]+cmo_params.Qrm[11]*cmo_params.Jgc[23];
  cmo_work.KKT[93] = cmo_params.Qrm[0]*cmo_params.Jgc[24]+cmo_params.Qrm[1]*cmo_params.Jgc[25]+cmo_params.Qrm[2]*cmo_params.Jgc[26]+cmo_params.Qrm[3]*cmo_params.Jgc[27]+cmo_params.Qrm[4]*cmo_params.Jgc[28]+cmo_params.Qrm[5]*cmo_params.Jgc[29]+cmo_params.Qrm[6]*cmo_params.Jgc[30]+cmo_params.Qrm[7]*cmo_params.Jgc[31]+cmo_params.Qrm[8]*cmo_params.Jgc[32]+cmo_params.Qrm[9]*cmo_params.Jgc[33]+cmo_params.Qrm[10]*cmo_params.Jgc[34]+cmo_params.Qrm[11]*cmo_params.Jgc[35];
  cmo_work.KKT[99] = cmo_params.Qrm[0]*cmo_params.Jgc[36]+cmo_params.Qrm[1]*cmo_params.Jgc[37]+cmo_params.Qrm[2]*cmo_params.Jgc[38]+cmo_params.Qrm[3]*cmo_params.Jgc[39]+cmo_params.Qrm[4]*cmo_params.Jgc[40]+cmo_params.Qrm[5]*cmo_params.Jgc[41]+cmo_params.Qrm[6]*cmo_params.Jgc[42]+cmo_params.Qrm[7]*cmo_params.Jgc[43]+cmo_params.Qrm[8]*cmo_params.Jgc[44]+cmo_params.Qrm[9]*cmo_params.Jgc[45]+cmo_params.Qrm[10]*cmo_params.Jgc[46]+cmo_params.Qrm[11]*cmo_params.Jgc[47];
  cmo_work.KKT[105] = cmo_params.Qrm[0]*cmo_params.Jgc[48]+cmo_params.Qrm[1]*cmo_params.Jgc[49]+cmo_params.Qrm[2]*cmo_params.Jgc[50]+cmo_params.Qrm[3]*cmo_params.Jgc[51]+cmo_params.Qrm[4]*cmo_params.Jgc[52]+cmo_params.Qrm[5]*cmo_params.Jgc[53]+cmo_params.Qrm[6]*cmo_params.Jgc[54]+cmo_params.Qrm[7]*cmo_params.Jgc[55]+cmo_params.Qrm[8]*cmo_params.Jgc[56]+cmo_params.Qrm[9]*cmo_params.Jgc[57]+cmo_params.Qrm[10]*cmo_params.Jgc[58]+cmo_params.Qrm[11]*cmo_params.Jgc[59];
  cmo_work.KKT[111] = cmo_params.Qrm[0]*cmo_params.Jgc[60]+cmo_params.Qrm[1]*cmo_params.Jgc[61]+cmo_params.Qrm[2]*cmo_params.Jgc[62]+cmo_params.Qrm[3]*cmo_params.Jgc[63]+cmo_params.Qrm[4]*cmo_params.Jgc[64]+cmo_params.Qrm[5]*cmo_params.Jgc[65]+cmo_params.Qrm[6]*cmo_params.Jgc[66]+cmo_params.Qrm[7]*cmo_params.Jgc[67]+cmo_params.Qrm[8]*cmo_params.Jgc[68]+cmo_params.Qrm[9]*cmo_params.Jgc[69]+cmo_params.Qrm[10]*cmo_params.Jgc[70]+cmo_params.Qrm[11]*cmo_params.Jgc[71];
  cmo_work.KKT[117] = cmo_params.Qrm[0]*cmo_params.Jgc[72]+cmo_params.Qrm[1]*cmo_params.Jgc[73]+cmo_params.Qrm[2]*cmo_params.Jgc[74]+cmo_params.Qrm[3]*cmo_params.Jgc[75]+cmo_params.Qrm[4]*cmo_params.Jgc[76]+cmo_params.Qrm[5]*cmo_params.Jgc[77]+cmo_params.Qrm[6]*cmo_params.Jgc[78]+cmo_params.Qrm[7]*cmo_params.Jgc[79]+cmo_params.Qrm[8]*cmo_params.Jgc[80]+cmo_params.Qrm[9]*cmo_params.Jgc[81]+cmo_params.Qrm[10]*cmo_params.Jgc[82]+cmo_params.Qrm[11]*cmo_params.Jgc[83];
  cmo_work.KKT[123] = cmo_params.Qrm[0]*cmo_params.Jgc[84]+cmo_params.Qrm[1]*cmo_params.Jgc[85]+cmo_params.Qrm[2]*cmo_params.Jgc[86]+cmo_params.Qrm[3]*cmo_params.Jgc[87]+cmo_params.Qrm[4]*cmo_params.Jgc[88]+cmo_params.Qrm[5]*cmo_params.Jgc[89]+cmo_params.Qrm[6]*cmo_params.Jgc[90]+cmo_params.Qrm[7]*cmo_params.Jgc[91]+cmo_params.Qrm[8]*cmo_params.Jgc[92]+cmo_params.Qrm[9]*cmo_params.Jgc[93]+cmo_params.Qrm[10]*cmo_params.Jgc[94]+cmo_params.Qrm[11]*cmo_params.Jgc[95];
  cmo_work.KKT[129] = cmo_params.Qrm[0]*cmo_params.Jgc[96]+cmo_params.Qrm[1]*cmo_params.Jgc[97]+cmo_params.Qrm[2]*cmo_params.Jgc[98]+cmo_params.Qrm[3]*cmo_params.Jgc[99]+cmo_params.Qrm[4]*cmo_params.Jgc[100]+cmo_params.Qrm[5]*cmo_params.Jgc[101]+cmo_params.Qrm[6]*cmo_params.Jgc[102]+cmo_params.Qrm[7]*cmo_params.Jgc[103]+cmo_params.Qrm[8]*cmo_params.Jgc[104]+cmo_params.Qrm[9]*cmo_params.Jgc[105]+cmo_params.Qrm[10]*cmo_params.Jgc[106]+cmo_params.Qrm[11]*cmo_params.Jgc[107];
  cmo_work.KKT[135] = cmo_params.Qrm[0]*cmo_params.Jgc[108]+cmo_params.Qrm[1]*cmo_params.Jgc[109]+cmo_params.Qrm[2]*cmo_params.Jgc[110]+cmo_params.Qrm[3]*cmo_params.Jgc[111]+cmo_params.Qrm[4]*cmo_params.Jgc[112]+cmo_params.Qrm[5]*cmo_params.Jgc[113]+cmo_params.Qrm[6]*cmo_params.Jgc[114]+cmo_params.Qrm[7]*cmo_params.Jgc[115]+cmo_params.Qrm[8]*cmo_params.Jgc[116]+cmo_params.Qrm[9]*cmo_params.Jgc[117]+cmo_params.Qrm[10]*cmo_params.Jgc[118]+cmo_params.Qrm[11]*cmo_params.Jgc[119];
  cmo_work.KKT[141] = cmo_params.Qrm[0]*cmo_params.Jgc[120]+cmo_params.Qrm[1]*cmo_params.Jgc[121]+cmo_params.Qrm[2]*cmo_params.Jgc[122]+cmo_params.Qrm[3]*cmo_params.Jgc[123]+cmo_params.Qrm[4]*cmo_params.Jgc[124]+cmo_params.Qrm[5]*cmo_params.Jgc[125]+cmo_params.Qrm[6]*cmo_params.Jgc[126]+cmo_params.Qrm[7]*cmo_params.Jgc[127]+cmo_params.Qrm[8]*cmo_params.Jgc[128]+cmo_params.Qrm[9]*cmo_params.Jgc[129]+cmo_params.Qrm[10]*cmo_params.Jgc[130]+cmo_params.Qrm[11]*cmo_params.Jgc[131];
  cmo_work.KKT[147] = cmo_params.Qrm[0]*cmo_params.Jgc[132]+cmo_params.Qrm[1]*cmo_params.Jgc[133]+cmo_params.Qrm[2]*cmo_params.Jgc[134]+cmo_params.Qrm[3]*cmo_params.Jgc[135]+cmo_params.Qrm[4]*cmo_params.Jgc[136]+cmo_params.Qrm[5]*cmo_params.Jgc[137]+cmo_params.Qrm[6]*cmo_params.Jgc[138]+cmo_params.Qrm[7]*cmo_params.Jgc[139]+cmo_params.Qrm[8]*cmo_params.Jgc[140]+cmo_params.Qrm[9]*cmo_params.Jgc[141]+cmo_params.Qrm[10]*cmo_params.Jgc[142]+cmo_params.Qrm[11]*cmo_params.Jgc[143];
}
