/* Produced by CVXGEN, 2020-08-23 02:50:50 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: matrix_support.c. */
/* Description: Support functions for matrix multiplication and vector filling. */
#include "legIK_solver.h"
void legIK_multbymA(double *lhs, double *rhs) {
}
void legIK_multbymAT(double *lhs, double *rhs) {
  lhs[0] = 0;
  lhs[1] = 0;
  lhs[2] = 0;
  lhs[3] = 0;
  lhs[4] = 0;
  lhs[5] = 0;
  lhs[6] = 0;
  lhs[7] = 0;
  lhs[8] = 0;
  lhs[9] = 0;
  lhs[10] = 0;
  lhs[11] = 0;
  lhs[12] = 0;
  lhs[13] = 0;
  lhs[14] = 0;
  lhs[15] = 0;
  lhs[16] = 0;
  lhs[17] = 0;
}
void legIK_multbymG(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(legIK_params.dt[0]*legIK_params.NeJc_1[0])-rhs[1]*(legIK_params.dt[0]*legIK_params.NeJc_1[1])-rhs[2]*(legIK_params.dt[0]*legIK_params.NeJc_1[2])-rhs[3]*(legIK_params.dt[0]*legIK_params.NeJc_1[3])-rhs[4]*(legIK_params.dt[0]*legIK_params.NeJc_1[4])-rhs[5]*(legIK_params.dt[0]*legIK_params.NeJc_1[5])-rhs[6]*(legIK_params.dt[0]*legIK_params.NeJc_1[6])-rhs[7]*(legIK_params.dt[0]*legIK_params.NeJc_1[7])-rhs[8]*(legIK_params.dt[0]*legIK_params.NeJc_1[8])-rhs[9]*(legIK_params.dt[0]*legIK_params.NeJc_1[9])-rhs[10]*(legIK_params.dt[0]*legIK_params.NeJc_1[10])-rhs[11]*(legIK_params.dt[0]*legIK_params.NeJc_1[11])-rhs[12]*(legIK_params.dt[0]*legIK_params.NeJc_1[12])-rhs[13]*(legIK_params.dt[0]*legIK_params.NeJc_1[13])-rhs[14]*(legIK_params.dt[0]*legIK_params.NeJc_1[14])-rhs[15]*(legIK_params.dt[0]*legIK_params.NeJc_1[15])-rhs[16]*(legIK_params.dt[0]*legIK_params.NeJc_1[16])-rhs[17]*(legIK_params.dt[0]*legIK_params.NeJc_1[17]);
  lhs[1] = -rhs[0]*(legIK_params.dt[0]*legIK_params.NeJc_2[0])-rhs[1]*(legIK_params.dt[0]*legIK_params.NeJc_2[1])-rhs[2]*(legIK_params.dt[0]*legIK_params.NeJc_2[2])-rhs[3]*(legIK_params.dt[0]*legIK_params.NeJc_2[3])-rhs[4]*(legIK_params.dt[0]*legIK_params.NeJc_2[4])-rhs[5]*(legIK_params.dt[0]*legIK_params.NeJc_2[5])-rhs[6]*(legIK_params.dt[0]*legIK_params.NeJc_2[6])-rhs[7]*(legIK_params.dt[0]*legIK_params.NeJc_2[7])-rhs[8]*(legIK_params.dt[0]*legIK_params.NeJc_2[8])-rhs[9]*(legIK_params.dt[0]*legIK_params.NeJc_2[9])-rhs[10]*(legIK_params.dt[0]*legIK_params.NeJc_2[10])-rhs[11]*(legIK_params.dt[0]*legIK_params.NeJc_2[11])-rhs[12]*(legIK_params.dt[0]*legIK_params.NeJc_2[12])-rhs[13]*(legIK_params.dt[0]*legIK_params.NeJc_2[13])-rhs[14]*(legIK_params.dt[0]*legIK_params.NeJc_2[14])-rhs[15]*(legIK_params.dt[0]*legIK_params.NeJc_2[15])-rhs[16]*(legIK_params.dt[0]*legIK_params.NeJc_2[16])-rhs[17]*(legIK_params.dt[0]*legIK_params.NeJc_2[17]);
  lhs[2] = -rhs[0]*(legIK_params.dt[0]*legIK_params.NeJc_3[0])-rhs[1]*(legIK_params.dt[0]*legIK_params.NeJc_3[1])-rhs[2]*(legIK_params.dt[0]*legIK_params.NeJc_3[2])-rhs[3]*(legIK_params.dt[0]*legIK_params.NeJc_3[3])-rhs[4]*(legIK_params.dt[0]*legIK_params.NeJc_3[4])-rhs[5]*(legIK_params.dt[0]*legIK_params.NeJc_3[5])-rhs[6]*(legIK_params.dt[0]*legIK_params.NeJc_3[6])-rhs[7]*(legIK_params.dt[0]*legIK_params.NeJc_3[7])-rhs[8]*(legIK_params.dt[0]*legIK_params.NeJc_3[8])-rhs[9]*(legIK_params.dt[0]*legIK_params.NeJc_3[9])-rhs[10]*(legIK_params.dt[0]*legIK_params.NeJc_3[10])-rhs[11]*(legIK_params.dt[0]*legIK_params.NeJc_3[11])-rhs[12]*(legIK_params.dt[0]*legIK_params.NeJc_3[12])-rhs[13]*(legIK_params.dt[0]*legIK_params.NeJc_3[13])-rhs[14]*(legIK_params.dt[0]*legIK_params.NeJc_3[14])-rhs[15]*(legIK_params.dt[0]*legIK_params.NeJc_3[15])-rhs[16]*(legIK_params.dt[0]*legIK_params.NeJc_3[16])-rhs[17]*(legIK_params.dt[0]*legIK_params.NeJc_3[17]);
  lhs[3] = -rhs[0]*(legIK_params.dt[0]*legIK_params.NeJc_4[0])-rhs[1]*(legIK_params.dt[0]*legIK_params.NeJc_4[1])-rhs[2]*(legIK_params.dt[0]*legIK_params.NeJc_4[2])-rhs[3]*(legIK_params.dt[0]*legIK_params.NeJc_4[3])-rhs[4]*(legIK_params.dt[0]*legIK_params.NeJc_4[4])-rhs[5]*(legIK_params.dt[0]*legIK_params.NeJc_4[5])-rhs[6]*(legIK_params.dt[0]*legIK_params.NeJc_4[6])-rhs[7]*(legIK_params.dt[0]*legIK_params.NeJc_4[7])-rhs[8]*(legIK_params.dt[0]*legIK_params.NeJc_4[8])-rhs[9]*(legIK_params.dt[0]*legIK_params.NeJc_4[9])-rhs[10]*(legIK_params.dt[0]*legIK_params.NeJc_4[10])-rhs[11]*(legIK_params.dt[0]*legIK_params.NeJc_4[11])-rhs[12]*(legIK_params.dt[0]*legIK_params.NeJc_4[12])-rhs[13]*(legIK_params.dt[0]*legIK_params.NeJc_4[13])-rhs[14]*(legIK_params.dt[0]*legIK_params.NeJc_4[14])-rhs[15]*(legIK_params.dt[0]*legIK_params.NeJc_4[15])-rhs[16]*(legIK_params.dt[0]*legIK_params.NeJc_4[16])-rhs[17]*(legIK_params.dt[0]*legIK_params.NeJc_4[17]);
  lhs[4] = -rhs[0]*(legIK_params.dt[0]*legIK_params.NeJc_5[0])-rhs[1]*(legIK_params.dt[0]*legIK_params.NeJc_5[1])-rhs[2]*(legIK_params.dt[0]*legIK_params.NeJc_5[2])-rhs[3]*(legIK_params.dt[0]*legIK_params.NeJc_5[3])-rhs[4]*(legIK_params.dt[0]*legIK_params.NeJc_5[4])-rhs[5]*(legIK_params.dt[0]*legIK_params.NeJc_5[5])-rhs[6]*(legIK_params.dt[0]*legIK_params.NeJc_5[6])-rhs[7]*(legIK_params.dt[0]*legIK_params.NeJc_5[7])-rhs[8]*(legIK_params.dt[0]*legIK_params.NeJc_5[8])-rhs[9]*(legIK_params.dt[0]*legIK_params.NeJc_5[9])-rhs[10]*(legIK_params.dt[0]*legIK_params.NeJc_5[10])-rhs[11]*(legIK_params.dt[0]*legIK_params.NeJc_5[11])-rhs[12]*(legIK_params.dt[0]*legIK_params.NeJc_5[12])-rhs[13]*(legIK_params.dt[0]*legIK_params.NeJc_5[13])-rhs[14]*(legIK_params.dt[0]*legIK_params.NeJc_5[14])-rhs[15]*(legIK_params.dt[0]*legIK_params.NeJc_5[15])-rhs[16]*(legIK_params.dt[0]*legIK_params.NeJc_5[16])-rhs[17]*(legIK_params.dt[0]*legIK_params.NeJc_5[17]);
  lhs[5] = -rhs[0]*(legIK_params.dt[0]*legIK_params.NeJc_6[0])-rhs[1]*(legIK_params.dt[0]*legIK_params.NeJc_6[1])-rhs[2]*(legIK_params.dt[0]*legIK_params.NeJc_6[2])-rhs[3]*(legIK_params.dt[0]*legIK_params.NeJc_6[3])-rhs[4]*(legIK_params.dt[0]*legIK_params.NeJc_6[4])-rhs[5]*(legIK_params.dt[0]*legIK_params.NeJc_6[5])-rhs[6]*(legIK_params.dt[0]*legIK_params.NeJc_6[6])-rhs[7]*(legIK_params.dt[0]*legIK_params.NeJc_6[7])-rhs[8]*(legIK_params.dt[0]*legIK_params.NeJc_6[8])-rhs[9]*(legIK_params.dt[0]*legIK_params.NeJc_6[9])-rhs[10]*(legIK_params.dt[0]*legIK_params.NeJc_6[10])-rhs[11]*(legIK_params.dt[0]*legIK_params.NeJc_6[11])-rhs[12]*(legIK_params.dt[0]*legIK_params.NeJc_6[12])-rhs[13]*(legIK_params.dt[0]*legIK_params.NeJc_6[13])-rhs[14]*(legIK_params.dt[0]*legIK_params.NeJc_6[14])-rhs[15]*(legIK_params.dt[0]*legIK_params.NeJc_6[15])-rhs[16]*(legIK_params.dt[0]*legIK_params.NeJc_6[16])-rhs[17]*(legIK_params.dt[0]*legIK_params.NeJc_6[17]);
  lhs[6] = -rhs[6]*(-legIK_params.dt[0]);
  lhs[7] = -rhs[7]*(-legIK_params.dt[0]);
  lhs[8] = -rhs[8]*(-legIK_params.dt[0]);
  lhs[9] = -rhs[9]*(-legIK_params.dt[0]);
  lhs[10] = -rhs[10]*(-legIK_params.dt[0]);
  lhs[11] = -rhs[11]*(-legIK_params.dt[0]);
  lhs[12] = -rhs[12]*(-legIK_params.dt[0]);
  lhs[13] = -rhs[13]*(-legIK_params.dt[0]);
  lhs[14] = -rhs[14]*(-legIK_params.dt[0]);
  lhs[15] = -rhs[15]*(-legIK_params.dt[0]);
  lhs[16] = -rhs[16]*(-legIK_params.dt[0]);
  lhs[17] = -rhs[17]*(-legIK_params.dt[0]);
  lhs[18] = -rhs[6]*(legIK_params.dt[0]);
  lhs[19] = -rhs[7]*(legIK_params.dt[0]);
  lhs[20] = -rhs[8]*(legIK_params.dt[0]);
  lhs[21] = -rhs[9]*(legIK_params.dt[0]);
  lhs[22] = -rhs[10]*(legIK_params.dt[0]);
  lhs[23] = -rhs[11]*(legIK_params.dt[0]);
  lhs[24] = -rhs[12]*(legIK_params.dt[0]);
  lhs[25] = -rhs[13]*(legIK_params.dt[0]);
  lhs[26] = -rhs[14]*(legIK_params.dt[0]);
  lhs[27] = -rhs[15]*(legIK_params.dt[0]);
  lhs[28] = -rhs[16]*(legIK_params.dt[0]);
  lhs[29] = -rhs[17]*(legIK_params.dt[0]);
}
void legIK_multbymGT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(legIK_params.dt[0]*legIK_params.NeJc_1[0])-rhs[1]*(legIK_params.dt[0]*legIK_params.NeJc_2[0])-rhs[2]*(legIK_params.dt[0]*legIK_params.NeJc_3[0])-rhs[3]*(legIK_params.dt[0]*legIK_params.NeJc_4[0])-rhs[4]*(legIK_params.dt[0]*legIK_params.NeJc_5[0])-rhs[5]*(legIK_params.dt[0]*legIK_params.NeJc_6[0]);
  lhs[1] = -rhs[0]*(legIK_params.dt[0]*legIK_params.NeJc_1[1])-rhs[1]*(legIK_params.dt[0]*legIK_params.NeJc_2[1])-rhs[2]*(legIK_params.dt[0]*legIK_params.NeJc_3[1])-rhs[3]*(legIK_params.dt[0]*legIK_params.NeJc_4[1])-rhs[4]*(legIK_params.dt[0]*legIK_params.NeJc_5[1])-rhs[5]*(legIK_params.dt[0]*legIK_params.NeJc_6[1]);
  lhs[2] = -rhs[0]*(legIK_params.dt[0]*legIK_params.NeJc_1[2])-rhs[1]*(legIK_params.dt[0]*legIK_params.NeJc_2[2])-rhs[2]*(legIK_params.dt[0]*legIK_params.NeJc_3[2])-rhs[3]*(legIK_params.dt[0]*legIK_params.NeJc_4[2])-rhs[4]*(legIK_params.dt[0]*legIK_params.NeJc_5[2])-rhs[5]*(legIK_params.dt[0]*legIK_params.NeJc_6[2]);
  lhs[3] = -rhs[0]*(legIK_params.dt[0]*legIK_params.NeJc_1[3])-rhs[1]*(legIK_params.dt[0]*legIK_params.NeJc_2[3])-rhs[2]*(legIK_params.dt[0]*legIK_params.NeJc_3[3])-rhs[3]*(legIK_params.dt[0]*legIK_params.NeJc_4[3])-rhs[4]*(legIK_params.dt[0]*legIK_params.NeJc_5[3])-rhs[5]*(legIK_params.dt[0]*legIK_params.NeJc_6[3]);
  lhs[4] = -rhs[0]*(legIK_params.dt[0]*legIK_params.NeJc_1[4])-rhs[1]*(legIK_params.dt[0]*legIK_params.NeJc_2[4])-rhs[2]*(legIK_params.dt[0]*legIK_params.NeJc_3[4])-rhs[3]*(legIK_params.dt[0]*legIK_params.NeJc_4[4])-rhs[4]*(legIK_params.dt[0]*legIK_params.NeJc_5[4])-rhs[5]*(legIK_params.dt[0]*legIK_params.NeJc_6[4]);
  lhs[5] = -rhs[0]*(legIK_params.dt[0]*legIK_params.NeJc_1[5])-rhs[1]*(legIK_params.dt[0]*legIK_params.NeJc_2[5])-rhs[2]*(legIK_params.dt[0]*legIK_params.NeJc_3[5])-rhs[3]*(legIK_params.dt[0]*legIK_params.NeJc_4[5])-rhs[4]*(legIK_params.dt[0]*legIK_params.NeJc_5[5])-rhs[5]*(legIK_params.dt[0]*legIK_params.NeJc_6[5]);
  lhs[6] = -rhs[0]*(legIK_params.dt[0]*legIK_params.NeJc_1[6])-rhs[1]*(legIK_params.dt[0]*legIK_params.NeJc_2[6])-rhs[2]*(legIK_params.dt[0]*legIK_params.NeJc_3[6])-rhs[3]*(legIK_params.dt[0]*legIK_params.NeJc_4[6])-rhs[4]*(legIK_params.dt[0]*legIK_params.NeJc_5[6])-rhs[5]*(legIK_params.dt[0]*legIK_params.NeJc_6[6])-rhs[6]*(-legIK_params.dt[0])-rhs[18]*(legIK_params.dt[0]);
  lhs[7] = -rhs[0]*(legIK_params.dt[0]*legIK_params.NeJc_1[7])-rhs[1]*(legIK_params.dt[0]*legIK_params.NeJc_2[7])-rhs[2]*(legIK_params.dt[0]*legIK_params.NeJc_3[7])-rhs[3]*(legIK_params.dt[0]*legIK_params.NeJc_4[7])-rhs[4]*(legIK_params.dt[0]*legIK_params.NeJc_5[7])-rhs[5]*(legIK_params.dt[0]*legIK_params.NeJc_6[7])-rhs[7]*(-legIK_params.dt[0])-rhs[19]*(legIK_params.dt[0]);
  lhs[8] = -rhs[0]*(legIK_params.dt[0]*legIK_params.NeJc_1[8])-rhs[1]*(legIK_params.dt[0]*legIK_params.NeJc_2[8])-rhs[2]*(legIK_params.dt[0]*legIK_params.NeJc_3[8])-rhs[3]*(legIK_params.dt[0]*legIK_params.NeJc_4[8])-rhs[4]*(legIK_params.dt[0]*legIK_params.NeJc_5[8])-rhs[5]*(legIK_params.dt[0]*legIK_params.NeJc_6[8])-rhs[8]*(-legIK_params.dt[0])-rhs[20]*(legIK_params.dt[0]);
  lhs[9] = -rhs[0]*(legIK_params.dt[0]*legIK_params.NeJc_1[9])-rhs[1]*(legIK_params.dt[0]*legIK_params.NeJc_2[9])-rhs[2]*(legIK_params.dt[0]*legIK_params.NeJc_3[9])-rhs[3]*(legIK_params.dt[0]*legIK_params.NeJc_4[9])-rhs[4]*(legIK_params.dt[0]*legIK_params.NeJc_5[9])-rhs[5]*(legIK_params.dt[0]*legIK_params.NeJc_6[9])-rhs[9]*(-legIK_params.dt[0])-rhs[21]*(legIK_params.dt[0]);
  lhs[10] = -rhs[0]*(legIK_params.dt[0]*legIK_params.NeJc_1[10])-rhs[1]*(legIK_params.dt[0]*legIK_params.NeJc_2[10])-rhs[2]*(legIK_params.dt[0]*legIK_params.NeJc_3[10])-rhs[3]*(legIK_params.dt[0]*legIK_params.NeJc_4[10])-rhs[4]*(legIK_params.dt[0]*legIK_params.NeJc_5[10])-rhs[5]*(legIK_params.dt[0]*legIK_params.NeJc_6[10])-rhs[10]*(-legIK_params.dt[0])-rhs[22]*(legIK_params.dt[0]);
  lhs[11] = -rhs[0]*(legIK_params.dt[0]*legIK_params.NeJc_1[11])-rhs[1]*(legIK_params.dt[0]*legIK_params.NeJc_2[11])-rhs[2]*(legIK_params.dt[0]*legIK_params.NeJc_3[11])-rhs[3]*(legIK_params.dt[0]*legIK_params.NeJc_4[11])-rhs[4]*(legIK_params.dt[0]*legIK_params.NeJc_5[11])-rhs[5]*(legIK_params.dt[0]*legIK_params.NeJc_6[11])-rhs[11]*(-legIK_params.dt[0])-rhs[23]*(legIK_params.dt[0]);
  lhs[12] = -rhs[0]*(legIK_params.dt[0]*legIK_params.NeJc_1[12])-rhs[1]*(legIK_params.dt[0]*legIK_params.NeJc_2[12])-rhs[2]*(legIK_params.dt[0]*legIK_params.NeJc_3[12])-rhs[3]*(legIK_params.dt[0]*legIK_params.NeJc_4[12])-rhs[4]*(legIK_params.dt[0]*legIK_params.NeJc_5[12])-rhs[5]*(legIK_params.dt[0]*legIK_params.NeJc_6[12])-rhs[12]*(-legIK_params.dt[0])-rhs[24]*(legIK_params.dt[0]);
  lhs[13] = -rhs[0]*(legIK_params.dt[0]*legIK_params.NeJc_1[13])-rhs[1]*(legIK_params.dt[0]*legIK_params.NeJc_2[13])-rhs[2]*(legIK_params.dt[0]*legIK_params.NeJc_3[13])-rhs[3]*(legIK_params.dt[0]*legIK_params.NeJc_4[13])-rhs[4]*(legIK_params.dt[0]*legIK_params.NeJc_5[13])-rhs[5]*(legIK_params.dt[0]*legIK_params.NeJc_6[13])-rhs[13]*(-legIK_params.dt[0])-rhs[25]*(legIK_params.dt[0]);
  lhs[14] = -rhs[0]*(legIK_params.dt[0]*legIK_params.NeJc_1[14])-rhs[1]*(legIK_params.dt[0]*legIK_params.NeJc_2[14])-rhs[2]*(legIK_params.dt[0]*legIK_params.NeJc_3[14])-rhs[3]*(legIK_params.dt[0]*legIK_params.NeJc_4[14])-rhs[4]*(legIK_params.dt[0]*legIK_params.NeJc_5[14])-rhs[5]*(legIK_params.dt[0]*legIK_params.NeJc_6[14])-rhs[14]*(-legIK_params.dt[0])-rhs[26]*(legIK_params.dt[0]);
  lhs[15] = -rhs[0]*(legIK_params.dt[0]*legIK_params.NeJc_1[15])-rhs[1]*(legIK_params.dt[0]*legIK_params.NeJc_2[15])-rhs[2]*(legIK_params.dt[0]*legIK_params.NeJc_3[15])-rhs[3]*(legIK_params.dt[0]*legIK_params.NeJc_4[15])-rhs[4]*(legIK_params.dt[0]*legIK_params.NeJc_5[15])-rhs[5]*(legIK_params.dt[0]*legIK_params.NeJc_6[15])-rhs[15]*(-legIK_params.dt[0])-rhs[27]*(legIK_params.dt[0]);
  lhs[16] = -rhs[0]*(legIK_params.dt[0]*legIK_params.NeJc_1[16])-rhs[1]*(legIK_params.dt[0]*legIK_params.NeJc_2[16])-rhs[2]*(legIK_params.dt[0]*legIK_params.NeJc_3[16])-rhs[3]*(legIK_params.dt[0]*legIK_params.NeJc_4[16])-rhs[4]*(legIK_params.dt[0]*legIK_params.NeJc_5[16])-rhs[5]*(legIK_params.dt[0]*legIK_params.NeJc_6[16])-rhs[16]*(-legIK_params.dt[0])-rhs[28]*(legIK_params.dt[0]);
  lhs[17] = -rhs[0]*(legIK_params.dt[0]*legIK_params.NeJc_1[17])-rhs[1]*(legIK_params.dt[0]*legIK_params.NeJc_2[17])-rhs[2]*(legIK_params.dt[0]*legIK_params.NeJc_3[17])-rhs[3]*(legIK_params.dt[0]*legIK_params.NeJc_4[17])-rhs[4]*(legIK_params.dt[0]*legIK_params.NeJc_5[17])-rhs[5]*(legIK_params.dt[0]*legIK_params.NeJc_6[17])-rhs[17]*(-legIK_params.dt[0])-rhs[29]*(legIK_params.dt[0]);
}
void legIK_multbyP(double *lhs, double *rhs) {
  /* TODO use the fact that P is symmetric? */
  /* TODO check doubling / half factor etc. */
  lhs[0] = rhs[0]*(2*legIK_params.Q[0])+rhs[1]*(2*legIK_params.Q[18])+rhs[2]*(2*legIK_params.Q[36])+rhs[3]*(2*legIK_params.Q[54])+rhs[4]*(2*legIK_params.Q[72])+rhs[5]*(2*legIK_params.Q[90])+rhs[6]*(2*legIK_params.Q[108])+rhs[7]*(2*legIK_params.Q[126])+rhs[8]*(2*legIK_params.Q[144])+rhs[9]*(2*legIK_params.Q[162])+rhs[10]*(2*legIK_params.Q[180])+rhs[11]*(2*legIK_params.Q[198])+rhs[12]*(2*legIK_params.Q[216])+rhs[13]*(2*legIK_params.Q[234])+rhs[14]*(2*legIK_params.Q[252])+rhs[15]*(2*legIK_params.Q[270])+rhs[16]*(2*legIK_params.Q[288])+rhs[17]*(2*legIK_params.Q[306]);
  lhs[1] = rhs[0]*(2*legIK_params.Q[1])+rhs[1]*(2*legIK_params.Q[19])+rhs[2]*(2*legIK_params.Q[37])+rhs[3]*(2*legIK_params.Q[55])+rhs[4]*(2*legIK_params.Q[73])+rhs[5]*(2*legIK_params.Q[91])+rhs[6]*(2*legIK_params.Q[109])+rhs[7]*(2*legIK_params.Q[127])+rhs[8]*(2*legIK_params.Q[145])+rhs[9]*(2*legIK_params.Q[163])+rhs[10]*(2*legIK_params.Q[181])+rhs[11]*(2*legIK_params.Q[199])+rhs[12]*(2*legIK_params.Q[217])+rhs[13]*(2*legIK_params.Q[235])+rhs[14]*(2*legIK_params.Q[253])+rhs[15]*(2*legIK_params.Q[271])+rhs[16]*(2*legIK_params.Q[289])+rhs[17]*(2*legIK_params.Q[307]);
  lhs[2] = rhs[0]*(2*legIK_params.Q[2])+rhs[1]*(2*legIK_params.Q[20])+rhs[2]*(2*legIK_params.Q[38])+rhs[3]*(2*legIK_params.Q[56])+rhs[4]*(2*legIK_params.Q[74])+rhs[5]*(2*legIK_params.Q[92])+rhs[6]*(2*legIK_params.Q[110])+rhs[7]*(2*legIK_params.Q[128])+rhs[8]*(2*legIK_params.Q[146])+rhs[9]*(2*legIK_params.Q[164])+rhs[10]*(2*legIK_params.Q[182])+rhs[11]*(2*legIK_params.Q[200])+rhs[12]*(2*legIK_params.Q[218])+rhs[13]*(2*legIK_params.Q[236])+rhs[14]*(2*legIK_params.Q[254])+rhs[15]*(2*legIK_params.Q[272])+rhs[16]*(2*legIK_params.Q[290])+rhs[17]*(2*legIK_params.Q[308]);
  lhs[3] = rhs[0]*(2*legIK_params.Q[3])+rhs[1]*(2*legIK_params.Q[21])+rhs[2]*(2*legIK_params.Q[39])+rhs[3]*(2*legIK_params.Q[57])+rhs[4]*(2*legIK_params.Q[75])+rhs[5]*(2*legIK_params.Q[93])+rhs[6]*(2*legIK_params.Q[111])+rhs[7]*(2*legIK_params.Q[129])+rhs[8]*(2*legIK_params.Q[147])+rhs[9]*(2*legIK_params.Q[165])+rhs[10]*(2*legIK_params.Q[183])+rhs[11]*(2*legIK_params.Q[201])+rhs[12]*(2*legIK_params.Q[219])+rhs[13]*(2*legIK_params.Q[237])+rhs[14]*(2*legIK_params.Q[255])+rhs[15]*(2*legIK_params.Q[273])+rhs[16]*(2*legIK_params.Q[291])+rhs[17]*(2*legIK_params.Q[309]);
  lhs[4] = rhs[0]*(2*legIK_params.Q[4])+rhs[1]*(2*legIK_params.Q[22])+rhs[2]*(2*legIK_params.Q[40])+rhs[3]*(2*legIK_params.Q[58])+rhs[4]*(2*legIK_params.Q[76])+rhs[5]*(2*legIK_params.Q[94])+rhs[6]*(2*legIK_params.Q[112])+rhs[7]*(2*legIK_params.Q[130])+rhs[8]*(2*legIK_params.Q[148])+rhs[9]*(2*legIK_params.Q[166])+rhs[10]*(2*legIK_params.Q[184])+rhs[11]*(2*legIK_params.Q[202])+rhs[12]*(2*legIK_params.Q[220])+rhs[13]*(2*legIK_params.Q[238])+rhs[14]*(2*legIK_params.Q[256])+rhs[15]*(2*legIK_params.Q[274])+rhs[16]*(2*legIK_params.Q[292])+rhs[17]*(2*legIK_params.Q[310]);
  lhs[5] = rhs[0]*(2*legIK_params.Q[5])+rhs[1]*(2*legIK_params.Q[23])+rhs[2]*(2*legIK_params.Q[41])+rhs[3]*(2*legIK_params.Q[59])+rhs[4]*(2*legIK_params.Q[77])+rhs[5]*(2*legIK_params.Q[95])+rhs[6]*(2*legIK_params.Q[113])+rhs[7]*(2*legIK_params.Q[131])+rhs[8]*(2*legIK_params.Q[149])+rhs[9]*(2*legIK_params.Q[167])+rhs[10]*(2*legIK_params.Q[185])+rhs[11]*(2*legIK_params.Q[203])+rhs[12]*(2*legIK_params.Q[221])+rhs[13]*(2*legIK_params.Q[239])+rhs[14]*(2*legIK_params.Q[257])+rhs[15]*(2*legIK_params.Q[275])+rhs[16]*(2*legIK_params.Q[293])+rhs[17]*(2*legIK_params.Q[311]);
  lhs[6] = rhs[0]*(2*legIK_params.Q[6])+rhs[1]*(2*legIK_params.Q[24])+rhs[2]*(2*legIK_params.Q[42])+rhs[3]*(2*legIK_params.Q[60])+rhs[4]*(2*legIK_params.Q[78])+rhs[5]*(2*legIK_params.Q[96])+rhs[6]*(2*legIK_params.Q[114])+rhs[7]*(2*legIK_params.Q[132])+rhs[8]*(2*legIK_params.Q[150])+rhs[9]*(2*legIK_params.Q[168])+rhs[10]*(2*legIK_params.Q[186])+rhs[11]*(2*legIK_params.Q[204])+rhs[12]*(2*legIK_params.Q[222])+rhs[13]*(2*legIK_params.Q[240])+rhs[14]*(2*legIK_params.Q[258])+rhs[15]*(2*legIK_params.Q[276])+rhs[16]*(2*legIK_params.Q[294])+rhs[17]*(2*legIK_params.Q[312]);
  lhs[7] = rhs[0]*(2*legIK_params.Q[7])+rhs[1]*(2*legIK_params.Q[25])+rhs[2]*(2*legIK_params.Q[43])+rhs[3]*(2*legIK_params.Q[61])+rhs[4]*(2*legIK_params.Q[79])+rhs[5]*(2*legIK_params.Q[97])+rhs[6]*(2*legIK_params.Q[115])+rhs[7]*(2*legIK_params.Q[133])+rhs[8]*(2*legIK_params.Q[151])+rhs[9]*(2*legIK_params.Q[169])+rhs[10]*(2*legIK_params.Q[187])+rhs[11]*(2*legIK_params.Q[205])+rhs[12]*(2*legIK_params.Q[223])+rhs[13]*(2*legIK_params.Q[241])+rhs[14]*(2*legIK_params.Q[259])+rhs[15]*(2*legIK_params.Q[277])+rhs[16]*(2*legIK_params.Q[295])+rhs[17]*(2*legIK_params.Q[313]);
  lhs[8] = rhs[0]*(2*legIK_params.Q[8])+rhs[1]*(2*legIK_params.Q[26])+rhs[2]*(2*legIK_params.Q[44])+rhs[3]*(2*legIK_params.Q[62])+rhs[4]*(2*legIK_params.Q[80])+rhs[5]*(2*legIK_params.Q[98])+rhs[6]*(2*legIK_params.Q[116])+rhs[7]*(2*legIK_params.Q[134])+rhs[8]*(2*legIK_params.Q[152])+rhs[9]*(2*legIK_params.Q[170])+rhs[10]*(2*legIK_params.Q[188])+rhs[11]*(2*legIK_params.Q[206])+rhs[12]*(2*legIK_params.Q[224])+rhs[13]*(2*legIK_params.Q[242])+rhs[14]*(2*legIK_params.Q[260])+rhs[15]*(2*legIK_params.Q[278])+rhs[16]*(2*legIK_params.Q[296])+rhs[17]*(2*legIK_params.Q[314]);
  lhs[9] = rhs[0]*(2*legIK_params.Q[9])+rhs[1]*(2*legIK_params.Q[27])+rhs[2]*(2*legIK_params.Q[45])+rhs[3]*(2*legIK_params.Q[63])+rhs[4]*(2*legIK_params.Q[81])+rhs[5]*(2*legIK_params.Q[99])+rhs[6]*(2*legIK_params.Q[117])+rhs[7]*(2*legIK_params.Q[135])+rhs[8]*(2*legIK_params.Q[153])+rhs[9]*(2*legIK_params.Q[171])+rhs[10]*(2*legIK_params.Q[189])+rhs[11]*(2*legIK_params.Q[207])+rhs[12]*(2*legIK_params.Q[225])+rhs[13]*(2*legIK_params.Q[243])+rhs[14]*(2*legIK_params.Q[261])+rhs[15]*(2*legIK_params.Q[279])+rhs[16]*(2*legIK_params.Q[297])+rhs[17]*(2*legIK_params.Q[315]);
  lhs[10] = rhs[0]*(2*legIK_params.Q[10])+rhs[1]*(2*legIK_params.Q[28])+rhs[2]*(2*legIK_params.Q[46])+rhs[3]*(2*legIK_params.Q[64])+rhs[4]*(2*legIK_params.Q[82])+rhs[5]*(2*legIK_params.Q[100])+rhs[6]*(2*legIK_params.Q[118])+rhs[7]*(2*legIK_params.Q[136])+rhs[8]*(2*legIK_params.Q[154])+rhs[9]*(2*legIK_params.Q[172])+rhs[10]*(2*legIK_params.Q[190])+rhs[11]*(2*legIK_params.Q[208])+rhs[12]*(2*legIK_params.Q[226])+rhs[13]*(2*legIK_params.Q[244])+rhs[14]*(2*legIK_params.Q[262])+rhs[15]*(2*legIK_params.Q[280])+rhs[16]*(2*legIK_params.Q[298])+rhs[17]*(2*legIK_params.Q[316]);
  lhs[11] = rhs[0]*(2*legIK_params.Q[11])+rhs[1]*(2*legIK_params.Q[29])+rhs[2]*(2*legIK_params.Q[47])+rhs[3]*(2*legIK_params.Q[65])+rhs[4]*(2*legIK_params.Q[83])+rhs[5]*(2*legIK_params.Q[101])+rhs[6]*(2*legIK_params.Q[119])+rhs[7]*(2*legIK_params.Q[137])+rhs[8]*(2*legIK_params.Q[155])+rhs[9]*(2*legIK_params.Q[173])+rhs[10]*(2*legIK_params.Q[191])+rhs[11]*(2*legIK_params.Q[209])+rhs[12]*(2*legIK_params.Q[227])+rhs[13]*(2*legIK_params.Q[245])+rhs[14]*(2*legIK_params.Q[263])+rhs[15]*(2*legIK_params.Q[281])+rhs[16]*(2*legIK_params.Q[299])+rhs[17]*(2*legIK_params.Q[317]);
  lhs[12] = rhs[0]*(2*legIK_params.Q[12])+rhs[1]*(2*legIK_params.Q[30])+rhs[2]*(2*legIK_params.Q[48])+rhs[3]*(2*legIK_params.Q[66])+rhs[4]*(2*legIK_params.Q[84])+rhs[5]*(2*legIK_params.Q[102])+rhs[6]*(2*legIK_params.Q[120])+rhs[7]*(2*legIK_params.Q[138])+rhs[8]*(2*legIK_params.Q[156])+rhs[9]*(2*legIK_params.Q[174])+rhs[10]*(2*legIK_params.Q[192])+rhs[11]*(2*legIK_params.Q[210])+rhs[12]*(2*legIK_params.Q[228])+rhs[13]*(2*legIK_params.Q[246])+rhs[14]*(2*legIK_params.Q[264])+rhs[15]*(2*legIK_params.Q[282])+rhs[16]*(2*legIK_params.Q[300])+rhs[17]*(2*legIK_params.Q[318]);
  lhs[13] = rhs[0]*(2*legIK_params.Q[13])+rhs[1]*(2*legIK_params.Q[31])+rhs[2]*(2*legIK_params.Q[49])+rhs[3]*(2*legIK_params.Q[67])+rhs[4]*(2*legIK_params.Q[85])+rhs[5]*(2*legIK_params.Q[103])+rhs[6]*(2*legIK_params.Q[121])+rhs[7]*(2*legIK_params.Q[139])+rhs[8]*(2*legIK_params.Q[157])+rhs[9]*(2*legIK_params.Q[175])+rhs[10]*(2*legIK_params.Q[193])+rhs[11]*(2*legIK_params.Q[211])+rhs[12]*(2*legIK_params.Q[229])+rhs[13]*(2*legIK_params.Q[247])+rhs[14]*(2*legIK_params.Q[265])+rhs[15]*(2*legIK_params.Q[283])+rhs[16]*(2*legIK_params.Q[301])+rhs[17]*(2*legIK_params.Q[319]);
  lhs[14] = rhs[0]*(2*legIK_params.Q[14])+rhs[1]*(2*legIK_params.Q[32])+rhs[2]*(2*legIK_params.Q[50])+rhs[3]*(2*legIK_params.Q[68])+rhs[4]*(2*legIK_params.Q[86])+rhs[5]*(2*legIK_params.Q[104])+rhs[6]*(2*legIK_params.Q[122])+rhs[7]*(2*legIK_params.Q[140])+rhs[8]*(2*legIK_params.Q[158])+rhs[9]*(2*legIK_params.Q[176])+rhs[10]*(2*legIK_params.Q[194])+rhs[11]*(2*legIK_params.Q[212])+rhs[12]*(2*legIK_params.Q[230])+rhs[13]*(2*legIK_params.Q[248])+rhs[14]*(2*legIK_params.Q[266])+rhs[15]*(2*legIK_params.Q[284])+rhs[16]*(2*legIK_params.Q[302])+rhs[17]*(2*legIK_params.Q[320]);
  lhs[15] = rhs[0]*(2*legIK_params.Q[15])+rhs[1]*(2*legIK_params.Q[33])+rhs[2]*(2*legIK_params.Q[51])+rhs[3]*(2*legIK_params.Q[69])+rhs[4]*(2*legIK_params.Q[87])+rhs[5]*(2*legIK_params.Q[105])+rhs[6]*(2*legIK_params.Q[123])+rhs[7]*(2*legIK_params.Q[141])+rhs[8]*(2*legIK_params.Q[159])+rhs[9]*(2*legIK_params.Q[177])+rhs[10]*(2*legIK_params.Q[195])+rhs[11]*(2*legIK_params.Q[213])+rhs[12]*(2*legIK_params.Q[231])+rhs[13]*(2*legIK_params.Q[249])+rhs[14]*(2*legIK_params.Q[267])+rhs[15]*(2*legIK_params.Q[285])+rhs[16]*(2*legIK_params.Q[303])+rhs[17]*(2*legIK_params.Q[321]);
  lhs[16] = rhs[0]*(2*legIK_params.Q[16])+rhs[1]*(2*legIK_params.Q[34])+rhs[2]*(2*legIK_params.Q[52])+rhs[3]*(2*legIK_params.Q[70])+rhs[4]*(2*legIK_params.Q[88])+rhs[5]*(2*legIK_params.Q[106])+rhs[6]*(2*legIK_params.Q[124])+rhs[7]*(2*legIK_params.Q[142])+rhs[8]*(2*legIK_params.Q[160])+rhs[9]*(2*legIK_params.Q[178])+rhs[10]*(2*legIK_params.Q[196])+rhs[11]*(2*legIK_params.Q[214])+rhs[12]*(2*legIK_params.Q[232])+rhs[13]*(2*legIK_params.Q[250])+rhs[14]*(2*legIK_params.Q[268])+rhs[15]*(2*legIK_params.Q[286])+rhs[16]*(2*legIK_params.Q[304])+rhs[17]*(2*legIK_params.Q[322]);
  lhs[17] = rhs[0]*(2*legIK_params.Q[17])+rhs[1]*(2*legIK_params.Q[35])+rhs[2]*(2*legIK_params.Q[53])+rhs[3]*(2*legIK_params.Q[71])+rhs[4]*(2*legIK_params.Q[89])+rhs[5]*(2*legIK_params.Q[107])+rhs[6]*(2*legIK_params.Q[125])+rhs[7]*(2*legIK_params.Q[143])+rhs[8]*(2*legIK_params.Q[161])+rhs[9]*(2*legIK_params.Q[179])+rhs[10]*(2*legIK_params.Q[197])+rhs[11]*(2*legIK_params.Q[215])+rhs[12]*(2*legIK_params.Q[233])+rhs[13]*(2*legIK_params.Q[251])+rhs[14]*(2*legIK_params.Q[269])+rhs[15]*(2*legIK_params.Q[287])+rhs[16]*(2*legIK_params.Q[305])+rhs[17]*(2*legIK_params.Q[323]);
}
void legIK_fillq(void) {
  legIK_work.q[0] = legIK_params.P[0];
  legIK_work.q[1] = legIK_params.P[1];
  legIK_work.q[2] = legIK_params.P[2];
  legIK_work.q[3] = legIK_params.P[3];
  legIK_work.q[4] = legIK_params.P[4];
  legIK_work.q[5] = legIK_params.P[5];
  legIK_work.q[6] = legIK_params.P[6];
  legIK_work.q[7] = legIK_params.P[7];
  legIK_work.q[8] = legIK_params.P[8];
  legIK_work.q[9] = legIK_params.P[9];
  legIK_work.q[10] = legIK_params.P[10];
  legIK_work.q[11] = legIK_params.P[11];
  legIK_work.q[12] = legIK_params.P[12];
  legIK_work.q[13] = legIK_params.P[13];
  legIK_work.q[14] = legIK_params.P[14];
  legIK_work.q[15] = legIK_params.P[15];
  legIK_work.q[16] = legIK_params.P[16];
  legIK_work.q[17] = legIK_params.P[17];
}
void legIK_fillh(void) {
  legIK_work.h[0] = legIK_params.deXc[0];
  legIK_work.h[1] = legIK_params.deXc[1];
  legIK_work.h[2] = legIK_params.deXc[2];
  legIK_work.h[3] = legIK_params.deXc[3];
  legIK_work.h[4] = legIK_params.deXc[4];
  legIK_work.h[5] = legIK_params.deXc[5];
  legIK_work.h[6] = -legIK_params.qmin[0];
  legIK_work.h[7] = -legIK_params.qmin[1];
  legIK_work.h[8] = -legIK_params.qmin[2];
  legIK_work.h[9] = -legIK_params.qmin[3];
  legIK_work.h[10] = -legIK_params.qmin[4];
  legIK_work.h[11] = -legIK_params.qmin[5];
  legIK_work.h[12] = -legIK_params.qmin[6];
  legIK_work.h[13] = -legIK_params.qmin[7];
  legIK_work.h[14] = -legIK_params.qmin[8];
  legIK_work.h[15] = -legIK_params.qmin[9];
  legIK_work.h[16] = -legIK_params.qmin[10];
  legIK_work.h[17] = -legIK_params.qmin[11];
  legIK_work.h[18] = legIK_params.qmax[0];
  legIK_work.h[19] = legIK_params.qmax[1];
  legIK_work.h[20] = legIK_params.qmax[2];
  legIK_work.h[21] = legIK_params.qmax[3];
  legIK_work.h[22] = legIK_params.qmax[4];
  legIK_work.h[23] = legIK_params.qmax[5];
  legIK_work.h[24] = legIK_params.qmax[6];
  legIK_work.h[25] = legIK_params.qmax[7];
  legIK_work.h[26] = legIK_params.qmax[8];
  legIK_work.h[27] = legIK_params.qmax[9];
  legIK_work.h[28] = legIK_params.qmax[10];
  legIK_work.h[29] = legIK_params.qmax[11];
}
void legIK_fillb(void) {
}
void legIK_pre_ops(void) {
}
