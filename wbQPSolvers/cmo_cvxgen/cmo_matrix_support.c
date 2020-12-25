/* Produced by CVXGEN, 2019-04-21 11:08:24 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: matrix_support.c. */
/* Description: Support functions for matrix multiplication and vector filling. */
#include "cmo_solver.h"
void cmo_multbymA(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(-1)-rhs[14]*(cmo_params.Qam[0]*cmo_params.Jgm[0]+cmo_params.Qam[1]*cmo_params.Jgm[1]+cmo_params.Qam[2]*cmo_params.Jgm[2]+cmo_params.Qam[3]*cmo_params.Jgm[3]+cmo_params.Qam[4]*cmo_params.Jgm[4]+cmo_params.Qam[5]*cmo_params.Jgm[5])-rhs[15]*(cmo_params.Qam[0]*cmo_params.Jgm[6]+cmo_params.Qam[1]*cmo_params.Jgm[7]+cmo_params.Qam[2]*cmo_params.Jgm[8]+cmo_params.Qam[3]*cmo_params.Jgm[9]+cmo_params.Qam[4]*cmo_params.Jgm[10]+cmo_params.Qam[5]*cmo_params.Jgm[11])-rhs[16]*(cmo_params.Qam[0]*cmo_params.Jgm[12]+cmo_params.Qam[1]*cmo_params.Jgm[13]+cmo_params.Qam[2]*cmo_params.Jgm[14]+cmo_params.Qam[3]*cmo_params.Jgm[15]+cmo_params.Qam[4]*cmo_params.Jgm[16]+cmo_params.Qam[5]*cmo_params.Jgm[17])-rhs[17]*(cmo_params.Qam[0]*cmo_params.Jgm[18]+cmo_params.Qam[1]*cmo_params.Jgm[19]+cmo_params.Qam[2]*cmo_params.Jgm[20]+cmo_params.Qam[3]*cmo_params.Jgm[21]+cmo_params.Qam[4]*cmo_params.Jgm[22]+cmo_params.Qam[5]*cmo_params.Jgm[23])-rhs[18]*(cmo_params.Qam[0]*cmo_params.Jgm[24]+cmo_params.Qam[1]*cmo_params.Jgm[25]+cmo_params.Qam[2]*cmo_params.Jgm[26]+cmo_params.Qam[3]*cmo_params.Jgm[27]+cmo_params.Qam[4]*cmo_params.Jgm[28]+cmo_params.Qam[5]*cmo_params.Jgm[29])-rhs[19]*(cmo_params.Qam[0]*cmo_params.Jgm[30]+cmo_params.Qam[1]*cmo_params.Jgm[31]+cmo_params.Qam[2]*cmo_params.Jgm[32]+cmo_params.Qam[3]*cmo_params.Jgm[33]+cmo_params.Qam[4]*cmo_params.Jgm[34]+cmo_params.Qam[5]*cmo_params.Jgm[35])-rhs[20]*(cmo_params.Qam[0]*cmo_params.Jgm[36]+cmo_params.Qam[1]*cmo_params.Jgm[37]+cmo_params.Qam[2]*cmo_params.Jgm[38]+cmo_params.Qam[3]*cmo_params.Jgm[39]+cmo_params.Qam[4]*cmo_params.Jgm[40]+cmo_params.Qam[5]*cmo_params.Jgm[41])-rhs[21]*(cmo_params.Qam[0]*cmo_params.Jgm[42]+cmo_params.Qam[1]*cmo_params.Jgm[43]+cmo_params.Qam[2]*cmo_params.Jgm[44]+cmo_params.Qam[3]*cmo_params.Jgm[45]+cmo_params.Qam[4]*cmo_params.Jgm[46]+cmo_params.Qam[5]*cmo_params.Jgm[47])-rhs[22]*(cmo_params.Qam[0]*cmo_params.Jgm[48]+cmo_params.Qam[1]*cmo_params.Jgm[49]+cmo_params.Qam[2]*cmo_params.Jgm[50]+cmo_params.Qam[3]*cmo_params.Jgm[51]+cmo_params.Qam[4]*cmo_params.Jgm[52]+cmo_params.Qam[5]*cmo_params.Jgm[53])-rhs[23]*(cmo_params.Qam[0]*cmo_params.Jgm[54]+cmo_params.Qam[1]*cmo_params.Jgm[55]+cmo_params.Qam[2]*cmo_params.Jgm[56]+cmo_params.Qam[3]*cmo_params.Jgm[57]+cmo_params.Qam[4]*cmo_params.Jgm[58]+cmo_params.Qam[5]*cmo_params.Jgm[59])-rhs[24]*(cmo_params.Qam[0]*cmo_params.Jgm[60]+cmo_params.Qam[1]*cmo_params.Jgm[61]+cmo_params.Qam[2]*cmo_params.Jgm[62]+cmo_params.Qam[3]*cmo_params.Jgm[63]+cmo_params.Qam[4]*cmo_params.Jgm[64]+cmo_params.Qam[5]*cmo_params.Jgm[65])-rhs[25]*(cmo_params.Qam[0]*cmo_params.Jgm[66]+cmo_params.Qam[1]*cmo_params.Jgm[67]+cmo_params.Qam[2]*cmo_params.Jgm[68]+cmo_params.Qam[3]*cmo_params.Jgm[69]+cmo_params.Qam[4]*cmo_params.Jgm[70]+cmo_params.Qam[5]*cmo_params.Jgm[71]);
  lhs[1] = -rhs[1]*(-1)-rhs[14]*(cmo_params.Qrm[0]*cmo_params.Jgc[0]+cmo_params.Qrm[1]*cmo_params.Jgc[1]+cmo_params.Qrm[2]*cmo_params.Jgc[2]+cmo_params.Qrm[3]*cmo_params.Jgc[3]+cmo_params.Qrm[4]*cmo_params.Jgc[4]+cmo_params.Qrm[5]*cmo_params.Jgc[5]+cmo_params.Qrm[6]*cmo_params.Jgc[6]+cmo_params.Qrm[7]*cmo_params.Jgc[7]+cmo_params.Qrm[8]*cmo_params.Jgc[8]+cmo_params.Qrm[9]*cmo_params.Jgc[9]+cmo_params.Qrm[10]*cmo_params.Jgc[10]+cmo_params.Qrm[11]*cmo_params.Jgc[11])-rhs[15]*(cmo_params.Qrm[0]*cmo_params.Jgc[12]+cmo_params.Qrm[1]*cmo_params.Jgc[13]+cmo_params.Qrm[2]*cmo_params.Jgc[14]+cmo_params.Qrm[3]*cmo_params.Jgc[15]+cmo_params.Qrm[4]*cmo_params.Jgc[16]+cmo_params.Qrm[5]*cmo_params.Jgc[17]+cmo_params.Qrm[6]*cmo_params.Jgc[18]+cmo_params.Qrm[7]*cmo_params.Jgc[19]+cmo_params.Qrm[8]*cmo_params.Jgc[20]+cmo_params.Qrm[9]*cmo_params.Jgc[21]+cmo_params.Qrm[10]*cmo_params.Jgc[22]+cmo_params.Qrm[11]*cmo_params.Jgc[23])-rhs[16]*(cmo_params.Qrm[0]*cmo_params.Jgc[24]+cmo_params.Qrm[1]*cmo_params.Jgc[25]+cmo_params.Qrm[2]*cmo_params.Jgc[26]+cmo_params.Qrm[3]*cmo_params.Jgc[27]+cmo_params.Qrm[4]*cmo_params.Jgc[28]+cmo_params.Qrm[5]*cmo_params.Jgc[29]+cmo_params.Qrm[6]*cmo_params.Jgc[30]+cmo_params.Qrm[7]*cmo_params.Jgc[31]+cmo_params.Qrm[8]*cmo_params.Jgc[32]+cmo_params.Qrm[9]*cmo_params.Jgc[33]+cmo_params.Qrm[10]*cmo_params.Jgc[34]+cmo_params.Qrm[11]*cmo_params.Jgc[35])-rhs[17]*(cmo_params.Qrm[0]*cmo_params.Jgc[36]+cmo_params.Qrm[1]*cmo_params.Jgc[37]+cmo_params.Qrm[2]*cmo_params.Jgc[38]+cmo_params.Qrm[3]*cmo_params.Jgc[39]+cmo_params.Qrm[4]*cmo_params.Jgc[40]+cmo_params.Qrm[5]*cmo_params.Jgc[41]+cmo_params.Qrm[6]*cmo_params.Jgc[42]+cmo_params.Qrm[7]*cmo_params.Jgc[43]+cmo_params.Qrm[8]*cmo_params.Jgc[44]+cmo_params.Qrm[9]*cmo_params.Jgc[45]+cmo_params.Qrm[10]*cmo_params.Jgc[46]+cmo_params.Qrm[11]*cmo_params.Jgc[47])-rhs[18]*(cmo_params.Qrm[0]*cmo_params.Jgc[48]+cmo_params.Qrm[1]*cmo_params.Jgc[49]+cmo_params.Qrm[2]*cmo_params.Jgc[50]+cmo_params.Qrm[3]*cmo_params.Jgc[51]+cmo_params.Qrm[4]*cmo_params.Jgc[52]+cmo_params.Qrm[5]*cmo_params.Jgc[53]+cmo_params.Qrm[6]*cmo_params.Jgc[54]+cmo_params.Qrm[7]*cmo_params.Jgc[55]+cmo_params.Qrm[8]*cmo_params.Jgc[56]+cmo_params.Qrm[9]*cmo_params.Jgc[57]+cmo_params.Qrm[10]*cmo_params.Jgc[58]+cmo_params.Qrm[11]*cmo_params.Jgc[59])-rhs[19]*(cmo_params.Qrm[0]*cmo_params.Jgc[60]+cmo_params.Qrm[1]*cmo_params.Jgc[61]+cmo_params.Qrm[2]*cmo_params.Jgc[62]+cmo_params.Qrm[3]*cmo_params.Jgc[63]+cmo_params.Qrm[4]*cmo_params.Jgc[64]+cmo_params.Qrm[5]*cmo_params.Jgc[65]+cmo_params.Qrm[6]*cmo_params.Jgc[66]+cmo_params.Qrm[7]*cmo_params.Jgc[67]+cmo_params.Qrm[8]*cmo_params.Jgc[68]+cmo_params.Qrm[9]*cmo_params.Jgc[69]+cmo_params.Qrm[10]*cmo_params.Jgc[70]+cmo_params.Qrm[11]*cmo_params.Jgc[71])-rhs[20]*(cmo_params.Qrm[0]*cmo_params.Jgc[72]+cmo_params.Qrm[1]*cmo_params.Jgc[73]+cmo_params.Qrm[2]*cmo_params.Jgc[74]+cmo_params.Qrm[3]*cmo_params.Jgc[75]+cmo_params.Qrm[4]*cmo_params.Jgc[76]+cmo_params.Qrm[5]*cmo_params.Jgc[77]+cmo_params.Qrm[6]*cmo_params.Jgc[78]+cmo_params.Qrm[7]*cmo_params.Jgc[79]+cmo_params.Qrm[8]*cmo_params.Jgc[80]+cmo_params.Qrm[9]*cmo_params.Jgc[81]+cmo_params.Qrm[10]*cmo_params.Jgc[82]+cmo_params.Qrm[11]*cmo_params.Jgc[83])-rhs[21]*(cmo_params.Qrm[0]*cmo_params.Jgc[84]+cmo_params.Qrm[1]*cmo_params.Jgc[85]+cmo_params.Qrm[2]*cmo_params.Jgc[86]+cmo_params.Qrm[3]*cmo_params.Jgc[87]+cmo_params.Qrm[4]*cmo_params.Jgc[88]+cmo_params.Qrm[5]*cmo_params.Jgc[89]+cmo_params.Qrm[6]*cmo_params.Jgc[90]+cmo_params.Qrm[7]*cmo_params.Jgc[91]+cmo_params.Qrm[8]*cmo_params.Jgc[92]+cmo_params.Qrm[9]*cmo_params.Jgc[93]+cmo_params.Qrm[10]*cmo_params.Jgc[94]+cmo_params.Qrm[11]*cmo_params.Jgc[95])-rhs[22]*(cmo_params.Qrm[0]*cmo_params.Jgc[96]+cmo_params.Qrm[1]*cmo_params.Jgc[97]+cmo_params.Qrm[2]*cmo_params.Jgc[98]+cmo_params.Qrm[3]*cmo_params.Jgc[99]+cmo_params.Qrm[4]*cmo_params.Jgc[100]+cmo_params.Qrm[5]*cmo_params.Jgc[101]+cmo_params.Qrm[6]*cmo_params.Jgc[102]+cmo_params.Qrm[7]*cmo_params.Jgc[103]+cmo_params.Qrm[8]*cmo_params.Jgc[104]+cmo_params.Qrm[9]*cmo_params.Jgc[105]+cmo_params.Qrm[10]*cmo_params.Jgc[106]+cmo_params.Qrm[11]*cmo_params.Jgc[107])-rhs[23]*(cmo_params.Qrm[0]*cmo_params.Jgc[108]+cmo_params.Qrm[1]*cmo_params.Jgc[109]+cmo_params.Qrm[2]*cmo_params.Jgc[110]+cmo_params.Qrm[3]*cmo_params.Jgc[111]+cmo_params.Qrm[4]*cmo_params.Jgc[112]+cmo_params.Qrm[5]*cmo_params.Jgc[113]+cmo_params.Qrm[6]*cmo_params.Jgc[114]+cmo_params.Qrm[7]*cmo_params.Jgc[115]+cmo_params.Qrm[8]*cmo_params.Jgc[116]+cmo_params.Qrm[9]*cmo_params.Jgc[117]+cmo_params.Qrm[10]*cmo_params.Jgc[118]+cmo_params.Qrm[11]*cmo_params.Jgc[119])-rhs[24]*(cmo_params.Qrm[0]*cmo_params.Jgc[120]+cmo_params.Qrm[1]*cmo_params.Jgc[121]+cmo_params.Qrm[2]*cmo_params.Jgc[122]+cmo_params.Qrm[3]*cmo_params.Jgc[123]+cmo_params.Qrm[4]*cmo_params.Jgc[124]+cmo_params.Qrm[5]*cmo_params.Jgc[125]+cmo_params.Qrm[6]*cmo_params.Jgc[126]+cmo_params.Qrm[7]*cmo_params.Jgc[127]+cmo_params.Qrm[8]*cmo_params.Jgc[128]+cmo_params.Qrm[9]*cmo_params.Jgc[129]+cmo_params.Qrm[10]*cmo_params.Jgc[130]+cmo_params.Qrm[11]*cmo_params.Jgc[131])-rhs[25]*(cmo_params.Qrm[0]*cmo_params.Jgc[132]+cmo_params.Qrm[1]*cmo_params.Jgc[133]+cmo_params.Qrm[2]*cmo_params.Jgc[134]+cmo_params.Qrm[3]*cmo_params.Jgc[135]+cmo_params.Qrm[4]*cmo_params.Jgc[136]+cmo_params.Qrm[5]*cmo_params.Jgc[137]+cmo_params.Qrm[6]*cmo_params.Jgc[138]+cmo_params.Qrm[7]*cmo_params.Jgc[139]+cmo_params.Qrm[8]*cmo_params.Jgc[140]+cmo_params.Qrm[9]*cmo_params.Jgc[141]+cmo_params.Qrm[10]*cmo_params.Jgc[142]+cmo_params.Qrm[11]*cmo_params.Jgc[143]);
}
void cmo_multbymAT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(-1);
  lhs[1] = -rhs[1]*(-1);
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
  lhs[14] = -rhs[0]*(cmo_params.Qam[0]*cmo_params.Jgm[0]+cmo_params.Qam[1]*cmo_params.Jgm[1]+cmo_params.Qam[2]*cmo_params.Jgm[2]+cmo_params.Qam[3]*cmo_params.Jgm[3]+cmo_params.Qam[4]*cmo_params.Jgm[4]+cmo_params.Qam[5]*cmo_params.Jgm[5])-rhs[1]*(cmo_params.Qrm[0]*cmo_params.Jgc[0]+cmo_params.Qrm[1]*cmo_params.Jgc[1]+cmo_params.Qrm[2]*cmo_params.Jgc[2]+cmo_params.Qrm[3]*cmo_params.Jgc[3]+cmo_params.Qrm[4]*cmo_params.Jgc[4]+cmo_params.Qrm[5]*cmo_params.Jgc[5]+cmo_params.Qrm[6]*cmo_params.Jgc[6]+cmo_params.Qrm[7]*cmo_params.Jgc[7]+cmo_params.Qrm[8]*cmo_params.Jgc[8]+cmo_params.Qrm[9]*cmo_params.Jgc[9]+cmo_params.Qrm[10]*cmo_params.Jgc[10]+cmo_params.Qrm[11]*cmo_params.Jgc[11]);
  lhs[15] = -rhs[0]*(cmo_params.Qam[0]*cmo_params.Jgm[6]+cmo_params.Qam[1]*cmo_params.Jgm[7]+cmo_params.Qam[2]*cmo_params.Jgm[8]+cmo_params.Qam[3]*cmo_params.Jgm[9]+cmo_params.Qam[4]*cmo_params.Jgm[10]+cmo_params.Qam[5]*cmo_params.Jgm[11])-rhs[1]*(cmo_params.Qrm[0]*cmo_params.Jgc[12]+cmo_params.Qrm[1]*cmo_params.Jgc[13]+cmo_params.Qrm[2]*cmo_params.Jgc[14]+cmo_params.Qrm[3]*cmo_params.Jgc[15]+cmo_params.Qrm[4]*cmo_params.Jgc[16]+cmo_params.Qrm[5]*cmo_params.Jgc[17]+cmo_params.Qrm[6]*cmo_params.Jgc[18]+cmo_params.Qrm[7]*cmo_params.Jgc[19]+cmo_params.Qrm[8]*cmo_params.Jgc[20]+cmo_params.Qrm[9]*cmo_params.Jgc[21]+cmo_params.Qrm[10]*cmo_params.Jgc[22]+cmo_params.Qrm[11]*cmo_params.Jgc[23]);
  lhs[16] = -rhs[0]*(cmo_params.Qam[0]*cmo_params.Jgm[12]+cmo_params.Qam[1]*cmo_params.Jgm[13]+cmo_params.Qam[2]*cmo_params.Jgm[14]+cmo_params.Qam[3]*cmo_params.Jgm[15]+cmo_params.Qam[4]*cmo_params.Jgm[16]+cmo_params.Qam[5]*cmo_params.Jgm[17])-rhs[1]*(cmo_params.Qrm[0]*cmo_params.Jgc[24]+cmo_params.Qrm[1]*cmo_params.Jgc[25]+cmo_params.Qrm[2]*cmo_params.Jgc[26]+cmo_params.Qrm[3]*cmo_params.Jgc[27]+cmo_params.Qrm[4]*cmo_params.Jgc[28]+cmo_params.Qrm[5]*cmo_params.Jgc[29]+cmo_params.Qrm[6]*cmo_params.Jgc[30]+cmo_params.Qrm[7]*cmo_params.Jgc[31]+cmo_params.Qrm[8]*cmo_params.Jgc[32]+cmo_params.Qrm[9]*cmo_params.Jgc[33]+cmo_params.Qrm[10]*cmo_params.Jgc[34]+cmo_params.Qrm[11]*cmo_params.Jgc[35]);
  lhs[17] = -rhs[0]*(cmo_params.Qam[0]*cmo_params.Jgm[18]+cmo_params.Qam[1]*cmo_params.Jgm[19]+cmo_params.Qam[2]*cmo_params.Jgm[20]+cmo_params.Qam[3]*cmo_params.Jgm[21]+cmo_params.Qam[4]*cmo_params.Jgm[22]+cmo_params.Qam[5]*cmo_params.Jgm[23])-rhs[1]*(cmo_params.Qrm[0]*cmo_params.Jgc[36]+cmo_params.Qrm[1]*cmo_params.Jgc[37]+cmo_params.Qrm[2]*cmo_params.Jgc[38]+cmo_params.Qrm[3]*cmo_params.Jgc[39]+cmo_params.Qrm[4]*cmo_params.Jgc[40]+cmo_params.Qrm[5]*cmo_params.Jgc[41]+cmo_params.Qrm[6]*cmo_params.Jgc[42]+cmo_params.Qrm[7]*cmo_params.Jgc[43]+cmo_params.Qrm[8]*cmo_params.Jgc[44]+cmo_params.Qrm[9]*cmo_params.Jgc[45]+cmo_params.Qrm[10]*cmo_params.Jgc[46]+cmo_params.Qrm[11]*cmo_params.Jgc[47]);
  lhs[18] = -rhs[0]*(cmo_params.Qam[0]*cmo_params.Jgm[24]+cmo_params.Qam[1]*cmo_params.Jgm[25]+cmo_params.Qam[2]*cmo_params.Jgm[26]+cmo_params.Qam[3]*cmo_params.Jgm[27]+cmo_params.Qam[4]*cmo_params.Jgm[28]+cmo_params.Qam[5]*cmo_params.Jgm[29])-rhs[1]*(cmo_params.Qrm[0]*cmo_params.Jgc[48]+cmo_params.Qrm[1]*cmo_params.Jgc[49]+cmo_params.Qrm[2]*cmo_params.Jgc[50]+cmo_params.Qrm[3]*cmo_params.Jgc[51]+cmo_params.Qrm[4]*cmo_params.Jgc[52]+cmo_params.Qrm[5]*cmo_params.Jgc[53]+cmo_params.Qrm[6]*cmo_params.Jgc[54]+cmo_params.Qrm[7]*cmo_params.Jgc[55]+cmo_params.Qrm[8]*cmo_params.Jgc[56]+cmo_params.Qrm[9]*cmo_params.Jgc[57]+cmo_params.Qrm[10]*cmo_params.Jgc[58]+cmo_params.Qrm[11]*cmo_params.Jgc[59]);
  lhs[19] = -rhs[0]*(cmo_params.Qam[0]*cmo_params.Jgm[30]+cmo_params.Qam[1]*cmo_params.Jgm[31]+cmo_params.Qam[2]*cmo_params.Jgm[32]+cmo_params.Qam[3]*cmo_params.Jgm[33]+cmo_params.Qam[4]*cmo_params.Jgm[34]+cmo_params.Qam[5]*cmo_params.Jgm[35])-rhs[1]*(cmo_params.Qrm[0]*cmo_params.Jgc[60]+cmo_params.Qrm[1]*cmo_params.Jgc[61]+cmo_params.Qrm[2]*cmo_params.Jgc[62]+cmo_params.Qrm[3]*cmo_params.Jgc[63]+cmo_params.Qrm[4]*cmo_params.Jgc[64]+cmo_params.Qrm[5]*cmo_params.Jgc[65]+cmo_params.Qrm[6]*cmo_params.Jgc[66]+cmo_params.Qrm[7]*cmo_params.Jgc[67]+cmo_params.Qrm[8]*cmo_params.Jgc[68]+cmo_params.Qrm[9]*cmo_params.Jgc[69]+cmo_params.Qrm[10]*cmo_params.Jgc[70]+cmo_params.Qrm[11]*cmo_params.Jgc[71]);
  lhs[20] = -rhs[0]*(cmo_params.Qam[0]*cmo_params.Jgm[36]+cmo_params.Qam[1]*cmo_params.Jgm[37]+cmo_params.Qam[2]*cmo_params.Jgm[38]+cmo_params.Qam[3]*cmo_params.Jgm[39]+cmo_params.Qam[4]*cmo_params.Jgm[40]+cmo_params.Qam[5]*cmo_params.Jgm[41])-rhs[1]*(cmo_params.Qrm[0]*cmo_params.Jgc[72]+cmo_params.Qrm[1]*cmo_params.Jgc[73]+cmo_params.Qrm[2]*cmo_params.Jgc[74]+cmo_params.Qrm[3]*cmo_params.Jgc[75]+cmo_params.Qrm[4]*cmo_params.Jgc[76]+cmo_params.Qrm[5]*cmo_params.Jgc[77]+cmo_params.Qrm[6]*cmo_params.Jgc[78]+cmo_params.Qrm[7]*cmo_params.Jgc[79]+cmo_params.Qrm[8]*cmo_params.Jgc[80]+cmo_params.Qrm[9]*cmo_params.Jgc[81]+cmo_params.Qrm[10]*cmo_params.Jgc[82]+cmo_params.Qrm[11]*cmo_params.Jgc[83]);
  lhs[21] = -rhs[0]*(cmo_params.Qam[0]*cmo_params.Jgm[42]+cmo_params.Qam[1]*cmo_params.Jgm[43]+cmo_params.Qam[2]*cmo_params.Jgm[44]+cmo_params.Qam[3]*cmo_params.Jgm[45]+cmo_params.Qam[4]*cmo_params.Jgm[46]+cmo_params.Qam[5]*cmo_params.Jgm[47])-rhs[1]*(cmo_params.Qrm[0]*cmo_params.Jgc[84]+cmo_params.Qrm[1]*cmo_params.Jgc[85]+cmo_params.Qrm[2]*cmo_params.Jgc[86]+cmo_params.Qrm[3]*cmo_params.Jgc[87]+cmo_params.Qrm[4]*cmo_params.Jgc[88]+cmo_params.Qrm[5]*cmo_params.Jgc[89]+cmo_params.Qrm[6]*cmo_params.Jgc[90]+cmo_params.Qrm[7]*cmo_params.Jgc[91]+cmo_params.Qrm[8]*cmo_params.Jgc[92]+cmo_params.Qrm[9]*cmo_params.Jgc[93]+cmo_params.Qrm[10]*cmo_params.Jgc[94]+cmo_params.Qrm[11]*cmo_params.Jgc[95]);
  lhs[22] = -rhs[0]*(cmo_params.Qam[0]*cmo_params.Jgm[48]+cmo_params.Qam[1]*cmo_params.Jgm[49]+cmo_params.Qam[2]*cmo_params.Jgm[50]+cmo_params.Qam[3]*cmo_params.Jgm[51]+cmo_params.Qam[4]*cmo_params.Jgm[52]+cmo_params.Qam[5]*cmo_params.Jgm[53])-rhs[1]*(cmo_params.Qrm[0]*cmo_params.Jgc[96]+cmo_params.Qrm[1]*cmo_params.Jgc[97]+cmo_params.Qrm[2]*cmo_params.Jgc[98]+cmo_params.Qrm[3]*cmo_params.Jgc[99]+cmo_params.Qrm[4]*cmo_params.Jgc[100]+cmo_params.Qrm[5]*cmo_params.Jgc[101]+cmo_params.Qrm[6]*cmo_params.Jgc[102]+cmo_params.Qrm[7]*cmo_params.Jgc[103]+cmo_params.Qrm[8]*cmo_params.Jgc[104]+cmo_params.Qrm[9]*cmo_params.Jgc[105]+cmo_params.Qrm[10]*cmo_params.Jgc[106]+cmo_params.Qrm[11]*cmo_params.Jgc[107]);
  lhs[23] = -rhs[0]*(cmo_params.Qam[0]*cmo_params.Jgm[54]+cmo_params.Qam[1]*cmo_params.Jgm[55]+cmo_params.Qam[2]*cmo_params.Jgm[56]+cmo_params.Qam[3]*cmo_params.Jgm[57]+cmo_params.Qam[4]*cmo_params.Jgm[58]+cmo_params.Qam[5]*cmo_params.Jgm[59])-rhs[1]*(cmo_params.Qrm[0]*cmo_params.Jgc[108]+cmo_params.Qrm[1]*cmo_params.Jgc[109]+cmo_params.Qrm[2]*cmo_params.Jgc[110]+cmo_params.Qrm[3]*cmo_params.Jgc[111]+cmo_params.Qrm[4]*cmo_params.Jgc[112]+cmo_params.Qrm[5]*cmo_params.Jgc[113]+cmo_params.Qrm[6]*cmo_params.Jgc[114]+cmo_params.Qrm[7]*cmo_params.Jgc[115]+cmo_params.Qrm[8]*cmo_params.Jgc[116]+cmo_params.Qrm[9]*cmo_params.Jgc[117]+cmo_params.Qrm[10]*cmo_params.Jgc[118]+cmo_params.Qrm[11]*cmo_params.Jgc[119]);
  lhs[24] = -rhs[0]*(cmo_params.Qam[0]*cmo_params.Jgm[60]+cmo_params.Qam[1]*cmo_params.Jgm[61]+cmo_params.Qam[2]*cmo_params.Jgm[62]+cmo_params.Qam[3]*cmo_params.Jgm[63]+cmo_params.Qam[4]*cmo_params.Jgm[64]+cmo_params.Qam[5]*cmo_params.Jgm[65])-rhs[1]*(cmo_params.Qrm[0]*cmo_params.Jgc[120]+cmo_params.Qrm[1]*cmo_params.Jgc[121]+cmo_params.Qrm[2]*cmo_params.Jgc[122]+cmo_params.Qrm[3]*cmo_params.Jgc[123]+cmo_params.Qrm[4]*cmo_params.Jgc[124]+cmo_params.Qrm[5]*cmo_params.Jgc[125]+cmo_params.Qrm[6]*cmo_params.Jgc[126]+cmo_params.Qrm[7]*cmo_params.Jgc[127]+cmo_params.Qrm[8]*cmo_params.Jgc[128]+cmo_params.Qrm[9]*cmo_params.Jgc[129]+cmo_params.Qrm[10]*cmo_params.Jgc[130]+cmo_params.Qrm[11]*cmo_params.Jgc[131]);
  lhs[25] = -rhs[0]*(cmo_params.Qam[0]*cmo_params.Jgm[66]+cmo_params.Qam[1]*cmo_params.Jgm[67]+cmo_params.Qam[2]*cmo_params.Jgm[68]+cmo_params.Qam[3]*cmo_params.Jgm[69]+cmo_params.Qam[4]*cmo_params.Jgm[70]+cmo_params.Qam[5]*cmo_params.Jgm[71])-rhs[1]*(cmo_params.Qrm[0]*cmo_params.Jgc[132]+cmo_params.Qrm[1]*cmo_params.Jgc[133]+cmo_params.Qrm[2]*cmo_params.Jgc[134]+cmo_params.Qrm[3]*cmo_params.Jgc[135]+cmo_params.Qrm[4]*cmo_params.Jgc[136]+cmo_params.Qrm[5]*cmo_params.Jgc[137]+cmo_params.Qrm[6]*cmo_params.Jgc[138]+cmo_params.Qrm[7]*cmo_params.Jgc[139]+cmo_params.Qrm[8]*cmo_params.Jgc[140]+cmo_params.Qrm[9]*cmo_params.Jgc[141]+cmo_params.Qrm[10]*cmo_params.Jgc[142]+cmo_params.Qrm[11]*cmo_params.Jgc[143]);
}
void cmo_multbymG(double *lhs, double *rhs) {
  lhs[0] = -rhs[2]*(-1)-rhs[14]*(1);
  lhs[1] = -rhs[3]*(-1)-rhs[15]*(1);
  lhs[2] = -rhs[4]*(-1)-rhs[16]*(1);
  lhs[3] = -rhs[5]*(-1)-rhs[17]*(1);
  lhs[4] = -rhs[6]*(-1)-rhs[18]*(1);
  lhs[5] = -rhs[7]*(-1)-rhs[19]*(1);
  lhs[6] = -rhs[8]*(-1)-rhs[20]*(1);
  lhs[7] = -rhs[9]*(-1)-rhs[21]*(1);
  lhs[8] = -rhs[10]*(-1)-rhs[22]*(1);
  lhs[9] = -rhs[11]*(-1)-rhs[23]*(1);
  lhs[10] = -rhs[12]*(-1)-rhs[24]*(1);
  lhs[11] = -rhs[13]*(-1)-rhs[25]*(1);
  lhs[12] = -rhs[2]*(-1)-rhs[14]*(-1);
  lhs[13] = -rhs[3]*(-1)-rhs[15]*(-1);
  lhs[14] = -rhs[4]*(-1)-rhs[16]*(-1);
  lhs[15] = -rhs[5]*(-1)-rhs[17]*(-1);
  lhs[16] = -rhs[6]*(-1)-rhs[18]*(-1);
  lhs[17] = -rhs[7]*(-1)-rhs[19]*(-1);
  lhs[18] = -rhs[8]*(-1)-rhs[20]*(-1);
  lhs[19] = -rhs[9]*(-1)-rhs[21]*(-1);
  lhs[20] = -rhs[10]*(-1)-rhs[22]*(-1);
  lhs[21] = -rhs[11]*(-1)-rhs[23]*(-1);
  lhs[22] = -rhs[12]*(-1)-rhs[24]*(-1);
  lhs[23] = -rhs[13]*(-1)-rhs[25]*(-1);
}
void cmo_multbymGT(double *lhs, double *rhs) {
  lhs[0] = 0;
  lhs[1] = 0;
  lhs[2] = -rhs[0]*(-1)-rhs[12]*(-1);
  lhs[3] = -rhs[1]*(-1)-rhs[13]*(-1);
  lhs[4] = -rhs[2]*(-1)-rhs[14]*(-1);
  lhs[5] = -rhs[3]*(-1)-rhs[15]*(-1);
  lhs[6] = -rhs[4]*(-1)-rhs[16]*(-1);
  lhs[7] = -rhs[5]*(-1)-rhs[17]*(-1);
  lhs[8] = -rhs[6]*(-1)-rhs[18]*(-1);
  lhs[9] = -rhs[7]*(-1)-rhs[19]*(-1);
  lhs[10] = -rhs[8]*(-1)-rhs[20]*(-1);
  lhs[11] = -rhs[9]*(-1)-rhs[21]*(-1);
  lhs[12] = -rhs[10]*(-1)-rhs[22]*(-1);
  lhs[13] = -rhs[11]*(-1)-rhs[23]*(-1);
  lhs[14] = -rhs[0]*(1)-rhs[12]*(-1);
  lhs[15] = -rhs[1]*(1)-rhs[13]*(-1);
  lhs[16] = -rhs[2]*(1)-rhs[14]*(-1);
  lhs[17] = -rhs[3]*(1)-rhs[15]*(-1);
  lhs[18] = -rhs[4]*(1)-rhs[16]*(-1);
  lhs[19] = -rhs[5]*(1)-rhs[17]*(-1);
  lhs[20] = -rhs[6]*(1)-rhs[18]*(-1);
  lhs[21] = -rhs[7]*(1)-rhs[19]*(-1);
  lhs[22] = -rhs[8]*(1)-rhs[20]*(-1);
  lhs[23] = -rhs[9]*(1)-rhs[21]*(-1);
  lhs[24] = -rhs[10]*(1)-rhs[22]*(-1);
  lhs[25] = -rhs[11]*(1)-rhs[23]*(-1);
}
void cmo_multbyP(double *lhs, double *rhs) {
  /* TODO use the fact that P is symmetric? */
  /* TODO check doubling / half factor etc. */
  lhs[0] = rhs[0]*(2);
  lhs[1] = rhs[1]*(2);
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
  lhs[18] = 0;
  lhs[19] = 0;
  lhs[20] = 0;
  lhs[21] = 0;
  lhs[22] = 0;
  lhs[23] = 0;
  lhs[24] = 0;
  lhs[25] = 0;
}
void cmo_fillq(void) {
  cmo_work.q[0] = 0;
  cmo_work.q[1] = 0;
  cmo_work.q[2] = cmo_params.lambda[0];
  cmo_work.q[3] = cmo_params.lambda[0];
  cmo_work.q[4] = cmo_params.lambda[0];
  cmo_work.q[5] = cmo_params.lambda[0];
  cmo_work.q[6] = cmo_params.lambda[0];
  cmo_work.q[7] = cmo_params.lambda[0];
  cmo_work.q[8] = cmo_params.lambda[0];
  cmo_work.q[9] = cmo_params.lambda[0];
  cmo_work.q[10] = cmo_params.lambda[0];
  cmo_work.q[11] = cmo_params.lambda[0];
  cmo_work.q[12] = cmo_params.lambda[0];
  cmo_work.q[13] = cmo_params.lambda[0];
  cmo_work.q[14] = 0;
  cmo_work.q[15] = 0;
  cmo_work.q[16] = 0;
  cmo_work.q[17] = 0;
  cmo_work.q[18] = 0;
  cmo_work.q[19] = 0;
  cmo_work.q[20] = 0;
  cmo_work.q[21] = 0;
  cmo_work.q[22] = 0;
  cmo_work.q[23] = 0;
  cmo_work.q[24] = 0;
  cmo_work.q[25] = 0;
}
void cmo_fillh(void) {
  cmo_work.h[0] = 0;
  cmo_work.h[1] = 0;
  cmo_work.h[2] = 0;
  cmo_work.h[3] = 0;
  cmo_work.h[4] = 0;
  cmo_work.h[5] = 0;
  cmo_work.h[6] = 0;
  cmo_work.h[7] = 0;
  cmo_work.h[8] = 0;
  cmo_work.h[9] = 0;
  cmo_work.h[10] = 0;
  cmo_work.h[11] = 0;
  cmo_work.h[12] = 0;
  cmo_work.h[13] = 0;
  cmo_work.h[14] = 0;
  cmo_work.h[15] = 0;
  cmo_work.h[16] = 0;
  cmo_work.h[17] = 0;
  cmo_work.h[18] = 0;
  cmo_work.h[19] = 0;
  cmo_work.h[20] = 0;
  cmo_work.h[21] = 0;
  cmo_work.h[22] = 0;
  cmo_work.h[23] = 0;
}
void cmo_fillb(void) {
  cmo_work.b[0] = cmo_params.Qam[0]*cmo_params.bam[0]+cmo_params.Qam[1]*cmo_params.bam[1]+cmo_params.Qam[2]*cmo_params.bam[2]+cmo_params.Qam[3]*cmo_params.bam[3]+cmo_params.Qam[4]*cmo_params.bam[4]+cmo_params.Qam[5]*cmo_params.bam[5];
  cmo_work.b[1] = cmo_params.Qrm[0]*cmo_params.brm[0]+cmo_params.Qrm[1]*cmo_params.brm[1]+cmo_params.Qrm[2]*cmo_params.brm[2]+cmo_params.Qrm[3]*cmo_params.brm[3]+cmo_params.Qrm[4]*cmo_params.brm[4]+cmo_params.Qrm[5]*cmo_params.brm[5]+cmo_params.Qrm[6]*cmo_params.brm[6]+cmo_params.Qrm[7]*cmo_params.brm[7]+cmo_params.Qrm[8]*cmo_params.brm[8]+cmo_params.Qrm[9]*cmo_params.brm[9]+cmo_params.Qrm[10]*cmo_params.brm[10]+cmo_params.Qrm[11]*cmo_params.brm[11];
}
void cmo_pre_ops(void) {
}
