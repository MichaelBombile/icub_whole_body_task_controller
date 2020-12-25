/* Produced by CVXGEN, 2020-06-01 20:42:17 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.h. */
/* Description: Header file with relevant definitions. */
#ifndef SOLVER_H
#define SOLVER_H
/* Uncomment the next line to remove all library dependencies. */
/*#define ZERO_LIBRARY_MODE */
#ifdef MATLAB_MEX_FILE
/* Matlab functions. MATLAB_MEX_FILE will be defined by the mex compiler. */
/* If you are not using the mex compiler, this functionality will not intrude, */
/* as it will be completely disabled at compile-time. */
#include "mex.h"
#else
#ifndef ZERO_LIBRARY_MODE
#include <stdio.h>
#endif
#endif
/* Space must be allocated somewhere (testsolver.c, csolve.c or your own */
/* program) for the global variables vars, params, work and settings. */
/* At the bottom of this file, they are externed. */
#ifndef ZERO_LIBRARY_MODE
#include <math.h>
#define pm(A, m, n) printmatrix(#A, A, m, n, 1)
#endif
typedef struct Params_t {
  double Qx[35];
  double Qy[29];
  double Qz[12];
  double Qw[85];
  double M_1[35];
  double aL[1];
  double Jt_19[35];
  double aR[1];
  double Jt_25[35];
  double Jt_20[35];
  double Jt_26[35];
  double Jt_21[35];
  double Jt_27[35];
  double b1[35];
  double M_2[35];
  double M_3[35];
  double M_4[35];
  double Jt_22[35];
  double Jt_28[35];
  double Jt_23[35];
  double Jt_29[35];
  double Jt_24[35];
  double Jt_30[35];
  double M_5[35];
  double M_6[35];
  double M_7[35];
  double M_8[35];
  double M_9[35];
  double M_10[35];
  double M_11[35];
  double M_12[35];
  double M_13[35];
  double M_14[35];
  double M_15[35];
  double M_16[35];
  double M_17[35];
  double M_18[35];
  double M_19[35];
  double M_20[35];
  double M_21[35];
  double M_22[35];
  double M_23[35];
  double M_24[35];
  double M_25[35];
  double M_26[35];
  double M_27[35];
  double M_28[35];
  double M_29[35];
  double M_30[35];
  double M_31[35];
  double M_32[35];
  double M_33[35];
  double M_34[35];
  double M_35[35];
  double XJbL_1[6];
  double XJbR_1[6];
  double b2[6];
  double XJbL_2[6];
  double XJbR_2[6];
  double XJbL_3[6];
  double XJbR_3[6];
  double XJbL_4[6];
  double XJbR_4[6];
  double XJbL_5[6];
  double XJbR_5[6];
  double XJbL_6[6];
  double XJbR_6[6];
  double Jt_1[35];
  double b3[62];
  double Jt_2[35];
  double Jt_3[35];
  double Jt_4[35];
  double Jt_5[35];
  double Jt_6[35];
  double Jt_7[35];
  double Jt_8[35];
  double Jt_9[35];
  double Jt_10[35];
  double Jt_11[35];
  double Jt_12[35];
  double Jt_13[35];
  double Jt_14[35];
  double Jt_15[35];
  double Jt_16[35];
  double Jt_17[35];
  double Jt_18[35];
  double Jto_1[35];
  double Jto_2[35];
  double Jto_3[35];
  double copMxl_1[6];
  double copMxl_2[6];
  double copMxr_1[6];
  double copMxr_2[6];
  double aw[1];
  double dwd[1];
  double dt[1];
  double b4[12];
  double T_max[29];
  double v_max[29];
  double q_min[29];
  double q_max[29];
  double CL_1[6];
  double CL_2[6];
  double CL_3[6];
  double CL_4[6];
  double CL_5[6];
  double CL_6[6];
  double CL_7[6];
  double CL_8[6];
  double CL_9[6];
  double CL_10[6];
  double CL_11[6];
  double CR_1[6];
  double CR_2[6];
  double CR_3[6];
  double CR_4[6];
  double CR_5[6];
  double CR_6[6];
  double CR_7[6];
  double CR_8[6];
  double CR_9[6];
  double CR_10[6];
  double CR_11[6];
  double *M[36];
  double *Jt[31];
  double *XJbL[7];
  double *XJbR[7];
  double *Jto[4];
  double *copMxl[3];
  double *copMxr[3];
  double *CL[12];
  double *CR[12];
} Params;
typedef struct Vars_t {
  double *x; /* 35 rows. */
  double *y; /* 29 rows. */
  double *z; /* 12 rows. */
  double *w; /* 85 rows. */
} Vars;
typedef struct Workspace_t {
  double h[196];
  double s_inv[196];
  double s_inv_z[196];
  double b[120];
  double q[161];
  double rhs[673];
  double x[673];
  double *s;
  double *z;
  double *y;
  double lhs_aff[673];
  double lhs_cc[673];
  double buffer[673];
  double buffer2[673];
  double KKT[2786];
  double L[2937];
  double d[673];
  double v[673];
  double d_inv[673];
  double gap;
  double optval;
  double ineq_resid_squared;
  double eq_resid_squared;
  double block_33[1];
  /* Pre-op symbols. */
  int converged;
} Workspace;
typedef struct Settings_t {
  double resid_tol;
  double eps;
  int max_iters;
  int refine_steps;
  int better_start;
  /* Better start obviates the need for s_init and z_init. */
  double s_init;
  double z_init;
  int verbose;
  /* Show extra details of the iterative refinement steps. */
  int verbose_refinement;
  int debug;
  /* For regularization. Minimum value of abs(D_ii) in the kkt D factor. */
  double kkt_reg;
} Settings;
extern Vars vars;
extern Params params;
extern Workspace work;
extern Settings settings;
/* Function definitions in ldl.c: */
void ldl_solve(double *target, double *var);
void ldl_factor(void);
double check_factorization(void);
void matrix_multiply(double *result, double *source);
double check_residual(double *target, double *multiplicand);
void fill_KKT(void);

/* Function definitions in matrix_support.c: */
void multbymA(double *lhs, double *rhs);
void multbymAT(double *lhs, double *rhs);
void multbymG(double *lhs, double *rhs);
void multbymGT(double *lhs, double *rhs);
void multbyP(double *lhs, double *rhs);
void fillq(void);
void fillh(void);
void fillb(void);
void pre_ops(void);

/* Function definitions in solver.c: */
double eval_gap(void);
void set_defaults(void);
void setup_pointers(void);
void setup_indexed_params(void);
void setup_indexing(void);
void set_start(void);
double eval_objv(void);
void fillrhs_aff(void);
void fillrhs_cc(void);
void refine(double *target, double *var);
double calc_ineq_resid_squared(void);
double calc_eq_resid_squared(void);
void better_start(void);
void fillrhs_start(void);
long solve(void);

/* Function definitions in testsolver.c: */
int main(int argc, char **argv);
void load_default_data(void);

/* Function definitions in util.c: */
void tic(void);
float toc(void);
float tocq(void);
void printmatrix(char *name, double *A, int m, int n, int sparse);
double unif(double lower, double upper);
float ran1(long*idum, int reset);
float randn_internal(long *idum, int reset);
double randn(void);
void reset_rand(void);

#endif
