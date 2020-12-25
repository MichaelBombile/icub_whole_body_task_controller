/* Produced by CVXGEN, 2019-04-27 12:39:37 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: wb2_solver.h. */
/* Description: Header file with relevant definitions. */
#ifndef wb2_SOLVER_H
#define wb2_SOLVER_H
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
/* Space must be allocated somewhere (wb2_testsolver.c, csolve.c or your own */
/* program) for the global variables wb2_vars, wb2_params, wb2_work and wb2_settings. */
/* At the bottom of this file, they are externed. */
#ifndef ZERO_LIBRARY_MODE
#include <math.h>
#define wb2_pm(A, m, n) wb2_printmatrix(#A, A, m, n, 1)
#endif
typedef struct wb2_Params_t {
  double Qx[35];
  double px[35];
  double Qy[29];
  double Qz[12];
  double Qw[6];
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
  double b3[36];
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
  double T_max[29];
  double v_max[29];
  double dt[1];
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
  double *CL[12];
  double *CR[12];
} wb2_Params;
typedef struct wb2_Vars_t {
  double *t_01; /* 1 rows. */
  double *t_02; /* 1 rows. */
  double *t_03; /* 1 rows. */
  double *t_04; /* 1 rows. */
  double *t_05; /* 1 rows. */
  double *t_06; /* 1 rows. */
  double *t_07; /* 1 rows. */
  double *t_08; /* 1 rows. */
  double *t_09; /* 1 rows. */
  double *t_10; /* 1 rows. */
  double *t_11; /* 1 rows. */
  double *t_12; /* 1 rows. */
  double *t_13; /* 1 rows. */
  double *t_14; /* 1 rows. */
  double *t_15; /* 1 rows. */
  double *t_16; /* 1 rows. */
  double *t_17; /* 1 rows. */
  double *t_18; /* 1 rows. */
  double *t_19; /* 1 rows. */
  double *t_20; /* 1 rows. */
  double *t_21; /* 1 rows. */
  double *t_22; /* 1 rows. */
  double *t_23; /* 1 rows. */
  double *t_24; /* 1 rows. */
  double *t_25; /* 1 rows. */
  double *t_26; /* 1 rows. */
  double *t_27; /* 1 rows. */
  double *t_28; /* 1 rows. */
  double *t_29; /* 1 rows. */
  double *y; /* 29 rows. */
  double *z; /* 12 rows. */
  double *w; /* 6 rows. */
  double *x; /* 35 rows. */
} wb2_Vars;
typedef struct wb2_Workspace_t {
  double h[196];
  double s_inv[196];
  double s_inv_z[196];
  double b[100];
  double q[111];
  double rhs[603];
  double x[603];
  double *s;
  double *z;
  double *y;
  double lhs_aff[603];
  double lhs_cc[603];
  double buffer[603];
  double buffer2[603];
  double KKT[2454];
  double L[2688];
  double d[603];
  double v[603];
  double d_inv[603];
  double gap;
  double optval;
  double ineq_resid_squared;
  double eq_resid_squared;
  double block_33[1];
  /* Pre-op symbols. */
  int converged;
} wb2_Workspace;
typedef struct wb2_Settings_t {
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
} wb2_Settings;
extern wb2_Vars wb2_vars;
extern wb2_Params wb2_params;
extern wb2_Workspace wb2_work;
extern wb2_Settings wb2_settings;
/* Function definitions in ldl.c: */
void wb2_ldl_wb2_solve(double *target, double *var);
void wb2_ldl_factor(void);
double wb2_check_factorization(void);
void wb2_matrix_multiply(double *result, double *source);
double wb2_check_residual(double *target, double *multiplicand);
void wb2_fill_KKT(void);

/* Function definitions in matrix_support.c: */
void wb2_multbymA(double *lhs, double *rhs);
void wb2_multbymAT(double *lhs, double *rhs);
void wb2_multbymG(double *lhs, double *rhs);
void wb2_multbymGT(double *lhs, double *rhs);
void wb2_multbyP(double *lhs, double *rhs);
void wb2_fillq(void);
void wb2_fillh(void);
void wb2_fillb(void);
void wb2_pre_ops(void);

/* Function definitions in solver.c: */
double wb2_eval_gap(void);
void wb2_set_defaults(void);
void wb2_setup_pointers(void);
void setup_indexed_wb2_params(void);
void wb2_setup_indexing(void);
void wb2_set_start(void);
double wb2_eval_objv(void);
void wb2_fillrhs_aff(void);
void wb2_fillrhs_cc(void);
void wb2_refine(double *target, double *var);
double wb2_calc_ineq_resid_squared(void);
double wb2_calc_eq_resid_squared(void);
void wb2_better_start(void);
void wb2_fillrhs_start(void);
long wb2_solve(void);

/* Function definitions in wb2_testsolver.c: */
int wb2_main(int argc, char **argv);
void wb2_load_default_data(void);

/* Function definitions in util.c: */
void wb2_tic(void);
float wb2_toc(void);
float wb2_tocq(void);
void wb2_printmatrix(char *name, double *A, int m, int n, int sparse);
double wb2_unif(double lower, double upper);
float wb2_ran1(long*idum, int reset);
float wb2_randn_internal(long *idum, int reset);
double wb2_randn(void);
void wb2_reset_rand(void);

#endif
