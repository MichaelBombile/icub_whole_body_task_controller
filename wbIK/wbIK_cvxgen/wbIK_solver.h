/* Produced by CVXGEN, 2020-02-23 10:59:16 -0500.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: wbIK_solver.h. */
/* Description: Header file with relevant definitions. */
#ifndef wbIK_SOLVER_H
#define wbIK_SOLVER_H
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
/* Space must be allocated somewhere (wbIK_testsolver.c, csolve.c or your own */
/* program) for the global variables wbIK_vars, wbIK_params, wbIK_work and wbIK_settings. */
/* At the bottom of this file, they are externed. */
#ifndef ZERO_LIBRARY_MODE
#include <math.h>
#define wbIK_pm(A, m, n) wbIK_printmatrix(#A, A, m, n, 1)
#endif
typedef struct wbIK_Params_t {
  double Qx[35];
  double px[35];
  double Qw[36];
  double Jt_1[35];
  double hard[36];
  double b1[36];
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
  double Jt_19[35];
  double Jt_20[35];
  double Jt_21[35];
  double Jt_22[35];
  double Jt_23[35];
  double Jt_24[35];
  double Jt_25[35];
  double Jt_26[35];
  double Jt_27[35];
  double Jt_28[35];
  double Jt_29[35];
  double Jt_30[35];
  double Jt_31[35];
  double Jt_32[35];
  double Jt_33[35];
  double Jt_34[35];
  double Jt_35[35];
  double Jt_36[35];
  double q_min[29];
  double dt[1];
  double q_max[29];
  double *Jt[37];
} wbIK_Params;
typedef struct wbIK_Vars_t {
  double *x; /* 35 rows. */
  double *w; /* 36 rows. */
} wbIK_Vars;
typedef struct wbIK_Workspace_t {
  double h[58];
  double s_inv[58];
  double s_inv_z[58];
  double b[36];
  double q[71];
  double rhs[223];
  double x[223];
  double *s;
  double *z;
  double *y;
  double lhs_aff[223];
  double lhs_cc[223];
  double buffer[223];
  double buffer2[223];
  double KKT[930];
  double L[998];
  double d[223];
  double v[223];
  double d_inv[223];
  double gap;
  double optval;
  double ineq_resid_squared;
  double eq_resid_squared;
  double block_33[1];
  /* Pre-op symbols. */
  double quad_441009807360[1];
  int converged;
} wbIK_Workspace;
typedef struct wbIK_Settings_t {
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
} wbIK_Settings;
extern wbIK_Vars wbIK_vars;
extern wbIK_Params wbIK_params;
extern wbIK_Workspace wbIK_work;
extern wbIK_Settings wbIK_settings;
/* Function definitions in ldl.c: */
void wbIK_ldl_wbIK_solve(double *target, double *var);
void wbIK_ldl_factor(void);
double wbIK_check_factorization(void);
void wbIK_matrix_multiply(double *result, double *source);
double wbIK_check_residual(double *target, double *multiplicand);
void wbIK_fill_KKT(void);

/* Function definitions in matrix_support.c: */
void wbIK_multbymA(double *lhs, double *rhs);
void wbIK_multbymAT(double *lhs, double *rhs);
void wbIK_multbymG(double *lhs, double *rhs);
void wbIK_multbymGT(double *lhs, double *rhs);
void wbIK_multbyP(double *lhs, double *rhs);
void wbIK_fillq(void);
void wbIK_fillh(void);
void wbIK_fillb(void);
void wbIK_pre_ops(void);

/* Function definitions in solver.c: */
double wbIK_eval_gap(void);
void wbIK_set_defaults(void);
void wbIK_setup_pointers(void);
void setup_indexed_wbIK_params(void);
void wbIK_setup_indexing(void);
void wbIK_set_start(void);
double wbIK_eval_objv(void);
void wbIK_fillrhs_aff(void);
void wbIK_fillrhs_cc(void);
void wbIK_refine(double *target, double *var);
double wbIK_calc_ineq_resid_squared(void);
double wbIK_calc_eq_resid_squared(void);
void wbIK_better_start(void);
void wbIK_fillrhs_start(void);
long wbIK_solve(void);

/* Function definitions in wbIK_testsolver.c: */
int wbIK_main(int argc, char **argv);
void wbIK_load_default_data(void);

/* Function definitions in util.c: */
void wbIK_tic(void);
float wbIK_toc(void);
float wbIK_tocq(void);
void wbIK_printmatrix(char *name, double *A, int m, int n, int sparse);
double wbIK_unif(double lower, double upper);
float wbIK_ran1(long*idum, int reset);
float wbIK_randn_internal(long *idum, int reset);
double wbIK_randn(void);
void wbIK_reset_rand(void);

#endif
