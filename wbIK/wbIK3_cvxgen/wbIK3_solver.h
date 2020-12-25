/* Produced by CVXGEN, 2020-08-19 11:45:50 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: wbIK3_solver.h. */
/* Description: Header file with relevant definitions. */
#ifndef wbIK3_SOLVER_H
#define wbIK3_SOLVER_H
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
/* Space must be allocated somewhere (wbIK3_testsolver.c, csolve.c or your own */
/* program) for the global variables wbIK3_vars, wbIK3_params, wbIK3_work and wbIK3_settings. */
/* At the bottom of this file, they are externed. */
#ifndef ZERO_LIBRARY_MODE
#include <math.h>
#define wbIK3_pm(A, m, n) wbIK3_printmatrix(#A, A, m, n, 1)
#endif
typedef struct wbIK3_Params_t {
  double Q[1225];
  double P[35];
  double NeJc_1[35];
  double dt[1];
  double deXc[6];
  double NeJc_2[35];
  double NeJc_3[35];
  double NeJc_4[35];
  double NeJc_5[35];
  double NeJc_6[35];
  double sMDw_1[35];
  double sdw[5];
  double sMDw_2[35];
  double sMDw_3[35];
  double sMDw_4[35];
  double sMDw_5[35];
  double qmin[29];
  double qmax[29];
  double *NeJc[7];
  double *sMDw[6];
} wbIK3_Params;
typedef struct wbIK3_Vars_t {
  double *qdot; /* 35 rows. */
} wbIK3_Vars;
typedef struct wbIK3_Workspace_t {
  double h[69];
  double s_inv[69];
  double s_inv_z[69];
  double *b;
  double q[35];
  double rhs[173];
  double x[173];
  double *s;
  double *z;
  double *y;
  double lhs_aff[173];
  double lhs_cc[173];
  double buffer[173];
  double buffer2[173];
  double KKT[1280];
  double L[1107];
  double d[173];
  double v[173];
  double d_inv[173];
  double gap;
  double optval;
  double ineq_resid_squared;
  double eq_resid_squared;
  double block_33[1];
  /* Pre-op symbols. */
  int converged;
} wbIK3_Workspace;
typedef struct wbIK3_Settings_t {
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
} wbIK3_Settings;
extern wbIK3_Vars wbIK3_vars;
extern wbIK3_Params wbIK3_params;
extern wbIK3_Workspace wbIK3_work;
extern wbIK3_Settings wbIK3_settings;
/* Function definitions in ldl.c: */
void wbIK3_ldl_wbIK3_solve(double *target, double *var);
void wbIK3_ldl_factor(void);
double wbIK3_check_factorization(void);
void wbIK3_matrix_multiply(double *result, double *source);
double wbIK3_check_residual(double *target, double *multiplicand);
void wbIK3_fill_KKT(void);

/* Function definitions in matrix_support.c: */
void wbIK3_multbymA(double *lhs, double *rhs);
void wbIK3_multbymAT(double *lhs, double *rhs);
void wbIK3_multbymG(double *lhs, double *rhs);
void wbIK3_multbymGT(double *lhs, double *rhs);
void wbIK3_multbyP(double *lhs, double *rhs);
void wbIK3_fillq(void);
void wbIK3_fillh(void);
void wbIK3_fillb(void);
void wbIK3_pre_ops(void);

/* Function definitions in solver.c: */
double wbIK3_eval_gap(void);
void wbIK3_set_defaults(void);
void wbIK3_setup_pointers(void);
void setup_indexed_wbIK3_params(void);
void wbIK3_setup_indexing(void);
void wbIK3_set_start(void);
double wbIK3_eval_objv(void);
void wbIK3_fillrhs_aff(void);
void wbIK3_fillrhs_cc(void);
void wbIK3_refine(double *target, double *var);
double wbIK3_calc_ineq_resid_squared(void);
double wbIK3_calc_eq_resid_squared(void);
void wbIK3_better_start(void);
void wbIK3_fillrhs_start(void);
long wbIK3_solve(void);

/* Function definitions in wbIK3_testsolver.c: */
int wbIK3_main(int argc, char **argv);
void wbIK3_load_default_data(void);

/* Function definitions in util.c: */
void wbIK3_tic(void);
float wbIK3_toc(void);
float wbIK3_tocq(void);
void wbIK3_printmatrix(char *name, double *A, int m, int n, int sparse);
double wbIK3_unif(double lower, double upper);
float wbIK3_ran1(long*idum, int reset);
float wbIK3_randn_internal(long *idum, int reset);
double wbIK3_randn(void);
void wbIK3_reset_rand(void);

#endif
