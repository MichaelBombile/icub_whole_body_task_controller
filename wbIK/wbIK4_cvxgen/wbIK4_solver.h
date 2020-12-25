/* Produced by CVXGEN, 2020-09-09 01:56:00 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: wbIK4_solver.h. */
/* Description: Header file with relevant definitions. */
#ifndef wbIK4_SOLVER_H
#define wbIK4_SOLVER_H
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
/* Space must be allocated somewhere (wbIK4_testsolver.c, csolve.c or your own */
/* program) for the global variables wbIK4_vars, wbIK4_params, wbIK4_work and wbIK4_settings. */
/* At the bottom of this file, they are externed. */
#ifndef ZERO_LIBRARY_MODE
#include <math.h>
#define wbIK4_pm(A, m, n) wbIK4_printmatrix(#A, A, m, n, 1)
#endif
typedef struct wbIK4_Params_t {
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
  double deXc2[6];
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
} wbIK4_Params;
typedef struct wbIK4_Vars_t {
  double *qdot; /* 35 rows. */
} wbIK4_Vars;
typedef struct wbIK4_Workspace_t {
  double h[75];
  double s_inv[75];
  double s_inv_z[75];
  double *b;
  double q[35];
  double rhs[185];
  double x[185];
  double *s;
  double *z;
  double *y;
  double lhs_aff[185];
  double lhs_cc[185];
  double buffer[185];
  double buffer2[185];
  double KKT[1508];
  double L[1323];
  double d[185];
  double v[185];
  double d_inv[185];
  double gap;
  double optval;
  double ineq_resid_squared;
  double eq_resid_squared;
  double block_33[1];
  /* Pre-op symbols. */
  int converged;
} wbIK4_Workspace;
typedef struct wbIK4_Settings_t {
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
} wbIK4_Settings;
extern wbIK4_Vars wbIK4_vars;
extern wbIK4_Params wbIK4_params;
extern wbIK4_Workspace wbIK4_work;
extern wbIK4_Settings wbIK4_settings;
/* Function definitions in ldl.c: */
void wbIK4_ldl_wbIK4_solve(double *target, double *var);
void wbIK4_ldl_factor(void);
double wbIK4_check_factorization(void);
void wbIK4_matrix_multiply(double *result, double *source);
double wbIK4_check_residual(double *target, double *multiplicand);
void wbIK4_fill_KKT(void);

/* Function definitions in matrix_support.c: */
void wbIK4_multbymA(double *lhs, double *rhs);
void wbIK4_multbymAT(double *lhs, double *rhs);
void wbIK4_multbymG(double *lhs, double *rhs);
void wbIK4_multbymGT(double *lhs, double *rhs);
void wbIK4_multbyP(double *lhs, double *rhs);
void wbIK4_fillq(void);
void wbIK4_fillh(void);
void wbIK4_fillb(void);
void wbIK4_pre_ops(void);

/* Function definitions in solver.c: */
double wbIK4_eval_gap(void);
void wbIK4_set_defaults(void);
void wbIK4_setup_pointers(void);
void setup_indexed_wbIK4_params(void);
void wbIK4_setup_indexing(void);
void wbIK4_set_start(void);
double wbIK4_eval_objv(void);
void wbIK4_fillrhs_aff(void);
void wbIK4_fillrhs_cc(void);
void wbIK4_refine(double *target, double *var);
double wbIK4_calc_ineq_resid_squared(void);
double wbIK4_calc_eq_resid_squared(void);
void wbIK4_better_start(void);
void wbIK4_fillrhs_start(void);
long wbIK4_solve(void);

/* Function definitions in wbIK4_testsolver.c: */
int wbIK4_main(int argc, char **argv);
void wbIK4_load_default_data(void);

/* Function definitions in util.c: */
void wbIK4_tic(void);
float wbIK4_toc(void);
float wbIK4_tocq(void);
void wbIK4_printmatrix(char *name, double *A, int m, int n, int sparse);
double wbIK4_unif(double lower, double upper);
float wbIK4_ran1(long*idum, int reset);
float wbIK4_randn_internal(long *idum, int reset);
double wbIK4_randn(void);
void wbIK4_reset_rand(void);

#endif
