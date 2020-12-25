/* Produced by CVXGEN, 2020-05-01 15:31:44 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: wbIK1_solver.h. */
/* Description: Header file with relevant definitions. */
#ifndef wbIK1_SOLVER_H
#define wbIK1_SOLVER_H
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
/* Space must be allocated somewhere (wbIK1_testsolver.c, csolve.c or your own */
/* program) for the global variables wbIK1_vars, wbIK1_params, wbIK1_work and wbIK1_settings. */
/* At the bottom of this file, they are externed. */
#ifndef ZERO_LIBRARY_MODE
#include <math.h>
#define wbIK1_pm(A, m, n) wbIK1_printmatrix(#A, A, m, n, 1)
#endif
typedef struct wbIK1_Params_t {
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
  double qmin[29];
  double qmax[29];
  double vmax[29];
  double *NeJc[7];
} wbIK1_Params;
typedef struct wbIK1_Vars_t {
  double *qdot; /* 35 rows. */
} wbIK1_Vars;
typedef struct wbIK1_Workspace_t {
  double h[122];
  double s_inv[122];
  double s_inv_z[122];
  double *b;
  double q[35];
  double rhs[279];
  double x[279];
  double *s;
  double *z;
  double *y;
  double lhs_aff[279];
  double lhs_cc[279];
  double buffer[279];
  double buffer2[279];
  double KKT[1322];
  double L[1043];
  double d[279];
  double v[279];
  double d_inv[279];
  double gap;
  double optval;
  double ineq_resid_squared;
  double eq_resid_squared;
  double block_33[1];
  /* Pre-op symbols. */
  int converged;
} wbIK1_Workspace;
typedef struct wbIK1_Settings_t {
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
} wbIK1_Settings;
extern wbIK1_Vars wbIK1_vars;
extern wbIK1_Params wbIK1_params;
extern wbIK1_Workspace wbIK1_work;
extern wbIK1_Settings wbIK1_settings;
/* Function definitions in ldl.c: */
void wbIK1_ldl_wbIK1_solve(double *target, double *var);
void wbIK1_ldl_factor(void);
double wbIK1_check_factorization(void);
void wbIK1_matrix_multiply(double *result, double *source);
double wbIK1_check_residual(double *target, double *multiplicand);
void wbIK1_fill_KKT(void);

/* Function definitions in matrix_support.c: */
void wbIK1_multbymA(double *lhs, double *rhs);
void wbIK1_multbymAT(double *lhs, double *rhs);
void wbIK1_multbymG(double *lhs, double *rhs);
void wbIK1_multbymGT(double *lhs, double *rhs);
void wbIK1_multbyP(double *lhs, double *rhs);
void wbIK1_fillq(void);
void wbIK1_fillh(void);
void wbIK1_fillb(void);
void wbIK1_pre_ops(void);

/* Function definitions in solver.c: */
double wbIK1_eval_gap(void);
void wbIK1_set_defaults(void);
void wbIK1_setup_pointers(void);
void setup_indexed_wbIK1_params(void);
void wbIK1_setup_indexing(void);
void wbIK1_set_start(void);
double wbIK1_eval_objv(void);
void wbIK1_fillrhs_aff(void);
void wbIK1_fillrhs_cc(void);
void wbIK1_refine(double *target, double *var);
double wbIK1_calc_ineq_resid_squared(void);
double wbIK1_calc_eq_resid_squared(void);
void wbIK1_better_start(void);
void wbIK1_fillrhs_start(void);
long wbIK1_solve(void);

/* Function definitions in wbIK1_testsolver.c: */
int wbIK1_main(int argc, char **argv);
void wbIK1_load_default_data(void);

/* Function definitions in util.c: */
void wbIK1_tic(void);
float wbIK1_toc(void);
float wbIK1_tocq(void);
void wbIK1_printmatrix(char *name, double *A, int m, int n, int sparse);
double wbIK1_unif(double lower, double upper);
float wbIK1_ran1(long*idum, int reset);
float wbIK1_randn_internal(long *idum, int reset);
double wbIK1_randn(void);
void wbIK1_reset_rand(void);

#endif
