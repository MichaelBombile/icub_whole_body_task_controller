/* Produced by CVXGEN, 2020-03-25 12:36:09 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: wbIK2_solver.h. */
/* Description: Header file with relevant definitions. */
#ifndef wbIK2_SOLVER_H
#define wbIK2_SOLVER_H
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
/* Space must be allocated somewhere (wbIK2_testsolver.c, csolve.c or your own */
/* program) for the global variables wbIK2_vars, wbIK2_params, wbIK2_work and wbIK2_settings. */
/* At the bottom of this file, they are externed. */
#ifndef ZERO_LIBRARY_MODE
#include <math.h>
#define wbIK2_pm(A, m, n) wbIK2_printmatrix(#A, A, m, n, 1)
#endif
typedef struct wbIK2_Params_t {
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
  double *NeJc[7];
} wbIK2_Params;
typedef struct wbIK2_Vars_t {
  double *qdot; /* 35 rows. */
} wbIK2_Vars;
typedef struct wbIK2_Workspace_t {
  double h[64];
  double s_inv[64];
  double s_inv_z[64];
  double *b;
  double q[35];
  double rhs[163];
  double x[163];
  double *s;
  double *z;
  double *y;
  double lhs_aff[163];
  double lhs_cc[163];
  double buffer[163];
  double buffer2[163];
  double KKT[1090];
  double L[927];
  double d[163];
  double v[163];
  double d_inv[163];
  double gap;
  double optval;
  double ineq_resid_squared;
  double eq_resid_squared;
  double block_33[1];
  /* Pre-op symbols. */
  int converged;
} wbIK2_Workspace;
typedef struct wbIK2_Settings_t {
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
} wbIK2_Settings;
extern wbIK2_Vars wbIK2_vars;
extern wbIK2_Params wbIK2_params;
extern wbIK2_Workspace wbIK2_work;
extern wbIK2_Settings wbIK2_settings;
/* Function definitions in ldl.c: */
void wbIK2_ldl_wbIK2_solve(double *target, double *var);
void wbIK2_ldl_factor(void);
double wbIK2_check_factorization(void);
void wbIK2_matrix_multiply(double *result, double *source);
double wbIK2_check_residual(double *target, double *multiplicand);
void wbIK2_fill_KKT(void);

/* Function definitions in matrix_support.c: */
void wbIK2_multbymA(double *lhs, double *rhs);
void wbIK2_multbymAT(double *lhs, double *rhs);
void wbIK2_multbymG(double *lhs, double *rhs);
void wbIK2_multbymGT(double *lhs, double *rhs);
void wbIK2_multbyP(double *lhs, double *rhs);
void wbIK2_fillq(void);
void wbIK2_fillh(void);
void wbIK2_fillb(void);
void wbIK2_pre_ops(void);

/* Function definitions in solver.c: */
double wbIK2_eval_gap(void);
void wbIK2_set_defaults(void);
void wbIK2_setup_pointers(void);
void setup_indexed_wbIK2_params(void);
void wbIK2_setup_indexing(void);
void wbIK2_set_start(void);
double wbIK2_eval_objv(void);
void wbIK2_fillrhs_aff(void);
void wbIK2_fillrhs_cc(void);
void wbIK2_refine(double *target, double *var);
double wbIK2_calc_ineq_resid_squared(void);
double wbIK2_calc_eq_resid_squared(void);
void wbIK2_better_start(void);
void wbIK2_fillrhs_start(void);
long wbIK2_solve(void);

/* Function definitions in wbIK2_testsolver.c: */
int wbIK2_main(int argc, char **argv);
void wbIK2_load_default_data(void);

/* Function definitions in util.c: */
void wbIK2_tic(void);
float wbIK2_toc(void);
float wbIK2_tocq(void);
void wbIK2_printmatrix(char *name, double *A, int m, int n, int sparse);
double wbIK2_unif(double lower, double upper);
float wbIK2_ran1(long*idum, int reset);
float wbIK2_randn_internal(long *idum, int reset);
double wbIK2_randn(void);
void wbIK2_reset_rand(void);

#endif
