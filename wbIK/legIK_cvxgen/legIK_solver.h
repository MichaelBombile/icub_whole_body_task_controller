/* Produced by CVXGEN, 2020-08-23 02:50:50 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: legIK_solver.h. */
/* Description: Header file with relevant definitions. */
#ifndef legIK_SOLVER_H
#define legIK_SOLVER_H
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
/* Space must be allocated somewhere (legIK_testsolver.c, csolve.c or your own */
/* program) for the global variables legIK_vars, legIK_params, legIK_work and legIK_settings. */
/* At the bottom of this file, they are externed. */
#ifndef ZERO_LIBRARY_MODE
#include <math.h>
#define legIK_pm(A, m, n) legIK_printmatrix(#A, A, m, n, 1)
#endif
typedef struct legIK_Params_t {
  double Q[324];
  double P[18];
  double NeJc_1[18];
  double dt[1];
  double deXc[6];
  double NeJc_2[18];
  double NeJc_3[18];
  double NeJc_4[18];
  double NeJc_5[18];
  double NeJc_6[18];
  double qmin[12];
  double qmax[12];
  double *NeJc[7];
} legIK_Params;
typedef struct legIK_Vars_t {
  double *qdot; /* 18 rows. */
} legIK_Vars;
typedef struct legIK_Workspace_t {
  double h[30];
  double s_inv[30];
  double s_inv_z[30];
  double *b;
  double q[18];
  double rhs[78];
  double x[78];
  double *s;
  double *z;
  double *y;
  double lhs_aff[78];
  double lhs_cc[78];
  double buffer[78];
  double buffer2[78];
  double KKT[393];
  double L[315];
  double d[78];
  double v[78];
  double d_inv[78];
  double gap;
  double optval;
  double ineq_resid_squared;
  double eq_resid_squared;
  double block_33[1];
  /* Pre-op symbols. */
  int converged;
} legIK_Workspace;
typedef struct legIK_Settings_t {
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
} legIK_Settings;
extern legIK_Vars legIK_vars;
extern legIK_Params legIK_params;
extern legIK_Workspace legIK_work;
extern legIK_Settings legIK_settings;
/* Function definitions in ldl.c: */
void legIK_ldl_legIK_solve(double *target, double *var);
void legIK_ldl_factor(void);
double legIK_check_factorization(void);
void legIK_matrix_multiply(double *result, double *source);
double legIK_check_residual(double *target, double *multiplicand);
void legIK_fill_KKT(void);

/* Function definitions in matrix_support.c: */
void legIK_multbymA(double *lhs, double *rhs);
void legIK_multbymAT(double *lhs, double *rhs);
void legIK_multbymG(double *lhs, double *rhs);
void legIK_multbymGT(double *lhs, double *rhs);
void legIK_multbyP(double *lhs, double *rhs);
void legIK_fillq(void);
void legIK_fillh(void);
void legIK_fillb(void);
void legIK_pre_ops(void);

/* Function definitions in solver.c: */
double legIK_eval_gap(void);
void legIK_set_defaults(void);
void legIK_setup_pointers(void);
void setup_indexed_legIK_params(void);
void legIK_setup_indexing(void);
void legIK_set_start(void);
double legIK_eval_objv(void);
void legIK_fillrhs_aff(void);
void legIK_fillrhs_cc(void);
void legIK_refine(double *target, double *var);
double legIK_calc_ineq_resid_squared(void);
double legIK_calc_eq_resid_squared(void);
void legIK_better_start(void);
void legIK_fillrhs_start(void);
long legIK_solve(void);

/* Function definitions in legIK_testsolver.c: */
int legIK_main(int argc, char **argv);
void legIK_load_default_data(void);

/* Function definitions in util.c: */
void legIK_tic(void);
float legIK_toc(void);
float legIK_tocq(void);
void legIK_printmatrix(char *name, double *A, int m, int n, int sparse);
double legIK_unif(double lower, double upper);
float legIK_ran1(long*idum, int reset);
float legIK_randn_internal(long *idum, int reset);
double legIK_randn(void);
void legIK_reset_rand(void);

#endif
