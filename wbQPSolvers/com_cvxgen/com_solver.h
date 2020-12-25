/* Produced by CVXGEN, 2020-12-10 23:07:19 -0500.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: com_solver.h. */
/* Description: Header file with relevant definitions. */
#ifndef com_SOLVER_H
#define com_SOLVER_H
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
/* Space must be allocated somewhere (com_testsolver.c, csolve.c or your own */
/* program) for the global variables com_vars, com_params, com_work and com_settings. */
/* At the bottom of this file, they are externed. */
#ifndef ZERO_LIBRARY_MODE
#include <math.h>
#define com_pm(A, m, n) com_printmatrix(#A, A, m, n, 1)
#endif
typedef struct com_Params_t {
  double Q[4];
  double P[2];
  double NeJc[12];
  double deXc[6];
} com_Params;
typedef struct com_Vars_t {
  double *xc; /* 2 rows. */
} com_Vars;
typedef struct com_Workspace_t {
  double h[6];
  double s_inv[6];
  double s_inv_z[6];
  double *b;
  double q[2];
  double rhs[14];
  double x[14];
  double *s;
  double *z;
  double *y;
  double lhs_aff[14];
  double lhs_cc[14];
  double buffer[14];
  double buffer2[14];
  double KKT[33];
  double L[19];
  double d[14];
  double v[14];
  double d_inv[14];
  double gap;
  double optval;
  double ineq_resid_squared;
  double eq_resid_squared;
  double block_33[1];
  /* Pre-op symbols. */
  int converged;
} com_Workspace;
typedef struct com_Settings_t {
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
} com_Settings;
extern com_Vars com_vars;
extern com_Params com_params;
extern com_Workspace com_work;
extern com_Settings com_settings;
/* Function definitions in ldl.c: */
void com_ldl_com_solve(double *target, double *var);
void com_ldl_factor(void);
double com_check_factorization(void);
void com_matrix_multiply(double *result, double *source);
double com_check_residual(double *target, double *multiplicand);
void com_fill_KKT(void);

/* Function definitions in matrix_support.c: */
void com_multbymA(double *lhs, double *rhs);
void com_multbymAT(double *lhs, double *rhs);
void com_multbymG(double *lhs, double *rhs);
void com_multbymGT(double *lhs, double *rhs);
void com_multbyP(double *lhs, double *rhs);
void com_fillq(void);
void com_fillh(void);
void com_fillb(void);
void com_pre_ops(void);

/* Function definitions in solver.c: */
double com_eval_gap(void);
void com_set_defaults(void);
void com_setup_pointers(void);
void com_setup_indexing(void);
void com_set_start(void);
double com_eval_objv(void);
void com_fillrhs_aff(void);
void com_fillrhs_cc(void);
void com_refine(double *target, double *var);
double com_calc_ineq_resid_squared(void);
double com_calc_eq_resid_squared(void);
void com_better_start(void);
void com_fillrhs_start(void);
long com_solve(void);

/* Function definitions in com_testsolver.c: */
int com_main(int argc, char **argv);
void com_load_default_data(void);

/* Function definitions in util.c: */
void com_tic(void);
float com_toc(void);
float com_tocq(void);
void com_printmatrix(char *name, double *A, int m, int n, int sparse);
double com_unif(double lower, double upper);
float com_ran1(long*idum, int reset);
float com_randn_internal(long *idum, int reset);
double com_randn(void);
void com_reset_rand(void);

#endif
