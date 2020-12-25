/* Produced by CVXGEN, 2020-10-04 06:20:49 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: CoMclp_solver.h. */
/* Description: Header file with relevant definitions. */
#ifndef CoMclp_SOLVER_H
#define CoMclp_SOLVER_H
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
/* Space must be allocated somewhere (CoMclp_testsolver.c, csolve.c or your own */
/* program) for the global variables CoMclp_vars, CoMclp_params, CoMclp_work and CoMclp_settings. */
/* At the bottom of this file, they are externed. */
#ifndef ZERO_LIBRARY_MODE
#include <math.h>
#define CoMclp_pm(A, m, n) CoMclp_printmatrix(#A, A, m, n, 1)
#endif
typedef struct CoMclp_Params_t {
  double Q[4];
  double P[2];
  double NeJc[12];
  double deXc[6];
} CoMclp_Params;
typedef struct CoMclp_Vars_t {
  double *xc; /* 2 rows. */
} CoMclp_Vars;
typedef struct CoMclp_Workspace_t {
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
} CoMclp_Workspace;
typedef struct CoMclp_Settings_t {
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
} CoMclp_Settings;
extern CoMclp_Vars CoMclp_vars;
extern CoMclp_Params CoMclp_params;
extern CoMclp_Workspace CoMclp_work;
extern CoMclp_Settings CoMclp_settings;
/* Function definitions in ldl.c: */
void CoMclp_ldl_CoMclp_solve(double *target, double *var);
void CoMclp_ldl_factor(void);
double CoMclp_check_factorization(void);
void CoMclp_matrix_multiply(double *result, double *source);
double CoMclp_check_residual(double *target, double *multiplicand);
void CoMclp_fill_KKT(void);

/* Function definitions in matrix_support.c: */
void CoMclp_multbymA(double *lhs, double *rhs);
void CoMclp_multbymAT(double *lhs, double *rhs);
void CoMclp_multbymG(double *lhs, double *rhs);
void CoMclp_multbymGT(double *lhs, double *rhs);
void CoMclp_multbyP(double *lhs, double *rhs);
void CoMclp_fillq(void);
void CoMclp_fillh(void);
void CoMclp_fillb(void);
void CoMclp_pre_ops(void);

/* Function definitions in solver.c: */
double CoMclp_eval_gap(void);
void CoMclp_set_defaults(void);
void CoMclp_setup_pointers(void);
void CoMclp_setup_indexing(void);
void CoMclp_set_start(void);
double CoMclp_eval_objv(void);
void CoMclp_fillrhs_aff(void);
void CoMclp_fillrhs_cc(void);
void CoMclp_refine(double *target, double *var);
double CoMclp_calc_ineq_resid_squared(void);
double CoMclp_calc_eq_resid_squared(void);
void CoMclp_better_start(void);
void CoMclp_fillrhs_start(void);
long CoMclp_solve(void);

/* Function definitions in CoMclp_testsolver.c: */
int CoMclp_main(int argc, char **argv);
void CoMclp_load_default_data(void);

/* Function definitions in util.c: */
void CoMclp_tic(void);
float CoMclp_toc(void);
float CoMclp_tocq(void);
void CoMclp_printmatrix(char *name, double *A, int m, int n, int sparse);
double CoMclp_unif(double lower, double upper);
float CoMclp_ran1(long*idum, int reset);
float CoMclp_randn_internal(long *idum, int reset);
double CoMclp_randn(void);
void CoMclp_reset_rand(void);

#endif
