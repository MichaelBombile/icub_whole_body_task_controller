/* Produced by CVXGEN, 2020-12-10 23:07:19 -0500.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: cop_solver.h. */
/* Description: Header file with relevant definitions. */
#ifndef cop_SOLVER_H
#define cop_SOLVER_H
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
/* Space must be allocated somewhere (cop_testsolver.c, csolve.c or your own */
/* program) for the global variables cop_vars, cop_params, cop_work and cop_settings. */
/* At the bottom of this file, they are externed. */
#ifndef ZERO_LIBRARY_MODE
#include <math.h>
#define cop_pm(A, m, n) cop_printmatrix(#A, A, m, n, 1)
#endif
typedef struct cop_Params_t {
  double Q[4];
  double P[2];
  double NeJc[12];
  double deXc[6];
} cop_Params;
typedef struct cop_Vars_t {
  double *xc; /* 2 rows. */
} cop_Vars;
typedef struct cop_Workspace_t {
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
} cop_Workspace;
typedef struct cop_Settings_t {
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
} cop_Settings;
extern cop_Vars cop_vars;
extern cop_Params cop_params;
extern cop_Workspace cop_work;
extern cop_Settings cop_settings;
/* Function definitions in ldl.c: */
void cop_ldl_cop_solve(double *target, double *var);
void cop_ldl_factor(void);
double cop_check_factorization(void);
void cop_matrix_multiply(double *result, double *source);
double cop_check_residual(double *target, double *multiplicand);
void cop_fill_KKT(void);

/* Function definitions in matrix_support.c: */
void cop_multbymA(double *lhs, double *rhs);
void cop_multbymAT(double *lhs, double *rhs);
void cop_multbymG(double *lhs, double *rhs);
void cop_multbymGT(double *lhs, double *rhs);
void cop_multbyP(double *lhs, double *rhs);
void cop_fillq(void);
void cop_fillh(void);
void cop_fillb(void);
void cop_pre_ops(void);

/* Function definitions in solver.c: */
double cop_eval_gap(void);
void cop_set_defaults(void);
void cop_setup_pointers(void);
void cop_setup_indexing(void);
void cop_set_start(void);
double cop_eval_objv(void);
void cop_fillrhs_aff(void);
void cop_fillrhs_cc(void);
void cop_refine(double *target, double *var);
double cop_calc_ineq_resid_squared(void);
double cop_calc_eq_resid_squared(void);
void cop_better_start(void);
void cop_fillrhs_start(void);
long cop_solve(void);

/* Function definitions in cop_testsolver.c: */
int cop_main(int argc, char **argv);
void cop_load_default_data(void);

/* Function definitions in util.c: */
void cop_tic(void);
float cop_toc(void);
float cop_tocq(void);
void cop_printmatrix(char *name, double *A, int m, int n, int sparse);
double cop_unif(double lower, double upper);
float cop_ran1(long*idum, int reset);
float cop_randn_internal(long *idum, int reset);
double cop_randn(void);
void cop_reset_rand(void);

#endif
