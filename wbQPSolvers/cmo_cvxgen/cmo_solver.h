/* Produced by CVXGEN, 2019-04-21 11:08:24 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: cmo_solver.h. */
/* Description: Header file with relevant definitions. */
#ifndef cmo_SOLVER_H
#define cmo_SOLVER_H
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
/* Space must be allocated somewhere (cmo_testsolver.c, csolve.c or your own */
/* program) for the global variables cmo_vars, cmo_params, cmo_work and cmo_settings. */
/* At the bottom of this file, they are externed. */
#ifndef ZERO_LIBRARY_MODE
#include <math.h>
#define cmo_pm(A, m, n) cmo_printmatrix(#A, A, m, n, 1)
#endif
typedef struct cmo_Params_t {
  double Qam[6];
  double Jgm[72];
  double bam[6];
  double Qrm[12];
  double Jgc[144];
  double brm[12];
  double lambda[1];
} cmo_Params;
typedef struct cmo_Vars_t {
  double *t_01; /* 1 rows. */
  double *t_02; /* 1 rows. */
  double *t_03; /* 12 rows. */
  double *x; /* 12 rows. */
} cmo_Vars;
typedef struct cmo_Workspace_t {
  double h[24];
  double s_inv[24];
  double s_inv_z[24];
  double b[2];
  double q[26];
  double rhs[76];
  double x[76];
  double *s;
  double *z;
  double *y;
  double lhs_aff[76];
  double lhs_cc[76];
  double buffer[76];
  double buffer2[76];
  double KKT[148];
  double L[111];
  double d[76];
  double v[76];
  double d_inv[76];
  double gap;
  double optval;
  double ineq_resid_squared;
  double eq_resid_squared;
  double block_33[1];
  /* Pre-op symbols. */
  int converged;
} cmo_Workspace;
typedef struct cmo_Settings_t {
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
} cmo_Settings;
extern cmo_Vars cmo_vars;
extern cmo_Params cmo_params;
extern cmo_Workspace cmo_work;
extern cmo_Settings cmo_settings;
/* Function definitions in ldl.c: */
void cmo_ldl_cmo_solve(double *target, double *var);
void cmo_ldl_factor(void);
double cmo_check_factorization(void);
void cmo_matrix_multiply(double *result, double *source);
double cmo_check_residual(double *target, double *multiplicand);
void cmo_fill_KKT(void);

/* Function definitions in matrix_support.c: */
void cmo_multbymA(double *lhs, double *rhs);
void cmo_multbymAT(double *lhs, double *rhs);
void cmo_multbymG(double *lhs, double *rhs);
void cmo_multbymGT(double *lhs, double *rhs);
void cmo_multbyP(double *lhs, double *rhs);
void cmo_fillq(void);
void cmo_fillh(void);
void cmo_fillb(void);
void cmo_pre_ops(void);

/* Function definitions in solver.c: */
double cmo_eval_gap(void);
void cmo_set_defaults(void);
void cmo_setup_pointers(void);
void cmo_setup_indexing(void);
void cmo_set_start(void);
double cmo_eval_objv(void);
void cmo_fillrhs_aff(void);
void cmo_fillrhs_cc(void);
void cmo_refine(double *target, double *var);
double cmo_calc_ineq_resid_squared(void);
double cmo_calc_eq_resid_squared(void);
void cmo_better_start(void);
void cmo_fillrhs_start(void);
long cmo_solve(void);

/* Function definitions in cmo_testsolver.c: */
int cmo_main(int argc, char **argv);
void cmo_load_default_data(void);

/* Function definitions in util.c: */
void cmo_tic(void);
float cmo_toc(void);
float cmo_tocq(void);
void cmo_printmatrix(char *name, double *A, int m, int n, int sparse);
double cmo_unif(double lower, double upper);
float cmo_ran1(long*idum, int reset);
float cmo_randn_internal(long *idum, int reset);
double cmo_randn(void);
void cmo_reset_rand(void);

#endif
