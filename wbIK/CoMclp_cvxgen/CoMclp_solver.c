/* Produced by CVXGEN, 2020-10-04 06:20:49 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.c. */
/* Description: Main solver file. */
#include "CoMclp_solver.h"
double CoMclp_eval_gap(void) {
  int i;
  double gap;
  gap = 0;
  for (i = 0; i < 6; i++)
    gap += CoMclp_work.z[i]*CoMclp_work.s[i];
  return gap;
}
void CoMclp_set_defaults(void) {
  CoMclp_settings.resid_tol = 1e-6;
  CoMclp_settings.eps = 1e-4;
  CoMclp_settings.max_iters = 25;
  CoMclp_settings.refine_steps = 1;
  CoMclp_settings.s_init = 1;
  CoMclp_settings.z_init = 1;
  CoMclp_settings.debug = 0;
  CoMclp_settings.verbose = 1;
  CoMclp_settings.verbose_refinement = 0;
  CoMclp_settings.better_start = 1;
  CoMclp_settings.kkt_reg = 1e-7;
}
void CoMclp_setup_pointers(void) {
  CoMclp_work.y = CoMclp_work.x + 2;
  CoMclp_work.s = CoMclp_work.x + 2;
  CoMclp_work.z = CoMclp_work.x + 8;
  CoMclp_vars.xc = CoMclp_work.x + 0;
}
void CoMclp_setup_indexing(void) {
  CoMclp_setup_pointers();
}
void CoMclp_set_start(void) {
  int i;
  for (i = 0; i < 2; i++)
    CoMclp_work.x[i] = 0;
  for (i = 0; i < 0; i++)
    CoMclp_work.y[i] = 0;
  for (i = 0; i < 6; i++)
    CoMclp_work.s[i] = (CoMclp_work.h[i] > 0) ? CoMclp_work.h[i] : CoMclp_settings.s_init;
  for (i = 0; i < 6; i++)
    CoMclp_work.z[i] = CoMclp_settings.z_init;
}
double CoMclp_eval_objv(void) {
  int i;
  double objv;
  /* Borrow space in CoMclp_work.rhs. */
  CoMclp_multbyP(CoMclp_work.rhs, CoMclp_work.x);
  objv = 0;
  for (i = 0; i < 2; i++)
    objv += CoMclp_work.x[i]*CoMclp_work.rhs[i];
  objv *= 0.5;
  for (i = 0; i < 2; i++)
    objv += CoMclp_work.q[i]*CoMclp_work.x[i];
  objv += 0;
  return objv;
}
void CoMclp_fillrhs_aff(void) {
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = CoMclp_work.rhs;
  r2 = CoMclp_work.rhs + 2;
  r3 = CoMclp_work.rhs + 8;
  r4 = CoMclp_work.rhs + 14;
  /* r1 = -A^Ty - G^Tz - Px - q. */
  CoMclp_multbymAT(r1, CoMclp_work.y);
  CoMclp_multbymGT(CoMclp_work.buffer, CoMclp_work.z);
  for (i = 0; i < 2; i++)
    r1[i] += CoMclp_work.buffer[i];
  CoMclp_multbyP(CoMclp_work.buffer, CoMclp_work.x);
  for (i = 0; i < 2; i++)
    r1[i] -= CoMclp_work.buffer[i] + CoMclp_work.q[i];
  /* r2 = -z. */
  for (i = 0; i < 6; i++)
    r2[i] = -CoMclp_work.z[i];
  /* r3 = -Gx - s + h. */
  CoMclp_multbymG(r3, CoMclp_work.x);
  for (i = 0; i < 6; i++)
    r3[i] += -CoMclp_work.s[i] + CoMclp_work.h[i];
  /* r4 = -Ax + b. */
  CoMclp_multbymA(r4, CoMclp_work.x);
  for (i = 0; i < 0; i++)
    r4[i] += CoMclp_work.b[i];
}
void CoMclp_fillrhs_cc(void) {
  int i;
  double *r2;
  double *ds_aff, *dz_aff;
  double mu;
  double alpha;
  double sigma;
  double smu;
  double minval;
  r2 = CoMclp_work.rhs + 2;
  ds_aff = CoMclp_work.lhs_aff + 2;
  dz_aff = CoMclp_work.lhs_aff + 8;
  mu = 0;
  for (i = 0; i < 6; i++)
    mu += CoMclp_work.s[i]*CoMclp_work.z[i];
  /* Don't finish calculating mu quite yet. */
  /* Find min(min(ds./s), min(dz./z)). */
  minval = 0;
  for (i = 0; i < 6; i++)
    if (ds_aff[i] < minval*CoMclp_work.s[i])
      minval = ds_aff[i]/CoMclp_work.s[i];
  for (i = 0; i < 6; i++)
    if (dz_aff[i] < minval*CoMclp_work.z[i])
      minval = dz_aff[i]/CoMclp_work.z[i];
  /* Find alpha. */
  if (-1 < minval)
      alpha = 1;
  else
      alpha = -1/minval;
  sigma = 0;
  for (i = 0; i < 6; i++)
    sigma += (CoMclp_work.s[i] + alpha*ds_aff[i])*
      (CoMclp_work.z[i] + alpha*dz_aff[i]);
  sigma /= mu;
  sigma = sigma*sigma*sigma;
  /* Finish calculating mu now. */
  mu *= 0.16666666666666666;
  smu = sigma*mu;
  /* Fill-in the rhs. */
  for (i = 0; i < 2; i++)
    CoMclp_work.rhs[i] = 0;
  for (i = 8; i < 14; i++)
    CoMclp_work.rhs[i] = 0;
  for (i = 0; i < 6; i++)
    r2[i] = CoMclp_work.s_inv[i]*(smu - ds_aff[i]*dz_aff[i]);
}
void CoMclp_refine(double *target, double *var) {
  int i, j;
  double *residual = CoMclp_work.buffer;
  double norm2;
  double *new_var = CoMclp_work.buffer2;
  for (j = 0; j < CoMclp_settings.refine_steps; j++) {
    norm2 = 0;
    CoMclp_matrix_multiply(residual, var);
    for (i = 0; i < 14; i++) {
      residual[i] = residual[i] - target[i];
      norm2 += residual[i]*residual[i];
    }
#ifndef ZERO_LIBRARY_MODE
    if (CoMclp_settings.verbose_refinement) {
      if (j == 0)
        printf("Initial residual before refinement has norm squared %.6g.\n", norm2);
      else
        printf("After refinement we get squared norm %.6g.\n", norm2);
    }
#endif
    /* Solve to find new_var = KKT \ (target - A*var). */
    CoMclp_ldl_CoMclp_solve(residual, new_var);
    /* Update var += new_var, or var += KKT \ (target - A*var). */
    for (i = 0; i < 14; i++) {
      var[i] -= new_var[i];
    }
  }
#ifndef ZERO_LIBRARY_MODE
  if (CoMclp_settings.verbose_refinement) {
    /* Check the residual once more, but only if we're reporting it, since */
    /* it's expensive. */
    norm2 = 0;
    CoMclp_matrix_multiply(residual, var);
    for (i = 0; i < 14; i++) {
      residual[i] = residual[i] - target[i];
      norm2 += residual[i]*residual[i];
    }
    if (j == 0)
      printf("Initial residual before refinement has norm squared %.6g.\n", norm2);
    else
      printf("After refinement we get squared norm %.6g.\n", norm2);
  }
#endif
}
double CoMclp_calc_ineq_resid_squared(void) {
  /* Calculates the norm ||-Gx - s + h||. */
  double norm2_squared;
  int i;
  /* Find -Gx. */
  CoMclp_multbymG(CoMclp_work.buffer, CoMclp_work.x);
  /* Add -s + h. */
  for (i = 0; i < 6; i++)
    CoMclp_work.buffer[i] += -CoMclp_work.s[i] + CoMclp_work.h[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 6; i++)
    norm2_squared += CoMclp_work.buffer[i]*CoMclp_work.buffer[i];
  return norm2_squared;
}
double CoMclp_calc_eq_resid_squared(void) {
  /* Calculates the norm ||-Ax + b||. */
  double norm2_squared;
  int i;
  /* Find -Ax. */
  CoMclp_multbymA(CoMclp_work.buffer, CoMclp_work.x);
  /* Add +b. */
  for (i = 0; i < 0; i++)
    CoMclp_work.buffer[i] += CoMclp_work.b[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 0; i++)
    norm2_squared += CoMclp_work.buffer[i]*CoMclp_work.buffer[i];
  return norm2_squared;
}
void CoMclp_better_start(void) {
  /* Calculates a better starting point, using a similar approach to CVXOPT. */
  /* Not yet speed optimized. */
  int i;
  double *x, *s, *z, *y;
  double alpha;
  CoMclp_work.block_33[0] = -1;
  /* Make sure sinvz is 1 to make hijacked KKT system ok. */
  for (i = 0; i < 6; i++)
    CoMclp_work.s_inv_z[i] = 1;
  CoMclp_fill_KKT();
  CoMclp_ldl_factor();
  CoMclp_fillrhs_start();
  /* Borrow CoMclp_work.lhs_aff for the solution. */
  CoMclp_ldl_CoMclp_solve(CoMclp_work.rhs, CoMclp_work.lhs_aff);
  /* Don't do any refinement for now. Precision doesn't matter too much. */
  x = CoMclp_work.lhs_aff;
  s = CoMclp_work.lhs_aff + 2;
  z = CoMclp_work.lhs_aff + 8;
  y = CoMclp_work.lhs_aff + 14;
  /* Just set x and y as is. */
  for (i = 0; i < 2; i++)
    CoMclp_work.x[i] = x[i];
  for (i = 0; i < 0; i++)
    CoMclp_work.y[i] = y[i];
  /* Now complete the initialization. Start with s. */
  /* Must have alpha > max(z). */
  alpha = -1e99;
  for (i = 0; i < 6; i++)
    if (alpha < z[i])
      alpha = z[i];
  if (alpha < 0) {
    for (i = 0; i < 6; i++)
      CoMclp_work.s[i] = -z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 6; i++)
      CoMclp_work.s[i] = -z[i] + alpha;
  }
  /* Now initialize z. */
  /* Now must have alpha > max(-z). */
  alpha = -1e99;
  for (i = 0; i < 6; i++)
    if (alpha < -z[i])
      alpha = -z[i];
  if (alpha < 0) {
    for (i = 0; i < 6; i++)
      CoMclp_work.z[i] = z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 6; i++)
      CoMclp_work.z[i] = z[i] + alpha;
  }
}
void CoMclp_fillrhs_start(void) {
  /* Fill rhs with (-q, 0, h, b). */
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = CoMclp_work.rhs;
  r2 = CoMclp_work.rhs + 2;
  r3 = CoMclp_work.rhs + 8;
  r4 = CoMclp_work.rhs + 14;
  for (i = 0; i < 2; i++)
    r1[i] = -CoMclp_work.q[i];
  for (i = 0; i < 6; i++)
    r2[i] = 0;
  for (i = 0; i < 6; i++)
    r3[i] = CoMclp_work.h[i];
  for (i = 0; i < 0; i++)
    r4[i] = CoMclp_work.b[i];
}
long CoMclp_solve(void) {
  int i;
  int iter;
  double *dx, *ds, *dy, *dz;
  double minval;
  double alpha;
  CoMclp_work.converged = 0;
  CoMclp_setup_pointers();
  CoMclp_pre_ops();
#ifndef ZERO_LIBRARY_MODE
  if (CoMclp_settings.verbose)
    printf("iter     objv        gap       |Ax-b|    |Gx+s-h|    step\n");
#endif
  CoMclp_fillq();
  CoMclp_fillh();
  CoMclp_fillb();
  if (CoMclp_settings.better_start)
    CoMclp_better_start();
  else
    CoMclp_set_start();
  for (iter = 0; iter < CoMclp_settings.max_iters; iter++) {
    for (i = 0; i < 6; i++) {
      CoMclp_work.s_inv[i] = 1.0 / CoMclp_work.s[i];
      CoMclp_work.s_inv_z[i] = CoMclp_work.s_inv[i]*CoMclp_work.z[i];
    }
    CoMclp_work.block_33[0] = 0;
    CoMclp_fill_KKT();
    CoMclp_ldl_factor();
    /* Affine scaling directions. */
    CoMclp_fillrhs_aff();
    CoMclp_ldl_CoMclp_solve(CoMclp_work.rhs, CoMclp_work.lhs_aff);
    CoMclp_refine(CoMclp_work.rhs, CoMclp_work.lhs_aff);
    /* Centering plus corrector directions. */
    CoMclp_fillrhs_cc();
    CoMclp_ldl_CoMclp_solve(CoMclp_work.rhs, CoMclp_work.lhs_cc);
    CoMclp_refine(CoMclp_work.rhs, CoMclp_work.lhs_cc);
    /* Add the two together and store in aff. */
    for (i = 0; i < 14; i++)
      CoMclp_work.lhs_aff[i] += CoMclp_work.lhs_cc[i];
    /* Rename aff to reflect its new meaning. */
    dx = CoMclp_work.lhs_aff;
    ds = CoMclp_work.lhs_aff + 2;
    dz = CoMclp_work.lhs_aff + 8;
    dy = CoMclp_work.lhs_aff + 14;
    /* Find min(min(ds./s), min(dz./z)). */
    minval = 0;
    for (i = 0; i < 6; i++)
      if (ds[i] < minval*CoMclp_work.s[i])
        minval = ds[i]/CoMclp_work.s[i];
    for (i = 0; i < 6; i++)
      if (dz[i] < minval*CoMclp_work.z[i])
        minval = dz[i]/CoMclp_work.z[i];
    /* Find alpha. */
    if (-0.99 < minval)
      alpha = 1;
    else
      alpha = -0.99/minval;
    /* Update the primal and dual variables. */
    for (i = 0; i < 2; i++)
      CoMclp_work.x[i] += alpha*dx[i];
    for (i = 0; i < 6; i++)
      CoMclp_work.s[i] += alpha*ds[i];
    for (i = 0; i < 6; i++)
      CoMclp_work.z[i] += alpha*dz[i];
    for (i = 0; i < 0; i++)
      CoMclp_work.y[i] += alpha*dy[i];
    CoMclp_work.gap = CoMclp_eval_gap();
    CoMclp_work.eq_resid_squared = CoMclp_calc_eq_resid_squared();
    CoMclp_work.ineq_resid_squared = CoMclp_calc_ineq_resid_squared();
#ifndef ZERO_LIBRARY_MODE
    if (CoMclp_settings.verbose) {
      CoMclp_work.optval = CoMclp_eval_objv();
      printf("%3d   %10.3e  %9.2e  %9.2e  %9.2e  % 6.4f\n",
          iter+1, CoMclp_work.optval, CoMclp_work.gap, sqrt(CoMclp_work.eq_resid_squared),
          sqrt(CoMclp_work.ineq_resid_squared), alpha);
    }
#endif
    /* Test termination conditions. Requires optimality, and satisfied */
    /* constraints. */
    if (   (CoMclp_work.gap < CoMclp_settings.eps)
        && (CoMclp_work.eq_resid_squared <= CoMclp_settings.resid_tol*CoMclp_settings.resid_tol)
        && (CoMclp_work.ineq_resid_squared <= CoMclp_settings.resid_tol*CoMclp_settings.resid_tol)
       ) {
      CoMclp_work.converged = 1;
      CoMclp_work.optval = CoMclp_eval_objv();
      return iter+1;
    }
  }
  return iter;
}
