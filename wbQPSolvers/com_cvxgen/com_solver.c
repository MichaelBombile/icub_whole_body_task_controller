/* Produced by CVXGEN, 2020-12-10 23:07:19 -0500.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.c. */
/* Description: Main solver file. */
#include "com_solver.h"
double com_eval_gap(void) {
  int i;
  double gap;
  gap = 0;
  for (i = 0; i < 6; i++)
    gap += com_work.z[i]*com_work.s[i];
  return gap;
}
void com_set_defaults(void) {
  com_settings.resid_tol = 1e-6;
  com_settings.eps = 1e-4;
  com_settings.max_iters = 25;
  com_settings.refine_steps = 1;
  com_settings.s_init = 1;
  com_settings.z_init = 1;
  com_settings.debug = 0;
  com_settings.verbose = 1;
  com_settings.verbose_refinement = 0;
  com_settings.better_start = 1;
  com_settings.kkt_reg = 1e-7;
}
void com_setup_pointers(void) {
  com_work.y = com_work.x + 2;
  com_work.s = com_work.x + 2;
  com_work.z = com_work.x + 8;
  com_vars.xc = com_work.x + 0;
}
void com_setup_indexing(void) {
  com_setup_pointers();
}
void com_set_start(void) {
  int i;
  for (i = 0; i < 2; i++)
    com_work.x[i] = 0;
  for (i = 0; i < 0; i++)
    com_work.y[i] = 0;
  for (i = 0; i < 6; i++)
    com_work.s[i] = (com_work.h[i] > 0) ? com_work.h[i] : com_settings.s_init;
  for (i = 0; i < 6; i++)
    com_work.z[i] = com_settings.z_init;
}
double com_eval_objv(void) {
  int i;
  double objv;
  /* Borrow space in com_work.rhs. */
  com_multbyP(com_work.rhs, com_work.x);
  objv = 0;
  for (i = 0; i < 2; i++)
    objv += com_work.x[i]*com_work.rhs[i];
  objv *= 0.5;
  for (i = 0; i < 2; i++)
    objv += com_work.q[i]*com_work.x[i];
  objv += 0;
  return objv;
}
void com_fillrhs_aff(void) {
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = com_work.rhs;
  r2 = com_work.rhs + 2;
  r3 = com_work.rhs + 8;
  r4 = com_work.rhs + 14;
  /* r1 = -A^Ty - G^Tz - Px - q. */
  com_multbymAT(r1, com_work.y);
  com_multbymGT(com_work.buffer, com_work.z);
  for (i = 0; i < 2; i++)
    r1[i] += com_work.buffer[i];
  com_multbyP(com_work.buffer, com_work.x);
  for (i = 0; i < 2; i++)
    r1[i] -= com_work.buffer[i] + com_work.q[i];
  /* r2 = -z. */
  for (i = 0; i < 6; i++)
    r2[i] = -com_work.z[i];
  /* r3 = -Gx - s + h. */
  com_multbymG(r3, com_work.x);
  for (i = 0; i < 6; i++)
    r3[i] += -com_work.s[i] + com_work.h[i];
  /* r4 = -Ax + b. */
  com_multbymA(r4, com_work.x);
  for (i = 0; i < 0; i++)
    r4[i] += com_work.b[i];
}
void com_fillrhs_cc(void) {
  int i;
  double *r2;
  double *ds_aff, *dz_aff;
  double mu;
  double alpha;
  double sigma;
  double smu;
  double minval;
  r2 = com_work.rhs + 2;
  ds_aff = com_work.lhs_aff + 2;
  dz_aff = com_work.lhs_aff + 8;
  mu = 0;
  for (i = 0; i < 6; i++)
    mu += com_work.s[i]*com_work.z[i];
  /* Don't finish calculating mu quite yet. */
  /* Find min(min(ds./s), min(dz./z)). */
  minval = 0;
  for (i = 0; i < 6; i++)
    if (ds_aff[i] < minval*com_work.s[i])
      minval = ds_aff[i]/com_work.s[i];
  for (i = 0; i < 6; i++)
    if (dz_aff[i] < minval*com_work.z[i])
      minval = dz_aff[i]/com_work.z[i];
  /* Find alpha. */
  if (-1 < minval)
      alpha = 1;
  else
      alpha = -1/minval;
  sigma = 0;
  for (i = 0; i < 6; i++)
    sigma += (com_work.s[i] + alpha*ds_aff[i])*
      (com_work.z[i] + alpha*dz_aff[i]);
  sigma /= mu;
  sigma = sigma*sigma*sigma;
  /* Finish calculating mu now. */
  mu *= 0.16666666666666666;
  smu = sigma*mu;
  /* Fill-in the rhs. */
  for (i = 0; i < 2; i++)
    com_work.rhs[i] = 0;
  for (i = 8; i < 14; i++)
    com_work.rhs[i] = 0;
  for (i = 0; i < 6; i++)
    r2[i] = com_work.s_inv[i]*(smu - ds_aff[i]*dz_aff[i]);
}
void com_refine(double *target, double *var) {
  int i, j;
  double *residual = com_work.buffer;
  double norm2;
  double *new_var = com_work.buffer2;
  for (j = 0; j < com_settings.refine_steps; j++) {
    norm2 = 0;
    com_matrix_multiply(residual, var);
    for (i = 0; i < 14; i++) {
      residual[i] = residual[i] - target[i];
      norm2 += residual[i]*residual[i];
    }
#ifndef ZERO_LIBRARY_MODE
    if (com_settings.verbose_refinement) {
      if (j == 0)
        printf("Initial residual before refinement has norm squared %.6g.\n", norm2);
      else
        printf("After refinement we get squared norm %.6g.\n", norm2);
    }
#endif
    /* Solve to find new_var = KKT \ (target - A*var). */
    com_ldl_com_solve(residual, new_var);
    /* Update var += new_var, or var += KKT \ (target - A*var). */
    for (i = 0; i < 14; i++) {
      var[i] -= new_var[i];
    }
  }
#ifndef ZERO_LIBRARY_MODE
  if (com_settings.verbose_refinement) {
    /* Check the residual once more, but only if we're reporting it, since */
    /* it's expensive. */
    norm2 = 0;
    com_matrix_multiply(residual, var);
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
double com_calc_ineq_resid_squared(void) {
  /* Calculates the norm ||-Gx - s + h||. */
  double norm2_squared;
  int i;
  /* Find -Gx. */
  com_multbymG(com_work.buffer, com_work.x);
  /* Add -s + h. */
  for (i = 0; i < 6; i++)
    com_work.buffer[i] += -com_work.s[i] + com_work.h[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 6; i++)
    norm2_squared += com_work.buffer[i]*com_work.buffer[i];
  return norm2_squared;
}
double com_calc_eq_resid_squared(void) {
  /* Calculates the norm ||-Ax + b||. */
  double norm2_squared;
  int i;
  /* Find -Ax. */
  com_multbymA(com_work.buffer, com_work.x);
  /* Add +b. */
  for (i = 0; i < 0; i++)
    com_work.buffer[i] += com_work.b[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 0; i++)
    norm2_squared += com_work.buffer[i]*com_work.buffer[i];
  return norm2_squared;
}
void com_better_start(void) {
  /* Calculates a better starting point, using a similar approach to CVXOPT. */
  /* Not yet speed optimized. */
  int i;
  double *x, *s, *z, *y;
  double alpha;
  com_work.block_33[0] = -1;
  /* Make sure sinvz is 1 to make hijacked KKT system ok. */
  for (i = 0; i < 6; i++)
    com_work.s_inv_z[i] = 1;
  com_fill_KKT();
  com_ldl_factor();
  com_fillrhs_start();
  /* Borrow com_work.lhs_aff for the solution. */
  com_ldl_com_solve(com_work.rhs, com_work.lhs_aff);
  /* Don't do any refinement for now. Precision doesn't matter too much. */
  x = com_work.lhs_aff;
  s = com_work.lhs_aff + 2;
  z = com_work.lhs_aff + 8;
  y = com_work.lhs_aff + 14;
  /* Just set x and y as is. */
  for (i = 0; i < 2; i++)
    com_work.x[i] = x[i];
  for (i = 0; i < 0; i++)
    com_work.y[i] = y[i];
  /* Now complete the initialization. Start with s. */
  /* Must have alpha > max(z). */
  alpha = -1e99;
  for (i = 0; i < 6; i++)
    if (alpha < z[i])
      alpha = z[i];
  if (alpha < 0) {
    for (i = 0; i < 6; i++)
      com_work.s[i] = -z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 6; i++)
      com_work.s[i] = -z[i] + alpha;
  }
  /* Now initialize z. */
  /* Now must have alpha > max(-z). */
  alpha = -1e99;
  for (i = 0; i < 6; i++)
    if (alpha < -z[i])
      alpha = -z[i];
  if (alpha < 0) {
    for (i = 0; i < 6; i++)
      com_work.z[i] = z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 6; i++)
      com_work.z[i] = z[i] + alpha;
  }
}
void com_fillrhs_start(void) {
  /* Fill rhs with (-q, 0, h, b). */
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = com_work.rhs;
  r2 = com_work.rhs + 2;
  r3 = com_work.rhs + 8;
  r4 = com_work.rhs + 14;
  for (i = 0; i < 2; i++)
    r1[i] = -com_work.q[i];
  for (i = 0; i < 6; i++)
    r2[i] = 0;
  for (i = 0; i < 6; i++)
    r3[i] = com_work.h[i];
  for (i = 0; i < 0; i++)
    r4[i] = com_work.b[i];
}
long com_solve(void) {
  int i;
  int iter;
  double *dx, *ds, *dy, *dz;
  double minval;
  double alpha;
  com_work.converged = 0;
  com_setup_pointers();
  com_pre_ops();
#ifndef ZERO_LIBRARY_MODE
  if (com_settings.verbose)
    printf("iter     objv        gap       |Ax-b|    |Gx+s-h|    step\n");
#endif
  com_fillq();
  com_fillh();
  com_fillb();
  if (com_settings.better_start)
    com_better_start();
  else
    com_set_start();
  for (iter = 0; iter < com_settings.max_iters; iter++) {
    for (i = 0; i < 6; i++) {
      com_work.s_inv[i] = 1.0 / com_work.s[i];
      com_work.s_inv_z[i] = com_work.s_inv[i]*com_work.z[i];
    }
    com_work.block_33[0] = 0;
    com_fill_KKT();
    com_ldl_factor();
    /* Affine scaling directions. */
    com_fillrhs_aff();
    com_ldl_com_solve(com_work.rhs, com_work.lhs_aff);
    com_refine(com_work.rhs, com_work.lhs_aff);
    /* Centering plus corrector directions. */
    com_fillrhs_cc();
    com_ldl_com_solve(com_work.rhs, com_work.lhs_cc);
    com_refine(com_work.rhs, com_work.lhs_cc);
    /* Add the two together and store in aff. */
    for (i = 0; i < 14; i++)
      com_work.lhs_aff[i] += com_work.lhs_cc[i];
    /* Rename aff to reflect its new meaning. */
    dx = com_work.lhs_aff;
    ds = com_work.lhs_aff + 2;
    dz = com_work.lhs_aff + 8;
    dy = com_work.lhs_aff + 14;
    /* Find min(min(ds./s), min(dz./z)). */
    minval = 0;
    for (i = 0; i < 6; i++)
      if (ds[i] < minval*com_work.s[i])
        minval = ds[i]/com_work.s[i];
    for (i = 0; i < 6; i++)
      if (dz[i] < minval*com_work.z[i])
        minval = dz[i]/com_work.z[i];
    /* Find alpha. */
    if (-0.99 < minval)
      alpha = 1;
    else
      alpha = -0.99/minval;
    /* Update the primal and dual variables. */
    for (i = 0; i < 2; i++)
      com_work.x[i] += alpha*dx[i];
    for (i = 0; i < 6; i++)
      com_work.s[i] += alpha*ds[i];
    for (i = 0; i < 6; i++)
      com_work.z[i] += alpha*dz[i];
    for (i = 0; i < 0; i++)
      com_work.y[i] += alpha*dy[i];
    com_work.gap = com_eval_gap();
    com_work.eq_resid_squared = com_calc_eq_resid_squared();
    com_work.ineq_resid_squared = com_calc_ineq_resid_squared();
#ifndef ZERO_LIBRARY_MODE
    if (com_settings.verbose) {
      com_work.optval = com_eval_objv();
      printf("%3d   %10.3e  %9.2e  %9.2e  %9.2e  % 6.4f\n",
          iter+1, com_work.optval, com_work.gap, sqrt(com_work.eq_resid_squared),
          sqrt(com_work.ineq_resid_squared), alpha);
    }
#endif
    /* Test termination conditions. Requires optimality, and satisfied */
    /* constraints. */
    if (   (com_work.gap < com_settings.eps)
        && (com_work.eq_resid_squared <= com_settings.resid_tol*com_settings.resid_tol)
        && (com_work.ineq_resid_squared <= com_settings.resid_tol*com_settings.resid_tol)
       ) {
      com_work.converged = 1;
      com_work.optval = com_eval_objv();
      return iter+1;
    }
  }
  return iter;
}
