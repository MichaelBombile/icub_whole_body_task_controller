/* Produced by CVXGEN, 2020-12-10 23:07:19 -0500.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.c. */
/* Description: Main solver file. */
#include "cop_solver.h"
double cop_eval_gap(void) {
  int i;
  double gap;
  gap = 0;
  for (i = 0; i < 6; i++)
    gap += cop_work.z[i]*cop_work.s[i];
  return gap;
}
void cop_set_defaults(void) {
  cop_settings.resid_tol = 1e-6;
  cop_settings.eps = 1e-4;
  cop_settings.max_iters = 25;
  cop_settings.refine_steps = 1;
  cop_settings.s_init = 1;
  cop_settings.z_init = 1;
  cop_settings.debug = 0;
  cop_settings.verbose = 1;
  cop_settings.verbose_refinement = 0;
  cop_settings.better_start = 1;
  cop_settings.kkt_reg = 1e-7;
}
void cop_setup_pointers(void) {
  cop_work.y = cop_work.x + 2;
  cop_work.s = cop_work.x + 2;
  cop_work.z = cop_work.x + 8;
  cop_vars.xc = cop_work.x + 0;
}
void cop_setup_indexing(void) {
  cop_setup_pointers();
}
void cop_set_start(void) {
  int i;
  for (i = 0; i < 2; i++)
    cop_work.x[i] = 0;
  for (i = 0; i < 0; i++)
    cop_work.y[i] = 0;
  for (i = 0; i < 6; i++)
    cop_work.s[i] = (cop_work.h[i] > 0) ? cop_work.h[i] : cop_settings.s_init;
  for (i = 0; i < 6; i++)
    cop_work.z[i] = cop_settings.z_init;
}
double cop_eval_objv(void) {
  int i;
  double objv;
  /* Borrow space in cop_work.rhs. */
  cop_multbyP(cop_work.rhs, cop_work.x);
  objv = 0;
  for (i = 0; i < 2; i++)
    objv += cop_work.x[i]*cop_work.rhs[i];
  objv *= 0.5;
  for (i = 0; i < 2; i++)
    objv += cop_work.q[i]*cop_work.x[i];
  objv += 0;
  return objv;
}
void cop_fillrhs_aff(void) {
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = cop_work.rhs;
  r2 = cop_work.rhs + 2;
  r3 = cop_work.rhs + 8;
  r4 = cop_work.rhs + 14;
  /* r1 = -A^Ty - G^Tz - Px - q. */
  cop_multbymAT(r1, cop_work.y);
  cop_multbymGT(cop_work.buffer, cop_work.z);
  for (i = 0; i < 2; i++)
    r1[i] += cop_work.buffer[i];
  cop_multbyP(cop_work.buffer, cop_work.x);
  for (i = 0; i < 2; i++)
    r1[i] -= cop_work.buffer[i] + cop_work.q[i];
  /* r2 = -z. */
  for (i = 0; i < 6; i++)
    r2[i] = -cop_work.z[i];
  /* r3 = -Gx - s + h. */
  cop_multbymG(r3, cop_work.x);
  for (i = 0; i < 6; i++)
    r3[i] += -cop_work.s[i] + cop_work.h[i];
  /* r4 = -Ax + b. */
  cop_multbymA(r4, cop_work.x);
  for (i = 0; i < 0; i++)
    r4[i] += cop_work.b[i];
}
void cop_fillrhs_cc(void) {
  int i;
  double *r2;
  double *ds_aff, *dz_aff;
  double mu;
  double alpha;
  double sigma;
  double smu;
  double minval;
  r2 = cop_work.rhs + 2;
  ds_aff = cop_work.lhs_aff + 2;
  dz_aff = cop_work.lhs_aff + 8;
  mu = 0;
  for (i = 0; i < 6; i++)
    mu += cop_work.s[i]*cop_work.z[i];
  /* Don't finish calculating mu quite yet. */
  /* Find min(min(ds./s), min(dz./z)). */
  minval = 0;
  for (i = 0; i < 6; i++)
    if (ds_aff[i] < minval*cop_work.s[i])
      minval = ds_aff[i]/cop_work.s[i];
  for (i = 0; i < 6; i++)
    if (dz_aff[i] < minval*cop_work.z[i])
      minval = dz_aff[i]/cop_work.z[i];
  /* Find alpha. */
  if (-1 < minval)
      alpha = 1;
  else
      alpha = -1/minval;
  sigma = 0;
  for (i = 0; i < 6; i++)
    sigma += (cop_work.s[i] + alpha*ds_aff[i])*
      (cop_work.z[i] + alpha*dz_aff[i]);
  sigma /= mu;
  sigma = sigma*sigma*sigma;
  /* Finish calculating mu now. */
  mu *= 0.16666666666666666;
  smu = sigma*mu;
  /* Fill-in the rhs. */
  for (i = 0; i < 2; i++)
    cop_work.rhs[i] = 0;
  for (i = 8; i < 14; i++)
    cop_work.rhs[i] = 0;
  for (i = 0; i < 6; i++)
    r2[i] = cop_work.s_inv[i]*(smu - ds_aff[i]*dz_aff[i]);
}
void cop_refine(double *target, double *var) {
  int i, j;
  double *residual = cop_work.buffer;
  double norm2;
  double *new_var = cop_work.buffer2;
  for (j = 0; j < cop_settings.refine_steps; j++) {
    norm2 = 0;
    cop_matrix_multiply(residual, var);
    for (i = 0; i < 14; i++) {
      residual[i] = residual[i] - target[i];
      norm2 += residual[i]*residual[i];
    }
#ifndef ZERO_LIBRARY_MODE
    if (cop_settings.verbose_refinement) {
      if (j == 0)
        printf("Initial residual before refinement has norm squared %.6g.\n", norm2);
      else
        printf("After refinement we get squared norm %.6g.\n", norm2);
    }
#endif
    /* Solve to find new_var = KKT \ (target - A*var). */
    cop_ldl_cop_solve(residual, new_var);
    /* Update var += new_var, or var += KKT \ (target - A*var). */
    for (i = 0; i < 14; i++) {
      var[i] -= new_var[i];
    }
  }
#ifndef ZERO_LIBRARY_MODE
  if (cop_settings.verbose_refinement) {
    /* Check the residual once more, but only if we're reporting it, since */
    /* it's expensive. */
    norm2 = 0;
    cop_matrix_multiply(residual, var);
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
double cop_calc_ineq_resid_squared(void) {
  /* Calculates the norm ||-Gx - s + h||. */
  double norm2_squared;
  int i;
  /* Find -Gx. */
  cop_multbymG(cop_work.buffer, cop_work.x);
  /* Add -s + h. */
  for (i = 0; i < 6; i++)
    cop_work.buffer[i] += -cop_work.s[i] + cop_work.h[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 6; i++)
    norm2_squared += cop_work.buffer[i]*cop_work.buffer[i];
  return norm2_squared;
}
double cop_calc_eq_resid_squared(void) {
  /* Calculates the norm ||-Ax + b||. */
  double norm2_squared;
  int i;
  /* Find -Ax. */
  cop_multbymA(cop_work.buffer, cop_work.x);
  /* Add +b. */
  for (i = 0; i < 0; i++)
    cop_work.buffer[i] += cop_work.b[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 0; i++)
    norm2_squared += cop_work.buffer[i]*cop_work.buffer[i];
  return norm2_squared;
}
void cop_better_start(void) {
  /* Calculates a better starting point, using a similar approach to CVXOPT. */
  /* Not yet speed optimized. */
  int i;
  double *x, *s, *z, *y;
  double alpha;
  cop_work.block_33[0] = -1;
  /* Make sure sinvz is 1 to make hijacked KKT system ok. */
  for (i = 0; i < 6; i++)
    cop_work.s_inv_z[i] = 1;
  cop_fill_KKT();
  cop_ldl_factor();
  cop_fillrhs_start();
  /* Borrow cop_work.lhs_aff for the solution. */
  cop_ldl_cop_solve(cop_work.rhs, cop_work.lhs_aff);
  /* Don't do any refinement for now. Precision doesn't matter too much. */
  x = cop_work.lhs_aff;
  s = cop_work.lhs_aff + 2;
  z = cop_work.lhs_aff + 8;
  y = cop_work.lhs_aff + 14;
  /* Just set x and y as is. */
  for (i = 0; i < 2; i++)
    cop_work.x[i] = x[i];
  for (i = 0; i < 0; i++)
    cop_work.y[i] = y[i];
  /* Now complete the initialization. Start with s. */
  /* Must have alpha > max(z). */
  alpha = -1e99;
  for (i = 0; i < 6; i++)
    if (alpha < z[i])
      alpha = z[i];
  if (alpha < 0) {
    for (i = 0; i < 6; i++)
      cop_work.s[i] = -z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 6; i++)
      cop_work.s[i] = -z[i] + alpha;
  }
  /* Now initialize z. */
  /* Now must have alpha > max(-z). */
  alpha = -1e99;
  for (i = 0; i < 6; i++)
    if (alpha < -z[i])
      alpha = -z[i];
  if (alpha < 0) {
    for (i = 0; i < 6; i++)
      cop_work.z[i] = z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 6; i++)
      cop_work.z[i] = z[i] + alpha;
  }
}
void cop_fillrhs_start(void) {
  /* Fill rhs with (-q, 0, h, b). */
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = cop_work.rhs;
  r2 = cop_work.rhs + 2;
  r3 = cop_work.rhs + 8;
  r4 = cop_work.rhs + 14;
  for (i = 0; i < 2; i++)
    r1[i] = -cop_work.q[i];
  for (i = 0; i < 6; i++)
    r2[i] = 0;
  for (i = 0; i < 6; i++)
    r3[i] = cop_work.h[i];
  for (i = 0; i < 0; i++)
    r4[i] = cop_work.b[i];
}
long cop_solve(void) {
  int i;
  int iter;
  double *dx, *ds, *dy, *dz;
  double minval;
  double alpha;
  cop_work.converged = 0;
  cop_setup_pointers();
  cop_pre_ops();
#ifndef ZERO_LIBRARY_MODE
  if (cop_settings.verbose)
    printf("iter     objv        gap       |Ax-b|    |Gx+s-h|    step\n");
#endif
  cop_fillq();
  cop_fillh();
  cop_fillb();
  if (cop_settings.better_start)
    cop_better_start();
  else
    cop_set_start();
  for (iter = 0; iter < cop_settings.max_iters; iter++) {
    for (i = 0; i < 6; i++) {
      cop_work.s_inv[i] = 1.0 / cop_work.s[i];
      cop_work.s_inv_z[i] = cop_work.s_inv[i]*cop_work.z[i];
    }
    cop_work.block_33[0] = 0;
    cop_fill_KKT();
    cop_ldl_factor();
    /* Affine scaling directions. */
    cop_fillrhs_aff();
    cop_ldl_cop_solve(cop_work.rhs, cop_work.lhs_aff);
    cop_refine(cop_work.rhs, cop_work.lhs_aff);
    /* Centering plus corrector directions. */
    cop_fillrhs_cc();
    cop_ldl_cop_solve(cop_work.rhs, cop_work.lhs_cc);
    cop_refine(cop_work.rhs, cop_work.lhs_cc);
    /* Add the two together and store in aff. */
    for (i = 0; i < 14; i++)
      cop_work.lhs_aff[i] += cop_work.lhs_cc[i];
    /* Rename aff to reflect its new meaning. */
    dx = cop_work.lhs_aff;
    ds = cop_work.lhs_aff + 2;
    dz = cop_work.lhs_aff + 8;
    dy = cop_work.lhs_aff + 14;
    /* Find min(min(ds./s), min(dz./z)). */
    minval = 0;
    for (i = 0; i < 6; i++)
      if (ds[i] < minval*cop_work.s[i])
        minval = ds[i]/cop_work.s[i];
    for (i = 0; i < 6; i++)
      if (dz[i] < minval*cop_work.z[i])
        minval = dz[i]/cop_work.z[i];
    /* Find alpha. */
    if (-0.99 < minval)
      alpha = 1;
    else
      alpha = -0.99/minval;
    /* Update the primal and dual variables. */
    for (i = 0; i < 2; i++)
      cop_work.x[i] += alpha*dx[i];
    for (i = 0; i < 6; i++)
      cop_work.s[i] += alpha*ds[i];
    for (i = 0; i < 6; i++)
      cop_work.z[i] += alpha*dz[i];
    for (i = 0; i < 0; i++)
      cop_work.y[i] += alpha*dy[i];
    cop_work.gap = cop_eval_gap();
    cop_work.eq_resid_squared = cop_calc_eq_resid_squared();
    cop_work.ineq_resid_squared = cop_calc_ineq_resid_squared();
#ifndef ZERO_LIBRARY_MODE
    if (cop_settings.verbose) {
      cop_work.optval = cop_eval_objv();
      printf("%3d   %10.3e  %9.2e  %9.2e  %9.2e  % 6.4f\n",
          iter+1, cop_work.optval, cop_work.gap, sqrt(cop_work.eq_resid_squared),
          sqrt(cop_work.ineq_resid_squared), alpha);
    }
#endif
    /* Test termination conditions. Requires optimality, and satisfied */
    /* constraints. */
    if (   (cop_work.gap < cop_settings.eps)
        && (cop_work.eq_resid_squared <= cop_settings.resid_tol*cop_settings.resid_tol)
        && (cop_work.ineq_resid_squared <= cop_settings.resid_tol*cop_settings.resid_tol)
       ) {
      cop_work.converged = 1;
      cop_work.optval = cop_eval_objv();
      return iter+1;
    }
  }
  return iter;
}
