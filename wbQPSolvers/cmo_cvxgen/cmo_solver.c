/* Produced by CVXGEN, 2019-04-21 11:08:24 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.c. */
/* Description: Main solver file. */
#include "cmo_solver.h"
double cmo_eval_gap(void) {
  int i;
  double gap;
  gap = 0;
  for (i = 0; i < 24; i++)
    gap += cmo_work.z[i]*cmo_work.s[i];
  return gap;
}
void cmo_set_defaults(void) {
  cmo_settings.resid_tol = 1e-6;
  cmo_settings.eps = 1e-4;
  cmo_settings.max_iters = 25;
  cmo_settings.refine_steps = 1;
  cmo_settings.s_init = 1;
  cmo_settings.z_init = 1;
  cmo_settings.debug = 0;
  cmo_settings.verbose = 1;
  cmo_settings.verbose_refinement = 0;
  cmo_settings.better_start = 1;
  cmo_settings.kkt_reg = 1e-7;
}
void cmo_setup_pointers(void) {
  cmo_work.y = cmo_work.x + 26;
  cmo_work.s = cmo_work.x + 28;
  cmo_work.z = cmo_work.x + 52;
  cmo_vars.x = cmo_work.x + 14;
}
void cmo_setup_indexing(void) {
  cmo_setup_pointers();
}
void cmo_set_start(void) {
  int i;
  for (i = 0; i < 26; i++)
    cmo_work.x[i] = 0;
  for (i = 0; i < 2; i++)
    cmo_work.y[i] = 0;
  for (i = 0; i < 24; i++)
    cmo_work.s[i] = (cmo_work.h[i] > 0) ? cmo_work.h[i] : cmo_settings.s_init;
  for (i = 0; i < 24; i++)
    cmo_work.z[i] = cmo_settings.z_init;
}
double cmo_eval_objv(void) {
  int i;
  double objv;
  /* Borrow space in cmo_work.rhs. */
  cmo_multbyP(cmo_work.rhs, cmo_work.x);
  objv = 0;
  for (i = 0; i < 26; i++)
    objv += cmo_work.x[i]*cmo_work.rhs[i];
  objv *= 0.5;
  for (i = 0; i < 26; i++)
    objv += cmo_work.q[i]*cmo_work.x[i];
  objv += 0;
  return objv;
}
void cmo_fillrhs_aff(void) {
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = cmo_work.rhs;
  r2 = cmo_work.rhs + 26;
  r3 = cmo_work.rhs + 50;
  r4 = cmo_work.rhs + 74;
  /* r1 = -A^Ty - G^Tz - Px - q. */
  cmo_multbymAT(r1, cmo_work.y);
  cmo_multbymGT(cmo_work.buffer, cmo_work.z);
  for (i = 0; i < 26; i++)
    r1[i] += cmo_work.buffer[i];
  cmo_multbyP(cmo_work.buffer, cmo_work.x);
  for (i = 0; i < 26; i++)
    r1[i] -= cmo_work.buffer[i] + cmo_work.q[i];
  /* r2 = -z. */
  for (i = 0; i < 24; i++)
    r2[i] = -cmo_work.z[i];
  /* r3 = -Gx - s + h. */
  cmo_multbymG(r3, cmo_work.x);
  for (i = 0; i < 24; i++)
    r3[i] += -cmo_work.s[i] + cmo_work.h[i];
  /* r4 = -Ax + b. */
  cmo_multbymA(r4, cmo_work.x);
  for (i = 0; i < 2; i++)
    r4[i] += cmo_work.b[i];
}
void cmo_fillrhs_cc(void) {
  int i;
  double *r2;
  double *ds_aff, *dz_aff;
  double mu;
  double alpha;
  double sigma;
  double smu;
  double minval;
  r2 = cmo_work.rhs + 26;
  ds_aff = cmo_work.lhs_aff + 26;
  dz_aff = cmo_work.lhs_aff + 50;
  mu = 0;
  for (i = 0; i < 24; i++)
    mu += cmo_work.s[i]*cmo_work.z[i];
  /* Don't finish calculating mu quite yet. */
  /* Find min(min(ds./s), min(dz./z)). */
  minval = 0;
  for (i = 0; i < 24; i++)
    if (ds_aff[i] < minval*cmo_work.s[i])
      minval = ds_aff[i]/cmo_work.s[i];
  for (i = 0; i < 24; i++)
    if (dz_aff[i] < minval*cmo_work.z[i])
      minval = dz_aff[i]/cmo_work.z[i];
  /* Find alpha. */
  if (-1 < minval)
      alpha = 1;
  else
      alpha = -1/minval;
  sigma = 0;
  for (i = 0; i < 24; i++)
    sigma += (cmo_work.s[i] + alpha*ds_aff[i])*
      (cmo_work.z[i] + alpha*dz_aff[i]);
  sigma /= mu;
  sigma = sigma*sigma*sigma;
  /* Finish calculating mu now. */
  mu *= 0.041666666666666664;
  smu = sigma*mu;
  /* Fill-in the rhs. */
  for (i = 0; i < 26; i++)
    cmo_work.rhs[i] = 0;
  for (i = 50; i < 76; i++)
    cmo_work.rhs[i] = 0;
  for (i = 0; i < 24; i++)
    r2[i] = cmo_work.s_inv[i]*(smu - ds_aff[i]*dz_aff[i]);
}
void cmo_refine(double *target, double *var) {
  int i, j;
  double *residual = cmo_work.buffer;
  double norm2;
  double *new_var = cmo_work.buffer2;
  for (j = 0; j < cmo_settings.refine_steps; j++) {
    norm2 = 0;
    cmo_matrix_multiply(residual, var);
    for (i = 0; i < 76; i++) {
      residual[i] = residual[i] - target[i];
      norm2 += residual[i]*residual[i];
    }
#ifndef ZERO_LIBRARY_MODE
    if (cmo_settings.verbose_refinement) {
      if (j == 0)
        printf("Initial residual before refinement has norm squared %.6g.\n", norm2);
      else
        printf("After refinement we get squared norm %.6g.\n", norm2);
    }
#endif
    /* Solve to find new_var = KKT \ (target - A*var). */
    cmo_ldl_cmo_solve(residual, new_var);
    /* Update var += new_var, or var += KKT \ (target - A*var). */
    for (i = 0; i < 76; i++) {
      var[i] -= new_var[i];
    }
  }
#ifndef ZERO_LIBRARY_MODE
  if (cmo_settings.verbose_refinement) {
    /* Check the residual once more, but only if we're reporting it, since */
    /* it's expensive. */
    norm2 = 0;
    cmo_matrix_multiply(residual, var);
    for (i = 0; i < 76; i++) {
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
double cmo_calc_ineq_resid_squared(void) {
  /* Calculates the norm ||-Gx - s + h||. */
  double norm2_squared;
  int i;
  /* Find -Gx. */
  cmo_multbymG(cmo_work.buffer, cmo_work.x);
  /* Add -s + h. */
  for (i = 0; i < 24; i++)
    cmo_work.buffer[i] += -cmo_work.s[i] + cmo_work.h[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 24; i++)
    norm2_squared += cmo_work.buffer[i]*cmo_work.buffer[i];
  return norm2_squared;
}
double cmo_calc_eq_resid_squared(void) {
  /* Calculates the norm ||-Ax + b||. */
  double norm2_squared;
  int i;
  /* Find -Ax. */
  cmo_multbymA(cmo_work.buffer, cmo_work.x);
  /* Add +b. */
  for (i = 0; i < 2; i++)
    cmo_work.buffer[i] += cmo_work.b[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 2; i++)
    norm2_squared += cmo_work.buffer[i]*cmo_work.buffer[i];
  return norm2_squared;
}
void cmo_better_start(void) {
  /* Calculates a better starting point, using a similar approach to CVXOPT. */
  /* Not yet speed optimized. */
  int i;
  double *x, *s, *z, *y;
  double alpha;
  cmo_work.block_33[0] = -1;
  /* Make sure sinvz is 1 to make hijacked KKT system ok. */
  for (i = 0; i < 24; i++)
    cmo_work.s_inv_z[i] = 1;
  cmo_fill_KKT();
  cmo_ldl_factor();
  cmo_fillrhs_start();
  /* Borrow cmo_work.lhs_aff for the solution. */
  cmo_ldl_cmo_solve(cmo_work.rhs, cmo_work.lhs_aff);
  /* Don't do any refinement for now. Precision doesn't matter too much. */
  x = cmo_work.lhs_aff;
  s = cmo_work.lhs_aff + 26;
  z = cmo_work.lhs_aff + 50;
  y = cmo_work.lhs_aff + 74;
  /* Just set x and y as is. */
  for (i = 0; i < 26; i++)
    cmo_work.x[i] = x[i];
  for (i = 0; i < 2; i++)
    cmo_work.y[i] = y[i];
  /* Now complete the initialization. Start with s. */
  /* Must have alpha > max(z). */
  alpha = -1e99;
  for (i = 0; i < 24; i++)
    if (alpha < z[i])
      alpha = z[i];
  if (alpha < 0) {
    for (i = 0; i < 24; i++)
      cmo_work.s[i] = -z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 24; i++)
      cmo_work.s[i] = -z[i] + alpha;
  }
  /* Now initialize z. */
  /* Now must have alpha > max(-z). */
  alpha = -1e99;
  for (i = 0; i < 24; i++)
    if (alpha < -z[i])
      alpha = -z[i];
  if (alpha < 0) {
    for (i = 0; i < 24; i++)
      cmo_work.z[i] = z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 24; i++)
      cmo_work.z[i] = z[i] + alpha;
  }
}
void cmo_fillrhs_start(void) {
  /* Fill rhs with (-q, 0, h, b). */
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = cmo_work.rhs;
  r2 = cmo_work.rhs + 26;
  r3 = cmo_work.rhs + 50;
  r4 = cmo_work.rhs + 74;
  for (i = 0; i < 26; i++)
    r1[i] = -cmo_work.q[i];
  for (i = 0; i < 24; i++)
    r2[i] = 0;
  for (i = 0; i < 24; i++)
    r3[i] = cmo_work.h[i];
  for (i = 0; i < 2; i++)
    r4[i] = cmo_work.b[i];
}
long cmo_solve(void) {
  int i;
  int iter;
  double *dx, *ds, *dy, *dz;
  double minval;
  double alpha;
  cmo_work.converged = 0;
  cmo_setup_pointers();
  cmo_pre_ops();
#ifndef ZERO_LIBRARY_MODE
  if (cmo_settings.verbose)
    printf("iter     objv        gap       |Ax-b|    |Gx+s-h|    step\n");
#endif
  cmo_fillq();
  cmo_fillh();
  cmo_fillb();
  if (cmo_settings.better_start)
    cmo_better_start();
  else
    cmo_set_start();
  for (iter = 0; iter < cmo_settings.max_iters; iter++) {
    for (i = 0; i < 24; i++) {
      cmo_work.s_inv[i] = 1.0 / cmo_work.s[i];
      cmo_work.s_inv_z[i] = cmo_work.s_inv[i]*cmo_work.z[i];
    }
    cmo_work.block_33[0] = 0;
    cmo_fill_KKT();
    cmo_ldl_factor();
    /* Affine scaling directions. */
    cmo_fillrhs_aff();
    cmo_ldl_cmo_solve(cmo_work.rhs, cmo_work.lhs_aff);
    cmo_refine(cmo_work.rhs, cmo_work.lhs_aff);
    /* Centering plus corrector directions. */
    cmo_fillrhs_cc();
    cmo_ldl_cmo_solve(cmo_work.rhs, cmo_work.lhs_cc);
    cmo_refine(cmo_work.rhs, cmo_work.lhs_cc);
    /* Add the two together and store in aff. */
    for (i = 0; i < 76; i++)
      cmo_work.lhs_aff[i] += cmo_work.lhs_cc[i];
    /* Rename aff to reflect its new meaning. */
    dx = cmo_work.lhs_aff;
    ds = cmo_work.lhs_aff + 26;
    dz = cmo_work.lhs_aff + 50;
    dy = cmo_work.lhs_aff + 74;
    /* Find min(min(ds./s), min(dz./z)). */
    minval = 0;
    for (i = 0; i < 24; i++)
      if (ds[i] < minval*cmo_work.s[i])
        minval = ds[i]/cmo_work.s[i];
    for (i = 0; i < 24; i++)
      if (dz[i] < minval*cmo_work.z[i])
        minval = dz[i]/cmo_work.z[i];
    /* Find alpha. */
    if (-0.99 < minval)
      alpha = 1;
    else
      alpha = -0.99/minval;
    /* Update the primal and dual variables. */
    for (i = 0; i < 26; i++)
      cmo_work.x[i] += alpha*dx[i];
    for (i = 0; i < 24; i++)
      cmo_work.s[i] += alpha*ds[i];
    for (i = 0; i < 24; i++)
      cmo_work.z[i] += alpha*dz[i];
    for (i = 0; i < 2; i++)
      cmo_work.y[i] += alpha*dy[i];
    cmo_work.gap = cmo_eval_gap();
    cmo_work.eq_resid_squared = cmo_calc_eq_resid_squared();
    cmo_work.ineq_resid_squared = cmo_calc_ineq_resid_squared();
#ifndef ZERO_LIBRARY_MODE
    if (cmo_settings.verbose) {
      cmo_work.optval = cmo_eval_objv();
      printf("%3d   %10.3e  %9.2e  %9.2e  %9.2e  % 6.4f\n",
          iter+1, cmo_work.optval, cmo_work.gap, sqrt(cmo_work.eq_resid_squared),
          sqrt(cmo_work.ineq_resid_squared), alpha);
    }
#endif
    /* Test termination conditions. Requires optimality, and satisfied */
    /* constraints. */
    if (   (cmo_work.gap < cmo_settings.eps)
        && (cmo_work.eq_resid_squared <= cmo_settings.resid_tol*cmo_settings.resid_tol)
        && (cmo_work.ineq_resid_squared <= cmo_settings.resid_tol*cmo_settings.resid_tol)
       ) {
      cmo_work.converged = 1;
      cmo_work.optval = cmo_eval_objv();
      return iter+1;
    }
  }
  return iter;
}
