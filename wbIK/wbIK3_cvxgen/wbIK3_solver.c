/* Produced by CVXGEN, 2020-08-19 11:45:50 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.c. */
/* Description: Main solver file. */
#include "wbIK3_solver.h"
double wbIK3_eval_gap(void) {
  int i;
  double gap;
  gap = 0;
  for (i = 0; i < 69; i++)
    gap += wbIK3_work.z[i]*wbIK3_work.s[i];
  return gap;
}
void wbIK3_set_defaults(void) {
  wbIK3_settings.resid_tol = 1e-6;
  wbIK3_settings.eps = 1e-4;
  wbIK3_settings.max_iters = 25;
  wbIK3_settings.refine_steps = 1;
  wbIK3_settings.s_init = 1;
  wbIK3_settings.z_init = 1;
  wbIK3_settings.debug = 0;
  wbIK3_settings.verbose = 1;
  wbIK3_settings.verbose_refinement = 0;
  wbIK3_settings.better_start = 1;
  wbIK3_settings.kkt_reg = 1e-7;
}
void wbIK3_setup_pointers(void) {
  wbIK3_work.y = wbIK3_work.x + 35;
  wbIK3_work.s = wbIK3_work.x + 35;
  wbIK3_work.z = wbIK3_work.x + 104;
  wbIK3_vars.qdot = wbIK3_work.x + 0;
}
void setup_indexed_wbIK3_params(void) {
  /* In CVXGEN, you can say */
  /*   parameters */
  /*     A[i] (5,3), i=1..4 */
  /*   end */
  /* This function sets up A[2] to be a pointer to A_2, which is a length-15 */
  /* vector of doubles. */
  /* If you access parameters that you haven't defined in CVXGEN, the result */
  /* is undefined. */
  wbIK3_params.NeJc[1] = wbIK3_params.NeJc_1;
  wbIK3_params.NeJc[2] = wbIK3_params.NeJc_2;
  wbIK3_params.NeJc[3] = wbIK3_params.NeJc_3;
  wbIK3_params.NeJc[4] = wbIK3_params.NeJc_4;
  wbIK3_params.NeJc[5] = wbIK3_params.NeJc_5;
  wbIK3_params.NeJc[6] = wbIK3_params.NeJc_6;
  wbIK3_params.sMDw[1] = wbIK3_params.sMDw_1;
  wbIK3_params.sMDw[2] = wbIK3_params.sMDw_2;
  wbIK3_params.sMDw[3] = wbIK3_params.sMDw_3;
  wbIK3_params.sMDw[4] = wbIK3_params.sMDw_4;
  wbIK3_params.sMDw[5] = wbIK3_params.sMDw_5;
}
void wbIK3_setup_indexing(void) {
  wbIK3_setup_pointers();
  setup_indexed_wbIK3_params();
}
void wbIK3_set_start(void) {
  int i;
  for (i = 0; i < 35; i++)
    wbIK3_work.x[i] = 0;
  for (i = 0; i < 0; i++)
    wbIK3_work.y[i] = 0;
  for (i = 0; i < 69; i++)
    wbIK3_work.s[i] = (wbIK3_work.h[i] > 0) ? wbIK3_work.h[i] : wbIK3_settings.s_init;
  for (i = 0; i < 69; i++)
    wbIK3_work.z[i] = wbIK3_settings.z_init;
}
double wbIK3_eval_objv(void) {
  int i;
  double objv;
  /* Borrow space in wbIK3_work.rhs. */
  wbIK3_multbyP(wbIK3_work.rhs, wbIK3_work.x);
  objv = 0;
  for (i = 0; i < 35; i++)
    objv += wbIK3_work.x[i]*wbIK3_work.rhs[i];
  objv *= 0.5;
  for (i = 0; i < 35; i++)
    objv += wbIK3_work.q[i]*wbIK3_work.x[i];
  objv += 0;
  return objv;
}
void wbIK3_fillrhs_aff(void) {
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = wbIK3_work.rhs;
  r2 = wbIK3_work.rhs + 35;
  r3 = wbIK3_work.rhs + 104;
  r4 = wbIK3_work.rhs + 173;
  /* r1 = -A^Ty - G^Tz - Px - q. */
  wbIK3_multbymAT(r1, wbIK3_work.y);
  wbIK3_multbymGT(wbIK3_work.buffer, wbIK3_work.z);
  for (i = 0; i < 35; i++)
    r1[i] += wbIK3_work.buffer[i];
  wbIK3_multbyP(wbIK3_work.buffer, wbIK3_work.x);
  for (i = 0; i < 35; i++)
    r1[i] -= wbIK3_work.buffer[i] + wbIK3_work.q[i];
  /* r2 = -z. */
  for (i = 0; i < 69; i++)
    r2[i] = -wbIK3_work.z[i];
  /* r3 = -Gx - s + h. */
  wbIK3_multbymG(r3, wbIK3_work.x);
  for (i = 0; i < 69; i++)
    r3[i] += -wbIK3_work.s[i] + wbIK3_work.h[i];
  /* r4 = -Ax + b. */
  wbIK3_multbymA(r4, wbIK3_work.x);
  for (i = 0; i < 0; i++)
    r4[i] += wbIK3_work.b[i];
}
void wbIK3_fillrhs_cc(void) {
  int i;
  double *r2;
  double *ds_aff, *dz_aff;
  double mu;
  double alpha;
  double sigma;
  double smu;
  double minval;
  r2 = wbIK3_work.rhs + 35;
  ds_aff = wbIK3_work.lhs_aff + 35;
  dz_aff = wbIK3_work.lhs_aff + 104;
  mu = 0;
  for (i = 0; i < 69; i++)
    mu += wbIK3_work.s[i]*wbIK3_work.z[i];
  /* Don't finish calculating mu quite yet. */
  /* Find min(min(ds./s), min(dz./z)). */
  minval = 0;
  for (i = 0; i < 69; i++)
    if (ds_aff[i] < minval*wbIK3_work.s[i])
      minval = ds_aff[i]/wbIK3_work.s[i];
  for (i = 0; i < 69; i++)
    if (dz_aff[i] < minval*wbIK3_work.z[i])
      minval = dz_aff[i]/wbIK3_work.z[i];
  /* Find alpha. */
  if (-1 < minval)
      alpha = 1;
  else
      alpha = -1/minval;
  sigma = 0;
  for (i = 0; i < 69; i++)
    sigma += (wbIK3_work.s[i] + alpha*ds_aff[i])*
      (wbIK3_work.z[i] + alpha*dz_aff[i]);
  sigma /= mu;
  sigma = sigma*sigma*sigma;
  /* Finish calculating mu now. */
  mu *= 0.014492753623188406;
  smu = sigma*mu;
  /* Fill-in the rhs. */
  for (i = 0; i < 35; i++)
    wbIK3_work.rhs[i] = 0;
  for (i = 104; i < 173; i++)
    wbIK3_work.rhs[i] = 0;
  for (i = 0; i < 69; i++)
    r2[i] = wbIK3_work.s_inv[i]*(smu - ds_aff[i]*dz_aff[i]);
}
void wbIK3_refine(double *target, double *var) {
  int i, j;
  double *residual = wbIK3_work.buffer;
  double norm2;
  double *new_var = wbIK3_work.buffer2;
  for (j = 0; j < wbIK3_settings.refine_steps; j++) {
    norm2 = 0;
    wbIK3_matrix_multiply(residual, var);
    for (i = 0; i < 173; i++) {
      residual[i] = residual[i] - target[i];
      norm2 += residual[i]*residual[i];
    }
#ifndef ZERO_LIBRARY_MODE
    if (wbIK3_settings.verbose_refinement) {
      if (j == 0)
        printf("Initial residual before refinement has norm squared %.6g.\n", norm2);
      else
        printf("After refinement we get squared norm %.6g.\n", norm2);
    }
#endif
    /* Solve to find new_var = KKT \ (target - A*var). */
    wbIK3_ldl_wbIK3_solve(residual, new_var);
    /* Update var += new_var, or var += KKT \ (target - A*var). */
    for (i = 0; i < 173; i++) {
      var[i] -= new_var[i];
    }
  }
#ifndef ZERO_LIBRARY_MODE
  if (wbIK3_settings.verbose_refinement) {
    /* Check the residual once more, but only if we're reporting it, since */
    /* it's expensive. */
    norm2 = 0;
    wbIK3_matrix_multiply(residual, var);
    for (i = 0; i < 173; i++) {
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
double wbIK3_calc_ineq_resid_squared(void) {
  /* Calculates the norm ||-Gx - s + h||. */
  double norm2_squared;
  int i;
  /* Find -Gx. */
  wbIK3_multbymG(wbIK3_work.buffer, wbIK3_work.x);
  /* Add -s + h. */
  for (i = 0; i < 69; i++)
    wbIK3_work.buffer[i] += -wbIK3_work.s[i] + wbIK3_work.h[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 69; i++)
    norm2_squared += wbIK3_work.buffer[i]*wbIK3_work.buffer[i];
  return norm2_squared;
}
double wbIK3_calc_eq_resid_squared(void) {
  /* Calculates the norm ||-Ax + b||. */
  double norm2_squared;
  int i;
  /* Find -Ax. */
  wbIK3_multbymA(wbIK3_work.buffer, wbIK3_work.x);
  /* Add +b. */
  for (i = 0; i < 0; i++)
    wbIK3_work.buffer[i] += wbIK3_work.b[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 0; i++)
    norm2_squared += wbIK3_work.buffer[i]*wbIK3_work.buffer[i];
  return norm2_squared;
}
void wbIK3_better_start(void) {
  /* Calculates a better starting point, using a similar approach to CVXOPT. */
  /* Not yet speed optimized. */
  int i;
  double *x, *s, *z, *y;
  double alpha;
  wbIK3_work.block_33[0] = -1;
  /* Make sure sinvz is 1 to make hijacked KKT system ok. */
  for (i = 0; i < 69; i++)
    wbIK3_work.s_inv_z[i] = 1;
  wbIK3_fill_KKT();
  wbIK3_ldl_factor();
  wbIK3_fillrhs_start();
  /* Borrow wbIK3_work.lhs_aff for the solution. */
  wbIK3_ldl_wbIK3_solve(wbIK3_work.rhs, wbIK3_work.lhs_aff);
  /* Don't do any refinement for now. Precision doesn't matter too much. */
  x = wbIK3_work.lhs_aff;
  s = wbIK3_work.lhs_aff + 35;
  z = wbIK3_work.lhs_aff + 104;
  y = wbIK3_work.lhs_aff + 173;
  /* Just set x and y as is. */
  for (i = 0; i < 35; i++)
    wbIK3_work.x[i] = x[i];
  for (i = 0; i < 0; i++)
    wbIK3_work.y[i] = y[i];
  /* Now complete the initialization. Start with s. */
  /* Must have alpha > max(z). */
  alpha = -1e99;
  for (i = 0; i < 69; i++)
    if (alpha < z[i])
      alpha = z[i];
  if (alpha < 0) {
    for (i = 0; i < 69; i++)
      wbIK3_work.s[i] = -z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 69; i++)
      wbIK3_work.s[i] = -z[i] + alpha;
  }
  /* Now initialize z. */
  /* Now must have alpha > max(-z). */
  alpha = -1e99;
  for (i = 0; i < 69; i++)
    if (alpha < -z[i])
      alpha = -z[i];
  if (alpha < 0) {
    for (i = 0; i < 69; i++)
      wbIK3_work.z[i] = z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 69; i++)
      wbIK3_work.z[i] = z[i] + alpha;
  }
}
void wbIK3_fillrhs_start(void) {
  /* Fill rhs with (-q, 0, h, b). */
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = wbIK3_work.rhs;
  r2 = wbIK3_work.rhs + 35;
  r3 = wbIK3_work.rhs + 104;
  r4 = wbIK3_work.rhs + 173;
  for (i = 0; i < 35; i++)
    r1[i] = -wbIK3_work.q[i];
  for (i = 0; i < 69; i++)
    r2[i] = 0;
  for (i = 0; i < 69; i++)
    r3[i] = wbIK3_work.h[i];
  for (i = 0; i < 0; i++)
    r4[i] = wbIK3_work.b[i];
}
long wbIK3_solve(void) {
  int i;
  int iter;
  double *dx, *ds, *dy, *dz;
  double minval;
  double alpha;
  wbIK3_work.converged = 0;
  wbIK3_setup_pointers();
  wbIK3_pre_ops();
#ifndef ZERO_LIBRARY_MODE
  if (wbIK3_settings.verbose)
    printf("iter     objv        gap       |Ax-b|    |Gx+s-h|    step\n");
#endif
  wbIK3_fillq();
  wbIK3_fillh();
  wbIK3_fillb();
  if (wbIK3_settings.better_start)
    wbIK3_better_start();
  else
    wbIK3_set_start();
  for (iter = 0; iter < wbIK3_settings.max_iters; iter++) {
    for (i = 0; i < 69; i++) {
      wbIK3_work.s_inv[i] = 1.0 / wbIK3_work.s[i];
      wbIK3_work.s_inv_z[i] = wbIK3_work.s_inv[i]*wbIK3_work.z[i];
    }
    wbIK3_work.block_33[0] = 0;
    wbIK3_fill_KKT();
    wbIK3_ldl_factor();
    /* Affine scaling directions. */
    wbIK3_fillrhs_aff();
    wbIK3_ldl_wbIK3_solve(wbIK3_work.rhs, wbIK3_work.lhs_aff);
    wbIK3_refine(wbIK3_work.rhs, wbIK3_work.lhs_aff);
    /* Centering plus corrector directions. */
    wbIK3_fillrhs_cc();
    wbIK3_ldl_wbIK3_solve(wbIK3_work.rhs, wbIK3_work.lhs_cc);
    wbIK3_refine(wbIK3_work.rhs, wbIK3_work.lhs_cc);
    /* Add the two together and store in aff. */
    for (i = 0; i < 173; i++)
      wbIK3_work.lhs_aff[i] += wbIK3_work.lhs_cc[i];
    /* Rename aff to reflect its new meaning. */
    dx = wbIK3_work.lhs_aff;
    ds = wbIK3_work.lhs_aff + 35;
    dz = wbIK3_work.lhs_aff + 104;
    dy = wbIK3_work.lhs_aff + 173;
    /* Find min(min(ds./s), min(dz./z)). */
    minval = 0;
    for (i = 0; i < 69; i++)
      if (ds[i] < minval*wbIK3_work.s[i])
        minval = ds[i]/wbIK3_work.s[i];
    for (i = 0; i < 69; i++)
      if (dz[i] < minval*wbIK3_work.z[i])
        minval = dz[i]/wbIK3_work.z[i];
    /* Find alpha. */
    if (-0.99 < minval)
      alpha = 1;
    else
      alpha = -0.99/minval;
    /* Update the primal and dual variables. */
    for (i = 0; i < 35; i++)
      wbIK3_work.x[i] += alpha*dx[i];
    for (i = 0; i < 69; i++)
      wbIK3_work.s[i] += alpha*ds[i];
    for (i = 0; i < 69; i++)
      wbIK3_work.z[i] += alpha*dz[i];
    for (i = 0; i < 0; i++)
      wbIK3_work.y[i] += alpha*dy[i];
    wbIK3_work.gap = wbIK3_eval_gap();
    wbIK3_work.eq_resid_squared = wbIK3_calc_eq_resid_squared();
    wbIK3_work.ineq_resid_squared = wbIK3_calc_ineq_resid_squared();
#ifndef ZERO_LIBRARY_MODE
    if (wbIK3_settings.verbose) {
      wbIK3_work.optval = wbIK3_eval_objv();
      printf("%3d   %10.3e  %9.2e  %9.2e  %9.2e  % 6.4f\n",
          iter+1, wbIK3_work.optval, wbIK3_work.gap, sqrt(wbIK3_work.eq_resid_squared),
          sqrt(wbIK3_work.ineq_resid_squared), alpha);
    }
#endif
    /* Test termination conditions. Requires optimality, and satisfied */
    /* constraints. */
    if (   (wbIK3_work.gap < wbIK3_settings.eps)
        && (wbIK3_work.eq_resid_squared <= wbIK3_settings.resid_tol*wbIK3_settings.resid_tol)
        && (wbIK3_work.ineq_resid_squared <= wbIK3_settings.resid_tol*wbIK3_settings.resid_tol)
       ) {
      wbIK3_work.converged = 1;
      wbIK3_work.optval = wbIK3_eval_objv();
      return iter+1;
    }
  }
  return iter;
}
