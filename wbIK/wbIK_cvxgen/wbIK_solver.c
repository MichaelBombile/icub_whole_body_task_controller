/* Produced by CVXGEN, 2020-02-23 10:59:15 -0500.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.c. */
/* Description: Main solver file. */
#include "wbIK_solver.h"
double wbIK_eval_gap(void) {
  int i;
  double gap;
  gap = 0;
  for (i = 0; i < 58; i++)
    gap += wbIK_work.z[i]*wbIK_work.s[i];
  return gap;
}
void wbIK_set_defaults(void) {
  wbIK_settings.resid_tol = 1e-6;
  wbIK_settings.eps = 1e-4;
  wbIK_settings.max_iters = 25;
  wbIK_settings.refine_steps = 1;
  wbIK_settings.s_init = 1;
  wbIK_settings.z_init = 1;
  wbIK_settings.debug = 0;
  wbIK_settings.verbose = 1;
  wbIK_settings.verbose_refinement = 0;
  wbIK_settings.better_start = 1;
  wbIK_settings.kkt_reg = 1e-7;
}
void wbIK_setup_pointers(void) {
  wbIK_work.y = wbIK_work.x + 71;
  wbIK_work.s = wbIK_work.x + 107;
  wbIK_work.z = wbIK_work.x + 165;
  wbIK_vars.w = wbIK_work.x + 0;
  wbIK_vars.x = wbIK_work.x + 36;
}
void setup_indexed_wbIK_params(void) {
  /* In CVXGEN, you can say */
  /*   parameters */
  /*     A[i] (5,3), i=1..4 */
  /*   end */
  /* This function sets up A[2] to be a pointer to A_2, which is a length-15 */
  /* vector of doubles. */
  /* If you access parameters that you haven't defined in CVXGEN, the result */
  /* is undefined. */
  wbIK_params.Jt[1] = wbIK_params.Jt_1;
  wbIK_params.Jt[2] = wbIK_params.Jt_2;
  wbIK_params.Jt[3] = wbIK_params.Jt_3;
  wbIK_params.Jt[4] = wbIK_params.Jt_4;
  wbIK_params.Jt[5] = wbIK_params.Jt_5;
  wbIK_params.Jt[6] = wbIK_params.Jt_6;
  wbIK_params.Jt[7] = wbIK_params.Jt_7;
  wbIK_params.Jt[8] = wbIK_params.Jt_8;
  wbIK_params.Jt[9] = wbIK_params.Jt_9;
  wbIK_params.Jt[10] = wbIK_params.Jt_10;
  wbIK_params.Jt[11] = wbIK_params.Jt_11;
  wbIK_params.Jt[12] = wbIK_params.Jt_12;
  wbIK_params.Jt[13] = wbIK_params.Jt_13;
  wbIK_params.Jt[14] = wbIK_params.Jt_14;
  wbIK_params.Jt[15] = wbIK_params.Jt_15;
  wbIK_params.Jt[16] = wbIK_params.Jt_16;
  wbIK_params.Jt[17] = wbIK_params.Jt_17;
  wbIK_params.Jt[18] = wbIK_params.Jt_18;
  wbIK_params.Jt[19] = wbIK_params.Jt_19;
  wbIK_params.Jt[20] = wbIK_params.Jt_20;
  wbIK_params.Jt[21] = wbIK_params.Jt_21;
  wbIK_params.Jt[22] = wbIK_params.Jt_22;
  wbIK_params.Jt[23] = wbIK_params.Jt_23;
  wbIK_params.Jt[24] = wbIK_params.Jt_24;
  wbIK_params.Jt[25] = wbIK_params.Jt_25;
  wbIK_params.Jt[26] = wbIK_params.Jt_26;
  wbIK_params.Jt[27] = wbIK_params.Jt_27;
  wbIK_params.Jt[28] = wbIK_params.Jt_28;
  wbIK_params.Jt[29] = wbIK_params.Jt_29;
  wbIK_params.Jt[30] = wbIK_params.Jt_30;
  wbIK_params.Jt[31] = wbIK_params.Jt_31;
  wbIK_params.Jt[32] = wbIK_params.Jt_32;
  wbIK_params.Jt[33] = wbIK_params.Jt_33;
  wbIK_params.Jt[34] = wbIK_params.Jt_34;
  wbIK_params.Jt[35] = wbIK_params.Jt_35;
  wbIK_params.Jt[36] = wbIK_params.Jt_36;
}
void wbIK_setup_indexing(void) {
  wbIK_setup_pointers();
  setup_indexed_wbIK_params();
}
void wbIK_set_start(void) {
  int i;
  for (i = 0; i < 71; i++)
    wbIK_work.x[i] = 0;
  for (i = 0; i < 36; i++)
    wbIK_work.y[i] = 0;
  for (i = 0; i < 58; i++)
    wbIK_work.s[i] = (wbIK_work.h[i] > 0) ? wbIK_work.h[i] : wbIK_settings.s_init;
  for (i = 0; i < 58; i++)
    wbIK_work.z[i] = wbIK_settings.z_init;
}
double wbIK_eval_objv(void) {
  int i;
  double objv;
  /* Borrow space in wbIK_work.rhs. */
  wbIK_multbyP(wbIK_work.rhs, wbIK_work.x);
  objv = 0;
  for (i = 0; i < 71; i++)
    objv += wbIK_work.x[i]*wbIK_work.rhs[i];
  objv *= 0.5;
  for (i = 0; i < 71; i++)
    objv += wbIK_work.q[i]*wbIK_work.x[i];
  objv += wbIK_work.quad_441009807360[0];
  return objv;
}
void wbIK_fillrhs_aff(void) {
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = wbIK_work.rhs;
  r2 = wbIK_work.rhs + 71;
  r3 = wbIK_work.rhs + 129;
  r4 = wbIK_work.rhs + 187;
  /* r1 = -A^Ty - G^Tz - Px - q. */
  wbIK_multbymAT(r1, wbIK_work.y);
  wbIK_multbymGT(wbIK_work.buffer, wbIK_work.z);
  for (i = 0; i < 71; i++)
    r1[i] += wbIK_work.buffer[i];
  wbIK_multbyP(wbIK_work.buffer, wbIK_work.x);
  for (i = 0; i < 71; i++)
    r1[i] -= wbIK_work.buffer[i] + wbIK_work.q[i];
  /* r2 = -z. */
  for (i = 0; i < 58; i++)
    r2[i] = -wbIK_work.z[i];
  /* r3 = -Gx - s + h. */
  wbIK_multbymG(r3, wbIK_work.x);
  for (i = 0; i < 58; i++)
    r3[i] += -wbIK_work.s[i] + wbIK_work.h[i];
  /* r4 = -Ax + b. */
  wbIK_multbymA(r4, wbIK_work.x);
  for (i = 0; i < 36; i++)
    r4[i] += wbIK_work.b[i];
}
void wbIK_fillrhs_cc(void) {
  int i;
  double *r2;
  double *ds_aff, *dz_aff;
  double mu;
  double alpha;
  double sigma;
  double smu;
  double minval;
  r2 = wbIK_work.rhs + 71;
  ds_aff = wbIK_work.lhs_aff + 71;
  dz_aff = wbIK_work.lhs_aff + 129;
  mu = 0;
  for (i = 0; i < 58; i++)
    mu += wbIK_work.s[i]*wbIK_work.z[i];
  /* Don't finish calculating mu quite yet. */
  /* Find min(min(ds./s), min(dz./z)). */
  minval = 0;
  for (i = 0; i < 58; i++)
    if (ds_aff[i] < minval*wbIK_work.s[i])
      minval = ds_aff[i]/wbIK_work.s[i];
  for (i = 0; i < 58; i++)
    if (dz_aff[i] < minval*wbIK_work.z[i])
      minval = dz_aff[i]/wbIK_work.z[i];
  /* Find alpha. */
  if (-1 < minval)
      alpha = 1;
  else
      alpha = -1/minval;
  sigma = 0;
  for (i = 0; i < 58; i++)
    sigma += (wbIK_work.s[i] + alpha*ds_aff[i])*
      (wbIK_work.z[i] + alpha*dz_aff[i]);
  sigma /= mu;
  sigma = sigma*sigma*sigma;
  /* Finish calculating mu now. */
  mu *= 0.017241379310344827;
  smu = sigma*mu;
  /* Fill-in the rhs. */
  for (i = 0; i < 71; i++)
    wbIK_work.rhs[i] = 0;
  for (i = 129; i < 223; i++)
    wbIK_work.rhs[i] = 0;
  for (i = 0; i < 58; i++)
    r2[i] = wbIK_work.s_inv[i]*(smu - ds_aff[i]*dz_aff[i]);
}
void wbIK_refine(double *target, double *var) {
  int i, j;
  double *residual = wbIK_work.buffer;
  double norm2;
  double *new_var = wbIK_work.buffer2;
  for (j = 0; j < wbIK_settings.refine_steps; j++) {
    norm2 = 0;
    wbIK_matrix_multiply(residual, var);
    for (i = 0; i < 223; i++) {
      residual[i] = residual[i] - target[i];
      norm2 += residual[i]*residual[i];
    }
#ifndef ZERO_LIBRARY_MODE
    if (wbIK_settings.verbose_refinement) {
      if (j == 0)
        printf("Initial residual before refinement has norm squared %.6g.\n", norm2);
      else
        printf("After refinement we get squared norm %.6g.\n", norm2);
    }
#endif
    /* Solve to find new_var = KKT \ (target - A*var). */
    wbIK_ldl_wbIK_solve(residual, new_var);
    /* Update var += new_var, or var += KKT \ (target - A*var). */
    for (i = 0; i < 223; i++) {
      var[i] -= new_var[i];
    }
  }
#ifndef ZERO_LIBRARY_MODE
  if (wbIK_settings.verbose_refinement) {
    /* Check the residual once more, but only if we're reporting it, since */
    /* it's expensive. */
    norm2 = 0;
    wbIK_matrix_multiply(residual, var);
    for (i = 0; i < 223; i++) {
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
double wbIK_calc_ineq_resid_squared(void) {
  /* Calculates the norm ||-Gx - s + h||. */
  double norm2_squared;
  int i;
  /* Find -Gx. */
  wbIK_multbymG(wbIK_work.buffer, wbIK_work.x);
  /* Add -s + h. */
  for (i = 0; i < 58; i++)
    wbIK_work.buffer[i] += -wbIK_work.s[i] + wbIK_work.h[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 58; i++)
    norm2_squared += wbIK_work.buffer[i]*wbIK_work.buffer[i];
  return norm2_squared;
}
double wbIK_calc_eq_resid_squared(void) {
  /* Calculates the norm ||-Ax + b||. */
  double norm2_squared;
  int i;
  /* Find -Ax. */
  wbIK_multbymA(wbIK_work.buffer, wbIK_work.x);
  /* Add +b. */
  for (i = 0; i < 36; i++)
    wbIK_work.buffer[i] += wbIK_work.b[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 36; i++)
    norm2_squared += wbIK_work.buffer[i]*wbIK_work.buffer[i];
  return norm2_squared;
}
void wbIK_better_start(void) {
  /* Calculates a better starting point, using a similar approach to CVXOPT. */
  /* Not yet speed optimized. */
  int i;
  double *x, *s, *z, *y;
  double alpha;
  wbIK_work.block_33[0] = -1;
  /* Make sure sinvz is 1 to make hijacked KKT system ok. */
  for (i = 0; i < 58; i++)
    wbIK_work.s_inv_z[i] = 1;
  wbIK_fill_KKT();
  wbIK_ldl_factor();
  wbIK_fillrhs_start();
  /* Borrow wbIK_work.lhs_aff for the solution. */
  wbIK_ldl_wbIK_solve(wbIK_work.rhs, wbIK_work.lhs_aff);
  /* Don't do any refinement for now. Precision doesn't matter too much. */
  x = wbIK_work.lhs_aff;
  s = wbIK_work.lhs_aff + 71;
  z = wbIK_work.lhs_aff + 129;
  y = wbIK_work.lhs_aff + 187;
  /* Just set x and y as is. */
  for (i = 0; i < 71; i++)
    wbIK_work.x[i] = x[i];
  for (i = 0; i < 36; i++)
    wbIK_work.y[i] = y[i];
  /* Now complete the initialization. Start with s. */
  /* Must have alpha > max(z). */
  alpha = -1e99;
  for (i = 0; i < 58; i++)
    if (alpha < z[i])
      alpha = z[i];
  if (alpha < 0) {
    for (i = 0; i < 58; i++)
      wbIK_work.s[i] = -z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 58; i++)
      wbIK_work.s[i] = -z[i] + alpha;
  }
  /* Now initialize z. */
  /* Now must have alpha > max(-z). */
  alpha = -1e99;
  for (i = 0; i < 58; i++)
    if (alpha < -z[i])
      alpha = -z[i];
  if (alpha < 0) {
    for (i = 0; i < 58; i++)
      wbIK_work.z[i] = z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 58; i++)
      wbIK_work.z[i] = z[i] + alpha;
  }
}
void wbIK_fillrhs_start(void) {
  /* Fill rhs with (-q, 0, h, b). */
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = wbIK_work.rhs;
  r2 = wbIK_work.rhs + 71;
  r3 = wbIK_work.rhs + 129;
  r4 = wbIK_work.rhs + 187;
  for (i = 0; i < 71; i++)
    r1[i] = -wbIK_work.q[i];
  for (i = 0; i < 58; i++)
    r2[i] = 0;
  for (i = 0; i < 58; i++)
    r3[i] = wbIK_work.h[i];
  for (i = 0; i < 36; i++)
    r4[i] = wbIK_work.b[i];
}
long wbIK_solve(void) {
  int i;
  int iter;
  double *dx, *ds, *dy, *dz;
  double minval;
  double alpha;
  wbIK_work.converged = 0;
  wbIK_setup_pointers();
  wbIK_pre_ops();
#ifndef ZERO_LIBRARY_MODE
  if (wbIK_settings.verbose)
    printf("iter     objv        gap       |Ax-b|    |Gx+s-h|    step\n");
#endif
  wbIK_fillq();
  wbIK_fillh();
  wbIK_fillb();
  if (wbIK_settings.better_start)
    wbIK_better_start();
  else
    wbIK_set_start();
  for (iter = 0; iter < wbIK_settings.max_iters; iter++) {
    for (i = 0; i < 58; i++) {
      wbIK_work.s_inv[i] = 1.0 / wbIK_work.s[i];
      wbIK_work.s_inv_z[i] = wbIK_work.s_inv[i]*wbIK_work.z[i];
    }
    wbIK_work.block_33[0] = 0;
    wbIK_fill_KKT();
    wbIK_ldl_factor();
    /* Affine scaling directions. */
    wbIK_fillrhs_aff();
    wbIK_ldl_wbIK_solve(wbIK_work.rhs, wbIK_work.lhs_aff);
    wbIK_refine(wbIK_work.rhs, wbIK_work.lhs_aff);
    /* Centering plus corrector directions. */
    wbIK_fillrhs_cc();
    wbIK_ldl_wbIK_solve(wbIK_work.rhs, wbIK_work.lhs_cc);
    wbIK_refine(wbIK_work.rhs, wbIK_work.lhs_cc);
    /* Add the two together and store in aff. */
    for (i = 0; i < 223; i++)
      wbIK_work.lhs_aff[i] += wbIK_work.lhs_cc[i];
    /* Rename aff to reflect its new meaning. */
    dx = wbIK_work.lhs_aff;
    ds = wbIK_work.lhs_aff + 71;
    dz = wbIK_work.lhs_aff + 129;
    dy = wbIK_work.lhs_aff + 187;
    /* Find min(min(ds./s), min(dz./z)). */
    minval = 0;
    for (i = 0; i < 58; i++)
      if (ds[i] < minval*wbIK_work.s[i])
        minval = ds[i]/wbIK_work.s[i];
    for (i = 0; i < 58; i++)
      if (dz[i] < minval*wbIK_work.z[i])
        minval = dz[i]/wbIK_work.z[i];
    /* Find alpha. */
    if (-0.99 < minval)
      alpha = 1;
    else
      alpha = -0.99/minval;
    /* Update the primal and dual variables. */
    for (i = 0; i < 71; i++)
      wbIK_work.x[i] += alpha*dx[i];
    for (i = 0; i < 58; i++)
      wbIK_work.s[i] += alpha*ds[i];
    for (i = 0; i < 58; i++)
      wbIK_work.z[i] += alpha*dz[i];
    for (i = 0; i < 36; i++)
      wbIK_work.y[i] += alpha*dy[i];
    wbIK_work.gap = wbIK_eval_gap();
    wbIK_work.eq_resid_squared = wbIK_calc_eq_resid_squared();
    wbIK_work.ineq_resid_squared = wbIK_calc_ineq_resid_squared();
#ifndef ZERO_LIBRARY_MODE
    if (wbIK_settings.verbose) {
      wbIK_work.optval = wbIK_eval_objv();
      printf("%3d   %10.3e  %9.2e  %9.2e  %9.2e  % 6.4f\n",
          iter+1, wbIK_work.optval, wbIK_work.gap, sqrt(wbIK_work.eq_resid_squared),
          sqrt(wbIK_work.ineq_resid_squared), alpha);
    }
#endif
    /* Test termination conditions. Requires optimality, and satisfied */
    /* constraints. */
    if (   (wbIK_work.gap < wbIK_settings.eps)
        && (wbIK_work.eq_resid_squared <= wbIK_settings.resid_tol*wbIK_settings.resid_tol)
        && (wbIK_work.ineq_resid_squared <= wbIK_settings.resid_tol*wbIK_settings.resid_tol)
       ) {
      wbIK_work.converged = 1;
      wbIK_work.optval = wbIK_eval_objv();
      return iter+1;
    }
  }
  return iter;
}
