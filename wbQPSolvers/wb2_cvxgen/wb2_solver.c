/* Produced by CVXGEN, 2019-04-27 12:39:34 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.c. */
/* Description: Main solver file. */
#include "wb2_solver.h"
double wb2_eval_gap(void) {
  int i;
  double gap;
  gap = 0;
  for (i = 0; i < 196; i++)
    gap += wb2_work.z[i]*wb2_work.s[i];
  return gap;
}
void wb2_set_defaults(void) {
  wb2_settings.resid_tol = 1e-6;
  wb2_settings.eps = 1e-4;
  wb2_settings.max_iters = 25;
  wb2_settings.refine_steps = 1;
  wb2_settings.s_init = 1;
  wb2_settings.z_init = 1;
  wb2_settings.debug = 0;
  wb2_settings.verbose = 1;
  wb2_settings.verbose_refinement = 0;
  wb2_settings.better_start = 1;
  wb2_settings.kkt_reg = 1e-7;
}
void wb2_setup_pointers(void) {
  wb2_work.y = wb2_work.x + 111;
  wb2_work.s = wb2_work.x + 211;
  wb2_work.z = wb2_work.x + 407;
  wb2_vars.w = wb2_work.x + 29;
  wb2_vars.x = wb2_work.x + 35;
  wb2_vars.y = wb2_work.x + 70;
  wb2_vars.z = wb2_work.x + 99;
}
void setup_indexed_wb2_params(void) {
  /* In CVXGEN, you can say */
  /*   parameters */
  /*     A[i] (5,3), i=1..4 */
  /*   end */
  /* This function sets up A[2] to be a pointer to A_2, which is a length-15 */
  /* vector of doubles. */
  /* If you access parameters that you haven't defined in CVXGEN, the result */
  /* is undefined. */
  wb2_params.M[1] = wb2_params.M_1;
  wb2_params.Jt[19] = wb2_params.Jt_19;
  wb2_params.Jt[25] = wb2_params.Jt_25;
  wb2_params.Jt[20] = wb2_params.Jt_20;
  wb2_params.Jt[26] = wb2_params.Jt_26;
  wb2_params.Jt[21] = wb2_params.Jt_21;
  wb2_params.Jt[27] = wb2_params.Jt_27;
  wb2_params.M[2] = wb2_params.M_2;
  wb2_params.M[3] = wb2_params.M_3;
  wb2_params.M[4] = wb2_params.M_4;
  wb2_params.Jt[22] = wb2_params.Jt_22;
  wb2_params.Jt[28] = wb2_params.Jt_28;
  wb2_params.Jt[23] = wb2_params.Jt_23;
  wb2_params.Jt[29] = wb2_params.Jt_29;
  wb2_params.Jt[24] = wb2_params.Jt_24;
  wb2_params.Jt[30] = wb2_params.Jt_30;
  wb2_params.M[5] = wb2_params.M_5;
  wb2_params.M[6] = wb2_params.M_6;
  wb2_params.M[7] = wb2_params.M_7;
  wb2_params.M[8] = wb2_params.M_8;
  wb2_params.M[9] = wb2_params.M_9;
  wb2_params.M[10] = wb2_params.M_10;
  wb2_params.M[11] = wb2_params.M_11;
  wb2_params.M[12] = wb2_params.M_12;
  wb2_params.M[13] = wb2_params.M_13;
  wb2_params.M[14] = wb2_params.M_14;
  wb2_params.M[15] = wb2_params.M_15;
  wb2_params.M[16] = wb2_params.M_16;
  wb2_params.M[17] = wb2_params.M_17;
  wb2_params.M[18] = wb2_params.M_18;
  wb2_params.M[19] = wb2_params.M_19;
  wb2_params.M[20] = wb2_params.M_20;
  wb2_params.M[21] = wb2_params.M_21;
  wb2_params.M[22] = wb2_params.M_22;
  wb2_params.M[23] = wb2_params.M_23;
  wb2_params.M[24] = wb2_params.M_24;
  wb2_params.M[25] = wb2_params.M_25;
  wb2_params.M[26] = wb2_params.M_26;
  wb2_params.M[27] = wb2_params.M_27;
  wb2_params.M[28] = wb2_params.M_28;
  wb2_params.M[29] = wb2_params.M_29;
  wb2_params.M[30] = wb2_params.M_30;
  wb2_params.M[31] = wb2_params.M_31;
  wb2_params.M[32] = wb2_params.M_32;
  wb2_params.M[33] = wb2_params.M_33;
  wb2_params.M[34] = wb2_params.M_34;
  wb2_params.M[35] = wb2_params.M_35;
  wb2_params.XJbL[1] = wb2_params.XJbL_1;
  wb2_params.XJbR[1] = wb2_params.XJbR_1;
  wb2_params.XJbL[2] = wb2_params.XJbL_2;
  wb2_params.XJbR[2] = wb2_params.XJbR_2;
  wb2_params.XJbL[3] = wb2_params.XJbL_3;
  wb2_params.XJbR[3] = wb2_params.XJbR_3;
  wb2_params.XJbL[4] = wb2_params.XJbL_4;
  wb2_params.XJbR[4] = wb2_params.XJbR_4;
  wb2_params.XJbL[5] = wb2_params.XJbL_5;
  wb2_params.XJbR[5] = wb2_params.XJbR_5;
  wb2_params.XJbL[6] = wb2_params.XJbL_6;
  wb2_params.XJbR[6] = wb2_params.XJbR_6;
  wb2_params.Jt[1] = wb2_params.Jt_1;
  wb2_params.Jt[2] = wb2_params.Jt_2;
  wb2_params.Jt[3] = wb2_params.Jt_3;
  wb2_params.Jt[4] = wb2_params.Jt_4;
  wb2_params.Jt[5] = wb2_params.Jt_5;
  wb2_params.Jt[6] = wb2_params.Jt_6;
  wb2_params.Jt[7] = wb2_params.Jt_7;
  wb2_params.Jt[8] = wb2_params.Jt_8;
  wb2_params.Jt[9] = wb2_params.Jt_9;
  wb2_params.Jt[10] = wb2_params.Jt_10;
  wb2_params.Jt[11] = wb2_params.Jt_11;
  wb2_params.Jt[12] = wb2_params.Jt_12;
  wb2_params.Jt[13] = wb2_params.Jt_13;
  wb2_params.Jt[14] = wb2_params.Jt_14;
  wb2_params.Jt[15] = wb2_params.Jt_15;
  wb2_params.Jt[16] = wb2_params.Jt_16;
  wb2_params.Jt[17] = wb2_params.Jt_17;
  wb2_params.Jt[18] = wb2_params.Jt_18;
  wb2_params.CL[1] = wb2_params.CL_1;
  wb2_params.CL[2] = wb2_params.CL_2;
  wb2_params.CL[3] = wb2_params.CL_3;
  wb2_params.CL[4] = wb2_params.CL_4;
  wb2_params.CL[5] = wb2_params.CL_5;
  wb2_params.CL[6] = wb2_params.CL_6;
  wb2_params.CL[7] = wb2_params.CL_7;
  wb2_params.CL[8] = wb2_params.CL_8;
  wb2_params.CL[9] = wb2_params.CL_9;
  wb2_params.CL[10] = wb2_params.CL_10;
  wb2_params.CL[11] = wb2_params.CL_11;
  wb2_params.CR[1] = wb2_params.CR_1;
  wb2_params.CR[2] = wb2_params.CR_2;
  wb2_params.CR[3] = wb2_params.CR_3;
  wb2_params.CR[4] = wb2_params.CR_4;
  wb2_params.CR[5] = wb2_params.CR_5;
  wb2_params.CR[6] = wb2_params.CR_6;
  wb2_params.CR[7] = wb2_params.CR_7;
  wb2_params.CR[8] = wb2_params.CR_8;
  wb2_params.CR[9] = wb2_params.CR_9;
  wb2_params.CR[10] = wb2_params.CR_10;
  wb2_params.CR[11] = wb2_params.CR_11;
}
void wb2_setup_indexing(void) {
  wb2_setup_pointers();
  setup_indexed_wb2_params();
}
void wb2_set_start(void) {
  int i;
  for (i = 0; i < 111; i++)
    wb2_work.x[i] = 0;
  for (i = 0; i < 100; i++)
    wb2_work.y[i] = 0;
  for (i = 0; i < 196; i++)
    wb2_work.s[i] = (wb2_work.h[i] > 0) ? wb2_work.h[i] : wb2_settings.s_init;
  for (i = 0; i < 196; i++)
    wb2_work.z[i] = wb2_settings.z_init;
}
double wb2_eval_objv(void) {
  int i;
  double objv;
  /* Borrow space in wb2_work.rhs. */
  wb2_multbyP(wb2_work.rhs, wb2_work.x);
  objv = 0;
  for (i = 0; i < 111; i++)
    objv += wb2_work.x[i]*wb2_work.rhs[i];
  objv *= 0.5;
  for (i = 0; i < 111; i++)
    objv += wb2_work.q[i]*wb2_work.x[i];
  objv += 0;
  return objv;
}
void wb2_fillrhs_aff(void) {
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = wb2_work.rhs;
  r2 = wb2_work.rhs + 111;
  r3 = wb2_work.rhs + 307;
  r4 = wb2_work.rhs + 503;
  /* r1 = -A^Ty - G^Tz - Px - q. */
  wb2_multbymAT(r1, wb2_work.y);
  wb2_multbymGT(wb2_work.buffer, wb2_work.z);
  for (i = 0; i < 111; i++)
    r1[i] += wb2_work.buffer[i];
  wb2_multbyP(wb2_work.buffer, wb2_work.x);
  for (i = 0; i < 111; i++)
    r1[i] -= wb2_work.buffer[i] + wb2_work.q[i];
  /* r2 = -z. */
  for (i = 0; i < 196; i++)
    r2[i] = -wb2_work.z[i];
  /* r3 = -Gx - s + h. */
  wb2_multbymG(r3, wb2_work.x);
  for (i = 0; i < 196; i++)
    r3[i] += -wb2_work.s[i] + wb2_work.h[i];
  /* r4 = -Ax + b. */
  wb2_multbymA(r4, wb2_work.x);
  for (i = 0; i < 100; i++)
    r4[i] += wb2_work.b[i];
}
void wb2_fillrhs_cc(void) {
  int i;
  double *r2;
  double *ds_aff, *dz_aff;
  double mu;
  double alpha;
  double sigma;
  double smu;
  double minval;
  r2 = wb2_work.rhs + 111;
  ds_aff = wb2_work.lhs_aff + 111;
  dz_aff = wb2_work.lhs_aff + 307;
  mu = 0;
  for (i = 0; i < 196; i++)
    mu += wb2_work.s[i]*wb2_work.z[i];
  /* Don't finish calculating mu quite yet. */
  /* Find min(min(ds./s), min(dz./z)). */
  minval = 0;
  for (i = 0; i < 196; i++)
    if (ds_aff[i] < minval*wb2_work.s[i])
      minval = ds_aff[i]/wb2_work.s[i];
  for (i = 0; i < 196; i++)
    if (dz_aff[i] < minval*wb2_work.z[i])
      minval = dz_aff[i]/wb2_work.z[i];
  /* Find alpha. */
  if (-1 < minval)
      alpha = 1;
  else
      alpha = -1/minval;
  sigma = 0;
  for (i = 0; i < 196; i++)
    sigma += (wb2_work.s[i] + alpha*ds_aff[i])*
      (wb2_work.z[i] + alpha*dz_aff[i]);
  sigma /= mu;
  sigma = sigma*sigma*sigma;
  /* Finish calculating mu now. */
  mu *= 0.00510204081632653;
  smu = sigma*mu;
  /* Fill-in the rhs. */
  for (i = 0; i < 111; i++)
    wb2_work.rhs[i] = 0;
  for (i = 307; i < 603; i++)
    wb2_work.rhs[i] = 0;
  for (i = 0; i < 196; i++)
    r2[i] = wb2_work.s_inv[i]*(smu - ds_aff[i]*dz_aff[i]);
}
void wb2_refine(double *target, double *var) {
  int i, j;
  double *residual = wb2_work.buffer;
  double norm2;
  double *new_var = wb2_work.buffer2;
  for (j = 0; j < wb2_settings.refine_steps; j++) {
    norm2 = 0;
    wb2_matrix_multiply(residual, var);
    for (i = 0; i < 603; i++) {
      residual[i] = residual[i] - target[i];
      norm2 += residual[i]*residual[i];
    }
#ifndef ZERO_LIBRARY_MODE
    if (wb2_settings.verbose_refinement) {
      if (j == 0)
        printf("Initial residual before refinement has norm squared %.6g.\n", norm2);
      else
        printf("After refinement we get squared norm %.6g.\n", norm2);
    }
#endif
    /* Solve to find new_var = KKT \ (target - A*var). */
    wb2_ldl_wb2_solve(residual, new_var);
    /* Update var += new_var, or var += KKT \ (target - A*var). */
    for (i = 0; i < 603; i++) {
      var[i] -= new_var[i];
    }
  }
#ifndef ZERO_LIBRARY_MODE
  if (wb2_settings.verbose_refinement) {
    /* Check the residual once more, but only if we're reporting it, since */
    /* it's expensive. */
    norm2 = 0;
    wb2_matrix_multiply(residual, var);
    for (i = 0; i < 603; i++) {
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
double wb2_calc_ineq_resid_squared(void) {
  /* Calculates the norm ||-Gx - s + h||. */
  double norm2_squared;
  int i;
  /* Find -Gx. */
  wb2_multbymG(wb2_work.buffer, wb2_work.x);
  /* Add -s + h. */
  for (i = 0; i < 196; i++)
    wb2_work.buffer[i] += -wb2_work.s[i] + wb2_work.h[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 196; i++)
    norm2_squared += wb2_work.buffer[i]*wb2_work.buffer[i];
  return norm2_squared;
}
double wb2_calc_eq_resid_squared(void) {
  /* Calculates the norm ||-Ax + b||. */
  double norm2_squared;
  int i;
  /* Find -Ax. */
  wb2_multbymA(wb2_work.buffer, wb2_work.x);
  /* Add +b. */
  for (i = 0; i < 100; i++)
    wb2_work.buffer[i] += wb2_work.b[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 100; i++)
    norm2_squared += wb2_work.buffer[i]*wb2_work.buffer[i];
  return norm2_squared;
}
void wb2_better_start(void) {
  /* Calculates a better starting point, using a similar approach to CVXOPT. */
  /* Not yet speed optimized. */
  int i;
  double *x, *s, *z, *y;
  double alpha;
  wb2_work.block_33[0] = -1;
  /* Make sure sinvz is 1 to make hijacked KKT system ok. */
  for (i = 0; i < 196; i++)
    wb2_work.s_inv_z[i] = 1;
  wb2_fill_KKT();
  wb2_ldl_factor();
  wb2_fillrhs_start();
  /* Borrow wb2_work.lhs_aff for the solution. */
  wb2_ldl_wb2_solve(wb2_work.rhs, wb2_work.lhs_aff);
  /* Don't do any refinement for now. Precision doesn't matter too much. */
  x = wb2_work.lhs_aff;
  s = wb2_work.lhs_aff + 111;
  z = wb2_work.lhs_aff + 307;
  y = wb2_work.lhs_aff + 503;
  /* Just set x and y as is. */
  for (i = 0; i < 111; i++)
    wb2_work.x[i] = x[i];
  for (i = 0; i < 100; i++)
    wb2_work.y[i] = y[i];
  /* Now complete the initialization. Start with s. */
  /* Must have alpha > max(z). */
  alpha = -1e99;
  for (i = 0; i < 196; i++)
    if (alpha < z[i])
      alpha = z[i];
  if (alpha < 0) {
    for (i = 0; i < 196; i++)
      wb2_work.s[i] = -z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 196; i++)
      wb2_work.s[i] = -z[i] + alpha;
  }
  /* Now initialize z. */
  /* Now must have alpha > max(-z). */
  alpha = -1e99;
  for (i = 0; i < 196; i++)
    if (alpha < -z[i])
      alpha = -z[i];
  if (alpha < 0) {
    for (i = 0; i < 196; i++)
      wb2_work.z[i] = z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 196; i++)
      wb2_work.z[i] = z[i] + alpha;
  }
}
void wb2_fillrhs_start(void) {
  /* Fill rhs with (-q, 0, h, b). */
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = wb2_work.rhs;
  r2 = wb2_work.rhs + 111;
  r3 = wb2_work.rhs + 307;
  r4 = wb2_work.rhs + 503;
  for (i = 0; i < 111; i++)
    r1[i] = -wb2_work.q[i];
  for (i = 0; i < 196; i++)
    r2[i] = 0;
  for (i = 0; i < 196; i++)
    r3[i] = wb2_work.h[i];
  for (i = 0; i < 100; i++)
    r4[i] = wb2_work.b[i];
}
long wb2_solve(void) {
  int i;
  int iter;
  double *dx, *ds, *dy, *dz;
  double minval;
  double alpha;
  wb2_work.converged = 0;
  wb2_setup_pointers();
  wb2_pre_ops();
#ifndef ZERO_LIBRARY_MODE
  if (wb2_settings.verbose)
    printf("iter     objv        gap       |Ax-b|    |Gx+s-h|    step\n");
#endif
  wb2_fillq();
  wb2_fillh();
  wb2_fillb();
  if (wb2_settings.better_start)
    wb2_better_start();
  else
    wb2_set_start();
  for (iter = 0; iter < wb2_settings.max_iters; iter++) {
    for (i = 0; i < 196; i++) {
      wb2_work.s_inv[i] = 1.0 / wb2_work.s[i];
      wb2_work.s_inv_z[i] = wb2_work.s_inv[i]*wb2_work.z[i];
    }
    wb2_work.block_33[0] = 0;
    wb2_fill_KKT();
    wb2_ldl_factor();
    /* Affine scaling directions. */
    wb2_fillrhs_aff();
    wb2_ldl_wb2_solve(wb2_work.rhs, wb2_work.lhs_aff);
    wb2_refine(wb2_work.rhs, wb2_work.lhs_aff);
    /* Centering plus corrector directions. */
    wb2_fillrhs_cc();
    wb2_ldl_wb2_solve(wb2_work.rhs, wb2_work.lhs_cc);
    wb2_refine(wb2_work.rhs, wb2_work.lhs_cc);
    /* Add the two together and store in aff. */
    for (i = 0; i < 603; i++)
      wb2_work.lhs_aff[i] += wb2_work.lhs_cc[i];
    /* Rename aff to reflect its new meaning. */
    dx = wb2_work.lhs_aff;
    ds = wb2_work.lhs_aff + 111;
    dz = wb2_work.lhs_aff + 307;
    dy = wb2_work.lhs_aff + 503;
    /* Find min(min(ds./s), min(dz./z)). */
    minval = 0;
    for (i = 0; i < 196; i++)
      if (ds[i] < minval*wb2_work.s[i])
        minval = ds[i]/wb2_work.s[i];
    for (i = 0; i < 196; i++)
      if (dz[i] < minval*wb2_work.z[i])
        minval = dz[i]/wb2_work.z[i];
    /* Find alpha. */
    if (-0.99 < minval)
      alpha = 1;
    else
      alpha = -0.99/minval;
    /* Update the primal and dual variables. */
    for (i = 0; i < 111; i++)
      wb2_work.x[i] += alpha*dx[i];
    for (i = 0; i < 196; i++)
      wb2_work.s[i] += alpha*ds[i];
    for (i = 0; i < 196; i++)
      wb2_work.z[i] += alpha*dz[i];
    for (i = 0; i < 100; i++)
      wb2_work.y[i] += alpha*dy[i];
    wb2_work.gap = wb2_eval_gap();
    wb2_work.eq_resid_squared = wb2_calc_eq_resid_squared();
    wb2_work.ineq_resid_squared = wb2_calc_ineq_resid_squared();
#ifndef ZERO_LIBRARY_MODE
    if (wb2_settings.verbose) {
      wb2_work.optval = wb2_eval_objv();
      printf("%3d   %10.3e  %9.2e  %9.2e  %9.2e  % 6.4f\n",
          iter+1, wb2_work.optval, wb2_work.gap, sqrt(wb2_work.eq_resid_squared),
          sqrt(wb2_work.ineq_resid_squared), alpha);
    }
#endif
    /* Test termination conditions. Requires optimality, and satisfied */
    /* constraints. */
    if (   (wb2_work.gap < wb2_settings.eps)
        && (wb2_work.eq_resid_squared <= wb2_settings.resid_tol*wb2_settings.resid_tol)
        && (wb2_work.ineq_resid_squared <= wb2_settings.resid_tol*wb2_settings.resid_tol)
       ) {
      wb2_work.converged = 1;
      wb2_work.optval = wb2_eval_objv();
      return iter+1;
    }
  }
  return iter;
}
