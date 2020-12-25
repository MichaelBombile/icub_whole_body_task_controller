/* Produced by CVXGEN, 2019-04-22 12:51:16 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.c. */
/* Description: Main solver file. */
#include "solver.h"
double eval_gap(void) {
  int i;
  double gap;
  gap = 0;
  for (i = 0; i < 196; i++)
    gap += work.z[i]*work.s[i];
  return gap;
}
void set_defaults(void) {
  settings.resid_tol = 1e-6;
  settings.eps = 1e-4;
  settings.max_iters = 25;
  settings.refine_steps = 1;
  settings.s_init = 1;
  settings.z_init = 1;
  settings.debug = 0;
  settings.verbose = 1;
  settings.verbose_refinement = 0;
  settings.better_start = 1;
  settings.kkt_reg = 1e-7;
}
void setup_pointers(void) {
  work.y = work.x + 141;
  work.s = work.x + 241;
  work.z = work.x + 437;
  vars.w = work.x + 0;
  vars.x = work.x + 65;
  vars.y = work.x + 100;
  vars.z = work.x + 129;
}
void setup_indexed_params(void) {
  /* In CVXGEN, you can say */
  /*   parameters */
  /*     A[i] (5,3), i=1..4 */
  /*   end */
  /* This function sets up A[2] to be a pointer to A_2, which is a length-15 */
  /* vector of doubles. */
  /* If you access parameters that you haven't defined in CVXGEN, the result */
  /* is undefined. */
  params.M[1] = params.M_1;
  params.Jt[19] = params.Jt_19;
  params.Jt[25] = params.Jt_25;
  params.Jt[20] = params.Jt_20;
  params.Jt[26] = params.Jt_26;
  params.Jt[21] = params.Jt_21;
  params.Jt[27] = params.Jt_27;
  params.M[2] = params.M_2;
  params.M[3] = params.M_3;
  params.M[4] = params.M_4;
  params.Jt[22] = params.Jt_22;
  params.Jt[28] = params.Jt_28;
  params.Jt[23] = params.Jt_23;
  params.Jt[29] = params.Jt_29;
  params.Jt[24] = params.Jt_24;
  params.Jt[30] = params.Jt_30;
  params.M[5] = params.M_5;
  params.M[6] = params.M_6;
  params.M[7] = params.M_7;
  params.M[8] = params.M_8;
  params.M[9] = params.M_9;
  params.M[10] = params.M_10;
  params.M[11] = params.M_11;
  params.M[12] = params.M_12;
  params.M[13] = params.M_13;
  params.M[14] = params.M_14;
  params.M[15] = params.M_15;
  params.M[16] = params.M_16;
  params.M[17] = params.M_17;
  params.M[18] = params.M_18;
  params.M[19] = params.M_19;
  params.M[20] = params.M_20;
  params.M[21] = params.M_21;
  params.M[22] = params.M_22;
  params.M[23] = params.M_23;
  params.M[24] = params.M_24;
  params.M[25] = params.M_25;
  params.M[26] = params.M_26;
  params.M[27] = params.M_27;
  params.M[28] = params.M_28;
  params.M[29] = params.M_29;
  params.M[30] = params.M_30;
  params.M[31] = params.M_31;
  params.M[32] = params.M_32;
  params.M[33] = params.M_33;
  params.M[34] = params.M_34;
  params.M[35] = params.M_35;
  params.XJbL[1] = params.XJbL_1;
  params.XJbR[1] = params.XJbR_1;
  params.XJbL[2] = params.XJbL_2;
  params.XJbR[2] = params.XJbR_2;
  params.XJbL[3] = params.XJbL_3;
  params.XJbR[3] = params.XJbR_3;
  params.XJbL[4] = params.XJbL_4;
  params.XJbR[4] = params.XJbR_4;
  params.XJbL[5] = params.XJbL_5;
  params.XJbR[5] = params.XJbR_5;
  params.XJbL[6] = params.XJbL_6;
  params.XJbR[6] = params.XJbR_6;
  params.Jt[1] = params.Jt_1;
  params.Jt[2] = params.Jt_2;
  params.Jt[3] = params.Jt_3;
  params.Jt[4] = params.Jt_4;
  params.Jt[5] = params.Jt_5;
  params.Jt[6] = params.Jt_6;
  params.Jt[7] = params.Jt_7;
  params.Jt[8] = params.Jt_8;
  params.Jt[9] = params.Jt_9;
  params.Jt[10] = params.Jt_10;
  params.Jt[11] = params.Jt_11;
  params.Jt[12] = params.Jt_12;
  params.Jt[13] = params.Jt_13;
  params.Jt[14] = params.Jt_14;
  params.Jt[15] = params.Jt_15;
  params.Jt[16] = params.Jt_16;
  params.Jt[17] = params.Jt_17;
  params.Jt[18] = params.Jt_18;
  params.CL[1] = params.CL_1;
  params.CL[2] = params.CL_2;
  params.CL[3] = params.CL_3;
  params.CL[4] = params.CL_4;
  params.CL[5] = params.CL_5;
  params.CL[6] = params.CL_6;
  params.CL[7] = params.CL_7;
  params.CL[8] = params.CL_8;
  params.CL[9] = params.CL_9;
  params.CL[10] = params.CL_10;
  params.CL[11] = params.CL_11;
  params.CR[1] = params.CR_1;
  params.CR[2] = params.CR_2;
  params.CR[3] = params.CR_3;
  params.CR[4] = params.CR_4;
  params.CR[5] = params.CR_5;
  params.CR[6] = params.CR_6;
  params.CR[7] = params.CR_7;
  params.CR[8] = params.CR_8;
  params.CR[9] = params.CR_9;
  params.CR[10] = params.CR_10;
  params.CR[11] = params.CR_11;
}
void setup_indexing(void) {
  setup_pointers();
  setup_indexed_params();
}
void set_start(void) {
  int i;
  for (i = 0; i < 141; i++)
    work.x[i] = 0;
  for (i = 0; i < 100; i++)
    work.y[i] = 0;
  for (i = 0; i < 196; i++)
    work.s[i] = (work.h[i] > 0) ? work.h[i] : settings.s_init;
  for (i = 0; i < 196; i++)
    work.z[i] = settings.z_init;
}
double eval_objv(void) {
  int i;
  double objv;
  /* Borrow space in work.rhs. */
  multbyP(work.rhs, work.x);
  objv = 0;
  for (i = 0; i < 141; i++)
    objv += work.x[i]*work.rhs[i];
  objv *= 0.5;
  for (i = 0; i < 141; i++)
    objv += work.q[i]*work.x[i];
  objv += 0;
  return objv;
}
void fillrhs_aff(void) {
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = work.rhs;
  r2 = work.rhs + 141;
  r3 = work.rhs + 337;
  r4 = work.rhs + 533;
  /* r1 = -A^Ty - G^Tz - Px - q. */
  multbymAT(r1, work.y);
  multbymGT(work.buffer, work.z);
  for (i = 0; i < 141; i++)
    r1[i] += work.buffer[i];
  multbyP(work.buffer, work.x);
  for (i = 0; i < 141; i++)
    r1[i] -= work.buffer[i] + work.q[i];
  /* r2 = -z. */
  for (i = 0; i < 196; i++)
    r2[i] = -work.z[i];
  /* r3 = -Gx - s + h. */
  multbymG(r3, work.x);
  for (i = 0; i < 196; i++)
    r3[i] += -work.s[i] + work.h[i];
  /* r4 = -Ax + b. */
  multbymA(r4, work.x);
  for (i = 0; i < 100; i++)
    r4[i] += work.b[i];
}
void fillrhs_cc(void) {
  int i;
  double *r2;
  double *ds_aff, *dz_aff;
  double mu;
  double alpha;
  double sigma;
  double smu;
  double minval;
  r2 = work.rhs + 141;
  ds_aff = work.lhs_aff + 141;
  dz_aff = work.lhs_aff + 337;
  mu = 0;
  for (i = 0; i < 196; i++)
    mu += work.s[i]*work.z[i];
  /* Don't finish calculating mu quite yet. */
  /* Find min(min(ds./s), min(dz./z)). */
  minval = 0;
  for (i = 0; i < 196; i++)
    if (ds_aff[i] < minval*work.s[i])
      minval = ds_aff[i]/work.s[i];
  for (i = 0; i < 196; i++)
    if (dz_aff[i] < minval*work.z[i])
      minval = dz_aff[i]/work.z[i];
  /* Find alpha. */
  if (-1 < minval)
      alpha = 1;
  else
      alpha = -1/minval;
  sigma = 0;
  for (i = 0; i < 196; i++)
    sigma += (work.s[i] + alpha*ds_aff[i])*
      (work.z[i] + alpha*dz_aff[i]);
  sigma /= mu;
  sigma = sigma*sigma*sigma;
  /* Finish calculating mu now. */
  mu *= 0.00510204081632653;
  smu = sigma*mu;
  /* Fill-in the rhs. */
  for (i = 0; i < 141; i++)
    work.rhs[i] = 0;
  for (i = 337; i < 633; i++)
    work.rhs[i] = 0;
  for (i = 0; i < 196; i++)
    r2[i] = work.s_inv[i]*(smu - ds_aff[i]*dz_aff[i]);
}
void refine(double *target, double *var) {
  int i, j;
  double *residual = work.buffer;
  double norm2;
  double *new_var = work.buffer2;
  for (j = 0; j < settings.refine_steps; j++) {
    norm2 = 0;
    matrix_multiply(residual, var);
    for (i = 0; i < 633; i++) {
      residual[i] = residual[i] - target[i];
      norm2 += residual[i]*residual[i];
    }
#ifndef ZERO_LIBRARY_MODE
    if (settings.verbose_refinement) {
      if (j == 0)
        printf("Initial residual before refinement has norm squared %.6g.\n", norm2);
      else
        printf("After refinement we get squared norm %.6g.\n", norm2);
    }
#endif
    /* Solve to find new_var = KKT \ (target - A*var). */
    ldl_solve(residual, new_var);
    /* Update var += new_var, or var += KKT \ (target - A*var). */
    for (i = 0; i < 633; i++) {
      var[i] -= new_var[i];
    }
  }
#ifndef ZERO_LIBRARY_MODE
  if (settings.verbose_refinement) {
    /* Check the residual once more, but only if we're reporting it, since */
    /* it's expensive. */
    norm2 = 0;
    matrix_multiply(residual, var);
    for (i = 0; i < 633; i++) {
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
double calc_ineq_resid_squared(void) {
  /* Calculates the norm ||-Gx - s + h||. */
  double norm2_squared;
  int i;
  /* Find -Gx. */
  multbymG(work.buffer, work.x);
  /* Add -s + h. */
  for (i = 0; i < 196; i++)
    work.buffer[i] += -work.s[i] + work.h[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 196; i++)
    norm2_squared += work.buffer[i]*work.buffer[i];
  return norm2_squared;
}
double calc_eq_resid_squared(void) {
  /* Calculates the norm ||-Ax + b||. */
  double norm2_squared;
  int i;
  /* Find -Ax. */
  multbymA(work.buffer, work.x);
  /* Add +b. */
  for (i = 0; i < 100; i++)
    work.buffer[i] += work.b[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 100; i++)
    norm2_squared += work.buffer[i]*work.buffer[i];
  return norm2_squared;
}
void better_start(void) {
  /* Calculates a better starting point, using a similar approach to CVXOPT. */
  /* Not yet speed optimized. */
  int i;
  double *x, *s, *z, *y;
  double alpha;
  work.block_33[0] = -1;
  /* Make sure sinvz is 1 to make hijacked KKT system ok. */
  for (i = 0; i < 196; i++)
    work.s_inv_z[i] = 1;
  fill_KKT();
  ldl_factor();
  fillrhs_start();
  /* Borrow work.lhs_aff for the solution. */
  ldl_solve(work.rhs, work.lhs_aff);
  /* Don't do any refinement for now. Precision doesn't matter too much. */
  x = work.lhs_aff;
  s = work.lhs_aff + 141;
  z = work.lhs_aff + 337;
  y = work.lhs_aff + 533;
  /* Just set x and y as is. */
  for (i = 0; i < 141; i++)
    work.x[i] = x[i];
  for (i = 0; i < 100; i++)
    work.y[i] = y[i];
  /* Now complete the initialization. Start with s. */
  /* Must have alpha > max(z). */
  alpha = -1e99;
  for (i = 0; i < 196; i++)
    if (alpha < z[i])
      alpha = z[i];
  if (alpha < 0) {
    for (i = 0; i < 196; i++)
      work.s[i] = -z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 196; i++)
      work.s[i] = -z[i] + alpha;
  }
  /* Now initialize z. */
  /* Now must have alpha > max(-z). */
  alpha = -1e99;
  for (i = 0; i < 196; i++)
    if (alpha < -z[i])
      alpha = -z[i];
  if (alpha < 0) {
    for (i = 0; i < 196; i++)
      work.z[i] = z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 196; i++)
      work.z[i] = z[i] + alpha;
  }
}
void fillrhs_start(void) {
  /* Fill rhs with (-q, 0, h, b). */
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = work.rhs;
  r2 = work.rhs + 141;
  r3 = work.rhs + 337;
  r4 = work.rhs + 533;
  for (i = 0; i < 141; i++)
    r1[i] = -work.q[i];
  for (i = 0; i < 196; i++)
    r2[i] = 0;
  for (i = 0; i < 196; i++)
    r3[i] = work.h[i];
  for (i = 0; i < 100; i++)
    r4[i] = work.b[i];
}
long solve(void) {
  int i;
  int iter;
  double *dx, *ds, *dy, *dz;
  double minval;
  double alpha;
  work.converged = 0;
  setup_pointers();
  pre_ops();
#ifndef ZERO_LIBRARY_MODE
  if (settings.verbose)
    printf("iter     objv        gap       |Ax-b|    |Gx+s-h|    step\n");
#endif
  fillq();
  fillh();
  fillb();
  if (settings.better_start)
    better_start();
  else
    set_start();
  for (iter = 0; iter < settings.max_iters; iter++) {
    for (i = 0; i < 196; i++) {
      work.s_inv[i] = 1.0 / work.s[i];
      work.s_inv_z[i] = work.s_inv[i]*work.z[i];
    }
    work.block_33[0] = 0;
    fill_KKT();
    ldl_factor();
    /* Affine scaling directions. */
    fillrhs_aff();
    ldl_solve(work.rhs, work.lhs_aff);
    refine(work.rhs, work.lhs_aff);
    /* Centering plus corrector directions. */
    fillrhs_cc();
    ldl_solve(work.rhs, work.lhs_cc);
    refine(work.rhs, work.lhs_cc);
    /* Add the two together and store in aff. */
    for (i = 0; i < 633; i++)
      work.lhs_aff[i] += work.lhs_cc[i];
    /* Rename aff to reflect its new meaning. */
    dx = work.lhs_aff;
    ds = work.lhs_aff + 141;
    dz = work.lhs_aff + 337;
    dy = work.lhs_aff + 533;
    /* Find min(min(ds./s), min(dz./z)). */
    minval = 0;
    for (i = 0; i < 196; i++)
      if (ds[i] < minval*work.s[i])
        minval = ds[i]/work.s[i];
    for (i = 0; i < 196; i++)
      if (dz[i] < minval*work.z[i])
        minval = dz[i]/work.z[i];
    /* Find alpha. */
    if (-0.99 < minval)
      alpha = 1;
    else
      alpha = -0.99/minval;
    /* Update the primal and dual variables. */
    for (i = 0; i < 141; i++)
      work.x[i] += alpha*dx[i];
    for (i = 0; i < 196; i++)
      work.s[i] += alpha*ds[i];
    for (i = 0; i < 196; i++)
      work.z[i] += alpha*dz[i];
    for (i = 0; i < 100; i++)
      work.y[i] += alpha*dy[i];
    work.gap = eval_gap();
    work.eq_resid_squared = calc_eq_resid_squared();
    work.ineq_resid_squared = calc_ineq_resid_squared();
#ifndef ZERO_LIBRARY_MODE
    if (settings.verbose) {
      work.optval = eval_objv();
      printf("%3d   %10.3e  %9.2e  %9.2e  %9.2e  % 6.4f\n",
          iter+1, work.optval, work.gap, sqrt(work.eq_resid_squared),
          sqrt(work.ineq_resid_squared), alpha);
    }
#endif
    /* Test termination conditions. Requires optimality, and satisfied */
    /* constraints. */
    if (   (work.gap < settings.eps)
        && (work.eq_resid_squared <= settings.resid_tol*settings.resid_tol)
        && (work.ineq_resid_squared <= settings.resid_tol*settings.resid_tol)
       ) {
      work.converged = 1;
      work.optval = eval_objv();
      return iter+1;
    }
  }
  return iter;
}
