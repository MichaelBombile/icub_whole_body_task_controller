/* Produced by CVXGEN, 2020-08-23 02:50:50 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.c. */
/* Description: Main solver file. */
#include "legIK_solver.h"
double legIK_eval_gap(void) {
  int i;
  double gap;
  gap = 0;
  for (i = 0; i < 30; i++)
    gap += legIK_work.z[i]*legIK_work.s[i];
  return gap;
}
void legIK_set_defaults(void) {
  legIK_settings.resid_tol = 1e-6;
  legIK_settings.eps = 1e-4;
  legIK_settings.max_iters = 25;
  legIK_settings.refine_steps = 1;
  legIK_settings.s_init = 1;
  legIK_settings.z_init = 1;
  legIK_settings.debug = 0;
  legIK_settings.verbose = 1;
  legIK_settings.verbose_refinement = 0;
  legIK_settings.better_start = 1;
  legIK_settings.kkt_reg = 1e-7;
}
void legIK_setup_pointers(void) {
  legIK_work.y = legIK_work.x + 18;
  legIK_work.s = legIK_work.x + 18;
  legIK_work.z = legIK_work.x + 48;
  legIK_vars.qdot = legIK_work.x + 0;
}
void setup_indexed_legIK_params(void) {
  /* In CVXGEN, you can say */
  /*   parameters */
  /*     A[i] (5,3), i=1..4 */
  /*   end */
  /* This function sets up A[2] to be a pointer to A_2, which is a length-15 */
  /* vector of doubles. */
  /* If you access parameters that you haven't defined in CVXGEN, the result */
  /* is undefined. */
  legIK_params.NeJc[1] = legIK_params.NeJc_1;
  legIK_params.NeJc[2] = legIK_params.NeJc_2;
  legIK_params.NeJc[3] = legIK_params.NeJc_3;
  legIK_params.NeJc[4] = legIK_params.NeJc_4;
  legIK_params.NeJc[5] = legIK_params.NeJc_5;
  legIK_params.NeJc[6] = legIK_params.NeJc_6;
}
void legIK_setup_indexing(void) {
  legIK_setup_pointers();
  setup_indexed_legIK_params();
}
void legIK_set_start(void) {
  int i;
  for (i = 0; i < 18; i++)
    legIK_work.x[i] = 0;
  for (i = 0; i < 0; i++)
    legIK_work.y[i] = 0;
  for (i = 0; i < 30; i++)
    legIK_work.s[i] = (legIK_work.h[i] > 0) ? legIK_work.h[i] : legIK_settings.s_init;
  for (i = 0; i < 30; i++)
    legIK_work.z[i] = legIK_settings.z_init;
}
double legIK_eval_objv(void) {
  int i;
  double objv;
  /* Borrow space in legIK_work.rhs. */
  legIK_multbyP(legIK_work.rhs, legIK_work.x);
  objv = 0;
  for (i = 0; i < 18; i++)
    objv += legIK_work.x[i]*legIK_work.rhs[i];
  objv *= 0.5;
  for (i = 0; i < 18; i++)
    objv += legIK_work.q[i]*legIK_work.x[i];
  objv += 0;
  return objv;
}
void legIK_fillrhs_aff(void) {
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = legIK_work.rhs;
  r2 = legIK_work.rhs + 18;
  r3 = legIK_work.rhs + 48;
  r4 = legIK_work.rhs + 78;
  /* r1 = -A^Ty - G^Tz - Px - q. */
  legIK_multbymAT(r1, legIK_work.y);
  legIK_multbymGT(legIK_work.buffer, legIK_work.z);
  for (i = 0; i < 18; i++)
    r1[i] += legIK_work.buffer[i];
  legIK_multbyP(legIK_work.buffer, legIK_work.x);
  for (i = 0; i < 18; i++)
    r1[i] -= legIK_work.buffer[i] + legIK_work.q[i];
  /* r2 = -z. */
  for (i = 0; i < 30; i++)
    r2[i] = -legIK_work.z[i];
  /* r3 = -Gx - s + h. */
  legIK_multbymG(r3, legIK_work.x);
  for (i = 0; i < 30; i++)
    r3[i] += -legIK_work.s[i] + legIK_work.h[i];
  /* r4 = -Ax + b. */
  legIK_multbymA(r4, legIK_work.x);
  for (i = 0; i < 0; i++)
    r4[i] += legIK_work.b[i];
}
void legIK_fillrhs_cc(void) {
  int i;
  double *r2;
  double *ds_aff, *dz_aff;
  double mu;
  double alpha;
  double sigma;
  double smu;
  double minval;
  r2 = legIK_work.rhs + 18;
  ds_aff = legIK_work.lhs_aff + 18;
  dz_aff = legIK_work.lhs_aff + 48;
  mu = 0;
  for (i = 0; i < 30; i++)
    mu += legIK_work.s[i]*legIK_work.z[i];
  /* Don't finish calculating mu quite yet. */
  /* Find min(min(ds./s), min(dz./z)). */
  minval = 0;
  for (i = 0; i < 30; i++)
    if (ds_aff[i] < minval*legIK_work.s[i])
      minval = ds_aff[i]/legIK_work.s[i];
  for (i = 0; i < 30; i++)
    if (dz_aff[i] < minval*legIK_work.z[i])
      minval = dz_aff[i]/legIK_work.z[i];
  /* Find alpha. */
  if (-1 < minval)
      alpha = 1;
  else
      alpha = -1/minval;
  sigma = 0;
  for (i = 0; i < 30; i++)
    sigma += (legIK_work.s[i] + alpha*ds_aff[i])*
      (legIK_work.z[i] + alpha*dz_aff[i]);
  sigma /= mu;
  sigma = sigma*sigma*sigma;
  /* Finish calculating mu now. */
  mu *= 0.03333333333333333;
  smu = sigma*mu;
  /* Fill-in the rhs. */
  for (i = 0; i < 18; i++)
    legIK_work.rhs[i] = 0;
  for (i = 48; i < 78; i++)
    legIK_work.rhs[i] = 0;
  for (i = 0; i < 30; i++)
    r2[i] = legIK_work.s_inv[i]*(smu - ds_aff[i]*dz_aff[i]);
}
void legIK_refine(double *target, double *var) {
  int i, j;
  double *residual = legIK_work.buffer;
  double norm2;
  double *new_var = legIK_work.buffer2;
  for (j = 0; j < legIK_settings.refine_steps; j++) {
    norm2 = 0;
    legIK_matrix_multiply(residual, var);
    for (i = 0; i < 78; i++) {
      residual[i] = residual[i] - target[i];
      norm2 += residual[i]*residual[i];
    }
#ifndef ZERO_LIBRARY_MODE
    if (legIK_settings.verbose_refinement) {
      if (j == 0)
        printf("Initial residual before refinement has norm squared %.6g.\n", norm2);
      else
        printf("After refinement we get squared norm %.6g.\n", norm2);
    }
#endif
    /* Solve to find new_var = KKT \ (target - A*var). */
    legIK_ldl_legIK_solve(residual, new_var);
    /* Update var += new_var, or var += KKT \ (target - A*var). */
    for (i = 0; i < 78; i++) {
      var[i] -= new_var[i];
    }
  }
#ifndef ZERO_LIBRARY_MODE
  if (legIK_settings.verbose_refinement) {
    /* Check the residual once more, but only if we're reporting it, since */
    /* it's expensive. */
    norm2 = 0;
    legIK_matrix_multiply(residual, var);
    for (i = 0; i < 78; i++) {
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
double legIK_calc_ineq_resid_squared(void) {
  /* Calculates the norm ||-Gx - s + h||. */
  double norm2_squared;
  int i;
  /* Find -Gx. */
  legIK_multbymG(legIK_work.buffer, legIK_work.x);
  /* Add -s + h. */
  for (i = 0; i < 30; i++)
    legIK_work.buffer[i] += -legIK_work.s[i] + legIK_work.h[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 30; i++)
    norm2_squared += legIK_work.buffer[i]*legIK_work.buffer[i];
  return norm2_squared;
}
double legIK_calc_eq_resid_squared(void) {
  /* Calculates the norm ||-Ax + b||. */
  double norm2_squared;
  int i;
  /* Find -Ax. */
  legIK_multbymA(legIK_work.buffer, legIK_work.x);
  /* Add +b. */
  for (i = 0; i < 0; i++)
    legIK_work.buffer[i] += legIK_work.b[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 0; i++)
    norm2_squared += legIK_work.buffer[i]*legIK_work.buffer[i];
  return norm2_squared;
}
void legIK_better_start(void) {
  /* Calculates a better starting point, using a similar approach to CVXOPT. */
  /* Not yet speed optimized. */
  int i;
  double *x, *s, *z, *y;
  double alpha;
  legIK_work.block_33[0] = -1;
  /* Make sure sinvz is 1 to make hijacked KKT system ok. */
  for (i = 0; i < 30; i++)
    legIK_work.s_inv_z[i] = 1;
  legIK_fill_KKT();
  legIK_ldl_factor();
  legIK_fillrhs_start();
  /* Borrow legIK_work.lhs_aff for the solution. */
  legIK_ldl_legIK_solve(legIK_work.rhs, legIK_work.lhs_aff);
  /* Don't do any refinement for now. Precision doesn't matter too much. */
  x = legIK_work.lhs_aff;
  s = legIK_work.lhs_aff + 18;
  z = legIK_work.lhs_aff + 48;
  y = legIK_work.lhs_aff + 78;
  /* Just set x and y as is. */
  for (i = 0; i < 18; i++)
    legIK_work.x[i] = x[i];
  for (i = 0; i < 0; i++)
    legIK_work.y[i] = y[i];
  /* Now complete the initialization. Start with s. */
  /* Must have alpha > max(z). */
  alpha = -1e99;
  for (i = 0; i < 30; i++)
    if (alpha < z[i])
      alpha = z[i];
  if (alpha < 0) {
    for (i = 0; i < 30; i++)
      legIK_work.s[i] = -z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 30; i++)
      legIK_work.s[i] = -z[i] + alpha;
  }
  /* Now initialize z. */
  /* Now must have alpha > max(-z). */
  alpha = -1e99;
  for (i = 0; i < 30; i++)
    if (alpha < -z[i])
      alpha = -z[i];
  if (alpha < 0) {
    for (i = 0; i < 30; i++)
      legIK_work.z[i] = z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 30; i++)
      legIK_work.z[i] = z[i] + alpha;
  }
}
void legIK_fillrhs_start(void) {
  /* Fill rhs with (-q, 0, h, b). */
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = legIK_work.rhs;
  r2 = legIK_work.rhs + 18;
  r3 = legIK_work.rhs + 48;
  r4 = legIK_work.rhs + 78;
  for (i = 0; i < 18; i++)
    r1[i] = -legIK_work.q[i];
  for (i = 0; i < 30; i++)
    r2[i] = 0;
  for (i = 0; i < 30; i++)
    r3[i] = legIK_work.h[i];
  for (i = 0; i < 0; i++)
    r4[i] = legIK_work.b[i];
}
long legIK_solve(void) {
  int i;
  int iter;
  double *dx, *ds, *dy, *dz;
  double minval;
  double alpha;
  legIK_work.converged = 0;
  legIK_setup_pointers();
  legIK_pre_ops();
#ifndef ZERO_LIBRARY_MODE
  if (legIK_settings.verbose)
    printf("iter     objv        gap       |Ax-b|    |Gx+s-h|    step\n");
#endif
  legIK_fillq();
  legIK_fillh();
  legIK_fillb();
  if (legIK_settings.better_start)
    legIK_better_start();
  else
    legIK_set_start();
  for (iter = 0; iter < legIK_settings.max_iters; iter++) {
    for (i = 0; i < 30; i++) {
      legIK_work.s_inv[i] = 1.0 / legIK_work.s[i];
      legIK_work.s_inv_z[i] = legIK_work.s_inv[i]*legIK_work.z[i];
    }
    legIK_work.block_33[0] = 0;
    legIK_fill_KKT();
    legIK_ldl_factor();
    /* Affine scaling directions. */
    legIK_fillrhs_aff();
    legIK_ldl_legIK_solve(legIK_work.rhs, legIK_work.lhs_aff);
    legIK_refine(legIK_work.rhs, legIK_work.lhs_aff);
    /* Centering plus corrector directions. */
    legIK_fillrhs_cc();
    legIK_ldl_legIK_solve(legIK_work.rhs, legIK_work.lhs_cc);
    legIK_refine(legIK_work.rhs, legIK_work.lhs_cc);
    /* Add the two together and store in aff. */
    for (i = 0; i < 78; i++)
      legIK_work.lhs_aff[i] += legIK_work.lhs_cc[i];
    /* Rename aff to reflect its new meaning. */
    dx = legIK_work.lhs_aff;
    ds = legIK_work.lhs_aff + 18;
    dz = legIK_work.lhs_aff + 48;
    dy = legIK_work.lhs_aff + 78;
    /* Find min(min(ds./s), min(dz./z)). */
    minval = 0;
    for (i = 0; i < 30; i++)
      if (ds[i] < minval*legIK_work.s[i])
        minval = ds[i]/legIK_work.s[i];
    for (i = 0; i < 30; i++)
      if (dz[i] < minval*legIK_work.z[i])
        minval = dz[i]/legIK_work.z[i];
    /* Find alpha. */
    if (-0.99 < minval)
      alpha = 1;
    else
      alpha = -0.99/minval;
    /* Update the primal and dual variables. */
    for (i = 0; i < 18; i++)
      legIK_work.x[i] += alpha*dx[i];
    for (i = 0; i < 30; i++)
      legIK_work.s[i] += alpha*ds[i];
    for (i = 0; i < 30; i++)
      legIK_work.z[i] += alpha*dz[i];
    for (i = 0; i < 0; i++)
      legIK_work.y[i] += alpha*dy[i];
    legIK_work.gap = legIK_eval_gap();
    legIK_work.eq_resid_squared = legIK_calc_eq_resid_squared();
    legIK_work.ineq_resid_squared = legIK_calc_ineq_resid_squared();
#ifndef ZERO_LIBRARY_MODE
    if (legIK_settings.verbose) {
      legIK_work.optval = legIK_eval_objv();
      printf("%3d   %10.3e  %9.2e  %9.2e  %9.2e  % 6.4f\n",
          iter+1, legIK_work.optval, legIK_work.gap, sqrt(legIK_work.eq_resid_squared),
          sqrt(legIK_work.ineq_resid_squared), alpha);
    }
#endif
    /* Test termination conditions. Requires optimality, and satisfied */
    /* constraints. */
    if (   (legIK_work.gap < legIK_settings.eps)
        && (legIK_work.eq_resid_squared <= legIK_settings.resid_tol*legIK_settings.resid_tol)
        && (legIK_work.ineq_resid_squared <= legIK_settings.resid_tol*legIK_settings.resid_tol)
       ) {
      legIK_work.converged = 1;
      legIK_work.optval = legIK_eval_objv();
      return iter+1;
    }
  }
  return iter;
}
