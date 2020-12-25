/* Produced by CVXGEN, 2019-11-18 18:31:12 -0500.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: csolve.c. */
/* Description: mex-able file for running cvxgen solver. */
#include "mex.h"
#include "solver.h"
Vars vars;
Params params;
Workspace work;
Settings settings;
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  int i, j;
  mxArray *xm, *cell, *xm_cell;
  double *src;
  double *dest;
  double *dest_cell;
  int valid_vars;
  int steps;
  int this_var_errors;
  int warned_diags;
  int prepare_for_c = 0;
  int extra_solves;
  const char *status_names[] = {"optval", "gap", "steps", "converged"};
  mwSize dims1x1of1[1] = {1};
  mwSize dims[1];
  const char *var_names[] = {"w", "x", "y", "z"};
  const int num_var_names = 4;
  /* Avoid compiler warnings of unused variables by using a dummy assignment. */
  warned_diags = j = 0;
  extra_solves = 0;
  set_defaults();
  /* Check we got the right number of arguments. */
  if (nrhs == 0)
    mexErrMsgTxt("Not enough arguments: You need to specify at least the parameters.\n");
  if (nrhs > 1) {
    /* Assume that the second argument is the settings. */
    if (mxGetField(prhs[1], 0, "eps") != NULL)
      settings.eps = *mxGetPr(mxGetField(prhs[1], 0, "eps"));
    if (mxGetField(prhs[1], 0, "max_iters") != NULL)
      settings.max_iters = *mxGetPr(mxGetField(prhs[1], 0, "max_iters"));
    if (mxGetField(prhs[1], 0, "refine_steps") != NULL)
      settings.refine_steps = *mxGetPr(mxGetField(prhs[1], 0, "refine_steps"));
    if (mxGetField(prhs[1], 0, "verbose") != NULL)
      settings.verbose = *mxGetPr(mxGetField(prhs[1], 0, "verbose"));
    if (mxGetField(prhs[1], 0, "better_start") != NULL)
      settings.better_start = *mxGetPr(mxGetField(prhs[1], 0, "better_start"));
    if (mxGetField(prhs[1], 0, "verbose_refinement") != NULL)
      settings.verbose_refinement = *mxGetPr(mxGetField(prhs[1], 0,
            "verbose_refinement"));
    if (mxGetField(prhs[1], 0, "debug") != NULL)
      settings.debug = *mxGetPr(mxGetField(prhs[1], 0, "debug"));
    if (mxGetField(prhs[1], 0, "kkt_reg") != NULL)
      settings.kkt_reg = *mxGetPr(mxGetField(prhs[1], 0, "kkt_reg"));
    if (mxGetField(prhs[1], 0, "s_init") != NULL)
      settings.s_init = *mxGetPr(mxGetField(prhs[1], 0, "s_init"));
    if (mxGetField(prhs[1], 0, "z_init") != NULL)
      settings.z_init = *mxGetPr(mxGetField(prhs[1], 0, "z_init"));
    if (mxGetField(prhs[1], 0, "resid_tol") != NULL)
      settings.resid_tol = *mxGetPr(mxGetField(prhs[1], 0, "resid_tol"));
    if (mxGetField(prhs[1], 0, "extra_solves") != NULL)
      extra_solves = *mxGetPr(mxGetField(prhs[1], 0, "extra_solves"));
    else
      extra_solves = 0;
    if (mxGetField(prhs[1], 0, "prepare_for_c") != NULL)
      prepare_for_c = *mxGetPr(mxGetField(prhs[1], 0, "prepare_for_c"));
  }
  valid_vars = 0;
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "CL_1");
  if (xm == NULL) {
    /* Attempt to pull CL_1 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "CL");
    if (cell != NULL)
      xm = mxGetCell(cell, 0);
  }
  if (xm == NULL) {
    printf("could not find params.CL_1 or params.CL{1}.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("CL_1 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter CL_1 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter CL_1 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter CL_1 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.CL_1;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "CL_2");
  if (xm == NULL) {
    /* Attempt to pull CL_2 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "CL");
    if (cell != NULL)
      xm = mxGetCell(cell, 1);
  }
  if (xm == NULL) {
    printf("could not find params.CL_2 or params.CL{2}.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("CL_2 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter CL_2 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter CL_2 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter CL_2 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.CL_2;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "CL_3");
  if (xm == NULL) {
    /* Attempt to pull CL_3 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "CL");
    if (cell != NULL)
      xm = mxGetCell(cell, 2);
  }
  if (xm == NULL) {
    printf("could not find params.CL_3 or params.CL{3}.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("CL_3 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter CL_3 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter CL_3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter CL_3 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.CL_3;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "CL_4");
  if (xm == NULL) {
    /* Attempt to pull CL_4 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "CL");
    if (cell != NULL)
      xm = mxGetCell(cell, 3);
  }
  if (xm == NULL) {
    printf("could not find params.CL_4 or params.CL{4}.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("CL_4 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter CL_4 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter CL_4 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter CL_4 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.CL_4;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "CL_5");
  if (xm == NULL) {
    /* Attempt to pull CL_5 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "CL");
    if (cell != NULL)
      xm = mxGetCell(cell, 4);
  }
  if (xm == NULL) {
    printf("could not find params.CL_5 or params.CL{5}.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("CL_5 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter CL_5 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter CL_5 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter CL_5 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.CL_5;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "CL_6");
  if (xm == NULL) {
    /* Attempt to pull CL_6 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "CL");
    if (cell != NULL)
      xm = mxGetCell(cell, 5);
  }
  if (xm == NULL) {
    printf("could not find params.CL_6 or params.CL{6}.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("CL_6 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter CL_6 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter CL_6 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter CL_6 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.CL_6;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "CL_7");
  if (xm == NULL) {
    /* Attempt to pull CL_7 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "CL");
    if (cell != NULL)
      xm = mxGetCell(cell, 6);
  }
  if (xm == NULL) {
    printf("could not find params.CL_7 or params.CL{7}.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("CL_7 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter CL_7 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter CL_7 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter CL_7 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.CL_7;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "CL_8");
  if (xm == NULL) {
    /* Attempt to pull CL_8 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "CL");
    if (cell != NULL)
      xm = mxGetCell(cell, 7);
  }
  if (xm == NULL) {
    printf("could not find params.CL_8 or params.CL{8}.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("CL_8 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter CL_8 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter CL_8 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter CL_8 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.CL_8;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "CL_9");
  if (xm == NULL) {
    /* Attempt to pull CL_9 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "CL");
    if (cell != NULL)
      xm = mxGetCell(cell, 8);
  }
  if (xm == NULL) {
    printf("could not find params.CL_9 or params.CL{9}.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("CL_9 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter CL_9 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter CL_9 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter CL_9 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.CL_9;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "CL_10");
  if (xm == NULL) {
    /* Attempt to pull CL_10 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "CL");
    if (cell != NULL)
      xm = mxGetCell(cell, 9);
  }
  if (xm == NULL) {
    printf("could not find params.CL_10 or params.CL{10}.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("CL_10 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter CL_10 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter CL_10 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter CL_10 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.CL_10;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "CL_11");
  if (xm == NULL) {
    /* Attempt to pull CL_11 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "CL");
    if (cell != NULL)
      xm = mxGetCell(cell, 10);
  }
  if (xm == NULL) {
    printf("could not find params.CL_11 or params.CL{11}.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("CL_11 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter CL_11 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter CL_11 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter CL_11 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.CL_11;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "CR_1");
  if (xm == NULL) {
    /* Attempt to pull CR_1 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "CR");
    if (cell != NULL)
      xm = mxGetCell(cell, 0);
  }
  if (xm == NULL) {
    printf("could not find params.CR_1 or params.CR{1}.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("CR_1 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter CR_1 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter CR_1 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter CR_1 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.CR_1;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "CR_2");
  if (xm == NULL) {
    /* Attempt to pull CR_2 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "CR");
    if (cell != NULL)
      xm = mxGetCell(cell, 1);
  }
  if (xm == NULL) {
    printf("could not find params.CR_2 or params.CR{2}.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("CR_2 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter CR_2 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter CR_2 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter CR_2 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.CR_2;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "CR_3");
  if (xm == NULL) {
    /* Attempt to pull CR_3 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "CR");
    if (cell != NULL)
      xm = mxGetCell(cell, 2);
  }
  if (xm == NULL) {
    printf("could not find params.CR_3 or params.CR{3}.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("CR_3 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter CR_3 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter CR_3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter CR_3 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.CR_3;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "CR_4");
  if (xm == NULL) {
    /* Attempt to pull CR_4 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "CR");
    if (cell != NULL)
      xm = mxGetCell(cell, 3);
  }
  if (xm == NULL) {
    printf("could not find params.CR_4 or params.CR{4}.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("CR_4 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter CR_4 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter CR_4 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter CR_4 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.CR_4;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "CR_5");
  if (xm == NULL) {
    /* Attempt to pull CR_5 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "CR");
    if (cell != NULL)
      xm = mxGetCell(cell, 4);
  }
  if (xm == NULL) {
    printf("could not find params.CR_5 or params.CR{5}.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("CR_5 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter CR_5 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter CR_5 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter CR_5 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.CR_5;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "CR_6");
  if (xm == NULL) {
    /* Attempt to pull CR_6 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "CR");
    if (cell != NULL)
      xm = mxGetCell(cell, 5);
  }
  if (xm == NULL) {
    printf("could not find params.CR_6 or params.CR{6}.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("CR_6 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter CR_6 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter CR_6 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter CR_6 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.CR_6;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "CR_7");
  if (xm == NULL) {
    /* Attempt to pull CR_7 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "CR");
    if (cell != NULL)
      xm = mxGetCell(cell, 6);
  }
  if (xm == NULL) {
    printf("could not find params.CR_7 or params.CR{7}.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("CR_7 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter CR_7 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter CR_7 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter CR_7 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.CR_7;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "CR_8");
  if (xm == NULL) {
    /* Attempt to pull CR_8 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "CR");
    if (cell != NULL)
      xm = mxGetCell(cell, 7);
  }
  if (xm == NULL) {
    printf("could not find params.CR_8 or params.CR{8}.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("CR_8 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter CR_8 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter CR_8 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter CR_8 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.CR_8;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "CR_9");
  if (xm == NULL) {
    /* Attempt to pull CR_9 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "CR");
    if (cell != NULL)
      xm = mxGetCell(cell, 8);
  }
  if (xm == NULL) {
    printf("could not find params.CR_9 or params.CR{9}.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("CR_9 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter CR_9 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter CR_9 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter CR_9 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.CR_9;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "CR_10");
  if (xm == NULL) {
    /* Attempt to pull CR_10 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "CR");
    if (cell != NULL)
      xm = mxGetCell(cell, 9);
  }
  if (xm == NULL) {
    printf("could not find params.CR_10 or params.CR{10}.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("CR_10 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter CR_10 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter CR_10 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter CR_10 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.CR_10;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "CR_11");
  if (xm == NULL) {
    /* Attempt to pull CR_11 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "CR");
    if (cell != NULL)
      xm = mxGetCell(cell, 10);
  }
  if (xm == NULL) {
    printf("could not find params.CR_11 or params.CR{11}.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("CR_11 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter CR_11 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter CR_11 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter CR_11 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.CR_11;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Jt_1");
  if (xm == NULL) {
    /* Attempt to pull Jt_1 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jt");
    if (cell != NULL)
      xm = mxGetCell(cell, 0);
  }
  if (xm == NULL) {
    printf("could not find params.Jt_1 or params.Jt{1}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("Jt_1 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Jt_1 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Jt_1 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Jt_1 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Jt_1;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Jt_2");
  if (xm == NULL) {
    /* Attempt to pull Jt_2 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jt");
    if (cell != NULL)
      xm = mxGetCell(cell, 1);
  }
  if (xm == NULL) {
    printf("could not find params.Jt_2 or params.Jt{2}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("Jt_2 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Jt_2 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Jt_2 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Jt_2 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Jt_2;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Jt_3");
  if (xm == NULL) {
    /* Attempt to pull Jt_3 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jt");
    if (cell != NULL)
      xm = mxGetCell(cell, 2);
  }
  if (xm == NULL) {
    printf("could not find params.Jt_3 or params.Jt{3}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("Jt_3 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Jt_3 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Jt_3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Jt_3 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Jt_3;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Jt_4");
  if (xm == NULL) {
    /* Attempt to pull Jt_4 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jt");
    if (cell != NULL)
      xm = mxGetCell(cell, 3);
  }
  if (xm == NULL) {
    printf("could not find params.Jt_4 or params.Jt{4}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("Jt_4 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Jt_4 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Jt_4 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Jt_4 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Jt_4;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Jt_5");
  if (xm == NULL) {
    /* Attempt to pull Jt_5 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jt");
    if (cell != NULL)
      xm = mxGetCell(cell, 4);
  }
  if (xm == NULL) {
    printf("could not find params.Jt_5 or params.Jt{5}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("Jt_5 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Jt_5 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Jt_5 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Jt_5 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Jt_5;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Jt_6");
  if (xm == NULL) {
    /* Attempt to pull Jt_6 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jt");
    if (cell != NULL)
      xm = mxGetCell(cell, 5);
  }
  if (xm == NULL) {
    printf("could not find params.Jt_6 or params.Jt{6}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("Jt_6 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Jt_6 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Jt_6 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Jt_6 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Jt_6;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Jt_7");
  if (xm == NULL) {
    /* Attempt to pull Jt_7 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jt");
    if (cell != NULL)
      xm = mxGetCell(cell, 6);
  }
  if (xm == NULL) {
    printf("could not find params.Jt_7 or params.Jt{7}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("Jt_7 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Jt_7 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Jt_7 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Jt_7 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Jt_7;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Jt_8");
  if (xm == NULL) {
    /* Attempt to pull Jt_8 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jt");
    if (cell != NULL)
      xm = mxGetCell(cell, 7);
  }
  if (xm == NULL) {
    printf("could not find params.Jt_8 or params.Jt{8}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("Jt_8 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Jt_8 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Jt_8 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Jt_8 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Jt_8;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Jt_9");
  if (xm == NULL) {
    /* Attempt to pull Jt_9 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jt");
    if (cell != NULL)
      xm = mxGetCell(cell, 8);
  }
  if (xm == NULL) {
    printf("could not find params.Jt_9 or params.Jt{9}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("Jt_9 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Jt_9 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Jt_9 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Jt_9 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Jt_9;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Jt_10");
  if (xm == NULL) {
    /* Attempt to pull Jt_10 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jt");
    if (cell != NULL)
      xm = mxGetCell(cell, 9);
  }
  if (xm == NULL) {
    printf("could not find params.Jt_10 or params.Jt{10}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("Jt_10 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Jt_10 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Jt_10 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Jt_10 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Jt_10;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Jt_11");
  if (xm == NULL) {
    /* Attempt to pull Jt_11 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jt");
    if (cell != NULL)
      xm = mxGetCell(cell, 10);
  }
  if (xm == NULL) {
    printf("could not find params.Jt_11 or params.Jt{11}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("Jt_11 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Jt_11 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Jt_11 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Jt_11 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Jt_11;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Jt_12");
  if (xm == NULL) {
    /* Attempt to pull Jt_12 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jt");
    if (cell != NULL)
      xm = mxGetCell(cell, 11);
  }
  if (xm == NULL) {
    printf("could not find params.Jt_12 or params.Jt{12}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("Jt_12 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Jt_12 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Jt_12 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Jt_12 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Jt_12;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Jt_13");
  if (xm == NULL) {
    /* Attempt to pull Jt_13 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jt");
    if (cell != NULL)
      xm = mxGetCell(cell, 12);
  }
  if (xm == NULL) {
    printf("could not find params.Jt_13 or params.Jt{13}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("Jt_13 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Jt_13 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Jt_13 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Jt_13 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Jt_13;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Jt_14");
  if (xm == NULL) {
    /* Attempt to pull Jt_14 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jt");
    if (cell != NULL)
      xm = mxGetCell(cell, 13);
  }
  if (xm == NULL) {
    printf("could not find params.Jt_14 or params.Jt{14}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("Jt_14 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Jt_14 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Jt_14 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Jt_14 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Jt_14;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Jt_15");
  if (xm == NULL) {
    /* Attempt to pull Jt_15 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jt");
    if (cell != NULL)
      xm = mxGetCell(cell, 14);
  }
  if (xm == NULL) {
    printf("could not find params.Jt_15 or params.Jt{15}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("Jt_15 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Jt_15 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Jt_15 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Jt_15 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Jt_15;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Jt_16");
  if (xm == NULL) {
    /* Attempt to pull Jt_16 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jt");
    if (cell != NULL)
      xm = mxGetCell(cell, 15);
  }
  if (xm == NULL) {
    printf("could not find params.Jt_16 or params.Jt{16}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("Jt_16 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Jt_16 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Jt_16 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Jt_16 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Jt_16;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Jt_17");
  if (xm == NULL) {
    /* Attempt to pull Jt_17 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jt");
    if (cell != NULL)
      xm = mxGetCell(cell, 16);
  }
  if (xm == NULL) {
    printf("could not find params.Jt_17 or params.Jt{17}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("Jt_17 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Jt_17 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Jt_17 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Jt_17 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Jt_17;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Jt_18");
  if (xm == NULL) {
    /* Attempt to pull Jt_18 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jt");
    if (cell != NULL)
      xm = mxGetCell(cell, 17);
  }
  if (xm == NULL) {
    printf("could not find params.Jt_18 or params.Jt{18}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("Jt_18 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Jt_18 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Jt_18 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Jt_18 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Jt_18;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Jt_19");
  if (xm == NULL) {
    /* Attempt to pull Jt_19 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jt");
    if (cell != NULL)
      xm = mxGetCell(cell, 18);
  }
  if (xm == NULL) {
    printf("could not find params.Jt_19 or params.Jt{19}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("Jt_19 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Jt_19 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Jt_19 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Jt_19 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Jt_19;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Jt_20");
  if (xm == NULL) {
    /* Attempt to pull Jt_20 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jt");
    if (cell != NULL)
      xm = mxGetCell(cell, 19);
  }
  if (xm == NULL) {
    printf("could not find params.Jt_20 or params.Jt{20}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("Jt_20 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Jt_20 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Jt_20 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Jt_20 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Jt_20;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Jt_21");
  if (xm == NULL) {
    /* Attempt to pull Jt_21 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jt");
    if (cell != NULL)
      xm = mxGetCell(cell, 20);
  }
  if (xm == NULL) {
    printf("could not find params.Jt_21 or params.Jt{21}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("Jt_21 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Jt_21 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Jt_21 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Jt_21 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Jt_21;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Jt_22");
  if (xm == NULL) {
    /* Attempt to pull Jt_22 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jt");
    if (cell != NULL)
      xm = mxGetCell(cell, 21);
  }
  if (xm == NULL) {
    printf("could not find params.Jt_22 or params.Jt{22}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("Jt_22 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Jt_22 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Jt_22 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Jt_22 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Jt_22;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Jt_23");
  if (xm == NULL) {
    /* Attempt to pull Jt_23 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jt");
    if (cell != NULL)
      xm = mxGetCell(cell, 22);
  }
  if (xm == NULL) {
    printf("could not find params.Jt_23 or params.Jt{23}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("Jt_23 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Jt_23 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Jt_23 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Jt_23 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Jt_23;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Jt_24");
  if (xm == NULL) {
    /* Attempt to pull Jt_24 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jt");
    if (cell != NULL)
      xm = mxGetCell(cell, 23);
  }
  if (xm == NULL) {
    printf("could not find params.Jt_24 or params.Jt{24}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("Jt_24 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Jt_24 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Jt_24 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Jt_24 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Jt_24;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Jt_25");
  if (xm == NULL) {
    /* Attempt to pull Jt_25 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jt");
    if (cell != NULL)
      xm = mxGetCell(cell, 24);
  }
  if (xm == NULL) {
    printf("could not find params.Jt_25 or params.Jt{25}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("Jt_25 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Jt_25 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Jt_25 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Jt_25 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Jt_25;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Jt_26");
  if (xm == NULL) {
    /* Attempt to pull Jt_26 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jt");
    if (cell != NULL)
      xm = mxGetCell(cell, 25);
  }
  if (xm == NULL) {
    printf("could not find params.Jt_26 or params.Jt{26}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("Jt_26 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Jt_26 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Jt_26 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Jt_26 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Jt_26;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Jt_27");
  if (xm == NULL) {
    /* Attempt to pull Jt_27 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jt");
    if (cell != NULL)
      xm = mxGetCell(cell, 26);
  }
  if (xm == NULL) {
    printf("could not find params.Jt_27 or params.Jt{27}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("Jt_27 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Jt_27 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Jt_27 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Jt_27 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Jt_27;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Jt_28");
  if (xm == NULL) {
    /* Attempt to pull Jt_28 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jt");
    if (cell != NULL)
      xm = mxGetCell(cell, 27);
  }
  if (xm == NULL) {
    printf("could not find params.Jt_28 or params.Jt{28}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("Jt_28 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Jt_28 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Jt_28 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Jt_28 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Jt_28;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Jt_29");
  if (xm == NULL) {
    /* Attempt to pull Jt_29 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jt");
    if (cell != NULL)
      xm = mxGetCell(cell, 28);
  }
  if (xm == NULL) {
    printf("could not find params.Jt_29 or params.Jt{29}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("Jt_29 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Jt_29 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Jt_29 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Jt_29 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Jt_29;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Jt_30");
  if (xm == NULL) {
    /* Attempt to pull Jt_30 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Jt");
    if (cell != NULL)
      xm = mxGetCell(cell, 29);
  }
  if (xm == NULL) {
    printf("could not find params.Jt_30 or params.Jt{30}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("Jt_30 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Jt_30 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Jt_30 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Jt_30 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Jt_30;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "M_1");
  if (xm == NULL) {
    /* Attempt to pull M_1 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "M");
    if (cell != NULL)
      xm = mxGetCell(cell, 0);
  }
  if (xm == NULL) {
    printf("could not find params.M_1 or params.M{1}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("M_1 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter M_1 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter M_1 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter M_1 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.M_1;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "M_2");
  if (xm == NULL) {
    /* Attempt to pull M_2 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "M");
    if (cell != NULL)
      xm = mxGetCell(cell, 1);
  }
  if (xm == NULL) {
    printf("could not find params.M_2 or params.M{2}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("M_2 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter M_2 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter M_2 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter M_2 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.M_2;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "M_3");
  if (xm == NULL) {
    /* Attempt to pull M_3 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "M");
    if (cell != NULL)
      xm = mxGetCell(cell, 2);
  }
  if (xm == NULL) {
    printf("could not find params.M_3 or params.M{3}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("M_3 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter M_3 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter M_3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter M_3 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.M_3;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "M_4");
  if (xm == NULL) {
    /* Attempt to pull M_4 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "M");
    if (cell != NULL)
      xm = mxGetCell(cell, 3);
  }
  if (xm == NULL) {
    printf("could not find params.M_4 or params.M{4}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("M_4 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter M_4 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter M_4 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter M_4 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.M_4;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "M_5");
  if (xm == NULL) {
    /* Attempt to pull M_5 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "M");
    if (cell != NULL)
      xm = mxGetCell(cell, 4);
  }
  if (xm == NULL) {
    printf("could not find params.M_5 or params.M{5}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("M_5 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter M_5 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter M_5 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter M_5 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.M_5;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "M_6");
  if (xm == NULL) {
    /* Attempt to pull M_6 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "M");
    if (cell != NULL)
      xm = mxGetCell(cell, 5);
  }
  if (xm == NULL) {
    printf("could not find params.M_6 or params.M{6}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("M_6 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter M_6 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter M_6 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter M_6 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.M_6;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "M_7");
  if (xm == NULL) {
    /* Attempt to pull M_7 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "M");
    if (cell != NULL)
      xm = mxGetCell(cell, 6);
  }
  if (xm == NULL) {
    printf("could not find params.M_7 or params.M{7}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("M_7 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter M_7 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter M_7 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter M_7 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.M_7;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "M_8");
  if (xm == NULL) {
    /* Attempt to pull M_8 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "M");
    if (cell != NULL)
      xm = mxGetCell(cell, 7);
  }
  if (xm == NULL) {
    printf("could not find params.M_8 or params.M{8}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("M_8 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter M_8 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter M_8 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter M_8 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.M_8;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "M_9");
  if (xm == NULL) {
    /* Attempt to pull M_9 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "M");
    if (cell != NULL)
      xm = mxGetCell(cell, 8);
  }
  if (xm == NULL) {
    printf("could not find params.M_9 or params.M{9}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("M_9 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter M_9 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter M_9 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter M_9 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.M_9;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "M_10");
  if (xm == NULL) {
    /* Attempt to pull M_10 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "M");
    if (cell != NULL)
      xm = mxGetCell(cell, 9);
  }
  if (xm == NULL) {
    printf("could not find params.M_10 or params.M{10}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("M_10 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter M_10 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter M_10 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter M_10 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.M_10;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "M_11");
  if (xm == NULL) {
    /* Attempt to pull M_11 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "M");
    if (cell != NULL)
      xm = mxGetCell(cell, 10);
  }
  if (xm == NULL) {
    printf("could not find params.M_11 or params.M{11}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("M_11 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter M_11 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter M_11 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter M_11 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.M_11;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "M_12");
  if (xm == NULL) {
    /* Attempt to pull M_12 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "M");
    if (cell != NULL)
      xm = mxGetCell(cell, 11);
  }
  if (xm == NULL) {
    printf("could not find params.M_12 or params.M{12}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("M_12 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter M_12 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter M_12 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter M_12 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.M_12;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "M_13");
  if (xm == NULL) {
    /* Attempt to pull M_13 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "M");
    if (cell != NULL)
      xm = mxGetCell(cell, 12);
  }
  if (xm == NULL) {
    printf("could not find params.M_13 or params.M{13}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("M_13 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter M_13 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter M_13 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter M_13 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.M_13;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "M_14");
  if (xm == NULL) {
    /* Attempt to pull M_14 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "M");
    if (cell != NULL)
      xm = mxGetCell(cell, 13);
  }
  if (xm == NULL) {
    printf("could not find params.M_14 or params.M{14}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("M_14 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter M_14 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter M_14 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter M_14 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.M_14;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "M_15");
  if (xm == NULL) {
    /* Attempt to pull M_15 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "M");
    if (cell != NULL)
      xm = mxGetCell(cell, 14);
  }
  if (xm == NULL) {
    printf("could not find params.M_15 or params.M{15}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("M_15 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter M_15 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter M_15 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter M_15 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.M_15;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "M_16");
  if (xm == NULL) {
    /* Attempt to pull M_16 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "M");
    if (cell != NULL)
      xm = mxGetCell(cell, 15);
  }
  if (xm == NULL) {
    printf("could not find params.M_16 or params.M{16}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("M_16 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter M_16 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter M_16 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter M_16 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.M_16;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "M_17");
  if (xm == NULL) {
    /* Attempt to pull M_17 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "M");
    if (cell != NULL)
      xm = mxGetCell(cell, 16);
  }
  if (xm == NULL) {
    printf("could not find params.M_17 or params.M{17}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("M_17 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter M_17 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter M_17 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter M_17 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.M_17;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "M_18");
  if (xm == NULL) {
    /* Attempt to pull M_18 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "M");
    if (cell != NULL)
      xm = mxGetCell(cell, 17);
  }
  if (xm == NULL) {
    printf("could not find params.M_18 or params.M{18}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("M_18 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter M_18 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter M_18 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter M_18 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.M_18;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "M_19");
  if (xm == NULL) {
    /* Attempt to pull M_19 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "M");
    if (cell != NULL)
      xm = mxGetCell(cell, 18);
  }
  if (xm == NULL) {
    printf("could not find params.M_19 or params.M{19}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("M_19 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter M_19 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter M_19 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter M_19 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.M_19;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "M_20");
  if (xm == NULL) {
    /* Attempt to pull M_20 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "M");
    if (cell != NULL)
      xm = mxGetCell(cell, 19);
  }
  if (xm == NULL) {
    printf("could not find params.M_20 or params.M{20}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("M_20 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter M_20 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter M_20 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter M_20 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.M_20;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "M_21");
  if (xm == NULL) {
    /* Attempt to pull M_21 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "M");
    if (cell != NULL)
      xm = mxGetCell(cell, 20);
  }
  if (xm == NULL) {
    printf("could not find params.M_21 or params.M{21}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("M_21 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter M_21 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter M_21 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter M_21 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.M_21;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "M_22");
  if (xm == NULL) {
    /* Attempt to pull M_22 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "M");
    if (cell != NULL)
      xm = mxGetCell(cell, 21);
  }
  if (xm == NULL) {
    printf("could not find params.M_22 or params.M{22}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("M_22 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter M_22 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter M_22 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter M_22 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.M_22;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "M_23");
  if (xm == NULL) {
    /* Attempt to pull M_23 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "M");
    if (cell != NULL)
      xm = mxGetCell(cell, 22);
  }
  if (xm == NULL) {
    printf("could not find params.M_23 or params.M{23}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("M_23 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter M_23 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter M_23 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter M_23 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.M_23;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "M_24");
  if (xm == NULL) {
    /* Attempt to pull M_24 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "M");
    if (cell != NULL)
      xm = mxGetCell(cell, 23);
  }
  if (xm == NULL) {
    printf("could not find params.M_24 or params.M{24}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("M_24 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter M_24 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter M_24 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter M_24 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.M_24;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "M_25");
  if (xm == NULL) {
    /* Attempt to pull M_25 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "M");
    if (cell != NULL)
      xm = mxGetCell(cell, 24);
  }
  if (xm == NULL) {
    printf("could not find params.M_25 or params.M{25}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("M_25 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter M_25 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter M_25 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter M_25 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.M_25;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "M_26");
  if (xm == NULL) {
    /* Attempt to pull M_26 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "M");
    if (cell != NULL)
      xm = mxGetCell(cell, 25);
  }
  if (xm == NULL) {
    printf("could not find params.M_26 or params.M{26}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("M_26 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter M_26 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter M_26 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter M_26 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.M_26;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "M_27");
  if (xm == NULL) {
    /* Attempt to pull M_27 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "M");
    if (cell != NULL)
      xm = mxGetCell(cell, 26);
  }
  if (xm == NULL) {
    printf("could not find params.M_27 or params.M{27}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("M_27 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter M_27 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter M_27 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter M_27 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.M_27;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "M_28");
  if (xm == NULL) {
    /* Attempt to pull M_28 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "M");
    if (cell != NULL)
      xm = mxGetCell(cell, 27);
  }
  if (xm == NULL) {
    printf("could not find params.M_28 or params.M{28}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("M_28 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter M_28 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter M_28 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter M_28 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.M_28;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "M_29");
  if (xm == NULL) {
    /* Attempt to pull M_29 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "M");
    if (cell != NULL)
      xm = mxGetCell(cell, 28);
  }
  if (xm == NULL) {
    printf("could not find params.M_29 or params.M{29}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("M_29 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter M_29 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter M_29 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter M_29 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.M_29;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "M_30");
  if (xm == NULL) {
    /* Attempt to pull M_30 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "M");
    if (cell != NULL)
      xm = mxGetCell(cell, 29);
  }
  if (xm == NULL) {
    printf("could not find params.M_30 or params.M{30}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("M_30 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter M_30 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter M_30 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter M_30 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.M_30;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "M_31");
  if (xm == NULL) {
    /* Attempt to pull M_31 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "M");
    if (cell != NULL)
      xm = mxGetCell(cell, 30);
  }
  if (xm == NULL) {
    printf("could not find params.M_31 or params.M{31}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("M_31 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter M_31 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter M_31 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter M_31 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.M_31;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "M_32");
  if (xm == NULL) {
    /* Attempt to pull M_32 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "M");
    if (cell != NULL)
      xm = mxGetCell(cell, 31);
  }
  if (xm == NULL) {
    printf("could not find params.M_32 or params.M{32}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("M_32 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter M_32 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter M_32 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter M_32 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.M_32;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "M_33");
  if (xm == NULL) {
    /* Attempt to pull M_33 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "M");
    if (cell != NULL)
      xm = mxGetCell(cell, 32);
  }
  if (xm == NULL) {
    printf("could not find params.M_33 or params.M{33}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("M_33 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter M_33 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter M_33 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter M_33 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.M_33;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "M_34");
  if (xm == NULL) {
    /* Attempt to pull M_34 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "M");
    if (cell != NULL)
      xm = mxGetCell(cell, 33);
  }
  if (xm == NULL) {
    printf("could not find params.M_34 or params.M{34}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("M_34 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter M_34 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter M_34 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter M_34 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.M_34;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "M_35");
  if (xm == NULL) {
    /* Attempt to pull M_35 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "M");
    if (cell != NULL)
      xm = mxGetCell(cell, 34);
  }
  if (xm == NULL) {
    printf("could not find params.M_35 or params.M{35}.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("M_35 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter M_35 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter M_35 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter M_35 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.M_35;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Qw");
  if (xm == NULL) {
    printf("could not find params.Qw.\n");
  } else {
    if (!((mxGetM(xm) == 70) && (mxGetN(xm) == 1))) {
      printf("Qw must be size (70,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Qw must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Qw must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Qw must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Qw;
      src = mxGetPr(xm);
      for (i = 0; i < 70; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Qx");
  if (xm == NULL) {
    printf("could not find params.Qx.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("Qx must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Qx must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Qx must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Qx must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Qx;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Qy");
  if (xm == NULL) {
    printf("could not find params.Qy.\n");
  } else {
    if (!((mxGetM(xm) == 29) && (mxGetN(xm) == 1))) {
      printf("Qy must be size (29,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Qy must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Qy must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Qy must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Qy;
      src = mxGetPr(xm);
      for (i = 0; i < 29; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Qz");
  if (xm == NULL) {
    printf("could not find params.Qz.\n");
  } else {
    if (!((mxGetM(xm) == 12) && (mxGetN(xm) == 1))) {
      printf("Qz must be size (12,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Qz must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Qz must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Qz must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Qz;
      src = mxGetPr(xm);
      for (i = 0; i < 12; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "T_max");
  if (xm == NULL) {
    printf("could not find params.T_max.\n");
  } else {
    if (!((mxGetM(xm) == 29) && (mxGetN(xm) == 1))) {
      printf("T_max must be size (29,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter T_max must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter T_max must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter T_max must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.T_max;
      src = mxGetPr(xm);
      for (i = 0; i < 29; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "XJbL_1");
  if (xm == NULL) {
    /* Attempt to pull XJbL_1 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "XJbL");
    if (cell != NULL)
      xm = mxGetCell(cell, 0);
  }
  if (xm == NULL) {
    printf("could not find params.XJbL_1 or params.XJbL{1}.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("XJbL_1 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter XJbL_1 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter XJbL_1 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter XJbL_1 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.XJbL_1;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "XJbL_2");
  if (xm == NULL) {
    /* Attempt to pull XJbL_2 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "XJbL");
    if (cell != NULL)
      xm = mxGetCell(cell, 1);
  }
  if (xm == NULL) {
    printf("could not find params.XJbL_2 or params.XJbL{2}.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("XJbL_2 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter XJbL_2 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter XJbL_2 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter XJbL_2 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.XJbL_2;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "XJbL_3");
  if (xm == NULL) {
    /* Attempt to pull XJbL_3 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "XJbL");
    if (cell != NULL)
      xm = mxGetCell(cell, 2);
  }
  if (xm == NULL) {
    printf("could not find params.XJbL_3 or params.XJbL{3}.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("XJbL_3 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter XJbL_3 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter XJbL_3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter XJbL_3 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.XJbL_3;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "XJbL_4");
  if (xm == NULL) {
    /* Attempt to pull XJbL_4 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "XJbL");
    if (cell != NULL)
      xm = mxGetCell(cell, 3);
  }
  if (xm == NULL) {
    printf("could not find params.XJbL_4 or params.XJbL{4}.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("XJbL_4 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter XJbL_4 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter XJbL_4 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter XJbL_4 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.XJbL_4;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "XJbL_5");
  if (xm == NULL) {
    /* Attempt to pull XJbL_5 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "XJbL");
    if (cell != NULL)
      xm = mxGetCell(cell, 4);
  }
  if (xm == NULL) {
    printf("could not find params.XJbL_5 or params.XJbL{5}.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("XJbL_5 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter XJbL_5 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter XJbL_5 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter XJbL_5 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.XJbL_5;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "XJbL_6");
  if (xm == NULL) {
    /* Attempt to pull XJbL_6 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "XJbL");
    if (cell != NULL)
      xm = mxGetCell(cell, 5);
  }
  if (xm == NULL) {
    printf("could not find params.XJbL_6 or params.XJbL{6}.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("XJbL_6 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter XJbL_6 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter XJbL_6 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter XJbL_6 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.XJbL_6;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "XJbR_1");
  if (xm == NULL) {
    /* Attempt to pull XJbR_1 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "XJbR");
    if (cell != NULL)
      xm = mxGetCell(cell, 0);
  }
  if (xm == NULL) {
    printf("could not find params.XJbR_1 or params.XJbR{1}.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("XJbR_1 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter XJbR_1 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter XJbR_1 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter XJbR_1 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.XJbR_1;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "XJbR_2");
  if (xm == NULL) {
    /* Attempt to pull XJbR_2 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "XJbR");
    if (cell != NULL)
      xm = mxGetCell(cell, 1);
  }
  if (xm == NULL) {
    printf("could not find params.XJbR_2 or params.XJbR{2}.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("XJbR_2 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter XJbR_2 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter XJbR_2 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter XJbR_2 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.XJbR_2;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "XJbR_3");
  if (xm == NULL) {
    /* Attempt to pull XJbR_3 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "XJbR");
    if (cell != NULL)
      xm = mxGetCell(cell, 2);
  }
  if (xm == NULL) {
    printf("could not find params.XJbR_3 or params.XJbR{3}.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("XJbR_3 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter XJbR_3 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter XJbR_3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter XJbR_3 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.XJbR_3;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "XJbR_4");
  if (xm == NULL) {
    /* Attempt to pull XJbR_4 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "XJbR");
    if (cell != NULL)
      xm = mxGetCell(cell, 3);
  }
  if (xm == NULL) {
    printf("could not find params.XJbR_4 or params.XJbR{4}.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("XJbR_4 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter XJbR_4 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter XJbR_4 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter XJbR_4 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.XJbR_4;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "XJbR_5");
  if (xm == NULL) {
    /* Attempt to pull XJbR_5 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "XJbR");
    if (cell != NULL)
      xm = mxGetCell(cell, 4);
  }
  if (xm == NULL) {
    printf("could not find params.XJbR_5 or params.XJbR{5}.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("XJbR_5 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter XJbR_5 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter XJbR_5 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter XJbR_5 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.XJbR_5;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "XJbR_6");
  if (xm == NULL) {
    /* Attempt to pull XJbR_6 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "XJbR");
    if (cell != NULL)
      xm = mxGetCell(cell, 5);
  }
  if (xm == NULL) {
    printf("could not find params.XJbR_6 or params.XJbR{6}.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("XJbR_6 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter XJbR_6 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter XJbR_6 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter XJbR_6 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.XJbR_6;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "aL");
  if (xm == NULL) {
    printf("could not find params.aL.\n");
  } else {
    if (!((mxGetM(xm) == 1) && (mxGetN(xm) == 1))) {
      printf("aL must be size (1,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter aL must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter aL must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter aL must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.aL;
      src = mxGetPr(xm);
      for (i = 0; i < 1; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "aR");
  if (xm == NULL) {
    printf("could not find params.aR.\n");
  } else {
    if (!((mxGetM(xm) == 1) && (mxGetN(xm) == 1))) {
      printf("aR must be size (1,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter aR must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter aR must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter aR must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.aR;
      src = mxGetPr(xm);
      for (i = 0; i < 1; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "aw");
  if (xm == NULL) {
    printf("could not find params.aw.\n");
  } else {
    if (!((mxGetM(xm) == 1) && (mxGetN(xm) == 1))) {
      printf("aw must be size (1,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter aw must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter aw must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter aw must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.aw;
      src = mxGetPr(xm);
      for (i = 0; i < 1; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "b1");
  if (xm == NULL) {
    printf("could not find params.b1.\n");
  } else {
    if (!((mxGetM(xm) == 35) && (mxGetN(xm) == 1))) {
      printf("b1 must be size (35,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter b1 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter b1 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter b1 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.b1;
      src = mxGetPr(xm);
      for (i = 0; i < 35; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "b2");
  if (xm == NULL) {
    printf("could not find params.b2.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("b2 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter b2 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter b2 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter b2 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.b2;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "b3");
  if (xm == NULL) {
    printf("could not find params.b3.\n");
  } else {
    if (!((mxGetM(xm) == 70) && (mxGetN(xm) == 1))) {
      printf("b3 must be size (70,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter b3 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter b3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter b3 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.b3;
      src = mxGetPr(xm);
      for (i = 0; i < 70; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "copMxl_1");
  if (xm == NULL) {
    /* Attempt to pull copMxl_1 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "copMxl");
    if (cell != NULL)
      xm = mxGetCell(cell, 0);
  }
  if (xm == NULL) {
    printf("could not find params.copMxl_1 or params.copMxl{1}.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("copMxl_1 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter copMxl_1 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter copMxl_1 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter copMxl_1 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.copMxl_1;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "copMxl_2");
  if (xm == NULL) {
    /* Attempt to pull copMxl_2 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "copMxl");
    if (cell != NULL)
      xm = mxGetCell(cell, 1);
  }
  if (xm == NULL) {
    printf("could not find params.copMxl_2 or params.copMxl{2}.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("copMxl_2 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter copMxl_2 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter copMxl_2 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter copMxl_2 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.copMxl_2;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "copMxr_1");
  if (xm == NULL) {
    /* Attempt to pull copMxr_1 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "copMxr");
    if (cell != NULL)
      xm = mxGetCell(cell, 0);
  }
  if (xm == NULL) {
    printf("could not find params.copMxr_1 or params.copMxr{1}.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("copMxr_1 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter copMxr_1 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter copMxr_1 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter copMxr_1 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.copMxr_1;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "copMxr_2");
  if (xm == NULL) {
    /* Attempt to pull copMxr_2 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "copMxr");
    if (cell != NULL)
      xm = mxGetCell(cell, 1);
  }
  if (xm == NULL) {
    printf("could not find params.copMxr_2 or params.copMxr{2}.\n");
  } else {
    if (!((mxGetM(xm) == 6) && (mxGetN(xm) == 1))) {
      printf("copMxr_2 must be size (6,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter copMxr_2 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter copMxr_2 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter copMxr_2 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.copMxr_2;
      src = mxGetPr(xm);
      for (i = 0; i < 6; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "dt");
  if (xm == NULL) {
    printf("could not find params.dt.\n");
  } else {
    if (!((mxGetM(xm) == 1) && (mxGetN(xm) == 1))) {
      printf("dt must be size (1,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter dt must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter dt must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter dt must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.dt;
      src = mxGetPr(xm);
      for (i = 0; i < 1; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "dwd");
  if (xm == NULL) {
    printf("could not find params.dwd.\n");
  } else {
    if (!((mxGetM(xm) == 1) && (mxGetN(xm) == 1))) {
      printf("dwd must be size (1,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter dwd must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter dwd must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter dwd must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.dwd;
      src = mxGetPr(xm);
      for (i = 0; i < 1; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "q_max");
  if (xm == NULL) {
    printf("could not find params.q_max.\n");
  } else {
    if (!((mxGetM(xm) == 29) && (mxGetN(xm) == 1))) {
      printf("q_max must be size (29,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter q_max must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter q_max must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter q_max must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.q_max;
      src = mxGetPr(xm);
      for (i = 0; i < 29; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "q_min");
  if (xm == NULL) {
    printf("could not find params.q_min.\n");
  } else {
    if (!((mxGetM(xm) == 29) && (mxGetN(xm) == 1))) {
      printf("q_min must be size (29,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter q_min must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter q_min must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter q_min must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.q_min;
      src = mxGetPr(xm);
      for (i = 0; i < 29; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "v_max");
  if (xm == NULL) {
    printf("could not find params.v_max.\n");
  } else {
    if (!((mxGetM(xm) == 29) && (mxGetN(xm) == 1))) {
      printf("v_max must be size (29,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter v_max must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter v_max must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter v_max must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.v_max;
      src = mxGetPr(xm);
      for (i = 0; i < 29; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  if (valid_vars != 119) {
    printf("Error: %d parameters are invalid.\n", 119 - valid_vars);
    mexErrMsgTxt("invalid parameters found.");
  }
  if (prepare_for_c) {
    printf("settings.prepare_for_c == 1. thus, outputting for C.\n");
    for (i = 0; i < 35; i++)
      printf("  params.Qx[%d] = %.6g;\n", i, params.Qx[i]);
    for (i = 0; i < 29; i++)
      printf("  params.Qy[%d] = %.6g;\n", i, params.Qy[i]);
    for (i = 0; i < 12; i++)
      printf("  params.Qz[%d] = %.6g;\n", i, params.Qz[i]);
    for (i = 0; i < 70; i++)
      printf("  params.Qw[%d] = %.6g;\n", i, params.Qw[i]);
    for (i = 0; i < 35; i++)
      printf("  params.M_1[%d] = %.6g;\n", i, params.M_1[i]);
    for (i = 0; i < 1; i++)
      printf("  params.aL[%d] = %.6g;\n", i, params.aL[i]);
    for (i = 0; i < 35; i++)
      printf("  params.Jt_19[%d] = %.6g;\n", i, params.Jt_19[i]);
    for (i = 0; i < 1; i++)
      printf("  params.aR[%d] = %.6g;\n", i, params.aR[i]);
    for (i = 0; i < 35; i++)
      printf("  params.Jt_25[%d] = %.6g;\n", i, params.Jt_25[i]);
    for (i = 0; i < 35; i++)
      printf("  params.Jt_20[%d] = %.6g;\n", i, params.Jt_20[i]);
    for (i = 0; i < 35; i++)
      printf("  params.Jt_26[%d] = %.6g;\n", i, params.Jt_26[i]);
    for (i = 0; i < 35; i++)
      printf("  params.Jt_21[%d] = %.6g;\n", i, params.Jt_21[i]);
    for (i = 0; i < 35; i++)
      printf("  params.Jt_27[%d] = %.6g;\n", i, params.Jt_27[i]);
    for (i = 0; i < 35; i++)
      printf("  params.b1[%d] = %.6g;\n", i, params.b1[i]);
    for (i = 0; i < 35; i++)
      printf("  params.M_2[%d] = %.6g;\n", i, params.M_2[i]);
    for (i = 0; i < 35; i++)
      printf("  params.M_3[%d] = %.6g;\n", i, params.M_3[i]);
    for (i = 0; i < 35; i++)
      printf("  params.M_4[%d] = %.6g;\n", i, params.M_4[i]);
    for (i = 0; i < 35; i++)
      printf("  params.Jt_22[%d] = %.6g;\n", i, params.Jt_22[i]);
    for (i = 0; i < 35; i++)
      printf("  params.Jt_28[%d] = %.6g;\n", i, params.Jt_28[i]);
    for (i = 0; i < 35; i++)
      printf("  params.Jt_23[%d] = %.6g;\n", i, params.Jt_23[i]);
    for (i = 0; i < 35; i++)
      printf("  params.Jt_29[%d] = %.6g;\n", i, params.Jt_29[i]);
    for (i = 0; i < 35; i++)
      printf("  params.Jt_24[%d] = %.6g;\n", i, params.Jt_24[i]);
    for (i = 0; i < 35; i++)
      printf("  params.Jt_30[%d] = %.6g;\n", i, params.Jt_30[i]);
    for (i = 0; i < 35; i++)
      printf("  params.M_5[%d] = %.6g;\n", i, params.M_5[i]);
    for (i = 0; i < 35; i++)
      printf("  params.M_6[%d] = %.6g;\n", i, params.M_6[i]);
    for (i = 0; i < 35; i++)
      printf("  params.M_7[%d] = %.6g;\n", i, params.M_7[i]);
    for (i = 0; i < 35; i++)
      printf("  params.M_8[%d] = %.6g;\n", i, params.M_8[i]);
    for (i = 0; i < 35; i++)
      printf("  params.M_9[%d] = %.6g;\n", i, params.M_9[i]);
    for (i = 0; i < 35; i++)
      printf("  params.M_10[%d] = %.6g;\n", i, params.M_10[i]);
    for (i = 0; i < 35; i++)
      printf("  params.M_11[%d] = %.6g;\n", i, params.M_11[i]);
    for (i = 0; i < 35; i++)
      printf("  params.M_12[%d] = %.6g;\n", i, params.M_12[i]);
    for (i = 0; i < 35; i++)
      printf("  params.M_13[%d] = %.6g;\n", i, params.M_13[i]);
    for (i = 0; i < 35; i++)
      printf("  params.M_14[%d] = %.6g;\n", i, params.M_14[i]);
    for (i = 0; i < 35; i++)
      printf("  params.M_15[%d] = %.6g;\n", i, params.M_15[i]);
    for (i = 0; i < 35; i++)
      printf("  params.M_16[%d] = %.6g;\n", i, params.M_16[i]);
    for (i = 0; i < 35; i++)
      printf("  params.M_17[%d] = %.6g;\n", i, params.M_17[i]);
    for (i = 0; i < 35; i++)
      printf("  params.M_18[%d] = %.6g;\n", i, params.M_18[i]);
    for (i = 0; i < 35; i++)
      printf("  params.M_19[%d] = %.6g;\n", i, params.M_19[i]);
    for (i = 0; i < 35; i++)
      printf("  params.M_20[%d] = %.6g;\n", i, params.M_20[i]);
    for (i = 0; i < 35; i++)
      printf("  params.M_21[%d] = %.6g;\n", i, params.M_21[i]);
    for (i = 0; i < 35; i++)
      printf("  params.M_22[%d] = %.6g;\n", i, params.M_22[i]);
    for (i = 0; i < 35; i++)
      printf("  params.M_23[%d] = %.6g;\n", i, params.M_23[i]);
    for (i = 0; i < 35; i++)
      printf("  params.M_24[%d] = %.6g;\n", i, params.M_24[i]);
    for (i = 0; i < 35; i++)
      printf("  params.M_25[%d] = %.6g;\n", i, params.M_25[i]);
    for (i = 0; i < 35; i++)
      printf("  params.M_26[%d] = %.6g;\n", i, params.M_26[i]);
    for (i = 0; i < 35; i++)
      printf("  params.M_27[%d] = %.6g;\n", i, params.M_27[i]);
    for (i = 0; i < 35; i++)
      printf("  params.M_28[%d] = %.6g;\n", i, params.M_28[i]);
    for (i = 0; i < 35; i++)
      printf("  params.M_29[%d] = %.6g;\n", i, params.M_29[i]);
    for (i = 0; i < 35; i++)
      printf("  params.M_30[%d] = %.6g;\n", i, params.M_30[i]);
    for (i = 0; i < 35; i++)
      printf("  params.M_31[%d] = %.6g;\n", i, params.M_31[i]);
    for (i = 0; i < 35; i++)
      printf("  params.M_32[%d] = %.6g;\n", i, params.M_32[i]);
    for (i = 0; i < 35; i++)
      printf("  params.M_33[%d] = %.6g;\n", i, params.M_33[i]);
    for (i = 0; i < 35; i++)
      printf("  params.M_34[%d] = %.6g;\n", i, params.M_34[i]);
    for (i = 0; i < 35; i++)
      printf("  params.M_35[%d] = %.6g;\n", i, params.M_35[i]);
    for (i = 0; i < 6; i++)
      printf("  params.XJbL_1[%d] = %.6g;\n", i, params.XJbL_1[i]);
    for (i = 0; i < 6; i++)
      printf("  params.XJbR_1[%d] = %.6g;\n", i, params.XJbR_1[i]);
    for (i = 0; i < 6; i++)
      printf("  params.b2[%d] = %.6g;\n", i, params.b2[i]);
    for (i = 0; i < 6; i++)
      printf("  params.XJbL_2[%d] = %.6g;\n", i, params.XJbL_2[i]);
    for (i = 0; i < 6; i++)
      printf("  params.XJbR_2[%d] = %.6g;\n", i, params.XJbR_2[i]);
    for (i = 0; i < 6; i++)
      printf("  params.XJbL_3[%d] = %.6g;\n", i, params.XJbL_3[i]);
    for (i = 0; i < 6; i++)
      printf("  params.XJbR_3[%d] = %.6g;\n", i, params.XJbR_3[i]);
    for (i = 0; i < 6; i++)
      printf("  params.XJbL_4[%d] = %.6g;\n", i, params.XJbL_4[i]);
    for (i = 0; i < 6; i++)
      printf("  params.XJbR_4[%d] = %.6g;\n", i, params.XJbR_4[i]);
    for (i = 0; i < 6; i++)
      printf("  params.XJbL_5[%d] = %.6g;\n", i, params.XJbL_5[i]);
    for (i = 0; i < 6; i++)
      printf("  params.XJbR_5[%d] = %.6g;\n", i, params.XJbR_5[i]);
    for (i = 0; i < 6; i++)
      printf("  params.XJbL_6[%d] = %.6g;\n", i, params.XJbL_6[i]);
    for (i = 0; i < 6; i++)
      printf("  params.XJbR_6[%d] = %.6g;\n", i, params.XJbR_6[i]);
    for (i = 0; i < 35; i++)
      printf("  params.Jt_1[%d] = %.6g;\n", i, params.Jt_1[i]);
    for (i = 0; i < 70; i++)
      printf("  params.b3[%d] = %.6g;\n", i, params.b3[i]);
    for (i = 0; i < 35; i++)
      printf("  params.Jt_2[%d] = %.6g;\n", i, params.Jt_2[i]);
    for (i = 0; i < 35; i++)
      printf("  params.Jt_3[%d] = %.6g;\n", i, params.Jt_3[i]);
    for (i = 0; i < 35; i++)
      printf("  params.Jt_4[%d] = %.6g;\n", i, params.Jt_4[i]);
    for (i = 0; i < 35; i++)
      printf("  params.Jt_5[%d] = %.6g;\n", i, params.Jt_5[i]);
    for (i = 0; i < 35; i++)
      printf("  params.Jt_6[%d] = %.6g;\n", i, params.Jt_6[i]);
    for (i = 0; i < 35; i++)
      printf("  params.Jt_7[%d] = %.6g;\n", i, params.Jt_7[i]);
    for (i = 0; i < 35; i++)
      printf("  params.Jt_8[%d] = %.6g;\n", i, params.Jt_8[i]);
    for (i = 0; i < 35; i++)
      printf("  params.Jt_9[%d] = %.6g;\n", i, params.Jt_9[i]);
    for (i = 0; i < 35; i++)
      printf("  params.Jt_10[%d] = %.6g;\n", i, params.Jt_10[i]);
    for (i = 0; i < 35; i++)
      printf("  params.Jt_11[%d] = %.6g;\n", i, params.Jt_11[i]);
    for (i = 0; i < 35; i++)
      printf("  params.Jt_12[%d] = %.6g;\n", i, params.Jt_12[i]);
    for (i = 0; i < 35; i++)
      printf("  params.Jt_13[%d] = %.6g;\n", i, params.Jt_13[i]);
    for (i = 0; i < 35; i++)
      printf("  params.Jt_14[%d] = %.6g;\n", i, params.Jt_14[i]);
    for (i = 0; i < 35; i++)
      printf("  params.Jt_15[%d] = %.6g;\n", i, params.Jt_15[i]);
    for (i = 0; i < 35; i++)
      printf("  params.Jt_16[%d] = %.6g;\n", i, params.Jt_16[i]);
    for (i = 0; i < 35; i++)
      printf("  params.Jt_17[%d] = %.6g;\n", i, params.Jt_17[i]);
    for (i = 0; i < 35; i++)
      printf("  params.Jt_18[%d] = %.6g;\n", i, params.Jt_18[i]);
    for (i = 0; i < 6; i++)
      printf("  params.copMxl_1[%d] = %.6g;\n", i, params.copMxl_1[i]);
    for (i = 0; i < 6; i++)
      printf("  params.copMxl_2[%d] = %.6g;\n", i, params.copMxl_2[i]);
    for (i = 0; i < 6; i++)
      printf("  params.copMxr_1[%d] = %.6g;\n", i, params.copMxr_1[i]);
    for (i = 0; i < 6; i++)
      printf("  params.copMxr_2[%d] = %.6g;\n", i, params.copMxr_2[i]);
    for (i = 0; i < 1; i++)
      printf("  params.aw[%d] = %.6g;\n", i, params.aw[i]);
    for (i = 0; i < 1; i++)
      printf("  params.dwd[%d] = %.6g;\n", i, params.dwd[i]);
    for (i = 0; i < 29; i++)
      printf("  params.T_max[%d] = %.6g;\n", i, params.T_max[i]);
    for (i = 0; i < 29; i++)
      printf("  params.v_max[%d] = %.6g;\n", i, params.v_max[i]);
    for (i = 0; i < 1; i++)
      printf("  params.dt[%d] = %.6g;\n", i, params.dt[i]);
    for (i = 0; i < 29; i++)
      printf("  params.q_min[%d] = %.6g;\n", i, params.q_min[i]);
    for (i = 0; i < 29; i++)
      printf("  params.q_max[%d] = %.6g;\n", i, params.q_max[i]);
    for (i = 0; i < 6; i++)
      printf("  params.CL_1[%d] = %.6g;\n", i, params.CL_1[i]);
    for (i = 0; i < 6; i++)
      printf("  params.CL_2[%d] = %.6g;\n", i, params.CL_2[i]);
    for (i = 0; i < 6; i++)
      printf("  params.CL_3[%d] = %.6g;\n", i, params.CL_3[i]);
    for (i = 0; i < 6; i++)
      printf("  params.CL_4[%d] = %.6g;\n", i, params.CL_4[i]);
    for (i = 0; i < 6; i++)
      printf("  params.CL_5[%d] = %.6g;\n", i, params.CL_5[i]);
    for (i = 0; i < 6; i++)
      printf("  params.CL_6[%d] = %.6g;\n", i, params.CL_6[i]);
    for (i = 0; i < 6; i++)
      printf("  params.CL_7[%d] = %.6g;\n", i, params.CL_7[i]);
    for (i = 0; i < 6; i++)
      printf("  params.CL_8[%d] = %.6g;\n", i, params.CL_8[i]);
    for (i = 0; i < 6; i++)
      printf("  params.CL_9[%d] = %.6g;\n", i, params.CL_9[i]);
    for (i = 0; i < 6; i++)
      printf("  params.CL_10[%d] = %.6g;\n", i, params.CL_10[i]);
    for (i = 0; i < 6; i++)
      printf("  params.CL_11[%d] = %.6g;\n", i, params.CL_11[i]);
    for (i = 0; i < 6; i++)
      printf("  params.CR_1[%d] = %.6g;\n", i, params.CR_1[i]);
    for (i = 0; i < 6; i++)
      printf("  params.CR_2[%d] = %.6g;\n", i, params.CR_2[i]);
    for (i = 0; i < 6; i++)
      printf("  params.CR_3[%d] = %.6g;\n", i, params.CR_3[i]);
    for (i = 0; i < 6; i++)
      printf("  params.CR_4[%d] = %.6g;\n", i, params.CR_4[i]);
    for (i = 0; i < 6; i++)
      printf("  params.CR_5[%d] = %.6g;\n", i, params.CR_5[i]);
    for (i = 0; i < 6; i++)
      printf("  params.CR_6[%d] = %.6g;\n", i, params.CR_6[i]);
    for (i = 0; i < 6; i++)
      printf("  params.CR_7[%d] = %.6g;\n", i, params.CR_7[i]);
    for (i = 0; i < 6; i++)
      printf("  params.CR_8[%d] = %.6g;\n", i, params.CR_8[i]);
    for (i = 0; i < 6; i++)
      printf("  params.CR_9[%d] = %.6g;\n", i, params.CR_9[i]);
    for (i = 0; i < 6; i++)
      printf("  params.CR_10[%d] = %.6g;\n", i, params.CR_10[i]);
    for (i = 0; i < 6; i++)
      printf("  params.CR_11[%d] = %.6g;\n", i, params.CR_11[i]);
  }
  /* Perform the actual solve in here. */
  steps = solve();
  /* For profiling purposes, allow extra silent solves if desired. */
  settings.verbose = 0;
  for (i = 0; i < extra_solves; i++)
    solve();
  /* Update the status variables. */
  plhs[1] = mxCreateStructArray(1, dims1x1of1, 4, status_names);
  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "optval", xm);
  *mxGetPr(xm) = work.optval;
  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "gap", xm);
  *mxGetPr(xm) = work.gap;
  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "steps", xm);
  *mxGetPr(xm) = steps;
  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "converged", xm);
  *mxGetPr(xm) = work.converged;
  /* Extract variable values. */
  plhs[0] = mxCreateStructArray(1, dims1x1of1, num_var_names, var_names);
  xm = mxCreateDoubleMatrix(70, 1, mxREAL);
  mxSetField(plhs[0], 0, "w", xm);
  dest = mxGetPr(xm);
  src = vars.w;
  for (i = 0; i < 70; i++) {
    *dest++ = *src++;
  }
  xm = mxCreateDoubleMatrix(35, 1, mxREAL);
  mxSetField(plhs[0], 0, "x", xm);
  dest = mxGetPr(xm);
  src = vars.x;
  for (i = 0; i < 35; i++) {
    *dest++ = *src++;
  }
  xm = mxCreateDoubleMatrix(29, 1, mxREAL);
  mxSetField(plhs[0], 0, "y", xm);
  dest = mxGetPr(xm);
  src = vars.y;
  for (i = 0; i < 29; i++) {
    *dest++ = *src++;
  }
  xm = mxCreateDoubleMatrix(12, 1, mxREAL);
  mxSetField(plhs[0], 0, "z", xm);
  dest = mxGetPr(xm);
  src = vars.z;
  for (i = 0; i < 12; i++) {
    *dest++ = *src++;
  }
}
