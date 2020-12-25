/* Produced by CVXGEN, 2020-12-10 23:07:19 -0500.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: com_testsolver.c. */
/* Description: Basic test harness for solver.c. */
#include "com_solver.h"
com_Vars com_vars;
com_Params com_params;
com_Workspace com_work;
com_Settings com_settings;
#define NUMTESTS 0
int com_main(int argc, char **argv) {
  int num_iters;
#if (NUMTESTS > 0)
  int i;
  double time;
  double time_per;
#endif
  com_set_defaults();
  com_setup_indexing();
  com_load_default_data();
  /* Solve problem instance for the record. */
  com_settings.verbose = 1;
  num_iters = com_solve();
#ifndef ZERO_LIBRARY_MODE
#if (NUMTESTS > 0)
  /* Now solve multiple problem instances for timing purposes. */
  com_settings.verbose = 0;
  com_tic();
  for (i = 0; i < NUMTESTS; i++) {
    com_solve();
  }
  time = com_tocq();
  printf("Timed %d solves over %.3f seconds.\n", NUMTESTS, time);
  time_per = time / NUMTESTS;
  if (time_per > 1) {
    printf("Actual time taken per solve: %.3g s.\n", time_per);
  } else if (time_per > 1e-3) {
    printf("Actual time taken per solve: %.3g ms.\n", 1e3*time_per);
  } else {
    printf("Actual time taken per solve: %.3g us.\n", 1e6*time_per);
  }
#endif
#endif
  return 0;
}
void com_load_default_data(void) {
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  com_params.Q[0] = 1.5507979025745755;
  com_params.Q[2] = 0;
  com_params.Q[1] = 0;
  com_params.Q[3] = 1.7081478226181048;
  com_params.P[0] = -0.8363810443482227;
  com_params.P[1] = 0.04331042079065206;
  com_params.NeJc[0] = 1.5717878173906188;
  com_params.NeJc[1] = 1.5851723557337523;
  com_params.NeJc[2] = -1.497658758144655;
  com_params.NeJc[3] = -1.171028487447253;
  com_params.NeJc[4] = -1.7941311867966805;
  com_params.NeJc[5] = -0.23676062539745413;
  com_params.NeJc[6] = -1.8804951564857322;
  com_params.NeJc[7] = -0.17266710242115568;
  com_params.NeJc[8] = 0.596576190459043;
  com_params.NeJc[9] = -0.8860508694080989;
  com_params.NeJc[10] = 0.7050196079205251;
  com_params.NeJc[11] = 0.3634512696654033;
  com_params.deXc[0] = -1.9040724704913385;
  com_params.deXc[1] = 0.23541635196352795;
  com_params.deXc[2] = -0.9629902123701384;
  com_params.deXc[3] = -0.3395952119597214;
  com_params.deXc[4] = -0.865899672914725;
  com_params.deXc[5] = 0.7725516732519853;
}
