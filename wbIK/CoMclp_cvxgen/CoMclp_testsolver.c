/* Produced by CVXGEN, 2020-10-04 06:20:49 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: CoMclp_testsolver.c. */
/* Description: Basic test harness for solver.c. */
#include "CoMclp_solver.h"
CoMclp_Vars CoMclp_vars;
CoMclp_Params CoMclp_params;
CoMclp_Workspace CoMclp_work;
CoMclp_Settings CoMclp_settings;
#define NUMTESTS 0
int CoMclp_main(int argc, char **argv) {
  int num_iters;
#if (NUMTESTS > 0)
  int i;
  double time;
  double time_per;
#endif
  CoMclp_set_defaults();
  CoMclp_setup_indexing();
  CoMclp_load_default_data();
  /* Solve problem instance for the record. */
  CoMclp_settings.verbose = 1;
  num_iters = CoMclp_solve();
#ifndef ZERO_LIBRARY_MODE
#if (NUMTESTS > 0)
  /* Now solve multiple problem instances for timing purposes. */
  CoMclp_settings.verbose = 0;
  CoMclp_tic();
  for (i = 0; i < NUMTESTS; i++) {
    CoMclp_solve();
  }
  time = CoMclp_tocq();
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
void CoMclp_load_default_data(void) {
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  CoMclp_params.Q[0] = 1.5507979025745755;
  CoMclp_params.Q[2] = 0;
  CoMclp_params.Q[1] = 0;
  CoMclp_params.Q[3] = 1.7081478226181048;
  CoMclp_params.P[0] = -0.8363810443482227;
  CoMclp_params.P[1] = 0.04331042079065206;
  CoMclp_params.NeJc[0] = 1.5717878173906188;
  CoMclp_params.NeJc[1] = 1.5851723557337523;
  CoMclp_params.NeJc[2] = -1.497658758144655;
  CoMclp_params.NeJc[3] = -1.171028487447253;
  CoMclp_params.NeJc[4] = -1.7941311867966805;
  CoMclp_params.NeJc[5] = -0.23676062539745413;
  CoMclp_params.NeJc[6] = -1.8804951564857322;
  CoMclp_params.NeJc[7] = -0.17266710242115568;
  CoMclp_params.NeJc[8] = 0.596576190459043;
  CoMclp_params.NeJc[9] = -0.8860508694080989;
  CoMclp_params.NeJc[10] = 0.7050196079205251;
  CoMclp_params.NeJc[11] = 0.3634512696654033;
  CoMclp_params.deXc[0] = -1.9040724704913385;
  CoMclp_params.deXc[1] = 0.23541635196352795;
  CoMclp_params.deXc[2] = -0.9629902123701384;
  CoMclp_params.deXc[3] = -0.3395952119597214;
  CoMclp_params.deXc[4] = -0.865899672914725;
  CoMclp_params.deXc[5] = 0.7725516732519853;
}
