/* Produced by CVXGEN, 2020-12-10 23:07:19 -0500.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: cop_testsolver.c. */
/* Description: Basic test harness for solver.c. */
#include "cop_solver.h"
cop_Vars cop_vars;
cop_Params cop_params;
cop_Workspace cop_work;
cop_Settings cop_settings;
#define NUMTESTS 0
int cop_main(int argc, char **argv) {
  int num_iters;
#if (NUMTESTS > 0)
  int i;
  double time;
  double time_per;
#endif
  cop_set_defaults();
  cop_setup_indexing();
  cop_load_default_data();
  /* Solve problem instance for the record. */
  cop_settings.verbose = 1;
  num_iters = cop_solve();
#ifndef ZERO_LIBRARY_MODE
#if (NUMTESTS > 0)
  /* Now solve multiple problem instances for timing purposes. */
  cop_settings.verbose = 0;
  cop_tic();
  for (i = 0; i < NUMTESTS; i++) {
    cop_solve();
  }
  time = cop_tocq();
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
void cop_load_default_data(void) {
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  cop_params.Q[0] = 1.5507979025745755;
  cop_params.Q[2] = 0;
  cop_params.Q[1] = 0;
  cop_params.Q[3] = 1.7081478226181048;
  cop_params.P[0] = -0.8363810443482227;
  cop_params.P[1] = 0.04331042079065206;
  cop_params.NeJc[0] = 1.5717878173906188;
  cop_params.NeJc[1] = 1.5851723557337523;
  cop_params.NeJc[2] = -1.497658758144655;
  cop_params.NeJc[3] = -1.171028487447253;
  cop_params.NeJc[4] = -1.7941311867966805;
  cop_params.NeJc[5] = -0.23676062539745413;
  cop_params.NeJc[6] = -1.8804951564857322;
  cop_params.NeJc[7] = -0.17266710242115568;
  cop_params.NeJc[8] = 0.596576190459043;
  cop_params.NeJc[9] = -0.8860508694080989;
  cop_params.NeJc[10] = 0.7050196079205251;
  cop_params.NeJc[11] = 0.3634512696654033;
  cop_params.deXc[0] = -1.9040724704913385;
  cop_params.deXc[1] = 0.23541635196352795;
  cop_params.deXc[2] = -0.9629902123701384;
  cop_params.deXc[3] = -0.3395952119597214;
  cop_params.deXc[4] = -0.865899672914725;
  cop_params.deXc[5] = 0.7725516732519853;
}
