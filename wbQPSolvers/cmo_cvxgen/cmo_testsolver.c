/* Produced by CVXGEN, 2019-04-21 11:08:24 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: cmo_testsolver.c. */
/* Description: Basic test harness for solver.c. */
#include "cmo_solver.h"
cmo_Vars cmo_vars;
cmo_Params cmo_params;
cmo_Workspace cmo_work;
cmo_Settings cmo_settings;
#define NUMTESTS 0
int cmo_main(int argc, char **argv) {
  int num_iters;
#if (NUMTESTS > 0)
  int i;
  double time;
  double time_per;
#endif
  cmo_set_defaults();
  cmo_setup_indexing();
  cmo_load_default_data();
  /* Solve problem instance for the record. */
  cmo_settings.verbose = 1;
  num_iters = cmo_solve();
#ifndef ZERO_LIBRARY_MODE
#if (NUMTESTS > 0)
  /* Now solve multiple problem instances for timing purposes. */
  cmo_settings.verbose = 0;
  cmo_tic();
  for (i = 0; i < NUMTESTS; i++) {
    cmo_solve();
  }
  time = cmo_tocq();
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
void cmo_load_default_data(void) {
  cmo_params.Qam[0] = 1.101595805149151;
  cmo_params.Qam[1] = 1.4162956452362097;
  cmo_params.Qam[2] = 0.5818094778258887;
  cmo_params.Qam[3] = 1.021655210395326;
  cmo_params.Qam[4] = 1.7858939086953094;
  cmo_params.Qam[5] = 1.7925861778668761;
  cmo_params.Jgm[0] = -1.497658758144655;
  cmo_params.Jgm[1] = -1.171028487447253;
  cmo_params.Jgm[2] = -1.7941311867966805;
  cmo_params.Jgm[3] = -0.23676062539745413;
  cmo_params.Jgm[4] = -1.8804951564857322;
  cmo_params.Jgm[5] = -0.17266710242115568;
  cmo_params.Jgm[6] = 0.596576190459043;
  cmo_params.Jgm[7] = -0.8860508694080989;
  cmo_params.Jgm[8] = 0.7050196079205251;
  cmo_params.Jgm[9] = 0.3634512696654033;
  cmo_params.Jgm[10] = -1.9040724704913385;
  cmo_params.Jgm[11] = 0.23541635196352795;
  cmo_params.Jgm[12] = -0.9629902123701384;
  cmo_params.Jgm[13] = -0.3395952119597214;
  cmo_params.Jgm[14] = -0.865899672914725;
  cmo_params.Jgm[15] = 0.7725516732519853;
  cmo_params.Jgm[16] = -0.23818512931704205;
  cmo_params.Jgm[17] = -1.372529046100147;
  cmo_params.Jgm[18] = 0.17859607212737894;
  cmo_params.Jgm[19] = 1.1212590580454682;
  cmo_params.Jgm[20] = -0.774545870495281;
  cmo_params.Jgm[21] = -1.1121684642712744;
  cmo_params.Jgm[22] = -0.44811496977740495;
  cmo_params.Jgm[23] = 1.7455345994417217;
  cmo_params.Jgm[24] = 1.9039816898917352;
  cmo_params.Jgm[25] = 0.6895347036512547;
  cmo_params.Jgm[26] = 1.6113364341535923;
  cmo_params.Jgm[27] = 1.383003485172717;
  cmo_params.Jgm[28] = -0.48802383468444344;
  cmo_params.Jgm[29] = -1.631131964513103;
  cmo_params.Jgm[30] = 0.6136436100941447;
  cmo_params.Jgm[31] = 0.2313630495538037;
  cmo_params.Jgm[32] = -0.5537409477496875;
  cmo_params.Jgm[33] = -1.0997819806406723;
  cmo_params.Jgm[34] = -0.3739203344950055;
  cmo_params.Jgm[35] = -0.12423900520332376;
  cmo_params.Jgm[36] = -0.923057686995755;
  cmo_params.Jgm[37] = -0.8328289030982696;
  cmo_params.Jgm[38] = -0.16925440270808823;
  cmo_params.Jgm[39] = 1.442135651787706;
  cmo_params.Jgm[40] = 0.34501161787128565;
  cmo_params.Jgm[41] = -0.8660485502711608;
  cmo_params.Jgm[42] = -0.8880899735055947;
  cmo_params.Jgm[43] = -0.1815116979122129;
  cmo_params.Jgm[44] = -1.17835862158005;
  cmo_params.Jgm[45] = -1.1944851558277074;
  cmo_params.Jgm[46] = 0.05614023926976763;
  cmo_params.Jgm[47] = -1.6510825248767813;
  cmo_params.Jgm[48] = -0.06565787059365391;
  cmo_params.Jgm[49] = -0.5512951504486665;
  cmo_params.Jgm[50] = 0.8307464872626844;
  cmo_params.Jgm[51] = 0.9869848924080182;
  cmo_params.Jgm[52] = 0.7643716874230573;
  cmo_params.Jgm[53] = 0.7567216550196565;
  cmo_params.Jgm[54] = -0.5055995034042868;
  cmo_params.Jgm[55] = 0.6725392189410702;
  cmo_params.Jgm[56] = -0.6406053441727284;
  cmo_params.Jgm[57] = 0.29117547947550015;
  cmo_params.Jgm[58] = -0.6967713677405021;
  cmo_params.Jgm[59] = -0.21941980294587182;
  cmo_params.Jgm[60] = -1.753884276680243;
  cmo_params.Jgm[61] = -1.0292983112626475;
  cmo_params.Jgm[62] = 1.8864104246942706;
  cmo_params.Jgm[63] = -1.077663182579704;
  cmo_params.Jgm[64] = 0.7659100437893209;
  cmo_params.Jgm[65] = 0.6019074328549583;
  cmo_params.Jgm[66] = 0.8957565577499285;
  cmo_params.Jgm[67] = -0.09964555746227477;
  cmo_params.Jgm[68] = 0.38665509840745127;
  cmo_params.Jgm[69] = -1.7321223042686946;
  cmo_params.Jgm[70] = -1.7097514487110663;
  cmo_params.Jgm[71] = -1.2040958948116867;
  cmo_params.bam[0] = -1.3925560119658358;
  cmo_params.bam[1] = -1.5995826216742213;
  cmo_params.bam[2] = -1.4828245415645833;
  cmo_params.bam[3] = 0.21311092723061398;
  cmo_params.bam[4] = -1.248740700304487;
  cmo_params.bam[5] = 1.808404972124833;
  cmo_params.Qrm[0] = 1.3632235576148533;
  cmo_params.Qrm[1] = 1.0820393467195424;
  cmo_params.Qrm[2] = 1.4143612016157954;
  cmo_params.Qrm[3] = 0.5277733419050268;
  cmo_params.Qrm[4] = 1.8534513685074556;
  cmo_params.Qrm[5] = 1.6783861155999413;
  cmo_params.Qrm[6] = 1.4526389968560744;
  cmo_params.Qrm[7] = 0.9604799121708201;
  cmo_params.Qrm[8] = 1.6842063717532936;
  cmo_params.Qrm[9] = 1.4895046468487185;
  cmo_params.Qrm[10] = 1.320651812799225;
  cmo_params.Qrm[11] = 1.8279505340118756;
  cmo_params.Jgc[0] = 0.5346622551502991;
  cmo_params.Jgc[1] = -0.5362376605895625;
  cmo_params.Jgc[2] = 0.2113782926017822;
  cmo_params.Jgc[3] = -1.2144776931994525;
  cmo_params.Jgc[4] = -1.2317108144255875;
  cmo_params.Jgc[5] = 0.9026784957312834;
  cmo_params.Jgc[6] = 1.1397468137245244;
  cmo_params.Jgc[7] = 1.8883934547350631;
  cmo_params.Jgc[8] = 1.4038856681660068;
  cmo_params.Jgc[9] = 0.17437730638329096;
  cmo_params.Jgc[10] = -1.6408365219077408;
  cmo_params.Jgc[11] = -0.04450702153554875;
  cmo_params.Jgc[12] = 1.7117453902485025;
  cmo_params.Jgc[13] = 1.1504727980139053;
  cmo_params.Jgc[14] = -0.05962309578364744;
  cmo_params.Jgc[15] = -0.1788825540764547;
  cmo_params.Jgc[16] = -1.1280569263625857;
  cmo_params.Jgc[17] = -1.2911464767927057;
  cmo_params.Jgc[18] = -1.7055053231225696;
  cmo_params.Jgc[19] = 1.56957275034837;
  cmo_params.Jgc[20] = 0.5607064675962357;
  cmo_params.Jgc[21] = -1.4266707301147146;
  cmo_params.Jgc[22] = -0.3434923211351708;
  cmo_params.Jgc[23] = -1.8035643024085055;
  cmo_params.Jgc[24] = -1.1625066019105454;
  cmo_params.Jgc[25] = 0.9228324965161532;
  cmo_params.Jgc[26] = 0.6044910817663975;
  cmo_params.Jgc[27] = -0.0840868104920891;
  cmo_params.Jgc[28] = -0.900877978017443;
  cmo_params.Jgc[29] = 0.608892500264739;
  cmo_params.Jgc[30] = 1.8257980452695217;
  cmo_params.Jgc[31] = -0.25791777529922877;
  cmo_params.Jgc[32] = -1.7194699796493191;
  cmo_params.Jgc[33] = -1.7690740487081298;
  cmo_params.Jgc[34] = -1.6685159248097703;
  cmo_params.Jgc[35] = 1.8388287490128845;
  cmo_params.Jgc[36] = 0.16304334474597537;
  cmo_params.Jgc[37] = 1.3498497306788897;
  cmo_params.Jgc[38] = -1.3198658230514613;
  cmo_params.Jgc[39] = -0.9586197090843394;
  cmo_params.Jgc[40] = 0.7679100474913709;
  cmo_params.Jgc[41] = 1.5822813125679343;
  cmo_params.Jgc[42] = -0.6372460621593619;
  cmo_params.Jgc[43] = -1.741307208038867;
  cmo_params.Jgc[44] = 1.456478677642575;
  cmo_params.Jgc[45] = -0.8365102166820959;
  cmo_params.Jgc[46] = 0.9643296255982503;
  cmo_params.Jgc[47] = -1.367865381194024;
  cmo_params.Jgc[48] = 0.7798537405635035;
  cmo_params.Jgc[49] = 1.3656784761245926;
  cmo_params.Jgc[50] = 0.9086083149868371;
  cmo_params.Jgc[51] = -0.5635699005460344;
  cmo_params.Jgc[52] = 0.9067590059607915;
  cmo_params.Jgc[53] = -1.4421315032701587;
  cmo_params.Jgc[54] = -0.7447235390671119;
  cmo_params.Jgc[55] = -0.32166897326822186;
  cmo_params.Jgc[56] = 1.5088481557772684;
  cmo_params.Jgc[57] = -1.385039165715428;
  cmo_params.Jgc[58] = 1.5204991609972622;
  cmo_params.Jgc[59] = 1.1958572768832156;
  cmo_params.Jgc[60] = 1.8864971883119228;
  cmo_params.Jgc[61] = -0.5291880667861584;
  cmo_params.Jgc[62] = -1.1802409243688836;
  cmo_params.Jgc[63] = -1.037718718661604;
  cmo_params.Jgc[64] = 1.3114512056856835;
  cmo_params.Jgc[65] = 1.8609125943756615;
  cmo_params.Jgc[66] = 0.7952399935216938;
  cmo_params.Jgc[67] = -0.07001183290468038;
  cmo_params.Jgc[68] = -0.8518009412754686;
  cmo_params.Jgc[69] = 1.3347515373726386;
  cmo_params.Jgc[70] = 1.4887180335977037;
  cmo_params.Jgc[71] = -1.6314736327976336;
  cmo_params.Jgc[72] = -1.1362021159208933;
  cmo_params.Jgc[73] = 1.327044361831466;
  cmo_params.Jgc[74] = 1.3932155883179842;
  cmo_params.Jgc[75] = -0.7413880049440107;
  cmo_params.Jgc[76] = -0.8828216126125747;
  cmo_params.Jgc[77] = -0.27673991192616;
  cmo_params.Jgc[78] = 0.15778600105866714;
  cmo_params.Jgc[79] = -1.6177327399735457;
  cmo_params.Jgc[80] = 1.3476485548544606;
  cmo_params.Jgc[81] = 0.13893948140528378;
  cmo_params.Jgc[82] = 1.0998712601636944;
  cmo_params.Jgc[83] = -1.0766549376946926;
  cmo_params.Jgc[84] = 1.8611734044254629;
  cmo_params.Jgc[85] = 1.0041092292735172;
  cmo_params.Jgc[86] = -0.6276245424321543;
  cmo_params.Jgc[87] = 1.794110587839819;
  cmo_params.Jgc[88] = 0.8020471158650913;
  cmo_params.Jgc[89] = 1.362244341944948;
  cmo_params.Jgc[90] = -1.8180107765765245;
  cmo_params.Jgc[91] = -1.7774338357932473;
  cmo_params.Jgc[92] = 0.9709490941985153;
  cmo_params.Jgc[93] = -0.7812542682064318;
  cmo_params.Jgc[94] = 0.0671374633729811;
  cmo_params.Jgc[95] = -1.374950305314906;
  cmo_params.Jgc[96] = 1.9118096386279388;
  cmo_params.Jgc[97] = 0.011004190697677885;
  cmo_params.Jgc[98] = 1.3160043138989015;
  cmo_params.Jgc[99] = -1.7038488148800144;
  cmo_params.Jgc[100] = -0.08433819112864738;
  cmo_params.Jgc[101] = -1.7508820783768964;
  cmo_params.Jgc[102] = 1.536965724350949;
  cmo_params.Jgc[103] = -0.21675928514816478;
  cmo_params.Jgc[104] = -1.725800326952653;
  cmo_params.Jgc[105] = -1.6940148707361717;
  cmo_params.Jgc[106] = 0.15517063201268;
  cmo_params.Jgc[107] = -1.697734381979077;
  cmo_params.Jgc[108] = -1.264910727950229;
  cmo_params.Jgc[109] = -0.2545716633339441;
  cmo_params.Jgc[110] = -0.008868675926170244;
  cmo_params.Jgc[111] = 0.3332476609670296;
  cmo_params.Jgc[112] = 0.48205072561962936;
  cmo_params.Jgc[113] = -0.5087540014293261;
  cmo_params.Jgc[114] = 0.4749463319223195;
  cmo_params.Jgc[115] = -1.371021366459455;
  cmo_params.Jgc[116] = -0.8979660982652256;
  cmo_params.Jgc[117] = 1.194873082385242;
  cmo_params.Jgc[118] = -1.3876427970939353;
  cmo_params.Jgc[119] = -1.106708108457053;
  cmo_params.Jgc[120] = -1.0280872812241797;
  cmo_params.Jgc[121] = -0.08197078070773234;
  cmo_params.Jgc[122] = -1.9970179118324083;
  cmo_params.Jgc[123] = -1.878754557910134;
  cmo_params.Jgc[124] = -0.15380739340877803;
  cmo_params.Jgc[125] = -1.349917260533923;
  cmo_params.Jgc[126] = 0.7180072150931407;
  cmo_params.Jgc[127] = 1.1808183487065538;
  cmo_params.Jgc[128] = 0.31265343495084075;
  cmo_params.Jgc[129] = 0.7790599086928229;
  cmo_params.Jgc[130] = -0.4361679370644853;
  cmo_params.Jgc[131] = -1.8148151880282066;
  cmo_params.Jgc[132] = -0.24231386948140266;
  cmo_params.Jgc[133] = -0.5120787511622411;
  cmo_params.Jgc[134] = 0.3880129688013203;
  cmo_params.Jgc[135] = -1.4631273212038676;
  cmo_params.Jgc[136] = -1.0891484131126563;
  cmo_params.Jgc[137] = 1.2591296661091191;
  cmo_params.Jgc[138] = -0.9426978934391474;
  cmo_params.Jgc[139] = -0.358719180371347;
  cmo_params.Jgc[140] = 1.7438887059831263;
  cmo_params.Jgc[141] = -0.8977901479165817;
  cmo_params.Jgc[142] = -1.4188401645857445;
  cmo_params.Jgc[143] = 0.8080805173258092;
  cmo_params.brm[0] = 0.2682662017650985;
  cmo_params.brm[1] = 0.44637534218638786;
  cmo_params.brm[2] = -1.8318765960257055;
  cmo_params.brm[3] = -0.3309324209710929;
  cmo_params.brm[4] = -1.9829342633313622;
  cmo_params.brm[5] = -1.013858124556442;
  cmo_params.brm[6] = 0.8242247343360254;
  cmo_params.brm[7] = -1.753837136317201;
  cmo_params.brm[8] = -0.8212260055868805;
  cmo_params.brm[9] = 1.9524510112487126;
  cmo_params.brm[10] = 1.884888920907902;
  cmo_params.brm[11] = -0.0726144452811801;
  cmo_params.lambda[0] = 1.4713867730564918;
}
