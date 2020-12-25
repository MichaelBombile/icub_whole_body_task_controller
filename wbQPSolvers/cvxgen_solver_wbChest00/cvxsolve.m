% Produced by CVXGEN, 2020-06-01 20:41:35 -0400.
% CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2017 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: cvxsolve.m.
% Description: Solution file, via cvx, for use with sample.m.
function [vars, status] = cvxsolve(params, settings)
if isfield(params, 'CL_1')
  CL_1 = params.CL_1;
elseif isfield(params, 'CL')
  CL_1 = params.CL{1};
else
  error 'could not find CL_1'
end
if isfield(params, 'CL_2')
  CL_2 = params.CL_2;
elseif isfield(params, 'CL')
  CL_2 = params.CL{2};
else
  error 'could not find CL_2'
end
if isfield(params, 'CL_3')
  CL_3 = params.CL_3;
elseif isfield(params, 'CL')
  CL_3 = params.CL{3};
else
  error 'could not find CL_3'
end
if isfield(params, 'CL_4')
  CL_4 = params.CL_4;
elseif isfield(params, 'CL')
  CL_4 = params.CL{4};
else
  error 'could not find CL_4'
end
if isfield(params, 'CL_5')
  CL_5 = params.CL_5;
elseif isfield(params, 'CL')
  CL_5 = params.CL{5};
else
  error 'could not find CL_5'
end
if isfield(params, 'CL_6')
  CL_6 = params.CL_6;
elseif isfield(params, 'CL')
  CL_6 = params.CL{6};
else
  error 'could not find CL_6'
end
if isfield(params, 'CL_7')
  CL_7 = params.CL_7;
elseif isfield(params, 'CL')
  CL_7 = params.CL{7};
else
  error 'could not find CL_7'
end
if isfield(params, 'CL_8')
  CL_8 = params.CL_8;
elseif isfield(params, 'CL')
  CL_8 = params.CL{8};
else
  error 'could not find CL_8'
end
if isfield(params, 'CL_9')
  CL_9 = params.CL_9;
elseif isfield(params, 'CL')
  CL_9 = params.CL{9};
else
  error 'could not find CL_9'
end
if isfield(params, 'CL_10')
  CL_10 = params.CL_10;
elseif isfield(params, 'CL')
  CL_10 = params.CL{10};
else
  error 'could not find CL_10'
end
if isfield(params, 'CL_11')
  CL_11 = params.CL_11;
elseif isfield(params, 'CL')
  CL_11 = params.CL{11};
else
  error 'could not find CL_11'
end
if isfield(params, 'CR_1')
  CR_1 = params.CR_1;
elseif isfield(params, 'CR')
  CR_1 = params.CR{1};
else
  error 'could not find CR_1'
end
if isfield(params, 'CR_2')
  CR_2 = params.CR_2;
elseif isfield(params, 'CR')
  CR_2 = params.CR{2};
else
  error 'could not find CR_2'
end
if isfield(params, 'CR_3')
  CR_3 = params.CR_3;
elseif isfield(params, 'CR')
  CR_3 = params.CR{3};
else
  error 'could not find CR_3'
end
if isfield(params, 'CR_4')
  CR_4 = params.CR_4;
elseif isfield(params, 'CR')
  CR_4 = params.CR{4};
else
  error 'could not find CR_4'
end
if isfield(params, 'CR_5')
  CR_5 = params.CR_5;
elseif isfield(params, 'CR')
  CR_5 = params.CR{5};
else
  error 'could not find CR_5'
end
if isfield(params, 'CR_6')
  CR_6 = params.CR_6;
elseif isfield(params, 'CR')
  CR_6 = params.CR{6};
else
  error 'could not find CR_6'
end
if isfield(params, 'CR_7')
  CR_7 = params.CR_7;
elseif isfield(params, 'CR')
  CR_7 = params.CR{7};
else
  error 'could not find CR_7'
end
if isfield(params, 'CR_8')
  CR_8 = params.CR_8;
elseif isfield(params, 'CR')
  CR_8 = params.CR{8};
else
  error 'could not find CR_8'
end
if isfield(params, 'CR_9')
  CR_9 = params.CR_9;
elseif isfield(params, 'CR')
  CR_9 = params.CR{9};
else
  error 'could not find CR_9'
end
if isfield(params, 'CR_10')
  CR_10 = params.CR_10;
elseif isfield(params, 'CR')
  CR_10 = params.CR{10};
else
  error 'could not find CR_10'
end
if isfield(params, 'CR_11')
  CR_11 = params.CR_11;
elseif isfield(params, 'CR')
  CR_11 = params.CR{11};
else
  error 'could not find CR_11'
end
if isfield(params, 'Jt_1')
  Jt_1 = params.Jt_1;
elseif isfield(params, 'Jt')
  Jt_1 = params.Jt{1};
else
  error 'could not find Jt_1'
end
if isfield(params, 'Jt_2')
  Jt_2 = params.Jt_2;
elseif isfield(params, 'Jt')
  Jt_2 = params.Jt{2};
else
  error 'could not find Jt_2'
end
if isfield(params, 'Jt_3')
  Jt_3 = params.Jt_3;
elseif isfield(params, 'Jt')
  Jt_3 = params.Jt{3};
else
  error 'could not find Jt_3'
end
if isfield(params, 'Jt_4')
  Jt_4 = params.Jt_4;
elseif isfield(params, 'Jt')
  Jt_4 = params.Jt{4};
else
  error 'could not find Jt_4'
end
if isfield(params, 'Jt_5')
  Jt_5 = params.Jt_5;
elseif isfield(params, 'Jt')
  Jt_5 = params.Jt{5};
else
  error 'could not find Jt_5'
end
if isfield(params, 'Jt_6')
  Jt_6 = params.Jt_6;
elseif isfield(params, 'Jt')
  Jt_6 = params.Jt{6};
else
  error 'could not find Jt_6'
end
if isfield(params, 'Jt_7')
  Jt_7 = params.Jt_7;
elseif isfield(params, 'Jt')
  Jt_7 = params.Jt{7};
else
  error 'could not find Jt_7'
end
if isfield(params, 'Jt_8')
  Jt_8 = params.Jt_8;
elseif isfield(params, 'Jt')
  Jt_8 = params.Jt{8};
else
  error 'could not find Jt_8'
end
if isfield(params, 'Jt_9')
  Jt_9 = params.Jt_9;
elseif isfield(params, 'Jt')
  Jt_9 = params.Jt{9};
else
  error 'could not find Jt_9'
end
if isfield(params, 'Jt_10')
  Jt_10 = params.Jt_10;
elseif isfield(params, 'Jt')
  Jt_10 = params.Jt{10};
else
  error 'could not find Jt_10'
end
if isfield(params, 'Jt_11')
  Jt_11 = params.Jt_11;
elseif isfield(params, 'Jt')
  Jt_11 = params.Jt{11};
else
  error 'could not find Jt_11'
end
if isfield(params, 'Jt_12')
  Jt_12 = params.Jt_12;
elseif isfield(params, 'Jt')
  Jt_12 = params.Jt{12};
else
  error 'could not find Jt_12'
end
if isfield(params, 'Jt_13')
  Jt_13 = params.Jt_13;
elseif isfield(params, 'Jt')
  Jt_13 = params.Jt{13};
else
  error 'could not find Jt_13'
end
if isfield(params, 'Jt_14')
  Jt_14 = params.Jt_14;
elseif isfield(params, 'Jt')
  Jt_14 = params.Jt{14};
else
  error 'could not find Jt_14'
end
if isfield(params, 'Jt_15')
  Jt_15 = params.Jt_15;
elseif isfield(params, 'Jt')
  Jt_15 = params.Jt{15};
else
  error 'could not find Jt_15'
end
if isfield(params, 'Jt_16')
  Jt_16 = params.Jt_16;
elseif isfield(params, 'Jt')
  Jt_16 = params.Jt{16};
else
  error 'could not find Jt_16'
end
if isfield(params, 'Jt_17')
  Jt_17 = params.Jt_17;
elseif isfield(params, 'Jt')
  Jt_17 = params.Jt{17};
else
  error 'could not find Jt_17'
end
if isfield(params, 'Jt_18')
  Jt_18 = params.Jt_18;
elseif isfield(params, 'Jt')
  Jt_18 = params.Jt{18};
else
  error 'could not find Jt_18'
end
if isfield(params, 'Jt_19')
  Jt_19 = params.Jt_19;
elseif isfield(params, 'Jt')
  Jt_19 = params.Jt{19};
else
  error 'could not find Jt_19'
end
if isfield(params, 'Jt_20')
  Jt_20 = params.Jt_20;
elseif isfield(params, 'Jt')
  Jt_20 = params.Jt{20};
else
  error 'could not find Jt_20'
end
if isfield(params, 'Jt_21')
  Jt_21 = params.Jt_21;
elseif isfield(params, 'Jt')
  Jt_21 = params.Jt{21};
else
  error 'could not find Jt_21'
end
if isfield(params, 'Jt_22')
  Jt_22 = params.Jt_22;
elseif isfield(params, 'Jt')
  Jt_22 = params.Jt{22};
else
  error 'could not find Jt_22'
end
if isfield(params, 'Jt_23')
  Jt_23 = params.Jt_23;
elseif isfield(params, 'Jt')
  Jt_23 = params.Jt{23};
else
  error 'could not find Jt_23'
end
if isfield(params, 'Jt_24')
  Jt_24 = params.Jt_24;
elseif isfield(params, 'Jt')
  Jt_24 = params.Jt{24};
else
  error 'could not find Jt_24'
end
if isfield(params, 'Jt_25')
  Jt_25 = params.Jt_25;
elseif isfield(params, 'Jt')
  Jt_25 = params.Jt{25};
else
  error 'could not find Jt_25'
end
if isfield(params, 'Jt_26')
  Jt_26 = params.Jt_26;
elseif isfield(params, 'Jt')
  Jt_26 = params.Jt{26};
else
  error 'could not find Jt_26'
end
if isfield(params, 'Jt_27')
  Jt_27 = params.Jt_27;
elseif isfield(params, 'Jt')
  Jt_27 = params.Jt{27};
else
  error 'could not find Jt_27'
end
if isfield(params, 'Jt_28')
  Jt_28 = params.Jt_28;
elseif isfield(params, 'Jt')
  Jt_28 = params.Jt{28};
else
  error 'could not find Jt_28'
end
if isfield(params, 'Jt_29')
  Jt_29 = params.Jt_29;
elseif isfield(params, 'Jt')
  Jt_29 = params.Jt{29};
else
  error 'could not find Jt_29'
end
if isfield(params, 'Jt_30')
  Jt_30 = params.Jt_30;
elseif isfield(params, 'Jt')
  Jt_30 = params.Jt{30};
else
  error 'could not find Jt_30'
end
if isfield(params, 'Jto_1')
  Jto_1 = params.Jto_1;
elseif isfield(params, 'Jto')
  Jto_1 = params.Jto{1};
else
  error 'could not find Jto_1'
end
if isfield(params, 'Jto_2')
  Jto_2 = params.Jto_2;
elseif isfield(params, 'Jto')
  Jto_2 = params.Jto{2};
else
  error 'could not find Jto_2'
end
if isfield(params, 'Jto_3')
  Jto_3 = params.Jto_3;
elseif isfield(params, 'Jto')
  Jto_3 = params.Jto{3};
else
  error 'could not find Jto_3'
end
if isfield(params, 'M_1')
  M_1 = params.M_1;
elseif isfield(params, 'M')
  M_1 = params.M{1};
else
  error 'could not find M_1'
end
if isfield(params, 'M_2')
  M_2 = params.M_2;
elseif isfield(params, 'M')
  M_2 = params.M{2};
else
  error 'could not find M_2'
end
if isfield(params, 'M_3')
  M_3 = params.M_3;
elseif isfield(params, 'M')
  M_3 = params.M{3};
else
  error 'could not find M_3'
end
if isfield(params, 'M_4')
  M_4 = params.M_4;
elseif isfield(params, 'M')
  M_4 = params.M{4};
else
  error 'could not find M_4'
end
if isfield(params, 'M_5')
  M_5 = params.M_5;
elseif isfield(params, 'M')
  M_5 = params.M{5};
else
  error 'could not find M_5'
end
if isfield(params, 'M_6')
  M_6 = params.M_6;
elseif isfield(params, 'M')
  M_6 = params.M{6};
else
  error 'could not find M_6'
end
if isfield(params, 'M_7')
  M_7 = params.M_7;
elseif isfield(params, 'M')
  M_7 = params.M{7};
else
  error 'could not find M_7'
end
if isfield(params, 'M_8')
  M_8 = params.M_8;
elseif isfield(params, 'M')
  M_8 = params.M{8};
else
  error 'could not find M_8'
end
if isfield(params, 'M_9')
  M_9 = params.M_9;
elseif isfield(params, 'M')
  M_9 = params.M{9};
else
  error 'could not find M_9'
end
if isfield(params, 'M_10')
  M_10 = params.M_10;
elseif isfield(params, 'M')
  M_10 = params.M{10};
else
  error 'could not find M_10'
end
if isfield(params, 'M_11')
  M_11 = params.M_11;
elseif isfield(params, 'M')
  M_11 = params.M{11};
else
  error 'could not find M_11'
end
if isfield(params, 'M_12')
  M_12 = params.M_12;
elseif isfield(params, 'M')
  M_12 = params.M{12};
else
  error 'could not find M_12'
end
if isfield(params, 'M_13')
  M_13 = params.M_13;
elseif isfield(params, 'M')
  M_13 = params.M{13};
else
  error 'could not find M_13'
end
if isfield(params, 'M_14')
  M_14 = params.M_14;
elseif isfield(params, 'M')
  M_14 = params.M{14};
else
  error 'could not find M_14'
end
if isfield(params, 'M_15')
  M_15 = params.M_15;
elseif isfield(params, 'M')
  M_15 = params.M{15};
else
  error 'could not find M_15'
end
if isfield(params, 'M_16')
  M_16 = params.M_16;
elseif isfield(params, 'M')
  M_16 = params.M{16};
else
  error 'could not find M_16'
end
if isfield(params, 'M_17')
  M_17 = params.M_17;
elseif isfield(params, 'M')
  M_17 = params.M{17};
else
  error 'could not find M_17'
end
if isfield(params, 'M_18')
  M_18 = params.M_18;
elseif isfield(params, 'M')
  M_18 = params.M{18};
else
  error 'could not find M_18'
end
if isfield(params, 'M_19')
  M_19 = params.M_19;
elseif isfield(params, 'M')
  M_19 = params.M{19};
else
  error 'could not find M_19'
end
if isfield(params, 'M_20')
  M_20 = params.M_20;
elseif isfield(params, 'M')
  M_20 = params.M{20};
else
  error 'could not find M_20'
end
if isfield(params, 'M_21')
  M_21 = params.M_21;
elseif isfield(params, 'M')
  M_21 = params.M{21};
else
  error 'could not find M_21'
end
if isfield(params, 'M_22')
  M_22 = params.M_22;
elseif isfield(params, 'M')
  M_22 = params.M{22};
else
  error 'could not find M_22'
end
if isfield(params, 'M_23')
  M_23 = params.M_23;
elseif isfield(params, 'M')
  M_23 = params.M{23};
else
  error 'could not find M_23'
end
if isfield(params, 'M_24')
  M_24 = params.M_24;
elseif isfield(params, 'M')
  M_24 = params.M{24};
else
  error 'could not find M_24'
end
if isfield(params, 'M_25')
  M_25 = params.M_25;
elseif isfield(params, 'M')
  M_25 = params.M{25};
else
  error 'could not find M_25'
end
if isfield(params, 'M_26')
  M_26 = params.M_26;
elseif isfield(params, 'M')
  M_26 = params.M{26};
else
  error 'could not find M_26'
end
if isfield(params, 'M_27')
  M_27 = params.M_27;
elseif isfield(params, 'M')
  M_27 = params.M{27};
else
  error 'could not find M_27'
end
if isfield(params, 'M_28')
  M_28 = params.M_28;
elseif isfield(params, 'M')
  M_28 = params.M{28};
else
  error 'could not find M_28'
end
if isfield(params, 'M_29')
  M_29 = params.M_29;
elseif isfield(params, 'M')
  M_29 = params.M{29};
else
  error 'could not find M_29'
end
if isfield(params, 'M_30')
  M_30 = params.M_30;
elseif isfield(params, 'M')
  M_30 = params.M{30};
else
  error 'could not find M_30'
end
if isfield(params, 'M_31')
  M_31 = params.M_31;
elseif isfield(params, 'M')
  M_31 = params.M{31};
else
  error 'could not find M_31'
end
if isfield(params, 'M_32')
  M_32 = params.M_32;
elseif isfield(params, 'M')
  M_32 = params.M{32};
else
  error 'could not find M_32'
end
if isfield(params, 'M_33')
  M_33 = params.M_33;
elseif isfield(params, 'M')
  M_33 = params.M{33};
else
  error 'could not find M_33'
end
if isfield(params, 'M_34')
  M_34 = params.M_34;
elseif isfield(params, 'M')
  M_34 = params.M{34};
else
  error 'could not find M_34'
end
if isfield(params, 'M_35')
  M_35 = params.M_35;
elseif isfield(params, 'M')
  M_35 = params.M{35};
else
  error 'could not find M_35'
end
Qw = params.Qw;
Qx = params.Qx;
Qy = params.Qy;
Qz = params.Qz;
T_max = params.T_max;
if isfield(params, 'XJbL_1')
  XJbL_1 = params.XJbL_1;
elseif isfield(params, 'XJbL')
  XJbL_1 = params.XJbL{1};
else
  error 'could not find XJbL_1'
end
if isfield(params, 'XJbL_2')
  XJbL_2 = params.XJbL_2;
elseif isfield(params, 'XJbL')
  XJbL_2 = params.XJbL{2};
else
  error 'could not find XJbL_2'
end
if isfield(params, 'XJbL_3')
  XJbL_3 = params.XJbL_3;
elseif isfield(params, 'XJbL')
  XJbL_3 = params.XJbL{3};
else
  error 'could not find XJbL_3'
end
if isfield(params, 'XJbL_4')
  XJbL_4 = params.XJbL_4;
elseif isfield(params, 'XJbL')
  XJbL_4 = params.XJbL{4};
else
  error 'could not find XJbL_4'
end
if isfield(params, 'XJbL_5')
  XJbL_5 = params.XJbL_5;
elseif isfield(params, 'XJbL')
  XJbL_5 = params.XJbL{5};
else
  error 'could not find XJbL_5'
end
if isfield(params, 'XJbL_6')
  XJbL_6 = params.XJbL_6;
elseif isfield(params, 'XJbL')
  XJbL_6 = params.XJbL{6};
else
  error 'could not find XJbL_6'
end
if isfield(params, 'XJbR_1')
  XJbR_1 = params.XJbR_1;
elseif isfield(params, 'XJbR')
  XJbR_1 = params.XJbR{1};
else
  error 'could not find XJbR_1'
end
if isfield(params, 'XJbR_2')
  XJbR_2 = params.XJbR_2;
elseif isfield(params, 'XJbR')
  XJbR_2 = params.XJbR{2};
else
  error 'could not find XJbR_2'
end
if isfield(params, 'XJbR_3')
  XJbR_3 = params.XJbR_3;
elseif isfield(params, 'XJbR')
  XJbR_3 = params.XJbR{3};
else
  error 'could not find XJbR_3'
end
if isfield(params, 'XJbR_4')
  XJbR_4 = params.XJbR_4;
elseif isfield(params, 'XJbR')
  XJbR_4 = params.XJbR{4};
else
  error 'could not find XJbR_4'
end
if isfield(params, 'XJbR_5')
  XJbR_5 = params.XJbR_5;
elseif isfield(params, 'XJbR')
  XJbR_5 = params.XJbR{5};
else
  error 'could not find XJbR_5'
end
if isfield(params, 'XJbR_6')
  XJbR_6 = params.XJbR_6;
elseif isfield(params, 'XJbR')
  XJbR_6 = params.XJbR{6};
else
  error 'could not find XJbR_6'
end
aL = params.aL;
aR = params.aR;
aw = params.aw;
b1 = params.b1;
b2 = params.b2;
b3 = params.b3;
b4 = params.b4;
if isfield(params, 'copMxl_1')
  copMxl_1 = params.copMxl_1;
elseif isfield(params, 'copMxl')
  copMxl_1 = params.copMxl{1};
else
  error 'could not find copMxl_1'
end
if isfield(params, 'copMxl_2')
  copMxl_2 = params.copMxl_2;
elseif isfield(params, 'copMxl')
  copMxl_2 = params.copMxl{2};
else
  error 'could not find copMxl_2'
end
if isfield(params, 'copMxr_1')
  copMxr_1 = params.copMxr_1;
elseif isfield(params, 'copMxr')
  copMxr_1 = params.copMxr{1};
else
  error 'could not find copMxr_1'
end
if isfield(params, 'copMxr_2')
  copMxr_2 = params.copMxr_2;
elseif isfield(params, 'copMxr')
  copMxr_2 = params.copMxr{2};
else
  error 'could not find copMxr_2'
end
dt = params.dt;
dwd = params.dwd;
q_max = params.q_max;
q_min = params.q_min;
v_max = params.v_max;
cvx_begin
  % Caution: automatically generated by cvxgen. May be incorrect.
  variable x(35, 1);
  variable y(29, 1);
  variable z(12, 1);
  variable w(85, 1);

  minimize(Qx'*square(x) + Qy'*square(y) + Qz'*square(z) + Qw'*square(w));
  subject to
    M_1(1)*x(1) + M_1(2)*x(2) + M_1(3)*x(3) + M_1(4)*x(4) + M_1(5)*x(5) + M_1(6)*x(6) + M_1(7)*x(7) + M_1(8)*x(8) + M_1(9)*x(9) + M_1(10)*x(10) + M_1(11)*x(11) + M_1(12)*x(12) + M_1(13)*x(13) + M_1(14)*x(14) + M_1(15)*x(15) + M_1(16)*x(16) + M_1(17)*x(17) + M_1(18)*x(18) + M_1(19)*x(19) + M_1(20)*x(20) + M_1(21)*x(21) + M_1(22)*x(22) + M_1(23)*x(23) + M_1(24)*x(24) + M_1(25)*x(25) + M_1(26)*x(26) + M_1(27)*x(27) + M_1(28)*x(28) + M_1(29)*x(29) + M_1(30)*x(30) + M_1(31)*x(31) + M_1(32)*x(32) + M_1(33)*x(33) + M_1(34)*x(34) + M_1(35)*x(35) - (aL*Jt_19(1)*z(1) + aR*Jt_25(1)*z(7) + aL*Jt_20(1)*z(2) + aR*Jt_26(1)*z(8) + aL*Jt_21(1)*z(3) + aR*Jt_27(1)*z(9)) == b1(1);
    M_2(1)*x(1) + M_2(2)*x(2) + M_2(3)*x(3) + M_2(4)*x(4) + M_2(5)*x(5) + M_2(6)*x(6) + M_2(7)*x(7) + M_2(8)*x(8) + M_2(9)*x(9) + M_2(10)*x(10) + M_2(11)*x(11) + M_2(12)*x(12) + M_2(13)*x(13) + M_2(14)*x(14) + M_2(15)*x(15) + M_2(16)*x(16) + M_2(17)*x(17) + M_2(18)*x(18) + M_2(19)*x(19) + M_2(20)*x(20) + M_2(21)*x(21) + M_2(22)*x(22) + M_2(23)*x(23) + M_2(24)*x(24) + M_2(25)*x(25) + M_2(26)*x(26) + M_2(27)*x(27) + M_2(28)*x(28) + M_2(29)*x(29) + M_2(30)*x(30) + M_2(31)*x(31) + M_2(32)*x(32) + M_2(33)*x(33) + M_2(34)*x(34) + M_2(35)*x(35) - (aL*Jt_19(2)*z(1) + aR*Jt_25(2)*z(7) + aL*Jt_20(2)*z(2) + aR*Jt_26(2)*z(8) + aL*Jt_21(2)*z(3) + aR*Jt_27(2)*z(9)) == b1(2);
    M_3(1)*x(1) + M_3(2)*x(2) + M_3(3)*x(3) + M_3(4)*x(4) + M_3(5)*x(5) + M_3(6)*x(6) + M_3(7)*x(7) + M_3(8)*x(8) + M_3(9)*x(9) + M_3(10)*x(10) + M_3(11)*x(11) + M_3(12)*x(12) + M_3(13)*x(13) + M_3(14)*x(14) + M_3(15)*x(15) + M_3(16)*x(16) + M_3(17)*x(17) + M_3(18)*x(18) + M_3(19)*x(19) + M_3(20)*x(20) + M_3(21)*x(21) + M_3(22)*x(22) + M_3(23)*x(23) + M_3(24)*x(24) + M_3(25)*x(25) + M_3(26)*x(26) + M_3(27)*x(27) + M_3(28)*x(28) + M_3(29)*x(29) + M_3(30)*x(30) + M_3(31)*x(31) + M_3(32)*x(32) + M_3(33)*x(33) + M_3(34)*x(34) + M_3(35)*x(35) - (aL*Jt_19(3)*z(1) + aR*Jt_25(3)*z(7) + aL*Jt_20(3)*z(2) + aR*Jt_26(3)*z(8) + aL*Jt_21(3)*z(3) + aR*Jt_27(3)*z(9)) == b1(3);
    M_4(1)*x(1) + M_4(2)*x(2) + M_4(3)*x(3) + M_4(4)*x(4) + M_4(5)*x(5) + M_4(6)*x(6) + M_4(7)*x(7) + M_4(8)*x(8) + M_4(9)*x(9) + M_4(10)*x(10) + M_4(11)*x(11) + M_4(12)*x(12) + M_4(13)*x(13) + M_4(14)*x(14) + M_4(15)*x(15) + M_4(16)*x(16) + M_4(17)*x(17) + M_4(18)*x(18) + M_4(19)*x(19) + M_4(20)*x(20) + M_4(21)*x(21) + M_4(22)*x(22) + M_4(23)*x(23) + M_4(24)*x(24) + M_4(25)*x(25) + M_4(26)*x(26) + M_4(27)*x(27) + M_4(28)*x(28) + M_4(29)*x(29) + M_4(30)*x(30) + M_4(31)*x(31) + M_4(32)*x(32) + M_4(33)*x(33) + M_4(34)*x(34) + M_4(35)*x(35) - (aL*Jt_19(4)*z(1) + aR*Jt_25(4)*z(7) + aL*Jt_20(4)*z(2) + aR*Jt_26(4)*z(8) + aL*Jt_21(4)*z(3) + aR*Jt_27(4)*z(9) + aL*Jt_22(4)*z(4) + aR*Jt_28(4)*z(10) + aL*Jt_23(4)*z(5) + aR*Jt_29(4)*z(11) + aL*Jt_24(4)*z(6) + aR*Jt_30(4)*z(12)) == b1(4);
    M_5(1)*x(1) + M_5(2)*x(2) + M_5(3)*x(3) + M_5(4)*x(4) + M_5(5)*x(5) + M_5(6)*x(6) + M_5(7)*x(7) + M_5(8)*x(8) + M_5(9)*x(9) + M_5(10)*x(10) + M_5(11)*x(11) + M_5(12)*x(12) + M_5(13)*x(13) + M_5(14)*x(14) + M_5(15)*x(15) + M_5(16)*x(16) + M_5(17)*x(17) + M_5(18)*x(18) + M_5(19)*x(19) + M_5(20)*x(20) + M_5(21)*x(21) + M_5(22)*x(22) + M_5(23)*x(23) + M_5(24)*x(24) + M_5(25)*x(25) + M_5(26)*x(26) + M_5(27)*x(27) + M_5(28)*x(28) + M_5(29)*x(29) + M_5(30)*x(30) + M_5(31)*x(31) + M_5(32)*x(32) + M_5(33)*x(33) + M_5(34)*x(34) + M_5(35)*x(35) - (aL*Jt_19(5)*z(1) + aR*Jt_25(5)*z(7) + aL*Jt_20(5)*z(2) + aR*Jt_26(5)*z(8) + aL*Jt_21(5)*z(3) + aR*Jt_27(5)*z(9) + aL*Jt_22(5)*z(4) + aR*Jt_28(5)*z(10) + aL*Jt_23(5)*z(5) + aR*Jt_29(5)*z(11) + aL*Jt_24(5)*z(6) + aR*Jt_30(5)*z(12)) == b1(5);
    M_6(1)*x(1) + M_6(2)*x(2) + M_6(3)*x(3) + M_6(4)*x(4) + M_6(5)*x(5) + M_6(6)*x(6) + M_6(7)*x(7) + M_6(8)*x(8) + M_6(9)*x(9) + M_6(10)*x(10) + M_6(11)*x(11) + M_6(12)*x(12) + M_6(13)*x(13) + M_6(14)*x(14) + M_6(15)*x(15) + M_6(16)*x(16) + M_6(17)*x(17) + M_6(18)*x(18) + M_6(19)*x(19) + M_6(20)*x(20) + M_6(21)*x(21) + M_6(22)*x(22) + M_6(23)*x(23) + M_6(24)*x(24) + M_6(25)*x(25) + M_6(26)*x(26) + M_6(27)*x(27) + M_6(28)*x(28) + M_6(29)*x(29) + M_6(30)*x(30) + M_6(31)*x(31) + M_6(32)*x(32) + M_6(33)*x(33) + M_6(34)*x(34) + M_6(35)*x(35) - (aL*Jt_19(6)*z(1) + aR*Jt_25(6)*z(7) + aL*Jt_20(6)*z(2) + aR*Jt_26(6)*z(8) + aL*Jt_21(6)*z(3) + aR*Jt_27(6)*z(9) + aL*Jt_22(6)*z(4) + aR*Jt_28(6)*z(10) + aL*Jt_23(6)*z(5) + aR*Jt_29(6)*z(11) + aL*Jt_24(6)*z(6) + aR*Jt_30(6)*z(12)) == b1(6);
    M_7(1)*x(1) + M_7(2)*x(2) + M_7(3)*x(3) + M_7(4)*x(4) + M_7(5)*x(5) + M_7(6)*x(6) + M_7(7)*x(7) + M_7(8)*x(8) + M_7(9)*x(9) + M_7(10)*x(10) + M_7(11)*x(11) + M_7(12)*x(12) + M_7(13)*x(13) + M_7(14)*x(14) + M_7(15)*x(15) + M_7(16)*x(16) + M_7(17)*x(17) + M_7(18)*x(18) + M_7(19)*x(19) + M_7(20)*x(20) + M_7(21)*x(21) + M_7(22)*x(22) + M_7(23)*x(23) - y(1) == b1(7);
    M_8(1)*x(1) + M_8(2)*x(2) + M_8(3)*x(3) + M_8(4)*x(4) + M_8(5)*x(5) + M_8(6)*x(6) + M_8(7)*x(7) + M_8(8)*x(8) + M_8(9)*x(9) + M_8(10)*x(10) + M_8(11)*x(11) + M_8(12)*x(12) + M_8(13)*x(13) + M_8(14)*x(14) + M_8(15)*x(15) + M_8(16)*x(16) + M_8(17)*x(17) + M_8(18)*x(18) + M_8(19)*x(19) + M_8(20)*x(20) + M_8(21)*x(21) + M_8(22)*x(22) + M_8(23)*x(23) - y(2) == b1(8);
    M_9(1)*x(1) + M_9(2)*x(2) + M_9(3)*x(3) + M_9(4)*x(4) + M_9(5)*x(5) + M_9(6)*x(6) + M_9(7)*x(7) + M_9(8)*x(8) + M_9(9)*x(9) + M_9(10)*x(10) + M_9(11)*x(11) + M_9(12)*x(12) + M_9(13)*x(13) + M_9(14)*x(14) + M_9(15)*x(15) + M_9(16)*x(16) + M_9(17)*x(17) + M_9(18)*x(18) + M_9(19)*x(19) + M_9(20)*x(20) + M_9(21)*x(21) + M_9(22)*x(22) + M_9(23)*x(23) - y(3) == b1(9);
    M_10(1)*x(1) + M_10(2)*x(2) + M_10(3)*x(3) + M_10(4)*x(4) + M_10(5)*x(5) + M_10(6)*x(6) + M_10(7)*x(7) + M_10(8)*x(8) + M_10(9)*x(9) + M_10(10)*x(10) + M_10(11)*x(11) + M_10(12)*x(12) + M_10(13)*x(13) + M_10(14)*x(14) + M_10(15)*x(15) + M_10(16)*x(16) - y(4) == b1(10);
    M_11(1)*x(1) + M_11(2)*x(2) + M_11(3)*x(3) + M_11(4)*x(4) + M_11(5)*x(5) + M_11(6)*x(6) + M_11(7)*x(7) + M_11(8)*x(8) + M_11(9)*x(9) + M_11(10)*x(10) + M_11(11)*x(11) + M_11(12)*x(12) + M_11(13)*x(13) + M_11(14)*x(14) + M_11(15)*x(15) + M_11(16)*x(16) - y(5) == b1(11);
    M_12(1)*x(1) + M_12(2)*x(2) + M_12(3)*x(3) + M_12(4)*x(4) + M_12(5)*x(5) + M_12(6)*x(6) + M_12(7)*x(7) + M_12(8)*x(8) + M_12(9)*x(9) + M_12(10)*x(10) + M_12(11)*x(11) + M_12(12)*x(12) + M_12(13)*x(13) + M_12(14)*x(14) + M_12(15)*x(15) + M_12(16)*x(16) - y(6) == b1(12);
    M_13(1)*x(1) + M_13(2)*x(2) + M_13(3)*x(3) + M_13(4)*x(4) + M_13(5)*x(5) + M_13(6)*x(6) + M_13(7)*x(7) + M_13(8)*x(8) + M_13(9)*x(9) + M_13(10)*x(10) + M_13(11)*x(11) + M_13(12)*x(12) + M_13(13)*x(13) + M_13(14)*x(14) + M_13(15)*x(15) + M_13(16)*x(16) - y(7) == b1(13);
    M_14(1)*x(1) + M_14(2)*x(2) + M_14(3)*x(3) + M_14(4)*x(4) + M_14(5)*x(5) + M_14(6)*x(6) + M_14(7)*x(7) + M_14(8)*x(8) + M_14(9)*x(9) + M_14(10)*x(10) + M_14(11)*x(11) + M_14(12)*x(12) + M_14(13)*x(13) + M_14(14)*x(14) + M_14(15)*x(15) + M_14(16)*x(16) - y(8) == b1(14);
    M_15(1)*x(1) + M_15(2)*x(2) + M_15(3)*x(3) + M_15(4)*x(4) + M_15(5)*x(5) + M_15(6)*x(6) + M_15(7)*x(7) + M_15(8)*x(8) + M_15(9)*x(9) + M_15(10)*x(10) + M_15(11)*x(11) + M_15(12)*x(12) + M_15(13)*x(13) + M_15(14)*x(14) + M_15(15)*x(15) + M_15(16)*x(16) - y(9) == b1(15);
    M_16(1)*x(1) + M_16(2)*x(2) + M_16(3)*x(3) + M_16(4)*x(4) + M_16(5)*x(5) + M_16(6)*x(6) + M_16(7)*x(7) + M_16(8)*x(8) + M_16(9)*x(9) + M_16(10)*x(10) + M_16(11)*x(11) + M_16(12)*x(12) + M_16(13)*x(13) + M_16(14)*x(14) + M_16(15)*x(15) + M_16(16)*x(16) - y(10) == b1(16);
    M_17(1)*x(1) + M_17(2)*x(2) + M_17(3)*x(3) + M_17(4)*x(4) + M_17(5)*x(5) + M_17(6)*x(6) + M_17(7)*x(7) + M_17(8)*x(8) + M_17(9)*x(9) + M_17(17)*x(17) + M_17(18)*x(18) + M_17(19)*x(19) + M_17(20)*x(20) + M_17(21)*x(21) + M_17(22)*x(22) + M_17(23)*x(23) - y(11) == b1(17);
    M_18(1)*x(1) + M_18(2)*x(2) + M_18(3)*x(3) + M_18(4)*x(4) + M_18(5)*x(5) + M_18(6)*x(6) + M_18(7)*x(7) + M_18(8)*x(8) + M_18(9)*x(9) + M_18(17)*x(17) + M_18(18)*x(18) + M_18(19)*x(19) + M_18(20)*x(20) + M_18(21)*x(21) + M_18(22)*x(22) + M_18(23)*x(23) - y(12) == b1(18);
    M_19(1)*x(1) + M_19(2)*x(2) + M_19(3)*x(3) + M_19(4)*x(4) + M_19(5)*x(5) + M_19(6)*x(6) + M_19(7)*x(7) + M_19(8)*x(8) + M_19(9)*x(9) + M_19(17)*x(17) + M_19(18)*x(18) + M_19(19)*x(19) + M_19(20)*x(20) + M_19(21)*x(21) + M_19(22)*x(22) + M_19(23)*x(23) - y(13) == b1(19);
    M_20(1)*x(1) + M_20(2)*x(2) + M_20(3)*x(3) + M_20(4)*x(4) + M_20(5)*x(5) + M_20(6)*x(6) + M_20(7)*x(7) + M_20(8)*x(8) + M_20(9)*x(9) + M_20(17)*x(17) + M_20(18)*x(18) + M_20(19)*x(19) + M_20(20)*x(20) + M_20(21)*x(21) + M_20(22)*x(22) + M_20(23)*x(23) - y(14) == b1(20);
    M_21(1)*x(1) + M_21(2)*x(2) + M_21(3)*x(3) + M_21(4)*x(4) + M_21(5)*x(5) + M_21(6)*x(6) + M_21(7)*x(7) + M_21(8)*x(8) + M_21(9)*x(9) + M_21(17)*x(17) + M_21(18)*x(18) + M_21(19)*x(19) + M_21(20)*x(20) + M_21(21)*x(21) + M_21(22)*x(22) + M_21(23)*x(23) - y(15) == b1(21);
    M_22(1)*x(1) + M_22(2)*x(2) + M_22(3)*x(3) + M_22(4)*x(4) + M_22(5)*x(5) + M_22(6)*x(6) + M_22(7)*x(7) + M_22(8)*x(8) + M_22(9)*x(9) + M_22(17)*x(17) + M_22(18)*x(18) + M_22(19)*x(19) + M_22(20)*x(20) + M_22(21)*x(21) + M_22(22)*x(22) + M_22(23)*x(23) - y(16) == b1(22);
    M_23(1)*x(1) + M_23(2)*x(2) + M_23(3)*x(3) + M_23(4)*x(4) + M_23(5)*x(5) + M_23(6)*x(6) + M_23(7)*x(7) + M_23(8)*x(8) + M_23(9)*x(9) + M_23(17)*x(17) + M_23(18)*x(18) + M_23(19)*x(19) + M_23(20)*x(20) + M_23(21)*x(21) + M_23(22)*x(22) + M_23(23)*x(23) - y(17) == b1(23);
    M_24(1)*x(1) + M_24(2)*x(2) + M_24(3)*x(3) + M_24(4)*x(4) + M_24(5)*x(5) + M_24(6)*x(6) + M_24(24)*x(24) + M_24(25)*x(25) + M_24(26)*x(26) + M_24(27)*x(27) + M_24(28)*x(28) + M_24(29)*x(29) - y(18) - (aL*Jt_19(24)*z(1) + aL*Jt_20(24)*z(2) + aL*Jt_21(24)*z(3) + aL*Jt_22(24)*z(4) + aL*Jt_23(24)*z(5) + aL*Jt_24(24)*z(6)) == b1(24);
    M_25(1)*x(1) + M_25(2)*x(2) + M_25(3)*x(3) + M_25(4)*x(4) + M_25(5)*x(5) + M_25(6)*x(6) + M_25(24)*x(24) + M_25(25)*x(25) + M_25(26)*x(26) + M_25(27)*x(27) + M_25(28)*x(28) + M_25(29)*x(29) - y(19) - (aL*Jt_19(25)*z(1) + aL*Jt_20(25)*z(2) + aL*Jt_21(25)*z(3) + aL*Jt_22(25)*z(4) + aL*Jt_23(25)*z(5) + aL*Jt_24(25)*z(6)) == b1(25);
    M_26(1)*x(1) + M_26(2)*x(2) + M_26(3)*x(3) + M_26(4)*x(4) + M_26(5)*x(5) + M_26(6)*x(6) + M_26(24)*x(24) + M_26(25)*x(25) + M_26(26)*x(26) + M_26(27)*x(27) + M_26(28)*x(28) + M_26(29)*x(29) - y(20) - (aL*Jt_19(26)*z(1) + aL*Jt_20(26)*z(2) + aL*Jt_21(26)*z(3) + aL*Jt_22(26)*z(4) + aL*Jt_23(26)*z(5) + aL*Jt_24(26)*z(6)) == b1(26);
    M_27(1)*x(1) + M_27(2)*x(2) + M_27(3)*x(3) + M_27(4)*x(4) + M_27(5)*x(5) + M_27(6)*x(6) + M_27(24)*x(24) + M_27(25)*x(25) + M_27(26)*x(26) + M_27(27)*x(27) + M_27(28)*x(28) + M_27(29)*x(29) - y(21) - (aL*Jt_19(27)*z(1) + aL*Jt_20(27)*z(2) + aL*Jt_21(27)*z(3) + aL*Jt_22(27)*z(4) + aL*Jt_23(27)*z(5) + aL*Jt_24(27)*z(6)) == b1(27);
    M_28(1)*x(1) + M_28(2)*x(2) + M_28(3)*x(3) + M_28(4)*x(4) + M_28(5)*x(5) + M_28(6)*x(6) + M_28(24)*x(24) + M_28(25)*x(25) + M_28(26)*x(26) + M_28(27)*x(27) + M_28(28)*x(28) + M_28(29)*x(29) - y(22) - (aL*Jt_19(28)*z(1) + aL*Jt_20(28)*z(2) + aL*Jt_21(28)*z(3) + aL*Jt_22(28)*z(4) + aL*Jt_23(28)*z(5) + aL*Jt_24(28)*z(6)) == b1(28);
    M_29(1)*x(1) + M_29(2)*x(2) + M_29(3)*x(3) + M_29(4)*x(4) + M_29(5)*x(5) + M_29(6)*x(6) + M_29(24)*x(24) + M_29(25)*x(25) + M_29(26)*x(26) + M_29(27)*x(27) + M_29(28)*x(28) + M_29(29)*x(29) - y(23) - (aL*Jt_19(29)*z(1) + aL*Jt_20(29)*z(2) + aL*Jt_21(29)*z(3) + aL*Jt_22(29)*z(4) + aL*Jt_23(29)*z(5) + aL*Jt_24(29)*z(6)) == b1(29);
    M_30(1)*x(1) + M_30(2)*x(2) + M_30(3)*x(3) + M_30(4)*x(4) + M_30(5)*x(5) + M_30(6)*x(6) + M_30(30)*x(30) + M_30(31)*x(31) + M_30(32)*x(32) + M_30(33)*x(33) + M_30(34)*x(34) + M_30(35)*x(35) - y(24) - (aR*Jt_25(30)*z(7) + aR*Jt_26(30)*z(8) + aR*Jt_27(30)*z(9) + aR*Jt_28(30)*z(10) + aR*Jt_29(30)*z(11) + aR*Jt_30(30)*z(12)) == b1(30);
    M_31(1)*x(1) + M_31(2)*x(2) + M_31(3)*x(3) + M_31(4)*x(4) + M_31(5)*x(5) + M_31(6)*x(6) + M_31(30)*x(30) + M_31(31)*x(31) + M_31(32)*x(32) + M_31(33)*x(33) + M_31(34)*x(34) + M_31(35)*x(35) - y(25) - (aR*Jt_25(31)*z(7) + aR*Jt_26(31)*z(8) + aR*Jt_27(31)*z(9) + aR*Jt_28(31)*z(10) + aR*Jt_29(31)*z(11) + aR*Jt_30(31)*z(12)) == b1(31);
    M_32(1)*x(1) + M_32(2)*x(2) + M_32(3)*x(3) + M_32(4)*x(4) + M_32(5)*x(5) + M_32(6)*x(6) + M_32(30)*x(30) + M_32(31)*x(31) + M_32(32)*x(32) + M_32(33)*x(33) + M_32(34)*x(34) + M_32(35)*x(35) - y(26) - (aR*Jt_25(32)*z(7) + aR*Jt_26(32)*z(8) + aR*Jt_27(32)*z(9) + aR*Jt_28(32)*z(10) + aR*Jt_29(32)*z(11) + aR*Jt_30(32)*z(12)) == b1(32);
    M_33(1)*x(1) + M_33(2)*x(2) + M_33(3)*x(3) + M_33(4)*x(4) + M_33(5)*x(5) + M_33(6)*x(6) + M_33(30)*x(30) + M_33(31)*x(31) + M_33(32)*x(32) + M_33(33)*x(33) + M_33(34)*x(34) + M_33(35)*x(35) - y(27) - (aR*Jt_25(33)*z(7) + aR*Jt_26(33)*z(8) + aR*Jt_27(33)*z(9) + aR*Jt_28(33)*z(10) + aR*Jt_29(33)*z(11) + aR*Jt_30(33)*z(12)) == b1(33);
    M_34(1)*x(1) + M_34(2)*x(2) + M_34(3)*x(3) + M_34(4)*x(4) + M_34(5)*x(5) + M_34(6)*x(6) + M_34(30)*x(30) + M_34(31)*x(31) + M_34(32)*x(32) + M_34(33)*x(33) + M_34(34)*x(34) + M_34(35)*x(35) - y(28) - (aR*Jt_25(34)*z(7) + aR*Jt_26(34)*z(8) + aR*Jt_27(34)*z(9) + aR*Jt_28(34)*z(10) + aR*Jt_29(34)*z(11) + aR*Jt_30(34)*z(12)) == b1(34);
    M_35(1)*x(1) + M_35(2)*x(2) + M_35(3)*x(3) + M_35(4)*x(4) + M_35(5)*x(5) + M_35(6)*x(6) + M_35(30)*x(30) + M_35(31)*x(31) + M_35(32)*x(32) + M_35(33)*x(33) + M_35(34)*x(34) + M_35(35)*x(35) - y(29) - (aR*Jt_25(35)*z(7) + aR*Jt_26(35)*z(8) + aR*Jt_27(35)*z(9) + aR*Jt_28(35)*z(10) + aR*Jt_29(35)*z(11) + aR*Jt_30(35)*z(12)) == b1(35);
    aL*XJbL_1(1)*z(1) + aR*XJbR_1(1)*z(7) + aL*XJbL_1(2)*z(2) + aR*XJbR_1(2)*z(8) + aL*XJbL_1(3)*z(3) + aR*XJbR_1(3)*z(9) + aL*XJbL_1(4)*z(4) + aR*XJbR_1(4)*z(10) + aL*XJbL_1(5)*z(5) + aR*XJbR_1(5)*z(11) + aL*XJbL_1(6)*z(6) + aR*XJbR_1(6)*z(12) + w(31) == b2(1);
    aL*XJbL_2(1)*z(1) + aR*XJbR_2(1)*z(7) + aL*XJbL_2(2)*z(2) + aR*XJbR_2(2)*z(8) + aL*XJbL_2(3)*z(3) + aR*XJbR_2(3)*z(9) + aL*XJbL_2(4)*z(4) + aR*XJbR_2(4)*z(10) + aL*XJbL_2(5)*z(5) + aR*XJbR_2(5)*z(11) + aL*XJbL_2(6)*z(6) + aR*XJbR_2(6)*z(12) + w(32) == b2(2);
    aL*XJbL_3(1)*z(1) + aR*XJbR_3(1)*z(7) + aL*XJbL_3(2)*z(2) + aR*XJbR_3(2)*z(8) + aL*XJbL_3(3)*z(3) + aR*XJbR_3(3)*z(9) + aL*XJbL_3(4)*z(4) + aR*XJbR_3(4)*z(10) + aL*XJbL_3(5)*z(5) + aR*XJbR_3(5)*z(11) + aL*XJbL_3(6)*z(6) + aR*XJbR_3(6)*z(12) + w(33) == b2(3);
    aL*XJbL_4(1)*z(1) + aR*XJbR_4(1)*z(7) + aL*XJbL_4(2)*z(2) + aR*XJbR_4(2)*z(8) + aL*XJbL_4(3)*z(3) + aR*XJbR_4(3)*z(9) + aL*XJbL_4(4)*z(4) + aR*XJbR_4(4)*z(10) + aL*XJbL_4(5)*z(5) + aR*XJbR_4(5)*z(11) + aL*XJbL_4(6)*z(6) + aR*XJbR_4(6)*z(12) + w(34) == b2(4);
    aL*XJbL_5(1)*z(1) + aR*XJbR_5(1)*z(7) + aL*XJbL_5(2)*z(2) + aR*XJbR_5(2)*z(8) + aL*XJbL_5(3)*z(3) + aR*XJbR_5(3)*z(9) + aL*XJbL_5(4)*z(4) + aR*XJbR_5(4)*z(10) + aL*XJbL_5(5)*z(5) + aR*XJbR_5(5)*z(11) + aL*XJbL_5(6)*z(6) + aR*XJbR_5(6)*z(12) + w(35) == b2(5);
    aL*XJbL_6(1)*z(1) + aR*XJbR_6(1)*z(7) + aL*XJbL_6(2)*z(2) + aR*XJbR_6(2)*z(8) + aL*XJbL_6(3)*z(3) + aR*XJbR_6(3)*z(9) + aL*XJbL_6(4)*z(4) + aR*XJbR_6(4)*z(10) + aL*XJbL_6(5)*z(5) + aR*XJbR_6(5)*z(11) + aL*XJbL_6(6)*z(6) + aR*XJbR_6(6)*z(12) + w(36) == b2(6);
    Jt_1(1)*x(1) + Jt_1(2)*x(2) + Jt_1(3)*x(3) + Jt_1(4)*x(4) + Jt_1(5)*x(5) + Jt_1(6)*x(6) + Jt_1(7)*x(7) + Jt_1(8)*x(8) + Jt_1(9)*x(9) + Jt_1(10)*x(10) + Jt_1(11)*x(11) + Jt_1(12)*x(12) + Jt_1(13)*x(13) + Jt_1(14)*x(14) + Jt_1(15)*x(15) + Jt_1(16)*x(16) + Jt_1(17)*x(17) + Jt_1(18)*x(18) + Jt_1(19)*x(19) + Jt_1(20)*x(20) + Jt_1(21)*x(21) + Jt_1(22)*x(22) + Jt_1(23)*x(23) + Jt_1(24)*x(24) + Jt_1(25)*x(25) + Jt_1(26)*x(26) + Jt_1(27)*x(27) + Jt_1(28)*x(28) + Jt_1(29)*x(29) + Jt_1(30)*x(30) + Jt_1(31)*x(31) + Jt_1(32)*x(32) + Jt_1(33)*x(33) + Jt_1(34)*x(34) + Jt_1(35)*x(35) + w(1) == b3(1);
    Jt_2(1)*x(1) + Jt_2(2)*x(2) + Jt_2(3)*x(3) + Jt_2(4)*x(4) + Jt_2(5)*x(5) + Jt_2(6)*x(6) + Jt_2(7)*x(7) + Jt_2(8)*x(8) + Jt_2(9)*x(9) + Jt_2(10)*x(10) + Jt_2(11)*x(11) + Jt_2(12)*x(12) + Jt_2(13)*x(13) + Jt_2(14)*x(14) + Jt_2(15)*x(15) + Jt_2(16)*x(16) + Jt_2(17)*x(17) + Jt_2(18)*x(18) + Jt_2(19)*x(19) + Jt_2(20)*x(20) + Jt_2(21)*x(21) + Jt_2(22)*x(22) + Jt_2(23)*x(23) + Jt_2(24)*x(24) + Jt_2(25)*x(25) + Jt_2(26)*x(26) + Jt_2(27)*x(27) + Jt_2(28)*x(28) + Jt_2(29)*x(29) + Jt_2(30)*x(30) + Jt_2(31)*x(31) + Jt_2(32)*x(32) + Jt_2(33)*x(33) + Jt_2(34)*x(34) + Jt_2(35)*x(35) + w(2) == b3(2);
    Jt_3(1)*x(1) + Jt_3(2)*x(2) + Jt_3(3)*x(3) + Jt_3(4)*x(4) + Jt_3(5)*x(5) + Jt_3(6)*x(6) + Jt_3(7)*x(7) + Jt_3(8)*x(8) + Jt_3(9)*x(9) + Jt_3(10)*x(10) + Jt_3(11)*x(11) + Jt_3(12)*x(12) + Jt_3(13)*x(13) + Jt_3(14)*x(14) + Jt_3(15)*x(15) + Jt_3(16)*x(16) + Jt_3(17)*x(17) + Jt_3(18)*x(18) + Jt_3(19)*x(19) + Jt_3(20)*x(20) + Jt_3(21)*x(21) + Jt_3(22)*x(22) + Jt_3(23)*x(23) + Jt_3(24)*x(24) + Jt_3(25)*x(25) + Jt_3(26)*x(26) + Jt_3(27)*x(27) + Jt_3(28)*x(28) + Jt_3(29)*x(29) + Jt_3(30)*x(30) + Jt_3(31)*x(31) + Jt_3(32)*x(32) + Jt_3(33)*x(33) + Jt_3(34)*x(34) + Jt_3(35)*x(35) + w(3) == b3(3);
    Jt_4(1)*x(1) + Jt_4(2)*x(2) + Jt_4(3)*x(3) + Jt_4(4)*x(4) + Jt_4(5)*x(5) + Jt_4(6)*x(6) + Jt_4(7)*x(7) + Jt_4(8)*x(8) + Jt_4(9)*x(9) + Jt_4(10)*x(10) + Jt_4(11)*x(11) + Jt_4(12)*x(12) + Jt_4(13)*x(13) + Jt_4(14)*x(14) + Jt_4(15)*x(15) + Jt_4(16)*x(16) + Jt_4(17)*x(17) + Jt_4(18)*x(18) + Jt_4(19)*x(19) + Jt_4(20)*x(20) + Jt_4(21)*x(21) + Jt_4(22)*x(22) + Jt_4(23)*x(23) + Jt_4(24)*x(24) + Jt_4(25)*x(25) + Jt_4(26)*x(26) + Jt_4(27)*x(27) + Jt_4(28)*x(28) + Jt_4(29)*x(29) + Jt_4(30)*x(30) + Jt_4(31)*x(31) + Jt_4(32)*x(32) + Jt_4(33)*x(33) + Jt_4(34)*x(34) + Jt_4(35)*x(35) + w(4) == b3(4);
    Jt_5(1)*x(1) + Jt_5(2)*x(2) + Jt_5(3)*x(3) + Jt_5(4)*x(4) + Jt_5(5)*x(5) + Jt_5(6)*x(6) + Jt_5(7)*x(7) + Jt_5(8)*x(8) + Jt_5(9)*x(9) + Jt_5(10)*x(10) + Jt_5(11)*x(11) + Jt_5(12)*x(12) + Jt_5(13)*x(13) + Jt_5(14)*x(14) + Jt_5(15)*x(15) + Jt_5(16)*x(16) + Jt_5(17)*x(17) + Jt_5(18)*x(18) + Jt_5(19)*x(19) + Jt_5(20)*x(20) + Jt_5(21)*x(21) + Jt_5(22)*x(22) + Jt_5(23)*x(23) + Jt_5(24)*x(24) + Jt_5(25)*x(25) + Jt_5(26)*x(26) + Jt_5(27)*x(27) + Jt_5(28)*x(28) + Jt_5(29)*x(29) + Jt_5(30)*x(30) + Jt_5(31)*x(31) + Jt_5(32)*x(32) + Jt_5(33)*x(33) + Jt_5(34)*x(34) + Jt_5(35)*x(35) + w(5) == b3(5);
    Jt_6(1)*x(1) + Jt_6(2)*x(2) + Jt_6(3)*x(3) + Jt_6(4)*x(4) + Jt_6(5)*x(5) + Jt_6(6)*x(6) + Jt_6(7)*x(7) + Jt_6(8)*x(8) + Jt_6(9)*x(9) + Jt_6(10)*x(10) + Jt_6(11)*x(11) + Jt_6(12)*x(12) + Jt_6(13)*x(13) + Jt_6(14)*x(14) + Jt_6(15)*x(15) + Jt_6(16)*x(16) + Jt_6(17)*x(17) + Jt_6(18)*x(18) + Jt_6(19)*x(19) + Jt_6(20)*x(20) + Jt_6(21)*x(21) + Jt_6(22)*x(22) + Jt_6(23)*x(23) + Jt_6(24)*x(24) + Jt_6(25)*x(25) + Jt_6(26)*x(26) + Jt_6(27)*x(27) + Jt_6(28)*x(28) + Jt_6(29)*x(29) + Jt_6(30)*x(30) + Jt_6(31)*x(31) + Jt_6(32)*x(32) + Jt_6(33)*x(33) + Jt_6(34)*x(34) + Jt_6(35)*x(35) + w(6) == b3(6);
    Jt_7(1)*x(1) + Jt_7(2)*x(2) + Jt_7(3)*x(3) + Jt_7(4)*x(4) + Jt_7(5)*x(5) + Jt_7(6)*x(6) + Jt_7(7)*x(7) + Jt_7(8)*x(8) + Jt_7(9)*x(9) + Jt_7(10)*x(10) + Jt_7(11)*x(11) + Jt_7(12)*x(12) + Jt_7(13)*x(13) + Jt_7(14)*x(14) + Jt_7(15)*x(15) + Jt_7(16)*x(16) + w(7) == b3(7);
    Jt_8(1)*x(1) + Jt_8(2)*x(2) + Jt_8(3)*x(3) + Jt_8(4)*x(4) + Jt_8(5)*x(5) + Jt_8(6)*x(6) + Jt_8(7)*x(7) + Jt_8(8)*x(8) + Jt_8(9)*x(9) + Jt_8(10)*x(10) + Jt_8(11)*x(11) + Jt_8(12)*x(12) + Jt_8(13)*x(13) + Jt_8(14)*x(14) + Jt_8(15)*x(15) + Jt_8(16)*x(16) + w(8) == b3(8);
    Jt_9(1)*x(1) + Jt_9(2)*x(2) + Jt_9(3)*x(3) + Jt_9(4)*x(4) + Jt_9(5)*x(5) + Jt_9(6)*x(6) + Jt_9(7)*x(7) + Jt_9(8)*x(8) + Jt_9(9)*x(9) + Jt_9(10)*x(10) + Jt_9(11)*x(11) + Jt_9(12)*x(12) + Jt_9(13)*x(13) + Jt_9(14)*x(14) + Jt_9(15)*x(15) + Jt_9(16)*x(16) + w(9) == b3(9);
    Jt_10(1)*x(1) + Jt_10(2)*x(2) + Jt_10(3)*x(3) + Jt_10(4)*x(4) + Jt_10(5)*x(5) + Jt_10(6)*x(6) + Jt_10(7)*x(7) + Jt_10(8)*x(8) + Jt_10(9)*x(9) + Jt_10(10)*x(10) + Jt_10(11)*x(11) + Jt_10(12)*x(12) + Jt_10(13)*x(13) + Jt_10(14)*x(14) + Jt_10(15)*x(15) + Jt_10(16)*x(16) + w(10) == b3(10);
    Jt_11(1)*x(1) + Jt_11(2)*x(2) + Jt_11(3)*x(3) + Jt_11(4)*x(4) + Jt_11(5)*x(5) + Jt_11(6)*x(6) + Jt_11(7)*x(7) + Jt_11(8)*x(8) + Jt_11(9)*x(9) + Jt_11(10)*x(10) + Jt_11(11)*x(11) + Jt_11(12)*x(12) + Jt_11(13)*x(13) + Jt_11(14)*x(14) + Jt_11(15)*x(15) + Jt_11(16)*x(16) + w(11) == b3(11);
    Jt_12(1)*x(1) + Jt_12(2)*x(2) + Jt_12(3)*x(3) + Jt_12(4)*x(4) + Jt_12(5)*x(5) + Jt_12(6)*x(6) + Jt_12(7)*x(7) + Jt_12(8)*x(8) + Jt_12(9)*x(9) + Jt_12(10)*x(10) + Jt_12(11)*x(11) + Jt_12(12)*x(12) + Jt_12(13)*x(13) + Jt_12(14)*x(14) + Jt_12(15)*x(15) + Jt_12(16)*x(16) + w(12) == b3(12);
    Jt_13(1)*x(1) + Jt_13(2)*x(2) + Jt_13(3)*x(3) + Jt_13(4)*x(4) + Jt_13(5)*x(5) + Jt_13(6)*x(6) + Jt_13(7)*x(7) + Jt_13(8)*x(8) + Jt_13(9)*x(9) + Jt_13(17)*x(17) + Jt_13(18)*x(18) + Jt_13(19)*x(19) + Jt_13(20)*x(20) + Jt_13(21)*x(21) + Jt_13(22)*x(22) + Jt_13(23)*x(23) + w(13) == b3(13);
    Jt_14(1)*x(1) + Jt_14(2)*x(2) + Jt_14(3)*x(3) + Jt_14(4)*x(4) + Jt_14(5)*x(5) + Jt_14(6)*x(6) + Jt_14(7)*x(7) + Jt_14(8)*x(8) + Jt_14(9)*x(9) + Jt_14(17)*x(17) + Jt_14(18)*x(18) + Jt_14(19)*x(19) + Jt_14(20)*x(20) + Jt_14(21)*x(21) + Jt_14(22)*x(22) + Jt_14(23)*x(23) + w(14) == b3(14);
    Jt_15(1)*x(1) + Jt_15(2)*x(2) + Jt_15(3)*x(3) + Jt_15(4)*x(4) + Jt_15(5)*x(5) + Jt_15(6)*x(6) + Jt_15(7)*x(7) + Jt_15(8)*x(8) + Jt_15(9)*x(9) + Jt_15(17)*x(17) + Jt_15(18)*x(18) + Jt_15(19)*x(19) + Jt_15(20)*x(20) + Jt_15(21)*x(21) + Jt_15(22)*x(22) + Jt_15(23)*x(23) + w(15) == b3(15);
    Jt_16(1)*x(1) + Jt_16(2)*x(2) + Jt_16(3)*x(3) + Jt_16(4)*x(4) + Jt_16(5)*x(5) + Jt_16(6)*x(6) + Jt_16(7)*x(7) + Jt_16(8)*x(8) + Jt_16(9)*x(9) + Jt_16(17)*x(17) + Jt_16(18)*x(18) + Jt_16(19)*x(19) + Jt_16(20)*x(20) + Jt_16(21)*x(21) + Jt_16(22)*x(22) + Jt_16(23)*x(23) + w(16) == b3(16);
    Jt_17(1)*x(1) + Jt_17(2)*x(2) + Jt_17(3)*x(3) + Jt_17(4)*x(4) + Jt_17(5)*x(5) + Jt_17(6)*x(6) + Jt_17(7)*x(7) + Jt_17(8)*x(8) + Jt_17(9)*x(9) + Jt_17(17)*x(17) + Jt_17(18)*x(18) + Jt_17(19)*x(19) + Jt_17(20)*x(20) + Jt_17(21)*x(21) + Jt_17(22)*x(22) + Jt_17(23)*x(23) + w(17) == b3(17);
    Jt_18(1)*x(1) + Jt_18(2)*x(2) + Jt_18(3)*x(3) + Jt_18(4)*x(4) + Jt_18(5)*x(5) + Jt_18(6)*x(6) + Jt_18(7)*x(7) + Jt_18(8)*x(8) + Jt_18(9)*x(9) + Jt_18(17)*x(17) + Jt_18(18)*x(18) + Jt_18(19)*x(19) + Jt_18(20)*x(20) + Jt_18(21)*x(21) + Jt_18(22)*x(22) + Jt_18(23)*x(23) + w(18) == b3(18);
    Jt_19(1)*x(1) + Jt_19(2)*x(2) + Jt_19(3)*x(3) + Jt_19(4)*x(4) + Jt_19(5)*x(5) + Jt_19(6)*x(6) + Jt_19(24)*x(24) + Jt_19(25)*x(25) + Jt_19(26)*x(26) + Jt_19(27)*x(27) + Jt_19(28)*x(28) + Jt_19(29)*x(29) + w(19) == b3(19);
    Jt_20(1)*x(1) + Jt_20(2)*x(2) + Jt_20(3)*x(3) + Jt_20(4)*x(4) + Jt_20(5)*x(5) + Jt_20(6)*x(6) + Jt_20(24)*x(24) + Jt_20(25)*x(25) + Jt_20(26)*x(26) + Jt_20(27)*x(27) + Jt_20(28)*x(28) + Jt_20(29)*x(29) + w(20) == b3(20);
    Jt_21(1)*x(1) + Jt_21(2)*x(2) + Jt_21(3)*x(3) + Jt_21(4)*x(4) + Jt_21(5)*x(5) + Jt_21(6)*x(6) + Jt_21(24)*x(24) + Jt_21(25)*x(25) + Jt_21(26)*x(26) + Jt_21(27)*x(27) + Jt_21(28)*x(28) + Jt_21(29)*x(29) + w(21) == b3(21);
    Jt_22(1)*x(1) + Jt_22(2)*x(2) + Jt_22(3)*x(3) + Jt_22(4)*x(4) + Jt_22(5)*x(5) + Jt_22(6)*x(6) + Jt_22(24)*x(24) + Jt_22(25)*x(25) + Jt_22(26)*x(26) + Jt_22(27)*x(27) + Jt_22(28)*x(28) + Jt_22(29)*x(29) + w(22) == b3(22);
    Jt_23(1)*x(1) + Jt_23(2)*x(2) + Jt_23(3)*x(3) + Jt_23(4)*x(4) + Jt_23(5)*x(5) + Jt_23(6)*x(6) + Jt_23(24)*x(24) + Jt_23(25)*x(25) + Jt_23(26)*x(26) + Jt_23(27)*x(27) + Jt_23(28)*x(28) + Jt_23(29)*x(29) + w(23) == b3(23);
    Jt_24(1)*x(1) + Jt_24(2)*x(2) + Jt_24(3)*x(3) + Jt_24(4)*x(4) + Jt_24(5)*x(5) + Jt_24(6)*x(6) + Jt_24(24)*x(24) + Jt_24(25)*x(25) + Jt_24(26)*x(26) + Jt_24(27)*x(27) + Jt_24(28)*x(28) + Jt_24(29)*x(29) + w(24) == b3(24);
    Jt_25(1)*x(1) + Jt_25(2)*x(2) + Jt_25(3)*x(3) + Jt_25(4)*x(4) + Jt_25(5)*x(5) + Jt_25(6)*x(6) + Jt_25(30)*x(30) + Jt_25(31)*x(31) + Jt_25(32)*x(32) + Jt_25(33)*x(33) + Jt_25(34)*x(34) + Jt_25(35)*x(35) + w(25) == b3(25);
    Jt_26(1)*x(1) + Jt_26(2)*x(2) + Jt_26(3)*x(3) + Jt_26(4)*x(4) + Jt_26(5)*x(5) + Jt_26(6)*x(6) + Jt_26(30)*x(30) + Jt_26(31)*x(31) + Jt_26(32)*x(32) + Jt_26(33)*x(33) + Jt_26(34)*x(34) + Jt_26(35)*x(35) + w(26) == b3(26);
    Jt_27(1)*x(1) + Jt_27(2)*x(2) + Jt_27(3)*x(3) + Jt_27(4)*x(4) + Jt_27(5)*x(5) + Jt_27(6)*x(6) + Jt_27(30)*x(30) + Jt_27(31)*x(31) + Jt_27(32)*x(32) + Jt_27(33)*x(33) + Jt_27(34)*x(34) + Jt_27(35)*x(35) + w(27) == b3(27);
    Jt_28(1)*x(1) + Jt_28(2)*x(2) + Jt_28(3)*x(3) + Jt_28(4)*x(4) + Jt_28(5)*x(5) + Jt_28(6)*x(6) + Jt_28(30)*x(30) + Jt_28(31)*x(31) + Jt_28(32)*x(32) + Jt_28(33)*x(33) + Jt_28(34)*x(34) + Jt_28(35)*x(35) + w(28) == b3(28);
    Jt_29(1)*x(1) + Jt_29(2)*x(2) + Jt_29(3)*x(3) + Jt_29(4)*x(4) + Jt_29(5)*x(5) + Jt_29(6)*x(6) + Jt_29(30)*x(30) + Jt_29(31)*x(31) + Jt_29(32)*x(32) + Jt_29(33)*x(33) + Jt_29(34)*x(34) + Jt_29(35)*x(35) + w(29) == b3(29);
    Jt_30(1)*x(1) + Jt_30(2)*x(2) + Jt_30(3)*x(3) + Jt_30(4)*x(4) + Jt_30(5)*x(5) + Jt_30(6)*x(6) + Jt_30(30)*x(30) + Jt_30(31)*x(31) + Jt_30(32)*x(32) + Jt_30(33)*x(33) + Jt_30(34)*x(34) + Jt_30(35)*x(35) + w(30) == b3(30);
    x(7) + w(37) == b3(31);
    x(8) + w(38) == b3(32);
    x(9) + w(39) == b3(33);
    x(10) + w(40) == b3(34);
    x(11) + w(41) == b3(35);
    x(12) + w(42) == b3(36);
    x(13) + w(43) == b3(37);
    x(14) + w(44) == b3(38);
    x(15) + w(45) == b3(39);
    x(16) + w(46) == b3(40);
    x(17) + w(47) == b3(41);
    x(18) + w(48) == b3(42);
    x(19) + w(49) == b3(43);
    x(20) + w(50) == b3(44);
    x(21) + w(51) == b3(45);
    x(22) + w(52) == b3(46);
    x(23) + w(53) == b3(47);
    x(24) + w(54) == b3(48);
    x(25) + w(55) == b3(49);
    x(26) + w(56) == b3(50);
    x(27) + w(57) == b3(51);
    x(28) + w(58) == b3(52);
    x(29) + w(59) == b3(53);
    x(30) + w(60) == b3(54);
    x(31) + w(61) == b3(55);
    x(32) + w(62) == b3(56);
    x(33) + w(63) == b3(57);
    x(34) + w(64) == b3(58);
    x(35) + w(65) == b3(59);
    Jto_1(1)*x(1) + Jto_1(2)*x(2) + Jto_1(3)*x(3) + Jto_1(4)*x(4) + Jto_1(5)*x(5) + Jto_1(6)*x(6) + Jto_1(7)*x(7) + Jto_1(8)*x(8) + Jto_1(9)*x(9) + w(83) == b3(60);
    Jto_2(1)*x(1) + Jto_2(2)*x(2) + Jto_2(3)*x(3) + Jto_2(4)*x(4) + Jto_2(5)*x(5) + Jto_2(6)*x(6) + Jto_2(7)*x(7) + Jto_2(8)*x(8) + Jto_2(9)*x(9) + w(84) == b3(61);
    Jto_3(1)*x(1) + Jto_3(2)*x(2) + Jto_3(3)*x(3) + Jto_3(4)*x(4) + Jto_3(5)*x(5) + Jto_3(6)*x(6) + Jto_3(7)*x(7) + Jto_3(8)*x(8) + Jto_3(9)*x(9) + w(85) == b3(62);
    copMxl_1(1)*z(1) + copMxl_1(2)*z(2) + copMxl_1(3)*z(3) + copMxl_1(4)*z(4) + copMxl_1(5)*z(5) + copMxl_1(6)*z(6) + w(66) == 0;
    copMxl_2(1)*z(1) + copMxl_2(2)*z(2) + copMxl_2(3)*z(3) + copMxl_2(4)*z(4) + copMxl_2(5)*z(5) + copMxl_2(6)*z(6) + w(67) == 0;
    copMxr_1(1)*z(7) + copMxr_1(2)*z(8) + copMxr_1(3)*z(9) + copMxr_1(4)*z(10) + copMxr_1(5)*z(11) + copMxr_1(6)*z(12) + w(68) == 0;
    copMxr_2(1)*z(7) + copMxr_2(2)*z(8) + copMxr_2(3)*z(9) + copMxr_2(4)*z(10) + copMxr_2(5)*z(11) + copMxr_2(6)*z(12) + w(69) == 0;
    aw*dwd*z(3) - aw*(1 - dwd)*z(9) + w(69) == 0;
    dt*Jt_19(1)*x(1) + dt*Jt_19(2)*x(2) + dt*Jt_19(3)*x(3) + dt*Jt_19(4)*x(4) + dt*Jt_19(5)*x(5) + dt*Jt_19(6)*x(6) + dt*Jt_19(24)*x(24) + dt*Jt_19(25)*x(25) + dt*Jt_19(26)*x(26) + dt*Jt_19(27)*x(27) + dt*Jt_19(28)*x(28) + dt*Jt_19(29)*x(29) + w(71) == b4(1);
    dt*Jt_20(1)*x(1) + dt*Jt_20(2)*x(2) + dt*Jt_20(3)*x(3) + dt*Jt_20(4)*x(4) + dt*Jt_20(5)*x(5) + dt*Jt_20(6)*x(6) + dt*Jt_20(24)*x(24) + dt*Jt_20(25)*x(25) + dt*Jt_20(26)*x(26) + dt*Jt_20(27)*x(27) + dt*Jt_20(28)*x(28) + dt*Jt_20(29)*x(29) + w(72) == b4(2);
    dt*Jt_21(1)*x(1) + dt*Jt_21(2)*x(2) + dt*Jt_21(3)*x(3) + dt*Jt_21(4)*x(4) + dt*Jt_21(5)*x(5) + dt*Jt_21(6)*x(6) + dt*Jt_21(24)*x(24) + dt*Jt_21(25)*x(25) + dt*Jt_21(26)*x(26) + dt*Jt_21(27)*x(27) + dt*Jt_21(28)*x(28) + dt*Jt_21(29)*x(29) + w(73) == b4(3);
    dt*Jt_22(1)*x(1) + dt*Jt_22(2)*x(2) + dt*Jt_22(3)*x(3) + dt*Jt_22(4)*x(4) + dt*Jt_22(5)*x(5) + dt*Jt_22(6)*x(6) + dt*Jt_22(24)*x(24) + dt*Jt_22(25)*x(25) + dt*Jt_22(26)*x(26) + dt*Jt_22(27)*x(27) + dt*Jt_22(28)*x(28) + dt*Jt_22(29)*x(29) + w(74) == b4(4);
    dt*Jt_23(1)*x(1) + dt*Jt_23(2)*x(2) + dt*Jt_23(3)*x(3) + dt*Jt_23(4)*x(4) + dt*Jt_23(5)*x(5) + dt*Jt_23(6)*x(6) + dt*Jt_23(24)*x(24) + dt*Jt_23(25)*x(25) + dt*Jt_23(26)*x(26) + dt*Jt_23(27)*x(27) + dt*Jt_23(28)*x(28) + dt*Jt_23(29)*x(29) + w(75) == b4(5);
    dt*Jt_24(1)*x(1) + dt*Jt_24(2)*x(2) + dt*Jt_24(3)*x(3) + dt*Jt_24(4)*x(4) + dt*Jt_24(5)*x(5) + dt*Jt_24(6)*x(6) + dt*Jt_24(24)*x(24) + dt*Jt_24(25)*x(25) + dt*Jt_24(26)*x(26) + dt*Jt_24(27)*x(27) + dt*Jt_24(28)*x(28) + dt*Jt_24(29)*x(29) + w(76) == b4(6);
    dt*Jt_25(1)*x(1) + dt*Jt_25(2)*x(2) + dt*Jt_25(3)*x(3) + dt*Jt_25(4)*x(4) + dt*Jt_25(5)*x(5) + dt*Jt_25(6)*x(6) + dt*Jt_25(30)*x(30) + dt*Jt_25(31)*x(31) + dt*Jt_25(32)*x(32) + dt*Jt_25(33)*x(33) + dt*Jt_25(34)*x(34) + dt*Jt_25(35)*x(35) + w(77) == b4(7);
    dt*Jt_26(1)*x(1) + dt*Jt_26(2)*x(2) + dt*Jt_26(3)*x(3) + dt*Jt_26(4)*x(4) + dt*Jt_26(5)*x(5) + dt*Jt_26(6)*x(6) + dt*Jt_26(30)*x(30) + dt*Jt_26(31)*x(31) + dt*Jt_26(32)*x(32) + dt*Jt_26(33)*x(33) + dt*Jt_26(34)*x(34) + dt*Jt_26(35)*x(35) + w(78) == b4(8);
    dt*Jt_27(1)*x(1) + dt*Jt_27(2)*x(2) + dt*Jt_27(3)*x(3) + dt*Jt_27(4)*x(4) + dt*Jt_27(5)*x(5) + dt*Jt_27(6)*x(6) + dt*Jt_27(30)*x(30) + dt*Jt_27(31)*x(31) + dt*Jt_27(32)*x(32) + dt*Jt_27(33)*x(33) + dt*Jt_27(34)*x(34) + dt*Jt_27(35)*x(35) + w(79) == b4(9);
    dt*Jt_28(1)*x(1) + dt*Jt_28(2)*x(2) + dt*Jt_28(3)*x(3) + dt*Jt_28(4)*x(4) + dt*Jt_28(5)*x(5) + dt*Jt_28(6)*x(6) + dt*Jt_28(30)*x(30) + dt*Jt_28(31)*x(31) + dt*Jt_28(32)*x(32) + dt*Jt_28(33)*x(33) + dt*Jt_28(34)*x(34) + dt*Jt_28(35)*x(35) + w(80) == b4(10);
    dt*Jt_29(1)*x(1) + dt*Jt_29(2)*x(2) + dt*Jt_29(3)*x(3) + dt*Jt_29(4)*x(4) + dt*Jt_29(5)*x(5) + dt*Jt_29(6)*x(6) + dt*Jt_29(30)*x(30) + dt*Jt_29(31)*x(31) + dt*Jt_29(32)*x(32) + dt*Jt_29(33)*x(33) + dt*Jt_29(34)*x(34) + dt*Jt_29(35)*x(35) + w(81) == b4(11);
    dt*Jt_30(1)*x(1) + dt*Jt_30(2)*x(2) + dt*Jt_30(3)*x(3) + dt*Jt_30(4)*x(4) + dt*Jt_30(5)*x(5) + dt*Jt_30(6)*x(6) + dt*Jt_30(30)*x(30) + dt*Jt_30(31)*x(31) + dt*Jt_30(32)*x(32) + dt*Jt_30(33)*x(33) + dt*Jt_30(34)*x(34) + dt*Jt_30(35)*x(35) + w(82) == b4(12);
    -T_max <= y;
    y <= T_max;
    -v_max(1) <= dt*x(7);
    -v_max(2) <= dt*x(8);
    -v_max(3) <= dt*x(9);
    -v_max(4) <= dt*x(10);
    -v_max(5) <= dt*x(11);
    -v_max(6) <= dt*x(12);
    -v_max(7) <= dt*x(13);
    -v_max(8) <= dt*x(14);
    -v_max(9) <= dt*x(15);
    -v_max(10) <= dt*x(16);
    -v_max(11) <= dt*x(17);
    -v_max(12) <= dt*x(18);
    -v_max(13) <= dt*x(19);
    -v_max(14) <= dt*x(20);
    -v_max(15) <= dt*x(21);
    -v_max(16) <= dt*x(22);
    -v_max(17) <= dt*x(23);
    -v_max(18) <= dt*x(24);
    -v_max(19) <= dt*x(25);
    -v_max(20) <= dt*x(26);
    -v_max(21) <= dt*x(27);
    -v_max(22) <= dt*x(28);
    -v_max(23) <= dt*x(29);
    -v_max(24) <= dt*x(30);
    -v_max(25) <= dt*x(31);
    -v_max(26) <= dt*x(32);
    -v_max(27) <= dt*x(33);
    -v_max(28) <= dt*x(34);
    -v_max(29) <= dt*x(35);
    dt*x(7) <= v_max(1);
    dt*x(8) <= v_max(2);
    dt*x(9) <= v_max(3);
    dt*x(10) <= v_max(4);
    dt*x(11) <= v_max(5);
    dt*x(12) <= v_max(6);
    dt*x(13) <= v_max(7);
    dt*x(14) <= v_max(8);
    dt*x(15) <= v_max(9);
    dt*x(16) <= v_max(10);
    dt*x(17) <= v_max(11);
    dt*x(18) <= v_max(12);
    dt*x(19) <= v_max(13);
    dt*x(20) <= v_max(14);
    dt*x(21) <= v_max(15);
    dt*x(22) <= v_max(16);
    dt*x(23) <= v_max(17);
    dt*x(24) <= v_max(18);
    dt*x(25) <= v_max(19);
    dt*x(26) <= v_max(20);
    dt*x(27) <= v_max(21);
    dt*x(28) <= v_max(22);
    dt*x(29) <= v_max(23);
    dt*x(30) <= v_max(24);
    dt*x(31) <= v_max(25);
    dt*x(32) <= v_max(26);
    dt*x(33) <= v_max(27);
    dt*x(34) <= v_max(28);
    dt*x(35) <= v_max(29);
    q_min(1) <= (1/2)*dt*dt*x(7);
    q_min(2) <= (1/2)*dt*dt*x(8);
    q_min(3) <= (1/2)*dt*dt*x(9);
    q_min(4) <= (1/2)*dt*dt*x(10);
    q_min(5) <= (1/2)*dt*dt*x(11);
    q_min(6) <= (1/2)*dt*dt*x(12);
    q_min(7) <= (1/2)*dt*dt*x(13);
    q_min(8) <= (1/2)*dt*dt*x(14);
    q_min(9) <= (1/2)*dt*dt*x(15);
    q_min(10) <= (1/2)*dt*dt*x(16);
    q_min(11) <= (1/2)*dt*dt*x(17);
    q_min(12) <= (1/2)*dt*dt*x(18);
    q_min(13) <= (1/2)*dt*dt*x(19);
    q_min(14) <= (1/2)*dt*dt*x(20);
    q_min(15) <= (1/2)*dt*dt*x(21);
    q_min(16) <= (1/2)*dt*dt*x(22);
    q_min(17) <= (1/2)*dt*dt*x(23);
    q_min(18) <= (1/2)*dt*dt*x(24);
    q_min(19) <= (1/2)*dt*dt*x(25);
    q_min(20) <= (1/2)*dt*dt*x(26);
    q_min(21) <= (1/2)*dt*dt*x(27);
    q_min(22) <= (1/2)*dt*dt*x(28);
    q_min(23) <= (1/2)*dt*dt*x(29);
    q_min(24) <= (1/2)*dt*dt*x(30);
    q_min(25) <= (1/2)*dt*dt*x(31);
    q_min(26) <= (1/2)*dt*dt*x(32);
    q_min(27) <= (1/2)*dt*dt*x(33);
    q_min(28) <= (1/2)*dt*dt*x(34);
    q_min(29) <= (1/2)*dt*dt*x(35);
    (1/2)*dt*dt*x(7) <= q_max(1);
    (1/2)*dt*dt*x(8) <= q_max(2);
    (1/2)*dt*dt*x(9) <= q_max(3);
    (1/2)*dt*dt*x(10) <= q_max(4);
    (1/2)*dt*dt*x(11) <= q_max(5);
    (1/2)*dt*dt*x(12) <= q_max(6);
    (1/2)*dt*dt*x(13) <= q_max(7);
    (1/2)*dt*dt*x(14) <= q_max(8);
    (1/2)*dt*dt*x(15) <= q_max(9);
    (1/2)*dt*dt*x(16) <= q_max(10);
    (1/2)*dt*dt*x(17) <= q_max(11);
    (1/2)*dt*dt*x(18) <= q_max(12);
    (1/2)*dt*dt*x(19) <= q_max(13);
    (1/2)*dt*dt*x(20) <= q_max(14);
    (1/2)*dt*dt*x(21) <= q_max(15);
    (1/2)*dt*dt*x(22) <= q_max(16);
    (1/2)*dt*dt*x(23) <= q_max(17);
    (1/2)*dt*dt*x(24) <= q_max(18);
    (1/2)*dt*dt*x(25) <= q_max(19);
    (1/2)*dt*dt*x(26) <= q_max(20);
    (1/2)*dt*dt*x(27) <= q_max(21);
    (1/2)*dt*dt*x(28) <= q_max(22);
    (1/2)*dt*dt*x(29) <= q_max(23);
    (1/2)*dt*dt*x(30) <= q_max(24);
    (1/2)*dt*dt*x(31) <= q_max(25);
    (1/2)*dt*dt*x(32) <= q_max(26);
    (1/2)*dt*dt*x(33) <= q_max(27);
    (1/2)*dt*dt*x(34) <= q_max(28);
    (1/2)*dt*dt*x(35) <= q_max(29);
    CL_1(1)*z(1) + CL_1(2)*z(2) + CL_1(3)*z(3) + CL_1(4)*z(4) + CL_1(5)*z(5) + CL_1(6)*z(6) <= 0;
    CL_2(1)*z(1) + CL_2(2)*z(2) + CL_2(3)*z(3) + CL_2(4)*z(4) + CL_2(5)*z(5) + CL_2(6)*z(6) <= 0;
    CL_3(1)*z(1) + CL_3(2)*z(2) + CL_3(3)*z(3) + CL_3(4)*z(4) + CL_3(5)*z(5) + CL_3(6)*z(6) <= 0;
    CL_4(1)*z(1) + CL_4(2)*z(2) + CL_4(3)*z(3) + CL_4(4)*z(4) + CL_4(5)*z(5) + CL_4(6)*z(6) <= 0;
    CL_5(1)*z(1) + CL_5(2)*z(2) + CL_5(3)*z(3) + CL_5(4)*z(4) + CL_5(5)*z(5) + CL_5(6)*z(6) <= 0;
    CL_6(1)*z(1) + CL_6(2)*z(2) + CL_6(3)*z(3) + CL_6(4)*z(4) + CL_6(5)*z(5) + CL_6(6)*z(6) <= 0;
    CL_7(1)*z(1) + CL_7(2)*z(2) + CL_7(3)*z(3) + CL_7(4)*z(4) + CL_7(5)*z(5) + CL_7(6)*z(6) <= 0;
    CL_8(1)*z(1) + CL_8(2)*z(2) + CL_8(3)*z(3) + CL_8(4)*z(4) + CL_8(5)*z(5) + CL_8(6)*z(6) <= 0;
    CL_9(1)*z(1) + CL_9(2)*z(2) + CL_9(3)*z(3) + CL_9(4)*z(4) + CL_9(5)*z(5) + CL_9(6)*z(6) <= 0;
    CL_10(1)*z(1) + CL_10(2)*z(2) + CL_10(3)*z(3) + CL_10(4)*z(4) + CL_10(5)*z(5) + CL_10(6)*z(6) <= 0;
    CL_11(1)*z(1) + CL_11(2)*z(2) + CL_11(3)*z(3) + CL_11(4)*z(4) + CL_11(5)*z(5) + CL_11(6)*z(6) <= 0;
    CR_1(1)*z(7) + CR_1(2)*z(8) + CR_1(3)*z(9) + CR_1(4)*z(10) + CR_1(5)*z(11) + CR_1(6)*z(12) <= 0;
    CR_2(1)*z(7) + CR_2(2)*z(8) + CR_2(3)*z(9) + CR_2(4)*z(10) + CR_2(5)*z(11) + CR_2(6)*z(12) <= 0;
    CR_3(1)*z(7) + CR_3(2)*z(8) + CR_3(3)*z(9) + CR_3(4)*z(10) + CR_3(5)*z(11) + CR_3(6)*z(12) <= 0;
    CR_4(1)*z(7) + CR_4(2)*z(8) + CR_4(3)*z(9) + CR_4(4)*z(10) + CR_4(5)*z(11) + CR_4(6)*z(12) <= 0;
    CR_5(1)*z(7) + CR_5(2)*z(8) + CR_5(3)*z(9) + CR_5(4)*z(10) + CR_5(5)*z(11) + CR_5(6)*z(12) <= 0;
    CR_6(1)*z(7) + CR_6(2)*z(8) + CR_6(3)*z(9) + CR_6(4)*z(10) + CR_6(5)*z(11) + CR_6(6)*z(12) <= 0;
    CR_7(1)*z(7) + CR_7(2)*z(8) + CR_7(3)*z(9) + CR_7(4)*z(10) + CR_7(5)*z(11) + CR_7(6)*z(12) <= 0;
    CR_8(1)*z(7) + CR_8(2)*z(8) + CR_8(3)*z(9) + CR_8(4)*z(10) + CR_8(5)*z(11) + CR_8(6)*z(12) <= 0;
    CR_9(1)*z(7) + CR_9(2)*z(8) + CR_9(3)*z(9) + CR_9(4)*z(10) + CR_9(5)*z(11) + CR_9(6)*z(12) <= 0;
    CR_10(1)*z(7) + CR_10(2)*z(8) + CR_10(3)*z(9) + CR_10(4)*z(10) + CR_10(5)*z(11) + CR_10(6)*z(12) <= 0;
    CR_11(1)*z(7) + CR_11(2)*z(8) + CR_11(3)*z(9) + CR_11(4)*z(10) + CR_11(5)*z(11) + CR_11(6)*z(12) <= 0;
cvx_end
vars.w = w;
vars.x = x;
vars.y = y;
vars.z = z;
status.cvx_status = cvx_status;
% Provide a drop-in replacement for csolve.
status.optval = cvx_optval;
status.converged = strcmp(cvx_status, 'Solved');
