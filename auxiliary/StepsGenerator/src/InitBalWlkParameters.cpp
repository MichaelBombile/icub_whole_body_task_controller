// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include "InitBalWlkParameters.h"

InitBalWlkParameters::InitBalWlkParameters(int period, const std::string robotName, int ForceFeedback)
{
  //
  RobotName     = robotName;
  //
  CoMHeight     = 0.47; //0.57;
  TotalMass     = 25; // [23 - 25] [kg] 

  // Type of inverted Pendulum Dynamic model
  CpModelOption  = 0;           // 0 : for states X(k)=[com cp]                Cp = [0 1]* X(k)
                                // 1 : for states X(k)=[com com_dot]           Cp = [1 1/W] * X(k) 
                                // 2 : for states X(k)=[com com_dot com_ddot]  Cp = [1 1/W 0] * X(k)
  //
    OrientOffest = 0.0;
  if(RobotName == "icub")
    OrientOffest = -0.1;

  if (period == 100)
  {
      SamplingTime  = 0.10;
      
      nsp           = 16;
      nsp1          = 8;
      nsp2          = 8;

      DurationSteps[0] = 1.600;
      DurationSteps[1] = 0.800;
      DurationSteps[2] = 0.800;
      DurationSteps[3] = 0.800;
      DurationSteps[4] = 0.00;

      // gains of the QP [alpha, beta, gamma, kappa]
      gains[0] = 0.50; //0.3e-2; //0.06
      gains[1] = 0.50;
      gains[2] = 1.50;
      gains[3] = 0.80;
      gains[8] = 0.0;

      Accel_factor = 50.0;
      Speed_factor = 20.5;      
  }
  else if(period == 50)
  {
      SamplingTime  = 0.050;
      CoMHeight     = 0.47; //0.57;  

      nsp           = 16;
      nsp1          = 16;
      nsp2          = 8;

      DurationSteps[0] = 0.800;
      DurationSteps[1] = 0.800;
      DurationSteps[2] = 0.800;
      DurationSteps[3] = 0.800;
      DurationSteps[4] = 0.200;

      // gains of the QP [alpha, beta, gamma, kappa]
      // gains of the QP [alpha, beta, gamma, kappa]
      // gains[0] = 0.20; //0.3e-2; //0.06
      // gains[1] = 0.50;
      // gains[2] = 1.50;
      // gains[3] = 0.80;
      // gains[8] = 0.00;   // for planning

      gains[0] = 0.20; //0.3e-2; //0.06
      gains[1] = 0.005;
      gains[2] = 0.05;
      gains[3] = 0.00;
      gains[8] = 1.50;   // for planning


      Accel_factor = 50.0;
      Speed_factor = 35.5; // 27.5
  }
  else if(period == 45)
  {
      SamplingTime  = 0.045;
      CoMHeight     = 0.47; //0.57;  

      nsp           = 16;
      nsp1          = 16;
      nsp2          = 8;

      DurationSteps[0] = 0.720;
      DurationSteps[1] = 0.720;
      DurationSteps[2] = 0.720;
      DurationSteps[3] = 0.720;
      DurationSteps[4] = 0.000;
      // gains of the QP [alpha, beta, gamma, kappa] : []
      // gains[0] = 0.20; //0.3e-2; //0.06
      // gains[1] = 0.50;
      // gains[2] = 1.50;
      // gains[3] = 0.80;
      // gains[8] = 0.0;   // for planning

      // gains[0] = 0.20; //0.3e-2; //0.06
      // gains[1] = 0.005;
      // gains[2] = 0.05;
      // gains[3] = 0.00;
      // gains[8] = 1.50;   // for planning

      gains[0] = 0.20; //0.3e-2; //0.06
      gains[1] = 0.0000000001;
      gains[2] = 1.50; // 1.50
      gains[3] = 0.0000000001;
      gains[8] = 5.50;   // for planning 3.5

      Accel_factor = 50.0;
      Speed_factor = 35.5; // 27.5
  }
  else
  {
      SamplingTime  = 0.040;
      CoMHeight     = 0.47; //0.47 0.57; 

      nsp           = 16;
      nsp1          = 16;
      nsp2          = 8;

      // DurationSteps[0] = 0.720;
      // DurationSteps[1] = 0.720;
      // DurationSteps[2] = 0.720;
      // DurationSteps[3] = 0.720;
      // DurationSteps[4] = 0.00;

      DurationSteps[0] = 0.640;
      DurationSteps[1] = 0.640;
      DurationSteps[2] = 0.640;
      DurationSteps[3] = 0.640;
      DurationSteps[4] = 0.160;  // <--------------

      // gains of the QP [alpha, beta, gamma, kappa]
      // gains[0] = 0.20; //0.3e-2; //0.06
      // gains[1] = 0.50;
      // gains[2] = 1.50;
      // gains[3] = 0.80;
      // gains[8] = 0.0;   // for planning

      gains[0] = 0.20; //0.3e-2; //0.06
      gains[1] = 0.0000000001;
      gains[2] = 1.50; //0.0001;
      gains[3] = 0.0000000001;
      gains[8] = 3.50;   // for planning

      Accel_factor = 50.0;
      Speed_factor = 27.5; // 27.5
  }

  if (CpModelOption == 2)
  {
      // gains of the QP [alpha, beta, gamma, kappa]

      if (SamplingTime == 0.050)
      {
          gains[0] = 0.4e-6;  // 0.1e-5
          gains[1] = 0.020;
          gains[2] = 4.00;
          gains[3] = 1.0;
      }
      else
      {
          gains[0] = 0.3e-5;  //0.3e-2; //0.1e-4
          gains[1] = 0.020;
          gains[2] = 4.50;
          gains[3] = 1.0;
      }
  }

  // gains for rotation QP [alpha_R, beta_R, gamma_R, kappa_R]
  // gains[4] = 0.1e-5;  //0.3e-2;
  // gains[5] = 1.0;
  // gains[6] = 1.00;
  // gains[7] = 0.10;
  // gains[9] = 0.00;

  gains[4] = 0.1e-5;  //0.3e-2;
  gains[5] = 0.0000000001;     // 1.00
  gains[6] = 1.00;   // 1.00
  gains[7] = 0.0000000001;
  gains[9] = 5.00;


//  int n12, n13;
//      n12 = (int)round(nsp/nsp1);
//      n13 = (int)round(nsp/nsp2);
//  DurationSteps[0] = nsp  * SamplingTime;
//  DurationSteps[1] = nsp1 * n12 * SamplingTime;
//  DurationSteps[2] = nsp2 * n13 * SamplingTime;
//  DurationSteps[3] = nsp2 * n13 * SamplingTime;
//  DurationSteps[4] = 0.00;

  // nb of sample for each step
  SamplesPerStep[0] = nsp;
  SamplesPerStep[1] = nsp1;
  SamplesPerStep[2] = nsp2;
  SamplesPerStep[3] = nsp2;

  // Control horizon
  CtrlHorizon =   SamplesPerStep[0] + SamplesPerStep[1]
                + SamplesPerStep[2] + SamplesPerStep[3];

   // Inverted pendulum natural frequency
  OmegaZero  = sqrt(Gravity/CoMHeight);

  // array of stance foot state {Left, Right}
  StanceIndicator[0] = 0;
  StanceIndicator[1] = 1;

  // constraint on maximum height of swing foot
  maxFootHeight      = 0.03; //0.028; // 0.3  // ////////////////////////////////////////////////

  // vector of Foosteps constraints
  SupportConstraints.resize(5);
  SupportConstraints(0) = 0.20;   // longitudinal forward    0.15  20
  SupportConstraints(1) = 0.20;   // longitudinal backward   0.15
  SupportConstraints(2) = 0.155;   // lateral inward   //0.7 * 2    // .145 //13.5 0.153  0.155;
  SupportConstraints(3) = 0.25;   // lateral outward  0.20
  SupportConstraints(4) = 0.55;   // rotation

  // translation vector from Base to the CoM
  Translation_B_CoM.resize(3);
  Translation_B_CoM.setZero(3);
  Translation_B_CoM(0) = 0.001;
  Translation_B_CoM(1) = 0.0;
  Translation_B_CoM(2) = 0.10;  // 0.1

  // Initial velocity of the frame attached  to the CoM
  InitialVelocity.resize(3);
  InitialVelocity.setZero(3);


  // Set initial stance foot positon and orientation
  InitCoPrefPose.resize(5);
  InitCoPrefPose.setZero(5);
  InitCoPrefPose(4)   = -SupportConstraints(2)/2.;   // true value 0.0681

  int n12, n13;
      n12 = (int)round(nsp/nsp1);
      n13 = (int)round(nsp/nsp2);

  // Vector of samples
  SamplesVector.resize(CtrlHorizon);
  for (int i=0; i<SamplesPerStep[0]; i++)
    {
        SamplesVector(i) = SamplingTime;
    }
  for (int i=0; i<SamplesPerStep[1]; i++)
    {
        SamplesVector(SamplesPerStep[0]+i) = DurationSteps[1]/nsp1;
    }
  for (int i=0; i<SamplesPerStep[2]; i++)
    {
        SamplesVector( SamplesPerStep[0]
                      +SamplesPerStep[1] + i) = DurationSteps[2]/nsp2;
        SamplesVector( SamplesPerStep[0]
                      +SamplesPerStep[1]
                      +SamplesPerStep[2] + i) = DurationSteps[2]/nsp2;
    }

  // Vectror of positions of support foot edge
  SupportFootEdgePositions.resize(4);
  SupportFootEdgePositions(0) = 0.07;
  SupportFootEdgePositions(1) = 0.02; // 0.3
  SupportFootEdgePositions(2) = 0.07;
  SupportFootEdgePositions(3) = 0.02; // 0.3

  // Matrix containing the normal vectors to the support foot edges
  SupportFootEdgeNormals.resize(2,4);
  SupportFootEdgeNormals.setZero(2,4);

  SupportFootEdgeNormals(0,0)  =  1.0;
  SupportFootEdgeNormals(1,1)  =  1.0;
  SupportFootEdgeNormals(0,2)  = -1.0;
  SupportFootEdgeNormals(1,3)  = -1.0;

  // arms admittance parameters
  // closed loop frequency
  double W_arm = 1.0;
  // 
  double damping_coeff_arm = 3.0;   // critically damped
  double k_arm_linear      = 0.5;
  double k_arm_angular     = 0.5;

  Eigen::VectorXd one_3(3);
  one_3.setOnes(3);


  // Properties of the admittance law 
  // stiffness
  lh_virtualStiffness.resize(6);
  lh_virtualStiffness.segment(0, 3) = k_arm_linear * one_3;   
  lh_virtualStiffness.segment(3, 3) = k_arm_angular * one_3;

  // inertia
  lh_virtualInertia.resize(6);
  lh_virtualInertia.segment(0, 3) = 1.0/(W_arm*W_arm) *k_arm_linear * one_3; 
  lh_virtualInertia.segment(3, 3) = 1.0/(W_arm*W_arm) *k_arm_angular * one_3; 

  // Damping
  lh_virtualDamping.resize(6);
  lh_virtualDamping.segment(0, 3) = 2.0* damping_coeff_arm* k_arm_linear/W_arm * one_3; 
  lh_virtualDamping.segment(3, 3) = 2.0* damping_coeff_arm* k_arm_angular/W_arm * one_3;


  // stiffness
  rh_virtualStiffness.resize(6);
  rh_virtualStiffness.segment(0, 3) = k_arm_linear * one_3;   
  rh_virtualStiffness.segment(3, 3) = k_arm_angular * one_3;

  // inertia
  rh_virtualInertia.resize(6);
  rh_virtualInertia.segment(0, 3) = 1.0/(W_arm*W_arm) *k_arm_linear * one_3; 
  rh_virtualInertia.segment(3, 3) = 1.0/(W_arm*W_arm) *k_arm_angular * one_3; 

  // Damping
  rh_virtualDamping.resize(6);
  rh_virtualDamping.segment(0, 3) = 2.0* damping_coeff_arm* k_arm_linear/W_arm * one_3; 
  rh_virtualDamping.segment(3, 3) = 2.0* damping_coeff_arm* k_arm_angular/W_arm * one_3;

  
  // // left arm
  // lh_virtualInertia.resize(6);
  // lh_virtualInertia << 1.0,  1.0,  1.0,    // position
  //                      1.0,   1.0,  1.0;  // orientation 

  // // Damping
  // lh_virtualDamping.resize(6);
  // lh_virtualDamping << 1.0,  1.0,  1.0,    // position
  //                        1.0,   1.0,  1.0;  // orientation 

  // // stiffness
  // lh_virtualStiffness.resize(6);
  // lh_virtualStiffness << 1.0,  1.0,  1.0,    // position
  //                       1.0,   1.0,  1.0;   // orientation 
  // // right arm
  // rh_virtualInertia.resize(6);
  // rh_virtualInertia << 1.0,  1.0,  1.0,    // position
  //                     1.0,   1.0,  1.0;  // orientation 

  // // Damping
  // rh_virtualDamping.resize(6);
  // rh_virtualDamping << 1.0,  1.0,  1.0,    // position
  //                      1.0,   1.0,  1.0;  // orientation   

  // // stiffness
  // rh_virtualStiffness.resize(6);
  // rh_virtualStiffness << 1.0,  1.0,  1.0,    // position
  //                        1.0,   1.0,  1.0;  // orientation 

 
  // footsteps constraints only or footsteps and ZMP constraints
  FootStepsCnstrOnly = false;

  // compensation of the Base orientation using the  IMU
  IMU_reference      = false;

  // number of sampling instant within a step
  num_samples = (int) (round(DurationSteps[0]/SamplingTime));

  // force threshold for support phase
  Threshold_SSP = 40.;

  // Sending Commands to the robot
  Command_exe = true;
  // sending grasping command
  ExecuteGrasping  = false;

  // sending grasping command
  ExecuteWalking  = true;

  //Switching Synchronization with FT sensor
  SwitchingSync = false;

  // variable for force torque feedback
  FT_feedback = ForceFeedback; //2;  //0: no admittance feedback, 1: feet; 2: arms 

  // calibration
  calibration = true;

  // Parameters for the commands writer
  useOffset = true;
  SmoothCmd = true;
  NbInterpolationPts = 5;

  // additional parameters for the arms motion
  VelocityIK      = true;
  Delta_theta_max = 0.3; //0.3

  // gain for the servoing of the hands
  Gain_hand.resize(6);
  Gain_hand.setIdentity(6,1);
  Gain_hand *= 0.55;

  // Threshold for the velocities in the state to input compensator
  VeloFeedbackThreshold << 0.03,  0.005,
                           0.015, 0.01,
                           0.02,  0.02;

  // initial position of the CoM
  init_com_position = Vector3d(0.0, 0.0, CoMHeight);


  // weighting factors from simulator to real robot
   //
   if ((RobotName == "icub")&& calibration)
   {
        HipRollFactor   = 1.0; // 1.45 1.355
        AnkleRollFactor = 1.0; // 1.32 1.30
        HipYawFactor    = 1.00;  // 1.3

   }
   else
   {
        HipRollFactor   = 1.0; // 1.0
        AnkleRollFactor = 1.0; // 1.5
        HipYawFactor    = 1.0; 
   }

}

InitBalWlkParameters::~InitBalWlkParameters() {}