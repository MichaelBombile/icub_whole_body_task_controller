
/* 
 * Copyright (C) 2017 Learning Algorithms and Systems Laboratory (LASA)
 * Author: Michael Bombile
 * email:  michael.bombile@epfl.ch
 * website: lasa.epfl.ch
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/


// PatternsGenerator
//  10.5. CP_SelectionMatrices
//  10.2. CpStanceFootPose
//  10.6. VelocitiesSetPoints
//  10.1. CpReactiveWalkingController
//  10.3. CpFootTrajectories
//  10.4. CpGaitTransformations


#include <string>
#include <iostream>
#include <fstream>

#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>

#include "PatternsGenerator.h"

//using namespace std;
//using namespace Eigen;

// ====================================================================================================================
// ====================================================================================================================
CpReactiveWalkingController::CpReactiveWalkingController(){} 

CpReactiveWalkingController::~CpReactiveWalkingController()
{
    CpReactiveWalkingController::ReleaseCpBalWlkController();
}


void CpReactiveWalkingController::ReleaseCpBalWlkController()
{
    
}


void CpReactiveWalkingController::InitializeCpBalWlkController(InitBalWlkParameters *Parameters)
{
  // initialization of the counter
    SwitchCounter = 0;

    CtrlCounter    = 0;
    WaitForContact = false;
    resetStep   = false;
    // initialisation of optimal Footsteps solution
    RelCoPXYR.resize(11);
    RelCoPXYR.setZero(11);
    RelCoPXYR_n1.resize(3);
    RelCoPXYR_n1.setZero(3);
    // Optimal control signal at t-1
    OptimSolXY_n1.resize(2);
    OptimSolXY_n1.setZero(2);
    // Optimal control signal at t
    RelCoPXY.resize(6);
    RelCoPXY.setZero(6);
    OrienCoP.resize(3);
    OrienCoP.setZero(3);

    step_size_x = 0.0;
    step_size_y = 0.0;
    step_theta  = 0.0;
    dx_com_normalized = 0.0;
    //
    OrientOffest = Parameters->OrientOffest;

    // LIPM object
    
    Parameters->CoMHeight = Parameters->init_com_position(2);
    DMod.Init(Parameters->SamplingTime, Parameters->CoMHeight, Parameters->CpModelOption);
    DMod.Create_CP_LIP_Model(Parameters->SamplingTime, Parameters->CoMHeight);
    // MPC Object
    // MpcModel = new MpcBased_CP_Model;
    MpcModel.CreateSampMpc_CP_Model( DMod, Parameters->SamplesVector, Parameters->SamplesPerStep, Parameters->DurationSteps, Parameters->CpModelOption);
    MpcModel.getEmx();
    MpcModel.prefactorization();
    // Cyclic selection matrices
    SMx.Initialize( Parameters->SamplesVector, Parameters->SamplingTime, Parameters->SamplesPerStep, Parameters->DurationSteps, Parameters->CoMHeight);
    // Center of Pressure (CoP) refernce Trajectory
    CoPref.getCpStanceFootPose(Parameters->InitCoPrefPose);
    // Compute Reference velocity in inertial frame
    // VeloRef = new VelocitiesSetPoints;
    VeloRef.WorldVelocitiesSetPts(SMx, Parameters->InitialVelocity);
    // Hessian Matrix
    QMx.Init(MpcModel,Parameters->gains, Parameters->CpModelOption);
    MatrixXd MQKxy = QMx.getQMxy(MpcModel, SMx);
    MatrixXd MQang = QMx.getQMang(MpcModel, SMx);
    // Gradient vector
    PVec.Init(MpcModel, Parameters->gains, Parameters->CpModelOption);
    VectorXd VPKxy = PVec.getVeckxy( MpcModel, DMod, CoPref, SMx, VeloRef, OptimSolXY_n1);
    VectorXd VPang = PVec.getVeckAng(MpcModel, DMod, CoPref, SMx, VeloRef);
    
    // // constraints of the footsteps placement
    CnstrFtStp.Init(CoPref, Parameters->StanceIndicator, Parameters->CtrlHorizon, Parameters->SupportConstraints);
    CnstrFtStp.UpdateFtstpConstraints(CoPref, Parameters->StanceIndicator);
    // constarints on the center of Pressure (ZmP)
    CnstrZmp.Init(MpcModel, SMx, DMod, CoPref, Parameters->CpModelOption);
    CnstrZmp.FindReducedZmpConstraints12(MpcModel, SMx, DMod, CoPref);

    // matrix of contraints on the CoP and the ZMP
    if (Parameters->FootStepsCnstrOnly)
    {
        // constraints matrix
        MconsXY.resize((CnstrFtStp.Mxy).rows(), (CnstrFtStp.Mxy).cols() + 2);
        MconsXY.setZero((CnstrFtStp.Mxy).rows(), (CnstrFtStp.Mxy).cols() + 2);
        MconsXY.block(0, 0, (CnstrFtStp.Mxy).rows(), (CnstrFtStp.Mxy).cols()) = CnstrFtStp.Mxy;
        // constrainst vector
        BconsXY = CnstrFtStp.Bxy;
    }
    else
    {
        // constraints matrix
        MconsXY.resize((CnstrZmp.MuAct).rows()+(CnstrFtStp.Mxy).rows(), (CnstrFtStp.Mxy).cols() + 2);
        MconsXY.setZero((CnstrZmp.MuAct).rows()+(CnstrFtStp.Mxy).rows(), (CnstrFtStp.Mxy).cols() + 2);
        MconsXY.block(0, 0, (CnstrFtStp.Mxy).rows(), (CnstrFtStp.Mxy).cols()) = CnstrFtStp.Mxy;
        MconsXY.block((CnstrFtStp.Mxy).rows(), 0, (CnstrZmp.MuAct).rows(), (CnstrZmp.MuAct).cols()) = CnstrZmp.MuAct;
        // constraints vector
        BconsXY.resize((CnstrFtStp.Bxy).rows() + (CnstrZmp.BuAct).rows());
        BconsXY.segment(0, (CnstrFtStp.Bxy).rows()) = CnstrFtStp.Bxy;
        BconsXY.segment((CnstrFtStp.Bxy).rows(), (CnstrZmp.BuAct).rows()) = CnstrZmp.BuAct;
    }

    //int_t nWSRAng = 200;
    Options QPAngOptionToSet;
    QPAngOptionToSet.setToMPC();
    QPAngOptionToSet.printLevel = PL_LOW;
    // Initializing the solver for the translations
    // QPxySolver =  new CP_QPSolver_OASES();
    QPxySolver.InitialiseQPSol(MQKxy, VPKxy, MconsXY, BconsXY);
    QPxySolver.setSolverOptions(QPAngOptionToSet);
    QPxySolver.CP_QPSolution_Oases_XY(MQKxy, VPKxy, MconsXY, BconsXY);
    // Initializing the solver for the orientation
    // QPAngSolver = new CP_QPSolver_OASES();
    QPAngSolver.InitialiseQPSol(MQang, VPang, CnstrFtStp.Mang, CnstrFtStp.Bang);
    QPAngSolver.setSolverOptions(QPAngOptionToSet);
    QPAngSolver.CP_QPSolution_Oases_Ang(MQang, VPang, CnstrFtStp.Mang, CnstrFtStp.Bang);

    // Feet reference trajectories generator
    FtTraj.Init(Parameters->StanceIndicator, RelCoPXYR, CoPref, SMx, Parameters->maxFootHeight);
    // Reference Homogeneous transformations for the legs
    GaitTrsf.Init(DMod, Parameters->StanceIndicator, CoPref, FtTraj, Parameters->Translation_B_CoM);

    dx_com_normalized = fabs(GaitTrsf.trsl_com_sft(0)/(0.5*FtTraj.sxyp1(0)+1e-5));


    //
    stance_foot   = "right";
    stance_right  = true;
    //
    pose_lf(0) =  0.0;
    pose_lf(1) =  0.5* Parameters->SupportConstraints(2);
    pose_lf.tail(5) << 0.0, 0.0, 0.0, 1.0, 0.0;
    pose_rf(0) =  0.0;
    pose_rf(1) = -0.5* Parameters->SupportConstraints(2);
    pose_rf.tail(5) << 0.0, 0.0, 0.0, 1.0, 0.0;
    this->UpdateFeetPose();
    
    //

    cout << "------------------------------Initial---------------------------------------------------------" << endl;
    std::cout << "CoPref.CoPRefX    is : \t " << CoPref.CoPRefX << std::endl;
    std::cout << "CoPref.CoPRefY    is : \t " << CoPref.CoPRefY << std::endl;
    std::cout << "StatesX           is : \t " << DMod.StatesX.transpose() << std::endl;
    std::cout << "StatesY           is : \t " << DMod.StatesY.transpose() << std::endl;

    std::cout << "CoPref.xi_0       is : \t " << CoPref.xi_0.transpose() << std::endl;
    std::cout << "CoPref.yi_0       is : \t " << CoPref.yi_0.transpose() << std::endl;

    std::cout << "QPxySol RelCoPXY  is : \t " << QPxySolver.RelCoPXY.transpose() << std::endl;
    std::cout << "RelCoPXYR         is : \t " << RelCoPXYR.transpose() << std::endl;
    std::cout << "RelCoPXYR_n1      is : \t " << RelCoPXYR_n1.transpose() << std::endl;
    cout << "----------------------------------------------------------------------------------------------" << endl;

}





void CpReactiveWalkingController::UpdateCpBalWlkController(InitBalWlkParameters *Parameters, VectorXd RelativeVelo, int CycleCounter)
{
    DTds = 2*Parameters->SamplingTime;
    Tgbs = (int)(round((Parameters->DurationSteps[0]-DTds)/Parameters->SamplingTime));
    Tgas = (int)(round(DTds/Parameters->SamplingTime));
    // **********************************************************
    // Selection of initial Stance foot
    // *********************************

    cout << "----------------------------------------------------------------------------------------------" << endl;
    cout << "Iteration in PG: " << CtrlCounter << endl;

    this->UpdateFeetPose();   // compute the foot postions wrt absolute frame
    
    if (CtrlCounter == 0)
    {
        // selection of the stance foot with predominance to external setting
        if(RelativeVelo(1) >= 0)  stance_right = true;
        if(stance_foot =="right") stance_right = true;
        if(stance_foot =="left")  stance_right = false;
        else {
            stance_right = true;
        }

        if (stance_right) //(RelativeVelo(1) >= 0)
        {
          // Right support foot
            // Stance Foot (CoP) reference pose
            Parameters->InitCoPrefPose(1) = this->a_pos_rf(1); //-0.055; //0.0681   // initial right foot Y position
            Parameters->InitCoPrefPose(4) = this->a_pos_rf(1); //-0.055;            // initial right foot Y position 1st future
            Parameters->InitCoPrefPose(2) = this->a_ori_rf; //-0.055;            // initial right foot R orientation 1st future
            CoPref.ReinitializeFootPose(Parameters->InitCoPrefPose);
            DMod.StatesY(1)= 1.0*CoPref.yi_0(0);
            DMod.StatesX(1)= -this->a_pos_rf(0); //0.05; //0.05;
            DMod.StatesR(0)= -this->a_ori_rf; //0.05; //0.05;
            // DMod.StatesX(1)= 0.05;                                                                  // this condition for stepping back
            // DMod.StatesY(1)= 0.0*CoPref.yi_0(0);
            if(Parameters->RobotName == "icub"){DMod.StatesY(1)= 0.5 * CoPref.yi_0(0);}  // 0.5

            // RelCoPXYR_n1(0) = -0.10; //-0.11;                                                       // this condition for stepping back
            RelCoPXYR_n1(1) = -1.0*Parameters->SupportConstraints(2); //-0.11;
            SwitchCounter = 1;
            // RelCoPXYR(0)    = -0.10;
            // RelCoPXYR_n1(0) = -0.10; //-0.11;
        }else {
            // Left support foot
            // Stance Foot (CoP) reference pose
            Parameters->InitCoPrefPose(1) = this->a_pos_lf(1); //0.055;
            Parameters->InitCoPrefPose(4) = this->a_pos_lf(1); //0.055;
            Parameters->InitCoPrefPose(2) = this->a_ori_lf; //-0.055;            // initial right foot R orientation 1st future
            CoPref.ReinitializeFootPose(Parameters->InitCoPrefPose);
            DMod.StatesY(1)= 1.0*CoPref.yi_0(0);
            DMod.StatesX(1)= -this->a_pos_lf(0); //0.05; //0.05;
            DMod.StatesR(0)= -this->a_ori_lf; //0.05; //0.05
            if(Parameters->RobotName == "icub"){DMod.StatesY(1)= 0.5 * CoPref.yi_0(0);}
            RelCoPXYR_n1(1) = 1.0*Parameters->SupportConstraints(2); //0.11;
            SwitchCounter = 0;
        }
        // update the cyclic Selection matrices
        SMx.CP_UpdateSelectionMx(CoPref.CoPRefR, QPAngSolver.OrienCoP, CtrlCounter);
        // SMx.CP_UpdateSelectionMx(CoPref.CoPRefR, VeloRef.FutureAngles, CtrlCounter);
    }
    else {
        // update the cyclic Selection matrices
        SMx.CP_UpdateSelectionMx(CoPref.CoPRefR ,QPAngSolver.OrienCoP, CtrlCounter);
        // SMx.CP_UpdateSelectionMx(CoPref.CoPRefR, VeloRef.FutureAngles, CtrlCounter);
        //
        // support foot indicator index
        if (SMx.IndexSFt == 0) { SwitchCounter += 1;}
    }


    // ===========================================================================================================

    if( (CtrlCounter > 0) && (fmod(CtrlCounter,16) == 0) && resetStep) //(CtrlCounter == 16)
    {
        // reinitialize the counter
        CtrlCounter = 0;
        //
        RelCoPXYR.setZero();
        //
        if (stance_right) //(RelativeVelo(1) >= 0)  // Right support foot
        {       
            // Stance Foot (CoP) reference pose
            Parameters->InitCoPrefPose(1) = CoPref.CoPRefY; //this->a_pos_rf(1); //-0.5* Parameters->SupportConstraints(2); //-0.055; //0.0681
            Parameters->InitCoPrefPose(4) = CoPref.CoPRefY; //this->a_pos_rf(1); //-0.5* Parameters->SupportConstraints(2); //-0.055; //0.0681
            Parameters->InitCoPrefPose(2) = CoPref.CoPRefR; //
            CoPref.ReinitializeFootPose(Parameters->InitCoPrefPose);
            DMod.StatesY(1)= 1.0*CoPref.yi_0(0);
            DMod.StatesX(1)= -this->a_pos_rf(0); //0.05; //0.05;                 // this condition for stepping back
            DMod.StatesR(0)= -this->a_ori_rf; //0.05; //0.05;
            // DMod.StatesY(1)= 0.0*CoPref.yi_0(0);
            if(Parameters->RobotName == "icub"){DMod.StatesY(1)= 0.5 * CoPref.yi_0(0);}
            // RelCoPXYR_n1(0) = -0.15; //-0.11;    
            RelCoPXYR_n1(0) = -RelCoPXYR_n1(0); //-0.11;                                               // this condition for stepping back
            RelCoPXYR_n1(1) = -RelCoPXYR_n1(1); //-1.0*Parameters->SupportConstraints(2); //-0.11;
            RelCoPXYR_n1(2) =  RelCoPXYR_n1(2); // orientation
            SwitchCounter = 1;

        }else {
            // Left support foot
            // Stance Foot (CoP) reference pose
            Parameters->InitCoPrefPose(1) = CoPref.CoPRefY; //this->a_pos_lf(1); //0.5* Parameters->SupportConstraints(2); //0.055;
            Parameters->InitCoPrefPose(4) = CoPref.CoPRefY; //this->a_pos_lf(1); //0.5* Parameters->SupportConstraints(2); //0.055;
            Parameters->InitCoPrefPose(2) = CoPref.CoPRefR; //
            CoPref.ReinitializeFootPose(Parameters->InitCoPrefPose);
            DMod.StatesY(1)= 1.0*CoPref.yi_0(0);
            DMod.StatesX(1)= -this->a_pos_lf(0); //0.05; //0.05;  
            DMod.StatesR(0)= -this->a_ori_lf; //0.05; //0.05;
            if(Parameters->RobotName == "icub"){DMod.StatesY(1)= 0.5 * CoPref.yi_0(0);}
            RelCoPXYR_n1(0) = -RelCoPXYR_n1(0); //-0.11;   
            RelCoPXYR_n1(1) = -RelCoPXYR_n1(1); // 1.0*Parameters->SupportConstraints(2); //0.11;
            RelCoPXYR_n1(2) =  RelCoPXYR_n1(2); // orientation
            SwitchCounter = 0;
        }
        // update the cyclic Selection matrices
        SMx.CP_UpdateSelectionMx(CoPref.CoPRefR, QPAngSolver.OrienCoP, CtrlCounter);
    }

    // ===========================================================================================================

        double flipper;
        // ------------------------------------------------------------
        if (fmod(SwitchCounter,2) == 0) {
            Parameters->StanceIndicator[0] = 1; // Left  Uk_0
            Parameters->StanceIndicator[1] = 0; // Right Uc_k_0
            flipper = 0.0;
            // std::cout << "Left stance  is : \t " << 1.0 << std::endl;
            // stance_right = false;
        }
        else {
            Parameters->StanceIndicator[0] = 0;
            Parameters->StanceIndicator[1] = 1;
            flipper = 1.0;
            // std::cout << "Right stance  is : \t " << 1.0 << std::endl;
            // stance_right = true;
        }
        //
        // Parameters->StanceIndicator[0] = 0;
        // Parameters->StanceIndicator[1] = 1;
    // Compute the absolute velocty to be applied
    VeloRef.WorldVelocitiesSetPts(SMx, RelativeVelo);

    double step_size = 0.12;  // 0.12

    //
    // if(CtrlCounter >=16){
    //     step_size_x = -0.10;
    // }
    // step_size_y
    // step_theta
    // VeloRef.FutureStepsXY(0) = -step_size_x * pow(-1.0, flipper);
    // VeloRef.FutureStepsXY(1) = -pow(-1.0, flipper) * Parameters->SupportConstraints(2) + step_size_y;
    // VeloRef.FutureStepsXY(2) = -step_size_x * pow(-1.0, flipper);
    // VeloRef.FutureStepsXY(3) =  pow(-1.0, flipper) * Parameters->SupportConstraints(2) + step_size_y;
    // VeloRef.FutureStepsXY(4) = -step_size_x * pow(-1.0, flipper);
    // VeloRef.FutureStepsXY(5) = -pow(-1.0, flipper) * Parameters->SupportConstraints(2) + step_size_y;

    VeloRef.FutureStepsXY(0) = step_size_x;
    VeloRef.FutureStepsXY(1) = -pow(-1.0, flipper) * Parameters->SupportConstraints(2) + step_size_y;
    VeloRef.FutureStepsXY(2) = step_size_x;
    VeloRef.FutureStepsXY(3) =  pow(-1.0, flipper) * Parameters->SupportConstraints(2) + step_size_y;
    VeloRef.FutureStepsXY(4) = step_size_x;
    VeloRef.FutureStepsXY(5) = -pow(-1.0, flipper) * Parameters->SupportConstraints(2) + step_size_y;

    VeloRef.FutureAngles(0) = step_theta;
    VeloRef.FutureAngles(1) = step_theta;
    VeloRef.FutureAngles(2) = step_theta;

    // VeloRef.FutureStepsXY(0) = step_size;
    // VeloRef.FutureStepsXY(1) = -pow(-1.0, flipper) * Parameters->SupportConstraints(2);
    // VeloRef.FutureStepsXY(2) = step_size;
    // VeloRef.FutureStepsXY(3) =  pow(-1.0, flipper) * Parameters->SupportConstraints(2);
    // VeloRef.FutureStepsXY(4) = step_size;
    // VeloRef.FutureStepsXY(5) = -pow(-1.0, flipper) * Parameters->SupportConstraints(2);



    // ===========================================================================================================
   
    // std::cout << "CoPref.xi_0 before update is : \t " << CoPref.xi_0.transpose() << std::endl;
    // std::cout << "CoPref.yi_0 before update is : \t " << CoPref.yi_0.transpose() << std::endl;


    // if(CtrlCounter >=16)
    // {
    //     Parameters->StanceIndicator[0] = 0;
    //     Parameters->StanceIndicator[1] = 1;
    //     flipper = 1.0;
    // }
    // ===========================================================================================================

    // Update the Pose the robot with the QP solution
    CoPref.UpdateStanceFootPose(SMx, RelCoPXYR, RelCoPXYR_n1);

    std::cout << " xk_fc_ang_n1   is : \t " << CoPref.xk_fc_xy_n1.transpose() << "  Iteration   is : \t " << CtrlCounter << std::endl;
    // Update of the Hessian Matrix
    MatrixXd MQKxy = QMx.getQMxy(MpcModel, SMx);
    MatrixXd MQang = QMx.getQMang(MpcModel, SMx);
    // Update of the Gradient vector
    OptimSolXY_n1(0) = QPxySolver.uX;
    OptimSolXY_n1(1) = QPxySolver.uY;

    VectorXd VPKxy = PVec.getVeckxy(MpcModel, DMod, CoPref, SMx, VeloRef, OptimSolXY_n1);
    VectorXd VPang = PVec.getVeckAng(MpcModel, DMod, CoPref, SMx, VeloRef);

    // Update the Constraints on the Footsteps
    CnstrFtStp.UpdateFtstpConstraints(CoPref, Parameters->StanceIndicator);
    // Determine the constraints on the Center of Pressure
    // adapting the constraints to the DSP
    if ((SMx.IndexSFt >=Tgbs)||(SMx.IndexSFt <=Tgas))
    {
        if (Parameters->StanceIndicator[1] == 1){
            Spolygon_limits.resize(4);
            Spolygon_limits(0) = 0.150; Spolygon_limits(1) = 0.20; //0.02;
            Spolygon_limits(2) = 0.150; Spolygon_limits(3) = 0.03; //0.02;

            CnstrZmp.SetSupportEdgesLimits(Spolygon_limits);
        }
        else {
            Spolygon_limits.resize(4);
            Spolygon_limits(0) = 0.150; Spolygon_limits(1) = 0.03; //0.02;
            Spolygon_limits(2) = 0.150; Spolygon_limits(3) = 0.20; //0.02;

            CnstrZmp.SetSupportEdgesLimits(Spolygon_limits);
        }
    }
    else {
        Spolygon_limits.resize(4);
        Spolygon_limits(0) = 0.070; Spolygon_limits(1) = 0.03; //0.02;
        Spolygon_limits(2) = 0.070; Spolygon_limits(3) = 0.03; //0.02;

        CnstrZmp.SetSupportEdgesLimits(Spolygon_limits);
    }

    CnstrZmp.FindReducedZmpConstraints12(MpcModel,SMx, DMod, CoPref);

    // matrix of contraints on the CoP and the ZMP
    if (Parameters->FootStepsCnstrOnly){
        // constraints matrix
        MconsXY.resize((CnstrFtStp.Mxy).rows(), (CnstrFtStp.Mxy).cols() + 2);
        MconsXY.setZero((CnstrFtStp.Mxy).rows(), (CnstrFtStp.Mxy).cols() + 2);
        MconsXY.block(0, 0, (CnstrFtStp.Mxy).rows(), (CnstrFtStp.Mxy).cols()) = CnstrFtStp.Mxy;
        // constrainst vector
        BconsXY = CnstrFtStp.Bxy;
    } else {
        // constraints matrix
        MconsXY.resize((CnstrZmp.MuAct).rows()+(CnstrFtStp.Mxy).rows(), (CnstrFtStp.Mxy).cols() + 2);
        MconsXY.setZero((CnstrZmp.MuAct).rows()+(CnstrFtStp.Mxy).rows(), (CnstrFtStp.Mxy).cols() + 2);
        MconsXY.block(0, 0, (CnstrFtStp.Mxy).rows(), (CnstrFtStp.Mxy).cols()) = CnstrFtStp.Mxy;
        MconsXY.block((CnstrFtStp.Mxy).rows(), 0, (CnstrZmp.MuAct).rows(), (CnstrZmp.MuAct).cols()) = CnstrZmp.MuAct;
        // constraints vector
        BconsXY.resize((CnstrFtStp.Bxy).rows() + (CnstrZmp.BuAct).rows());
        BconsXY.segment(0, (CnstrFtStp.Bxy).rows()) = CnstrFtStp.Bxy;
        BconsXY.segment((CnstrFtStp.Bxy).rows(), (CnstrZmp.BuAct).rows()) = CnstrZmp.BuAct;
    }

    // Get the optimal solution
    nWSR = 1000;  //200  450
    //
    QPxySolver.setnbWorkingSetRecalculation(nWSR);
    QPxySolver.CP_QPSolution_Oases_XY(MQKxy, VPKxy, MconsXY, BconsXY);

    QPAngSolver.setnbWorkingSetRecalculation(nWSR);
    QPAngSolver.CP_QPSolution_Oases_Ang(MQang, VPang, CnstrFtStp.Mang, CnstrFtStp.Bang);
    //
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TESTING FOR CHANGING THE COM HEIGHT
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Eigen::Vector3d rel_pendulum_position;

    rel_pendulum_position(0) = (DMod.StatesX)(0) - CoPref.CoPRefX;
    rel_pendulum_position(1) = (DMod.StatesY)(0) - CoPref.CoPRefY;
    rel_pendulum_position(2) =  DMod.zc;
    //
    double adapted_com_height = sqrt(  Parameters->CoMHeight*Parameters->CoMHeight 
                                     - rel_pendulum_position(0)*rel_pendulum_position(0)
                                     - rel_pendulum_position(1)*rel_pendulum_position(1));
    // Updating the height
    // DMod.zc = adapted_com_height;
    // DMod.zc = Parameters->CoMHeight;
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // update of the states
    DMod.UpdateStates_CP_LIP_M(QPxySolver.uX, QPxySolver.uY, QPAngSolver.uR);
    // Updating the height
    // DMod.zc = adapted_com_height;
    // FOOT TRAJECTORIES
    FtTraj.ComputeFootTrajectories(Parameters->StanceIndicator, RelCoPXYR, CoPref, SMx);
    //cout << "RelCoPXYR is = "<< RelCoPXYR << endl;

    // Legs Transformations
    // Update the position of the base wrt to CoM.
    // GaitTrsf.SetTranslationBaseCoM(Parameters->Translation_B_CoM);  //< ==========================

    // Compute the gait transformations
    GaitTrsf.ComputeGaitTransforms(DMod, Parameters->StanceIndicator, CoPref, FtTraj);
    //Update the Pose the robot with the QP solution
    RelCoPXYR.segment(0, RelCoPXY.rows()) = QPxySolver.RelCoPXY;
    RelCoPXYR.segment(6, 2)               = QPxySolver.RelEoSXY;
    RelCoPXYR.segment(8, OrienCoP.rows()) = QPAngSolver.OrienCoP;

    RelCoPXYR_n1(0) = QPxySolver.RelCoPXY_n1(0);
    RelCoPXYR_n1(1) = QPxySolver.RelCoPXY_n1(1);
    RelCoPXYR_n1(2) = QPAngSolver.OrienCoP_n1(0);

    // RelCoPXYR.segment(0, RelCoPXY.rows()) = VeloRef.FutureStepsXY;
    // RelCoPXYR.segment(6, 2)               = QPxySolver.RelEoSXY;
    // RelCoPXYR.segment(8, OrienCoP.rows()) = VeloRef.FutureAngles;

    // RelCoPXYR_n1(0) = RelCoPXYR(0);
    // RelCoPXYR_n1(1) = RelCoPXYR(1);
    // RelCoPXYR_n1(2) = RelCoPXYR(8);

    VectorXd zx_ref = SMx.CP_GlobalCurrentSelX * CoPref.CoPRefX + SMx.CP_GlobalFutureSelX * QPxySolver.RelCoPXY;
    VectorXd zy_ref = SMx.CP_GlobalCurrentSelY * CoPref.CoPRefY + SMx.CP_GlobalFutureSelY * QPxySolver.RelCoPXY;

    CoPref.xCP_ref = (SMx.FeE_t * QPxySolver.RelEoSXY(0) + SMx.Fep_t * zx_ref)(0);
    CoPref.yCP_ref = (SMx.FeE_t * QPxySolver.RelEoSXY(1) + SMx.Fep_t * zy_ref)(0);



    // ==========================
    if (CtrlCounter == 0)
        dx_com_normalized = 0.0;
    else{
        dx_com_normalized = fabs(GaitTrsf.trsl_com_sft(0)/(0.5*FtTraj.sxyp1(0)+1e-5));
    }
    // =================

    std::cout << "CtrlCounter is : \t " << CtrlCounter << std::endl;

    if(WaitForContact) {
        CtrlCounter = CtrlCounter;
    }
    else {
        CtrlCounter ++;       
    }
    
    //
    
    // std::cout << "VeloRef.FutureStepsXY  is : \t " << VeloRef.FutureStepsXY.transpose() << std::endl;
    // std::cout << "RelCoPXYR     rotation is : \t " << QPAngSolver.OrienCoP.transpose() << std::endl;
    // std::cout << "VeloRef.FutureAngles   is : \t " << VeloRef.FutureAngles.transpose() << std::endl; 

       
    // std::cout << "CoPref.CoPRefX    is : \t " << CoPref.CoPRefX << std::endl;
    // std::cout << "CoPref.CoPRefY    is : \t " << CoPref.CoPRefY << std::endl;
    // std::cout << "CoPref.CoPRefR    is : \t " << CoPref.CoPRefR << std::endl;
    // std::cout << "StatesX           is : \t " << DMod.StatesX.transpose() << std::endl;
    // std::cout << "StatesY           is : \t " << DMod.StatesY.transpose() << std::endl;

    // std::cout << "CoPref.xi_0       is : \t " << CoPref.xi_0.transpose() << std::endl;
    // std::cout << "CoPref.yi_0       is : \t " << CoPref.yi_0.transpose() << std::endl;

    // std::cout << "QPxySol RelCoPXY  is : \t " << QPxySolver.RelCoPXY.transpose() << std::endl;
    // std::cout << "RelCoPXYR         is : \t " << RelCoPXYR.transpose() << std::endl;
    // std::cout << "RelCoPXYR_n1      is : \t " << RelCoPXYR_n1.transpose() << std::endl;

    // std::cout << " NORMALIZED DISTANCE is  : \t" << dx_com_normalized << std::endl;
    // std::cout << " COM IN STANCE FOOT  is  : \t" << GaitTrsf.trsl_com_sft(0) << std::endl;
    // std::cout << " PREDICTED STEP POS  is  : \t" << FtTraj.sxyp1(0) << std::endl;

    // std::cout << " =======================================================================================>>>>>>>> "  << std::endl;
 

    // std::cout << "Trsf_lfoot_com  is : \n " << GaitTrsf.Trsf_lfoot_com << std::endl;
    // std::cout << "Trsf_rfoot_com  is : \n " << GaitTrsf.Trsf_rfoot_com << std::endl;

    // dx_com_normalized = fabs(GaitTrsf.trsl_com_sft(0)/(0.5*FtTraj.sxyp1(0)+1e-5));

}

//
void CpReactiveWalkingController::set_step_magnitude(Vector3d step_magnitude)
{
    //
    this->step_size_x =  step_magnitude(0); 
    this->step_size_y =  step_magnitude(1); 
    this->step_theta  =  step_magnitude(2) + OrientOffest;   
}




void CpReactiveWalkingController::UpdateFeetPose()
{
    // absolute foot pose
    Vector7d W_Pose_aF;
    Transforms.get_absolute_pose(pose_lf, pose_rf, W_Pose_aF);
    Matrix4d W_H_aF_ = Transforms.PoseVector2HomogenousMx(W_Pose_aF); 
    Matrix4d aF_H_lf = W_H_aF_.inverse() * Transforms.PoseVector2HomogenousMx(pose_lf);
    Matrix4d aF_H_rf = W_H_aF_.inverse() * Transforms.PoseVector2HomogenousMx(pose_rf);
    //
    // translation
    // -----------
    a_pos_ft   = W_H_aF_.block<2,1>(0,3);
    a_pos_lf   = aF_H_lf.block<2,1>(0,3); 
    a_pos_rf   = aF_H_rf.block<2,1>(0,3);
    a_pos_l_rf = a_pos_lf - a_pos_rf;
    a_pos_r_lf = a_pos_rf - a_pos_lf;
    //
    // orientation
    // -----------
    // extraction of Euler angles of  the feet rotation 
    Vector3d o_aF = Transforms.getEulerAnglesXYZ_FixedFrame(W_Pose_aF);
    Vector3d o_lf = Transforms.getEulerAnglesXYZ_FixedFrame(pose_lf);
    Vector3d o_rf = Transforms.getEulerAnglesXYZ_FixedFrame(pose_rf);
    //
    a_ori_ft    = o_aF(2);
    a_ori_lf    = o_lf(2) - a_ori_ft;
    a_ori_rf    = o_rf(2) - a_ori_ft;
    a_ori_l_rf  = a_ori_lf - a_ori_rf;
    a_ori_r_lf  = a_ori_rf - a_ori_lf;
    //
}

// void CpReactiveWalkingController::UpdateFeetPose()
// {
//     a_pos_ft   = 0.5*(pose_lf.head(2) + pose_rf.head(2));
//     a_pos_lf   = pose_lf.head(2) - a_pos_ft;
//     a_pos_rf   = pose_rf.head(2) - a_pos_ft;
//     a_pos_l_rf = a_pos_lf - a_pos_rf;
//     a_pos_r_lf = a_pos_rf - a_pos_lf;
//     //
//     // extraction of Euler angles of  the feet rotation 
//     Vector3d o_lf = Transforms.getEulerAnglesXYZ_FixedFrame(pose_lf);
//     Vector3d o_rf = Transforms.getEulerAnglesXYZ_FixedFrame(pose_rf);

//     a_ori_ft    = 0.5*(o_lf(2) + o_rf(2));
//     a_ori_lf    = o_lf(2) - a_ori_ft;
//     a_ori_rf    = o_rf(2) - a_ori_ft;
//     a_ori_l_rf  = a_ori_lf - a_ori_rf;
//     a_ori_r_lf  = a_ori_rf - a_ori_lf;
// }

// ===============================================================================================

// ===============================================================================================

CpFootTrajectories::CpFootTrajectories(){}
CpFootTrajectories::~CpFootTrajectories(){}

void CpFootTrajectories::Init(                   int SptFt[],
                                            VectorXd RelCoPXYR,
                                    CpStanceFootPose CoPref,
                                CP_SelectionMatrices SMx,
                                              double FtZmax)
{
    // resize the vector PrevStanceXY
    PrevStanceXY.resize(2);
    PrevStanceXY.setZero(2);
    // Assigning foot maximun height
    MaxFootHeight       = FtZmax;
    // stance foot indicator
    StanceIndicator[0]  = SptFt[0];
    StanceIndicator[1]  = SptFt[1];
    // calling the FooTrajectories methodes
    CpFootTrajectories::ComputeFootTrajectories(SptFt, RelCoPXYR, CoPref, SMx);           
}
                       
void CpFootTrajectories::ComputeFootTrajectories(              int SptFt[],
                                                        VectorXd RelCoPXYR,
                                                 CpStanceFootPose CoPref,
                                                 CP_SelectionMatrices SMx)
                        
{
    //********************************************************
    // update the stance foot states
    // stance foot indicator
    StanceIndicator[0] = SptFt[0];
    StanceIndicator[1] = SptFt[1];
    // Input Arguments
    tm = SMx.IndexSFt * ((SMx.DrS[0]/SMx.Tp)/(SMx.DrS[0]/SMx.Tp -1));
    double Ts = SMx.DrS[0]/SMx.Tp;

    int dtz = 0;
    double tmz = max((SMx.IndexSFt-dtz), 0) * (((SMx.DrS[0]-dtz*SMx.Tp)/SMx.Tp)/((SMx.DrS[0]-dtz*SMx.Tp)/SMx.Tp -1));
    double Tsz = (SMx.DrS[0]-dtz*SMx.Tp)/SMx.Tp;

    std::cout << " xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx " << tm << std::endl;
    std::cout << " TIME MO GEN " << tm/Ts << std::endl;
    std::cout << " TIME Z MO GEN " << tmz/Tsz << std::endl;
    std::cout << " SMx.IndexSFt   " << SMx.IndexSFt << std::endl;
    std::cout << " SAMPLING TIME   " << SMx.Tp << std::endl;
    std::cout << " xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx " << tm << std::endl;

    PrevStanceXY   = CoPref.xk_fc_xy_n1;
    PrevStanceAng  = CoPref.xk_fc_ang_n1;

    double z_max = MaxFootHeight;
    double D_theta, D_theta_p1;
   
    D_theta      = CoPref.CoPRefR - PrevStanceAng; // Delta_theta (Theta_0 - Theta_-1)
    D_theta_p1   = RelCoPXYR(8) - CoPref.CoPRefR;  //Delta_theta+1 (Theta_+1 -Theta_0)// RelCoPXYR(8)== OrienCoP(0)
   // angle for COM orientation
    double Angtj_ts;
    Angtj_ts = -2*((D_theta)/pow(Ts,3.))*pow(tm,3.) +
                3*(D_theta/pow(Ts,2.))*pow(tm,2.);

   //%%%%%%%% X, Y, Z and Theta TRAJECTORIES %%%%%%%%%%%%%%%
   //
   // Computation of previous and future footstep relative to the
   // stance foot
   // ----------------------------------------------------------
   // Previous and 1st future steps relative to stance foot
   // sxyn1 and sxyp1
   sxyn1.resize(2); // previous step relative to stance (sxy1)
   sxyp1.resize(2); // 1st future step relative to stance (sxy2_t)
                             // as funcion of time (at every sample instant)
   // sxyn1(0) =  cos(D_theta)*PrevStanceXY(0) + sin(D_theta)*PrevStanceXY(1);
   // sxyn1(1) = -sin(D_theta)*PrevStanceXY(0) + cos(D_theta)*PrevStanceXY(1);

   sxyn1(0) =  cos(Angtj_ts)*PrevStanceXY(0) + sin(Angtj_ts)*PrevStanceXY(1);
   sxyn1(1) = -sin(Angtj_ts)*PrevStanceXY(0) + cos(Angtj_ts)*PrevStanceXY(1);

   sxyp1(0) = (1.+cos(-D_theta+Angtj_ts))*1/2.*(RelCoPXYR)(0)    //RelCoPXYR(0) == RelCoPXY(0)
                 -sin(-D_theta+Angtj_ts)*1/2.*(RelCoPXYR)(1);       //RelCoPXYR(1) == RelCoPXY(1)
   sxyp1(1) = sin(-D_theta+Angtj_ts)*1/2.*(RelCoPXYR)(0)
             + (1.+cos(-D_theta+Angtj_ts))*1/2.*(RelCoPXYR)(1);

    
    std::cout << "D_theta    is : \t " << D_theta << std::endl;
    std::cout << "sxyn1    is : \t " << sxyn1.transpose() << std::endl;
    std::cout << "sxyp1    is : \t " << sxyp1.transpose() << std::endl;
    std::cout << "sxyp1(0)-sxyn1(0)    is : \t " << sxyp1(0)-sxyn1(0) << std::endl;

    // Quintic Polynomial trajectories for X and Y
    // ===========================================
    double xtj_ts, // x swing foot trajectory
           ytj_ts, // y swing trajectory
           ztj_ts, // z swing trajectory
           A_mft_tj_ts; // angular swing trajectory
           
    xtj_ts =   6*((sxyp1(0)-sxyn1(0))/pow(Ts,5.))*pow(tm,5.)
             -15*((sxyp1(0)-sxyn1(0))/pow(Ts,4.))*pow(tm,4.)
             +10*((sxyp1(0)-sxyn1(0))/pow(Ts,3.))*pow(tm,3.)
             + sxyn1(0);
             
    ytj_ts =   6*((sxyp1(1)-sxyn1(1))/pow(Ts,5.))*pow(tm,5.)
             -15*((sxyp1(1)-sxyn1(1))/pow(Ts,4.))*pow(tm,4.)
             +10*((sxyp1(1)-sxyn1(1))/pow(Ts,3.))*pow(tm,3.)
             + sxyn1(1);            
    // polynomial trajectory for Z via zmax
    //-------------------------------------
    // ztj_ts = - 64*(z_max/pow(Ts,6.))*pow(tm,6.)
    //          +192*(z_max/pow(Ts,5.))*pow(tm,5.)
    //          -192*(z_max/pow(Ts,4.))*pow(tm,4.)
    //          + 64*(z_max/pow(Ts,3.))*pow(tm,3.);

     ztj_ts = - 64*(z_max/pow(Tsz,6.))*pow(tmz,6.)
              +192*(z_max/pow(Tsz,5.))*pow(tmz,5.)
              -192*(z_max/pow(Tsz,4.))*pow(tmz,4.)
              + 64*(z_max/pow(Tsz,3.))*pow(tmz,3.);
    // Angle for swinging foot orientation
    //--------------------------------------
    A_mft_tj_ts =  6*((D_theta_p1+D_theta)/pow(Ts,5.))*pow(tm,5.)
                 -15*((D_theta_p1+D_theta)/pow(Ts,4.))*pow(tm,4.)
                 +10*((D_theta_p1+D_theta)/pow(Ts,3.))*pow(tm,3.)
                 - D_theta;  // -ve sign
    //
    // Feet trajectories in Stance Foot Frame
    // =======================================
    // Left Foot
    XTrajLeftFoot   = xtj_ts * SptFt[1]; //Uc_k_0(1,1);         // X
    YTrajLeftFoot   = ytj_ts * SptFt[1]; //Uc_k_0(1,1);         // Y
    ZTrajLeftFoot   = ztj_ts * SptFt[1]; //Uc_k_0(1,1);         // Z
    AngTrajLeftFoot = A_mft_tj_ts * SptFt[1]; //Uc_k_0(1,1);    // theta
    //
    // Right Foot
    XTrajRightFoot   = xtj_ts * SptFt[0]; //Uk_0(1,1);         // X
    YTrajRightFoot   = ytj_ts * SptFt[0]; //Uk_0(1,1);         // Y
    ZTrajRightFoot   = ztj_ts * SptFt[0]; //Uk_0(1,1);         // Z
    AngTrajRightFoot = A_mft_tj_ts * SptFt[0]; //Uk_0(1,1);    // theta
    //
    DelTheta0n1 = D_theta;
    DelThetap1  = D_theta_p1;                     
    //
}
    
void CpFootTrajectories::SetMaxFootHeight(double z_max)
{
    MaxFootHeight = z_max;  
};


// ====================================================================================================================
// ====================================================================================================================

CpGaitTransformations::CpGaitTransformations(){};
CpGaitTransformations::~CpGaitTransformations(){};

void CpGaitTransformations::Init(Discrete_CP_LIP_Model St,
                                                               int SptFt[],
                                                  CpStanceFootPose CoPref,
                                                CpFootTrajectories FtTraj,
                                                          VectorXd t_B_CoM)
{
    // translation from Com to base %%% (base expressed in com frame)
    TrslBaseCoM = t_B_CoM; //[x_B_com; y_B_com; z_B_com];
    //
    z_CoM = St.zc;
    //
    trsl_com_sft.setZero();
    CpGaitTransformations:: ComputeGaitTransforms( St, SptFt, CoPref, FtTraj);
}
  
void CpGaitTransformations:: ComputeGaitTransforms( Discrete_CP_LIP_Model St,
                                                                      int SptFt[],
                                                         CpStanceFootPose CoPref,
                                                       CpFootTrajectories FtTraj)
{
    // From inertial to support foot frame %%%
    // Transformation from support for to inertial frame   
    // Rotation matrix from sft to inertial from
    Matrix3d R_sft_inertial_f   =  Eigen::MatrixXd::Zero(3,3);
    R_sft_inertial_f(0,0)       =  cos(CoPref.CoPRefR);
    R_sft_inertial_f(0,1)       = -sin(CoPref.CoPRefR);
    R_sft_inertial_f(1,0)       =  sin(CoPref.CoPRefR);  
    R_sft_inertial_f(1,1)       =  cos(CoPref.CoPRefR);
    R_sft_inertial_f(2,2)       =  1.;
    // Translation vector from sft to inertial frame
    Vector3d trsl_sft_inertial_f; // Translation vector
    trsl_sft_inertial_f(0)      = CoPref.CoPRefX;   //ZMP ref x
    trsl_sft_inertial_f(1)      = CoPref.CoPRefY;   //ZMP ref y
    trsl_sft_inertial_f(2)      = 0.;               //ZMP height

    // Homogeneous Transformation from sft to inertial frame // Homogeneous transformation from 
    Matrix4d T_sft_inertial_f   = Eigen::MatrixXd::Identity(4,4);
    T_sft_inertial_f.block(0, 0, 3, 3) = R_sft_inertial_f;
    T_sft_inertial_f.block(0, 3, 3, 1) = trsl_sft_inertial_f;

    //  Transformation from CoM to Support foot position of CoM in support foot 
    Eigen::Vector3d trsl_com_sft_inertial;
    // expressed in the inertial frame
    trsl_com_sft_inertial(0)    = (St.StatesX)(0) - CoPref.CoPRefX;
    trsl_com_sft_inertial(1)    = (St.StatesY)(0) - CoPref.CoPRefY;
    trsl_com_sft_inertial(2)    =  St.zc;

    // expressed in the support ft frame 
    trsl_com_sft = R_sft_inertial_f.transpose() * trsl_com_sft_inertial;

    // std::cout << " Translation COM is Stance foot : " << trsl_com_sft.transpose() << std::endl;

    // Homogeneous tranformation of Feet trajectories in the base 
    // transformation from base to support foot
    Matrix4d T_B_sft = Eigen::MatrixXd::Zero(4,4);      T_B_sft(3,3) = 1.0;

    // Adapting transformation from General Inertial frame to Icub definition frame
    /* The general inertial frame is defined with the x axis lying in the frontal plane
    * pointing in front of the robot (roll axis)
    * the y axis form the center to the left foot and the z axis in the vertical direction
    * opposite to the gravity
    * the general base frame follows the same convention */
    // From General base to  icub
    Matrix3d    R_GB_icubB      = Eigen::MatrixXd::Zero(3,3);
                R_GB_icubB(0,0) = -1.0;
                R_GB_icubB(1,1) = -1.0;
                R_GB_icubB(2,2) =  1.0;
    Matrix4d    T_GB_icubB      = Eigen::MatrixXd::Zero(4,4);  T_GB_icubB(3,3) = 1.;
                T_GB_icubB.block(0,0, 3, 3) = R_GB_icubB;
    // inverse transformation From icub Base to the General Base
    Matrix4d    invT_GB_icubB   = Eigen::MatrixXd::Zero(4,4);  invT_GB_icubB(3,3) = 1.;
                invT_GB_icubB.block(0,0,3,3) = R_GB_icubB.transpose();
    // Transformation from icub foot frame to the Genaral foot frame
    Matrix3d    R_icubF_GF      = Eigen::MatrixXd::Zero(3,3);
                R_icubF_GF(0,2) =  1.0;
                R_icubF_GF(1,1) = -1.0;
                R_icubF_GF(2,0) =  1.0;
    Matrix4d    T_icubF_GF      = Eigen::MatrixXd::Zero(4,4);  T_icubF_GF(3,3) = 1.;
                T_icubF_GF.block(0,0, 3, 3) = R_icubF_GF;
    // inverse transformation From icub Base to the General Base
    Matrix4d    invT_icubF_GF   = Eigen::MatrixXd::Zero(4,4);  invT_icubF_GF(3,3) = 1.;
                invT_icubF_GF.block(0,0,3,3) = R_icubF_GF.transpose();

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Matrix4d    T_Pitch_Base      = Eigen::MatrixXd::Zero(4,4);  T_Pitch_Base(3,3) = 1.;
                double pitch;
                pitch = 1. *0.00; //0.03
                T_Pitch_Base(0,0) = cos(pitch);
                T_Pitch_Base(0,2) = sin(pitch);
                T_Pitch_Base(1,1) = 1.;
                T_Pitch_Base(2,0) = -sin(pitch);
                T_Pitch_Base(2,2) = cos(pitch);

    // feet roll corection
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~
    double roll_coef = 1.0 *0.0;  // 0.03
    // left
    Matrix4d    T_l_foot_roll       = Eigen::MatrixXd::Zero(4,4);  T_l_foot_roll(3,3) = 1.;
                T_l_foot_roll(0,0)  =  cos(roll_coef);
                T_l_foot_roll(0,1)  = -sin(roll_coef);
                T_l_foot_roll(1,0)  =  sin(roll_coef);
                T_l_foot_roll(1,1)  =  cos(roll_coef);
                T_l_foot_roll(2,2)  =  1.;
    // right
    Matrix4d    T_r_foot_roll       =  Eigen::MatrixXd::Zero(4,4);  T_r_foot_roll(3,3) = 1.;
                T_r_foot_roll(0,0)  =  cos(roll_coef);  //-
                T_r_foot_roll(0,1)  = -sin(roll_coef);
                T_r_foot_roll(1,0)  =  sin(roll_coef);
                T_r_foot_roll(1,1)  =  cos(roll_coef);
                T_r_foot_roll(2,2)  =  1.;

    // **********************************************************************
    if  (SptFt[0] ==1)  // the left foot is the stance foot
    {
        // Transformation from the CoM to the left stance foot
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Rotation matrix from CoM to Sft
        Matrix3d    R_com_sft       =  Eigen::MatrixXd::Zero(3,3);
                    R_com_sft(0,0)  =  cos(FtTraj.AngTrajRightFoot/2.);
                    R_com_sft(0,1)  = -sin(FtTraj.AngTrajRightFoot/2.);
                    R_com_sft(1,0)  =  sin(FtTraj.AngTrajRightFoot/2.);
                    R_com_sft(1,1)  =  cos(FtTraj.AngTrajRightFoot/2.);
                    R_com_sft(2,2)  =  1.;

        // Homogeneous transformation from CoM to Sft
        // T_com_sft.resize(4,4);  
        Matrix4d    T_com_sft = Eigen::MatrixXd::Zero(4,4);    T_com_sft(3,3) = 1.0;
                    T_com_sft.block(0, 0, 3, 3) = R_com_sft;
                    T_com_sft.block(0, 3, 3, 1) = trsl_com_sft;
        // Homogeneous transformation from support foot to base
        T_B_sft.block(0, 0, 3, 3) = R_com_sft;
        T_B_sft.block(0, 3, 3, 1) = R_com_sft*TrslBaseCoM + trsl_com_sft;

        Matrix4d    T_sft_B = Eigen::MatrixXd::Zero(4,4);     T_sft_B(3,3) = 1.0;
                    T_sft_B.block(0, 0, 3, 3) =  R_com_sft.transpose();
                    T_sft_B.block(0, 3, 3, 1) = -TrslBaseCoM - R_com_sft.transpose()*trsl_com_sft;

        // Transformation from right moving (swing) foot to Support foot
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Rotation matrix from right swing ft to L sft
        Matrix3d    R_right_mft_sft         = Eigen::MatrixXd::Zero(3,3);
                    R_right_mft_sft(0,0)    =  cos(FtTraj.AngTrajRightFoot);
                    R_right_mft_sft(0,1)    = -sin(FtTraj.AngTrajRightFoot);
                    R_right_mft_sft(1,0)    =  sin(FtTraj.AngTrajRightFoot);
                    R_right_mft_sft(1,1)    =  cos(FtTraj.AngTrajRightFoot);
                    R_right_mft_sft(2,2)    =  1.;

        // Translation vector from R moving ft to L stance foot
        Vector3d    trsl_right_mft_sft; // Translation vector
                    trsl_right_mft_sft(0) = FtTraj.XTrajRightFoot;
                    trsl_right_mft_sft(1) = FtTraj.YTrajRightFoot;
                    trsl_right_mft_sft(2) = FtTraj.ZTrajRightFoot;
        // Homogeneous Transformation from R moving ft to L stance foot
        Matrix4d    T_right_mft_sft = Eigen::MatrixXd::Zero(4,4);   T_right_mft_sft(3,3) = 1.0;
                    T_right_mft_sft.block(0, 0, 3, 3) = R_right_mft_sft;
                    T_right_mft_sft.block(0, 3, 3, 1) = trsl_right_mft_sft;

        // Overall Transformations
        // **************************
        Trsf_lfoot_base_world = T_sft_B;
        Trsf_rfoot_base_world = T_sft_B * T_right_mft_sft;

        Trsf_lfoot_base_icub  = T_GB_icubB * Trsf_lfoot_base_world;
        Trsf_rfoot_base_icub  = T_GB_icubB * Trsf_rfoot_base_world;

        // // Transformation from Left Leg to the Base Frame
        // TrsfLeftFootBase  = T_Pitch_Base * T_GB_icubB * T_sft_B * T_icubF_GF;
        // // Transformation from Right Leg to the Base Frame
        // TrsfRightFootBase = T_Pitch_Base * T_GB_icubB * T_sft_B * T_right_mft_sft  * T_icubF_GF * T_r_foot_roll;

        // Transformation from Left Leg to the Base Frame
        TrsfLeftFootBase  = T_Pitch_Base * Trsf_lfoot_base_icub * T_icubF_GF;
        // Transformation from Right Leg to the Base Frame
        TrsfRightFootBase = T_Pitch_Base * Trsf_rfoot_base_icub  * T_icubF_GF * T_r_foot_roll;
        //
        // Transformations of the feet in the CoM frame
        Trsf_lfoot_com = T_com_sft.inverse();
        Trsf_rfoot_com = T_com_sft.inverse() * T_right_mft_sft;
    }
    else   // (Uc_k_0(1,1)==1) the right foot is the stance foot
    {
        // Rotation from the CoM to the right stance foot
        // Rotation matrix
        Matrix3d    R_com_sft = Eigen::MatrixXd::Zero(3,3);
                    R_com_sft(0,0) =  cos(FtTraj.AngTrajLeftFoot/2.);
                    R_com_sft(0,1) = -sin(FtTraj.AngTrajLeftFoot/2.);
                    R_com_sft(1,0) =  sin(FtTraj.AngTrajLeftFoot/2.);
                    R_com_sft(1,1) =  cos(FtTraj.AngTrajLeftFoot/2.);
                    R_com_sft(2,2) =  1.;

        // Homogeneous transformation 
        Matrix4d    T_com_sft = Eigen::MatrixXd::Zero(4,4);     T_com_sft(3,3) = 1;
                    T_com_sft.block(0, 0, 3, 3) = R_com_sft;
                    T_com_sft.block(0, 3, 3, 1) = trsl_com_sft;

        // Homogeneous transformation from support foot to base
        T_B_sft.block(0, 0, 3, 3) = R_com_sft;
        T_B_sft.block(0, 3, 3, 1) = R_com_sft*TrslBaseCoM + trsl_com_sft;

        Matrix4d    T_sft_B = Eigen::MatrixXd::Zero(4,4);   T_sft_B(3,3) = 1;
                    T_sft_B.block(0, 0, 3, 3) = R_com_sft.transpose();  // R_B_com = I
                    T_sft_B.block(0, 3, 3, 1) = -TrslBaseCoM - R_com_sft.transpose()*trsl_com_sft;

        // Transformation from left moving (swing) foot to Support foot
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Rotation matrix from Left swing ft to R sft    
        Matrix3d    R_left_mft_sft      =  Eigen::MatrixXd::Zero(3,3);
                    R_left_mft_sft(0,0) =  cos(FtTraj.AngTrajLeftFoot);
                    R_left_mft_sft(0,1) = -sin(FtTraj.AngTrajLeftFoot);
                    R_left_mft_sft(1,0) =  sin(FtTraj.AngTrajLeftFoot);
                    R_left_mft_sft(1,1) =  cos(FtTraj.AngTrajLeftFoot);
                    R_left_mft_sft(2,2) =  1.;

        // Translation vector from L moving ft to R stance foot
        Vector3d    trsl_left_mft_sft; // Translation vector
                    trsl_left_mft_sft(0) = FtTraj.XTrajLeftFoot;
                    trsl_left_mft_sft(1) = FtTraj.YTrajLeftFoot;
                    trsl_left_mft_sft(2) = FtTraj.ZTrajLeftFoot;

        // Homogeneous Transformation from L moving ft to R stance foot
        Matrix4d    T_left_mft_sft = Eigen::MatrixXd::Zero(4,4);   T_left_mft_sft(3,3) = 1.0;    // Homogeneous transformation
                    T_left_mft_sft.block(0, 0, 3, 3) = R_left_mft_sft;
                    T_left_mft_sft.block(0, 3, 3, 1) = trsl_left_mft_sft;

        // Overall Transformations
        // **************************
        Trsf_lfoot_base_world = T_sft_B * T_left_mft_sft;
        Trsf_rfoot_base_world = T_sft_B;

        Trsf_lfoot_base_icub  = T_GB_icubB * Trsf_lfoot_base_world;
        Trsf_rfoot_base_icub  = T_GB_icubB * Trsf_rfoot_base_world;
        // // Transformation from Left Leg to the Base Frame
        // TrsfLeftFootBase  = T_Pitch_Base * T_GB_icubB * T_sft_B * T_left_mft_sft  * T_icubF_GF * T_l_foot_roll;
        // // Transformation from Right Leg to the Base Frame
        // TrsfRightFootBase = T_Pitch_Base * T_GB_icubB * T_sft_B  * T_icubF_GF;

        // Transformation from Left Leg to the Base Frame
        TrsfLeftFootBase  = T_Pitch_Base * Trsf_lfoot_base_icub  * T_icubF_GF * T_l_foot_roll;
        // Transformation from Right Leg to the Base Frame
        TrsfRightFootBase = T_Pitch_Base * Trsf_rfoot_base_icub  * T_icubF_GF;

        // Transformations of the feet in the CoM frame
        Trsf_lfoot_com = T_com_sft.inverse() * T_left_mft_sft;
        Trsf_rfoot_com = T_com_sft.inverse();
    }
    // Homogeneous transformation of Base trajectory in World frame %%
    TrsfBaseInWorld         = T_sft_inertial_f * T_B_sft;
    // Translation of the base relative to the stance foot frame.
    RefT_icubB_icubF        = invT_icubF_GF * T_B_sft * invT_GB_icubB;
    // Desired positon of the icub base wrt. icub foot
    RefTransl_Base_in_Foot  = RefT_icubB_icubF.block(0,3, 3, 1);
        
};

void CpGaitTransformations::SetTranslationBaseCoM(VectorXd t_B_CoM)
{
    // translation from Com to base %%% (base expressed in com frame)
    TrslBaseCoM = t_B_CoM;
};



// ===============================================================================================

SyncRobotModelStanceFoot::SyncRobotModelStanceFoot() {}

SyncRobotModelStanceFoot::~SyncRobotModelStanceFoot() {}

void SyncRobotModelStanceFoot::InitStanceSync(double F_Threshld, int T_Threshld)
{
    // default values
    minForceThrshld = 10.0;
    minTimeThrshld  = 5; 
    // user defined values
    minForceThrshld = F_Threshld;
    minTimeThrshld  = T_Threshld; 
}

void SyncRobotModelStanceFoot::SyncStancePhase(               int CycleCounter,
                                                InitBalWlkParameters *Parameters,
                                                CpReactiveWalkingController *CpBalWlkController,
                                                         VectorXd l_foot_FT_vector,
                                                         VectorXd r_foot_FT_vector)
{
    if (fabs(l_foot_FT_vector(2))-fabs(r_foot_FT_vector(2)) > minForceThrshld)
    {
        stance_left  = 1;
        stance_right = 0;
    }
    else if (fabs(r_foot_FT_vector(2))-fabs(l_foot_FT_vector(2)) > minForceThrshld)
    {
        stance_left  = 0;
        stance_right = 1;
    }

    if (CycleCounter <= minTimeThrshld)
    {
        stance_left  = Parameters->StanceIndicator[0];
        stance_right = Parameters->StanceIndicator[1];
    }
    // wait for the supposed support foot to touch the ground
    // check the stance as per the model and the actual stance foot measured 
    // by the torque/ force sensors
    if (Parameters->SwitchingSync)
    {
        if (Parameters->StanceIndicator[0] == 1 && stance_left !=1){
            // wait
            CpBalWlkController->WaitForContact = true;
        }
        else if (Parameters->StanceIndicator[1] == 1  && stance_right !=1){
            // wait
            CpBalWlkController->WaitForContact = true;
        }
        else {
            CpBalWlkController->WaitForContact = false;
        }
    }
}



// ==================================================================================================================