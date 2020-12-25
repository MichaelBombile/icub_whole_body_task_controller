/* 
 * Copyright (C) 2020 Learning Algorithms and Systems Laboratory (LASA)
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

// MPCOptimizer
// 	1. class CP_QPSolver_OASES 		: the QP solver
// 	2. class CpQMatrix				: the Hessiam Matrix
// 	3. class CpPVectorQP			: the Gradient Vector
// 	4. class CpConstraintsFootsteps	: the Footstep Constraints
// 	5. class CpConstraintsOutputZmp : the ZMP Constraints

#include "ComPredictionModels.h"
#include "MPCOptimization.h"

USING_NAMESPACE_QPOASES

using namespace std;
using namespace Eigen;

// ===================================================================================
/*
 * CP_QPSolver_OASES : This class encodes a QP solver based on 
 * qpOASES library 
 *
*/
// ===================================================================================

QPSolver_OASES::QPSolver_OASES(){}

QPSolver_OASES::~QPSolver_OASES(){}

void QPSolver_OASES::InitialiseQPSol(      MatrixXd H,
                                              VectorXd f,
                                         MatrixXd A_cons,
                                               VectorXd b)
{
	//intitialisation
    uX  = 0;
    uY  = 0;
    uDx = 0;
    uDy = 0;

	OasesSolver = SQProblem (H.rows(), A_cons.rows());

	real_t xOpt[H.rows()];

	/* Setup data of first QP. */
   real_t HMx[H.rows() * H.cols()];
   real_t gVc[H.rows()];
   real_t AMx[A_cons.rows() * A_cons.cols()];
   real_t ubA[b.rows()];
   //
   int iter = 0;

   for (int i=0; i<H.rows(); i++)
   {
       for (int j=0; j<H.cols(); j++)
       {
           HMx[iter] = H(i,j);
           iter +=1;
       }
       gVc[i] = f(i);
   }
   iter = 0;
   
   for (int i=0; i<A_cons.rows(); i++)
   {
       for (int j=0; j<A_cons.cols(); j++)
       {
           AMx[iter] = A_cons(i,j);
           iter +=1;
       }
       ubA[i] = b(i);
   }

   nbWorkSetRecal = 1000;  //200  450

   OasesSolver.init(HMx,gVc,AMx,0,0,0,ubA, nbWorkSetRecal, 0);

   OasesSolver.getPrimalSolution(xOpt);

};

 VectorXd QPSolver_OASES::qpOasesSolver (      MatrixXd H,
                                                    VectorXd f,
                                               MatrixXd A_cons,
                                                    VectorXd b)
 {
       //
       VectorXd OptimalSol(H.rows());
       real_t xOpt[H.rows()];

       /* Setup data of first QP. */
           real_t HMx_new[H.rows()*H.cols()];
           real_t gVc_new[H.rows()];
           real_t AMx_new[A_cons.rows()*A_cons.cols()];
           real_t ubA_new[b.rows()];

           int iter;
           iter = 0;
           for (int i=0; i<H.rows(); i++)
           {
               for (int j=0; j<H.cols(); j++)
               {
                   HMx_new[iter] = H(i,j);
                   iter +=1;
               }
               gVc_new[i] = f(i);
           }
           iter = 0;

           for (int i=0; i<A_cons.rows(); i++)
           {
               for (int j=0; j<A_cons.cols(); j++)
               {
                   AMx_new[iter] = A_cons(i,j);
                   iter +=1;
               }
               ubA_new[i] = b(i);
           }
           //
           OasesSolver.hotstart(HMx_new,gVc_new,AMx_new, 0, 0, 0, ubA_new, nbWorkSetRecal, 0);
          //extracting the optimal solution
           OasesSolver.getPrimalSolution(xOpt);

           for (int i=0; i<H.rows(); i++)
           {
               OptimalSol(i) = xOpt[i];
           }

        return OptimalSol;

};

void QPSolver_OASES::setnbWorkingSetRecalculation(int nWSR)
{
     nbWorkSetRecal = nWSR;
}

void QPSolver_OASES::setSolverOptions(Options optionToSet)
{
    OasesSolver.setOptions(optionToSet);
}


VectorXd QPSolver_OASES::QPSolution_Oases_Stab( MatrixXd Q,     	// Hessian matrix
                                                VectorXd p_k,   	// gradient vector
                                                MatrixXd M,   		// Matrix of constraints
                                                VectorXd gam)		// vector of constraints
{
    // saving the results one sample before
    return QPSolver_OASES::qpOasesSolver(Q,p_k,M,gam);
};


void QPSolver_OASES::QPSolution_Oases_XY(   MatrixXd Q,         // Hessian matrix
                                                VectorXd p_k,       // gradient vector
                                                MatrixXd M,         // Matrix of constraints
                                                VectorXd gam)       // vector of constraints
{
    // saving the results one sample before
    VectorXd Optim_uk = QPSolver_OASES::qpOasesSolver(Q,p_k,M,gam);
    int nu = Optim_uk.rows()/4; 
    //
    uX  = Optim_uk(0);
    uY  = Optim_uk(nu);
    uDx = Optim_uk(2*nu);
    uDy = Optim_uk(3*nu);
};

// ===================================================================================
/*
 * CpQMatrix : This class encodes the Hessian matrix related to the MPC QP problem
*/
// ===================================================================================
QMatrix::QMatrix(){}
QMatrix::~QMatrix(){}

void QMatrix::Init(PredictionModel MpcModel, double GainsArray[])
{

   // weights to penalize different objective of the LMPC walkin
   // problem
   for (int i=0; i<5; i++)
   {
       GainsVec[i] = GainsArray[i];
   }

   double alp = GainsVec[0]; // Uact : Control action
   double bet = GainsVec[1]; // IVel : Instantaneous velocity
   double gam = GainsVec[2]; // Ztrk : Output tracking
   double kap = GainsVec[3]; // AVel : Disturbance tracking

   //
   MQstab   = MatrixXd::Zero(2*MpcModel.Nu, 2*MpcModel.Nu);
   MQstabXY = MatrixXd::Zero(4*MpcModel.Nu, 4*MpcModel.Nu);
}

//
MatrixXd QMatrix::getQstab(PredictionModel MpcModel, StepReferecences StpRef)
{
	//
	MQstab = MatrixXd::Zero(2*MpcModel.Nu, 2*MpcModel.Nu);
	//
	MatrixXd Eye_Nu;
	Eye_Nu.resize(MpcModel.Nu, MpcModel.Nu);
	Eye_Nu.setIdentity(MpcModel.Nu, MpcModel.Nu);

   	// Prefactorisation of some term of the Q matrix
    // MatrixXd Pzu_lbda_Ppu = MpcModel.Pzu + StpRef.lambda_z * MpcModel.Ppu;
    MatrixXd Pzu_lbda_Ppu = MpcModel.Pzu;

    MatrixXd MQjj = MatrixXd::Zero(MpcModel.Nu, MpcModel.Nu);
    MatrixXd MQdd = MatrixXd::Zero(MpcModel.Nu, MpcModel.Nu);
    MatrixXd MQjd = MatrixXd::Zero(MpcModel.Nu, MpcModel.Nu);
    // (j:jerk, d: disturbance)
    MQjj    =  GainsVec[0] * Eye_Nu 
            +  GainsVec[1] * (MpcModel.Pvu).transpose() * MpcModel.Pvu
            +  GainsVec[2] *  Pzu_lbda_Ppu.transpose()  * Pzu_lbda_Ppu;

    MQdd    =  GainsVec[4] * Eye_Nu 
            +  (GainsVec[2]+GainsVec[3]) * MpcModel.Pdu.transpose() * MpcModel.Pdu;

    MQjd    = GainsVec[2] * Pzu_lbda_Ppu.transpose() * MpcModel.Pdu;
    //
    MQstab.topLeftCorner(MpcModel.Nu,MpcModel.Nu)     = MQjj;
    MQstab.bottomRightCorner(MpcModel.Nu,MpcModel.Nu) = MQdd;

    MQstab.topRightCorner(MpcModel.Nu,MpcModel.Nu)    = MQjd;
    MQstab.bottomLeftCorner(MpcModel.Nu,MpcModel.Nu)  = MQjd.transpose();

	return MQstab;
}

//
MatrixXd QMatrix::getQstabXY(PredictionModel MpcModel, StepReferecences StpRef)
{
    //
    MQstabXY = MatrixXd::Zero(4*MpcModel.Nu, 4*MpcModel.Nu);
    this->getQstab(MpcModel, StpRef);

    int Nu = MpcModel.Nu;

    MQstabXY.block(   0,    0, Nu,Nu) = this->MQstab.topLeftCorner(Nu,Nu);
    MQstabXY.block(  Nu,   Nu, Nu,Nu) = this->MQstab.topLeftCorner(Nu,Nu);
    MQstabXY.block(2*Nu, 2*Nu, Nu,Nu) = this->MQstab.bottomRightCorner(Nu,Nu);
    MQstabXY.block(3*Nu, 3*Nu, Nu,Nu) = this->MQstab.bottomRightCorner(Nu,Nu);

    MQstabXY.block( 0,   2*Nu, Nu,Nu) = this->MQstab.topRightCorner(Nu,Nu);
    MQstabXY.block(Nu,   3*Nu, Nu,Nu) = this->MQstab.topRightCorner(Nu,Nu);

    MQstabXY.block(2*Nu,    0, Nu,Nu) = this->MQstab.bottomLeftCorner(Nu,Nu);
    MQstabXY.block(3*Nu,   Nu, Nu,Nu) = this->MQstab.bottomLeftCorner(Nu,Nu);
    
    return MQstabXY;
}

// MatrixXd QMatrix::getQzmp(PredictionModel MpcModel, StepReferecences StpRef)
// {
// 	//
// 	MQstab = MatrixXd::Zero(MpcModel.Nu, MpcModel.Nu);
// 	//
// 	MatrixXd Eye_Nu;
// 	Eye_Nu.resize(MpcModel.Nu, MpcModel.Nu);
// 	Eye_Nu.setIdentity(MpcModel.Nu, MpcModel.Nu);

//    	// Prefactorisation of some term of the Q matrix
//     MatrixXd Pdu_lbda_Ppu = MpcModel.Pdu - StpRef.lambda_z * MpcModel.Ppu;

//     MQstab  =  GainsVec[0] * Eye_Nu   
//     		+  GainsVec[2] * (MpcModel.Pcu).transpose() * MpcModel.Pcu;

// 	return MQstab;
// }

 	// Pps
	// Pvs
	// Pzs
	// Pds
	// PEs

 // ===================================================================================
/*
 * PVectorQP : This class encodes the Gradient Vector related to the MPC QP problem
*/
// ===================================================================================

PVectorQP::PVectorQP(){};
PVectorQP::~PVectorQP(){};

void PVectorQP::Init(PredictionModel MpcModel, double GainsArray[])
{

    // weights to penalize different objectives of the LMPC walking problem
    for (int i=0; i<5; i++){
        GainsVec[i] = GainsArray[i];
    }
    
    VPstabX  = VectorXd::Zero(2*MpcModel.Nu);
    VPstabY  = VectorXd::Zero(2*MpcModel.Nu);
    VPstabXY = VectorXd::Zero(4*MpcModel.Nu);
}


VectorXd PVectorQP::getPstab(	PredictionModel MpcModel,    	// LMPC model of the LIPM
								Perturbed_LIPM St,      		// CoM State variables
								StepReferecences  StpRef,  		// step references pose
								std::string option) 			// direction
{
	// ****************************
	// double alp = GainsVec[0]; // Uact : Control action
	// double bet = GainsVec[1]; // IVel : Instantaneous velocity
	// double gam = GainsVec[2]; // Ztrk : Output tracking
	// double kap = GainsVec[3]; // AVel : Disturbance tracking
	VectorXd VPstab(2*MpcModel.Nu);

    // MatrixXd Pzu_lbda_Ppu = MpcModel.Pzu + StpRef.lambda_z * MpcModel.Ppu;
    // MatrixXd Pzs_lbda_Pps = MpcModel.Pzs + StpRef.lambda_z * MpcModel.Pps;
    MatrixXd Pzu_lbda_Ppu = MpcModel.Pzu;
    MatrixXd Pzs_lbda_Pps = MpcModel.Pzs;

    

	// Gradient vector
	if(option == "Y" || option == "y")
	{
		//
		VPstab.head(MpcModel.Nu)    = GainsVec[1]* (MpcModel.Pvu).transpose() * (MpcModel.Pvs * St.StatesY - StpRef.VyAbsStab)                                // ref velocity
                                    + GainsVec[2]* Pzu_lbda_Ppu.transpose() * (Pzs_lbda_Pps * St.StatesY + MpcModel.Pds * St.StatesDy  - StpRef.FootstepsY);  // ref foosteps or reference CoP
        VPstab.tail(MpcModel.Nu)    = GainsVec[2]* MpcModel.Pdu.transpose() * (Pzs_lbda_Pps * St.StatesY + MpcModel.Pds * St.StatesDy  - StpRef.FootstepsY)   // ref foosteps or reference CoP
                                    + GainsVec[3]* MpcModel.Pdu.transpose() * (MpcModel.Pds * St.StatesDy - StpRef.Disturb_cY);                               // ref disturbance const part
        //
		VPstabY = VPstab;
	}
	else
	{
		//
		VPstab.head(MpcModel.Nu)    = GainsVec[1]* (MpcModel.Pvu).transpose() * (MpcModel.Pvs * St.StatesX - StpRef.VxAbsStab)                                // ref velocity
                                    + GainsVec[2]* Pzu_lbda_Ppu.transpose() * (Pzs_lbda_Pps * St.StatesX + MpcModel.Pds * St.StatesDx  - StpRef.FootstepsX);  // ref foosteps or reference CoP
        VPstab.tail(MpcModel.Nu)    = GainsVec[2]* MpcModel.Pdu.transpose() * (Pzs_lbda_Pps * St.StatesX + MpcModel.Pds * St.StatesDx  - StpRef.FootstepsX)   // ref foosteps or reference CoP
                                    + GainsVec[3]* MpcModel.Pdu.transpose() * (MpcModel.Pds * St.StatesDx - StpRef.Disturb_cX);                               // ref disturbance const part
		//						   
		VPstabX = VPstab;
	}

	return VPstab;
}


 VectorXd PVectorQP::getPstabXY(PredictionModel     MpcModel,   // LMPC model of the LIPM
                                Perturbed_LIPM      St,         // CoM State variables
                                StepReferecences    StpRef)     // step references pose
{

    this->getPstab( MpcModel, St, StpRef, "x");
    this->getPstab( MpcModel, St, StpRef, "y");

    int Nu = MpcModel.Nu;

    VPstabXY.segment(   0, Nu) = VPstabX.head(Nu);      // jjx (j:jerk)
    VPstabXY.segment(  Nu, Nu) = VPstabY.head(Nu);      // jjy
    VPstabXY.segment(2*Nu, Nu) = VPstabX.tail(Nu);      // ddx
    VPstabXY.segment(3*Nu, Nu) = VPstabY.tail(Nu);      // ddy

    return VPstabXY;
}


// VectorXd PVectorQP::getPzmp(	PredictionModel MpcModel,    	// LMPC model of the LIPM
// 								Perturbed_LIPM St,      		// CoM State variables
// 								StepReferecences  StpRef,       // step references pose
// 								std::string option) 			// direction
// {
// 	//
// 	// ****************************
// 	// double alp = GainsVec[0]; // Uact : Control action
// 	// double bet = GainsVec[1]; // IVel : Instantaneous velocity
// 	// double gam = GainsVec[2]; // Ztrk : Output tracking
// 	// double kap = GainsVec[3]; // AVel : Disturbance tracking
// 	VectorXd VPstab(MpcModel.Nu);

// 	// Gradient vector
// 	//
// 	VPstab =   -GainsVec[2]* (MpcModel.Pcu).transpose() * (MpcModel.Pcs * St.StatesR - StpRef.FootstepsX);										// ref foosteps
				   

// 	return VPstab;
// }


// ===================================================================================
/*
 * ConstraintsCoP : this class encodes the constraints imposed on the
 * ZMP .
 * From the normal and positions of the foot sides with respect to the foot 
 * origin and from the desired footstep position and orientation, it computes
 * the constraints matrices and the bounds vector
*/

// ===================================================================================
ConstraintsCoP::ConstraintsCoP(){}
ConstraintsCoP::~ConstraintsCoP(){}

void ConstraintsCoP::Init(  PredictionModel     MpcModel,
                            Perturbed_LIPM      DM,
                            StepReferecences    StpRef,
                            MatrixXd            EdgeNormals_,
                            VectorXd            EdgePositions_)

{
        // Initialisation of the normal
        EdgeNormals = EdgeNormals_;
        // EdgeNormals(0,0) = 1.; EdgeNormals(1,1) = 1.; //[1  0  -1  0;    // x
        // EdgeNormals(0,2) =-1.; EdgeNormals(1,3) =-1.; // 0  1   0 -1];   // y
        // Initialisation of the edges positions
        EdgePositions = EdgePositions_;
        // [0.050; 0.01; 0.03; 0.01];
        // EdgePositions(0) = 0.070; EdgePositions(1) = 0.03;  //0.02;
        // EdgePositions(2) = 0.070; EdgePositions(3) = 0.03;  //0.02;
        //
        int n0, nu, nbs;

        n0  = MpcModel.SamplesPerStep;                      //nb of samples in Step1
        nu  = (MpcModel.PEu).cols();                        // nb of columns of Pzu matrix
        nbs = EdgeNormals.cols();                           // nb of vertices of support foot

        MuAct.resize(nbs*n0, 2*nu+2*nu);     MuAct.setZero();    // Overall Constraints Matrix
        BuAct.resize(nbs*n0);                BuAct.setZero();    // Overall Constraints Vector

        ConstraintsCoP::GetCoPConstraints(  MpcModel,
                                            DM,
                                            StpRef);
};

// 
// this assumes that the norma vector and edges have been updated 
void ConstraintsCoP::GetCoPConstraints( PredictionModel     MpcModel,
                                        Perturbed_LIPM      DM,
                                        StepReferecences    StpRef)
{
    // //
    int n0, nu, nbs;
    n0  = MpcModel.SamplesPerStep;         //nb of samples in Step1
    nu  = (MpcModel.PEu).cols();           // nb of columns of Pzu matrix
    nbs = this->EdgeNormals.cols();              // nb of vertices of support foot
    //
    MatrixXd dxy_0 = this->EdgeNormals;           //(dxy_0.row(1)) : current step (step1)     (dxy_0.row(2)) : current step (step1)

    // Matrix of the normal vectors components considering the foot orientation (here the foot orientation is 0 (The reference frame is defined wrt the foot))
    MatrixXd dx_theta1 = MatrixXd::Zero(nbs*n0, n0);
    MatrixXd dy_theta1 = MatrixXd::Zero(nbs*n0, n0);
    //
    for (int i=0; i<n0; i++)
    {
        dx_theta1.block(nbs*i, i, ((dxy_0.row(0)).transpose()).rows(), ((dxy_0.row(0)).transpose()).cols()) =  (dxy_0.row(0)).transpose();     // Edges Normal x
        dy_theta1.block(nbs*i, i, ((dxy_0.row(1)).transpose()).rows(), ((dxy_0.row(1)).transpose()).cols()) =  (dxy_0.row(1)).transpose();     // Edges Normal y
    }
    //
    // combination matrix of the control variable (Delta_p or C_dddot) and the constraints Product with Matrix Pzu
    MatrixXd DX_theta = MatrixXd::Zero(nbs*n0,nu); 
    MatrixXd DY_theta = MatrixXd::Zero(nbs*n0,nu);
    //
    DX_theta.block(0, 0, dx_theta1.rows(), nu) = dx_theta1 * MpcModel.Pzu.block(0, 0, n0, nu);  // This matrix can be precomputed 
    DY_theta.block(0, 0, dy_theta1.rows(), nu) = dy_theta1 * MpcModel.Pzu.block(0, 0, n0, nu);  // This matrix can be precomputed

    MatrixXd DDx_theta = MatrixXd::Zero(nbs*n0,nu); 
    MatrixXd DDy_theta = MatrixXd::Zero(nbs*n0,nu);
    //
    DDx_theta.block(0, 0, dx_theta1.rows(), nu) = dx_theta1 * MpcModel.Pdu.block(0, 0, n0, nu);  // This matrix can be precomputed 
    DDy_theta.block(0, 0, dy_theta1.rows(), nu) = dy_theta1 * MpcModel.Pdu.block(0, 0, n0, nu);  // This matrix can be precomputed

    // Overall Constraints Matrix
    MuAct.setZero();
    MuAct.block(0, 0,  DX_theta.rows(), DX_theta.cols())    =  DX_theta;
    MuAct.block(0, nu, DY_theta.rows(), DY_theta.cols())    =  DY_theta;
    MuAct.block(0, 2*nu,  DX_theta.rows(), DX_theta.cols()) =  DDx_theta;
    MuAct.block(0, 3*nu,  DY_theta.rows(), DY_theta.cols()) =  DDy_theta;


    // vector of CoP constraints due to the state of the CoM and the current stance foot position
    VectorXd S_Ux0 = StpRef.FootstepsX - MpcModel.Pzs * DM.StatesX - MpcModel.Pds * DM.StatesDx; 
    VectorXd S_Uy0 = StpRef.FootstepsY - MpcModel.Pzs * DM.StatesY - MpcModel.Pds * DM.StatesDy;
    //
    VectorXd DX_S_Ux0 = VectorXd::Zero(nbs*n0); 
    VectorXd DY_S_Uy0 = VectorXd::Zero(nbs*n0);
    //
    DX_S_Ux0.segment(0, nbs*n0) = dx_theta1 * S_Ux0.segment(0, n0);
    DY_S_Uy0.segment(0, nbs*n0) = dy_theta1 * S_Uy0.segment(0, n0);


    VectorXd B_theta = VectorXd::Zero(nbs*n0);
    for (int i=0; i<n0; i++) {
        B_theta.segment(nbs*i, (EdgePositions).rows()) = EdgePositions;
    }

    // Overall Constraints Vector
    BuAct = B_theta + (DX_S_Ux0 + DY_S_Uy0);
}

//
void ConstraintsCoP::SetSupportEdgesLimits(VectorXd Edge_lim)
{
    this->EdgePositions = Edge_lim;
};

void ConstraintsCoP::SetSupportEdgesNormals(MatrixXd Edge_Nor)
{
    this->EdgeNormals = Edge_Nor;
}

void ConstraintsCoP::UpdateSupportPolygon(MatrixXd Edge_Nor, VectorXd Edge_lim)
{
    this->EdgeNormals   = Edge_Nor;
    this->EdgePositions = Edge_lim;
}


// ===================================================================================
/*
 * ReachConstraintCoM : this class encodes the constraints imposed on the
 * COM that ensures reachability of the target object by the robot's hands.
 * From the normal and positions of the foot sides with respect to the foot 
 * origin and from the desired footstep position and orientation, it computes
 * the constraints matrices and the bounds vector
*/
// ===================================================================================
ReachConstraintCoM::ReachConstraintCoM(){}
ReachConstraintCoM::~ReachConstraintCoM(){}

void ReachConstraintCoM::Init(  PredictionModel     MpcModel,
                                Perturbed_LIPM      DM,
                                StepReferecences    StpRef,
                                Vector2d            ffmmcom_)

{
    
    int n0, nu;
    n0  = MpcModel.SamplesPerStep;                      // nb of samples in Step1
    nu  = (MpcModel.Ppu).cols();                        // nb of columns of Pzu matrix
    MuCoM.resize(n0, 2*nu+2*nu);     MuCoM.setZero();   // Overall Constraints Matrix
    BuCoM.resize(n0);                BuCoM.setZero();   // Overall Constraints Vector

    ReachConstraintCoM::applyReachConstraintCoM(MpcModel,
                                                DM,
                                                StpRef,
                                                ffmmcom_);
};

// 
// this assumes that the norma vector and edges have been updated 
void ReachConstraintCoM::applyReachConstraintCoM(   PredictionModel     MpcModel,
                                                    Perturbed_LIPM      DM,
                                                    StepReferecences    StpRef,
                                                    Vector2d            ffmmcom_)
{
    // //
    int n0, nu;
    n0  = MpcModel.SamplesPerStep;         //nb of samples in Step1
    nu  = (MpcModel.PEu).cols();           // nb of columns of Pzu matrix
    //
    Vector2d dxy_0 = ffmmcom_.normalized();           //(dxy_0.row(1)) : current step (step1)     (dxy_0.row(2)) : current step (step1)

    // Matrix of the normal vectors components considering the foot orientation (here the foot orientation is 0 (The reference frame is defined wrt the foot))
    MatrixXd dx_theta1 = MatrixXd::Zero(n0, n0);
    MatrixXd dy_theta1 = MatrixXd::Zero(n0, n0);
    //
    for (int i=0; i<n0; i++)
    {
        // dx_theta1.block(i, i, 1, 1) =  dxy_0(0);     // Edges Normal x
        // dy_theta1.block(i, i, 1, 1) =  dxy_0(1);     // Edges Normal y
        dx_theta1(i, i) =  dxy_0(0);     // Edges Normal x
        dy_theta1(i, i) =  dxy_0(1);     // Edges Normal y
    }
    //
    // combination matrix of the control variable (Delta_p or C_dddot) and the constraints Product with Matrix Pzu
    MatrixXd DX_theta = MatrixXd::Zero(n0,nu); 
    MatrixXd DY_theta = MatrixXd::Zero(n0,nu);
    //
    DX_theta.block(0, 0, dx_theta1.rows(), nu) = dx_theta1 * MpcModel.Ppu.block(0, 0, n0, nu);  // This matrix can be precomputed 
    DY_theta.block(0, 0, dy_theta1.rows(), nu) = dy_theta1 * MpcModel.Ppu.block(0, 0, n0, nu);  // This matrix can be precomputed
    // Overall Constraints Matrix
    MuCoM.setZero();
    MuCoM.block(0, 0,  DX_theta.rows(), DX_theta.cols())    =  DX_theta;
    MuCoM.block(0, nu, DY_theta.rows(), DY_theta.cols())    =  DY_theta;

    // vector of CoP constraints due to the state of the CoM and the current stance foot position
    VectorXd S_Ux0 = StpRef.FootstepsX - MpcModel.Pps * DM.StatesX; 
    VectorXd S_Uy0 = StpRef.FootstepsY - MpcModel.Pps * DM.StatesY;
    //
    VectorXd DX_S_Ux0 = VectorXd::Zero(n0); 
    VectorXd DY_S_Uy0 = VectorXd::Zero(n0);
    //
    DX_S_Ux0.segment(0, n0) = dx_theta1 * S_Ux0.segment(0, n0);
    DY_S_Uy0.segment(0, n0) = dy_theta1 * S_Uy0.segment(0, n0);


    VectorXd B_theta = VectorXd::Zero(n0);
    for (int i=0; i<n0; i++) {
        // B_theta.segment(i, 1) = ffmmcom_.norm();
        B_theta(i) = ffmmcom_.norm();
    }
    // Overall Constraints Vector
    BuCoM = B_theta + (DX_S_Ux0 + DY_S_Uy0);
}
