
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

#ifndef MPCOptimization_H 
#define MPCOptimization_H

#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/SVD>
#include <Eigen/LU>

#include <qpOASES.hpp>

#include "ComPredictionModels.h"
#include "wbhpidcUtilityFunctions.hpp"
// #include "PatternsGenerator.h"



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


class QPSolver_OASES
{
   // members

   public :

       double uX     ;  // X current control action (Variation of CoP or jerk)
       double uY     ;  // Y current control action (Variation of CoP or jerk)
       double uDx    ;  // X current variation of the disturbance
       double uDy    ;  // Y current variation of the disturbance
       double uX_n1  ;  // X current control action (Variation of CoP or jerk)
       double uY_n1  ;  // X current control action (Variation of CoP or jerk)

       SQProblem OasesSolver;

       //real_t xOpt[];
       //int_t nbWorkSetRecal;
       int nbWorkSetRecal;

       // Solving the QP problem

       QPSolver_OASES();
       ~QPSolver_OASES();

       void InitialiseQPSol(MatrixXd H,
                            VectorXd f,
                            MatrixXd A_cons,
                            VectorXd b);

       VectorXd qpOasesSolver (MatrixXd H,
                               VectorXd f,
                               MatrixXd A_cons,
                               VectorXd b);

       void setnbWorkingSetRecalculation(int nWSR);

       void setSolverOptions(Options optionToSet);

       VectorXd QPSolution_Oases_Stab(  MatrixXd Q,     	// Hessian matrix
                                        VectorXd p_k,   	// gradient vector
                                        MatrixXd M,   		// Matrix of constraints
                                        VectorXd gam);		// vector of constraints


       void QPSolution_Oases_XY(  MatrixXd Q,       // Hessian matrix
                                    VectorXd p_k,       // gradient vector
                                    MatrixXd M,         // Matrix of constraints
                                    VectorXd gam);      // vector of constraints

};

// ===================================================================================
/*
 * CpQMatrix : This class encodes the Hessian matrix related to the MPC QP problem
*/
// ===================================================================================
class QMatrix 
{


    public :

        double GainsVec[10]; 	// Vector of QP gains

        MatrixXd MQstab;		// Hessian matrix stabilizability x
        MatrixXd MQstabXY;      // Hessian matrix stabilizability x and y
        // MatrixXd MQstab;		// Hessian matrix stabilizability y

        QMatrix();
        ~QMatrix();

        void Init(PredictionModel MpcModel, double GainsArray[]);

        MatrixXd getQstab(PredictionModel MpcModel, StepReferecences StpRef_);
        MatrixXd getQstabXY(PredictionModel MpcModel, StepReferecences StpRef_);

        // MatrixXd getQzmp(PredictionModel MpcModel, StepReferecences StpRef_);
};

// ===================================================================================
/*
 * CpPVectorQP : This class encodes the Gradient Vector related to the MPC QP problem
*/
// ===================================================================================

class PVectorQP
{
	public :

	double GainsVec[10]; //

	MatrixXd P1trl; // Constant part of vector P translation

	VectorXd VPstabX; 	//	P vector X
	VectorXd VPstabY; 	//	P vector Y
    VectorXd VPstabXY;  //  P vector X and Y

	int CpModelOption;  // Type of LIPM Model

	PVectorQP();
	~PVectorQP();

	void Init(PredictionModel MpcModel, double GainsArray[]);

	VectorXd getPstab(	PredictionModel MpcModel,   // LMPC model of the LIPM
                        Perturbed_LIPM St,          // CoM State variables
                        StepReferecences  StpRef,   // step references pose
                        std::string option);        // direction

    VectorXd getPstabXY(PredictionModel MpcModel,   // LMPC model of the LIPM
                        Perturbed_LIPM St,          // CoM State variables
                        StepReferecences  StpRef);   // step references pose

	// VectorXd getPzmp(	PredictionModel MpcModel,   // LMPC model of the LIPM
 //                        Perturbed_LIPM St,          // CoM State variables
 //                        StepReferecences  StpRef,   // step references pose
 //                        std::string option);        // direction
};
   

// ====================================================================================================
// class MatrixPseudoInverse2
// {

//     public : 

//         MatrixPseudoInverse2(){}

//         ~MatrixPseudoInverse2(){}

//         // Compute the pseudo inverse of a matrix
//         template<typename _Matrix_Type_> _Matrix_Type_ get_pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
//         {
            
//             Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);

//             int svdSize = svd.singularValues().size();

//             double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);

//             return svd.matrixV().leftCols(svdSize) *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().leftCols(svdSize).adjoint();
//         }

//         bool get_HhQRPseudoInverse(MatrixXd myMatrix, MatrixXd &PsdInvmyMatrix)
//         {

//             HouseholderQR<MatrixXd> qr(myMatrix.transpose());
//             PsdInvmyMatrix.setIdentity(myMatrix.cols(), myMatrix.rows());
//             PsdInvmyMatrix = qr.householderQ() * PsdInvmyMatrix;
//             PsdInvmyMatrix = qr.matrixQR().topLeftCorner(myMatrix.rows(),myMatrix.rows()).triangularView<Upper>().transpose().solve<OnTheRight>(PsdInvmyMatrix);

//             return true;

//         }

//         bool get_CODecomPseudoInverse(MatrixXd myMatrix, MatrixXd &PsdInvmyMatrix)
//         {
//             //
//             CompleteOrthogonalDecomposition<MatrixXd> cqr(myMatrix);
//             PsdInvmyMatrix = cqr.pseudoInverse();

//             return true;
//         }

//         bool get_LLTSolveInverse(MatrixXd myMatrix, MatrixXd &Inv_myMatrix)
//         {
//             //
//             Eigen::MatrixXd UnitMx = Eigen::MatrixXd::Identity(myMatrix.cols(), myMatrix.rows()); 
//             Inv_myMatrix = myMatrix.llt().solve(UnitMx);

//             return true;
//         }

//         bool get_LUSolveInverse(MatrixXd myMatrix, MatrixXd &Inv_myMatrix)
//         {
//             //
//             Eigen::MatrixXd UnitMx = Eigen::MatrixXd::Identity(myMatrix.cols(), myMatrix.rows()); 
//             Inv_myMatrix = myMatrix.lu().solve(UnitMx);

//             return true;
//         }
//         bool get_LDLTSolveInverse(MatrixXd myMatrix, MatrixXd &Inv_myMatrix)
//         {
//             //
//             Eigen::MatrixXd UnitMx = Eigen::MatrixXd::Identity(myMatrix.cols(), myMatrix.rows()); 
//             Inv_myMatrix = myMatrix.ldlt().solve(UnitMx);

//             return true;
//         }

// };

// ===================================================================================
/*
 * ConstraintsCoP : this class encodes the constraints imposed on the
 * CoP/ZMP .
 * From the normal and positions of the foot sides with respect to the foot 
 * origin and from the desired footstep position and orientation, it computes
 * the constraints matrices and the bounds vector
*/

// ===================================================================================
class ConstraintsCoP
{
    // Definition of normal to the edges (4 sides)
    MatrixXd EdgeNormals;
    // Definition of the limits of the distance
    VectorXd EdgePositions;
    // Edge_pos=[0.15; 0.1; 0.15; 0.1];
    public :

        MatrixXd MuAct;
        VectorXd BuAct;

        MatrixPseudoInverse2 PsdInv;
        // Option for the type of LIPM model used
        ConstraintsCoP();
        ~ConstraintsCoP();
        // 
        void Init(  PredictionModel     MpcModel,
                    Perturbed_LIPM      DM,
                    StepReferecences    StpRef,
                    MatrixXd            EdgeNormals_,
                    VectorXd            EdgePositions_);

        /*This method computes the constraints matrix and vector over the first two steps only */
        void GetCoPConstraints( PredictionModel     MpcModel,
                                Perturbed_LIPM      DM,
                                StepReferecences    StpRef_);
        /*this method is used to set the positions of the support foot sides with respect to the foot origin.*/
        void SetSupportEdgesLimits(VectorXd Edge_lim);
        /*this method is used to set the normal to the foot sides with respect to the foot origin.*/
        void SetSupportEdgesNormals(MatrixXd Edge_Nor);
        //
        void UpdateSupportPolygon(MatrixXd Edge_Nor, VectorXd Edge_lim);
};


// ===================================================================================
/*
 * ReachConstraintCoM : this class encodes the constraints imposed on the
 * COM that ensures reachability of the target object by the robot's hands.
 * From the normal and positions of the foot sides with respect to the foot 
 * origin and from the desired footstep position and orientation, it computes
 * the constraints matrices and the bounds vector
*/
// ===================================================================================
class ReachConstraintCoM
{
    public :

        MatrixXd    MuCoM;
        VectorXd    BuCoM;
        
        ReachConstraintCoM();
        ~ReachConstraintCoM();
        // 
        void Init(  PredictionModel     MpcModel,
                    Perturbed_LIPM      DM,
                    StepReferecences    StpRef,
                    Vector2d            ffmmcom_);

        /*This method computes the constraints matrix and vector over the prediction horizon only */
        void applyReachConstraintCoM(   PredictionModel     MpcModel,
                                        Perturbed_LIPM      DM,
                                        StepReferecences    StpRef,
                                        Vector2d            ffmmcom_);
};



#endif // MPCOptimization_H


