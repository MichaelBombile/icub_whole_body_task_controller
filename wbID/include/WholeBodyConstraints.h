#pragma once


#ifndef WholeBodyConstraints_H
#define WholeBodyConstraints_H

#include <iostream>
#include <cmath>
#include <stdexcept>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>

#include "wbhpidcUtilityFunctions.hpp"
#include "ConvexHullofPoints.hpp"


using namespace std;
using namespace Eigen;

typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;
typedef Eigen::Matrix<double, 6, 12> Matrix6_12d;

class WholeBodyConstraints
{
	

		double mu_hand;
		double gamma_hand;
		double deltaX_hand;
		double deltaY_hand;

		double mu_feet;
		double gamma_feet;
		double DeltaX_feet;
		double DeltaY_feet;
		double offset_x;
		//
		int n_actuatedDofs;

	public:	

		//
		VectorXd torqueConstraint_vector;
		VectorXd VelocityLimitsVector;
		VectorXd JointLimitsVector;
		//
		VectorXd torqueSaturationLimit;
		VectorXd velocitySaturationLimit;
		VectorXd maxJointLimits;
		VectorXd minJointLimits;

		MatrixXd IneqConstraintMatrix_lf;
		VectorXd IneqConstraintVector_lf;
		MatrixXd IneqConstraintMatrix_rf; 
		VectorXd IneqConstraintVector_rf;
		MatrixXd IneqConstraintMatrix_AllFeet;
		VectorXd IneqConstraintVector_AllFeet;

		// hands
		MatrixXd IneqConstraintMatrix_lh;
		VectorXd IneqConstraintVector_lh;
		MatrixXd IneqConstraintMatrix_rh; 
		VectorXd IneqConstraintVector_rh;
		MatrixXd IneqConstraintMatrix_AllHands;
		VectorXd IneqConstraintVector_AllHands;

		VectorXd jts_velocity_0;

		KineTransformations Transforms;
		//
		ConvexHullofPoints ConvHull;
		//
		Vector2d pxy_CoM_n1;
		Vector2d pxy_CoM;


		WholeBodyConstraints();
		~WholeBodyConstraints();

		// parameters initialization
		void Initialize(	 int actuatedDofs,
						VectorXd HandContactParameters, 
						VectorXd FeetContactParameters, 
						VectorXd torqueSaturationLimit_, 
						VectorXd velocitySaturationLimit_,
						VectorXd minJointLimits_,
						VectorXd maxJointLimits_);

		// 
		bool get_WbInequalityConstraints(  			 		  double	run_period_sec,
													 JointspaceStates	JtsStates_,
										    WholeBodyTaskSpaceStates	wb_TskSpStates_);

		// torque constraints
		bool get_WbTorqueConstraints();

		bool get_WbVelocityLimitsConstraints(	double run_period_sec,
												VectorXd jts_velocity);

		bool get_WbJointLimitsConstraints(   double run_period_sec,
											VectorXd jts_position,
											VectorXd jts_velocity);

		// // Contact constraints
		// // =====================
		// feet
		bool get_lfoot_ContactConstraints(Matrix3d world_R_lfoot);

		bool get_rfootContactConstraints(Matrix3d world_R_rfoot);

		// hands
		bool get_lhand_ContactConstraints(Matrix3d world_R_lhand);

		bool get_rhand_ContactConstraints(Matrix3d world_R_rhand);


		bool get_WbHandsContactConstraints(Matrix3d world_R_lhand, Matrix3d world_R_rhand);

		bool get_WbFeetContactConstraints(Matrix3d world_R_lfoot, Matrix3d world_R_rfoot);

		bool set_CoM_limits(int stance, double DeltaSupport[], Matrix4d w_H_lf, Matrix4d w_H_rf, Vector7d &ref_pose_com_w);

		bool get_CoP_constraints_matrix(Vector2d d_CoP, Matrix3d w_R_f, Eigen::Matrix<double, 2, 6>& CoP_cMx);

		void get_feet_support_points(MatrixXd PtsInFoot, Vector7d Pose_lfoot, Vector7d Pose_rfoot, 
									 MatrixXd &AllPtsInAbsFoot_, Matrix2d &W_Rot_AbsFoot, Vector2d &W_Pos_AbsFoot);

		bool getConvexHullVariables(MatrixXd PtsInFoot_, Vector7d Pose_lfoot, Vector7d Pose_rfoot,  MatrixXd &W_EdgesNormalCHull, VectorXd &DistanceEdgesCHull, Matrix2d &W_Rot_AbsFoot, Vector2d &W_Pos_AbsFoot);

		bool predict_stability(MatrixXd W_EdgesNormalCHull_, VectorXd DistanceEdgesCHull_, Vector2d W_CoM_, Vector2d W_Pos_AbsFt_);

		bool clamping_CoM_sstability(MatrixXd PtsInFoot_, Vector7d Pose_lfoot, Vector7d Pose_rfoot, Vector7d Pose_CoM, double margin_dist, Vector7d &ref_pose_com_w);
		
};

#endif // WholeBodyConstraints_H

