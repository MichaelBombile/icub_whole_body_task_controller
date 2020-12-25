


#pragma once

#ifndef GraspingConstraints_H
#define GraspingConstraints_H

#include <iostream>
#include <cmath>
#include <stdexcept>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>

#include "wbhpidcUtilityFunctions.hpp"


using namespace std;
using namespace Eigen;

typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;
typedef Eigen::Matrix<double, 6, 12> Matrix6_12d;



class GraspingConstraints
{
		

	public:

	//
	// double tol_dist2contact;
	// double dist2contact_lh;
	// double dist2contact_rh;

	// hands
	MatrixXd ContactConstraintMatrix_lh;
	VectorXd ContactConstraintVector_lh;
	MatrixXd ContactConstraintMatrix_rh; 
	VectorXd ContactConstraintVector_rh;
	MatrixXd ContactConstraintMatrix_AllHands;
	VectorXd ContactConstraintVector_AllHands;

	MatrixXd ComplementaryConstraintMatrix_lh;
	MatrixXd ComplementaryConstraintMatrix_rh;

	double min_nF_lh;
	double max_nF_lh;
	double min_nF_rh;
	double max_nF_rh;

	double mu_hand	;
	double gamma_hand ;
	double deltaX_hand;
	double deltaY_hand;



	GraspingConstraints();

	~GraspingConstraints();

	void Initialize(VectorXd HandContactParameters); 

	// hands
	bool get_lhand_ContactConstraints(Matrix6d world_Xstar_lhand);

	bool get_rhand_ContactConstraints(Matrix6d world_Xstar_rhand);

	bool get_WbHandsContactConstraints(Matrix6d world_Xstar_lhand, Matrix6d world_Xstar_rhand);

	bool computeBimanualGraspMatrix(Vector7d ObjectPose_world, Vector7d lhandPose_world, Vector7d rhandPose_world, MatrixXd &GraspMxHands_);

	bool getGraspEqualityConstraints(Matrix6d Mo_world,
									 Vector6d bo_world,
									 Eigen::MatrixXd GraspMxHands_,
									 Eigen::MatrixXd Psd_GraspMxHands_,
									 Eigen::MatrixXd dotGraspMxHands_,
									 Eigen::VectorXd Velocity_hands,
									 Eigen::VectorXd ref_object_accel,
									 Eigen::VectorXd f_ext_object,
									 Eigen::MatrixXd &GraspEqualConstraint_matrix_,
									 Eigen::VectorXd &GraspEqualConstraint_vector_);

	bool getGraspInequalityConstraints(Matrix6d world_Xstar_lhand_,
									   Matrix6d world_Xstar_rhand_,
									   Eigen::MatrixXd &GrasInequalConstraint_matrix,
									   Eigen::VectorXd &GrasInequalConstraint_vector);

	// bool get_lhand_ComplementaryConstraints(Matrix6d world_Xstar_deslhand_, 
	// 										Vector7d lhandPose_world_, 
	// 										Vector7d object_lhand_grasp_pose_, 
	// 										double tol_dist2contact);

	// bool get_rhand_ComplementaryConstraints(Matrix6d world_Xstar_desrhand_, 
	// 										Vector7d rhandPose_world_, 
	// 										Vector7d object_rhand_grasp_pose_, 
	// 										double tol_dist2contact);

	// bool get_lhand_Distance2Contact(Matrix6d world_Xstar_deslhand_, 
	// 								Vector7d lhandPose_world_, 
	// 								Vector7d object_lhand_grasp_pose_);

	// bool get_rhand_Distance2Contact(Matrix6d world_Xstar_desrhand_, 
	// 								Vector7d rhandPose_world_, 
	// 								Vector7d object_rhand_grasp_pose_);

	bool get_lhand_ComplementaryConstraints(Matrix6d world_Xstar_deslhand_, double dist2contact_lh, double tol_dist2contact);

	bool get_rhand_ComplementaryConstraints(Matrix6d world_Xstar_desrhand_, double dist2contact_rh, double tol_dist2contact);

	bool get_hands_ComplementaryConstraints(Matrix6d world_Xstar_deslhand_, Matrix6d world_Xstar_desrhand_, double dist2contact_lh, double dist2contact_rh, double tol_dist2contact);


};


#endif // GraspingConstraints



// =================
// Hands
// --------------------------------------------------------------------------

