#pragma once


#ifndef BimanualCooperativeController_H
#define BimanualCooperativeController_H

#include <string>
#include <iostream>
#include <fstream>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Geometry>

#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>

#include "wbhpidcUtilityFunctions.hpp"
#include "GraspingConstraints.h"
#include "Object_to_Grasp.hpp"
#include "ControllerParameters.h"
#include "ioStateManager.h"


extern "C" {
#include "bwc_solver.h"
}

extern "C" {
#include "cmo_solver.h"
}


using namespace std;
using namespace Eigen;

typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;
typedef Eigen::Matrix<double, 6, 12> Matrix6_12d;

class BimanualCooperativeController //: public yarp::os::RateThread 
{
	public:

		// int ThreadPeriod;
		// double run_period_sec;
		// Grasping
		// =========
		Eigen::MatrixXd GraspHessianMatrix;
		Eigen::VectorXd GraspGradientVector;
		Eigen::MatrixXd GraspEqualConstraint_matrix;
		Eigen::VectorXd GraspEqualConstraint_vector;
		Eigen::MatrixXd GraspInequalConstraint_matrix;
		Eigen::VectorXd GraspInequalConstraint_vector;
		// GRasp constriants class
		GraspingConstraints GraspConstraints;

		Eigen::MatrixXd GrasplumpedConstraints_matrix;
		Eigen::VectorXd GrasplumpedConstraints_vector;
		Eigen::MatrixXd GraspMatrixHands;
		//
		Vector7d object_lhand_grasp_pose;
		Vector7d object_rhand_grasp_pose;

		// Desired external object wrench
		Vector6d Desired_object_wrench;
		// optimal solution
		Eigen::VectorXd optimal_contact_wrench_hands;
		Eigen::VectorXd optimal_slack;
		// contact confidence indicator
		double ContactConfidence;
		//
		double tol_dist2contact;
		double dist2contact_lh;
		double dist2contact_rh;
		// compute the pseudo inverse matrix
    	MatrixPseudoInverse2 MxPsdInv;
    	// Kinematic transformations
        KineTransformations Transforms;
    	// Transformations
    	// hands
		Matrix6d world_Xstar_deslhand;
		Matrix6d world_Xstar_desrhand;
		//
		Eigen::Matrix<double, 12, 1>  Weight_hands_wrench;
		Eigen::Matrix<double, 6, 1>  Weight_hands_slack;
		Eigen::Matrix<double, 4, 1>  Hands_contact_param;

		Eigen::Matrix<double, 6,  1>  Weight_absolute_motion;
		Eigen::Matrix<double, 12, 1>  Weight_relative_motion;
		double weight_regularizaion;

		Vector7d object_pose;
		Vector7d lhandPose_world;
		Vector7d rhandPose_world;

		// mottion tasks
		// =========================================================
		Eigen::VectorXd   optimal_hands_velocity;
		Eigen::Matrix<double, 12, 1>  optimal_hands_acceleration;
		// Jacobians
		Eigen::MatrixXd				  Psd_GraspMatrixHands;
		
		Eigen::Matrix<double,  6, 12> GraspMotion_Jacobian;
		Eigen::Matrix<double, 12, 12> GraspConstraint_Jacobian;
		Eigen::Matrix<double,  6, 12> dotGraspMotion_Jacobian;
		Eigen::Matrix<double, 12, 12> dotGraspConstraint_Jacobian;
		Eigen::Matrix<double,  6, 12> GraspMotion_Jacobian_prev;
		Eigen::Matrix<double, 12, 12> GraspConstraint_Jacobian_prev;
		Eigen::Matrix<double,  6,  1> dotGraspMotion_Jacobian_Xhdot;
		Eigen::Matrix<double, 12,  1> dotGraspConstraint_Jacobian_Xhdot;
		Eigen::Matrix<double,  6, 1>  b_absolute_motion;
		Eigen::Matrix<double, 12, 1>  b_relative_motion;
		// force correction
		Vector6d wrench_correction_lh;
		Vector6d wrench_correction_rh;

		Eigen::Vector3d torque_correction_lh;
		Eigen::Vector3d torque_correction_rh;
		// 	
		Vector6d F_applied_lhand;
		Vector6d F_applied_rhand;
		Vector6d F_In_lhand;
		Vector6d F_In_rhand;

		double min_normalForce;
		double max_normalForce;
		double gain_tau_hands;
		bool EnableSaturation;
		bool TorqueCorrection;
		bool apply_wrench;

		BimanualCooperativeController();
		~BimanualCooperativeController();

		// bool Initialize(ControllerParameters &ctrl_param,
		// 				Vector7d object_pose,
		// 				Vector7d lhandPose_world,
		// 				Vector7d rhandPose_world);
		// virtual void Release();

		// virtual bool threadInit();
		// virtual void threadRelease();
		// virtual void run();

		bool getGraspKineDynVariables(	Vector7d object_pose,
										Matrix4d w_H_cp_l,
				 						Matrix4d w_H_cp_r,
										Vector7d lhandPose_world,
										Vector7d rhandPose_world);

		void computeControlWrench(Vector6d Desired_object_wrench_);

		// void computeControlWrench(	double   nu_Wrench_,
		// 							Vector7d object_pose,
		// 							Matrix4d w_H_cp_l,
		// 		 					Matrix4d w_H_cp_r,
		// 							Vector7d lhandPose_world,
		// 							Vector7d rhandPose_world,
		// 							Vector6d Desired_object_wrench_);

		void load_wrench_data(Vector6d Desired_object_wrench_);

		void load_motion_data();

		bool get_TaskJacobians(double run_period_sec);

		bool get_TaskConsistentHandsVelocity(double run_period_sec,
											 Eigen::Matrix<double, 6, 1> desired_object_velocity);

		bool get_TaskConsistentHandsAcceleration(double run_period_sec,
												 Eigen::Matrix<double, 6, 1> desired_object_acceleration,
												 Eigen::Matrix<double, 6, 1> w_velocity_lhand,
												 Eigen::Matrix<double, 6, 1> w_velocity_rhand);

		bool thresholdNormalForcesHands(double min, double max, Matrix3d w_R_h, Vector6d &Wrench_w);
		bool setMinNormalForcesHands(double min, Matrix3d w_R_h, Vector6d &Wrench_w);

		// =======================================================================================================
		bool Initialize(ControllerParameters &ctrl_param, Object_to_Grasp &GrspObj, ioStateManager &ioSM_);

		void computeControlWrench(	double   nu_Wrench_, Object_to_Grasp &GrspObj, ioStateManager &ioSM_, Vector6d Desired_object_wrench_);

		void PredictControlWrench(Object_to_Grasp &GrspObj, Vector7d lhandPose_world, Vector7d rhandPose_world, Vector6d Desired_object_wrench_, VectorXd &Hands_Wrenches);

		void check_contact_confidence(	Matrix4d w_H_cp_l, Matrix4d w_H_cp_r, Vector7d lhandPose_world, Vector7d rhandPose_world);



};

#endif // BimanualCooperativeController_H



