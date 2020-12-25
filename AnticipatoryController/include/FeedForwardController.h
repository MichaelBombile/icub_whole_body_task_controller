// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#ifndef FeedForwardController_H
#define FeedForwardController_H

#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <mutex>
#include <cstdlib>

#include <yarp/os/Time.h>

// Eigen headers 
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>
#include "wbhpidcUtilityFunctions.hpp"
#include "anticipatory_ctrl.h"

#include "ioStateManager.h"
#include "wb_InverseKinematics.h"

#include "ComRefGenerator.h"


class FeedForwardController
{
	//
	public : 

		int n_joints;
		VectorXd Prior_q;
		VectorXd Prior_qdot;
		VectorXd Prior_qddot;

		MatrixXd Mean_q;
		MatrixXd Mean_qdot;
		MatrixXd Mean_qddot;

		MatrixXd CovMx_q;
		MatrixXd CovMx_qdot;
		MatrixXd CovMx_qddot;

		MatrixXd E_Cov_I_q;
		MatrixXd E_Cov_I_qdot;
		MatrixXd E_Cov_I_qddot;

		MatrixXd E_Cov_O_q;
		MatrixXd E_Cov_O_qdot;
		MatrixXd E_Cov_O_qddot;
		
		VectorXd q_dot_hat_;
		VectorXd q_ddot_hat_;
		
		VectorXd q_star;
		VectorXd q_dot_star;
		VectorXd q_ddot_star;
		//
		string stance_ft;
		VectorXd jts_pos;
		Matrix4d W_H_B;   
	    Vector6d VB;   
	    Joints_Measurements m_joints_sensor_wb;

	    //
	    BimanualTaskSpaceTraj xi;
		VectorXd xi_star_;
		VectorXd xi_dot_star_;
		VectorXd xi_ddot_star_;
		//
		Vector7d des_X_EE[8];
		//
		double low_Sr, high_Sr;
		double low_dl, high_dl;
		double low_dr, high_dr;
		double low_TF, high_TF;

		bool DStepMaxReached;
		bool stable_prediction;
		Vector6d DeltaFootstepPose;
		VectorXd RelativeStep;
		// Vector7d pose_lhand;
	 //    Vector7d pose_rhand;
	 //    Vector7d pose_lfoot;
	 //    Vector7d pose_rfoot;
	 //    Vector7d pose_CoM;
	 //    Vector7d pose_Pelvis;
	 //    Vector7d pose_Chest;
		//
		// robot model
		// WbRobotModel&     robot_model;
		// anticipatory controller
		anticipatory_ctrl a_ctrl; 
		// wholebody inverse kinemtaics
		wb_InverseKinematics   WBIK;

		Vector3d fmmCoM;

		// // ======================================================================================
		// //  Hands - Centroidal Task model
		// // ======================================================================================
		// VectorXd Mean_hands;
		// MatrixXd Ap_hands;
		// VectorXd Mean_CoM;
		// MatrixXd Ap_CoM;

		// VectorXd Prior_HandsCoM;
		// MatrixXd Mean_HandsCoM; 
		// MatrixXd CovMx_HandsCoM;
		// MatrixXd E_Cov_Hands; 
		// MatrixXd E_Cov_CoM;
		// // ======================================================================================
		Vector2d Disturb_c_;
		Vector2d CP_star_;
		double fu_z_mgbetaW_;
		double StepMagnitude;
		double DeltaStepMax;
		//
		int l_count;
		int g_count;
		int l_count_max;
		bool cnt; 
		bool ik_run;
		bool isReachable;
		bool n_loop;
		bool reach_batch_start;
		bool fmmCoM_batch_start;
		bool aStep_batch_start;
		bool InitGuess_batch_start;
		bool aStepGlobal_batch_start;
		bool aStepGlobal_All_batch_start;
		std::string stance_foot_step;



		//
		bool Reach_Done   ;
    	bool fmmCoM_Done  ;
    	bool AntiStep_Done;
    	bool Stable_Done;
    	bool isStable;
		bool InitGuess_Done;
		bool aStepGlobal_Done;

		QPSolverOASES qpCp;
		int nWSR;

		// Com Ref Generator
		ComRefGenerator 	ComRefGen;


		//
		// data logger
	    std::ofstream OutRecord_stability_var;
	    std::ofstream OutRecord_PointsCHull;
	    std::ofstream OutRecord_NormalCHull;
	    std::ofstream OutRecord_DistanceCHull;
	    std::ofstream OutRecord_JointsPosition;

	    std::ofstream OutRecord_stability_var2;
	    std::ofstream OutRecord_PointsCHull2;
	    std::ofstream OutRecord_NormalCHull2;
	    std::ofstream OutRecord_DistanceCHull2;
	    std::ofstream OutRecord_JointsPosition2;

	    // ======================================
	    aStepLooger log_reach;
	    aStepLooger log_fmmcom;
	    aStepLooger log_astep;
	    aStepLooger log_iGuess;
	    aStepLooger log_aGstep;


	    // data logger solvers
	    // ===================


		FeedForwardController();
		~FeedForwardController();

		// void Initialize3(RobotInterface& robot_interface_, WbRobotModel& robot_model_, std::string n_data);
		void Initialize3(ioStateManager& ioSM_, WbRobotModel& robot_model_, std::string n_data);

		// bool compute_stepping_pose(	WbRobotModel& robot_model_, VectorXd Wrench_hands_star_, BimanualTaskSpaceTraj xi_, 
		// 							Vector7d des_X_EE[], Vector7d Pose_lfoot, Vector7d Pose_rfoot);
		// bool compute_stepping_pose(	WbRobotModel& robot_model_, VectorXd Wrench_hands_star_, Vector6d des_TspMotion[], 
		// 							Vector7d des_X_EE[], Vector7d Pose_lfoot, Vector7d Pose_rfoot, bool heuristic);
		bool approximate_stepping_pose(	RobotInterface& robot_interface_,  WbRobotModel& robot_model_, VectorXd Wrench_hands_star_, Vector6d des_TspMotion[], 
														Vector7d des_X_EE_[], Vector7d Pose_lfoot, Vector7d Pose_rfoot);

		bool compute_stepping_pose(	RobotInterface& robot_interface_, WbRobotModel& robot_model_, VectorXd Wrench_hands_star_, Vector6d des_TspMotion[], 
														Vector7d des_X_EE_[], Vector7d Pose_lfoot, Vector7d Pose_rfoot);

		bool heuristic_gridSeach_Step(	RobotInterface& robot_interface_, WbRobotModel& robot_model_, VectorXd Wrench_hands_star_, Vector6d des_TspMotion[], 
														Vector7d des_X_EE_[], Vector7d Pose_lfoot, Vector7d Pose_rfoot);

		bool approximate_stepping_pose2(RobotInterface& robot_interface_, WbRobotModel& robot_model_, VectorXd Wrench_hands_star_, Vector6d des_TspMotion[], 
														Vector7d des_X_EE_[], Vector7d Pose_lfoot, Vector7d Pose_rfoot);

		bool update_task_variables(Vector2d DeltaStep, Vector6d des_TspMotion[], Vector7d des_X_EE_[]);

		bool predict_stability(	WbRobotModel& robot_model_, VectorXd Wrench_hands_star_, VectorXd predicted_jts_pos, 
								Matrix4d  predicted_W_H_B, Vector7d Pose_lfoot, Vector7d Pose_rfoot);

		bool compute_anticipative_step(	WbRobotModel& robot_model_, VectorXd Wrench_hands_star_, VectorXd predicted_jts_pos, 
														Matrix4d  predicted_W_H_B, Vector7d Pose_lfoot, Vector7d Pose_rfoot);

		// ========================================
		bool Esimate_KinoDynamicStates(double dt_, Vector7d Pose_EE[], VectorXd Velo_Hands_k[], VectorXd Velo_Hands_k1[], VectorXd Wrench_Hands_);

		bool get_RobotStabilityState(double dt_, Vector7d Pose_EE[], VectorXd Velo_Hands_k[], VectorXd Velo_Hands_k1[], VectorXd Wrench_Hands_, std::string option);

		// =================================================================

		bool is_Stabilizable(VectorXd Wrench_hands_star_, Vector3d d_pos_lh, Vector3d d_pos_rh, Vector7d Pose_lfoot, Vector7d Pose_rfoot, Vector6d CM_,
									 Vector2d DeltaStep, double DeltaThetaFeet_);
		bool predict_Stabilizability(VectorXd Wrench_hands_star_, Vector3d d_pos_lh, Vector3d d_pos_rh, Vector7d pose_lfoot, Vector7d pose_rfoot, Vector6d CM_);

		Vector2d getOptimalInitCapturePoint(Vector2d Disturb_c, double fu_z_mgbetaW, Vector2d W_Pos_AbsFoot, Vector2d W_Pos_AbsHand, MatrixXd W_EdgesNormalCHull, VectorXd DistanceEdgesCHull);

		bool compute_anticipative_step2( VectorXd Wrench_hands_star_,  Vector3d d_pos_lh, Vector3d d_pos_rh,  Vector7d pose_lfoot, Vector7d pose_rfoot, Vector6d CM_);

		bool getUniformConvexSupportSize(MatrixXd W_EdgesNormalCHull_0, VectorXd DistanceEdgesCHull_0, MatrixXd &W_EdgesNormalCHull, VectorXd &DistanceEdgesCHull);

		Vector3d get_FMMCoM(WbRobotModel& robot_model_, VectorXd jts_pos, VectorXd Wrench_hands_star_, 
                                            Vector7d des_X_EE_[], Vector7d pose_lfoot, Vector7d pose_rfoot, string stanceFoot);

		std::string get_stance_foot(Vector7d pose_lfoot, Vector7d pose_rfoot, Vector2d BalPert_);
		Vector3d get_DeltaStep(Vector7d pose_lfoot, Vector7d pose_rfoot, std::string stance_ft);

		bool check_stabilizability(VectorXd Wrench_hands_star_, Vector3d d_pos_lh, Vector3d d_pos_rh, Vector7d pose_lfoot, Vector7d pose_rfoot, 
									Vector3d CoM_, Vector3d CoM_ddot, Vector3d AM_);

		VectorXd estimate_min_anticip_step(WbRobotModel& robot_model_, VectorXd jts_pos, VectorXd Wrench_hands_star_, Vector7d  des_X_EE[], Vector7d pose_lfoot, Vector7d pose_rfoot);

		bool check_reachability(WbRobotModel& robot_model_, VectorXd jts_position, Matrix4d WHB, Vector7d des_X_EE_[], string stanceFoot, Vector7d pose_lfoot, Vector7d pose_rfoot);

		bool check_reachability_batch(WbRobotModel& robot_model_, VectorXd jts_position, Vector7d des_X_EE_[], string stanceFoot, Vector7d pose_lfoot, Vector7d pose_rfoot);
		bool get_FMMCoM_batch(WbRobotModel& robot_model_, VectorXd jts_pos, VectorXd Wrench_hands_star_, Vector7d des_X_EE_[], Vector7d pose_lfoot, Vector7d pose_rfoot, string stanceFoot);
		bool estimate_min_anticip_step_batch(WbRobotModel& robot_model_, VectorXd jts_pos, VectorXd Wrench_hands_star_, Vector7d  des_X_EE_[], Vector7d pose_lfoot, Vector7d pose_rfoot);

		bool run_batch(WbRobotModel& robot_model_, VectorXd jts_pos, VectorXd Wrench_hands_star_, Vector7d des_X_EE_[], 
			   			Vector7d pose_lfoot, Vector7d pose_rfoot, string stanceFoot, bool &executing_step, VectorXd &ref_step);
		void close();

		VectorXd getInitialGuess(WbRobotModel& robot_model_, VectorXd jts_pos, VectorXd Wrench_hands_star_, 
                                      Vector7d  des_X_EE_[], Vector7d pose_lfoot, Vector7d pose_rfoot);

		VectorXd getGlobal_anticip_step( WbRobotModel& robot_model_, VectorXd jts_pos, VectorXd Wrench_hands_star_, Vector7d  des_X_EE_[], Vector7d pose_lfoot, Vector7d pose_rfoot, std::string stanceFoot);

		bool getInitialGuess_batch(WbRobotModel& robot_model_, VectorXd jts_pos, VectorXd Wrench_hands_star_, Vector7d  des_X_EE_[], Vector7d pose_lfoot, Vector7d pose_rfoot);

		bool getGlobal_anticip_step_batch(WbRobotModel& robot_model_, VectorXd jts_pos, VectorXd Wrench_hands_star_, Vector7d  des_X_EE_[], Vector7d pose_lfoot, Vector7d pose_rfoot, std::string stanceFoot);

		//
		bool aStep_Global_batch(WbRobotModel& robot_model_, VectorXd jts_pos, VectorXd Wrench_hands_star_, Vector7d des_X_EE_[], Vector7d pose_lfoot, Vector7d pose_rfoot);

		bool aGstep_run_batch(WbRobotModel& robot_model_, VectorXd jts_pos, VectorXd Wrench_hands_star_, Vector7d des_X_EE_[], 
                                             Vector7d pose_lfoot, Vector7d pose_rfoot, string stanceFoot, bool &executing_step, VectorXd &ref_step);

		Vector2d generate_Com_reference(VectorXd Disturb_cx_, VectorXd Disturb_cy_, double fu_z_mgbeta, Vector3d ffmmCOM_, Vector3d ref_Cop); //double ftstep_x, double ftstep_y);


};

#endif // FeedForwardController_H