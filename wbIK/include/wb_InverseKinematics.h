// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** Class wb_InverseKinematics

*/

#pragma once

#ifndef wb_InverseKinematics_H
#define wb_InverseKinematics_H

#include <string>
#include <fstream>
#include <iostream>
#include <cmath>
#include <stdexcept>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <yarp/os/Mutex.h>
#include <yarp/os/LockGuard.h>
#include <yarp/os/Time.h>

#include "WbRobotModel.h"
#include "wbhpidcUtilityFunctions.hpp"
#include "ConvexHullofPoints.hpp"
#include "QPOasesSolver.hpp"



extern "C" {

	#include "wbIK1_solver.h"   // velocity
	#include "wbIK2_solver.h"
	#include "wbIK3_solver.h"
	#include "wbIK4_solver.h"
	#include "legIK_solver.h"
	#include "CoMclp_solver.h"
}


using namespace std;
using namespace Eigen;

// ====================================================================================================
// INVERSE KINEMACIS
// ====================================================================================================
class wb_InverseKinematics
{	

	public: //private:

		int nJts;
		int count_max;
		double epsilon;
		double virtual_sampTime;
		double virtual_gain;
		VectorXd minJointLimits;
		VectorXd maxJointLimits;
		VectorXd maxVeloLimit;
		int n_task;
		Vector6d gain_vector;  
		Vector6d gain_vector_0;  
		Vector6d var_gain;  
		double alpha_g;

		double coef_grad;
		VectorXd virt_jts_pos_0;  	// joint position previous step
		// Posture
		VectorXd virt_jts_pos;		// joint position
		VectorXd virt_jts_vel;		// joint velicity
		Matrix4d virt_WHB;			// floating base pose
		Vector6d virt_VB;			// floating base velocity

		string EE[8];				// end-effector body or link name
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Jacobian_EE[8];	// Jacobian of EE
		Vector7d Pose_EE[8];
		Vector6d error_pose[8];
		double error_pos_norm[8];
		double error_ori_norm[8]; 
		MatrixXd m_massMatrix;
		
		VectorXd q_0; 				// desired joint posture (resting posture)
		VectorXd q_dot_star;
		VectorXd q_dot_star_0;
		VectorXd des_q_dot;
		double alpha_q;					// stifness gain
		double beta_q;					// damping gain
		VectorXd Weight_velocity;
		VectorXd Weight_slack;
		
		Eigen::MatrixXd		J_SoT;
		Vector7d des_X_EE[8]; 		// desired end-effector pose
		Vector6d des_X_dot_EE[8];	// desired end-effector acceleration
		VectorXd X_dot_SoT;		// desired task acceleration;
		VectorXd X_dot_SoT_0;		// desired task acceleration;
		VectorXd hard;				// setter of herd constraints
		VectorXd freeze;
		VectorXd des_jts_pos;
		Matrix4d des_WHB;
		//
    	MatrixXd PtsInFoot;
		KineTransformations Transforms; 
    	ConvexHullofPoints ConvHull;		// convex hull of the feet contact points
    	MatrixPseudoInverse2 pinv;
		//
		QPOasesSolver 		QPOsSolver;
		//
		// yarp::os::Mutex m_mutex;

		// ==================================================
		double regularization;
		Vector6d task_weight[8];
		Vector6d task_weight_reach[8];
		Vector6d task_weight_reach_v[8];
		Vector6d task_weight_default[8];
		Vector6d task_weight_fmmcom[8];
		Vector6d task_weight_aStep[8];
		Vector6d task_weight_legIK[8];
		Vector6d task_weight_retract[8];

		VectorXd posture_weight;
		VectorXd posture_weight_default;
		VectorXd posture_weight_reach;
		VectorXd posture_weight_retract;

		MatrixXd Jacobian_posture;

		MatrixXd J_T_Wx_J;
		VectorXd J_T_Wx_Xdot;

		bool  isFreeze;

		// MatrixXd W_EdgesNormalCHull; 
	 //    VectorXd DistanceEdgesCHull;
		MatrixXd PointsCHull;
	    Matrix2d W_Rot_AbsFoot;
	    Vector2d W_Pos_AbsFoot;  
	    Vector2d pxy_CoM; 
	    Vector2d pxy_CoM_n1; 

	    //
	    bool isCoMClamping;
		double max_reach;
		Vector2d reach_tol;

		//
		MatrixXd Jac_lf;
		MatrixXd Jac_rf;
		MatrixXd Jac_com;
		MatrixXd Jl_T_Wx_Jl;
		VectorXd Jl_T_Wx_Xdot;
		VectorXd q_dot_star_legIK;

		//
		bool ik_cont;
		bool legIK_Done;
		bool legIK_batch_start;
		int l_count;
		int g_count;
		int l_count_max;
		int count_max_legIK;
		



		// data logger
		// ==========================
    	std::ofstream Out_joints_pos;
    	std::ofstream Out_joints_vel;


		wb_InverseKinematics();
		~wb_InverseKinematics();

		void InitializeWBIK(WbRobotModel& robot_model_,  VectorXd jts_position, Matrix4d WHB, double gain_, int count_max_, double epilon_, double step_);

		// bool update_model(WbRobotModel& robot_model_, VectorXd q_ddot_star_);
		bool update_model(WbRobotModel& robot_model_, VectorXd q_dot_star_, Vector7d pose_lfoot, Vector7d pose_rfoot, string stanceFoot);
		bool solve_QPcvxgen(WbRobotModel& robot_model_, Vector7d pose_lfoot, Vector7d pose_rfoot, string stanceFoot);
		// bool solve_QPoasesInit(WbRobotModel& robot_model_, Vector7d pose_lfoot, Vector7d pose_rfoot, string stanceFoot);
		// bool solve_QPoases(WbRobotModel& robot_model_, Vector7d pose_lfoot, Vector7d pose_rfoot, string stanceFoot);
		bool solve_QPoasesInit(WbRobotModel& robot_model_, Vector7d pose_lfoot, Vector7d pose_rfoot, string stanceFoot, MatrixXd &W_EdgesNormalCHull, VectorXd &DistanceEdgesCHull);
		bool solve_QPoases(WbRobotModel& robot_model_, Vector7d pose_lfoot, Vector7d pose_rfoot, string stanceFoot, MatrixXd &W_EdgesNormalCHull, VectorXd &DistanceEdgesCHull);
		

		bool CvxgenSolveQP_Velo(WbRobotModel& robot_model_, Vector7d pose_lfoot, Vector7d pose_rfoot, string stanceFoot);
		bool load_wbIK_PoseTask();
		bool load_wbIK_PoseTask_as(string stanceFoot);
		bool load_wbIK_VeloTask(Vector6d des_Velo_EE_[]);
		bool load_wbIK_VeloTask2(Vector6d des_Velo_EE_[], VectorXd des_jts_Velo);


		// bool get_wb_IK(WbRobotModel& robot_model_, VectorXd jts_position, Matrix4d WHB, Vector7d des_X_EE_[]);
		bool get_wb_IK(WbRobotModel& robot_model_, VectorXd jts_position, Matrix4d WHB, Vector7d des_X_EE_[], string stanceFoot, Vector7d pose_lfoot, Vector7d pose_rfoot);
		VectorXd get_wbIK_velo(WbRobotModel& robot_model_, VectorXd jts_position, Matrix4d WHB, Vector6d des_Velo_EE_[], string stanceFoot, Vector7d pose_lfoot, Vector7d pose_rfoot);
		Vector6d get_hands_errors();
		Matrix4d get_fbase_pose(WbRobotModel& robot_model_, Vector7d pose_lfoot, Vector7d pose_rfoot, std::string stanceFoot, VectorXd jts_pos_);

		void get_feet_support_points(MatrixXd PtsInFoot, Vector7d Pose_lfoot, Vector7d Pose_rfoot, 
									 MatrixXd &AllPtsInAbsFoot_, Matrix2d &W_Rot_AbsFoot, Vector2d &W_Pos_AbsFoot);

		bool getConvexHullVariables(MatrixXd PtsInFoot_, Vector7d Pose_lfoot, Vector7d Pose_rfoot,  
							MatrixXd &W_EdgesNormalCHull, VectorXd &DistanceEdgesCHull, Matrix2d &W_Rot_AbsFoot, Vector2d &W_Pos_AbsFoot);

		bool predict_stability(MatrixXd W_EdgesNormalCHull_, VectorXd DistanceEdgesCHull_, Vector2d W_CoM_, Vector2d W_Pos_AbsFt_);

		bool solve_QPcvxgen(WbRobotModel& robot_model_, Vector7d pose_lfoot, Vector7d pose_rfoot, string stanceFoot, MatrixXd &W_EdgesNormalCHull, VectorXd &DistanceEdgesCHull);

		//
		bool check_reachability(WbRobotModel& robot_model_, VectorXd jts_position, Matrix4d WHB, Vector7d des_X_EE_[], string stanceFoot, Vector7d pose_lfoot, Vector7d pose_rfoot);

		//
		Vector3d get_wb_fmmcom(WbRobotModel& robot_model_, VectorXd jts_pos, Vector7d des_X_EE_[], string stanceFoot, 
								Vector7d lfoot_pose, Vector7d rfoot_pose, Vector2d Disturb_c_, double fu_z_mgbetaW_);

		bool update_stepping_weight(std::string stance_ft_);

		bool getTaskCompletionState();

		void getFeetSelfCollisionAvoidanceConstraints(string stance, double Ly_min, MatrixXd Jac_lf, MatrixXd Jac_rf, 
														Vector7d pose_lfoot, Vector7d pose_rfoot, MatrixXd &Mssp, VectorXd &dssp);

		bool solve_QPcvxgen3(WbRobotModel& robot_model_, Vector7d pose_lfoot, Vector7d pose_rfoot, string stanceFoot, MatrixXd &W_EdgesNormalCHull, VectorXd &DistanceEdgesCHull);

		//
		bool update_model_legsIK(WbRobotModel& robot_model_, VectorXd q_dot_star_, Vector7d pose_lfoot, Vector7d pose_rfoot, string stanceFoot);
		bool solve_QP_legIK(WbRobotModel& robot_model_, Vector7d pose_lfoot, Vector7d pose_rfoot, string stanceFoot, MatrixXd &W_EdgesNormalCHull, VectorXd &DistanceEdgesCHull);
		bool get_legIK(WbRobotModel& robot_model_, VectorXd jts_position, Matrix4d WHB, Vector7d des_X_EE_[], string stanceFoot, Vector7d pose_lfoot, Vector7d pose_rfoot);
		//
		bool get_legIKCompletionState();
		//
		bool get_legIK_batch(WbRobotModel& robot_model_, VectorXd jts_position, Matrix4d WHB, Vector7d des_X_EE_[], string stanceFoot, Vector7d pose_lfoot, Vector7d pose_rfoot);
		//
		bool solve_QPcvxgen4(WbRobotModel& robot_model_, Vector7d pose_lfoot, Vector7d pose_rfoot, string stanceFoot, 
										   MatrixXd &W_EdgesNormalCHull, VectorXd &DistanceEdgesCHull, Vector2d Disturb_c, double fu_z_mgbetaW);

		VectorXd get_wbIK_velo2(WbRobotModel& robot_model_, VectorXd jts_position, Matrix4d WHB, Vector6d des_Velo_EE_[], VectorXd jts_velo,
											string stanceFoot, Vector7d pose_lfoot, Vector7d pose_rfoot, bool retract);

		Vector3d ConstrainCoM(Vector7d Pose_lfoot, Vector7d Pose_rfoot, Vector3d CoM);

};

#endif // wb_InverseKinematics