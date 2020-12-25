/////////////////////////////////////////////////////////////////////////////////////////////////////////////////



/** Class anticipatory_ctrl / header file
	class that that predict the stability of the robot for a given manipulation
	task and accordingly determines or not an anticipatory control action
	(stepping) to prevent the fall of the robot.
*/

#pragma once

#ifndef anticipatory_ctrl_H
#define anticipatory_ctrl_H


#include <iomanip>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <cmath>
#include <stdexcept>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>
#include "WbRobotModel.h"
#include "wbhpidcUtilityFunctions.hpp"

#include "ConvexHullofPoints.hpp"


using namespace std; 
using namespace Eigen;

class anticipatory_ctrl
{
	public:

		int nJts;
		double Beta_w;
		double mass_bot;
		// effort distribution coefficients
		double alpha_x;  // F_lfoot(0)/(F_lfoot(0)+F_rfoot(0));  
		double alpha_y;  // F_lfoot(1)/(F_lfoot(1)+F_rfoot(1));
		double alpha_z;  // F_lfoot(2)/(F_lfoot(2)+F_rfoot(2));
		// Coordination parameters
		double a_bi;
		double b_bi;

		string EE[4];

		MatrixXd PtsInFoot;

		Vector2d Delat_R_cmp;
		Vector2d DeltaZ_com;
		Vector6d Wrench_UB;
		Vector6d CM;
		Vector6d CM_dot;
		Vector3d CoM;
		Vector3d CoM_ddot;
		Vector3d CoM_dot;
		Vector3d CP;
		//
		// VectorXd q_star;
		// VectorXd q_dot_star;
		// VectorXd q_ddot_star;

		//
		VectorXd jts_pos_star;
		Matrix4d WHB_star;
		//
		VectorXd jts_vel_star;
		Vector6d VB_star;
		//
		Vector7d Pose_EE[4];
		Vector7d Pose_CoM;
		Vector4d Pos_eq_foot;
		//
		Eigen::MatrixXd massMatrix;
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Jacobian_CM;
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Jacobian_EE[4]; 

		Eigen::MatrixXd Jac_2hands;
		Eigen::MatrixXd Jac_2feet;
		Eigen::MatrixXd Jac_bi;

		Vector6d DJacobianDq_EE[4];
		Vector6d DJacobianDq_CM;

		//
		Eigen::Matrix<double, 12, 1>  Wrench_hands_start;
		Eigen::Matrix<double, 6, 12>  WrenchMap_UP;
		Eigen::Matrix<double, 12, 12> C_hands;

		MatrixXd E_Cov_I;		// Expected Input covariance
		MatrixXd E_Cov_O;  	 	// Expected Output covariance

		// MatrixXd AllPtsInAbsFoot(2,np_f*2); 	// coordinates of the point in the absolute frame
		VectorXd xi_dot_star;
		VectorXd xi_ddot_star;
		VectorXd q_dot_hat;
		VectorXd q_ddot_hat;

		MatrixXd Weight_feet;

		MatrixXd invSigma_xidot;
		MatrixXd invSigma_xiddot;
		MatrixXd invSigma_qdot;
		MatrixXd invSigma_qddot;

		// W_EdgesNormalCHull_;
		// W_CP_;
		// W_Pos_AbsFt_;
		// W_Pertub_;

		std::string stance_0;
		std::string stance_1;

		double theta_stance;
		double theta_swing;
		double Delta_theta_feet;
		double Dtheta_feet_max;
		double DyFeet_min;
		double lengthStep;
		double lengthStep_max;
		double stepIncrease;
		Vector6d step_values; // DeltaStep(0), DeltaStep(1), Delta_theta_feet, stance_ind(0) (right), stance_ind(1) (left), 0.0 (stability);

		// kinematic transformation 
		KineTransformations Transforms;
		// Pseudo Inverser
		MatrixPseudoInverse2 Psdinv;

		// Convex hull of points
		ConvexHullofPoints ConvHull;

		// data logger
	    // std::ofstream OutRecord_stability_var;
	    // std::ofstream OutRecord_PointsCHull;
	    // std::ofstream OutRecord_NormalCHull;
	    // std::ofstream OutRecord_DistanceCHull;

	    // ======================================================================================
		//  Hands - Centroidal Task model
		// ======================================================================================
		VectorXd Mean_hands;
		MatrixXd Ap_hands;
		VectorXd Mean_CoM;
		MatrixXd Ap_CoM;

		VectorXd Prior_HandsCoM;
		MatrixXd Mean_HandsCoM; 
		MatrixXd CovMx_HandsCoM;
		MatrixXd E_Cov_Hands; 
		MatrixXd E_Cov_CoM;
		// ======================================================================================
		Matrix2d W_Rot_AbsFoot_;
		Vector2d W_Pos_AbsFoot_ ;
		MatrixXd W_EdgesNormalCHull_;
		VectorXd DistanceEdgesCHull_;
		Vector2d CP_star_;
		double cost_step;
		//=========================================================================================


		anticipatory_ctrl();
		~anticipatory_ctrl();

		void Initialize(WbRobotModel& robot_model_, std::string n_data);

		bool update_model(WbRobotModel& robot_model_,  VectorXd jts_pos_star_, Matrix4d WHB_star_);
		bool get_instantaneous_IK(	WbRobotModel& robot_model_, VectorXd xi_dot_star, VectorXd q_dot_hat, VectorXd xi_ddot_star, VectorXd q_ddot_hat, 
									VectorXd &q_dot_star, VectorXd &q_ddot_star);
		bool get_expected_posture(VectorXd Prior_, MatrixXd Mean_q, MatrixXd CovMx, VectorXd xi_star_, VectorXd &q_star_, MatrixXd &E_Cov_I, MatrixXd &E_Cov_O);
		bool get_expected_posture(VectorXd Prior_, MatrixXd Mean_q, MatrixXd CovMx, VectorXd xi_star_, double &prob, VectorXd &q_star_, MatrixXd &E_Cov_I, MatrixXd &E_Cov_O);
		// bool get_expected_covariance(VectorXd Prior, double CovMx[][][], MatrixXd &E_Cov_I, MatrixXd &E_Cov_O);
		bool get_centroidal_variables(WbRobotModel& robot_model_, VectorXd jts_pos_star_, Matrix4d WHB_star_, VectorXd q_dot_star_, VectorXd q_ddot_star_);
		Vector2d compute_posture_perturbation(VectorXd Wrench_hands_start);
		Vector2d compute_posture_perturbation2(double Beta_w_, Vector6d Wrench_UB_);

		bool compute_delta_disturb(VectorXd Wrench_hands_start, Vector2d DeltaStep_, Vector2d &DeltaDist_);
		Vector2d compute_capture_point(VectorXd Wrench_hands_start);

		Vector3d compute_capture_point2(double Beta_w_, Vector3d pCoM, Vector3d vCoM);
		bool predict_stability(MatrixXd W_EdgesNormalCHull_, VectorXd DistanceEdgesCHull_, Vector2d W_CP_, Vector2d W_Pos_AbsFt_, Vector2d W_Pertub_);
		bool predict_stabilityCP(MatrixXd W_EdgesNormalCHull_, VectorXd DistanceEdgesCHull_, Vector2d W_CP_, Vector2d W_Pos_AbsFt_, Vector2d W_Pertub_);
		bool predict_stabilityZMP(MatrixXd W_EdgesNormalCHull_, VectorXd DistanceEdgesCHull_, Vector3d pCoM, Vector3d aCoM, Vector2d W_Pos_AbsFt_, Vector2d W_Pertub_, double beta_w_);


		void get_feet_support_points(Vector7d Pose_lfoot, Vector7d Pose_rfoot, Vector2d DeltaStep, MatrixXd &AllPtsInAbsFoot_, Matrix2d &W_Rot_AbsFoot, Vector2d &W_Pos_AbsFoot);


		Vector6d compute_stepping_values(Vector7d Pose_lfoot, Vector7d Pose_rfoot, VectorXd Wrench_hands_star);
		//
		void getJointFeetSsupportPoints_2(	string stance_1, double theta_swing_0, double des_DyFeet, 
													Vector7d Pose_lfoot, Vector7d Pose_rfoot, Vector2d DeltaStep, 
												    MatrixXd &AllPtsInAbsFoot_, Matrix2d &W_Rot_AbsFoot, Vector2d &W_Pos_AbsFoot);

		void get_feet_support_points_C(	Vector7d Pose_lfoot, Vector7d Pose_rfoot, Vector2d DeltaStep, double DeltaThetaFeet_,
										MatrixXd &AllPtsInAbsFoot_, Matrix2d &W_Rot_AbsFoot, Vector2d &W_Pos_AbsFoot);

		bool LoadDataFromFile(std::string file_name, VectorXd &data_all_val);

		bool Load_gmm_param(std::string file_name[], int dataDim, int nbStates, VectorXd &Priors_, MatrixXd &Means_, MatrixXd &Covars_);
		bool Load_gmm_param2(std::string file_name[], VectorXd &Priors_, MatrixXd &Means_, MatrixXd &Covars_);

		double gaussPDF(VectorXd Data, VectorXd Mu, MatrixXd Sigma);

		bool set_IIK_weights(MatrixXd E_Cov_O_dot, MatrixXd E_Cov_I_dot, MatrixXd E_Cov_O_ddot, MatrixXd E_Cov_I_ddot);

		// ===================================================================
		bool Load_PCA_model(std::string file_name[], VectorXd &mean_, MatrixXd &Ap_);
		bool pca_projection(VectorXd mean_, MatrixXd Ap_, VectorXd in_data, VectorXd &out_proj_data);
		bool pca_reconstruction(VectorXd mean_, MatrixXd Ap_, VectorXd in_proj_data, VectorXd &out_data);
		bool get_GMR_IO(VectorXd Prior, MatrixXd Mean_gmm, MatrixXd CovMx, VectorXd Input_, VectorXd &Output_, MatrixXd &E_Cov_I, MatrixXd &E_Cov_O);
		// bool get_expected_centroidal_task(VectorXd hands_task, VectorXd &pos_CoM_hat, VectorXd &cntr_mtm_hat);

		bool get_expected_centroidal_task(VectorXd hands_task, Vector3d &pos_CoM_hat, Vector6d &cntr_mtm_hat, MatrixXd &E_Cov_Hands_, MatrixXd &E_Cov_CoM_);

		// bool Esimate_KinoDynamicStates(double dt_, Vector7d Pose_EE[], VectorXd Velo_Hands_k[], VectorXd Velo_Hands_k1[], VectorXd Wrench_Hands_);

		// New function related associated with stabilizability
		// =======================================================
		bool compute_perturbation_terms(VectorXd Wrench_hands_star_, Vector3d d_pos_lh, Vector3d d_pos_rh, Vector6d CM_, Vector2d &Disturb_c, double &fu_z_mgbetaW);
		bool compute_perturbation_terms2(VectorXd Wrench_hands_star_, Vector3d d_pos_lh, Vector3d d_pos_rh, Vector3d AM_, Vector3d CoM_ddot, 
														Vector2d &Disturb_c, double &fu_z_mgbetaW);

		bool getSupportConstraints(Vector7d pose_lfoot, Vector7d pose_rfoot, Vector2d DeltaStep, double DeltaThetaFeet_, 
					  				MatrixXd &W_EdgesNormalCHull, VectorXd &DistanceEdgesCHull, Vector2d &W_Pos_AbsFoot);

		double getSteppingCost(Vector2d CP_star, Vector2d DeltaStep, double DeltaThetaFeet_);

		double getSteppingCost2(Vector2d DeltaStep, double DeltaThetaFeet_);

		Vector7d getSlidingFootReference(Vector2d Disturb_c_, double fu_z_mgbetaW_, Vector3d CoM_, Vector7d pose_lfoot, Vector7d pose_rfoot, string stanceFoot, double stpMagn);

		Vector7d getSlidingFootReference2(Vector2d Disturb_c_, double fu_z_mgbetaW_, Vector3d CoM_, 
										  Vector7d pose_lfoot, Vector7d pose_rfoot, 
										  Vector7d pose_lf, Vector7d pose_rf, string stanceFoot, double stpMagn);

		Vector7d getInitGuessFootReference(	Vector2d Disturb_c_, double fu_z_mgbetaW_, Vector7d pose_lfoot, Vector7d pose_rfoot, string stanceFoot);

		
};

#endif // anticipatory_ctrl_H