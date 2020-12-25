
// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once


#ifndef ControllerParameters_H
#define ControllerParameters_H

#include <string>
#include <iostream>
#include <fstream>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Geometry>

// #include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>

#include "wbhpidcUtilityFunctions.hpp"

using namespace std;
using namespace Eigen;

typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;
typedef Eigen::Matrix<double, 6, 12> Matrix6_12d;



class ControllerParameters // : public yarp::os::RateThread 
{
	
	public:

		// Module name
		std::string 		ModuleName;
		std::string 		RobotName;

		// number of actuated joints
		int actuatedDofs;

		int period_step;
		// initial pid_gains legs
		Joints_PIDs init_legs_pid;
		Joints_PIDs step_legs_pid;
		// gain posture
		double gains_posture[5];
		// boolean variables to change states
		bool writeCommands;
		bool runtimeCommands;
		bool OrientChest;
		bool apply_wrench;
		bool Task_Hierarchy_Active;
		bool inputPortsActive; 				// activation for inverse dynamics inputs ports
		bool AllPositionArmsTorqueMode;  	// to set all other body part in positon mode and the arms in torque mode
		bool no_tracking_object; 
		bool AccelCommand;
		bool init_posture_hands;  			// send the hands at a desired initial posture
		bool PositionCommands;
		bool CmdsFromWBID;

    	//
		torqueControlledJoints   main_joints_list;				// struct of array of main joints
		std::vector<std::string> list_of_considered_joints;		// list of considered torque controlled joints
		//

		// /////////////////////////////////////////////////////////////////////// //
		//                           Inverse dynamics Parameters                   //
		// //////////////////////////////////////////////////////////////////////////

		// Contact parameters
		Eigen::Matrix<double, 4, 1> Hands_contact_param;
		Eigen::Matrix<double, 5, 1> Feet_contact_param;

		// characteristics of feet support
		double DeltaSupport[4];  //[0] Delta x forward  limit; //[1] Delta x backward limit; 
								 //[2] Delta y left  limit; //[3] Delta y right limit

		// QP Weight
		Eigen::VectorXd Weight_accel;						// weight on acceleration
		Eigen::VectorXd Weight_tau; 						// weight on torque
		Eigen::VectorXd Weight_F_contact;					// weight on contact force
		Eigen::VectorXd Weight_posture;						// weight of posture task

		Eigen::VectorXd StartingPosture;
		//
		// Weight for Motion and Force tasks
		Eigen::VectorXd Weight_SoT;  						// weight stack of task
		Vector6d Weight_lfoot; 								// weight for the left foot
		Vector6d Weight_rfoot; 								// weight for the right foot
		Vector6d Weight_lhand; 								// weight for the left hannd
		Vector6d Weight_rhand; 								// weight for the right hannd
		Vector6d Weight_centroidal; 	    				// weight for the centroidal momentum
		Vector6d Weight_Chest;
		Vector6d Weight_pelvis;								// weight_chest 

		// Velocity and torque saturation limits
		Eigen::VectorXd velocitySaturationLimit;
		Eigen::VectorXd torqueSaturationLimit;

		// //////////////////////////////////////////////////////////////////////////
		std::string 		Pelvis_port_name;


		// /////////////////////////////////////////////////////////////////////// //
		//                           Manipulation tasks                  		   //
		// //////////////////////////////////////////////////////////////////////////

		// Gains for the manipulation tasks
		// ================================
		// free-motion phase
		Eigen::Matrix3d virtualObjectScaling;
		BimanualGains  bimanip_gains;
		Eigen::Vector3d hand_offset[N_eef];
		double alpha_manip_filter;
		double a_bi;
		double b_bi;

		Matrix4d des_lh_H_lf;
		Matrix4d des_rh_H_lf;

		Vector7d des_lh_Pose_af_1;
		Vector7d des_rh_Pose_af_1;
		Vector7d des_lh_Pose_af_2;
		Vector7d des_rh_Pose_af_2;
		Vector7d des_lh_Pose_af_3;
		Vector7d des_rh_Pose_af_3;
		VectorXd q_ini;

		Vector6d max_reaching_velocity;

		//
		double tol_dist2contact;
		// constrained phase
		double weight_regularizaion;
		Eigen::Matrix<double, 12, 1> Weight_hands_accel;
		Eigen::Matrix<double, 12, 1> Weight_hands_wrench;
		Eigen::Matrix<double, 6, 1>  Weight_hands_slack; 
		Eigen::Matrix<double, 6, 1>  Weight_absolute_motion;
		Eigen::Matrix<double, 12, 1> Weight_relative_motion;
		//
		double min_normalForce;
		double max_normalForce;
		double gain_tau_hands;
		bool EnableSaturation;
		bool TorqueCorrection;


		// object
		Matrix6d K_object;
		Matrix6d D_object;

		// pole and gain for the filter
		double filter_gain_object_pos ;
		double filter_gain_object_rot ;
		// 
		double alpha_object_filter_pos;
		double alpha_object_filter_rot;

		// ////////////////////////////////////////////////////////////////////////////////// //
		//                    Motion parameters for initial and final posture                 //
		// ////////////////////////////////////////////////////////////////////////////////// //
		double AccelParam_init[6];
		double SpeedParam_init[6];
		double AccelParam_final[6];
		double SpeedParam_final[6];

		Vector6d i_jts_posture_lleg;
		Vector6d i_jts_posture_rleg;
		Vector6d f_jts_posture_lleg;
		Vector6d f_jts_posture_rleg;

		// initial robot posture
		// -----------------------
	    Eigen::VectorXd init_joints_config;
	    //
	    std::string select_object;
	    //
	    Matrix4d object_H_Markers;

		// ////////////////////////////////////////////////////////////////////////////////////
		// 				Object parameters
		// ////////////////////////////////////////////////////////////////////////////////////
		std::string     object_name;
		std::string     object_port_name;
		double          object_mass;
		Eigen::Matrix3d object_inertia; // Jo_
		ObjectDimension object_dimensions;
		// 
		Vector7d        object_Pose_Gpoints[N_eef];
		int RobotIndex;
		VectorXd    	des_orientation_hands;
		// 
		// ////////////////////////////////////////////////////////////////////////////////////
		// 				Center of pressure and weight distribution
		// ////////////////////////////////////////////////////////////////////////////////////
		Vector2d weight_cop_lfoot;
		Vector2d weight_cop_rfoot;
		double   coeff_weight_distribution;
		Eigen::Matrix<double, 5, 1> Weight_CoP;
		// parameters for the CoP reference Regulation 
		double wu_cop;
		double wc_cop;
		double w_cop;  // [0,1]


		Eigen::Vector2d desired_CoP_lfoot;
        Eigen::Vector2d desired_CoP_rfoot;
        double desired_weight_prortion_rfoot;

        Matrix3d desRot_icubPelvis_W;

        MatrixXd PtsInFoot;

        // CoM gains
        Vector6d vM_CoM_c;
		Vector6d vD_CoM_c;
		Vector6d vK_CoM_c;
		Vector6d vM_CoM_s;
		Vector6d vD_CoM_s;
		Vector6d vK_CoM_s;

		//
		VectorXd Weight_acceleration;
		VectorXd posture_weight;
		VectorXd posture_weight_retract;
		VectorXd posture_weight_grasp;
		Vector6d task_weight[7];
		Vector6d task_weight_retract[7];

		double epsil_reach;

		

		
		ControllerParameters();
		~ControllerParameters();

		bool Initialize(std::string robotName, std::string moduleName, int n_actuatedDofs, bool PositionMode_);
		bool getVelocityLimits(int actuatedDofs, double velo_limits[], Eigen::VectorXd &velocitySaturationLimit);
		bool getTorqueLimits(int actuatedDofs, double torque_limits[], Eigen::VectorXd &torqueSaturationLimit);
		bool getPostureWeights(int n_actuatedDofs, double gains_posture[], Eigen::VectorXd &Weight_posture);

		// 
		bool get_list_of_considered_joints(std::vector<std::string> &list_of_considered_joints);

};

#endif // ControllerParameters_H

