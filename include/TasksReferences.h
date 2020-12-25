/** Class Reference

*/


#pragma once

#ifndef TasksReferences_H
#define TasksReferences_H


#include <iostream>
#include <cmath>
#include <stdexcept>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>

#include "wbhpidcUtilityFunctions.hpp"
#include "ControllerParameters.h"
#include "RobotInterface.h"
#include "WbRobotModel.h"
#include "Object_to_Grasp.hpp"
#include "ioStateManager.h"
#include "BimanualFreeMotionController.h"
#include "BimanualCooperativeController.h"
#include "BalanceReference.h"
#include "FeedForwardController.h"

#define N_eef 2

using namespace std;
using namespace Eigen;


typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;


class TasksReferences
{
	
		int n_actuated_dofs;

	public:

		//
		// Kinematic transformations
        KineTransformations Transforms;

		// reference trajectories for the tasks
		// left hand
		bool 							isTaskReferenceActive;
		// whole body task space trajectories (acceleration, velocity and pose)
		WholeBodyTaskSpaceTrajectories 	wb_reference_TS_trajectories;
		// vector of robot posture
		Eigen::VectorXd 				reference_joints_posture;
		// Whole body task space acceleration of the end-effectors
		WholeBodyTaskSpaceAcceleration 	wb_desired_acceleration;
		// Gains of the whole body end-effectors
		WholeBodyTaskSpaceGains         wb_TS_gains;
		// gain of the posture task
		PostureGains		   			Ps_gains;

		//
		// Filter object motion
		firstOrderFilter  Filter_object_pos;  
		firstOrderFilter  Filter_object_rot;



		// 
		// Constructor
		TasksReferences();

		// Destructor
		~TasksReferences();

		//
		// void TasksReferences_Init(ControllerParameters &ctrl_param, Object_to_Grasp &object2grasp, ioStateManager &ioSM, int 	actuatedDofs,   WholeBodyTaskSpaceTrajectories 	ref_TS_trajectories_,   VectorXd  ref_joints_posture);
		void TasksReferences_Init(	WbRobotModel& robot_model_, ControllerParameters &ctrl_param, Object_to_Grasp &object2grasp, ioStateManager &ioSM, 
									WholeBodyTaskSpaceTrajectories ref_TS_trajectories_,  VectorXd  ref_joints_posture, std::string n_data);

		bool set_impedance_gain(Vector6d M_, Vector6d D_, Vector6d K_, TaskSpaceGains& TS_gains);

		bool get_desired_acceleration(  TaskSpaceStates TS_States_, TaskSpaceTrajectories  ref_TS_traj_, TaskSpaceGains   TS_gains_eef_,  Vector6d  F_external_, Vector6d  &desired_acceleration);

		bool get_desired_centroidalMomentum(TaskSpaceStates            TS_StatesCoM_,
		                                    TaskSpaceTrajectories 	   ref_Ts_traj_CoM_,
		                                    TaskSpaceGains             TS_gainsCoM_,
		                                    Vector6d                   F_external_CoM_,
		                                    Matrix6d                   CentroidalDynamicMatrix_,
		                                    Vector3d                   dotCentroidalInertia_mxI_,
		                                    Vector6d                   &desired_centroidalMomentum_dot);


		bool get_desired_acceleration_posture( 	JointspaceStates            JtsStates,
	                                            TaskSpaceStates             TS_StatesPelvis_,
	                                            TaskSpaceTrajectories 		ref_Ts_trajPelvis_,
	                                            Eigen::VectorXd             reference_joints_posture_,
	                                            TaskSpaceGains              TS_gainsPelvis_,
	                                            Eigen::VectorXd             &desired_posture_acceleration);

		bool set_posture_references(VectorXd ref_joints_posture);

		bool update_references( WholeBodyTaskSpaceTrajectories ref_TS_trajectories_, VectorXd  ref_joints_posture_);

		bool get_stack_of_motion_tasks( ioStateManager &ioSM_, 
										WholeBodyTaskSpaceTrajectories  wb_ref_TS_traj_,
										Eigen::VectorXd  				ref_jts_posture_,  
										TaskSpaceForces  				F_imp_,
										WholeBodyTaskSpaceAcceleration	&wb_des_acceleration_);

		// Object reference trajectories
		// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		bool Init_filters_object(double stime, ControllerParameters &ctrl_param);

		bool get_reference_trajectories_object(	Vector7d w_Pose_object_cur, ControllerParameters &ctrl_param, Matrix6d PropGain,  TaskSpaceTrajectories& des_States_Object);

		bool get_object_effective_wrench(  Object_to_Grasp& object2grasp_, TaskSpaceTrajectories ref_trajectories_object_,  Matrix6d object_damping_gain_, Vector6d F_external_object_, Vector6d& Desired_object_wrench);


		void moveToLegPosture( 	RobotInterface& robot_interface_,
								WbRobotModel& robot_model,
								Eigen::VectorXd Des_left_joints,
	                            Eigen::VectorXd Des_right_joints,
	                            bool sendCommands_,
	                            int nb_iter);

		void moveToLegPosture( RobotInterface& robot_interface_,
								WbRobotModel& robot_model,
								Eigen::VectorXd Des_left_joints,
                                Eigen::VectorXd Des_right_joints,
                                bool sendCommands_,
                                int nb_iter,
                                double delay_);

		// =====================================================
		void posture_management(int 		contact_confidence,
								VectorXd    jts_position,
		                        VectorXd    init_ref_jts_posture,
		                        VectorXd    old_Weight_posture,
		                        VectorXd&   new_Weight_posture,
		                        VectorXd&   ref_jts_posture);



		// ------------------------------------------------
		// Bimanual Free motion controller 
		BimanualFreeMotionController 	*FreeMotionCtrl;
		// Bimanual Cooperative controller
		BimanualCooperativeController 	CooperativeCtrl;

		Matrix6d 	Damping_UnconsHand;
		Matrix6d 	Damping_ConsHand;
		Vector6d 	hands_gains;   

		Vector6d 	Desired_object_wrench;
		Vector6d 	F_external_object;
		Vector6d 	F_imp_lh; 
		Vector6d 	F_imp_rh;
		Vector6d 	appWrench_lhand;
		Vector6d 	appWrench_rhand;

		double nu_Wh;
		double nu_Ao;

		Vector7d w_desLHand_Pose;
		Vector7d w_desRHand_Pose;
		Matrix4d cp_desH_Obj[2];

		// KineTransformations Transforms;

		// Manipulation();
		// ~Manipulation();

		void get_bimanip_references(ControllerParameters &ctrl_param, Object_to_Grasp &object2grasp, ioStateManager &ioSM, bool RRetract_, bool isLift, WholeBodyTaskSpaceTrajectories &wbTskRef);

		Vector6d get_hands_ref_acceleration(std::string hand, Matrix6d g_imp_u, Matrix6d g_imp_c, ioStateManager &ioSM_, bool wConstr);
		void get_hands_ref_impedance_forces(ioStateManager &ioSM_, bool isoImp);
		void getCurrentReferenceGraspPoints(ioStateManager &ioSM_, Vector7d w_Pose_ref, Matrix4d (&h_desH_ref)[2]);
		VectorXd get_expected_object_load(ControllerParameters &ctrl_param, Object_to_Grasp &object2grasp, ioStateManager &ioSM,  Vector7d des_lh_pose, Vector7d des_rh_pose, Vector6d ExtWrenchObj);
		void close();

		// ----------------------------------------------------

		BalanceReference        balanceRef;                 // Bimanual Manicontroller

		FeedForwardController   ff_ctrl;                // Feeforward Controller with anticopatory action


		// ---------------------------------

		Vector3d des_CoP_robot;
		Vector3d pred_CoP_robot;
		Vector3d Ref_CoP_robot;
		Vector2d ref_CoM_xy;

		VectorXd Disturb_cx_;
		VectorXd Disturb_cy_;

		Vector2d get_ref_CoM_trajectory(WbRobotModel& robot_model_, ControllerParameters &ctrl_param, ioStateManager &ioSM_, 
								VectorXd ID_WrenchesFeet, VectorXd Wrench_hands_star, Vector7d lh_pose, Vector7d rh_pose, 
								Vector3d desCoP_absF, bool RRetract_, bool &StartRelease_, int &iter_sm_);




};

#endif // TasksReferences_H


