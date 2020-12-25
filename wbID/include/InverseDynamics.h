#pragma once

#ifndef InverseDynamics_H
#define InverseDynamics_H

#include <string>
#include <iostream>
#include <fstream>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Geometry>

#include <yarp/os/RateThread.h>
// #include <yarp/os/Mutex.h>
#include <yarp/os/LockGuard.h>
// #include <wbi/wholeBodyInterface.h>


#include "wbhpidcUtilityFunctions.hpp"
#include "ControllerParameters.h"
#include "RobotInterface.h"
#include "WbRobotModel.h"
#include "ioStateManager.h"
#include "WholeBodyConstraints.h"


extern "C" {
#include "solver.h"
}

extern "C" {
#include "wb2_solver.h"
}

using namespace std;
using namespace Eigen;

// typedef Eigen::Matrix<double, 7, 1> Vector7d;
// typedef Eigen::Matrix<double, 6, 1> Vector6d;
// typedef Eigen::Matrix<double, 6, 6> Matrix6d;
// typedef Eigen::Matrix<double, 4, 4> Matrix4d;

class InverseDynamics : public yarp::os::RateThread 
{

	private:
		std::string RobotName;
		int 		n_actuatedDofs;
		int 		ThreadPeriod;
		double 		run_period_sec;
		int 		Cycle_counter;

	public:
	//
	// InputOutput State Manager
	ioStateManager&            ioSM;
	//
	// RobotInterface& robot_interface;
	//
	// yarp::os::Mutex m_mutex;

	// Controller parameters
	// ----------------------
	ControllerParameters&  	ctrl_param;
	// robot model
	WbRobotModel& 			robot_model;
	// Kinematic transformations
    KineTransformations Transforms;

    //ContactContraints contactHandler;
	WholeBodyConstraints 	WbConstraints;

	//
	bool 	StopCtrl;
	double 	trqfac;

	// Forward dynamics variables
	Eigen::Matrix<double, 	Eigen::Dynamic, Eigen::Dynamic> massMatrix;
	Eigen::VectorXd 		h_biasForces;
	Eigen::VectorXd 		gravityBiasTorques;
	Eigen::VectorXd 		bias_fcontact_torque;

	// States of the robot
	JointspaceStates 		 	JtsStates;
	TaskSpaceStates 		 	TskSpStates;
	WholeBodyTaskSpaceStates	wb_TskSpStates;
	ContactsStates				Cont_States;

	Joints_Measurements 		m_wb_joints_sensor;
	Matrix4d  					world_H_fBase;
	Vector6d  					world_Velo_fBase;

	// Centroidal dynamics variables
	// Matrix6d 	CentroidalDynamicMatrix;
	// Matrix6d 	Centroidal_Wrench_Map_pelvis;
	// Matrix6d 	Centroidal_Wrench_Map_lhand;
	// Matrix6d 	Centroidal_Wrench_Map_rhand;
	// Matrix6d 	Centroidal_Wrench_Map_lfoot;
	// Matrix6d 	Centroidal_Wrench_Map_rfoot;
	Vector6d 	Gravity_Force;
	Vector6d	desired_centroidal_wrench;
	// Vector3d 	CentroidalInertia_mxI_previous;
	// Vector3d 	dotCentroidalInertia_mxI;

	// Jacobians
	// =======================================	
	// MatrixXdRowMaj 		Jacobian_pelvis;
	// MatrixXdRowMaj 		Jacobian_left_hand;
	// MatrixXdRowMaj 		Jacobian_right_hand;
	// MatrixXdRowMaj 		Jacobian_left_foot;
	// MatrixXdRowMaj 		Jacobian_right_foot;
	// MatrixXdRowMaj 		Jacobian_CoM;
	// MatrixXdRowMaj 		Jacobian_Chest;
	// MatrixXdRowMaj 		Jacobian_centro_momentum;
	Eigen::MatrixXd		Jacobian_SoT;
	

	Eigen::VectorXd 	dotJacobianDq_SoT;

	// Vector6d 	DJacobianDq_pelvis;
	// Vector6d 	DJacobianDq_left_hand;
	// Vector6d 	DJacobianDq_right_hand;
	// Vector6d 	DJacobianDq_left_foot;
	// Vector6d 	DJacobianDq_right_foot;
	// Vector6d 	DJacobianDq_centro_momentum;
	// Vector6d 	DJacobianDq_CoM;
	// Vector6d 	DJacobianDq_Chest;


	Vector6d Weight_centroidal;
	Vector6d Weight_pelvis;
	Vector6d Weight_lhand;
	Vector6d Weight_rhand;
	Vector6d Weight_lfoot;
	Vector6d Weight_rfoot;
	//
	VectorXd  	Weight_posture;

	// Tasks
	//=============================
	// Motion
	Eigen::VectorXd 	desired_posture_acceleration;
	Eigen::VectorXd		desired_StackOfTasks;
	Vector6d 			desired_rate_centroidalMomentum;
	// Force
	TaskSpaceForces 	F_external;
	Eigen::VectorXd		Stack_of_Forces;

	// MatrixXd 			J_T_Wq_J;      //
	// VectorXd 			J_T_Wq_ddot;   // 
	WholeBodyTaskSpaceAcceleration  wb_des_tasks_acceleration;


	// stance foot
	std::string			stance_foot;

	// QP solution
	// =============================
	Eigen::VectorXd 	optimal_acceleration;
	Eigen::VectorXd 	Tau_actuated;
	Eigen::VectorXd 	optimal_feet_contact_forces;
	Eigen::VectorXd 	optimal_slack;

	Eigen::VectorXd 	Joints_cmds;
	Eigen::VectorXd 	Joints_accel_FwD;
	
	
	// CoP and weight distribution
	Eigen::Vector2d desired_CoP_lfoot;
    Eigen::Vector2d desired_CoP_rfoot;
    double desired_weight_prortion_rfoot;

 //    // CoP constraints matrix
	Eigen::Matrix<double, 2, 6> CoP_constraint_matrix_lfoot;
	Eigen::Matrix<double, 2, 6> CoP_constraint_matrix_rfoot;

	// Type of contact support
		// constants declaration
	static const int DSP = 0;
	static const int LSP = 1;
	static const int RSP = 2;


	Eigen::MatrixXd J_T_Wx_J;
	Eigen::VectorXd J_T_Wx_Xdot;

	Vector6d task_weight[7];
	Vector6d des_X_dot_EE[7];
	Eigen::VectorXd des_q_dot;
	VectorXd Weight_acceleration;
	VectorXd posture_weight;

	//
	MatrixXd inv_M;
	MatrixXd Lambda_SoT;
	MatrixXd JSoT;
	MatrixXd N_SoT;
	MatrixXd JSoT_bar;

	MatrixPseudoInverse2 PsInv;


	// InverseDynamics(int period, std::string robotName, int actuatedDofs, ControllerParameters&  ctrl_param_, WbRobotModel& robot, RobotInterface& robot_interface_, ioStateManager&  ioSM_);
	InverseDynamics(int period, std::string robotName, int actuatedDofs, ControllerParameters&  ctrl_param_, WbRobotModel& robot, ioStateManager&  ioSM_);
	~InverseDynamics();

	virtual bool threadInit();
	virtual void threadRelease();
	virtual void run();

	//
	bool getJointsLimits();
	bool UpdateRobotStates();
	bool get_Fwd_Kinematics_and_DynamicsVariables();
	bool get_dotCentroidalInertia_matrix_I();
	bool update_force_stack_of_tasks();
	bool get_WbStackOfTaskJacobian();
	bool get_ConstraintsMatrices();
	void SendTorqueCommands();
	void get_optimal_solution();
	void load_default_data();
	//
	bool getJointsStates(JointspaceStates& JtsStates);
	bool getTasksSpaceStates(TaskSpaceStates& TskSpStates);
	bool UpdateStanceFoot(ContactsStates Cont_States, std::string& stance_foot);

};

#endif // InverseDynamics_H