

// /////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#ifndef ioStateManager_H
#define ioStateManager_H

#include <string>
#include <iostream>
#include <fstream>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Geometry>

#include <yarp/os/RateThread.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/LockGuard.h>

#include "wbhpidcUtilityFunctions.hpp"
#include "ControllerParameters.h"
#include "RobotInterface.h"
#include "WbRobotModel.h"

using namespace std;
using namespace Eigen;

class ioStateManager : public yarp::os::RateThread 
{

	private:
		std::string RobotName;
		int 		n_actuatedDofs;
		int 		ThreadPeriod;
		double 		run_period_sec;
		// int 		Cycle_counter;

	public:

	//
	// yarp::os::Mutex m_mutex;
	//
	int 		Cycle_counter;
	bool 	switch_pelvis_chest;

	// Controller parameters
	// ----------------------
	ControllerParameters&  	ctrl_param;
	// robot interface
	RobotInterface& 		robot_interface;
	// robot model
	WbRobotModel& 			robot_model;
	// Kinematic transformations
    KineTransformations Transforms;
	//
	bool StopCtrl;
	double trqfac;

	// Forward dynamics variables
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> massMatrix;
	Eigen::VectorXd 		h_biasForces;
	Eigen::VectorXd 		gravityBiasTorques;

	// States of the robot
	JointspaceStates 		 	JtsStates;
	TaskSpaceStates 		 	TskSpStates;
	WholeBodyTaskSpaceStates	wbTS; //wb_TskSpStates;
	ContactsStates				Cont_States;

	Joints_Measurements 		m_wb_joints_sensor;
	Matrix4d  					world_H_fBase;
	Vector6d  					world_Velo_fBase;

	// Centroidal dynamics variables
	Matrix6d 	CentroidalDynamicMatrix;
	Matrix6d 	Centroidal_Wrench_Map_pelvis;
	Matrix6d 	Centroidal_Wrench_Map_lhand;
	Matrix6d 	Centroidal_Wrench_Map_rhand;
	Matrix6d 	Centroidal_Wrench_Map_lfoot;
	Matrix6d 	Centroidal_Wrench_Map_rfoot;
	Vector6d 	Gravity_Force;
	Vector6d	desired_centroidal_wrench;
	Vector3d 	CentroidalInertia_mxI_previous;
	Vector3d 	dotCentroidalInertia_mxI;

	// Jacobians
	// =======================================	
	MatrixXdRowMaj 	Jacobian_pelvis;
	MatrixXdRowMaj 	Jacobian_left_hand;
	MatrixXdRowMaj 	Jacobian_right_hand;
	MatrixXdRowMaj 	Jacobian_left_foot;
	MatrixXdRowMaj 	Jacobian_right_foot;
	MatrixXdRowMaj 	Jacobian_CoM;
	MatrixXdRowMaj 	Jacobian_Chest;
	MatrixXdRowMaj 	Jacobian_centro_momentum;

	Vector6d 	DJacobianDq_pelvis;
	Vector6d 	DJacobianDq_left_hand;
	Vector6d 	DJacobianDq_right_hand;
	Vector6d 	DJacobianDq_left_foot;
	Vector6d 	DJacobianDq_right_foot;
	Vector6d 	DJacobianDq_centro_momentum;
	Vector6d 	DJacobianDq_CoM;
	Vector6d 	DJacobianDq_Chest;

	Eigen::VectorXd q_dot;
	//=============================
	VectorXd ee_contacts; 
	// Motion
	WholeBodyTaskSpaceAcceleration  wb_des_tasks_acceleration;
	// stance foot
	std::string			stance_foot;

	Eigen::VectorXd 	Joints_cmds;
	//
	Matrix4d w_H_lfoot;
	Matrix4d w_H_rfoot;
	Matrix4d w_H_absF;		// absolute foot frame homogeneous transformation
	Vector7d w_Pose_absF; 	// absolute foot frame pose

	Vector7d w_Pose_laFTs;
	Vector7d w_Pose_raFTs;

	//
	//
    Eigen::Matrix<double, 6, 7> Jac_larm_hand;
    Eigen::Matrix<double, 6, 7> Jac_rarm_hand;

    //
    Eigen::MatrixXd		Jacobian_SoT_top6;
    Vector6d		DJacobianDq_SoT_top6;


	// joints command vector
	VectorXd Joints_pos_cmds;
	VectorXd Joints_vel_cmds;
	VectorXd Joints_pos_filtered_cmds;
	// Ports of input variables
	// =======================+
    // CoP constraints matrix
	// Eigen::Matrix<double, 2, 6> CoP_constraint_matrix_lfoot;
	// Eigen::Matrix<double, 2, 6> CoP_constraint_matrix_rfoot;

	// Type of contact support
		// constants declaration
	static const int DSP = 0;
	static const int LSP = 1;
	static const int RSP = 2;


	ioStateManager(int period, std::string robotName, int actuatedDofs, Matrix4d w_H_lf, Matrix4d w_H_rf, std::string stanceFoot, ControllerParameters&  ctrl_param_, WbRobotModel& robot, RobotInterface& robot_interface_);
	~ioStateManager();

	virtual bool threadInit();
	virtual void threadRelease();
	virtual void run();
	//
	bool UpdateRobotStates();
	bool get_Fwd_Kinematics_and_DynamicsVariables();
	bool get_dotCentroidalInertia_matrix_I();
	
	bool getJointsStates(JointspaceStates& JtsStates);
	bool getTasksSpaceStates(TaskSpaceStates& TskSpStates);
	bool UpdateStanceFoot(ContactsStates Cont_States, std::string& stance_foot);
	void SendTorqueCommands(VectorXd Tau_actuated_);
	void copy_whole_body_poses2array8(Vector7d (&wbPoses)[8]);

	bool SwitchControlModes(char c);
};

#endif // ioStateManager_H