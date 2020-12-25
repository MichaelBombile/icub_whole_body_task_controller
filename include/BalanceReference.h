#pragma once

#ifndef BalanceReference_H 
#define BalanceReference_H

#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/SVD>
#include <Eigen/LU>

// #include <qpOASES.hpp>
#include "wbhpidcUtilityFunctions.hpp"

#include "WbRobotModel.h"
#include "ioStateManager.h"

#include "StepsGenerator.h"
#include "IK_legs_Solver.hpp"

extern "C" {
	#include "cop_solver.h"
}


// USING_NAMESPACE_QPOASES

using namespace std;
using namespace Eigen;

// ===================================================================================
/*
 * BalanceReference : This class encodes a function that compute reference CoM and CoP
 *
*/
// ===================================================================================

class BalanceReference
{
	public :


	MatrixXd PtsInFoot;  		// PtsInFoot.resize(2,4);
	// Matrix2d W_Rot_AbsFoot; 
	// Vector2d W_Pos_AbsFoot;

	// int nWSR;
	// QPSolverOASES qpCoM;
	// QPSolverOASES qpCoP;



	BalanceReference();
	~BalanceReference();

	void Initialize(WbRobotModel& robot_model_, ControllerParameters &ctrl_param, ioStateManager &ioSM_);

	Vector3d ConstrainCoM(WbRobotModel& robot_model_, ioStateManager &ioSM_);
	Vector3d ConstrainCoP(WbRobotModel& robot_model_, ioStateManager &ioSM_, Vector3d CoP);

	Vector3d get_reference_CoP(WbRobotModel& robot_model_, ControllerParameters &ctrl_param, ioStateManager &ioSM_, Vector3d CoP, Vector3d des_CoP);
	Vector3d get_cop_from_wrench(VectorXd Wch_);
	Vector3d get_robot_feet_cop(ioStateManager &ioSM_, VectorXd Wl_, VectorXd Wr_);


	// =================================================================================
	StepsGenerator  step_ctrl;              // SteppingController
	IK_legs_Solver  IK_legs;                // Inverse kinemeatics of the legs

	Matrix4d 		rbase_H_des_lfoot;
	Matrix4d 		rbase_H_des_rfoot;
	Vector6d 		lleg_ik_jts;
	Vector6d 		rleg_ik_jts;

	void make_step(ioStateManager &ioSM_);
	void get_leg_inverseKin(WbRobotModel& m_robot_model_, ioStateManager &ioSM_, Matrix4d rbase_H_des_lfoot_, Matrix4d rbase_H_des_rfoot_, VectorXd &jts_cmds);

	void generate_step_commands(WbRobotModel& m_robot_model_, ioStateManager &ioSM_, bool &SwitchStep, bool &executing_step, bool &StepCompleted, 
							    int &stepCount, int nSampleStep, std::string &stance_ft, Vector3d StepFoot_1, double d_theta_torso_pitch_,  VectorXd &Joints_pos_cmds);

};

#endif // BalanceReference_H


