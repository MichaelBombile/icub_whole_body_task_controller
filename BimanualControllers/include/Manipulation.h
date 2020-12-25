

/** Class Manipulation

*/
#pragma once

#ifndef Manipulation_H
#define Manipulation_H


#include "ioStateManager.h"
#include "Object_to_Grasp.hpp"
#include "TasksReferences.h"
#include "BimanualFreeMotionController.h"
#include "BimanualCooperativeController.h"

using namespace std;
using namespace Eigen;

class Manipulation
{
	
	public :

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

	KineTransformations Transforms;

	Manipulation();
	~Manipulation();

	void Initialize(ControllerParameters &ctrl_param, Object_to_Grasp &object2grasp, ioStateManager &ioSM, TasksReferences &references);
	void get_references(ControllerParameters &ctrl_param, Object_to_Grasp &object2grasp, ioStateManager &ioSM, TasksReferences &references,
								  bool RRetract_, bool isLift, WholeBodyTaskSpaceTrajectories &wbTskRef);

	Vector6d get_hands_ref_acceleration(std::string hand, Matrix6d g_imp_u, Matrix6d g_imp_c, ioStateManager &ioSM_, bool wConstr);
	void get_hands_ref_impedance_forces(ioStateManager &ioSM_, bool isoImp);
	void getCurrentReferenceGraspPoints(ioStateManager &ioSM_, Vector7d w_Pose_ref, Matrix4d (&h_desH_ref)[2]);
	VectorXd get_expected_object_load(ControllerParameters &ctrl_param, Object_to_Grasp &object2grasp, ioStateManager &ioSM,  TasksReferences &references, Vector7d des_lh_pose, Vector7d des_rh_pose, Vector6d ExtWrenchObj);
	void close();
};


#endif // Manipulation_H