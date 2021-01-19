
#include <iostream>
#include <cmath>

#include "TasksReferences.h"

using namespace std;
using namespace Eigen;




// Constructor
TasksReferences::TasksReferences() {}

// Destructor
TasksReferences::~TasksReferences() {}

//



// Initialization of the reference trajectories 
// void TasksReferences::TasksReferences_Init(ControllerParameters &ctrl_param, Object_to_Grasp &object2grasp, ioStateManager &ioSM, int actuatedDofs,  WholeBodyTaskSpaceTrajectories ref_TS_trajectories_,  VectorXd ref_joints_posture)
void TasksReferences::TasksReferences_Init(	WbRobotModel& robot_model_, ControllerParameters &ctrl_param, Object_to_Grasp &object2grasp, ioStateManager &ioSM, 
											WholeBodyTaskSpaceTrajectories ref_TS_trajectories_,  VectorXd ref_joints_posture, std::string n_data)
{
	
	// initialization
	// =================================================
	count = 0;
	n_actuated_dofs = ctrl_param.actuatedDofs;
	//
	wb_desired_acceleration.initialize(n_actuated_dofs+6);
	// Joints posture
	reference_joints_posture.resize(n_actuated_dofs);
	// 
	Ps_gains.K.resize(n_actuated_dofs);	Ps_gains.K.setOnes();
	Ps_gains.D.resize(n_actuated_dofs);	Ps_gains.D.setOnes();
	//
	// Ps_gains.K *= 10.0;
	// Ps_gains.D *= 50.5;

	Ps_gains.K *= 5.0;
	Ps_gains.D *= 6.5;
	
	// initialize the whole body task space gains to identity
	wb_TS_gains.setIdentity();			// 

	// defining vector for the diagonal element of gain matrices 
	// can com from a file 
	Vector6d vM_lhand_, 	vD_lhand_, 		vK_lhand_;
	Vector6d vM_rhand_, 	vD_rhand_, 		vK_rhand_;
	Vector6d vM_lfoot_, 	vD_lfoot_, 		vK_lfoot_;
	Vector6d vM_rfoot_, 	vD_rfoot_, 		vK_rfoot_;
	Vector6d vM_CoM_,   	vD_CoM_,   		vK_CoM_;
	Vector6d vM_Pelvis_, 	vD_Pelvis_,   	vK_Pelvis_;
	Vector6d vM_Chest_,   	vD_Chest_,  	vK_Chest_;
	// 
	vM_lhand_ <<   1.0,   1.0,   1.0,  1.0,    1.0,   1.0; 
	vD_lhand_ <<  12.0,  12.0,  12.0,  12.0,  12.0,  12.0; 
	vK_lhand_ <<  30.0,  30.0,  30.0,  30.0,  30.0,  30.0;
	vD_lhand_ *=1.5;
	vD_lhand_ *= 0.0;
	vK_lhand_ *= 0.0;

	vM_rhand_ <<  1.0,    1.0,   1.0,   1.0,   1.0,   1.0; 
	vD_rhand_ <<  12.0,  12.0,  12.0,  12.0,  12.0,  12.0; 
	vK_rhand_ <<  30.0,  30.0,  30.0,  30.0,  30.0,  30.0;
	vD_rhand_ *= 1.5;
	vD_rhand_ *= 0.0;
	vK_rhand_ *= 0.0;


	vM_lfoot_ <<   4.0,   4.0,   4.0,  4.0,    4.0,   4.0; 
	vD_lfoot_ <<  45.0,  45.0,  45.0, 30.0,   30.0,  30.0; 
	vK_lfoot_ <<  50.0,  50.0,  50.0, 50.0,   50.0,  50.0;

	vM_rfoot_ <<   4.0,   4.0, 4.0,  4.0,  4.0,  4.0; 
	vD_rfoot_ <<  45.0,  45.0, 45.0, 30.0, 30.0, 30.0; 
	vK_rfoot_ <<  50.0,  50.0, 50.0, 50.0, 50.0, 50.0;

	vM_CoM_ <<   2.0,    2.0,   2.0,    2.0,  2.0,  2.0; 
	vD_CoM_ <<   10.0,   10.0,  6.5,   15.0, 15.0, 15.0;  
	vK_CoM_ <<  20.0,    10.0,  5.0,   20.0, 20.0, 20.0;  

	// vM_Pelvis_ <<   2.0,    2.0,   2.0,    2.0,  2.0,  2.0; 
	// vD_Pelvis_ <<   10.0,   10.0,  6.5,   15.0, 15.0, 15.0;  
	// vK_Pelvis_ <<  10.0,    10.0,  5.0,   20.0, 20.0, 20.0;
	vM_Pelvis_ <<   1.0,   1.0,   1.0,  1.0,    1.0,   1.0; 
	vD_Pelvis_ <<   12.0,  12.0,  12.0,  12.0,  12.0,  12.0;  
	vK_Pelvis_ <<   30.0,  30.0,  30.0,  30.0,  30.0,  30.0;

	vM_Chest_ <<   2.0,    2.0,   2.0,    2.0,  2.0,  2.0; 
	vD_Chest_ <<   10.0,   10.0,  6.5,   15.0, 15.0, 15.0;  
	vK_Chest_ <<  10.0,    10.0,  5.0,   20.0, 20.0, 20.0;




	// vM_lhand_ <<   1.0,   1.0,   1.0,  1.0,    1.0,   1.0; 
	// vD_lhand_ <<  12.0,  12.0,  12.0,  12.0,  12.0,  12.0; 
	// vK_lhand_ <<  30.0,  30.0,  30.0,  30.0,  30.0,  30.0;
	// vD_lhand_ *=1.5;
	// vD_lhand_ *= 0.0;
	// vK_lhand_ *= 0.0;

	// vM_rhand_ <<  1.0,    1.0,   1.0,   1.0,   1.0,   1.0; 
	// vD_rhand_ <<  12.0,  12.0,  12.0,  12.0,  12.0,  12.0; 
	// vK_rhand_ <<  30.0,  30.0,  30.0,  30.0,  30.0,  30.0;
	// vD_rhand_ *= 1.5;
	// vD_rhand_ *= 0.0;
	// vK_rhand_ *= 0.0;


	// vM_lfoot_ <<    1.0,    1.0,    1.0,   1.0,    1.0,   1.0; 
	// vD_lfoot_ <<   20.0,   20.0,   20.0,  20.0,   20.0,  20.0; 
	// vK_lfoot_ <<  100.0,  100.0,  100.0, 100.0,  100.0, 100.0;

	// vM_rfoot_ <<    1.0,    1.0,    1.0,   1.0,    1.0,   1.0;
	// vD_rfoot_ <<   20.0,   20.0,   20.0,  20.0,   20.0,  20.0;
	// vK_rfoot_ <<  100.0,  100.0,  100.0, 100.0,  100.0, 100.0;

	// vM_CoM_ <<   2.0,    2.0,   2.0,    2.0,  2.0,  2.0; 
	// vD_CoM_ <<   10.0,   10.0,  6.5,   15.0, 15.0, 15.0;  
	// vK_CoM_ <<  20.0,    10.0,  5.0,   20.0, 20.0, 20.0;  

	// // vM_Pelvis_ <<   2.0,    2.0,   2.0,    2.0,  2.0,  2.0; 
	// // vD_Pelvis_ <<   10.0,   10.0,  6.5,   15.0, 15.0, 15.0;  
	// // vK_Pelvis_ <<  10.0,    10.0,  5.0,   20.0, 20.0, 20.0;
	// vM_Pelvis_ <<   1.0,   1.0,   1.0,  1.0,    1.0,   1.0; 
	// vD_Pelvis_ <<   12.0,  12.0,  12.0,  12.0,  12.0,  12.0;  
	// vK_Pelvis_ <<   30.0,  30.0,  30.0,  30.0,  30.0,  30.0;

	// vM_Chest_ <<   2.0,    2.0,   2.0,    2.0,  2.0,  2.0; 
	// vD_Chest_ <<   10.0,   10.0,  6.5,   15.0, 15.0, 15.0;  
	// vK_Chest_ <<  10.0,    10.0,  5.0,   20.0, 20.0, 20.0;




	// CONTROL PARAMETERS HERE
	// assigning the prefined gains 
	this->set_impedance_gain(vM_lhand_, 	vD_lhand_, 		vK_lhand_, 		wb_TS_gains.lhand);		// lhand
	this->set_impedance_gain(vM_rhand_, 	vD_rhand_, 		vK_rhand_, 		wb_TS_gains.rhand);		// rhand
	this->set_impedance_gain(vM_lfoot_, 	vD_lfoot_, 		vK_lfoot_, 		wb_TS_gains.lfoot);		// lfoot
	this->set_impedance_gain(vM_rfoot_, 	vD_rfoot_, 		vK_rfoot_, 		wb_TS_gains.rfoot);		// rfoot
	this->set_impedance_gain(vM_CoM_,   	vD_CoM_,   		vK_CoM_, 		wb_TS_gains.CoM);		// CoM
	this->set_impedance_gain(vM_Pelvis_, 	vD_Pelvis_,   	vK_Pelvis_, 	wb_TS_gains.Pelvis);	// Pelvis
	this->set_impedance_gain(vM_Chest_,   	vD_Chest_,  	vK_Chest_, 		wb_TS_gains.Chest);		// Chest
	// 
	isTaskReferenceActive = false;
	// Task space motion
	wb_reference_TS_trajectories 	= 	ref_TS_trajectories_;

	ini_wbTskRef 					= wb_reference_TS_trajectories;
	// reference posture
	reference_joints_posture	    = 	ref_joints_posture;



	// -------------------------------------------------------------------------------------------------------
	//
	nu_Wh = 0.0;
	nu_Ao = 0.0;
    //
    Vector6d g_u; g_u << 50.0, 50.0, 50.0, 30.0, 30.0, 30.0;
    Vector6d g_c; g_c <<  1.0,  1.0,  1.0,  0.0,  0.0,  0.0;

	Damping_UnconsHand  = g_u.asDiagonal();
	Damping_ConsHand 	= g_c.asDiagonal();
	hands_gains 	  	<<  3.0,  2.0,  2.0,  1.0,  1.0,  1.0;   // 12x  wy 0.2

    Desired_object_wrench.setZero();
    F_external_object.setZero();

	F_imp_lh.setZero();
	F_imp_rh.setZero();
	appWrench_lhand.setZero();
	appWrench_rhand.setZero();

    cp_desH_Obj[0]     = Transforms.PoseVector2HomogenousMx(object2grasp.Obj_Pose_Gpoints[0]);
    cp_desH_Obj[1]     = Transforms.PoseVector2HomogenousMx(object2grasp.Obj_Pose_Gpoints[1]);

	// ==========================================================================================
    // Bimanual Free motion controller
    // ==========================================================================================
	FreeMotionCtrl = new BimanualFreeMotionController(ctrl_param);
	FreeMotionCtrl->InitializeReach2GraspTask(object2grasp.period_sec, object2grasp, ioSM);
    FreeMotionCtrl->w_H_hand_des[0] = ioSM.w_H_absF * Transforms.PoseVector2HomogenousMx(ctrl_param.des_lh_Pose_af_1);
    FreeMotionCtrl->w_H_hand_des[1] = ioSM.w_H_absF * Transforms.PoseVector2HomogenousMx(ctrl_param.des_rh_Pose_af_1);
    FreeMotionCtrl->gamma_reachable_p = 0.0;
    FreeMotionCtrl->gamma_reachable_o = 0.0;
    // =====================================================================
    // Bimanual Cooperative controller (Wrench computation)
    // =====================================================================
    CooperativeCtrl.Initialize(ctrl_param, object2grasp, ioSM);
    // compute the object desired wrench
    this->get_object_effective_wrench( object2grasp, object2grasp.Ref_Object, ctrl_param.D_object, F_external_object,  Desired_object_wrench);
    CooperativeCtrl.computeControlWrench(nu_Wh,  object2grasp, ioSM, Desired_object_wrench);
    // =====================================================================
    // -------- TASK REFERENCE ---------------------------------------------
    // =====================================================================
    w_desLHand_Pose.head<3>() = ioSM.wbTS.lhand.Pose.head<3>();       
    w_desRHand_Pose.head<3>() = ioSM.wbTS.rhand.Pose.head<3>();       
    w_desLHand_Pose.tail<4>() = ctrl_param.des_orientation_hands;
    w_desRHand_Pose.tail<4>() = ctrl_param.des_orientation_hands;
    //
    Matrix4d w_H_hand_des_l   = ioSM.w_H_absF * Transforms.PoseVector2HomogenousMx(ctrl_param.des_lh_Pose_af_1);
    Matrix4d w_H_hand_des_r   = ioSM.w_H_absF * Transforms.PoseVector2HomogenousMx(ctrl_param.des_rh_Pose_af_1);
    w_desLHand_Pose.head<3>() = w_H_hand_des_l.block<3,1>(0,3);
    w_desRHand_Pose.head<3>() = w_H_hand_des_r.block<3,1>(0,3);

 	// ------------------------------------------------------------------
 	balanceRef.Initialize(robot_model_, ctrl_param, ioSM);


 	ff_ctrl.Initialize3(ioSM,  robot_model_, n_data);
	// ------------------------------------------------------------------
	des_CoP_robot.setZero();
	pred_CoP_robot.setZero();
	Ref_CoP_robot.setZero();
	ref_CoM_xy 	= ioSM.wbTS.CoM.Pose.head(2);
	Disturb_cx_ = VectorXd::Ones(ff_ctrl.ComRefGen.nsp);
	Disturb_cy_ = VectorXd::Ones(ff_ctrl.ComRefGen.nsp);
	Disturb_c   = Vector2d(0.,0.);

	// 
	d_theta_torso_pitch = 0.0*M_PI/180.;
	timeCmdsStep	   = yarp::os::Time::now();
	t_EndStep   	   = yarp::os::Time::now();
	StepInitiated 	   = false;
	isReachable   	   = false;
	//
	ReleaseAndRetract  = false;
	StartRelease       = false;
	isLifting   	   = false;

	executing_step     = false;
	SwitchStep   	   = false;
	StepCompleted      = false;

	iter_sm    		   = 0;
	iter_g 	  		   = 0;
	stepCount   	   = 0;
	nSampleStep   	   = balanceRef.step_ctrl.Parameters->nsp;
	
	ref_step  		   = VectorXd::Zero(8);
	//
	t_Contact = yarp::os::Time::now();
	isContact = false;

	//
	Vector7d des_pose_lh = Transforms.HomogenousMx2PoseVector(object2grasp.w_H_Gpoints[0]);  // left hand      //IDctrl->wbTS.lhand.Pose;      
	Vector7d des_pose_rh = Transforms.HomogenousMx2PoseVector(object2grasp.w_H_Gpoints[1]);  // right hand     //IDctrl->wbTS.rhand.Pose;
	Vector6d velo_obj    = VectorXd::Zero(6);
	// compute the hands error norm
	xe_lh = VectorXd::Zero(100);
	ye_lh = VectorXd::Zero(100);
	xe_rh = VectorXd::Zero(100);
	ye_rh = VectorXd::Zero(100);
	xe_lh(0) = this->compute_hand_error_norm(ioSM, ctrl_param, des_pose_lh, "left");
	ye_lh(0) = this->compute_hand_error_dot_norm(ioSM, ctrl_param, velo_obj, "left");
	xe_rh(0) = this->compute_hand_error_norm(ioSM, ctrl_param, des_pose_rh, "right");
	ye_rh(0) = this->compute_hand_error_dot_norm(ioSM, ctrl_param, velo_obj, "right");

	hand_error_tol = ctrl_param.tol_dist2contact;
	hand_error     = 0.5*(this->compute_hand_error_norm(ioSM, ctrl_param, des_pose_lh, "left") + this->compute_hand_error_norm(ioSM, ctrl_param, des_pose_rh, "right"));

	time2contact = 100.;
	time2release = 10000.;

}
//
bool TasksReferences::set_impedance_gain(Vector6d M_, Vector6d D_, Vector6d K_, TaskSpaceGains& TS_gains)
{
	TS_gains.M 		= M_.asDiagonal();
	TS_gains.D 		= D_.asDiagonal();
	TS_gains.K 		= K_.asDiagonal();
	TS_gains.invM 	= TS_gains.M.inverse();
	
	return true;
}
//
bool TasksReferences::get_desired_acceleration(	TaskSpaceStates  TS_States_, TaskSpaceTrajectories 	ref_TS_traj_, TaskSpaceGains TS_gains_eef_, Vector6d F_external_, Vector6d &desired_acceleration)
{
	// velocity twist and pose errors
	Eigen::VectorXd velocity_error 	= TS_States_.Velo - ref_TS_traj_.velocity;
	Eigen::VectorXd pose_error     	= Transforms.computePoseErrorNormalizedAxisAngle(TS_States_.Pose, ref_TS_traj_.pose);
	desired_acceleration 			= ref_TS_traj_.acceleration - TS_gains_eef_.invM * ((  TS_gains_eef_.D * velocity_error  + TS_gains_eef_.K * pose_error) - F_external_);
	return true;
}


//
// bool TasksReferences::get_desired_centroidalMomentum(TaskSpaceStates 			TS_StatesCoM_,
// 													 TaskSpaceTrajectories 	    ref_Ts_traj_CoM_,
// 													 TaskSpaceGains				TS_gainsCoM_,
// 													 Vector6d 					F_external_CoM_,
// 													 Matrix6d 					CentroidalDynamicMatrix_,
// 													 Vector3d 					dotCentroidalInertia_mxI_,
// 													 Vector6d 					&desired_centroidalMomentum_dot)
// {
// 	// pose error
// 	Vector6d CoM_pose_error = Transforms.computePoseErrorNormalizedAxisAngle(TS_StatesCoM_.Pose, ref_Ts_traj_CoM_.pose);
// 	// velocity twist and pose errors
// 	Vector6d CoM_velocity_error = TS_StatesCoM_.Velo - ref_Ts_traj_CoM_.velocity;
// 	//
// 	Vector6d bias_forces_momentum_CoM;		bias_forces_momentum_CoM.setZero();
// 	bias_forces_momentum_CoM.tail(3) = dotCentroidalInertia_mxI_.asDiagonal() * TS_StatesCoM_.Velo.segment<3>(3);
// 	desired_centroidalMomentum_dot   = CentroidalDynamicMatrix_ * (ref_Ts_traj_CoM_.acceleration - TS_gainsCoM_.invM * ((TS_gainsCoM_.D * CoM_velocity_error 
// 																													 + TS_gainsCoM_.K * CoM_pose_error) 
// 																													 - F_external_CoM_))
// 								    																				 + 0.0*bias_forces_momentum_CoM;

// 	std::cout << " ref_Ts_traj_CoM_.velocity: \t" << ref_Ts_traj_CoM_.velocity.transpose() << endl;							    																				 
// 	std::cout << " CoM_pose_error: \t" << CoM_pose_error.transpose() << endl;
// 	std::cout << " desired_centroidalMomentum_dot : \t" << desired_centroidalMomentum_dot.transpose() << endl;
// 	std::cout << " CentroidalDynamicMatrix_ : \n" << CentroidalDynamicMatrix_ << endl;
// 	return true;
// }

bool TasksReferences::get_desired_centroidalMomentum(ioStateManager 			&ioSM_,
													 TaskSpaceStates 			TS_StatesCoM_,
													 TaskSpaceTrajectories 	    ref_Ts_traj_CoM_,
													 TaskSpaceGains				TS_gainsCoM_,
													 Vector6d 					F_external_CoM_,
													 Matrix6d 					CentroidalDynamicMatrix_,
													 Vector3d 					dotCentroidalInertia_mxI_,
													 Vector6d 					&desired_centroidalMomentum_dot)
{
	// pose error
	Vector6d CoM_pose_error = Transforms.computePoseErrorNormalizedAxisAngle(TS_StatesCoM_.Pose, ref_Ts_traj_CoM_.pose);
	// velocity twist and pose errors
	Vector6d CoM_velocity_error = TS_StatesCoM_.Velo - ref_Ts_traj_CoM_.velocity;

	Vector6d DampCM;
	DampCM.head(3) = Vector3d(0.0, 0.0, 0.0);
	DampCM.tail(3) = Vector3d(0.0, 0.0, 0.0);
	//
	Vector6d bias_forces_momentum_CoM;		bias_forces_momentum_CoM.setZero();
	bias_forces_momentum_CoM.tail(3) = dotCentroidalInertia_mxI_.asDiagonal() * TS_StatesCoM_.Velo.segment<3>(3);
	desired_centroidalMomentum_dot   = CentroidalDynamicMatrix_ * (ref_Ts_traj_CoM_.acceleration - TS_gainsCoM_.invM * ((TS_gainsCoM_.D * CoM_velocity_error 
																													 + TS_gainsCoM_.K * CoM_pose_error) 
																													 - F_external_CoM_))
								    																				 + 0.0*bias_forces_momentum_CoM; // - DampCM.asDiagonal()*TS_gainsCoM_.D * ioSM_.Jacobian_centro_momentum * ioSM_.q_dot;

	// std::cout << " ref_Ts_traj_CoM_.velocity: \t" << ref_Ts_traj_CoM_.velocity.transpose() << endl;							    																				 
	// std::cout << " CoM_pose_error: \t" << CoM_pose_error.transpose() << endl;
	// std::cout << " desired_centroidalMomentum_dot : \t" << desired_centroidalMomentum_dot.transpose() << endl;
	// std::cout << " CentroidalDynamicMatrix_ : \n" << CentroidalDynamicMatrix_ << endl;
	return true;
}


//
bool TasksReferences::get_desired_acceleration_posture(	JointspaceStates 			JtsStates,
														TaskSpaceStates 			TS_StatesPelvis_,
													 	TaskSpaceTrajectories 	    ref_Ts_trajPelvis_,
													 	Eigen::VectorXd				reference_joints_posture_,
													 	TaskSpaceGains				TS_gainsPelvis_,
													 	Eigen::VectorXd 			&desired_posture_acceleration)

{
	// pose error
	Vector6d Base_pose_error     = Transforms.computePoseErrorNormalizedAxisAngle(TS_StatesPelvis_.Pose, ref_Ts_trajPelvis_.pose);
	// velocity twist error
	Vector6d Base_velocity_error = TS_StatesPelvis_.Velo - ref_Ts_trajPelvis_.velocity;
	desired_posture_acceleration.head(6) = (ref_Ts_trajPelvis_.acceleration - TS_gainsPelvis_.invM *(TS_gainsPelvis_.K * Base_pose_error 
																									   + 1.0*TS_gainsPelvis_.D * Base_velocity_error));
	desired_posture_acceleration.head(3).setZero();
	desired_posture_acceleration.segment(6, JtsStates.position.rows()) = -(Ps_gains.K.asDiagonal() * (JtsStates.position - reference_joints_posture_)
																		 + Ps_gains.D.asDiagonal() *  JtsStates.velocity);
	//
	return true;
}



//
bool TasksReferences::set_posture_references(VectorXd ref_joints_posture)
{
	reference_joints_posture = ref_joints_posture;
	return true;
}

bool TasksReferences::update_references(WholeBodyTaskSpaceTrajectories 	ref_TS_trajectories_,  VectorXd ref_joints_posture_)
{
    wb_reference_TS_trajectories   =   ref_TS_trajectories_;	// Task space motion  
    reference_joints_posture       =   ref_joints_posture_;		// reference posture
    return true;
}

bool TasksReferences::get_stack_of_motion_tasks(ioStateManager 					&ioSM_,
												WholeBodyTaskSpaceTrajectories  wb_ref_TS_traj_,
												Eigen::VectorXd  				ref_jts_posture_,  
												TaskSpaceForces  				F_imp_,
												WholeBodyTaskSpaceAcceleration	&wb_des_acceleration_)
{
	// 
	bool ok = true;
	//
	ok = ok && get_desired_acceleration(ioSM_.wbTS.lhand,     wb_ref_TS_traj_.lhand, wb_TS_gains.lhand, F_imp_.lhand, wb_des_acceleration_.lhand);
	ok = ok && get_desired_acceleration(ioSM_.wbTS.rhand,     wb_ref_TS_traj_.rhand, wb_TS_gains.rhand, F_imp_.rhand, wb_des_acceleration_.rhand);
	ok = ok && get_desired_acceleration(ioSM_.wbTS.lfoot,     wb_ref_TS_traj_.lfoot, wb_TS_gains.lfoot, F_imp_.lfoot, wb_des_acceleration_.lfoot);
	ok = ok && get_desired_acceleration(ioSM_.wbTS.rfoot,     wb_ref_TS_traj_.rfoot, wb_TS_gains.rfoot, F_imp_.rfoot, wb_des_acceleration_.rfoot);
	ok = ok && get_desired_acceleration(ioSM_.wbTS.Chest,     wb_ref_TS_traj_.Chest, wb_TS_gains.Chest, F_imp_.Chest, wb_des_acceleration_.Chest);
	ok = ok && get_desired_acceleration(ioSM_.wbTS.Pelvis,    wb_ref_TS_traj_.Pelvis, wb_TS_gains.Pelvis, F_imp_.Pelvis, wb_des_acceleration_.Pelvis);
	// ok = ok && get_desired_centroidalMomentum(ioSM_.wbTS.CoM, wb_ref_TS_traj_.CoM,   wb_TS_gains.CoM,   F_imp_.CoM,   ioSM_.CentroidalDynamicMatrix, ioSM_.dotCentroidalInertia_mxI, wb_des_acceleration_.CMdot);
	ok = ok && get_desired_centroidalMomentum(ioSM_, ioSM_.wbTS.CoM, wb_ref_TS_traj_.CoM,   wb_TS_gains.CoM,   F_imp_.CoM,   ioSM_.CentroidalDynamicMatrix, ioSM_.dotCentroidalInertia_mxI, wb_des_acceleration_.CMdot);
	ok = ok && get_desired_acceleration_posture(ioSM_.JtsStates, ioSM_.wbTS.Pelvis, wb_ref_TS_traj_.Pelvis, ref_jts_posture_, wb_TS_gains.Pelvis, wb_des_acceleration_.posture);
	
	return ok;
}

// Object reference trajectories
bool TasksReferences::Init_filters_object(double stime, ControllerParameters &ctrl_param)
{
	//
	Vector3d init_velo; init_velo.setZero();
	Filter_object_pos.InitializeFilter(stime, ctrl_param.filter_gain_object_pos, ctrl_param.filter_gain_object_pos, init_velo); 
	Filter_object_rot.InitializeFilter(stime, ctrl_param.filter_gain_object_rot, ctrl_param.filter_gain_object_rot, init_velo);
	return true;
}

bool TasksReferences::get_reference_trajectories_object(Vector7d w_Pose_object_cur, ControllerParameters &ctrl_param, Matrix6d PropGain, TaskSpaceTrajectories& des_States_Object)
{
    //
	Vector6d w_velocity_object_tmp;
	Vector6d w_velocity_object;
	Vector6d w_acceleration_object;
	Vector6d error_object;
	Matrix3d L_Mu_Theta_object;
	Matrix6d L_eta_object_in_w;
    // 
    Matrix4d w_H_object_cur	= Transforms.PoseVector2HomogenousMx(w_Pose_object_cur); 
	Matrix4d w_H_object_des	= Transforms.PoseVector2HomogenousMx(des_States_Object.pose); 
	//
    // get the 6D servoing variables
    Transforms.compute_6Dservoing_variables(w_H_object_cur,             // origin (current)
                                            w_H_object_des,          	// destination (desired)
                                            error_object,
                                            L_Mu_Theta_object,
                                            L_eta_object_in_w);
    // compute the velocity
    w_velocity_object = -L_eta_object_in_w.inverse() * PropGain * error_object;
    // apply saturation to the computed value
	w_velocity_object = Transforms.SaturationTwist(2.*ctrl_param.max_reaching_velocity(0), ctrl_param.max_reaching_velocity(3), w_velocity_object);

	// computation of the acceleration through filtered derivative
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// filtered derivative to compute the acceleration
	Filter_object_pos.pole    = Filter_object_pos.gain = ctrl_param.filter_gain_object_pos * ctrl_param.alpha_object_filter_pos;
	Filter_object_rot.pole    = Filter_object_rot.gain = ctrl_param.filter_gain_object_rot * ctrl_param.alpha_object_filter_rot;
	//
	w_velocity_object_tmp = w_velocity_object;

	w_velocity_object.head(3)     = Filter_object_pos.getEulerIntegral(w_velocity_object.head(3));
	w_velocity_object.tail(3)     = Filter_object_rot.getEulerIntegral(w_velocity_object.tail(3));

	w_acceleration_object.head(3) = Filter_object_pos.pole * (w_velocity_object_tmp.head(3) - w_velocity_object.head(3));
	w_acceleration_object.tail(3) = Filter_object_rot.pole * (w_velocity_object_tmp.tail(3) - w_velocity_object.tail(3));
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	des_States_Object.velocity 		= w_velocity_object;
	des_States_Object.acceleration 	= w_acceleration_object;

    return true;
}

bool TasksReferences::get_object_effective_wrench( 	Object_to_Grasp& 	  object2grasp_, 
													TaskSpaceTrajectories ref_traj_Obj,
		                                           	Matrix6d 			  object_damping_gain_,
		                                           	Vector6d 			  F_external_object_,
		                                           	Vector6d& 			  Desired_object_wrench)
{
	//
	Vector6d desired_object_acceleration = ref_traj_Obj.acceleration - object_damping_gain_ * 2.*(1.0*object2grasp_.States_Object.velocity - ref_traj_Obj.velocity);

	Matrix6d impObject; 	Vector6d gain_imp_obj;  gain_imp_obj << 0.2, 0.2, 0.2, 0.0005, 0.0005, 0.0005;
	impObject = gain_imp_obj.asDiagonal();
	// Desired_object_wrench = (object2grasp_.Mo_world + 0.0*impObject)* desired_object_acceleration + object2grasp_.bo_world - F_external_object_;
	Desired_object_wrench = (object2grasp_.Mo_world + 0.0*impObject)* desired_object_acceleration + object2grasp_.bo_world - F_external_object_;

	return true;
}


// =======================================================================================================
void TasksReferences::moveToLegPosture(RobotInterface& robot_interface_,
										WbRobotModel& robot_model,
										Eigen::VectorXd Des_left_joints,
                                        Eigen::VectorXd Des_right_joints,
                                        bool sendCommands_,
                                        int nb_iter)
{
    //
    double delay_ = 0.040;
    this->moveToLegPosture( robot_interface_, robot_model, Des_left_joints,  Des_right_joints, sendCommands_, nb_iter, delay_);
}

void TasksReferences::moveToLegPosture( RobotInterface& robot_interface_, 
										WbRobotModel& robot_model,
										Eigen::VectorXd Des_left_joints,
                                        Eigen::VectorXd Des_right_joints,
                                        bool sendCommands_,
                                        int nb_iter,
                                        double delay_)
{
    // create vetors to contain the encoders and commands values
    Eigen::VectorXd commands_left(6);     // left leg
    Eigen::VectorXd commands_right(6);    // right leg  
    //
    Eigen::VectorXd Delta_q_l(6); 
    Eigen::VectorXd Delta_q_r(6);
    // 
    // int nb_iter = 30;
    double alpha;
           alpha = 0; 
    double DEG2RAD = (M_PI/180.0);
    // -----------------------------------------------------------
    Eigen::VectorXd joints_posture(robot_model.getDoFs());
    Eigen::VectorXd legs_joints_posture(12);
    Eigen::VectorXd ref_legs_joints_posture(12);
    Eigen::VectorXd left_legs_joints_posture(6);
    Eigen::VectorXd right_legs_joints_posture(6);

    // Two passes starting fisrt with picth and then roll
    for (int ii=0; ii<1; ii++) //2
    {
        // getting actual joints valu
        robot_model.UpdateWholeBodyModel(robot_interface_, "left");
        joints_posture              = robot_model.m_jts_sensor_wb.position;
        legs_joints_posture         = joints_posture.tail(12);
        left_legs_joints_posture    = legs_joints_posture.head(6);
        right_legs_joints_posture   = legs_joints_posture.tail(6);

        // std::cout << " JJJJ Joint posture is : \n" << 180.0/M_PI *joints_posture << std::endl;

        joints_posture(2) = 0.0;

        // compute the joints position errors
        Delta_q_l = DEG2RAD*Des_left_joints  - left_legs_joints_posture;
        Delta_q_r = DEG2RAD*Des_right_joints - right_legs_joints_posture;
        // smoothing the commands
        for(int i=0; i<nb_iter+1; i++)
        {
            alpha = 3.*pow(i,2.)/pow(nb_iter, 2.)-2.*pow(i,3.)/pow(nb_iter, 3.);

            switch (ii)
                {
                    case 0: 
                    {
                    	// left
                        commands_left(0)  =  left_legs_joints_posture(0)  + alpha * Delta_q_l(0);
                        commands_left(1)  =  left_legs_joints_posture(1)  + 0.00; //0.00;
                        commands_left(2)  =  left_legs_joints_posture(2)  + 0.00;
                        commands_left(3)  =  left_legs_joints_posture(3)  + alpha * Delta_q_l(3);
                        commands_left(4)  =  left_legs_joints_posture(4)  + alpha * Delta_q_l(4);
                        commands_left(5)  =  left_legs_joints_posture(5)  + 0.00; //0.00;
                        // right
                        commands_right(0) =  right_legs_joints_posture(0) + alpha * Delta_q_r(0);
                        commands_right(1) =  right_legs_joints_posture(1) + 0.00; //0.00;
                        commands_right(2) =  right_legs_joints_posture(2) + 0.00;
                        commands_right(3) =  right_legs_joints_posture(3) + alpha * Delta_q_r(3);
                        commands_right(4) =  right_legs_joints_posture(4) + alpha * Delta_q_r(4);
                        commands_right(5) =  right_legs_joints_posture(5) + 0.00; //0.00;
                        break; 
                    }
                    case 1: 
                    {
                        // left
                        commands_left(0)  =  left_legs_joints_posture(0)  + alpha * Delta_q_l(0);
                        commands_left(1)  =  left_legs_joints_posture(1)  + alpha * Delta_q_l(1); //0.00;
                        commands_left(2)  =  left_legs_joints_posture(2)  + alpha * Delta_q_l(2);
                        commands_left(3)  =  left_legs_joints_posture(3)  + alpha * Delta_q_l(3);
                        commands_left(4)  =  left_legs_joints_posture(4)  + alpha * Delta_q_l(4);
                        commands_left(5)  =  left_legs_joints_posture(5)  + alpha * Delta_q_l(5); //0.00;
                        // right
                        commands_right(0) =  right_legs_joints_posture(0) + alpha * Delta_q_r(0);
                        commands_right(1) =  right_legs_joints_posture(1) + alpha * Delta_q_r(1); //0.00;
                        commands_right(2) =  right_legs_joints_posture(2) + alpha * Delta_q_r(2);
                        commands_right(3) =  right_legs_joints_posture(3) + alpha * Delta_q_r(3);
                        commands_right(4) =  right_legs_joints_posture(4) + alpha * Delta_q_r(4);
                        commands_right(5) =  right_legs_joints_posture(5) + alpha * Delta_q_r(5); //0.00;
                        break;  
                    }
                };  
                // ============ write the commands ==================== 
                ref_legs_joints_posture.head<6>() = commands_left;      // legs  (6  joints)
                ref_legs_joints_posture.tail<6>() = commands_right;     // legs  (6  joints)
                joints_posture.tail<12>() = ref_legs_joints_posture;
                // Execute the commands
                if(sendCommands_) {
                    robot_model.robot_interface.setWholeBodyControlReferences(joints_posture);
                    yarp::os::Time::delay(delay_);  // 0.040
                }                                        
        } // for nb_iter
    }
}

// ==========================================================================================================================

void TasksReferences::get_JS_posture_references(	int 		contact_confidence,
													VectorXd    jts_position,
							                        VectorXd    init_ref_jts_posture,
							                        VectorXd    old_Weight_posture,
							                        VectorXd&   new_Weight_posture,
							                        VectorXd&   ref_jts_posture)
{   
    if(n_actuated_dofs == 29)
    {
        //
        if(contact_confidence == 1.0)
        {
		    new_Weight_posture(6)    = 20.*old_Weight_posture(6);   //3.5;                         // torso pitch 
		    new_Weight_posture(7)    = 40.*old_Weight_posture(7);  //5.5;                         // torso roll  5.5
		    new_Weight_posture(10)   = 5.0*old_Weight_posture(10); // left shoulder roll
		    new_Weight_posture(17)   = 5.0*old_Weight_posture(17); // right shoulder roll

		    ref_jts_posture(5)  	= 30.*M_PI/180.;             // shoulder yaw 20.
		    ref_jts_posture(12) 	= 30.*M_PI/180.;		     // shoulder yaw 20.

		    std::cout << " contact_confidence  is : " << 1.0 << std::endl;
        }else{

		    new_Weight_posture(6)   = old_Weight_posture(6); //0.2;                         // torso pitch 
		    new_Weight_posture(7)   = old_Weight_posture(7); //0.2;                         // torso roll
		    new_Weight_posture(10)  = old_Weight_posture(10);     // left shoulder roll
		    new_Weight_posture(17)  = old_Weight_posture(17);     // right shoulder roll

		    std::cout << " contact_confidence  is : " << 0.0 << std::endl;
        }


        // arms joints
        ref_jts_posture.tail<12>() = jts_position.tail<12>();

        // shoulder pitch
        if(jts_position(3)  >= -90.*M_PI/180.) {
            ref_jts_posture(3)  = -90.*M_PI/180.; 
            new_Weight_posture(9) = 100000.* old_Weight_posture(9);
        } else {
            new_Weight_posture(9) = old_Weight_posture(9);
            ref_jts_posture(3)  = jts_position(3);
        }
        if(jts_position(10)  >= -90.*M_PI/180.) {
            ref_jts_posture(10)  = -90.*M_PI/180.; 
            new_Weight_posture(16) = 100000.* old_Weight_posture(16);
        } else {
            new_Weight_posture(16) = old_Weight_posture(16);
            ref_jts_posture(10)  = jts_position(10);
        }

        // elbow
        if(jts_position(6)  <= 10.*M_PI/180.) {
            ref_jts_posture(6)  = 10.*M_PI/180.; 
            new_Weight_posture(12) = 100000.* old_Weight_posture(12);
        } else {
            new_Weight_posture(12) = old_Weight_posture(12);
            ref_jts_posture(6)  = jts_position(6);
        }
        if(jts_position(13)  <= 10.*M_PI/180.) {
            ref_jts_posture(13)  = 10.*M_PI/180.; 
            new_Weight_posture(19) = 100000.* old_Weight_posture(19);
        } else {
            new_Weight_posture(19) = old_Weight_posture(19);
            ref_jts_posture(13)  = jts_position(13);
        }

        // torso pitch
        if(jts_position(2)  > 65.*M_PI/180.) {
            ref_jts_posture(2)  = 65.*M_PI/180.; 
            new_Weight_posture(8) = 10000* old_Weight_posture(8);
        } else {
            new_Weight_posture(8) = old_Weight_posture(8);
            ref_jts_posture(2)  = jts_position(2);
        }

        // legs joints
        // left knee
        if(jts_position(20)  >= - 4.0*M_PI/180.) {
            ref_jts_posture(20)  = - 4.0*M_PI/180.; 
            new_Weight_posture(26) = 10000000* old_Weight_posture(26);
        } else {
            new_Weight_posture(26) = old_Weight_posture(26);
            ref_jts_posture(20)  = jts_position(20); 
        }
        // right knee
        if(jts_position(26)  >= - 4.0*M_PI/180.) {
            ref_jts_posture(26)  = - 4.0*M_PI/180.;
            new_Weight_posture(32) = 10000000* old_Weight_posture(32);
        } else {
            new_Weight_posture(32) = old_Weight_posture(32);
            ref_jts_posture(26)  = jts_position(26); 
        }

        // left leg hip pitch
        if(jts_position(17)  > 78.*M_PI/180.) {
            ref_jts_posture(17)  = 78.*M_PI/180.; 
            new_Weight_posture(23) = 1000000* old_Weight_posture(23);
        } else {
            new_Weight_posture(23) = old_Weight_posture(23);
            ref_jts_posture(17)  = jts_position(17); 
        }
         // right leg hip pitch
        if(jts_position(23)  > 78.*M_PI/180.) {
            ref_jts_posture(23)  = 78.*M_PI/180.;
            new_Weight_posture(29) = 1000000* old_Weight_posture(29);
        } else {
            new_Weight_posture(29) = old_Weight_posture(29);
            ref_jts_posture(23)  = jts_position(23); 
        }

        // //
        // // left shoulder roll
        // if(jts_position(4)  < 15.*M_PI/180.) {
        //     ref_jts_posture(4)  = 15.*M_PI/180.; 
        //     new_Weight_posture(10) = 10000* old_Weight_posture(10);
        // } else {
        //     new_Weight_posture(10) = old_Weight_posture(10);
        //     ref_jts_posture(4)  = jts_position(4); 
        // }
        //  // right shoulder roll
        // if(jts_position(11)  < 15.*M_PI/180.) {
        //     ref_jts_posture(11)  = 15.*M_PI/180.;
        //     new_Weight_posture(17) = 10000* old_Weight_posture(17);
        // } else {
        //     new_Weight_posture(17) = old_Weight_posture(17);
        //     ref_jts_posture(11)  = jts_position(11); 
        // }

        ref_jts_posture(4)  = 45.*M_PI/180.;  // shoulder roll
        ref_jts_posture(11) = 45.*M_PI/180.;  // shoulder roll
 
    }


}


// -------------------------------------------------------

void TasksReferences::close()
{
    //close controller thread
    if(FreeMotionCtrl) 
    {
        delete FreeMotionCtrl;
        FreeMotionCtrl = 0;
    }
}

void TasksReferences::get_TS_bimanip_references(ControllerParameters &ctrl_param, Object_to_Grasp &object2grasp, ioStateManager &ioSM,
								  bool RRetract_, bool isLift, WholeBodyTaskSpaceTrajectories &wbTskRef)
{
	// ==================================================================
    // Bimanual Free motion controller
    // ==================================================================
    if(ctrl_param.no_tracking_object) {
    	FreeMotionCtrl->gamma_reachable_p = 0.0;
    	FreeMotionCtrl->gamma_reachable_o = 0.0;
    }

    if(RRetract_){  FreeMotionCtrl->Release_and_Retract2( object2grasp, ioSM); 
    }
    else if(isLift){  
    		// cp_desH_Obj[0]     = Transforms.PoseVector2HomogenousMx(object2grasp.Obj_Pose_Gpoints[0]);
			// cp_desH_Obj[1]     = Transforms.PoseVector2HomogenousMx(object2grasp.Obj_Pose_Gpoints[1]);
			Vector6d w_l_arm_FT_meas = this->RotateWrench(ioSM.w_Pose_laFTs, ioSM.robot_interface.l_arm_FT_vector);
			Vector6d w_r_arm_FT_meas = this->RotateWrench(ioSM.w_Pose_raFTs, ioSM.robot_interface.r_arm_FT_vector);



			// cp_desH_Obj[0](2,3) += 0.02 - 2.5*(w_l_arm_FT_meas(1) - 20.0); 
			// cp_desH_Obj[1](2,3) -= 0.02 - 2.5*(w_r_arm_FT_meas(1) - 20.0);
			// cp_desH_Obj[0](2,3) +=  (-1.5*(w_l_arm_FT_meas(1) - 20.0)); 
			// cp_desH_Obj[1](2,3) -=  (-1.5*(-w_r_arm_FT_meas(1) - 20.0));
			// cp_desH_Obj[0](2,3) +=  (-1.5*(0.5*fabs(w_l_arm_FT_meas(1) + w_r_arm_FT_meas(1)) - 15.0)); 
			// cp_desH_Obj[1](2,3) -=  (-1.5*(0.5*fabs(w_l_arm_FT_meas(1) + w_r_arm_FT_meas(1)) - 15.0));
    	FreeMotionCtrl->getBimanualConstrainedMotion( cp_desH_Obj, object2grasp.Ref_Object.pose, ioSM);  
    	// FreeMotionCtrl->getBimanualConstrainedMotion( cp_desH_Obj, object2grasp.States_Object.pose, ioSM);
    }
    else {  FreeMotionCtrl->compute_Reach2GraspMotion2(  object2grasp, ioSM); 
    }
    // ==================================================================
    // Bimanual Cooperative controller (Wrench computation)
    // ==================================================================
    this->get_reference_trajectories_object( object2grasp.Object_Pose,  ctrl_param, ctrl_param.K_object, object2grasp.Ref_Object);
    this->get_object_effective_wrench( object2grasp, object2grasp.Ref_Object, ctrl_param.D_object, F_external_object,  Desired_object_wrench);     // compute the object desired wrench
    std::cout << " Desired_object_wrench  is : \n" << Desired_object_wrench.transpose() << std::endl;
    //
    CooperativeCtrl.computeControlWrench(nu_Wh,  object2grasp, ioSM, Desired_object_wrench);
    CooperativeCtrl.get_TaskConsistentHandsVelocity(object2grasp.period_sec, nu_Wh*object2grasp.Ref_Object.velocity);                                   // Motion distribution between the hands 
    //
    // if(ctrl_param.apply_wrench && CooperativeCtrl.ContactConfidence ==1.0 && !RRetract_)  
    if(CooperativeCtrl.ContactConfidence ==1.0 && !RRetract_)  
    {
        nu_Wh	= 0.80*nu_Wh + 0.20;
        // nu_Wh	= 0.95*nu_Wh + 0.05;
        nu_Ao	= 0.92*nu_Ao + 0.08;
    }
    else
    {
    	nu_Wh	= 0.0;
        nu_Ao	= 0.0;
        CooperativeCtrl.ContactConfidence = 0.0;
    }
    // ==================================================================
    // outputs variables
    // ==================================================================
    // Motion
    //--------
    wbTskRef.lhand.pose 		= ioSM.wbTS.lhand.Pose;
    wbTskRef.rhand.pose 		= ioSM.wbTS.rhand.Pose;
	wbTskRef.lhand.velocity 	= ioSM.wbTS.lhand.Velo;
	wbTskRef.rhand.velocity 	= ioSM.wbTS.rhand.Velo;

    if(ctrl_param.AccelCommand)
    {
    	wbTskRef.lhand.acceleration = this->get_hands_ref_acceleration("left",  Damping_UnconsHand, Damping_ConsHand, ioSM, false);
		wbTskRef.rhand.acceleration = this->get_hands_ref_acceleration("right", Damping_UnconsHand, Damping_ConsHand, ioSM, false);
		//
		F_imp_lh.setZero();
		F_imp_rh.setZero();
    }
    else
    {
    	wbTskRef.lhand.acceleration.setZero(); 
		wbTskRef.rhand.acceleration.setZero(); 
		//
		this->get_hands_ref_impedance_forces(ioSM, false);
    }
    
	// Forces
    //---------
    Vector6d act_ft = VectorXd::Zero(6);    
    		 act_ft << nu_Ao, nu_Wh, nu_Ao, nu_Ao, nu_Ao, nu_Ao;
    // act_ft << nu_Wh, nu_Wh, nu_Wh, nu_Wh, nu_Wh, nu_Wh;
    appWrench_lhand = 0.3*act_ft.asDiagonal() * (-CooperativeCtrl.F_applied_lhand) - F_imp_lh;
    appWrench_rhand = 0.3*act_ft.asDiagonal() * (-CooperativeCtrl.F_applied_rhand) - F_imp_rh;

}


Vector6d TasksReferences::get_hands_ref_acceleration(std::string hand, Matrix6d g_imp_u, Matrix6d g_imp_c, ioStateManager &ioSM_, bool wConstr)
{
    //
    Vector6d ref_a, // reference hand acceleration
    		 d_v_u, // desired velocity unconstrined hands
    		 d_a_u, // desired velocity unconstrined hands
    		 d_v_c, // desired velocity unconstrined hands
    		 d_a_c, // desired velocity unconstrined hands
    		 m_v;   // measured hand velocity

    if(hand == "left" || hand == "LEFT")
    {
    	d_v_u = FreeMotionCtrl->w_velocity_eef[0];
		d_a_u = FreeMotionCtrl->w_acceleration_eef[0];
		d_v_c = CooperativeCtrl.optimal_hands_velocity.head<6>();
        m_v   = ioSM_.wbTS.lhand.Velo;
		d_a_c.setZero();
    }
    else // hand == "right" || hand == "RIGHT"
    {
    	d_v_u = FreeMotionCtrl->w_velocity_eef[1];
		d_a_u = FreeMotionCtrl->w_acceleration_eef[1];
		d_v_c = CooperativeCtrl.optimal_hands_velocity.tail<6>();
        m_v   = ioSM_.wbTS.rhand.Velo;
		d_a_c.setZero();
    }
	//
    ref_a =  (1.-nu_Wh)*(d_a_u + g_imp_u *(d_v_u - m_v)); 
    if(wConstr){
    	ref_a =  (1.-nu_Wh)*(d_a_u + g_imp_u *(d_v_u - m_v)) + (nu_Wh *nu_Ao)*(d_a_c + g_imp_c * (d_v_c - m_v));
    }
    return ref_a;
}


void TasksReferences::get_hands_ref_impedance_forces(ioStateManager &ioSM_, bool isoImp)
{
	// Update Motion and Forces in the force space
    //=========================================================================================================
    double epsil_bq = 1e-10;
    MatrixXd BasisQ_lh = MatrixXd::Random(3,3);      
    MatrixXd BasisQ_rh = MatrixXd::Random(3,3);
    BasisQ_lh.block<3,1>(0,0) = (1.-nu_Wh*0.0) * FreeMotionCtrl->w_velocity_eef[0].head(3); 
    BasisQ_rh.block<3,1>(0,0) = (1.-nu_Wh*0.0) * FreeMotionCtrl->w_velocity_eef[1].head(3);  
    BasisQ_lh(0,0) += epsil_bq;
    BasisQ_rh(0,0) += epsil_bq;

    Transforms.orthonormalize(BasisQ_lh);
    Transforms.orthonormalize(BasisQ_rh);
    Matrix3d Damping_pos_lh = BasisQ_lh * hands_gains.head(3).asDiagonal() * BasisQ_lh.transpose();
    Matrix3d Damping_pos_rh = BasisQ_rh * hands_gains.head(3).asDiagonal() * BasisQ_rh.transpose();

    if(isoImp)
    {
    	Damping_pos_lh = hands_gains(0)*MatrixXd::Identity(3,3);
    	Damping_pos_rh = hands_gains(0)*MatrixXd::Identity(3,3);
    }
    
    //
    F_imp_lh.head(3) = (1.-nu_Wh*0.0) * Damping_pos_lh * (FreeMotionCtrl->w_velocity_eef[0].head(3) - ioSM_.wbTS.lhand.Velo.head(3));
    F_imp_rh.head(3) = (1.-nu_Wh*0.0) * Damping_pos_rh * (FreeMotionCtrl->w_velocity_eef[1].head(3) - ioSM_.wbTS.rhand.Velo.head(3));
    F_imp_lh.tail(3) = (1.-nu_Wh*0.0) * hands_gains.tail(3).asDiagonal() * FreeMotionCtrl->w_velocity_eef[0].tail(3);
    F_imp_rh.tail(3) = (1.-nu_Wh*0.0) * hands_gains.tail(3).asDiagonal() * FreeMotionCtrl->w_velocity_eef[1].tail(3);

}



void TasksReferences::getCurrentReferenceGraspPoints(ioStateManager &ioSM_, Vector7d w_Pose_ref, Matrix4d (&h_desH_ref)[2])
{
    Matrix4d w_H_ref= Transforms.PoseVector2HomogenousMx(w_Pose_ref);
    h_desH_ref[0]   = Transforms.PoseVector2HomogenousMx(ioSM_.wbTS.lhand.Pose).inverse() * w_H_ref; 
    h_desH_ref[1]   = Transforms.PoseVector2HomogenousMx(ioSM_.wbTS.rhand.Pose).inverse() * w_H_ref;
}

VectorXd TasksReferences::get_expected_object_load(ControllerParameters &ctrl_param, Object_to_Grasp &object2grasp, ioStateManager &ioSM,  Vector7d des_lh_pose, Vector7d des_rh_pose, Vector6d ExtWrenchObj)
{
    //
    VectorXd Wrench_hands_star_ = Eigen::VectorXd::Zero(12);
    Vector6d exp_object_wrench  = Eigen::VectorXd::Zero(6);

    this->get_reference_trajectories_object( object2grasp.Object_Pose,  ctrl_param, ctrl_param.K_object, object2grasp.Ref_Object);
    this->get_object_effective_wrench( object2grasp, object2grasp.Ref_Object, ctrl_param.D_object, ExtWrenchObj,  exp_object_wrench);     // compute the object desired wrench
    std::cout << " expected_object_wrench  is : \n" << exp_object_wrench.transpose() << std::endl;
    //
    CooperativeCtrl.PredictControlWrench( object2grasp, des_lh_pose, des_rh_pose, exp_object_wrench, Wrench_hands_star_); 

    return -Wrench_hands_star_;
}


Vector3d TasksReferences::compute_task_ref_CoP(WbRobotModel& robot_model_, ControllerParameters &ctrl_param, ioStateManager &ioSM_, VectorXd ID_WrenchesFeet, Vector3d desCoP_absF, bool RRetract_)
{
	// compute the reference CoP
    des_CoP_robot   	 = ioSM_.w_H_absF.block<3,3>(0,0) * desCoP_absF + ioSM_.w_H_absF.block<3,1>(0,3);                // desired resting CoP position 
    pred_CoP_robot  	 = balanceRef.get_robot_feet_cop(ioSM_, ID_WrenchesFeet.head<6>(), ID_WrenchesFeet.tail<6>());   // Predicted CoP (using ID solution) 
    //
	// if(ctrl_param.apply_wrench && CooperativeCtrl.ContactConfidence ==1.0 && !RRetract_)
	if(CooperativeCtrl.ContactConfidence ==1.0 && !RRetract_)
        ctrl_param.w_cop = 0.10;
    else if(RRetract_) 
        ctrl_param.w_cop = 0.1;
    else 
        ctrl_param.w_cop = 0.90;
    //
    Ref_CoP_robot   	 = balanceRef.get_reference_CoP(robot_model_, ctrl_param, ioSM_, pred_CoP_robot, des_CoP_robot);   // TO DO : in two functions with and without ff_ctrl

    return Ref_CoP_robot;
}

// Vector2d TasksReferences::compute_ffwd_task_ref_CoM_xy(VectorXd Wrench_hands_star, Vector7d lh_pose, Vector7d rh_pose, Vector3d Ref_CoP_robot_, bool RRetract_, bool &StartRelease_, int &iter_sm_)
// {
// 	// ------------------------------------------------------
//     double fu_z_mgbeta = 0.0;                   	
//     Vector2d Disturb_c = Vector2d(0.0, 0.0);    

//     if(CooperativeCtrl.ContactConfidence == 1.0) {
//         ff_ctrl.a_ctrl.compute_perturbation_terms2(Wrench_hands_star, lh_pose.head(3), rh_pose.head(3), Vector3d(0.,0.,0.), Vector3d(0.,0.,0.), Disturb_c, fu_z_mgbeta);
//     }
//     else {
//         fu_z_mgbeta = 0.0;
//         Disturb_c   = Vector2d(0.0, 0.0);
//     }

// 	if(RRetract_){
//         if(iter_sm_<(ff_ctrl.ComRefGen.nsp)) {
//             Disturb_cx_(ff_ctrl.ComRefGen.nsp-1-iter_sm_)        = 0.0;          // step of the disturbance 
//             Disturb_cy_(ff_ctrl.ComRefGen.nsp-1-iter_sm_)        = 0.0;          // step of the disturbance 
//             iter_sm_ ++;
//             if(iter_sm_ == ff_ctrl.ComRefGen.nsp) StartRelease_  = true;
//         }
//     } else {
//         Disturb_cx_  = Disturb_c(0) * VectorXd::Ones(ff_ctrl.ComRefGen.nsp);    // step of the disturbance 
//         Disturb_cy_  = Disturb_c(1) * VectorXd::Ones(ff_ctrl.ComRefGen.nsp);    // step of the disturbance 
//     }
//     // Apply the reference CoP to the CoM reference generator
//     // -------------------------------------------------------
//     Vector3d fmmCoM_ = Vector3d(0.10, 0.0, 0.0);
//     // Vector2d refCoM_xy = ff_ctrl.generate_Com_reference(Disturb_cx_, Disturb_cy_, fu_z_mgbeta, ff_ctrl.fmmCoM, Ref_CoP_robot);
//     Vector2d CoMxy   = ff_ctrl.generate_Com_reference(Disturb_cx_, Disturb_cy_, fu_z_mgbeta, fmmCoM_, Ref_CoP_robot_); //pred_CoP_robot); //

// 	return CoMxy;
// }
Vector2d TasksReferences::compute_ffwd_task_ref_CoM_xy(VectorXd Wrench_hands_star, Vector7d lh_pose, Vector7d rh_pose, Vector3d Ref_CoP_robot_, bool RRetract_, bool &StartRelease_, int &iter_sm_, Vector3d fmmCoM_)
{
    // ------------------------------------------------------
    double fu_z_mgbeta = 0.0;                       
    // Vector2d Disturb_c = Vector2d(0.0, 0.0);    
    Disturb_c = Vector2d(0.0, 0.0);



    if(CooperativeCtrl.ContactConfidence == 1.0) {
        ff_ctrl.a_ctrl.compute_perturbation_terms2(Wrench_hands_star, lh_pose.head(3), rh_pose.head(3), Vector3d(0.,0.,0.), Vector3d(0.,0.,0.), Disturb_c, fu_z_mgbeta);
    }
    else if(time2contact <= (ff_ctrl.ComRefGen.nsp*ff_ctrl.ComRefGen.sTime)-1.0)
    {
        ff_ctrl.a_ctrl.compute_perturbation_terms2(Wrench_hands_star, lh_pose.head(3), rh_pose.head(3), Vector3d(0.,0.,0.), Vector3d(0.,0.,0.), Disturb_c, fu_z_mgbeta);
    }
    else
    {
        fu_z_mgbeta = 0.0;
        Disturb_c   = Vector2d(0.0, 0.0);
        iter_g      = 0;
        iter_sm_    = 0;
    }

    // cout << " XXXXXXXX       ///////////////     TIME START GRASP XXXXXXXXX        /////////////// \t" << (time2contact <= (ff_ctrl.ComRefGen.nsp*ff_ctrl.ComRefGen.sTime)-1.0) << endl;

    // if(RRetract_ || (time2release <=(ff_ctrl.ComRefGen.nsp*ff_ctrl.ComRefGen.sTime))){
    bool t2R = this->time2release <= (ff_ctrl.ComRefGen.nsp*ff_ctrl.ComRefGen.sTime);
     // cout << " XXXXXXXX       ///////////////     TIME TO RELEASE XXXXXXXXX        /////////////// \t" << t2R << endl;
    if(RRetract_ || false){
    // if(RRetract_ ){
        if(iter_sm_<(ff_ctrl.ComRefGen.nsp)) {
            Disturb_cx_(ff_ctrl.ComRefGen.nsp-1-iter_sm_)        = 0.0;          // step of the disturbance 
            Disturb_cy_(ff_ctrl.ComRefGen.nsp-1-iter_sm_)        = 0.0;          // step of the disturbance 
            iter_sm_ ++;
            if(iter_sm_ == ff_ctrl.ComRefGen.nsp) StartRelease_  = true;
        }
    } 
    else if(CooperativeCtrl.ContactConfidence != 1.0 && time2contact <= (ff_ctrl.ComRefGen.nsp*ff_ctrl.ComRefGen.sTime)+1.0)
    {
        if(iter_g < ff_ctrl.ComRefGen.nsp) {
            VectorXd SampHorizon = VectorXd::Zero(ff_ctrl.ComRefGen.nsp);
            SampHorizon.tail(1+iter_g) = VectorXd::Ones(1+iter_g);
            Disturb_cx_  = Disturb_c(0) * SampHorizon;
            Disturb_cy_  = Disturb_c(1) * SampHorizon;
            // cout << " XXXXXXXX       ///////////////     IN THIS PART GRASP XXXXXXXXX        /////////////// \t" << iter_g << endl;
            iter_g ++;
        }
        else {
        	Disturb_cx_  = Disturb_c(0) * VectorXd::Ones(ff_ctrl.ComRefGen.nsp);    // step of the disturbance 
        	Disturb_cy_  = Disturb_c(1) * VectorXd::Ones(ff_ctrl.ComRefGen.nsp);    // step of the disturbance 
        }
    }
    else 
    {
        Disturb_cx_  = Disturb_c(0) * VectorXd::Ones(ff_ctrl.ComRefGen.nsp);    // step of the disturbance 
        Disturb_cy_  = Disturb_c(1) * VectorXd::Ones(ff_ctrl.ComRefGen.nsp);    // step of the disturbance 

        // cout << " XXXXXXXX       ///////////////     IN THIS PART NO GRASP XXXXXXXXX        /////////////// \t" << iter_g << endl;
    }
    // Apply the reference CoP to the CoM reference generator
    // -------------------------------------------------------
    // Vector3d fmmCoM_ = Vector3d(0.10, 0.0, 0.0);
    // Vector2d refCoM_xy = ff_ctrl.generate_Com_reference(Disturb_cx_, Disturb_cy_, fu_z_mgbeta, ff_ctrl.fmmCoM, Ref_CoP_robot);
    Vector2d CoMxy   = ff_ctrl.generate_Com_reference(Disturb_cx_, Disturb_cy_, fu_z_mgbeta, fmmCoM_, Ref_CoP_robot_); //pred_CoP_robot); //

    return CoMxy;
}

Vector2d TasksReferences::get_ref_CoM_trajectory(	WbRobotModel& robot_model_, ControllerParameters &ctrl_param, ioStateManager &ioSM_, 
													VectorXd ID_WrenchesFeet, VectorXd Wrench_hands_star, Vector7d lh_pose, Vector7d rh_pose, 
													Vector3d desCoP_absF, bool RRetract_, bool &StartRelease_, int &iter_sm_)
{
	// Ref Com without FeedForward controller actions
	Vector3d Ref_CoP_robot_ = this->compute_task_ref_CoP(robot_model_, ctrl_param, ioSM_, ID_WrenchesFeet, desCoP_absF, RRetract_);
    if(!ctrl_param.with_FFwdCtrl)	{
    	ref_CoM_xy = Ref_CoP_robot_.head(2);
    }
    else 	{ // Ref CoM with FeedForward controller actions
    	Vector3d fmmCoM_ = Vector3d(0.10, 0.0, 0.0);
    	ref_CoM_xy = this->compute_ffwd_task_ref_CoM_xy(Wrench_hands_star, lh_pose, rh_pose, Ref_CoP_robot_, RRetract_, StartRelease_, iter_sm_, fmmCoM_);
    }

    return ref_CoM_xy;
}


void TasksReferences::get_ffctrl_references(	WbRobotModel& m_robot_model_, ioStateManager &ioSM_, bool &SwitchStep, bool &executing_step, bool &StepCompleted, 
												int &stepCount, int nSampleStep, std::string &stance_ft,
												Vector7d des_X_EE[], VectorXd Wrench_hands_star, VectorXd &ref_step, VectorXd &Joints_pos_cmds)
{                    
    //
	if(!executing_step) {

	    ff_ctrl.aGstep_run_batch(m_robot_model_, ioSM_.JtsStates.position, Wrench_hands_star, des_X_EE, 
	    						 ioSM_.wbTS.lfoot.Pose, ioSM_.wbTS.rfoot.Pose, stance_ft, executing_step, ref_step);
	    	// executing_step = true;
	    if(executing_step) InitiateStepping(ioSM_, ref_step, Joints_pos_cmds);
	    nSampleStep = balanceRef.step_ctrl.Parameters->nsp;
	}   
	else if(executing_step)  {// should coorespond to pressing the 'b' key
	    // perform the step at the end of the execution set executing_step back to phase
	    if(fmod(count, 4)==0)   {
	        balanceRef.generate_step_commands(m_robot_model_, ioSM_, SwitchStep, executing_step, StepCompleted, stepCount, nSampleStep, 
	                                                        stance_ft, Vector3d(0.10, 0.0, 0.0), d_theta_torso_pitch,  Joints_pos_cmds);
	        // update the stance foot
	        ioSM_.stance_foot = stance_ft;
	    }
	    count ++;
	    // executing_step = false;                 // desactivate the step exectution
	}
}


//
bool TasksReferences::InitiateStepping(ioStateManager &ioSM_, VectorXd ref_step, VectorXd &Joints_pos_cmds)
{
    //
    StepInitiated  = true;
    timeCmdsStep   = yarp::os::Time::now();
    isReachable    = false;
   	FreeMotionCtrl->gamma_reachable_p = 0.0;
   	FreeMotionCtrl->gamma_reachable_o = 0.0;

    ioSM_.Joints_pos_filtered_cmds = ioSM_.Joints_pos_cmds = ioSM_.JtsStates.position; //m_joints_sensor_wb.position;


    // ref_step//
    if(ref_step(6) == 1 && ref_step(7) ==0){
       	balanceRef.step_ctrl.CpBalWlkController->stance_foot = "right";
        // StepFoot_1 = ref_step.head(3);
        // StepFoot_2.setZero();
        nSampleStep = balanceRef.step_ctrl.Parameters->nsp;
    }
    else if(ref_step(6) == 0 && ref_step(7) == 1){
       	balanceRef.step_ctrl.CpBalWlkController->stance_foot = "left";
        // StepFoot_1 = ref_step.head(3);
        // StepFoot_2.setZero();
        nSampleStep = balanceRef.step_ctrl.Parameters->nsp;
    }
    else if(ref_step(6) == 1 && ref_step(7) == 1) {
       	balanceRef.step_ctrl.CpBalWlkController->stance_foot = "right";
        // StepFoot_1 = ref_step.head(3);
        // StepFoot_2 = ref_step.segment(3,3);
        nSampleStep = 2*balanceRef.step_ctrl.Parameters->nsp;
    }
    else{
       	balanceRef.step_ctrl.CpBalWlkController->stance_foot = "right";
        // StepFoot_1.setZero();
        // StepFoot_2.setZero();
        nSampleStep = balanceRef.step_ctrl.Parameters->nsp;
    }   
    return true;
}


bool TasksReferences::get_TS_balance_references(WbRobotModel& m_robot_model_, ControllerParameters &ctrl_param, ioStateManager &ioSM_, VectorXd ID_WrenchesFeet, Vector3d desCoP_absF, VectorXd Wrench_hands_star, 
                                																		std::string stance_ft, double &t_EndStep, Vector7d (&des_X_EE)[8], WholeBodyTaskSpaceTrajectories &wbTskRef)
{
    
    Vector3d Ref_CoP_robot_ = this->compute_task_ref_CoP(m_robot_model_, ctrl_param, ioSM_, ID_WrenchesFeet, desCoP_absF, ReleaseAndRetract);
    // Ref CoM with FeedForward controller actions
    if(!ctrl_param.with_FFwdCtrl) {
        ref_CoM_xy = Ref_CoP_robot_.head(2);
    }
    else    // Ref CoM with FeedForward controller actions
    {
        // int nSampleStep = balanceRef.step_ctrl.Parameters->nsp;
        // TasksRefs.get_ffctrl_references(  *m_robot_model, *ioSM, SwitchStep, executing_step, StepCompleted, stepCount, nSampleStep, stance_ft, des_X_EE, Wrench_hands_star, ref_step, Joints_pos_cmds);
        if(!executing_step) {
            ff_ctrl.aGstep_run_batch(m_robot_model_, ioSM_.JtsStates.position, Wrench_hands_star, des_X_EE, ioSM_.wbTS.lfoot.Pose, ioSM_.wbTS.rfoot.Pose, stance_ft, executing_step, ref_step);
            // 
            if(fmod(count, 20)==0) {
            	Vector2d p_absHands = 0.5*(ioSM_.wbTS.lhand.Pose.head(2)+ioSM_.wbTS.lhand.Pose.head(2)) - ioSM_.w_Pose_absF.head(2);
            	MatrixXd num_ = p_absHands.transpose() * ff_ctrl.fmmCoM.head(2);
            	double deno_  = p_absHands.norm()*ff_ctrl.fmmCoM.head(2).norm();

				double dir_ConstCoM = 1./(deno_ + 1e-10) * num_(0,0);
				Vector3d fmmCoM_ = Vector3d(0.10, 0.0, 0.0);
				if(dir_ConstCoM <= 0) 	fmmCoM_ = -ff_ctrl.fmmCoM;
				else 					fmmCoM_ =  ff_ctrl.fmmCoM;

                ref_CoM_xy = this->compute_ffwd_task_ref_CoM_xy(Wrench_hands_star, des_X_EE[0], des_X_EE[1], Ref_CoP_robot_, ReleaseAndRetract, StartRelease, iter_sm, fmmCoM_);
            }
            // executing_step = true && count < 10;
            if(executing_step) InitiateStepping(ioSM_, ref_step, ioSM_.Joints_pos_cmds);
        }   
        else if(executing_step)  {// should coorespond to pressing the 'b' key
            // perform the step at the end of the execution set executing_step back to phase
            if(fmod(count, 4)==0)   {
                balanceRef.generate_step_commands(m_robot_model_, ioSM_, SwitchStep, executing_step, StepCompleted, stepCount, nSampleStep, stance_ft, Vector3d(0.10, 0.0, 0.0), d_theta_torso_pitch,  ioSM_.Joints_pos_cmds);
                // update the stance foot
                ioSM_.stance_foot = stance_ft;
                //
                if(stepCount == (nSampleStep -1))
                {
                    // update the reference feet pose for the wb IK and ID.
                    des_X_EE[2] = wbTskRef.lfoot.pose = ioSM_.wbTS.lfoot.Pose;
                    des_X_EE[3] = wbTskRef.rfoot.pose = ioSM_.wbTS.rfoot.Pose; 
                    //
                    ioSM_.Joints_pos_filtered_cmds = ioSM_.Joints_pos_cmds = ioSM_.JtsStates.position; //m_joints_sensor_wb.position;
                    reference_joints_posture.tail(12) = ioSM_.JtsStates.position.tail(12);
                    // q_ref.tail(12)           = ioSM_.JtsStates.position.tail(12);  
                }
                t_EndStep = yarp::os::Time::now();
            }
            // executing_step = false;                 // desactivate the step exectution
        }
    }
    count ++;
    // desired CoM Motion tasks
    // =========================================================================================================================
    wbTskRef.CoM.pose    = ioSM_.wbTS.CoM.Pose;
    wbTskRef.CoM.pose(0) = ref_CoM_xy(0);
    wbTskRef.CoM.pose(1) = ref_CoM_xy(1); 
    //
    des_X_EE[4].head(3)  = wbTskRef.CoM.pose.head(3);
    // if(ReleaseAndRetract || !TasksRefs.isReachable || isLifting || PreStepPosture)  {
    if(ReleaseAndRetract || !isReachable || isLifting)  {
        des_X_EE[4](1) = wbTskRef.CoM.pose(1) = this->des_CoP_robot(1);         // 
        des_X_EE[4](2) = wbTskRef.CoM.pose(2) = ctrl_param.rest_height_com;     // 0.49;   //  TO DO : Put this in ctrl_param
        // if(ReleaseAndRetract || !TasksRefs.isReachable || PreStepPosture) 
        if(ReleaseAndRetract || !isReachable)
            des_X_EE[4](0) = this->des_CoP_robot(0);
        std::cout << " /// ============================== //// RFE COM POS : \t" << wbTskRef.CoM.pose.head(3).transpose() << std::endl;
    } 

    return true;
}

bool TasksReferences::get_TS_posture_references(ControllerParameters &ctrl_param, ioStateManager &ioSM_, WholeBodyTaskSpaceTrajectories &wbTskRef, Vector7d (&des_X_EE)[8])  // Pelvis and Chest
{
    if(CooperativeCtrl.ContactConfidence ==1.0 || ReleaseAndRetract || isLifting ){
        // Motion
        wbTskRef.Pelvis.pose   = ini_wbTskRef.Pelvis.pose;      wbTskRef.Pelvis.pose.tail<4>() <<  0.0, 0.0, 1.0, M_PI;
        wbTskRef.Chest.pose    = ini_wbTskRef.Chest.pose;       wbTskRef.Chest.pose.tail<4>()  <<  1.0, 0.0, 0.0, M_PI/2.; //
        // des_X_EE[7].tail(4)    = wbTskRef.Chest.pose.tail<4>();
    }   else {
        // Motion
        wbTskRef.Pelvis.pose   = ioSM_.wbTS.Pelvis.Pose; 
        wbTskRef.Chest.pose    = ioSM_.wbTS.Chest.Pose;
        // des_X_EE[7].tail(4)    = ioSM_.wbTS.Chest.Pose.tail<4>();
    }

    return true;
}


double TasksReferences::compute_hand_error_norm(ioStateManager &ioSM_, ControllerParameters &ctrl_param, Vector7d des_hand_pos, std::string hand)
{
    Matrix4d w_H_hand = MatrixXd::Identity(4,4);
    if(hand == "right" || hand =="RIGHT")   {    // right
        w_H_hand         = Transforms.PoseVector2HomogenousMx(ioSM_.wbTS.rhand.Pose);           
        w_H_hand.block<3,1>(0,3) +=  w_H_hand.block<3,3>(0,0)*ctrl_param.hand_offset[1];
    }
    else  {
        w_H_hand         = Transforms.PoseVector2HomogenousMx(ioSM_.wbTS.lhand.Pose);           
        w_H_hand.block<3,1>(0,3) +=  w_H_hand.block<3,3>(0,0)*ctrl_param.hand_offset[0];
    }
    //
    Matrix4d w_H_hand_des     = ioSM_.w_H_absF * Transforms.PoseVector2HomogenousMx(des_hand_pos);
    double hand_error_norm    = (w_H_hand.block<3,1>(0,3) - w_H_hand_des.block<3,1>(0,3)).norm();

    return hand_error_norm;
}


double TasksReferences::compute_hand_error_dot_norm(ioStateManager &ioSM_, ControllerParameters &ctrl_param, Vector6d velo_obj, std::string hand)
{	
    Vector3d w_velo_hand = Vector3d(0.0, 0.0, 0.0);
    if(hand == "right" || hand == "RIGHT")  
        w_velo_hand   = ioSM_.wbTS.rhand.Velo.head(3);            
    else  
        w_velo_hand   = ioSM_.wbTS.lhand.Velo.head(3); 

    double hand_error_dot_norm = (w_velo_hand-velo_obj.head(3)).norm();
    //
    // cout << " XXXXXXXXXXXXXXXXX  left  hand velocity XXXXXXXXXXXXXX IS : " << ioSM_.wbTS.lhand.Velo.head(3).transpose() << endl; 
    // cout << " XXXXXXXXXXXXXXXXX  right hand velocity XXXXXXXXXXXXXX IS : " << ioSM_.wbTS.rhand.Velo.head(3).transpose() << endl; 
    //
    return hand_error_dot_norm;
}


double TasksReferences::estimate_time2contact(ioStateManager &ioSM_, ControllerParameters &ctrl_param, Vector7d des_pose_lh, Vector7d des_pose_rh, Vector6d velo_obj)
{
	//
	xe_lh.tail(99) = xe_lh.head(99);
	ye_lh.tail(99) = ye_lh.head(99);
	xe_lh(0) = this->compute_hand_error_norm(ioSM_, ctrl_param, des_pose_lh, "left");
	ye_lh(0) = this->compute_hand_error_dot_norm(ioSM_, ctrl_param, velo_obj, "left");

	xe_rh.tail(99) = xe_rh.head(99);
	ye_rh.tail(99) = ye_rh.head(99);
	xe_rh(0) = this->compute_hand_error_norm(ioSM_, ctrl_param, des_pose_rh, "right");
	ye_rh(0) = this->compute_hand_error_dot_norm(ioSM_, ctrl_param, velo_obj, "right");

	//
	MatrixXd deno = (xe_lh.transpose() * xe_lh + xe_rh.transpose() * xe_rh);
	MatrixXd nume = (xe_lh.transpose() * ye_lh + xe_rh.transpose() * ye_rh);
	double alpha = 1./( deno(0,0) + 1.e-10) * (nume(0,0));

	hand_error   = 0.5*(this->compute_hand_error_norm(ioSM_, ctrl_param, des_pose_lh, "left") + this->compute_hand_error_norm(ioSM_, ctrl_param, des_pose_rh, "right"));
	// time2contact = -1./alpha * log(hand_error_tol/hand_error); // + yarp::os::Time::now();
	double t2c = -1./alpha * log(hand_error_tol/hand_error); // + yarp::os::Time::now();

	// cout << " XXXXXXXXXXXXXXXXX  deno(0,0) XXXXXXXXXXXXXX IS : " << deno(0,0) << endl; 
	// cout << " XXXXXXXXXXXXXXXXX  nume(0,0) XXXXXXXXXXXXXX IS : " << nume(0,0) << endl; 
	// cout << " XXXXXXXXXXXXXXXXX  alpha     XXXXXXXXXXXXXX IS : " << alpha << endl; 
	// cout << " XXXXXXXXXXXXXXXXX  hand_error XXXXXXXXXXXXXX IS : " << hand_error << endl; 

	return t2c;
}

// 
Matrix6d TasksReferences::WrenchMapMx(Vector7d pose)
{
	Matrix6d wMap = MatrixXd::Identity(6,6);
	wMap.block<3,3>(0,0) = Transforms.PoseVector2HomogenousMx(pose).block(0,0,3,3);
	wMap.block<3,3>(3,3) = wMap.block<3,3>(0,0);

	return wMap;
}

Vector6d TasksReferences::RotateWrench(Vector7d pose, Vector6d Wrench)
{
	return WrenchMapMx(pose) * Wrench;
	// return Wrench;
}