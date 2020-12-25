
#include <string>
#include <iostream>
#include <fstream>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/QR>

#include <yarp/os/Time.h>
#include <yarp/math/Math.h>
#include <yarp/os/LockGuard.h>

#include "InverseDynamics.h"

using namespace Eigen;
using namespace std;

Vars vars;
Params params;
Workspace work;
Settings settings;

wb2_Vars 		wb2_vars;
wb2_Params 		wb2_params;
wb2_Workspace 	wb2_work;
wb2_Settings 	wb2_settings;


// InverseDynamics::InverseDynamics(int period, std::string robotName, int actuatedDofs, ControllerParameters&  ctrl_param_, WbRobotModel& robot, RobotInterface& robot_interface_, ioStateManager& ioSM_) //, TasksReferences& Ref_motion)  
InverseDynamics::InverseDynamics(int period, std::string robotName, int actuatedDofs, ControllerParameters&  ctrl_param_, WbRobotModel& robot, ioStateManager& ioSM_) //, TasksReferences& Ref_motion)  
								: RateThread(period)
								, RobotName(robotName)
								// , robot_interface(robot_interface_)
								, ioSM(ioSM_)
								, n_actuatedDofs(actuatedDofs)
								, ctrl_param(ctrl_param_)
								, robot_model(robot)
								, bias_fcontact_torque(actuatedDofs+6)
								, Jacobian_SoT(30, actuatedDofs+6)
								, dotJacobianDq_SoT(30)
								// Tasks
								// Motion
								, desired_posture_acceleration(actuatedDofs+6)
								, desired_StackOfTasks(30)
								, desired_rate_centroidalMomentum(6)
								// Forces
								, Stack_of_Forces(42)
								, Weight_posture(actuatedDofs+6)
								// QP solution
								, optimal_acceleration(actuatedDofs+6)
								, Tau_actuated(actuatedDofs)
								, optimal_feet_contact_forces(12)
								, optimal_slack(30)
								, Joints_accel_FwD(actuatedDofs+6)
								, J_T_Wx_J(actuatedDofs+6, actuatedDofs+6)
								, J_T_Wx_Xdot(actuatedDofs+6)
								, des_q_dot(actuatedDofs+6)
								, Weight_acceleration(actuatedDofs+6)
								, posture_weight(actuatedDofs)

								, inv_M(actuatedDofs+6, actuatedDofs+6)
								, Lambda_SoT(33, 33)
								, JSoT(actuatedDofs+6, actuatedDofs+6)
								, N_SoT(actuatedDofs+6, actuatedDofs+6)
								, JSoT_bar(33, actuatedDofs+6)

						        { ThreadPeriod = period; n_actuatedDofs = actuatedDofs; run_period_sec = 0.001 * period; }
										        

InverseDynamics::~InverseDynamics() { }

// ============================  THREAD INITIALIZATION ===========================
bool InverseDynamics::threadInit()
{
	//
	StopCtrl 	= false;
	Cycle_counter = 0;
	trqfac = 0.0001;

	bias_fcontact_torque.setZero();
	wb_des_tasks_acceleration.initialize(n_actuatedDofs+6);
	// TskSpStates.Pose.setZero();
	// TskSpStates.Velo.setZero();

	// Cont_States.Pelvis 	= 0.0;
	// Cont_States.lhand 	= 0.0;
	// Cont_States.rhand 	= 0.0;
	// Cont_States.lfoot 	= 1.0;
	// Cont_States.rfoot 	= 1.0;

	// stance_foot 		= "left";

	// Jacobians
	// ============================
	Jacobian_SoT.setZero();
	dotJacobianDq_SoT.setZero();

	//
	J_T_Wx_J.setZero();
	J_T_Wx_Xdot.setZero();
	//
	// Weight_acceleration = Eigen::VectorXd::Ones(n_actuatedDofs+6);
	// Weight_acceleration.segment( 0, 6) << 50., 50., 10., 20., 20., 20.; 				// floating base
	// // Weight_acceleration.segment( 6, 3) << 2.5, 2.5, 2.5; 							// torso
	// // Weight_acceleration.segment( 9, 7) << 1., 1., 1., 1., 1., 1., 1.; 				// left arm
	// // Weight_acceleration.segment(16, 7) << 1., 1., 1., 1., 1., 1., 1.; 				// right arm
	// // Weight_acceleration.segment(23, 6) << 1., 1., 1., 1., 1., 1.; 					// left leg 	leg hip yaw *= 2.5;
	// // Weight_acceleration.segment(29, 6) << 1., 1., 1., 1., 1., 1.; 					// right leg 	leg hip yaw *= 2.5;
	// Weight_acceleration *= 1e-4;

	// posture_weight = Eigen::VectorXd::Ones(n_actuatedDofs);
	// // posture_weight.segment( 0, 3) << 3.5, 2.5, 1.; 					// torso
	// posture_weight *= 5.5;

	// task_weight[0] <<  15.e-0, 15.e-0, 5.e-1, 10.e-0, 10.e-0, 10.e-0;  	// weight_CoMp; 
	// task_weight[1] <<     5.0,    5.0,   5.0,   2.50,   2.50,   2.50;  	// weight_lhand; 
	// task_weight[2] <<     5.0,    5.0,   5.0,   2.50,   2.50,   2.50;  	// weight_rhand; 
	// task_weight[3] <<   100.0,  100.0, 100.0,  200.0,  200.0,  200.0; 	// weight_lfoot; 
	// task_weight[4] <<   100.0,  100.0, 100.0,  200.0,  200.0,  200.0; 	// weight_rfoot;
	// task_weight[5] <<    0.00,   0.00,  0.00,  10.e-0, 5.e-0, 10.e-0;  	// weight_pelvis;  	2.e-1, 1.e-2, 5.e-2
	// task_weight[6] <<    0.00,   0.00,  0.00,  1.e-5,  1.e-5,  5.e-5;  	// weight_chest; 

	// task_weight[0] *=20.;
	// task_weight[1] *=10.0;
	// task_weight[2] *=10.0;
	// task_weight[3] *=50.;
	// task_weight[4] *=50.;
	// task_weight[5] *=20.0;
	// task_weight[6] *=20.0;


	// Weight_acceleration = Eigen::VectorXd::Ones(n_actuatedDofs+6);
	// Weight_acceleration.segment( 0, 6) << 50., 50., 10., 20., 20., 20.; 				// floating base
	// // Weight_acceleration.segment( 6, 3) << 2.5, 2.5, 2.5; 							// torso
	// // Weight_acceleration.segment( 9, 7) << 1., 1., 1., 1., 1., 1., 1.; 				// left arm
	// // Weight_acceleration.segment(16, 7) << 1., 1., 1., 1., 1., 1., 1.; 				// right arm
	// // Weight_acceleration.segment(23, 6) << 1., 1., 1., 1., 1., 1.; 					// left leg 	leg hip yaw *= 2.5;
	// // Weight_acceleration.segment(29, 6) << 1., 1., 1., 1., 1., 1.; 					// right leg 	leg hip yaw *= 2.5;
	// Weight_acceleration *= 1e-4;

	// posture_weight = Eigen::VectorXd::Ones(n_actuatedDofs);
	// // posture_weight.segment( 0, 3) << 3.5, 2.5, 1.; 					// torso
	// posture_weight *= 10.5;
	// posture_weight(2) *= 5.0;

	// task_weight[0] <<  15.e-0, 15.e-0, 5.e-1, 10.e-0, 10.e-0, 10.e-0;  	// weight_CoMp; 
	// task_weight[1] <<     5.0,    5.0,   5.0,   2.50,   2.50,   2.50;  	// weight_lhand; 
	// task_weight[2] <<     5.0,    5.0,   5.0,   2.50,   2.50,   2.50;  	// weight_rhand; 
	// task_weight[3] <<   100.0,  100.0, 100.0,  200.0,  200.0,  200.0; 	// weight_lfoot; 
	// task_weight[4] <<   100.0,  100.0, 100.0,  200.0,  200.0,  200.0; 	// weight_rfoot;
	// task_weight[5] <<    0.00,   0.00,  0.00,  10.e-0, 5.e-0, 10.e-0;  	// weight_pelvis;  	2.e-1, 1.e-2, 5.e-2
	// task_weight[6] <<    0.00,   0.00,  0.00,  2.e-0,  2.e-0,  2.e-0;  	// weight_chest;  	1.e-5,  1.e-5,  5.e-5

	// task_weight[0] *=8.;
	// task_weight[1] *=10.0;
	// task_weight[2] *=10.0;
	// task_weight[3] *=50.;
	// task_weight[4] *=50.;
	// task_weight[5] *=20.0;
	// task_weight[6] *=30.0;

	Weight_acceleration	= ctrl_param.Weight_acceleration;
	posture_weight      = ctrl_param.posture_weight;
	memcpy(task_weight, &ctrl_param.task_weight[0], 7 * sizeof *ctrl_param.task_weight); 


	for(int i=0; i<7; i++) des_X_dot_EE[i].setZero();
	des_q_dot.setZero();
	// Tasks
	//=============================
	// Motion
	desired_posture_acceleration.setZero();
	desired_StackOfTasks.setZero();
	desired_rate_centroidalMomentum.setZero();
	Stack_of_Forces.setZero();
	//
	Weight_centroidal	= ctrl_param.Weight_centroidal;
	Weight_pelvis		= ctrl_param.Weight_pelvis;
	Weight_lhand		= ctrl_param.Weight_lhand;
	Weight_rhand		= ctrl_param.Weight_rhand;
	Weight_lfoot 		= ctrl_param.Weight_lfoot;
	Weight_rfoot 		= ctrl_param.Weight_rfoot;
	Weight_posture 		= ctrl_param.Weight_posture;

	// Force
	desired_centroidal_wrench.setZero();
	F_external.setZero();

	// QP solution
	// =============================
	optimal_acceleration.setZero();
	Tau_actuated.setZero();
	optimal_feet_contact_forces.setZero();
	optimal_slack.setZero();
	Joints_accel_FwD.setZero();

	// CoP and weight distribition
	// ===========================
	desired_CoP_lfoot.setZero();
	desired_CoP_rfoot.setZero();
	desired_weight_prortion_rfoot = 0.5;
	// //
	CoP_constraint_matrix_lfoot.setZero();
	CoP_constraint_matrix_rfoot.setZero();

	//
	inv_M.setZero();
	Lambda_SoT.setZero();
	JSoT.setZero();
	N_SoT.setZero();
	JSoT_bar.setZero();

	// instantiating whole body constraints object
	// -------------------------------------------
	// robot_model_.getJointsLimits();
	VectorXd minJointLimits = this->robot_model.m_min_jtsLimits;
	minJointLimits(4)  = 12.*M_PI/180.;
	minJointLimits(11) = 12.*M_PI/180.;
	VectorXd maxJointLimits = this->robot_model.m_max_jtsLimits;
	maxJointLimits(n_actuatedDofs-9) = -2.0*M_PI/180.;
	maxJointLimits(n_actuatedDofs-3) = -2.0*M_PI/180.;

	// Instantiation of a WhileBodyContraints object to handle the constraints
	WbConstraints.Initialize(n_actuatedDofs,
							 ctrl_param.Hands_contact_param, 
							 ctrl_param.Feet_contact_param, 
							 this->robot_model.m_torqueSaturationLimit, 
							 this->robot_model.m_velocitySaturationLimit,
							 minJointLimits,
							 maxJointLimits);
	// Solver variables
	// ==================
	set_defaults();
	setup_indexing();
	// for the hierarchical
	wb2_set_defaults();
	wb2_setup_indexing();

	// 
	bool ok = false;

	// update the robot state
	// ok = this->UpdateRobotStates();    // ------------------------------>>> chech the effect of this comments

	// // initialize previous Centroidal inertia I(q) for dotI(q)
	// dotCentroidalInertia_mxI.setZero();
 //    CentroidalInertia_mxI_previous.setZero();
 //    CentroidalInertia_mxI_previous(0) = CentroidalDynamicMatrix(3,3);
 //    CentroidalInertia_mxI_previous(1) = CentroidalDynamicMatrix(4,4);
 //    CentroidalInertia_mxI_previous(2) = CentroidalDynamicMatrix(5,5);
 //    //
	// update the stack of tasks (SoT)
	ok = this->update_force_stack_of_tasks();

	//
	return true;
}

// ============================  THREAD RELEASE =================================
void InverseDynamics::threadRelease()
{ 
	std::cerr << "Deactivating control\n";
}

// ============================  THREAD RUNNING LOOP ============================
void InverseDynamics::run()		// compute ID decision variables
{
	double t_run = yarp::os::Time::now();    // << IOState
	// update the stack of tasks (SoT)
	this->update_force_stack_of_tasks(); 		// << IOState
	//  compute the whole body Stack of task Jacobian
	this->get_WbStackOfTaskJacobian();		// << IOState
 	// get the equality and inequality matrices
	this->get_ConstraintsMatrices();		// << IOState
	// QP Solution
	// ============
 	this->get_optimal_solution();			// << IOState

 	// Tau_actuated(2) += -0.1*(ioSM.JtsStates.position(2) - 0.0);

 	inv_M = ioSM.massMatrix.inverse();

 // 	JSoT.block( 0, 0, 6, n_actuatedDofs + 6) =	ioSM.Jacobian_centro_momentum; 	// CM 
 // 	JSoT.block( 6, 0, 6, n_actuatedDofs + 6) =	ioSM.Jacobian_left_hand; 		// Left Hand
 // 	JSoT.block(12, 0, 6, n_actuatedDofs + 6) =	ioSM.Jacobian_right_hand; 		// Right Hand
 // 	JSoT.block(18, 0, 6, n_actuatedDofs + 6) =	ioSM.Jacobian_left_foot; 		// Left Foot
 // 	JSoT.block(24, 0, 6, n_actuatedDofs + 6) =	ioSM.Jacobian_right_foot; 		// Right Foot
 // 	JSoT.block(30, 0, 3, n_actuatedDofs + 6) =	ioSM.Jacobian_pelvis.bottomRows(3); 			// Pelvis

 // 	MatrixXd JSoT_invM  = JSoT * inv_M;
 // 	MatrixXd inv_Lambda = JSoT_invM * JSoT.transpose();
	// // PsInv.get_LLTSolveInverse(inv_Lambda, Lambda_SoT);
	// // PsInv.get_HhQRPseudoInverse(inv_Lambda, Lambda_SoT);
	// // Lambda_SoT = PsInv.get_pseudoInverse(inv_Lambda);
	// JSoT_bar = Lambda_SoT * JSoT_invM;
	// N_SoT	 = MatrixXd::Identity(n_actuatedDofs+6, n_actuatedDofs+6) - JSoT.transpose() * JSoT_bar;

	// VectorXd ST_tau_0 = N_SoT * (ioSM.massMatrix * des_q_dot + ioSM.h_biasForces);

	// std::cout << " inv_Lambda is : \n" << inv_Lambda << std::endl;
	// std::cout << " \n" << std::endl;
	// std::cout << " JSoT_invM is : \n" << JSoT_invM << std::endl;
	// std::cout << " POSTURE TORQUE is : \n" << ST_tau_0.tail(n_actuatedDofs) << std::endl;
	// Tau_actuated += ST_tau_0.tail(n_actuatedDofs);
	

 	VectorXd STau = VectorXd::Zero(n_actuatedDofs + 6);
 	STau.tail(n_actuatedDofs) = Tau_actuated;

 	Joints_accel_FwD = inv_M * (STau + ioSM.Jacobian_left_foot.transpose()  *optimal_feet_contact_forces.head(6) 
	 													 + ioSM.Jacobian_right_foot.transpose() *optimal_feet_contact_forces.tail(6) 
	 													 + bias_fcontact_torque);



    if(ctrl_param.writeCommands && !ctrl_param.PositionCommands)
 	{
		// send the computed torque commands
	 	ioSM.SendTorqueCommands(Tau_actuated);	
 	}

 	// std::cout << " COMPUTED TORQUE is : \n" << Tau_actuated << std::endl;
 	std::cout << " LEFT  FOOT CONTACT WRENCH : \t" << optimal_feet_contact_forces.head(6).transpose() << std::endl;
 	std::cout << " RIGHT FOOT CONTACT WRENCH : \t" << optimal_feet_contact_forces.tail(6).transpose() << std::endl;
 	// std::cout << " desired_StackOfTasks is : \n" << desired_StackOfTasks << std::endl;
 	// std::cout << " COMPUTED ACCELEARTION is : \n" << optimal_acceleration << std::endl;
 	

 	std::cout<< " /////////// ========================================> INVERSE DYNAMICS RUN  in  : " << yarp::os::Time::now()-t_run << " s" << std::endl;
 	std::cout<< "ID COUNTER IS : " << Cycle_counter << std::endl;
 	Cycle_counter ++;

}

// ====================================================================================================================================
bool InverseDynamics::update_force_stack_of_tasks()
{
	// force_task variables
	bias_fcontact_torque      	  = -ioSM.h_biasForces 	+ 	(1.-ioSM.Cont_States.Pelvis) 	* ioSM.Jacobian_pelvis.transpose() 		* F_external.Pelvis
														+	(1.-ioSM.Cont_States.lhand) 	* ioSM.Jacobian_left_hand.transpose() 	* F_external.lhand
														+	(1.-ioSM.Cont_States.rhand) 	* ioSM.Jacobian_right_hand.transpose() 	* F_external.rhand
														+	(1.-ioSM.Cont_States.lfoot) 	* ioSM.Jacobian_left_foot.transpose() 	* F_external.lfoot
														+	(1.-ioSM.Cont_States.rfoot) 	* ioSM.Jacobian_right_foot.transpose() 	* F_external.rfoot;
	//
	desired_centroidal_wrench 	  = desired_rate_centroidalMomentum
												    -  ioSM.Gravity_Force 
												    -  (1.-ioSM.Cont_States.Pelvis) *	ioSM.Centroidal_Wrench_Map_pelvis 	* F_external.Pelvis
													-  (1.-ioSM.Cont_States.lhand) 	*	ioSM.Centroidal_Wrench_Map_lhand 	* F_external.lhand
													-  (1.-ioSM.Cont_States.rhand) 	*	ioSM.Centroidal_Wrench_Map_rhand 	* F_external.rhand
													-  (1.-ioSM.Cont_States.lfoot) 	*	ioSM.Centroidal_Wrench_Map_lfoot 	* F_external.lfoot
													-  (1.-ioSM.Cont_States.rfoot) 	*	ioSM.Centroidal_Wrench_Map_rfoot 	* F_external.rfoot;

	// std::cout << " desired_centroidal_wrench is : \n" << desired_centroidal_wrench << std::endl;

	Matrix4d world_H_lfoot = Transforms.PoseVector2HomogenousMx(ioSM.wbTS.lfoot.Pose);		// Homogenous transf. form left foot to World frame
	Matrix4d world_H_rfoot = Transforms.PoseVector2HomogenousMx(ioSM.wbTS.rfoot.Pose);		// Homogenous transf. form right foot to World frame

	// limiting the desired CoP within the convex hull of the foot
    	 if(desired_CoP_lfoot(0) <=-ctrl_param.DeltaSupport[1]) desired_CoP_lfoot(0) = -ctrl_param.DeltaSupport[1]; 
    else if(desired_CoP_lfoot(0) >= ctrl_param.DeltaSupport[0]) desired_CoP_lfoot(0) =  ctrl_param.DeltaSupport[0];
         if(desired_CoP_rfoot(0) <=-ctrl_param.DeltaSupport[1]) desired_CoP_rfoot(0) = -ctrl_param.DeltaSupport[1]; 
    else if(desired_CoP_rfoot(0) >= ctrl_param.DeltaSupport[0]) desired_CoP_rfoot(0) =  ctrl_param.DeltaSupport[0];
	// desired Centre of pressure constraint matrix
	WbConstraints.get_CoP_constraints_matrix(desired_CoP_lfoot, world_H_lfoot.block<3,3>(0,0), CoP_constraint_matrix_lfoot);
	WbConstraints.get_CoP_constraints_matrix(desired_CoP_rfoot, world_H_rfoot.block<3,3>(0,0), CoP_constraint_matrix_rfoot);
	//
	return true;
}



bool InverseDynamics::get_WbStackOfTaskJacobian()
{
	//
	// if(!ctrl_param.OrientChest)
	// {
	// 	Jacobian_SoT.block(0,  0, 6, n_actuatedDofs+6) = ioSM.Jacobian_centro_momentum ;//.topRows<3>(); 
	// }else { 
	// 	Jacobian_SoT.block(0, 0, 3, n_actuatedDofs+6)  = ioSM.Jacobian_centro_momentum.topRows<3>(); 
	// 	// Jacobian_SoT.block(27, 0, 3, n_actuatedDofs+6) = JacobianChest.bottomRows<3>();
	// 	Jacobian_SoT.block(3, 0, 3, n_actuatedDofs+6)  = ioSM.Jacobian_pelvis.bottomRows<3>(); 
	// }
	Jacobian_SoT.block(0,  0, 6, n_actuatedDofs+6)	= ioSM.Jacobian_SoT_top6 ;//.topRows<3>(); 
	Jacobian_SoT.block(6,  0, 6, n_actuatedDofs+6)	= ioSM.Jacobian_left_hand;
	Jacobian_SoT.block(12, 0, 6, n_actuatedDofs+6)	= ioSM.Jacobian_right_hand;
	Jacobian_SoT.block(18, 0, 6, n_actuatedDofs+6)	= ioSM.Jacobian_left_foot;
	Jacobian_SoT.block(24, 0, 6, n_actuatedDofs+6)	= ioSM.Jacobian_right_foot;

	// 
	// if(!ctrl_param.OrientChest)
	// {
	// 	dotJacobianDq_SoT.segment(0, 6)   = ioSM.DJacobianDq_centro_momentum; //.head<3>();
	// }else {
	// 	dotJacobianDq_SoT.segment(0, 3)   = ioSM.DJacobianDq_centro_momentum.head<3>();
	// 	// dotJacobianDq_SoT.segment(27, 3) = DJacobianDqChest.tail<3>();
	// 	dotJacobianDq_SoT.segment(3, 3)   = ioSM.DJacobianDq_pelvis.tail<3>();
	// }
	dotJacobianDq_SoT.segment(0, 6) 	= ioSM.DJacobianDq_SoT_top6; //.head<3>();
	dotJacobianDq_SoT.segment(6,  6)	= ioSM.DJacobianDq_left_hand;
	dotJacobianDq_SoT.segment(12, 6)	= ioSM.DJacobianDq_right_hand;
	dotJacobianDq_SoT.segment(18, 6)	= ioSM.DJacobianDq_left_foot;
	dotJacobianDq_SoT.segment(24, 6)	= ioSM.DJacobianDq_right_foot;

	

	//
	return true;
}

//
bool InverseDynamics::get_ConstraintsMatrices()
{
	//
	bool ok = false;
	// get inequality matrix
	if(!(ok = WbConstraints.get_WbInequalityConstraints( run_period_sec, ioSM.JtsStates, ioSM.wbTS)))
	{
		printf("Failed to compute get Inequality contraints. \n");
		return false;
	}
	//
	return ok;
}


void InverseDynamics::get_optimal_solution()
{
	this->load_default_data();

	settings.verbose = 0;
	//num_iters = 
	solve();
	// acceleration
	for(int i=0; i<n_actuatedDofs+6; i++) {optimal_acceleration(i) = vars.x[i];	}
	// torque
	for(int i=0; i<n_actuatedDofs; i++){ Tau_actuated(i) = vars.y[i]; }
	// contact forces
	for(int i=0; i<12; i++) { optimal_feet_contact_forces(i) = vars.z[i]; }
	// slack variables
	for(int i=0; i<30; i++) { optimal_slack(i) = vars.w[i]; }

}

// void InverseDynamics::load_default_data()
// {
// 	//
// 	double ref_period = 4.0;
// 	double omega = 2.0*M_PI/ref_period;

// 	params.dt[0] = run_period_sec;
// 	params.aL[0] = 1.0;
// 	params.aR[0] = 1.0; 
// 	params.dwd[0] = desired_weight_prortion_rfoot;
// 	params.aw[0]  = 1.0;

// 	// weight on acceleration
// 	for(int i=0; i<n_actuatedDofs+6; i++) { params.Qx[i] = ctrl_param.Weight_accel(i); }
// 	// weight on torque
// 	for(int i=0; i<n_actuatedDofs; i++) { params.Qy[i] = ctrl_param.Weight_tau(i); }
// 	// weight on contact forces
// 	for(int i=0; i<12; i++) { params.Qz[i] = ctrl_param.Weight_F_contact(i); }
// 	// weight of the stack of task
// 	for(int i=0; i<30; i++) { params.Qw[i] = ctrl_param.Weight_SoT(i); }

// 	for(int i=0; i<6; i++) { params.Qw[30+i] = 10*ctrl_param.Weight_SoT(i); }

// 	for(int i=0; i<n_actuatedDofs; i++) { params.Qw[36+i] = Weight_posture(6+i); }

// 	for(int i=0; i<5; i++) { params.Qw[36+n_actuatedDofs+i] = ctrl_param.Weight_CoP(i); } //weight_cop_lfoot weight_cop_rfoot weight_distribution_rfoot

// 	for(int i=0; i<6; i++) { params.Qw[36+n_actuatedDofs+5+i]   = 0.1*ctrl_param.Weight_SoT(18+i); 	//weight_rfoot
// 							 params.Qw[36+n_actuatedDofs+5+6+i] = 0.1*ctrl_param.Weight_SoT(24+i);} //weight_rfoot
	


// 	int N_DOF = n_actuatedDofs+6;

// 	// Mass matrix
// 	for(int i=0; i<n_actuatedDofs+6; i++)
// 	{
		
// 		// for(int i=0;i<AIR_N_U;i++)
// 		memcpy(params.M[i+1], &ioSM.massMatrix(0,i),     sizeof(double)*(N_DOF));
// 		//
// 		params.b1[i] = bias_fcontact_torque(i);  // -h + Jf^T * F_m
// 	}

// // Task Jacobian
// 	for(int i=0; i<n_actuatedDofs+6; i++)
// 	{
// 		// centroidal 
// 		params.Jt_1[i] = Jacobian_SoT(0,i);		params.Jt_2[i] = Jacobian_SoT(1,i);		params.Jt_3[i] = Jacobian_SoT(2,i);	
// 		params.Jt_4[i] = Jacobian_SoT(3,i);		params.Jt_5[i] = Jacobian_SoT(4,i);		params.Jt_6[i] = Jacobian_SoT(5,i);
// 		// left hand 
// 		params.Jt_7[i] = Jacobian_SoT(6,i);		params.Jt_8[i] = Jacobian_SoT(7,i);		params.Jt_9[i] = Jacobian_SoT(8,i);	
// 		params.Jt_10[i] = Jacobian_SoT(9,i);	params.Jt_11[i] = Jacobian_SoT(10,i);	params.Jt_12[i] = Jacobian_SoT(11,i);
// 		// right hand 
// 		params.Jt_13[i] = Jacobian_SoT(12,i);	params.Jt_14[i] = Jacobian_SoT(13,i);	params.Jt_15[i] = Jacobian_SoT(14,i);	
// 		params.Jt_16[i] = Jacobian_SoT(15,i);	params.Jt_17[i] = Jacobian_SoT(16,i);	params.Jt_18[i] = Jacobian_SoT(17,i);
// 		// left foot
// 		params.Jt_19[i] = Jacobian_SoT(18,i);	params.Jt_20[i] = Jacobian_SoT(19,i);	params.Jt_21[i] = Jacobian_SoT(20,i);	
// 		params.Jt_22[i] = Jacobian_SoT(21,i);	params.Jt_23[i] = Jacobian_SoT(22,i);	params.Jt_24[i] = Jacobian_SoT(23,i);
// 		// right foot 
// 		params.Jt_25[i] = Jacobian_SoT(24,i);	params.Jt_26[i] = Jacobian_SoT(25,i);	params.Jt_27[i] = Jacobian_SoT(26,i);	
// 		params.Jt_28[i] = Jacobian_SoT(27,i);	params.Jt_29[i] = Jacobian_SoT(28,i);	params.Jt_30[i] = Jacobian_SoT(29,i);
// 	}

// 	// J_T_Wq_J
// // J_T_Wq_ddot

// 	// desired centroidal momentum variation :Hdot_d -  c_X^-T_b *Jf^T_b * F_m - Gf
// 	for(int i=0; i<6; i++)
// 	{
// 		// left foot
// 		params.XJbL_1[i] = ioSM.Centroidal_Wrench_Map_lfoot(0,i);	params.XJbL_2[i] = ioSM.Centroidal_Wrench_Map_lfoot(1,i); 	params.XJbL_3[i] = ioSM.Centroidal_Wrench_Map_lfoot(2,i);
// 		params.XJbL_4[i] = ioSM.Centroidal_Wrench_Map_lfoot(3,i);	params.XJbL_5[i] = ioSM.Centroidal_Wrench_Map_lfoot(4,i); 	params.XJbL_6[i] = ioSM.Centroidal_Wrench_Map_lfoot(5,i);
// 		// right foot
// 		params.XJbR_1[i] = ioSM.Centroidal_Wrench_Map_rfoot(0,i);	params.XJbR_2[i] = ioSM.Centroidal_Wrench_Map_rfoot(1,i); 	params.XJbR_3[i] = ioSM.Centroidal_Wrench_Map_rfoot(2,i);
// 		params.XJbR_4[i] = ioSM.Centroidal_Wrench_Map_rfoot(3,i);	params.XJbR_5[i] = ioSM.Centroidal_Wrench_Map_rfoot(4,i); 	params.XJbR_6[i] = ioSM.Centroidal_Wrench_Map_rfoot(5,i);
// 		// centroidal wrench
// 		params.b2[i] = desired_centroidal_wrench(i);  // -h_b + Jf^T_b * F_m
// 	}
	
// 	// -h + Jf^T * F_m 
// 	for(int i=0; i<30; i++) { params.b3[i] = desired_StackOfTasks(i) - dotJacobianDq_SoT(i);  }
// 	//
// 	// for(int i=0; i<6; i++) { params.b3[30+i] = 0.0;  }
// 	for(int i=0; i<6; i++) { 
// 		params.b3[30+i] = 0.0*desired_posture_acceleration(i);  
// 	}

// 	//
// 	for(int i=0; i<n_actuatedDofs; i++) { params.b3[36+i] = desired_posture_acceleration(6+i);  }

// 	// Velocity constraints on the feet
// 	for(int i=0; i<6; i++)
// 	{
// 		params.b4[i]    = 0.0*ioSM.wbTS.lfoot.Velo(i); // 0.0;		//left foot
// 		params.b4[i+6]  = 0.0*ioSM.wbTS.rfoot.Velo(i); // 0.0;		// right foot			
// 	}

// 	// joints limits
// 	for(int i=0; i<n_actuatedDofs; i++) 
// 	{
// 		// torque
// 		params.T_max[i]	=  WbConstraints.torqueConstraint_vector(i);
// 		// velocity
// 		params.v_max[i] =  WbConstraints.VelocityLimitsVector(i);
// 		// position
// 		params.q_max[i] =  WbConstraints.JointLimitsVector(i);
// 		params.q_min[i] = -WbConstraints.JointLimitsVector(n_actuatedDofs+i);
// 	}

// 	// Inequality constraints on the feet contact
// 	for(int i=0; i<6; i++)
// 	{
// 		// left foot
// 		params.CL_1[i]  = WbConstraints.IneqConstraintMatrix_lf(0,i);		params.CL_2[i]  = WbConstraints.IneqConstraintMatrix_lf(1,i);		
// 		params.CL_3[i]  = WbConstraints.IneqConstraintMatrix_lf(2,i);		params.CL_4[i]  = WbConstraints.IneqConstraintMatrix_lf(3,i);			
// 		params.CL_5[i]  = WbConstraints.IneqConstraintMatrix_lf(4,i);		params.CL_6[i]  = WbConstraints.IneqConstraintMatrix_lf(5,i);			
// 		params.CL_7[i]  = WbConstraints.IneqConstraintMatrix_lf(6,i);		params.CL_8[i]  = WbConstraints.IneqConstraintMatrix_lf(7,i);			
// 		params.CL_9[i]  = WbConstraints.IneqConstraintMatrix_lf(8,i);		params.CL_10[i] = WbConstraints.IneqConstraintMatrix_lf(9,i);		
// 		params.CL_11[i] = WbConstraints.IneqConstraintMatrix_lf(10,i);		
// 		// right foot
// 		params.CR_1[i]  = WbConstraints.IneqConstraintMatrix_rf(0,i);		params.CR_2[i]  = WbConstraints.IneqConstraintMatrix_rf(1,i);				
// 		params.CR_3[i]  = WbConstraints.IneqConstraintMatrix_rf(2,i);		params.CR_4[i]  = WbConstraints.IneqConstraintMatrix_rf(3,i);		
// 		params.CR_5[i]  = WbConstraints.IneqConstraintMatrix_rf(4,i);		params.CR_6[i]  = WbConstraints.IneqConstraintMatrix_rf(5,i);		
// 		params.CR_7[i]  = WbConstraints.IneqConstraintMatrix_rf(6,i);		params.CR_8[i]  = WbConstraints.IneqConstraintMatrix_rf(7,i);		
// 		params.CR_9[i]  = WbConstraints.IneqConstraintMatrix_rf(8,i);		params.CR_10[i] = WbConstraints.IneqConstraintMatrix_rf(9,i);		
// 		params.CR_11[i] = WbConstraints.IneqConstraintMatrix_rf(10,i);
// 	}
// 	//
// 	// Inequality constraints on the feet contact
// 	for(int i=0; i<6; i++)
// 	{
// 		params.copMxl_1[i]  = CoP_constraint_matrix_lfoot(0,i);		params.copMxl_2[i]  = CoP_constraint_matrix_lfoot(1,i);		//left foot
// 		params.copMxr_1[i]  = CoP_constraint_matrix_rfoot(0,i);		params.copMxr_2[i]  = CoP_constraint_matrix_rfoot(1,i);		// right foot			
// 	}
// }


void InverseDynamics::load_default_data()
{
	//
	double ref_period = 4.0;
	double omega = 2.0*M_PI/ref_period;

	params.dt[0] = run_period_sec;
	params.aL[0] = 1.0;
	params.aR[0] = 1.0; 
	params.dwd[0] = desired_weight_prortion_rfoot;
	params.aw[0]  = 1.0;



	// weight on acceleration
	// for(int i=0; i<n_actuatedDofs+6; i++) { params.Qx[i] = ctrl_param.Weight_accel(i); }


	// -----------------------------------------------------------------------------------
	// weight on torque
	for(int i=0; i<n_actuatedDofs; i++) { params.Qy[i] = ctrl_param.Weight_tau(i); }
	// weight on contact forces
	for(int i=0; i<12; i++) { params.Qz[i] = 1e-2*ctrl_param.Weight_F_contact(i); }
	// -----------------------------------------------------------------------------------

	// weight of the desired centroidal momentum  w[0--5]
	for(int i=0; i<6; i++)  params.Qw[i]   = 10.*task_weight[0](i); //ctrl_param.Weight_SoT(i); }
	//
	for(int i=0; i<5; i++) { params.Qw[6+i] = ctrl_param.Weight_CoP(i); } //weight_cop_lfoot weight_cop_rfoot weight_distribution_rfoot


	MatrixXd Jacobian_posture = MatrixXd::Zero(n_actuatedDofs, n_actuatedDofs+6);
	Jacobian_posture.rightCols(n_actuatedDofs) = MatrixXd::Identity(n_actuatedDofs,n_actuatedDofs);
	//
	// VectorXd posture_weight = Eigen::VectorXd::Ones(n_actuatedDofs);
	// // posture_weight.segment( 0, 3) << 3.5, 2.5, 1.; 					// torso
	// posture_weight *= 2.0;
	//
	J_T_Wx_J.setZero();
	J_T_Wx_Xdot.setZero();
	//
	J_T_Wx_J    +=  ioSM.Jacobian_centro_momentum.transpose() * task_weight[0].asDiagonal() * ioSM.Jacobian_centro_momentum;
	J_T_Wx_Xdot -=  ioSM.Jacobian_centro_momentum.transpose() * task_weight[0].asDiagonal() * des_X_dot_EE[0];
	//
	J_T_Wx_J    +=  ioSM.Jacobian_left_hand.transpose() * task_weight[1].asDiagonal() * ioSM.Jacobian_left_hand;
	J_T_Wx_Xdot -=  ioSM.Jacobian_left_hand.transpose() * task_weight[1].asDiagonal() * des_X_dot_EE[1];
	//
	J_T_Wx_J    +=  ioSM.Jacobian_right_hand.transpose() * task_weight[2].asDiagonal() * ioSM.Jacobian_right_hand;
	J_T_Wx_Xdot -=  ioSM.Jacobian_right_hand.transpose() * task_weight[2].asDiagonal() * des_X_dot_EE[2];
	//
	J_T_Wx_J    +=  ioSM.Jacobian_left_foot.transpose() * task_weight[3].asDiagonal() * ioSM.Jacobian_left_foot;
	J_T_Wx_Xdot -=  ioSM.Jacobian_left_foot.transpose() * task_weight[3].asDiagonal() * des_X_dot_EE[3];
	//
	J_T_Wx_J    +=  ioSM.Jacobian_right_foot.transpose() * task_weight[4].asDiagonal() * ioSM.Jacobian_right_foot;
	J_T_Wx_Xdot -=  ioSM.Jacobian_right_foot.transpose() * task_weight[4].asDiagonal() * des_X_dot_EE[4];
	//
	J_T_Wx_J    +=  ioSM.Jacobian_pelvis.bottomRows(3).transpose() * task_weight[5].tail(3).asDiagonal() * ioSM.Jacobian_pelvis.bottomRows(3);
	J_T_Wx_Xdot -=  ioSM.Jacobian_pelvis.bottomRows(3).transpose() * task_weight[5].tail(3).asDiagonal() * des_X_dot_EE[5].tail(3);
	//
	J_T_Wx_J    +=  ioSM.Jacobian_Chest.bottomRows(3).transpose() * task_weight[6].tail(3).asDiagonal() * ioSM.Jacobian_Chest.bottomRows(3);
	J_T_Wx_Xdot -=  ioSM.Jacobian_Chest.bottomRows(3).transpose() * task_weight[6].tail(3).asDiagonal() * des_X_dot_EE[6].tail(3);

	J_T_Wx_J    +=  Jacobian_posture.transpose() * posture_weight.asDiagonal() * Jacobian_posture;			// 0.01
	J_T_Wx_Xdot -=  Jacobian_posture.transpose() * posture_weight.asDiagonal() * 2.0*des_q_dot.tail(n_actuatedDofs);		// 0.01

	// regularization
	// J_T_Wx_J += Weight_acceleration.asDiagonal() * (1./ioSM.massMatrix(0,0)) * ioSM.massMatrix; // MatrixXd::Identity(n_actuatedDofs+6, n_actuatedDofs+6);
	J_T_Wx_J += Weight_acceleration.asDiagonal() * MatrixXd::Identity(n_actuatedDofs+6, n_actuatedDofs+6); //ioSM.massMatrix; //

	J_T_Wx_J *=0.5;

	// std::cout << " J_T_Wx_J is : \n " << J_T_Wx_J << std::endl;


	// Hessian matrix
	memcpy(params.Qx, J_T_Wx_J.data(), sizeof(double) * J_T_Wx_J.size());
	memcpy(params.px, &J_T_Wx_Xdot[0], sizeof(double)*(n_actuatedDofs+6)); 				// weight acceleration and desired posture

	// -----------------------------------------------------------------------------------
	int N_DOF = n_actuatedDofs+6;

	// Mass matrix
	for(int i=0; i<n_actuatedDofs+6; i++)
	{
		// for(int i=0;i<AIR_N_U;i++)
		memcpy(params.M[i+1], &ioSM.massMatrix(0,i),     sizeof(double)*(N_DOF));
		//
		params.b1[i] = bias_fcontact_torque(i);  // -h + Jf^T * F_m
	}

	// Task Jacobian
	for(int i=0; i<n_actuatedDofs+6; i++)
	{
		// // centroidal 
		// params.Jt_1[i] = Jacobian_SoT(0,i);		params.Jt_2[i] = Jacobian_SoT(1,i);		params.Jt_3[i] = Jacobian_SoT(2,i);	
		// params.Jt_4[i] = Jacobian_SoT(3,i);		params.Jt_5[i] = Jacobian_SoT(4,i);		params.Jt_6[i] = Jacobian_SoT(5,i);
		// // left hand 
		// params.Jt_7[i] = Jacobian_SoT(6,i);		params.Jt_8[i] = Jacobian_SoT(7,i);		params.Jt_9[i] = Jacobian_SoT(8,i);	
		// params.Jt_10[i] = Jacobian_SoT(9,i);	params.Jt_11[i] = Jacobian_SoT(10,i);	params.Jt_12[i] = Jacobian_SoT(11,i);
		// // right hand 
		// params.Jt_13[i] = Jacobian_SoT(12,i);	params.Jt_14[i] = Jacobian_SoT(13,i);	params.Jt_15[i] = Jacobian_SoT(14,i);	
		// params.Jt_16[i] = Jacobian_SoT(15,i);	params.Jt_17[i] = Jacobian_SoT(16,i);	params.Jt_18[i] = Jacobian_SoT(17,i);
		// left foot
		params.Jt_19[i] = Jacobian_SoT(18,i);	params.Jt_20[i] = Jacobian_SoT(19,i);	params.Jt_21[i] = Jacobian_SoT(20,i);	
		params.Jt_22[i] = Jacobian_SoT(21,i);	params.Jt_23[i] = Jacobian_SoT(22,i);	params.Jt_24[i] = Jacobian_SoT(23,i);
		// right foot 
		params.Jt_25[i] = Jacobian_SoT(24,i);	params.Jt_26[i] = Jacobian_SoT(25,i);	params.Jt_27[i] = Jacobian_SoT(26,i);	
		params.Jt_28[i] = Jacobian_SoT(27,i);	params.Jt_29[i] = Jacobian_SoT(28,i);	params.Jt_30[i] = Jacobian_SoT(29,i);
	}
	// desired centroidal momentum variation :Hdot_d -  c_X^-T_b *Jf^T_b * F_m - Gf
	for(int i=0; i<6; i++)
	{
		// left foot
		params.XJbL_1[i] = ioSM.Centroidal_Wrench_Map_lfoot(0,i);	params.XJbL_2[i] = ioSM.Centroidal_Wrench_Map_lfoot(1,i); 	params.XJbL_3[i] = ioSM.Centroidal_Wrench_Map_lfoot(2,i);
		params.XJbL_4[i] = ioSM.Centroidal_Wrench_Map_lfoot(3,i);	params.XJbL_5[i] = ioSM.Centroidal_Wrench_Map_lfoot(4,i); 	params.XJbL_6[i] = ioSM.Centroidal_Wrench_Map_lfoot(5,i);
		// right foot
		params.XJbR_1[i] = ioSM.Centroidal_Wrench_Map_rfoot(0,i);	params.XJbR_2[i] = ioSM.Centroidal_Wrench_Map_rfoot(1,i); 	params.XJbR_3[i] = ioSM.Centroidal_Wrench_Map_rfoot(2,i);
		params.XJbR_4[i] = ioSM.Centroidal_Wrench_Map_rfoot(3,i);	params.XJbR_5[i] = ioSM.Centroidal_Wrench_Map_rfoot(4,i); 	params.XJbR_6[i] = ioSM.Centroidal_Wrench_Map_rfoot(5,i);
		// centroidal wrench
		params.b2[i] = desired_centroidal_wrench(i);  // -h_b + Jf^T_b * F_m
	}
	// joints limits
	for(int i=0; i<n_actuatedDofs; i++) 
	{
		// torque
		params.T_max[i]	=  WbConstraints.torqueConstraint_vector(i);
		// velocity
		params.v_max[i] =  WbConstraints.VelocityLimitsVector(i);
		// position
		params.q_max[i] =  WbConstraints.JointLimitsVector(i);
		params.q_min[i] = -WbConstraints.JointLimitsVector(n_actuatedDofs+i);
	}

	// Inequality constraints on the feet contact
	for(int i=0; i<6; i++)
	{
		// left foot
		params.CL_1[i]  = WbConstraints.IneqConstraintMatrix_lf(0,i);		params.CL_2[i]  = WbConstraints.IneqConstraintMatrix_lf(1,i);		
		params.CL_3[i]  = WbConstraints.IneqConstraintMatrix_lf(2,i);		params.CL_4[i]  = WbConstraints.IneqConstraintMatrix_lf(3,i);			
		params.CL_5[i]  = WbConstraints.IneqConstraintMatrix_lf(4,i);		params.CL_6[i]  = WbConstraints.IneqConstraintMatrix_lf(5,i);			
		params.CL_7[i]  = WbConstraints.IneqConstraintMatrix_lf(6,i);		params.CL_8[i]  = WbConstraints.IneqConstraintMatrix_lf(7,i);			
		params.CL_9[i]  = WbConstraints.IneqConstraintMatrix_lf(8,i);		params.CL_10[i] = WbConstraints.IneqConstraintMatrix_lf(9,i);		
		params.CL_11[i] = WbConstraints.IneqConstraintMatrix_lf(10,i);		
		// right foot
		params.CR_1[i]  = WbConstraints.IneqConstraintMatrix_rf(0,i);		params.CR_2[i]  = WbConstraints.IneqConstraintMatrix_rf(1,i);				
		params.CR_3[i]  = WbConstraints.IneqConstraintMatrix_rf(2,i);		params.CR_4[i]  = WbConstraints.IneqConstraintMatrix_rf(3,i);		
		params.CR_5[i]  = WbConstraints.IneqConstraintMatrix_rf(4,i);		params.CR_6[i]  = WbConstraints.IneqConstraintMatrix_rf(5,i);		
		params.CR_7[i]  = WbConstraints.IneqConstraintMatrix_rf(6,i);		params.CR_8[i]  = WbConstraints.IneqConstraintMatrix_rf(7,i);		
		params.CR_9[i]  = WbConstraints.IneqConstraintMatrix_rf(8,i);		params.CR_10[i] = WbConstraints.IneqConstraintMatrix_rf(9,i);		
		params.CR_11[i] = WbConstraints.IneqConstraintMatrix_rf(10,i);
	}
	//
	// Inequality constraints on the feet contact
	for(int i=0; i<6; i++)
	{
		params.copMxl_1[i]  = CoP_constraint_matrix_lfoot(0,i);		params.copMxl_2[i]  = CoP_constraint_matrix_lfoot(1,i);		//left foot
		params.copMxr_1[i]  = CoP_constraint_matrix_rfoot(0,i);		params.copMxr_2[i]  = CoP_constraint_matrix_rfoot(1,i);		// right foot			
	}
}
