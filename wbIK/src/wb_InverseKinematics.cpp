
// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** Class wb_InverseKinematics

*/
#include <iostream>
#include <cmath>
#include <stdexcept>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>

#include "wb_InverseKinematics.h"

wbIK1_Vars 			wbIK1_vars;
wbIK1_Params 		wbIK1_params;
wbIK1_Workspace 	wbIK1_work;
wbIK1_Settings 		wbIK1_settings;

wbIK2_Vars 			wbIK2_vars;
wbIK2_Params 		wbIK2_params;
wbIK2_Workspace 	wbIK2_work;
wbIK2_Settings 		wbIK2_settings;

wbIK3_Vars 			wbIK3_vars;
wbIK3_Params 		wbIK3_params;
wbIK3_Workspace 	wbIK3_work;
wbIK3_Settings 		wbIK3_settings;

wbIK4_Vars 			wbIK4_vars;
wbIK4_Params 		wbIK4_params;
wbIK4_Workspace 	wbIK4_work;
wbIK4_Settings 		wbIK4_settings;

legIK_Vars 			legIK_vars;
legIK_Params 		legIK_params;
legIK_Workspace 	legIK_work;
legIK_Settings 		legIK_settings;

CoMclp_Vars 		CoMclp_vars;
CoMclp_Params 		CoMclp_params;
CoMclp_Workspace 	CoMclp_work;
CoMclp_Settings 	CoMclp_settings;

wb_InverseKinematics::wb_InverseKinematics(){}

wb_InverseKinematics::~wb_InverseKinematics(){
		//
	Out_joints_pos.close();
	Out_joints_vel.close();
}

void wb_InverseKinematics::InitializeWBIK(WbRobotModel& robot_model_, VectorXd jts_position, Matrix4d WHB, double gain_, int count_max_, double epilon_, double step_)
{
	std::cout << " WBIK INITIALIZE OK : \t" << 0.0 << std::endl;
	//
	// // initialization of the cvxgen solver for the wb inverse kinematics
	wbIK1_set_defaults();
	wbIK1_setup_indexing();

	wbIK2_set_defaults();
	wbIK2_setup_indexing();

	wbIK3_set_defaults();
	wbIK3_setup_indexing();

	wbIK4_set_defaults();
	wbIK4_setup_indexing();

	legIK_set_defaults();
	legIK_setup_indexing();

	CoMclp_set_defaults();
	CoMclp_setup_indexing();

	// yarp::os::LockGuard guard(m_mutex);
	// convergence parameters
	virtual_sampTime	= step_;
	epsilon 	 		= epilon_;
	count_max  	 		= count_max_;
	virtual_gain 		= gain_;
	alpha_q				= 0.5;
	beta_q				= 2.* sqrt(alpha_q);

	// count_max = 20.;
	//
	alpha_g = 0.05;
	gain_vector.setOnes();
	gain_vector_0.setOnes();
	var_gain.setZero();
	// gain_vector.head(3) *= -virtual_gain;
	// gain_vector.tail(3) *= -0.50*virtual_gain;
	gain_vector_0.head(3) = -Vector3d(virtual_gain, virtual_gain, virtual_gain); //-virtual_gain;
	gain_vector_0.tail(3) = -0.50*Vector3d(virtual_gain, virtual_gain, virtual_gain); //-0.50*virtual_gain;

	// m_mutex.lock();		// ====================================================================
	// 
	nJts = robot_model_.getDoFs();
	minJointLimits.resize(robot_model_.getDoFs());
	maxJointLimits.resize(robot_model_.getDoFs());
	maxVeloLimit.resize(robot_model_.getDoFs());

	// robot_model_.getJointsLimits();
	minJointLimits = robot_model_.m_min_jtsLimits;
	minJointLimits(4)  = 15.*M_PI/180.;
	minJointLimits(11) = 15.*M_PI/180.;
	maxJointLimits = robot_model_.m_max_jtsLimits;
	maxJointLimits(nJts-9) = -5.0*M_PI/180.;
	maxJointLimits(nJts-3) = -5.0*M_PI/180.;
	maxVeloLimit   = robot_model_.m_velocitySaturationLimit;
	//
	q_0.resize(nJts);
	// q_0 = 0.5*(minJointLimits + maxJointLimits);	

	// q_0.segment( 0, 3) << 0.000459142, 	-0.000422478,    0.0588484;
	// q_0.segment( 3, 7) << -0.500256,     	0.503886,   0.00208811,     0.776355,  -7.3435e-05,  0.000213336,  1.10298e-07;   
	// q_0.segment(10, 7) << -0.500408,     	0.504419,    0.0018862,     0.776468, -8.04962e-05,  0.000150285, -8.32686e-07;     
	// q_0.segment(17, 6) << 0.432508,  	 -0.00215064, -0.000830919,    -0.898362,    -0.454761,   0.00122362;
 // 	q_0.segment(23, 6) << 0.448097,  	 0.000123756,   0.00349643,    -0.897924,    -0.438732,   0.00174252; 
	q_0.segment( 0, 3) << 0.000459142, 	-0.000422478,    0.0588484;
	q_0.segment( 3, 7) << -0.500256,     	0.603886,   0.00208811,     0.776355,  -7.3435e-05,  0.000213336,  1.10298e-07;   
	q_0.segment(10, 7) << -0.500408,     	0.603886,   0.00208811,     0.776355,  -7.3435e-05,  0.000213336,  1.10298e-07;     
	q_0.segment(17, 6) << 0.432508,  	 -0.00215064, -0.000830919,    -0.898362,    -0.454761,   0.00122362;
 	q_0.segment(23, 6) << 0.448097,  	 0.000123756,   0.00349643,    -0.897924,    -0.438732,   0.00174252; 
	//
	EE[0] = "left_hand"; 	EE[1] = "right_hand"; 
	EE[2] = "left_foot"; 	EE[3] = "right_foot"; 
	EE[4] = "CoM"; 			EE[5] = "Pelvis"; //"centroidal_momentum"; 
	EE[6] = "Pelvis";		EE[7] = "Chest";
	//
	n_task = 33; //36;
	//
	for (int i=0; i<8; i++)	{
		Jacobian_EE[i].resize(6, nJts+6);
		Pose_EE[i].setZero();
		error_pose[i].setZero();
	}


	//
	m_massMatrix.resize(nJts+6, nJts+6);
	//
	J_SoT.resize(n_task, nJts+6);
	J_SoT.setZero();
	X_dot_SoT.resize(n_task);	
	X_dot_SoT.setZero();
	X_dot_SoT_0.resize(n_task);	
	X_dot_SoT_0.setZero();
	hard.resize(n_task);
	hard.setZero();
	
	// virt_jts_pos = jts_position;  	// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
	// virt_WHB	 = world_H_fBase;		// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx	
	virt_jts_pos.resize(nJts);
	virt_jts_vel.resize(nJts);
	virt_jts_pos = jts_position;
	virt_jts_vel.setZero();
	virt_VB.setZero(); 
	virt_jts_pos   = jts_position;
	virt_jts_pos_0 = jts_position;
	virt_WHB	 = WHB;
	//
	freeze.resize(nJts+6);
	freeze.setZero();
	isFreeze = false;
	// freeze.segment(6,17) = VectorXd::Ones(17);


	//
	coef_grad = 1.0;

	// QP Weight
	// =================================================
	// Acceleration weight in QP
	Weight_velocity = Eigen::VectorXd::Ones(nJts+6);
	Weight_velocity.segment( 0, 6) << 50., 50., 10., 5., 5., 5.; 				// floating base
	Weight_velocity.segment( 6, 3) << 2.5, 2.5, 1.; 							// torso
	Weight_velocity.segment( 9, 7) << 1., 1., 1., 1., 1., 1., 1.; 				// left arm
	Weight_velocity.segment(16, 7) << 1., 1., 1., 1., 1., 1., 1.; 				// right arm
	Weight_velocity.segment(23, 6) << 1., 1., 1., 1., 1., 1.; 					// left leg 	leg hip yaw *= 2.5;
	Weight_velocity.segment(29, 6) << 1., 1., 1., 1., 1., 1.; 					// right leg 	leg hip yaw *= 2.5;
	Weight_velocity *= 1e-4;

// Slack variable weight in QP	
	Weight_slack = Eigen::VectorXd::Zero(n_task); //.resize(n_task);
	Weight_slack.segment(0,  6) << 5.0, 5.0, 5.0, 3.0, 3.0, 3.0; 				// weight_lhand;
	Weight_slack.segment(6,  6) << 5.0, 5.0, 5.0, 3.0, 3.0, 3.0; 				// weight_rhand;
	Weight_slack.segment(12, 3) << 15.e-0, 15.e-0, 5.e-1; 						// weight_CoMp;  5.e-1, 5.e-1, 5.e-1; 
	Weight_slack.segment(15, 3) << 2.e-1, 1.e-2, 1.e-2; 						// weight_CMa;
	Weight_slack.segment(18, 3) << 1.e-5, 1.e-5, 1.e-5; 						// weight_Chest;
	Weight_slack.segment(21, 6) << 100.0, 100.0, 100.0, 100.0, 100.0, 100.0; 	// weight_lfoot;
	Weight_slack.segment(27, 6) << 100.0, 100.0, 100.0, 100.0, 100.0, 100.0; 	// weight_rfoot;

	// Weight_slack.segment(12, 2)  *= 10.;  // x and y CoM
	Weight_slack.segment(21, 12) *= 10.;  // feet
	// Weight_slack *= 1e+1;

	// QP solution
	q_dot_star   	 = Eigen::VectorXd::Zero(nJts+6);
	q_dot_star_0 	 = Eigen::VectorXd::Zero(nJts+6);     	// 
	des_q_dot    	 = Eigen::VectorXd::Zero(nJts+6);     	// 
	des_jts_pos	 	 = Eigen::VectorXd::Zero(nJts);     	// 
	des_WHB		 	 = Eigen::MatrixXd::Zero(4,4);     		// 
	q_dot_star_legIK = Eigen::VectorXd::Zero(18);
	// 
	// support polygon points of the foot
	PtsInFoot.resize(2,4);
    // x                // y
    // PtsInFoot(0,0) =  0.065;  PtsInFoot(1,0) =  0.015;  // point 1
    // PtsInFoot(0,1) = -0.030;  PtsInFoot(1,1) =  0.015;  // point 2
    // PtsInFoot(0,2) = -0.030;  PtsInFoot(1,2) = -0.015;  // point 3
    // PtsInFoot(0,3) =  0.065;  PtsInFoot(1,3) = -0.015;  // point 4

    // PtsInFoot(0,0) =  0.055;  PtsInFoot(1,0) =  0.015;  // point 1
    // PtsInFoot(0,1) = -0.015;  PtsInFoot(1,1) =  0.015;  // point 2
    // PtsInFoot(0,2) = -0.015;  PtsInFoot(1,2) = -0.015;  // point 3
    // PtsInFoot(0,3) =  0.055;  PtsInFoot(1,3) = -0.015;  // point 4

    PtsInFoot(0,0) =  0.100;  PtsInFoot(1,0) =  0.015;  // point 1		  p1 ---- p4
    PtsInFoot(0,1) = -0.015;  PtsInFoot(1,1) =  0.015;  // point 2			|	 |
    PtsInFoot(0,2) = -0.015;  PtsInFoot(1,2) = -0.015;  // point 3			|	 |
    PtsInFoot(0,3) =  0.100;  PtsInFoot(1,3) = -0.015;  // point 4  	 p2 |____| p3
   
    // Initialization of the QPOASES SOLVER
    // =======================================
    robot_model_.getLinkPose(jts_position, WHB, "left_hand",   des_X_EE[0]);
    robot_model_.getLinkPose(jts_position, WHB, "right_hand",  des_X_EE[1]);
    robot_model_.getLinkPose(jts_position, WHB, "left_foot",   des_X_EE[2]);
    robot_model_.getLinkPose(jts_position, WHB, "right_foot",  des_X_EE[3]);
    robot_model_.getLinkPose(jts_position, WHB, "CoM",         des_X_EE[4]);
    robot_model_.getLinkPose(jts_position, WHB, "Pelvis",      des_X_EE[6]);
    robot_model_.getLinkPose(jts_position, WHB, "Chest",       des_X_EE[7]);   
    des_X_EE[5] = des_X_EE[6];

	// string stanceFoot = "left";
	// for(int i=0; i<8; i++) des_X_EE[i] = des_X_EE_[i];
	// this->get_wb_IK(robot_model_, jts_position, WHB, des_X_EE, stanceFoot, des_X_EE[2], des_X_EE[3]);
	// this->solve_QPoasesInit(robot_model_, des_X_EE[2], des_X_EE[3], stanceFoot); 

	Jacobian_posture = MatrixXd::Zero(nJts, nJts+6);
	Jacobian_posture.rightCols(nJts) = MatrixXd::Identity(nJts,nJts);

	J_T_Wx_J    = MatrixXd::Zero(nJts+6, nJts+6);
	J_T_Wx_Xdot = VectorXd::Zero(nJts+6); 	
	//
	Jac_lf.resize(6,18);		Jac_lf.setZero();
	Jac_rf.resize(6,18);		Jac_rf.setZero();
	Jac_com.resize(3,18);		Jac_com.setZero();
	Jl_T_Wx_Jl.resize(18,18);	Jl_T_Wx_Jl.setZero();
	Jl_T_Wx_Xdot.resize(18);	Jl_T_Wx_Xdot.setZero();
	//
	posture_weight = Eigen::VectorXd::Ones(nJts);
	posture_weight.segment( 0, 3) << 3.5, 2.5, 1.; 					// torso

	posture_weight_default	= Eigen::VectorXd::Ones(nJts);		posture_weight_default.segment( 0, 3) << 3.5, 2.5, 1.;
	posture_weight_reach	= Eigen::VectorXd::Ones(nJts);		posture_weight_reach.segment( 0, 3) << 3.5, 5.5, 1.;

	posture_weight_retract = Eigen::VectorXd::Ones(nJts);
	posture_weight_retract.segment( 0,3) *= 5.0;
	posture_weight_retract.segment( 3,3) *= 3.0;
	posture_weight_retract.segment(10,3) *= 3.0;
	posture_weight_retract *= 2.0;
	// posture_weight.segment( 3, 7) << 1., 1., 1., 1., 1., 1., 1.; 	// left arm
	// posture_weight.segment(10, 7) << 1., 1., 1., 1., 1., 1., 1.; 	// right arm
	// posture_weight.segment(17, 6) << 1., 1., 1., 1., 1., 1.; 		// left leg 	leg hip yaw *= 2.5;
	// posture_weight.segment(23, 6) << 1., 1., 1., 1., 1., 1.; 		// right leg 	leg hip yaw *= 2.5;

	// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	task_weight_default[0] <<     5.0,    5.0,   5.0,   1.00,  1.00,  1.00;  	// weight_lhand; 
	task_weight_default[1] <<     5.0,    5.0,   5.0,   1.00,  1.00,  1.00;  	// weight_rhand; 
	task_weight_default[2] <<   100.0,  100.0, 100.0,  100.0, 100.0, 100.0; 	// weight_lfoot; 
	task_weight_default[3] <<   100.0,  100.0, 100.0,  100.0, 100.0, 100.0; 	// weight_rfoot;
	task_weight_default[4] <<   5.e-0,  5.e-0, 5.e-1,   0.00,  0.00,  0.00;  	// weight_CoMp; 
	task_weight_default[5] <<    0.00,   0.00,  0.00,  2.e-1, 1.e-2, 1.e-2;  	// weight_pelvis;  	
	task_weight_default[6] <<    0.00,   0.00,  0.00,  1.e-5, 1.e-5, 1.e-5;  	// weight_chest; 
	task_weight_default[7] <<    0.00,   0.00,  0.00,  2.e-1, 1.e-2, 1.e-2;  	// weight_pelvis; 
	task_weight_default[2] *=0.5;
	task_weight_default[3] *=10.;
	task_weight_default[4] *=4.;
	task_weight_default[0] *=4.0;
	task_weight_default[1] *=4.0;

	task_weight_reach[0] <<     5.0,    5.0,   5.0,   1.00,  1.00,  1.00;  	// weight_lhand; 
	task_weight_reach[1] <<     5.0,    5.0,   5.0,   1.00,  1.00,  1.00;  	// weight_rhand; 
	task_weight_reach[2] <<   100.0,  100.0, 100.0,  100.0, 100.0, 100.0; 	// weight_lfoot; 
	task_weight_reach[3] <<   100.0,  100.0, 100.0,  100.0, 100.0, 100.0; 	// weight_rfoot;
	task_weight_reach[4] << 15.e-0, 15.e-0, 5.e-1,   0.00,  0.00,  0.00;  	// weight_CoMp; 
	task_weight_reach[5] <<    0.00,   0.00,  0.00,  2.e-1, 1.e-2, 5.e-2;  	// weight_pelvis;  	
	task_weight_reach[6] <<    0.00,   0.00,  0.00,  1.e-5, 1.e-5, 5.e-5;  	// weight_chest; 
	task_weight_reach[7] <<    0.00,   0.00,  0.00,  2.e-1, 1.e-2, 5.e-2;  	// weight_pelvis; 

	task_weight_reach[2] *=10.;
	task_weight_reach[3] *=10.;
	task_weight_reach[4] *=2.;
	task_weight_reach[0] *=20.0;
	task_weight_reach[1] *=20.0;

	task_weight_reach[5] *=10.0;
	task_weight_reach[6] *=10.0;


	task_weight_reach_v[0] <<     5.0,    5.0,   5.0,   1.00,  1.00,  1.00;  	// weight_lhand; 
	task_weight_reach_v[1] <<     5.0,    5.0,   5.0,   5.00,  5.00,  5.00;  	// weight_rhand; 
	task_weight_reach_v[2] <<   100.0,  100.0, 100.0,  100.0, 100.0, 100.0; 	// weight_lfoot; 
	task_weight_reach_v[3] <<   100.0,  100.0, 100.0,  100.0, 100.0, 100.0; 	// weight_rfoot;
	task_weight_reach_v[4] << 15.e-0, 15.e-0, 5.e-1,   0.00,  0.00,  0.00;  	// weight_CoMp; 
	task_weight_reach_v[5] <<    0.00,   0.00,  0.00,  2.e-1, 1.e-2, 5.e-2;  	// weight_pelvis;  	
	task_weight_reach_v[6] <<    0.00,   0.00,  0.00,  1.e-5, 1.e-5, 5.e-5;  	// weight_chest; 
	task_weight_reach_v[7] <<    0.00,   0.00,  0.00,  2.e-1, 1.e-2, 5.e-2;  	// weight_pelvis; 

	task_weight_reach_v[2] *=7.;
	task_weight_reach_v[3] *=7.;
	task_weight_reach_v[4] *=20.;
	// task_weight_reach_v[0] *=10.0;
	// task_weight_reach_v[1] *=10.0;
	task_weight_reach_v[0] *=15.0;
	task_weight_reach_v[1] *=15.0;
	task_weight_reach_v[5] *=10.0;
	task_weight_reach_v[6] *=10.0;

	
	task_weight_fmmcom[0] <<     5.0,    5.0,   5.0,   1.00,  1.00,  1.00; 	// weight_lhand; 
	task_weight_fmmcom[1] <<     5.0,    5.0,   5.0,   1.00,  1.00,  1.00; 	// weight_rhand; 
	task_weight_fmmcom[2] <<   100.0,  100.0, 100.0,  100.0, 100.0, 100.0; 	// weight_lfoot; 
	task_weight_fmmcom[3] <<   100.0,  100.0, 100.0,  100.0, 100.0, 100.0; 	// weight_rfoot;
	task_weight_fmmcom[4] << 15.e-0, 15.e-0, 5.e-1,   0.00,  0.00,  0.00;  	// weight_CoMp; 
	task_weight_fmmcom[5] <<    0.00,   0.00,  0.00,  2.e-1, 1.e-2, 5.e-2;  // weight_pelvis;  	
	task_weight_fmmcom[6] <<    0.00,   0.00,  0.00,  1.e-5, 1.e-5, 5.e-5;  // weight_chest; 
	task_weight_fmmcom[7] <<    0.00,   0.00,  0.00,  2.e-1, 1.e-2, 5.e-2;  // weight_pelvis; 

	task_weight_fmmcom[2] *=10.;
	task_weight_fmmcom[3] *=10.;
	task_weight_fmmcom[4] *=1.;     // 1.
	task_weight_fmmcom[0] *=20.0;
	task_weight_fmmcom[1] *=20.0;

	task_weight_fmmcom[5] *=10.0;
	task_weight_fmmcom[6] *=10.0;



	task_weight_aStep[0] <<     5.0,    5.0,   5.0,   1.00,  1.00,  1.00;  	// weight_lhand; 
	task_weight_aStep[1] <<     5.0,    5.0,   5.0,   1.00,  1.00,  1.00;  	// weight_rhand; 
	task_weight_aStep[2] <<   100.0,  100.0, 100.0,  100.0, 100.0, 100.0; 	// weight_lfoot; 
	task_weight_aStep[3] <<   100.0,  100.0, 100.0,  100.0, 100.0, 100.0; 	// weight_rfoot;
	task_weight_aStep[4] <<   5.e-0,  5.e-0, 5.e-1,   0.00,  0.00,  0.00;  	// weight_CoMp; 
	task_weight_aStep[5] <<    0.00,   0.00,  0.00,  2.e-1, 1.e-2, 1.e-2;  	// weight_pelvis;  	
	task_weight_aStep[6] <<    0.00,   0.00,  0.00,  1.e-5, 1.e-5, 1.e-5;  	// weight_chest; 
	task_weight_aStep[7] <<    0.00,   0.00,  0.00,  2.e-1, 1.e-2, 1.e-2;  	// weight_pelvis; 

	// task_weight_aStep[2] *=0.5;
	// task_weight_aStep[3] *=10.;
	task_weight_aStep[4] *=4.;
	task_weight_aStep[0] *=4.0;
	task_weight_aStep[1] *=4.0;



	task_weight_legIK[0] <<     1.0,    1.0,   1.0,   0.30,  0.30,  0.30;  	// weight_lhand; 
	task_weight_legIK[1] <<     1.0,    1.0,   1.0,   0.30,  0.30,  0.30;  	// weight_rhand; 
	task_weight_legIK[2] <<   100.0,  100.0, 100.0,  100.0, 100.0, 100.0; 	// weight_lfoot; 
	task_weight_legIK[3] <<   100.0,  100.0, 100.0,  100.0, 100.0, 100.0; 	// weight_rfoot;
	task_weight_legIK[4] <<  100.e-0, 100.e-0, 100.e-0,   0.00,  0.00,  0.00;  	// weight_CoMp; 
	// task_weight_legIK[5] <<    0.00,   0.00,  0.00,  2.e-1, 1.e-2, 1.e-2;  	// weight_pelvis;  	
	task_weight_legIK[5] <<    0.00,   0.00,  0.00,  20., 20., 20.;  	// weight_pelvis;  	
	task_weight_legIK[6] <<    0.00,   0.00,  0.00,  1.e-5, 1.e-5, 1.e-5;  	// weight_chest; 
	task_weight_legIK[7] <<    0.00,   0.00,  0.00,  2.e-1, 1.e-2, 1.e-2;  	// weight_pelvis; 

	task_weight_legIK[2] *=10.;
	task_weight_legIK[3] *=10.;
	// task_weight_reach[4] *=0.1;
	task_weight_legIK[0] *=5.0;
	task_weight_legIK[1] *=5.0;
	task_weight_legIK[5] *=2.05;

	task_weight_retract[0] <<     5.0,    5.0,   5.0,   1.00,  1.00,  1.00;  	// weight_lhand; 
	task_weight_retract[1] <<     5.0,    5.0,   5.0,   1.00,  1.00,  1.00;  	// weight_rhand; 
	task_weight_retract[2] <<   100.0,  100.0, 100.0,  100.0, 100.0, 100.0; 	// weight_lfoot; 
	task_weight_retract[3] <<   100.0,  100.0, 100.0,  100.0, 100.0, 100.0; 	// weight_rfoot;
	task_weight_retract[4] <<  15.e-0, 15.e-0, 5.e-0,   0.00,  0.00,  0.00;  	// weight_CoMp; 	15.e-0, 15.e-0, 5.e-1,
	task_weight_retract[5] <<    0.00,   4.e-1,  0.00,  2.e-1, 1.e-2, 5.e-2;  	// weight_pelvis;  	
	task_weight_retract[6] <<    0.00,   0.00,  0.00,  1.e-5, 1.e-5, 5.e-5;  	// weight_pelvis; 
	task_weight_retract[7] <<    0.00,   0.00,  0.00,  2.e-1, 2.e-2, 1.e-1;  	// weight_chest;  2.e-1, 2.e-2, 1.e-2;

	task_weight_retract[2] *=7.;
	task_weight_retract[3] *=7.;
	task_weight_retract[4] *=8.; // 8.0
	task_weight_retract[0] *=3.0; // 2.0
	task_weight_retract[1] *=3.0; // 2.0

	task_weight_retract[5] *=50.0;
	task_weight_retract[7] *=50.0;


	memcpy(task_weight, &task_weight_reach[0], 8 * sizeof *task_weight_reach); 
	// W_EdgesNormalCHull = MatrixXd::Zero(6,6);
	// DistanceEdgesCHull = VectorXd::Zero(6);
	//
	isCoMClamping  =  false;
	max_reach	   =   0.40;
	reach_tol << 0.02, 0.17;

	pxy_CoM    = des_X_EE[4].head(2);
	pxy_CoM_n1 = des_X_EE[4].head(2);
	//
	legIK_batch_start = false;
	ik_cont = false;
	legIK_Done = false;
	l_count = 0;
	g_count = 0;
	l_count_max = 3;

	//
	PointsCHull = MatrixXd::Zero(8,2);

	// data logger
	// ==========================
	string path_log_joints_pos  = "log_virt_joints_pos_.txt";
	string path_log_joints_vel  = "log_virt_joints_vel_.txt";
	Out_joints_pos.open(path_log_joints_pos.c_str());
	Out_joints_vel.open(path_log_joints_vel.c_str());

	return;
}

// bool wb_InverseKinematics::update_model(WbRobotModel& robot_model_, VectorXd q_dot_star_)
bool wb_InverseKinematics::update_model(WbRobotModel& robot_model_, VectorXd q_dot_star_, Vector7d pose_lfoot, Vector7d pose_rfoot, string stanceFoot)
{
	// yarp::os::LockGuard guard(m_mutex);
	//
	q_dot_star_0   = q_dot_star_;
	virt_jts_pos_0 = virt_jts_pos;
	// Update the robot model
	// ======================
	q_dot_star_ = q_dot_star_.cwiseProduct(VectorXd::Ones(nJts+6)-freeze);	 
	// update the joint position and velocity
	virt_jts_pos += q_dot_star_.tail(nJts) * virtual_sampTime;
	// virt_jts_pos += 0.5*(q_dot_star_.tail(nJts) +  q_dot_star_0.tail(nJts))* virtual_sampTime;
	virt_jts_vel   = q_dot_star_;
	// Enforcing joints limits
	for(int i=0; i<robot_model_.getDoFs(); i++)
	{
		// set joint limits hard limits
		if(virt_jts_pos(i) > maxJointLimits(i)){  // upper limit
			virt_jts_pos(i) = maxJointLimits(i) - 0.02;
		} else if(virt_jts_pos(i) < minJointLimits(i)){ // lower limit
			virt_jts_pos(i) = minJointLimits(i) + 0.02;
		} 
	}
	// update the floating base pose and velocity
	// Transforms.UpdatePose_From_VelocityTwist(virtual_sampTime, q_dot_star_.head(6), virt_WHB);
	// virt_VB = q_dot_star_.head(6);
	double t_wbIK = yarp::os::Time::now();
	//
	virt_WHB = get_fbase_pose(robot_model_, pose_lfoot, pose_rfoot, stanceFoot, virt_jts_pos);
	
	// yarp::os::LockGuard guard(m_mutex);
	// m_mutex.lock();
	for (int i=0; i<8; i++){
		robot_model_.getJacobian(virt_jts_pos, virt_WHB, EE[i], Jacobian_EE[i]);							// EE Jacobian
		if(i!=5){ 	robot_model_.getLinkPose(virt_jts_pos, virt_WHB, EE[i],  Pose_EE[i]);	}						// EE_Pose (FWK)	
	}
	Pose_EE[5] = Pose_EE[6];  // assinging the pelvis orientation to the robot center of mass
	// get the mass matrix  m_massMatrix
	robot_model_.getMassMatrix(virt_jts_pos, virt_WHB, m_massMatrix);
	// m_mutex.unlock();

	// // Task Jacobian
	// J_SoT.block(0,  0, 6, nJts+6) = Jacobian_EE[0];							// left hand
	// J_SoT.block(6,  0, 6, nJts+6) = Jacobian_EE[1];							// right hand
	// J_SoT.block(12, 0, 3, nJts+6) = Jacobian_EE[4].topRows(3);				// CoM Position  
	// J_SoT.block(15, 0, 3, nJts+6) = Jacobian_EE[5].bottomRows(3);			// Angular momentum
	// J_SoT.block(18, 0, 3, nJts+6) = Jacobian_EE[7].bottomRows(3);			// Chest
	// J_SoT.block(21, 0, 6, nJts+6) = Jacobian_EE[2];							// left foot
	// J_SoT.block(27, 0, 6, nJts+6) = Jacobian_EE[3];							// right foot
	//
	std::cout<< " JACOBIAN AND POSES GOT IN : " <<  yarp::os::Time::now()-t_wbIK << " s" << std::endl;	
	
	return true;
}

bool wb_InverseKinematics::load_wbIK_PoseTask()
{ 
	// Update the desired task as function of the updated robot state
	// ==============================================================
	alpha_g = 0.25;	//0.1
	var_gain = (1.-alpha_g) * var_gain + alpha_g * 4.0* gain_vector_0;  // 0.3
	gain_vector = gain_vector_0 + var_gain;

	std::cout<< " ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  : \t" <<  gain_vector(0) << std::endl;	
	std::cout<< " GAIN VECTOR  : \t" <<  gain_vector(0) << std::endl;	

	for (int i=0; i<8; i++)	
	{
		// pose error
		error_pose[i] 	  = Transforms.computePoseErrorNormalizedAxisAngle(Pose_EE[i], des_X_EE[i]);
		error_pos_norm[i] = error_pose[i].head(3).norm();
		error_ori_norm[i] = error_pose[i].tail(3).norm();
		des_X_dot_EE[i]	  = gain_vector.asDiagonal() *error_pose[i];							// scaled error by convergence rate (gain)
	}
	//
	// des_X_dot_EE[4]	  = coef_grad*gain_vector.asDiagonal() *error_pose[4];
	//
	if(isFreeze)
	{
		for(int i=0;i<nJts+6;i++){
			if(freeze[i]==1){
				// J_SoT.block(0,i,n_task,1) *= 0;
				for(int j=0;j<8;j++){
					Jacobian_EE[j].col(i)= VectorXd::Zero(6);  //Jacobian_EE[j].block(0,i,6,1) *= 0;
				}
			}
		}
	}
	// Joint space Task (postural)
	des_q_dot.tail(nJts) = -alpha_q *(virt_jts_pos -q_0) * 0.1; 			// 0.3
	des_q_dot = des_q_dot.cwiseProduct(VectorXd::Ones(nJts+6)-freeze);

	//
	// Task Space Task (end-effectors velocity)
	// X_dot_SoT.segment(0,  6) = des_X_dot_EE[0]; 							// left hand
	// X_dot_SoT.segment(6,  6) = des_X_dot_EE[1]; 							// right hand
	// X_dot_SoT.segment(12, 3) = des_X_dot_EE[4].head(3); 					// CoM Position
	// X_dot_SoT.segment(15, 3) = des_X_dot_EE[5].tail(3); 					// Angular momentum
	// X_dot_SoT.segment(18, 3) = des_X_dot_EE[7].tail(3); 					// Chest
	// X_dot_SoT.segment(21, 6) = des_X_dot_EE[2];							// left foot
	// X_dot_SoT.segment(27, 6) = des_X_dot_EE[3];							// right foot	
	//
	// X_dot_SoT = 0.80*X_dot_SoT + 0.20*X_dot_SoT_0; 						// 
	//
	J_T_Wx_J.setZero();
	J_T_Wx_Xdot.setZero();
	// des_X_dot_EE[0] *= 0.0;
	// des_X_dot_EE[1] *= 0.0;

	for(int i=0; i<8; i++)
	{
		if(i<=3) // hands and feet
		{
			J_T_Wx_J    +=  Jacobian_EE[i].transpose() * task_weight[i].asDiagonal() * Jacobian_EE[i];
			J_T_Wx_Xdot -=  Jacobian_EE[i].transpose() * task_weight[i].asDiagonal() * des_X_dot_EE[i];
		}
		if(i==4)  // CoM position
		{
			J_T_Wx_J    +=  Jacobian_EE[i].topRows(3).transpose() * task_weight[i].head(3).asDiagonal() * Jacobian_EE[i].topRows(3);
			J_T_Wx_Xdot -=  Jacobian_EE[i].topRows(3).transpose() * task_weight[i].head(3).asDiagonal() * des_X_dot_EE[i].head(3);
		}
		if(i==5 || i==7)  // pelvis orientation or chest orientation
		{
			J_T_Wx_J    +=  Jacobian_EE[i].bottomRows(3).transpose() * task_weight[i].tail(3).asDiagonal() * Jacobian_EE[i].bottomRows(3);
			J_T_Wx_Xdot -=  Jacobian_EE[i].bottomRows(3).transpose() * task_weight[i].tail(3).asDiagonal() * des_X_dot_EE[i].tail(3);
		}
		if(i==6)  // postural task
		{
			J_T_Wx_J    +=  0.05*Jacobian_posture.transpose() * posture_weight.asDiagonal() * Jacobian_posture;			// 0.01
			J_T_Wx_Xdot -=  0.05*Jacobian_posture.transpose() * posture_weight.asDiagonal() * des_q_dot.tail(nJts);		// 0.01
		}
	}
	// regularization
			J_T_Wx_J += Weight_velocity.asDiagonal() * (1./m_massMatrix(0,0)) * m_massMatrix; //MatrixXd::Identity(nJts+6, nJts+6);

	// J_T_Wx_J *=0.5;

	return true;
}

//
bool wb_InverseKinematics::load_wbIK_PoseTask_as(string stanceFoot)
{ 
	// Update the desired task as function of the updated robot state
	// ==============================================================
	alpha_g = 0.25;	//0.1
	var_gain = (1.-alpha_g) * var_gain + alpha_g * 4.0* gain_vector_0;  // 0.3
	gain_vector = gain_vector_0 + var_gain;

	for (int i=0; i<8; i++)	
	{
		// pose error
		error_pose[i] 	  = Transforms.computePoseErrorNormalizedAxisAngle(Pose_EE[i], des_X_EE[i]);
		error_pos_norm[i] = error_pose[i].head(3).norm();
		error_ori_norm[i] = error_pose[i].tail(3).norm();
		des_X_dot_EE[i]	  = gain_vector.asDiagonal() *error_pose[i];							// scaled error by convergence rate (gain)
	}
	//
	if(stanceFoot == "left")
	{
		des_X_dot_EE[3]	  = 0.1*this->coef_grad*gain_vector.asDiagonal() *error_pose[3];
	}
	else
	{
		des_X_dot_EE[2]	  = 0.1*this->coef_grad*gain_vector.asDiagonal() *error_pose[2];
	}
	//
	if(isFreeze)
	{
		for(int i=0;i<nJts+6;i++){
			if(freeze[i]==1){
				// J_SoT.block(0,i,n_task,1) *= 0;
				for(int j=0;j<8;j++){
					Jacobian_EE[j].col(i)= VectorXd::Zero(6);  //Jacobian_EE[j].block(0,i,6,1) *= 0;
				}
			}
		}
	}
	// Joint space Task (postural)
	des_q_dot.tail(nJts) = -alpha_q *(virt_jts_pos -q_0) * 0.1; 			// 0.3
	des_q_dot = des_q_dot.cwiseProduct(VectorXd::Ones(nJts+6)-freeze);
	//
	J_T_Wx_J.setZero();
	J_T_Wx_Xdot.setZero();
	// des_X_dot_EE[0] *= 0.0;
	// des_X_dot_EE[1] *= 0.0;

	for(int i=0; i<8; i++)
	{
		if(i<=3) // hands and feet
		{
			J_T_Wx_J    +=  Jacobian_EE[i].transpose() * task_weight[i].asDiagonal() * Jacobian_EE[i];
			J_T_Wx_Xdot -=  Jacobian_EE[i].transpose() * task_weight[i].asDiagonal() * des_X_dot_EE[i];
		}
		if(i==4)  // CoM position
		{
			J_T_Wx_J    +=  Jacobian_EE[i].topRows(3).transpose() * task_weight[i].head(3).asDiagonal() * Jacobian_EE[i].topRows(3);
			J_T_Wx_Xdot -=  Jacobian_EE[i].topRows(3).transpose() * task_weight[i].head(3).asDiagonal() * des_X_dot_EE[i].head(3);
		}
		if(i==5 || i==7)  // pelvis orientation or chest orientation
		{
			J_T_Wx_J    +=  Jacobian_EE[i].bottomRows(3).transpose() * task_weight[i].tail(3).asDiagonal() * Jacobian_EE[i].bottomRows(3);
			J_T_Wx_Xdot -=  Jacobian_EE[i].bottomRows(3).transpose() * task_weight[i].tail(3).asDiagonal() * des_X_dot_EE[i].tail(3);
		}
		if(i==6)  // postural task
		{
			J_T_Wx_J    +=  0.05*Jacobian_posture.transpose() * posture_weight.asDiagonal() * Jacobian_posture;
			J_T_Wx_Xdot -=  0.05*Jacobian_posture.transpose() * posture_weight.asDiagonal() * des_q_dot.tail(nJts);
		}
	}
	// regularization
			J_T_Wx_J += Weight_velocity.asDiagonal() * (1./m_massMatrix(0,0)) * m_massMatrix; //MatrixXd::Identity(nJts+6, nJts+6);

	// J_T_Wx_J *=0.5;

	return true;
}

// bool wb_InverseKinematics::load_wbIK_VeloTask(Vector6d des_Velo_EE_[])
// { 
// 	// Update the desired task
// 	// =======================
// 	// Task Space Task (end-effectors velocity)
// 	X_dot_SoT.segment(0,  6) = des_Velo_EE_[0]; 							// left hand
// 	X_dot_SoT.segment(6,  6) = des_Velo_EE_[1]; 							// right hand
// 	X_dot_SoT.segment(12, 3) = des_Velo_EE_[4].head(3); 					// CoM Position
// 	X_dot_SoT.segment(15, 3) = des_Velo_EE_[5].tail(3); 					// Angular momentum
// 	X_dot_SoT.segment(18, 3) = des_Velo_EE_[7].tail(3); 					// Chest
// 	X_dot_SoT.segment(21, 6) = des_Velo_EE_[2];								// left foot
// 	X_dot_SoT.segment(27, 6) = des_Velo_EE_[3];								// right foot
	
// 	//
// 	for(int i=0;i<nJts+6;i++)
// 			if(freeze[i]==1)
// 				J_SoT.block(0,i,n_task,1) *= 0;
// 	// Joint space Task (postural)
// 	des_q_dot.tail(nJts) = -alpha_q *(virt_jts_pos -q_0) * 0.2; // 0.3
// 	des_q_dot = des_q_dot.cwiseProduct(VectorXd::Ones(nJts+6)-freeze);

// 	return true;
// }

bool wb_InverseKinematics::load_wbIK_VeloTask(Vector6d des_Velo_EE_[])
{ 
	// Update the desired task
	// =======================
	for(int i=0;i<8;i++)
	{
		des_X_dot_EE[i] = des_Velo_EE_[i];
	}
	des_X_dot_EE[4].head(2) = coef_grad * des_Velo_EE_[4].head(2);
	
	//
	if(isFreeze)
	{
		for(int i=0;i<nJts+6;i++){
			if(freeze[i]==1){
				// J_SoT.block(0,i,n_task,1) *= 0;
				for(int j=0;j<8;j++){
					Jacobian_EE[j].col(i)= VectorXd::Zero(6);  //Jacobian_EE[j].block(0,i,6,1) *= 0;
				}
			}
		}
	}
	// Joint space Task (postural)
	des_q_dot.tail(nJts) = -alpha_q *(virt_jts_pos -q_0) * 0.1; 			// 0.3
	des_q_dot = des_q_dot.cwiseProduct(VectorXd::Ones(nJts+6)-freeze);
	//
	J_T_Wx_J.setZero();
	J_T_Wx_Xdot.setZero();
	// des_X_dot_EE[0] *= 0.0;
	// des_X_dot_EE[1] *= 0.0;

	for(int i=0; i<8; i++)
	{
		if(i<=3) // hands and feet
		{
			J_T_Wx_J    +=  Jacobian_EE[i].transpose() * task_weight[i].asDiagonal() * Jacobian_EE[i];
			J_T_Wx_Xdot -=  Jacobian_EE[i].transpose() * task_weight[i].asDiagonal() * des_X_dot_EE[i];
		}
		if(i==4)  // CoM position
		{
			J_T_Wx_J    +=  Jacobian_EE[i].topRows(3).transpose() * task_weight[i].head(3).asDiagonal() * Jacobian_EE[i].topRows(3);
			J_T_Wx_Xdot -=  Jacobian_EE[i].topRows(3).transpose() * task_weight[i].head(3).asDiagonal() * des_X_dot_EE[i].head(3);
		}
		if(i==5 || i==7)  // pelvis orientation or chest orientation
		{
			J_T_Wx_J    +=  Jacobian_EE[i].bottomRows(3).transpose() * task_weight[i].tail(3).asDiagonal() * Jacobian_EE[i].bottomRows(3);
			J_T_Wx_Xdot -=  Jacobian_EE[i].bottomRows(3).transpose() * task_weight[i].tail(3).asDiagonal() * des_X_dot_EE[i].tail(3);
		}
		if(i==6)  // postural task
		{
			J_T_Wx_J    +=  0.05*Jacobian_posture.transpose() * posture_weight.asDiagonal() * Jacobian_posture;
			J_T_Wx_Xdot -=  0.05*Jacobian_posture.transpose() * posture_weight.asDiagonal() * des_q_dot.tail(nJts);
		}
	}
	// regularization
			J_T_Wx_J += Weight_velocity.asDiagonal() * (1./m_massMatrix(0,0)) * m_massMatrix; //MatrixXd::Identity(nJts+6, nJts+6);

	// J_T_Wx_J *=0.5;

	return true;
}

bool wb_InverseKinematics::load_wbIK_VeloTask2(Vector6d des_Velo_EE_[], VectorXd des_jts_Velo)
{
	//
	// Update the desired task
	// =======================
	for(int i=0;i<8;i++)
	{
		des_X_dot_EE[i] = des_Velo_EE_[i];
	}
	des_X_dot_EE[4].head(2) = coef_grad * des_Velo_EE_[4].head(2);
	
	//
	if(isFreeze)
	{
		for(int i=0;i<nJts+6;i++){
			if(freeze[i]==1){
				// J_SoT.block(0,i,n_task,1) *= 0;
				for(int j=0;j<8;j++){
					Jacobian_EE[j].col(i)= VectorXd::Zero(6);  //Jacobian_EE[j].block(0,i,6,1) *= 0;
				}
			}
		}
	}
	// Joint space Task (postural)
	// des_q_dot.tail(nJts) = -alpha_q *(virt_jts_pos -q_0) * 0.1; 			// 0.3
	des_q_dot = des_jts_Velo.cwiseProduct(VectorXd::Ones(nJts+6)-freeze);
	//
	J_T_Wx_J.setZero();
	J_T_Wx_Xdot.setZero();
	// des_X_dot_EE[0] *= 0.0;
	// des_X_dot_EE[1] *= 0.0;

	for(int i=0; i<8; i++)
	{
		if(i<=3) // hands and feet
		{
			J_T_Wx_J    +=  Jacobian_EE[i].transpose() * task_weight[i].asDiagonal() * Jacobian_EE[i];
			J_T_Wx_Xdot -=  Jacobian_EE[i].transpose() * task_weight[i].asDiagonal() * des_X_dot_EE[i];
		}
		if(i==4)  // CoM position
		{
			J_T_Wx_J    +=  Jacobian_EE[i].topRows(3).transpose() * task_weight[i].head(3).asDiagonal() * Jacobian_EE[i].topRows(3);
			J_T_Wx_Xdot -=  Jacobian_EE[i].topRows(3).transpose() * task_weight[i].head(3).asDiagonal() * des_X_dot_EE[i].head(3);
		}
		if(i==5 || i==7)  // pelvis orientation or chest orientation
		{
			J_T_Wx_J    +=  Jacobian_EE[i].bottomRows(3).transpose() * task_weight[i].tail(3).asDiagonal() * Jacobian_EE[i].bottomRows(3);
			J_T_Wx_Xdot -=  Jacobian_EE[i].bottomRows(3).transpose() * task_weight[i].tail(3).asDiagonal() * des_X_dot_EE[i].tail(3);
		}
		if(i==6)  // postural task
		{
			J_T_Wx_J    +=  0.05*Jacobian_posture.transpose() * posture_weight.asDiagonal() * Jacobian_posture;			// 0.05
			J_T_Wx_Xdot -=  0.05*Jacobian_posture.transpose() * posture_weight.asDiagonal() * des_q_dot.tail(nJts);		// 0.05
		}
	}
	// y position pelvis (symmetry during retraction)
			J_T_Wx_J    +=  Jacobian_EE[5].row(1).transpose() * task_weight[5](1) * Jacobian_EE[5].row(1);
			J_T_Wx_Xdot -=  Jacobian_EE[5].row(1).transpose() * task_weight[5](1) * des_X_dot_EE[5](1);

	// regularization
			// J_T_Wx_J += Weight_velocity.asDiagonal() * (100./m_massMatrix(0,0)) * m_massMatrix; //MatrixXd::Identity(nJts+6, nJts+6);
			J_T_Wx_J += Weight_velocity.asDiagonal() * (1./m_massMatrix(0,0)) * m_massMatrix; //MatrixXd::Identity(nJts+6, nJts+6);

	// J_T_Wx_J *=0.5;

	return true;
}


// bool wb_InverseKinematics::solve_QPcvxgen(WbRobotModel& robot_model_, Vector7d pose_lfoot, Vector7d pose_rfoot, string stanceFoot)
bool wb_InverseKinematics::solve_QPcvxgen(WbRobotModel& robot_model_, Vector7d pose_lfoot, Vector7d pose_rfoot, string stanceFoot, MatrixXd &W_EdgesNormalCHull, VectorXd &DistanceEdgesCHull)
{
	// initialization of the cvxgen solver for the wb inverse kinematics
    double t_wbIK = yarp::os::Time::now();
    //
	this->update_model(robot_model_, q_dot_star, pose_lfoot, pose_rfoot, stanceFoot);
	std::cout<< " UPDATE MODEL IN : " <<  yarp::os::Time::now()-t_wbIK << " s" << std::endl;
	this->load_wbIK_PoseTask();
	// =====================================================================================
	std::cout<< " UPDATE MODEL + PARAM LOADING IN : " <<  yarp::os::Time::now()-t_wbIK << " s" << std::endl;
	// computation of the convex polygon   
	this->getConvexHullVariables(this->PtsInFoot, Pose_EE[2], Pose_EE[3],  W_EdgesNormalCHull, DistanceEdgesCHull, W_Rot_AbsFoot, W_Pos_AbsFoot);

	MatrixXd NeJc(6, nJts+6); NeJc.setZero();
	Vector6d deXc;  deXc.setZero();
	if(W_EdgesNormalCHull.rows() <= 5){
		int r = W_EdgesNormalCHull.rows();
		NeJc.topRows(r) = W_EdgesNormalCHull.topRows(r)* Jacobian_EE[4].topRows(2);
		deXc.head(r)    = DistanceEdgesCHull.head(r) + W_EdgesNormalCHull.topRows(r)*(W_Pos_AbsFoot - Pose_EE[4].head(2));
	}
	else{
		NeJc = W_EdgesNormalCHull.topRows(6)* Jacobian_EE[4].topRows(2);
		deXc = DistanceEdgesCHull.head(6) + W_EdgesNormalCHull.topRows(6)*(W_Pos_AbsFoot - Pose_EE[4].head(2));
	}
	// Hessian matrix
	memcpy(wbIK2_params.Q, J_T_Wx_J.data(), sizeof(double) * J_T_Wx_J.size());
	memcpy(wbIK2_params.P, &J_T_Wx_Xdot[0], sizeof(double)*(nJts+6)); 				// weight acceleration and desired posture
	for(int i=0;i<6;i++) 
		memcpy(wbIK2_params.NeJc[i+1], &NeJc(i,0),	sizeof(double)*(nJts+6));		// convexHull constraints Matrix for CoM
	memcpy(wbIK2_params.deXc, &deXc[0], 			sizeof(double)*(6));			// convexHull constraints vector for CoM
	// setting of hard constraints
	// Joint limits min and max
	for(int i=0; i<nJts; i++) 	wbIK2_params.qmin[i] = (minJointLimits(i) - virt_jts_pos(i));
	for(int i=0; i<nJts; i++) 	wbIK2_params.qmax[i] = (maxJointLimits(i) - virt_jts_pos(i));
	// sampling time
	wbIK2_params.dt[0] = 1.0*virtual_sampTime;
	// Solve the QP (acceleration)
	wbIK2_settings.verbose = 0;
	wbIK2_settings.eps = 15e-5;
	wbIK2_settings.resid_tol = 1e-4;
	// wbIK2_settings.kkt_reg = 1e-7;
	// wbIK2_settings.refine_steps = 1;

	int iter = wbIK2_solve();
	// Extract the QP solution (acceleration and slack)
	for(int i=0; i<nJts+6; i++) q_dot_star(i) = wbIK2_vars.qdot[i];
	//
	std::cout<< "ONE WBIK ITERATION RUN IN : " <<  yarp::os::Time::now()-t_wbIK << " s" << std::endl;
	std::cout<< "CHECKING COM CONSTRAINTS : \n" <<  virtual_sampTime*NeJc*q_dot_star - deXc << std::endl;

	bool isSSt = true;
	for(int i=0; i<6; i++){
		isSSt = isSSt && (virtual_sampTime*NeJc.row(i)*q_dot_star + 0.005 < deXc(i));
	}
	//
	// if(!isSSt)
	// {
	// 	q_dot_star   = q_dot_star_0;
	// 	virt_jts_pos = virt_jts_pos_0;
	// 	coef_grad *= 0.5;
	// 	std::cout<< "RESET CONSTRAINTS : \n" <<  yarp::os::Time::now()-t_wbIK << " s" << std::endl;  //Mssp
	// }



	return true;
}
//
bool wb_InverseKinematics::solve_QPcvxgen3(WbRobotModel& robot_model_, Vector7d pose_lfoot, Vector7d pose_rfoot, string stanceFoot, MatrixXd &W_EdgesNormalCHull, VectorXd &DistanceEdgesCHull)
{
	// initialization of the cvxgen solver for the wb inverse kinematics
    double t_wbIK = yarp::os::Time::now();
    //
	this->update_model(robot_model_, q_dot_star, pose_lfoot, pose_rfoot, stanceFoot);
	std::cout<< " UPDATE MODEL IN : " <<  yarp::os::Time::now()-t_wbIK << " s" << std::endl;
	this->load_wbIK_PoseTask();
	// =====================================================================================
	std::cout<< " UPDATE MODEL + PARAM LOADING IN : " <<  yarp::os::Time::now()-t_wbIK << " s" << std::endl;
	// computation of the convex polygon   
	this->getConvexHullVariables(this->PtsInFoot, Pose_EE[2], Pose_EE[3],  W_EdgesNormalCHull, DistanceEdgesCHull, W_Rot_AbsFoot, W_Pos_AbsFoot);

	MatrixXd NeJc = MatrixXd::Zero(6, nJts+6);
	Vector6d deXc = VectorXd::Zero(6);
	if(W_EdgesNormalCHull.rows() <= 5){
		int r = W_EdgesNormalCHull.rows();
		NeJc.topRows(r) = W_EdgesNormalCHull.topRows(r)* Jacobian_EE[4].topRows(2);
		deXc.head(r)    = DistanceEdgesCHull.head(r) + W_EdgesNormalCHull.topRows(r)*(W_Pos_AbsFoot - Pose_EE[4].head(2));
	}
	else{
		NeJc = W_EdgesNormalCHull.topRows(6)* Jacobian_EE[4].topRows(2);
		deXc = DistanceEdgesCHull.head(6) + W_EdgesNormalCHull.topRows(6)*(W_Pos_AbsFoot - Pose_EE[4].head(2));
	}
	//
	// constraints for the support polygon
	//=====================================
	double Ly_min = 0.1;
	MatrixXd Mssp;
	VectorXd dssp;

	this->getFeetSelfCollisionAvoidanceConstraints(stanceFoot, Ly_min, Jacobian_EE[2], Jacobian_EE[3], Pose_EE[2], Pose_EE[3], Mssp, dssp); 
	MatrixXd sMDw = MatrixXd::Zero(5, nJts+6);
	VectorXd sdw = VectorXd::Zero(5);
	sMDw.row(0) = Mssp.row(1);
	sdw(0)     = -Ly_min + dssp(1);


	// Hessian matrix
	memcpy(wbIK3_params.Q, J_T_Wx_J.data(), sizeof(double) * J_T_Wx_J.size());
	memcpy(wbIK3_params.P, &J_T_Wx_Xdot[0], sizeof(double)*(nJts+6)); 				// weight acceleration and desired posture
	for(int i=0;i<6;i++) 
		memcpy(wbIK3_params.NeJc[i+1], &NeJc(i,0),	sizeof(double)*(nJts+6));		// convexHull constraints Matrix for CoM
	memcpy(wbIK3_params.deXc, &deXc[0], 			sizeof(double)*(6));			// convexHull constraints vector for CoM

	for(int i=0;i<5;i++) 
		memcpy(wbIK3_params.sMDw[i+1], &sMDw(i,0),	sizeof(double)*(nJts+6));		// convexHull constraints Matrix for CoM
	memcpy(wbIK3_params.sdw, &sdw[0], 			sizeof(double)*(5));				// convexHull constraints vector for CoM
	// setting of hard constraints
	// Joint limits min and max
	for(int i=0; i<nJts; i++) 	wbIK3_params.qmin[i] = (minJointLimits(i) - virt_jts_pos(i));
	for(int i=0; i<nJts; i++) 	wbIK3_params.qmax[i] = (maxJointLimits(i) - virt_jts_pos(i));
	// sampling time
	wbIK3_params.dt[0] = 1.0*virtual_sampTime;
	// Solve the QP (acceleration)
	wbIK3_settings.verbose = 0;
	wbIK3_settings.eps = 15e-5;
	wbIK3_settings.resid_tol = 1e-4;
	// wbIK2_settings.kkt_reg = 1e-7;
	// wbIK2_settings.refine_steps = 1;
	int iter = wbIK3_solve();
	// Extract the QP solution (acceleration and slack)
	for(int i=0; i<nJts+6; i++) q_dot_star(i) = wbIK3_vars.qdot[i];
	//
	std::cout<< "ONE WBIK ITERATION RUN IN : " <<  yarp::os::Time::now()-t_wbIK << " s" << std::endl;
	// std::cout<< "CHECKING COM CONSTRAINTS : \n" <<  virtual_sampTime*NeJc*q_dot_star - deXc << std::endl;  //Mssp
	std::cout<< "CHECKING SELF_COLLISION CONSTRAINTS : \n" <<  virtual_sampTime*sMDw*q_dot_star -sdw << std::endl;  //

	return true;
}

//
bool wb_InverseKinematics::solve_QPcvxgen4(WbRobotModel& robot_model_, Vector7d pose_lfoot, Vector7d pose_rfoot, string stanceFoot, 
										   MatrixXd &W_EdgesNormalCHull, VectorXd &DistanceEdgesCHull, Vector2d Disturb_c, double fu_z_mgbetaW)
{
	// initialization of the cvxgen solver for the wb inverse kinematics
    double t_wbIK = yarp::os::Time::now();
    //
	this->update_model(robot_model_, coef_grad*q_dot_star, pose_lfoot, pose_rfoot, stanceFoot);
	std::cout<< " UPDATE MODEL IN : " <<  yarp::os::Time::now()-t_wbIK << " s" << std::endl;
	this->load_wbIK_PoseTask_as(stanceFoot);
	// =====================================================================================
	std::cout<< " UPDATE MODEL + PARAM LOADING IN : " <<  yarp::os::Time::now()-t_wbIK << " s" << std::endl;
	// computation of the convex polygon   
	this->getConvexHullVariables(this->PtsInFoot, Pose_EE[2], Pose_EE[3],  W_EdgesNormalCHull, DistanceEdgesCHull, W_Rot_AbsFoot, W_Pos_AbsFoot);

	MatrixXd NeJc = MatrixXd::Zero(6, nJts+6);
	Vector6d deXc = VectorXd::Zero(6);
	Vector6d deXc2 = VectorXd::Zero(6);
	if(W_EdgesNormalCHull.rows() <= 5){
		int r = W_EdgesNormalCHull.rows();
		NeJc.topRows(r) = W_EdgesNormalCHull.topRows(r)* Jacobian_EE[4].topRows(2);
		deXc.head(r)    = DistanceEdgesCHull.head(r) + W_EdgesNormalCHull.topRows(r)*(W_Pos_AbsFoot - Pose_EE[4].head(2));
		deXc2.head(r)   = DistanceEdgesCHull.head(r) + W_EdgesNormalCHull.topRows(r)*(W_Pos_AbsFoot - (1+fu_z_mgbetaW)*Pose_EE[4].head(2) - Disturb_c);
	}
	else{
		NeJc  = W_EdgesNormalCHull.topRows(6)* Jacobian_EE[4].topRows(2);
		deXc  = DistanceEdgesCHull.head(6) + W_EdgesNormalCHull.topRows(6)*(W_Pos_AbsFoot - Pose_EE[4].head(2));
		deXc2 = DistanceEdgesCHull.head(6) + W_EdgesNormalCHull.topRows(6)*(W_Pos_AbsFoot - (1+fu_z_mgbetaW)*Pose_EE[4].head(2) - Disturb_c);
	}
	//
	// constraints for the support polygon
	//=====================================
	double Ly_min = 0.10;
	MatrixXd Mssp;
	VectorXd dssp;

	this->getFeetSelfCollisionAvoidanceConstraints(stanceFoot, Ly_min, Jacobian_EE[2], Jacobian_EE[3], Pose_EE[2], Pose_EE[3], Mssp, dssp);
	MatrixXd sMDw = MatrixXd::Zero(5, nJts+6);
	VectorXd sdw = VectorXd::Zero(5);
	sMDw.row(0) = 1.0*Mssp.row(1);
	sdw(0)      = 1.0*(-Ly_min + dssp(1));


	// Hessian matrix
	memcpy(wbIK4_params.Q, J_T_Wx_J.data(), sizeof(double) * J_T_Wx_J.size());
	memcpy(wbIK4_params.P, &J_T_Wx_Xdot[0], sizeof(double)*(nJts+6)); 				// weight acceleration and desired posture
	for(int i=0;i<6;i++) 
		memcpy(wbIK4_params.NeJc[i+1], &NeJc(i,0),	sizeof(double)*(nJts+6));		// convexHull constraints Matrix for CoM
	memcpy(wbIK4_params.deXc, &deXc[0], 			sizeof(double)*(6));			// convexHull constraints vector for CoM
	memcpy(wbIK4_params.deXc2, &deXc2[0], 			sizeof(double)*(6));			// convexHull constraints vector for 0-step capturability

	for(int i=0;i<5;i++) 
		memcpy(wbIK4_params.sMDw[i+1], &sMDw(i,0),	sizeof(double)*(nJts+6));		// convexHull constraints Matrix for CoM
	memcpy(wbIK4_params.sdw, &sdw[0], 			sizeof(double)*(5));				// convexHull constraints vector for CoM
	// setting of hard constraints
	// Joint limits min and max
	for(int i=0; i<nJts; i++) 	wbIK4_params.qmin[i] = (minJointLimits(i) - virt_jts_pos(i));
	for(int i=0; i<nJts; i++) 	wbIK4_params.qmax[i] = (maxJointLimits(i) - virt_jts_pos(i));
	// sampling time
	wbIK4_params.dt[0] = 1.0*virtual_sampTime;
	// Solve the QP (acceleration)
	wbIK4_settings.verbose = 0;
	wbIK4_settings.eps = 15e-5;
	wbIK4_settings.resid_tol = 1e-4;
	// wbIK2_settings.kkt_reg = 1e-7;
	// wbIK2_settings.refine_steps = 1;
	int iter = wbIK4_solve();
	// Extract the QP solution (acceleration and slack)
	for(int i=0; i<nJts+6; i++) q_dot_star(i) = wbIK4_vars.qdot[i];
	//
	std::cout<< "ONE WBIK ITERATION RUN IN : " <<  yarp::os::Time::now()-t_wbIK << " s" << std::endl;
	// std::cout<< "CHECKING COM CONSTRAINTS : \n" <<  virtual_sampTime*NeJc*q_dot_star - deXc << std::endl;  //Mssp
	std::cout<< "CHECKING SELF_COLLISION CONSTRAINTS : \n" <<  virtual_sampTime*sMDw*q_dot_star - sdw << std::endl;  //
	std::cout<< "CHECKING SELF_COLLISION CONSTRAINTS SDW: \t" <<  dssp(1) << std::endl;  //
	std::cout<< "CHECKING 0-STEP CAP CONSTRAINTS : \n" <<  virtual_sampTime*NeJc*q_dot_star - deXc2 << std::endl;  //Mssp
	std::cout<< "CHECKING COM CONSTRAINTS : \n" <<  virtual_sampTime*NeJc*q_dot_star - deXc << std::endl;  //Mssp

	bool isCap = true;
	bool isSSt = true;
	for(int i=0; i<6; i++){
		isCap = isCap && (virtual_sampTime*NeJc.row(i)*q_dot_star + 0.005 < deXc2(i));
		isSSt = isSSt && (virtual_sampTime*NeJc.row(i)*q_dot_star + 0.005 < deXc(i));
	}

	bool selfcolft = virtual_sampTime*sMDw.row(0)*q_dot_star + 0.005 < (sdw(0));
	//
	if(!selfcolft || !isCap || !isSSt)
	{
		q_dot_star   = q_dot_star_0;
		virt_jts_pos = virt_jts_pos_0;
		coef_grad *= 0.5;

		std::cout<< "RESET CONSTRAINTS : \n" <<  yarp::os::Time::now()-t_wbIK << " s" << std::endl;  //Mssp
	}
	// else
	// {
	// 	coef_grad = 1.0;
	// }

	return true;
}

//
//
bool wb_InverseKinematics::CvxgenSolveQP_Velo(WbRobotModel& robot_model_, Vector7d pose_lfoot, Vector7d pose_rfoot, string stanceFoot)
{
	// initialization of the cvxgen solver for the wb inverse kinematics
    double t_wbIK = yarp::os::Time::now();

    // update the model : Done directly from the measurements
    // ================

    // load the task 
    // ==============

	// ===========================================================================================
	// computation of the convex polygon
	// ===========================================================================================
    MatrixXd W_EdgesNormalCHull; 
    VectorXd DistanceEdgesCHull;
    // Matrix2d W_Rot_AbsFoot;
    // Vector2d W_Pos_AbsFoot;   

	this->getConvexHullVariables(this->PtsInFoot, Pose_EE[2], Pose_EE[3],  W_EdgesNormalCHull, DistanceEdgesCHull, W_Rot_AbsFoot, W_Pos_AbsFoot);

	MatrixXd NeJc = MatrixXd::Zero(6, nJts+6);
	Vector6d deXc = VectorXd::Zero(6);
	if(W_EdgesNormalCHull.rows() <= 5){
		int r = W_EdgesNormalCHull.rows();
		NeJc.topRows(r) = W_EdgesNormalCHull.topRows(r)* Jacobian_EE[4].topRows(2);
		deXc.head(r)    = DistanceEdgesCHull.head(r) + W_EdgesNormalCHull.topRows(r)*(W_Pos_AbsFoot - Pose_EE[4].head(2));
	}
	else{
		NeJc = W_EdgesNormalCHull.topRows(6)* Jacobian_EE[4].topRows(2);
		deXc = DistanceEdgesCHull.head(6) + W_EdgesNormalCHull.topRows(6)*(W_Pos_AbsFoot - Pose_EE[4].head(2));
	}



	// ===========================================================================================
	// Loading data for the cvxgen solver
	// ===========================================================================================
	double Omega_inv_dt = sqrt(Pose_EE[4](2)/9.81)/virtual_sampTime;
	NeJc = NeJc + Omega_inv_dt * NeJc;
	// Hessian matrix
	memcpy(wbIK1_params.Q, J_T_Wx_J.data(), sizeof(double) * J_T_Wx_J.size());
	memcpy(wbIK1_params.P, &J_T_Wx_Xdot[0], sizeof(double)*(nJts+6)); 				// weight acceleration and desired posture
	for(int i=0;i<6;i++) 
		memcpy(wbIK1_params.NeJc[i+1], &NeJc(i,0),	sizeof(double)*(nJts+6));		// convexHull constraints Matrix for CoM
	memcpy(wbIK1_params.deXc, &deXc[0], 			sizeof(double)*(6));			// convexHull constraints vector for CoM
	// setting of hard constraints
	// Joint limits min and max
	for(int i=0; i<nJts; i++) 	wbIK1_params.qmin[i]  = (minJointLimits(i) - virt_jts_pos(i));	//	qmin
	for(int i=0; i<nJts; i++) 	wbIK1_params.qmax[i]  = (maxJointLimits(i) - virt_jts_pos(i));	// 	qmax
	for(int i=0; i<nJts; i++) 	wbIK1_params.vmax[i]  =  maxVeloLimit(i);						// 	vmax
	// sampling time
	wbIK1_params.dt[0] = virtual_sampTime;
	// solver parameters
	wbIK1_settings.verbose 		= 0;
	wbIK1_settings.eps 			= 15e-5; 
	wbIK1_settings.resid_tol	= 1e-4;

	int iter = wbIK1_solve();
	// Extract the QP solution (acceleration and slack)
	for(int i=0; i<nJts+6; i++) q_dot_star(i) = wbIK1_vars.qdot[i];
	//
	std::cout<< "WBIK VELOCITY ITERATION RUN IN : " <<  yarp::os::Time::now()-t_wbIK << " s" << std::endl;
	// std::cout<< "CHECK STABILITY CONSTRAINT : \n" << NeJc * q_dot_star * virtual_sampTime - deXc << std::endl;

	bool isSSt = true;
	int nr = W_EdgesNormalCHull.rows();
	for(int i=0; i< min(nr, 6); i++){
		isSSt = isSSt && (virtual_sampTime*NeJc.row(i)*q_dot_star + 0.005 < deXc(i));
	}
	//
	if(!isSSt)
	{
		// q_dot_star   = q_dot_star_0;
		// virt_jts_pos = virt_jts_pos_0;
		// coef_grad *= -0.2;
		// std::cout<< "RESET CONSTRAINTS : \n" <<  yarp::os::Time::now()-t_wbIK << " s" << std::endl;  //Mssp
	}
	else
	{
		coef_grad *= 1.0;
	}

	return true;
}

// bool wb_InverseKinematics::get_wb_IK(WbRobotModel& robot_model_, VectorXd jts_position, Matrix4d WHB, Vector7d des_X_EE_[])
bool wb_InverseKinematics::get_wb_IK(WbRobotModel& robot_model_, VectorXd jts_position, Matrix4d WHB, Vector7d des_X_EE_[], string stanceFoot, Vector7d pose_lfoot, Vector7d pose_rfoot)
{
	// virt_jts_pos = jts_position;
	// virt_jts_pos(nJts-9) -=0.02;  // add angle to set the knee out joints limits and singularity
	// virt_jts_pos(nJts-3) -=0.02;  // add angle to set the knee out joints limits and singularity
	virt_jts_pos = q_0;
	q_dot_star.setZero();
	// virt_WHB	 = WHB;
	for(int i=0; i<8; i++) des_X_EE[i] = des_X_EE_[i];
	//
	int count = 0;
	bool cnt, e_lh, e_rh, e_lf, e_rf, ik_run;
		 cnt = e_lh = e_rh = e_lf = e_rf = ik_run = false;
	// loop over the specified number of iteration if the error is still large
	double t_wbIK = yarp::os::Time::now();

	memcpy(task_weight, &task_weight_reach[0], 8 * sizeof *task_weight_reach);

	// data logging  // temporary
	string path_log_joints_pos  = "log_virt_joints_pos_.txt";
	string path_log_joints_vel  = "log_virt_joints_vel_.txt";
	Out_joints_pos.open(path_log_joints_pos.c_str());
	Out_joints_vel.open(path_log_joints_vel.c_str()); 

	var_gain.setZero();

	while (!ik_run)
	{
		MatrixXd W_EdgesNormalCHull_; 
		VectorXd DistanceEdgesCHull_;
		// this->solve_QPcvxgen(robot_model_, pose_lfoot, pose_rfoot, stanceFoot);     // TODO PLOT EVOLUTION OF COM AND CLAMPI IT
		this->solve_QPcvxgen(robot_model_, pose_lfoot, pose_rfoot, stanceFoot, W_EdgesNormalCHull_, DistanceEdgesCHull_);     // TODO PLOT EVOLUTION OF COM AND CLAMPI IT
		// this->solve_QPoases(robot_model_, pose_lfoot, pose_rfoot, stanceFoot, W_EdgesNormalCHull_, DistanceEdgesCHull_);     // TODO PLOT EVOLUTION OF COM AND CLAMPI IT
		// des_X_EE[0] = Pose_EE[0];
		// des_X_EE[1] = Pose_EE[1];
		// des_X_EE[7] = Pose_EE[7];		
		// clamping the CoM position to move freely but within the convex hull
		// --------------------------------------------------------------------
		if(this->predict_stability(W_EdgesNormalCHull_, DistanceEdgesCHull_, Pose_EE[4].head(2), W_Pos_AbsFoot))
		{
			// des_X_EE[4].head(3) = Pose_EE[4].head(3);
			pxy_CoM_n1 = pxy_CoM;
            pxy_CoM    = (Pose_EE[4].head(2) - W_Pos_AbsFoot);
        }
        else
        {
            // // 
            std::cout<< "\n" << std::endl;
            std::cout<< "COM CLAMPING ACTIVE with : " << pxy_CoM_n1.transpose() << std::endl;
            // // pxy_CoM = (pxy_CoM.norm() - 0.01)/(pxy_CoM.norm()+1e-10) * pxy_CoM;
            // pxy_CoM_n1 = (pxy_CoM_n1.norm() - 0.01)/(pxy_CoM_n1.norm()+1e-10) * pxy_CoM_n1;
            Vector2d DeltaCoMxy = (pxy_CoM_n1.norm() - 0.03)/(pxy_CoM_n1.norm()+1e-10) * pxy_CoM_n1;
            des_X_EE[4].head(2) = DeltaCoMxy + W_Pos_AbsFoot;
            des_X_EE[4](2)      = Pose_EE[4](2);
        }

		// this->solve_QPoases(robot_model_, pose_lfoot, pose_rfoot, stanceFoot);   		// solve_QPoases
		// des_X_EE[4].head(3) = Pose_EE[4].head(3);
		// des_X_EE[5] = Pose_EE[5];
		// des_X_EE[6] = Pose_EE[6];
		// des_X_EE[7].tail(4) = Pose_EE[7].tail(4);
		//
		cnt  = (count >= count_max);
        ik_run = cnt || getTaskCompletionState();
		//
		std::cout<< "WBIK COUNTER IS : " << count << std::endl;
		std::cout<< "\n : " << std::endl;
		// data logging 
		Out_joints_pos << count << "	" << 180./M_PI * virt_jts_pos.transpose() << std::endl;
		Out_joints_vel << count << "	" << q_dot_star.transpose() << std::endl;
		//
		count++;
	}

	//
	// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	for (int i=0; i<8; i++)
	{
		std::cout << " ERROR POS and ORI NORM " << EE[i] << "  : \t" << error_pos_norm[i] << "  and : " << error_ori_norm[i] << std::endl;
		// std::cout << " DESIRED     TASK   VEL " << EE[i] << "  : \t" << des_X_dot_EE[i].transpose() << std::endl;
	}
	//
	des_jts_pos	 = virt_jts_pos;
	des_WHB      = virt_WHB;
	//
	std::cout<< "WBIK SOLUTION LOOP RUN IN : " <<  yarp::os::Time::now()-t_wbIK << " s" << std::endl;
	// std::cout<< "CHECK TASKS IS : \n" << J_SoT * q_dot_star - X_dot_SoT << std::endl;
	for (int i=0; i<8; i++){
		if(i<=3)
			std::cout << " CHECK TASKS " << EE[i] << "  : \n" << Jacobian_EE[i] * q_dot_star - des_X_dot_EE[i] << std::endl;
		if(i==4)  // CoM position
			std::cout << " CHECK TASKS " << EE[i] << "  : \n" << Jacobian_EE[i].topRows(3) * q_dot_star - des_X_dot_EE[i].head(3) << std::endl;
		if(i==5 || i==7)  // pelvis orientation
			std::cout << " CHECK TASKS " << EE[i] << "  : \n" << Jacobian_EE[i].bottomRows(3) * q_dot_star - des_X_dot_EE[i].tail(3) << std::endl;
	}

	// cout << " Desired Joint position are : \n" << 180./M_PI * des_jts_pos.transpose() << endl;
	cout << " Desired Joint position Torso are : \t" << 180./M_PI * des_jts_pos.head(3).transpose() << endl;
	cout << " Desired Joint position Lhand are : \t" << 180./M_PI * des_jts_pos.head(10).tail(7).transpose() << endl;
	cout << " Desired Joint position Rhand are : \t" << 180./M_PI * des_jts_pos.head(17).tail(7).transpose() << endl;
	cout << " Desired Joint position Lleg  are : \t" << 180./M_PI * des_jts_pos.tail(12).head(6).transpose() << endl;
	cout << " Desired Joint position Rleg  are : \t" << 180./M_PI * des_jts_pos.tail(6).transpose() << endl;
	cout << " Desired Base transformation  is : \n" << des_WHB << endl;

	memcpy(task_weight, &task_weight_default[0], 8*sizeof *task_weight_default);

	return true;
}

// ///////////////////////////////  Velocity inverse kinematics //////////////////////////////////////////////////////////////////////
// =====================================================================================================================================
VectorXd wb_InverseKinematics::get_wbIK_velo(WbRobotModel& robot_model_, VectorXd jts_position, Matrix4d WHB, Vector6d des_Velo_EE_[], 
											string stanceFoot, Vector7d pose_lfoot, Vector7d pose_rfoot)
{
	// 
	// coef_grad = 1.0;
	this->q_dot_star.setZero();
	this->virt_jts_pos = jts_position;
	this->update_model(robot_model_, q_dot_star, pose_lfoot, pose_rfoot, stanceFoot);
	this->load_wbIK_VeloTask(des_Velo_EE_);
	this->CvxgenSolveQP_Velo(robot_model_, pose_lfoot, pose_rfoot, stanceFoot);
	//
	// std::cout<< "CHECK TASKS IS : \n" << J_SoT * q_dot_star - X_dot_SoT << std::endl;
	// cout << " Desired Joint position are : \n" << 180./M_PI * des_jts_pos.transpose() << endl;
	cout << " Desired Joint velocity fBase are : \t" << q_dot_star.segment( 0, 6).transpose() << endl;
	cout << " Desired Joint velocity Torso are : \t" << q_dot_star.segment( 6, 3).transpose() << endl;
	cout << " Desired Joint velocity Lhand are : \t" << q_dot_star.segment( 9, 7).transpose() << endl;
	cout << " Desired Joint velocity Rhand are : \t" << q_dot_star.segment(16, 7).transpose() << endl;
	cout << " Desired Joint velocity Lleg  are : \t" << q_dot_star.segment(23, 6).transpose() << endl;
	cout << " Desired Joint velocity Rleg  are : \t" << q_dot_star.segment(29, 6).transpose() << endl;
	//
	return this->q_dot_star;
}

VectorXd wb_InverseKinematics::get_wbIK_velo2(WbRobotModel& robot_model_, VectorXd jts_position, Matrix4d WHB, Vector6d des_Velo_EE_[], VectorXd des_jts_velo,
												string stanceFoot, Vector7d pose_lfoot, Vector7d pose_rfoot, bool retract)
{
	// 
	if(retract)
	{
		memcpy(task_weight, &task_weight_retract[0], 8 * sizeof *task_weight_retract); 
		posture_weight = posture_weight_retract;
		// coef_grad = 1.0;
		this->q_dot_star.setZero();
		this->virt_jts_pos = jts_position;
		this->update_model(robot_model_, q_dot_star, pose_lfoot, pose_rfoot, stanceFoot);
		des_Velo_EE_[7] *=0.5;

		this->load_wbIK_VeloTask2(des_Velo_EE_, des_jts_velo);
		// this->load_wbIK_VeloTask(des_Velo_EE_);
		this->CvxgenSolveQP_Velo(robot_model_, pose_lfoot, pose_rfoot, stanceFoot);

		memcpy(task_weight, &task_weight_default[0], 8 * sizeof *task_weight_default);  // task_weight_retract
		posture_weight.setOnes(nJts);
	}
	else
	{
		memcpy(task_weight, &task_weight_reach_v[0], 8 * sizeof *task_weight_reach_v); 
		// posture_weight.setOnes(nJts);
		posture_weight = posture_weight_reach;
		posture_weight(4)  *= 2.0;
		posture_weight(11) *= 2.0;

		this->q_dot_star.setZero();
		this->virt_jts_pos = jts_position;
		this->update_model(robot_model_, q_dot_star, pose_lfoot, pose_rfoot, stanceFoot);
		this->load_wbIK_VeloTask2(des_Velo_EE_, des_jts_velo);
		// this->load_wbIK_VeloTask(des_Velo_EE_);
		this->CvxgenSolveQP_Velo(robot_model_, pose_lfoot, pose_rfoot, stanceFoot);

		memcpy(task_weight, &task_weight_default[0], 8 * sizeof *task_weight_default);  //
		// posture_weight.setOnes(nJts);
		posture_weight = posture_weight_default;
	}
	//
	// std::cout<< "CHECK TASKS IS : \n" << J_SoT * q_dot_star - X_dot_SoT << std::endl;
	// cout << " Desired Joint position are : \n" << 180./M_PI * des_jts_pos.transpose() << endl;
	
	// cout << " Desired Joint velocity fBase are : \t" << q_dot_star.segment( 0, 6).transpose() << endl;
	// cout << " Desired Joint velocity Torso are : \t" << q_dot_star.segment( 6, 3).transpose() << endl;
	// cout << " Desired Joint velocity Lhand are : \t" << q_dot_star.segment( 9, 7).transpose() << endl;
	// cout << " Desired Joint velocity Rhand are : \t" << q_dot_star.segment(16, 7).transpose() << endl;
	// cout << " Desired Joint velocity Lleg  are : \t" << q_dot_star.segment(23, 6).transpose() << endl;
	// cout << " Desired Joint velocity Rleg  are : \t" << q_dot_star.segment(29, 6).transpose() << endl;
	//
	return this->q_dot_star;
}
// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



Vector6d wb_InverseKinematics::get_hands_errors()
{
	Vector6d hands_errors; 	hands_errors.setZero();
	hands_errors(0) = this->error_pos_norm[0];
	hands_errors(1) = this->error_ori_norm[0];
	hands_errors(2) = this->error_pose[0].norm();
	hands_errors(3) = this->error_pos_norm[1];
	hands_errors(4) = this->error_ori_norm[1];
	hands_errors(5) = this->error_pose[1].norm();
	//
	return hands_errors;
}

Matrix4d wb_InverseKinematics::get_fbase_pose(WbRobotModel& robot_model_, Vector7d pose_lfoot, Vector7d pose_rfoot, std::string stanceFoot, VectorXd jts_pos_)
{
	Matrix4d world_H_base_ = Eigen::MatrixXd::Identity(4,4);
	Vector7d virtW_stf_pose;
	Matrix4d virtW_H_sft;
	Matrix4d W_H_sft;
	//
	if(stanceFoot != "left")  // right stance foot
	{
    	robot_model_.getLinkPose(jts_pos_, world_H_base_, "right_foot",  virtW_stf_pose);
    	W_H_sft =  robot_model_.Transforms.PoseVector2HomogenousMx(pose_rfoot);  // current pose of the stance foot wrt. the world frame
	} else {
      	robot_model_.getLinkPose(jts_pos_, world_H_base_, "left_foot",  virtW_stf_pose);
      	W_H_sft = robot_model_.Transforms.PoseVector2HomogenousMx(pose_lfoot);   // current pose of the stance foot wrt. the world frame
    }
    //
    virtW_H_sft = robot_model_.Transforms.PoseVector2HomogenousMx(virtW_stf_pose);
       	
   	return W_H_sft * virtW_H_sft.inverse();  // virtW_H_sft.inverse() : floating base pose in virtual world defined by stance foot
}

// express the feet support points in the absolute frame of the feet
void wb_InverseKinematics::get_feet_support_points(	MatrixXd PtsInFoot, Vector7d Pose_lfoot, Vector7d Pose_rfoot, 
													MatrixXd &AllPtsInAbsFoot_, Matrix2d &W_Rot_AbsFoot, Vector2d &W_Pos_AbsFoot)
{
    // Kinematic transformations class
    // KineTransformations Transforms_;
    // 3D rotation matrix of the feet wrt the world frame
    Eigen::Matrix3d WR_lf = Transforms.PoseVector2HomogenousMx(Pose_lfoot).block(0,0,3,3);
    Eigen::Matrix3d WR_rf = Transforms.PoseVector2HomogenousMx(Pose_rfoot).block(0,0,3,3);
    // Planar orientations of the feet 
    Vector3d o_lf = Transforms.getEulerAnglesXYZ_FixedFrame(WR_lf);
    Vector3d o_rf = Transforms.getEulerAnglesXYZ_FixedFrame(WR_rf);
    //
    int np_f = PtsInFoot.cols();        // number of points in one foot support polygon

    AllPtsInAbsFoot_.resize(2,np_f*2); // coordinates of the point in the absolute frame
    AllPtsInAbsFoot_.setZero();
    MatrixXd W_PtsSwingFt(2,np_f);      												// Matrix of points in the swing foot (foot that will perform the stepping)
    MatrixXd W_PtsStanceFt(2,np_f);     												// Matrix of points in the stance foot (foot that stays fixed)
    MatrixXd RotStanceFt(2,2);          												// Plananr Rotation matrix of the stance foot wrt the the world frane
    Vector2d TransStanceFt(2);          												// Plananr Position vector of the stance foot wrt the world frame
    MatrixXd RotSwingFt(2,2);           												// Plananr Rotation matrix of the swing foot wrt the the world frane
    Vector2d TransSwingFt(2);           												// Plananr Position vector of the swing foot wrt the world frame
    Matrix2d AbsFoot_Rot_W;             												// Rotation of the world frame relatibve to the absolute feet frame
    Vector2d AbsFoot_Trans_W;           												// position of the world frame relatibve to the absolute feet frame
    //
    double DthetaFt = 0.0;              												// Relative angle between stance and swing foot

    // Extraction of the feet orientation
    double theta_stance = o_lf(2);      												//Transforms.getEulerAnglesXYZ_FixedFrame(WR_lf).(2);
    double theta_swing  = o_rf(2); 
    // Planar rotations of the feet
    RotStanceFt << cos(theta_stance), -sin(theta_stance), sin(theta_stance), cos(theta_stance);
    RotSwingFt  << cos(theta_swing),  -sin(theta_swing),  sin(theta_swing),  cos(theta_swing);
    // Planar translations of the feet
    TransStanceFt = Pose_lfoot.head(2);
    TransSwingFt  = Pose_rfoot.head(2);    
    // Expression of pts from foot frame to world frame
    for (int i=0; i<np_f; i++)   {
        W_PtsStanceFt.col(i) = RotStanceFt * PtsInFoot.col(i) + TransStanceFt;
        W_PtsSwingFt.col(i)  = RotSwingFt  * PtsInFoot.col(i) + TransSwingFt;
    }    
    // Expression of the feet polygon points in the absolute foot frame and 
    MatrixXd W_All_Pts(2,np_f*2);
    W_All_Pts.leftCols(np_f)  = W_PtsStanceFt;
    W_All_Pts.rightCols(np_f) = W_PtsSwingFt;

    // get the absolute feet pose
    // ---------------------------
    W_Pos_AbsFoot = 0.5 * (TransStanceFt + TransSwingFt);  								// 2D position 
    double abs_angle = 0.5*(theta_stance + theta_swing);	    						// 2D rotation
    // double abs_angle = theta_stance;
    // double abs_angle = theta_stance;
    W_Rot_AbsFoot << cos(abs_angle), -sin(abs_angle), sin(abs_angle), cos(abs_angle); 
    AbsFoot_Trans_W = -W_Pos_AbsFoot;													// Get the position of the absolute foot frame
    AbsFoot_Rot_W   = W_Rot_AbsFoot.transpose();										// Planar rotation of the equivalent foot frame (absolute frame)
    for (int i=0; i<np_f*2; i++) 
        AllPtsInAbsFoot_.col(i) = AbsFoot_Rot_W * W_All_Pts.col(i) + AbsFoot_Trans_W;  	// Transformation
    //
    return;
}

// get convex hull variables
// -------------------------

bool wb_InverseKinematics::getConvexHullVariables(MatrixXd PtsInFoot_, Vector7d Pose_lfoot, Vector7d Pose_rfoot,  
												  MatrixXd &W_EdgesNormalCHull, VectorXd &DistanceEdgesCHull, 
												  Matrix2d &W_Rot_AbsFoot, Vector2d &W_Pos_AbsFoot)
{
	// computation of the convex polygon
    MatrixXd AllPtsInAbsFoot;
    // MatrixXd PointsCHull;
    MatrixXd Abs_EdgesNormalCHull;
    
	// Expressing the feet contact points in the absolute feet frame
	this->get_feet_support_points(PtsInFoot_, Pose_lfoot, Pose_rfoot, AllPtsInAbsFoot, W_Rot_AbsFoot, W_Pos_AbsFoot);  // chech also with des_X_EE
	// In the absolute feet frame compute the convex hull and the normals to the edges and 
	ConvHull.convexHull(AllPtsInAbsFoot, this->PointsCHull, Abs_EdgesNormalCHull, DistanceEdgesCHull); 
	// Express the normal in the world frame
	W_EdgesNormalCHull.resize(Abs_EdgesNormalCHull.rows(), Abs_EdgesNormalCHull.cols());
	W_EdgesNormalCHull = Abs_EdgesNormalCHull * W_Rot_AbsFoot.transpose();

	for(int i=0; i<W_EdgesNormalCHull.rows(); i++)
	{
		W_EdgesNormalCHull.row(i) = W_EdgesNormalCHull.row(i)/(W_EdgesNormalCHull.row(i).norm() + 1.e-20);
	}

	// std::cout << " POINTS CONVEX HULL : \n" << this->PointsCHull << std::endl;

	return true;
}


// Stability condition
bool wb_InverseKinematics::predict_stability(MatrixXd W_EdgesNormalCHull_, VectorXd DistanceEdgesCHull_, Vector2d W_CoM_, Vector2d W_Pos_AbsFt_)
{
    //
    int nCHPts = W_EdgesNormalCHull_.rows();

    bool result = true;
    bool check = true;

    for(int i=0; i<nCHPts; i++){
    	check = (W_EdgesNormalCHull_.row(i) * (W_CoM_ - W_Pos_AbsFt_) <= DistanceEdgesCHull_(i));
        result = result && check;
        //
        // cout << " DCOM is: \t" << (W_CoM_ - W_Pos_AbsFt_).transpose()  << " \t";
        // cout << " AbsF is: \t" << W_Pos_AbsFt_.transpose()  << " \t";
        // cout << " computed distance is: \t" << W_EdgesNormalCHull_.row(i) * (W_CoM_ - W_Pos_AbsFt_)  << " \t";
        // cout << " Distance is :\t" << DistanceEdgesCHull_(i) << "\t";
        // cout << " check is   : \t" << check << endl;
    }

    return result;  
}

//
bool wb_InverseKinematics::check_reachability(WbRobotModel& robot_model_, VectorXd jts_position, Matrix4d WHB, Vector7d des_X_EE_[], string stanceFoot, Vector7d pose_lfoot, Vector7d pose_rfoot)
{
	bool ok = false;
	bool e_lh, e_rh;

	// load the reachability parameters
	// task_weight[] = task_weight_reach[];
	memcpy(task_weight, &task_weight_reach[0], 8 * sizeof *task_weight_reach);
	//	
	isCoMClamping = true;

	Vector2d w_aH_pos = 0.5*(des_X_EE_[0].head(2) + des_X_EE_[1].head(2));
	Vector2d w_aF_pos = 0.5*(des_X_EE_[2].head(2) + des_X_EE_[3].head(2));

	if((w_aH_pos-w_aF_pos).norm() <= max_reach)
	{
		this->get_wb_IK(robot_model_, jts_position, WHB, des_X_EE_, stanceFoot, pose_lfoot, pose_rfoot);
		this->get_hands_errors();
		// check of the hands errors
		e_lh = ((error_pos_norm[0] < reach_tol(0)) && (error_ori_norm[0] < 10.*reach_tol(1)));	    // error lhand
		e_rh = ((error_pos_norm[1] < reach_tol(0)) && (error_ori_norm[1] < 10.*reach_tol(1)));		// error rhand

		if(e_lh && e_rh)
			ok = true;
		else
			ok = false;
	}

	// set back the weights to their default values
	memcpy(task_weight, &task_weight_default[0], 8*sizeof *task_weight_default);

	return ok;
}


Vector3d wb_InverseKinematics::get_wb_fmmcom(WbRobotModel& robot_model_, VectorXd jts_pos, Vector7d des_X_EE_[], string stanceFoot, 
											 Vector7d pose_lfoot, Vector7d pose_rfoot, Vector2d Disturb_c_, double fu_z_mgbetaW_)
{
	//
	virt_jts_pos = jts_pos;
	// virt_jts_pos(nJts-9) -=0.02;  // add angle to set the knee out joints limits and singularity
	// virt_jts_pos(nJts-3) -=0.02;  // add angle to set the knee out joints limits and singularity
	virt_jts_pos = q_0;
	q_dot_star.setZero();
	// virt_WHB	 = WHB;
	for(int i=0; i<8; i++) des_X_EE[i] = des_X_EE_[i];
	//
	int count = 0;
	bool cnt, e_lh, e_rh, e_lf, e_rf, ik_run;
		 cnt = e_lh = e_rh = e_lf = e_rf = ik_run = false;
	// loop over the specified number of iteration if the error is still large
	double t_wbIK = yarp::os::Time::now();

	// data logging  // temporary
	string path_log_joints_pos  = "log_virt_joints_pos_.txt";
	string path_log_joints_vel  = "log_virt_joints_vel_.txt";
	Out_joints_pos.open(path_log_joints_pos.c_str());
	Out_joints_vel.open(path_log_joints_vel.c_str()); 

	var_gain.setZero();

	while (!ik_run)
	{
		MatrixXd W_EdgesNormalCHull_; 
		VectorXd DistanceEdgesCHull_;

		this->solve_QPcvxgen(robot_model_, pose_lfoot, pose_rfoot, stanceFoot, W_EdgesNormalCHull_, DistanceEdgesCHull_);     // TODO PLOT EVOLUTION OF COM AND CLAMPI IT
		// specify the reference value of the CoM
		des_X_EE[4].head(2) = -(Disturb_c_ + fu_z_mgbetaW_ * Pose_EE[4].head(2) - 0.5*(Pose_EE[2].head(2) + Pose_EE[3].head(2)));
		// clamping the CoM position to move freely but within the convex hull
		if(this->predict_stability(W_EdgesNormalCHull_, DistanceEdgesCHull_, Pose_EE[4].head(2), W_Pos_AbsFoot))
		{
			des_X_EE[4](2) = Pose_EE[4](2);
			pxy_CoM_n1 = pxy_CoM;
            pxy_CoM    = (Pose_EE[4].head(2) - W_Pos_AbsFoot);
        }
        else
        {
            std::cout<< "\n" << std::endl;
            std::cout<< "COM CLAMPING ACTIVE with : " << pxy_CoM_n1.transpose() << std::endl;
            // pxy_CoM_n1 = (pxy_CoM_n1.norm() - 0.01)/(pxy_CoM_n1.norm()+1e-10) * pxy_CoM_n1;
            Vector2d DeltaCoMxy = (pxy_CoM_n1.norm() - 0.03)/(pxy_CoM_n1.norm()+1e-10) * pxy_CoM_n1;
            des_X_EE[4].head(2) = DeltaCoMxy + W_Pos_AbsFoot;
            des_X_EE[4](2)      = Pose_EE[4](2);
        }
        //
		cnt  = (count >= count_max);
		e_lh = ((error_pos_norm[0] < epsilon) && (error_ori_norm[0] < 10.*epsilon));	    // error lhand
		e_rh = ((error_pos_norm[1] < epsilon) && (error_ori_norm[1] < 10.*epsilon));		// error rhand
		e_lf = ((error_pos_norm[2] < epsilon) && (error_ori_norm[2] < 10.*epsilon));		// error lfoot
		e_rf = ((error_pos_norm[3] < epsilon) && (error_ori_norm[3] < 10.*epsilon)); 		// error rfoot
		//
		ik_run = cnt || (e_lh && e_rh && e_lf && e_rf);
		//
		std::cout<< "WBIK COUNTER IS : " << count << std::endl;
		std::cout<< "\n : " << std::endl;
		// data logging 
		Out_joints_pos << count << "	" << 180./M_PI * virt_jts_pos.transpose() << std::endl;
		Out_joints_vel << count << "	" << q_dot_star.transpose() << std::endl;
		//
		count++;
	}
	// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	des_jts_pos	 = virt_jts_pos;
	des_WHB      = virt_WHB;
	//
	for (int i=0; i<8; i++)
		std::cout << " ERROR POS and ORI NORM " << EE[i] << "  : \t" << error_pos_norm[i] << "  and : " << error_ori_norm[i] << std::endl;

	//
	std::cout<< "WBIK SOLUTION LOOP RUN IN : " <<  yarp::os::Time::now()-t_wbIK << " s" << std::endl;
	// std::cout<< "CHECK TASKS IS : \n" << J_SoT * q_dot_star - X_dot_SoT << std::endl;
	for (int i=0; i<8; i++){
		if(i<=3)
			std::cout << " CHECK TASKS " << EE[i] << "  : \n" << Jacobian_EE[i] * q_dot_star - des_X_dot_EE[i] << std::endl;
		if(i==4)  // CoM position
			std::cout << " CHECK TASKS " << EE[i] << "  : \n" << Jacobian_EE[i].topRows(3) * q_dot_star - des_X_dot_EE[i].head(3) << std::endl;
		if(i==5 || i==7)  // pelvis orientation
			std::cout << " CHECK TASKS " << EE[i] << "  : \n" << Jacobian_EE[i].bottomRows(3) * q_dot_star - des_X_dot_EE[i].tail(3) << std::endl;
	}

	// cout << " Desired Joint position are : \n" << 180./M_PI * des_jts_pos.transpose() << endl;
	cout << " Desired Joint position Torso are : \t" << 180./M_PI * des_jts_pos.head(3).transpose() << endl;
	cout << " Desired Joint position Lhand are : \t" << 180./M_PI * des_jts_pos.head(10).tail(7).transpose() << endl;
	cout << " Desired Joint position Rhand are : \t" << 180./M_PI * des_jts_pos.head(17).tail(7).transpose() << endl;
	cout << " Desired Joint position Lleg  are : \t" << 180./M_PI * des_jts_pos.tail(12).head(6).transpose() << endl;
	cout << " Desired Joint position Rleg  are : \t" << 180./M_PI * des_jts_pos.tail(6).transpose() << endl;
	cout << " Desired Base transformation  is : \n" << des_WHB << endl;

	return this->Pose_EE[4].head(3);
}

bool wb_InverseKinematics::update_stepping_weight(std::string stance_ft_)
{
	//
	task_weight_aStep[0] <<     5.0,    5.0,   5.0,   1.00,  1.00,  1.00;  	// weight_lhand; 
	task_weight_aStep[1] <<     5.0,    5.0,   5.0,   1.00,  1.00,  1.00;  	// weight_rhand; 
	task_weight_aStep[2] <<   100.0,  100.0, 100.0,  100.0, 100.0, 100.0; 	// weight_lfoot; 
	task_weight_aStep[3] <<   100.0,  100.0, 100.0,  100.0, 100.0, 100.0; 	// weight_rfoot;
	task_weight_aStep[4] <<   5.e-0,  5.e-0, 5.e-1,   0.00,  0.00,  0.00;  	// weight_CoMp; 
	task_weight_aStep[5] <<    0.00,   0.00,  0.00,  2.e-1, 1.e-2, 5.e-2;  	// weight_pelvis;  	
	task_weight_aStep[6] <<    0.00,   0.00,  0.00,  1.e-5, 1.e-5, 5.e-5;  	// weight_chest; 
	task_weight_aStep[7] <<    0.00,   0.00,  0.00,  2.e-1, 1.e-2, 5.e-2;  	// weight_pelvis; 

	task_weight_aStep[4] *=4.;
	// task_weight_aStep[0] *=8.0;
	// task_weight_aStep[1] *=8.0;
	task_weight_aStep[0] *=15.0;
	task_weight_aStep[1] *=15.0;

	task_weight_aStep[5] *=10.0;
	task_weight_aStep[6] *=10.0;

	if(stance_ft_ == "left")
	{
		task_weight_aStep[2] *=10.;
		task_weight_aStep[3] *=0.6;
	}
	else  // right stance foot
	{
		task_weight_aStep[2] *=0.6;
		task_weight_aStep[3] *=10.;
	}
	return true;
}

bool wb_InverseKinematics::getTaskCompletionState()
{
	//
	bool e_lh,  e_rh,  e_lf,  e_rf;
         e_lh = e_rh = e_lf = e_rf  = false;
    //
	e_lh = ((this->error_pos_norm[0] < epsilon) && (this->error_ori_norm[0] < 9.*epsilon));   // error lhand
    e_rh = ((this->error_pos_norm[1] < epsilon) && (this->error_ori_norm[1] < 9.*epsilon));   // error rhand
    e_lf = ((this->error_pos_norm[2] < epsilon) && (this->error_ori_norm[2] < 9.*epsilon));   // error lfoot
    e_rf = ((this->error_pos_norm[3] < epsilon) && (this->error_ori_norm[3] < 9.*epsilon));   // error rfoot

	return (e_lh && e_rh && e_lf && e_rf);
}



void wb_InverseKinematics::getFeetSelfCollisionAvoidanceConstraints(string stance, double Ly_min, MatrixXd Jac_lf, MatrixXd Jac_rf, 
																	Vector7d pose_lfoot, Vector7d pose_rfoot, MatrixXd &Mssp, VectorXd &dssp)
{
	// Kinematic transformations class
    KineTransformations Transforms_;
    // 3D rotation matrix of the feet wrt the world frame
    Eigen::Matrix3d WR_lf = Transforms_.PoseVector2HomogenousMx(pose_lfoot).block(0,0,3,3);
    Eigen::Matrix3d WR_rf = Transforms_.PoseVector2HomogenousMx(pose_rfoot).block(0,0,3,3);
    // Planar orientations of the feet 
    Vector3d o_lf = Transforms_.getEulerAnglesXYZ_FixedFrame(WR_lf);
    Vector3d o_rf = Transforms_.getEulerAnglesXYZ_FixedFrame(WR_rf);
    //
    Matrix2d Rot_st = MatrixXd::Identity(2,2);
    //
    if(stance == "left")
    {
    	Rot_st << cos(o_lf(2)), -sin(o_lf(2)), sin(o_lf(2)), cos(o_lf(2));
    	Mssp = Rot_st.transpose() * (Jac_rf.topRows(2) - Jac_lf.topRows(2));
    	dssp = -Rot_st.transpose() * (pose_rfoot.head(2) - pose_lfoot.head(2));
    }
    else
    {
    	Rot_st << cos(o_rf(2)), -sin(o_rf(2)), sin(o_rf(2)), cos(o_rf(2));
    	Mssp = -Rot_st.transpose() * (Jac_lf.topRows(2) - Jac_rf.topRows(2));
    	dssp = Rot_st.transpose() * (pose_lfoot.head(2) - pose_rfoot.head(2));
    }
}

//
bool wb_InverseKinematics::update_model_legsIK(WbRobotModel& robot_model_, VectorXd q_dot_star_, Vector7d pose_lfoot, Vector7d pose_rfoot, string stanceFoot)
{
	// yarp::os::LockGuard guard(m_mutex);
	// Update the robot model
	// ======================	 
	// update the joint position and velocity
	virt_jts_pos.tail(12) += q_dot_star_.tail(12) * virtual_sampTime; 
	// virt_jts_pos += 0.5*(q_dot_star_.tail(nJts) +  q_dot_star_0.tail(nJts))* virtual_sampTime;
	// q_dot_star_0 = q_dot_star_;
	virt_jts_vel.head(6)  = q_dot_star_.head(6);
	virt_jts_vel.tail(12) = q_dot_star_.tail(12);
	// Enforcing joints limits
	for(int i=0; i<robot_model_.getDoFs(); i++)
	{
		// set joint limits hard limits
		if(virt_jts_pos(i) > maxJointLimits(i)){  // upper limit
			virt_jts_pos(i) = maxJointLimits(i) - 0.02;
		} else if(virt_jts_pos(i) < minJointLimits(i)){ // lower limit
			virt_jts_pos(i) = minJointLimits(i) + 0.02;
		} 
	}
	// update the floating base pose and velocity
	// Transforms.UpdatePose_From_VelocityTwist(virtual_sampTime, q_dot_star_.head(6), virt_WHB);
	// virt_VB = q_dot_star_.head(6);
	double t_wbIK = yarp::os::Time::now();
	//
	virt_WHB = get_fbase_pose(robot_model_, pose_lfoot, pose_rfoot, stanceFoot, virt_jts_pos);
	// yarp::os::LockGuard guard(m_mutex);
	for(int i=2; i<5; i++)
	{
		robot_model_.getJacobian(virt_jts_pos, virt_WHB, EE[i], Jacobian_EE[i]);
		robot_model_.getLinkPose(virt_jts_pos, virt_WHB, EE[i],  Pose_EE[i]);
	}
	//
	Jac_lf.leftCols(6)   = Jacobian_EE[2].leftCols(6);
	Jac_lf.rightCols(12) = Jacobian_EE[2].rightCols(12);
	//
	Jac_rf.leftCols(6)   = Jacobian_EE[3].leftCols(6);
	Jac_rf.rightCols(12) = Jacobian_EE[3].rightCols(12);
	//
	Jac_com.leftCols(6)   = Jacobian_EE[4].topLeftCorner(3,6);
	Jac_com.rightCols(12) = Jacobian_EE[4].topRightCorner(3,12);
	//
	// std::cout<< " JACOBIAN AND POSES GOT IN : " <<  yarp::os::Time::now()-t_wbIK << " s" << std::endl;	

	// Update the desired task as function of the updated robot state
	// ==============================================================
	alpha_g = 0.25;	//0.1
	var_gain = (1.-alpha_g) * var_gain + alpha_g * 4.0* gain_vector_0;  // 0.3
	gain_vector = gain_vector_0 + var_gain;

	for (int i=2; i<5; i++)	
	{
		// pose error
		error_pose[i] 	  = Transforms.computePoseErrorNormalizedAxisAngle(Pose_EE[i], des_X_EE[i]);
		error_pos_norm[i] = error_pose[i].head(3).norm();
		error_ori_norm[i] = error_pose[i].tail(3).norm();
		des_X_dot_EE[i]	  = gain_vector.asDiagonal() *error_pose[i];							// scaled error by convergence rate (gain)
	}
	//
	des_q_dot.tail(nJts) = -alpha_q *(virt_jts_pos -q_0) * 0.01; 			// 0.3
	//
	Jl_T_Wx_Jl.setZero();
	Jl_T_Wx_Xdot.setZero();

	// std::cout<< " JACOBIAN AND POSES GOT IN : " <<  yarp::os::Time::now()-t_wbIK << " s" << std::endl;	

	MatrixXd 	Jac_posture = MatrixXd::Zero(12,18);
				Jac_posture.rightCols(12) = MatrixXd::Identity(12,12);
	// Jac_posT_Wp.bottomRows(12) = posture_weight.tail(12).asDiagonal();

	Jl_T_Wx_Jl  += Jac_lf.transpose()  * task_weight[2].asDiagonal() * Jac_lf
			    +  Jac_rf.transpose()  * task_weight[3].asDiagonal() * Jac_rf
			    +  Jac_com.transpose() * task_weight[4].head(3).asDiagonal() * Jac_com
			    +  1e-4 * MatrixXd::Identity(18,18)  // regularization
			    +  0.02*Jac_posture.transpose() * posture_weight.tail(12).asDiagonal() *Jac_posture;

	Jl_T_Wx_Xdot -=  Jac_lf.transpose()  * task_weight[2].asDiagonal() * des_X_dot_EE[2]
			     -   Jac_rf.transpose()  * task_weight[3].asDiagonal() * des_X_dot_EE[3]
			     -   Jac_com.transpose() * task_weight[4].head(3).asDiagonal() * des_X_dot_EE[4].head(3)
			     -   0.02*Jac_posture.transpose() * posture_weight.tail(12).asDiagonal() * des_q_dot.tail(12);

	// std::cout<< " Hessian and Gradiant QP : " <<  yarp::os::Time::now()-t_wbIK << " s" << std::endl;	

	// Jacobian_posture = MatrixXd::Zero(nJts, nJts+6);
	// Jacobian_posture.rightCols(nJts) = MatrixXd::Identity(nJts,nJts);
	// J_T_Wx_J    +=  0.01*Jacobian_posture.transpose() * posture_weight.asDiagonal() * Jacobian_posture;
	// 		J_T_Wx_Xdot -=  0.01*Jacobian_posture.transpose() * posture_weight.asDiagonal() * des_q_dot.tail(nJts);

	return true;
}

//
bool wb_InverseKinematics::solve_QP_legIK(WbRobotModel& robot_model_, Vector7d pose_lfoot, Vector7d pose_rfoot, string stanceFoot, MatrixXd &W_EdgesNormalCHull, VectorXd &DistanceEdgesCHull)
{
	//
	// initialization of the cvxgen solver for the wb inverse kinematics
    double t_wbIK = yarp::os::Time::now();
    //
	this->update_model_legsIK(robot_model_, q_dot_star_legIK, pose_lfoot, pose_rfoot, stanceFoot);
	
	// computation of the convex polygon   
	this->getConvexHullVariables(this->PtsInFoot, Pose_EE[2], Pose_EE[3],  W_EdgesNormalCHull, DistanceEdgesCHull, W_Rot_AbsFoot, W_Pos_AbsFoot);
	//
	MatrixXd NeJc = MatrixXd::Zero(6, 18);
	Vector6d deXc = VectorXd::Zero(6);
	if(W_EdgesNormalCHull.rows() <= 5){
		int r = W_EdgesNormalCHull.rows();
		NeJc.topRows(r) = W_EdgesNormalCHull.topRows(r)* Jac_com.topRows(2);
		deXc.head(r)    = DistanceEdgesCHull.head(r) + W_EdgesNormalCHull.topRows(r)*(W_Pos_AbsFoot - Pose_EE[4].head(2));
	}
	else{
		NeJc = W_EdgesNormalCHull.topRows(6)* Jac_com.topRows(2);
		deXc = DistanceEdgesCHull.head(6) + W_EdgesNormalCHull.topRows(6)*(W_Pos_AbsFoot - Pose_EE[4].head(2));
	}

	// NeJc *= 0.0;
	// deXc *= 0.0;
	// Hessian matrix
	memcpy(legIK_params.Q, Jl_T_Wx_Jl.data(), sizeof(double) * Jl_T_Wx_Jl.size());
	memcpy(legIK_params.P, &Jl_T_Wx_Xdot[0], sizeof(double)*(18)); 				// weight acceleration and desired posture
	for(int i=0;i<6;i++) 
		memcpy(legIK_params.NeJc[i+1], &NeJc(i,0),	sizeof(double)*(18));		// convexHull constraints Matrix for CoM
	memcpy(legIK_params.deXc, &deXc[0], 			sizeof(double)*(6));			// convexHull constraints vector for CoM
	
	// setting of hard constraints
	// Joint limits min and max
	for(int i=0; i<12; i++) 	legIK_params.qmin[i] = (minJointLimits(i+17) - virt_jts_pos(i+17));
	for(int i=0; i<12; i++) 	legIK_params.qmax[i] = (maxJointLimits(i+17) - virt_jts_pos(i+17));
	// sampling time
	legIK_params.dt[0] = virtual_sampTime;
	// Solve the QP (acceleration)
	legIK_settings.verbose = 0;
	legIK_settings.eps = 15e-5;
	legIK_settings.resid_tol = 1e-4;
	// legIK_settings.kkt_reg = 1e-7;
	// legIK_settings.refine_steps = 1;
	int iter = legIK_solve();
	// Extract the QP solution (acceleration and slack)
	for(int i=0; i<18; i++) q_dot_star_legIK(i) = legIK_vars.qdot[i];

	return true;
}

// ========================================================================

bool wb_InverseKinematics::get_legIK(WbRobotModel& robot_model_, VectorXd jts_position, Matrix4d WHB, Vector7d des_X_EE_[], string stanceFoot, Vector7d pose_lfoot, Vector7d pose_rfoot)
{
	virt_jts_pos = jts_position;
	virt_jts_pos(nJts-9) -=0.02;  // add angle to set the knee out joints limits and singularity
	virt_jts_pos(nJts-3) -=0.02;  // add angle to set the knee out joints limits and singularity
	// virt_jts_pos = q_0;
	q_dot_star.setZero();
	// virt_WHB	 = WHB;
	for(int i=0; i<8; i++) des_X_EE[i] = des_X_EE_[i];
	//
	int count = 0;
	bool cnt, e_lf, e_rf, ik_run;
		 cnt = e_lf = e_rf = ik_run = false;
	// loop over the specified number of iteration if the error is still large
	double t_wbIK = yarp::os::Time::now();
	// data logging  // temporary
	string path_log_joints_pos  = "log_virt_joints_pos_.txt";
	string path_log_joints_vel  = "log_virt_joints_vel_.txt";
	Out_joints_pos.open(path_log_joints_pos.c_str());
	Out_joints_vel.open(path_log_joints_vel.c_str()); 
	//
	memcpy(task_weight, &task_weight_legIK[0], 8*sizeof *task_weight_legIK);
	//
	var_gain.setZero();
	//
	while (!ik_run)
	{
		MatrixXd W_EdgesNormalCHull_; 
		VectorXd DistanceEdgesCHull_;
		this->solve_QP_legIK(robot_model_, pose_lfoot, pose_rfoot, stanceFoot, W_EdgesNormalCHull_, DistanceEdgesCHull_);     // TODO PLOT EVOLUTION OF COM AND CLAMPI IT

		// clamping the CoM position to move freely but within the convex hull
		// --------------------------------------------------------------------
		if(this->predict_stability(W_EdgesNormalCHull_, DistanceEdgesCHull_, Pose_EE[4].head(2), W_Pos_AbsFoot))
		{
			// des_X_EE[4].head(3) = Pose_EE[4].head(3);
			pxy_CoM_n1 = pxy_CoM;
            pxy_CoM    = (Pose_EE[4].head(2) - W_Pos_AbsFoot);
        }
        else
        {
            // // 
            std::cout<< "\n" << std::endl;
            std::cout<< "COM CLAMPING ACTIVE with : " << pxy_CoM_n1.transpose() << std::endl;
            Vector2d DeltaCoMxy = (pxy_CoM_n1.norm() - 0.03)/(pxy_CoM_n1.norm()+1e-10) * pxy_CoM_n1;
            // des_X_EE[4].head(2) = DeltaCoMxy + W_Pos_AbsFoot;
            // des_X_EE[4](2)      = Pose_EE[4](2);
        }

		cnt  = (count >= count_max);
		e_lf = ((error_pos_norm[2] < epsilon) && (error_ori_norm[2] < 10.*epsilon));		// error lfoot
		e_rf = ((error_pos_norm[3] < epsilon) && (error_ori_norm[3] < 10.*epsilon)); 		// error rfoot
		//
		ik_run = cnt || (e_lf && e_rf);
		//
		std::cout<< "WBIK COUNTER IS : " << count << std::endl;
		std::cout<< "\n : " << std::endl;

		// data logging 
		Out_joints_pos << count << "	" << 180./M_PI * virt_jts_pos.transpose() << std::endl;
		VectorXd wb_qdot = VectorXd::Zero(virt_jts_pos.rows()+6);
		wb_qdot.head(6) = q_dot_star_legIK.head(6);
		wb_qdot.tail(12) = q_dot_star_legIK.tail(12);
		Out_joints_vel << count << "	" << wb_qdot.transpose() << std::endl;
		//
		count++;
	}
	//
	memcpy(task_weight, &task_weight_default[0], 8*sizeof *task_weight_default);
	// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	for (int i=2; i<5; i++)
	{
		std::cout << " ERROR POS and ORI NORM " << EE[i] << "  : \t" << error_pos_norm[i] << "  and : " << error_ori_norm[i] << std::endl;
		// std::cout << " DESIRED     TASK   VEL " << EE[i] << "  : \t" << des_X_dot_EE[i].transpose() << std::endl;
	}
	//
	des_jts_pos	 = virt_jts_pos;
	des_WHB      = virt_WHB;
	//
	std::cout<< "WBIK SOLUTION LOOP RUN IN : " <<  yarp::os::Time::now()-t_wbIK << " s" << std::endl;

	// cout << " Desired Joint position are : \n" << 180./M_PI * des_jts_pos.transpose() << endl;
	cout << " Desired Joint position Torso are : \t" << 180./M_PI * des_jts_pos.head(3).transpose() << endl;
	cout << " Desired Joint position Lhand are : \t" << 180./M_PI * des_jts_pos.head(10).tail(7).transpose() << endl;
	cout << " Desired Joint position Rhand are : \t" << 180./M_PI * des_jts_pos.head(17).tail(7).transpose() << endl;
	cout << " Desired Joint position Lleg  are : \t" << 180./M_PI * des_jts_pos.tail(12).head(6).transpose() << endl;
	cout << " Desired Joint position Rleg  are : \t" << 180./M_PI * des_jts_pos.tail(6).transpose() << endl;
	cout << " Desired Base transformation  is : \n" << des_WHB << endl;

	return true;
}


//
bool wb_InverseKinematics::get_legIKCompletionState()
{
	//
	bool e_cm,  e_lf,  e_rf;
         e_cm = e_lf = e_rf  = false;
    //
    e_lf = ((this->error_pos_norm[2] < epsilon) && (this->error_ori_norm[2] < 10.*epsilon));   // error lfoot
    e_rf = ((this->error_pos_norm[3] < epsilon) && (this->error_ori_norm[3] < 10.*epsilon));   // error rfoot
    e_cm = ((this->error_pos_norm[4] < epsilon) && (this->error_ori_norm[4] < 10.*epsilon));   // error rfoot

	return (e_cm && e_lf && e_rf);
}


//
bool wb_InverseKinematics::get_legIK_batch(WbRobotModel& robot_model_, VectorXd jts_position, Matrix4d WHB, Vector7d des_X_EE_[], string stanceFoot, Vector7d pose_lfoot, Vector7d pose_rfoot)
{
    //
    std::cout<< "get_legIK_batch  started : " << 0.0 << std::endl;
    bool cnt    = false;
    bool n_loop = false;

    //------------------------------------------------
    if(!legIK_batch_start)
    {
    	//
    	isFreeze = true;
    	freeze.segment(6,17) = VectorXd::Ones(17);
        // Inititialization();
        l_count = 0;
        g_count = 0;
        ik_cont  = false;
        // ==================================================================================================================
        virt_jts_pos = jts_position;
        virt_jts_pos(nJts-9) -=0.02;  // add angle to set the knee out joints limits and singularity
		virt_jts_pos(nJts-3) -=0.02;  // add angle to set the knee out joints limits and singularity
        // virt_jts_pos = q_0;
        q_dot_star.setZero();

        // virt_WHB  = WHB;
        for(int i=0; i<8; i++) des_X_EE[i] = des_X_EE_[i];
        // load the FMMCOM parameters
        memcpy(task_weight, &task_weight_legIK[0], 8*sizeof *task_weight_legIK);
    	//
    	var_gain.setZero();
	    gain_vector_0.head(3) = -Vector3d(1.0, 1.0, 1.0); //-virtual_gain;
		gain_vector_0.tail(3) = -0.50*Vector3d(1.0, 1.0, 1.0); //-0.50*virtual_gain;
        //
        legIK_Done   = false; 
        // change so that this portion is run only once when the function is called for the first time before the final output
        legIK_batch_start = true;

        // data logging  // temporary
		string path_log_joints_pos  = "log_virt_joints_pos1_.txt";
		string path_log_joints_vel  = "log_virt_joints_vel1_.txt";
		Out_joints_pos.open(path_log_joints_pos.c_str());
		Out_joints_vel.open(path_log_joints_vel.c_str()); 

        std::cout<< "INITIALIZE REACH : " << 0.0 << std::endl;
    }
    // =======================================================================================================================

    if(!ik_cont && legIK_batch_start)
    {
        l_count = 0;
        while(!n_loop && !ik_cont)
        {
            //========================================================================================================
            MatrixXd W_EdgesNormalCHull_; 
            VectorXd DistanceEdgesCHull_;
            this->solve_QPcvxgen(robot_model_, pose_lfoot, pose_rfoot, stanceFoot, W_EdgesNormalCHull_, DistanceEdgesCHull_);     // TODO PLOT EVOLUTION OF COM AND CLAMPI IT
            // clamping the CoM position to move freely but within the convex hull
            // --------------------------------------------------------------------
            des_X_EE[0] = Pose_EE[0];
			des_X_EE[1] = Pose_EE[1];
			des_X_EE[7] = Pose_EE[7];

	        // --------------------------------------------
            n_loop = (l_count >= l_count_max);  
            cnt    = (g_count >= count_max);
            ik_cont = cnt || get_legIKCompletionState();
            // --------------------------------------------

            std::cout<< "LEGIK BATCH LCOUNT : " << l_count << std::endl;
            std::cout<< "LEGIK BATCH GCOUNT : " << g_count << std::endl;

            // data logging 
			Out_joints_pos << g_count << "	" << 180./M_PI * virt_jts_pos.transpose() << std::endl;
			Out_joints_vel << g_count << "	" << q_dot_star.transpose() << std::endl;

            l_count ++; // local counter
            g_count ++; // global counter 
        }
    }

    if(ik_cont || cnt)
    {
        //
        des_jts_pos  = virt_jts_pos;
        des_WHB      = virt_WHB;
        //
        // reset_variables();
        legIK_Done        = true; 
        legIK_batch_start = false;
        //
        isFreeze = false;
    	freeze.setZero();

        // load the reachability parameters
        memcpy(task_weight, &task_weight_default[0], 8 * sizeof*task_weight_default);
        //
        gain_vector_0.head(3) = -Vector3d(virtual_gain, virtual_gain, virtual_gain); //-virtual_gain;
		gain_vector_0.tail(3) = -0.50*Vector3d(virtual_gain, virtual_gain, virtual_gain); //-0.50*virtual_gain;

        // cout << " Desired Joint position are : \n" << 180./M_PI * des_jts_pos.transpose() << endl;
        // cout << " Desired Joint position Torso are : \t" << 180./M_PI * des_jts_pos.head(3).transpose() << endl;
        // cout << " Desired Joint position Lhand are : \t" << 180./M_PI * des_jts_pos.head(10).tail(7).transpose() << endl;
        // cout << " Desired Joint position Rhand are : \t" << 180./M_PI * des_jts_pos.head(17).tail(7).transpose() << endl;
        // cout << " Desired Joint position Lleg  are : \t" << 180./M_PI * des_jts_pos.tail(12).head(6).transpose() << endl;
        // cout << " Desired Joint position Rleg  are : \t" << 180./M_PI * des_jts_pos.tail(6).transpose() << endl;
        // cout << " Desired Base transformation  is : \n" << des_WHB << endl;

        for (int i=2; i<5; i++)
            std::cout << " ERROR POS and ORI NORM " << EE[i] << "  : \t" << error_pos_norm[i] << "  and : " << error_ori_norm[i] << std::endl;

    }

    return true;
}

//
Vector3d wb_InverseKinematics::ConstrainCoM(Vector7d Pose_lfoot, Vector7d Pose_rfoot, Vector3d CoM)
{	
	//
	double t_wbIK = yarp::os::Time::now();
	//
	Vector3d d_CoM = CoM;
	// computation of the convex polygon
	MatrixXd W_EdgesNormalCHull; 
    VectorXd DistanceEdgesCHull;  
	this->getConvexHullVariables(this->PtsInFoot, Pose_lfoot, Pose_rfoot,  W_EdgesNormalCHull, DistanceEdgesCHull, W_Rot_AbsFoot, W_Pos_AbsFoot);
	//
	Matrix2d Hc = 0.5*MatrixXd::Identity(2, 2);
	Vector2d pc = -CoM.head(2);
	//
	MatrixXd NeJc = MatrixXd::Zero(6, 2);
	Vector6d deXc = VectorXd::Zero(6);
	//
	if(W_EdgesNormalCHull.rows() <= 5){
		int r = W_EdgesNormalCHull.rows();
		NeJc.topRows(r) = W_EdgesNormalCHull.topRows(r);
		deXc.head(r)    = 0.9*DistanceEdgesCHull.head(r) + W_EdgesNormalCHull.topRows(r)*(W_Pos_AbsFoot);
	}
	else{
		NeJc = W_EdgesNormalCHull.topRows(6);
		deXc = 0.9*DistanceEdgesCHull.head(6) + W_EdgesNormalCHull.topRows(6)*(W_Pos_AbsFoot);
	}
	// Hessian matrix
	memcpy(CoMclp_params.Q, Hc.data(), sizeof(double) * Hc.size());
	memcpy(CoMclp_params.P, &pc[0], sizeof(double)*(2)); 						// weight acceleration and desired posture
	// setting of hard constraints
	for(int i=0;i<6;i++) 
		memcpy(CoMclp_params.NeJc, NeJc.data(),	sizeof(double)*NeJc.size());	// convexHull constraints Matrix for CoM
	memcpy(CoMclp_params.deXc, &deXc[0], 		sizeof(double)*(6));			// convexHull constraints vector for CoM
	
	// Solve the QP (position)
	CoMclp_settings.verbose 	 = 0;
	// CoMclp_settings.eps 		 = 15e-5;
	// CoMclp_settings.resid_tol = 1e-4;
	int iter = CoMclp_solve();
	// Extract the QP solution (acceleration and slack)
	for(int i=0; i<2; i++) d_CoM(i) = CoMclp_vars.xc[i];

	std::cout<< "CONSTRAINTS COM SOLVED IN : " <<  yarp::os::Time::now()-t_wbIK << " s" << std::endl;
	// std::cout<< "CHECKING COM CLAMPING   : \n" <<  NeJc*d_CoM.head(2) - deXc << std::endl;

	return d_CoM;
}