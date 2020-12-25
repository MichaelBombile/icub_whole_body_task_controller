

#include "BimanualCoordinationTasks.h"

using namespace std;
using namespace Eigen;

double Thresolding(double min, double max, double value)
{
	// 
	double out; 
	if(value<=min){ out = min; }
	else if(value >= max){out = max;}
	else {out = value;}
	return out;
}

Vector6d SaturationTwist(double p_lim, double o_lim, Vector6d Velo)
{
	// 
	Eigen::Vector3d pos, orient;
	pos    = Velo.head<3>();
	orient = Velo.tail<3>();

	if((fabs(pos(0))>p_lim) || (fabs(pos(1))>p_lim) || (fabs(pos(2))>p_lim)){
		pos = p_lim * (1./pos.norm() * pos);
	}

	if((fabs(orient(0))>o_lim) || (fabs(orient(1))>o_lim) || (fabs(orient(2))>o_lim)){
		orient = o_lim * (1./orient.norm() * orient);
	}

	Vector6d out_velo;
	out_velo.head<3>() = pos;
	out_velo.tail<3>() = orient;
	return out_velo;
}


BimanualCoordinationTasks::BimanualCoordinationTasks()	: GraspHessianMatrix(42,42)
														, GraspGradientVector(42)
														, GraspEqualConstraint_matrix(24,42)
														, GraspEqualConstraint_vector(24)
														, GraspInequalConstraint_matrix(22,42)
														, GraspInequalConstraint_vector(22)
														, GrasplumpedConstraints_matrix(2*24+22,42)
														, GrasplumpedConstraints_vector(2*24+22) 
														, GraspMatrixHands(6,12)
														, Psd_GraspMatrixHands(12,6)
														, optimal_contact_wrench_hands(12)
														, optimal_slack(6) 
{
	// Reach to Grasp task Parameters
	for(int i=0; i<N_eef; i++)
	{
		// Homogeneous transformation
		w_H_absHands.setIdentity();
		w_H_v[i].setIdentity();
		w_H_vstar[i].setIdentity();
		w_H_hand[i].setIdentity();
		absHands_H_v[i].setZero();
		absHands_H_v[i](3,3) = 1.0;

		absHands_H_vstar_ini[i].setZero();
		absHands_H_vstar_ini[i](3,3) = 1.0;
		absHands_H_vstar_fin[i].setZero();
		absHands_H_vstar_fin[i](3,3) = 1.0;
		//
		w_H_vstar_t[i].setIdentity();
		// to center the grasp point
		Obj_H_absGpoints.setIdentity();
		w_H_absGpoints.setIdentity();
		absGpoints_H_cp[i].setZero();
		absGpoints_H_cp[i](3,3) = 1.0;
		//
		// for the release and retract motion
		w_H_absHands_des.setIdentity();				// desired absolute hands pose
		w_H_hand_des[i].setZero();					// desired hands poses (after release and retract)
		w_H_hand_des[i](3,3) = 1.0;
		w_H_vstar_des[i].setZero();				// desired virtual object poses (after release and retract)
		w_H_vstar_des[i](3,3) = 1.0;

		// error 6D
		error_approach[i].setZero();
		error_aperture[i].setZero();
		error_synchro[i].setZero();

		error_approach_ini[i].setZero();

		// Orientation Jacobian
		L_Mu_Theta_approach[i].setZero();
		L_Mu_Theta_aperture[i].setZero();
		L_Mu_Theta_synchro[i].setZero();

		// Interaction matrices
		L_eta_approach_in_w[i].setZero();
		L_eta_aperture_in_w[i].setZero();
		L_eta_synchro_in_w[i].setZero();

		// Velocity vectors 
		w_velocity_approach[i].setZero();
		w_velocity_aperture[i].setZero();
		w_velocity_synchro[i].setZero();
		w_velocity_ro_gpoint[i].setZero();
		w_velocity_uvo_gpoint[i].setZero(); 
		w_velocity_vo_gpoint[i].setZero();
		w_velocity_eef[i].setZero();

		// acceleration vectors 
		w_acceleration_approach[i].setZero();
		w_acceleration_aperture[i].setZero();
		w_acceleration_synchro[i].setZero();
		w_acceleration_ro_gpoint[i].setZero();
		w_acceleration_uvo_gpoint[i].setZero();
		w_acceleration_vo_gpoint[i].setZero();
		w_acceleration_eef[i].setZero();
	}

	// Initialization to zero of some variables
	// ----------------------------------------
	world_Xstar_deslhand.setZero();
	world_Xstar_desrhand.setZero();
	optimal_contact_wrench_hands.setZero();
	optimal_slack.setZero();
	GraspMatrixHands.setZero();
	// ---------------------------------
	optimal_hands_velocity.setZero();
	optimal_hands_acceleration.setZero();

	Psd_GraspMatrixHands.setZero();
	GraspMotion_Jacobian.setZero();
	GraspConstraint_Jacobian.setZero();
	dotGraspMotion_Jacobian.setZero();
	dotGraspConstraint_Jacobian.setZero();
	GraspMotion_Jacobian_prev.setZero();
	GraspConstraint_Jacobian_prev.setZero();
	dotGraspMotion_Jacobian_Xhdot.setZero();
	dotGraspConstraint_Jacobian_Xhdot.setZero();

	b_absolute_motion.setZero();
	b_relative_motion.setZero();
		//
	force_correction_lh.setZero();
	force_correction_rh.setZero(); 

}

BimanualCoordinationTasks::~BimanualCoordinationTasks(){}



BimanualGains  bimanip_gains;
BimanualPoses  bimanip_pose;
GraspLocations GraspPoints;
Matrix3d 	   ObjectScaling;
W_Pose_Object_;

ctrl_param.bimanip_gains.synchro
//
Obj_H_absGpoints;
w_H_absGpoints;
absGpoints_H_cp;
w_H_hand;
w_H_absHands;

BimanualGains bimanip_gains;
double beta_;
//
void BimanualTasks::InitializeReach2GraspTask(	  double dt,
												Vector7d Obj_Pose_cp_[],
												Vector7d w_Pose_Obj_,
												Vector7d w_Pose_lhand,
												Vector7d w_Pose_rhand)
{
	

	gain_approach = gain_approach_;		// gain of the approach task
	gain_aperture = gain_aperture_;		// gain of the pregrasp task (from scaled to unscaled virtual object)
	gain_synchro  = gain_synchro_; 		// gaon of the grasp task (robot's end-effector to grasp point)
	// Non-zero initialization
	// -----------------------
	tol_dist2contact = tol_dist2contact_;
	//
	Weight_hands_wrench_ = Weight_hands_wrench;
	Weight_hands_slack_  = Weight_hands_slack;
	//
	Weight_absolute_motion_ = Weight_absolute_motion;
	Weight_relative_motion_ = Weight_relative_motion;
	weight_regularizaion    = weight_regularizaion;

	// /////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 										Reach to grasp task variables
	// /////////////////////////////////////////////////////////////////////////////////////////////////////////			

	// Transformations related to Grasping points
	// ==========================================
	// World homogeneous transformation of the grasp configurations absolute frame : w_H_absGpoints
	// ----------------------------------------------------------------------------
	Obj_H_absGpoints.block<3,3>(0,0) = Eigen::MatrixXd::Identity(3,3);   //
	for(int i=0; i<N_eef; i++) 
		Obj_H_absGpoints.block<3,1>(0,3) += (double) (1./N_eef) * Obj_Pose_cp_[i].head<3>();
	//
	// Homogeneous transformation of object frame in world
	Matrix4d w_H_Obj = Transforms.PoseVector2HomogenousMx(w_Pose_Obj_); 	
	w_H_absGpoints = w_H_Obj * Obj_H_absGpoints;
	// world transformation of grasp points
	for(int i=0; i<N_eef; i++) 
		w_H_cp[i]	  = w_H_Obj * Transforms.PoseVector2HomogenousMx(Obj_Pose_cp_[i]);

	// homogeneous transformation of grasp point in the absolute grasp point frame
	// ---------------------------------------------------------------------------
	for(int i=0; i<N_eef; i++) 
		absGpoints_H_cp[i] = Obj_H_absGpoints.inverse() * Transforms.PoseVector2HomogenousMx(Obj_Pose_cp_[i]);

	// Transformations related to robot Hands
	// ======================================
	// world pose of the hands [0]: left and [1]: right 
	// for(int i=0; i<N_eef; i++) 
	w_H_hand[0] = Transforms.PoseVector2HomogenousMx(w_Pose_lhand);
	w_H_hand[1] = Transforms.PoseVector2HomogenousMx(w_Pose_rhand);
	// World homogeneous transformation of the hands absolute frame : w_H_absHands
	// ----------------------------------------------------------------------------
	w_H_absHands.block<3,3>(0,0) = w_H_Obj.block<3,3>(0,0);
	for(int i=0; i<N_eef; i++) 
		w_H_absHands.block<3,1>(0,3) += (double) (1./N_eef) * w_H_hand[i].block<3,1>(0,3);
	
	// Transformations related to the Virtual object
	// =============================================
	Matrix4d absHands_H_vstar[N_eef];	// grasp point in the unscaled virtual object wrt. the absolute hands frame
	for(int i=0; i<N_eef; i++)	
	{
		// Unscaled virtual object
		absHands_H_vstar[i] = absGpoints_H_cp[i];
		// world transformation
		w_H_vstar[i]        = w_H_absHands * absHands_H_vstar[i];
		
		// scaled virtual object
		absHands_H_v[i].block<3,3>(0,0) = absHands_H_vstar[i].block<3,3>(0,0);
		absHands_H_v[i].block<3,1>(0,3) = ctrl_param.ObjectScaling * absHands_H_vstar[i].block<3,1>(0,3);
		// world transformation
		w_H_v[i] 	                    = w_H_absHands * absHands_H_v[i];
		
		// For transition via resting configuration
		absHands_H_vstar_ini[i] = absHands_H_v[i];
		absHands_H_vstar_fin[i] = absGpoints_H_cp[i];		
	}

	// Motion phases Low-pass Filters
	// ===============================
	Vector6d init_velo; init_velo.setZero();
	//
	for(int i=0; i<N_eef; i++)
	{
		Filter_approach[i].InitializeFilter(dt, 10.0, 10.0, init_velo);
		Filter_aperture[i].InitializeFilter(dt, 10.0, 10.0, init_velo);
		Filter_synchro[i].InitializeFilter(dt, 10.0, 10.0, init_velo);
	}

	//
	gamma_reachable_p = 0.0;
	gamma_reachable_o = 0.0;
	// TO BE SET RELATIVE TO THE STANCE FOOT FRAME
	// Rotation Matrices
	w_H_hand_des[0].block<3,3>(0,0) << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0;
	w_H_hand_des[1].block<3,3>(0,0) << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0;
	// translatiion
	w_H_hand_des[0].block<3,1>(0,3) << 0.18,  0.17, 0.67;
	w_H_hand_des[1].block<3,1>(0,3) << 0.18, -0.17, 0.67;

	// /////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 							     Cooperative manipulation task variables
	// /////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Initialization of the Grasp constraint class
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	GraspConstraints.Initialize(ctrl_param.Hands_contact_param); 
	GraspConstraints.computeBimanualGraspMatrix(w_Pose_Obj_, w_Pose_lhand, w_Pose_rhand, GraspMatrixHands);
	// motion Jacobian
	GraspMotion_Jacobian = Psd_GraspMatrixHands.transpose();
	// Constraint Jacobian
	GraspConstraint_Jacobian      = Eigen::MatrixXd::Identity(12,12) - GraspMatrixHands.transpose() * Psd_GraspMatrixHands.transpose();
	//
	GraspMotion_Jacobian_prev     = GraspMotion_Jacobian;
	GraspConstraint_Jacobian_prev = GraspConstraint_Jacobian;

	// Instatiation of the object class
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	ContactConfidence = 0.0;
	dist2contact_lh = 0.1;  // something different than 0 
	dist2contact_rh = 0.1;

	// initialization of the cvxgen solver for the cooperative manipulation
	bwc_set_defaults();
	bwc_setup_indexing();
	//
	cmo_set_defaults();
	cmo_setup_indexing();

}




// ////////////////////////////////////////////////////////////////////////////////////////////////////
//  								REACHING AND GRASPING TASKS
// ////////////////////////////////////////////////////////////////////////////////////////////////////
TaskSpaceTrajectories  StatesObject_;
// 
Eigen::Vector3d hand_offset[N_eef];
hand_offset[0].setZero();	hand_offset[0](2) = -0.025;
hand_offset[1].setZero();	hand_offset[1](2) =  0.025;
// 
// velocity saturation
Vector6d max_reaching_velocity;
max_reaching_velocity << 0.7, 0.7, 0.7,  2.0, 2.0, 2.0;
double alpha_manip_filter = 5.0;


bool BimanualFreeMotionController::compute_Reach2GraspMotion(	Vector7d 				Obj_Pose_cp[],
												TaskSpaceTrajectories  	StatesObject_,
												Vector7d 				w_Pose_lhand,
												Vector7d 				w_Pose_rhand,
												  double 				dt)
														
{

	// world pose of the hand
	// =======================
	w_H_hand[0] = Transforms.PoseVector2HomogenousMx(w_Pose_lhand);				// left
	w_H_hand[1] = Transforms.PoseVector2HomogenousMx(w_Pose_rhand);				// right
	// Offset coorection
	w_H_hand[0].block<3,1>(0,3) +=  w_H_hand[0].block<3,3>(0,0)*ctrl_param.hand_offset[0];
	w_H_hand[1].block<3,1>(0,3) +=  w_H_hand[1].block<3,3>(0,0)*ctrl_param.hand_offset[1];

	//velocity of grasp points on the object
	// =====================================
	Matrix6d velocity_Twist_Mx;		velocity_Twist_Mx.setIdentity();
	Eigen::Vector3d t[N_eef];
	Matrix4d Obj_H_gpoint[N_eef];

	for(int i=0; i<N_eef; i++)
	{
		t[i] = Transforms.PoseVector2HomogenousMx(Obj_Pose_cp[i]).block<3,1>(0,3);
		Matrix3d skew_Mx_gpt << 	      0.0,  -t[i](2),  	 t[i](1),
					                  t[i](2),       0.0,  	-t[i](0),
					                 -t[i](1),   t[i](0),        0.0;
		//			                 
		velocity_Twist_Mx.block<3,3>(0,3) = -skew_Mx_gpt;
		// velocity
		w_velocity_ro_gpoint[i] = velocity_Twist_Mx * StatesObject_.velocity*1.0;
		// acceleration
		w_acceleration_ro_gpoint[i] = StatesObject_.acceleration;
		w_acceleration_ro_gpoint[i].head<3>() += StatesObject_.velocity.tail<3>().cross(StatesObject_.velocity.tail<3>().cross(t[i]));
	}

	// =======================================================================================================================
	// Synchronization phase (robot's end-effectors towards grasp point on scaled virtual object)
	// =======================================================================================================================
	Vector6d w_velocity_synchro_0[N_eef];
	//	
	for(int i=0; i<N_eef; i++)
	{	
		//
		w_H_hand_t[i].block<3,1>(0,3) = (1.0-gamma_reachable_p)*w_H_hand[i].block<3,1>(0,3) + gamma_reachable_p *w_H_v[i].block<3,1>(0,3);
		w_H_hand_t[i].block<3,3>(0,0) = this->getCombinedRotationMatrix(gamma_reachable_o, w_H_hand[i].block<3,3>(0,0), w_H_v[i].block<3,3>(0,0));

		// get the 6D servoing variables
		compute_6Dservoing_variables(w_H_hand[i],			// origin
									 w_H_hand_t, 			//w_H_v[i],	// destination
									 error_synchro[i],
									 L_Mu_Theta_synchro[i],
									 L_eta_synchro_in_w[i]);
		// compute the velocity
		w_velocity_synchro[i] = -L_eta_synchro_in_w[i].inverse() * ctrl_param.bimanip_gains.synchro.asDiagonal()* error_synchro[i];
		// apply saturation to the computed value
		w_velocity_synchro[i] = SaturationTwist(max_reaching_velocity(0), max_reaching_velocity(3), w_velocity_synchro[i]);
		
		// computation of the acceleration through filtered derivative
		// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		// filter grasp
		Filter_synchro[i].pole = Filter_synchro[i].gain = ctrl_param.bimanip_gains.synchro(0) * ctrl_param.alpha_manip_filter;

		w_velocity_synchro_0[i]   = w_velocity_synchro[i];
		w_velocity_synchro[i]     = Filter_synchro[i].getEulerIntegral(w_velocity_synchro[i]);
		w_acceleration_synchro[i] = Filter_synchro[i].pole * (w_velocity_synchro_0[i] - w_velocity_synchro[i]);
	}

	// =======================================================================================================================
	// Approach phase (absolute hands frame towards the absolute frame of the grasp configurations)
	// =======================================================================================================================
	double gamma_synchro; 		gamma_synchro = 0.0;
	double psd_time_synchro;
	//
	// Vector3d error_h_v = 0.5*(error_synchro[0].head<3>() + error_synchro[1].head<3>());
	Vector6d error_h_v = 0.5*(error_synchro[0] + error_synchro[1]);
	psd_time_coord     = 1.0/(50.*error_h_v.norm()+1e-15);
	gamma_synchro      = 1.0 - exp(-psd_time_synchro/0.04); //0.06

	// get the 6D servoing variables of the (abs towards obj)
	Matrix4d w_H_Obj = Transforms.PoseVector2HomogenousMx(StatesObject_.pose); 
	w_H_absGpoints = w_H_Obj * Obj_H_absGpoints; 

	// desired absolute hands frame
	this->get_absolute_hands_frame(w_H_Obj, w_H_hand_des, w_H_absHands_des);
	// assign the same orienatation as the world
	w_H_absHands_des.block<3,3>(0,0) = Eigen::MatrixXd::Identity(3,3);

	// get the frame interpolation
	Matrix4d w_H_absHands_t; 	w_H_absHands_t.setIdentity();
	w_H_absHands_t.block<3,1>(0,3) = (1.0-gamma_synchro)*w_H_absHands_des.block<3,1>(0,3) + gamma_synchro*w_H_absGpoints.block<3,1>(0,3);
	w_H_absHands_t.block<3,3>(0,0) = this->getCombinedRotationMatrix(gamma_synchro, w_H_absHands_des.block<3,3>(0,0), w_H_absGpoints.block<3,3>(0,0));
	//
	Vector6d w_velocity_approach_0[N_eef];

	for(int i=0; i<N_eef; i++)
	{	
		// get the 6D servoing variables
		this->compute_6Dservoing_variables(	w_H_absHands,					// origin
											w_H_absHands_t,				// destination (target : w_H_absGpoints)
											error_approach[i],
											L_Mu_Theta_approach[i],
											L_eta_approach_in_w[i]);
		// compute the velocity
		w_velocity_approach[i] = -L_eta_approach_in_w[i].inverse() * ctrl_param.bimanip_gains.approach.asDiagonal()* error_approach[i];
		// apply saturation to the computed value
		w_velocity_approach[i] = SaturationTwist(max_reaching_velocity(0), max_reaching_velocity(3), w_velocity_approach[i]);
		
		// computation of the acceleration through filtered derivative
		// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		// filter grasp
		Filter_approach[i].pole    = Filter_approach[i].gain = ctrl_param.bimanip_gains.approach(0) * ctrl_param.alpha_manip_filter;

		w_velocity_approach_0[i]   = w_velocity_approach[i];
		w_velocity_approach[i]     = Filter_approach[i].getEulerIntegral(w_velocity_approach[i]);
		w_acceleration_approach[i] = Filter_approach[i].pole * (w_velocity_approach_0[i] - w_velocity_approach[i]);
	}

	// =======================================================================================================================
	// Grasping phase (shrinking of the virtual object: closing of the hands aperture) (v towards vstar)
	// =======================================================================================================================
	// Absolute hands frame
	this->get_absolute_hands_frame(w_H_Obj, w_H_hand, w_H_absHands);
	// update of the absolute frame rotation matrix
	w_H_absHands.block<3,3>(0,0) = w_H_absHands_t.block<3,3>(0,0);
	//
	double gamma_approach = 0.0; 
	double psd_time_approach;
	//
	Vector6d w_velocity_aperture_0[N_eef];
	Eigen::Vector3d error_reach[N_eef];

	for(int i=0; i<N_eef; i++)
	{	
		// 
		// error_reach[i] = w_H_cp[i].block<3,1>(0,3) - w_H_vstar[i].block<3,1>(0,3);
		psd_time_approach = 1.0/(50.*error_approach[i].head<3>().norm()+1e-15);
		gamma_approach    = 1.0 - exp(-pow(psd_time_approach,2.0)/0.1); 
		// (virtual grasp point of scaled virtual object towards grasp point on the unscaled virtual object)
		w_H_vstar_t[i]    = w_H_absHands * ((1.0-gamma_approach)*absE_H_vstar_ini[i] + gamma_approach*absE_H_vstar_fin[i]);  //	Obj_H_gpoint[i];

		// get the 6D servoing variables
		this->compute_6Dservoing_variables(	w_H_v[i],
											w_H_vstar_t[i],   				//	w_H_vstar_t[i],
											error_aperture[i],
											L_Mu_Theta_aperture[i],
											L_eta_aperture_in_w[i]);
		// compute the velocity
		w_velocity_aperture[i] = -L_eta_aperture_in_w[i].inverse() * ctrl_param.bimanip_gains.aperture.asDiagonal()* error_aperture[i];
		// apply saturation to the computed value
		w_velocity_aperture[i] = SaturationTwist(max_reaching_velocity(0), max_reaching_velocity(3), w_velocity_aperture[i]);
		
		// computation of the acceleration through filtered derivative
		// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		// filter grasp
		Filter_aperture[i].pole    = Filter_aperture[i].gain = ctrl_param.bimanip_gains.aperture(0) * ctrl_param.alpha_manip_filter;

		w_velocity_aperture_0[i]   = w_velocity_aperture[i];
		w_velocity_aperture[i]     = Filter_aperture[i].getEulerIntegral(w_velocity_aperture[i]);
		w_acceleration_aperture[i] = Filter_aperture[i].pole * (w_velocity_aperture_0[i] - w_velocity_aperture[i]);
	}

	// =======================================================================================================================
	// Computation of world velocity and acceleration of desired velocity and acceleration of frames of interest
	// =======================================================================================================================
		
	for(int i=0; i<N_eef; i++)
	{
		// velocity and acceleration of unscaled virtual object
		w_velocity_uvo_gpoint[i]     = w_velocity_approach[i]     + w_velocity_ro_gpoint[i];
		w_acceleration_uvo_gpoint[i] = w_acceleration_approach[i] + w_acceleration_ro_gpoint[i];
		
		// velocity and acceleration of virtual object
		w_velocity_vo_gpoint[i]   	 = w_velocity_aperture[i]     + w_velocity_uvo_gpoint[i];
		w_acceleration_vo_gpoint[i]	 = w_acceleration_aperture[i] + w_acceleration_uvo_gpoint[i];
		
		// Desired velocity and acceleration of the robot's hands (end-effectors)
		// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
		w_velocity_eef[i] 			 = w_velocity_synchro[i] 	  + w_velocity_vo_gpoint[i];
		w_acceleration_eef[i] 		 = w_acceleration_synchro[i]  + w_acceleration_vo_gpoint[i];
	}

	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// Update the pose of the virtual object relative to the world frame  w_H_v
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	Matrix6d vstar_6x6DR_w[N_eef];  
	Matrix4d vstar_H_v[N_eef];
	
	for(int i=0; i<N_eef; i++)
	{
		vstar_H_v[i] = w_H_vstar[i].inverse() * w_H_v[i];
		vstar_6x6DR_w[i].setZero();
		vstar_6x6DR_w[i].block<3,3>(0,0) = w_H_vstar[i].block<3,3>(0,0).transpose();
		vstar_6x6DR_w[i].block<3,3>(3,3) = vstar_6x6DR_w[i].block<3,3>(0,0);

		Transforms.UpdatePose_From_VelocityTwist(dt, vstar_6x6DR_w[i]*w_velocity_aperture[i], vstar_H_v[i]);
		w_H_v[i] = w_H_vstar[i] * vstar_H_v[i];
	}

}

// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  								RELEASING AND RETRACTING TASKS
// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool BimanualFreeMotionController::Release_and_Retract(Vector7d 				Obj_Pose_cp[],
														TaskSpaceTrajectories  	StatesObject_,
														Vector7d 				w_Pose_lhand,
														Vector7d 				w_Pose_rhand,
														  double 				dt)
{
	//
	// get the 6D servoing variables of the (abs towards obj)
	Matrix4d w_H_Obj = Transforms.PoseVector2HomogenousMx(w_Pose_Obj); 
	
	// world pose of the hand
	w_H_hand[0] = Transforms.PoseVector2HomogenousMx(w_Pose_lhand);
	w_H_hand[1] = Transforms.PoseVector2HomogenousMx(w_Pose_rhand);

	// absolute hands frame
	this->get_absolute_hands_frame(w_H_Obj, w_H_hand, w_H_absHands);
	// desired absolute hands frame
	this->get_absolute_hands_frame(w_H_Obj, w_H_hand_des, w_H_absHands_des);

	//
	for(int i=0; i<Ne; i++)
	{
		w_H_vstar[i] = w_H_absHands * absGpoints_H_cp[i];
		w_H_cp[i] 	 = w_H_Obj  * Transforms.PoseVector2HomogenousMx(Obj_Pose_cp[i]);
	}

	// release motion of the object: expension of the virtual object  (v towards vstar)
	//=================================================================================
	// (virtual grasp point of scaled virtual object towards
	// grasp point on the unscaled virtual object)
	//
	Vector6d w_velocity_aperture_0[Ne];
	//
	for(int i=0; i<Ne; i++)
	{
		//
		// Hand aperture closing (through coupling with Approach DS)
		w_H_vstar_t[i] = w_H_absE * absE_H_vstar_ini[i];
		// get the 6D servoing variables
		this->compute_6Dservoing_variables(	w_H_v[i],				// origin
											w_H_vstar_t[i],			// target (desired)
											error_aperture[i],
											L_Mu_Theta_aperture[i],
											L_eta_aperture_in_w[i]);

		// compute the velocity
		w_velocity_aperture[i] = -L_eta_aperture_in_w[i].inverse() * ctrl_param.bimanip_gains.aperture.asDiagonal()* error_aperture[i];
		// apply saturation to the computed value
		w_velocity_aperture[i] = SaturationTwist(max_reaching_velocity(0), max_reaching_velocity(3), w_velocity_aperture[i]);
		
		// computation of the acceleration through filtered derivative
		// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		// filter grasp
		Filter_aperture[i].pole    = Filter_aperture[i].gain = ctrl_param.bimanip_gains.aperture(0) * ctrl_param.alpha_manip_filter;

		w_velocity_aperture_0[i]   = w_velocity_aperture[i];
		w_velocity_aperture[i]     = Filter_aperture[i].getEulerIntegral(w_velocity_aperture[i]);
		w_acceleration_aperture[i] = Filter_aperture[i].pole * (w_velocity_aperture_0[i] - w_velocity_aperture[i]);
	}

	// -----------------------------------------------------------------------------------------
	// Approach task (vstar towards o_i)
	// ==================================
	double gamma_app    = 0.0;
	double psd_time_app = 0.0;	
	//
	Vector6d w_velocity_approach_0[Ne];
	//
	Matrix4d w_H_absHands_t, absHands_H_vstar_des[N_eef];			
	w_H_absHands_t.setIdentity();
	w_H_absHands_t.block<3,3>(0,0) = w_H_Obj.block<3,3>(0,0);
	//
	for(int i=0; i<Ne; i++)
	{
		//
		psd_time_app = 1.0/(50.*error_aperture[i].head<3>().norm()+1e-10);
		gamma_app    = 1.0 - exp(-pow(psd_time_app, 2.5)/0.05); //3.0/0.1
		// ------
		// retract motion (through coupling with aperture DS)
		w_H_absHands_t.block<3,1>(0,3) = (1.0-gamma_app)*w_H_absHands.block<3,1>(0,3) + gamma_app*w_H_absHands_des.block<3,1>(0,3);
		// desired virtual frame after release
		absE_H_vstar_des[i] = absE_H_vstar_ini[i];
		absE_H_vstar_des[i].block<3,1>(0,3) *=0.4; 
		w_H_vstar_des[i] = w_H_absHands_t * absGpoints_H_cp[i]; 				//Obj_H_gpoint[i]; //absE_H_vstar_des[i];
		//
		this->compute_6Dservoing_variables( w_H_vstar[i],						//	origin
											w_H_vstar_des[i],      				//	target (desired)  w_H_cp[i],
											error_approach[i],
											L_Mu_Theta_approach[i],
											L_eta_approach_in_w[i]);
		// compute the velocity
		w_velocity_approach[i] = -L_eta_approach_in_w[i].inverse() * ctrl_param.bimanip_gains.approach.asDiagonal()* error_approach[i];
		// apply saturation to the computed value
		w_velocity_approach[i] = SaturationTwist(max_reaching_velocity(0), max_reaching_velocity(3), w_velocity_approach[i]);
		
		// computation of the acceleration through filtered derivative
		// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		// filter approach
		Filter_approach[i].pole    = Filter_approach[i].gain = ctrl_param.bimanip_gains.approach(0) * ctrl_param.alpha_manip_filter;

		w_velocity_approach_0[i]   = w_velocity_approach[i];
		w_velocity_approach[i]     = Filter_approach[i].getEulerIntegral(w_velocity_approach[i]);
		w_acceleration_approach[i] = Filter_approach[i].pole * (w_velocity_approach_0[i] - w_velocity_approach[i]);
	}

	// -------------------------------------------------------------------------------------------
	// Synchronisation of the arms task (h towards v)
	//================================================
	// robot's end-effectors towards grasp point on scaled virtual object
	// -------------------------------------------------------------------
	double gamma_synchro    = 0.0;
	double psd_time_synchro = 0.0;	
	//
	Matrix4d w_H_hd_t[Ne];
	Vector6d w_velocity_synchro_0[Ne];
	//
	for(int i=0; i<Ne; i++)
	{
		//
		w_H_hd_t[i].setIdentity();

		psd_time_synchro = 1.0/(50.*error_approach[i].head<3>().norm()+1e-10);
		gamma_synchro    = 1.0 - exp(-pow(psd_time_synchro, 2.5)/0.05); //3.0/0.1
		//
		w_H_hd_t[i].block<3,1>(0,3) = (1. - gamma_synchro)*w_H_v[i].block<3,1>(0,3) + gamma_synchro*w_H_hand_des[i].block<3,1>(0,3);
		w_H_hd_t[i].block<3,3>(0,0) = this->getCombinedRotationMatrix(gamma_synchro, w_H_hand_des[i].block<3,3>(0,0), w_H_v[i].block<3,3>(0,0));
	
		// get the 6D servoing variables
		this->compute_6Dservoing_variables(	w_H_hand[i],					// current
											w_H_hd_t[i],					// desired w_H_v[i]
											error_synchro[i],
											L_Mu_Theta_synchro[i],
											L_eta_synchro_in_w[i]);
		// compute the velocity
		w_velocity_synchro[i] = -L_eta_synchro_in_w[i].inverse() * ctrl_param.bimanip_gains.synchro.asDiagonal()* error_synchro[i];
		// apply saturation to the computed value
		w_velocity_synchro[i] = SaturationTwist(max_reaching_velocity(0), max_reaching_velocity(3), w_velocity_synchro[i]);
		
		// computation of the acceleration through filtered derivative
		// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		// filter grasp
		Filter_synchro[i].pole = Filter_synchro[i].gain = ctrl_param.bimanip_gains.synchro(0) * ctrl_param.alpha_manip_filter;

		w_velocity_synchro_0[i]   = w_velocity_synchro[i];
		w_velocity_synchro[i]     = Filter_synchro[i].getEulerIntegral(w_velocity_synchro[i]);
		w_acceleration_synchro[i] = Filter_synchro[i].pole * (w_velocity_synchro_0[i] - w_velocity_synchro[i]);
	}

	// ----------------------------------------------------------------------------------------
	//velocity of grasp points on the object
	// =====================================
	Matrix6d velocity_Twist_Mx;		velocity_Twist_Mx.setIdentity();
	Eigen::Vector3d t[N_eef];
	Matrix4d Obj_H_gpoint[N_eef];

	for(int i=0; i<N_eef; i++)
	{
		t[i] = Transforms.PoseVector2HomogenousMx(Obj_Pose_cp[i]).block<3,1>(0,3);
		Matrix3d skew_Mx_gpt << 	      0.0,  -t[i](2),  	 t[i](1),
					                  t[i](2),       0.0,  	-t[i](0),
					                 -t[i](1),   t[i](0),        0.0;
		//			                 
		velocity_Twist_Mx.block<3,3>(0,3) = -skew_Mx_gpt;
		// velocity
		w_velocity_ro_gpoint[i] = velocity_Twist_Mx * StatesObject_.velocity*1.0;
		// acceleration
		w_acceleration_ro_gpoint[i] = StatesObject_.acceleration;
		w_acceleration_ro_gpoint[i].head<3>() += StatesObject_.velocity.tail<3>().cross(StatesObject_.velocity.tail<3>().cross(t[i]));
	}
	
	
	// =======================================================================================================================
	// Computation of world velocity and acceleration of desired velocity and acceleration of frames of interest
	// =======================================================================================================================
		
	for(int i=0; i<N_eef; i++)
	{
		// velocity and acceleration of unscaled virtual object
		w_velocity_uvo_gpoint[i]     = w_velocity_approach[i]     + w_velocity_ro_gpoint[i];
		w_acceleration_uvo_gpoint[i] = w_acceleration_approach[i] + w_acceleration_ro_gpoint[i];
		
		// velocity and acceleration of virtual object
		w_velocity_vo_gpoint[i]   	 = w_velocity_aperture[i]     + w_velocity_uvo_gpoint[i];
		w_acceleration_vo_gpoint[i]	 = w_acceleration_aperture[i] + w_acceleration_uvo_gpoint[i];
		
		// Desired velocity and acceleration of the robot's hands (end-effectors)
		// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
		w_velocity_eef[i] 			 = w_velocity_synchro[i] 	  + w_velocity_vo_gpoint[i];
		w_acceleration_eef[i] 		 = w_acceleration_synchro[i]  + w_acceleration_vo_gpoint[i];
	}
	
	//
	Matrix6d vstar_6x6DR_w[Ne];  
	Matrix4d vstar_H_v[Ne];
	//--------------------------------------------
	Matrix6d vstar_invInteractionMx_aperture[Ne];
	Vector6d vstar_velocity_aperture[Ne];
	
	for(int i=0; i<Ne; i++)
	{
		// update the pose of the virtual object relative to the world frame  w_H_v
		vstar_H_v[i] = w_H_vstar[i].inverse() * w_H_v[i];
		vstar_6x6DR_w[i].setZero();
		vstar_6x6DR_w[i].block<3,3>(0,0) = w_H_vstar[i].block<3,3>(0,0).transpose();
		vstar_6x6DR_w[i].block<3,3>(3,3) = vstar_6x6DR_w[i].block<3,3>(0,0);

		// -------------------------------------------------------------------------------------------------
		Transforms.UpdatePose_From_VelocityTwist(dt, vstar_6x6DR_w[i]*w_velocity_aperture[i], vstar_H_v[i]);
		w_H_v[i] = w_H_vstar[i] * vstar_H_v[i];

	}

}


// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Vector of feature error
Vector6d BimanualCoordinationTasks::get_PoseError_cur2des(Matrix4d d_H_c)
{	
    // Pass 
    Eigen::VectorXd d_eta_c(6);
    d_eta_c.segment(0,3) << d_H_c(0,3), d_H_c(1,3), d_H_c(2,3);
    // extracrion of the rotation
    Eigen::AngleAxisd d_AxisAngle_c(d_H_c.block<3,3>(0,0));
    Eigen::Vector3d d_Axis_c = d_AxisAngle_c.axis();
    d_eta_c(3) = d_Axis_c(0) * d_AxisAngle_c.angle();
    d_eta_c(4) = d_Axis_c(1) * d_AxisAngle_c.angle();
    d_eta_c(5) = d_Axis_c(2) * d_AxisAngle_c.angle();

    return d_eta_c;
}

Eigen::Matrix3d BimanualCoordinationTasks::get_L_Mu_Theta_Matrix(Matrix3d d_R_c)
{
	// extracrion of the rotation
    Eigen::AngleAxisd d_AxisAngle_c(d_R_c);
    // function sinc(theta) and sinc(theta/2)
    double sinc_theta, sinc_theta_2;
    sinc_theta   = sin(d_AxisAngle_c.angle() + 1e-6)/(d_AxisAngle_c.angle() + 1e-6);
    sinc_theta_2 = sin((d_AxisAngle_c.angle() + 1e-6)/2.)/((d_AxisAngle_c.angle() + 1e-6)/2.);
    //
    Eigen::Vector3d d_Axis_c = d_AxisAngle_c.axis();
    Eigen::Matrix3d Skew_Mu; 	Skew_Mu.setZero(3,3);
    //
    Skew_Mu << 		     0.0, 	-d_Axis_c(2), 	 d_Axis_c(1),
    			 d_Axis_c(2),		     0.0,	-d_Axis_c(0),
    			-d_Axis_c(1),	 d_Axis_c(0),		     0.0;

    // Jacobian of the rotation
    Eigen::Matrix3d L_Mu_Theta;
    L_Mu_Theta.setIdentity(3,3);
    L_Mu_Theta = L_Mu_Theta - (d_AxisAngle_c.angle()/2.)* Skew_Mu + (1.-(sinc_theta/pow(sinc_theta_2, 2.))) * Skew_Mu * Skew_Mu;

	return L_Mu_Theta;
}
//
Matrix6d BimanualCoordinationTasks::getInteractionMxForAxisAngle(MatrixXd d_H_c)
{
    // Jacobian associated with the configuration error
    MatrixXd Jac_Mu_Theta;	Jac_Mu_Theta.resize(6,6);	Jac_Mu_Theta.setZero(6,6);
    // Jacobian of the rotation
    Matrix3d L_Mu_Theta = this->get_L_Mu_Theta_Matrix(d_H_c.block<3,3>(0,0));        
    // Building the overall jacobian
    Matrix6d InteractionMx_Mu_Theta; InteractionMx_Mu_Theta.setZero();
    InteractionMx_Mu_Theta.block<3,3>(0,0) = d_H_c.block<3,3>(0,0);
    InteractionMx_Mu_Theta.block<3,3>(3,3) = L_Mu_Theta;
    //
    return InteractionMx_Mu_Theta;
}
//
void BimanualCoordinationTasks::compute_6Dservoing_variables(Matrix4d w_H_c,
															  Matrix4d w_H_d,
															  Vector6d &error_c_in_d,
															  Matrix3d &L_Mu_Theta_cd,
															  Matrix6d &L_eta_cd_in_w)
{
	// transformation from current to desired d_H_d
	Matrix4d d_H_c = w_H_d.inverse() * w_H_c;
	//
	error_c_in_d = this->get_PoseError_cur2des(d_H_c);

	// 3D Orientation Jacobian 
	L_Mu_Theta_cd = this->get_L_Mu_Theta_Matrix(d_H_c.block<3,3>(0,0));

	// 6D Interaction matrix of the task
	L_eta_cd_in_w.setZero();
	L_eta_cd_in_w.block<3,3>(0,0) = w_H_d.block<3,3>(0,0).transpose();
	L_eta_cd_in_w.block<3,3>(3,3) = L_Mu_Theta_cd * w_H_c.block<3,3>(0,0).transpose();

}


//
Eigen::Matrix3d BimanualCoordinationTasks::getCombinedRotationMatrix(double Weight, Eigen::Matrix3d w_R_c, Eigen::Matrix3d w_R_d)
{
    // 
    Eigen::Quaterniond qc(w_R_c);             // current 
    Eigen::Quaterniond qd(w_R_d);             // desired 
    //
    Eigen::Quaterniond q_t = qc.slerp(Weight, qd);
    //
    Eigen::Matrix3d w_R_cd_t = q_t.toRotationMatrix();
    
    return q_t.toRotationMatrix();
}