
#include "BimanualFreeMotionController.h"

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

BimanualFreeMotionController::BimanualFreeMotionController(ControllerParameters&  	ctrl_param_): ctrl_param(ctrl_param_)
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
		absHands_H_vstar_ini_R[i].setZero();
		absHands_H_vstar_ini_R[i](3,3) = 1.0;
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
		w_H_vstar_des[i].setZero();					// desired virtual object poses (after release and retract)
		w_H_vstar_des[i](3,3) = 1.0;

		// error 6D
		error_approach[i].setZero();
		error_aperture[i].setZero();
		error_synchro[i].setZero();

		// error_approach_ini[i].setZero();

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

		//	
		w_velocity_abs.setZero();
		w_velocity_rel.setZero();
		w_acceleration_abs.setZero();
		w_acceleration_rel.setZero();
		w_velocity_abs_0.setZero();
		w_velocity_rel_0.setZero();
		error_abs.setZero();
		error_rel.setZero();
	}

}

BimanualFreeMotionController::~BimanualFreeMotionController(){}


//
// void BimanualFreeMotionController::InitializeReach2GraspTask(	  double period_,
// 																Vector7d Obj_Pose_cp_[],
// 																Vector7d w_Pose_Obj_,
// 																Vector7d w_Pose_lhand,
// 																Vector7d w_Pose_rhand)
void BimanualFreeMotionController::InitializeReach2GraspTask(   double period_, Object_to_Grasp &GrspObj, ioStateManager &ioSM_)
{
	
	// /////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 										Reach to grasp task variables
	// /////////////////////////////////////////////////////////////////////////////////////////////////////////	 ioSM.wbTS.lhand.Pose
	period = period_;		

	// Transformations related to Grasping points
	// ==========================================
	// World homogeneous transformation of the grasp configurations absolute frame : w_H_absGpoints
	// ----------------------------------------------------------------------------
	Obj_H_absGpoints.block<3,3>(0,0) = Eigen::MatrixXd::Identity(3,3);   //
	for(int i=0; i<N_eef; i++) 
		Obj_H_absGpoints.block<3,1>(0,3) += (double) (1./N_eef) * GrspObj.Obj_Pose_Gpoints[i].head<3>();
	//
	// Homogeneous transformation of object frame in world
	Matrix4d w_H_Obj = Transforms.PoseVector2HomogenousMx(GrspObj.Object_Pose);
	w_H_absGpoints = w_H_Obj * Obj_H_absGpoints;
	// world transformation of grasp points
	for(int i=0; i<N_eef; i++) 
		w_H_cp[i]	  = w_H_Obj * Transforms.PoseVector2HomogenousMx(GrspObj.Obj_Pose_Gpoints[i]);

	// homogeneous transformation of grasp point in the absolute grasp point frame
	// ---------------------------------------------------------------------------
	for(int i=0; i<N_eef; i++) 
		absGpoints_H_cp[i] = Obj_H_absGpoints.inverse() * Transforms.PoseVector2HomogenousMx(GrspObj.Obj_Pose_Gpoints[i]);

	// Transformations related to robot Hands
	// ======================================
	// world pose of the hands [0]: left and [1]: right 
	// for(int i=0; i<N_eef; i++) 
	w_H_hand[0] = Transforms.PoseVector2HomogenousMx(ioSM_.wbTS.lhand.Pose);
	w_H_hand[1] = Transforms.PoseVector2HomogenousMx(ioSM_.wbTS.rhand.Pose);
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
		absHands_H_v[i].block<3,1>(0,3) = ctrl_param.virtualObjectScaling * absHands_H_vstar[i].block<3,1>(0,3);
		// world transformation
		w_H_v[i] 	                    = w_H_absHands * absHands_H_v[i];
		
		// For transition via resting configuration
		absHands_H_vstar_ini[i] = absHands_H_v[i];
		absHands_H_vstar_fin[i] = absGpoints_H_cp[i];	
		//
		absHands_H_vstar_ini_R[i].block<3,3>(0,0) = absHands_H_vstar[i].block<3,3>(0,0); 	
		absHands_H_vstar_ini_R[i].block<3,1>(0,3) = 0.7*ctrl_param.virtualObjectScaling * absHands_H_vstar[i].block<3,1>(0,3);
	}

	// Motion phases Low-pass Filters
	// ===============================
	Vector6d init_velo; init_velo.setZero();
	//
	for(int i=0; i<N_eef; i++)
	{
		Filter_approach[i].InitializeFilter(period, 10.0, 10.0, init_velo);
		Filter_aperture[i].InitializeFilter(period, 10.0, 10.0, init_velo);
		Filter_synchro[i].InitializeFilter(period, 10.0, 10.0, init_velo);
	}

	Filter_absolute.InitializeFilter(period, 10.0, 10.0, init_velo);
	Filter_relative.InitializeFilter(period, 10.0, 10.0, init_velo);

	//
	gamma_reachable_p = 0.0;
	gamma_reachable_o = 0.0;
	// TO BE SET RELATIVE TO THE STANCE FOOT FRAME
	// Rotation Matrices
	// w_H_hand_des[0].block<3,3>(0,0) << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0;
	// w_H_hand_des[1].block<3,3>(0,0) << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0;
	// // translatiion
	// w_H_hand_des[0].block<3,1>(0,3) << 0.18,  0.17, 0.67;
	// w_H_hand_des[1].block<3,1>(0,3) << 0.18, -0.17, 0.67;

}




// ////////////////////////////////////////////////////////////////////////////////////////////////////
//  								REACHING AND GRASPING TASKS
// ////////////////////////////////////////////////////////////////////////////////////////////////////


// bool BimanualFreeMotionController::compute_Reach2GraspMotion(	Vector7d 				Obj_Pose_cp[],
// 																TaskSpaceTrajectories  	StatesObject_,
// 																Vector7d 				w_Pose_lhand,
// 																Vector7d 				w_Pose_rhand)
bool BimanualFreeMotionController::compute_Reach2GraspMotion(	Object_to_Grasp &GrspObj, ioStateManager  &ioSM_)
														
{
	// world pose of the hand
	// =======================
	w_H_hand[0] = Transforms.PoseVector2HomogenousMx(ioSM_.wbTS.lhand.Pose);				// left
	w_H_hand[1] = Transforms.PoseVector2HomogenousMx(ioSM_.wbTS.rhand.Pose);				// right
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
		// Obj_H_gpoint[i] = Transforms.PoseVector2HomogenousMx(GrspObj.Obj_Pose_Gpoints[i]);
		// t[i] = Obj_H_gpoint[i].block<3,1>(0,3);
		t[i] = Transforms.PoseVector2HomogenousMx(GrspObj.Obj_Pose_Gpoints[i]).block<3,1>(0,3);
		Matrix3d skew_Mx_gpt;
		skew_Mx_gpt <<       0.0,  -t[i](2),  	 t[i](1),
					     t[i](2),       0.0,  	-t[i](0),
					    -t[i](1),   t[i](0),        0.0;
		//			                 
		velocity_Twist_Mx.block<3,3>(0,3) = -skew_Mx_gpt;
		// velocity
		w_velocity_ro_gpoint[i] = velocity_Twist_Mx * GrspObj.States_Object.velocity*1.0;
		// acceleration
		w_acceleration_ro_gpoint[i] = GrspObj.States_Object.acceleration;
		w_acceleration_ro_gpoint[i].head<3>() += GrspObj.States_Object.velocity.tail<3>().cross(GrspObj.States_Object.velocity.tail<3>().cross(t[i]));
	}

	// =======================================================================================================================
	// Synchronization phase (robot's end-effectors towards grasp point on scaled virtual object)
	// =======================================================================================================================
	Vector6d w_velocity_synchro_0[N_eef];
	Matrix4d w_H_hand_t[N_eef];
	w_H_hand_t[0].setIdentity();
	w_H_hand_t[1].setIdentity();
	//	
	// // gamma_synchro orientation to BE MADE FUNCTON OF THE POSITION (COUPLED)    #############################################################################################################################################
	// for(int i=0; i<N_eef; i++)
	// {	
	// 	//
	// 	w_H_hand_t[i].block<3,1>(0,3) = (1.0-gamma_reachable_p)*w_H_hand[i].block<3,1>(0,3) + gamma_reachable_p *w_H_v[i].block<3,1>(0,3);
	// 	w_H_hand_t[i].block<3,3>(0,0) = Transforms.getCombinedRotationMatrix(gamma_reachable_o, w_H_hand[i].block<3,3>(0,0), w_H_v[i].block<3,3>(0,0));

	// 	// get the 6D servoing variables
	// 	Transforms.compute_6Dservoing_variables(w_H_hand[i],			// origin
	// 											w_H_hand_t[i], 			//w_H_v[i],	// destination
	// 											error_synchro[i],
	// 											L_Mu_Theta_synchro[i],
	// 											L_eta_synchro_in_w[i]);
	// 	// compute the velocity
	// 	w_velocity_synchro[i] = -L_eta_synchro_in_w[i].inverse() * ctrl_param.bimanip_gains.synchro.asDiagonal()* error_synchro[i];
	// 	// apply saturation to the computed value
	// 	w_velocity_synchro[i] = Transforms.SaturationTwist(ctrl_param.max_reaching_velocity(0), ctrl_param.max_reaching_velocity(3), w_velocity_synchro[i]);
		
	// 	// computation of the acceleration through filtered derivative
	// 	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// 	// filter grasp
	// 	Filter_synchro[i].pole    = Filter_synchro[i].gain = ctrl_param.bimanip_gains.synchro(0) * ctrl_param.alpha_manip_filter;

	// 	w_velocity_synchro_0[i]   = w_velocity_synchro[i];
	// 	w_velocity_synchro[i]     = Filter_synchro[i].getEulerIntegral(w_velocity_synchro[i]);
	// 	w_acceleration_synchro[i] = Filter_synchro[i].pole * (w_velocity_synchro_0[i] - w_velocity_synchro[i]);
	// }
	// coupling the orientation with the position
	double coupling_synchro[N_eef]; 		for(int i=0; i<N_eef; i++) coupling_synchro[i] = 0.0;
	double psd_time_cpl_synchro[N_eef];
	Vector6d error_c_in_d;
	Matrix4d d_H_c;

	// Position
	// ----------
	for(int i=0; i<N_eef; i++)
	{
		error_synchro[i].setZero();
		// Position error
		error_synchro[i].head(3) = w_H_hand[i].block<3,1>(0,3) - gamma_reachable_p *w_H_v[i].block<3,1>(0,3) 
									// -(1.-gamma_reachable_p)*w_H_hand[i].block<3,1>(0,3);
									 	-(1.-gamma_reachable_p)*w_H_hand_des[i].block<3,1>(0,3);
		//
		psd_time_cpl_synchro[i]	= 1.0/(50.*error_synchro[i].head(3).norm()+1e-15);					
		coupling_synchro[i]		= 1.0 - exp(-psd_time_cpl_synchro[i]/0.04);		
		//
		// Coupling the orientation with the position error
		w_H_hand_t[i].block<3,3>(0,0) = Transforms.getCombinedRotationMatrix(coupling_synchro[i], w_H_hand[i].block<3,3>(0,0), w_H_v[i].block<3,3>(0,0)); //desired
		//
		d_H_c = w_H_hand_t[i].inverse() * w_H_hand[i];
		// relative pose error
        error_c_in_d = Transforms.get_PoseError_cur2des(d_H_c);
        error_synchro[i].tail(3) = error_c_in_d.tail(3);
        // 3D Orientation Jacobian 
        L_Mu_Theta_synchro[i] = Transforms.get_L_Mu_Theta_Matrix(d_H_c.block<3,3>(0,0)) * w_H_hand[i].block<3,3>(0,0).transpose();

        // computing the velocity
        w_velocity_synchro[i].head(3) = -1. * ctrl_param.bimanip_gains.synchro.head(3).asDiagonal()*error_synchro[i].head(3);
        w_velocity_synchro[i].tail(3) = -L_Mu_Theta_synchro[i].inverse() * ctrl_param.bimanip_gains.synchro.tail(3).asDiagonal()*error_synchro[i].tail(3);

        // apply saturation to the computed value
		w_velocity_synchro[i] = Transforms.SaturationTwist(ctrl_param.max_reaching_velocity(0), ctrl_param.max_reaching_velocity(3), w_velocity_synchro[i]);
		
		// computation of the acceleration through filtered derivative
		// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		// filter grasp
		Filter_synchro[i].pole    = Filter_synchro[i].gain = ctrl_param.bimanip_gains.synchro(0) * ctrl_param.alpha_manip_filter;

		w_velocity_synchro_0[i]   = w_velocity_synchro[i];
		w_velocity_synchro[i]     = Filter_synchro[i].getEulerIntegral(w_velocity_synchro[i]);
		w_acceleration_synchro[i] = Filter_synchro[i].pole * (w_velocity_synchro_0[i] - w_velocity_synchro[i]);
	}

	// // =======================================================================================================================
	// // Approach phase (absolute hands frame towards the absolute frame of the grasp configurations)
	// // =======================================================================================================================
	double gamma_synchro; 		gamma_synchro = 0.0;
	double psd_time_synchro;
	//
	// Vector3d error_h_v = 0.5*(error_synchro[0].head<3>() + error_synchro[1].head<3>());
	Vector6d error_h_v = 0.5*(error_synchro[0] + error_synchro[1]);
	psd_time_synchro   = 1.0/(50.*error_h_v.norm()+1e-15);
	gamma_synchro      = 1.0 - exp(-psd_time_synchro/0.04); //0.06

	// get the 6D servoing variables of the (abs towards obj)
	Matrix4d w_H_Obj = Transforms.PoseVector2HomogenousMx(GrspObj.States_Object.pose); 
	w_H_absGpoints = w_H_Obj * Obj_H_absGpoints; 

	// desired absolute hands frame
	this->get_absolute_hands_frame(w_H_Obj, w_H_hand_des, w_H_absHands_des);
	// assign the same orienatation as the world
	w_H_absHands_des.block<3,3>(0,0) = Eigen::MatrixXd::Identity(3,3);

	// get the frame interpolation
	Matrix4d w_H_absHands_t; 	w_H_absHands_t.setIdentity();
	w_H_absHands_t.block<3,1>(0,3) = (1.0-gamma_synchro*gamma_reachable_p)*w_H_absHands_des.block<3,1>(0,3) + gamma_synchro*gamma_reachable_p*w_H_absGpoints.block<3,1>(0,3);
	w_H_absHands_t.block<3,3>(0,0) = Transforms.getCombinedRotationMatrix(gamma_reachable_o*gamma_synchro, w_H_absHands_des.block<3,3>(0,0), w_H_absGpoints.block<3,3>(0,0));
	//
	Vector6d w_velocity_approach_0[N_eef];

	for(int i=0; i<N_eef; i++)
	{	
		// get the 6D servoing variables
		Transforms.compute_6Dservoing_variables(	w_H_absHands,					// origin
											w_H_absHands_t,				// destination (target : w_H_absGpoints)
											error_approach[i],
											L_Mu_Theta_approach[i],
											L_eta_approach_in_w[i]);
		// compute the velocity
		w_velocity_approach[i] = -L_eta_approach_in_w[i].inverse() * ctrl_param.bimanip_gains.approach.asDiagonal()* error_approach[i];
		// apply saturation to the computed value
		w_velocity_approach[i] = Transforms.SaturationTwist(ctrl_param.max_reaching_velocity(0), ctrl_param.max_reaching_velocity(3), w_velocity_approach[i]);
		
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
		// gamma_approach    = 1.0 - exp(-pow(psd_time_approach,2.0)/0.1); 
		gamma_approach    = 1.0 - exp(-pow(psd_time_approach,2.0)/0.08);  // 0.04
		// gamma_approach = 1.0;
		// (virtual grasp point of scaled virtual object towards grasp point on the unscaled virtual object)
		w_H_vstar_t[i]    = w_H_absHands * ((1.0-gamma_reachable_p*gamma_approach)*absHands_H_vstar_ini[i] + gamma_reachable_p*gamma_approach*absHands_H_vstar_fin[i]);  //	Obj_H_gpoint[i];

		// get the 6D servoing variables
		Transforms.compute_6Dservoing_variables(	w_H_v[i],
													w_H_vstar_t[i],   				//	w_H_vstar_t[i],
													error_aperture[i],
													L_Mu_Theta_aperture[i],
													L_eta_aperture_in_w[i]);
		// compute the velocity
		w_velocity_aperture[i] = -L_eta_aperture_in_w[i].inverse() * ctrl_param.bimanip_gains.aperture.asDiagonal()* error_aperture[i];
		// apply saturation to the computed value
		w_velocity_aperture[i] = Transforms.SaturationTwist(ctrl_param.max_reaching_velocity(0), ctrl_param.max_reaching_velocity(3), w_velocity_aperture[i]);
		
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

		Transforms.UpdatePose_From_VelocityTwist(period, vstar_6x6DR_w[i]*w_velocity_aperture[i], vstar_H_v[i]);
		w_H_v[i] = w_H_vstar[i] * vstar_H_v[i];
	}
	return true;
}

// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  								RELEASING AND RETRACTING TASKS
// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// bool BimanualFreeMotionController::Release_and_Retract(Vector7d 				Obj_Pose_cp[],
// 														TaskSpaceTrajectories  	StatesObject_,
// 														Vector7d 				w_Pose_lhand,
// 														Vector7d 				w_Pose_rhand)

bool BimanualFreeMotionController::Release_and_Retract(Object_to_Grasp &GrspObj, ioStateManager  &ioSM_)
{
	//
	// get the 6D servoing variables of the (abs towards obj)
	Matrix4d w_H_Obj = Transforms.PoseVector2HomogenousMx(GrspObj.States_Object.pose); 
	
	// world pose of the hand
	w_H_hand[0] = Transforms.PoseVector2HomogenousMx(ioSM_.wbTS.lhand.Pose);
	w_H_hand[1] = Transforms.PoseVector2HomogenousMx(ioSM_.wbTS.rhand.Pose);

	// absolute hands frame
	this->get_absolute_hands_frame(w_H_Obj, w_H_hand, w_H_absHands);
	// desired absolute hands frame
	this->get_absolute_hands_frame(w_H_Obj, w_H_hand_des, w_H_absHands_des);

	//
	for(int i=0; i<N_eef; i++)
	{
		w_H_vstar[i] = w_H_absHands * absGpoints_H_cp[i];
		w_H_cp[i] 	 = w_H_Obj  * Transforms.PoseVector2HomogenousMx(GrspObj.Obj_Pose_Gpoints[i]);
	}

	// release motion of the object: expension of the virtual object  (v towards vstar)
	//=================================================================================
	// (virtual grasp point of scaled virtual object towards
	// grasp point on the unscaled virtual object)
	//
	Vector6d w_velocity_aperture_0[N_eef];
	//
	for(int i=0; i<N_eef; i++)
	{
		//
		// Hand aperture closing (through coupling with Approach DS)
		w_H_vstar_t[i] = w_H_absHands * absHands_H_vstar_ini_R[i];
		// get the 6D servoing variables
		Transforms.compute_6Dservoing_variables(w_H_v[i],				// origin
												w_H_vstar_t[i],			// target (desired)
												error_aperture[i],
												L_Mu_Theta_aperture[i],
												L_eta_aperture_in_w[i]);

		// compute the velocity
		w_velocity_aperture[i] = -L_eta_aperture_in_w[i].inverse() * ctrl_param.bimanip_gains.aperture.asDiagonal()* error_aperture[i];
		// apply saturation to the computed value
		w_velocity_aperture[i] = Transforms.SaturationTwist(ctrl_param.max_reaching_velocity(0), ctrl_param.max_reaching_velocity(3), w_velocity_aperture[i]);
		
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
	Vector6d w_velocity_approach_0[N_eef];
	//
	Matrix4d w_H_absHands_t, absHands_H_vstar_des[N_eef];			
	w_H_absHands_t.setIdentity();
	w_H_absHands_t.block<3,3>(0,0) = w_H_Obj.block<3,3>(0,0);
	//
	for(int i=0; i<N_eef; i++)
	{
		//
		psd_time_app = 1.0/(50.*error_aperture[i].head<3>().norm()+1e-10);
		gamma_app    = 1.0 - exp(-pow(psd_time_app, 2.5)/0.05); //3.0/0.1
		// ------
		// retract motion (through coupling with aperture DS)
		w_H_absHands_t.block<3,1>(0,3) = (1.0-gamma_app)*w_H_absHands.block<3,1>(0,3) + gamma_app*w_H_absHands_des.block<3,1>(0,3);
		// desired virtual frame after release
		absHands_H_vstar_des[i] = absHands_H_vstar_ini[i];
		absHands_H_vstar_des[i].block<3,1>(0,3) *=0.4; 
		w_H_vstar_des[i] = w_H_absHands_t * absGpoints_H_cp[i]; 				//Obj_H_gpoint[i]; //absE_H_vstar_des[i];
		//
		Transforms.compute_6Dservoing_variables( w_H_vstar[i],						//	origin
											w_H_vstar_des[i],      				//	target (desired)  w_H_cp[i],
											error_approach[i],
											L_Mu_Theta_approach[i],
											L_eta_approach_in_w[i]);
		// compute the velocity
		w_velocity_approach[i] = -L_eta_approach_in_w[i].inverse() * ctrl_param.bimanip_gains.approach.asDiagonal()* error_approach[i];
		// apply saturation to the computed value
		w_velocity_approach[i] = Transforms.SaturationTwist(ctrl_param.max_reaching_velocity(0), ctrl_param.max_reaching_velocity(3), w_velocity_approach[i]);
		
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
	Matrix4d w_H_hd_t[N_eef];
	Vector6d w_velocity_synchro_0[N_eef];
	//
	for(int i=0; i<N_eef; i++)
	{
		//
		w_H_hd_t[i].setIdentity();

		psd_time_synchro = 1.0/(50.*error_approach[i].head<3>().norm()+1e-10);
		gamma_synchro    = 1.0 - exp(-pow(psd_time_synchro, 2.5)/0.05); //3.0/0.1
		//
		w_H_hd_t[i].block<3,1>(0,3) = (1. - gamma_synchro)*w_H_v[i].block<3,1>(0,3) + gamma_synchro*w_H_hand_des[i].block<3,1>(0,3);
		w_H_hd_t[i].block<3,3>(0,0) = Transforms.getCombinedRotationMatrix(gamma_synchro, w_H_hand_des[i].block<3,3>(0,0), w_H_v[i].block<3,3>(0,0));
	
		// get the 6D servoing variables
		Transforms.compute_6Dservoing_variables(	w_H_hand[i],					// current
											w_H_hd_t[i],					// desired w_H_v[i]
											error_synchro[i],
											L_Mu_Theta_synchro[i],
											L_eta_synchro_in_w[i]);
		// compute the velocity
		w_velocity_synchro[i] = -L_eta_synchro_in_w[i].inverse() * ctrl_param.bimanip_gains.synchro.asDiagonal()* error_synchro[i];
		// apply saturation to the computed value
		w_velocity_synchro[i] = Transforms.SaturationTwist(ctrl_param.max_reaching_velocity(0), ctrl_param.max_reaching_velocity(3), w_velocity_synchro[i]);
		
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
		t[i] = Transforms.PoseVector2HomogenousMx(GrspObj.Obj_Pose_Gpoints[i]).block<3,1>(0,3);
		Matrix3d skew_Mx_gpt;
		skew_Mx_gpt << 	    0.0,  -t[i](2),  	 t[i](1),
					    t[i](2),       0.0,  	-t[i](0),
					   -t[i](1),   t[i](0),        0.0;
		//			                 
		velocity_Twist_Mx.block<3,3>(0,3) = -skew_Mx_gpt;
		// velocity
		w_velocity_ro_gpoint[i] = velocity_Twist_Mx * GrspObj.States_Object.velocity*1.0;
		// acceleration
		w_acceleration_ro_gpoint[i] = GrspObj.States_Object.acceleration;
		w_acceleration_ro_gpoint[i].head<3>() += GrspObj.States_Object.velocity.tail<3>().cross(GrspObj.States_Object.velocity.tail<3>().cross(t[i]));
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
	Matrix6d vstar_6x6DR_w[N_eef];  
	Matrix4d vstar_H_v[N_eef];
	//--------------------------------------------
	Matrix6d vstar_invInteractionMx_aperture[N_eef];
	Vector6d vstar_velocity_aperture[N_eef];
	
	for(int i=0; i<N_eef; i++)
	{
		// update the pose of the virtual object relative to the world frame  w_H_v
		vstar_H_v[i] = w_H_vstar[i].inverse() * w_H_v[i];
		vstar_6x6DR_w[i].setZero();
		vstar_6x6DR_w[i].block<3,3>(0,0) = w_H_vstar[i].block<3,3>(0,0).transpose();
		vstar_6x6DR_w[i].block<3,3>(3,3) = vstar_6x6DR_w[i].block<3,3>(0,0);

		// -------------------------------------------------------------------------------------------------
		Transforms.UpdatePose_From_VelocityTwist(period, vstar_6x6DR_w[i]*w_velocity_aperture[i], vstar_H_v[i]);
		w_H_v[i] = w_H_vstar[i] * vstar_H_v[i];

	}
	return true;
}

//
// bool BimanualFreeMotionController::Release_and_Retract2(	Vector7d 				Obj_Pose_cp[],
// 															TaskSpaceTrajectories  	StatesObject_,
// 															Vector7d 				w_Pose_lhand,
// 															Vector7d 				w_Pose_rhand)
bool BimanualFreeMotionController::Release_and_Retract2(Object_to_Grasp &GrspObj, ioStateManager  &ioSM_)
{
	// world pose of the hand
	// =======================
	w_H_hand[0] = Transforms.PoseVector2HomogenousMx(ioSM_.wbTS.lhand.Pose);	 // left
	w_H_hand[1] = Transforms.PoseVector2HomogenousMx(ioSM_.wbTS.rhand.Pose);	 // right
	// Offset coorection
	w_H_hand[0].block<3,1>(0,3) +=  w_H_hand[0].block<3,3>(0,0)*ctrl_param.hand_offset[0];
	w_H_hand[1].block<3,1>(0,3) +=  w_H_hand[1].block<3,3>(0,0)*ctrl_param.hand_offset[1];
	//
	// get the 6D servoing variables of the (abs towards obj)
	Matrix4d w_H_Obj = Transforms.PoseVector2HomogenousMx(GrspObj.States_Object.pose); 
	// world transformation of grasp points
	for(int i=0; i<N_eef; i++) 
		w_H_cp[i]	  = w_H_Obj * Transforms.PoseVector2HomogenousMx(GrspObj.Obj_Pose_Gpoints[i]);
	//
	// get the task transformations
	Matrix4d W_H_ah, lh_H_rh;
	Matrix4d W_H_ap, lp_H_rp;
	Matrix4d W_H_ah_std, lh_H_rh_std;

	// hands transforms
	this->get_bimanual_coordination_transforms(w_H_hand[0], w_H_hand[1], W_H_ah, lh_H_rh);
	// object's grasp points transforms
	this->get_bimanual_coordination_transforms(w_H_cp[0], w_H_cp[1], W_H_ap, lp_H_rp);
	// standby hands transformations
	this->get_bimanual_coordination_transforms(w_H_hand_des[0], w_H_hand_des[1], W_H_ah_std, lh_H_rh_std);
	//
	// std::cout << " w_H_hand[0] 		\n" << w_H_hand[0] << std::endl;
	// std::cout << " w_H_hand[1]  	\n" << w_H_hand[1] << std::endl;
	std::cout << " W_H_ah 		\n" << W_H_ah << std::endl;
	std::cout << " lh_H_rh  	\n" << lh_H_rh << std::endl;
	std::cout << " W_H_ap 		\n" << W_H_ap << std::endl;
	std::cout << " lp_H_rp 		\n" << lp_H_rp << std::endl;

	// // relative transformation for velocity
	// Matrix4d ap_H_ah = W_H_ap.inverse() * W_H_ah;
	// Matrix4d lp_H_lh = lp_H_rp * lh_H_rh.inverse();
	//
	// /////////////////////////////////////////////////////////////////////////////////////
	// =====================================
	//velocity of grasp points on the object
	// =====================================
	// /////////////////////////////////////////////////////////////////////////////////////
	Matrix6d velocity_Twist_Mx;		velocity_Twist_Mx.setIdentity();
	Eigen::Vector3d t[N_eef];
	Matrix4d Obj_H_gpoint[N_eef];

	for(int i=0; i<N_eef; i++)
	{
		// Obj_H_gpoint[i] = Transforms.PoseVector2HomogenousMx(GrspObj.Obj_Pose_Gpoints[i]);
		// t[i] = Obj_H_gpoint[i].block<3,1>(0,3);
		t[i] = Transforms.PoseVector2HomogenousMx(GrspObj.Obj_Pose_Gpoints[i]).block<3,1>(0,3);
		Matrix3d skew_Mx_gpt;
		skew_Mx_gpt <<       0.0,  -t[i](2),  	 t[i](1),
					     t[i](2),       0.0,  	-t[i](0),
					    -t[i](1),   t[i](0),        0.0;
		//			                 
		velocity_Twist_Mx.block<3,3>(0,3) = -skew_Mx_gpt;
		// velocity
		w_velocity_ro_gpoint[i] = velocity_Twist_Mx * GrspObj.States_Object.velocity*1.0;
		// acceleration
		w_acceleration_ro_gpoint[i] = GrspObj.States_Object.acceleration;
		w_acceleration_ro_gpoint[i].head<3>() += GrspObj.States_Object.velocity.tail<3>().cross(GrspObj.States_Object.velocity.tail<3>().cross(t[i]));
	}

	//
	// /////////////////////////////////////////////////////////////////////////////////////
	// =====================================
	// Relative velocity of the hands
	// =====================================
	// ///////////////////////////////////////////////////////////////////////////////////// 
	// Coupling the orientation with the position error
	Matrix3d des_rel_rot = lh_H_rh_std.block<3,3>(0,0); // lp_H_rp.block<3,3>(0,0);
	Matrix4d lh_H_rh_t = lh_H_rh;
	lh_H_rh_t.block<3,3>(0,0) = Transforms.getCombinedRotationMatrix(1.0, lh_H_rh.block<3,3>(0,0), des_rel_rot); //desired
	// relative transformation
	Matrix4d d_H_c = lh_H_rh_t.inverse() * lh_H_rh;  // expressed in the left hand frame
	// orientation error
    error_rel.tail(3) = Transforms.get_PoseError_cur2des(d_H_c).tail(3);
    // 3D Orientation Jacobian 
    Matrix3d L_Mu_Theta_rel = Transforms.get_L_Mu_Theta_Matrix(d_H_c.block<3,3>(0,0)) * w_H_hand[1].block<3,3>(0,0).transpose(); // wrt. the world

	// ///////////////////////////////////////////////////////////////////////////////////////
	Vector3d des_rel_pos = lh_H_rh_std.block<3,1>(0,3); // lp_H_rp.block<3,1>(0,3); 
	// Vector3d des_rel_pos = lp_H_rp.block<3,1>(0,3); //+ (1.-gamma_reachable_p*gamma_abs)*lh_H_rh_std.block<3,1>(0,3); // TBC 
	error_rel.head(3) = lh_H_rh.block<3,1>(0,3) - des_rel_pos;

    // computing the velocity
    // ~~~~~~~~~~~~~~~~~~~~~~~
    w_velocity_rel.head(3) = -1.*ctrl_param.bimanip_gains.relative.head(3).asDiagonal()*error_rel.head(3);
    w_velocity_rel.tail(3) = -L_Mu_Theta_rel.inverse() * ctrl_param.bimanip_gains.relative.tail(3).asDiagonal()*error_rel.tail(3);

    // apply saturation to the computed value
	w_velocity_rel = Transforms.SaturationTwist(ctrl_param.max_reaching_velocity(0), ctrl_param.max_reaching_velocity(3), w_velocity_rel);
	
	// computation of the acceleration through filtered derivative
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// filter grasp
	Filter_relative.pole = Filter_relative.gain = ctrl_param.bimanip_gains.relative(0) * ctrl_param.alpha_manip_filter;

	w_velocity_rel_0     = w_velocity_rel;
	w_velocity_rel       = Filter_relative.getEulerIntegral(w_velocity_rel);
	w_acceleration_rel   = Filter_relative.pole * (w_velocity_rel_0 - w_velocity_rel);

 
	// /////////////////////////////////////////////////////////////////////////////////////
	// =====================================
	// Absolute velocity of the hands
	// =====================================
	// /////////////////////////////////////////////////////////////////////////////////////
	double coupling_rel, coupling_abs; 
	double psd_time_cpl_rel, psd_time_cpl_abs;
	double gamma_abs = 0.0;

	psd_time_cpl_rel = 1.0/(50.*error_rel.head<3>().norm()+1e-10);
	gamma_abs        = 1.0 - exp(-pow(psd_time_cpl_rel, 2.5)/0.05); //3.0/0.1

	// position error accounting for the reachability of the target
	// Vector3d des_abs_pos = gamma_abs *W_H_ap.block<3,1>(0,3) + (1.-gamma_abs)*W_H_ah_std.block<3,1>(0,3);
	Vector3d des_abs_pos = gamma_abs *W_H_ah_std.block<3,1>(0,3) + (1.-gamma_abs)*W_H_ah.block<3,1>(0,3);
	error_abs.head(3)    = W_H_ah.block<3,1>(0,3) - des_abs_pos;

	psd_time_cpl_abs	= 1.0/(50.*error_abs.head(3).norm()+1e-15);					
	coupling_abs		= 1.0 - exp(-psd_time_cpl_abs/0.01); //0.04
	//
	// Coupling the orientation with the position error
	Matrix3d des_abs_rot = gamma_abs *W_H_ah_std.block<3,3>(0,0) + (1.-gamma_abs)*W_H_ah.block<3,3>(0,0);
	Matrix4d W_H_ah_t = W_H_ah;
	W_H_ah_t.block<3,3>(0,0) = Transforms.getCombinedRotationMatrix(coupling_abs, W_H_ah.block<3,3>(0,0), des_abs_rot); //desired
	// relative transformation
	d_H_c = W_H_ah_t.inverse() * W_H_ah;
	// orientation error
    error_abs.tail(3) = Transforms.get_PoseError_cur2des(d_H_c).tail(3);
    // 3D Orientation Jacobian 
    Matrix3d L_Mu_Theta_abs = Transforms.get_L_Mu_Theta_Matrix(d_H_c.block<3,3>(0,0)) * W_H_ah.block<3,3>(0,0).transpose();

    // computing the velocity
    // ~~~~~~~~~~~~~~~~~~~~~~~
    w_velocity_abs.head(3) = -1.*ctrl_param.bimanip_gains.absolute.head(3).asDiagonal()*error_abs.head(3);
    w_velocity_abs.tail(3) = -L_Mu_Theta_abs.inverse() * ctrl_param.bimanip_gains.absolute.tail(3).asDiagonal()*error_abs.tail(3);

    // apply saturation to the computed value
	w_velocity_abs = Transforms.SaturationTwist(ctrl_param.max_reaching_velocity(0), ctrl_param.max_reaching_velocity(3), w_velocity_abs);
	
	// computation of the acceleration through filtered derivative
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// filter grasp
	Filter_absolute.pole = Filter_absolute.gain = ctrl_param.bimanip_gains.absolute(0) * ctrl_param.alpha_manip_filter;

	w_velocity_abs_0     = w_velocity_abs;
	w_velocity_abs       = Filter_absolute.getEulerIntegral(w_velocity_abs);
	w_acceleration_abs   = Filter_absolute.pole * (w_velocity_abs_0 - w_velocity_abs);


	// /////////////////////////////////////////////////////////////////////////////////////
	// ========================================
	// Computation of individual hands motion
	// ========================================
	// ///////////////////////////////////////////////////////////////////////////////////// 
	// velocity
	this->get_left_and_right_Twist(w_velocity_abs, w_velocity_rel, w_velocity_eef[0], w_velocity_eef[1]);
	w_velocity_eef[0] 		+= 0.0*w_velocity_ro_gpoint[0];
	w_velocity_eef[1] 		+= 0.0*w_velocity_ro_gpoint[1];

	// w_velocity_eef[0].head(3) *= 0.0;
	// w_velocity_eef[1].head(3) *= 0.0;

	// acceleration
	this->get_left_and_right_Twist(w_acceleration_abs, w_acceleration_rel, w_acceleration_eef[0], w_acceleration_eef[1]);
	w_acceleration_eef[0] 	+= 0.0*w_acceleration_ro_gpoint[0];
	w_acceleration_eef[1] 	+= 0.0*w_acceleration_ro_gpoint[1];	

	// w_acceleration_eef[0].head(3) *= 0.0;
	// w_acceleration_eef[1].head(3) *= 0.0;

	//
	std::cout << " \n" << std::endl;
	std::cout << " gamma_abs \t" << gamma_abs << std::endl;
	std::cout << " error_abs " << error_abs.transpose() << std::endl;
	std::cout << " error_rel " << error_rel.transpose() << std::endl;
	std::cout << " w_velocity_abs " << w_velocity_abs.transpose() << std::endl;
	std::cout << " w_velocity_rel " << w_velocity_rel.transpose() << std::endl;
	std::cout << " w_velocity_eef left " << w_velocity_eef[0].transpose() << std::endl;
	std::cout << " w_velocity_eef right" << w_velocity_eef[1].transpose() << std::endl;

	return true;
}


// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

//
void BimanualFreeMotionController::get_absolute_hands_frame(Matrix4d w_H_Obj_, Matrix4d w_H_h_[], Matrix4d &w_H_absE_)
{
	//
	w_H_absE_.block<3,3>(0,0) = w_H_Obj_.block<3,3>(0,0);			
	w_H_absE_.block<3,1>(0,3) << 0.0, 0.0, 0.0;
	//
	for(int i=0; i<N_eef; i++)	w_H_absE_.block<3,1>(0,3) += (double) (1./N_eef)* w_H_h_[i].block<3,1>(0,3);
	// w_H_absE_.block<3,1>(0,3) *= (double) (1./N_eef);
}
//
bool BimanualFreeMotionController::get_bimanual_coordination_transforms(Matrix4d W_H_l, Matrix4d W_H_r, Matrix4d &W_H_a_, Matrix4d &l_H_r_)
{
	//
	W_H_a_.setIdentity(4,4); 
	l_H_r_.setIdentity(4,4); 
	//
	// relative transformation
	// ========================
	// l_H_r = W_H_l.inverse() * W_H_r;
	l_H_r_.block<3,1>(0,3) = W_H_r.block<3,1>(0,3) - W_H_l.block<3,1>(0,3);				// translation expresse wrt. the world
	l_H_r_.block<3,3>(0,0) = W_H_l.block<3,3>(0,0).transpose() * W_H_r.block<3,3>(0,0);	// orienatation wrt. the left hand
	// find the abolute transformation
	// ======================================================
	// Axis angle of relative hands orientation
    Eigen::AngleAxisd l_orientation_r(l_H_r_.block<3,3>(0,0));
    // // Average orientation between hands
    Vector3d axis_ = l_orientation_r.axis();
    double theta_  = 0.5*l_orientation_r.angle();
    Eigen::AngleAxisd av_rot(theta_, axis_);

    
    // Rotation matrix of the absolute hand frame expressed in the asbolute foot frame
    Matrix3d W_R_a =  W_H_l.block<3,3>(0,0) * av_rot.toRotationMatrix();

	W_H_a_.block<3,1>(0,3) = 0.5*(W_H_l.block<3,1>(0,3) + W_H_r.block<3,1>(0,3));
	// W_H_a_.block<3,3>(0,0) = W_H_l.block<3,3>(0,0); //W_R_a;
	W_H_a_.block<3,3>(0,0) = Transforms.getCombinedRotationMatrix(0.5, W_H_l.block<3,3>(0,0), W_H_r.block<3,3>(0,0));

	return true;
}

void BimanualFreeMotionController::get_left_and_right_Twist(Vector6d vel_a, Vector6d vel_r, Vector6d &left_t, Vector6d &right_t)
{
	//
	MatrixXd Th = Transforms.get_bimanual_task_TwistMap_inv(ctrl_param.a_bi, ctrl_param.b_bi);

	// std::cout << " BIMAN TWIST MAP  Th 		\n" << Th << std::endl;

	// Th.block(0,3, 12, 3) *= 0.0; 

	left_t 	= Th.topLeftCorner(6,6)  *vel_a
			+ Th.topRightCorner(6,6) *vel_r;

	right_t = Th.bottomLeftCorner(6,6)  *vel_a
			+ Th.bottomRightCorner(6,6) *vel_r;
}

// ////////////////////////////////////////////////////
// bool BimanualFreeMotionController::compute_Reach2GraspMotion2(	Vector7d 				Obj_Pose_cp[],
// 																TaskSpaceTrajectories  	StatesObject_,
// 																Vector7d 				w_Pose_lhand,
// 																Vector7d 				w_Pose_rhand)
bool BimanualFreeMotionController::compute_Reach2GraspMotion2(Object_to_Grasp &GrspObj, ioStateManager  &ioSM_)
{
	// world pose of the hand
	// =======================
	w_H_hand[0] = Transforms.PoseVector2HomogenousMx(ioSM_.wbTS.lhand.Pose);	 // left
	w_H_hand[1] = Transforms.PoseVector2HomogenousMx(ioSM_.wbTS.rhand.Pose);	 // right
	// Offset coorection
	w_H_hand[0].block<3,1>(0,3) +=  w_H_hand[0].block<3,3>(0,0)*ctrl_param.hand_offset[0];
	w_H_hand[1].block<3,1>(0,3) +=  w_H_hand[1].block<3,3>(0,0)*ctrl_param.hand_offset[1];
	//
	// get the 6D servoing variables of the (abs towards obj)
	Matrix4d w_H_Obj = Transforms.PoseVector2HomogenousMx(GrspObj.States_Object.pose); 
	// world transformation of grasp points
	for(int i=0; i<N_eef; i++) 
		w_H_cp[i]	  = w_H_Obj * Transforms.PoseVector2HomogenousMx(GrspObj.Obj_Pose_Gpoints[i]);
	//
	// get the task transformations
	Matrix4d W_H_ah, lh_H_rh;
	Matrix4d W_H_ap, lp_H_rp;
	Matrix4d W_H_ah_std, lh_H_rh_std;
	Matrix4d lp_H_rp_pgrasp;

	// hands transforms
	this->get_bimanual_coordination_transforms(w_H_hand[0], w_H_hand[1], W_H_ah, lh_H_rh);
	// object's grasp points transforms
	this->get_bimanual_coordination_transforms(w_H_cp[0], w_H_cp[1], W_H_ap, lp_H_rp);
	// standby hands transformations
	this->get_bimanual_coordination_transforms(w_H_hand_des[0], w_H_hand_des[1], W_H_ah_std, lh_H_rh_std);
	//
	lp_H_rp_pgrasp      = lp_H_rp;
	lp_H_rp_pgrasp(1, 3) *= 0.75*ctrl_param.virtualObjectScaling(1,1); // * lp_H_rp(1, 3);
	
	// std::cout << " w_H_hand[0] 		\n" << w_H_hand[0] << std::endl;
	// std::cout << " w_H_hand[1]  	\n" << w_H_hand[1] << std::endl;

	// std::cout << " w_H_cp[0] 		\n" << w_H_cp[0] << std::endl;
	// std::cout << " w_H_cp[1]  	\n" << w_H_cp[1] << std::endl;
	
	// std::cout << " W_H_ah 		\n" << W_H_ah << std::endl;
	// std::cout << " lh_H_rh  	\n" << lh_H_rh << std::endl;
	// std::cout << " W_H_ap 		\n" << W_H_ap << std::endl;
	// std::cout << " lp_H_rp 		\n" << lp_H_rp << std::endl;

	std::cout << " Error hand poses[0] 		\n" << (w_H_hand[0].block<3,1>(0,3)-w_H_cp[0].block<3,1>(0,3)).transpose()  << std::endl;
	std::cout << " Error hand poses[1]  	\n" << (w_H_hand[1].block<3,1>(0,3)-w_H_cp[1].block<3,1>(0,3)).transpose()  << std::endl;


	// // relative transformation for velocity
	// Matrix4d ap_H_ah = W_H_ap.inverse() * W_H_ah;
	// Matrix4d lp_H_lh = lp_H_rp * lh_H_rh.inverse();
	//
	// /////////////////////////////////////////////////////////////////////////////////////
	// =====================================
	//velocity of grasp points on the object
	// =====================================
	// /////////////////////////////////////////////////////////////////////////////////////
	Matrix6d velocity_Twist_Mx;		velocity_Twist_Mx.setIdentity();
	Eigen::Vector3d t[N_eef];
	Matrix4d Obj_H_gpoint[N_eef];

	for(int i=0; i<N_eef; i++)
	{
		// Obj_H_gpoint[i] = Transforms.PoseVector2HomogenousMx(GrspObj.Obj_Pose_Gpoints[i]);
		// t[i] = Obj_H_gpoint[i].block<3,1>(0,3);
		t[i] = Transforms.PoseVector2HomogenousMx(GrspObj.Obj_Pose_Gpoints[i]).block<3,1>(0,3);
		Matrix3d skew_Mx_gpt;
		skew_Mx_gpt <<       0.0,  -t[i](2),  	 t[i](1),
					     t[i](2),       0.0,  	-t[i](0),
					    -t[i](1),   t[i](0),        0.0;
		//			                 
		velocity_Twist_Mx.block<3,3>(0,3) = -skew_Mx_gpt;
		// velocity
		w_velocity_ro_gpoint[i] = velocity_Twist_Mx * GrspObj.States_Object.velocity*1.0;
		// acceleration
		w_acceleration_ro_gpoint[i] = GrspObj.States_Object.acceleration;
		w_acceleration_ro_gpoint[i].head<3>() += GrspObj.States_Object.velocity.tail<3>().cross(GrspObj.States_Object.velocity.tail<3>().cross(t[i]));
	}
 
	// /////////////////////////////////////////////////////////////////////////////////////
	// =====================================
	// Absolute velocity of the hands
	// =====================================
	// /////////////////////////////////////////////////////////////////////////////////////
	double coupling_abs; 
	double psd_time_cpl_abs;
	// gamma_reachable_p = 0.0;
	// position error accounting for the reachability of the target
	// Vector3d des_abs_pos = gamma_reachable_p *W_H_ap.block<3,1>(0,3) + (1.-gamma_reachable_p)*W_H_ah.block<3,1>(0,3);
	Vector3d des_abs_pos = gamma_reachable_p *W_H_ap.block<3,1>(0,3) + (1.-gamma_reachable_p)*W_H_ah_std.block<3,1>(0,3);
	error_abs.head(3)    = W_H_ah.block<3,1>(0,3) - des_abs_pos;

	psd_time_cpl_abs	= 1.0/(50.*error_abs.head(3).norm()+1e-15);					
	coupling_abs		= 1.0 - exp(-psd_time_cpl_abs/0.02); //0.04
	//
	// Coupling the orientation with the position error
	Matrix3d des_abs_rot = gamma_reachable_p *W_H_ap.block<3,3>(0,0) + (1.-gamma_reachable_p)*W_H_ah_std.block<3,3>(0,0);
	Matrix4d W_H_ah_t = W_H_ah;
	W_H_ah_t.block<3,3>(0,0) = Transforms.getCombinedRotationMatrix(coupling_abs, W_H_ah.block<3,3>(0,0), des_abs_rot); //desired
	// relative transformation
	Matrix4d d_H_c = W_H_ah_t.inverse() * W_H_ah;
	// orientation error
    error_abs.tail(3) = Transforms.get_PoseError_cur2des(d_H_c).tail(3);
    // 3D Orientation Jacobian 
    Matrix3d L_Mu_Theta_abs = Transforms.get_L_Mu_Theta_Matrix(d_H_c.block<3,3>(0,0)) * W_H_ah.block<3,3>(0,0).transpose();

    // computing the velocity
    // ~~~~~~~~~~~~~~~~~~~~~~~
    w_velocity_abs.head(3) = -1.*ctrl_param.bimanip_gains.absolute.head(3).asDiagonal()*error_abs.head(3);
    w_velocity_abs.tail(3) = -L_Mu_Theta_abs.inverse() * ctrl_param.bimanip_gains.absolute.tail(3).asDiagonal()*error_abs.tail(3);

    // apply saturation to the computed value
	w_velocity_abs = Transforms.SaturationTwist(ctrl_param.max_reaching_velocity(0), ctrl_param.max_reaching_velocity(3), w_velocity_abs);
	
	// computation of the acceleration through filtered derivative
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// filter grasp
	Filter_absolute.pole = Filter_absolute.gain = ctrl_param.bimanip_gains.absolute(0) * ctrl_param.alpha_manip_filter;

	w_velocity_abs_0     = w_velocity_abs;
	w_velocity_abs       = Filter_absolute.getEulerIntegral(w_velocity_abs);
	w_acceleration_abs   = Filter_absolute.pole * (w_velocity_abs_0 - w_velocity_abs);




	// /////////////////////////////////////////////////////////////////////////////////////
	// =====================================
	// Relative velocity of the hands
	// =====================================
	// ///////////////////////////////////////////////////////////////////////////////////// 
	// Matrix4d lp_H_lh = lp_H_rp * lh_H_rh.inverse();

	double coupling_rel; 
	double psd_time_rel;
	double gamma_abs = 0.0;

	psd_time_rel   = 1.0/(50.*error_abs.head(3).norm()+1e-15);
	gamma_abs      = 1.0 - exp(-psd_time_rel/0.04); //0.06

	// /////////////////////////////////////////////////////////////////////////////////////// lp_H_rp_pgrasp
	// Coupling the orientation with the position error
	// Matrix3d des_rel_rot = gamma_reachable_p*gamma_abs*lp_H_rp.block<3,3>(0,0) + (1.-gamma_reachable_p*gamma_abs)*lh_H_rh_std.block<3,3>(0,0);
	Matrix3d des_rel_rot = gamma_reachable_o*gamma_abs*lp_H_rp.block<3,3>(0,0) 
						 + gamma_reachable_o*(1.-gamma_abs)*lp_H_rp_pgrasp.block<3,3>(0,0) 
						 + (1.-gamma_reachable_o)*lh_H_rh_std.block<3,3>(0,0);

	Matrix4d lh_H_rh_t = lh_H_rh;
	lh_H_rh_t.block<3,3>(0,0) = Transforms.getCombinedRotationMatrix(gamma_abs, lh_H_rh.block<3,3>(0,0), des_rel_rot); //desired
	// relative transformation
	// d_H_c = lh_H_rh_t * lh_H_rh.inverse();  // expressed in the left hand frame
	d_H_c = lh_H_rh_t.inverse() * lh_H_rh;  // expressed in the left hand frame
	
	// orientation error
    error_rel.tail(3) = Transforms.get_PoseError_cur2des(d_H_c).tail(3);
    // 3D Orientation Jacobian 
    Matrix3d L_Mu_Theta_rel = Transforms.get_L_Mu_Theta_Matrix(d_H_c.block<3,3>(0,0)) * w_H_hand[1].block<3,3>(0,0).transpose(); // wrt. the world


	// ///////////////////////////////////////////////////////////////////////////////////////
	psd_time_rel = 1.0/(50.*error_abs.head(3).norm()+1e-15);					
	// coupling_rel = 1.0 - exp(-psd_time_rel/0.1); 
	// coupling_rel = 1.0 - exp(-pow(psd_time_rel,2.8)/0.1); 
	coupling_rel = 1.0 - exp(-pow(psd_time_rel,2.8)/0.05); 
	// coupling_rel = 0.0;

	// position error accounting for the reachability of the target
	// Vector3d des_rel_pos = gamma_reachable_p*coupling_rel *lp_H_rp.block<3,1>(0,3) + (1.-gamma_reachable_p*coupling_rel)*lh_H_rh_std.block<3,1>(0,3); // TBC
	Vector3d des_rel_pos = gamma_reachable_p*coupling_rel *lp_H_rp.block<3,1>(0,3) 
						 + gamma_reachable_p*(1.-coupling_rel)*lp_H_rp_pgrasp.block<3,1>(0,3)
						 + (1.-gamma_reachable_p)*lh_H_rh_std.block<3,1>(0,3); // TBC  

	error_rel.head(3) = lh_H_rh.block<3,1>(0,3) - des_rel_pos;

	// psd_time_rel	= 1.0/(50.*error_rel.norm()+1e-15);					
	// coupling_rel	= 1.0; // - exp(-psd_time_rel/0.04);
	//
	// ///////////////////////////////////////////////////////////////////////////////////////

	// // Coupling the orientation with the position error
	// Matrix3d des_rel_rot = gamma_reachable_p*gamma_abs*lp_H_rp.block<3,3>(0,0) + (1.-gamma_reachable_p*gamma_abs)*lh_H_rh_std.block<3,3>(0,0);
	// // Matrix3d des_rel_rot = lp_H_rp.block<3,3>(0,0); // + (1.-gamma_reachable_p*gamma_abs)*lh_H_rh_std.block<3,3>(0,0);
	// Matrix4d lh_H_rh_t = lh_H_rh;
	// lh_H_rh_t.block<3,3>(0,0) = Transforms.getCombinedRotationMatrix(coupling_abs, lh_H_rh.block<3,3>(0,0), des_rel_rot); //desired
	// // relative transformation
	// // d_H_c = lh_H_rh_t * lh_H_rh.inverse();  // expressed in the left hand frame
	// d_H_c = lh_H_rh_t.inverse() * lh_H_rh;  // expressed in the left hand frame
	
	// // orientation error
 //    error_rel.tail(3) = Transforms.get_PoseError_cur2des(d_H_c).tail(3);
 //    // 3D Orientation Jacobian 
 //    Matrix3d L_Mu_Theta_rel = Transforms.get_L_Mu_Theta_Matrix(d_H_c.block<3,3>(0,0)) * w_H_hand[1].block<3,3>(0,0).transpose(); // wrt. the world

    // computing the velocity
    // ~~~~~~~~~~~~~~~~~~~~~~~
    w_velocity_rel.head(3) = -1.*ctrl_param.bimanip_gains.relative.head(3).asDiagonal()*error_rel.head(3);
    w_velocity_rel.tail(3) = -L_Mu_Theta_rel.inverse() * ctrl_param.bimanip_gains.relative.tail(3).asDiagonal()*error_rel.tail(3);

    // apply saturation to the computed value
	w_velocity_rel = Transforms.SaturationTwist(ctrl_param.max_reaching_velocity(0), ctrl_param.max_reaching_velocity(3), w_velocity_rel);
	
	// computation of the acceleration through filtered derivative
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// filter grasp
	Filter_relative.pole = Filter_relative.gain = ctrl_param.bimanip_gains.relative(0) * ctrl_param.alpha_manip_filter;

	w_velocity_rel_0     = w_velocity_rel;
	w_velocity_rel       = Filter_relative.getEulerIntegral(w_velocity_rel);
	w_acceleration_rel   = Filter_relative.pole * (w_velocity_rel_0 - w_velocity_rel);

	// /////////////////////////////////////////////////////////////////////////////////////
	// ========================================
	// Computation of individual hands motion
	// ========================================
	// ///////////////////////////////////////////////////////////////////////////////////// 
	// velocity
	this->get_left_and_right_Twist(w_velocity_abs, w_velocity_rel, w_velocity_eef[0], w_velocity_eef[1]);
	w_velocity_eef[0] 		+= w_velocity_ro_gpoint[0];
	w_velocity_eef[1] 		+= w_velocity_ro_gpoint[1];

	// w_velocity_eef[0].head(3) *= 0.0;
	// w_velocity_eef[1].head(3) *= 0.0;

	// acceleration
	this->get_left_and_right_Twist(w_acceleration_abs, w_acceleration_rel, w_acceleration_eef[0], w_acceleration_eef[1]);
	w_acceleration_eef[0] 	+= w_acceleration_ro_gpoint[0];
	w_acceleration_eef[1] 	+= w_acceleration_ro_gpoint[1];	

	// w_acceleration_eef[0].head(3) *= 0.0;
	// w_acceleration_eef[1].head(3) *= 0.0;

	//
	std::cout << " \n" << std::endl;
	// std::cout << " gamma_abs \t" << gamma_abs << std::endl;
	// std::cout << " error_abs " << error_abs.transpose() << std::endl;
	// std::cout << " error_rel " << error_rel.transpose() << std::endl;
	// std::cout << " w_velocity_abs " << w_velocity_abs.transpose() << std::endl;
	// std::cout << " w_velocity_rel " << w_velocity_rel.transpose() << std::endl;
	// std::cout << " w_velocity_eef left " << w_velocity_eef[0].transpose() << std::endl;
	// std::cout << " w_velocity_eef right" << w_velocity_eef[1].transpose() << std::endl;

	return true;
}


// compute bimanual constrained motion of the hands
// bool BimanualFreeMotionController::getBimanualConstrainedMotion(Matrix4d    cp_desH_Obj_[],
// 																Vector7d    w_Pose_obj_ref,
//                                                                 Vector7d    w_Pose_lhand,
//                                                                 Vector7d    w_Pose_rhand)

bool BimanualFreeMotionController::getBimanualConstrainedMotion(Matrix4d    cp_desH_Obj_[],
																Vector7d    w_Pose_obj_ref,
					                                            ioStateManager  &ioSM_)
{
    // world pose of the hand
    // =======================
    w_H_hand[0] = Transforms.PoseVector2HomogenousMx(ioSM_.wbTS.lhand.Pose);  // left
    w_H_hand[1] = Transforms.PoseVector2HomogenousMx(ioSM_.wbTS.rhand.Pose);  // right
    // Offset coorection
    // w_H_hand[0].block<3,1>(0,3) +=  w_H_hand[0].block<3,3>(0,0)*ctrl_param.hand_offset[0];
    // w_H_hand[1].block<3,1>(0,3) +=  w_H_hand[1].block<3,3>(0,0)*ctrl_param.hand_offset[1];

    // world transformation of grasp points
    // for(int i=0; i<N_eef; i++) 
    //     w_H_cp[i]     = Transforms.PoseVector2HomogenousMx(w_Pose_cp[i]);
    // // get the current grasping points with the respect to the desired object pose in the world frame
    Matrix4d w_desH_cp[2];
    Matrix4d w_ref_H_Obj  = Transforms.PoseVector2HomogenousMx(w_Pose_obj_ref); 
    w_desH_cp[0] = w_ref_H_Obj * cp_desH_Obj_[0].inverse();
    w_desH_cp[1] = w_ref_H_Obj * cp_desH_Obj_[1].inverse();

    //
    // get the task transformations
    Matrix4d W_H_ah, lh_H_rh;
    Matrix4d W_H_ap, lp_H_rp;
    Matrix4d W_H_ah_std, lh_H_rh_std;
    // hands transforms
    this->get_bimanual_coordination_transforms(w_H_hand[0], w_H_hand[1], W_H_ah, lh_H_rh);
    // object's grasp points transforms
    this->get_bimanual_coordination_transforms(w_desH_cp[0], w_desH_cp[1], W_H_ap, lp_H_rp);
    // standby hands transformations
    this->get_bimanual_coordination_transforms(w_H_hand_des[0], w_H_hand_des[1], W_H_ah_std, lh_H_rh_std);
    //
    // std::cout << " W_H_ah       \n" << W_H_ah << std::endl;
    // std::cout << " lh_H_rh      \n" << lh_H_rh << std::endl;
    // std::cout << " W_H_ap       \n" << W_H_ap << std::endl;
    // std::cout << " lp_H_rp      \n" << lp_H_rp << std::endl;
    // /////////////////////////////////////////////////////////////////////////////////////
    // =====================================
    // Relative velocity of the hands
    // =====================================
    // ///////////////////////////////////////////////////////////////////////////////////// 
    // Coupling the orientation with the position error
    Matrix3d des_rel_rot = lp_H_rp.block<3,3>(0,0);
    Matrix4d lh_H_rh_t = lh_H_rh;
    lh_H_rh_t.block<3,3>(0,0) = des_rel_rot; //desired
    // relative transformation
    Matrix4d d_H_c = lh_H_rh_t.inverse() * lh_H_rh;  // expressed in the left hand frame
    
    // orientation error
    error_rel.tail(3) = Transforms.get_PoseError_cur2des(d_H_c).tail(3);
    // 3D Orientation Jacobian 
    Matrix3d L_Mu_Theta_rel = Transforms.get_L_Mu_Theta_Matrix(d_H_c.block<3,3>(0,0)) * w_H_hand[1].block<3,3>(0,0).transpose(); // wrt. the world

    // ///////////////////////////////////////////////////////////////////////////////////////
    Vector3d des_rel_pos = lp_H_rp.block<3,1>(0,3); // TBC 
    error_rel.head(3) = lh_H_rh.block<3,1>(0,3) - des_rel_pos;

    // computing the velocity
    // ~~~~~~~~~~~~~~~~~~~~~~~
    w_velocity_rel.head(3) = -2.5*ctrl_param.bimanip_gains.relative.head(3).asDiagonal()*error_rel.head(3);
    w_velocity_rel.tail(3) = -2.5*L_Mu_Theta_rel.inverse() * ctrl_param.bimanip_gains.relative.tail(3).asDiagonal()*error_rel.tail(3);

    // apply saturation to the computed value
    w_velocity_rel = Transforms.SaturationTwist(ctrl_param.max_reaching_velocity(0), ctrl_param.max_reaching_velocity(3), w_velocity_rel);
    // ///////////////////////////////////////////////////////////////////////////////////////

    double coupling_rel, coupling_abs; 
	double psd_time_cpl_rel, psd_time_cpl_abs;
	double gamma_abs = 0.0;

	psd_time_cpl_rel = 1.0/(50.*error_rel.head<3>().norm()+1e-10);
	gamma_abs        = 1.0 - exp(-pow(psd_time_cpl_rel, 2.5)/0.05); //3.0/0.1
    
    // /////////////////////////////////////////////////////////////////////////////////////
    // =====================================
    // Absolute velocity of the hands
    // =====================================
    // /////////////////////////////////////////////////////////////////////////////////////
    // position error accounting for the reachability of the target
    // Vector3d des_abs_pos = gamma_abs *W_H_ap.block<3,1>(0,3) + (1.-gamma_abs)*W_H_ah.block<3,1>(0,3);
    Vector3d des_abs_pos = W_H_ap.block<3,1>(0,3);
    error_abs.head(3)    = W_H_ah.block<3,1>(0,3) - des_abs_pos;
    //
    // Coupling the orientation with the position error
    // Matrix3d des_abs_rot = gamma_abs *W_H_ap.block<3,3>(0,0) + (1.-gamma_abs)*W_H_ah.block<3,3>(0,0);
    Matrix3d des_abs_rot = W_H_ap.block<3,3>(0,0);
    Matrix4d W_H_ah_t = W_H_ah;
    W_H_ah_t.block<3,3>(0,0) = des_abs_rot; //Transforms.getCombinedRotationMatrix(coupling_abs, W_H_ah.block<3,3>(0,0), des_abs_rot); //desired
    // W_H_ah_t.block<3,3>(0,0) = Transforms.getCombinedRotationMatrix(coupling_abs, W_H_ah.block<3,3>(0,0), des_abs_rot); //desired
    // relative transformation
    d_H_c = W_H_ah_t.inverse() * W_H_ah;
    // orientation error
    error_abs.tail(3) = Transforms.get_PoseError_cur2des(d_H_c).tail(3);
    // 3D Orientation Jacobian 
    Matrix3d L_Mu_Theta_abs = Transforms.get_L_Mu_Theta_Matrix(d_H_c.block<3,3>(0,0)) * W_H_ah.block<3,3>(0,0).transpose();

    // computing the velocity
    // ~~~~~~~~~~~~~~~~~~~~~~~
    w_velocity_abs.head(3) = -1.*ctrl_param.bimanip_gains.absolute.head(3).asDiagonal()*error_abs.head(3);
    w_velocity_abs.tail(3) = -L_Mu_Theta_abs.inverse() * ctrl_param.bimanip_gains.absolute.tail(3).asDiagonal()*error_abs.tail(3);

    // apply saturation to the computed value
    w_velocity_abs = Transforms.SaturationTwist(ctrl_param.max_reaching_velocity(0), ctrl_param.max_reaching_velocity(3), w_velocity_abs);
    
    // computation of the acceleration through filtered derivative
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // filter grasp
    Filter_absolute.pole = Filter_absolute.gain = ctrl_param.bimanip_gains.absolute(0) * ctrl_param.alpha_manip_filter;

    w_velocity_abs_0     = w_velocity_abs;
    w_velocity_abs       = Filter_absolute.getEulerIntegral(w_velocity_abs);
    w_acceleration_abs   = Filter_absolute.pole * (w_velocity_abs_0 - w_velocity_abs);

    
    
    
    // computation of the acceleration through filtered derivative
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // filter grasp
    Filter_relative.pole = Filter_relative.gain = ctrl_param.bimanip_gains.relative(0) * ctrl_param.alpha_manip_filter;

    w_velocity_rel_0     = w_velocity_rel;
    w_velocity_rel       = Filter_relative.getEulerIntegral(w_velocity_rel);
    w_acceleration_rel   = Filter_relative.pole * (w_velocity_rel_0 - w_velocity_rel);

    // /////////////////////////////////////////////////////////////////////////////////////
    // ========================================
    // Computation of individual hands motion
    // ========================================
    // ///////////////////////////////////////////////////////////////////////////////////// 
    // velocity
    this->get_left_and_right_Twist(w_velocity_abs, w_velocity_rel, w_velocity_eef[0], w_velocity_eef[1]);

    // acceleration
    this->get_left_and_right_Twist(w_acceleration_abs, w_acceleration_rel, w_acceleration_eef[0], w_acceleration_eef[1]);
    //
    std::cout << " \n" << std::endl;
    // std::cout << " error_abs " << error_abs.transpose() << std::endl;
    // std::cout << " error_rel " << error_rel.transpose() << std::endl;
    // std::cout << " w_velocity_abs " << w_velocity_abs.transpose() << std::endl;
    // std::cout << " w_velocity_rel " << w_velocity_rel.transpose() << std::endl;
    // std::cout << " w_velocity_eef left " << w_velocity_eef[0].transpose() << std::endl;
    // std::cout << " w_velocity_eef right" << w_velocity_eef[1].transpose() << std::endl;

    return true;
}

// // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


