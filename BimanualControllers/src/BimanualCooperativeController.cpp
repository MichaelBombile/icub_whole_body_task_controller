
#include "BimanualCooperativeController.h"

using namespace std;
using namespace Eigen;


bwc_Vars 		bwc_vars;
bwc_Params 		bwc_params;
bwc_Workspace 	bwc_work;
bwc_Settings 	bwc_settings;

cmo_Vars 		cmo_vars;
cmo_Params 		cmo_params;
cmo_Workspace 	cmo_work;
cmo_Settings 	cmo_settings;


BimanualCooperativeController::BimanualCooperativeController()	: GraspHessianMatrix(42,42)
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
																, optimal_hands_velocity(12) {}

BimanualCooperativeController::~BimanualCooperativeController(){}


// bool BimanualCooperativeController::Initialize(	ControllerParameters &ctrl_param,
// 												Vector7d object_pose,
// 												Vector7d lhandPose_world,
// 												Vector7d rhandPose_world)

bool BimanualCooperativeController::Initialize(ControllerParameters &ctrl_param, Object_to_Grasp &GrspObj, ioStateManager &ioSM_)
// bool BimanualCooperativeController::threadInit()
{
	
	// Initialization to zero of some variables ioSM_.wbTS.lhand.Pose 
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

	// Non-zero initialization
	// -----------------------
	tol_dist2contact     	= 	ctrl_param.tol_dist2contact;
	//
	Weight_hands_wrench 	= 	ctrl_param.Weight_hands_wrench;
	Weight_hands_slack  	= 	ctrl_param.Weight_hands_slack;
	//
	Weight_absolute_motion 	= 	ctrl_param.Weight_absolute_motion;
	Weight_relative_motion 	= 	ctrl_param.Weight_relative_motion;
	weight_regularizaion    = 	ctrl_param.weight_regularizaion;

	min_normalForce  =  ctrl_param.min_normalForce;
	max_normalForce  =  ctrl_param.max_normalForce;
	gain_tau_hands	 = 	ctrl_param.gain_tau_hands;
	EnableSaturation =  ctrl_param.EnableSaturation;
	TorqueCorrection =  ctrl_param.TorqueCorrection;
	apply_wrench     =  ctrl_param.apply_wrench;

	// Initialization of the Grasp constraint class
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	GraspConstraints.Initialize(ctrl_param.Hands_contact_param); 
	//
	GraspConstraints.min_nF_lh =  18.0;
	GraspConstraints.max_nF_lh =  25.0; //ctrl_param.max_normalForce;
	GraspConstraints.min_nF_rh =  18.0;
	GraspConstraints.max_nF_rh =  25.0; //ctrl_param.max_normalForce;

	GraspConstraints.computeBimanualGraspMatrix(GrspObj.Object_Pose, ioSM_.wbTS.lhand.Pose, ioSM_.wbTS.rhand.Pose, GraspMatrixHands);
	// motion Jacobian
	GraspMotion_Jacobian 		  = Psd_GraspMatrixHands.transpose();
	// Constraint Jacobian
	GraspConstraint_Jacobian      = Eigen::MatrixXd::Identity(12,12) - GraspMatrixHands.transpose() * Psd_GraspMatrixHands.transpose();
	//
	GraspMotion_Jacobian_prev     = GraspMotion_Jacobian;
	GraspConstraint_Jacobian_prev = GraspConstraint_Jacobian;

	// Instatiation of the object class
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	ContactConfidence 	= 0.0;
	dist2contact_lh 	= 0.1;  // something different than 0 
	dist2contact_rh 	= 0.1;
	// initialization of the cvxgen solver for the cooperative manipulation
	bwc_set_defaults();
	bwc_setup_indexing();
	//
	cmo_set_defaults();
	cmo_setup_indexing();
	//
	wrench_correction_lh.setZero();
	wrench_correction_rh.setZero(); 

	torque_correction_lh.setZero();
	torque_correction_rh.setZero();

	F_applied_lhand.setZero();
	F_applied_rhand.setZero();
	F_In_lhand.setZero();
	F_In_rhand.setZero();


	// test the algorithm 

	return true;
		
}


bool BimanualCooperativeController::getGraspKineDynVariables(	Vector7d object_pose,
																Matrix4d w_H_cp_l,
										 						Matrix4d w_H_cp_r,
																Vector7d lhandPose_world,
																Vector7d rhandPose_world)
{
	// update the grasp matrix
	GraspConstraints.computeBimanualGraspMatrix(object_pose, lhandPose_world, rhandPose_world, GraspMatrixHands);
	//
	// hands
	Vector3d w_axis_lh = lhandPose_world.segment<3>(3)/lhandPose_world.segment<3>(3).norm();
	Vector3d w_axis_rh = rhandPose_world.segment<3>(3)/rhandPose_world.segment<3>(3).norm();

	// Rotation matrix for the left hand frame to have it Z axis pointing towiards the object
	Eigen::Matrix3d w_coor_R_lh;
	w_coor_R_lh.setZero(); 	w_coor_R_lh(0,0) = 1.0; 		w_coor_R_lh(1,1) = -1.0; 		w_coor_R_lh(2,2) = -1.0; 	
	// 
	Eigen::Matrix3d w_R_lh = Eigen::AngleAxisd(lhandPose_world(6), w_axis_lh).toRotationMatrix() * w_coor_R_lh;
	Eigen::Matrix3d w_R_rh = Eigen::AngleAxisd(rhandPose_world(6), w_axis_rh).toRotationMatrix();
	// 
	// left hand
	world_Xstar_deslhand.block<3,3>(0,0) = w_R_lh;
	world_Xstar_deslhand.block<3,3>(3,3) = w_R_lh;

	// left hand
	world_Xstar_desrhand.block<3,3>(0,0) = w_R_rh;
	world_Xstar_desrhand.block<3,3>(3,3) = w_R_rh;
	//
	double min_Fz = 15.0;
	//
	this->setMinNormalForcesHands(min_Fz, w_R_lh, wrench_correction_lh);
	this->setMinNormalForcesHands(min_Fz, w_R_rh, wrench_correction_rh);

	// // Contacts
	// // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// 	// // distances normal to contacts
	// 	// dist2contact_lh = (w_H_cp_l.block<3,1>(0,3)-lhandPose_world.head<3>()).transpose()*w_R_lh.block<3,1>(0,2);	// left
	// 	// dist2contact_rh = (w_H_cp_r.block<3,1>(0,3)-rhandPose_world.head<3>()).transpose()*w_R_rh.block<3,1>(0,2);	// right
	// 	// // dist2contact_lh = (w_H_cp_l.block<3,1>(0,3)-lhandPose_world.head<3>()).norm();	// left
	// 	// // dist2contact_rh = (w_H_cp_r.block<3,1>(0,3)-rhandPose_world.head<3>()).norm(); 	// right
	// 	// cout << " dist2contact_lh is  \t" << dist2contact_lh << endl;
	// 	// cout << " dist2contact_rh is  \t" << dist2contact_rh << endl;
	// 	// // update the Contact confidence indicator
	// 	// if((fabs(dist2contact_rh) <= tol_dist2contact) && (fabs(dist2contact_lh) <= 0.95*tol_dist2contact))
	// 	// {
	// 	// 	ContactConfidence = 1.0;
	// 	// } else 	{
	// 	// 	ContactConfidence = 0.0;
	// 	// }
	// 	// // apply wrench based on contact confidence
	// 	// if (!apply_wrench)
	// 	// 	ContactConfidence = 0.0;
	// 	// cout << " ContactConfidence is  \t" << ContactConfidence << endl;
	// // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// // Distances normal to contacts
	// dist2contact_lh = (w_H_cp_l.block<3,1>(0,3)-lhandPose_world.head<3>()).transpose()*w_R_lh.block<3,1>(0,2);	// left
	// dist2contact_rh = (w_H_cp_r.block<3,1>(0,3)-rhandPose_world.head<3>()).transpose()*w_R_rh.block<3,1>(0,2);	// right
	// // Positioning error in hand frames
	// Vector3d lh_er = w_R_lh.block<3,3>(0,0).transpose()*(w_H_cp_l.block<3,1>(0,3)-lhandPose_world.head<3>()); 	// 
	// Vector3d rh_er = w_R_rh.block<3,3>(0,0).transpose()*(w_H_cp_r.block<3,1>(0,3)-rhandPose_world.head<3>()); 	// 

	// // if norm in x and y are less than thrxy and z less than tol
	// //------------------------------------------------------------
	// double l_exy = lh_er.head(2).norm();
	// double r_exy = rh_er.head(2).norm();
	// bool tsk = false; 
	// if((lh_er.head(2).norm() <= 1.2*tol_dist2contact) && (rh_er.head(2).norm() <= 1.2*tol_dist2contact)) tsk = true;
	// else tsk = false;
	// //
	// dist2contact_lh = (lh_er(2));
	// dist2contact_rh = (rh_er(2));
	// // dist2contact_lh = (w_H_cp_l.block<3,1>(0,3)-lhandPose_world.head<3>()).norm(); 	// left
	// // dist2contact_rh = (w_H_cp_r.block<3,1>(0,3)-rhandPose_world.head<3>()).norm();	// right
	// cout << " dist2contact_lh is  \t" << dist2contact_lh << endl;
	// cout << " dist2contact_rh is  \t" << dist2contact_rh << endl;
	// cout << " l_exy is  \t" << l_exy << endl;
	// cout << " r_exy is  \t" << r_exy << endl;
	// cout << " contact task    is  \t" << tsk << endl;
	// // update the Contact confidence indicator
	// // if((fabs(dist2contact_rh) <= tol_dist2contact) && (fabs(dist2contact_lh) <= 0.95*tol_dist2contact))
	// if(tsk && (fabs(dist2contact_rh) <= tol_dist2contact) && (fabs(dist2contact_lh) <= tol_dist2contact)){
	// // if( (fabs(dist2contact_rh) <= tol_dist2contact) && (fabs(dist2contact_lh) <= tol_dist2contact)){
	// 	ContactConfidence = 1.0;
	// } else 	{
	// 	ContactConfidence = 0.0;
	// }
	// cout << " ContactConfidence before is  \t" << ContactConfidence << endl;
	// cout << " apply_wrench before is  \t" << apply_wrench << endl;
	// // apply wrench based on contact confidence
	// // if (!apply_wrench) ContactConfidence = 0.0;

	// cout << " ContactConfidence l is  \t" << (fabs(dist2contact_rh) <= tol_dist2contact) << endl;
	// cout << " ContactConfidence r is  \t" << (fabs(dist2contact_lh) <= tol_dist2contact) << endl;
	// cout << " ContactConfidence is  \t" << ContactConfidence << endl;
	// // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	return true;
}


void BimanualCooperativeController::check_contact_confidence(	Matrix4d w_H_cp_l, Matrix4d w_H_cp_r, Vector7d lhandPose_world, Vector7d rhandPose_world)
{
	
	// rotation wrt. of hands with their z axis pointing towards the object contact surface
	Eigen::Matrix3d w_R_lh = this->world_Xstar_deslhand.block<3,3>(0,0);
	Eigen::Matrix3d w_R_rh = this->world_Xstar_desrhand.block<3,3>(0,0);
	// Distances normal to contacts
	// dist2contact_lh = (w_H_cp_l.block<3,1>(0,3)-lhandPose_world.head<3>()).transpose()*w_R_lh.block<3,1>(0,2);	// left
	// dist2contact_rh = (w_H_cp_r.block<3,1>(0,3)-rhandPose_world.head<3>()).transpose()*w_R_rh.block<3,1>(0,2);	// right
	// Positioning error in hand frames
	Vector3d lh_er = w_R_lh.block<3,3>(0,0).transpose()*(w_H_cp_l.block<3,1>(0,3)-lhandPose_world.head<3>()); 	// 
	Vector3d rh_er = w_R_rh.block<3,3>(0,0).transpose()*(w_H_cp_r.block<3,1>(0,3)-rhandPose_world.head<3>()); 	// 

	// if norm in x and y are less than thrxy and z less than tol
	//------------------------------------------------------------
	double l_exy = lh_er.head(2).norm();
	double r_exy = rh_er.head(2).norm();
	bool tsk = false; 
	if((lh_er.head(2).norm() <= 1.2*tol_dist2contact) && (rh_er.head(2).norm() <= 1.2*tol_dist2contact)) tsk = true;
	else tsk = false;
	//
	dist2contact_lh = (lh_er(2));
	dist2contact_rh = (rh_er(2));
	// dist2contact_lh = (w_H_cp_l.block<3,1>(0,3)-lhandPose_world.head<3>()).norm(); 	// left
	// dist2contact_rh = (w_H_cp_r.block<3,1>(0,3)-rhandPose_world.head<3>()).norm();	// right
	cout << " dist2contact_lh is  \t" << dist2contact_lh << endl;
	cout << " dist2contact_rh is  \t" << dist2contact_rh << endl;
	cout << " l_exy is  \t" << l_exy << endl;
	cout << " r_exy is  \t" << r_exy << endl;
	cout << " contact task    is  \t" << tsk << endl;
	// update the Contact confidence indicator
	// if((fabs(dist2contact_rh) <= tol_dist2contact) && (fabs(dist2contact_lh) <= 0.95*tol_dist2contact))
	if(tsk && (fabs(dist2contact_rh) <= tol_dist2contact) && (fabs(dist2contact_lh) <= tol_dist2contact)){
	// if( (fabs(dist2contact_rh) <= tol_dist2contact) && (fabs(dist2contact_lh) <= tol_dist2contact)){
		ContactConfidence = 1.0;
	} else 	{
		ContactConfidence = 0.0;
	}
	cout << " ContactConfidence before is  \t" << ContactConfidence << endl;
	cout << " apply_wrench before is  \t" << apply_wrench << endl;
	// apply wrench based on contact confidence
	// if (!apply_wrench) ContactConfidence = 0.0;
	cout << " ContactConfidence l is  \t" << (fabs(dist2contact_rh) <= tol_dist2contact) << endl;
	cout << " ContactConfidence r is  \t" << (fabs(dist2contact_lh) <= tol_dist2contact) << endl;
	cout << " ContactConfidence is  \t" << ContactConfidence << endl;
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

void BimanualCooperativeController::computeControlWrench(Vector6d Desired_object_wrench_)
{
	double t_ctrl = yarp::os::Time::now();

	// get the contact constraints
	GraspConstraints.get_lhand_ContactConstraints(world_Xstar_deslhand);
	GraspConstraints.get_rhand_ContactConstraints(world_Xstar_desrhand);
	// get complementary constraints
	GraspConstraints.get_lhand_ComplementaryConstraints(world_Xstar_deslhand, dist2contact_lh, tol_dist2contact);
	GraspConstraints.get_rhand_ComplementaryConstraints(world_Xstar_desrhand, dist2contact_rh, tol_dist2contact);
	//
	// GraspConstraints.get_hands_ComplementaryConstraints(world_Xstar_deslhand, world_Xstar_desrhand, dist2contact_lh, dist2contact_rh, tol_dist2contact);

	// load the data for the solver
	// -----------------------------
	this->load_wrench_data(Desired_object_wrench_);

	bwc_settings.verbose = 0;
	//num_iters = 

	// compute the optimal solution (wrench and acceleration)
	// ======================================================
	bwc_solve();

	// Extract the optimal solution vectors
	// ------------------------------------
	// contact forces
	for(int i=0; i<12; i++)
	{
		optimal_contact_wrench_hands(i) = bwc_vars.Fh[i];
	}
	// slack variables
	for(int i=0; i<6; i++)
	{
		optimal_slack(i) = bwc_vars.w[i];
	}
	//
	std::cout << " compute controller solved in " << yarp::os::Time::now()- t_ctrl << std::endl;

}

//
// void BimanualCooperativeController::computeControlWrench(	double   nu_Wrench_,
// 															Vector7d object_pose,
// 															Matrix4d w_H_cp_l,
// 										 					Matrix4d w_H_cp_r,
// 															Vector7d lhandPose_world,
// 															Vector7d rhandPose_world,
// 															Vector6d Desired_object_wrench_)

void BimanualCooperativeController::computeControlWrench(double   nu_Wrench_, Object_to_Grasp &GrspObj, ioStateManager &ioSM_, Vector6d Desired_object_wrench_)
{
	
	this->getGraspKineDynVariables(	GrspObj.Object_Pose, GrspObj.w_H_Gpoints[0], GrspObj.w_H_Gpoints[1], ioSM_.wbTS.lhand.Pose, ioSM_.wbTS.rhand.Pose);		// update kinedynamic task varibles
	this->check_contact_confidence(	GrspObj.w_H_Gpoints[0],  GrspObj.w_H_Gpoints[1], ioSM_.wbTS.lhand.Pose, ioSM_.wbTS.rhand.Pose);							// check closeness to contact surface
	this->computeControlWrench(Desired_object_wrench_);

	// Extraction of left and right hands wrenches
	F_applied_lhand = optimal_contact_wrench_hands.head<6>();
	F_applied_rhand = optimal_contact_wrench_hands.tail<6>();
	// Normal forces saturation and moments correction
	Matrix4d world_H_lhand   = Transforms.PoseVector2HomogenousMx(ioSM_.wbTS.lhand.Pose); 
	Matrix4d world_H_rhand   = Transforms.PoseVector2HomogenousMx(ioSM_.wbTS.rhand.Pose); 
	Vector7d w_desLHand_Pose = Transforms.HomogenousMx2PoseVector(GrspObj.w_H_Gpoints[0]); 
	Vector7d w_desRHand_Pose = Transforms.HomogenousMx2PoseVector(GrspObj.w_H_Gpoints[1]);

	F_In_lhand.head<3>() = world_H_lhand.block<3,3>(0,0).transpose()*F_applied_lhand.head<3>();
	F_In_lhand.head<3>() = world_H_lhand.block<3,3>(0,0).transpose()*F_applied_lhand.head<3>();
	F_In_rhand.head<3>() = world_H_lhand.block<3,3>(0,0).transpose()*F_applied_rhand.tail<3>();
	F_In_rhand.head<3>() = world_H_lhand.block<3,3>(0,0).transpose()*F_applied_rhand.tail<3>();
	//
	// correction of drift due to saturing the forces in lateral direction (Y)
	double cor_l = 1.0;
	double cor_r = 1.0;

	if(fabs(F_applied_lhand(1)) >= fabs(F_applied_rhand(1)) ){
		cor_r = fabs(F_applied_rhand(1))/ fabs(F_applied_lhand(1));
	} else {
		cor_l = fabs(F_applied_lhand(1))/ fabs(F_applied_rhand(1));
	}
	// Thresholding normal forces
	if(EnableSaturation) {
		this->thresholdNormalForcesHands(min_normalForce, max_normalForce, world_H_lhand.block<3,3>(0,0), F_applied_lhand); 
    	this->thresholdNormalForcesHands(min_normalForce, max_normalForce, world_H_rhand.block<3,3>(0,0), F_applied_rhand);
	}

	// Correction of the hands torques
	if(TorqueCorrection)
	{
		torque_correction_lh = -nu_Wrench_*gain_tau_hands * Transforms.computePoseErrorNormalizedAxisAngle(ioSM_.wbTS.lhand.Pose, w_desLHand_Pose).tail<3>();
		torque_correction_rh = -nu_Wrench_*gain_tau_hands * Transforms.computePoseErrorNormalizedAxisAngle(ioSM_.wbTS.rhand.Pose, w_desRHand_Pose).tail<3>();
	}   

	//
    F_applied_lhand.tail<3>() += torque_correction_lh;
    F_applied_rhand.tail<3>() += torque_correction_rh;
    //
    F_applied_lhand(1) *= cor_l;
	F_applied_rhand(1) *= cor_r;

	// Printing some results
	// ---------------------
	// std::cout << " OPTIMAL HAND WRENCH   LEFT \t " << optimal_contact_wrench_hands.head(6).transpose() << std::endl;
	// std::cout << " OPTIMAL HAND WRENCH  RIGHT \t " << optimal_contact_wrench_hands.tail(6).transpose() << std::endl;
 //    std::cout << " APPLIED HAND WRENCH  LEFT \t " << F_applied_lhand.transpose() << std::endl;
	// std::cout << " APPLIED HAND WRENCH RIGHT \t " << F_applied_rhand.transpose() << std::endl;

}

void BimanualCooperativeController::PredictControlWrench( Object_to_Grasp &GrspObj, Vector7d lhandPose_world, Vector7d rhandPose_world, Vector6d Desired_object_wrench_, VectorXd &Hands_Wrenches)
{
	this->getGraspKineDynVariables(	GrspObj.Object_Pose, GrspObj.w_H_Gpoints[0], GrspObj.w_H_Gpoints[1], lhandPose_world, rhandPose_world);

	this->computeControlWrench(Desired_object_wrench_);

	// Extraction of left and right hands wrenches
	// F_applied_lhand = optimal_contact_wrench_hands.head<6>();
	// F_applied_rhand = optimal_contact_wrench_hands.tail<6>();
	Hands_Wrenches = Eigen::VectorXd::Zero(12);
	Hands_Wrenches.head(6) = optimal_contact_wrench_hands.head<6>();
	Hands_Wrenches.tail(6) = optimal_contact_wrench_hands.tail<6>();
}

void BimanualCooperativeController::load_wrench_data(Vector6d Desired_object_wrench_)
{

	// weight hands wrench
	for(int i=0; i<12; i++)
	{
		bwc_params.QFh[i] = Weight_hands_wrench(i);
	}
	// weight hands slack
	for(int i=0; i<6; i++)
	{
		bwc_params.Qw[i] = Weight_hands_slack(i);
	}

	// desired contact wrench wrench
	for(int i=0; i<6; i++)
	{
		bwc_params.pFh[i]   = 0.0 + wrench_correction_lh(i);
		bwc_params.pFh[6+i] = 0.0 + wrench_correction_rh(i);
	}
	// bwc_params.pFh[2] =   10.0 ;
	// bwc_params.pFh[8] =  -10.0 ;



	// contact activator
	bwc_params.beta[0] = ContactConfidence;  

		
	// Grasp Matrix
	for(int i=0; i<12; i++)
	{
		// bwc_params.Gh_1[i] = GraspMatrixHands(0,i);
		// bwc_params.Gh_2[i] = GraspMatrixHands(1,i); 
		// bwc_params.Gh_3[i] = GraspMatrixHands(2,i); 
		bwc_params.Gh_4[i] = GraspMatrixHands(3,i);
		bwc_params.Gh_5[i] = GraspMatrixHands(4,i);
		bwc_params.Gh_6[i] = GraspMatrixHands(5,i);
	}

	// b1 == Mo*ddot_Xo + bo - fenv
	for(int i=0; i<6; i++)
	{
		bwc_params.b1[i] = Desired_object_wrench_(i);
	}

	// complementary condition
	for (int i=0; i<6; i++)
	{
		bwc_params.Cplh[i] =  GraspConstraints.ComplementaryConstraintMatrix_lh(0,i);
		bwc_params.Cprh[i] =  GraspConstraints.ComplementaryConstraintMatrix_rh(0,i);
	}

	// contact constaints
	for(int i=0; i<6; i++)
	{
		//
		// GraspConstraints.ContactConstraintMatrix_lh *= 0.0;
		// GraspConstraints.ContactConstraintMatrix_rh *= 0.0;

		bwc_params.CLH_1[i] = GraspConstraints.ContactConstraintMatrix_lh(0,i);		bwc_params.CLH_7[i]  = GraspConstraints.ContactConstraintMatrix_lh(6,i);
		bwc_params.CLH_2[i] = GraspConstraints.ContactConstraintMatrix_lh(1,i);		bwc_params.CLH_8[i]  = GraspConstraints.ContactConstraintMatrix_lh(7,i);
		bwc_params.CLH_3[i] = GraspConstraints.ContactConstraintMatrix_lh(2,i);		bwc_params.CLH_9[i]  = GraspConstraints.ContactConstraintMatrix_lh(8,i);
		bwc_params.CLH_4[i] = GraspConstraints.ContactConstraintMatrix_lh(3,i);		bwc_params.CLH_10[i] = GraspConstraints.ContactConstraintMatrix_lh(9,i);
		bwc_params.CLH_5[i] = GraspConstraints.ContactConstraintMatrix_lh(4,i);		bwc_params.CLH_11[i] = GraspConstraints.ContactConstraintMatrix_lh(10,i);
		bwc_params.CLH_6[i] = GraspConstraints.ContactConstraintMatrix_lh(5,i);

		bwc_params.CRH_1[i] = GraspConstraints.ContactConstraintMatrix_rh(0,i);		bwc_params.CRH_7[i]  = GraspConstraints.ContactConstraintMatrix_rh(6,i);
		bwc_params.CRH_2[i] = GraspConstraints.ContactConstraintMatrix_rh(1,i);		bwc_params.CRH_8[i]  = GraspConstraints.ContactConstraintMatrix_rh(7,i);
		bwc_params.CRH_3[i] = GraspConstraints.ContactConstraintMatrix_rh(2,i);		bwc_params.CRH_9[i]  = GraspConstraints.ContactConstraintMatrix_rh(8,i);
		bwc_params.CRH_4[i] = GraspConstraints.ContactConstraintMatrix_rh(3,i);		bwc_params.CRH_10[i] = GraspConstraints.ContactConstraintMatrix_rh(9,i);
		bwc_params.CRH_5[i] = GraspConstraints.ContactConstraintMatrix_rh(4,i);		bwc_params.CRH_11[i] = GraspConstraints.ContactConstraintMatrix_rh(10,i);
		bwc_params.CRH_6[i] = GraspConstraints.ContactConstraintMatrix_rh(5,i);
	}

	// for(int i=0; i<12; i++)
	// {
	// 	bwc_params.blh[i] =  GraspConstraints.ContactConstraintVector_lh(i);
	// 	bwc_params.brh[i] =  GraspConstraints.ContactConstraintVector_rh(i);
	// }

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool BimanualCooperativeController::get_TaskJacobians(double run_period_sec)
{
	// update the grasp kinematics
	MxPsdInv.get_HhQRPseudoInverse(GraspMatrixHands, Psd_GraspMatrixHands);
	// motion Jacobian
	GraspMotion_Jacobian = Psd_GraspMatrixHands.transpose();
	// Constraint Jacobian
	GraspConstraint_Jacobian    = Eigen::MatrixXd::Identity(12,12) - GraspMatrixHands.transpose() * Psd_GraspMatrixHands.transpose();
	// 
	// numerical derivation of the motion and constraint Jacobian
	dotGraspMotion_Jacobian 	= (1./run_period_sec) *(GraspMotion_Jacobian - GraspMotion_Jacobian_prev);
	dotGraspConstraint_Jacobian = (1./run_period_sec) *(GraspConstraint_Jacobian - GraspConstraint_Jacobian_prev);
	//
	// dotGraspMotion_Jacobian_Xhdot     = dotGraspMotion_Jacobian * bimanual_velocity;
	// dotGraspConstraint_Jacobian_Xhdot = dotGraspConstraint_Jacobian * bimanual_velocity;
	//
	GraspMotion_Jacobian_prev     = GraspMotion_Jacobian;
	GraspConstraint_Jacobian_prev = GraspConstraint_Jacobian;

	return true;
}

bool BimanualCooperativeController::get_TaskConsistentHandsVelocity(double run_period_sec,
																 Eigen::Matrix<double, 6, 1> desired_object_velocity)
{
	double t_mo = yarp::os::Time::now();

	// Motion and constraints jacobian matrices
	this->get_TaskJacobians(run_period_sec);

	//
	// load the data for the solver
	// -----------------------------
	b_absolute_motion = desired_object_velocity;
	b_relative_motion.setZero();

	// this->load_motion_data();

	// cmo_settings.verbose = 0;

	// // compute the optimal solution (velocities)
	// // ========================================
	// cmo_solve();

	// // Extract the optimal solution vectors
	// // ------------------------------------
	// // hand motion
	// for(int i=0; i<12; i++)
	// {
	// 	optimal_hands_velocity(i) = cmo_vars.x[i];
	// }

	optimal_hands_velocity = GraspMatrixHands.transpose() * desired_object_velocity;

	// cout << " OPTIMAL HANDS VELOCITY \t" << optimal_hands_velocity.transpose() << endl;

	std::cout << " COMPUTE HANDS solved in " << yarp::os::Time::now()- t_mo << std::endl;

	return true;
}

bool BimanualCooperativeController::get_TaskConsistentHandsAcceleration(double run_period_sec,
																	 Eigen::Matrix<double, 6, 1> desired_object_acceleration,
																	 Eigen::Matrix<double, 6, 1> w_velocity_lhand,
																	 Eigen::Matrix<double, 6, 1> w_velocity_rhand)
{
	// Motion and constraints jacobian matrices
	this->get_TaskJacobians(run_period_sec);
	// load the data for the solver
	// -----------------------------
	Eigen::Matrix<double, 12, 1> bimanual_velocity;
	//
	bimanual_velocity.head<6>() = w_velocity_lhand;
	bimanual_velocity.tail<6>() = w_velocity_rhand;
	//
	b_absolute_motion =  desired_object_acceleration - dotGraspMotion_Jacobian*bimanual_velocity;
	b_relative_motion = -dotGraspConstraint_Jacobian*bimanual_velocity;

	this->load_motion_data();

	cmo_settings.verbose = 0;

	// compute the optimal solution (acceleration)
	// ===========================================
	cmo_solve();

	// Extract the optimal solution vectors
	// ------------------------------------
	// hand motion
	for(int i=0; i<12; i++)
	{
		optimal_hands_acceleration(i) = cmo_vars.x[i];
	}
	//
	cout << " OPTIMAL HANDS ACCELERATION \t" << optimal_hands_acceleration.transpose() << endl;
	return true;
}

void BimanualCooperativeController::load_motion_data()
{
	// weight hands acceleration and absolute motion
	for(int i=0; i<6; i++)
	{
		cmo_params.Qam[i]   = Weight_absolute_motion(i);
		cmo_params.bam[i]   = b_absolute_motion[i];
	}
	// weight hands slack  and relative motion
	for(int i=0; i<12; i++)
	{
		cmo_params.Qrm[i]  = Weight_relative_motion(i);
		cmo_params.brm[i]  = b_relative_motion[i];
	}
	
	// contact activator
	cmo_params.lambda[0] = weight_regularizaion;  

	// Jacobian motion
	for(int i=0; i<6; i++)
	{
		for(int j=0; j<12; j++)
		{
			cmo_params.Jgm[i*6+j] = GraspMotion_Jacobian(i,j);
		}
	}

	// Jacobian motion
	for(int i=0; i<12; i++)
	{
		for(int j=0; j<12; j++)
		{
			cmo_params.Jgc[i*12+j] = GraspConstraint_Jacobian(i,j);
		}
	}

}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool BimanualCooperativeController::thresholdNormalForcesHands(double min, double max, Matrix3d w_R_h, Vector6d &Wrench_w)
{
	// Thresholding normal forces
	Vector6d Wrench_h;
	Wrench_h.head<3>() = w_R_h.transpose()*Wrench_w.head<3>();
	Wrench_h.tail<3>() = w_R_h.transpose()*Wrench_w.tail<3>();

	if(Wrench_h(2) <=min) Wrench_h(2) = min; else if(Wrench_h(2) >= max) Wrench_h(2) = max;
	//
	// Wrench_h(2) = Wrench_h(2)/(fabs(Wrench_h(2))+1e-20) * fabs(max);
	//
	Wrench_w.head<3>() = w_R_h*Wrench_h.head<3>();
	Wrench_w.tail<3>() = w_R_h*Wrench_h.tail<3>();

	return true;
}


Vector6d Wrench_h_l, Wrench_h_r;

bool BimanualCooperativeController::setMinNormalForcesHands(double min, Matrix3d w_R_h, Vector6d &Wrench_w)
{
	// Thresholding normal forces
	Vector6d Wrench_h = Eigen::VectorXd::Zero(6);
	Wrench_h(2) = min;
	//
	Wrench_w.head<3>() = w_R_h*Wrench_h.head<3>();
	Wrench_w.tail<3>() = w_R_h*Wrench_h.tail<3>();

	return true;
}