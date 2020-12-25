

#include "GraspingConstraints.h"

using namespace std;
using namespace Eigen;



GraspingConstraints::GraspingConstraints()  : ContactConstraintMatrix_lh(11, 6)
											, ContactConstraintVector_lh(12)
											, ContactConstraintMatrix_rh(11, 6) 
											, ContactConstraintVector_rh(12)
											, ContactConstraintMatrix_AllHands(22, 12)
											, ContactConstraintVector_AllHands(22) 
											, ComplementaryConstraintMatrix_lh(1,6)
											, ComplementaryConstraintMatrix_rh(1,6)
											{}

GraspingConstraints::~GraspingConstraints(){}

void GraspingConstraints::Initialize( 	VectorXd HandContactParameters) 
{
	
	//  
	mu_hand		= HandContactParameters(0);
	gamma_hand  = HandContactParameters(1);
	deltaX_hand = HandContactParameters(2);
	deltaY_hand = HandContactParameters(3);
	//
	ContactConstraintVector_lh.setZero();
	ContactConstraintVector_rh.setZero();
	//
	min_nF_lh =  10.0;
	max_nF_lh = 20.0;
	min_nF_rh =  10.0;
	max_nF_rh = 20.0;
}

bool GraspingConstraints::get_lhand_ContactConstraints(Matrix6d world_Xstar_lhand)
{
	Matrix6d lhand_X_world = world_Xstar_lhand.inverse();
	// Inequality Constraint matrix left foot
	ContactConstraintMatrix_lh.block<1,6>(0,0)  = -1.0 * lhand_X_world.block<1,6>(2,0);
	ContactConstraintMatrix_lh.block<1,6>(1,0)  = -mu_hand/sqrt(2.0) * lhand_X_world.block<1,6>(2,0) - lhand_X_world.block<1,6>(0,0);
	ContactConstraintMatrix_lh.block<1,6>(2,0)  = -mu_hand/sqrt(2.0) * lhand_X_world.block<1,6>(2,0) + lhand_X_world.block<1,6>(0,0);
	ContactConstraintMatrix_lh.block<1,6>(3,0)  = -mu_hand/sqrt(2.0) * lhand_X_world.block<1,6>(2,0) - lhand_X_world.block<1,6>(1,0);
	ContactConstraintMatrix_lh.block<1,6>(4,0)  = -mu_hand/sqrt(2.0) * lhand_X_world.block<1,6>(2,0) + lhand_X_world.block<1,6>(1,0);
	ContactConstraintMatrix_lh.block<1,6>(5,0)  = 	   	 -gamma_hand * lhand_X_world.block<1,6>(2,0) - lhand_X_world.block<1,6>(5,0);
	ContactConstraintMatrix_lh.block<1,6>(6,0)  = 	     -gamma_hand * lhand_X_world.block<1,6>(2,0) + lhand_X_world.block<1,6>(5,0);
	ContactConstraintMatrix_lh.block<1,6>(7,0)  = 	    -deltaX_hand * lhand_X_world.block<1,6>(2,0) - lhand_X_world.block<1,6>(4,0);
	ContactConstraintMatrix_lh.block<1,6>(8,0)  = 	    -deltaX_hand * lhand_X_world.block<1,6>(2,0) + lhand_X_world.block<1,6>(4,0);
	ContactConstraintMatrix_lh.block<1,6>(9,0)  = 	    -deltaY_hand * lhand_X_world.block<1,6>(2,0) - lhand_X_world.block<1,6>(3,0);
	ContactConstraintMatrix_lh.block<1,6>(10,0) = 	    -deltaY_hand * lhand_X_world.block<1,6>(2,0) + lhand_X_world.block<1,6>(3,0);

	// Inequality Constraint vector left foot
	// ContactConstraintVector_lh.setZero();
	//
	ContactConstraintVector_lh(0)  = min_nF_lh;  // minimal normal forces
	ContactConstraintVector_lh(11) = max_nF_lh;  // maximun normal forces


	return true;
}
		
bool GraspingConstraints::get_rhand_ContactConstraints(Matrix6d world_Xstar_rhand)
{
	Matrix6d rhand_X_world = world_Xstar_rhand.inverse();
	// Inequality Constraint matrix right foot
	ContactConstraintMatrix_rh.block<1,6>(0,0)  = -1.0 * rhand_X_world.block<1,6>(2,0);
	ContactConstraintMatrix_rh.block<1,6>(1,0)  = -mu_hand/sqrt(2.0) * rhand_X_world.block<1,6>(2,0) - rhand_X_world.block<1,6>(0,0);
	ContactConstraintMatrix_rh.block<1,6>(2,0)  = -mu_hand/sqrt(2.0) * rhand_X_world.block<1,6>(2,0) + rhand_X_world.block<1,6>(0,0);
	ContactConstraintMatrix_rh.block<1,6>(3,0)  = -mu_hand/sqrt(2.0) * rhand_X_world.block<1,6>(2,0) - rhand_X_world.block<1,6>(1,0);
	ContactConstraintMatrix_rh.block<1,6>(4,0)  = -mu_hand/sqrt(2.0) * rhand_X_world.block<1,6>(2,0) + rhand_X_world.block<1,6>(1,0);
	ContactConstraintMatrix_rh.block<1,6>(5,0)  = 	     -gamma_hand * rhand_X_world.block<1,6>(2,0) - rhand_X_world.block<1,6>(5,0);
	ContactConstraintMatrix_rh.block<1,6>(6,0)  = 	  	 -gamma_hand * rhand_X_world.block<1,6>(2,0) + rhand_X_world.block<1,6>(5,0);
	ContactConstraintMatrix_rh.block<1,6>(7,0)  = 	  	-deltaX_hand * rhand_X_world.block<1,6>(2,0) - rhand_X_world.block<1,6>(4,0);
	ContactConstraintMatrix_rh.block<1,6>(8,0)  = 	  	-deltaX_hand * rhand_X_world.block<1,6>(2,0) + rhand_X_world.block<1,6>(4,0);
	ContactConstraintMatrix_rh.block<1,6>(9,0)  = 	  	-deltaY_hand * rhand_X_world.block<1,6>(2,0) - rhand_X_world.block<1,6>(3,0);
	ContactConstraintMatrix_rh.block<1,6>(10,0) = 	  	-deltaY_hand * rhand_X_world.block<1,6>(2,0) + rhand_X_world.block<1,6>(3,0);

	// Inequality constrint vector right foot
	// ContactConstraintVector_rh.setZero();
	ContactConstraintVector_rh(0)  = min_nF_rh; // minimal normal forces
	ContactConstraintVector_rh(11) = max_nF_rh; // maximun normal forces



	return true;
}

//
bool GraspingConstraints::get_WbHandsContactConstraints(Matrix6d world_Xstar_lhand, Matrix6d world_Xstar_rhand)
{
	// 
	bool ok = false;
	//
	ok = this->get_lhand_ContactConstraints(world_Xstar_lhand);
	//
	ok = ok && this->get_rhand_ContactConstraints(world_Xstar_rhand);

	// Overall inequality constraint matrix
	ContactConstraintMatrix_AllHands.block<11,6>(0,0)  = this->ContactConstraintMatrix_lh;
	ContactConstraintMatrix_AllHands.block<11,6>(11,6) = this->ContactConstraintMatrix_rh;

	// Overall Inequality constraint vector
	ContactConstraintVector_AllHands.setZero();
	//
	return ok;
}

//
bool GraspingConstraints::computeBimanualGraspMatrix(Vector7d ObjectPose_world, Vector7d lhandPose_world, Vector7d rhandPose_world, MatrixXd &GraspMxHands_)
{
	//
	GraspMxHands_.setZero();
	//
	Eigen::Vector3d r_o_lh;
	Eigen::Vector3d r_o_rh; 
	r_o_lh = lhandPose_world.segment<3>(0) - ObjectPose_world.segment<3>(0);
	r_o_rh = rhandPose_world.segment<3>(0) - ObjectPose_world.segment<3>(0);
	//
	Eigen::Matrix3d Skew_r_o_lh;
	Skew_r_o_lh <<   	   0.0, -r_o_lh(2),  r_o_lh(1),
				     r_o_lh(2),        0.0, -r_o_lh(0),
				    -r_o_lh(1),  r_o_lh(0),        0.0;

	Eigen::Matrix3d Skew_r_o_rh;
	Skew_r_o_rh <<   	   0.0, -r_o_rh(2),  r_o_rh(1),
				     r_o_rh(2),        0.0, -r_o_rh(0),
				    -r_o_rh(1),  r_o_rh(0),        0.0;
	
	//
	// left hand
	GraspMxHands_.block<3,3>(0,0) = Eigen::MatrixXd::Identity(3, 3);
	GraspMxHands_.block<3,3>(3,0) = -Skew_r_o_lh;
	GraspMxHands_.block<3,3>(3,3) = Eigen::MatrixXd::Identity(3, 3);

	// right hand
	GraspMxHands_.block<3,3>(0,6) = Eigen::MatrixXd::Identity(3, 3);
	GraspMxHands_.block<3,3>(3,6) = -Skew_r_o_rh;
	GraspMxHands_.block<3,3>(3,9) = Eigen::MatrixXd::Identity(3, 3);

	return true;
}


bool GraspingConstraints::getGraspEqualityConstraints(Matrix6d Mo_world,
													 Vector6d bo_world,
													 Eigen::MatrixXd GraspMxHands_,
													 Eigen::MatrixXd Psd_GraspMxHands_,
													 Eigen::MatrixXd dotGraspMxHands_,
													 Eigen::VectorXd Velocity_hands,
													 Eigen::VectorXd ref_object_accel,
													 Eigen::VectorXd f_ext_object,
													 Eigen::MatrixXd &GraspEqualConstraint_matrix_,
													 Eigen::VectorXd &GraspEqualConstraint_vector_)
{
	// 
	GraspEqualConstraint_matrix_.block<6,12>(0,0) = Mo_world*Psd_GraspMxHands_.transpose();
	GraspEqualConstraint_matrix_.block<6,12>(0,12) = -GraspMxHands_;

	GraspEqualConstraint_matrix_.block<12,12>(6,0)  = Eigen::MatrixXd::Identity(12,12) - GraspMxHands_.transpose() * Psd_GraspMxHands_.transpose();
	GraspEqualConstraint_matrix_.block<12,12>(6,24) = Eigen::MatrixXd::Identity(12,12);

	GraspEqualConstraint_matrix_.block<6,12>(18,0) = Psd_GraspMxHands_.transpose();
	GraspEqualConstraint_matrix_.block<6,6>(18,36) = Eigen::MatrixXd::Identity(6,6);


	GraspEqualConstraint_vector_.segment<6>(0)  = Mo_world * (Psd_GraspMxHands_.transpose() * dotGraspMxHands_.transpose() * Psd_GraspMxHands_.transpose()) * Velocity_hands - bo_world + f_ext_object;
	GraspEqualConstraint_vector_.segment<12>(6) = (Eigen::MatrixXd::Identity(12,12) - GraspMxHands_.transpose() * Psd_GraspMxHands_.transpose())*dotGraspMxHands_.transpose() * Psd_GraspMxHands_.transpose() *  Velocity_hands;
	GraspEqualConstraint_vector_.segment<6>(18) = ref_object_accel +(Psd_GraspMxHands_.transpose() * dotGraspMxHands_.transpose() * Psd_GraspMxHands_.transpose()) * Velocity_hands; 

	return true;
}


//
bool GraspingConstraints::getGraspInequalityConstraints(Matrix6d world_Xstar_lhand_,
													   Matrix6d world_Xstar_rhand_,
													   Eigen::MatrixXd &GrasInequalConstraint_matrix,
													   Eigen::VectorXd &GrasInequalConstraint_vector)
{
	//
	get_WbHandsContactConstraints(world_Xstar_lhand_, world_Xstar_rhand_);

	GrasInequalConstraint_matrix.block<11,6>(0,12)  = ContactConstraintMatrix_lh;
	GrasInequalConstraint_matrix.block<11,6>(11,18) = ContactConstraintMatrix_rh;

	//
	GrasInequalConstraint_vector.setZero();

	return true;
} 

// bool GraspingConstraints::get_lhand_ComplementaryConstraints(Matrix6d world_Xstar_deslhand_, Vector7d lhandPose_world_, Vector7d object_lhand_grasp_pose_, double tol_dist2contact)
// {
	
// 	// distance to contact
// 	dist2contact_lh = (object_lhand_grasp_pose_.head<3>()-lhandPose_world_.head<3>()).transpose()*world_Xstar_deslhand_.block<3,1>(0,2);

// 	if(dist2contact_lh <= tol_dist2contact)
// 	{
// 		dist2contact_lh = 0.0;
// 	}
// 	dist2contact_lh = 0.0;

// 	// transformation from the world to desired lhand frame
// 	Matrix6d deslhand_X_world = world_Xstar_deslhand_.inverse();

// 	// Inequality Constraint matrix left foot
// 	ComplementaryConstraintMatrix_lh.block<1,6>(0,0)  = dist2contact_lh * deslhand_X_world.block<1,6>(2,0);
// 	//ComplementaryConstraintMatrix_lh.block<1,6>(0,0)  = 0.005 * deslhand_X_world.block<1,6>(2,0);
// 	cout << " dist2contact_lh is  " << dist2contact_lh << endl;
// }

// bool GraspingConstraints::get_rhand_ComplementaryConstraints(Matrix6d world_Xstar_desrhand_, Vector7d rhandPose_world_, Vector7d object_rhand_grasp_pose_, double tol_dist2contact)
// {
	
// 	// distance to contact
// 	dist2contact_rh = (object_rhand_grasp_pose_.head<3>()-rhandPose_world_.head<3>()).transpose()*world_Xstar_desrhand_.block<3,1>(0,2);

// 	if(dist2contact_rh <= tol_dist2contact)
// 	{
// 		dist2contact_rh = 0.0;
// 	}
// 	dist2contact_rh = 0.0;

// 	// transformation from the world to desired lhand frame
// 	Matrix6d desrhand_X_world = world_Xstar_desrhand_.inverse();

// 		// Inequality Constraint matrix left foot
// 	ComplementaryConstraintMatrix_rh.block<1,6>(0,0)  = dist2contact_rh * desrhand_X_world.block<1,6>(2,0);
// 	//ComplementaryConstraintMatrix_rh.block<1,6>(0,0)  = 0.005 * desrhand_X_world.block<1,6>(2,0);

// 	cout << " dist2contact_rh is  " << dist2contact_rh << endl;
// }


//
// bool GraspingConstraints::get_lhand_Distance2Contact(Matrix6d world_Xstar_deslhand_, Vector7d lhandPose_world_, Vector7d object_lhand_grasp_pose_)
// {
// 	// distance to contact
// 	dist2contact_lh = (object_lhand_grasp_pose_.head<3>()-lhandPose_world_.head<3>()).transpose()*world_Xstar_deslhand_.block<3,1>(0,2);
// 	cout << " dist2contact_lh is  " << dist2contact_lh << endl;
// 	return true;
// }


// bool GraspingConstraints::get_rhand_Distance2Contact(Matrix6d world_Xstar_desrhand_, Vector7d rhandPose_world_, Vector7d object_rhand_grasp_pose_)
// {
// 	// distance to contact
// 	dist2contact_rh = (object_rhand_grasp_pose_.head<3>()-rhandPose_world_.head<3>()).transpose()*world_Xstar_desrhand_.block<3,1>(0,2);
// 	cout << " dist2contact_rh is  " << dist2contact_rh << endl;
// 	return true;
// }


//
bool GraspingConstraints::get_lhand_ComplementaryConstraints(Matrix6d world_Xstar_deslhand_, double dist2contact_lh, double tol_dist2contact)
{
	// thresholding of distance to contact
	double thresh_dist2cnt = dist2contact_lh;
	if(fabs(dist2contact_lh) <= tol_dist2contact) {
		thresh_dist2cnt = 0.0;
	}
	// thresh_dist2cnt = 0.0;
	// transformation from the world to desired lhand frame
	Matrix6d deslhand_X_world = world_Xstar_deslhand_.transpose();
	// Inequality Constraint matrix left foot
	ComplementaryConstraintMatrix_lh.block<1,6>(0,0)  = thresh_dist2cnt * deslhand_X_world.block<1,6>(2,0);

	return true;
}


bool GraspingConstraints::get_rhand_ComplementaryConstraints(Matrix6d world_Xstar_desrhand_, double dist2contact_rh, double tol_dist2contact)
{
	// thresholding of distance to contact
	double thresh_dist2cnt = dist2contact_rh;
	if(fabs(dist2contact_rh) <= tol_dist2contact){
		thresh_dist2cnt = 0.0;
	}
	// thresh_dist2cnt = 0.0;
	// transformation from the world to desired lhand frame
	Matrix6d desrhand_X_world = world_Xstar_desrhand_.inverse();
	// Inequality Constraint matrix left foot
	ComplementaryConstraintMatrix_rh.block<1,6>(0,0)  = thresh_dist2cnt * desrhand_X_world.block<1,6>(2,0);

	return true;
}

//
bool GraspingConstraints::get_hands_ComplementaryConstraints(Matrix6d world_Xstar_deslhand_, Matrix6d world_Xstar_desrhand_, double dist2contact_lh, double dist2contact_rh, double tol_dist2contact)
{
	// thresholding of distance to contact
	double thresh_dist2cnt_l = dist2contact_lh;
	double thresh_dist2cnt_r = dist2contact_rh;

	if((fabs(dist2contact_lh) <= tol_dist2contact) && (fabs(dist2contact_rh) <= tol_dist2contact)) {
		thresh_dist2cnt_l = 0.0;
		thresh_dist2cnt_r = 0.0;
	}
	// thresh_dist2cnt = 0.0;
	// transformation from the world to desired lhand frame
	Matrix6d deslhand_X_world = world_Xstar_deslhand_.transpose();
	Matrix6d desrhand_X_world = world_Xstar_desrhand_.transpose();
	// Inequality Constraint matrix left foot
	ComplementaryConstraintMatrix_lh.block<1,6>(0,0)  = thresh_dist2cnt_l * deslhand_X_world.block<1,6>(2,0);
	ComplementaryConstraintMatrix_rh.block<1,6>(0,0)  = thresh_dist2cnt_r * desrhand_X_world.block<1,6>(2,0);

	return true;
}