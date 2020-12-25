// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/** Class anticipatory_ctrl / .cpp file
	class that that predict the stability of the robot for a given manipulation
	task and accordingly determines or not an anticipatory control action
	(stepping) to prevent the fall of the robot.
*/

#include "anticipatory_ctrl.h"

anticipatory_ctrl::anticipatory_ctrl()
{
	Delat_R_cmp.setZero();
	DeltaZ_com.setZero();
	Wrench_UB.setZero();
	CM.setZero();
	CM_dot.setZero();
	CoM_dot.setZero();
	CP.setZero();
	Pos_eq_foot.setZero();

	Wrench_hands_start.setZero();
	WrenchMap_UP.setZero();

	EE[0] = "left_hand";
	EE[1] = "right_hand";
	EE[2] = "left_foot";
	EE[3] = "right_foot";
	//
	step_values = Eigen::VectorXd::Zero(6);

}


anticipatory_ctrl::~anticipatory_ctrl(){};


void anticipatory_ctrl::Initialize(WbRobotModel& robot_model_, std::string n_data)
{
	//
	alpha_x 	 	= 0.5;
	alpha_y 	 	= 0.5;
	alpha_z 	 	= 0.5;
	a_bi 		 	= 0.5;
	b_bi 		 	= 1.0;
	stepIncrease 	= 0.01;
	lengthStep_max 	= 0.20;
	DyFeet_min  	= 0.145;
	// Dtheta_feet_max = 0.5;
	Dtheta_feet_max = 0.25*M_PI;
	

	PtsInFoot = Eigen::MatrixXd::Zero(2,4);
	// x                // y
    PtsInFoot(0,0) =  0.065;  PtsInFoot(1,0) =  0.015;  // point 1
    PtsInFoot(0,1) = -0.030;  PtsInFoot(1,1) =  0.015;  // point 2
    PtsInFoot(0,2) = -0.030;  PtsInFoot(1,2) = -0.015;  // point 3
    PtsInFoot(0,3) =  0.065;  PtsInFoot(1,3) = -0.015;  // point 4
	//
	nJts = robot_model_.getDoFs();
	//
	massMatrix.resize(nJts+6, nJts+6);
	Jacobian_CM.resize(6, nJts+6);
	for(int i=0; i<4; i++) 
		Jacobian_EE[i].resize(6, nJts+6);

	Jac_2hands	= Eigen::MatrixXd::Zero(12, nJts+6);
	Jac_2feet	= Eigen::MatrixXd::Zero(12, nJts+6);
	Jac_bi		= Eigen::MatrixXd::Zero(12, nJts+6);
	C_hands.setZero();
	//
	jts_pos_star = Eigen::VectorXd::Zero(nJts);
	WHB_star.setIdentity();
	//
	robot_model_.getMassMatrix(jts_pos_star, WHB_star, massMatrix);
	mass_bot = massMatrix(0,0); 	// robot mass
	// q_star   	= Eigen::VectorXd::Zero(nJts+7);
	// q_dot_star  = Eigen::VectorXd::Zero(nJts+6); 	
	// q_ddot_star = Eigen::VectorXd::Zero(nJts+6);  	
	//
	E_Cov_I = Eigen::MatrixXd::Identity(nJts+6, nJts+6);		// Expected Input covariance
	E_Cov_O = Eigen::MatrixXd::Identity(12, 12);			 	// Expected Output covariance
	//
	Weight_feet		= 10. * Eigen::MatrixXd::Identity(12,12);
	invSigma_xidot 	= Eigen::MatrixXd::Identity(12,12); 	
	invSigma_xiddot	= Eigen::MatrixXd::Identity(12,12);		 	
	invSigma_qdot 	= Eigen::MatrixXd::Identity(nJts+6, nJts+6); 	
	invSigma_qddot 	= Eigen::MatrixXd::Identity(nJts+6, nJts+6);
	// AllPtsInAbsFoot.resize(2,np_f*2);
	// AllPtsInAbsFoot.setZero();	
    // string path_log_stability_var   = "log_stability_var_"   + n_data + ".txt";
    // string path_log_PointsCHull     = "log_PointsCHull_"     + n_data + ".txt";
    // string path_log_NormalCHull     = "log_NormalCHull_"     + n_data + ".txt";
    // string path_log_DistanceCHull   = "log_DistanceCHull_"   + n_data + ".txt";
    // //
    // OutRecord_stability_var.open(path_log_stability_var.c_str());
    // OutRecord_PointsCHull.open(path_log_PointsCHull.c_str());
    // OutRecord_NormalCHull.open(path_log_NormalCHull.c_str());
    // OutRecord_DistanceCHull.open(path_log_DistanceCHull.c_str());

    W_EdgesNormalCHull_ = MatrixXd::Zero(7,2);
    DistanceEdgesCHull_ = VectorXd::Zero(7);

    cost_step = 0.0;	

}


bool anticipatory_ctrl::update_model(WbRobotModel& robot_model_, VectorXd jts_pos_star_, Matrix4d WHB_star_)
{
	// Update the robot model
	// ======================
	jts_pos_star  =	jts_pos_star_;
	WHB_star      = WHB_star_;
	//
	bool ok =true;

	ok = ok && robot_model_.getMassMatrix(jts_pos_star_, WHB_star_, massMatrix);
	for(int i=0; i<4; i++)
	{
		ok = ok && robot_model_.getLinkPose(jts_pos_star_, WHB_star_, EE[i],  Pose_EE[i]);
		ok = ok && robot_model_.getJacobian(jts_pos_star_, WHB_star_, EE[i],	Jacobian_EE[i]);
	}
	// CoM
	ok = ok && robot_model_.getLinkPose(  jts_pos_star_, WHB_star_, "CoM",  	    Pose_CoM);
	ok = ok && robot_model_.getJacobian(jts_pos_star_, WHB_star_, "centroidal_momentum",	Jacobian_CM);

	//
	Jac_2hands.topRows(6)    = Jacobian_EE[0];
	Jac_2hands.bottomRows(6) = Jacobian_EE[1];
	Jac_2feet.topRows(6)     = Jacobian_EE[2];
	Jac_2feet.bottomRows(6)  = Jacobian_EE[3];

	// Bimanual transformation
	C_hands.block<6,6>(0,0) =      a_bi * Eigen::MatrixXd::Identity(6,6);
	C_hands.block<6,6>(0,6) = (1.-a_bi) * Eigen::MatrixXd::Identity(6,6);
	C_hands.block<6,6>(6,0) =     -b_bi * Eigen::MatrixXd::Identity(6,6);
	C_hands.block<6,6>(6,6) =             Eigen::MatrixXd::Identity(6,6);
	//
	Jac_bi = C_hands * Jac_2hands;

	// Balance perturbation variables
	// ===============================
	mass_bot = massMatrix(0,0); 	// robot mass
	//
	Pos_eq_foot(0) = alpha_x * Pose_EE[2](0) + (1.-alpha_x) * Pose_EE[3](0);
	Pos_eq_foot(1) = alpha_y * Pose_EE[2](1) + (1.-alpha_y) * Pose_EE[3](1);
	Pos_eq_foot(2) = alpha_x * Pose_EE[2](2) + (1.-alpha_x) * Pose_EE[3](2);
	Pos_eq_foot(3) = alpha_y * Pose_EE[2](2) + (1.-alpha_y) * Pose_EE[3](2);

	// Equivalent CoM height relative to the feet
	DeltaZ_com(0) = Pose_CoM(2) - Pos_eq_foot(2);
	DeltaZ_com(1) = Pose_CoM(2) - Pos_eq_foot(3);

	// Wrench Map 
	Matrix6d WrenchMap_lh, WrenchMap_rh;
	WrenchMap_lh.setIdentity(6,6);	WrenchMap_rh.setIdentity(6,6);
	WrenchMap_lh.block<3,3>(3,0) = Transforms.ComputeSkewSymmetricMatrix(Pose_EE[0].head(3) - Pose_CoM.head(3));
	WrenchMap_rh.block<3,3>(3,0) = Transforms.ComputeSkewSymmetricMatrix(Pose_EE[1].head(3) - Pose_CoM.head(3));
	//
	WrenchMap_UP.leftCols(6)  = WrenchMap_lh;
	WrenchMap_UP.rightCols(6) = WrenchMap_rh;

	// Compute the normals and the distance to normal
	return true;
}

//
bool anticipatory_ctrl::get_instantaneous_IK(WbRobotModel& robot_model_, VectorXd xi_dot_star, VectorXd q_dot_hat, VectorXd xi_ddot_star, VectorXd q_ddot_hat, 
											 VectorXd &q_dot_star, VectorXd &q_ddot_star)
{
	//===========================================================================================================
	// Expected joint velocity
	//===========================================================================================================
	Eigen::MatrixXd J_task_velo = ( Jac_bi.transpose() * invSigma_xidot * Jac_bi + invSigma_qdot 
								  + Jac_2feet.transpose() * Weight_feet * Jac_2feet);

	Eigen::VectorXd task_velo   = (Jac_bi.transpose() * invSigma_xidot * xi_dot_star + invSigma_qdot * q_dot_hat);

	// computation of velocity
	q_dot_star = J_task_velo.colPivHouseholderQr().solve(task_velo);

	//===========================================================================================================
	// Expected joint acceleration
	//===========================================================================================================
	// compute J_dot_q_dot
	// -------------------
	for(int i=0; i<4; i++)  // lhand rhand lfoot rfoot
		robot_model_.getDJdq(jts_pos_star, WHB_star, q_dot_star.tail(nJts), q_dot_star.head(6), EE[i], DJacobianDq_EE[i]);
	// centroidal
	robot_model_.getDJdq(jts_pos_star, WHB_star, q_dot_star.tail(nJts), q_dot_star.head(6), "centroidal_momentum", DJacobianDq_CM);

	Eigen::Matrix<double, 12, 1> DJacobianDq_bi, DJacobianDq_feet; 
	// J_bi_dot_q_dot
	DJacobianDq_bi.head(6) = C_hands.topLeftCorner(6,6)    * DJacobianDq_EE[0] + C_hands.topRightCorner(6,6)   * DJacobianDq_EE[1];	
	DJacobianDq_bi.tail(6) = C_hands.bottomLeftCorner(6,6) * DJacobianDq_EE[0] + C_hands.bottomRightCorner(6,6) * DJacobianDq_EE[1];
	// J_2feet_dot_q_dot
	DJacobianDq_feet.head(6) = DJacobianDq_EE[2];	
	DJacobianDq_feet.tail(6) = DJacobianDq_EE[3];
	//
	Eigen::MatrixXd J_task_accel = (  Jac_bi.transpose() * invSigma_xiddot * Jac_bi + invSigma_qddot 
								   	+ Jac_2feet.transpose() * Weight_feet * Jac_2feet);
	Eigen::VectorXd task_accel   = (  Jac_bi.transpose() * invSigma_xiddot * (xi_ddot_star - DJacobianDq_bi) 
									+ invSigma_qddot * q_ddot_hat - Jac_2feet.transpose() * DJacobianDq_feet);

	//computation of acceleration
	q_ddot_star = J_task_accel.colPivHouseholderQr().solve(task_accel);

	//
	//===========================================================================================================
	// Rate of centroidal momentum
	//===========================================================================================================
	// // 
	// CM_ddot   = Jacobian_CM * q_ddot_star + DJacobianDq_CM;
	// // Expected CoM acceleration
	// CoM_ddot  = 1./mass_bot * CM_ddot.head(3);  

	// // Computation of the Center of Mass
	// CoM_dot   = 1./mass_bot * (Jacobian_CM.topRows(3) * q_dot_star);

	return true;
}

// //
bool anticipatory_ctrl::get_expected_posture(VectorXd Prior, MatrixXd Mean_q, MatrixXd CovMx, VectorXd xi_star_, VectorXd &q_star_, MatrixXd &E_Cov_I, MatrixXd &E_Cov_O)
{
	// Extract the number of Gaussian
	int n_G = Prior.rows();
	// Extract dimension of q_star and xi_star
	
	int n_xi = xi_star_.rows();
	int n_q  = Mean_q.rows() - n_xi;
	q_star_.resize(n_q);
	q_star_.setZero();

	//
	E_Cov_I.resize(n_q, n_q);		    E_Cov_I.setZero();	// Expected Input covariance q
	E_Cov_O.resize(n_xi, n_xi);			E_Cov_O.setZero();  // Expected Output covariance xi
	//
	MatrixXd Cov_I_k(n_q,n_q);
	MatrixXd Cov_O_k(n_xi,n_xi);
	MatrixXd Cov_IO_k(n_q,n_xi);
	MatrixXd Cov_OI_k(n_xi,n_q);

	int dim = n_q+n_xi;
	//
	MatrixXd Cov_k(dim, dim);
	//
	// Computation of h
	VectorXd h = VectorXd::Zero(n_G);
	VectorXd Normal_h = VectorXd::Zero(n_G);
	VectorXd gPDF = VectorXd::Zero(n_G);
	//
	for(int k=0; k<n_G; k++)
	{
		Cov_O_k = CovMx.block(k*dim +n_q,  n_q, n_xi, n_xi);
		//
		gPDF(k) =  this->gaussPDF(xi_star_, Mean_q.col(k).tail(n_xi), Cov_O_k);
		h(k) = Prior(k) * gPDF(k); 
	}

	// cout << " gPDF \n" << gPDF << endl;
	// cout << " h(k) \n" << h << endl;

	Normal_h = h/h.sum();

	// cout << " Normal_h \n" << Normal_h << endl;
	//
	for(int k=0; k<n_G; k++)
	{
		//
		Cov_k 	 = CovMx.block(k*dim, 0, dim, dim);
		//
		Cov_I_k  = Cov_k.block(  0,    0,  n_q, n_q);
		Cov_IO_k = Cov_k.block(  0,  n_q,  n_q, n_xi);    //
		Cov_OI_k = Cov_k.block(n_q,    0, n_xi, n_q);
		Cov_O_k  = Cov_k.block(n_q,  n_q, n_xi, n_xi);
		//
		// Posterior of q
		VectorXd D_xi_mu 			 = xi_star_ - Mean_q.col(k).tail(n_xi);
		MatrixXd Sigma_IO_invSigma_O = Cov_IO_k * Psdinv.get_pseudoInverse(Cov_O_k);

		q_star_ += Normal_h(k)*( Mean_q.col(k).head(n_q) + Sigma_IO_invSigma_O * D_xi_mu );
		// Associated covaraince of Posterior q
		E_Cov_I += Normal_h(k)*Normal_h(k)*(Cov_I_k - Sigma_IO_invSigma_O * Cov_OI_k);
		//
		E_Cov_O += Normal_h(k)*Normal_h(k)*(Cov_O_k);		
	}

	return true;
}
//
bool anticipatory_ctrl::get_expected_posture(VectorXd Prior, MatrixXd Mean_q, MatrixXd CovMx, VectorXd xi_star_, double &prob, VectorXd &q_star_, MatrixXd &E_Cov_I, MatrixXd &E_Cov_O)
{
	// Extract the number of Gaussian
	int n_G = Prior.rows();
	// Extract dimension of q_star and xi_star
	
	int n_xi = xi_star_.rows();
	int n_q  = Mean_q.rows() - n_xi;
	q_star_.resize(n_q);
	q_star_.setZero();

	//
	E_Cov_I.resize(n_q, n_q);		    E_Cov_I.setZero();	// Expected Input covariance q
	E_Cov_O.resize(n_xi, n_xi);			E_Cov_O.setZero();  // Expected Output covariance xi
	//
	MatrixXd Cov_I_k(n_q,n_q);
	MatrixXd Cov_O_k(n_xi,n_xi);
	MatrixXd Cov_IO_k(n_q,n_xi);
	MatrixXd Cov_OI_k(n_xi,n_q);

	int dim = n_q+n_xi;
	//
	MatrixXd Cov_k(dim, dim);
	//
	// Computation of h
	VectorXd h = VectorXd::Zero(n_G);
	VectorXd Normal_h = VectorXd::Zero(n_G);
	//
	for(int k=0; k<n_G; k++)
	{
		Cov_O_k = CovMx.block(k*dim +n_q,  n_q, n_xi, n_xi);
		//
		h(k) = Prior(k) * this->gaussPDF(xi_star_, Mean_q.col(k).tail(n_xi), Cov_O_k);
	}
	// likelihood
	// ================
	prob = h.sum();
	//

	Normal_h = h/h.sum();
	//
	for(int k=0; k<n_G; k++)
	{
		//
		Cov_k 	 = CovMx.block(k*dim, 0, dim, dim);
		//
		Cov_I_k  = Cov_k.block(  0,    0,  n_q, n_q);
		Cov_IO_k = Cov_k.block(  0, n_xi,  n_q, n_xi);
		Cov_OI_k = Cov_k.block(n_q,    0, n_xi, n_q);
		Cov_O_k  = Cov_k.block(n_q,  n_q, n_xi, n_xi);
		//
		// Posterior of q
		VectorXd D_xi_mu 			 = xi_star_ - Mean_q.col(k).tail(n_xi);
		MatrixXd Sigma_IO_invSigma_O = Cov_IO_k * Psdinv.get_pseudoInverse(Cov_O_k);

		q_star_ += Normal_h(k)*( Mean_q.col(k).head(n_q) - Sigma_IO_invSigma_O * D_xi_mu );
		// Associated covaraince of Posterior q
		E_Cov_I += Normal_h(k)*Normal_h(k)*(Cov_I_k - Sigma_IO_invSigma_O * Cov_OI_k);
		//
		E_Cov_O += Normal_h(k)*Normal_h(k)*(Cov_O_k);		
	}

	return true;
}


bool anticipatory_ctrl::get_centroidal_variables(WbRobotModel& robot_model_, VectorXd jts_pos_star_, Matrix4d WHB_star_, VectorXd q_dot_star_, VectorXd q_ddot_star_)
{
	// centroidal
	robot_model_.getDJdq(jts_pos_star_, WHB_star_, q_dot_star_.tail(nJts), q_dot_star_.head(6), "centroidal_momentum", DJacobianDq_CM);
	//
	// Rate of centroidal momentum
	CM_dot    = Jacobian_CM * q_ddot_star_ + DJacobianDq_CM;
	CoM_ddot  = 1./mass_bot * CM_dot.head(3);
	//
	return true;
}

// balance perturbation
Vector2d anticipatory_ctrl::compute_posture_perturbation(VectorXd Wrench_hands_star_)
{
	
	// Upper body wrench about the CoM 
	Wrench_UB = WrenchMap_UP * Wrench_hands_star_;
	// 
	Beta_w = (1. + CoM_ddot(2)/9.81 - Wrench_UB(2)/(mass_bot * 9.81));
	//
	double mgBetaW = mass_bot * 9.81 * Beta_w * Beta_w;
	//
	Delat_R_cmp(0) = 1./(mgBetaW) * (-Wrench_UB(0) * DeltaZ_com(0) - Wrench_UB(4) + CM_dot(4));
	Delat_R_cmp(1) = 1./(mgBetaW) * (-Wrench_UB(1) * DeltaZ_com(1) + Wrench_UB(3) - CM_dot(3));
	//
	return Delat_R_cmp;
}

// balance perturbation
Vector2d anticipatory_ctrl::compute_posture_perturbation2(double Beta_w_, Vector6d Wrench_UB_)
{
	//
	double mgBetaW = mass_bot * 9.81 * Beta_w_ * Beta_w_;
	//
	Vector2d BalPerturb_= VectorXd::Zero(2);
	BalPerturb_(0) = 1./(mgBetaW) * (-Wrench_UB_(0) * DeltaZ_com(0) - Wrench_UB_(4) + CM_dot(4));
	BalPerturb_(1) = 1./(mgBetaW) * (-Wrench_UB_(1) * DeltaZ_com(1) + Wrench_UB_(3) - CM_dot(3));
	//
	return BalPerturb_;
}

// balance  disturbance variation due to steping
bool anticipatory_ctrl::compute_delta_disturb(VectorXd Wrench_hands_start, Vector2d DeltaCOM_, Vector2d &DeltaDist_)
{
	//
	Vector3d D_CoM3d(	DeltaCOM_(0), DeltaCOM_(1), 0.0);

	Vector3d f_ub(	Wrench_hands_start(0)+Wrench_hands_start(0+6),  
					Wrench_hands_start(1)+Wrench_hands_start(1+6), 
					Wrench_hands_start(2)+Wrench_hands_start(2+6));

	Vector3d DTau_ub = -D_CoM3d.cross(f_ub);

	Beta_w = (1. + CoM_ddot(2)/9.81 - Wrench_UB(2)/(mass_bot * 9.81));
	double mgBetaW = mass_bot * 9.81 * Beta_w * Beta_w;

	DeltaDist_(0) = -1./(mgBetaW) * DTau_ub(1);
	DeltaDist_(1) =  1./(mgBetaW) * DTau_ub(0);
	
	return true;
}


Vector2d anticipatory_ctrl::compute_capture_point(VectorXd Wrench_hands_star_)
{
	
	// Upper body wrench about the CoM 
	Wrench_UB = WrenchMap_UP * Wrench_hands_star_;
	// 
	Beta_w = (1. + CoM_ddot(2)/9.81 - Wrench_UB(2)/(mass_bot * 9.81));
	//
	double Omega = sqrt(9.81/0.5*(DeltaZ_com(0) + DeltaZ_com(1)));
	//
	double mgBetaW = mass_bot * 9.81 * Beta_w * Beta_w;
	// Computation of the capture point
	CP(0) = Pose_CoM(0) + CoM_dot(0)/ (Omega * Beta_w*Beta_w);
	CP(1) = Pose_CoM(1) + CoM_dot(1)/ (Omega * Beta_w*Beta_w);
	//

	return CP.head(2);
}

Vector3d anticipatory_ctrl::compute_capture_point2(double Beta_w_, Vector3d pCoM, Vector3d vCoM)
{
	//
	double Omega = sqrt( 9.81/(0.5*pCoM(2)) );
	double mgBetaW = mass_bot * 9.81 * Beta_w_*Beta_w_;
	//
	Vector3d CP_ = VectorXd::Zero(3);
	// Computation of the capture point
	CP_(0) = pCoM(0) + vCoM(0)/ (Omega * Beta_w_*Beta_w_);
	CP_(1) = pCoM(1) + vCoM(1)/ (Omega * Beta_w_*Beta_w_);
	CP_(2) = pCoM(2) + vCoM(2)/ (Omega * Beta_w_*Beta_w_);
	//
	return CP_;
}

void anticipatory_ctrl::get_feet_support_points(Vector7d Pose_lfoot, Vector7d Pose_rfoot, Vector2d DeltaStep,
												MatrixXd &AllPtsInAbsFoot_, Matrix2d &W_Rot_AbsFoot, Vector2d &W_Pos_AbsFoot)
{
    //
    // 3D rotation matrix of the feet wrt the world frame
    Eigen::Matrix3d WR_lf = Transforms.PoseVector2HomogenousMx(Pose_lfoot).block(0,0,3,3);
    Eigen::Matrix3d WR_rf = Transforms.PoseVector2HomogenousMx(Pose_rfoot).block(0,0,3,3);
    // Planar orientations of the feet 
    Vector3d o_lf = Transforms.getEulerAnglesXYZ_FixedFrame(WR_lf);
    Vector3d o_rf = Transforms.getEulerAnglesXYZ_FixedFrame(WR_rf);
    //
    Matrix2d 	rot_robot;
    			rot_robot << cos(0.5*(o_lf(2)+o_rf(2))), -sin(0.5*(o_lf(2)+o_rf(2))), sin(0.5*(o_lf(2)+o_rf(2))), cos(0.5*(o_lf(2)+o_rf(2)));
    // Perturbation expressed relative to the robot frame to help choose the stance foot
    Vector2d r_D_rcmp = rot_robot.transpose() * Delat_R_cmp;

    // determine the stance based on the direction of the balance perturbation
    // string stance;
    //
    if     ((r_D_rcmp(0)>=0) && (r_D_rcmp(1)>=0))     stance_0 = "right";    //
    else if((r_D_rcmp(0)>=0) && (r_D_rcmp(1)<0))      stance_0 = "left";
    else if((r_D_rcmp(0)<0)  && (r_D_rcmp(1)>=0))     stance_0 = "right";
    else if((r_D_rcmp(0)<0)  && (r_D_rcmp(1)<0))      stance_0 = "left";
    else                                              stance_0 = "right";

    // std::cout << " stance_0 is : \t" << stance_0 << std::endl;
    // std::cout << " r_D_rcmp(0) is : \t" << r_D_rcmp(0) << "  r_D_rcmp(1) is : \t" << r_D_rcmp(1) << std::endl;

    // direction of the balance disturbance
    double  theta_Dist = atan2(Delat_R_cmp(1), Delat_R_cmp(0));
    //
    int np_f = PtsInFoot.cols();        // number of points in one foot support polygon

    AllPtsInAbsFoot_.resize(2,np_f*2); // coordinates of the point in the absolute frame
    AllPtsInAbsFoot_.setZero();

    MatrixXd W_PtsSwingFt(2,np_f);      // Matrix of points in the swing foot (foot that will perform the stepping)
    MatrixXd W_PtsStanceFt(2,np_f);     // Matrix of points in the stance foot (foot that stays fixed)

    MatrixXd RotStanceFt(2,2);          // Plananr Rotation matrix of the stance foot wrt the the world frane
    Vector2d TransStanceFt(2);          // Plananr Position vector of the stance foot wrt the world frame

    MatrixXd RotSwingFt(2,2);           // Plananr Rotation matrix of the swing foot wrt the the world frane
    Vector2d TransSwingFt(2);           // Plananr Position vector of the swing foot wrt the world frame

    Matrix2d AbsFoot_Rot_W;				// Rotation of the world frame relatibve to the absolute feet frame
	Vector2d AbsFoot_Trans_W;			// position of the world frame relatibve to the absolute feet frame

    //
    double DthetaFt = 0.0;              // Relative angle between stance and swing foot

    // step length
    Vector2d 	StepFeet;
    			StepFeet << Pose_lfoot(0) - Pose_rfoot(0), Pose_lfoot(1) - (Pose_rfoot(1)+ this->DyFeet_min);
    lengthStep = StepFeet.norm();

   
    if (stance_0 =="left")  // foot
    {
        stance_1 = "right";
        // Extraction of the feet orientation
        theta_stance = o_lf(2); //Transforms.getEulerAnglesXYZ_FixedFrame(WR_lf)(2);
        // computing the swing orientation to be executed
        DthetaFt = theta_Dist - theta_stance;
        if((-Dtheta_feet_max <= DthetaFt) || (DthetaFt <= Dtheta_feet_max))
            theta_swing = theta_Dist;
        else 
            theta_swing = theta_stance;

        // Planar rotations of the feet
        RotStanceFt << cos(theta_stance), -sin(theta_stance), sin(theta_stance), cos(theta_stance);
        RotSwingFt  << cos(theta_swing),  -sin(theta_swing),  sin(theta_swing),  cos(theta_swing);
        // Planar rotation of the equivalent foot frame (absolute frame)
        AbsFoot_Rot_W = RotStanceFt.transpose();
        // Planar translations of the feet
        TransStanceFt = Pose_lfoot.head(2);
        TransSwingFt  = Pose_rfoot.head(2) + DeltaStep;
        //
        // StepFeet << TransStanceFt(0) - TransSwingFt(0), TransStanceFt(1) - (TransSwingFt(1)+ this->DyFeet_min);
        StepFeet << TransSwingFt(0) - TransStanceFt(0), (TransSwingFt(1)+ this->DyFeet_min) - TransStanceFt(1);
        lengthStep = StepFeet.norm(); 
        
        // Expression of pts from foot frame to world frame
        for (int i=0; i<np_f; i++)   {
            W_PtsStanceFt.col(i) = RotStanceFt * PtsInFoot.col(i) + TransStanceFt;
            W_PtsSwingFt.col(i)  = RotSwingFt  * PtsInFoot.col(i) + TransSwingFt;
        }
    } 
    else // stance == "right" foot
    {
        stance_1 = "left";
        // Extraction of the feet orientation
        theta_stance = o_rf(2); //Transforms.getEulerAnglesXYZ_FixedFrame(WR_rf).(2);
        // computing the swing orientation to be executed
        DthetaFt = theta_Dist - theta_stance;
        if((-Dtheta_feet_max <= DthetaFt) || (DthetaFt <= Dtheta_feet_max))
            theta_swing = theta_Dist;
        else 
            theta_swing = theta_stance;
        
        // Planar rotations of the feet
        RotStanceFt << cos(theta_stance), -sin(theta_stance), sin(theta_stance), cos(theta_stance);
        RotSwingFt  << cos(theta_swing),  -sin(theta_swing),  sin(theta_swing),  cos(theta_swing);
        // Planar rotation of the equivalent foot frame (absolute frame)
        AbsFoot_Rot_W = RotStanceFt.transpose();
        // Planar translations of the feet
        TransStanceFt = Pose_rfoot.head(2);
        TransSwingFt  = Pose_lfoot.head(2) + DeltaStep;
        //
        StepFeet << TransSwingFt(0) - TransStanceFt(0), TransSwingFt(1) - (TransStanceFt(1)+ this->DyFeet_min);
        lengthStep = StepFeet.norm();
        
        // Expression of pts from foot frame to world frame
        for (int i=0; i<np_f; i++)   {
            W_PtsStanceFt.col(i) = RotStanceFt * PtsInFoot.col(i) + TransStanceFt;
            W_PtsSwingFt.col(i)  = RotSwingFt  * PtsInFoot.col(i) + TransSwingFt;
        }
    }

    // Expression of the feet polygon points in the absolute foot frame and 
    MatrixXd W_All_Pts(2,np_f*2);
    W_All_Pts.leftCols(np_f)  = W_PtsStanceFt;
    W_All_Pts.rightCols(np_f) = W_PtsSwingFt;

    // Get the position of the absolute foot frame
    AbsFoot_Trans_W = -0.5 * (TransStanceFt + TransSwingFt);

    for (int i=0; i<np_f*2; i++) 
    	AllPtsInAbsFoot_.col(i) = AbsFoot_Rot_W * W_All_Pts.col(i) + AbsFoot_Trans_W;  // Transformation
    //
    W_Rot_AbsFoot =  AbsFoot_Rot_W.transpose();
    W_Pos_AbsFoot = -AbsFoot_Trans_W;
    //
    Delta_theta_feet = theta_swing - theta_stance;

    // cout << " W_All_Pts second 1 \n" << W_All_Pts << endl;

    //
    return;
}

// support polygon points with joints feet after the taking a first step
void anticipatory_ctrl::getJointFeetSsupportPoints_2(	string stance_1, double theta_swing_0, double des_DyFeet, 
													Vector7d Pose_lfoot, Vector7d Pose_rfoot, Vector2d DeltaStep, 
												    MatrixXd &AllPtsInAbsFoot_, Matrix2d &W_Rot_AbsFoot, Vector2d &W_Pos_AbsFoot)
{
    //
    int np_f = PtsInFoot.cols();        // number of points in one foot support polygon

    AllPtsInAbsFoot_.resize(2,np_f*2); // coordinates of the point in the absolute frame
    AllPtsInAbsFoot_.setZero();

    MatrixXd W_PtsSwingFt(2,np_f);      // Matrix of points in the swing foot (foot that will perform the stepping)
    MatrixXd W_PtsStanceFt(2,np_f);     // Matrix of points in the stance foot (foot that stays fixed)

    MatrixXd RotStanceFt(2,2);          // Plananr Rotation matrix of the stance foot wrt the the world frane
    Vector2d TransStanceFt(2);          // Plananr Position vector of the stance foot wrt the world frame

    MatrixXd RotSwingFt(2,2);           // Plananr Rotation matrix of the swing foot wrt the the world frane
    Vector2d TransSwingFt(2);           // Plananr Position vector of the swing foot wrt the world frame

    Matrix2d AbsFoot_Rot_W;				// Rotation of the world frame relatibve to the absolute feet frame
	Vector2d AbsFoot_Trans_W;			// position of the world frame relatibve to the absolute feet frame

    //
    double DthetaFt = 0.0;              // Relative angle between stance and swing foot

    // Extraction of the feet orientation
    theta_stance = theta_swing_0;  
    // Planar rotations of the feet
    RotStanceFt << cos(theta_stance), -sin(theta_stance), sin(theta_stance), cos(theta_stance);
    RotSwingFt  = RotStanceFt;
    // Planar rotation of the equivalent foot frame (absolute frame)
    AbsFoot_Rot_W = RotStanceFt.transpose();

    //
     // step length
    Vector2d 	StepFeet;
    			StepFeet << Pose_lfoot(0) - Pose_rfoot(0), Pose_lfoot(1) - (Pose_rfoot(1)+ this->DyFeet_min);
    lengthStep = StepFeet.norm();

    // desired transformation of the swing foot wrt to the stance foot
    Matrix3d des_st_H_sw = MatrixXd::Identity(3,3);

    if (stance_1 =="left")  // foot
    {
        // desired transformation of the swing foot wrt to the stance foot
        des_st_H_sw(0,2) = 0.0;
        des_st_H_sw(1,2) = -des_DyFeet;

        // Planar translations of the feet
        TransStanceFt = Pose_lfoot.head(2) + DeltaStep;
        TransSwingFt  = RotStanceFt * des_st_H_sw.block<2,1>(0,2) +  TransStanceFt; 
        //
        StepFeet << TransStanceFt(0) - TransSwingFt(0), TransStanceFt(1) - (TransSwingFt(1)+ this->DyFeet_min);
        lengthStep = StepFeet.norm(); 
        
        // Expression of pts from foot frame to world frame
        for (int i=0; i<np_f; i++)   {
            W_PtsStanceFt.col(i) = RotStanceFt * PtsInFoot.col(i) + TransStanceFt;
            W_PtsSwingFt.col(i)  = RotSwingFt  * PtsInFoot.col(i) + TransSwingFt;
        }
    } 
    else // stance == "right" foot
    {
        // desired transformation of the swing foot wrt to the stance foot
        des_st_H_sw(0,2) = 0.0;
        des_st_H_sw(1,2) = des_DyFeet;

        // Planar translations of the feet
        TransStanceFt = Pose_rfoot.head(2)+ DeltaStep;
        TransSwingFt  = RotStanceFt * des_st_H_sw.block<2,1>(0,2) +  TransStanceFt;
        //
        StepFeet << TransSwingFt(0) - TransStanceFt(0), TransSwingFt(1) - (TransStanceFt(1)+ this->DyFeet_min);
        lengthStep = StepFeet.norm();
        
        // Expression of pts from foot frame to world frame
        for (int i=0; i<np_f; i++)   {
            W_PtsStanceFt.col(i) = RotStanceFt * PtsInFoot.col(i) + TransStanceFt;
            W_PtsSwingFt.col(i)  = RotSwingFt  * PtsInFoot.col(i) + TransSwingFt;
        }
    }

    // Expression of the feet polygon points in the absolute foot frame and 
    MatrixXd W_All_Pts(2,np_f*2);
    W_All_Pts.leftCols(np_f)  = W_PtsStanceFt;
    W_All_Pts.rightCols(np_f) = W_PtsSwingFt;

    // Get the position of the absolute foot frame
    AbsFoot_Trans_W = -0.5 * (TransStanceFt + TransSwingFt);

    for (int i=0; i<np_f*2; i++) 
    	AllPtsInAbsFoot_.col(i) = AbsFoot_Rot_W * W_All_Pts.col(i) + AbsFoot_Trans_W;  // Transformation
    //
    W_Rot_AbsFoot =  AbsFoot_Rot_W.transpose();
    W_Pos_AbsFoot = -AbsFoot_Trans_W;


    // cout << " W_All_Pts second 2 \n" << W_All_Pts << endl;
    //
    return;

}

void anticipatory_ctrl::get_feet_support_points_C(	Vector7d Pose_lfoot, Vector7d Pose_rfoot, Vector2d DeltaStep, double DeltaThetaFeet_,
													MatrixXd &AllPtsInAbsFoot_, Matrix2d &W_Rot_AbsFoot, Vector2d &W_Pos_AbsFoot)
{
    //
    // 3D rotation matrix of the feet wrt the world frame
    Eigen::Matrix3d WR_lf = Transforms.PoseVector2HomogenousMx(Pose_lfoot).block(0,0,3,3);
    Eigen::Matrix3d WR_rf = Transforms.PoseVector2HomogenousMx(Pose_rfoot).block(0,0,3,3);
    // Planar orientations of the feet 
    Vector3d o_lf = Transforms.getEulerAnglesXYZ_FixedFrame(WR_lf);
    Vector3d o_rf = Transforms.getEulerAnglesXYZ_FixedFrame(WR_rf);
    //
    Matrix2d 	rot_robot;
    			rot_robot << cos(0.5*(o_lf(2)+o_rf(2))), -sin(0.5*(o_lf(2)+o_rf(2))), sin(0.5*(o_lf(2)+o_rf(2))), cos(0.5*(o_lf(2)+o_rf(2)));
    // Perturbation expressed relative to the robot frame to help choose the stance foot
    Vector2d r_D_rcmp = rot_robot.transpose() * Delat_R_cmp;

    // determine the stance based on the direction of the balance perturbation
    // string stance;
    //
    if     ((r_D_rcmp(0)>=0) && (r_D_rcmp(1)>=0))     stance_0 = "right";    //
    else if((r_D_rcmp(0)>=0) && (r_D_rcmp(1)<0))      stance_0 = "left";
    else if((r_D_rcmp(0)<0)  && (r_D_rcmp(1)>=0))     stance_0 = "right";
    else if((r_D_rcmp(0)<0)  && (r_D_rcmp(1)<0))      stance_0 = "left";
    else                                              stance_0 = "right";

    // direction of the balance disturbance
    double  theta_Dist = atan2(Delat_R_cmp(1), Delat_R_cmp(0));
    //
    int np_f = PtsInFoot.cols();        // number of points in one foot support polygon

    AllPtsInAbsFoot_.resize(2,np_f*2); // coordinates of the point in the absolute frame
    AllPtsInAbsFoot_.setZero();

    MatrixXd W_PtsSwingFt(2,np_f);      // Matrix of points in the swing foot (foot that will perform the stepping)
    MatrixXd W_PtsStanceFt(2,np_f);     // Matrix of points in the stance foot (foot that stays fixed)

    MatrixXd RotStanceFt(2,2);          // Plananr Rotation matrix of the stance foot wrt the the world frane
    Vector2d TransStanceFt(2);          // Plananr Position vector of the stance foot wrt the world frame

    MatrixXd RotSwingFt(2,2);           // Plananr Rotation matrix of the swing foot wrt the the world frane
    Vector2d TransSwingFt(2);           // Plananr Position vector of the swing foot wrt the world frame

    Matrix2d AbsFoot_Rot_W;				// Rotation of the world frame relatibve to the absolute feet frame
	Vector2d AbsFoot_Trans_W;			// position of the world frame relatibve to the absolute feet frame

    //
    double DthetaFt = 0.0;              // Relative angle between stance and swing foot

    // step length
    Vector2d 	StepFeet;
    			StepFeet << Pose_lfoot(0) - Pose_rfoot(0), Pose_lfoot(1) - (Pose_rfoot(1)+ this->DyFeet_min);
    lengthStep = StepFeet.norm();

   
    if (stance_0 =="left")  // foot
    {
        stance_1 = "right";
        // Extraction of the feet orientation
        theta_stance = o_lf(2); //Transforms.getEulerAnglesXYZ_FixedFrame(WR_lf).(2);
        // computing the swing orientation to be executed
        theta_swing  = theta_stance + DeltaThetaFeet_;
        // DthetaFt = theta_Dist - theta_stance;
        // if((-Dtheta_feet_max <= DthetaFt) || (DthetaFt <= Dtheta_feet_max))
        //     theta_swing = theta_Dist;
        // else 
        //     theta_swing = theta_stance;

        // Planar rotations of the feet
        RotStanceFt << cos(theta_stance), -sin(theta_stance), sin(theta_stance), cos(theta_stance);
        RotSwingFt  << cos(theta_swing),  -sin(theta_swing),  sin(theta_swing),  cos(theta_swing);
        // Planar rotation of the equivalent foot frame (absolute frame)
        AbsFoot_Rot_W = RotStanceFt.transpose();
        // Planar translations of the feet
        TransStanceFt = Pose_lfoot.head(2);
        TransSwingFt  = Pose_rfoot.head(2) + DeltaStep;
        //
        // StepFeet << TransStanceFt(0) - TransSwingFt(0), TransStanceFt(1) - (TransSwingFt(1)+ this->DyFeet_min);
        StepFeet << TransSwingFt(0) - TransStanceFt(0), (TransSwingFt(1)+ this->DyFeet_min) - TransStanceFt(1);
        lengthStep = StepFeet.norm(); 
        
        // Expression of pts from foot frame to world frame
        for (int i=0; i<np_f; i++)   {
            W_PtsStanceFt.col(i) = RotStanceFt * PtsInFoot.col(i) + TransStanceFt;
            W_PtsSwingFt.col(i)  = RotSwingFt  * PtsInFoot.col(i) + TransSwingFt;
        }
    } 
    else // stance == "right" foot
    {
        stance_1 = "left";
        // Extraction of the feet orientation
        theta_stance = o_rf(2); //Transforms.getEulerAnglesXYZ_FixedFrame(WR_rf).(2);
        // computing the swing orientation to be executed
        theta_swing  = theta_stance + DeltaThetaFeet_;
        // DthetaFt = theta_Dist - theta_stance;
        // if((-Dtheta_feet_max <= DthetaFt) || (DthetaFt <= Dtheta_feet_max))
        //     theta_swing = theta_Dist;
        // else 
        //     theta_swing = theta_stance;
        
        // Planar rotations of the feet
        RotStanceFt << cos(theta_stance), -sin(theta_stance), sin(theta_stance), cos(theta_stance);
        RotSwingFt  << cos(theta_swing),  -sin(theta_swing),  sin(theta_swing),  cos(theta_swing);
        // Planar rotation of the equivalent foot frame (absolute frame)
        AbsFoot_Rot_W = RotStanceFt.transpose();
        // Planar translations of the feet
        TransStanceFt = Pose_rfoot.head(2);
        TransSwingFt  = Pose_lfoot.head(2) + DeltaStep;
        //
        StepFeet << TransSwingFt(0) - TransStanceFt(0), TransSwingFt(1) - (TransStanceFt(1)+ this->DyFeet_min);
        lengthStep = StepFeet.norm();
        
        // Expression of pts from foot frame to world frame
        for (int i=0; i<np_f; i++)   {
            W_PtsStanceFt.col(i) = RotStanceFt * PtsInFoot.col(i) + TransStanceFt;
            W_PtsSwingFt.col(i)  = RotSwingFt  * PtsInFoot.col(i) + TransSwingFt;
        }
    }

    // Expression of the feet polygon points in the absolute foot frame and 
    MatrixXd W_All_Pts(2,np_f*2);
    W_All_Pts.leftCols(np_f)  = W_PtsStanceFt;
    W_All_Pts.rightCols(np_f) = W_PtsSwingFt;

    // Get the position of the absolute foot frame
    AbsFoot_Trans_W = -0.5 * (TransStanceFt + TransSwingFt);

    for (int i=0; i<np_f*2; i++) 
    	AllPtsInAbsFoot_.col(i) = AbsFoot_Rot_W * W_All_Pts.col(i) + AbsFoot_Trans_W;  // Transformation
    //
    W_Rot_AbsFoot =  AbsFoot_Rot_W.transpose();
    W_Pos_AbsFoot = -AbsFoot_Trans_W;
    //
    Delta_theta_feet = theta_swing - theta_stance;

    // cout << " W_All_Pts second 1 \n" << W_All_Pts << endl;

    //
    return;
}

// Stability condition
bool anticipatory_ctrl::predict_stability(MatrixXd W_EdgesNormalCHull_, VectorXd DistanceEdgesCHull_, Vector2d W_CP_, Vector2d W_Pos_AbsFt_, Vector2d W_Pertub_)
{
	bool result = true;

	int nCHPts = W_EdgesNormalCHull_.rows();

	std::cout << " CAPTURE POINT IS : \t" << W_CP_.transpose() << std::endl;
	std::cout << " ABSOLUTE FEET POSITION IS : \t" << W_Pos_AbsFt_.transpose() << std::endl;

	for(int i=0; i<nCHPts; i++)
		result = result && (W_EdgesNormalCHull_.row(i) * (W_CP_ - W_Pos_AbsFt_ - W_Pertub_) <= DistanceEdgesCHull_(i));

	return result;
}

bool anticipatory_ctrl::predict_stabilityCP(MatrixXd W_EdgesNormalCHull_, VectorXd DistanceEdgesCHull_, Vector2d W_CP_, Vector2d W_Pos_AbsFt_, Vector2d W_Pertub_)
{
	return this->predict_stability(W_EdgesNormalCHull_, DistanceEdgesCHull_, W_CP_, W_Pos_AbsFt_, W_Pertub_);
}


bool anticipatory_ctrl::predict_stabilityZMP(MatrixXd W_EdgesNormalCHull_, VectorXd DistanceEdgesCHull_, Vector3d pCoM, Vector3d aCoM, Vector2d W_Pos_AbsFt_, Vector2d W_Pertub_, double beta_w_)
{ 
	//
	bool result = true;

	int nCHPts = W_EdgesNormalCHull_.rows();

	for(int i=0; i<nCHPts; i++)
		result = result && (W_EdgesNormalCHull_.row(i) * (pCoM.head(2) - W_Pos_AbsFt_ - 1.0*(pCoM(2)/(9.81*beta_w_*beta_w_))*aCoM.head(2) - W_Pertub_) <= DistanceEdgesCHull_(i));

	return result;
}

// // compute the anticipative step length
Vector6d anticipatory_ctrl::compute_stepping_values(Vector7d Pose_lfoot, Vector7d Pose_rfoot, VectorXd Wrench_hands_star)
{
	bool stable_0 = false;
	bool stable_1 = false;

	MatrixXd AllPtsInAbsFoot;
	MatrixXd PointsCHull;
	VectorXd DistanceEdgesCHull;
	MatrixXd Abs_EdgesNormalCHull;
	MatrixXd W_EdgesNormalCHull;
	Matrix2d W_Rot_AbsFoot;
	Vector2d W_Pos_AbsFoot;
	Vector2d W_Disturb;
	Vector2d W_CP_(CP(0), CP(1));

	double DeltaStepLength = 0;
	// step increase
	// direction of the balance disturbance
    double  theta_Dist = atan2(this->Delat_R_cmp(1), this->Delat_R_cmp(0));
    Vector2d DeltaStep(DeltaStepLength * cos(theta_Dist), DeltaStepLength * sin(theta_Dist));
    Vector2d DeltaDisturb(0.0, 0.0);
    //
	int count = 0;

	// while((stable_0 == true)||(lengthStep <= lengthStep_max))
	while((!stable_0) && (lengthStep <= lengthStep_max))
	{
		
		// update step
		DeltaStepLength = double(count) * this->stepIncrease;
		DeltaStep << DeltaStepLength * cos(theta_Dist), DeltaStepLength * sin(theta_Dist);

		// update the balance perturbation
		// --------------------------------
		// compute the variation of the perturbation due to the stepping
		this->compute_delta_disturb(Wrench_hands_star, 0.5*DeltaStep, DeltaDisturb);

		// updating the Perturbation
		W_Disturb = Delat_R_cmp + DeltaDisturb;

		// update capture point and absolute frame
		W_CP_     = CP.head(2)  + 0.5 * DeltaStep;
		// get the points defining the support polygon of each feet
		this->get_feet_support_points(Pose_lfoot, Pose_rfoot, DeltaStep, AllPtsInAbsFoot, W_Rot_AbsFoot, W_Pos_AbsFoot);  // -->> AllPtsInAbsFoot, W_Rot_AbsFoot, W_Pos_AbsFoot
		// update convex hull
		ConvHull.convexHull(AllPtsInAbsFoot, PointsCHull, Abs_EdgesNormalCHull, DistanceEdgesCHull); // --> PointsCHull, EdgesNormalCHull, DistanceEdgesCHull
	
		// Express the normal in the world frame
		W_EdgesNormalCHull = Abs_EdgesNormalCHull * W_Rot_AbsFoot.transpose();

		// predict the stability
		stable_0 = this->predict_stability(W_EdgesNormalCHull, DistanceEdgesCHull, W_CP_, W_Pos_AbsFoot, W_Disturb);
		//
		// cout << " Delat_R_cmp     \t" << Delat_R_cmp.transpose() << endl;
		// cout << " theta_Dist      \t" << theta_Dist << endl;
		// cout << " DeltaStepLength \t" << DeltaStepLength << endl;
		
		//
		// cout << "count  \n" <<  count << endl; 
		// cout << "PointsCHull  \n" <<  PointsCHull << endl; 
		// cout << "Abs_EdgesNormalCHull  \n" <<  Abs_EdgesNormalCHull << endl; 
		// cout << "DistanceEdgesCHull  \n" <<  DistanceEdgesCHull << endl; 
		// cout << "W_EdgesNormalCHull  \n" <<  W_EdgesNormalCHull << endl; 
		// cout << " DeltaStep       \t" << DeltaStep.transpose() << endl;
		// cout << " lengthStep       \t" << lengthStep << endl;
		
		cout << "DeltaDisturb  \t" <<  DeltaDisturb.transpose() << endl; 
		cout << "W_Disturb  \t" <<  W_Disturb.transpose() << endl; 
		cout << "stable_0  \t" <<  stable_0 << endl;  

		cout << "PointsCHull  \n" <<  PointsCHull << endl; 
		cout << "SIZE OF  PointsCHull \t" <<  PointsCHull.rows() << " and \t" <<  PointsCHull.cols() << endl; 

		//
		// *****************************************************************************************
	    // Record the data Poses and joints configuration
	    // *****************************************************************************************
	    // int stance_left = 0; 
	    // if(stance_0 =="left") stance_left = 1;
	    // OutRecord_stability_var << count << "  " << W_Disturb.transpose() << "  " << W_CP_.transpose() << "  " << W_Pos_AbsFoot.transpose() <<  "  ";
	    // OutRecord_stability_var << W_Rot_AbsFoot.row(0) << "  " << W_Rot_AbsFoot.row(1) <<  "  ";
	    // OutRecord_stability_var << DeltaStep.transpose() << "  " << Delta_theta_feet << "  " << stable_0 << "  " << stance_left << "  " << 0.0 << endl;
	    
	    // if(PointsCHull.rows() == 5)
	    // {
	    // 	OutRecord_PointsCHull 	<< PointsCHull.col(0).transpose() << "  " << PointsCHull(0,0) << "  " << PointsCHull(0,0) << "  ";   // x
	    // 	OutRecord_PointsCHull 	<< PointsCHull.col(1).transpose() << "  " << PointsCHull(0,1) << "  " << PointsCHull(0,1) << endl;   // y

		   //  OutRecord_NormalCHull  	<< W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull(0,0) << "  " << W_EdgesNormalCHull(0,0) << "  ";   // x
		   //  OutRecord_NormalCHull  	<< W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull(0,1) << "  " << W_EdgesNormalCHull(0,1) << endl;	  // y

		   //  OutRecord_DistanceCHull << DistanceEdgesCHull.transpose() << "  " << DistanceEdgesCHull(0) << "  " << DistanceEdgesCHull(0) << endl;	
	    // }
	    // else if(PointsCHull.rows() == 6)
	    // {
	    // 	OutRecord_PointsCHull 	<< PointsCHull.col(0).transpose() << "  " << PointsCHull(0,0) << "  " ;   // x
	    // 	OutRecord_PointsCHull 	<< PointsCHull.col(1).transpose() << "  " << PointsCHull(0,1) << endl;   // y

		   //  OutRecord_NormalCHull  	<< W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull(0,0) << "  ";   // x
		   //  OutRecord_NormalCHull  	<< W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull(0,1) << endl;	  // y

		   //  OutRecord_DistanceCHull << DistanceEdgesCHull.transpose() << "  " << DistanceEdgesCHull(0) << endl;
	    // }
	    // else
	    // {
	    // 	OutRecord_PointsCHull 	<< PointsCHull.col(0).transpose() << "  " << PointsCHull.col(1).transpose() << endl;
		   //  OutRecord_NormalCHull  	<< W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull.col(1).transpose() << endl;
		   //  OutRecord_DistanceCHull << DistanceEdgesCHull.transpose() << endl;	
	    // }
		   
		//  
		count++;
	} 

	// stance indicator
	Vector2d stance_ind;
		if(stance_0 =="left")
			stance_ind << 0.0, 1.0;
		else // stance is right
			stance_ind << 1.0, 0.0;
	//
	if((stable_0 == true) && (lengthStep <= lengthStep_max))
		step_values << DeltaStep(0), DeltaStep(1), this->Delta_theta_feet, stance_ind(0), stance_ind(1), 0.0;

	else //if((stable_0 != true) && (lengthStep >= lengthStep_max))   // TO BE CHECKED FOR MATHEMATICAL CONSISTENCY ANN STABILITY
	{
		// update the balance perturbation
		// --------------------------------
		// compute the variation of the perturbation due to the stepping
		this->compute_delta_disturb(Wrench_hands_star, DeltaStep, DeltaDisturb);
		// updating the Perturbation
		W_Disturb = Delat_R_cmp + DeltaDisturb;
		// update capture point and absolute frame
		W_CP_     = CP.head(2)  + DeltaStep;
		// get the points defining the support polygon of each feet
		this->getJointFeetSsupportPoints_2(stance_1, theta_swing, this->DyFeet_min, Pose_lfoot, Pose_rfoot, DeltaStep, 
										   								AllPtsInAbsFoot, W_Rot_AbsFoot, W_Pos_AbsFoot);
		// update convex hull
		ConvHull.convexHull(AllPtsInAbsFoot, PointsCHull, Abs_EdgesNormalCHull, DistanceEdgesCHull); // --> PointsCHull, EdgesNormalCHull, DistanceEdgesCHull
		// Express the normal in the world frame
		W_EdgesNormalCHull = Abs_EdgesNormalCHull * W_Rot_AbsFoot.transpose();
		// predict the stability
		stable_1 = this->predict_stability(W_EdgesNormalCHull, DistanceEdgesCHull, W_CP_, W_Pos_AbsFoot, W_Disturb);
		//
		step_values << DeltaStep(0), DeltaStep(1), this->Delta_theta_feet, stance_ind(0), stance_ind(1), 1.0;
		//
		cout << "DeltaDisturb  \t" <<  DeltaDisturb.transpose() << endl;
		cout << "W_Disturb  \t" <<  W_Disturb.transpose() << endl; 
		cout << "stable_1  \t" <<  stable_1 << endl; 

		// *****************************************************************************************
	    // Record the data Poses and joints configuration
	    // *****************************************************************************************
	    // int stance_left = 0; 
	    // if(stance_0 =="left") stance_left = 1;
	    // if(PointsCHull.rows() == 5)
	    // {
	    // 	OutRecord_PointsCHull 	<< PointsCHull.col(0).transpose() << "  " << PointsCHull(0,0) << "  " << PointsCHull(0,0) << "  ";   // x
	    // 	OutRecord_PointsCHull 	<< PointsCHull.col(1).transpose() << "  " << PointsCHull(0,1) << "  " << PointsCHull(0,1) << endl;   // y

		   //  OutRecord_NormalCHull  	<< W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull(0,0) << "  " << W_EdgesNormalCHull(0,0) << "  ";   // x
		   //  OutRecord_NormalCHull  	<< W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull(0,1) << "  " << W_EdgesNormalCHull(0,1) << endl;	  // y

		   //  OutRecord_DistanceCHull << DistanceEdgesCHull.transpose() << "  " << DistanceEdgesCHull(0) << "  " << DistanceEdgesCHull(0) << endl;	
	    // }
	    // else if(PointsCHull.rows() == 6)
	    // {
	    // 	OutRecord_PointsCHull 	<< PointsCHull.col(0).transpose() << "  " << PointsCHull(0,0) << "  " ;   // x
	    // 	OutRecord_PointsCHull 	<< PointsCHull.col(1).transpose() << "  " << PointsCHull(0,1) << endl;   // y

		   //  OutRecord_NormalCHull  	<< W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull(0,0) << "  ";   // x
		   //  OutRecord_NormalCHull  	<< W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull(0,1) << endl;	  // y

		   //  OutRecord_DistanceCHull << DistanceEdgesCHull.transpose() << "  " << DistanceEdgesCHull(0) << endl;
	    // }
	    // else
	    // {
	    // 	OutRecord_PointsCHull 	<< PointsCHull.col(0).transpose() << "  " << PointsCHull.col(1).transpose() << endl;
		   //  OutRecord_NormalCHull  	<< W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull.col(1).transpose() << endl;
		   //  OutRecord_DistanceCHull << DistanceEdgesCHull.transpose() << endl;	
	    // }
		    
	    //
	}
	
	//
	// OutRecord_stability_var.close();
	// OutRecord_PointsCHull.close();
	// OutRecord_NormalCHull.close();
	// OutRecord_DistanceCHull.close();

	return step_values;
}


//
bool anticipatory_ctrl::LoadDataFromFile(std::string file_name, VectorXd &data_all_val)
{
    //
    ifstream inFile;
    // inFile.open("/home/michael/Dropbox/wbHandsReachabilityData_01/log_joints_pos_04.txt");
    inFile.open(file_name);
    if (!inFile) {
        cout << "Unable to open file \n";
        exit(1);                            // terminate with error
    }
    //
    std::vector<double> data_val;
    double x; 
    //
    while(inFile >> x) {
        data_val.push_back(x);
    }
    //
    int size_data_val = data_val.size();
    //
    data_all_val.resize(size_data_val);
    for(int i=0; i<size_data_val; i++) 
        data_all_val(i) = data_val[i];

    return true;
}


bool anticipatory_ctrl::Load_gmm_param(std::string file_name[], int dataDim, int nbStates, VectorXd &Priors_, MatrixXd &Means_, MatrixXd &Covars_)
{

	// 
	std::ifstream inFile_Priors;
	std::ifstream inFile_Means;
	std::ifstream inFile_Covar;

	std::string Priors_file_name  = file_name[0]; // + "_prio.txt";  
	std::string Means_file_name   = file_name[1]; // + "_mu.txt";  
	std::string Covar_file_name   = file_name[2]; // + "_sigma.txt";  
	//
	VectorXd priors_all_val;
	VectorXd means_all_val;
	VectorXd covars_all_val;
	//
	this->LoadDataFromFile(Priors_file_name, priors_all_val);
	this->LoadDataFromFile(Means_file_name,  means_all_val);
	this->LoadDataFromFile(Covar_file_name,  covars_all_val);
	//
	// Priors
	Priors_ = priors_all_val;
	// Means
	Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > Means_Mx(means_all_val.data(),dataDim, nbStates);
	Means_ = Means_Mx;
	
	//
	int row_cov = dataDim * nbStates;
	// Covariance
	Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > Covar_Mx(covars_all_val.data(),row_cov,dataDim);
	Covars_ = Covar_Mx;


	return true;
}


bool anticipatory_ctrl::Load_gmm_param2(std::string file_name[], VectorXd &Priors_, MatrixXd &Means_, MatrixXd &Covars_)
{

	// 
	std::ifstream inFile_Priors;
	std::ifstream inFile_Means;
	std::ifstream inFile_Covar;

	std::string Priors_file_name  = file_name[0]; // + "_prio.txt";  
	std::string Means_file_name   = file_name[1]; // + "_mu.txt";  
	std::string Covar_file_name   = file_name[2]; // + "_sigma.txt";  
	//
	VectorXd priors_all_val;
	VectorXd means_all_val;
	VectorXd covars_all_val;
	//
	this->LoadDataFromFile(Priors_file_name, priors_all_val);
	this->LoadDataFromFile(Means_file_name,  means_all_val);
	this->LoadDataFromFile(Covar_file_name,  covars_all_val);
	//
	// Priors
	Priors_  = priors_all_val;
	//
	int nbStates = priors_all_val.rows();
	int dataDim  = int(means_all_val.rows()/nbStates);
	// Means
	Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > Means_Mx(means_all_val.data(),dataDim, nbStates);
	Means_ = Means_Mx;

	//
	int row_cov = dataDim * nbStates;
	// Covariance
	Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > Covar_Mx(covars_all_val.data(),row_cov,dataDim);
	Covars_ = Covar_Mx;


	return true;
}


double anticipatory_ctrl::gaussPDF(VectorXd Data, VectorXd Mu, MatrixXd Sigma)
{
	VectorXd delta_x = Data - Mu;
	double x_mu = delta_x.transpose() * Psdinv.get_pseudoInverse(Sigma) * delta_x;

	return exp(-0.5*x_mu)/sqrt(pow(2*M_PI, Data.rows()) * (fabs(Sigma.determinant()) + 1e-30));
}

//
bool anticipatory_ctrl::set_IIK_weights(MatrixXd E_Cov_O_dot, MatrixXd E_Cov_I_dot, MatrixXd E_Cov_O_ddot, MatrixXd E_Cov_I_ddot)
{
	//
	this->invSigma_xidot  = E_Cov_O_dot.inverse();
    this->invSigma_qdot   = E_Cov_I_dot.inverse();
    this->invSigma_xiddot = E_Cov_O_ddot.inverse();
    this->invSigma_qddot  = E_Cov_I_ddot.inverse();

	return true;
}

// ========================================

bool anticipatory_ctrl::Load_PCA_model(std::string file_name[], VectorXd &mean_, MatrixXd &Ap_)
{
	//
	std::string mean_file_name  = file_name[0]; //   
	std::string Ap_file_name  	= file_name[1]; //   

	VectorXd mean_all_val;
	VectorXd Ap_all_val;
	// PCA variables
	this->LoadDataFromFile(mean_file_name, mean_all_val);
	this->LoadDataFromFile(Ap_file_name,   Ap_all_val);

	// Mean vector
	mean_ = mean_all_val;

	// Projection Matrix (nDim x nCmp)
	int nDim = mean_all_val.rows();
	int nCmp = int(Ap_all_val.rows()/nDim);
	
	Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > Ap_Mx(Ap_all_val.data(),nDim, nCmp);
	Ap_ = Ap_Mx;

	return true;
}	

bool anticipatory_ctrl::pca_projection(VectorXd mean_, MatrixXd Ap_, VectorXd in_data, VectorXd &out_proj_data)
{
	//
	if(mean_.rows() != in_data.rows())
	{
		printf("Error: input data and mean have different dimensions");
		return false;
	}
	VectorXd cntr_data = in_data - mean_;
	//
	out_proj_data = Ap_.transpose() * cntr_data;

	return true;
}

bool anticipatory_ctrl::pca_reconstruction(VectorXd mean_, MatrixXd Ap_, VectorXd in_proj_data, VectorXd &out_data)
{
	//
	if(Ap_.cols() != in_proj_data.rows())
	{
		printf("Error: latent data and projecion matrix have incompatible dimensions");
		return false;
	}
	//
	out_data = Ap_ * in_proj_data + mean_;

	return true;
}

bool anticipatory_ctrl::get_GMR_IO(VectorXd Prior, MatrixXd Mean_gmm, MatrixXd CovMx, VectorXd Input_, VectorXd &Output_, MatrixXd &E_Cov_I, MatrixXd &E_Cov_O)
{
	// Extract the number of Gaussian
	int nStates = Prior.rows();
	// Extract dimension of q_star and xi_star
	int nI = Input_.rows();
	int nO = Mean_gmm.rows() - nI;

	Output_.resize(nO);
	Output_.setZero();
	//
	E_Cov_I.resize(nI, nI);		    E_Cov_I.setZero();	// Expected Input covariance q
	E_Cov_O.resize(nO, nO);			E_Cov_O.setZero();  // Expected Output covariance xi
	//
	MatrixXd Cov_I_k(nI,nI);
	MatrixXd Cov_O_k(nO,nO);
	MatrixXd Cov_IO_k(nI,nO);
	MatrixXd Cov_OI_k(nO,nI);

	// |  nI | nIO |  
	// |-----------|
	// | nOI | nO  | 

	
	// Computation of h
	VectorXd h = VectorXd::Zero(nStates);
	VectorXd Normal_h = VectorXd::Zero(nStates);
	//
	int dim = nI+nO;
	//
	for(int k=0; k<nStates; k++)
	{
		Cov_I_k = CovMx.block(k*dim,  0, nI, nI);
		//
		h(k) = Prior(k) * this->gaussPDF(Input_, Mean_gmm.col(k).head(nI), Cov_I_k);
	}

	Normal_h = h/h.sum();
	//
	MatrixXd Cov_k(dim, dim);
	//
	for(int k=0; k<nStates; k++)
	{
		//
		Cov_k 	 = CovMx.block(k*dim, 0, dim, dim);
		//
		Cov_I_k  = Cov_k.block( 0,   0,  nI, nI);	//
		Cov_IO_k = Cov_k.block( 0,  nI,  nI, nO);   //
		Cov_OI_k = Cov_k.block(nI,   0,  nO, nI);	//
		Cov_O_k  = Cov_k.block(nI,  nI,  nO, nO);   //
		//
		// Posterior of q
		VectorXd D_In_mu 			 = Input_ - Mean_gmm.col(k).head(nI);

		MatrixXd Sigma_OI_invSigma_I = Cov_OI_k * Psdinv.get_pseudoInverse(Cov_I_k);

		Output_ += Normal_h(k)*( Mean_gmm.col(k).tail(nO) + Sigma_OI_invSigma_I * D_In_mu );
		// Associated covaraince of Posterior q
		E_Cov_O += Normal_h(k)*Normal_h(k)*(Cov_O_k - Sigma_OI_invSigma_I * Cov_IO_k);
		//
		E_Cov_I += Normal_h(k)*Normal_h(k)*(Cov_I_k);		
	}

	return true;
}

//
bool anticipatory_ctrl::get_expected_centroidal_task(VectorXd hands_task, Vector3d &pos_CoM_hat, Vector6d &cntr_mtm_hat, MatrixXd &E_Cov_Hands_, MatrixXd &E_Cov_CoM_)
{
    
    VectorXd exp_com_tsk(9);
    VectorXd proj_hands_tsk; 
    VectorXd proj_CoM_tsk;
    //
    this->pca_projection(Mean_hands, Ap_hands, hands_task, proj_hands_tsk);
    //
    this->get_GMR_IO(Prior_HandsCoM, Mean_HandsCoM, CovMx_HandsCoM, proj_hands_tsk, proj_CoM_tsk, E_Cov_Hands_, E_Cov_CoM_);
    //
    this->pca_reconstruction(Mean_CoM, Ap_CoM, proj_CoM_tsk, exp_com_tsk);
    //
    pos_CoM_hat  = exp_com_tsk.head(3);
    cntr_mtm_hat = exp_com_tsk.tail(6);

    return true;
}


// New function related associated with stabilizability
// =======================================================

bool anticipatory_ctrl::compute_perturbation_terms(VectorXd Wrench_hands_star_, 
													Vector3d d_pos_lh, Vector3d d_pos_rh, Vector6d CM_, 
													Vector2d &Disturb_c, double &fu_z_mgbetaW)
{
	//
	MatrixXd M_flh(2,3),  M_frh(2,3);
			 M_flh << 					  0.0, -Wrench_hands_star_(2),  Wrench_hands_star_(1),
			 			Wrench_hands_star_(2),           		  0.0, -Wrench_hands_star_(0);

			 M_frh << 					   0.0, -Wrench_hands_star_(2+6),  Wrench_hands_star_(1+6),
			 			Wrench_hands_star_(2+6),           		     0.0, -Wrench_hands_star_(0+6);
	Vector2d Tau_H;
	Tau_H << Wrench_hands_star_(3) + Wrench_hands_star_(3+6),
			 Wrench_hands_star_(4) + Wrench_hands_star_(4+6);

	Tau_H -= (M_flh * d_pos_lh + M_frh * d_pos_rh);

	double BetaW = (1. + CM_(2)/(mass_bot*9.81) - (Wrench_hands_star_(2)+Wrench_hands_star_(2+6))/(mass_bot * 9.81));
	double mgBetaW2 = mass_bot * 9.81 * BetaW * BetaW;
	//
	// --------------------------
	Disturb_c(0) = 1./(mgBetaW2) * ( Tau_H(1) - CM_(4));
	Disturb_c(1) = 1./(mgBetaW2) * (-Tau_H(0) + CM_(3));
	//
	fu_z_mgbetaW = (Wrench_hands_star_(2)+Wrench_hands_star_(2+6))/mgBetaW2;

	return true;
}

bool anticipatory_ctrl::compute_perturbation_terms2(VectorXd Wrench_hands_star_, 
													Vector3d d_pos_lh, Vector3d d_pos_rh, Vector3d AM_, Vector3d CoM_ddot,
													Vector2d &Disturb_c, double &fu_z_mgbetaW)
{
	//
	MatrixXd M_flh(2,3),  M_frh(2,3);
			 M_flh << 					  0.0, -Wrench_hands_star_(2),  Wrench_hands_star_(1),
			 			Wrench_hands_star_(2),           		  0.0, -Wrench_hands_star_(0);

			 M_frh << 					   0.0, -Wrench_hands_star_(2+6),  Wrench_hands_star_(1+6),
			 			Wrench_hands_star_(2+6),           		     0.0, -Wrench_hands_star_(0+6);
	Vector2d Tau_H;
	Tau_H << Wrench_hands_star_(3) + Wrench_hands_star_(3+6),
			 Wrench_hands_star_(4) + Wrench_hands_star_(4+6);

	Tau_H -= (M_flh * d_pos_lh + M_frh * d_pos_rh);

	double BetaW = (1. + CoM_ddot(2)/(mass_bot*9.81) - (Wrench_hands_star_(2)+Wrench_hands_star_(2+6))/(mass_bot * 9.81));
	double mgBetaW2 = mass_bot * 9.81 * BetaW * BetaW;
	//
	// --------------------------
	Disturb_c(0) = 1./(mgBetaW2) * ( Tau_H(1) - AM_(1));
	Disturb_c(1) = 1./(mgBetaW2) * (-Tau_H(0) + AM_(0));
	//
	fu_z_mgbetaW = (Wrench_hands_star_(2)+Wrench_hands_star_(2+6))/mgBetaW2;

	return true;
}

bool anticipatory_ctrl::getSupportConstraints(Vector7d pose_lfoot, Vector7d pose_rfoot, Vector2d DeltaStep, double DeltaThetaFeet_, 
					  						  MatrixXd &W_EdgesNormalCHull, VectorXd &DistanceEdgesCHull, Vector2d &W_Pos_AbsFoot)
{
	//
    MatrixXd AllPtsInAbsFoot;
    MatrixXd PointsCHull;
    MatrixXd Abs_EdgesNormalCHull;
    Matrix2d W_Rot_AbsFoot;

	// get the points of the Convex hull 
	get_feet_support_points_C(pose_lfoot, pose_rfoot, DeltaStep, DeltaThetaFeet_, AllPtsInAbsFoot, W_Rot_AbsFoot, W_Pos_AbsFoot);
	//
	ConvHull.convexHull(AllPtsInAbsFoot, PointsCHull, Abs_EdgesNormalCHull, DistanceEdgesCHull);
	//
	W_EdgesNormalCHull = Abs_EdgesNormalCHull * W_Rot_AbsFoot.transpose();

	for(int i=0; i<W_EdgesNormalCHull.rows(); i++)
	{
		W_EdgesNormalCHull.row(i) = W_EdgesNormalCHull.row(i)/W_EdgesNormalCHull.row(i).norm();
	}
	//
	this->W_Rot_AbsFoot_ = W_Rot_AbsFoot;
	this->W_Pos_AbsFoot_ = W_Pos_AbsFoot;
	this->W_EdgesNormalCHull_ = W_EdgesNormalCHull;
	this->DistanceEdgesCHull_ = DistanceEdgesCHull;

	// std::cout << " cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc \t" <<  std::endl;
 //    std::cout << " \n" <<  std::endl;
 //    std::cout << " W_Rot_AbsFoot_   IS : \n" << W_Rot_AbsFoot_ << std::endl;
 //    std::cout << " \n" <<  std::endl;
 //    std::cout << " cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc \t" <<  std::endl;

	return true;
}

double anticipatory_ctrl::getSteppingCost(Vector2d CP_star, Vector2d DeltaStep, double DeltaThetaFeet_)
{
    Vector2d c1 = CP_star - this->W_Pos_AbsFoot_ - this->Delat_R_cmp;

    double cost = 1./this->lengthStep_max  * (pow(DeltaStep(0),2.) + pow(DeltaStep(1),2.)) 
                + 1./this->Dtheta_feet_max * (pow(DeltaThetaFeet_, 2.))//;
                + 50./this->lengthStep_max  * c1.transpose() * c1;
    //
    return cost;
}

double anticipatory_ctrl::getSteppingCost2(Vector2d DeltaStep, double DeltaThetaFeet_)
{
    double cost = 1./this->lengthStep_max  * (pow(DeltaStep(0),2.) + pow(DeltaStep(1),2.)) 
                + 1./this->Dtheta_feet_max * (pow(DeltaThetaFeet_, 2.));
    //
    return cost;
}

Vector7d anticipatory_ctrl::getSlidingFootReference(Vector2d Disturb_c_, double fu_z_mgbetaW_, Vector3d CoM_, Vector7d pose_lfoot, Vector7d pose_rfoot, string stanceFoot, double stpMagn)
{
	//
	Vector7d pose_Sldfoot;
	Vector2d BalPert_ = Disturb_c_ + fu_z_mgbetaW_*CoM_.head(2) - 0.5*(pose_lfoot.head(2) + pose_rfoot.head(2));


	std::cout<< "PERTURBATION   DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD   : \t" << BalPert_.transpose() << std::endl;
	//
	Vector3d o_lf = Transforms.getEulerAnglesXYZ_FixedFrame(pose_lfoot);
	Vector3d o_rf = Transforms.getEulerAnglesXYZ_FixedFrame(pose_rfoot);
	double theta_Dist = stpMagn/fabs(stpMagn+1e-10) *atan2(BalPert_(1), BalPert_(0));
	double dTheta_st = 0.0;
	// foot orientation
	Matrix3d des_Obj_orient;

	if(stanceFoot =="left")
	{
		//
		dTheta_st = o_lf(2);
	    double dTheta_sw = theta_Dist - dTheta_st;
        if     (dTheta_sw <= -Dtheta_feet_max) dTheta_sw = -Dtheta_feet_max;
        else if(dTheta_sw >=  Dtheta_feet_max) dTheta_sw =  Dtheta_feet_max;
        // new orientation with desired orientation of the feet based on the perturbation
		Matrix3d d_R_ = Transforms.rot_vector2rpy(Vector3d(o_rf(0), o_rf(1), dTheta_sw + dTheta_st));
		Eigen::AngleAxisd d_aa(d_R_);

		pose_Sldfoot.head(2)      = pose_rfoot.head(2) + stpMagn * BalPert_ /(BalPert_.norm() + 1e-10);
		pose_Sldfoot(2)		      = pose_lfoot(2); // pose_rfoot(2); same hight as stance foot
		pose_Sldfoot.segment(3,3) = d_aa.axis();
		pose_Sldfoot(6)           = d_aa.angle();
	}
	else
	{
		
		//
		dTheta_st = o_rf(2);
		double dTheta_sw = theta_Dist - dTheta_st;
        if     (dTheta_sw <= -Dtheta_feet_max) dTheta_sw = -Dtheta_feet_max;
        else if(dTheta_sw >=  Dtheta_feet_max) dTheta_sw =  Dtheta_feet_max;
        // new orientation with desired orientation of the feet based on the perturbation
		Matrix3d d_R_ = Transforms.rot_vector2rpy(Vector3d(o_lf(0), o_lf(1), dTheta_sw + dTheta_st));
		Eigen::AngleAxisd d_aa(d_R_);

		pose_Sldfoot.head(2)      = pose_lfoot.head(2) +  stpMagn * BalPert_ /(BalPert_.norm() + 1e-10);
		pose_Sldfoot(2)		      = pose_rfoot(2);  // pose_lfoot(2); same hight as stance foot
		pose_Sldfoot.segment(3,3) = d_aa.axis();
		pose_Sldfoot(6)           = d_aa.angle();
	}

	
	std::cout<< "pose_rfoot   : " << pose_rfoot.transpose() << std::endl;
	std::cout<< "pose_lfoot   : " << pose_lfoot.transpose() << std::endl;
	std::cout<< "pose_Sldfoot : " << pose_Sldfoot.transpose() << std::endl;

	return pose_Sldfoot;
}


Vector7d anticipatory_ctrl::getSlidingFootReference2(Vector2d Disturb_c_, double fu_z_mgbetaW_, Vector3d CoM_, 
													 Vector7d pose_lfoot, Vector7d pose_rfoot, 
													 Vector7d pose_lf, Vector7d pose_rf, string stanceFoot, double stpMagn)
{
	//
	Vector7d pose_Sldfoot;
	Vector2d BalPert_ = Disturb_c_ + fu_z_mgbetaW_*0.5*(pose_lf.head(2) + pose_rf.head(2)) - 0.5*(pose_lf.head(2) + pose_rf.head(2));


	std::cout<< "PERTURBATION   DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD   : \t" << BalPert_.transpose() << std::endl;
	//
	Vector3d o_lf = Transforms.getEulerAnglesXYZ_FixedFrame(pose_lf);
	Vector3d o_rf = Transforms.getEulerAnglesXYZ_FixedFrame(pose_rf);
	double theta_Dist = stpMagn/fabs(stpMagn+1e-10) *atan2(BalPert_(1), BalPert_(0));
	double dTheta_st = 0.0;
	// foot orientation
	Matrix3d des_Obj_orient;

	if(stanceFoot =="left")
	{
		//
		dTheta_st = o_lf(2);
	    double dTheta_sw = theta_Dist - dTheta_st;
        if     (dTheta_sw <= -Dtheta_feet_max) dTheta_sw = -Dtheta_feet_max;
        else if(dTheta_sw >=  Dtheta_feet_max) dTheta_sw =  Dtheta_feet_max;
        // new orientation with desired orientation of the feet based on the perturbation
		Matrix3d d_R_ = Transforms.rot_vector2rpy(Vector3d(o_rf(0), o_rf(1), 0.0*dTheta_sw + dTheta_st));
		Eigen::AngleAxisd d_aa(d_R_);

		pose_Sldfoot.head(2)      = pose_rfoot.head(2) + stpMagn * BalPert_ /(BalPert_.norm() + 1e-10);
		pose_Sldfoot(2)		      = pose_lfoot(2); // pose_rfoot(2); same hight as stance foot
		pose_Sldfoot.segment(3,3) = d_aa.axis();
		pose_Sldfoot(6)           = d_aa.angle();
	}
	else
	{
		
		//
		dTheta_st = o_rf(2);
		double dTheta_sw = theta_Dist - dTheta_st;
        if     (dTheta_sw <= -Dtheta_feet_max) dTheta_sw = -Dtheta_feet_max;
        else if(dTheta_sw >=  Dtheta_feet_max) dTheta_sw =  Dtheta_feet_max;
        // new orientation with desired orientation of the feet based on the perturbation
		Matrix3d d_R_ = Transforms.rot_vector2rpy(Vector3d(o_lf(0), o_lf(1), 0.0*dTheta_sw + dTheta_st));
		Eigen::AngleAxisd d_aa(d_R_);

		pose_Sldfoot.head(2)      = pose_lfoot.head(2) +  stpMagn * BalPert_ /(BalPert_.norm() + 1e-10);
		pose_Sldfoot(2)		      = pose_rfoot(2);  // pose_lfoot(2); same hight as stance foot
		pose_Sldfoot.segment(3,3) = d_aa.axis();
		pose_Sldfoot(6)           = d_aa.angle();
	}

	
	std::cout<< "pose_rfoot   : " << pose_rfoot.transpose() << std::endl;
	std::cout<< "pose_lfoot   : " << pose_lfoot.transpose() << std::endl;
	std::cout<< "pose_Sldfoot : " << pose_Sldfoot.transpose() << std::endl;

	return pose_Sldfoot;
}

Vector7d anticipatory_ctrl::getInitGuessFootReference(	Vector2d Disturb_c_, double fu_z_mgbetaW_, Vector7d pose_lfoot, Vector7d pose_rfoot, string stanceFoot)
{
	//
	Vector7d pose_Guessfoot;
	// Vector2d InitDeltaStep = -2./(fu_z_mgbetaW_ - 1.) *(Disturb_c_ + (1.+fu_z_mgbetaW_)*0.5*(pose_lf.head(2) + pose_rf.head(2)) - 0.5*(pose_lf.head(2) + pose_rf.head(2)));
	Vector2d BalPert_      = Disturb_c_ + fu_z_mgbetaW_*(0.5*(pose_lfoot.head(2) + pose_rfoot.head(2)) - 0.5*(pose_lfoot.head(2) + pose_rfoot.head(2)));
	// Vector2d InitDeltaStep = -2./(fu_z_mgbetaW_ - 1.) * (Disturb_c_ + fu_z_mgbetaW_ * Vector2d(0.0, 0.0));  // 2/(lamda_c -1)*((1+lambda_c)*c + /varPhi - r_bar) // c=r_bar
	Vector2d InitDeltaStep = -2. * (Disturb_c_ + fu_z_mgbetaW_ * Vector2d(0.0, 0.0));  // 2/(lamda_c -1)*((1+lambda_c)*c + /varPhi - r_bar) // c=r_bar

	std::cout<< "BalPert_ before     : \t" << BalPert_.transpose() << std::endl;
	std::cout<< "InitDeltaStep before     : \t" << InitDeltaStep.transpose() << std::endl;
	// //clamping within the step limits
	// // ===============================
	// if(InitDeltaStep.norm() > this->lengthStep_max)
	// 	InitDeltaStep = InitDeltaStep/(InitDeltaStep.norm() + 1e-10) *this->lengthStep_max;
	if(stanceFoot =="left")
	{
		// InitDeltaStep = -2.*(Disturb_c_ + (1.+fu_z_mgbetaW_)*0.5*(pose_rfoot.head(2) - pose_lfoot.head(2)) - 0.5*(pose_rfoot.head(2) - pose_lfoot.head(2)));
		// InitDeltaStep = -2. * (Disturb_c_ + fu_z_mgbetaW_ * Vector2d(0.0, 0.0));
		InitDeltaStep = 2. * (Disturb_c_/(fu_z_mgbetaW_ + 1.) + fu_z_mgbetaW_ * Vector2d(0.0, 0.0));
	}
	else
	{
		// InitDeltaStep = -2.*(Disturb_c_ + (1.+fu_z_mgbetaW_)*0.5*(pose_lfoot.head(2) - pose_rfoot.head(2)) - 0.5*(pose_lfoot.head(2) - pose_rfoot.head(2)));
		InitDeltaStep = 2. * (Disturb_c_ /(fu_z_mgbetaW_ + 1.) + fu_z_mgbetaW_ * Vector2d(0.0, 0.0));
	}
	//
	//clamping within the step limits
	// ===============================
	if(InitDeltaStep.norm() > this->lengthStep_max)
		InitDeltaStep = InitDeltaStep/(InitDeltaStep.norm() + 1e-10) *this->lengthStep_max;


	std::cout<< "PERTURBATION   DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD   : \t" << BalPert_.transpose() << std::endl;
	std::cout<< "InitDeltaStep      : \t" << InitDeltaStep.transpose() << std::endl;
	// std::cout<< "CHECK InitDeltaStep      : \t" << (Disturb_c_ + fu_z_mgbetaW_*0.5*InitDeltaStep - 0.5*InitDeltaStep).transpose() << std::endl;
	std::cout<< "CHECK InitDeltaStep      : \t" << (Disturb_c_ + (1.+fu_z_mgbetaW_)*(-0.5*InitDeltaStep)).transpose() << std::endl;
	std::cout<< "stance foot      : \t" << stanceFoot << std::endl;
	//
	Vector3d o_lf = Transforms.getEulerAnglesXYZ_FixedFrame(pose_lfoot);
	Vector3d o_rf = Transforms.getEulerAnglesXYZ_FixedFrame(pose_rfoot);

	double theta_Dist = atan2(BalPert_(1), BalPert_(0));
	double dTheta_st = 0.0;
	// foot orientation
	Matrix3d des_Obj_orient;
	//
	Matrix2d RotStanceFt = MatrixXd::Identity(2,2);     // Rotation of stance foot wrt the world

	if(stanceFoot =="left")
	{
		//
		dTheta_st = o_lf(2);
	    double dTheta_sw = theta_Dist - dTheta_st;
        if     (dTheta_sw <= -Dtheta_feet_max) dTheta_sw = -Dtheta_feet_max;
        else if(dTheta_sw >=  Dtheta_feet_max) dTheta_sw =  Dtheta_feet_max;
        // new orientation with desired orientation of the feet based on the perturbation
		Matrix3d d_R_ = Transforms.rot_vector2rpy(Vector3d(o_rf(0), o_rf(1), dTheta_sw + dTheta_st));
		Eigen::AngleAxisd d_aa(d_R_);

		RotStanceFt << cos(o_lf(2)), -sin(o_lf(2)), sin(o_lf(2)), cos(o_lf(2));
		// pose_Sldfoot.head(2)      = pose_rfoot.head(2) + stpMagn * BalPert_ /(BalPert_.norm() + 1e-10);
		// pose_Guessfoot.head(2)      = RotStanceFt*(InitDeltaStep - Vector2d(0.0,  this->DyFeet_min)) + pose_lfoot.head(2);
		// pose_Guessfoot.head(2)		= pose_lfoot.head(2) +  InitDeltaStep;
		pose_Guessfoot.head(2)		= pose_rfoot.head(2) +  RotStanceFt*InitDeltaStep;
		pose_Guessfoot(2)		    = pose_lfoot(2); // pose_rfoot(2); same hight as stance foot
		pose_Guessfoot.segment(3,3) = d_aa.axis();
		pose_Guessfoot(6)           = d_aa.angle();
	}
	else
	{
		//
		dTheta_st = o_rf(2);
		double dTheta_sw = theta_Dist - dTheta_st;
        if     (dTheta_sw <= -Dtheta_feet_max) dTheta_sw = -Dtheta_feet_max;
        else if(dTheta_sw >=  Dtheta_feet_max) dTheta_sw =  Dtheta_feet_max;
        // new orientation with desired orientation of the feet based on the perturbation
		Matrix3d d_R_ = Transforms.rot_vector2rpy(Vector3d(o_lf(0), o_lf(1), dTheta_sw + dTheta_st));
		Eigen::AngleAxisd d_aa(d_R_);

		RotStanceFt << cos(o_rf(2)), -sin(o_rf(2)), sin(o_rf(2)), cos(o_rf(2));
		// pose_Guessfoot.head(2)      = pose_lfoot.head(2) +  stpMagn * BalPert_ /(BalPert_.norm() + 1e-10);
		// pose_Guessfoot.head(2)      = RotStanceFt*(InitDeltaStep + Vector2d(0.0,  this->DyFeet_min)) + pose_rfoot.head(2);
		// pose_Guessfoot.head(2)      = pose_rfoot.head(2) + InitDeltaStep;
		pose_Guessfoot.head(2)      = pose_lfoot.head(2) + RotStanceFt*InitDeltaStep;
		pose_Guessfoot(2)		    = pose_rfoot(2);  // pose_lfoot(2); same hight as stance foot
		pose_Guessfoot.segment(3,3) = d_aa.axis();
		pose_Guessfoot(6)           = d_aa.angle();
	}

	//
	std::cout<< "pose_rfoot   	: " << pose_rfoot.transpose() << std::endl;
	std::cout<< "pose_lfoot   	: " << pose_lfoot.transpose() << std::endl;
	std::cout<< "stance foot   	: " << stanceFoot << std::endl;
	std::cout<< "pose_Guessfoot : " << pose_Guessfoot.transpose() << std::endl;

	return pose_Guessfoot;
}