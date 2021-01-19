// ===================================================================================
/*
 * BalanceReference : This class encodes a function that compute reference CoM and CoP
 *
*/
// ===================================================================================

#include "BalanceReference.h"

USING_NAMESPACE_QPOASES

using namespace std;
using namespace Eigen;


cop_Vars 		cop_vars;
cop_Params 		cop_params;
cop_Workspace 	cop_work;
cop_Settings 	cop_settings;


BalanceReference::BalanceReference(){};

BalanceReference::~BalanceReference(){};

void BalanceReference::Initialize(WbRobotModel& robot_model_, ControllerParameters &ctrl_param, ioStateManager &ioSM_)
{
	//
	cop_set_defaults();
	cop_setup_indexing();
	//

	PtsInFoot.resize(2,4);  // model the convex hull of the foot with 4 points
	// PtsInFoot(0,0) =  0.100;  PtsInFoot(1,0) =  0.015;  // point 1		  p1 ---- p4
	// PtsInFoot(0,1) = -0.015;  PtsInFoot(1,1) =  0.015;  // point 2			|	 |
	// PtsInFoot(0,2) = -0.015;  PtsInFoot(1,2) = -0.015;  // point 3			|	 |
	// PtsInFoot(0,3) =  0.100;  PtsInFoot(1,3) = -0.015;  // point 4  	 	 p2 |____| p3
	PtsInFoot = ctrl_param.PtsInFoot;

    //-------------------------------------------
    // Convex hull
    //-------------------------------------------
    MatrixXd W_EdgesNormalCHull; 
    VectorXd DistanceEdgesCHull;  
    Matrix2d W_Rot_AbsFoot; 
	Vector2d W_Pos_AbsFoot;
	robot_model_.getConvexHullVariables(this->PtsInFoot, ioSM_.wbTS.lfoot.Pose, ioSM_.wbTS.rfoot.Pose,  W_EdgesNormalCHull, DistanceEdgesCHull, W_Rot_AbsFoot, W_Pos_AbsFoot);
	MatrixXd NeCH = MatrixXd::Zero(6, 2);
	Vector6d deCH = VectorXd::Zero(6);
	//
	if(W_EdgesNormalCHull.rows() <= 5){
		int r = W_EdgesNormalCHull.rows();
		NeCH.topRows(r) = W_EdgesNormalCHull.topRows(r);
		deCH.head(r)    = DistanceEdgesCHull.head(r) + W_EdgesNormalCHull.topRows(r)*(W_Pos_AbsFoot);
	}
	else{
		NeCH = W_EdgesNormalCHull.topRows(6);
		deCH = DistanceEdgesCHull.head(6) + W_EdgesNormalCHull.topRows(6)*(W_Pos_AbsFoot);
	}
	//-------------------------------------------
    // CoM
    //-------------------------------------------
    Matrix2d Q_com 	=  MatrixXd::Identity(2, 2);
    Vector2d p_com 	= -ioSM_.wbTS.CoM.Pose.head(2);
    MatrixXd Mcom 	=  MatrixXd::Zero(6, 2);
	Vector6d Bcom 	=  VectorXd::Zero(6);
	//-------------------------------------------
    // CoP
    //-------------------------------------------
    Matrix2d Q_cop  = (ctrl_param.w_cop + (1. - ctrl_param.w_cop))*MatrixXd::Identity(2, 2);
    Vector2d p_cop  = -(ctrl_param.w_cop * ioSM_.wbTS.CoM.Pose.head(2) + (1. - ctrl_param.w_cop) * ioSM_.wbTS.CoM.Pose.head(2));   // initialize with CoM position
    MatrixXd Mcop 	=  MatrixXd::Zero(6, 2);
	Vector6d Bcop 	=  VectorXd::Zero(6);
    //
	//---------------------------------------------------------------------------------------
	// stepping controller and leg inverse kinematics
	rbase_H_des_lfoot = MatrixXd::Identity(4,4); 
	rbase_H_des_rfoot = MatrixXd::Identity(4,4); 

	lleg_ik_jts = ioSM_.JtsStates.position.tail(12).head(6);
	rleg_ik_jts = ioSM_.JtsStates.position.tail(6);

    //   STEPPING CONTROLLER  
    // ======================
    Vector3d abs_CoM_pos = ioSM_.wbTS.CoM.Pose.head(3) - ioSM_.w_Pose_absF.head(3);
    //
    step_ctrl.init(ctrl_param.RobotName, ctrl_param.period_step, 0, abs_CoM_pos);
    step_ctrl.Parameters->CoMHeight       = ioSM_.wbTS.CoM.Pose(2);
    step_ctrl.CpBalWlkController->DMod.zc = ioSM_.wbTS.CoM.Pose(2);

    step_ctrl.CpBalWlkController->set_step_magnitude(Vector3d(0.00, 0.00, -0.0)); // 0.10, 0.02, 0.00
    step_ctrl.CpBalWlkController->stance_foot = ioSM_.stance_foot;

    //   Leg inverse kinematics 
    // =========================
    IK_legs.InitializeIK(robot_model_, 0.90, 30, 1e-4, 0.450);  	// TO DO : put all these parameters in the ctrl_param object


   
}

//
Vector3d BalanceReference::ConstrainCoM(WbRobotModel& robot_model_, ioStateManager &ioSM_) // //Vector7d Pose_lfoot, Vector7d Pose_rfoot, Vector3d CoM)
{
	//
	Vector3d com_star = ioSM_.wbTS.CoM.Pose.head(3);
	// computation of the convex polygon
	MatrixXd W_EdgesNormalCHull; 
    VectorXd DistanceEdgesCHull;  
    Matrix2d W_Rot_AbsFoot; 
	Vector2d W_Pos_AbsFoot;
	robot_model_.getConvexHullVariables(this->PtsInFoot, ioSM_.wbTS.lfoot.Pose, ioSM_.wbTS.rfoot.Pose,  W_EdgesNormalCHull, DistanceEdgesCHull, W_Rot_AbsFoot, W_Pos_AbsFoot);
	//
	Matrix2d Q_com =  0.5*MatrixXd::Identity(2, 2);
	Vector2d p_com = -ioSM_.wbTS.CoM.Pose.head(2);
	//
	MatrixXd NeCH = MatrixXd::Zero(6, 2);
	Vector6d deCH = VectorXd::Zero(6);
	//
	if(W_EdgesNormalCHull.rows() <= 5){
		int r = W_EdgesNormalCHull.rows();
		NeCH.topRows(r) = W_EdgesNormalCHull.topRows(r);
		deCH.head(r)    = DistanceEdgesCHull.head(r) + W_EdgesNormalCHull.topRows(r)*(W_Pos_AbsFoot);
	}
	else{
		NeCH = W_EdgesNormalCHull.topRows(6);
		deCH = DistanceEdgesCHull.head(6) + W_EdgesNormalCHull.topRows(6)*(W_Pos_AbsFoot);
	}

	// QP Solution
	// -------------
	// Hessian matrix
	memcpy(cop_params.Q, Q_com.data(), sizeof(double) * Q_com.size());
	memcpy(cop_params.P, &p_com[0], sizeof(double)*(2)); 						// weight acceleration and desired posture
	// setting of hard constraints
	for(int i=0;i<6;i++) 
		memcpy(cop_params.NeJc, NeCH.data(),	sizeof(double)*NeCH.size());	// convexHull constraints Matrix for CoM
	memcpy(cop_params.deXc, &deCH[0], 		sizeof(double)*(6));				// convexHull constraints vector for CoM
	
	// Solve the QP (position)
	cop_settings.verbose 	 = 0;
	// cop_settings.eps 		 = 15e-5;
	// cop_settings.resid_tol = 1e-4;
	int iter = cop_solve();
	// Extract the QP solution (acceleration and slack)
	for(int i=0; i<2; i++) com_star(i) = cop_vars.xc[i];


	std::cout<< "CHECKING COM Constraints   : \n" <<  NeCH*com_star.head(2) - deCH << std::endl;

	return com_star;
}


Vector3d BalanceReference::ConstrainCoP(WbRobotModel& robot_model_, ioStateManager &ioSM_, Vector3d CoP) // //Vector7d Pose_lfoot, Vector7d Pose_rfoot, Vector3d CoP)
{
	//
	Vector3d CoP_star = CoP;
	// computation of the convex polygon
	MatrixXd W_EdgesNormalCHull; 
    VectorXd DistanceEdgesCHull;  
    Matrix2d W_Rot_AbsFoot; 
	Vector2d W_Pos_AbsFoot;
	robot_model_.getConvexHullVariables(this->PtsInFoot, ioSM_.wbTS.lfoot.Pose, ioSM_.wbTS.rfoot.Pose,  W_EdgesNormalCHull, DistanceEdgesCHull, W_Rot_AbsFoot, W_Pos_AbsFoot);
	//
	Matrix2d Q_cop  = 0.5*MatrixXd::Identity(2, 2);
    Vector2d p_cop  = -CoP.head(2);
	//
	MatrixXd NeCH = MatrixXd::Zero(6, 2);
	Vector6d deCH = VectorXd::Zero(6);
	//
	if(W_EdgesNormalCHull.rows() <= 5){
		int r = W_EdgesNormalCHull.rows();
		NeCH.topRows(r) = W_EdgesNormalCHull.topRows(r);
		deCH.head(r)    = DistanceEdgesCHull.head(r) + W_EdgesNormalCHull.topRows(r)*(W_Pos_AbsFoot);
	}
	else{
		NeCH = W_EdgesNormalCHull.topRows(6);
		deCH = DistanceEdgesCHull.head(6) + W_EdgesNormalCHull.topRows(6)*(W_Pos_AbsFoot);
	}

	// QP Solution
	// --------------
	// Hessian matrix
	memcpy(cop_params.Q, Q_cop.data(), sizeof(double) * Q_cop.size());
	memcpy(cop_params.P, &p_cop[0], sizeof(double)*(2)); 						// weight acceleration and desired posture
	// setting of hard constraints
	for(int i=0;i<6;i++) 
		memcpy(cop_params.NeJc, NeCH.data(),	sizeof(double)*NeCH.size());	// convexHull constraints Matrix for CoM
	memcpy(cop_params.deXc, &deCH[0], 		sizeof(double)*(6));				// convexHull constraints vector for CoM
	
	// Solve the QP (position)
	cop_settings.verbose 	 = 0;
	// cop_settings.eps 		 = 15e-5;
	// cop_settings.resid_tol = 1e-4;
	int iter = cop_solve();
	// Extract the QP solution (acceleration and slack)
	for(int i=0; i<2; i++) CoP_star(i) = cop_vars.xc[i];

	//
	std::cout<< "CHECKING COP Constraints   : \n" <<  NeCH*CoP_star.head(2) - deCH << std::endl;

	return CoP_star;
}

Vector3d BalanceReference::get_reference_CoP(WbRobotModel& robot_model_, ControllerParameters &ctrl_param, ioStateManager &ioSM_, Vector3d CoP, Vector3d des_CoP) // //Vector7d Pose_lfoot, Vector7d Pose_rfoot, Vector3d CoM)
{
	//
	Vector3d CoP_star = CoP;
	// computation of the convex polygon
	MatrixXd W_EdgesNormalCHull; 
    VectorXd DistanceEdgesCHull;  
    Matrix2d W_Rot_AbsFoot; 
	Vector2d W_Pos_AbsFoot;
	robot_model_.getConvexHullVariables(this->PtsInFoot, ioSM_.wbTS.lfoot.Pose, ioSM_.wbTS.rfoot.Pose,  W_EdgesNormalCHull, DistanceEdgesCHull, W_Rot_AbsFoot, W_Pos_AbsFoot);
	//
	Matrix2d Q_cop  = 0.5*(ctrl_param.w_cop + (1. - ctrl_param.w_cop))*MatrixXd::Identity(2, 2);
    Vector2d p_cop  = -(ctrl_param.w_cop * CoP.head(2) + (1. - ctrl_param.w_cop) * des_CoP.head(2));
	//
	MatrixXd NeCH = MatrixXd::Zero(6, 2);
	Vector6d deCH = VectorXd::Zero(6);
	//
	if(W_EdgesNormalCHull.rows() <= 5){
		int r = W_EdgesNormalCHull.rows();
		NeCH.topRows(r) = W_EdgesNormalCHull.topRows(r);
		deCH.head(r)    = DistanceEdgesCHull.head(r) + W_EdgesNormalCHull.topRows(r)*(W_Pos_AbsFoot);
	}
	else{
		NeCH = W_EdgesNormalCHull.topRows(6);
		deCH = DistanceEdgesCHull.head(6) + W_EdgesNormalCHull.topRows(6)*(W_Pos_AbsFoot);
	}

	// QP Solution
	// -----------
	// Hessian matrix
	memcpy(cop_params.Q, Q_cop.data(), sizeof(double) * Q_cop.size());
	memcpy(cop_params.P, &p_cop[0], sizeof(double)*(2)); 						// weight acceleration and desired posture
	// setting of hard constraints
	for(int i=0;i<6;i++) 
		memcpy(cop_params.NeJc, NeCH.data(),	sizeof(double)*NeCH.size());	// convexHull constraints Matrix for CoM
	memcpy(cop_params.deXc, &deCH[0], 		sizeof(double)*(6));				// convexHull constraints vector for CoM
	
	// Solve the QP (position)
	cop_settings.verbose 	 = 0;
	// cop_settings.eps 		 = 15e-5;
	// cop_settings.resid_tol = 1e-4;
	int iter = cop_solve();
	// Extract the QP solution (acceleration and slack)
	for(int i=0; i<2; i++) CoP_star(i) = cop_vars.xc[i];

	//
	std::cout<< "CHECKING COP Constraints   : \n" <<  NeCH*CoP_star.head(2) - deCH << std::endl;

	return CoP_star;
}
//
Vector3d BalanceReference::get_cop_from_wrench(VectorXd Wch_)
{
	return Vector3d(-Wch_(4)/Wch_(2), Wch_(3)/Wch_(2), 0.0);
}

Vector3d BalanceReference::get_robot_feet_cop(ioStateManager &ioSM_, VectorXd Wl_, VectorXd Wr_)
{
	Vector3d p_cop_lf = get_cop_from_wrench(Wl_);
	Vector3d p_cop_rf = get_cop_from_wrench(Wr_);
	//
	Vector3d w_cop_lf = ioSM_.w_H_lfoot.block<3,3>(0,0) * p_cop_lf + ioSM_.w_H_lfoot.block<3,1>(0,3);
    Vector3d w_cop_rf = ioSM_.w_H_rfoot.block<3,3>(0,0) * p_cop_rf + ioSM_.w_H_rfoot.block<3,1>(0,3);

	Vector3d cop_rb_ft = 1./(Wl_(2) + Wr_(2)) * (Wl_(2) * w_cop_lf + Wr_(2) * w_cop_rf); 
	
	return cop_rb_ft; 
}


//
void BalanceReference::make_step(ioStateManager &ioSM_)
{
    //
    int number_steps = 1; 
    double stride_x  = 0.00;
    step_ctrl.CpBalWlkController->pose_lf = ioSM_.wbTS.lfoot.Pose;
    step_ctrl.CpBalWlkController->pose_rf = ioSM_.wbTS.rfoot.Pose;
    //
    step_ctrl.walk(number_steps, stride_x, ioSM_.wbTS.lhand.Pose, ioSM_.wbTS.rhand.Pose, ioSM_.wbTS);
    //
    rbase_H_des_lfoot = step_ctrl.Trsf_des_lfoot2base_robot;
    rbase_H_des_rfoot = step_ctrl.Trsf_des_rfoot2base_robot;
}


void BalanceReference::get_leg_inverseKin(WbRobotModel& m_robot_model_, ioStateManager &ioSM_, Matrix4d rbase_H_des_lfoot_, Matrix4d rbase_H_des_rfoot_, VectorXd &jts_cmds)
{
    lleg_ik_jts = IK_legs.get_legs_IK(m_robot_model_, "l_sole", rbase_H_des_lfoot_, ioSM_.JtsStates.position);
    rleg_ik_jts = IK_legs.get_legs_IK(m_robot_model_, "r_sole", rbase_H_des_rfoot_, ioSM_.JtsStates.position);
    // output of leg IK joints
    jts_cmds.tail(12).head(6) = lleg_ik_jts;
    jts_cmds.tail(6)          = rleg_ik_jts;

    std::cout << " LEG JOINTS  LEFT STEPPING is  : \t" << 180./M_PI * lleg_ik_jts.transpose() << std::endl;
    std::cout << " LEG JOINTS RIGHT STEPPING is  : \t" << 180./M_PI * rleg_ik_jts.transpose() << std::endl;
}


void BalanceReference::generate_step_commands(WbRobotModel& m_robot_model_, ioStateManager &ioSM_, bool &SwitchStep, bool &executing_step, bool &StepCompleted, 
							 					int &stepCount, int nSampleStep, std::string &stance_ft, Vector3d StepFoot_1, double d_theta_torso_pitch_,  VectorXd &Joints_pos_cmds)
{
	if(SwitchStep) // executing_step
    {
        step_ctrl.CpBalWlkController->set_step_magnitude(StepFoot_1);  	// magnitude of the step
        this->make_step(ioSM_);
        this->get_leg_inverseKin(m_robot_model_, ioSM_, this->rbase_H_des_lfoot, this->rbase_H_des_rfoot, Joints_pos_cmds);
        // adjust torso picth angle
        // Joints_pos_cmds(2) = d_theta_torso_pitch_; //* step_ctrl.CpBalWlkController->dx_com_normalized;
        Joints_pos_cmds(2) = 0.90*Joints_pos_cmds(2) + 0.1*d_theta_torso_pitch_; //* step_ctrl.CpBalWlkController->dx_com_normalized;

        if(step_ctrl.Parameters->StanceIndicator[0] == 1) stance_ft = "left";
        else    stance_ft = "right";

        if(stepCount == (nSampleStep -1))
        {
            SwitchStep     = false;
            executing_step = SwitchStep; 
            StepCompleted  = true;
            stepCount      = 0;
            // desactivate the step exectution
            step_ctrl.CpBalWlkController->set_step_magnitude(Vector3d(0.00, 0.00, 0.00));
            // d_theta_torso_pitch = 0.0;
        }
        //
        // this->record_SteppingData();
        std::cout << " ooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo is  : \t" << stepCount << std::endl;
        stepCount ++;
    }
}