/** Class Manipulation

*/
#include "Manipulation.h"

using namespace std;
using namespace Eigen;

Manipulation::Manipulation(){};
Manipulation::~Manipulation(){};

void Manipulation::close()
{
    //close controller thread
    if(FreeMotionCtrl) 
    {
        delete FreeMotionCtrl;
        FreeMotionCtrl = 0;
    }
}

void Manipulation::Initialize(ControllerParameters &ctrl_param, Object_to_Grasp &object2grasp, ioStateManager &ioSM, TasksReferences &references)
{
	//
	nu_Wh = 0.0;
	nu_Ao = 0.0;
    //
    Vector6d g_u; g_u << 60.0, 60.0, 60.0, 40.0, 40.0, 40.0;
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
    references.get_object_effective_wrench( object2grasp, object2grasp.Ref_Object, ctrl_param.D_object, F_external_object,  Desired_object_wrench);
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
 	// 
}




void Manipulation::get_references(ControllerParameters &ctrl_param, Object_to_Grasp &object2grasp, ioStateManager &ioSM,  TasksReferences &references,
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
    else if(isLift){ FreeMotionCtrl->getBimanualConstrainedMotion( cp_desH_Obj, object2grasp.Ref_Object.pose, ioSM);
    }
    else {  FreeMotionCtrl->compute_Reach2GraspMotion2(  object2grasp, ioSM); 
    }
    // ==================================================================
    // Bimanual Cooperative controller (Wrench computation)
    // ==================================================================
    references.get_reference_trajectories_object( object2grasp.Object_Pose,  ctrl_param, ctrl_param.K_object, object2grasp.Ref_Object);
    references.get_object_effective_wrench( object2grasp, object2grasp.Ref_Object, ctrl_param.D_object, F_external_object,  Desired_object_wrench);     // compute the object desired wrench
    std::cout << " Desired_object_wrench  is : \n" << Desired_object_wrench.transpose() << std::endl;
    //
    CooperativeCtrl.computeControlWrench(nu_Wh,  object2grasp, ioSM, Desired_object_wrench);
    CooperativeCtrl.get_TaskConsistentHandsVelocity(object2grasp.period_sec, nu_Wh*object2grasp.Ref_Object.velocity);                                   // Motion distribution between the hands 
    //
    if(ctrl_param.apply_wrench && CooperativeCtrl.ContactConfidence ==1.0 && !RRetract_)  
    {
        nu_Wh	= 0.95*nu_Wh + 0.05;
        nu_Ao	= 0.98*nu_Ao + 0.02;
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
    appWrench_lhand = act_ft.asDiagonal() * (-CooperativeCtrl.F_applied_lhand) - F_imp_lh;
    appWrench_rhand = act_ft.asDiagonal() * (-CooperativeCtrl.F_applied_rhand) - F_imp_rh;

}


Vector6d Manipulation::get_hands_ref_acceleration(std::string hand, Matrix6d g_imp_u, Matrix6d g_imp_c, ioStateManager &ioSM_, bool wConstr)
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


void Manipulation::get_hands_ref_impedance_forces(ioStateManager &ioSM_, bool isoImp)
{
	// Update Motion and Forces in the force space
    //=========================================================================================================
    double epsil_bq = 1e-10;
    MatrixXd BasisQ_lh = MatrixXd::Random(3,3);      
    MatrixXd BasisQ_rh = MatrixXd::Random(3,3);
    BasisQ_lh.block<3,1>(0,0) = (1.-nu_Wh) * FreeMotionCtrl->w_velocity_eef[0].head(3); 
    BasisQ_rh.block<3,1>(0,0) = (1.-nu_Wh) * FreeMotionCtrl->w_velocity_eef[1].head(3);  
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
    F_imp_lh.head(3) = (1.-nu_Wh) * Damping_pos_lh * (FreeMotionCtrl->w_velocity_eef[0].head(3) - ioSM_.wbTS.lhand.Velo.head(3));
    F_imp_rh.head(3) = (1.-nu_Wh) * Damping_pos_rh * (FreeMotionCtrl->w_velocity_eef[1].head(3) - ioSM_.wbTS.rhand.Velo.head(3));
    F_imp_lh.tail(3) = (1.-nu_Wh) * hands_gains.tail(3).asDiagonal() * FreeMotionCtrl->w_velocity_eef[0].tail(3);
    F_imp_rh.tail(3) = (1.-nu_Wh) * hands_gains.tail(3).asDiagonal() * FreeMotionCtrl->w_velocity_eef[1].tail(3);

}



void Manipulation::getCurrentReferenceGraspPoints(ioStateManager &ioSM_, Vector7d w_Pose_ref, Matrix4d (&h_desH_ref)[2])
{
    Matrix4d w_H_ref= Transforms.PoseVector2HomogenousMx(w_Pose_ref);
    h_desH_ref[0]   = Transforms.PoseVector2HomogenousMx(ioSM_.wbTS.lhand.Pose).inverse() * w_H_ref; 
    h_desH_ref[1]   = Transforms.PoseVector2HomogenousMx(ioSM_.wbTS.rhand.Pose).inverse() * w_H_ref;
}

VectorXd Manipulation::get_expected_object_load(ControllerParameters &ctrl_param, Object_to_Grasp &object2grasp, ioStateManager &ioSM,  TasksReferences &references, Vector7d des_lh_pose, Vector7d des_rh_pose, Vector6d ExtWrenchObj)
{
    //
    VectorXd Wrench_hands_star_ = Eigen::VectorXd::Zero(12);
    Vector6d exp_object_wrench  = Eigen::VectorXd::Zero(6);

    references.get_reference_trajectories_object( object2grasp.Object_Pose,  ctrl_param, ctrl_param.K_object, object2grasp.Ref_Object);
    references.get_object_effective_wrench( object2grasp, object2grasp.Ref_Object, ctrl_param.D_object, ExtWrenchObj,  exp_object_wrench);     // compute the object desired wrench
    std::cout << " expected_object_wrench  is : \n" << exp_object_wrench.transpose() << std::endl;
    //
    CooperativeCtrl.PredictControlWrench( object2grasp, des_lh_pose, des_rh_pose, exp_object_wrench, Wrench_hands_star_); 

    return -Wrench_hands_star_;
}