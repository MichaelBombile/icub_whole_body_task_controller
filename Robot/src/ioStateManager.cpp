///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ioStateManager

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

#include "ioStateManager.h"

using namespace Eigen;
using namespace std;


ioStateManager::ioStateManager(int period, std::string robotName, int actuatedDofs, Matrix4d w_H_lf, Matrix4d w_H_rf, std::string stanceFoot, ControllerParameters&  ctrl_param_, WbRobotModel& robot, RobotInterface& robot_interface_) //, TasksReferences& Ref_motion)  
								: RateThread(period)
								, RobotName(robotName)
								, robot_interface(robot_interface_)
								, n_actuatedDofs(actuatedDofs)
								, ctrl_param(ctrl_param_)
								, robot_model(robot)
								, massMatrix(actuatedDofs+6, actuatedDofs + 6)
								, h_biasForces(actuatedDofs+6)
								, gravityBiasTorques(actuatedDofs+6)
								// Jacobians
								, Jacobian_pelvis(6 , actuatedDofs+6)
								, Jacobian_left_hand(6 , actuatedDofs+6)
								, Jacobian_right_hand(6 , actuatedDofs+6)
								, Jacobian_left_foot(6 , actuatedDofs+6)
								, Jacobian_right_foot(6 , actuatedDofs+6)
								, Jacobian_CoM(6 , actuatedDofs+6)
								, Jacobian_Chest(6 , actuatedDofs+6)
								, Jacobian_centro_momentum(6 , actuatedDofs+6)
								, Jacobian_SoT_top6(6 , actuatedDofs+6)
								, ee_contacts(5)
								// command
								, Joints_cmds(actuatedDofs)
								, stance_foot(stanceFoot)
								, w_H_lfoot(w_H_lf)
								, w_H_rfoot(w_H_rf)

						        { ThreadPeriod = period; n_actuatedDofs = actuatedDofs; run_period_sec = 0.001 * period; 
						        	w_H_lfoot = w_H_lf;
									w_H_rfoot = w_H_rf;
						        }
										        

ioStateManager::~ioStateManager() { }

// ============================  THREAD INITIALIZATION ===========================
bool ioStateManager::threadInit()
{
	//
	StopCtrl 	= false;
	Cycle_counter = 0;
	trqfac = 0.0001;

	// Forward dynamics variables
	massMatrix.setZero();
	h_biasForces.setZero();
	gravityBiasTorques.setZero();
	// Joint space states
	JtsStates.position.resize(n_actuatedDofs);
	JtsStates.velocity.resize(n_actuatedDofs);
	JtsStates.acceleration.resize(n_actuatedDofs);
	// States of the robot
	JtsStates.position.setZero();
	JtsStates.velocity.setZero();
	JtsStates.acceleration.setZero();

	wbTS.setZero();
	wb_des_tasks_acceleration.initialize(n_actuatedDofs+6);
	// TskSpStates.Pose.setZero();
	// TskSpStates.Velo.setZero();
	Cont_States.Pelvis 	= 0.0;
	Cont_States.lhand 	= 0.0;
	Cont_States.rhand 	= 0.0;
	Cont_States.lfoot 	= 1.0;
	Cont_States.rfoot 	= 1.0;

	ee_contacts.setZero();
	ee_contacts << 	Cont_States.Pelvis, 
					Cont_States.lhand,
					Cont_States.rhand,
					Cont_States.lfoot,
					Cont_States.rfoot;


	// stance_foot 		= "left";
	//
	m_wb_joints_sensor.position.resize(n_actuatedDofs);
    m_wb_joints_sensor.velocity.resize(n_actuatedDofs);
    m_wb_joints_sensor.acceleration.resize(n_actuatedDofs);
    m_wb_joints_sensor.torque.resize(n_actuatedDofs);
    //
    world_H_fBase.setIdentity();
	world_Velo_fBase.setZero();

	// Centroidal dynamics variables
	CentroidalDynamicMatrix.setZero();
	Centroidal_Wrench_Map_pelvis.setZero();
	Centroidal_Wrench_Map_lhand.setZero();
	Centroidal_Wrench_Map_rhand.setZero();
	Centroidal_Wrench_Map_lfoot.setZero();
	Centroidal_Wrench_Map_rfoot.setZero();
	Gravity_Force.setZero();
	// Jacobians
	// ============================
	Jacobian_pelvis.setZero();
	Jacobian_left_hand.setZero();
	Jacobian_right_hand.setZero();
	Jacobian_left_foot.setZero();
	Jacobian_right_foot.setZero();
	Jacobian_CoM.setZero();
	Jacobian_Chest.setZero();
	Jacobian_centro_momentum.setZero();
	Jacobian_SoT_top6.setZero();
	DJacobianDq_SoT_top6.setZero();

	DJacobianDq_pelvis.setZero();
	DJacobianDq_left_hand.setZero();
	DJacobianDq_right_hand.setZero();
	DJacobianDq_left_foot.setZero();
	DJacobianDq_right_foot.setZero();
	DJacobianDq_centro_momentum.setZero();
	DJacobianDq_CoM.setZero();
	DJacobianDq_Chest.setZero();
	//
	q_dot = Eigen::VectorXd::Zero(n_actuatedDofs+6);
	//
	// CoP_constraint_matrix_lfoot.setZero();
	// CoP_constraint_matrix_rfoot.setZero();
	//
	Jac_larm_hand.setZero();
	Jac_rarm_hand.setZero();
	//
	switch_pelvis_chest = true;
	//

	robot_model.EstimateRobotStates(this->robot_interface, this->stance_foot, m_wb_joints_sensor, world_H_fBase, world_Velo_fBase);
	robot_model.getLinkPose(m_wb_joints_sensor.position, world_H_fBase, "left_foot",  wbTS.lfoot.Pose);
	robot_model.getLinkPose(m_wb_joints_sensor.position, world_H_fBase, "right_foot", wbTS.rfoot.Pose);
	//
	Vector7d W_Pose_aF_;
	Transforms.get_absolute_pose(wbTS.lfoot.Pose, wbTS.rfoot.Pose, W_Pose_aF_);
	Vector2d t_lf_aF = wbTS.lfoot.Pose.head(2) - W_Pose_aF_.head(2);
	Vector2d t_rf_aF = wbTS.rfoot.Pose.head(2) - W_Pose_aF_.head(2);

	w_H_lfoot.setIdentity(4,4);		w_H_lfoot.block(0,3,2,1) = t_lf_aF + Vector2d(0.0, 0.0);
	w_H_rfoot.setIdentity(4,4);		w_H_rfoot.block(0,3,2,1) = t_rf_aF;

	//
	w_Pose_absF	= W_Pose_aF_;
	w_H_absF    = Transforms.PoseVector2HomogenousMx(w_Pose_absF); 
	
	
	// update the robot state
	this->UpdateRobotStates();    // ------------------------------>>> chech the effect of this comments
	this->get_Fwd_Kinematics_and_DynamicsVariables();
	// initialize previous Centroidal inertia I(q) for dotI(q)
	dotCentroidalInertia_mxI.setZero();
    CentroidalInertia_mxI_previous.setZero();
    CentroidalInertia_mxI_previous(0) = CentroidalDynamicMatrix(3,3);
    CentroidalInertia_mxI_previous(1) = CentroidalDynamicMatrix(4,4);
    CentroidalInertia_mxI_previous(2) = CentroidalDynamicMatrix(5,5);
	//
	return true;
}

// ============================  THREAD RUNNING LOOP ============================
void ioStateManager::run()
{
	double t_run = yarp::os::Time::now();
	// update the robot state
	this->UpdateRobotStates();	
	//
	this->get_Fwd_Kinematics_and_DynamicsVariables();

	// //
	// if(!ctrl_param.OrientChest)
	// {
	// 	Jacobian_SoT_top6		= Jacobian_centro_momentum;
	// 	DJacobianDq_SoT_top6    = DJacobianDq_centro_momentum;
	// }
	// else
	// {
	// 	// Jacobian liniear momentum (related to CoM task)
	// 	Jacobian_SoT_top6.topRows<3>() = Jacobian_centro_momentum.topRows<3>();
	// 	DJacobianDq_SoT_top6.head<3>() = DJacobianDq_centro_momentum.head<3>();
	// 	//
	// 	Jacobian_SoT_top6.bottomRows<3>() = Jacobian_pelvis.bottomRows<3>();
	// 	DJacobianDq_SoT_top6.tail<3>()    = DJacobianDq_pelvis.tail<3>();
	// 	//
	// 	// if(fmod(Cycle_counter, 2) == 0){
			
	// 	// 	Jacobian_SoT_top6.bottomRows<3>() = Jacobian_pelvis.bottomRows<3>();
	// 	// 	DJacobianDq_SoT_top6.tail<3>()    = DJacobianDq_pelvis.tail<3>();
	// 	// }
	// 	// if(fmod(Cycle_counter, 2) == 1){
	// 	// 	Jacobian_SoT_top6.bottomRows<3>() = Jacobian_Chest.bottomRows<3>();
	// 	// 	DJacobianDq_SoT_top6.tail<3>()    = DJacobianDq_Chest.tail<3>();
	// 	// }



	// 	// if(fmod(Cycle_counter, 10) == 0) switch_pelvis_chest = !switch_pelvis_chest;
	// 	// if(switch_pelvis_chest){
			
	// 	// 	Jacobian_SoT_top6.bottomRows<3>() = Jacobian_pelvis.bottomRows<3>();
	// 	// 	DJacobianDq_SoT_top6.tail<3>()    = DJacobianDq_pelvis.tail<3>();
	// 	// }
	// 	// if(!switch_pelvis_chest){
	// 	// 	Jacobian_SoT_top6.bottomRows<3>() = Jacobian_Chest.bottomRows<3>();
	// 	// 	DJacobianDq_SoT_top6.tail<3>()    = DJacobianDq_Chest.tail<3>();
	// 	// }
	// }
	

 	// if (ctrl_param.writeCommands) 
 	// {
		// // send the computed torque commands
	 // 	this->SendTorqueCommands();	
 	// }
 	// std::cout << " desired_StackOfTasks is : \n" << desired_StackOfTasks << std::endl;
 	// std::cout << " COMPUTED TORQUE is : \n" << Tau_actuated << std::endl;
 	// std::cout << " COMPUTED ACCELEARTION is : \n" << optimal_acceleration << std::endl;
 	// std::cout << " CONTACT FORCE FEET are : \n" << optimal_feet_contact_forces << std::endl;
 	std::cout<< " /////////// =================================================> IO STATE MANAGER RUN  in  : " << yarp::os::Time::now()-t_run << " s" << std::endl;
 	std::cout<< "IOSM COUNTER IS : " << Cycle_counter << std::endl;
 	Cycle_counter ++;

}
//
void ioStateManager::threadRelease()
{ 
	std::cerr << "Deactivating the ioStateManager\n";
}


// ========================================== robot_interface dependent functions ============================================================
bool ioStateManager::UpdateRobotStates()
{
	//	
	// yarp::os::LockGuard guard(m_mutex);
	//
	// ee_contacts : pelvis, lhand, rhand, lfoot and rfoot;
	Cont_States.update(ee_contacts(0), ee_contacts(1), ee_contacts(2), ee_contacts(3), ee_contacts(4));  // =====================>>>>
	// this->UpdateStanceFoot(Cont_States, stance_foot);
	// robot_model.EstimateRobotStates(this->robot_interface, this->stance_foot, m_wb_joints_sensor, world_H_fBase, world_Velo_fBase);
	robot_model.EstimateRobotStates(this->robot_interface, this->w_H_lfoot, this->w_H_rfoot, this->stance_foot, m_wb_joints_sensor, world_H_fBase, world_Velo_fBase);
	// robot_model.UpdateWholeBodyModel(this->stance_foot);

	// cout << " LEFT HOM TRANS  \n" << w_H_lfoot << endl;
    // cout << " RIGHT HOM TRANS  \n" << w_H_rfoot << endl;
    // cout << " PELVIS HOM TRANS  \n" << world_H_fBase << endl;
	//
	return true;
}
//
void ioStateManager::SendTorqueCommands(VectorXd Tau_actuated)
{	
	//
	if(ctrl_param.AllPositionArmsTorqueMode)
	{
		// robot_model.robot_interface.setControlReferenceTorqueArmsOnly(Tau_actuated);		// sent the computed torques values to arms only
		this->robot_interface.setControlReferenceTorqueArmsOnly(Tau_actuated);		// sent the computed torques values to arms only
		trqfac = 0.040;
	}		
	else
	{
		// trqfac = 0.995 * trqfac + 0.005;
		yarp::os::Time::delay(trqfac);
		// robot_model.setControlReference(this->robot_interface, Tau_actuated);
		this->robot_interface.setWholeBodyControlReference(Tau_actuated);
		trqfac = 0.0001;									// sent the computed torques values to all
	}
}

// ==============================================================================================================================================


bool ioStateManager::UpdateStanceFoot(ContactsStates Cont_States, std::string& stance_foot)
{
	if((Cont_States.lfoot == 0.) && (Cont_States.rfoot == 1.)) {
		stance_foot = "right";
	} else{
		stance_foot = "left";
	}
	return true;
}

//
bool ioStateManager::get_Fwd_Kinematics_and_DynamicsVariables()
{
	//
	bool ok = true;
	//
	ok = ok && robot_model.getMassMatrix(m_wb_joints_sensor.position, world_H_fBase, massMatrix);
	ok = ok && robot_model.getGeneralizedBiasForces(m_wb_joints_sensor.position, world_H_fBase, m_wb_joints_sensor.velocity, world_Velo_fBase, h_biasForces);
	ok = ok && robot_model.getGeneralizedGravityForces(m_wb_joints_sensor.position, world_H_fBase, m_wb_joints_sensor.velocity, world_Velo_fBase, gravityBiasTorques);

	Gravity_Force(2) = -massMatrix(0,0) * 9.81;
	//	
	ok = ok && robot_model.getJacobian(m_wb_joints_sensor.position, world_H_fBase, "left_hand", 		 	Jacobian_left_hand);
	ok = ok && robot_model.getJacobian(m_wb_joints_sensor.position, world_H_fBase, "right_hand", 		 	Jacobian_right_hand);
	ok = ok && robot_model.getJacobian(m_wb_joints_sensor.position, world_H_fBase, "left_foot", 		 	Jacobian_left_foot);
	ok = ok && robot_model.getJacobian(m_wb_joints_sensor.position, world_H_fBase, "right_foot", 		 	Jacobian_right_foot);
	ok = ok && robot_model.getJacobian(m_wb_joints_sensor.position, world_H_fBase, "Pelvis", 			 	Jacobian_pelvis);
	ok = ok && robot_model.getJacobian(m_wb_joints_sensor.position, world_H_fBase, "CoM", 			 		Jacobian_CoM);
	ok = ok && robot_model.getJacobian(m_wb_joints_sensor.position, world_H_fBase, "Chest", 			 	Jacobian_Chest);
	ok = ok && robot_model.getJacobian(m_wb_joints_sensor.position, world_H_fBase, "centroidal_momentum",	Jacobian_centro_momentum);

	ok = ok && robot_model.getDJdq(m_wb_joints_sensor.position, world_H_fBase, m_wb_joints_sensor.velocity, world_Velo_fBase, "left_hand", 		 	 DJacobianDq_left_hand);
	ok = ok && robot_model.getDJdq(m_wb_joints_sensor.position, world_H_fBase, m_wb_joints_sensor.velocity, world_Velo_fBase, "right_hand", 		 DJacobianDq_right_hand);
	ok = ok && robot_model.getDJdq(m_wb_joints_sensor.position, world_H_fBase, m_wb_joints_sensor.velocity, world_Velo_fBase, "left_foot", 		 	 DJacobianDq_left_foot);
	ok = ok && robot_model.getDJdq(m_wb_joints_sensor.position, world_H_fBase, m_wb_joints_sensor.velocity, world_Velo_fBase, "right_foot", 		 DJacobianDq_right_foot);
	ok = ok && robot_model.getDJdq(m_wb_joints_sensor.position, world_H_fBase, m_wb_joints_sensor.velocity, world_Velo_fBase, "Pelvis", 			 DJacobianDq_pelvis);
	ok = ok && robot_model.getDJdq(m_wb_joints_sensor.position, world_H_fBase, m_wb_joints_sensor.velocity, world_Velo_fBase, "CoM", 			 	 DJacobianDq_CoM);
	ok = ok && robot_model.getDJdq(m_wb_joints_sensor.position, world_H_fBase, m_wb_joints_sensor.velocity, world_Velo_fBase, "Chest", 			 	 DJacobianDq_Chest);
	ok = ok && robot_model.getDJdq(m_wb_joints_sensor.position, world_H_fBase, m_wb_joints_sensor.velocity, world_Velo_fBase, "centroidal_momentum", DJacobianDq_centro_momentum);
	//
	Eigen::Matrix<double, 6,6> CoM_X_Base;
	robot_model.compute_transform_CoM_Base(m_wb_joints_sensor.position, world_H_fBase, CoM_X_Base);
	//
	Centroidal_Wrench_Map_pelvis = CoM_X_Base.inverse().transpose()*Jacobian_pelvis.transpose().topRows(6);
	Centroidal_Wrench_Map_lhand  = CoM_X_Base.inverse().transpose()*Jacobian_left_hand.transpose().topRows(6);
	Centroidal_Wrench_Map_rhand  = CoM_X_Base.inverse().transpose()*Jacobian_right_hand.transpose().topRows(6);
	Centroidal_Wrench_Map_lfoot  = CoM_X_Base.inverse().transpose()*Jacobian_left_foot.transpose().topRows(6);
	Centroidal_Wrench_Map_rfoot  = CoM_X_Base.inverse().transpose()*Jacobian_right_foot.transpose().topRows(6);

	// centroidal dynamic_matrix
	CentroidalDynamicMatrix = Jacobian_centro_momentum.leftCols(6) * CoM_X_Base.inverse();
	//
	this->get_dotCentroidalInertia_matrix_I();
	// Joints states
	JtsStates.position          =   this->m_wb_joints_sensor.position;
    JtsStates.velocity          =   this->m_wb_joints_sensor.velocity;
    JtsStates.acceleration      =   this->m_wb_joints_sensor.acceleration;
    // Task space poses
	ok = ok && robot_model.getLinkPose(m_wb_joints_sensor.position, world_H_fBase, "left_hand",  wbTS.lhand.Pose);
	ok = ok && robot_model.getLinkPose(m_wb_joints_sensor.position, world_H_fBase, "right_hand", wbTS.rhand.Pose);
	ok = ok && robot_model.getLinkPose(m_wb_joints_sensor.position, world_H_fBase, "left_foot",  wbTS.lfoot.Pose);
	ok = ok && robot_model.getLinkPose(m_wb_joints_sensor.position, world_H_fBase, "right_foot", wbTS.rfoot.Pose);
	ok = ok && robot_model.getLinkPose(m_wb_joints_sensor.position, world_H_fBase, "Pelvis", 	 wbTS.Pelvis.Pose);
	ok = ok && robot_model.getLinkPose(m_wb_joints_sensor.position, world_H_fBase, "Chest",  	 wbTS.Chest.Pose);
	ok = ok && robot_model.getLinkPose(m_wb_joints_sensor.position, world_H_fBase, "CoM",  		 wbTS.CoM.Pose);
	//
	w_H_lfoot = Transforms.PoseVector2HomogenousMx(wbTS.lfoot.Pose);
	w_H_rfoot = Transforms.PoseVector2HomogenousMx(wbTS.rfoot.Pose);

	Transforms.get_absolute_pose(wbTS.lfoot.Pose, wbTS.rfoot.Pose, w_Pose_absF);
	w_H_absF  = Transforms.PoseVector2HomogenousMx(w_Pose_absF); 


	//
	// Eigen::VectorXd q_dot(n_actuatedDofs+6);
	q_dot.head(6) = robot_model.eigRobotState.baseVel; 
	q_dot.segment(6, n_actuatedDofs) = JtsStates.velocity;
	//
	wbTS.lhand.Velo 	= 	Jacobian_left_hand	* q_dot;
	wbTS.rhand.Velo 	= 	Jacobian_right_hand * q_dot;
	wbTS.lfoot.Velo 	= 	Jacobian_left_foot  * q_dot;
	wbTS.rfoot.Velo 	= 	Jacobian_right_foot * q_dot;
	wbTS.Pelvis.Velo  =  	Jacobian_pelvis 	* q_dot;
	wbTS.Chest.Velo 	= 	Jacobian_Chest  	* q_dot;
	// wbTS.CoM.Velo 	= 	Jacobian_CoM 		* q_dot;
	wbTS.CoM.Velo 	=   CoM_X_Base * world_Velo_fBase //robot_model.eigRobotState.baseVel
									+ CoM_X_Base * massMatrix.block<6,6>(0,0).inverse()*massMatrix.block(0,6, 6, n_actuatedDofs)*JtsStates.velocity; 
	//
	Jac_larm_hand = Jacobian_left_hand.block<6,7>(0, 9);
	Jac_rarm_hand = Jacobian_right_hand.block<6,7>(0,16);

	//

	return ok;
}


// estimation of the time derivative of the centroidal inertial matrix
bool ioStateManager::get_dotCentroidalInertia_matrix_I()
{
	//
	dotCentroidalInertia_mxI(0) = (CentroidalDynamicMatrix(3,3)- CentroidalInertia_mxI_previous(0))/run_period_sec;
	dotCentroidalInertia_mxI(1) = (CentroidalDynamicMatrix(4,4)- CentroidalInertia_mxI_previous(1))/run_period_sec;
	dotCentroidalInertia_mxI(2) = (CentroidalDynamicMatrix(5,5)- CentroidalInertia_mxI_previous(2))/run_period_sec;

	// update
	CentroidalInertia_mxI_previous(0) = CentroidalDynamicMatrix(3,3);
    CentroidalInertia_mxI_previous(1) = CentroidalDynamicMatrix(4,4);
    CentroidalInertia_mxI_previous(2) = CentroidalDynamicMatrix(5,5);

	return true;
}


void ioStateManager::copy_whole_body_poses2array8(Vector7d (&wbPoses)[8])
{
    wbPoses[0] = this->wbTS.lhand.Pose;    // left hand            //    
    wbPoses[1] = this->wbTS.rhand.Pose;    // right hand           //                       
    wbPoses[2] = this->wbTS.lfoot.Pose;    // left foot
    wbPoses[3] = this->wbTS.rfoot.Pose;    // right foot
    wbPoses[4] = this->wbTS.CoM.Pose;      // CoM Position
    wbPoses[5] = this->wbTS.Pelvis.Pose;   // Angular momentum
    wbPoses[6] = this->wbTS.Pelvis.Pose;   // Pelvis
    wbPoses[7] = this->wbTS.Chest.Pose;    // Chest
}