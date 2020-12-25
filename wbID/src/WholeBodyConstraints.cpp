
#include <iostream>
#include <cmath>
#include <stdexcept>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>

#include "WholeBodyConstraints.h"


using namespace std;
using namespace Eigen;

WholeBodyConstraints::WholeBodyConstraints(){}

WholeBodyConstraints::~WholeBodyConstraints(){}


void WholeBodyConstraints::Initialize( 		 int actuatedDofs,
										VectorXd HandContactParameters, 
										VectorXd FeetContactParameters, 
										VectorXd torqueSaturationLimit_, 
										VectorXd velocitySaturationLimit_,
										VectorXd minJointLimits_,
										VectorXd maxJointLimits_) 
{
	//
	n_actuatedDofs = actuatedDofs;
	//
	torqueConstraint_vector.resize(2*actuatedDofs);
	VelocityLimitsVector.resize(2*actuatedDofs);
	JointLimitsVector.resize(2*actuatedDofs);
	torqueSaturationLimit.resize(actuatedDofs);
	velocitySaturationLimit.resize(actuatedDofs);
	maxJointLimits.resize(actuatedDofs);
	minJointLimits.resize(actuatedDofs);
	IneqConstraintMatrix_lf.resize(11, 6);
	IneqConstraintVector_lf.resize(11);
	IneqConstraintMatrix_rf.resize(11, 6); 
	IneqConstraintVector_rf.resize(11);
	IneqConstraintMatrix_AllFeet.resize(22, 12);
	IneqConstraintVector_AllFeet.resize(22);
	IneqConstraintMatrix_lh.resize(11, 6);
	IneqConstraintVector_lh.resize(11);
	IneqConstraintMatrix_rh.resize(11, 6); 
	IneqConstraintVector_rh.resize(11);
	IneqConstraintMatrix_AllHands.resize(22, 12);
	IneqConstraintVector_AllHands.resize(22);
	jts_velocity_0.resize(actuatedDofs);

	//  
	mu_hand		= HandContactParameters(0);
	gamma_hand  = HandContactParameters(1);
	deltaX_hand = HandContactParameters(2);
	deltaY_hand = HandContactParameters(3);
	//
	mu_feet		= FeetContactParameters(0);
	gamma_feet  = FeetContactParameters(1);
	DeltaX_feet = FeetContactParameters(2);
	DeltaY_feet = FeetContactParameters(3);
	offset_x    = FeetContactParameters(4);

	//
	torqueSaturationLimit 	= torqueSaturationLimit_;
	velocitySaturationLimit = velocitySaturationLimit_;
	maxJointLimits 			= maxJointLimits_;
	minJointLimits 			= minJointLimits_;

	//
	// torqueConstraint_vector.resize(2*n_actuatedDofs);
	// VelocityLimitsVector.resize(2*n_actuatedDofs);
	// JointLimitsVector.resize(2*n_actuatedDofs);
	//
	torqueConstraint_vector.setZero();
	VelocityLimitsVector.setZero();
	JointLimitsVector.setZero();

	jts_velocity_0.setZero();
	//
	IneqConstraintMatrix_lf.setZero();
	IneqConstraintVector_lf.setZero();
	IneqConstraintMatrix_rf.setZero(); 
	IneqConstraintVector_rf.setZero();
	IneqConstraintMatrix_AllFeet.setZero();
	IneqConstraintVector_AllFeet.setZero();
	//
	IneqConstraintMatrix_lh.setZero();
	IneqConstraintVector_lh.setZero();
	IneqConstraintMatrix_rh.setZero(); 
	IneqConstraintVector_rh.setZero();
	IneqConstraintMatrix_AllHands.setZero();
	IneqConstraintVector_AllHands.setZero();

	//
	pxy_CoM_n1.setZero();
	pxy_CoM.setZero();
	
}

//

//
// torque constraints
bool WholeBodyConstraints::get_WbTorqueConstraints()
{
	//
	torqueConstraint_vector.head(n_actuatedDofs) = torqueSaturationLimit;
	torqueConstraint_vector.tail(n_actuatedDofs) = torqueSaturationLimit;

	return true;
}

//
//
bool WholeBodyConstraints::get_WbVelocityLimitsConstraints(double run_period_sec,
														   		VectorXd jts_velocity)
{
	//
	VelocityLimitsVector.head(n_actuatedDofs) =  velocitySaturationLimit - jts_velocity;
	VelocityLimitsVector.tail(n_actuatedDofs) =  velocitySaturationLimit + jts_velocity; 

	return true;
}

//
//
bool WholeBodyConstraints::get_WbJointLimitsConstraints(  double run_period_sec,
															 VectorXd jts_position,
															 VectorXd jts_velocity)
{
	//
	JointLimitsVector.head(n_actuatedDofs) =   maxJointLimits - (jts_position + run_period_sec * 0.5*(jts_velocity + jts_velocity_0));
	JointLimitsVector.tail(n_actuatedDofs) =  -minJointLimits + (jts_position + run_period_sec * 0.5*(jts_velocity + jts_velocity_0));

	jts_velocity_0 = jts_velocity;

	return true;
}
//

// Hands
// --------------------------------------------------------------------------
bool WholeBodyConstraints::get_lhand_ContactConstraints(Matrix3d world_R_lhand)
{
	Matrix6d lhand_X_world;   lhand_X_world.setZero();

	lhand_X_world.block<3,3>(0,0) = world_R_lhand.transpose();
	lhand_X_world.block<3,3>(3,3) = world_R_lhand.transpose();

	// Inequality Constraint matrix left foot
	IneqConstraintMatrix_lh.block<1,6>(0,0)  = -1.0 * lhand_X_world.block<1,6>(2,0);
	IneqConstraintMatrix_lh.block<1,6>(1,0)  = -mu_hand/sqrt(2.0) * lhand_X_world.block<1,6>(2,0) - lhand_X_world.block<1,6>(0,0);
	IneqConstraintMatrix_lh.block<1,6>(2,0)  = -mu_hand/sqrt(2.0) * lhand_X_world.block<1,6>(2,0) + lhand_X_world.block<1,6>(0,0);
	IneqConstraintMatrix_lh.block<1,6>(3,0)  = -mu_hand/sqrt(2.0) * lhand_X_world.block<1,6>(2,0) - lhand_X_world.block<1,6>(1,0);
	IneqConstraintMatrix_lh.block<1,6>(4,0)  = -mu_hand/sqrt(2.0) * lhand_X_world.block<1,6>(2,0) + lhand_X_world.block<1,6>(1,0);
	IneqConstraintMatrix_lh.block<1,6>(5,0)  = 	   	  -gamma_hand * lhand_X_world.block<1,6>(2,0) - lhand_X_world.block<1,6>(5,0);
	IneqConstraintMatrix_lh.block<1,6>(6,0)  = 	      -gamma_hand * lhand_X_world.block<1,6>(2,0) + lhand_X_world.block<1,6>(5,0);
	IneqConstraintMatrix_lh.block<1,6>(7,0)  = 	     -deltaX_hand * lhand_X_world.block<1,6>(2,0) - lhand_X_world.block<1,6>(4,0);
	IneqConstraintMatrix_lh.block<1,6>(8,0)  = 	     -deltaX_hand * lhand_X_world.block<1,6>(2,0) + lhand_X_world.block<1,6>(4,0);
	IneqConstraintMatrix_lh.block<1,6>(9,0)  = 	     -deltaY_hand * lhand_X_world.block<1,6>(2,0) - lhand_X_world.block<1,6>(3,0);
	IneqConstraintMatrix_lh.block<1,6>(10,0) = 	     -deltaY_hand * lhand_X_world.block<1,6>(2,0) + lhand_X_world.block<1,6>(3,0);

	// Inequality Constraint vector left foot
	IneqConstraintVector_lh.setZero();


	return true;
}
		
bool WholeBodyConstraints::get_rhand_ContactConstraints(Matrix3d world_R_rhand)
{
	Matrix6d rhand_X_world;   rhand_X_world.setZero();

	rhand_X_world.block<3,3>(0,0) = world_R_rhand.transpose();
	rhand_X_world.block<3,3>(3,3) = world_R_rhand.transpose();

	// Inequality Constraint matrix right foot
	IneqConstraintMatrix_rh.block<1,6>(0,0)  = -1.0 * rhand_X_world.block<1,6>(2,0);
	IneqConstraintMatrix_rh.block<1,6>(1,0)  = -mu_hand/sqrt(2.0) * rhand_X_world.block<1,6>(2,0) - rhand_X_world.block<1,6>(0,0);
	IneqConstraintMatrix_rh.block<1,6>(2,0)  = -mu_hand/sqrt(2.0) * rhand_X_world.block<1,6>(2,0) + rhand_X_world.block<1,6>(0,0);
	IneqConstraintMatrix_rh.block<1,6>(3,0)  = -mu_hand/sqrt(2.0) * rhand_X_world.block<1,6>(2,0) - rhand_X_world.block<1,6>(1,0);
	IneqConstraintMatrix_rh.block<1,6>(4,0)  = -mu_hand/sqrt(2.0) * rhand_X_world.block<1,6>(2,0) + rhand_X_world.block<1,6>(1,0);
	IneqConstraintMatrix_rh.block<1,6>(5,0)  = 	      -gamma_hand * rhand_X_world.block<1,6>(2,0) - rhand_X_world.block<1,6>(5,0);
	IneqConstraintMatrix_rh.block<1,6>(6,0)  = 	  	  -gamma_hand * rhand_X_world.block<1,6>(2,0) + rhand_X_world.block<1,6>(5,0);
	IneqConstraintMatrix_rh.block<1,6>(7,0)  = 	  	 -deltaX_hand * rhand_X_world.block<1,6>(2,0) - rhand_X_world.block<1,6>(4,0);
	IneqConstraintMatrix_rh.block<1,6>(8,0)  = 	  	 -deltaX_hand * rhand_X_world.block<1,6>(2,0) + rhand_X_world.block<1,6>(4,0);
	IneqConstraintMatrix_rh.block<1,6>(9,0)  = 	  	 -deltaY_hand * rhand_X_world.block<1,6>(2,0) - rhand_X_world.block<1,6>(3,0);
	IneqConstraintMatrix_rh.block<1,6>(10,0) = 	  	 -deltaY_hand * rhand_X_world.block<1,6>(2,0) + rhand_X_world.block<1,6>(3,0);

	// Inequality constrint vector right foot
	IneqConstraintVector_rh.setZero();


	return true;
}

//
bool WholeBodyConstraints::get_WbHandsContactConstraints(Matrix3d world_R_lhand, Matrix3d world_R_rhand)
{
	// 
	bool ok = false;
	//
	ok = this->get_lhand_ContactConstraints(world_R_lhand);
	//
	ok = ok && this->get_rhand_ContactConstraints(world_R_rhand);

	// Overall inequality constraint matrix
	IneqConstraintMatrix_AllHands.block<11,6>(0,0)  = this->IneqConstraintMatrix_lh;
	IneqConstraintMatrix_AllHands.block<11,6>(11,6) = this->IneqConstraintMatrix_rh;

	// Overall Inequality constraint vector
	IneqConstraintVector_AllHands.setZero();
	//
	return ok;
}


//
bool WholeBodyConstraints::get_lfoot_ContactConstraints(Matrix3d world_R_lfoot)
{
	//
	// double offset_x = 0.03;
	//
	Matrix6d lfoot_X_world;   lfoot_X_world.setZero();

	lfoot_X_world.block<3,3>(0,0) = world_R_lfoot.transpose();
	lfoot_X_world.block<3,3>(3,3) = world_R_lfoot.transpose();


	// //Inequality Constraint matrix left foot
	IneqConstraintMatrix_lf.block<1,6>(0,0)  = 		-lfoot_X_world.block<1,6>(2,0);
	IneqConstraintMatrix_lf.block<1,6>(1,0)  = 		-mu_feet/sqrt(2.0) * lfoot_X_world.block<1,6>(2,0) 	- 	lfoot_X_world.block<1,6>(0,0);
	IneqConstraintMatrix_lf.block<1,6>(2,0)  = 		-mu_feet/sqrt(2.0) * lfoot_X_world.block<1,6>(2,0) 	+ 	lfoot_X_world.block<1,6>(0,0);
	IneqConstraintMatrix_lf.block<1,6>(3,0)  = 		-mu_feet/sqrt(2.0) * lfoot_X_world.block<1,6>(2,0) 	- 	lfoot_X_world.block<1,6>(1,0);
	IneqConstraintMatrix_lf.block<1,6>(4,0)  = 		-mu_feet/sqrt(2.0) * lfoot_X_world.block<1,6>(2,0) 	+ 	lfoot_X_world.block<1,6>(1,0);
	IneqConstraintMatrix_lf.block<1,6>(5,0)  = 	   	  	-gamma_feet * lfoot_X_world.block<1,6>(2,0) 	- 	lfoot_X_world.block<1,6>(5,0);
	IneqConstraintMatrix_lf.block<1,6>(6,0)  = 	      	-gamma_feet * lfoot_X_world.block<1,6>(2,0) 	+ 	lfoot_X_world.block<1,6>(5,0);
	IneqConstraintMatrix_lf.block<1,6>(7,0)  = -(DeltaX_feet+offset_x) * lfoot_X_world.block<1,6>(2,0) 	- 	lfoot_X_world.block<1,6>(4,0);
	IneqConstraintMatrix_lf.block<1,6>(8,0)  = -(DeltaX_feet-offset_x) * lfoot_X_world.block<1,6>(2,0) 	+ 	lfoot_X_world.block<1,6>(4,0);
	IneqConstraintMatrix_lf.block<1,6>(9,0)  = 	     	  -DeltaY_feet * lfoot_X_world.block<1,6>(2,0) 	- 	lfoot_X_world.block<1,6>(3,0);
	IneqConstraintMatrix_lf.block<1,6>(10,0) = 	     	  -DeltaY_feet * lfoot_X_world.block<1,6>(2,0) 	+ 	lfoot_X_world.block<1,6>(3,0);

	// std::cout << " IneqConstraintMatrix_lf  is \n" << IneqConstraintMatrix_lf << std::endl;
	// IneqConstraintMatrix_lf *=0.0;

	// Inequality Constraint vector left foot
	IneqConstraintVector_lf.setZero();

	return true;
}

bool WholeBodyConstraints::get_rfootContactConstraints(Matrix3d world_R_rfoot)
{
	
	//
	double offset_x = 0.03;
	//
	Matrix6d rfoot_X_world;   rfoot_X_world.setZero();

	rfoot_X_world.block<3,3>(0,0) = world_R_rfoot.transpose();
	rfoot_X_world.block<3,3>(3,3) = world_R_rfoot.transpose();
	//

	// //Inequality Constraint matrix right foot
	IneqConstraintMatrix_rf.block<1,6>(0,0)  = -rfoot_X_world.block<1,6>(2,0);
	IneqConstraintMatrix_rf.block<1,6>(1,0)  = -mu_feet/sqrt(2.0) * rfoot_X_world.block<1,6>(2,0) - rfoot_X_world.block<1,6>(0,0);
	IneqConstraintMatrix_rf.block<1,6>(2,0)  = -mu_feet/sqrt(2.0) * rfoot_X_world.block<1,6>(2,0) + rfoot_X_world.block<1,6>(0,0);
	IneqConstraintMatrix_rf.block<1,6>(3,0)  = -mu_feet/sqrt(2.0) * rfoot_X_world.block<1,6>(2,0) - rfoot_X_world.block<1,6>(1,0);
	IneqConstraintMatrix_rf.block<1,6>(4,0)  = -mu_feet/sqrt(2.0) * rfoot_X_world.block<1,6>(2,0) + rfoot_X_world.block<1,6>(1,0);
	IneqConstraintMatrix_rf.block<1,6>(5,0)  = 	      -gamma_feet * rfoot_X_world.block<1,6>(2,0) - rfoot_X_world.block<1,6>(5,0);
	IneqConstraintMatrix_rf.block<1,6>(6,0)  = 	  	  -gamma_feet * rfoot_X_world.block<1,6>(2,0) + rfoot_X_world.block<1,6>(5,0);
	IneqConstraintMatrix_rf.block<1,6>(7,0)  = -(DeltaX_feet+offset_x) * rfoot_X_world.block<1,6>(2,0) - rfoot_X_world.block<1,6>(4,0);
	IneqConstraintMatrix_rf.block<1,6>(8,0)  = -(DeltaX_feet-offset_x) * rfoot_X_world.block<1,6>(2,0) + rfoot_X_world.block<1,6>(4,0);
	IneqConstraintMatrix_rf.block<1,6>(9,0)  = 	  	 -DeltaY_feet * rfoot_X_world.block<1,6>(2,0) - rfoot_X_world.block<1,6>(3,0);
	IneqConstraintMatrix_rf.block<1,6>(10,0) = 	  	 -DeltaY_feet * rfoot_X_world.block<1,6>(2,0) + rfoot_X_world.block<1,6>(3,0);

	// std::cout << " IneqConstraintMatrix_rf  is \n" << IneqConstraintMatrix_rf << std::endl;
	// IneqConstraintMatrix_rf *=0.0;

	// Inequality constrint vector right foot
	IneqConstraintVector_rf.setZero();

	return true;
}

//
bool WholeBodyConstraints::get_WbFeetContactConstraints(Matrix3d world_R_lfoot, Matrix3d world_R_rfoot)
{
	// 
	bool ok = false;
	//
	ok = this->get_lfoot_ContactConstraints(world_R_lfoot);
	//
	ok = ok && this->get_rfootContactConstraints(world_R_rfoot);

	// Overall inequality constraint matrix
	IneqConstraintMatrix_AllFeet.block<11,6>(0,0)  = this->IneqConstraintMatrix_lf;
	IneqConstraintMatrix_AllFeet.block<11,6>(11,6) = this->IneqConstraintMatrix_rf;

	// Overall Inequality constraint vector
	IneqConstraintVector_AllFeet.setZero();
	//
	return ok;
}


//
bool WholeBodyConstraints::get_WbInequalityConstraints(   		 		  double 	run_period_sec,
																 JointspaceStates	JtsStates_,
														WholeBodyTaskSpaceStates	wbTS_)
{
	//
	Matrix4d world_H_lhand = Transforms.PoseVector2HomogenousMx(wbTS_.lhand.Pose); 
	Matrix4d world_H_rhand = Transforms.PoseVector2HomogenousMx(wbTS_.rhand.Pose);
	Matrix4d world_H_lfoot = Transforms.PoseVector2HomogenousMx(wbTS_.lfoot.Pose); 
	Matrix4d world_H_rfoot = Transforms.PoseVector2HomogenousMx(wbTS_.rfoot.Pose);
	//
	bool ok = false;   // ok = ok && 

	if (!(ok = get_WbTorqueConstraints())) 
	{
		printf("Failed to compute the whole body torque constraints. \n");
		return false;
	}
	//
	if (!(ok = ok && get_WbVelocityLimitsConstraints(run_period_sec,
												   	 JtsStates_.velocity))) 
	{
		printf("Failed to compute the whole body Velocity Limit Constraints. \n");
		return false;
	}
	//
	if (!(ok = ok && get_WbJointLimitsConstraints(  run_period_sec,
													JtsStates_.position, 
													JtsStates_.velocity))) 
	{
		printf("Failed to compute the whole body Joint Limit Constraints. \n");
		return false;
	}
	//
	if (!(ok = ok && get_WbHandsContactConstraints(world_H_lhand.block<3,3>(0,0), world_H_rhand.block<3,3>(0,0)))) 
	{
		printf("Failed to compute the whole body hands contact constraints. \n");
		return false;
	}

	if (!(ok = ok && get_WbFeetContactConstraints(world_H_lfoot.block<3,3>(0,0), world_H_rfoot.block<3,3>(0,0)))) 
	{
		printf("Failed to compute the whole body feet contact constraints. \n");
		return false;
	}
	
	return ok;

}


//
// yarp::os::BufferedPort<yarp::os::Bottle> l_foot_FT_inputPort;
// yarp::os::Bottle 						*l_foot_FT_data;
// Eigen::VectorXd 						 l_foot_FT_vector;


// l_foot_FT_inputPort.open("/l_foot_FT_readPort:i");
//     std::string l_foot_FT_portName ="/";
// 	            l_foot_FT_portName += robotName;
// 	            l_foot_FT_portName += "/left_foot/analog:o";

// this->l_foot_FT_data = l_foot_FT_inputPort.read();

// this->l_foot_FT_vector.resize(this->l_foot_FT_data->size());


// //

// // run
// Vector6d RobotSensors::getLeftLegForceTorqueValues()
// {
// 	l_foot_FT_data  = l_foot_FT_inputPort.read(); 

//     for (int i=0; i<l_foot_FT_data->size(); i++) {
//         l_foot_FT_vector(i)= l_foot_FT_data->get(i).asDouble();
//     }
//     return l_foot_FT_vector;
// }
// //
// l_foot_FT_inputPort.close();




// OpenPort(yarp::os::BufferedPort<yarp::os::Bottle> 	inputPort,
// 								 yarp::os::Bottle  *port_data_pointer,
// 								  Eigen::VectorXd 	vector_data_values,
// 								      std::string   local_port_name,
// 								      std::string   remote_port_name,
// 								      std::string   robot_name)
// {
// 	inputPort.open
// }


bool WholeBodyConstraints::set_CoM_limits(int stance, double DeltaSupport[], Matrix4d w_H_lf, Matrix4d w_H_rf, Vector7d &ref_pose_com_w)
{

	// DeltaSupport[0] : x forward  limit
	// DeltaSupport[1] : x backward limit
	// DeltaSupport[2] : y left  limit
	// DeltaSupport[3] : y right limit

	Matrix4d w_H_Stance; w_H_Stance.setIdentity();
	Vector3d st_ref_posi_com;

	double min_x, min_y, max_x, max_y;
	Matrix4d st_H_lf; st_H_lf.setIdentity();
	Matrix4d st_H_rf; st_H_rf.setIdentity();
	
	switch(stance)
	{
		case 0 : //DSP :
		{
			// support foot/feet
			w_H_Stance.block<3,3>(0,0) = w_H_lf.block<3,3>(0,0);
			w_H_Stance.block<3,1>(0,3) = 0.5*(w_H_lf.block<3,1>(0,3) + w_H_rf.block<3,1>(0,3));
			// st_H_lf and st_H_rf
			st_H_lf = w_H_Stance.inverse()* w_H_lf;
			st_H_rf = w_H_Stance.inverse()* w_H_rf;

			min_y = st_H_rf(1,3) - DeltaSupport[3]; 
			max_y = st_H_lf(1,3) + DeltaSupport[2]; 
		}
		break;

		case 1 : //LSP :
		{
			// support foot/feet
			w_H_Stance.block<3,3>(0,0) = w_H_lf.block<3,3>(0,0);
			w_H_Stance.block<3,1>(0,3) = w_H_lf.block<3,1>(0,3);
			// st_H_lf and st_H_rf
			st_H_lf = w_H_Stance.inverse()* w_H_lf;
			st_H_rf = w_H_Stance.inverse()* w_H_rf;

			min_y = st_H_lf(1,3) - DeltaSupport[3]; 
			max_y = st_H_lf(1,3) + DeltaSupport[2]; 
		}
		break;

		case 2 : //RSP :
		{
			// support foot/feet
			w_H_Stance.block<3,3>(0,0) = w_H_lf.block<3,3>(0,0);
			w_H_Stance.block<3,1>(0,3) = w_H_rf.block<3,1>(0,3);
			// st_H_lf and st_H_rf
			st_H_lf = w_H_Stance.inverse()* w_H_lf;
			st_H_rf = w_H_Stance.inverse()* w_H_rf;

			min_y = st_H_rf(1,3) - DeltaSupport[3]; 
			max_y = st_H_rf(1,3) + DeltaSupport[2]; 		
		}
		break;

		default :
		{
			// support foot/feet
			w_H_Stance.block<3,3>(0,0) = w_H_lf.block<3,3>(0,0);
			w_H_Stance.block<3,1>(0,3) = 0.5*(w_H_lf.block<3,1>(0,3) + w_H_rf.block<3,1>(0,3));

			// st_H_lf and st_H_rf
			st_H_lf = w_H_Stance.inverse()* w_H_lf;
			st_H_rf = w_H_Stance.inverse()* w_H_rf;

			min_y = st_H_rf(1,3) - DeltaSupport[3]; 
			max_y = st_H_lf(1,3) + DeltaSupport[2]; 
		}
		break;
	}

	min_x = -DeltaSupport[1]; 
	max_x = DeltaSupport[0]; 

	// ref com in stance
	st_ref_posi_com = w_H_Stance.block<3,3>(0,0).transpose()*(ref_pose_com_w.head<3>() - w_H_Stance.block<3,1>(0,3));
	//
	// compute the limits
	// =====================
	if(st_ref_posi_com(0) <=min_x) st_ref_posi_com(0) = min_x; else if(st_ref_posi_com(0) >= max_x) st_ref_posi_com(0) = max_x;
	if(st_ref_posi_com(1) <=min_y) st_ref_posi_com(1) = min_y; else if(st_ref_posi_com(1) >= max_y) st_ref_posi_com(1) = max_y;
	//
	// Projection back to world frame
	ref_pose_com_w.head<3>() = w_H_Stance.block<3,3>(0,0) * st_ref_posi_com + w_H_Stance.block<3,1>(0,3);

	return true;

}



bool WholeBodyConstraints::get_CoP_constraints_matrix(Vector2d d_CoP, Matrix3d w_R_f, Eigen::Matrix<double, 2, 6>& CoP_cMx)
{
	Eigen::Matrix<double, 2, 6> l_CoPMx = Eigen::MatrixXd::Zero(2,6);
	l_CoPMx(0,2) = d_CoP(0);	l_CoPMx(0,4) = 1.0;
	l_CoPMx(1,2) = d_CoP(1);	l_CoPMx(1,3) = -1.0;

	CoP_cMx.leftCols(3)  = l_CoPMx.leftCols(3)  * w_R_f.transpose();
	CoP_cMx.rightCols(3) = l_CoPMx.rightCols(3) * w_R_f.transpose();
	return true;
}


//
void WholeBodyConstraints::get_feet_support_points(	MatrixXd PtsInFoot, Vector7d Pose_lfoot, Vector7d Pose_rfoot, 
													MatrixXd &AllPtsInAbsFoot_, Matrix2d &W_Rot_AbsFoot, Vector2d &W_Pos_AbsFoot)
{
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

bool WholeBodyConstraints::getConvexHullVariables(MatrixXd PtsInFoot_, Vector7d Pose_lfoot, Vector7d Pose_rfoot,  
												  MatrixXd &W_EdgesNormalCHull, VectorXd &DistanceEdgesCHull, 
												  Matrix2d &W_Rot_AbsFoot, Vector2d &W_Pos_AbsFoot)
{
	// computation of the convex polygon
    MatrixXd AllPtsInAbsFoot;
   	MatrixXd PointsCHull;
    MatrixXd Abs_EdgesNormalCHull;
    
	// Expressing the feet contact points in the absolute feet frame
	this->get_feet_support_points(PtsInFoot_, Pose_lfoot, Pose_rfoot, AllPtsInAbsFoot, W_Rot_AbsFoot, W_Pos_AbsFoot);  // chech also with des_X_EE
	// In the absolute feet frame compute the convex hull and the normals to the edges and 
	ConvHull.convexHull(AllPtsInAbsFoot, PointsCHull, Abs_EdgesNormalCHull, DistanceEdgesCHull); 
	// Express the normal in the world frame
	W_EdgesNormalCHull.resize(Abs_EdgesNormalCHull.rows(), Abs_EdgesNormalCHull.cols());
	W_EdgesNormalCHull = Abs_EdgesNormalCHull * W_Rot_AbsFoot.transpose();

	for(int i=0; i<W_EdgesNormalCHull.rows(); i++)
	{
		W_EdgesNormalCHull.row(i) = W_EdgesNormalCHull.row(i)/(W_EdgesNormalCHull.row(i).norm() + 1.e-10);
	}

	// std::cout << " POINTS CONVEX HULL IN WBCONSTRAINT: \n" << PointsCHull << std::endl;
	return true;
}

//
bool WholeBodyConstraints::predict_stability(MatrixXd W_EdgesNormalCHull_, VectorXd DistanceEdgesCHull_, Vector2d W_CoM_, Vector2d W_Pos_AbsFt_)
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


// clamping CoM
bool WholeBodyConstraints::clamping_CoM_sstability(MatrixXd PtsInFoot_, Vector7d Pose_lfoot, Vector7d Pose_rfoot, Vector7d Pose_CoM, double margin_dist, Vector7d &ref_pose_com_w)
{
	//
	MatrixXd W_EdgesNormalCHull(7,2); 
    VectorXd DistanceEdgesCHull(7);
    Matrix2d W_Rot_AbsFoot;
    Vector2d W_Pos_AbsFoot;
    this->getConvexHullVariables(PtsInFoot_, Pose_lfoot, Pose_rfoot,  W_EdgesNormalCHull, DistanceEdgesCHull, W_Rot_AbsFoot, W_Pos_AbsFoot);

    if(this->predict_stability(W_EdgesNormalCHull, DistanceEdgesCHull, Pose_CoM.head(2),  W_Pos_AbsFoot))
    {
        ref_pose_com_w.head(3) = Pose_CoM.head(3);
        pxy_CoM_n1 = pxy_CoM;
        pxy_CoM    = (Pose_CoM.head(2) - W_Pos_AbsFoot);
    }
    else
    {
        std::cout<< "\n" << std::endl;
        std::cout<< "COM CLAMPING ACTIVE with : " << pxy_CoM_n1.transpose() << std::endl;
        Vector2d DeltaCoMxy = (pxy_CoM_n1.norm() - margin_dist)/(pxy_CoM_n1.norm()+1e-10) * pxy_CoM_n1;
        ref_pose_com_w.head(2) = DeltaCoMxy + W_Pos_AbsFoot;
        ref_pose_com_w(2)      = Pose_CoM(2);
    }


	return true;
}