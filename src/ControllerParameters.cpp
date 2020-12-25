
#include <string>
#include <iostream>
#include <fstream>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Geometry>

// #include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include "ControllerParameters.h"

using namespace std;
using namespace Eigen;


ControllerParameters::ControllerParameters(){};

ControllerParameters::~ControllerParameters(){};


bool ControllerParameters::Initialize(std::string robotName, std::string moduleName, int n_actuatedDofs, bool PositionMode_)
{
	//
	ModuleName 					= moduleName;
	RobotName 					= robotName;
	actuatedDofs 				= n_actuatedDofs;
	writeCommands 				= false;
	runtimeCommands 			= true;
	AllPositionArmsTorqueMode 	= false;
	OrientChest   				= true;
	apply_wrench  				= true;
	Task_Hierarchy_Active 		= false;
	inputPortsActive 			= false;
	no_tracking_object 			= false;
	AccelCommand 				= true;
	init_posture_hands 			= false;
	PositionCommands			= PositionMode_; //false;
	CmdsFromWBID				= false;

	epsil_reach 				= 0.06;

	period_step                 = 40;

	if(PositionCommands) apply_wrench = false;

	// //////////////////////////////////////////////////////////////////////////
	// MODEL PARAMETERS
	// //////////////////////////////////////////////////////////////////////////
	std::string torso[]     	= {"torso_yaw", "torso_roll", "torso_pitch"};
    std::string neck[]      	= {"neck_pitch", "neck_roll", "neck_yaw"};
    std::string left_arm[]  	= {"l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw", "l_elbow", "l_wrist_prosup", "l_wrist_pitch", "l_wrist_yaw"};
    std::string right_arm[] 	= {"r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw", "r_elbow", "r_wrist_prosup", "r_wrist_pitch", "r_wrist_yaw"};
    std::string left_leg[]  	= {"l_hip_pitch", "l_hip_roll", "l_hip_yaw", "l_knee", "l_ankle_pitch", "l_ankle_roll"};
    std::string right_leg[] 	= {"r_hip_pitch", "r_hip_roll", "r_hip_yaw", "r_knee", "r_ankle_pitch", "r_ankle_roll"};

    // update the list of the main joints
	main_joints_list.assign(torso, neck, left_arm, right_arm, left_leg, right_leg);
	// get the list of the joints considered for torque control
	this->get_list_of_considered_joints(list_of_considered_joints);
	// Name of the pelvis port
	Pelvis_port_name 	= 	"/get_root_link_WorldPose:o";

		// PIDS
	init_legs_pid.resize(12);
	step_legs_pid.resize(12);
	// lleg
	init_legs_pid.kp.head(6) << 17.453, 17.453, 17.453, 17.453, 17.453, 17.453;
	init_legs_pid.kd.head(6) <<  0.174,  0.174,  0.174,  0.174,  0.174,  0.174;
	init_legs_pid.ki.head(6) <<  0.174,  0.174,  0.174,  0.174,  0.174,  0.174;
	// rleg
	init_legs_pid.kp.tail(6) << 17.453, 17.453, 17.453, 17.453, 17.453, 17.453;
	init_legs_pid.kd.tail(6) <<  0.174,  0.174,  0.174,  0.174,  0.174,  0.174;
	init_legs_pid.ki.tail(6) <<  0.174,  0.174,  0.174,  0.174,  0.174,  0.174;
	// lleg
	// step_legs_pid.kp.head(6) << 17.453, 17.453, 17.453, 17.453/1., 17.453, 17.453;
	// step_legs_pid.kd.head(6) <<  0.174,  0.174,  0.174,  0.174/1.,  0.174,  0.174;
	// step_legs_pid.ki.head(6) <<  0.000,  0.000,  0.000,  0.000/1.,  0.000,  0.000;
	// // rleg
	// step_legs_pid.kp.tail(6) << 17.453, 17.453, 17.453, 17.453/1., 17.453, 17.453;
	// step_legs_pid.kd.tail(6) <<  0.174,  0.174,  0.174,  0.174/1.,  0.174,  0.174;
	// step_legs_pid.ki.tail(6) <<  0.000,  0.000,  0.000,  0.000/1.,  0.000,  0.000;
	//
	// lleg
	step_legs_pid.kp.head(6) << 17.453, 17.453, 17.453, 17.453, 17.453, 17.453;
	step_legs_pid.kd.head(6) <<  0.174,  0.174,  0.174,  0.174,  0.174,  0.174;
	step_legs_pid.ki.head(6) <<  0.174,  0.174,  0.174,  0.174,  0.174,  0.174;
	//
	// rleg
	step_legs_pid.kp.tail(6) << 17.453, 17.453, 17.453, 17.453, 17.453, 17.453;
	step_legs_pid.kd.tail(6) <<  0.174,  0.174,  0.174,  0.174,  0.174,  0.174;
	step_legs_pid.ki.tail(6) <<  0.174,  0.174,  0.174,  0.174,  0.174,  0.174;

	// step_legs_pid.kp.head(6) *=0.5;
	// step_legs_pid.kd.head(6) *=0.5;
	// step_legs_pid.ki.head(6) *=0.5;

	// step_legs_pid.kp.tail(6) *=0.5;
	// step_legs_pid.kd.tail(6) *=0.5;
	// step_legs_pid.ki.tail(6) *=0.5;

	if(RobotName == "icub")
	{
		// lleg
		init_legs_pid.kp.head(6) << -1066.66, 2066.66,  -711.11, -1066.66, 2200, 2200;
		init_legs_pid.kd.head(6) <<  	 0.0,     0.0, 	    0.0, 	  0.0,  0.0,  0.0;
		init_legs_pid.ki.head(6) << -10666.6, 14222.2, -7111.09, -1066.64, 0.09, 0.09;
		// rleg
		init_legs_pid.kp.tail(6) << 1066.66, -2066.66,  711.11, 1066.66, -2105, -2310;
		init_legs_pid.kd.tail(6) <<    0.00,     0.00,    0.00,    0.00,  0.00,  0.00;
		init_legs_pid.ki.tail(6) << 10666.6, -14222.2, 7111.09, 1066.64, -0.09, -0.09;

		// lleg
		step_legs_pid.kp.head(6) << -1066.66, 2066.66,  -711.11, -1066.66, 2200, 2200;
		step_legs_pid.kd.head(6) <<  	 0.0,     0.0, 	    0.0, 	  0.0,  0.0,  0.0;
		step_legs_pid.ki.head(6) << -10666.6, 14222.2, -7111.09, -1066.64, 0.09, 0.09;

		step_legs_pid.kp.head(6) *=1.0;
		step_legs_pid.kd.head(6) *=1.0;
		step_legs_pid.ki.head(6) *=0.0;
		// rleg
		step_legs_pid.kp.tail(6) << 1066.66, -2066.66,  711.11, 1066.66, -2105, -2310;
		step_legs_pid.kd.tail(6) <<    0.00,     0.00,    0.00,    0.00,  0.00,  0.00;
		step_legs_pid.ki.tail(6) << 10666.6, -14222.2, 7111.09, 1066.64, -0.09, -0.09;

		step_legs_pid.kp.tail(6) *=1.0;
		step_legs_pid.kd.tail(6) *=1.0;
		step_legs_pid.ki.tail(6) *=0.0;
	}

	// ////////////////////////////////////////////////////////////////////////////////////////// //
	//                                  Inverse dynamics Parameters                               //
	// /////////////////////////////////////////////////////////////////////////////////////////////

	// Surface parameters
	Hands_contact_param(0) = 0.4;					// mu hands    	:	friction coefficient
	Hands_contact_param(1) = 0.4;					// gamma hands 	: 	moment friction coeff
	Hands_contact_param(2) = 0.025;					// deltaX_hands : 	length of the contact surface
	Hands_contact_param(3) = 0.025;					// deltaY_hands : 	width of the contact surface
	// feet contact surface parameters
	Feet_contact_param(0)  = 0.4;					// mu feet     	:	friction coefficient	0.4
	Feet_contact_param(1)  = 0.2;					// Gamma feet  	: 	moment friction coeff	0.4
	Feet_contact_param(2)  = 0.06; //0.7			// DeltaX_ feet : 	length of the contact surface
	Feet_contact_param(3)  = 0.03;					// DeltaY_ feet : 	width of the contact surface
	Feet_contact_param(4)  = 0.03;					// offset of the foot origin in the x direction
	//
	DeltaSupport[0] = 0.06;  	// Delta x forward  limit 0.06 0.07
	DeltaSupport[1] = 0.03;  	// Delta x backward limit 0.03 0.045
	DeltaSupport[2] = 0.015; 	// Delta y left  limit
	DeltaSupport[3] = 0.015; 	// Delta y right limit

	// foot considered dmensions (points of foot convex-hull)
	PtsInFoot.resize(2,4);
	PtsInFoot(0,0) =  0.100;  PtsInFoot(1,0) =  0.015;  // point 1		  p1 ---- p4
    PtsInFoot(0,1) = -0.015;  PtsInFoot(1,1) =  0.015;  // point 2			|	 |
    PtsInFoot(0,2) = -0.015;  PtsInFoot(1,2) = -0.015;  // point 3			|	 |
    PtsInFoot(0,3) =  0.100;  PtsInFoot(1,3) = -0.015;  // point 4  	 p2 |____| p3

	// gains for the Quadratic Programmind solver
	Weight_SoT.resize(30);							
	Weight_accel.resize(actuatedDofs+6);			
	Weight_tau.resize(actuatedDofs+6);				
	Weight_posture.resize(actuatedDofs+6);			
	Weight_F_contact.resize(12);					

	Weight_SoT.setZero(); 
	Weight_accel.setOnes();
	Weight_F_contact.setOnes();
	Weight_tau.setOnes();
	Weight_posture.setOnes();

	Weight_accel 		*= 1e-4; //1e-6; //0.01    // <==============================
	Weight_F_contact 	*= 1e-1;  //0.010
	// Weight_F_contact *= 0.00000001;   
	Weight_tau 			*= 1e-8;  // 0.001
	Weight_tau.head(3)  *= 1e+2;
	Weight_tau.tail(12) *= 1e+2;
	// Weight_tau(0) *= 0.1; //0.001
	// Weight_tau(20) 	 	*= 10.01;					// left knee
	// Weight_tau(26) 	 	*= 10.01;					// right knee
	// Weight_tau(21) 	 	*= 50.05;					// left ankle pitch
	// Weight_tau(27) 	 	*= 50.05;					// right ankle pitch	
	// Weight_tau(21) 	 	*= 10.05;					// left ankle pitch
	// Weight_tau(27) 	 	*= 10.05;					// right ankle pitch

	// Weight of the end-effectors
	Weight_lfoot 		<< 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0; 		 
	Weight_rfoot 		<< 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0; 		 
	// Weight_lfoot 	<< 100.0, 100.0, 100.0, 100.0, 100.0, 100.0; 		// 
	// Weight_rfoot 	<< 100.0, 100.0, 100.0, 100.0, 100.0, 100.0; 		//
	Weight_lhand 		<<  10.4,  10.4,  10.4,   5.0,   5.0,   5.0;  
	Weight_rhand 		<<  10.4,  10.4,  10.4,   5.0,   5.0,   5.0;
	// Weight_centroidal 	<<  15.0,  15.0,   1.0,   8.0,   8.0,   8.0;   // 1.0z
	Weight_centroidal 	<<  30.0,  30.0,   1.0,   8.0,   8.0,   8.0;   // 1.0z
	Weight_pelvis       <<  1e-6,  1e-6,  1e-6,   1e-3,   1e-3,   1e-3; 
	Weight_Chest		<<  1e-6,  1e-6,  1e-6,   1e-3,   1e-3,   1e-3; 
	// Weight_centroidal.tail(3) *=0.1;
	if(CmdsFromWBID && PositionCommands)  Weight_centroidal(2) *=0.5;
	Weight_lhand *=2.0;
	Weight_rhand *=2.0;

	// weight stack of task 				//  
	Weight_SoT.segment( 0, 6) = Weight_centroidal;
	Weight_SoT.segment( 6, 6) = Weight_lhand;
	Weight_SoT.segment(12, 6) = Weight_rhand;
	Weight_SoT.segment(18, 6) = Weight_lfoot;
	Weight_SoT.segment(24, 6) = Weight_rfoot;

	// 
    gains_posture[0] = 0.05*Weight_lhand(0);		// Base wrt to the left leg (codyco reference frame)
    gains_posture[1] = 0.05*Weight_lhand(0);		// Torso
    gains_posture[2] = 0.05*Weight_lhand(0);		// Head
    gains_posture[3] = 0.05*Weight_lhand(0);		// arms 0.09;
    gains_posture[4] = 0.05*Weight_lhand(0);		// legs	

    // CoM compliant
    vM_CoM_c <<   2.0,    2.0,   2.0,    2.0,  2.0,  2.0; 
	vD_CoM_c <<   10.0,   10.0,  6.5,   15.0, 15.0, 15.0;  
	vK_CoM_c <<  10.0,    10.0,  5.0,   20.0, 20.0, 20.0; 
	// stiff
	vM_CoM_s <<   2.0,    2.0,   2.0,    2.0,  2.0,  2.0; 
	vD_CoM_s <<   15.0,   15.0,  10.0,   15.0, 15.0, 15.0;  // <<   65.0,   65.0,   65.0, 10.0, 10.0, 10.0; 
	vK_CoM_s <<  20.0,    20.0,  10.0,   20.0, 20.0, 20.0;  // <<  100.0,  100.0,  100.0, 20.0, 20.0, 20.0; 

	//
	Weight_acceleration = Eigen::VectorXd::Ones(n_actuatedDofs+6);
	Weight_acceleration.segment( 0, 6) << 50., 50., 10., 20., 20., 20.; 				// floating base
	Weight_acceleration *= 1e-4; // 1e-4

	posture_weight = Eigen::VectorXd::Ones(n_actuatedDofs);
	// posture_weight.segment( 0, 3) << 3.5, 2.5, 1.; 					// torso
	posture_weight *= 10.5;
	// posture_weight *= 2e-4;
	// posture_weight(2) *= 5.0;
	posture_weight_retract = posture_weight;
	posture_weight_retract(2) *= 2.;

	posture_weight_grasp = posture_weight;
	posture_weight_grasp(4)  *= 20.0;
	posture_weight_grasp(11) *= 20.0;



	task_weight[0] <<  15.e-0, 15.e-0, 5.e-1, 10.e-0, 10.e-0, 10.e-0;  	// weight_CoMp; 
	task_weight[1] <<     5.0,    5.0,   5.0,   2.50,   2.50,   2.50;  	// weight_lhand; 
	task_weight[2] <<     5.0,    5.0,   5.0,   2.50,   2.50,   2.50;  	// weight_rhand; 
	task_weight[3] <<   100.0,  100.0, 100.0,  200.0,  200.0,  200.0; 	// weight_lfoot; 
	task_weight[4] <<   100.0,  100.0, 100.0,  200.0,  200.0,  200.0; 	// weight_rfoot;
	task_weight[5] <<    0.00,   0.00,  0.00,  10.e-0, 5.e-0, 10.e-0;  	// weight_pelvis;  	2.e-1, 1.e-2, 5.e-2
	task_weight[6] <<    0.00,   0.00,  0.00,  2.e-0,  2.e-0,  2.e-0;  	// weight_chest;  	1.e-5,  1.e-5,  5.e-5

	task_weight[0] *=15.;  // 8.
	task_weight[1] *=15.0; // 0.15
	task_weight[2] *=15.0; // 0.15
	task_weight[3] *=50.;
	task_weight[4] *=50.;
	task_weight[5] *=5.0; // 20.
	task_weight[6] *=1.0;
	// for(int i=0; i<7; i++) task_weight[i] *=2e-2;

	task_weight_retract[0] <<  15.e-0, 15.e-0, 5.e-1, 10.e-0, 10.e-0, 10.e-0;  	// weight_CoMp; 
	task_weight_retract[1] <<     5.0,    5.0,   5.0,   1.00,   1.00,   1.00;  	// weight_lhand; 
	task_weight_retract[2] <<     5.0,    5.0,   5.0,   1.00,   1.00,   1.00;  	// weight_rhand; 
	task_weight_retract[3] <<   100.0,  100.0, 100.0,  200.0,  200.0,  200.0; 	// weight_lfoot; 
	task_weight_retract[4] <<   100.0,  100.0, 100.0,  200.0,  200.0,  200.0; 	// weight_rfoot;
	task_weight_retract[5] <<    0.00,   0.00,  0.00,  10.e-0, 5.e-0, 10.e-0;  	// weight_pelvis;  	2.e-1, 1.e-2, 5.e-2
	task_weight_retract[6] <<    0.00,   0.00,  0.00,  2.e-0,  2.e-0,  2.e-0;  	// weight_chest;  	1.e-5,  1.e-5,  5.e-5

	task_weight_retract[0] *=12.;
	task_weight_retract[1] *=10.0;
	task_weight_retract[2] *=10.0;
	task_weight_retract[3] *=50.;
	task_weight_retract[4] *=50.;
	task_weight_retract[5] *=5.0;  // 20.
	task_weight_retract[6] *=15.0; // 30.
	// for(int i=0; i<7; i++) task_weight_retract[i] *=2e-4;

    // ////////////////////////////////////////////////////////////////////////////////////
	// 				Center of pressure and weight distribution
	// ////////////////////////////////////////////////////////////////////////////////////
	coeff_weight_distribution     =   1.e-1; //5.0;
	weight_cop_lfoot.setZero();
	weight_cop_rfoot.setZero();
	Weight_CoP.setZero();

	weight_cop_lfoot << 20e-1,  20e-1; // 10e-4,  10e-4; // 10e-1,  10e-1; //  <=============================
	weight_cop_rfoot << 20e-1,  20e-1; // 10e-4,  10e-4; // 10e-1,  10e-1; //  <=============================
	Weight_CoP.segment(0, 2) = weight_cop_lfoot;
	Weight_CoP.segment(2, 2) = weight_cop_rfoot;
	Weight_CoP(4)            = coeff_weight_distribution;

	// parameters for the CoP reference Regulation 
	w_cop  = 0.80;
	wu_cop = w_cop;
	wc_cop = 1. - w_cop;

    // Velocity and torque saturation limits
    // ======================================
    velocitySaturationLimit.resize(actuatedDofs);
	torqueSaturationLimit.resize(actuatedDofs);

	velocitySaturationLimit.setZero();
	torqueSaturationLimit.setZero();					/* actuatedDOFs */

	bool result = true;

	if (!(result = this->getPostureWeights(actuatedDofs, gains_posture, Weight_posture))) {
        printf("Failed to set the posture weight. \n");
        return false;
    }

	// velocity limits
    double velo_limits[4];
    velo_limits[0] = M_PI/3.0;			// Torso
    velo_limits[1] = M_PI/4.0;			// Head
    velo_limits[2] = M_PI/4.0;			// Hands
    velo_limits[3] = 2.0*M_PI/3.0;		// Feets
    //
    if (!(result = result && this->getVelocityLimits(actuatedDofs, velo_limits, velocitySaturationLimit))) {
        //yError("Failed to compute joint limits.");
        printf("Failed to set velocity limits. \n");
        return false;
    }
    // torque limits
    double torque_limits[4];
    torque_limits[0] = 30.0;			// Torso 	24
    torque_limits[1] =  5.0;			// Head  	10
    torque_limits[2] = 20.0;			// Hands 	12
    torque_limits[3] = 40.0; 			// Feets 	40

    if (!(result = result && this->getTorqueLimits(actuatedDofs, torque_limits, torqueSaturationLimit))) {
        //yError("Failed to compute joint limits.");
        printf("Failed to set torque limits. \n");
        return false;
    }


    // //////////////////////////////////////////////////////////////////////////////////////////////////
    //                  Object to Grasp
    // ///////////////////////////////////////////////////////////////////////////////////////////////////
    select_object 			=   "LargeBoxSim";
    RobotIndex 				=   2; // 

    cout << " des_orientation_hands before \t" << 0.0 << endl; 
    // desired hands orientaion relative to object or world
    des_orientation_hands.resize(4);
    if(RobotIndex == 2)	des_orientation_hands << 0.0, -sqrt(2)/2., sqrt(2)/2., M_PI;
    else 		    	des_orientation_hands << -1.0, 0.0, 0.0, M_PI/2.;

    cout << " des_orientation_hands before \t" << des_orientation_hands.transpose() << endl;

	Vector3d Jo_obj;
	double ofst = 0.0;
	double wobj;
	double lobj;
	object_H_Markers.setIdentity(4,4);
	//
	if(select_object == "Box")
	{
		//
		object_port_name 		= 	"/";
		object_port_name 		+= 	"Box";
		object_port_name 		+= 	"/GraspedObject_WorldPose:o";
		//
		Vector3d dim_o  = Vector3d(0.16, 0.22, 0.12);
		object_name 	=	"Box";
		object_mass 	= 	0.5;
		Jo_obj 			= Vector3d(0.1666, 0.1666, 0.1666);
		object_inertia  = Jo_obj.asDiagonal();
		object_dimensions.length = dim_o(0);
		object_dimensions.width  = dim_o(1);
		object_dimensions.height = dim_o(2);
		object_dimensions.radius = dim_o(1);
		if(PositionCommands)	ofst = 1.0;
		// left
		object_Pose_Gpoints[0].head(3) = Vector3d(0.0, 0.5*object_dimensions.width, 0.0) + ofst*Vector3d(-0.0, 0.02, -0.00);
		object_Pose_Gpoints[0].tail(4) = des_orientation_hands;
		// right
		object_Pose_Gpoints[1].head(3) = Vector3d(0.0, -0.5*object_dimensions.width, 0.0) + ofst*Vector3d(-0.0, -0.02, -0.00);
		object_Pose_Gpoints[1].tail(4) = des_orientation_hands;
		// hands offset for simulator
		hand_offset[0].setZero(); 	hand_offset[0](2) =  -0.029;
		hand_offset[1].setZero(); 	hand_offset[1](2) =   0.029;
		//
		object_H_Markers.block<3,1>(0,0) = Vector3d(0.05, -0.01, -0.06); //Vector3d(-0.05, 0.01, -0.06);
	}
	else if(select_object == "BoxLarge")
	{
		//
		object_port_name 		= 	"/";
		object_port_name 		+= 	"BoxLarge";
		object_port_name 		+= 	"/GraspedObject_WorldPose:o";
		//
		Vector3d dim_o  = Vector3d(0.24, 0.64, 0.09);
		object_name 	=	"BoxLarge";
		object_mass 	= 	0.8;
		Jo_obj 			= Vector3d(0.1666, 0.1666, 0.1666);
		object_inertia  = Jo_obj.asDiagonal();
		object_dimensions.length = dim_o(0);
		object_dimensions.width  = dim_o(1);
		object_dimensions.height = dim_o(2);
		object_dimensions.radius = dim_o(1);
		if(PositionCommands)	ofst = 1.0;
		// left
		object_Pose_Gpoints[0].head(3) = Vector3d(0.0, 0.25*object_dimensions.width, -object_dimensions.height);
		object_Pose_Gpoints[0].tail(4) = des_orientation_hands;
		// right
		object_Pose_Gpoints[1].head(3) = Vector3d(0.0, -0.25*object_dimensions.width, -object_dimensions.height);
		object_Pose_Gpoints[1].tail(4) = des_orientation_hands;
		// hands offset for simulator
		hand_offset[0].setZero(); 	hand_offset[0](1) =  -0.040;
		hand_offset[1].setZero(); 	hand_offset[1](1) =  -0.040;
		//
		object_H_Markers.block<3,1>(0,0) = Vector3d(-0.0, 0.0, -0.0);
	}
	else if(select_object == "Shelf")
	{
		//
		object_port_name 		= 	"/";
		object_port_name 		+= 	"Shelf";
		object_port_name 		+= 	"/GraspedObject_WorldPose:o";
		//
		Vector3d dim_o  = Vector3d(0.24, 0.24, 0.75);
		object_name 	=	"Shelf";
		object_mass 	= 	5.0;
		Jo_obj 			= Vector3d(0.1666, 0.1666, 0.1666);
		object_inertia  = Jo_obj.asDiagonal();
		object_dimensions.length = dim_o(0);
		object_dimensions.width  = dim_o(1);
		object_dimensions.height = dim_o(2);
		object_dimensions.radius = dim_o(1);
		if(PositionCommands)	ofst = 1.0;
		// left
		object_Pose_Gpoints[0].head(3) = Vector3d(-0.5*object_dimensions.length, 0.5*object_dimensions.width, 0.10)
										 + ofst*Vector3d(0.01, -0.03, 0.00);
		object_Pose_Gpoints[0].tail(4) = des_orientation_hands;
		// right
		object_Pose_Gpoints[1].head(3) = Vector3d(-0.5*object_dimensions.length, -0.5*object_dimensions.width, 0.10)
										 + ofst*Vector3d(0.01, 0.03, 0.00);
		object_Pose_Gpoints[1].tail(4) = des_orientation_hands;
		// hands offset for simulator
		hand_offset[0].setZero(); 	hand_offset[0](0) =   0.0625; //-0.06;  //
		hand_offset[1].setZero(); 	hand_offset[1](0) =   0.0625; // 0.06;  //
		//
		object_H_Markers.block<3,1>(0,0) = Vector3d(-0.0, 0.0, -0.0);
	}
	else if(select_object == "BoxSim")
	{
		//
		object_port_name 		= 	"/";
		// object_port_name 		+= 	"icubSim/";
		object_port_name 		+= 	"GraspedObject_WorldPose:o";
		//
		Vector3d dim_o  = Vector3d(0.16, 0.16, 0.15);
		object_name 	=	"BoxSim";
		object_mass 	= 	0.5;
		Jo_obj 			= Vector3d(0.1666, 0.1666, 0.1666);
		object_inertia  = Jo_obj.asDiagonal();
		object_dimensions.length = dim_o(0);
		object_dimensions.width  = dim_o(1);
		object_dimensions.height = dim_o(2);
		object_dimensions.radius = dim_o(1);
		if(PositionCommands)	ofst = 1.0;
		wobj = 0.5*object_dimensions.width;
		// // left
		object_Pose_Gpoints[0].head(3) = Vector3d(0.0-0.029, wobj+0.00, -0.00)   + 1.0*ofst*Vector3d(-0.00,  0.005, -0.00);
		object_Pose_Gpoints[0].tail(4) = des_orientation_hands;
		// // right
		object_Pose_Gpoints[1].head(3) << Vector3d(0.0-0.029, -wobj-0.00, -0.00) + 1.0*ofst*Vector3d(-0.00, -0.005, -0.00);
		object_Pose_Gpoints[1].tail(4) = des_orientation_hands;
		// // hands offset for simulator
		hand_offset[0].setZero(); 	hand_offset[0](2) =  -0.029;
		hand_offset[1].setZero(); 	hand_offset[1](2) =   0.029;
		//
		object_H_Markers.block<3,1>(0,0) = Vector3d(0.0, 0.0, 0.0);
	}
	else if(select_object == "LargeBoxSim")
	{
		//
		object_port_name 		= 	"/";
		// object_port_name 		+= 	"icubSim/";
		object_port_name 		+= 	"GraspedObject_WorldPose:o";
		//
		Vector3d dim_o  = Vector3d(0.32, 0.16, 0.15);
		object_name 	=	"LargeBoxSim";
		object_mass 	= 	0.5;
		Jo_obj 			= Vector3d(0.1666, 0.1666, 0.1666);
		object_inertia  = Jo_obj.asDiagonal();
		object_dimensions.length = dim_o(0);
		object_dimensions.width  = dim_o(1);
		object_dimensions.height = dim_o(2);
		object_dimensions.radius = dim_o(1);
		if(PositionCommands)	ofst = 1.0;
		wobj = 0.5*object_dimensions.width;
		lobj = 0.5*object_dimensions.length;
		// // left
		object_Pose_Gpoints[0].head(3) = Vector3d(-lobj+0.05, wobj+0.00, -0.00)   + 1.0*ofst*Vector3d(-0.00,  0.005, -0.00);
		object_Pose_Gpoints[0].tail(4) = des_orientation_hands;
		// // right
		object_Pose_Gpoints[1].head(3) << Vector3d(-lobj+0.05, -wobj-0.00, -0.00) + 1.0*ofst*Vector3d(-0.00, -0.005, -0.00);
		object_Pose_Gpoints[1].tail(4) = des_orientation_hands;

		if(RobotIndex == 2)
		{
			object_Pose_Gpoints[0].head(3) = Vector3d(lobj-0.045, -wobj-0.00, -0.00)   + 1.0*ofst*Vector3d(-0.00, -0.005, -0.00);
			object_Pose_Gpoints[0].tail(4) = des_orientation_hands; 	//-1.0, 0.0, 0.0, M_PI/2.;
			// // right
			object_Pose_Gpoints[1].head(3) << Vector3d(lobj-0.045, wobj+0.00, -0.00) + 1.0*ofst*Vector3d(-0.00,    0.005, -0.00);
			object_Pose_Gpoints[1].tail(4) = des_orientation_hands; 	//-1.0, 0.0, 0.0, M_PI/2.;
		}
		// // hands offset for simulator
		hand_offset[0].setZero(); 	hand_offset[0](2) =  -0.029;
		hand_offset[1].setZero(); 	hand_offset[1](2) =   0.029;
		//
		object_H_Markers.block<3,1>(0,0) = Vector3d(0.0, 0.0, 0.0);
	}
    // /////////////////////////////////////////////////////////////////////////////////////////////// //
	//                                         Manipulation tasks                            		   //
	// /////////////////////////////////////////////////////////////////////////////////////////////// //

	virtualObjectScaling.setIdentity();   
	virtualObjectScaling(1,1) =  2.2;

	bimanip_gains.synchro  << 8.5, 8.5, 8.5, 8.5, 12.5, 8.5;  //2.5
	bimanip_gains.approach << 7.0, 7.0, 7.0, 10., 10., 10.;
	bimanip_gains.aperture << 8.0, 8.0, 8.0, 8.0, 8.0, 8.0;
	bimanip_gains.absolute << 7.0, 7.0, 7.0, 5., 5., 5.;
	bimanip_gains.relative << 10.0, 10.0, 10.0, 8.0, 8.0, 8.0;

	bimanip_gains.synchro  *=3.0; 
	bimanip_gains.approach *=3.0;
	bimanip_gains.aperture *=3.0;
	bimanip_gains.absolute *=3.0;
	bimanip_gains.relative *=3.0;
	a_bi = 0.5;
	b_bi = 1.0;

	// bimanip_gains.synchro  << 7.0, 7.0, 7.0, 14., 14., 14.;
	// bimanip_gains.approach << 6.0, 6.0, 6.0, 10., 10., 10.;
	// bimanip_gains.aperture << 5.0, 5.0, 5.0, 8.0, 8.0, 8.0;
	//
	// hand_offset[0].setZero(); 	hand_offset[0](2) =  -0.029;
	// hand_offset[1].setZero(); 	hand_offset[1](2) =   0.029;
	//
	alpha_manip_filter = 5.0;  // 5.0
	//
	des_lh_H_lf.setZero(); des_lh_H_lf(3,3) = 1.0;
	des_rh_H_lf.setZero(); des_rh_H_lf(3,3) = 1.0;

	des_lh_H_lf.block<3,3>(0,0) << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0;
	des_lh_H_lf.block<3,1>(0,3) << 0.2375, 0.1221, 0.6369; 

	des_rh_H_lf.block<3,3>(0,0) << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0;
	des_rh_H_lf.block<3,1>(0,3) << 0.2375, -0.2579, 0.6370;
	//
	des_lh_Pose_af_1 << 0.20,   0.18,  0.65,  -1.0,  0.0,  0.0,  M_PI/2.; // des_lh_Pose_af_1.tail(4) = des_orientation_hands;
	des_rh_Pose_af_1 << 0.20,  -0.18,  0.65,  -1.0,  0.0,  0.0,  M_PI/2.; // des_rh_Pose_af_1.tail(4) = des_orientation_hands;
	//
	// des_lh_Pose_af_1 << 0.15,   0.15,  0.57,  -1.0,  0.0,  0.0,  M_PI/2.;
	// des_rh_Pose_af_1 << 0.15,  -0.15,  0.57,  -1.0,  0.0,  0.0,  M_PI/2.;
	//
	des_lh_Pose_af_2 << 0.220,  0.168672,  0.490441, -0.876724,  0.354524,  0.325066,   1.7;
	des_rh_Pose_af_2 << 0.220, -0.176665,  0.493909, -0.876724,  0.354524,  0.325066,   1.7;
	//
	des_lh_Pose_af_3 << 0.18,   0.15,  0.60,  -1.0,  0.0,  0.0,  M_PI/2.;
	des_rh_Pose_af_3 << 0.18,  -0.15,  0.60,  -1.0,  0.0,  0.0,  M_PI/2.;
	//
	//
	max_reaching_velocity << 0.7, 0.7, 0.7,  2.0, 2.0, 2.0;
	//
	q_ini.resize(n_actuatedDofs);
	//
	// q_ini.segment( 0, 3) << -0.00783742,  -0.00712752,     0.241608;   
	q_ini.segment( 0, 3) << -0.00783742,  -0.00712752,     0.341608; 
	q_ini.segment( 3, 7) <<  -0.0875197,     0.203676,   -0.0529734,	 1.52662,   -0.178339,    0.0748416,  1.61051e-07;   
	q_ini.segment(10, 7) <<  -0.0655689,     0.221304,   -0.0654525,	 1.55191,     -0.1905,    0.0271419,  -1.0293e-06;     
	q_ini.segment(17, 6) <<    0.673026,      0.01927,    0.0363338,	-1.24848,   -0.563186,   0.00743787;     
	q_ini.segment(23, 6) <<    0.664523,    0.0220916,    0.0281664,	-1.24844,   -0.571749, -0.000213204;


    // Objective function gains
	Weight_hands_accel.setOnes();
	Weight_hands_wrench.setOnes();
	Weight_hands_slack.setZero(); 
	Weight_absolute_motion.setZero();
	Weight_relative_motion.setZero();
	K_object.setIdentity();
	D_object.setIdentity();

	Weight_hands_accel  *= 0.002;  // 0.0001;
	// Weight_hands_wrench *= 1.0e-2;	
	Weight_hands_wrench.segment(0, 3) *= 50.0e-2; 	//1.0e-02  Forces
	Weight_hands_wrench.segment(3, 3) *= 500.0e-2; 	//1.0e-02 	Moments
	Weight_hands_wrench.segment(6, 3) *= 50.0e-2; 	//1.0e-02 	Forces
	Weight_hands_wrench.segment(9, 3) *= 500.0e-2; 	//1.0e-02	Moments	

	Weight_hands_slack 	<< 100.0, 100.0, 100.0, 200.0, 200.0, 200.0;

	weight_regularizaion   =  1.0e-8;  
	Weight_absolute_motion 			 <<  1.0,  1.0,  1.0,  1.0,  1.0,  1.0;
	Weight_relative_motion.head<6>() << 10.0, 10.0, 10.0, 10.0, 10.0, 10.0;
	Weight_relative_motion.tail<6>() << 10.0, 10.0, 10.0, 10.0, 10.0, 10.0;
	//
	tol_dist2contact  	= 0.0425;
	min_normalForce 	= -20.0;
	max_normalForce 	=  20.0;
	gain_tau_hands  	=  12.0;
	EnableSaturation 	=  true;
	TorqueCorrection 	=  true;

	// gain of the reach2grasp controller
	Vector6d stiff_object, damp_object; 
	// damp_object  	<<  15.0,  15.0,  15.0,    6.0,   6.0,   6.0;   
	// stiff_object 	<<  50.0,  50.0,  50.0,   10.0,  10.0,  10.0;
	damp_object  	<<   8.0,   8.0,   6.0,    6.0,   6.0,   6.0;   
	stiff_object 	<<  10.0,  12.0,  10.0,   10.0,  10.0,  10.0;

	stiff_object *=1.2; 
	K_object = stiff_object.asDiagonal();
	D_object = 2.0*damp_object.asDiagonal(); // 2.0
	// K_object = 0.05* stiff_object.asDiagonal();
	// D_object = 0.05* 2.*damp_object.asDiagonal();

	// pole and gain for the filter
	filter_gain_object_pos 	= 4.0; //2.0;
	filter_gain_object_rot 	= 4.0; //2.0;
	// 
	alpha_object_filter_pos = 3.5;
	alpha_object_filter_rot = 3.5;
	

	// /////////////////////////////////////////////////////////////////////////////////////////////// //
	//                    Motion parameters for initial and final posture                              //
	// /////////////////////////////////////////////////////////////////////////////////////////////// //
	AccelParam_init[0] = 80.;        SpeedParam_init[0] = 100.;       // torso
	AccelParam_init[1] = 30.;        SpeedParam_init[1] =  50.;       // head
	AccelParam_init[2] = 30.;        SpeedParam_init[2] =  50.;       // lhand
	AccelParam_init[3] = 30.;        SpeedParam_init[3] =  50.;       // rhand
	AccelParam_init[4] = 80.;        SpeedParam_init[4] = 100.;       // lleg
	AccelParam_init[5] = 80.;        SpeedParam_init[5] = 100.;       // rleg
	//
	AccelParam_final[0] = 40.;       SpeedParam_final[0] = 50.;       // torso
	AccelParam_final[1] = 30.;       SpeedParam_final[1] = 30.;       // head
	AccelParam_final[2] = 30.;       SpeedParam_final[2] = 30.;       // lhand
	AccelParam_final[3] = 30.;       SpeedParam_final[3] = 30.;       // rhand
	AccelParam_final[4] = 40.;       SpeedParam_final[4] = 50.;       // lleg
	AccelParam_final[5] = 40.;       SpeedParam_final[5] = 50.;       // rleg
	//

	double d_q_pitch =  16.0; //2.00; 25
	double d_q_roll  = -0.0;
	i_jts_posture_lleg << d_q_pitch, -d_q_roll, 0.00, -2.*d_q_pitch, -1.0*d_q_pitch, d_q_roll;
	i_jts_posture_rleg << d_q_pitch, -d_q_roll, 0.00, -2.*d_q_pitch, -1.0*d_q_pitch, d_q_roll;

	d_q_pitch = 2.0; //2.00;
	d_q_roll  = -0.00; //0.70;
	f_jts_posture_lleg << d_q_pitch, -d_q_roll, 0.00, -2.*d_q_pitch, -d_q_pitch, d_q_roll;
	f_jts_posture_rleg << d_q_pitch, -d_q_roll, 0.00, -2.*d_q_pitch, -d_q_pitch, d_q_roll;

	if(RobotName == "icub")
	{
		//
		d_q_pitch =  16.0; //2.00; 22
		d_q_roll  = -0.0;
		//
		i_jts_posture_lleg << d_q_pitch, -d_q_roll+0.0, 0.00, -2.*d_q_pitch, -1.0*d_q_pitch, d_q_roll+2.8;
		i_jts_posture_rleg << d_q_pitch, -d_q_roll+0.0, 0.00, -2.*d_q_pitch, -1.0*d_q_pitch, d_q_roll+2.8;
		//
		d_q_pitch = 2.0; //2.00;
		d_q_roll  = -0.00; //0.70;
		//
		f_jts_posture_lleg << d_q_pitch, -d_q_roll+0.0, 0.00, -2.*d_q_pitch, -d_q_pitch, d_q_roll+2.8;
		f_jts_posture_rleg << d_q_pitch, -d_q_roll+0.0, 0.00, -2.*d_q_pitch, -d_q_pitch, d_q_roll+2.8;
	}

	// ----------------------------------------------------------------
	StartingPosture = Eigen::VectorXd::Zero(n_actuatedDofs);
	init_joints_config = Eigen::VectorXd::Zero(n_actuatedDofs);

	// 
	int nbjts[6];
	int ind[6];
	//		
	switch (n_actuatedDofs)
	{
		//		  torso 		head 			larm 			rarm 			lleg 			rleg
		case 32 : nbjts[0] = 3;	nbjts[1] = 3;	nbjts[2] = 7;	nbjts[3] = 7;	nbjts[4] = 6;	nbjts[5] = 6; break;
		case 29 : nbjts[0] = 3;	nbjts[1] = 0;	nbjts[2] = 7;	nbjts[3] = 7;	nbjts[4] = 6;	nbjts[5] = 6; break;
		case 25 : nbjts[0] = 3;	nbjts[1] = 0;	nbjts[2] = 5;	nbjts[3] = 5;	nbjts[4] = 6;	nbjts[5] = 6; break;
		case 23 : nbjts[0] = 3;	nbjts[1] = 0;	nbjts[2] = 4;	nbjts[3] = 4;	nbjts[4] = 6;	nbjts[5] = 6; break;
		default:  nbjts[0] = 3;	nbjts[1] = 3;	nbjts[2] = 7;	nbjts[3] = 7;	nbjts[4] = 6;	nbjts[5] = 6; break;
	}
	ind[0]  = 0;
	for(int i=1; i<6; i++) ind[i] = ind[i-1] + nbjts[i-1];

	//
	if((n_actuatedDofs == 29)||(n_actuatedDofs == 25)||(n_actuatedDofs == 23))
	 {
        StartingPosture.segment(ind[0],  nbjts[0]) << 0.00134905, -0.000131649,     0.305198;       														// Torso
        StartingPosture.segment(ind[2],  nbjts[2]) <<  -0.920917,     0.756169,     0.280663,     0.897856,    -0.541291,     0.174534,  1.56787e-07;    	// l hand
        StartingPosture.segment(ind[3],  nbjts[3]) <<   -0.91865,     0.755179,     0.282227,     0.900616,    -0.544833,     0.174542, -8.08933e-07;    	// r hand
        StartingPosture.segment(ind[4],  nbjts[4]) <<   0.607709, -0.000507854,  -0.00166302,    -0.893059,    -0.316886, -0.000177067;        				// l leg
        StartingPosture.segment(ind[5],  nbjts[5]) <<   0.608013,  0.000348752,   0.00187891,    -0.892178,    -0.315686,   0.00038491;        				// r leg
   		
   		init_joints_config.segment(ind[0],  nbjts[0]) <<  0.0, 0.0, 0.0;      														// Torso
        init_joints_config.segment(ind[2],  nbjts[2]) <<  -5.00,   16.0, 5.0, 30.0,  -0.00, 0.00,  5.00;    // l hand
        init_joints_config.segment(ind[3],  nbjts[3]) <<  -5.00,   16.0, 5.0, 30.0,  -0.00, 0.00,  5.00;    // r hand
        init_joints_config.segment(ind[4],  nbjts[4]) <<   1.0,     0.0, 0.0,   -2.0,    -1.0,  0.0;        			// l leg
        init_joints_config.segment(ind[5],  nbjts[5]) <<   1.0,     0.0, 0.0,   -2.0,    -1.0,  0.0;        			// r leg
        init_joints_config *=M_PI/180.0;
    }
    else
    {
        StartingPosture.segment(ind[0],  nbjts[0]) << 0.00134905, -0.000131649,     0.305198;       														// Torso
        StartingPosture.segment(ind[1],  nbjts[1]) <<      0.000,        0.000,        0.000;       														// head
        StartingPosture.segment(ind[2],  nbjts[2]) <<  -0.920917,     0.756169,     0.280663,     0.897856,    -0.541291,     0.174534,  1.56787e-07;    	// l hand
        StartingPosture.segment(ind[3],  nbjts[3]) <<   -0.91865,     0.755179,     0.282227,     0.900616,    -0.544833,     0.174542, -8.08933e-07;    	// r hand
        StartingPosture.segment(ind[4],  nbjts[4]) <<   0.607709, -0.000507854,  -0.00166302,    -0.893059,    -0.316886, -0.000177067;        				// l leg
        StartingPosture.segment(ind[5],  nbjts[5]) <<   0.608013,  0.000348752,   0.00187891,    -0.892178,    -0.315686,   0.00038491;      				// r leg

        init_joints_config.segment(ind[0],  nbjts[0]) <<  0.0, 0.0, 0.0;       														// Torso
        init_joints_config.segment(ind[1],  nbjts[1]) <<  0.0, 0.0, 0.0;       														// head
        init_joints_config.segment(ind[2],  nbjts[2]) <<  -5.00,   16.0, 5.0, 30.0,  -0.00, 0.00,  5.00;    		// l hand
        init_joints_config.segment(ind[3],  nbjts[3]) <<  -5.00,   16.0, 5.0, 30.0,  -0.00, 0.00,  5.00;    		// r hand
        init_joints_config.segment(ind[4],  nbjts[4]) <<   1.0,     0.0, 0.0,   -2.0,    -1.0,  0.0;        			// l leg
        init_joints_config.segment(ind[5],  nbjts[5]) <<   1.0,     0.0, 0.0,   -2.0,    -1.0,  0.0;     			// r leg
        init_joints_config *=M_PI/180.0;
    }


	return result &&  true;
}


bool ControllerParameters::getVelocityLimits(int actuatedDofs, double velo_limits[], Eigen::VectorXd &velocitySaturationLimit)
{
	//
	velocitySaturationLimit.setOnes();					/* actuatedDOFs */
	//
	if(actuatedDofs == 32) 
	{
		velocitySaturationLimit.head(3) 	  *= velo_limits[0];		// Torso
	    velocitySaturationLimit.segment(3, 3) *= velo_limits[1];		// Head
	    velocitySaturationLimit.segment(6,14) *= velo_limits[2];		// Hands
	    velocitySaturationLimit.tail(12) 	  *= velo_limits[3];	 	// Feets
	}
	else if (actuatedDofs == 29)
	{
		velocitySaturationLimit.head(3) 	  *= velo_limits[0];		// Torso
	    velocitySaturationLimit.segment(3,14) *= velo_limits[2];		// Hands
	    velocitySaturationLimit.tail(12) 	  *= velo_limits[3];		// Feets
	}
	else if (actuatedDofs == 25)
	{
		velocitySaturationLimit.head(3) 	  *= velo_limits[0];		// Torso
	    velocitySaturationLimit.segment(3,10) *= velo_limits[2];		// Hands
	    velocitySaturationLimit.tail(12) 	  *= velo_limits[3];		// Feets
	}
	else if(actuatedDofs == 23) 
	{
		velocitySaturationLimit.head(3) 	  *= velo_limits[0];		// Torso
	    velocitySaturationLimit.segment(3,8)  *= velo_limits[2];		// Hands
	    velocitySaturationLimit.tail(12) 	  *= velo_limits[3];		// Feets
	}

	return true;
}

bool ControllerParameters::getTorqueLimits(int actuatedDofs, double torque_limits[], Eigen::VectorXd &torqueSaturationLimit)
{
	//
	torqueSaturationLimit.setOnes();					/* actuatedDOFs */
	//
	if(actuatedDofs == 32) 
	{
		torqueSaturationLimit.head(3) 	    *= torque_limits[0];		// Torso
	    torqueSaturationLimit.segment(3, 3) *= torque_limits[1];		// Head
	    torqueSaturationLimit.segment(6,14) *= torque_limits[2];		// Hands
	    torqueSaturationLimit.tail(12) 	    *= torque_limits[3];		// Feets
	}
	else if (actuatedDofs == 29)
	{
		torqueSaturationLimit.head(3) 	    *= torque_limits[0];		// Torso
	    torqueSaturationLimit.segment(3,14) *= torque_limits[2];		// Hands
	    torqueSaturationLimit.tail(12) 	    *= torque_limits[3];		// Feets
	}
	else if (actuatedDofs == 25)
	{
		torqueSaturationLimit.head(3) 	    *= torque_limits[0];		// Torso
	    torqueSaturationLimit.segment(3,10) *= torque_limits[2];		// Hands
	    torqueSaturationLimit.tail(12) 	    *= torque_limits[3];		// Feets
	}
	else if(actuatedDofs == 23) 
	{
		torqueSaturationLimit.head(3) 	    *= torque_limits[0];		// Torso
	    torqueSaturationLimit.segment(3,8)  *= torque_limits[2];		// Hands
	    torqueSaturationLimit.tail(12) 	    *= torque_limits[3];		// Feets
	}

	return true;
}

bool ControllerParameters::getPostureWeights(int n_actuatedDofs, double gains_posture[], Eigen::VectorXd &Weight_posture)
{
	//
	Weight_posture.setOnes();					/* actuatedDOFs */
	//
	if(actuatedDofs == 32) 
	{
		Weight_posture.head(6) 	     *= gains_posture[0];		// Base
		Weight_posture.segment(6, 3) *= gains_posture[1];		// Torso
	    Weight_posture.segment(9, 3) *= gains_posture[2];		// Head
	    Weight_posture.segment(12,14)*= gains_posture[3];		// arms
	    Weight_posture.tail(12) 	 *= gains_posture[4];	 	// legs
	}
	else if (actuatedDofs == 29)
	{
		Weight_posture.head(3) 	     *= gains_posture[0];		// Base
		Weight_posture.segment(3,3)  *= gains_posture[0];		// Base
		Weight_posture.segment(6, 3) *= gains_posture[1];		// Torso
	    Weight_posture.segment(9,14) *= gains_posture[3];	// arms
	    Weight_posture.tail(12) 	 *= gains_posture[4];		// legs

	    Weight_posture(9) 	 *= 0.01;		// left shoulder pitch
	    Weight_posture(16) 	 *= 0.01;		// right shoulder pitch

	    Weight_posture(10) 	 *= 6.0;		// left shoulder roll  8
	    Weight_posture(17) 	 *= 6.0;		// right shoulder roll 8
	    Weight_posture(11) 	 *= 1.0;		// left shoulder yaw
	    Weight_posture(18) 	 *= 1.0;		// right shoulder yaw

	    Weight_posture(25) 	 *= 0.1;		// left anle pitch
	    Weight_posture(31) 	 *= 0.1;		// right anle pitch

	    Weight_posture(26) 	 *= 0.1;		// left knee
	    Weight_posture(32) 	 *= 0.1;		// right knee
	}
	else if (actuatedDofs == 25)
	{
		Weight_posture.head(6) 	     *= gains_posture[0];		// Base
		Weight_posture.segment(6, 3) *= gains_posture[1];		// Torso
	    Weight_posture.segment(9,10) *= gains_posture[3];		// arms
	    Weight_posture.tail(12) 	 *= gains_posture[4];		// legs
	}
	else if(actuatedDofs == 23) 
	{
		Weight_posture.head(6) 	     *= gains_posture[0];		// Base
		Weight_posture.segment(6, 3) *= gains_posture[1];		// Torso
	    Weight_posture.segment(9, 8) *= gains_posture[3];		// arms
	    Weight_posture.tail(12) 	 *= gains_posture[4];		// legs
	}

	return true;
}

bool ControllerParameters::get_list_of_considered_joints(std::vector<std::string> &list_of_considered_joints_)
{
	// 
	int nbjts[6];
	//		
	switch (actuatedDofs)
	{
		//		  torso 		head 			larm 			rarm 			lleg 			rleg
		case 32 : nbjts[0] = 3;	nbjts[1] = 3;	nbjts[2] = 7;	nbjts[3] = 7;	nbjts[4] = 6;	nbjts[5] = 6; break;
		case 29 : nbjts[0] = 3;	nbjts[1] = 0;	nbjts[2] = 7;	nbjts[3] = 7;	nbjts[4] = 6;	nbjts[5] = 6; break;
		case 25 : nbjts[0] = 3;	nbjts[1] = 0;	nbjts[2] = 5;	nbjts[3] = 5;	nbjts[4] = 6;	nbjts[5] = 6; break;
		case 23 : nbjts[0] = 3;	nbjts[1] = 0;	nbjts[2] = 4;	nbjts[3] = 4;	nbjts[4] = 6;	nbjts[5] = 6; break;
		default:  nbjts[0] = 3;	nbjts[1] = 3;	nbjts[2] = 7;	nbjts[3] = 7;	nbjts[4] = 6;	nbjts[5] = 6; break;
	}
	// 
	if((actuatedDofs == 29)||(actuatedDofs == 25)||(actuatedDofs == 23))
	{
		for(int i=0; i<nbjts[0]; i++) list_of_considered_joints_.push_back(main_joints_list.torso[i]);         // torso
	    for(int i=0; i<nbjts[2]; i++) list_of_considered_joints_.push_back(main_joints_list.left_arm[i]);     // left hand
	    for(int i=0; i<nbjts[3]; i++) list_of_considered_joints_.push_back(main_joints_list.right_arm[i]);    // right hand
	    for(int i=0; i<nbjts[4]; i++) list_of_considered_joints_.push_back(main_joints_list.left_leg[i]);      // left leg
	    for(int i=0; i<nbjts[5]; i++) list_of_considered_joints_.push_back(main_joints_list.right_leg[i]);     // right leg

	}
	else
	{
		for(int i=0; i<nbjts[0]; i++) list_of_considered_joints_.push_back(main_joints_list.torso[i]);         // torso
	    for(int i=0; i<nbjts[1]; i++) list_of_considered_joints_.push_back(main_joints_list.neck[i]);          // neck
	    for(int i=0; i<nbjts[2]; i++) list_of_considered_joints_.push_back(main_joints_list.left_arm[i]);     // left hand
	    for(int i=0; i<nbjts[3]; i++) list_of_considered_joints_.push_back(main_joints_list.right_arm[i]);    // right hand
	    for(int i=0; i<nbjts[4]; i++) list_of_considered_joints_.push_back(main_joints_list.left_leg[i]);      // left leg
	    for(int i=0; i<nbjts[5]; i++) list_of_considered_joints_.push_back(main_joints_list.right_leg[i]);     // right leg
	}


	return true;
}