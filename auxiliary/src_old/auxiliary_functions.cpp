
#include <string>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <cmath>
#include <signal.h>
#include <unistd.h>

#include "wrapper.h"
#include "IK.h"
#include "ID.h"

#include "auxiliary_functions.h"

using namespace std;
using namespace Eigen;


enum state {BALANCE=0,  PICK_APPROACH, PICK, PICK_STAND, DROP_APPROACH, DROP, DROP_STAND, WALK_START, WALK, WALK_STOP};
enum trigger {NONE=0, GO, HALT, TAKE, RELEASE};


Eigen::Vector3d getEulerAnglesXYZ_FixedFrame(Eigen::Matrix3d R)
{
    // this function computed for a given rotation matrix the rotation angles around X, Y and Z axis considered as fixed.
    // the rotation matrix is assumed to be a Euler rotation matrix of type ZYX
    Eigen::Vector3d Angles;
    double Psi_X, Theta_Y, Phi_Z;
        Psi_X   = atan2(R(2,1),R(2,2));
        Theta_Y = atan2(-R(2,0), abs(sqrt(pow(R(0,0), 2.)+pow(R(1,0), 2.))));
        Phi_Z   = atan2(R(1,0),R(0,0));
    if ((Theta_Y>M_PI/2.)||(Theta_Y<-M_PI/2.))
    {
        Psi_X   = atan2(-R(2,1),-R(2,2));
        Theta_Y = atan2(-R(2,0),-abs(sqrt(pow(R(0,0), 2.)+pow(R(1,0), 2.))));
        Phi_Z   = atan2(-R(1,0),-R(0,0));
    }
    Angles(0) = Psi_X;
    Angles(1) = Theta_Y;
    Angles(2) = Phi_Z;

    return Angles;
}

/* Index reference and a natural posture of icub           */
/* global positions                                        */
/* pelvis pos:     pos[0] pos[1] pos[2]                    */
/* pelvis rot:     pos[3] pos[4] pos[5] pos[38]            */
/*                 // right      // left                   */
/* head                   pos[11]                          */
/* neck roll              pos[10]                          */
/* neck pitch             pos[9]                           */
/* shoulder pitch  pos[31]       pos[24]                   */
/* shoulder roll   pos[32]       pos[25]                   */
/* shoulder yaw    pos[33]       pos[26]                   */
/* elbow           pos[34]       pos[27]                   */
/* forearm yaw     pos[35]       pos[28]                   */
/* forearm roll    pos[36]       pos[29]                   */
/* forearm pitch   pos[37]       pos[30]                   */
/* torso pitch            pos[6]                           */
/* torso roll             pos[7]                           */
/* torso yaw              pos[8]                           */
/* hip pitch       pos[18]      pos[12]                    */
/* hip roll        pos[19]       pos[13]                   */
/* hip yaw         pos[20]       pos[14]                   */
/* knee            pos[21]       pos[15]                   */
/* ankle pitch     pos[22]       pos[16]                   */
/* ankle roll      pos[23]       pos[17]                   */



auxiliary_functions::auxiliary_functions(){}
auxiliary_functions::~auxiliary_functions(){}

void auxiliary_functions::init(std::string robot_Name)
{
	//
	wrench_lh = Cvector::Zero(6);
	wrench_rh = Cvector::Zero(6);

	W.robotName = "/" + robot_Name;

	W.initialize();

	// // definitions
	// Model model;
	model.init();

	// joint variables
	ref_pos  = Cvector::Zero(AIR_N_U+1); ref_pos[AIR_N_U] = 1;
	ref_vel  = Cvector::Zero(AIR_N_U);
	ref_acc  = Cvector::Zero(AIR_N_U);
	ref_tau  = Cvector::Zero(AIR_N_U);
	sens_pos = Cvector::Zero(AIR_N_U+1); sens_pos[AIR_N_U] = 1;
	sens_vel = Cvector::Zero(AIR_N_U);
	sens_acc = Cvector::Zero(AIR_N_U);
	sens_tau = Cvector::Zero(AIR_N_U);
	init_pos = Cvector::Zero(AIR_N_U+1); init_pos[AIR_N_U] = 1;
	next_pos = Cvector::Zero(AIR_N_U+1); next_pos[AIR_N_U] = 1;
	freezeIK = Cvector::Zero(AIR_N_U);
	mode 	 = Cvector::Zero(AIR_N_U);

	// contact definition : Contact_Manager points;
	points.initialize(&model, 5);
	points[CN_CM].init(CN_CM, body_base,   offset_base,   CT_FULL, PS_FLOATING,  0              , 0             );
	points[CN_LF].init(CN_LF, body_l_foot, offset_l_foot, CT_FULL, PS_CONTACTED, foot_length*0.8, foot_width*0.8);
	points[CN_RF].init(CN_RF, body_r_foot, offset_r_foot, CT_FULL, PS_CONTACTED, foot_length*0.8, foot_width*0.8);
	points[CN_LH].init(CN_LH, body_l_hand, offset_l_hand, CT_FULL, PS_FLOATING,  hand_length*0.8, hand_width*0.8);
	points[CN_RH].init(CN_RH, body_r_hand, offset_r_hand, CT_FULL, PS_FLOATING,  hand_length*0.8, hand_width*0.8);

	/*Initialize the stance foot to the left and
	with External estimation of the base's state*/
	this->StanceFoot[0] 	= 0; 		// left 
	this->StanceFoot[1] 	= 1;		// right
	this->isExtBase 		= true; 

	//
	T_base2world  = Eigen::MatrixXd::Identity(4,4);

	// read sensors
	this->loadData(W, points, init_pos,  sens_vel,  sens_acc,  sens_tau);

	// set position/torque control modes, update limits
	// W.initializeJoint(mode.segment(6,AIR_N_U-6));
	W.getJointLimits(&model.qmin[0], &model.qmax[0]);
}

void auxiliary_functions::loadData(wrapper &W, Contact_Manager &points, Cvector &pos,  Cvector &vel,  Cvector &acc,  Cvector &tau)
{
	//
	
	W.readSensors();

	pos.segment(6,AIR_N_U-6)	= W.sens_pos / 180.0 * M_PI;
	vel.segment(6,AIR_N_U-6)	= W.sens_vel / 180.0 * M_PI;
	acc.segment(6,AIR_N_U-6)	= W.sens_acc / 180.0 * M_PI;
	tau.segment(6,AIR_N_U-6)	= W.sens_tau;

	//remove yaw angle
	double roll 	= atan2(W.R(2,1), W.R(2,2));
	double pitch 	= -asin(W.R(2,0));
	W.YAW 			= atan2(W.R(1,0), W.R(0,0));
	Cmatrix rot 	= ang2dc(Cvector3(0,pitch,0)) * ang2dc(Cvector3(roll,0,0));
	// ------------------------------------------------------------------------
	Eigen::Matrix3d Rroot 	=  W.rotmatrix(Eigen::Vector3d(0,0,M_PI)) * quat2dc(W.Root.segment(3,4));
	// extract the Euler angles
	Eigen::Vector3d RPY 	= getEulerAnglesXYZ_FixedFrame(Rroot);
	W.YAW = RPY(2);
	rot = ang2dc(Cvector3(0,RPY(1),0)) * ang2dc(Cvector3(RPY(0),0,0));

	// /////////////////////////////////////////////////////////////////////////
	Cvector q					= dc2quat(rot);
	pos.segment(3,3)			= q.segment(0,3);
	pos[AIR_N_U]				= q[3];
	vel.segment(3,3)			= W.angvel / 180.0 * M_PI;
	acc.segment(3,3)			= W.linacc;
	for(int i=1; i<5; i++)
	{
		points[i].T.F_sens = W.FTsensor[i-1][0];
		points[i].R.F_sens = W.FTsensor[i-1][1];
	}

	// WARNING: make sure to cancel IMU yaw on the robot
	points.M->set_state(0, pos, vel);
	// Base defined in the middle of the two feet
	Cvector3 base  =  points.M->get_pos(points[CN_LF].body, points[CN_LF].offset) * 0.5
					+ points.M->get_pos(points[CN_RF].body, points[CN_RF].offset) * 0.5;
	Cvector3 dbase =  points.M->get_vel(points[CN_LF].body, points[CN_LF].offset) * 0.5
					+ points.M->get_vel(points[CN_RF].body, points[CN_RF].offset) * 0.5;

	// -------------------------------
	if(this->isExtBase){
			base = W.Root.head(3);
			// base(0) = 0.0*W.Root(0);
			// base(1) = 0.0*W.Root(1);
		
	}else {
		if(this->StanceFoot[0] ==1)  // the left foot is the stance foot
		{ 	base  = -points.M->get_pos(points[CN_LF].body, points[CN_LF].offset); 	}
		else // the right foot is the stance foot
		{	base  = -points.M->get_pos(points[CN_RF].body, points[CN_RF].offset);  	}
	}
	//
	pos.segment(0,3) = base;
	vel.segment(0,3) = dbase;
	// update the robot state
	points.M->set_state(0, pos, vel);
	points.update_kinematics();

	// robot base in in world reference frame (Origin in gazebo or stance foot)
	// wld_T_b  = Eigen::MatrixXd::Identity(4,4);
	T_base2world.block<3,3>(0,0) = W.rotmatrix(Eigen::Vector3d(0,0,M_PI)) * quat2dc(W.Root.segment(3,4));
	T_base2world.block<3,1>(0,3) = init_pos.head(3);

	// cout <<" MEASURED BASE POSITION is : \n " << pos.segment(0,3) << endl;
	// cout <<" MEASURED BASE POSITION is : \n " << base << endl;
	cout <<" LEFT FOOT POSITION AFTER UPDATE is : \n " << points.M->get_pos(points[CN_LF].body, points[CN_LF].offset) << endl;
	cout <<" RIGHT FOOT POSITION AFTER UPDATE is : \n " << points.M->get_pos(points[CN_RF].body, points[CN_RF].offset) << endl;
}


bool auxiliary_functions::get_hands_wrenches_offsets()
{
	//
	// read sensors
	this->loadData(W, points, sens_pos,  sens_vel,  sens_acc,  sens_tau);
	// arm force estimation, this compensates lower-arm weights, and rotates to world-frame
	Cmatrix Forces(AIR_N_BODY,3);
	Cmatrix Torques(AIR_N_BODY,3);
	// model.get_reac(Forces, Torques);
	points.M->get_reac(Forces, Torques);

	Cvector bias_lh = Cvector::Zero(6);
	bias_lh << -11.8466,  -14.6179,  -36.7255,  0.177665, -0.748966,   0.18572;
	Cvector bias_rh = Cvector::Zero(6);
	bias_rh <<  32.6632,  -5.57603,  -55.8315, -0.748966, -0.511756,  0.313862;

	// Cmatrix RR = model.get_orient(26);
	Cmatrix RR = points.M->get_orient(26);
	Cvector reading_lh = vectorbig(	RR*(-points[CN_LH].T.F_sens-0.0*bias_lh.segment(0,3)) + RR*Forces.block(26,0,1,3).transpose(), 
		  							RR*(-points[CN_LH].R.F_sens-0.0*bias_lh.segment(3,3)) + RR*Torques.block(26,0,1,3).transpose());

	RR = points.M->get_orient(34);
	Cvector reading_rh = vectorbig(	RR*(-points[CN_RH].T.F_sens-0.0*bias_rh.segment(0,3)) + RR*Forces.block(34,0,1,3).transpose(),
		  							RR*(-points[CN_RH].R.F_sens-0.0*bias_rh.segment(3,3)) + RR*Torques.block(34,0,1,3).transpose());

	//
	offset_wrench_lh = offset_wrench_lh * 0.99 + reading_lh * 0.01;
	offset_wrench_rh = offset_wrench_rh * 0.99 + reading_rh * 0.01;

	return true;
}

bool auxiliary_functions::get_estimated_hands_wrenches()
{
	// read sensors
	this->loadData(W, points, sens_pos,  sens_vel,  sens_acc,  sens_tau);
	// arm force estimation, this compensates lower-arm weights, and rotates to world-frame
	Cmatrix Forces(AIR_N_BODY,3);
	Cmatrix Torques(AIR_N_BODY,3);
	// model.get_reac(Forces, Torques);
	points.M->get_reac(Forces, Torques);

	Cvector bias_lh = Cvector::Zero(6);
	bias_lh << -11.8466,  -14.6179,  -36.7255,  0.177665, -0.748966,   0.18572;
	Cvector bias_rh = Cvector::Zero(6);
	bias_rh <<  32.6632,  -5.57603,  -55.8315, -0.748966, -0.511756,  0.313862;

	// Cmatrix RR = model.get_orient(26);
	Cmatrix RR = points.M->get_orient(26);
	Cvector reading_lh = vectorbig(	RR*(-points[CN_LH].T.F_sens-0.0*bias_lh.segment(0,3)) + RR*Forces.block(26,0,1,3).transpose(), 
		  							RR*(-points[CN_LH].R.F_sens-0.0*bias_lh.segment(3,3)) + RR*Torques.block(26,0,1,3).transpose());

	RR = points.M->get_orient(34);
	Cvector reading_rh = vectorbig(	RR*(-points[CN_RH].T.F_sens-0.0*bias_rh.segment(0,3)) + RR*Forces.block(34,0,1,3).transpose(),
		  							RR*(-points[CN_RH].R.F_sens-0.0*bias_rh.segment(3,3)) + RR*Torques.block(34,0,1,3).transpose());

	//
	wrench_lh = wrench_lh * 0.95 + (reading_lh-0.0*offset_wrench_lh) * 0.05;
	wrench_rh = wrench_rh * 0.95 + (reading_rh-0.0*offset_wrench_rh) * 0.05;

	
	cout <<" NET WRENCH LH  is : \t " << wrench_lh.transpose() << endl;
	cout <<" NET WRENCH RH  is : \t " << wrench_rh.transpose() << endl;


	return true;
}

// INverse kinematics //////////////////////////////////////////////////////////////////////
void auxiliary_functions::init_IK()
{
	//
	// IKPARAM ikparam;
	ikparam.num_iter = 15;

	// freeze neck joints
	for(int i=9;i<=11;i++)
		freezeIK[i] = 1;

	// control CoM instead of pelvis
	points.ifCoM = true;

	// task flexibility IK
	points[CN_CM].T.slack_cost = Cvector3(100,100,0.01);
	points[CN_CM].R.slack_cost = Cvector3(0.01,0.01,0.01);
	points[CN_LF].T.slack_cost = Cvector3(1,1,1) * 100;
	points[CN_LF].R.slack_cost = Cvector3(1,1,1) * 100;
	points[CN_RF].T.slack_cost = Cvector3(1,1,1) * 100;
	points[CN_RF].R.slack_cost = Cvector3(1,1,1) * 100;
	points[CN_LH].R.slack_cost = Cvector3(1,1,1) * 1;
	points[CN_LH].T.slack_cost = Cvector3(1,1,1) * 1;
	points[CN_RH].R.slack_cost = Cvector3(1,1,1) * 1;
	points[CN_RH].T.slack_cost = Cvector3(1,1,1) * 1;

}

bool auxiliary_functions::get_inverse_kinematics(wb_ref_position ref_p_wb,  wb_ref_orientation ref_o_wb)
{
	// get time
	// double time = W.time;

	// read sensors
	loadData(W, points, sens_pos,  sens_vel,  sens_acc,  sens_tau);

	//
	// arm tasks /////////////////////////////////////////////////////////////////////////////////	
	points[CN_LH].ref_p.pos = ref_p_wb.p[0];
	points[CN_LH].ref_o.pos = ref_o_wb.o[0];

	points[CN_RH].ref_p.pos = ref_p_wb.p[1];
	points[CN_RH].ref_o.pos = ref_o_wb.o[1];

	// feet tasks /////////////////////////////////////////////////////////////////////////////////
	points[CN_LF].ref_p.pos = ref_p_wb.p[2];
	points[CN_LF].ref_o.pos = ref_o_wb.o[2];

	points[CN_RF].ref_p.pos = ref_p_wb.p[3];
	points[CN_RF].ref_o.pos = ref_o_wb.o[3];

	// base tasks ////////////////////////////////////////////////////////////////////////////////
	points[CN_CM].ref_p.pos = ref_p_wb.p[4];
	points[CN_CM].ref_o.pos = ref_o_wb.o[4];

	// whole-body IK ////////////////////////////////////////////////////////////////////////////////
	double IK_time = IK(points, ikparam, ref_pos, freezeIK);
	points.M->set_state(0, sens_pos, sens_vel);
	points.update_kinematics();
	points.print_IK_errors();


	return true;
}