

#include <string>
#include <iostream>
#include <fstream>
#include <cmath>

#include "StepperThread.h"

using namespace Eigen;
using namespace std;


StepperThread::StepperThread(std::string robot_name, int period_, 
							 WholeBodyTaskSpaceStates wbEEstates_, 
							 WholeBodyTaskSpaceTrajectories wbEEtraj_) : robotName(robot_name)
																	   , RateThread(period_)
																																				  
{
	ThreadPeriod = period_; 
	wbEEstates   = wbEEstates_;
	wbEEtraj     = wbEEtraj_;
}
// StepperThread::StepperThread(std::string robot_name, int period_) : RateThread(period_)
// 																  , robotName(robot_name)
// 																  , ThreadPeriod(period_){}

StepperThread::~StepperThread() 
{
	// this->releaseStepper();
}

bool StepperThread::threadInit()
{
	//
    //   STEP GENERATOR  //////////////////////////////////////////////////////////////////////
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	FeedbackType = 0;
    // Starting the BalanceWalkingController Thread
    myWalker.init(ThreadPeriod, robotName, FeedbackType);
    // Set the Desired CoM velocity
    myWalker.Des_RelativeVelocity(0) = 0.00;
    myWalker.Des_RelativeVelocity(1) = 0.00;
    myWalker.Des_RelativeVelocity(2) = 0.00;
    //
    // Update the stance foot
	if(myWalker.Parameters->StanceIndicator[0] == 1){   // left if 1 
		lstance_switch = true;
		rstance_switch = false;
	}else{												// right
		lstance_switch = false;
		rstance_switch = true;
	}

	// left and right foot desired homogenous transformation in base frame with initial world orientation
	Trsf_des_ee2base_world[2]  	= myWalker.CpBalWlkController-> GaitTrsf.Trsf_lfoot_base_world;
	Trsf_des_ee2base_world[3] 	= myWalker.CpBalWlkController-> GaitTrsf.Trsf_lfoot_base_world;
	// Transformation Com in the feet frames
	Trsf_des_com2lfoot			= myWalker.CpBalWlkController-> GaitTrsf.Trsf_lfoot_com.inverse();	
	Trsf_des_com2rfoot			= myWalker.CpBalWlkController-> GaitTrsf.Trsf_rfoot_com.inverse();

	Trsf_des_lfoot2base_robot  = myWalker.CpBalWlkController-> GaitTrsf.Trsf_lfoot_base_icub;
	Trsf_des_rfoot2base_robot  = myWalker.CpBalWlkController-> GaitTrsf.Trsf_rfoot_base_icub;
	//
	number_steps  = 1;
	stride_x      = 0.0;

	
	return true;
}

void StepperThread::threadRelease()
{
	this->releaseStepper();
}


void StepperThread::releaseStepper()
{
    // Set to Zero the Desired CoM velocity before stoping
    myWalker.Des_RelativeVelocity(0) = 0.00;  // TO DO
    myWalker.Des_RelativeVelocity(1) = 0.00;
    myWalker.Des_RelativeVelocity(2) = 0.00;

    myWalker.Release();
}


void StepperThread::run()
{
	//
	// set the Base-CoM offset
	Eigen::Vector3d t_B_CoM = 	  Transforms.PoseVector2HomogenousMx(wbEEstates.Pelvis.Pose).block(0,3,3,1) 
								- Transforms.PoseVector2HomogenousMx(wbEEstates.CoM.Pose).block(0,3,3,1);


	myWalker.CpBalWlkController->GaitTrsf.SetTranslationBaseCoM(t_B_CoM);
	// Loop for the walking
    myWalker.walk();
	//
	Matrix4d  	w_T_pe_w 				 = Transforms.PoseVector2HomogenousMx(wbEEstates.Pelvis.Pose);
				w_T_pe_w.block(0,0, 3,3) = Transforms.rot_vector2rpy(Eigen::Vector3d(0,0,M_PI)) * w_T_pe_w.block(0,0, 3,3);
	// left and right hands desired homogenous transformation in base frame with initial world orientation
	Trsf_des_ee2base_world[0]   = w_T_pe_w.inverse() * Transforms.PoseVector2HomogenousMx(wbEEtraj.lhand.pose);
	Trsf_des_ee2base_world[1]   = w_T_pe_w.inverse() * Transforms.PoseVector2HomogenousMx(wbEEtraj.rhand.pose);
	// left and right foot desired homogenous transformation in base frame with initial world orientation
	Trsf_des_ee2base_world[2]  	= myWalker.CpBalWlkController-> GaitTrsf.Trsf_lfoot_base_world;
	Trsf_des_ee2base_world[3] 	= myWalker.CpBalWlkController-> GaitTrsf.Trsf_rfoot_base_world;
	
	// left and right foot desired homogenous transformation in robot base frame
	Trsf_des_lfoot2base_robot  = myWalker.CpBalWlkController-> GaitTrsf.Trsf_lfoot_base_icub;
	Trsf_des_rfoot2base_robot  = myWalker.CpBalWlkController-> GaitTrsf.Trsf_rfoot_base_icub;

	// Transformation Com in the feet frames
	Trsf_des_com2lfoot	= myWalker.CpBalWlkController-> GaitTrsf.Trsf_lfoot_com.inverse();	
	Trsf_des_com2rfoot	= myWalker.CpBalWlkController-> GaitTrsf.Trsf_rfoot_com.inverse();

	// Tasks  ////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	if(myWalker.Parameters->StanceIndicator[0] == 1) // left stance
	{
		if(lstance_switch)
		{
	 		Trsf_lstancefoot2world = Transforms.PoseVector2HomogenousMx(wbEEstates.lfoot.Pose);
	 		//
	 		lstance_switch = !lstance_switch;
	 		rstance_switch = !rstance_switch;
		}
		// 
		// Trsf_des_rfoot2world = Trsf_lstancefoot2world * Trsf_des_rswingfoot2lfoot;
		Trsf_des_ee2world[0] = Trsf_lstancefoot2world * Trsf_des_ee2base_world[2].inverse() * Trsf_des_ee2base_world[0];
		Trsf_des_ee2world[1] = Trsf_lstancefoot2world * Trsf_des_ee2base_world[2].inverse() * Trsf_des_ee2base_world[1];
		Trsf_des_ee2world[2] = Trsf_lstancefoot2world;
		Trsf_des_ee2world[3] = Trsf_lstancefoot2world * Trsf_des_ee2base_world[2].inverse() * Trsf_des_ee2base_world[3];
		Trsf_des_ee2world[4] = Trsf_lstancefoot2world * Trsf_des_com2lfoot;

	}
	else // right stance foot
	{
		if(rstance_switch)
		{
	 		Trsf_rstancefoot2world = Transforms.PoseVector2HomogenousMx(wbEEstates.rfoot.Pose);
	 		//
	 		rstance_switch = !rstance_switch;
	 		lstance_switch = !lstance_switch;
		}
		// 
		// Trsf_des_lfoot2world = Trsf_rstancefoot2world * Trsf_des_lswingfoot2lfoot;
		Trsf_des_ee2world[0] = Trsf_rstancefoot2world * Trsf_des_ee2base_world[3].inverse() * Trsf_des_ee2base_world[0];
		Trsf_des_ee2world[1] = Trsf_rstancefoot2world * Trsf_des_ee2base_world[3].inverse() * Trsf_des_ee2base_world[1];
		Trsf_des_ee2world[2] = Trsf_rstancefoot2world * Trsf_des_ee2base_world[3].inverse() * Trsf_des_ee2base_world[2];
		Trsf_des_ee2world[3] = Trsf_rstancefoot2world; 
		Trsf_des_ee2world[4] = Trsf_rstancefoot2world * Trsf_des_com2rfoot;
	}
	
	cout << " \n" << endl;
	cout << " REF position lfoot_base r IS \t " << Trsf_des_lfoot2base_robot.block<3,1>(0,3).transpose()  << endl;
	cout << " REF position rfoot_base r IS \t " << Trsf_des_rfoot2base_robot.block<3,1>(0,3).transpose()  << endl;

	return;

}


bool StepperThread::set_handsTasks_in_base(Vector7d lh_pose, Vector7d rh_pose, Vector7d pelvis_pose)
{
	//
	MatrixXd  	w_T_lh = Transforms.PoseVector2HomogenousMx(lh_pose); 
	MatrixXd  	w_T_rh = Transforms.PoseVector2HomogenousMx(rh_pose);
	MatrixXd  	w_T_pe = Transforms.PoseVector2HomogenousMx(pelvis_pose);

	Matrix4d  	w_T_pe_w =  w_T_pe;
				w_T_pe_w.block(0,0, 3,3) = Transforms.rot_vector2rpy(Vector3d(0,0,M_PI)) * w_T_pe.block(0,0, 3,3);


	Matrix4d w_pe_T_lh = w_T_pe_w.inverse() * w_T_lh;
	Matrix4d w_pe_T_rh = w_T_pe_w.inverse() * w_T_rh;

	// left and right hands desired homogenous transformation in base frame with initial world orientation
	Trsf_des_ee2base_world[0] = w_pe_T_lh;
	Trsf_des_ee2base_world[1] = w_pe_T_rh;

	return true;
}


void StepperThread::get_step_transforms(int number_steps, double stride_x, Vector7d des_pose_lh, Vector7d des_pose_rh, WholeBodyTaskSpaceStates wb_ts)
{
	//
	myWalker.CpBalWlkController->step_size_x = stride_x;
	// set the Base-CoM offset
	Eigen::Vector3d t_B_CoM = 	  Transforms.PoseVector2HomogenousMx(wb_ts.Pelvis.Pose).block(0,3,3,1) 
								- Transforms.PoseVector2HomogenousMx(wb_ts.CoM.Pose).block(0,3,3,1);

	myWalker.CpBalWlkController->GaitTrsf.SetTranslationBaseCoM(t_B_CoM);
	// Loop for the walking
    myWalker.walk();
	//
	Matrix4d  	w_T_pe_w 				 = Transforms.PoseVector2HomogenousMx(wb_ts.Pelvis.Pose);
				w_T_pe_w.block(0,0, 3,3) = Transforms.rot_vector2rpy(Eigen::Vector3d(0,0,M_PI)) * w_T_pe_w.block(0,0, 3,3);
	// left and right hands desired homogenous transformation in base frame with initial world orientation
	Trsf_des_ee2base_world[0]   = w_T_pe_w.inverse() * Transforms.PoseVector2HomogenousMx(wb_ts.lhand.Pose);
	Trsf_des_ee2base_world[1]   = w_T_pe_w.inverse() * Transforms.PoseVector2HomogenousMx(wb_ts.rhand.Pose);
	// left and right foot desired homogenous transformation in base frame with initial world orientation
	Trsf_des_ee2base_world[2]  	= myWalker.CpBalWlkController-> GaitTrsf.Trsf_lfoot_base_world;
	Trsf_des_ee2base_world[3] 	= myWalker.CpBalWlkController-> GaitTrsf.Trsf_rfoot_base_world;

	// left and right foot desired homogenous transformation in robot base frame
	Trsf_des_lfoot2base_robot  = myWalker.CpBalWlkController-> GaitTrsf.Trsf_lfoot_base_icub;
	Trsf_des_rfoot2base_robot  = myWalker.CpBalWlkController-> GaitTrsf.Trsf_rfoot_base_icub;

	// Transformation Com in the feet frames
	Trsf_des_com2lfoot	= myWalker.CpBalWlkController-> GaitTrsf.Trsf_lfoot_com.inverse();	
	Trsf_des_com2rfoot	= myWalker.CpBalWlkController-> GaitTrsf.Trsf_rfoot_com.inverse();

	// Tasks  ////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	if(myWalker.Parameters->StanceIndicator[0] == 1) // left stance
	{
		if(lstance_switch)
		{
	 		Trsf_lstancefoot2world = Transforms.PoseVector2HomogenousMx(wb_ts.lfoot.Pose);
	 		//
	 		lstance_switch = !lstance_switch;
	 		rstance_switch = !rstance_switch;
		}
		// 
		// Trsf_des_rfoot2world = Trsf_lstancefoot2world * Trsf_des_rswingfoot2lfoot;
		Trsf_des_ee2world[0] = Trsf_lstancefoot2world * Trsf_des_ee2base_world[2].inverse() * Trsf_des_ee2base_world[0];
		Trsf_des_ee2world[1] = Trsf_lstancefoot2world * Trsf_des_ee2base_world[2].inverse() * Trsf_des_ee2base_world[1];
		Trsf_des_ee2world[2] = Trsf_lstancefoot2world;
		Trsf_des_ee2world[3] = Trsf_lstancefoot2world * Trsf_des_ee2base_world[2].inverse() * Trsf_des_ee2base_world[3];
		Trsf_des_ee2world[4] = Trsf_lstancefoot2world * Trsf_des_com2lfoot;

	}
	else // right stance foot
	{
		if(rstance_switch)
		{
	 		Trsf_rstancefoot2world = Transforms.PoseVector2HomogenousMx(wb_ts.rfoot.Pose);
	 		//
	 		rstance_switch = !rstance_switch;
	 		lstance_switch = !lstance_switch;
		}
		// 
		// Trsf_des_lfoot2world = Trsf_rstancefoot2world * Trsf_des_lswingfoot2lfoot;
		Trsf_des_ee2world[0] = Trsf_rstancefoot2world * Trsf_des_ee2base_world[3].inverse() * Trsf_des_ee2base_world[0];
		Trsf_des_ee2world[1] = Trsf_rstancefoot2world * Trsf_des_ee2base_world[3].inverse() * Trsf_des_ee2base_world[1];
		Trsf_des_ee2world[2] = Trsf_rstancefoot2world * Trsf_des_ee2base_world[3].inverse() * Trsf_des_ee2base_world[2];
		Trsf_des_ee2world[3] = Trsf_rstancefoot2world; 
		Trsf_des_ee2world[4] = Trsf_rstancefoot2world * Trsf_des_com2rfoot;
	}
	

	// cout << " REF Trsf_lfoot_world   	IS \t " << Transforms.PoseVector2HomogenousMx(wb_ts.lfoot.Pose).block<3,1>(0,3).transpose()  << endl;
	// cout << " REF Trsf_rfoot_world   	IS \t " << Transforms.PoseVector2HomogenousMx(wb_ts.rfoot.Pose).block<3,1>(0,3).transpose()  << endl;
	// cout << " REF Trsf_lfoot_base_gen   IS \t " << Trsf_des_ee2base_world[2].block<3,1>(0,3).transpose()  << endl;
	// cout << " REF Trsf_rfoot_base_gen	IS \t " << Trsf_des_ee2base_world[3].block<3,1>(0,3).transpose()  << endl;
	// cout << " REF Trsf_com_world  		IS \t " << Trsf_des_ee2world[4].block<3,1>(0,3).transpose()  << endl;
	cout << " \n" << endl;
	cout << " REF position lfoot_base r IS \t " << Trsf_des_lfoot2base_robot.block<3,1>(0,3).transpose()  << endl;
	cout << " REF position rfoot_base r IS \t " << Trsf_des_rfoot2base_robot.block<3,1>(0,3).transpose()  << endl;

}