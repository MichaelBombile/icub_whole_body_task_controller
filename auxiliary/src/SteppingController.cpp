// #include "SteppingController.h"

// using namespace std;
// using namespace Eigen;

// SteppingController::SteppingController(){}
// SteppingController::~SteppingController(){}

// bool SteppingController::Initilize(std::string robot_name, int period_, int feed_back_type)
// {
// // 	//
// // 	period 		= period_;
// // 	robotName 	= robot_name;
// // 	//
// // 	aux_function.init(robotName);
// // 	//
// //  //    aux_function.StanceFoot[0] 	= 0; 		// left 
// // 	// aux_function.StanceFoot[1] 	= 1;		// right
// // 	aux_function.isExtBase 		= true; 

// // 	// Inverse kinematics
// //     aux_function.init_IK();

// //     //   STEP GENERATOR  //////////////////////////////////////////////////////////////////////
// // 	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// //     // Starting the BalanceWalkingController Thread
// //     myWalker.init(period, robotName, feed_back_type);
// //     // Set the Desired CoM velocity
// //     myWalker.Des_RelativeVelocity(0) = 0.00;
// //     myWalker.Des_RelativeVelocity(1) = 0.00;
// //     myWalker.Des_RelativeVelocity(2) = 0.00;
// //     //
// //     // Update the stance foot
// // 	aux_function.StanceFoot[0] 	= myWalker.Parameters->StanceIndicator[0]; 		// left if 1 
// // 	aux_function.StanceFoot[1] 	= myWalker.Parameters->StanceIndicator[1];		// right

// // 	if(aux_function.StanceFoot[0] == 1){
// // 		lstance_switch = true;
// // 		rstance_switch = false;
// // 	}else{
// // 		lstance_switch = false;
// // 		rstance_switch = true;
// // 	}

// // 	// left and right foot desired homogenous transformation in base frame with initial world orientation
// // 	Trsf_des_ee2base_world[2]  	= myWalker.CpBalWlkController-> GaitTrsf.Trsf_lfoot_base_world;
// // 	Trsf_des_ee2base_world[3] 	= myWalker.CpBalWlkController-> GaitTrsf.Trsf_rfoot_base_world;
// // 	// Transformation Com in the feet frames
// // 	Trsf_des_com2lfoot			= myWalker.CpBalWlkController-> GaitTrsf.Trsf_lfoot_com.inverse();	
// // 	Trsf_des_com2rfoot			= myWalker.CpBalWlkController-> GaitTrsf.Trsf_rfoot_com.inverse();

// // 	Trsf_des_lfoot2base_robot  = myWalker.CpBalWlkController-> GaitTrsf.Trsf_lfoot_base_icub;
// // 	Trsf_des_rfoot2base_robot  = myWalker.CpBalWlkController-> GaitTrsf.Trsf_rfoot_base_icub;

// //     // robot base in in world reference frame (Origin in gazebo or stance foot)
// //     Trsf_base2world.setIdentity();
// //     Trsf_base2world.block<3,3>(0,0) = aux_function.W.rotmatrix(Eigen::Vector3d(0,0,M_PI)) * quat2dc(aux_function.W.Root.segment(3,4));
// // 	Trsf_base2world.block<3,1>(0,3) = aux_function.sens_pos.head(3);
// // 	//
// // 	Trsf_lfoot2base_world.setIdentity();
// // 	Trsf_rfoot2base_world.setIdentity();
// // 	Trsf_lfoot2base_world.block<3,1>(0,3) = aux_function.points.M->get_pos(aux_function.points[CN_LF].body, aux_function.points[CN_LF].offset);
// // 	Trsf_lfoot2base_world.block<3,3>(0,0) = aux_function.points.M->get_orient(aux_function.points[CN_LF].body);
// // 	Trsf_rfoot2base_world.block<3,1>(0,3) = aux_function.points.M->get_pos(aux_function.points[CN_RF].body, aux_function.points[CN_RF].offset);
// // 	Trsf_rfoot2base_world.block<3,3>(0,0) = aux_function.points.M->get_orient(aux_function.points[CN_RF].body);

// // 	//
// 	return true;
// }

// bool SteppingController::InitStepper(std::string robot_name, int period_, int feed_back_type)
// {
// 	//
// 	period 		= period_;
// 	robotName 	= robot_name;
// 	//
//     //   STEP GENERATOR  //////////////////////////////////////////////////////////////////////
// 	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//     // Starting the BalanceWalkingController Thread
//     myWalker.init(period, robotName, feed_back_type);
//     // Set the Desired CoM velocity
//     myWalker.Des_RelativeVelocity(0) = 0.00;
//     myWalker.Des_RelativeVelocity(1) = 0.00;
//     myWalker.Des_RelativeVelocity(2) = 0.00;
//     //
//     // Update the stance foot
// 	if(myWalker.Parameters->StanceIndicator[0] == 1){   // left if 1 
// 		lstance_switch = true;
// 		rstance_switch = false;
// 	}else{												// right
// 		lstance_switch = false;
// 		rstance_switch = true;
// 	}

// 	// left and right foot desired homogenous transformation in base frame with initial world orientation
// 	Trsf_des_ee2base_world[2]  	= myWalker.CpBalWlkController-> GaitTrsf.Trsf_lfoot_base_world;
// 	Trsf_des_ee2base_world[3] 	= myWalker.CpBalWlkController-> GaitTrsf.Trsf_lfoot_base_world;
// 	// Transformation Com in the feet frames
// 	Trsf_des_com2lfoot			= myWalker.CpBalWlkController-> GaitTrsf.Trsf_lfoot_com.inverse();	
// 	Trsf_des_com2rfoot			= myWalker.CpBalWlkController-> GaitTrsf.Trsf_rfoot_com.inverse();

// 	Trsf_des_lfoot2base_robot  = myWalker.CpBalWlkController-> GaitTrsf.Trsf_lfoot_base_icub;
// 	Trsf_des_rfoot2base_robot  = myWalker.CpBalWlkController-> GaitTrsf.Trsf_rfoot_base_icub;

// 	//
// 	return true;
// }

// bool SteppingController::InitStepper(std::string robot_name, int period_, int feed_back_type, Vector3d init_com_pos)
// {
// 	//
// 	period 		= period_;
// 	robotName 	= robot_name;
// 	//
//     //   STEP GENERATOR  //////////////////////////////////////////////////////////////////////
// 	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//     // Starting the BalanceWalkingController Thread
//     // myWalker.init(period, robotName, feed_back_type);
//     myWalker.init(period, robotName, feed_back_type, init_com_pos);
//     // Set the Desired CoM velocity
//     myWalker.Des_RelativeVelocity(0) = 0.00;
//     myWalker.Des_RelativeVelocity(1) = 0.00;
//     myWalker.Des_RelativeVelocity(2) = 0.00;
//     //
//     // Update the stance foot
// 	if(myWalker.Parameters->StanceIndicator[0] == 1){   // left if 1 
// 		lstance_switch = true;
// 		rstance_switch = false;
// 	}else{												// right
// 		lstance_switch = false;
// 		rstance_switch = true;
// 	}

// 	// left and right foot desired homogenous transformation in base frame with initial world orientation
// 	Trsf_des_ee2base_world[2]  	= myWalker.CpBalWlkController-> GaitTrsf.Trsf_lfoot_base_world;
// 	Trsf_des_ee2base_world[3] 	= myWalker.CpBalWlkController-> GaitTrsf.Trsf_lfoot_base_world;
// 	// Transformation Com in the feet frames
// 	Trsf_des_com2lfoot			= myWalker.CpBalWlkController-> GaitTrsf.Trsf_lfoot_com.inverse();	
// 	Trsf_des_com2rfoot			= myWalker.CpBalWlkController-> GaitTrsf.Trsf_rfoot_com.inverse();

// 	Trsf_des_lfoot2base_robot  = myWalker.CpBalWlkController-> GaitTrsf.Trsf_lfoot_base_icub;
// 	Trsf_des_rfoot2base_robot  = myWalker.CpBalWlkController-> GaitTrsf.Trsf_rfoot_base_icub;

// 	//
// 	return true;
// }

// void SteppingController::release()
// {
//     // Set to Zero the Desired CoM velocity before stoping
//     myWalker.Des_RelativeVelocity(0) = 0.00;  // TO DO
//     myWalker.Des_RelativeVelocity(1) = 0.00;
//     myWalker.Des_RelativeVelocity(2) = 0.00;

//     myWalker.Release();
//     // aux_function.W.close();
// }

// void SteppingController::releaseStepper()
// {
//     // Set to Zero the Desired CoM velocity before stoping
//     myWalker.Des_RelativeVelocity(0) = 0.00;  // TO DO
//     myWalker.Des_RelativeVelocity(1) = 0.00;
//     myWalker.Des_RelativeVelocity(2) = 0.00;

//     myWalker.Release();
// }

// // bool SteppingController::set_handsTasks_in_base(Vector7d lh_pose, Vector7d rh_pose, Vector7d pelvis_pose)
// // {
// // 	//
// // 	MatrixXd  	w_T_lh = Transforms.PoseVector2HomogenousMx(lh_pose); 
// // 	MatrixXd  	w_T_rh = Transforms.PoseVector2HomogenousMx(rh_pose);
// // 	MatrixXd  	w_T_pe = Transforms.PoseVector2HomogenousMx(pelvis_pose);

// // 	Matrix4d  	w_T_pe_w =  w_T_pe;
// // 				w_T_pe_w.block(0,0, 3,3) = aux_function.W.rotmatrix(Eigen::Vector3d(0,0,M_PI)) * w_T_pe.block(0,0, 3,3);

// // 	Matrix4d w_pe_T_lh = w_T_pe_w.inverse() * w_T_lh;
// // 	Matrix4d w_pe_T_rh = w_T_pe_w.inverse() * w_T_rh;

// // 	des_ee_pos_b[0] = w_pe_T_lh.block<3,1>(0,3);
// // 	des_ee_pos_b[1] = w_pe_T_rh.block<3,1>(0,3);
	
// // 	des_ee_rot_b[0] =  w_pe_T_lh.block<3,3>(0,0);
// // 	des_ee_rot_b[1] =  w_pe_T_rh.block<3,3>(0,0);

// // 	return true;
// // }


// // void SteppingController::step(int number_steps, double stride_x, Eigen::VectorXd &ref_joints_position)
// // {
	
// // 	// set the Base-CoM offset
// // 	Eigen::Vector3d t_B_CoM = aux_function.sens_pos.head(3) - aux_function.model.get_cm();

// // 	myWalker.CpBalWlkController->GaitTrsf.SetTranslationBaseCoM(t_B_CoM);
// // 	// Loop for the walking
// //     myWalker.walk();
// //     // Update the stance foot
// // 	aux_function.StanceFoot[0] 	= myWalker.Parameters->StanceIndicator[0]; 		// left if 1 
// // 	aux_function.StanceFoot[1] 	= myWalker.Parameters->StanceIndicator[1];		// right

// // 		// left foor postion and orientation
// // 	des_ee_pos_b[2] = myWalker.CpBalWlkController-> GaitTrsf.Trsf_lfoot_base_world.block<3,1>(0,3);
// // 	des_ee_rot_b[2] = myWalker.CpBalWlkController-> GaitTrsf.Trsf_lfoot_base_world.block<3,3>(0,0);
// // 	// right foor postion and orientation
// // 	des_ee_pos_b[3] = myWalker.CpBalWlkController-> GaitTrsf.Trsf_rfoot_base_world.block<3,1>(0,3);
// // 	des_ee_rot_b[3] = myWalker.CpBalWlkController-> GaitTrsf.Trsf_rfoot_base_world.block<3,3>(0,0);

		
// // 	// Tasks  ////////////////////////////////////////////////////////////////////////////////////////////////////
// // 	// hands and feet 
// // 	for(int j=0; j<4; j++) 
// // 	{
// // 		ref_p_wb.p[j] 	=   Trsf_base2world.block<3,3>(0,0) * des_ee_pos_b[j] + Trsf_base2world.block<3,1>(0,3);  	// position	
// // 		ref_o_wb.o[j] 	=   dc2quat(Trsf_base2world.block<3,3>(0,0) * des_ee_rot_b[j]);						// orientation
// // 	}
// // 	// computation of the Com in the world frame
// // 	Matrix4d Trsf_com_world 	 = Eigen::MatrixXd::Identity(4,4);
// // 	Matrix4d Trsf_lfoot_world 	 = Eigen::MatrixXd::Identity(4,4);
// // 	Matrix4d Trsf_rfoot_world    = Eigen::MatrixXd::Identity(4,4);

// // 	Trsf_lfoot_world.block<3,3>(0,0) = Trsf_base2world.block<3,3>(0,0) * des_ee_rot_b[2];
// // 	Trsf_lfoot_world.block<3,1>(0,3) = Trsf_base2world.block<3,3>(0,0) * des_ee_pos_b[2] + Trsf_base2world.block<3,1>(0,3);

// // 	Trsf_rfoot_world.block<3,3>(0,0) = Trsf_base2world.block<3,3>(0,0) * des_ee_rot_b[3];
// // 	Trsf_rfoot_world.block<3,1>(0,3) = Trsf_base2world.block<3,3>(0,0) * des_ee_pos_b[3]  + Trsf_base2world.block<3,1>(0,3);
// // 	// transformation as function of the stance foot
// // 	if(aux_function.StanceFoot[0] == 1) Trsf_com_world  = 	Trsf_lfoot_world * myWalker.CpBalWlkController-> GaitTrsf.Trsf_lfoot_com.inverse();		// left stance foot
// // 	else								Trsf_com_world  = 	Trsf_rfoot_world * myWalker.CpBalWlkController-> GaitTrsf.Trsf_rfoot_com.inverse(); 	// right stance foot
	
// // 	// com task
// // 	ref_p_wb.p[4] = Trsf_com_world.block<3,1>(0,3);				// position
// // 	ref_o_wb.o[4] = dc2quat(Trsf_com_world.block<3,3>(0,0));	// orientation
// // 	// IK solution ///////////////////////////////////////////////////////////////////////////////////////////////
// // 	aux_function.get_inverse_kinematics(ref_p_wb,  ref_o_wb);

// // 	ref_joints_position.segment(0, 3)  = aux_function.ref_pos.segment( 6,3);
// // 	ref_joints_position.segment(3, 7)  = aux_function.ref_pos.segment(24,7);
// // 	ref_joints_position.segment(10, 7) = aux_function.ref_pos.segment(31,7);
// // 	ref_joints_position.segment(17, 6) = aux_function.ref_pos.segment(12,6);
// // 	ref_joints_position.segment(23, 6) = aux_function.ref_pos.segment(18,6);

// // 	// update the base frame with respect to the world.
// // 	Trsf_base2world.block<3,3>(0,0) = aux_function.W.rotmatrix(Eigen::Vector3d(0,0,M_PI)) * quat2dc(aux_function.W.Root.segment(3,4));
// // 	Trsf_base2world.block<3,1>(0,3) = aux_function.sens_pos.head(3);

	
// // 	cout << " REF Trsf_lfoot_world   IS \t " << Trsf_lfoot_world.block<3,1>(0,3).transpose()  << endl;
// // 	cout << " REF Trsf_rfoot_world   IS \t " << Trsf_rfoot_world.block<3,1>(0,3).transpose()  << endl;
// // 	// cout << " REF Trsf_lfoot_com_gen   IS \t " << myWalker.CpBalWlkController-> GaitTrsf.Trsf_lfoot_com.block<3,1>(0,3).transpose()  << endl;
// // 	// cout << " REF Trsf_rfoot_com_gen   IS \t " << myWalker.CpBalWlkController-> GaitTrsf.Trsf_rfoot_com.block<3,1>(0,3).transpose()  << endl;

// // 	cout << " REF Trsf_lfoot_base_gen   IS \t " << des_ee_pos_b[2].transpose()  << endl;
// // 	cout << " REF Trsf_rfoot_base_gen   IS \t " << des_ee_pos_b[3].transpose()  << endl;
// // 	cout << " REF Trsf_com_world  	 IS \t " << Trsf_com_world.block<3,1>(0,3).transpose()  << endl;
// // 	cout << " REF Trsf_base2world [3] 	 IS \t " << Trsf_base2world.block<3,1>(0,3).transpose()  << endl;

// // 	cout <<" REFERENCE ANG LEFT LEG is : 	\t " << 180./M_PI *aux_function.ref_pos.segment(12,6).transpose()  << endl;
// // 	cout <<" REFERENCE ANG RIGHT LEG is : 	\t " << 180./M_PI *aux_function.ref_pos.segment(18,6).transpose()  << endl;
	

// // }

// bool SteppingController::set_handsTasks_in_base(Vector7d lh_pose, Vector7d rh_pose, Vector7d pelvis_pose)
// {
// 	//
// 	MatrixXd  	w_T_lh = Transforms.PoseVector2HomogenousMx(lh_pose); 
// 	MatrixXd  	w_T_rh = Transforms.PoseVector2HomogenousMx(rh_pose);
// 	MatrixXd  	w_T_pe = Transforms.PoseVector2HomogenousMx(pelvis_pose);

// 	Matrix4d  	w_T_pe_w =  w_T_pe;
// 				w_T_pe_w.block(0,0, 3,3) = Transforms.rot_vector2rpy(Eigen::Vector3d(0,0,M_PI)) * w_T_pe.block(0,0, 3,3);
// 				// w_T_pe_w.block(0,0, 3,3) = aux_function.W.rotmatrix(Eigen::Vector3d(0,0,M_PI)) * w_T_pe.block(0,0, 3,3);
				

// 	Matrix4d w_pe_T_lh = w_T_pe_w.inverse() * w_T_lh;
// 	Matrix4d w_pe_T_rh = w_T_pe_w.inverse() * w_T_rh;

// 	// left and right hands desired homogenous transformation in base frame with initial world orientation
// 	Trsf_des_ee2base_world[0] = w_pe_T_lh;
// 	Trsf_des_ee2base_world[1] = w_pe_T_rh;

// 	return true;
// }

// void SteppingController::step(int number_steps, double stride_x, Eigen::VectorXd &ref_joints_position)
// {
	
// 	// // set the Base-CoM offset
// 	// Eigen::Vector3d t_B_CoM = aux_function.sens_pos.head(3) - aux_function.model.get_cm();

// 	// myWalker.CpBalWlkController->GaitTrsf.SetTranslationBaseCoM(t_B_CoM);
// 	// // Loop for the walking
//  //    myWalker.walk();
//  //    // Update the stance foot
// 	// aux_function.StanceFoot[0] 	= myWalker.Parameters->StanceIndicator[0]; 		// left if 1 
// 	// aux_function.StanceFoot[1] 	= myWalker.Parameters->StanceIndicator[1];		// right

// 	// // left and right foot desired homogenous transformation in base frame with initial world orientation
// 	// Trsf_des_ee2base_world[2]  	= myWalker.CpBalWlkController-> GaitTrsf.Trsf_lfoot_base_world;
// 	// Trsf_des_ee2base_world[3] 	= myWalker.CpBalWlkController-> GaitTrsf.Trsf_rfoot_base_world;
// 	// // Transformation Com in the feet frames
// 	// Trsf_des_com2lfoot	= myWalker.CpBalWlkController-> GaitTrsf.Trsf_lfoot_com.inverse();	
// 	// Trsf_des_com2rfoot	= myWalker.CpBalWlkController-> GaitTrsf.Trsf_rfoot_com.inverse();

// 	// // Tasks  ////////////////////////////////////////////////////////////////////////////////////////////////////
// 	// //
// 	// if(aux_function.StanceFoot[0] == 1) // left stance
// 	// {
// 	// 	if(lstance_switch)
// 	// 	{
// 	//  		// Trsf_lstancefoot2world = Trsf_base2world * Trsf_lfoot2base_world;
// 	//  		Trsf_lstancefoot2world = Trsf_lfoot2base_world;
// 	//  		//
// 	//  		lstance_switch = !lstance_switch;
// 	//  		rstance_switch = !rstance_switch;

// 	//  		cout << " Here left stance \t " << 100000.00  << endl;
// 	// 	}
// 	// 	// 
// 	// 	// Trsf_des_rfoot2world = Trsf_lstancefoot2world * Trsf_des_rswingfoot2lfoot;
// 	// 	Trsf_des_ee2world[0] = Trsf_lstancefoot2world * Trsf_des_ee2base_world[2].inverse() * Trsf_des_ee2base_world[0];
// 	// 	Trsf_des_ee2world[1] = Trsf_lstancefoot2world * Trsf_des_ee2base_world[2].inverse() * Trsf_des_ee2base_world[1];
// 	// 	Trsf_des_ee2world[2] = Trsf_lstancefoot2world;
// 	// 	Trsf_des_ee2world[3] = Trsf_lstancefoot2world * Trsf_des_ee2base_world[2].inverse() * Trsf_des_ee2base_world[3];
// 	// 	Trsf_des_ee2world[4] = Trsf_lstancefoot2world * Trsf_des_com2lfoot;

// 	// }
// 	// else // right stance foot
// 	// {
// 	// 	if(rstance_switch)
// 	// 	{
// 	//  		// Trsf_rstancefoot2world = Trsf_base2world * Trsf_rfoot2base_world;
// 	//  		Trsf_rstancefoot2world = Trsf_rfoot2base_world;
// 	//  		//
// 	//  		rstance_switch = !rstance_switch;
// 	//  		lstance_switch = !lstance_switch;

// 	//  		cout << " Here right stance \t " << 200000.00  << endl;
// 	// 	}
// 	// 	// 
// 	// 	// Trsf_des_lfoot2world = Trsf_rstancefoot2world * Trsf_des_lswingfoot2lfoot;
// 	// 	Trsf_des_ee2world[0] = Trsf_rstancefoot2world * Trsf_des_ee2base_world[3].inverse() * Trsf_des_ee2base_world[0];
// 	// 	Trsf_des_ee2world[1] = Trsf_rstancefoot2world * Trsf_des_ee2base_world[3].inverse() * Trsf_des_ee2base_world[1];
// 	// 	Trsf_des_ee2world[2] = Trsf_rstancefoot2world * Trsf_des_ee2base_world[3].inverse() * Trsf_des_ee2base_world[2];
// 	// 	Trsf_des_ee2world[3] = Trsf_rstancefoot2world; 
// 	// 	Trsf_des_ee2world[4] = Trsf_rstancefoot2world * Trsf_des_com2rfoot;
// 	// }

// 	// for(int j=0; j<5; j++) 
// 	// {
// 	// 	ref_p_wb.p[j] 	=   Trsf_des_ee2world[j].block<3,1>(0,3);  			// position	
// 	// 	ref_o_wb.o[j] 	=   dc2quat(Trsf_des_ee2world[j].block<3,3>(0,0));	// orientation
// 	// }
// 	// // IK solution ///////////////////////////////////////////////////////////////////////////////////////////////
// 	// aux_function.get_inverse_kinematics(ref_p_wb,  ref_o_wb);

// 	// ref_joints_position.segment(0, 3)  = aux_function.ref_pos.segment( 6,3);
// 	// ref_joints_position.segment(3, 7)  = aux_function.ref_pos.segment(24,7);
// 	// ref_joints_position.segment(10, 7) = aux_function.ref_pos.segment(31,7);
// 	// ref_joints_position.segment(17, 6) = aux_function.ref_pos.segment(12,6);
// 	// ref_joints_position.segment(23, 6) = aux_function.ref_pos.segment(18,6);

// 	// // update the base frame with respect to the world.
// 	// Trsf_base2world.block<3,3>(0,0) = aux_function.W.rotmatrix(Eigen::Vector3d(0,0,M_PI)) * quat2dc(aux_function.W.Root.segment(3,4));
// 	// Trsf_base2world.block<3,1>(0,3) = aux_function.sens_pos.head(3);
// 	// //
// 	// Trsf_lfoot2base_world.block<3,1>(0,3) = aux_function.points.M->get_pos(aux_function.points[CN_LF].body, aux_function.points[CN_LF].offset);
// 	// Trsf_lfoot2base_world.block<3,3>(0,0) = aux_function.points.M->get_orient(aux_function.points[CN_LF].body);
// 	// Trsf_rfoot2base_world.block<3,1>(0,3) = aux_function.points.M->get_pos(aux_function.points[CN_RF].body, aux_function.points[CN_RF].offset);
// 	// Trsf_rfoot2base_world.block<3,3>(0,0) = aux_function.points.M->get_orient(aux_function.points[CN_RF].body);
// 	// //
// 	// Matrix4d Trsf_lfoot_world 	 = Eigen::MatrixXd::Identity(4,4);
// 	// Matrix4d Trsf_rfoot_world    = Eigen::MatrixXd::Identity(4,4);

// 	// // Trsf_lfoot_world = Trsf_base2world * Trsf_lfoot2base_world;
// 	// // Trsf_rfoot_world = Trsf_base2world * Trsf_rfoot2base_world;

// 	// Trsf_lfoot_world = Trsf_lfoot2base_world;
// 	// Trsf_rfoot_world = Trsf_rfoot2base_world;
	
// 	// cout << " REF Trsf_lfoot_world   IS \t " << Trsf_lfoot_world.block<3,1>(0,3).transpose()  << endl;
// 	// cout << " REF Trsf_rfoot_world   IS \t " << Trsf_rfoot_world.block<3,1>(0,3).transpose()  << endl;
// 	// // cout << " REF Trsf_lfoot_com_gen   IS \t " << myWalker.CpBalWlkController-> GaitTrsf.Trsf_lfoot_com.block<3,1>(0,3).transpose()  << endl;
// 	// // cout << " REF Trsf_rfoot_com_gen   IS \t " << myWalker.CpBalWlkController-> GaitTrsf.Trsf_rfoot_com.block<3,1>(0,3).transpose()  << endl;

// 	// cout << " REF Trsf_lfoot_base_gen   IS \t " << Trsf_des_ee2base_world[2].block<3,1>(0,3).transpose()  << endl;
// 	// cout << " REF Trsf_rfoot_base_gen	IS \t " << Trsf_des_ee2base_world[3].block<3,1>(0,3).transpose()  << endl;
// 	// cout << " REF Trsf_com_world  		IS \t " << Trsf_des_ee2world[4].block<3,1>(0,3).transpose()  << endl;
// 	// cout << " REF Trsf_base2world [3] 	IS \t " << Trsf_base2world.block<3,1>(0,3).transpose()  << endl;
// 	// cout << " REF Trsf_lfoot_world	   	IS \t " << Trsf_des_ee2world[2].block<3,1>(0,3).transpose()  << endl;
// 	// cout << " REF Trsf_rfoot_world		IS \t " << Trsf_des_ee2world[3].block<3,1>(0,3).transpose()  << endl;

// 	// cout <<" REFERENCE ANG LEFT LEG is : 	\t " << 180./M_PI *aux_function.ref_pos.segment(12,6).transpose()  << endl;
// 	// cout <<" REFERENCE ANG RIGHT LEG is : 	\t " << 180./M_PI *aux_function.ref_pos.segment(18,6).transpose()  << endl;

// 	return;	
// }


// void SteppingController::get_step_transforms(int number_steps, double stride_x, Vector7d des_pose_lh, Vector7d des_pose_rh, WholeBodyTaskSpaceStates wb_ts)
// {
// 	//
// 	// myWalker.CpBalWlkController->step_size_x = stride_x;

	
// 	// set the Base-CoM offset
// 	// Eigen::Vector3d t_B_CoM = 	  Transforms.PoseVector2HomogenousMx(wb_ts.Pelvis.Pose).block(0,3,3,1) 
// 	// 							- Transforms.PoseVector2HomogenousMx(wb_ts.CoM.Pose).block(0,3,3,1);
// 	Eigen::Vector3d t_B_CoM;
// 	// t_B_CoM = 	  wb_ts.Pelvis.Pose.head(3) - wb_ts.CoM.Pose.head(3) + Vector3d(0.04, 0.0, 0.0);
// 	t_B_CoM <<  0.0, //wb_ts.Pelvis.Pose(0) - wb_ts.CoM.Pose(0), 
// 				0.0, //wb_ts.Pelvis.Pose(1) - wb_ts.CoM.Pose(1), 
// 				wb_ts.Pelvis.Pose(2) - wb_ts.CoM.Pose(2);

// 	myWalker.CpBalWlkController->GaitTrsf.SetTranslationBaseCoM(t_B_CoM);
// 	// Loop for the walking
//     myWalker.walk();
// 	//
// 	Matrix4d  	w_T_pe_w 				 = Transforms.PoseVector2HomogenousMx(wb_ts.Pelvis.Pose);
// 				w_T_pe_w.block(0,0, 3,3) = Transforms.rot_vector2rpy(Eigen::Vector3d(0,0,M_PI)) * w_T_pe_w.block(0,0, 3,3);
// 	// left and right hands desired homogenous transformation in base frame with initial world orientation
// 	Trsf_des_ee2base_world[0]   = w_T_pe_w.inverse() * Transforms.PoseVector2HomogenousMx(wb_ts.lhand.Pose);
// 	Trsf_des_ee2base_world[1]   = w_T_pe_w.inverse() * Transforms.PoseVector2HomogenousMx(wb_ts.rhand.Pose);
// 	// left and right foot desired homogenous transformation in base frame with initial world orientation
// 	Trsf_des_ee2base_world[2]  	= myWalker.CpBalWlkController->GaitTrsf.Trsf_lfoot_base_world;
// 	Trsf_des_ee2base_world[3] 	= myWalker.CpBalWlkController->GaitTrsf.Trsf_rfoot_base_world;

// 	// left and right foot desired homogenous transformation in robot base frame
// 	Trsf_des_lfoot2base_robot  = myWalker.CpBalWlkController->GaitTrsf.Trsf_lfoot_base_icub;
// 	Trsf_des_rfoot2base_robot  = myWalker.CpBalWlkController->GaitTrsf.Trsf_rfoot_base_icub;

// 	// Transformation Com in the feet frames
// 	Trsf_des_com2lfoot	= myWalker.CpBalWlkController->GaitTrsf.Trsf_lfoot_com.inverse();	
// 	Trsf_des_com2rfoot	= myWalker.CpBalWlkController->GaitTrsf.Trsf_rfoot_com.inverse();

// 	std::cout << " COM POSITION IS : \t" << wb_ts.CoM.Pose.head(3).transpose() << std::endl;
// 	std::cout << " COM IN  LEFT FOOT IS : \t" << Trsf_des_com2lfoot.block(0,3,3,1).transpose() << std::endl;
// 	std::cout << " COM IN RIGHT FOOT IS : \t" << Trsf_des_com2rfoot.block(0,3,3,1).transpose() << std::endl;

// 	// Tasks  ////////////////////////////////////////////////////////////////////////////////////////////////////
// 	//
// 	if(myWalker.Parameters->StanceIndicator[0] == 1) // left stance
// 	{
// 		if(lstance_switch)
// 		{
// 	 		Trsf_lstancefoot2world = Transforms.PoseVector2HomogenousMx(wb_ts.lfoot.Pose);
// 	 		//
// 	 		lstance_switch = !lstance_switch;
// 	 		rstance_switch = !rstance_switch;
// 		}
// 		// 
// 		// Trsf_des_rfoot2world = Trsf_lstancefoot2world * Trsf_des_rswingfoot2lfoot;
// 		Trsf_des_ee2world[0] = Trsf_lstancefoot2world * Trsf_des_ee2base_world[2].inverse() * Trsf_des_ee2base_world[0];
// 		Trsf_des_ee2world[1] = Trsf_lstancefoot2world * Trsf_des_ee2base_world[2].inverse() * Trsf_des_ee2base_world[1];
// 		Trsf_des_ee2world[2] = Trsf_lstancefoot2world;
// 		Trsf_des_ee2world[3] = Trsf_lstancefoot2world * Trsf_des_ee2base_world[2].inverse() * Trsf_des_ee2base_world[3];
// 		Trsf_des_ee2world[4] = Trsf_lstancefoot2world * Trsf_des_com2lfoot;

// 	}
// 	else // right stance foot
// 	{
// 		if(rstance_switch)
// 		{
// 	 		Trsf_rstancefoot2world = Transforms.PoseVector2HomogenousMx(wb_ts.rfoot.Pose);
// 	 		//
// 	 		rstance_switch = !rstance_switch;
// 	 		lstance_switch = !lstance_switch;
// 		}
// 		// 
// 		// Trsf_des_lfoot2world = Trsf_rstancefoot2world * Trsf_des_lswingfoot2lfoot;
// 		Trsf_des_ee2world[0] = Trsf_rstancefoot2world * Trsf_des_ee2base_world[3].inverse() * Trsf_des_ee2base_world[0];
// 		Trsf_des_ee2world[1] = Trsf_rstancefoot2world * Trsf_des_ee2base_world[3].inverse() * Trsf_des_ee2base_world[1];
// 		Trsf_des_ee2world[2] = Trsf_rstancefoot2world * Trsf_des_ee2base_world[3].inverse() * Trsf_des_ee2base_world[2];
// 		Trsf_des_ee2world[3] = Trsf_rstancefoot2world; 
// 		Trsf_des_ee2world[4] = Trsf_rstancefoot2world * Trsf_des_com2rfoot;
// 	}
	

// 	// cout << " REF Trsf_lfoot_world   	IS \t " << Transforms.PoseVector2HomogenousMx(wb_ts.lfoot.Pose).block<3,1>(0,3).transpose()  << endl;
// 	// cout << " REF Trsf_rfoot_world   	IS \t " << Transforms.PoseVector2HomogenousMx(wb_ts.rfoot.Pose).block<3,1>(0,3).transpose()  << endl;
// 	// cout << " REF Trsf_lfoot_base_gen   IS \t " << Trsf_des_ee2base_world[2].block<3,1>(0,3).transpose()  << endl;
// 	// cout << " REF Trsf_rfoot_base_gen	IS \t " << Trsf_des_ee2base_world[3].block<3,1>(0,3).transpose()  << endl;
// 	// cout << " REF Trsf_com_world  		IS \t " << Trsf_des_ee2world[4].block<3,1>(0,3).transpose()  << endl;
// 	cout << " \n" << endl;
// 	cout << " REF position lfoot_base r IS \t " << Trsf_des_lfoot2base_robot.block<3,1>(0,3).transpose()  << endl;
// 	cout << " REF position rfoot_base r IS \t " << Trsf_des_rfoot2base_robot.block<3,1>(0,3).transpose()  << endl;

// }

