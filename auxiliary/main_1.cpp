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
#include "StepsGenerator.h"


using namespace std;
using namespace Eigen;

bool exit_sim = false;
void my_handler(int s)
{
	cout << "Exit Program Now ... " << endl;
	exit_sim = true;
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

int main(int argc, char *argv[])
{
	// setup exiting condition
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = my_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);

	// logger
	std::ofstream OutRecord;
	// OutRecord.open("./tmp/log.txt");
	OutRecord.open("./log.txt");
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	yarp::os::Property params;

	params.fromCommand(argc, argv);
    if (!params.check("robot"))
    {
        fprintf(stderr, "Please specify the name of the robot\n");
        fprintf(stderr, "--robot name (e.g. icubSim)\n");
        return 1;
    }
    std::string robotName = params.find("robot").asString();

	// ///////////////////////////////////////////////////////
	auxiliary_functions 	aux_function;

	aux_function.init(robotName);

    for (int i=0; i<10; i++) 
    	aux_function.get_hands_wrenches_offsets();

    //
    aux_function.StanceFoot[0] 	= 1; 		// left 
	aux_function.StanceFoot[1] 	= 0;		// right
	aux_function.isExtBase 		= true; 

    // Inverse kinematics
    aux_function.init_IK();

	// object properties
	// W.readObject(0, "BoxSmall");
	double obj_width = 0.3;
	Cvector3 des_obj_pos(0.2, 0.0, 0.65);
	Cvector3 des_obj_rot(0.0, 0.0, 0.0);
	Cvector3 next_obj_pos = des_obj_pos;
	Cvector3 next_obj_rot = des_obj_rot;
	bool grasp = false;
	//   STEP GENERATOR  /////////////////////////////////////////////////////////////////////////////////////////////
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//Load configuration-time parameters
    std::string m_moduleName    = "StepsGenerator";  
    std::string m_robotName     = robotName;  
    int period                  = 40; 
    double RunDuration          = 2.0; 
    int FT_feedbackType         = 0; 
 
    // convert the period from millisecond to second
    double m_period = 0.001 * (double) period;

    // instantiate the 
    StepsGenerator myWalker; //period is 40ms
    // Starting the BalanceWalkingController Thread
    myWalker.init(period, m_robotName, FT_feedbackType);

    // Set the Desired CoM velocity
    myWalker.Des_RelativeVelocity(0) = 0.00;
    myWalker.Des_RelativeVelocity(1) = 0.00;
    myWalker.Des_RelativeVelocity(2) = 0.00;



	// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// conversion of the frame due to the change of the base
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// robot base in in world frame defined between the feet (wo)
	Eigen::MatrixXd wo_T_b  	= Eigen::MatrixXd::Identity(4,4);
					wo_T_b(2,3) = aux_function.sens_pos(2);
	// robot base in in world reference frame (Origin in gazebo or stance foot)
	Eigen::MatrixXd	wld_T_b  = Eigen::MatrixXd::Identity(4,4);
					wld_T_b.block<3,3>(0,0) = aux_function.W.rotmatrix(Eigen::Vector3d(0,0,M_PI)) * quat2dc(aux_function.W.Root.segment(3,4));
					wld_T_b.block<3,1>(0,3) = aux_function.sens_pos.head(3);

	// Desired end-effector in world frame defined between the feet (wo)
	Eigen::Vector3d des_ee_pos_wo[5]; // [0] lhand, [1] rhand, [2] lfoot, [3] rfoot, [4] CM
					des_ee_pos_wo[0] = next_obj_pos + ang2dc(next_obj_rot) * Cvector3(0,obj_width/2.0,0);
					des_ee_pos_wo[1] = next_obj_pos + ang2dc(next_obj_rot) * Cvector3(0,-obj_width/2.0,0);
					des_ee_pos_wo[2] = Cvector3(0.00, 0.065, 0.00); 
					des_ee_pos_wo[3] = Cvector3(0.00,-0.065, 0.00);
					des_ee_pos_wo[4] = Cvector3(0.05, 0.000, 0.47);

	// Desired end-effector in the robot base frame (root link) with world orientation
	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		// Position
	Eigen::Vector3d des_ee_pos_b[5]; // [0] lhand, [1] rhand, [2] lfoot, [3] rfoot, [4] CM	
		for(int i=0; i<5; i++) 
		{ 
			des_ee_pos_b[i] =   wo_T_b.block<3,3>(0,0).transpose() * des_ee_pos_wo[i] 
							  - wo_T_b.block<3,3>(0,0).transpose() * wo_T_b.block<3,1>(0,3); 
		}	
		// rotation
	Eigen::Matrix3d des_ee_rot_b[5]; // [0] lhand, [1] rhand, [2] lfoot, [3] rfoot, [4] CM
					des_ee_rot_b[0] = quat2dc(quat_mul(ang2quat(next_obj_rot), ang2quat(Cvector3(0,-M_PI/2,0))));
					des_ee_rot_b[1] = quat2dc(quat_mul(ang2quat(next_obj_rot), ang2quat(Cvector3(0,-M_PI/2,0))));
					des_ee_rot_b[2] = quat2dc(zero_quat); 
					des_ee_rot_b[3] = quat2dc(zero_quat);
					des_ee_rot_b[4] = quat2dc(zero_quat);
	// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Desired end-effector in world reference frame (Origin in gazebo or stance foot)
	Eigen::Vector3d des_ee_pos_wld[5]; 	// [0] lhand, [1] rhand, [2] lfoot, [3] rfoot, [4] CM	

	wb_ref_position 	ref_p_wb; 		// reference position of the end-effectors
	wb_ref_orientation  ref_o_wb;		// reference orientation of the end-effectors



	// LOOP ///////////////////////////////////////////////////////////////////////////////////////////////////////
	// nonblock(1);
	// while(!exit_sim)
	for(int i=0; i<5; i++) 
	{
		double time = yarp::os::Time::now();
		// aux_function.get_estimated_hands_wrenches();
		// set the Base-CoM offset
		Eigen::Vector3d t_B_CoM = aux_function.sens_pos.head(3) - aux_function.model.get_cm();

		myWalker.CpBalWlkController->GaitTrsf.SetTranslationBaseCoM(t_B_CoM);
		// Loop for the walking
        myWalker.walk();
        // Update the stance foot
		aux_function.StanceFoot[0] 	= myWalker.Parameters->StanceIndicator[0]; 		// left if 1 
		aux_function.StanceFoot[1] 	= myWalker.Parameters->StanceIndicator[1];		// right

  //       myWalker.CpBalWlkController-> GaitTrsf.Trsf_lfoot_base_world
		// myWalker.CpBalWlkController-> GaitTrsf.Trsf_rfoot_base_world

		// cout <<" POSITION LEFT  FOOT IN ROBOT BASE  is :	\n " << des_ee_pos_b[2] << endl;
		// cout <<" POSITION RIGHT FOOT IN ROBOT BASE  is :	\n " << des_ee_pos_b[3] << endl;

		cout <<" TRANSF LEFT  FOOT IN BASE  is :	\n " << myWalker.CpBalWlkController-> GaitTrsf.Trsf_lfoot_base_world << endl;
		cout <<" TRANSF RIGHT FOOT IN BASE  is :	\n " << myWalker.CpBalWlkController-> GaitTrsf.Trsf_rfoot_base_world << endl;
		// left foor postion and orientation
		des_ee_pos_b[2] = myWalker.CpBalWlkController-> GaitTrsf.Trsf_lfoot_base_world.block<3,1>(0,3);
		des_ee_rot_b[2] = myWalker.CpBalWlkController-> GaitTrsf.Trsf_lfoot_base_world.block<3,3>(0,0);
		// right foor postion and orientation
		des_ee_pos_b[3] = myWalker.CpBalWlkController-> GaitTrsf.Trsf_rfoot_base_world.block<3,1>(0,3);
		des_ee_rot_b[3] = myWalker.CpBalWlkController-> GaitTrsf.Trsf_rfoot_base_world.block<3,3>(0,0);
		// Trsf_lfoot_com
		// Trsf_rfoot_com
		cout <<" TRANSF BASE 2 WORLD 0  is :	\n " << wo_T_b << endl;
		cout <<" POSITION LEFT  FOOT IN GEN BASE  is :	\n " << des_ee_pos_b[2] << endl;
		cout <<" POSITION RIGHT FOOT IN GEN BASE  is :	\n " << des_ee_pos_b[3] << endl;


		// Tasks  ////////////////////////////////////////////////////////////////////////////////////////////
		// hands and feet 
		for(int j=0; j<4; j++) 
		{
			// position
			ref_p_wb.p[j] 	=   wld_T_b.block<3,3>(0,0) * des_ee_pos_b[j] + wld_T_b.block<3,1>(0,3); 
			// orientation
			ref_o_wb.o[j] 	=   dc2quat(wld_T_b.block<3,3>(0,0) * des_ee_rot_b[j]);
		}
		// computation of the Com in the world frame
		Matrix4d Trsf_com_world 	 = Eigen::MatrixXd::Identity(4,4);
		Matrix4d Trsf_lfoot_world 	 = Eigen::MatrixXd::Identity(4,4);
		Matrix4d Trsf_rfoot_world    = Eigen::MatrixXd::Identity(4,4);

		Trsf_lfoot_world.block<3,3>(0,0) = wld_T_b.block<3,3>(0,0) * des_ee_rot_b[2];
		Trsf_lfoot_world.block<3,1>(0,3) =  wld_T_b.block<3,3>(0,0) * des_ee_pos_b[2] + wld_T_b.block<3,1>(0,3);

		Trsf_rfoot_world.block<3,3>(0,0) = wld_T_b.block<3,3>(0,0) * des_ee_rot_b[3];
		Trsf_rfoot_world.block<3,1>(0,3) = wld_T_b.block<3,3>(0,0) * des_ee_pos_b[3]  + wld_T_b.block<3,1>(0,3);

		cout <<" TRANSF LEFT  FOOT IN WORLD is :	\n " << Trsf_lfoot_world << endl;
		cout <<" TRANSF RIGHT FOOT IN WORLD  is :	\n " << Trsf_rfoot_world << endl;

		if(aux_function.StanceFoot[0] == 1) Trsf_com_world  = 	Trsf_lfoot_world * myWalker.CpBalWlkController-> GaitTrsf.Trsf_lfoot_com.inverse();		// left stance foot
		else								Trsf_com_world  = 	Trsf_rfoot_world * myWalker.CpBalWlkController-> GaitTrsf.Trsf_rfoot_com.inverse(); 	// right stance foot
		
		// com task
		ref_p_wb.p[4] = Trsf_com_world.block<3,1>(0,3);				// position
		ref_o_wb.o[4] = dc2quat(Trsf_com_world.block<3,3>(0,0));	// orientation
	
		// IK
		aux_function.get_inverse_kinematics(ref_p_wb,  ref_o_wb);

		// cout <<" REFERENCE POS is : 			\n " << aux_function.ref_pos << endl;
		cout <<" REFERENCE POS LEFT LEG is : 	\n " << 180./M_PI *aux_function.ref_pos.segment(12,6) << endl;
		cout <<" REFERENCE POS RIGHT LEG is : 	\n " << 180./M_PI *aux_function.ref_pos.segment(18,6) << endl;
		cout <<" BASE POSITION is : 			\n " << aux_function.sens_pos.head(3) << endl;
		cout <<" COM POSITION is : 				\n " << aux_function.model.get_cm() << endl;
		cout <<" BASE COM OFFSET  is : 		 	\n " << t_B_CoM << endl;
		cout <<" COM IN WORLD  is : 		 	\n " << Trsf_com_world << endl;
		
		// log joint-space positions
		// OutRecord << time << " " << sens_pos.transpose() << endl;
		cout <<" SOLUTION TIME is : \t " << yarp::os::Time::now() - time << " s " << endl;
	}

	OutRecord.close();
	// Set to Zero the Desired CoM velocity before stoping
    myWalker.Des_RelativeVelocity(0) = 0.00;  // TO DO
    myWalker.Des_RelativeVelocity(1) = 0.00;
    myWalker.Des_RelativeVelocity(2) = 0.00;

    // Loop for the walking
    myWalker.Release();

	aux_function.W.initializeJoint(Cvector::Zero(AIR_N_U-6));
	// nonblock(0);
	aux_function.W.close();
	return 0;
}
