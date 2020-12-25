/*stepping controller that combined a walking pattern generator and inverse kinematics*/

#pragma once


#ifndef SteppingController_H
#define SteppingController_H

#include <string>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <cmath>
#include <signal.h>
#include <unistd.h>

// #include "wrapper.h"
// #include "IK.h"
// #include "ID.h"

// #include "auxiliary_functions.h"
#include "StepsGenerator.h"

using namespace std;
using namespace Eigen;

class SteppingController
{
    
    public :

        std::string robotName;
        int 		period;

       	/* function that contains a wrench estimator and an inverse kinematics solver*/
        // auxiliary_functions 	aux_function;

        
        /* Stepping or walking pattern generator */
        StepsGenerator 			myWalker;
        /* Desired end-effector in the robot base frame (root link) with world orientation*/
        // Eigen::Vector3d des_ee_pos_b[5]; // [0] lhand, [1] rhand, [2] lfoot, [3] rfoot, [4] CM   // position
        // Eigen::Matrix3d des_ee_rot_b[5]; // [0] lhand, [1] rhand, [2] lfoot, [3] rfoot, [4] CM   // orientation
        /* references tasks position and orientation */
        wb_ref_position 	ref_p_wb; 		// reference position of the end-effectors
		wb_ref_orientation  ref_o_wb;		// reference orientation of the end-effectors
		/*robot base in in world reference frame (Origin in gazebo or stance foot)*/
		// Matrix4d Trsf_base2world;
        //
        bool lstance_switch;
        bool rstance_switch;
        Matrix4d Trsf_des_ee2world[5];          // [0] lhand, [1] rhand, [2] lfoot, [3] rfoot, [4] CM   // position
        Matrix4d Trsf_des_ee2base_world[5];     // [0] lhand, [1] rhand, [2] lfoot, [3] rfoot, [4] CM   // position
        Matrix4d Trsf_lstancefoot2world;
        Matrix4d Trsf_rstancefoot2world;
        Matrix4d Trsf_base2world;               // from measurement or estimation
        Matrix4d Trsf_lfoot2base_world;         // form kinematic chain (using joint sensors)
        Matrix4d Trsf_rfoot2base_world;         // form kinematic chain (using joint sensors)
        Matrix4d Trsf_des_com2lfoot;
        Matrix4d Trsf_des_com2rfoot;

        Matrix4d Trsf_des_lfoot2base_robot;
        Matrix4d Trsf_des_rfoot2base_robot;

		//
		KineTransformations Transforms;

        // Methods
        SteppingController();
        ~SteppingController();

        bool Initilize(std::string robot_name, int period_, int feed_back_type);
        bool InitStepper(std::string robot_name, int period_, int feed_back_type);

        void step(int number_steps, double stride_x, Eigen::VectorXd &ref_joints_position);

        void release();
        void releaseStepper();

        bool set_handsTasks_in_base(Vector7d lh_pose, Vector7d rh_pose, Vector7d pelvis_pose);

        void get_step_transforms(int number_steps, double stride_x, Vector7d des_pose_lh, Vector7d des_pose_rh, WholeBodyTaskSpaceStates wb_ts);
        
};

#endif // SteppingController_H

