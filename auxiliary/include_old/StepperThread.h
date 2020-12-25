
#pragma once

#ifndef StepperThread_H
#define StepperThread_H

#include <string>
#include <iostream>
#include <fstream>
#include <cmath>

#include <yarp/os/RateThread.h>
#include <yarp/os/Mutex.h>
#include <wbi/wholeBodyInterface.h>

#include "StepsGenerator.h"

using namespace std;
using namespace Eigen;

class StepperThread : public yarp::os::RateThread 
{

	// private :

	// 	int ThreadPeriod;						// thread period
	public :

        std::string robotName;
        int ThreadPeriod;						// thread period
        // int 		period;
        int 		FeedbackType;

        /* Stepping or walking pattern generator */
        StepsGenerator 			myWalker;
        /* Desired end-effector in the robot base frame (root link) with world orientation*/
        // Eigen::Vector3d des_ee_pos_b[5]; // [0] lhand, [1] rhand, [2] lfoot, [3] rfoot, [4] CM   // position
        // Eigen::Matrix3d des_ee_rot_b[5]; // [0] lhand, [1] rhand, [2] lfoot, [3] rfoot, [4] CM   // orientation
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

        int number_steps;
        double stride_x;
        Vector7d des_pose_lh;
        Vector7d des_pose_rh;
        WholeBodyTaskSpaceStates                wbEEstates;
        WholeBodyTaskSpaceTrajectories           wbEEtraj;

        // Methods
        StepperThread(std::string robot_name, int period_, WholeBodyTaskSpaceStates wbEEstates, WholeBodyTaskSpaceTrajectories wbEEtraj);
        // StepperThread(std::string robot_name, int period_);
        ~StepperThread();

        virtual bool threadInit();
		virtual void threadRelease();
		virtual void run();

        void releaseStepper();
        bool set_handsTasks_in_base(Vector7d lh_pose, Vector7d rh_pose, Vector7d pelvis_pose);
        void get_step_transforms(int number_steps, double stride_x, Vector7d des_pose_lh, Vector7d des_pose_rh, WholeBodyTaskSpaceStates wb_ts);


};

#endif // StepperThread_H