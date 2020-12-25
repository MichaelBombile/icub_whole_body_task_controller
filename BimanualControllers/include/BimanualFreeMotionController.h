
/** Class Reference

*/


#pragma once

#ifndef BimanualFreeMotionController_H
#define BimanualFreeMotionController_H

#include <string>
#include <iostream>
#include <cmath>
#include <stdexcept>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Geometry>

#include "wbhpidcUtilityFunctions.hpp"
#include "Object_to_Grasp.hpp"
#include "ioStateManager.h"

using namespace std;
using namespace Eigen;


typedef Eigen::Matrix<double, 7, 1>  Vector7d;
typedef Eigen::Matrix<double, 6, 1>  Vector6d;
typedef Eigen::Matrix<double, 6, 6>  Matrix6d;
typedef Eigen::Matrix<double, 4, 4>  Matrix4d;
typedef Eigen::Matrix<double, 6, 12> Matrix6_12d;


class BimanualFreeMotionController
{
	
	public:

		// parameteres for the reach to grasp task
		// =======================================

		double period; 

		Matrix4d w_H_absHands;
		Matrix4d w_H_v[N_eef];
		Matrix4d w_H_vstar[N_eef];
		Matrix4d w_H_hand[N_eef];
		Matrix4d w_H_cp[N_eef];

		Matrix4d absHands_H_v[N_eef]; 			// grasp point of scaled virtual object wrt. the absolute frame
		Matrix4d absHands_H_vstar_ini[N_eef];
		Matrix4d absHands_H_vstar_fin[N_eef];
		Matrix4d w_H_vstar_t[N_eef];

		Matrix4d absHands_H_vstar_ini_R[N_eef];

		// to center the grasp point
		Matrix4d Obj_H_absGpoints;
		Matrix4d w_H_absGpoints;
		Matrix4d absGpoints_H_cp[N_eef];

		// for the retract motion
		Matrix4d w_H_absHands_des;				// desired absolute hands pose
		Matrix4d w_H_hand_des[N_eef];			// desired hands poses (after release and retract)
		Matrix4d w_H_vstar_des[N_eef];		// desired virtual object poses (after release and retract)

		// error 6D
		Vector6d error_approach[N_eef];
		Vector6d error_aperture[N_eef];
		Vector6d error_synchro[N_eef];
		//
		Vector6d error_approach_ini[N_eef];

		// Orientation Jacobian
		Matrix3d L_Mu_Theta_approach[N_eef];
		Matrix3d L_Mu_Theta_aperture[N_eef];
		Matrix3d L_Mu_Theta_synchro[N_eef];

		// Interaction matrices
		Matrix6d L_eta_approach_in_w[N_eef];
		Matrix6d L_eta_aperture_in_w[N_eef];
		Matrix6d L_eta_synchro_in_w[N_eef];

		// Velocity vectors 
		Vector6d w_velocity_approach[N_eef];
		Vector6d w_velocity_aperture[N_eef];
		Vector6d w_velocity_synchro[N_eef];

		Vector6d w_velocity_ro_gpoint[N_eef];
		Vector6d w_velocity_uvo_gpoint[N_eef]; 
		Vector6d w_velocity_vo_gpoint[N_eef];
		Vector6d w_velocity_eef[N_eef];

		// acceleration vectors 
		Vector6d w_acceleration_approach[N_eef];
		Vector6d w_acceleration_aperture[N_eef];
		Vector6d w_acceleration_synchro[N_eef];

		Vector6d w_acceleration_ro_gpoint[N_eef];
		Vector6d w_acceleration_uvo_gpoint[N_eef]; 
		Vector6d w_acceleration_vo_gpoint[N_eef];
		Vector6d w_acceleration_eef[N_eef];

		// Kinematic transformations
        KineTransformations Transforms;

        // 
        // Filter hand motion
		firstOrderFilter  Filter_approach[N_eef];  // [0] left [1] right
		firstOrderFilter  Filter_aperture[N_eef]; 
		firstOrderFilter  Filter_synchro[N_eef];

		firstOrderFilter  Filter_velocity[N_eef];

		//

		firstOrderFilter Filter_absolute;
		firstOrderFilter Filter_relative;
		Vector6d w_velocity_abs;
		Vector6d w_velocity_rel;
		Vector6d w_acceleration_abs;
		Vector6d w_acceleration_rel;
		Vector6d w_velocity_abs_0;
		Vector6d w_velocity_rel_0;

		Vector6d error_abs;
		Vector6d error_rel;

		//
		double gamma_reachable_p;
		double gamma_reachable_o;

		// Controller parameters
		// =====================
		ControllerParameters&  	ctrl_param;
		
		BimanualFreeMotionController(ControllerParameters&  	ctrl_param_);

		~BimanualFreeMotionController();

		void InitializeReach2GraspTask(   double period_, Object_to_Grasp &GrspObj, ioStateManager &ioSM_);

		bool compute_Reach2GraspMotion(	Object_to_Grasp &GrspObj, ioStateManager  &ioSM_);

		bool compute_Reach2GraspMotion2(Object_to_Grasp &GrspObj, ioStateManager  &ioSM_); 

		bool Release_and_Retract(Object_to_Grasp &GrspObj, ioStateManager  &ioSM_);

		bool Release_and_Retract2(	Object_to_Grasp &GrspObj, ioStateManager  &ioSM_);

		//
		// void InitializeReach2GraspTask(   double period_,
		// 								Vector7d Obj_Pose_cp_[],
		// 								Vector7d w_Pose_Obj_,
		// 								Vector7d w_Pose_lhand,
		// 								Vector7d w_Pose_rhand);

		// bool compute_Reach2GraspMotion(	Vector7d 				Obj_Pose_cp[],
		// 								TaskSpaceTrajectories  	StatesObject_,
		// 								Vector7d 				w_Pose_lhand,
		// 								Vector7d 				w_Pose_rhand);

		// bool compute_Reach2GraspMotion2(Vector7d 				Obj_Pose_cp[],
		// 								TaskSpaceTrajectories  	StatesObject_,
		// 								Vector7d 				w_Pose_lhand,
		// 								Vector7d 				w_Pose_rhand);

		// bool Release_and_Retract(Vector7d 				Obj_Pose_cp[],
		// 						 TaskSpaceTrajectories  StatesObject_,
		// 						 Vector7d 				w_Pose_lhand,
		// 						 Vector7d 				w_Pose_rhand);

		// bool Release_and_Retract2(	Vector7d 				Obj_Pose_cp[],
		// 							TaskSpaceTrajectories  	StatesObject_,
		// 							Vector7d 				w_Pose_lhand,
		// 							Vector7d 				w_Pose_rhand);

		// -----------------------------------------------------------------------------------------------------

		void get_absolute_hands_frame(	Matrix4d w_H_Obj_, 
										Matrix4d w_H_h_[], 
										Matrix4d &w_H_absE_);

		bool get_bimanual_coordination_transforms(	Matrix4d W_H_l, Matrix4d W_H_r, 
													Matrix4d &W_H_a_, Matrix4d &l_H_r_);

		void get_left_and_right_Twist(Vector6d vel_a, Vector6d vel_r, Vector6d &left_t, Vector6d &right_t);

		// bool getBimanualConstrainedMotion(  Matrix4d    cp_desH_Obj_[],
		// 									Vector7d    w_Pose_obj_ref,
  //                                           Vector7d    w_Pose_lhand,
  //                                           Vector7d    w_Pose_rhand);
		bool getBimanualConstrainedMotion(  Matrix4d    cp_desH_Obj_[],
											Vector7d    w_Pose_obj_ref,
                                            ioStateManager  &ioSM_);

		// -----------------------------------------------------------------------------------------------------
		// -----------------------------------------------------------------------------------------------------
		
};

#endif // BimanualFreeMotionController_H

// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////