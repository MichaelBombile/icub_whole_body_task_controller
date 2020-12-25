
/** Class Reference

*/


#pragma once

#ifndef IK_legs_Solver_H
#define IK_legs_Solver_H


#include <iostream>
#include <cmath>
#include <stdexcept>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>

// #include <wbi/wholeBodyInterface.h>
#include "WbRobotModel.h"

#include "wbhpidcUtilityFunctions.hpp"

using namespace std;
using namespace Eigen;

// ====================================================================================================
// INVERSE KINEMACIS
// ====================================================================================================
class IK_legs_Solver
{
	public:


		// other inverse kinematics solver
		int count_max;
		double epsilon;
		double virtual_sampTime;
		double virtual_gain;
		VectorXd minJointLimits;
		VectorXd maxJointLimits;

		IK_legs_Solver(){}
		~IK_legs_Solver(){}

		void InitializeIK(WbRobotModel& robot_model_, double gain_, int count_max_, double epilon_, double step_)
		{
			virtual_sampTime	= step_;
			epsilon 	 		= epilon_;
			count_max  	 		= count_max_;
			virtual_gain 		= gain_;
			minJointLimits.resize(robot_model_.getDoFs());
			maxJointLimits.resize(robot_model_.getDoFs());
			// robot_model_.getJointLimits(minJointLimits.data(), maxJointLimits.data());
			// robot_model_.getJointsLimits();
			minJointLimits = robot_model_.m_min_jtsLimits;
			maxJointLimits = robot_model_.m_max_jtsLimits;

			// std::cout << " Jts limits MIN is :\n" << 180./M_PI * minJointLimits << std::endl;
			// std::cout << " Jts limits MAX is :\n" << 180./M_PI * maxJointLimits << std::endl;
		}

		VectorXd get_legs_IK(WbRobotModel& robot_model_, std::string EE_name, Matrix4d b_H_d_, VectorXd jts_position)
		{	
			//
			KineTransformations Transforms;
			// int LinkID;
			// robot_model_.getFrameList().idToIndex(EE_name, LinkID);

			iDynTree::FrameIndex Link_frameIdx;
			robot_model_.getFrame_Index(EE_name,    Link_frameIdx);

			int nDoF = robot_model_.getDoFs();
			// chain Jacbian
			Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> JacobianLinkID(6,nDoF+6);
			Vector7d w_Pose_LinkID;
			// wbi::Frame world2BaseFrame_;
			Matrix4d virt_world_H_fBase = MatrixXd::Identity(4,4);
			Eigen::VectorXd world2BaseFrameSerialization_(16);
			Matrix4d world_H_LinkID;

			// for(int i=0; i<4; i++){
			//     for(int j=0; j<4; j++){
			//         world2BaseFrameSerialization_(4*i+j) = virt_world_H_fBase(i,j);
			//     }
			// } 
			// wbi::frameFromSerialization(world2BaseFrameSerialization_.data(), world2BaseFrame_);
		 
			VectorXd virt_jts_pos     = jts_position;
			VectorXd virtual_jts_velo = Eigen::VectorXd::Zero(6);
			
			// compute the homogeneous matrix of the desired eef pose wrt the base frame
			Matrix4d b_H_d = b_H_d_; //Transforms.PoseVector2HomogenousMx(desPose);
			//  homogeneous transformation related to the  current pose of the eef wrt the base
			Matrix4d b_H_c;
			// relative pose between the current and desired end effector
			Matrix4d d_H_c;
			//
			Matrix6d eef_Rot6D_b;
					 eef_Rot6D_b.setIdentity();
			//
			int count = 0;
			double error_norm = 1.0;	
			double error_pos_norm = 1.0;
			double error_ori_norm = 1.0;
		    //
		    Vector6d gain_vector;   
		    gain_vector.setOnes();
		   	gain_vector.head(3) *= -virtual_gain;
		   	gain_vector.tail(3) *= -virtual_gain;
			//
			// while((count <= count_max) && (error_norm > epsilon))
			while(    (count <= count_max) && ((error_pos_norm > epsilon) || (error_ori_norm > 10.*epsilon))   )
			{
				// update the chain with the virtual joints positions and get the pose and the jacobian
				// robot_model_.forwardKinematics(virt_jts_pos.data(), world2BaseFrame_, LinkID, w_Pose_LinkID.data());
				// robot_model_.computeJacobian(virt_jts_pos.data(), world2BaseFrame_, LinkID, JacobianLinkID.data());


				// robot_model_.eigRobotState.jointPos		= virt_jts_pos;
				// robot_model_.eigRobotState.jointVel.setZero(); 
				// robot_model_.eigRobotState.world_H_base 	= virt_world_H_fBase;
				// robot_model_.eigRobotState.baseVel.setZero();
				// robot_model_.UpdateModelStates();


				robot_model_.forwardKinematics(virt_jts_pos, virt_world_H_fBase, Link_frameIdx, 	w_Pose_LinkID, 	world_H_LinkID);
				robot_model_.computeJacobian(  virt_jts_pos, virt_world_H_fBase, Link_frameIdx,    JacobianLinkID.data());


				// get the homogeneous transformation related to the  current pose of the eef wrt the base
				b_H_c = Transforms.PoseVector2HomogenousMx(w_Pose_LinkID);
				// get relative pose between the current and desired end effector
				d_H_c = b_H_d.inverse() * b_H_c;
				Matrix4d c_H_d = d_H_c.inverse();
				//Eigen::AngleAxisd orient_error(c_H_d.block<3,3>(0,0));
		        Eigen::AngleAxisd orient_error(d_H_c.block<3,3>(0,0));
				// compute the task error
				Vector6d error_pose = Transforms.get_PoseError_cur2des(d_H_c); // in desired : in current
				// compute the norm of the error  // here psition and orientation (can be separared into two different norms)
				// error_norm = error_pose.norm();
				error_pos_norm = error_pose.head(3).norm();
				error_ori_norm = error_pose.tail(3).norm();

				// cout << " ITERATION NUMBER  IS  : \t" << count << endl;
				// cout << " POSITION ERROR NORM IS  : \t" << error_pose.head(3).norm() << endl;
				// cout << " ORIENTATION ERROR NORM IS  : \t" << error_pose.tail(3).norm() << endl; 
				// compute interaction matrix

				Matrix6d L_eta_chain = Transforms.getInteractionMxForAxisAngle(d_H_c);
		        // Matrix6d L_eta_chain = getInteractionMxForAxisAngle(c_H_d);
				// 6D rotation matrix from base to end effector
				eef_Rot6D_b.block<3,3>(0,0) = b_H_c.block<3,3>(0,0).transpose();
				eef_Rot6D_b.block<3,3>(3,3) = b_H_c.block<3,3>(0,0).transpose();
				// compute the task jacobian 
				// compute the chain jacobian
				Matrix6d chain_jacobian;
				if(EE_name == "l_sole") chain_jacobian = JacobianLinkID.rightCols(12).leftCols(6);
				else if(EE_name == "r_sole") chain_jacobian = JacobianLinkID.rightCols(6);
				// ---
				// compute the joint veleocity
				Eigen::MatrixXd Jacobian_task = L_eta_chain * eef_Rot6D_b * chain_jacobian;
				error_pose = gain_vector.asDiagonal() *error_pose;
				virtual_jts_velo = Jacobian_task.colPivHouseholderQr().solve(error_pose); 

				// compute the joint position variation
				Eigen::VectorXd delta_virtual_jts(virtual_jts_velo.rows());
				delta_virtual_jts = virtual_jts_velo * virtual_sampTime;

				// update the virtual joints positions
				if(EE_name == "l_sole")
				{
					virt_jts_pos.tail(12).head(6) += delta_virtual_jts;
				}
				else if(EE_name == "r_sole")
				{
					virt_jts_pos.tail(6) += delta_virtual_jts;
				}

				for(int i=0; i<robot_model_.getDoFs(); i++)
				{
					// set joint limits hard limits
					if(virt_jts_pos(i) > maxJointLimits(i)){  // upper limit
						virt_jts_pos(i) = maxJointLimits(i) - 0.02;
					} else if(virt_jts_pos(i) < minJointLimits(i)){ // lower limit
						virt_jts_pos(i) = minJointLimits(i) + 0.02;
					} 
				}
				count++;
			}
			//
			// return virt_jts_pos;
			if(EE_name == "l_sole"){
				return virt_jts_pos.tail(12).head(6);
			}
			else {//if(EE_name == "r_sole")
				return virt_jts_pos.tail(6);
			}
		}

};

#endif // IK_legs_Solver_H




