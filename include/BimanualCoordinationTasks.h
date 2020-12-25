/** Class Reference

*/

#pragma once

#ifndef BimanualCoordinationTasks_H
#define BimanualCoordinationTasks_H

#include <string>
#include <iostream>
#include <cmath>
#include <stdexcept>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Geometry>

#include "wbhpidcUtilityFunctions.hpp"
#include "GraspingConstraints.h"
#include "GraspedObject.hpp"

#define N_eef 2

extern "C" {
#include "bwc_solver.h"
}

extern "C" {
#include "cmo_solver.h"
}

using namespace std;
using namespace Eigen;


typedef Eigen::Matrix<double, 7, 1>  Vector7d;
typedef Eigen::Matrix<double, 6, 1>  Vector6d;
typedef Eigen::Matrix<double, 6, 6>  Matrix6d;
typedef Eigen::Matrix<double, 4, 4>  Matrix4d;
typedef Eigen::Matrix<double, 6, 12> Matrix6_12d;


class BimanualCoordinationTasks
{
	
	public:
		
		BimanualCoordinationTasks();
		~BimanualCoordinationTasks();

		// -----------------------------------------------------------------------------------------------------
		Vector6d get_PoseError_cur2des(	Matrix4d d_H_c);
		Eigen::Matrix3d get_L_Mu_Theta_Matrix(	Matrix3d d_R_c);
		Matrix6d getInteractionMxForAxisAngle(	MatrixXd d_H_c);
		void compute_6Dservoing_variables(	Matrix4d w_H_c,
										  	Matrix4d w_H_d,
										  	Vector2d gain_cd,
										  	Vector6d &error_c_in_d,
										  	Matrix3d &L_Mu_Theta_cd,
										  	Matrix6d &L_eta_cd_in_w);

};

#endif // BimanualCoordinationTasks_H