/** Class Reference

*/

#pragma once

#ifndef auxiliary_functions_H
#define auxiliary_functions_H

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

#include "wbhpidcUtilityFunctions.hpp"

using namespace std;
using namespace Eigen;

typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;
typedef Eigen::Matrix<double, 6, 12> Matrix6_12d;



class auxiliary_functions
{

	public:

	Vector6d wrench_lh;
	Vector6d wrench_rh;

	Vector6d offset_wrench_lh;
	Vector6d offset_wrench_rh;	

	// joint variables
	Cvector ref_pos;  
	Cvector ref_vel; 
	Cvector ref_acc;  
	Cvector ref_tau;  
	Cvector sens_pos; 
	Cvector sens_vel; 
	Cvector sens_acc; 
	Cvector sens_tau; 
	Cvector init_pos; 
	Cvector next_pos; 
	Cvector freezeIK; 
	Cvector mode;

	int StanceFoot[2]; 
	bool isExtBase; 

	// robot base in world reference frame 
	// (Origin in gazebo or stance foot)
	Matrix4d T_base2world;

	IKPARAM ikparam;

	// YARP wrapper
	wrapper W;
	// definitions
	Model model;
	// contact definition
	Contact_Manager points;

	auxiliary_functions();
	~auxiliary_functions();

	void init(std::string robot_Name);

	void init_IK();

	void loadData(wrapper &W, Contact_Manager &points, Cvector &pos,  Cvector &vel,  Cvector &acc,  Cvector &tau);

	bool get_hands_wrenches_offsets();

	bool get_estimated_hands_wrenches ();

	bool get_inverse_kinematics(wb_ref_position ref_p_wb,  wb_ref_orientation ref_o_wb);

	
};


#endif // auxiliary_functions
