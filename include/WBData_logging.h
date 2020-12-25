#pragma once


#ifndef WBData_logging_H
#define WBData_logging_H

#include <iostream>
#include <fstream>
#include <cmath>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;
// 
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;
//
//
class WBData_logging
{

    public:

        double SimTime;

        std::string logRobotTasks;
        std::string logTargetObject;
        std::string logRelativeErrors;
        std::string logRelativeVelocities;
        std::string logGlobalVelocities;
        std::string logHomogeneousTransforms;
        std::string logObjectPose;

        std::ofstream Robot_Tasks;
        std::ofstream Target_Object;
        std::ofstream Relative_Errors;
        std::ofstream Relative_velocities;
        std::ofstream Global_velocities;
        std::ofstream Homo_transforms;
        std::ofstream Object_pose;


        WBData_logging();

        ~WBData_logging();

        void InitializeLogger(std::string count);

        void Write_Data(double SamplingTime,
                        int Cycle_counter,
                        int Ne,
                        Vector6d error_approach[],
                        Vector6d error_aperture[],
                        Vector6d error_coordin[],

                        Vector6d w_velocity_approach[],
                        Vector6d w_velocity_aperture[],
                        Vector6d w_velocity_coordin[],

                        Vector6d w_velocity_ro_gpoint[],
                        Vector6d w_velocity_uvo_gpoint[],
                        Vector6d w_velocity_vo_gpoint[], 
                        Vector6d w_velocity_eef[],

                        Matrix4d w_H_Obj,
                        Matrix4d w_H_absE,
                        Matrix4d w_H_vstar[],
                        Matrix4d w_H_v[],
                        Matrix4d w_H_h[],
                        Matrix4d w_H_cp[],

                        // Robot
                        // ======
                        // poses
                        Vector7d w_Pose_lh_,
                        Vector7d w_Pose_rh_,
                        Vector7d w_Pose_lf_,
                        Vector7d w_Pose_rf_,
                        Vector7d w_Pose_Base_,
                        Vector7d w_Pose_CoM_,
                        // wrenches
                        // =============
                        // computed
                        Vector6d gsp_lh_Wrench,
                        Vector6d gsp_rh_Wrench,
                        Vector6d wbd_lf_Wrench,
                        Vector6d wbd_rf_Wrench,
                        // compensated
                        Eigen::Vector3d cmp_lh_Moment,
                        Eigen::Vector3d cmp_rh_Moment,
                        // Applied
                        Vector6d appl_lh_Wrench,
                        Vector6d appl_rh_Wrench,
                        // Estimated
                        Vector6d Est_lh_Wrench,
                        Vector6d Est_rh_Wrench,
                        Vector6d Est_lf_Wrench,
                        Vector6d Est_rf_Wrench,

                        // Target Object
                        // =============
                        Vector7d object_pose_,
                        Vector7d d_obj_pose_,
                        Vector6d d_obj_velo
                        );

        void Close_files();
};

 
#endif // WBData_logging_H
