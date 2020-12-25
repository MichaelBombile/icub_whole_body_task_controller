#pragma once

#ifndef wbhpidcUtilityFunctions_H
#define wbhpidcUtilityFunctions_H

#include <stdio.h>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <cstdlib>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/QR>
#include <Eigen/LU> 
#include <Eigen/Geometry>

#include <qpOASES.hpp>
USING_NAMESPACE_QPOASES

#define N_eef 2             // number of end-effectors in manipulation task (2)

using namespace std;
using namespace Eigen;

using namespace yarp::sig;
using namespace yarp::math;


// Eigen::Matrix3d AxisAngle2RotationMx(Eigen::VectorXd Pose_V);
// Eigen::Vector4d rotationMx2axisangle(Eigen::Matrix3d Rot_mx);
// Eigen::Matrix3d ComputeSkewSymmetricMatrix(Eigen::Vector3d positionVect);
// Eigen::MatrixXd ComputeTwistMatrix(Eigen::MatrixXd o_H_ee);
// Eigen::MatrixXd PoseVector2HomogenousMx(Eigen::VectorXd Pose_vector);
// bool Get6DTwistTransform(Eigen::MatrixXd destination_R_origin, Eigen::VectorXd destination_P_origin, Eigen::MatrixXd &OutputTwistMatrix);
// bool Get6DWrenchTransform(Eigen::MatrixXd destination_R_origin, Eigen::VectorXd destination_P_origin, Eigen::MatrixXd &OutputWrenchMatrix);

typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXdRowMaj;

typedef struct  Jac_eef_info
                {
                    int p[2];
                    int l[2];

                } Jac_endeff_info;

struct wb_ref_position
{
    Eigen::Matrix<double, 3, 1> lh;
    Eigen::Matrix<double, 3, 1> rh;
    Eigen::Matrix<double, 3, 1> lf;
    Eigen::Matrix<double, 3, 1> rf;
    Eigen::Matrix<double, 3, 1> cm;
};

struct wb_ref_orientation
{
    Eigen::Matrix<double, 4, 1> lh;
    Eigen::Matrix<double, 4, 1> rh;
    Eigen::Matrix<double, 4, 1> lf;
    Eigen::Matrix<double, 4, 1> rf;
    Eigen::Matrix<double, 4, 1> cm;
};

struct WholeBodyTaskSpaceAcceleration
{
    Vector6d lhand;
    Vector6d rhand;
    Vector6d lfoot;
    Vector6d rfoot;
    Vector6d Chest;
    Vector6d CoM;
    Vector6d Pelvis;
    Vector6d CMdot;
    VectorXd posture;

    void initialize(int n_wb_dofs)
    {
        posture.resize(n_wb_dofs);
        posture.setZero();
        lhand.setZero();
        rhand.setZero();
        lfoot.setZero();
        rfoot.setZero();
        Chest.setZero();
        CoM.setZero();
        Pelvis.setZero();
        CMdot.setZero();
        
    }
}; 

struct TaskSpaceForces
{
    Vector6d lhand;
    Vector6d rhand;
    Vector6d lfoot;
    Vector6d rfoot;
    Vector6d Chest;
    Vector6d CoM;
    Vector6d Pelvis;

    void setZero()
    {
        lhand.setZero();
        rhand.setZero();
        lfoot.setZero();
        rfoot.setZero();
        Chest.setZero();
        CoM.setZero();
        Pelvis.setZero();
    }
}; 



struct JointspaceStates
{
    VectorXd position;
    VectorXd velocity;
    VectorXd acceleration;
};

struct ContactsStates
{
    double Pelvis;
    double lhand;
    double rhand;
    double lfoot;
    double rfoot;

    void update(double pl, double lh, double rh, double lf, double rf)
    {
        Pelvis  = pl;
        lhand   = lh; 
        rhand   = rh; 
        lfoot   = lf; 
        rfoot   = rf; 
    }
};

//
struct TaskSpaceGains 
{
    Matrix6d M;     // virtual inertia
    Matrix6d D;     // virtual damping
    Matrix6d K;     // virtual stiffness    

    Matrix6d invM;  // inverse of M

    void setIdentity()
    {
        M.setIdentity();
        D.setIdentity();
        K.setIdentity();
        invM.setIdentity();
    }

};


struct WholeBodyTaskSpaceGains
{
    TaskSpaceGains lhand;
    TaskSpaceGains rhand;
    TaskSpaceGains lfoot;
    TaskSpaceGains rfoot;
    TaskSpaceGains Chest;
    TaskSpaceGains CoM;
    TaskSpaceGains Pelvis;

    void setIdentity()
    {
        lhand.setIdentity();
        rhand.setIdentity();
        lfoot.setIdentity();
        rfoot.setIdentity();
        Chest.setIdentity();
        CoM.setIdentity();
        Pelvis.setIdentity();
    }
};

//
struct PostureGains
{
    Eigen::VectorXd K;
    Eigen::VectorXd D;

    void resize(int n_dofs){
        K.resize(n_dofs);
        D.resize(n_dofs);
    }
    void setZero(){
        K.setZero();
        D.setZero();
    }
    void setOnes(){
        K.setOnes();
        D.setOnes();
    }
};

//
struct TaskSpaceStates
{
    Vector7d Pose;
    Vector6d Velo;

    void setZero()
    {
        Pose.setZero();
        Velo.setZero();
    }
};

struct WholeBodyTaskSpaceStates
{
    TaskSpaceStates lhand;
    TaskSpaceStates rhand;
    TaskSpaceStates lfoot;
    TaskSpaceStates rfoot;
    TaskSpaceStates Chest;
    TaskSpaceStates CoM;
    TaskSpaceStates Pelvis;

    void setZero()
    {
        lhand.setZero();
        rhand.setZero();
        lfoot.setZero();
        rfoot.setZero();
        Chest.setZero();
        CoM.setZero();
        Pelvis.setZero();
    }
};

struct TaskSpaceTrajectories
{
    Vector7d pose;
    Vector6d velocity;
    Vector6d acceleration;

    void setZero()
    {
        pose.setZero();
        velocity.setZero();
        acceleration.setZero();
    }
};
//
struct WholeBodyTaskSpaceTrajectories
{
    
    TaskSpaceTrajectories lhand;
    TaskSpaceTrajectories rhand;
    TaskSpaceTrajectories lfoot;
    TaskSpaceTrajectories rfoot;
    TaskSpaceTrajectories Chest;
    TaskSpaceTrajectories CoM;
    TaskSpaceTrajectories Pelvis;

    void setZero()
    {
        lhand.setZero();
        rhand.setZero();
        lfoot.setZero();
        rfoot.setZero();
        Chest.setZero();
        CoM.setZero();
        Pelvis.setZero();
    }
    void copy_pose(WholeBodyTaskSpaceStates ss)
    {
        lhand.pose   = ss.lhand.Pose;
        rhand.pose   = ss.rhand.Pose;
        lfoot.pose   = ss.lfoot.Pose;
        rfoot.pose   = ss.rfoot.Pose;
        Chest.pose   = ss.Chest.Pose;
        CoM.pose     = ss.CoM.Pose;
        Pelvis.pose  = ss.Pelvis.Pose;
    }
};

// 
struct torqueControlledJoints
{
    std::string torso[3];     
    std::string neck[3];      
    std::string left_arm[7];  
    std::string right_arm[7]; 
    std::string left_leg[6];  
    std::string right_leg[6]; 

    void assign(std::string torso_[], std::string neck_[], std::string l_arm_[], 
                std::string r_arm_[], std::string l_leg_[],  std::string r_leg_[])
    {
        std::copy(torso_, torso_ + 3, torso);
        std::copy( neck_,  neck_ + 3, neck);
        std::copy(l_arm_, l_arm_ + 7, left_arm);
        std::copy(r_arm_, r_arm_ + 7, right_arm);
        std::copy(l_leg_, l_leg_ + 6, left_leg);
        std::copy(r_leg_, r_leg_ + 6, right_leg);
    }
};


struct Joints_Measurements 
{
    Eigen::VectorXd     position;           // Position
    Eigen::VectorXd     velocity;           // Velocity
    Eigen::VectorXd     acceleration;       // Acceleration
    Eigen::VectorXd     torque;             // Torque

    int nb_joints;

    void resize(int nDofs)
    {
        nb_joints = nDofs; 

        position.resize(nDofs);         position.setZero();
        velocity.resize(nDofs);         velocity.setZero();
        acceleration.resize(nDofs);     acceleration.setZero();
        torque.resize(nDofs);           torque.setZero();
    }

};

struct Joints_PIDs 
{
    Eigen::VectorXd     kp;       // proportional
    Eigen::VectorXd     kd;       // derivative
    Eigen::VectorXd     ki;       // intergra
    int nb_joints;

    void resize(int nDofs)
    {
        nb_joints = nDofs; 
        kp = Eigen::VectorXd::Zero(nDofs); 
        kd = Eigen::VectorXd::Zero(nDofs); 
        ki = Eigen::VectorXd::Zero(nDofs); 
    }

};

struct ObjectDimension
{
    double length;
    double width;
    double height;
    double radius;
};

//
struct BimanualGains
{
    Vector6d    synchro;
    Vector6d    approach;
    Vector6d    aperture;
    Vector6d    absolute;
    Vector6d    relative;
};
//
struct BimanualTaskSpaceTraj
{
    Eigen::Matrix<double, 14, 1> pose;
    Eigen::Matrix<double, 12, 1> velo;
    Eigen::Matrix<double, 12, 1> accel;
};

//
struct aStepLooger 
{
    std::ofstream Out_joints_pos;
    std::ofstream Out_joints_vel;
    std::ofstream Out_stability_var;
    std::ofstream Out_PointsCHull;
    std::ofstream Out_NormalCHull;
    std::ofstream Out_DistanceCHull;

    void open(std::string path, std::string type, std::string nData)
    {
        string path_log_joints_pos      = path + "log_joints_pos"   + "_" + type + "_" + nData + ".txt";    
        string path_log_joints_vel      = path + "log_joints_vel"   + "_" + type + "_" + nData + ".txt";
        string path_log_stability_var   = path + "log_stability_var"+ "_" + type + "_" + nData + ".txt";
        string path_log_PointsCHull     = path + "log_PointsCHull"  + "_" + type + "_" + nData + ".txt";
        string path_log_NormalCHull     = path + "log_NormalCHull"  + "_" + type + "_" + nData + ".txt";
        string path_log_DistanceCHull   = path + "log_DistanceCHull"+ "_" + type + "_" + nData + ".txt";
        //
        Out_joints_pos.open(path_log_joints_pos.c_str());
        Out_joints_vel.open(path_log_joints_vel.c_str());
        Out_stability_var.open(path_log_stability_var.c_str());
        Out_PointsCHull.open(path_log_PointsCHull.c_str());
        Out_NormalCHull.open(path_log_NormalCHull.c_str());
        Out_DistanceCHull.open(path_log_DistanceCHull.c_str());
    }

    void close()
    {
        Out_joints_pos.close();
        Out_joints_vel.close();
        Out_stability_var.close();
        Out_PointsCHull.close();
        Out_NormalCHull.close();
        Out_DistanceCHull.close();
    }
};
//

class KineTransformations
{
    public:

        KineTransformations() {};
        ~KineTransformations() {};

        Eigen::Matrix3d AxisAngle2RotationMx(Eigen::VectorXd AxisAngle)
        {
            //
            Eigen::Matrix3d RotationMx(3,3);
            //
            RotationMx(0,0) = cos(AxisAngle(3)) + AxisAngle(0)*AxisAngle(0)*(1.0  - cos(AxisAngle(3)));
            RotationMx(1,0) = AxisAngle(1)*AxisAngle(0)*(1.0 - cos(AxisAngle(3))) + AxisAngle(2)*sin(AxisAngle(3));
            RotationMx(2,0) = AxisAngle(2)*AxisAngle(0)*(1.0 - cos(AxisAngle(3))) - AxisAngle(1)*sin(AxisAngle(3));

            RotationMx(0,1) = AxisAngle(0)*AxisAngle(1)*(1.0 - cos(AxisAngle(3))) - AxisAngle(2)*sin(AxisAngle(3));
            RotationMx(1,1) = cos(AxisAngle(3)) + AxisAngle(1)*AxisAngle(1)*(1.0  - cos(AxisAngle(3)));
            RotationMx(2,1) = AxisAngle(2)*AxisAngle(1)*(1.0 - cos(AxisAngle(3))) + AxisAngle(0)*sin(AxisAngle(3));

            RotationMx(0,2) = AxisAngle(0)*AxisAngle(2)*(1.0 - cos(AxisAngle(3))) + AxisAngle(1)*sin(AxisAngle(3));
            RotationMx(1,2) = AxisAngle(1)*AxisAngle(2)*(1.0 - cos(AxisAngle(3))) - AxisAngle(0)*sin(AxisAngle(3));
            RotationMx(2,2) = cos(AxisAngle(3)) + AxisAngle(2)*AxisAngle(2)*(1.0  - cos(AxisAngle(3)));

            return RotationMx;
        }


        Eigen::Vector4d rotationMx2axisangle(Eigen::Matrix3d Rot_mx)
        {
            //
            Eigen::Vector4d axis_angle;
            double angle;
            //
            // computation of the angle
            angle = acos((Rot_mx.trace()-1.)/2.);

            // computation of axis
            //
            axis_angle(0) = Rot_mx(2,1)-Rot_mx(1,2);
            axis_angle(1) = Rot_mx(0,2)-Rot_mx(2,0);
            axis_angle(2) = Rot_mx(1,0)-Rot_mx(0,1);

            axis_angle = (1./(2.*sin(angle+0.000001))) * axis_angle;
            //
            axis_angle(3) = angle;

            return axis_angle;

        }


        Eigen::Matrix3d ComputeSkewSymmetricMatrix(Eigen::Vector3d positionVect)
        {
            // 
            Eigen::Matrix3d Skew_matrix;
            Skew_matrix.setZero();
            // 
            Skew_matrix(0,1) = -positionVect(2);
            Skew_matrix(0,2) =  positionVect(1);
            Skew_matrix(1,0) =  positionVect(2);
            Skew_matrix(1,2) = -positionVect(0);
            Skew_matrix(2,0) = -positionVect(1);
            Skew_matrix(2,1) =  positionVect(0);

            return Skew_matrix;
        }


        /** 
         * Compute the twsit transformation matrix associated with an homogenous transformation
         * expressed in the reference frame of the Homogenous matrix
         *
         */
        Eigen::MatrixXd ComputeTwistMatrix(Eigen::MatrixXd o_H_ee)
        {
            // 
            Eigen::MatrixXd TwistMx(6,6);
            TwistMx.setIdentity();
            // 
            TwistMx.block<3,3>(3,0) = ComputeSkewSymmetricMatrix(o_H_ee.block<3,1>(0,3));

            return TwistMx;
        }


        /** 
         * Compute the homogenous transformation related to a pose vector 
         * where the orientation is represented with axis angle representation
         */
        Eigen::MatrixXd PoseVector2HomogenousMx(Eigen::VectorXd Pose_vector)
        {
            // 
            // Eigen::MatrixXd H(4,4);
            // H.setIdentity();
            // // 
            // Eigen::Vector3d axis = Pose_vector.segment(3,3);
            // axis *=1./axis.norm();
            // Eigen::AngleAxisd aa_rot(Pose_vector(6), axis);
            // //
            // H.block<3,1>(0,3) = Pose_vector.head(3);
            // H.block<3,3>(0,0) = aa_rot.toRotationMatrix();

            Matrix4d H = MatrixXd::Identity(4,4);
            // 
            Eigen::Vector3d axis = Pose_vector.segment(3,3);
            axis *=1./axis.norm();
            Eigen::AngleAxisd aa_rot(Pose_vector(6), axis);
            //
            H.block<3,1>(0,3) = Pose_vector.head(3);
            H.block<3,3>(0,0) = aa_rot.toRotationMatrix();

            return H;
        }

        Vector7d HomogenousMx2PoseVector(Matrix4d d_H_o)
        { 
            // 
            Vector7d d_pose_o;
            d_pose_o.head<3>() = d_H_o.block<3,1>(0,3);

            Eigen::AngleAxisd d_orientation_o; 
            d_orientation_o = Eigen::AngleAxisd(d_H_o.block<3,3>(0,0));

            d_pose_o.segment<3>(3) = d_orientation_o.axis();
            d_pose_o.tail<1>() << d_orientation_o.angle();

            return d_pose_o;
        }

        /** 
         * Compute the twist transformation from a rotation matrix and a position vector associated to
         * the origin and destination frame
         */
        bool Get6DTwistTransform(Eigen::MatrixXd destination_R_origin, Eigen::VectorXd destination_P_origin, Eigen::MatrixXd &OutputTwistMatrix)
        {
            // 6 x e transformation for 6D twist Vector [vx;vy;vz; wx;wy;wz]
            Eigen::MatrixXd d_X_o(6,6);
            d_X_o.setIdentity();

            d_X_o.block<3,3>(0,0) = destination_R_origin;
            d_X_o.block<3,3>(3,3) = destination_R_origin;
            d_X_o.block<3,3>(0,3) = ComputeSkewSymmetricMatrix(destination_P_origin) * destination_R_origin;

            // output twist transformaion matrix
            OutputTwistMatrix = d_X_o;

            return true;
        }

        /** 
         * Compute the wrench transformation from a rotation matrix and a position vector associated to
         * the origin and destination frame
         */
        bool Get6DWrenchTransform(Eigen::MatrixXd destination_R_origin, Eigen::VectorXd destination_P_origin, Eigen::MatrixXd &OutputWrenchMatrix)
        {
            // 6 x e transformation for 6D Wrench Vector [fx;fy;fz; mx;my;mz]
            Eigen::MatrixXd d_Xstar_o(6,6);
            d_Xstar_o.setIdentity();

            d_Xstar_o.block<3,3>(0,0) = destination_R_origin;
            d_Xstar_o.block<3,3>(3,3) = destination_R_origin;
            d_Xstar_o.block<3,3>(3,0) = ComputeSkewSymmetricMatrix(destination_P_origin) * destination_R_origin;

            // outout wrench transformation matrix
            OutputWrenchMatrix = d_Xstar_o;

            return true;
        }


        Eigen::VectorXd getPoseErrorWithAxisAngle(Eigen::VectorXd current_pose, Eigen::VectorXd desired_pose)  
        {
            
            // extraction of the translation
            Eigen::MatrixXd w_H_c(4,4),
                            w_H_d(4,4),
                            d_H_c(4,4);

            w_H_c.setIdentity();
            w_H_d.setIdentity();
            d_H_c.setIdentity();

            // current end effector pose expressed in the world frame
            w_H_c.block<3,1>(0,3) = current_pose.segment(0,3);
            w_H_c.block<3,3>(0,0) = AxisAngle2RotationMx(current_pose.segment(3,4));

            // desired end effector pose expressed in the world frame
            w_H_d.block<3,1>(0,3) = desired_pose.segment(0,3);
            w_H_d.block<3,3>(0,0) = AxisAngle2RotationMx(desired_pose.segment(3,4));

            // computation of the transformation from current EE frame to the desired frame
            d_H_c = w_H_d.inverse() * w_H_c;

            // computation of the pose error
            Eigen::VectorXd pose_error(6), axis_angle(4);
            axis_angle = rotationMx2axisangle(d_H_c.block<3,3>(0,0));

            // translation
            pose_error.segment(0, 3) = w_H_c.inverse().block<3,1>(0,3) - w_H_d.inverse().block<3,1>(0,3);  // from position
            pose_error.segment(3, 3) = axis_angle.segment<3>(0) * axis_angle(3);

            // Twist transformation form world frame to End effector Frame
            Eigen::MatrixXd world_Twist_cee(6,6);
            world_Twist_cee.setIdentity();
            // world_Twist_cee.block<3,3>(0,0) = w_H_c.block<3,3>(0,0);
            // world_Twist_cee.block<3,3>(3,3) = w_H_c.block<3,3>(0,0);

                     
            return world_Twist_cee * pose_error;
        }

        Eigen::VectorXd computePoseErrorNormalizedAxisAngle(Eigen::VectorXd current_pose, Eigen::VectorXd desired_pose)  
        {            
            VectorXd pose_error(6);
            pose_error.setZero();            
            // orientation error
            Vector3d cur_orient_axis = current_pose.segment<3>(3)/current_pose.segment<3>(3).norm();
            Vector3d des_orient_axis = desired_pose.segment<3>(3)/desired_pose.segment<3>(3).norm();

            Matrix3d cur_Rotation_in_des = Eigen::AngleAxisd(current_pose(6), cur_orient_axis).toRotationMatrix() * Eigen::AngleAxisd(desired_pose(6), des_orient_axis).toRotationMatrix().transpose();
            Eigen::AngleAxisd cur_Orientation_in_des(cur_Rotation_in_des);

            // pose error
            pose_error.head(3) = current_pose.head(3) - desired_pose.head(3);
            pose_error.tail(3) = cur_Orientation_in_des.axis() * cur_Orientation_in_des.angle();
                             
            return pose_error;
        }

        Eigen::VectorXd computePoseErrorNormalizedAxisAngle(Eigen::VectorXd current_pose, Eigen::VectorXd desired_pose, Matrix6d &w_R_c)  
        {
            
            VectorXd pose_error(6);
            pose_error.setZero();

            
            
            // orientation error
            Vector3d cur_orient_axis = current_pose.segment<3>(3)/current_pose.segment<3>(3).norm();
            Vector3d des_orient_axis = desired_pose.segment<3>(3)/desired_pose.segment<3>(3).norm();

            Matrix3d cur_Rotation_in_des = Eigen::AngleAxisd(current_pose(6), cur_orient_axis).toRotationMatrix() * Eigen::AngleAxisd(desired_pose(6), des_orient_axis).toRotationMatrix().transpose();
            Eigen::AngleAxisd cur_Orientation_in_des(cur_Rotation_in_des);

            // transformation of the error from current to world frame
            w_R_c.setIdentity();
            w_R_c.block<3,3>(3,3) = Eigen::AngleAxisd(current_pose(6), cur_orient_axis).toRotationMatrix();

            // pose error
            pose_error.head(3) = current_pose.head(3) - desired_pose.head(3);
            pose_error.tail(3) = cur_Orientation_in_des.axis() * cur_Orientation_in_des.angle();
                             
            return pose_error;
        }

        //
        void UpdatePose_From_VelocityTwist(double dt,
                                            Vector6d in_veloTwist,
                                            Matrix4d &Hmg_Trsf)
        {
            //
            Eigen::Vector3d pos = Hmg_Trsf.block<3,1>(0,3);
            Eigen::Matrix3d rot = Hmg_Trsf.block<3,3>(0,0);

            // update position
            pos += dt * in_veloTwist.head<3>();

            // update orientation
            Eigen::Quaterniond q(rot);

            Eigen::MatrixXd Trf_quat(4,3);
            Trf_quat << -q.x(), -q.y(), -q.z(),
                         q.w(),  q.z(), -q.y(),
                        -q.z(),  q.w(),  q.x(),
                         q.y(), -q.x(),  q.w(); 
            // update the quaternion
            Eigen::VectorXd qcoeff(4);

            qcoeff << q.w(), q.x(), q.y(), q.z();
            qcoeff +=  dt* 0.5 *Trf_quat * in_veloTwist.tail<3>();

            // normalizing the quaternion
            qcoeff = qcoeff/ qcoeff.norm();
            //
            Eigen::Quaterniond q_new(qcoeff(0), qcoeff(1), qcoeff(2), qcoeff(3)); // w, x, y, z
            //
            Hmg_Trsf.setZero();
            //
            Hmg_Trsf.block<3,3>(0,0) = rot; //q_new.toRotationMatrix();
            Hmg_Trsf.block<3,1>(0,3) = pos;
            Hmg_Trsf(3,3) = 1.0;
        }
        //
        // compute the kinematic servoing variables for position and orientation with axis angle
        void compute_6Dservoing_variables(  Matrix4d w_H_c,
                                            Matrix4d w_H_d,
                                            Vector6d &error_c_in_d,
                                            Matrix3d &L_Mu_Theta_cd,
                                            Matrix6d &L_eta_cd_in_w)
        {
            // transformation from current to desired d_H_d
            Matrix4d d_H_c = w_H_d.inverse() * w_H_c;
            //
            error_c_in_d = get_PoseError_cur2des(d_H_c);

            // 3D Orientation Jacobian 
            L_Mu_Theta_cd = get_L_Mu_Theta_Matrix(d_H_c.block<3,3>(0,0));

            // 6D Interaction matrix of the task
            L_eta_cd_in_w.setZero();
            L_eta_cd_in_w.block<3,3>(0,0) = w_H_d.block<3,3>(0,0).transpose();
            L_eta_cd_in_w.block<3,3>(3,3) = L_Mu_Theta_cd * w_H_c.block<3,3>(0,0).transpose();
        }

        // Vector of feature error
        Vector6d get_PoseError_cur2des(Matrix4d d_H_c)
        {   
            // Pass 
            Eigen::VectorXd d_eta_c(6);
            d_eta_c.segment(0,3) << d_H_c(0,3), d_H_c(1,3), d_H_c(2,3);
            // extracrion of the rotation
            Eigen::AngleAxisd d_AxisAngle_c(d_H_c.block<3,3>(0,0));
            Eigen::Vector3d d_Axis_c = d_AxisAngle_c.axis();
            d_eta_c(3) = d_Axis_c(0) * d_AxisAngle_c.angle();
            d_eta_c(4) = d_Axis_c(1) * d_AxisAngle_c.angle();
            d_eta_c(5) = d_Axis_c(2) * d_AxisAngle_c.angle();

            return d_eta_c;
        }

        Eigen::Matrix3d get_L_Mu_Theta_Matrix(Matrix3d d_R_c)
        {
            // extracrion of the rotation
            Eigen::AngleAxisd d_AxisAngle_c(d_R_c);
            // function sinc(theta) and sinc(theta/2)
            double sinc_theta, sinc_theta_2;
            sinc_theta   = sin(d_AxisAngle_c.angle() + 1e-6)/(d_AxisAngle_c.angle() + 1e-6);
            sinc_theta_2 = sin((d_AxisAngle_c.angle() + 1e-6)/2.)/((d_AxisAngle_c.angle() + 1e-6)/2.);
            //
            Eigen::Vector3d d_Axis_c = d_AxisAngle_c.axis();
            Eigen::Matrix3d Skew_Mu;    Skew_Mu.setZero(3,3);
            //
            Skew_Mu <<           0.0,   -d_Axis_c(2),    d_Axis_c(1),
                         d_Axis_c(2),            0.0,   -d_Axis_c(0),
                        -d_Axis_c(1),    d_Axis_c(0),            0.0;

            // Jacobian of the rotation
            Eigen::Matrix3d L_Mu_Theta;
            L_Mu_Theta.setIdentity(3,3);
            L_Mu_Theta = L_Mu_Theta - (d_AxisAngle_c.angle()/2.)* Skew_Mu + (1.-(sinc_theta/pow(sinc_theta_2, 2.))) * Skew_Mu * Skew_Mu;

            return L_Mu_Theta;
        }

        /* This function computes the Jacobian associated with a rigid transformation
         * represented as homogeneous transformation matrix from the current frame to
         * the desired frame (d_H_c) and where the orientation is represented with an Axis/Angle
         */
        Matrix6d getInteractionMxForAxisAngle(Matrix4d d_H_c)
        {  
            // Building the overall jacobian
            Matrix6d InteractionMx_Mu_Theta = Eigen::MatrixXd::Identity(6,6);
            InteractionMx_Mu_Theta.block<3,3>(0,0) = d_H_c.block<3,3>(0,0);                                 // Map for position error
            InteractionMx_Mu_Theta.block<3,3>(3,3) = this->get_L_Mu_Theta_Matrix(d_H_c.block<3,3>(0,0));    // Map for orientation error in axis/angle

            return InteractionMx_Mu_Theta;
        }

        //
        Eigen::Matrix3d getCombinedRotationMatrix(double Weight, Eigen::Matrix3d w_R_c, Eigen::Matrix3d w_R_d)
        {
            // 
            Eigen::Quaterniond qc(w_R_c);             // current 
            Eigen::Quaterniond qd(w_R_d);             // desired 
            //
            Eigen::Quaterniond q_t = qc.slerp(Weight, qd);
            //
            Eigen::Matrix3d w_R_cd_t = q_t.toRotationMatrix();
            
            return q_t.toRotationMatrix();
        }

        Vector6d SaturationTwist(double p_lim, double o_lim, Vector6d Velo)
        {
            // 
            Eigen::Vector3d pos, orient;
            pos    = Velo.head<3>();
            orient = Velo.tail<3>();

            if((fabs(pos(0))>p_lim) || (fabs(pos(1))>p_lim) || (fabs(pos(2))>p_lim)){
                pos = p_lim * (1./pos.norm() * pos);
            }

            if((fabs(orient(0))>o_lim) || (fabs(orient(1))>o_lim) || (fabs(orient(2))>o_lim)){
                orient = o_lim * (1./orient.norm() * orient);
            }

            Vector6d out_velo;
            out_velo.head<3>() = pos;
            out_velo.tail<3>() = orient;
            return out_velo;
        }

        Eigen::Matrix3d rot_vector2rpy(Eigen::Vector3d a)
        {
            // a: roll/pitch/yaw or XYZ Tait-Bryan angles
            // https://en.wikipedia.org/wiki/Euler_angles
            double cos1 = cos(a[0]);
            double cos2 = cos(a[1]);
            double cos3 = cos(a[2]);
            double sin1 = sin(a[0]);
            double sin2 = sin(a[1]);
            double sin3 = sin(a[2]);

            Eigen::Matrix3d dircos;
            dircos(0,0) = (cos2*cos3);
            dircos(0,1) = -(cos2*sin3);
            dircos(0,2) = sin2;
            dircos(1,0) = ((cos1*sin3)+(sin1*(cos3*sin2)));
            dircos(1,1) = ((cos1*cos3)-(sin1*(sin2*sin3)));
            dircos(1,2) = -(cos2*sin1);
            dircos(2,0) = ((sin1*sin3)-(cos1*(cos3*sin2)));
            dircos(2,1) = ((cos1*(sin2*sin3))+(cos3*sin1));
            dircos(2,2) = (cos1*cos2);
            return dircos;
        }
        //
        Eigen::Matrix3d rpy2rFF(Eigen::Vector3d a)
        {
            double cos1 = cos(a[0]);
            double cos2 = cos(a[1]);
            double cos3 = cos(a[2]);
            double sin1 = sin(a[0]);
            double sin2 = sin(a[1]);
            double sin3 = sin(a[2]);

            Eigen::Matrix3d RxyzFF;
            RxyzFF(0,0) = cos3*cos2;
            RxyzFF(0,1) = cos3*sin1*sin2 - cos1*sin3;
            RxyzFF(0,2) = sin3*sin1 + cos3*cos1*sin2;

            RxyzFF(1,0) = cos2*sin3;
            RxyzFF(1,1) = cos3*cos1 + sin3*sin1*sin2;
            RxyzFF(1,2) = cos1*sin3*sin2 - cos3*sin1;

            RxyzFF(2,0) = -sin2;
            RxyzFF(2,1) = cos2*sin1;
            RxyzFF(2,2) = cos1*cos2;

            return RxyzFF;
        }

        Eigen::Vector3d getEulerAnglesXYZ_FixedFrame(Eigen::Matrix3d R)
        {
            // this function computed for a given rotation matrix the rotation angles around X, Y and Z axis considered as fixed.
            // the rotation matrix is assumed to be a Euler rotation matrix of type ZYX
            Eigen::Vector3d Angles;
            double Psi_X, Theta_Y, Phi_Z;
                Psi_X   = atan2(R(2,1),R(2,2));
                Theta_Y = atan2(-R(2,0), abs(sqrt(pow(R(0,0), 2.)+pow(R(1,0), 2.))));
                Phi_Z   = atan2(R(1,0),R(0,0));
            if ((Theta_Y>M_PI/2.)||(Theta_Y<-M_PI/2.))
            {
                Psi_X   = atan2(-R(2,1),-R(2,2));
                Theta_Y = atan2(-R(2,0),-abs(sqrt(pow(R(0,0), 2.)+pow(R(1,0), 2.))));
                Phi_Z   = atan2(-R(1,0),-R(0,0));
            }
            Angles(0) = Psi_X;
            Angles(1) = Theta_Y;
            Angles(2) = Phi_Z;

            return Angles;
        }
        //
        Eigen::Vector3d getEulerAnglesXYZ_FixedFrame(Vector7d pose)
        {
            //
            Eigen::Matrix3d WR = this->PoseVector2HomogenousMx(pose).block(0,0,3,3);
            return this->getEulerAnglesXYZ_FixedFrame(WR);
        }
        // 
        void orthonormalize(Eigen::MatrixXd& basis)
        {
            assert(basis.rows() == basis.cols());
            uint dim = basis.rows();
            basis.col(0).normalize();
            for(uint i=1;i<dim;i++){
                for(uint j=0;j<i;j++)
                    basis.col(i) -= basis.col(j).dot(basis.col(i))*basis.col(j);
                basis.col(i).normalize();
            }
        } 

        // compute the absolute pose between two poses
        bool get_absolute_pose(Vector7d Pose_l, Vector7d Pose_r, Vector7d &Pose_a)
        {
            //
            Matrix4d W_H_l = this->PoseVector2HomogenousMx(Pose_l);
            Matrix4d W_H_r = this->PoseVector2HomogenousMx(Pose_r);
            //
            //
            Matrix3d l_R_r = W_H_l.block<3,3>(0,0).transpose() * W_H_r.block<3,3>(0,0);
            // Axis angle of relative hands orientation
            Eigen::AngleAxisd l_orientation_r(l_R_r);
            // // Average orientation between hands
            Vector3d axis_ = l_orientation_r.axis();
            double theta_  = 0.5*l_orientation_r.angle();
            Eigen::AngleAxisd av_rot(theta_, axis_);

            // Rotation matrix of the absolute hand frame expressed in the asbolute foot frame
            // Matrix3d W_R_a =  W_H_l.block<3,3>(0,0) * av_rot.toRotationMatrix();
            Matrix3d W_R_a = getCombinedRotationMatrix(0.5, W_H_l.block<3,3>(0,0), W_H_r.block<3,3>(0,0));
            //
            Eigen::AngleAxisd W_orientation_a(W_R_a);
            
            //
            Pose_a.setZero(); 
            Pose_a.head(3) = 0.5*(W_H_l.block<3,1>(0,3) + W_H_r.block<3,1>(0,3));
            Pose_a.segment<3>(3) = W_orientation_a.axis();
            Pose_a.tail<1>()     << W_orientation_a.angle();

            return true;
        }

        // compute the relative pose between two poses
        bool get_relative_pose(Vector7d Pose_l, Vector7d Pose_r, Vector7d &Pose_rel)
        {
            //
            Matrix4d W_H_l = this->PoseVector2HomogenousMx(Pose_l);
            Matrix4d W_H_r = this->PoseVector2HomogenousMx(Pose_r);
            //
            Matrix3d l_R_r = W_H_l.block<3,3>(0,0).transpose() * W_H_r.block<3,3>(0,0);
            // Axis angle of relative hands orientation
            Eigen::AngleAxisd l_orientation_r(l_R_r);
            //
            Pose_rel.setZero();
            Pose_rel.head(3) = W_H_r.block<3,1>(0,3) - W_H_l.block<3,1>(0,3);
            Pose_rel.segment<3>(3) = l_orientation_r.axis();
            Pose_rel.tail<1>()    << l_orientation_r.angle();

            return true;
        }   
        //
        VectorXd get_Coordinated_task_pose(Vector7d Pose_lhand, Vector7d Pose_rhand, Matrix4d WH_aF_)
        {
            // Coordinated task pose is (Xa;Xr): Absolute and relative poses
            VectorXd cTsPose(14);   cTsPose.setZero();
            // 
            Vector7d Pose_abs;  Pose_abs.setZero(); // absolute pose
            Vector7d Pose_rel;  Pose_rel.setZero(); // relative pose

            // Transformation of the hand pose in the absolute feet frame
            Matrix4d aF_H_lh = WH_aF_.inverse() * this->PoseVector2HomogenousMx(Pose_lhand);
            Matrix4d aF_H_rh = WH_aF_.inverse() * this->PoseVector2HomogenousMx(Pose_rhand);
            //
            Vector7d aF_Pose_lhand = this->HomogenousMx2PoseVector(aF_H_lh);
            Vector7d aF_Pose_rhand = this->HomogenousMx2PoseVector(aF_H_rh);

            this->get_absolute_pose(aF_Pose_lhand, aF_Pose_rhand, Pose_abs);
            this->get_relative_pose(aF_Pose_lhand, aF_Pose_rhand, Pose_rel);
            //
            // 
            cTsPose.head(7) = Pose_abs;
            cTsPose.tail(7) = Pose_rel;

            return cTsPose;
        }

        //
        MatrixXd get_bimanual_task_TwistMap(double a_bi, double b_bi) // from individual hands velocities to absolute and relative velocities
        {
            // 
            double a_bi_ = a_bi;
            double b_bi_ = b_bi;
            if(a_bi_!= 0.0)
                b_bi_ = 1.0;
            else if (a_bi_== 1.0)
                b_bi_ = 0.0;

            Eigen::Matrix<double, 12, 12> C_hands;
            C_hands.setZero();
            // Bimanual transformation
            C_hands.block<6,6>(0,0) =      a_bi_ * Eigen::MatrixXd::Identity(6,6);
            C_hands.block<6,6>(0,6) = (1.-a_bi_) * Eigen::MatrixXd::Identity(6,6);
            C_hands.block<6,6>(6,0) =    -b_bi_  * Eigen::MatrixXd::Identity(6,6);
            C_hands.block<6,6>(6,6) =              Eigen::MatrixXd::Identity(6,6);

            //
            return C_hands;
        }

        //
        MatrixXd get_bimanual_task_TwistMap_inv(double a_bi, double b_bi)  // from absolute and relative velocities to individual hands velocities
        {
            // 
            double a_bi_ = a_bi;
            double b_bi_ = b_bi;
            if(a_bi_!= 0.0)
                b_bi_ = 1.0;
            else if (a_bi_== 1.0)
                b_bi_ = 0.0;

            Eigen::Matrix<double, 12, 12> C_hands;
            C_hands.setZero();
            // Bimanual transformation
            C_hands.block<6,6>(0,0) =               Eigen::MatrixXd::Identity(6,6);
            C_hands.block<6,6>(0,6) = -(1.-a_bi_) * Eigen::MatrixXd::Identity(6,6);
            C_hands.block<6,6>(6,0) =      b_bi_  * Eigen::MatrixXd::Identity(6,6);
            C_hands.block<6,6>(6,6) =       a_bi_ * Eigen::MatrixXd::Identity(6,6);
            //
            return C_hands;
        }
};



class KinConverter
{
  public:

    KinConverter(){}
    ~KinConverter(){}

    MatrixXd yarpPose2eigenHmatrix(Vector yarpPoseVect)
    {
        MatrixXd H_out(4,4);
        H_out.setIdentity(4,4);

        H_out(0,3) = yarpPoseVect[0];
        H_out(1,3) = yarpPoseVect[1];
        H_out(2,3) = yarpPoseVect[2];

        yarp::sig::Matrix RotMx;
        RotMx = yarp::math::axis2dcm(yarpPoseVect.subVector(3,6));
        // extracting data from a Eigen matrix to a yarp matrix
        for (int row=0; row<3; row++)
        {
            for (int col=0; col<3; col++)
            {
                H_out(row, col) = RotMx(row,col);
            }
        }
        return H_out;
    }
};

// int mysign(double Val)
// {
//     return (0.0 < Val) - (Val < 0.0);
// }

// Cubic polynomial Interpolator

class CubicInterpolator
{
    public:

    int nb_of_points;
    VectorXd Down_Index;
    VectorXd Up_Index;

    CubicInterpolator() {}
    ~CubicInterpolator(){}

    void DecreasingInterpolation(int nbpts)
    {
        nb_of_points = nbpts;
        Down_Index.resize(nb_of_points);
        Down_Index.setZero(nb_of_points);

        if (nb_of_points == 1)
        {
            Down_Index(0)= 1. ;
        }
        else
        {
            for(int i=0; i<nb_of_points; i++)
            {
                Down_Index(i) =  1.
                               -(3.*pow(i,2.)/pow(nb_of_points-1, 2.)
                               - 2.*pow(i,3.)/pow(nb_of_points-1, 3.));
            }
        }



    }

    void IncreasingInterpolation(int nbpts)
    {
        nb_of_points = nbpts;
        Up_Index.resize(nb_of_points);
        Up_Index.setZero(nb_of_points);

        if (nb_of_points == 1)
        {
            Down_Index(0)= 1. ;
        }
        else
        {
            for(int i=0; i<nb_of_points; i++)
            {
                Up_Index(i) =  3.*pow(i,2.)/pow(nb_of_points-1, 2.)
                              -2.*pow(i,3.)/pow(nb_of_points-1, 3.);
            }
        }


    }

};



class MatrixPseudoInverse
{

    public : 

        MatrixPseudoInverse(){}

        ~MatrixPseudoInverse(){}

        // Compute the pseudo inverse of a matrix
        template<typename _Matrix_Type_> _Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
        {
            
            Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);

            double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);

            return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
        }

};

class MatrixPseudoInverse2
{

    public : 

        MatrixPseudoInverse2(){}

        ~MatrixPseudoInverse2(){}

        // Compute the pseudo inverse of a matrix
        template<typename _Matrix_Type_> _Matrix_Type_ get_pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
        {
            
            Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);

            int svdSize = svd.singularValues().size();

            double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);

            return svd.matrixV().leftCols(svdSize) *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().leftCols(svdSize).adjoint();
        }

        bool get_HhQRPseudoInverse(MatrixXd myMatrix, MatrixXd &PsdInvmyMatrix)
        {

            HouseholderQR<MatrixXd> qr(myMatrix.transpose());
            PsdInvmyMatrix.setIdentity(myMatrix.cols(), myMatrix.rows());
            PsdInvmyMatrix = qr.householderQ() * PsdInvmyMatrix;
            PsdInvmyMatrix = qr.matrixQR().topLeftCorner(myMatrix.rows(),myMatrix.rows()).triangularView<Upper>().transpose().solve<OnTheRight>(PsdInvmyMatrix);

            return true;

        }

        bool get_CODecomPseudoInverse(MatrixXd myMatrix, MatrixXd &PsdInvmyMatrix)
        {
            //
            CompleteOrthogonalDecomposition<MatrixXd> cqr(myMatrix);
            PsdInvmyMatrix = cqr.pseudoInverse();

            return true;
        }

        bool get_LLTSolveInverse(MatrixXd myMatrix, MatrixXd &Inv_myMatrix)
        {
            //
            Eigen::MatrixXd UnitMx = Eigen::MatrixXd::Identity(myMatrix.cols(), myMatrix.rows()); 
            Inv_myMatrix = myMatrix.llt().solve(UnitMx);

            return true;
        }

        bool get_LUSolveInverse(MatrixXd myMatrix, MatrixXd &Inv_myMatrix)
        {
            //
            Eigen::MatrixXd UnitMx = Eigen::MatrixXd::Identity(myMatrix.cols(), myMatrix.rows()); 
            Inv_myMatrix = myMatrix.lu().solve(UnitMx);

            return true;
        }
        bool get_LDLTSolveInverse(MatrixXd myMatrix, MatrixXd &Inv_myMatrix)
        {
            //
            Eigen::MatrixXd UnitMx = Eigen::MatrixXd::Identity(myMatrix.cols(), myMatrix.rows()); 
            Inv_myMatrix = myMatrix.ldlt().solve(UnitMx);

            return true;
        }

};


// ===============================================================================================================================
// codyco math utilities
// ===============================================================================================================================

class codycoMathUtilities
{
    public :

    codycoMathUtilities() {}
    ~codycoMathUtilities() {}
        
    /** Check if a double is isnan.
     * We reimplement this function to avoid depending on C++11 (std::isnan) 
     * or C99 isnan macro.
     *
     * @param value value to be checked for nan
     * @return true if the value is a nan, false otherwise.
     */
    inline bool isnan(double value)
    {
        volatile double currentValue = value;
        return currentValue != currentValue;
    }
    
    /** @brief Computes the truncated pseudo inverse of a matrix
     *
     * This function computes the truncated pseudo inverse of a matrix.
     * Singular values less than the input tolerance will be set to sero.
     * By default the pseudo inverse will be computed by using the thin U and V unitary matrices.
     * @todo add default tolerance value.
     * @note this method will allocate memory for the SVD decomposition. If want to avoid this use 
     * codyco::math::pseudoInverse(const Eigen::Ref<const Eigen::MatrixXd>&, Eigen::JacobiSVD<typename Eigen::MatrixXd::PlainObject>&,
     * Eigen::Ref<Eigen::MatrixXd>, double, unsigned int)
     *
     * @param A the matrix to be pseudoinverted
     * @param Apinv the matrix in which to save the pseudoinversion of A. The size must be correct (same as \f$A^\top\f$)
     * @param tolerance tolerance to be used for the truncation
     * @param computationOptions Eigen options for the computation. By default compute the thin U and V matrices.
     */
    void pseudoInverse(const Eigen::Ref<const Eigen::MatrixXd>& A,
                       Eigen::Ref<Eigen::MatrixXd> Apinv,
                       double tolerance,
                       unsigned int computationOptions = Eigen::ComputeThinU|Eigen::ComputeThinV)
    {
        Eigen::JacobiSVD<typename Eigen::MatrixXd::PlainObject> svdDecomposition(A.rows(), A.cols());
        pseudoInverse(A, svdDecomposition, Apinv, tolerance, computationOptions);
    }
    
    /** @brief Computes the truncated pseudo inverse of a matrix
     *
     * This function computes the truncated pseudo inverse of a matrix.
     * Singular values less than the input tolerance will be set to sero.
     * By default the pseudo inverse will be computed by using the thin U and V unitary matrices.
     * @todo add default tolerance value.
     *
     * @param A the matrix to be pseudoinverted
     * @param svdDecomposition the decomposition object (already allocated) to be used
     * @param Apinv the matrix in which to save the pseudoinversion of A. The size must be correct (same as \f$A^\top\f$)
     * @param tolerance tolerance to be used for the truncation
     * @param computationOptions Eigen options for the computation. By default compute the thin U and V matrices.
     */
    void pseudoInverse(const Eigen::Ref<const Eigen::MatrixXd>& A,
                       Eigen::JacobiSVD<Eigen::MatrixXd::PlainObject>& svdDecomposition,
                       Eigen::Ref<Eigen::MatrixXd> Apinv,
                       double tolerance,
                       unsigned int computationOptions = Eigen::ComputeThinU|Eigen::ComputeThinV)
    {
        using namespace Eigen;
        int nullSpaceRows = -1, nullSpaceCols = -1;
        pseudoInverse(A, svdDecomposition, Apinv, tolerance,
                      (double*)0, nullSpaceRows, nullSpaceCols, computationOptions);
    }

    /** @brief Computes the truncated pseudo inverse of a matrix
     *
     * This function computes the truncated pseudo inverse of a matrix.
     * Singular values less than the input tolerance will be set to sero.
     * By default the pseudo inverse will be computed by using the thin U and V unitary matrices.
     * @note: this does not allow the null space basis to be computed
     * If you want to compute also the null space basis you have to:
     * - specify Eigen::ComputeFullV as computationOptions
     * - provide and already allocated buffer in nullSpaceBasisOfA
     * @todo add default tolerance value.
     *
     * @param A the matrix to be pseudoinverted
     * @param svdDecomposition the decomposition object (already allocated) to be used
     * @param Apinv the matrix in which to save the pseudoinversion of A. The size must be correct (same as \f$A^\top\f$)
     * @param tolerance tolerance to be used for the truncation
     * @param nullSpaceBasisOfA null space basis of the input matrix A. Pass NULL to avoid computation
     * @param[out] nullSpaceRows resulting rows for of the null space basis
     * @param[out] nullSpaceCols resulting columns for of the null space basis
     * @param computationOptions Eigen options for the computation. By default compute the thin U and V matrices.
     */
    void pseudoInverse(const Eigen::Ref<const Eigen::MatrixXd>& A,
                       Eigen::JacobiSVD<Eigen::MatrixXd::PlainObject>& svdDecomposition,
                       Eigen::Ref<Eigen::MatrixXd> Apinv,
                       double tolerance,
                       double * nullSpaceBasisOfA,
                       int &nullSpaceRows, int &nullSpaceCols,
                       unsigned int computationOptions = Eigen::ComputeThinU|Eigen::ComputeThinV)
    {
        using namespace Eigen;
        
        if (computationOptions == 0) return; //if no computation options we cannot compute the pseudo inverse
        svdDecomposition.compute(A, computationOptions);
        
        JacobiSVD<MatrixXd::PlainObject>::SingularValuesType singularValues = svdDecomposition.singularValues();
        int singularValuesSize = singularValues.size();
        int rank = 0;
        for (int idx = 0; idx < singularValuesSize; idx++) {
            if (tolerance > 0 && singularValues(idx) > tolerance) {
                singularValues(idx) = 1.0 / singularValues(idx);
                rank++;
            } else {
                singularValues(idx) = 0.0;
            }
        }
        
        //equivalent to this U/V matrix in case they are computed full
        Apinv = svdDecomposition.matrixV().leftCols(singularValuesSize) * singularValues.asDiagonal() * svdDecomposition.matrixU().leftCols(singularValuesSize).adjoint();

        if (nullSpaceBasisOfA && (computationOptions & ComputeFullV)) {
            //we can compute the null space basis for A
            nullSpaceBasisFromDecomposition(svdDecomposition, rank, nullSpaceBasisOfA, nullSpaceRows, nullSpaceCols);
        }
    }
    
    /** @brief Computes the damped pseudo inverse of a matrix
     *
     * This function computes the damped pseudo inverse of a matrix.
     * Rank is computed with the specified tolerance.
     * Singular values are regularized with the specified damping term, i.e.
     * we solve the minimization problem ||Ax - b||^2 + \rho^2 ||x||^2
     * with \rho the damping factor
     * By default the pseudo inverse will be computed by using the thin U and V unitary matrices.
     * @note: this does not allow the null space basis to be computed
     * If you want to compute also the null space basis you have to:
     * - specify Eigen::ComputeFullV as computationOptions
     * - provide and already allocated buffer in nullSpaceBasisOfA
     * @note the null space basis will be computed with the rank obtained by the tolerance factor, i.e. damping is not considered
     * @todo add default tolerance value.
     *
     * @param A the matrix to be pseudoinverted
     * @param svdDecomposition the decomposition object (already allocated) to be used
     * @param Apinv the matrix in which to save the pseudoinversion of A. The size must be correct (same as \f$A^\top\f$)
     * @param tolerance tolerance to be used for the truncation
     * @param computationOptions Eigen options for the computation. By default compute the thin U and V matrices.
     * @param nullSpaceBasisOfA null space basis of the input matrix A. Pass NULL to avoid computation
     * @param[out] nullSpaceRows resulting rows for of the null space basis
     * @param[out] nullSpaceCols resulting columns for of the null space basis
     */
    void dampedPseudoInverse(const Eigen::Ref<const Eigen::MatrixXd>& A,
                             Eigen::JacobiSVD<Eigen::MatrixXd::PlainObject>& svdDecomposition,
                             Eigen::Ref<Eigen::MatrixXd> Apinv,
                             double tolerance,
                             double dampingFactor,
                            unsigned int computationOptions = Eigen::ComputeThinU|Eigen::ComputeThinV,
                             double * nullSpaceBasisOfA = NULL,
                             int *nullSpaceRows = NULL, int *nullSpaceCols = NULL)
    {
        using namespace Eigen;
        
        if (computationOptions == 0) return; //if no computation options we cannot compute the pseudo inverse
        svdDecomposition.compute(A, computationOptions);
        
        JacobiSVD<MatrixXd::PlainObject>::SingularValuesType singularValues = svdDecomposition.singularValues();
        
        //rank will be used for the null space basis.
        //not sure if this is correct
        int singularValuesSize = singularValues.size();
        int rank = 0;
        for (int idx = 0; idx < singularValuesSize; idx++) {
            if (tolerance > 0 && singularValues(idx) > tolerance) {
                rank++;
            }
            singularValues(idx) = singularValues(idx) / ((singularValues(idx) * singularValues(idx)) + (dampingFactor * dampingFactor));
        }

        //equivalent to this U/V matrix in case they are computed full
        Apinv = svdDecomposition.matrixV().leftCols(singularValuesSize) * singularValues.asDiagonal() * svdDecomposition.matrixU().leftCols(singularValuesSize).adjoint();
        
        if (nullSpaceBasisOfA && nullSpaceRows && nullSpaceCols
            && (computationOptions & ComputeFullV)) {
            //we can compute the null space basis for A
            nullSpaceBasisFromDecomposition(svdDecomposition, rank, nullSpaceBasisOfA, *nullSpaceRows, *nullSpaceCols);
        }
    }
    
    /** @brief Compute the null space basis for the already computed SVD
     *
     * The nullSpaceBasisMatrix must be pre-allocated (for example to the size of the original matrix to be sure). You can then use the nullspace basis with the Eigen::Map object specifying as dimensions the output variables rows and cols
     * \param[in] svdDecomposition the already computed SVD decomposition
     * \param[in] tolerance the tolerance needed to compute the rank
     * \param[in] an already allocated buffer for the null space basis
     * \param[out] rows the rows of the nullspace basis
     * \param[out] cols the columns of the nullspace basis
     */
    void nullSpaceBasisFromDecomposition(Eigen::JacobiSVD<Eigen::MatrixXd::PlainObject>& svdDecomposition,
                                              double tolerance,
                                              double * nullSpaceBasisMatrix,
                                              int &rows, int &cols)
    {
        using namespace Eigen;
        JacobiSVD<MatrixXd::PlainObject>::SingularValuesType singularValues = svdDecomposition.singularValues();
        int rank = 0;
        for (int idx = 0; idx < singularValues.size(); idx++) {
            if (tolerance > 0 && singularValues(idx) > tolerance) {
                rank++;
            }
        }
        nullSpaceBasisFromDecomposition(svdDecomposition, rank, nullSpaceBasisMatrix, rows, cols);
        
    }
    
    /** @brief Compute the null space basis for the already computed SVD
     *
     * The nullSpaceBasisMatrix must be pre-allocated (for example to the size of the original matrix to be sure). You can then use the nullspace basis with the Eigen::Map object specifying as dimensions the output variables rows and cols
     * \param[in] svdDecomposition the already computed SVD decomposition
     * \param[in] rank rank of the original matrix
     * \param[in] an already allocated buffer for the null space basis
     * \param[out] rows the rows of the nullspace basis
     * \param[out] cols the columns of the nullspace basis
     */
    void nullSpaceBasisFromDecomposition(Eigen::JacobiSVD<Eigen::MatrixXd::PlainObject>& svdDecomposition,
                                         int rank,
                                         double * nullSpaceBasisMatrix,
                                         int &rows, int &cols)
    {
        using namespace Eigen;
        const MatrixXd &vMatrix = svdDecomposition.matrixV();
        //A \in \mathbb{R}^{uMatrix.rows() \times vMatrix.cols()}
        rows = vMatrix.cols();
        cols = vMatrix.cols() - rank;
        Map<MatrixXd> map(nullSpaceBasisMatrix, rows, cols);
        map = vMatrix.rightCols(vMatrix.cols() - rank);
    }
    /** @brief Computes the skew-symmetric (3D) matrix form of
     *  the input 3D vector for cross-product computation
     *
     * This function computes the skew symmetric matrix form of the input
     * vector needed for cross-product computation, i.e. 
     * given the following cross product
     * \f[
     *      \vec{a} \times \vec{b} = \left[ \vec{a} \right]_\times \vec{b}
     * \f]
     * this function returns \f$ \left[ \vec{a} \right]_\times \f$.
     *
     * @param vector 3D vector
     * @param skewSymmetricMatrix resultant skew symmetric matrix
     */
    void skewSymmentricMatrixFrom3DVector(const Eigen::Ref<const Eigen::Vector3d>& vector, 
                                          Eigen::Ref<Eigen::Matrix3d> skewSymmetricMatrix)

    {
        skewSymmetricMatrix.setZero();
        //            S = [   0,   -w(3),    w(2);
        //                 w(3),   0,     -w(1);
        //                 -w(2),  w(1),     0   ];
        skewSymmetricMatrix(0, 1) = -vector(2);
        skewSymmetricMatrix(0, 2) = vector(1);
        skewSymmetricMatrix(1, 2) = -vector(0);
        skewSymmetricMatrix.bottomLeftCorner<2, 2>() = -skewSymmetricMatrix.topRightCorner<2, 2>().transpose();
    }


};


// ==========================
class firstOrderFilter
{

        // double pole;
        // double gain;
        double Ts;

        // VectorXd init_fn;
        VectorXd init_fn2;
        VectorXd init_fn3;
        VectorXd init_fn4;
        VectorXd delta1;
        VectorXd delta2;
        VectorXd delta3;
        VectorXd delta4;
        VectorXd y_t;

    public:

        double pole;
        double gain;
        VectorXd init_fn;

        firstOrderFilter(){}
        // 
        void InitializeFilter(double T, double gn, double pl, Eigen::VectorXd init_fn_val)
        {
            Ts = T;
            gain = gn;
            pole = pl;

            init_fn.resize(init_fn_val.rows(), init_fn_val.cols());
            init_fn2.resize(init_fn_val.rows(), init_fn_val.cols());
            init_fn3.resize(init_fn_val.rows(), init_fn_val.cols());
            init_fn4.resize(init_fn_val.rows(), init_fn_val.cols());

            delta1.resize(init_fn_val.rows(), init_fn_val.cols());
            delta2.resize(init_fn_val.rows(), init_fn_val.cols());
            delta3.resize(init_fn_val.rows(), init_fn_val.cols());
            delta4.resize(init_fn_val.rows(), init_fn_val.cols());

            y_t.resize(init_fn_val.rows(), init_fn_val.cols());


            init_fn = init_fn_val;

        }

         ~firstOrderFilter(){}

        VectorXd function_dot(double gn, double pl, const Eigen::VectorXd &init_fn_val, const Eigen::VectorXd &fn_t)
        {
            return - pl * init_fn_val + gn * fn_t;
        }

        // compute the integral of first order differential eq. using RK4
        VectorXd getRK4Integral(const Eigen::VectorXd &fn_t)
        {
    //
            delta1   = Ts * function_dot(gain, pole, init_fn, fn_t);

            init_fn2 = init_fn + 0.5 * delta1;

            delta2   = Ts * function_dot(gain, pole, init_fn2, fn_t);

            init_fn3 = init_fn + 0.5 * delta2;

            delta3   = Ts * function_dot(gain, pole, init_fn3, fn_t);

            init_fn4 = init_fn + 0.5 * delta3;

            delta4   = Ts * function_dot(gain, pole, init_fn4, fn_t);

            // solution
            y_t      = init_fn + 1/6. * (delta1 + 2.* delta2 + 2.* delta3 + delta4);
            init_fn  = y_t;

            return y_t;

        }

        VectorXd getEulerIntegral(const Eigen::VectorXd &fn_t)
        {
    //
            delta1   = Ts * function_dot(gain, pole, init_fn, fn_t);

            // solution
            y_t      = init_fn + delta1;
            init_fn  = y_t;

            return y_t;

        }

        void setGain(double _gain)
        {
            gain = _gain;
        }

        void setPole(double _pole)
        {
            pole = _pole;
        }

        void setSampleTime(double T)
        {
            Ts = T;
        }

        
};


class SecondOrderFilter
{

        // double pole;
        // double gain;
        double Ts;

        // VectorXd init_fn;
        VectorXd init_fn2;
        VectorXd init_fn3;
        VectorXd init_fn4;
        VectorXd delta1;
        VectorXd delta2;
        VectorXd delta3;
        VectorXd delta4;
        VectorXd y_t;

    public:

        double pole;
        double gain;
        Eigen::MatrixXd K;
        Eigen::MatrixXd D;

        VectorXd init_fn;

        SecondOrderFilter(){}
        // 
        void InitializeFilter(double T, Eigen::MatrixXd K_, Eigen::MatrixXd D_, Eigen::VectorXd init_fn_val)
        {
            Ts = T;
            K = K_;
            D = D_;

            init_fn.resize(2*init_fn_val.rows(), init_fn_val.cols());
            init_fn2.resize(2*init_fn_val.rows(), init_fn_val.cols());
            init_fn3.resize(2*init_fn_val.rows(), init_fn_val.cols());
            init_fn4.resize(2*init_fn_val.rows(), init_fn_val.cols());

            delta1.resize(2*init_fn_val.rows(), init_fn_val.cols());
            delta2.resize(2*init_fn_val.rows(), init_fn_val.cols());
            delta3.resize(2*init_fn_val.rows(), init_fn_val.cols());
            delta4.resize(2*init_fn_val.rows(), init_fn_val.cols());

            y_t.resize(2*init_fn_val.rows(), init_fn_val.cols());

            init_fn = init_fn_val;

        }

         ~SecondOrderFilter(){}

        VectorXd function_dot(const Eigen::VectorXd &init_fn_val, const Eigen::VectorXd &fn_t)
        {
            //
            Eigen::MatrixXd Dyn(init_fn_val.rows(), init_fn_val.rows());
            Dyn.setZero();
            Dyn.block(0, K.cols(), K.rows(), D.cols()) = Eigen::MatrixXd::Identity(K.rows(), D.cols());
            Dyn.block(K.rows(), 0, K.rows(), K.cols()) = -K;
            Dyn.block(K.rows(), K.cols(), D.rows(), D.cols()) = -D;

            Eigen::VectorXd inpt(2*K.rows());
            inpt.setZero();
            inpt.segment(K.rows(), K.rows()) = fn_t;

            return Dyn * init_fn_val + inpt;
        }

        // compute the integral of first order differential eq. using RK4
        VectorXd getRK4Integral(const Eigen::VectorXd &fn_t)
        {
            delta1   = Ts * function_dot(init_fn, fn_t);
            init_fn2 = init_fn + 0.5 * delta1;
            delta2   = Ts * function_dot(init_fn2, fn_t);
            init_fn3 = init_fn + 0.5 * delta2;
            delta3   = Ts * function_dot(init_fn3, fn_t);
            init_fn4 = init_fn + 0.5 * delta3;
            delta4   = Ts * function_dot(init_fn4, fn_t);
            // solution
            y_t      = init_fn + 1/6. * (delta1 + 2.* delta2 + 2.* delta3 + delta4);
            init_fn  = y_t;

            return y_t;
        }

        VectorXd getEulerIntegral(const Eigen::VectorXd &fn_t)
        {
            delta1   = Ts * function_dot(init_fn, fn_t);
            // solution
            y_t      = init_fn + delta1;
            init_fn  = y_t;

            return y_t;
        }

        void setDamper(Eigen::MatrixXd D_)
        {
            D = D_;
        }

        void setStiffness(Eigen::MatrixXd K_)
        {
            K = K_;
        }

        void setSampleTime(double T)
        {
            Ts = T;
        }
        
};


class QPSolverOASES
{

   public :

    SQProblem OasesSolver;

    //real_t xOpt[];
    //int_t nbWorkSetRecal;
    int nbWorkSetRecal;

    // Solving the QP problem
    QPSolverOASES(){};
    ~QPSolverOASES(){};

    void InitialiseQPSol(   MatrixXd H,
                            VectorXd f,
                            MatrixXd A_cons,
                            VectorXd b)
    {
        //
        OasesSolver = SQProblem (H.rows(), A_cons.rows());

        real_t xOpt[H.rows()];

        /* Setup data of first QP. */
        real_t HMx[H.rows() * H.cols()];
        real_t gVc[H.rows()];
        real_t AMx[A_cons.rows() * A_cons.cols()];
        real_t ubA[b.rows()];
        //
        int iter = 0;

        for (int i=0; i<H.rows(); i++)
        {
        for (int j=0; j<H.cols(); j++)
        {
           HMx[iter] = H(i,j);
           iter +=1;
        }
        gVc[i] = f(i);
        }
        iter = 0;

        for (int i=0; i<A_cons.rows(); i++)
        {
        for (int j=0; j<A_cons.cols(); j++)
        {
           AMx[iter] = A_cons(i,j);
           iter +=1;
        }
        ubA[i] = b(i);
        }

        nbWorkSetRecal = 1000;  //200  450
        OasesSolver.init(HMx,gVc,AMx,0,0,0,ubA, nbWorkSetRecal, 0);
        OasesSolver.getPrimalSolution(xOpt);
    };

    VectorXd qpOasesSolver (MatrixXd H,
                           VectorXd f,
                           MatrixXd A_cons,
                           VectorXd b)
    {
        //
        VectorXd OptimalSol(H.rows());
        real_t xOpt[H.rows()];

        /* Setup data of first QP. */
       real_t HMx_new[H.rows()*H.cols()];
       real_t gVc_new[H.rows()];
       real_t AMx_new[A_cons.rows()*A_cons.cols()];
       real_t ubA_new[b.rows()];

       int iter;
       iter = 0;
       for (int i=0; i<H.rows(); i++)
       {
           for (int j=0; j<H.cols(); j++)
           {
               HMx_new[iter] = H(i,j);
               iter +=1;
           }
           gVc_new[i] = f(i);
       }
       iter = 0;

       for (int i=0; i<A_cons.rows(); i++)
       {
           for (int j=0; j<A_cons.cols(); j++)
           {
               AMx_new[iter] = A_cons(i,j);
               iter +=1;
           }
           ubA_new[i] = b(i);
       }

       OasesSolver.hotstart(HMx_new,gVc_new,AMx_new, 0, 0, 0, ubA_new, nbWorkSetRecal, 0);
      //extracting the optimal solution
       OasesSolver.getPrimalSolution(xOpt);

       for (int i=0; i<H.rows(); i++)
       {
           OptimalSol(i) = xOpt[i];
       }

       return OptimalSol;
    };

    void setnbWorkingSetRecalculation(int nWSR)
    {
         nbWorkSetRecal = nWSR;
    };

    void setSolverOptions(Options optionToSet)
    {
        OasesSolver.setOptions(optionToSet);
    };

};


#endif // wbhpidcUtilityFunctions_H