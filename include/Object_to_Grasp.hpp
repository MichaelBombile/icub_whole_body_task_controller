
// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#ifndef Object_to_Grasp_H
#define Object_to_Grasp_H

#include <string>
#include <iostream>
#include <fstream>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Geometry>

#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>

#include "wbhpidcUtilityFunctions.hpp"
#include "ControllerParameters.h"

using namespace std;
using namespace Eigen;



class Object_to_Grasp
{
        
        double g;

    public:

        Object_to_Grasp(){}

        ~Object_to_Grasp()
        {
            if(isObjectPortOpen){
                this->closeObjectPort();
            }
        }

        std::string Object_Name;
        

        double mass;
        Eigen::Matrix3d Jo;
        Matrix6d Mo;
        Vector6d bo;
        Matrix6d Mo_world;
        Vector6d bo_world;

        ObjectDimension Dimensions;
        Vector7d Obj_Pose_Gpoints[N_eef];
        Matrix4d w_H_Gpoints[N_eef];                 // pose object grasp point in the world frame

        Vector7d Object_Pose;
        Vector6d Object_Velocity;
        Vector6d Object_Acceleration;

        Vector7d init_object_pose;

        
        // Eigen::Vector3d Wo;

        Matrix4d w_H_Object;

        Vector7d filt_w_Pose_Object;
        Eigen::Vector3d w_object_position_previous;
        Eigen::Matrix3d w_Rot_object_previous;

        bool isObjectPortOpen;    

        // Object
        std::string                              ObjectPortName;
        yarp::os::Bottle                         *ObjectlinkPose_values;
        yarp::os::BufferedPort<yarp::os::Bottle> ObjectlinkPose_port_In;
        Eigen::VectorXd                          ObjectlinkPose_measurements;

        double period_sec;

        KineTransformations  Transforms;

        // ----------------------------------------------
        TaskSpaceTrajectories States_Object;
        TaskSpaceTrajectories Ref_Object;

        //
        Matrix4d obj_H_Markers;

        // Vector6d w_velocity_object_0;
        // Vector6d error_object;
        // Matrix3d L_Mu_Theta_object;
        // Matrix6d L_eta_object_in_w;
        // // Filter object motion
        // firstOrderFilter  Filter_object;
        // ----------------------------------------------
         // Estimation of the object velocity from its pose
        Vector7d            ObjectPoseEstimate;
        Vector7d            filtered_ObjectPoseEstimate;
        Vector7d            dot_ObjectPoseEstimate;
        firstOrderFilter    ObjectFilter;
        double              filter_gain_Object;
        //
        MatrixPseudoInverse2  PseudoInverser;       // Object to compute the pseudo inverse of a matrix


        void Initialize(ControllerParameters &ctrl_param, double period_s)
        {
            // sampling time (running period)
            period_sec          = period_s;
            // gravity
            g = 9.81;
            //
            Object_Name         = ctrl_param.object_name;
            ObjectPortName      = ctrl_param.object_port_name;
            mass                = ctrl_param.object_mass;
            Jo                  = ctrl_param.object_inertia; // Jo_
            Dimensions          = ctrl_param.object_dimensions;
            // 
            Obj_Pose_Gpoints[0] = ctrl_param.object_Pose_Gpoints[0];
            Obj_Pose_Gpoints[1] = ctrl_param.object_Pose_Gpoints[1];
            //
            Mo.block<3,3>(0,0)  = mass * Eigen::MatrixXd::Identity(3, 3);
            Mo.block<3,3>(3,3)  = Jo;
            //
            Mo_world.setZero();
            bo_world.setZero();
            //
            Object_Pose.setZero();
            Object_Velocity.setZero();
            Object_Acceleration.setZero();

            w_H_Gpoints[0].setIdentity();
            w_H_Gpoints[1].setIdentity();
            w_H_Object.setIdentity();

            filt_w_Pose_Object.setZero();

            //
            States_Object.setZero();
            Ref_Object.setZero();

            //
            obj_H_Markers.setIdentity(4,4);
            obj_H_Markers = ctrl_param.object_H_Markers;
            //
            //
            ObjectPoseEstimate.setZero();
            filtered_ObjectPoseEstimate.setZero();
            dot_ObjectPoseEstimate.setZero();
            //
            
            
        }

        //
        bool getWorldInertia(Vector7d Object_Pose_)
        {
            //
            Mo_world.setIdentity();
            //
            Vector3d cur_orient_axis = Object_Pose_.segment<3>(3)/Object_Pose_.segment<3>(3).norm();
            //
            Mo_world.block<3,3>(0,0) = mass * Eigen::MatrixXd::Identity(3, 3);
            Mo_world.block<3,3>(3,3) = Eigen::AngleAxisd(Object_Pose_(6), cur_orient_axis).toRotationMatrix() * Jo * Eigen::AngleAxisd(Object_Pose_(6), cur_orient_axis).toRotationMatrix().transpose();
            return true;
        }

        bool getWorldBiasForce(Vector7d Object_Pose_, Eigen::Vector3d Wo)
        {
            Eigen::Matrix3d Jo_world;       Jo_world.setIdentity();
            //
            Vector3d cur_orient_axis = Object_Pose_.segment<3>(3)/Object_Pose_.segment<3>(3).norm();
            Jo_world = Eigen::AngleAxisd(Object_Pose_(6), cur_orient_axis).toRotationMatrix() * Jo * Eigen::AngleAxisd(Object_Pose_(6), cur_orient_axis).toRotationMatrix().transpose();
            // skew Mx
            Eigen::Matrix3d Skew_Wo_world;
            Skew_Wo_world <<   0.0, -Wo(2),  Wo(1),
                             Wo(2),    0.0, -Wo(0),
                            -Wo(1),  Wo(0),    0.0;

            bo_world.segment<3>(0) << 0.0, 0.0, g*mass;
            bo_world.segment<3>(3) = -Skew_Wo_world * Jo_world * Wo;

            return true;
        }

        bool update_object_state()
        {
            //
            // if(isObjectPortOpen)
            // {
                this->Get_Object_World_Pose(Object_Pose);
                this->Estimate_object_World_Twist(Object_Pose, Object_Velocity);
            // }

            this->getWorldInertia(Object_Pose);
            this->getWorldBiasForce(Object_Pose, Object_Velocity.tail<3>());

            //
            w_H_Object     = Transforms.PoseVector2HomogenousMx(Object_Pose);
            w_H_Gpoints[0] = w_H_Object * Transforms.PoseVector2HomogenousMx(Obj_Pose_Gpoints[0]);
            w_H_Gpoints[1] = w_H_Object * Transforms.PoseVector2HomogenousMx(Obj_Pose_Gpoints[1]);

            States_Object.pose      = Object_Pose;
            States_Object.velocity  = Object_Velocity;

            return true;
        }

        bool update_object_state(Vector7d Object_Pose_, Eigen::Vector3d Wo)
        {
            //
            if(isObjectPortOpen)
            {
                this->Get_Object_World_Pose(Object_Pose_);
                this->Estimate_object_World_Twist(Object_Pose_, Object_Velocity);
            }

            this->getWorldInertia(Object_Pose_);
            this->getWorldBiasForce(Object_Pose_, 0.0*Object_Velocity.tail<3>());

            //
            w_H_Object     = Transforms.PoseVector2HomogenousMx(Object_Pose_);
            w_H_Gpoints[0] = w_H_Object * Transforms.PoseVector2HomogenousMx(Obj_Pose_Gpoints[0]);
            w_H_Gpoints[1] = w_H_Object * Transforms.PoseVector2HomogenousMx(Obj_Pose_Gpoints[1]);

            States_Object.pose      = Object_Pose;
            States_Object.velocity  = Object_Velocity;
           
            return true;
        }       
        //
        bool update_object_state(Matrix4d aF_H_GW)
        {
            //
            // if(isObjectPortOpen)
            // {
                this->Get_Object_World_Pose(Object_Pose);
                this->Estimate_object_World_Twist(Object_Pose, Object_Velocity);
            // }
                Matrix4d aF_H_o  = aF_H_GW * Transforms.PoseVector2HomogenousMx(Object_Pose);
                Object_Pose = Transforms.HomogenousMx2PoseVector(aF_H_o);
                Object_Velocity.head(3) = aF_H_GW.block(0,0,3,3) * Object_Velocity.head(3);
                Object_Velocity.tail(3) = aF_H_GW.block(0,0,3,3) * Object_Velocity.tail(3);

            this->getWorldInertia(Object_Pose);
            this->getWorldBiasForce(Object_Pose, Object_Velocity.tail<3>());

            // w correspond to absolute feet frame
            w_H_Object     = Transforms.PoseVector2HomogenousMx(Object_Pose);
            
            w_H_Gpoints[0] = w_H_Object * Transforms.PoseVector2HomogenousMx(Obj_Pose_Gpoints[0]);
            w_H_Gpoints[1] = w_H_Object * Transforms.PoseVector2HomogenousMx(Obj_Pose_Gpoints[1]);

            States_Object.pose      = Object_Pose;
            States_Object.velocity  = Object_Velocity;

            return true;
        }

        bool update_object_state(Vector7d W_P_pelvis, Matrix4d W_H_absF, Matrix4d m_W_H_pelvis)
        {
            //
            // if(isObjectPortOpen)
            // {
                this->Get_Object_World_Pose(Object_Pose);
                this->Estimate_object_World_Twist(Object_Pose, Object_Velocity);
            // }
                //
                Matrix4d absF_H_P = W_H_absF.inverse() * Transforms.PoseVector2HomogenousMx(W_P_pelvis);
                Matrix4d m_P_H_o  = m_W_H_pelvis.inverse() * Transforms.PoseVector2HomogenousMx(Object_Pose);
                Matrix4d absF_H_o = absF_H_P * m_P_H_o;
                //

                // Matrix4d aF_H_o  = aF_H_GW * Transforms.PoseVector2HomogenousMx(Object_Pose);
                Object_Pose = Transforms.HomogenousMx2PoseVector(absF_H_o);
                Object_Velocity.head(3) = absF_H_P.block(0,0,3,3) * m_W_H_pelvis.block(0,0,3,3).transpose() * Object_Velocity.head(3);
                Object_Velocity.tail(3) = absF_H_P.block(0,0,3,3) * m_W_H_pelvis.block(0,0,3,3).transpose() * Object_Velocity.tail(3);

            this->getWorldInertia(Object_Pose);
            this->getWorldBiasForce(Object_Pose, Object_Velocity.tail<3>());

            // w correspond to absolute feet frame
            w_H_Object     = Transforms.PoseVector2HomogenousMx(Object_Pose);
            
            w_H_Gpoints[0] = w_H_Object * Transforms.PoseVector2HomogenousMx(Obj_Pose_Gpoints[0]);
            w_H_Gpoints[1] = w_H_Object * Transforms.PoseVector2HomogenousMx(Obj_Pose_Gpoints[1]);

            States_Object.pose      = Object_Pose;
            States_Object.velocity  = Object_Velocity;

            return true;
        }

        // ===========================================================================================

        bool OpenObjectPort(std::string RobotName)
        {
            //
            std::string Local_ObjectlinkPose_portName="/";
            Local_ObjectlinkPose_portName += RobotName;
            Local_ObjectlinkPose_portName += "/Object_Pose_In:i";
            // Read object world Pose
            ObjectlinkPose_port_In.open(Local_ObjectlinkPose_portName.c_str());
            if(!yarp::os::Network::connect(this->ObjectPortName.c_str(), ObjectlinkPose_port_In.getName().c_str())){
                printf(" Unable to connect to the Object port");
                return false;
            }
            //---------------------------------------------------------------------
            // read the object pose
            //---------------------------------------------------------------------
            ObjectlinkPose_values = ObjectlinkPose_port_In.read(); 
            ObjectlinkPose_measurements.resize(ObjectlinkPose_values->size());    

            this->Get_Object_World_Pose(Object_Pose);

            init_object_pose = Object_Pose;

            filt_w_Pose_Object = Object_Pose;
            w_object_position_previous =    Object_Pose.head<3>();

            // compute the rotation mx from axis angle
            Eigen::Vector3d axis    = Object_Pose.segment(3,3)/Object_Pose.segment(3,3).norm();
            w_Rot_object_previous   = Eigen::AngleAxisd(Object_Pose(6), axis).toRotationMatrix();
            //
            this->Get_Object_World_Pose(Object_Pose);
            Matrix4d w_H_O = Transforms.PoseVector2HomogenousMx(Object_Pose);
            Eigen::Quaterniond qb(w_H_O.block<3,3>(0,0));
            ObjectPoseEstimate.head<3>() = w_H_O.block<3,1>(0,3);
            ObjectPoseEstimate.tail<4>() << qb.x(), qb.y(), qb.z(), qb.w();
            // Initialize the filter
            filter_gain_Object = 5.0;  // pole of the filter
            ObjectFilter.InitializeFilter(period_sec, filter_gain_Object, filter_gain_Object, ObjectPoseEstimate);
            

            isObjectPortOpen = true;

            return true;
        }

        //
        void Get_Object_World_Pose(Vector7d &w_Pose_Object_)
        {
            //
            // read the object pose
            //---------------------------------------------------------------------
            ObjectlinkPose_values = ObjectlinkPose_port_In.read(); 
            // ObjectlinkPose_measurements.resize(ObjectlinkPose_values->size());    
            for (int i= 0;i < ObjectlinkPose_values->size(); i++)
                ObjectlinkPose_measurements(i) = ObjectlinkPose_values->get(i).asDouble();

            // update the quaternion
            Eigen::VectorXd qcoeff(4);
            qcoeff << ObjectlinkPose_measurements(6), 
                      ObjectlinkPose_measurements(3),
                      ObjectlinkPose_measurements(4),
                      ObjectlinkPose_measurements(5);
            // normalizing the quaternion
            qcoeff = qcoeff/ qcoeff.norm();
            Eigen::Quaterniond q_obj(qcoeff(0), qcoeff(1), qcoeff(2), qcoeff(3));  // w, x, y, z
            Eigen::AngleAxisd obj_orient(q_obj);
            //
            w_Pose_Object_.head<3>()     = ObjectlinkPose_measurements.head<3>() + obj_orient.toRotationMatrix() * obj_H_Markers.block<3,1>(0,0);
            w_Pose_Object_.segment<3>(3) = obj_orient.axis();
            w_Pose_Object_(6)            = obj_orient.angle();

            Object_Pose = w_Pose_Object_;

        }

        // void Estimate_object_World_Twist(Vector7d w_Pose_object_, Vector6d& Object_Velocity)
        // {
        //     //
        //     // filt_w_Pose_Object = 0.995*filt_w_Pose_Object + 0.005*w_Pose_object_;
        //     filt_w_Pose_Object = w_Pose_object_;


        //     // compute the rotation mx from axis angle
        //     Eigen::Vector3d axis = w_Pose_object_.segment(3,3)/w_Pose_object_.segment(3,3).norm();
        //     Eigen::Matrix3d Rot_object_world = Eigen::AngleAxisd(w_Pose_object_(6), axis).toRotationMatrix();

        //     // Eigen::Quaterniond w_obj_quat(filt_w_Pose_Object(6), filt_w_Pose_Object(3), filt_w_Pose_Object(4), filt_w_Pose_Object(5));
        //     // Eigen::Matrix3d Rot_object_world = w_obj_quat.normalized().toRotationMatrix();


        //     // get the skew-symmetric matrix associated with angular velocity
        //     Eigen::Matrix3d skew_omega_object_world = 1./period_sec * (Rot_object_world - w_Rot_object_previous) * Rot_object_world.transpose();

        //     Vector6d object_twist_world;
        //     object_twist_world.topRows(3)    = 1./period_sec * (filt_w_Pose_Object.head<3>() - w_object_position_previous);
        //     object_twist_world.bottomRows(3) << skew_omega_object_world(1,0), skew_omega_object_world(0,2), skew_omega_object_world(2,1);

        //     w_object_position_previous = filt_w_Pose_Object.head<3>();
        //     w_Rot_object_previous      = Rot_object_world;

        //     Object_Velocity = object_twist_world;
        // }

        void Estimate_object_World_Twist(Vector7d w_Pose_object_, Vector6d& Object_Velocity)
        {
            // ============================================================================================================================
            Matrix4d w_H_O = Transforms.PoseVector2HomogenousMx(w_Pose_object_);

            Eigen::Quaterniond qb(w_H_O.block<3,3>(0,0));
            ObjectPoseEstimate.head<3>() = w_H_O.block<3,1>(0,3);
            ObjectPoseEstimate.tail<4>() << qb.x(), qb.y(), qb.z(), qb.w();

            //  filter and get the derivative of the measurements
            filtered_ObjectPoseEstimate  = ObjectFilter.getRK4Integral(ObjectPoseEstimate);
            dot_ObjectPoseEstimate       = filter_gain_Object * (ObjectPoseEstimate - filtered_ObjectPoseEstimate);
            //
            Eigen::MatrixXd Omega2dqMx(4,3), psdinvOmega2dqMx(3,4);

            Omega2dqMx(0,0) = -ObjectPoseEstimate(3);  Omega2dqMx(0,1) = -ObjectPoseEstimate(4);  Omega2dqMx(0,2) = -ObjectPoseEstimate(5);  
            Omega2dqMx(1,0) =  ObjectPoseEstimate(6);  Omega2dqMx(1,1) =  ObjectPoseEstimate(5);  Omega2dqMx(1,2) = -ObjectPoseEstimate(4);
            Omega2dqMx(2,0) = -ObjectPoseEstimate(5);  Omega2dqMx(2,1) =  ObjectPoseEstimate(6);  Omega2dqMx(2,2) =  ObjectPoseEstimate(3);
            Omega2dqMx(3,0) =  ObjectPoseEstimate(4);  Omega2dqMx(3,1) = -ObjectPoseEstimate(3);  Omega2dqMx(3,2) =  ObjectPoseEstimate(6);

            psdinvOmega2dqMx = PseudoInverser.get_pseudoInverse(Omega2dqMx); 
            //
            Eigen::VectorXd quat_dot(4);
            quat_dot(0) = dot_ObjectPoseEstimate(6);
            quat_dot(1) = dot_ObjectPoseEstimate(3);
            quat_dot(2) = dot_ObjectPoseEstimate(4);
            quat_dot(3) = dot_ObjectPoseEstimate(5);
            //
            Object_Velocity.head<3>() = dot_ObjectPoseEstimate.head<3>();
            Object_Velocity.tail<3>() = 2.*psdinvOmega2dqMx * quat_dot;
            // ============================================================================================================================

        }

        //
        void closeObjectPort()
        {
            this->ObjectlinkPose_port_In.close();
        }

};

#endif // Object_to_Grasp_H