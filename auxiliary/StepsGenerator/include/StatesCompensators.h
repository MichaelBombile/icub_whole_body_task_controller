/* 
 * Copyright (C) 2017 Learning Algorithms and Systems Laboratory (LASA)
 * Author: Michael Bombile
 * email:  michael.bombile@epfl.ch
 * website: lasa.epfl.ch
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

// 4. EstimatorCompensators
// 	4.1. InertialCompensator
// 	4.2. StatesToInputCompemsator
// 	4.3. ReferencesCompensator
// 	4.4. CoMStatesEstimator

#ifndef StatesCompensators_H
#define StatesCompensators_H


#include <iostream>
#include <cmath>
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>

#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include <iCub/iKin/iKinFwd.h>

#include "PatternsGenerator.h"
#include "TemplateModels.h"
#include "CpMath_Utilities.h"

using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace yarp::os;
using namespace iCub::ctrl;

using namespace std;
using namespace Eigen;



class InertialCompensator
{
    
    Vector3d TrslBaseCoM;      // Translation vector from base to CoM

    public :

        Matrix4d DesTrsfRealStanceFootInHorBase;   // 
        Matrix4d CurTrsfRealStanceFootInHorBase;
        Matrix4d ReferenceTrsfHorFootInHorBase;
        Matrix4d TrsfCurBaseInWorldIMU;
        Matrix4d FwKRightFootInBase;
        Matrix4d FwKLeftFootInBase;
        Matrix3d CurRotRealStFootInHorBase;
        Matrix3d RotRealStFootInHorStFoot;
        Matrix4d ReferenceTrsfSwingFootInHorBase;
        Matrix4d DesTrsfSwingFootInCurBase;
        Matrix4d DesTrsfLeftFootInHorBase;
        Matrix4d DesTrsfRightFootInHorBase;

        // Transformation of Homogeoneous transformation into Pose vector with axis/angle representation of orientation
        Vector7d DesiredLeftLegPoseAsAxisAngles;
        Vector7d DesiredRightLegPoseAsAxisAngles;
        Eigen::Matrix<double, 4,1> DesiredLeftLegAngleAxisOrientation;
        Eigen::Matrix<double, 4,1> DesiredRightLegAngleAxisOrientation;
        Matrix3d DesiredRotationMatrixLeftLeg;
        Matrix3d DesiredRotationMatrixRightLeg;

        // Methods

        InertialCompensator();
        ~InertialCompensator();

        void getDesiredFeetInCorrectedBase(                   int SptFt[],
                                                Vector7d LLegPose,
                                                Vector7d RLegPose,
                                                Vector3d m_orientation_rpy,
                                            CpGaitTransformations *GTj);

        // void getDesiredFeetInCorrectedBase(                   int SptFt[],
        //                                         yarp::sig::Vector LLegPose,
        //                                         yarp::sig::Vector RLegPose,
        //                                         Eigen::VectorXd Inertial_measurements,
        //                                     CpGaitTransformations *GTj);

        void SetImuTransformation(Vector3d IMU_RollPitchYaw);
        Vector3d getEulerAnglesXYZ_FixedFrame(Matrix3d CurRotRealStFootInHorBase);
        Matrix3d getComposedOrientFixedFrame(Vector3d OrientXY_F_hF);
        Matrix3d getComposedEulerOriention(Vector3d a);
        							
        //MatrixXd getDesiredSwingFootInCurBase();
        void SetImuTransformation();					 
        void SetTranslationBaseCoM(VectorXd t_B_CoM);
        void CompensateTransformsWithIMU(CpGaitTransformations *GTj,
                                                      Vector7d lleg_EE_pose,
                                                      Vector7d rleg_EE_pose,
                                                      Vector3d ImuOrientationRYP,
                                                       int Stance[],
                                                      bool isActive);
        
};

#endif // StatesCompensators_H