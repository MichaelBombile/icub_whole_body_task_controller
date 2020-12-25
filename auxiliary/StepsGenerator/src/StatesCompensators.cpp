


#include "StatesCompensators.h"

// ====================================================================================================================
// ====================================================================================================================
InertialCompensator::InertialCompensator()
{

    TrsfCurBaseInWorldIMU.setIdentity();
    FwKLeftFootInBase.setIdentity();
    FwKRightFootInBase.setIdentity();
    DesiredLeftLegPoseAsAxisAngles.setZero();
    DesiredRightLegPoseAsAxisAngles.setZero();

    DesiredLeftLegAngleAxisOrientation.resize(4, 0.0);
    DesiredRightLegAngleAxisOrientation.resize(4, 0.0);

    DesiredRotationMatrixLeftLeg.resize(3,3);
    DesiredRotationMatrixRightLeg.resize(3,3);

}

InertialCompensator::~InertialCompensator() {}

void InertialCompensator::getDesiredFeetInCorrectedBase(                  int SptFt[],
                                                                     Vector7d LLegPose,
                                                                     Vector7d RLegPose,
                                                                     Vector3d m_orientation_rpy,
                                                        CpGaitTransformations *GTj)

{
    // Transformation from the Base to the IMU frame.
    Eigen::Vector3d roll_pitch_Wld(3);
    roll_pitch_Wld(0) = m_orientation_rpy(0)/180.0* M_PI;
    roll_pitch_Wld(1) = m_orientation_rpy(1)/180.0* M_PI;
    roll_pitch_Wld(2) = 0.0;

    // rotation of the of the IMU with respect to the world with the yaw angle removed
    TrsfCurBaseInWorldIMU.block<3,3>(0,0) = getComposedEulerOriention(roll_pitch_Wld);
  
    // Building the 4X4 Homogeneous transformation matrix for the legs end effectors
    FwKLeftFootInBase(0,3) = LLegPose(0);
    FwKLeftFootInBase(1,3) = LLegPose(1);
    FwKLeftFootInBase(2,3) = LLegPose(2);

    FwKRightFootInBase(0,3) = RLegPose(0);
    FwKRightFootInBase(1,3) = RLegPose(1);
    FwKRightFootInBase(2,3) = RLegPose(2);

    // Create a yarp matrix for the output of the yarp::math::axis2dcm method
    FwKLeftFootInBase.block<3,3>(0,0)  = Eigen::AngleAxisd(LLegPose(6), LLegPose.segment(3,3)).toRotationMatrix();
    FwKRightFootInBase.block<3,3>(0,0) = Eigen::AngleAxisd(RLegPose(6), RLegPose.segment(3,3)).toRotationMatrix();

    // Expressing the orientation of the world frame (IMU) in the base frame
    Matrix4d    TrsfWorldIMUInCurBase = Eigen::MatrixXd::Identity(4,4);
                TrsfWorldIMUInCurBase.block(0,0, 3,3) = (TrsfCurBaseInWorldIMU.block(0,0, 3,3)).transpose();

    cout << " TrsfCurBaseInWorldIMU is :\n"<< TrsfCurBaseInWorldIMU << endl;

    // Compute the current stance foot in the base frame
    Matrix3d Rot_Base2BaseFoot = Eigen::MatrixXd::Zero(3,3);
    Rot_Base2BaseFoot(2,0) = -1.0;
    Rot_Base2BaseFoot(1,1) =  1.0;
    Rot_Base2BaseFoot(0,2) =  1.0;

    //
    Matrix4d StanceFootInCurBase;

    if (SptFt[0] == 1)  // left stance foot
    {
        StanceFootInCurBase             = FwKLeftFootInBase;
        ReferenceTrsfHorFootInHorBase   = GTj->TrsfLeftFootBase;
        ReferenceTrsfSwingFootInHorBase = GTj->TrsfRightFootBase;
        //
        CurTrsfRealStanceFootInHorBase = TrsfCurBaseInWorldIMU * StanceFootInCurBase;     // compute TrsfCurBaseInWorldIMU
        // extracting the rotation matrix
        CurRotRealStFootInHorBase = CurTrsfRealStanceFootInHorBase.block(0, 0, 3, 3);
        // orientation of the current stance foot wrt. the horizontal plane
        Vector3d EulerXYZ_F_hB, OrientXY_F_hF;
        EulerXYZ_F_hB    = getEulerAnglesXYZ_FixedFrame(Rot_Base2BaseFoot * CurRotRealStFootInHorBase);         
        OrientXY_F_hF    = EulerXYZ_F_hB;
        OrientXY_F_hF(2) = 0.0;

        RotRealStFootInHorStFoot = getComposedOrientFixedFrame(OrientXY_F_hF);

        // Desired Transformation of the real stance foot expressed in the Horizon base
        // Trsf form real stance foot in horizontal stance foot frame
        Matrix4d    T_RealStFootInHorStFoot = Eigen::MatrixXd::Identity(4,4);
                    T_RealStFootInHorStFoot.block(0,0, 3,3) = RotRealStFootInHorStFoot;

        DesTrsfRealStanceFootInHorBase = ReferenceTrsfHorFootInHorBase * T_RealStFootInHorStFoot;
        // Corrected reference for the swing leg
        DesTrsfSwingFootInCurBase = TrsfWorldIMUInCurBase * ReferenceTrsfSwingFootInHorBase;
        DesTrsfLeftFootInHorBase  = ReferenceTrsfHorFootInHorBase;
        DesTrsfRightFootInHorBase = TrsfCurBaseInWorldIMU * ReferenceTrsfSwingFootInHorBase;
    }
    else   // the right foot is assumed to be the stance foot
    {
        StanceFootInCurBase             = FwKRightFootInBase;
        ReferenceTrsfHorFootInHorBase   = GTj->TrsfRightFootBase;
        ReferenceTrsfSwingFootInHorBase = GTj->TrsfLeftFootBase;
        //
        CurTrsfRealStanceFootInHorBase = TrsfCurBaseInWorldIMU * StanceFootInCurBase;
        // extracting the rotation matrix
        CurRotRealStFootInHorBase = CurTrsfRealStanceFootInHorBase.block(0, 0, 3, 3);

        // orientation of the current stance foot wrt. the horizontal plane
        Vector3d EulerXYZ_F_hB, OrientXY_F_hF;
        EulerXYZ_F_hB = getEulerAnglesXYZ_FixedFrame(Rot_Base2BaseFoot * CurRotRealStFootInHorBase);

        OrientXY_F_hF    = EulerXYZ_F_hB;
        OrientXY_F_hF(2) = 0.0;

        RotRealStFootInHorStFoot = getComposedOrientFixedFrame(OrientXY_F_hF);

        // Desired Transformation of the real stance foot expressed in the Horizon base
        // Trsf form real stance foot in horizontal stance foot frame
        Matrix4d    T_RealStFootInHorStFoot = Eigen::MatrixXd::Identity(4,4);
                    T_RealStFootInHorStFoot.block(0,0, 3,3) = RotRealStFootInHorStFoot;

        DesTrsfRealStanceFootInHorBase = ReferenceTrsfHorFootInHorBase * T_RealStFootInHorStFoot;
        // corrected reference for the swing
        DesTrsfSwingFootInCurBase = TrsfWorldIMUInCurBase * ReferenceTrsfSwingFootInHorBase;
        DesTrsfLeftFootInHorBase  = TrsfCurBaseInWorldIMU * ReferenceTrsfSwingFootInHorBase;
        DesTrsfRightFootInHorBase = ReferenceTrsfHorFootInHorBase;
    }
}

void InertialCompensator::SetImuTransformation(Vector3d IMU_RollPitchYaw)
{
    Matrix4d T_B_IMU = Eigen::MatrixXd::Identity(4,4);

    // Homogeneous transformation expressin the rotation of the base with respect the IMU_intertial frame
    Matrix3d base_in_imu =    (Eigen::AngleAxisd(IMU_RollPitchYaw(2), Vector3d::UnitZ())
                                * Eigen::AngleAxisd(IMU_RollPitchYaw(1), Vector3d::UnitY())
                                * Eigen::AngleAxisd(IMU_RollPitchYaw(0), Vector3d::UnitX())).matrix();
    TrsfCurBaseInWorldIMU.block<3,3>(0,0) = base_in_imu;
}


Vector3d InertialCompensator::getEulerAnglesXYZ_FixedFrame(Matrix3d R)
{
    // this function computed for a given rotation matrix the rotation angles around X, Y and Z axis considered as fixed.
    // the rotation matrix is assumed to be a Euler rotation matrix of type ZYX
    Vector3d Angles;
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

Matrix3d InertialCompensator::getComposedOrientFixedFrame(Vector3d OrientXY_F_hF)
{
    // this function computes the Euler composition rotation matrix for rotation angles given wrt. fixed frame
    Matrix3d EulerXYZ;
    EulerXYZ.setZero(3,3);

    // OrientXY_F_hF(0): Psi_X
    // OrientXY_F_hF(1): Theta_Y
    // OrientXY_F_hF(2): Phi_Z

    EulerXYZ(0,0) =  cos(OrientXY_F_hF(2))*cos(OrientXY_F_hF(1));
    EulerXYZ(1,0) =  sin(OrientXY_F_hF(2))*cos(OrientXY_F_hF(1));
    EulerXYZ(2,0) = -sin(OrientXY_F_hF(1));

    EulerXYZ(0,1) = -sin(OrientXY_F_hF(2))*cos(OrientXY_F_hF(0))+cos(OrientXY_F_hF(2))*sin(OrientXY_F_hF(1))*sin(OrientXY_F_hF(0));
    EulerXYZ(1,1) =  cos(OrientXY_F_hF(2))*cos(OrientXY_F_hF(0))+sin(OrientXY_F_hF(2))*sin(OrientXY_F_hF(1))*sin(OrientXY_F_hF(0));
    EulerXYZ(2,1) =  cos(OrientXY_F_hF(1))*sin(OrientXY_F_hF(0));

    EulerXYZ(0,2) =  sin(OrientXY_F_hF(2))*sin(OrientXY_F_hF(0))+cos(OrientXY_F_hF(2))*sin(OrientXY_F_hF(1))*cos(OrientXY_F_hF(0));
    EulerXYZ(1,2) = -cos(OrientXY_F_hF(2))*sin(OrientXY_F_hF(0))+sin(OrientXY_F_hF(2))*sin(OrientXY_F_hF(1))*cos(OrientXY_F_hF(0));
    EulerXYZ(2,2) =  cos(OrientXY_F_hF(1))*cos(OrientXY_F_hF(0));

    return EulerXYZ;
}

Matrix3d InertialCompensator::getComposedEulerOriention(Vector3d a)
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

void InertialCompensator::CompensateTransformsWithIMU(CpGaitTransformations *GTj,
                                                          Vector7d lleg_EE_pose,
                                                          Vector7d rleg_EE_pose,
                                                          Vector3d m_orientation_rpy,  // filtered
                                                                        int Stance[],
                                                                         bool isActive)
{
    if (!isActive)
    {

        // translation
        // left leg
        DesiredLeftLegPoseAsAxisAngles(0)  = GTj->TrsfLeftFootBase(0,3);
        DesiredLeftLegPoseAsAxisAngles(1)  = GTj->TrsfLeftFootBase(1,3);
        DesiredLeftLegPoseAsAxisAngles(2)  = GTj->TrsfLeftFootBase(2,3);
        // right leg
        DesiredRightLegPoseAsAxisAngles(0) = GTj->TrsfRightFootBase(0,3);
        DesiredRightLegPoseAsAxisAngles(1) = GTj->TrsfRightFootBase(1,3);
        DesiredRightLegPoseAsAxisAngles(2) = GTj->TrsfRightFootBase(2,3);
        // rotation
        DesiredRotationMatrixLeftLeg  = GTj->TrsfLeftFootBase.block<3,3>(0,0);   // left
        DesiredRotationMatrixRightLeg = GTj->TrsfRightFootBase.block<3,3>(0,0);  // right
 
        // left: conversion of rotation matrix into an axis/angle
        Eigen::AngleAxisd d_lleg_o = Eigen::AngleAxisd(DesiredRotationMatrixLeftLeg);
        DesiredLeftLegPoseAsAxisAngles.segment(0,3) = d_lleg_o.axis();
        DesiredLeftLegPoseAsAxisAngles(3)           = d_lleg_o.angle();

        // right: conversion of rotation matrix into an axis/angle
        Eigen::AngleAxisd d_rleg_o = Eigen::AngleAxisd(DesiredRotationMatrixRightLeg);
        DesiredRightLegPoseAsAxisAngles.segment(0,3) = d_rleg_o.axis();
        DesiredRightLegPoseAsAxisAngles(3)           = d_rleg_o.angle();
    }
    else
    {
        this->getDesiredFeetInCorrectedBase(   Stance,              // Parameters->StanceIndicator, 
                                                lleg_EE_pose,       // LeftLegChain->EndEffPose(), 
                                                rleg_EE_pose,       // RightLegChain->EndEffPose(),                                                      
                                                m_orientation_rpy,  // m_orientation_rpy, 
                                                GTj);
        // translation
        // left leg
        DesiredLeftLegPoseAsAxisAngles(0)  = this->DesTrsfLeftFootInHorBase(0,3);
        DesiredLeftLegPoseAsAxisAngles(1)  = this->DesTrsfLeftFootInHorBase(1,3);
        DesiredLeftLegPoseAsAxisAngles(2)  = this->DesTrsfLeftFootInHorBase(2,3);
        // right leg
        DesiredRightLegPoseAsAxisAngles(0) = this->DesTrsfRightFootInHorBase(0,3);
        DesiredRightLegPoseAsAxisAngles(1) = this->DesTrsfRightFootInHorBase(1,3);
        DesiredRightLegPoseAsAxisAngles(2) = this->DesTrsfRightFootInHorBase(2,3);
        // rotation
        DesiredRotationMatrixLeftLeg  = this->DesTrsfLeftFootInHorBase.block<3,3>(0,0);     // left
        DesiredRotationMatrixRightLeg = this->DesTrsfRightFootInHorBase.block<3,3>(0,0);    // right
      
        // left: conversion of rotation matrix into an axis/angle
        // left: conversion of rotation matrix into an axis/angle
        Eigen::AngleAxisd d_lleg_o = Eigen::AngleAxisd(DesiredRotationMatrixLeftLeg);
        DesiredLeftLegPoseAsAxisAngles.segment(0,3) = d_lleg_o.axis();
        DesiredLeftLegPoseAsAxisAngles(3)           = d_lleg_o.angle();

        // right: conversion of rotation matrix into an axis/angle
        Eigen::AngleAxisd d_rleg_o = Eigen::AngleAxisd(DesiredRotationMatrixRightLeg);
        DesiredRightLegPoseAsAxisAngles.segment(0,3) = d_rleg_o.axis();
        DesiredRightLegPoseAsAxisAngles(3)           = d_rleg_o.angle();

    }
}