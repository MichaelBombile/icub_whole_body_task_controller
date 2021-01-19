
// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <stdexcept>
#include <iomanip>
#include <cmath>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>

#include <cstdlib>

#include "WbRobotModel.h"

using namespace yarp::dev;


//
WbRobotModel::WbRobotModel(std::string robotName, RobotInterface& robot_interface_) : RobotName(robotName)
                                                                                    , robot_interface(robot_interface_){}


WbRobotModel::~WbRobotModel()
{
    // // close the device
    // robot_interface.CloseWholeBodyRobotDevices();
}

int WbRobotModel::getDoFs() {
    return actuatedDofs;
}

// initialization of WbRobotModel
bool WbRobotModel::init(std::string moduleName_, std::string modelFile_, std::vector<std::string> list_of_considered_joints, RobotInterface& robot_interface_)
{
    //
    this->ModuleName  = moduleName_;
    //
    DEG2RAD = M_PI/180.0;
    RAD2DEG = 180.0/M_PI;

    //
    bool ok = mdlLoader.loadReducedModelFromFile(modelFile_, list_of_considered_joints);
    if( !ok ){
        std::cerr << "KinDynComputationsWithEigen: impossible to load reduced model of the robot model in a KinDynComputations class:" << std::endl
                  << mdlLoader.model().toString() << std::endl;
        return EXIT_FAILURE;
    }

    // Initialize the KinDynComputations object
    ok = m_kinDynComp.loadRobotModel(mdlLoader.model());
    if( !ok )
    {
        std::cerr << "KinDynComputationsWithEigen: impossible to load the following model in a KinDynComputations class:" << std::endl
                  << mdlLoader.model().toString() << std::endl;
        return EXIT_FAILURE;
    }
    //
    m_genTrqs.resize(m_kinDynComp.model());
    m_genGravityTrqs.resize(m_kinDynComp.model());
    idyn_m_massMatrix.resize(m_kinDynComp.model());
    m_frameJacobian.resize(m_kinDynComp.model());
    m_momentumJacobian.resize(m_kinDynComp.model());
    m_comJacobian.resize(3,m_kinDynComp.getNrOfDegreesOfFreedom()+6);
    //
    idynRobotState.resize(m_kinDynComp.getNrOfDegreesOfFreedom());
    eigRobotState.resize(m_kinDynComp.getNrOfDegreesOfFreedom());
    eigRobotAccel.resize(m_kinDynComp.getNrOfDegreesOfFreedom());
    //
    actuatedDofs = m_kinDynComp.getNrOfDegreesOfFreedom();
    //
    m_jts_sensor_wb.position.resize(m_kinDynComp.getNrOfDegreesOfFreedom());
    m_jts_sensor_wb.velocity.resize(m_kinDynComp.getNrOfDegreesOfFreedom());
    m_jts_sensor_wb.acceleration.resize(m_kinDynComp.getNrOfDegreesOfFreedom());
    m_jts_sensor_wb.torque.resize(m_kinDynComp.getNrOfDegreesOfFreedom());
    //
    m_min_jtsLimits.resize(m_kinDynComp.getNrOfDegreesOfFreedom());
    m_max_jtsLimits.resize(m_kinDynComp.getNrOfDegreesOfFreedom());
    m_velocitySaturationLimit.resize(m_kinDynComp.getNrOfDegreesOfFreedom());
    m_torqueSaturationLimit.resize(m_kinDynComp.getNrOfDegreesOfFreedom());

    // Interface with the robot
    // ===========================================
    // get the joint limits
    if (!(this->getJointsLimits(robot_interface_) )) {
        printf("Failed to get joint limits. \n");
        return false;
    }

    //
    //
    fBasePoseEstimate.setZero();
	filtered_fBasePoseEstimate.setZero();
	dot_fBasePoseEstimate.setZero();

    // Update the robot model
    // ---------------------------------
    std::string stanceFoot = "left";

    if(!this->UpdateWholeBodyModel_init(robot_interface_, "left")){
        printf("Failed to update the robot model \n");
        return false;
    }

    fBasePoseEstimate = this->get_fBasePoseEstimate(eigRobotState.world_H_base);
    // Initialize the filter
    filter_gain_fBase = 5.0;  // pole of the filter
    fBaseFilter.InitializeFilter(robot_interface_.run_period_sec, filter_gain_fBase, filter_gain_fBase, fBasePoseEstimate);

    return true;
}


// Update the robot states
bool WbRobotModel::UpdateModelStates()
{
    // Now we convert the Eigen data structures in iDynTree ones
    iDynTree::toEigen(idynRobotState.gravity)   =       eigRobotState.gravity;    
    iDynTree::toEigen(idynRobotState.jointPos)  =       eigRobotState.jointPos;    
    iDynTree::toEigen(idynRobotState.jointVel)  =       eigRobotState.jointVel;

    iDynTree::fromEigen(idynRobotState.world_H_base,    eigRobotState.world_H_base);
    iDynTree::fromEigen(idynRobotState.baseVel,         eigRobotState.baseVel);
    

    // Now we create the KinDynComputations class, so we can set the state
    m_kinDynComp.setRobotState( idynRobotState.world_H_base, 
                                idynRobotState.jointPos, 
                                idynRobotState.baseVel, 
                                idynRobotState.jointVel, 
                                idynRobotState.gravity);

    return true;
}
//
bool WbRobotModel::computeMassMatrix(Eigen::VectorXd jts_pos, Matrix4d w_H_B_, double * MassMatrix_)
{
    // update the dynamic model
    // ----------------------------------------------------------------------------------------
    // Update the Eigen States for the model
        Eigen::VectorXd jts_velo(m_kinDynComp.getNrOfDegreesOfFreedom());   jts_velo.setZero();
        Vector6d VB_;   VB_.setZero();

        m_mutex.lock();
            eigRobotState.update(jts_pos, jts_velo,  w_H_B_, VB_);
            this->UpdateModelStates();  // update the states of the dynamic model
        m_mutex.unlock();
        
    // ----------------------------------------------------------------------------------------
    // compute the mass matrix with the updated dynamic model
    m_kinDynComp.getFreeFloatingMassMatrix(idyn_m_massMatrix);
    // Both iDynTree and WBI store matrices in row-major format, so it is quite trivial to copy them
    Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > outMat(MassMatrix_,idyn_m_massMatrix.rows(),idyn_m_massMatrix.cols());

    outMat = iDynTree::toEigen(idyn_m_massMatrix);

    return true;
}

// Compute the Generalized bias torques (Coriolis and Gravity)
bool WbRobotModel::computeGeneralizedBiasForces(Eigen::VectorXd jts_pos, Matrix4d w_H_B_, Eigen::VectorXd jts_velo, Vector6d VB_, double * Gen_bias_torques)
{
    // update the dynamic model
    // ----------------------------------------------------------------------------------------
    // Update the Eigen States for the model
        m_mutex.lock();
            eigRobotState.update(jts_pos, jts_velo,  w_H_B_, VB_);
            this->UpdateModelStates();  // update the states of the dynamic model
        m_mutex.unlock();
        
    // ----------------------------------------------------------------------------------------
    //Computing specialized inverse dynamics
    m_kinDynComp.generalizedBiasForces(m_genTrqs);
    // Gen_bias_torques = iDynTree::toEigen(m_genTrqs);

    Eigen::Map< Eigen::VectorXd > outGenTrqs(Gen_bias_torques, m_genTrqs.jointTorques().size()+6);

    outGenTrqs.segment(0,6) = iDynTree::toEigen(m_genTrqs.baseWrench());
    outGenTrqs.segment(6, m_genTrqs.jointTorques().size()) = iDynTree::toEigen(m_genTrqs.jointTorques());

    return true;
}

// Compute the Generalized Gravity torques
bool WbRobotModel::computeGravityTorques(Eigen::VectorXd jts_pos, Matrix4d w_H_B_, Eigen::VectorXd jts_velo, Vector6d VB_, double * Gen_Gravity_torques)
{
    // update the dynamic model
    // ----------------------------------------------------------------------------------------
    // Update the Eigen States for the model
    // 
    m_mutex.lock();
        eigRobotState.update(jts_pos, jts_velo,  w_H_B_, VB_);
        this->UpdateModelStates();  // update the states of the dynamic model
    m_mutex.unlock();
        
    // ----------------------------------------------------------------------------------------
    //Computing specialized inverse dynamics
    m_kinDynComp.generalizedGravityForces(m_genGravityTrqs);
    // Gen_Gravity_torques = iDynTree::toEigen(m_genGravityTrqs);
    Eigen::Map< Eigen::VectorXd > outGenTrqs(Gen_Gravity_torques, m_genGravityTrqs.jointTorques().size()+6);

    outGenTrqs.segment(0,6) = iDynTree::toEigen(m_genGravityTrqs.baseWrench());
    outGenTrqs.segment(6, m_genGravityTrqs.jointTorques().size()) = iDynTree::toEigen(m_genGravityTrqs.jointTorques());

    return true;
}

// Compute the Free floating base Jacobian Matrix of a frame
bool WbRobotModel::computeJacobian(Eigen::VectorXd jts_pos, Matrix4d w_H_B_, iDynTree::FrameIndex arbitraryFrameIndex,  double * Jacobian)
{
    // update the dynamic model
    // ----------------------------------------------------------------------------------------
    // Update the Eigen States for the model
        Eigen::VectorXd jts_velo(m_kinDynComp.getNrOfDegreesOfFreedom());   jts_velo.setZero();
        Vector6d VB_;   VB_.setZero();
        // 
        m_mutex.lock();
            eigRobotState.update(jts_pos, jts_velo,  w_H_B_, VB_);
            this->UpdateModelStates();  // update the states of the dynamic model
        m_mutex.unlock();
        
    // ----------------------------------------------------------------------------------------
    //Computing specialized inverse dynamics
    m_kinDynComp.getFrameFreeFloatingJacobian(arbitraryFrameIndex, m_frameJacobian);
    // Both iDynTree and WBI store matrices in row-major format, so it is quite trivial to copy them
    Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > outMat(Jacobian, m_frameJacobian.rows(), m_frameJacobian.cols());

    outMat = iDynTree::toEigen(m_frameJacobian);

    return true;
}

// Compute the Free floating base Jacobian Matrix of the Centre of mass
bool WbRobotModel::computeCoMJacobian(Eigen::VectorXd jts_pos, Matrix4d w_H_B_, double * CoM_Jacobian)
{
    // update the dynamic model
    // ----------------------------------------------------------------------------------------
    // Update the Eigen States for the model
        Eigen::VectorXd jts_velo(m_kinDynComp.getNrOfDegreesOfFreedom());   jts_velo.setZero();
        Vector6d VB_;   VB_.setZero();
        // 
        m_mutex.lock();
            eigRobotState.update(jts_pos, jts_velo,  w_H_B_, VB_);
            this->UpdateModelStates();  // update the states of the dynamic model
        m_mutex.unlock();
        
    // ----------------------------------------------------------------------------------------
    //Computing specialized inverse dynamics
    m_kinDynComp.getCenterOfMassJacobian(m_comJacobian);
    // Both iDynTree and WBI store matrices in row-major format, so it is quite trivial to copy them
    Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > outMat(CoM_Jacobian,m_comJacobian.rows(),m_comJacobian.cols());

    outMat = iDynTree::toEigen(m_comJacobian);

    return true;
}

// Compute the bias acceleration of a frame dJdq
bool WbRobotModel::computeDJdq(Eigen::VectorXd jts_pos, Matrix4d w_H_B_, Eigen::VectorXd jts_velo, Vector6d VB_, iDynTree::FrameIndex arbitraryFrameIndex, double *Bias_acceleration)
{
    // update the dynamic model
    // ----------------------------------------------------------------------------------------
    // Update the Eigen States for the model
        // 
        m_mutex.lock();
            eigRobotState.update(jts_pos, jts_velo,  w_H_B_, VB_);
            this->UpdateModelStates();  // update the states of the dynamic model
        m_mutex.unlock();
        
    // ----------------------------------------------------------------------------------------
    iDynTree::Vector6 Frame_biasAcc = m_kinDynComp.getFrameBiasAcc(arbitraryFrameIndex);

    Eigen::Map< Eigen::VectorXd > outGenTrqs(Bias_acceleration, Frame_biasAcc.size());
    outGenTrqs = iDynTree::toEigen(Frame_biasAcc);
    return true;
}

// Compute the bias acceleration of a frame dJdq of the Centre of mass
bool WbRobotModel::computeCoMDJdq(Eigen::VectorXd jts_pos, Matrix4d w_H_B_, Eigen::VectorXd jts_velo, Vector6d VB_, double *CoM_Bias_acceleration)
{
    // update the dynamic model
    // ----------------------------------------------------------------------------------------
    // Update the Eigen States for the model
        // 
    m_mutex.lock();
        eigRobotState.update(jts_pos, jts_velo,  w_H_B_, VB_);
        this->UpdateModelStates();  // update the states of the dynamic model
    m_mutex.unlock();
        
    // ----------------------------------------------------------------------------------------
    iDynTree::Vector3 CoM_biasAcc = m_kinDynComp.getCenterOfMassBiasAcc();
    Eigen::Map< Eigen::VectorXd > outGenTrqs(CoM_Bias_acceleration, CoM_biasAcc.size());
    outGenTrqs = iDynTree::toEigen(CoM_biasAcc);
    return true;
}


bool WbRobotModel::forwardKinematics(Eigen::VectorXd jts_pos, Matrix4d w_H_B_, iDynTree::FrameIndex arbitraryFrameIndex, Eigen::Matrix<double, 7,1> &Frame_Pose, Eigen::Matrix4d &world_HmgTransf_Frame)
{
    // update the dynamic model
    // ----------------------------------------------------------------------------------------
    // Update the Eigen States for the model
        Eigen::VectorXd jts_velo(m_kinDynComp.getNrOfDegreesOfFreedom());   jts_velo.setZero();
        Vector6d VB_;   VB_.setZero();
        // 
        m_mutex.lock();
            eigRobotState.update(jts_pos, jts_velo,  w_H_B_, VB_);
            this->UpdateModelStates();  // update the states of the dynamic model
        m_mutex.unlock();
       
    // ----------------------------------------------------------------------------------------
    // Cast the frameId  to a iDynTree::FrameIndex
    // iDynTree::FrameIndex frameIndex = static_cast<iDynTree::FrameIndex>(frameId);
    world_HmgTransf_Frame = iDynTree::toEigen(m_kinDynComp.getWorldTransform(arbitraryFrameIndex).asHomogeneousTransform());
    //
    Frame_Pose.head<3>() = world_HmgTransf_Frame.block<3,1>(0,3);
    Eigen::AngleAxisd d_orientation_o; 
    d_orientation_o = Eigen::AngleAxisd(world_HmgTransf_Frame.block<3,3>(0,0));

    Frame_Pose.segment<3>(3) = d_orientation_o.axis();
    Frame_Pose.tail<1>() << d_orientation_o.angle();

    return true;
}

bool WbRobotModel::forwardKinematicsCoM(Eigen::VectorXd jts_pos, Matrix4d w_H_B_, Eigen::Matrix<double, 7,1> &CoM_Pose, Eigen::Matrix4d &world_HmgTransf_CoM)
{
    // update the dynamic model
    // ----------------------------------------------------------------------------------------
    // Update the Eigen States for the model
        Eigen::VectorXd jts_velo(m_kinDynComp.getNrOfDegreesOfFreedom());   jts_velo.setZero();
        Vector6d VB_;   VB_.setZero();
        // 
        m_mutex.lock();
            eigRobotState.update(jts_pos, jts_velo,  w_H_B_, VB_);
            this->UpdateModelStates();  // update the states of the dynamic model
        m_mutex.unlock();
        
    // ----------------------------------------------------------------------------------------
    // Cast the frameId  to a iDynTree::FrameIndex
    // iDynTree::FrameIndex frameIndex = static_cast<iDynTree::FrameIndex>(frameId);
    //
    iDynTree::Transform world_H_frame = iDynTree::Transform::Identity();
    //
    world_H_frame.setPosition(m_kinDynComp.getCenterOfMassPosition());
    iDynTree::Matrix4x4 homMat = world_H_frame.asHomogeneousTransform();
    world_HmgTransf_CoM = iDynTree::toEigen(homMat);
    //
    CoM_Pose.head<3>() = world_HmgTransf_CoM.block<3,1>(0,3);
    Eigen::AngleAxisd d_orientation_o; 
    d_orientation_o = Eigen::AngleAxisd(world_HmgTransf_CoM.block<3,3>(0,0));

    CoM_Pose.segment<3>(3) = d_orientation_o.axis();
    CoM_Pose.tail<1>() << d_orientation_o.angle();

    return true;
}
//
bool WbRobotModel::compute_transform_CoM_Base(Eigen::VectorXd jts_pos, Matrix4d w_H_B_, Eigen::Matrix<double, 6,6> &CoM_X_Base)
{
    //
    Vector7d w_CoM_pose, w_Base_pose;
    Eigen::Matrix<double, 4,4> w_H_link;
    // Pose of the CoM frame
    this->forwardKinematicsCoM(jts_pos, w_H_B_, w_CoM_pose, w_H_link);
    // Pose of the Base (root) frame
    iDynTree::FrameIndex root_frameIdx;
    this->getFrame_Index("root_link", root_frameIdx); 
    this->forwardKinematics(jts_pos, w_H_B_, root_frameIdx, w_Base_pose, w_H_link);
    //
    Eigen::Matrix3d Skew_Base_CoM;  Skew_Base_CoM.setZero();
    CoM_X_Base.setZero();
    // compute the skew symmetry matrix
    Skew_Base_CoM = Transforms.ComputeSkewSymmetricMatrix(w_CoM_pose.segment<3>(0) - w_Base_pose.segment<3>(0));
    //  compute the transformation
    CoM_X_Base.block<3,3>(0,0) = Eigen::MatrixXd::Identity(3, 3);
    CoM_X_Base.block<3,3>(3,3) = Eigen::MatrixXd::Identity(3, 3);
    CoM_X_Base.block<3,3>(0,3) = -Skew_Base_CoM;

    return true;
}
//




bool WbRobotModel::getFrame_Index(std::string frame_name, iDynTree::FrameIndex &frameIdx)
{
    frameIdx = m_kinDynComp.getFrameIndex(frame_name);

    return true;
}

bool WbRobotModel::getFrameName(iDynTree::FrameIndex frameIdx, std::string &frame_name)
{

    frame_name = m_kinDynComp.model().getFrameName(frameIdx);

    return true;
}


bool WbRobotModel::WbRobotModel::getMassMatrix(Eigen::VectorXd jts_pos, Matrix4d w_H_B_, Eigen::MatrixXd&  mass_matrix)
{
    //
    this->computeMassMatrix(jts_pos, w_H_B_, mass_matrix.data());
    return true;
}

bool WbRobotModel::getInverseMassMatrix(Eigen::VectorXd jts_pos, Matrix4d w_H_B_, Eigen::MatrixXd&  inv_mass_matrix)
{
    //
    Eigen::MatrixXd  mass_matrix;
    this->computeMassMatrix(jts_pos, w_H_B_, mass_matrix.data());
    //
    inv_mass_matrix = mass_matrix.inverse();

    return true;
}

bool WbRobotModel::WbRobotModel::getGeneralizedBiasForces(Eigen::VectorXd jts_pos, Matrix4d w_H_B_, Eigen::VectorXd jts_velo, Vector6d VB_, Eigen::VectorXd& Gen_biasForces)
{
    this->computeGeneralizedBiasForces(jts_pos, w_H_B_, jts_velo, VB_, Gen_biasForces.data());
    return true;
}

bool WbRobotModel::WbRobotModel::getGeneralizedGravityForces(Eigen::VectorXd jts_pos, Matrix4d w_H_B_, Eigen::VectorXd jts_velo, Vector6d VB_, Eigen::VectorXd& Gen_gravityTorques)
{
    this->computeGravityTorques(jts_pos, w_H_B_, jts_velo, VB_, Gen_gravityTorques.data());
    return true;
}

//
bool WbRobotModel::getJacobian(Eigen::VectorXd jts_pos, Matrix4d w_H_B_, std::string frame_name, MatrixXdRowMaj& Jacobian)
{
    // Index of frames to consider
    iDynTree::FrameIndex frameIdx_;
    // get the indexes of the considered frame
    std::string frame_id_;
    //
    Eigen::Matrix<double, 6,6> CoM_X_Base;
    
    if(frame_name == "CoM")
    {
        this->computeCoMJacobian(jts_pos, w_H_B_, Jacobian.data());
    }
    else if(frame_name == "centroidal_momentum")
    {
        Eigen::MatrixXd mass_matrix(actuatedDofs+6, actuatedDofs+6);
        this->computeMassMatrix(jts_pos, w_H_B_, mass_matrix.data());
        this->compute_transform_CoM_Base(jts_pos, w_H_B_, CoM_X_Base);
        Jacobian = CoM_X_Base.inverse().transpose() * mass_matrix.topRows(6);
    }
    else
    {
        //
        if(frame_name == "left_hand")           frame_id_ = "l_hand";
        else if(frame_name == "right_hand")     frame_id_ = "r_hand";
        else if(frame_name == "left_foot")      frame_id_ = "l_sole";
        else if(frame_name == "right_foot")     frame_id_ = "r_sole";
        else if(frame_name == "Chest")          frame_id_ = "chest";
        else if(frame_name == "Pelvis")         frame_id_ = "root_link";

        //
        this->getFrame_Index(frame_id_,    frameIdx_);
        //
        this->computeJacobian(jts_pos, w_H_B_, frameIdx_,  Jacobian.data());
    }

    //
    return true;
}

bool WbRobotModel::getDJdq(Eigen::VectorXd jts_pos, Matrix4d w_H_B_, Eigen::VectorXd jts_velo, Vector6d VB_, std::string frame_name, Eigen::Matrix<double, 6,1> &DJacobianDq_eef)
{
    // Index of frames to consider
    iDynTree::FrameIndex frameIdx_;
    // get the indexes of the considered frame
    std::string frame_id_;
    //
    Eigen::Matrix<double, 6,6> CoM_X_Base;
    
    if(frame_name == "CoM")
    {
        this->computeCoMDJdq(jts_pos, w_H_B_, jts_velo, VB_, DJacobianDq_eef.data());

    }
    else if(frame_name == "centroidal_momentum")
    {
        //
        Eigen::VectorXd bias_forces(actuatedDofs+6);
        Eigen::VectorXd gravity_bias_torques(actuatedDofs+6);
        //
        this->computeGeneralizedBiasForces(jts_pos, w_H_B_, jts_velo, VB_, bias_forces.data());
        this->computeGravityTorques(jts_pos, w_H_B_, jts_velo, VB_, gravity_bias_torques.data());
        this->compute_transform_CoM_Base(jts_pos, w_H_B_, CoM_X_Base);
        DJacobianDq_eef = CoM_X_Base.inverse().transpose() * (bias_forces.head(6) - gravity_bias_torques.head(6));
    }
    else
    {
        //
        if(frame_name == "left_hand")           frame_id_ = "l_hand";
        else if(frame_name == "right_hand")     frame_id_ = "r_hand";
        else if(frame_name == "left_foot")      frame_id_ = "l_sole";
        else if(frame_name == "right_foot")     frame_id_ = "r_sole";
        else if(frame_name == "Chest")          frame_id_ = "chest";
        else if(frame_name == "Pelvis")         frame_id_ = "root_link";

        //
        this->getFrame_Index(frame_id_,    frameIdx_);
        //
        this->computeDJdq(jts_pos, w_H_B_, jts_velo, VB_, frameIdx_, DJacobianDq_eef.data());
    }

    return true;
}

bool WbRobotModel::getLinkPose(Eigen::VectorXd jts_pos, Matrix4d w_H_B_, std::string frame_name,  Eigen::Matrix<double, 7,1> &frame_pose)
{
    // Index of frames to consider
    iDynTree::FrameIndex frameIdx_;
    // get the indexes of the considered frame
    std::string frame_id_;
    Eigen::Matrix<double, 4,4> w_H_link;
    //  
    if(frame_name == "CoM")
    {
        this->forwardKinematicsCoM(jts_pos, w_H_B_, frame_pose, w_H_link);
    }
    else
    {
        //
        if(frame_name == "left_hand")           frame_id_ = "l_hand";
        else if(frame_name == "right_hand")     frame_id_ = "r_hand";
        else if(frame_name == "left_foot")      frame_id_ = "l_sole";
        else if(frame_name == "right_foot")     frame_id_ = "r_sole";
        else if(frame_name == "Chest")          frame_id_ = "chest";
        else if(frame_name == "Pelvis")         frame_id_ = "root_link";
        else if(frame_name == "l_arm_FT")       frame_id_ = "l_upper_arm";
        else if(frame_name == "r_arm_FT")       frame_id_ = "r_upper_arm";
        //
        this->getFrame_Index(frame_id_,    frameIdx_);
        //
        this->forwardKinematics(jts_pos, w_H_B_, frameIdx_, frame_pose, w_H_link);
    }

    return true;
}

// ---------------------------------------------------------------------------- robot_interface dependent function ------------------
//
// bool WbRobotModel::getEstimate_BaseWorldPose(RobotInterface& robot_interface_, std::string stanceFoot, Eigen::Matrix4d &W_H_B)
// {
//     //
//     // bool isBaseExternal = robot_interface.isBaseExternal;
//     bool isBaseExternal = robot_interface_.isBaseExternal;
//     //
//         eigRobotState.jointPos = m_jts_sensor_wb.position;
//         eigRobotState.jointVel = m_jts_sensor_wb.velocity;

//     Matrix4d world_H_base_;
//     world_H_base_.setIdentity();
        
//     // Estimation of the base pose
//     //-----------------------------
//     //
//     if(isBaseExternal) {// Base pose is provided from exeteroceptio
//         //
//         robot_interface_.get_fBase_HomoTransformation();
//         //
//         W_H_B = robot_interface_.world_H_pelvis;
//         //
//     } 
//     else  
//     {   // base pose is estimated locally
// 	    // this->UpdateModelStates();
//         //  // ===============================================
//         iDynTree::FrameIndex l_foot_FrameIndex;
//         iDynTree::FrameIndex r_foot_FrameIndex;

//         this->getFrame_Index("l_sole",    l_foot_FrameIndex);
//         this->getFrame_Index("r_sole",    r_foot_FrameIndex);

//         Eigen::Matrix4d world_HmgTransf_lf;
//         Eigen::Matrix4d world_HmgTransf_rf;

//         Eigen::Matrix<double, 7,1> lf_Pose;
//         Eigen::Matrix<double, 7,1> rf_Pose;

//         if(stanceFoot != "left")  // right stance foot
//         {
//           this->forwardKinematics(m_jts_sensor_wb.position, world_H_base_, r_foot_FrameIndex, rf_Pose, world_HmgTransf_rf);
//           // W_H_B = world_HmgTransf_rf.inverse();
//           W_H_B.block<3,3>(0,0) =  world_HmgTransf_rf.block<3,3>(0,0).transpose();
//           W_H_B.block<3,1>(0,3) =  -W_H_B.block<3,3>(0,0)*world_HmgTransf_rf.block<3,1>(0,3);
//           W_H_B(3,3) = 1.0;
//         }
//         else
//         {
//           this->forwardKinematics(m_jts_sensor_wb.position, world_H_base_, l_foot_FrameIndex, lf_Pose, world_HmgTransf_lf);
//           // W_H_B = world_HmgTransf_lf.inverse();
//           W_H_B.block<3,3>(0,0) =  world_HmgTransf_lf.block<3,3>(0,0).transpose();
//           W_H_B.block<3,1>(0,3) =  -W_H_B.block<3,3>(0,0)*world_HmgTransf_lf.block<3,1>(0,3);
//           W_H_B(3,3) = 1.0;
//         }
//     }
//     // Update the model state with the estimate
//     eigRobotState.world_H_base = W_H_B;

//     return true;
// }
// //
// bool WbRobotModel::getEstimate_BaseWorldPose(RobotInterface& robot_interface_, Matrix4d w_H_lf, Matrix4d w_H_rf, std::string stanceFoot, Eigen::Matrix4d &W_H_B)
// {
//     // bool isBaseExternal = robot_interface.isBaseExternal;
//     bool isBaseExternal = robot_interface_.isBaseExternal;
//     //
//         eigRobotState.jointPos = m_jts_sensor_wb.position;
//         eigRobotState.jointVel = m_jts_sensor_wb.velocity;

//     Matrix4d world_H_base_;
//     world_H_base_.setIdentity();
//     Matrix4d W_H_sft;
//     double  roll  = 0.0;
//     double  pitch = 0.0;
//     // Estimation of the base pose
//     //-----------------------------
//     if(isBaseExternal) {// Base pose is provided from exeteroceptio
//         //
//         robot_interface_.get_fBase_HomoTransformation();
//         //
//         W_H_B = robot_interface_.world_H_pelvis;
//         //
//     } 
//     else  
//     {   // base pose is estimated locally
//         // this->UpdateModelStates();
//         //  // ===============================================
//         iDynTree::FrameIndex l_foot_FrameIndex;
//         iDynTree::FrameIndex r_foot_FrameIndex;

//         this->getFrame_Index("l_sole",    l_foot_FrameIndex);
//         this->getFrame_Index("r_sole",    r_foot_FrameIndex);

//         Eigen::Matrix4d world_HmgTransf_stf;
//         // Eigen::Matrix4d world_HmgTransf_rf;

//         Eigen::Matrix<double, 7,1> lf_Pose;
//         Eigen::Matrix<double, 7,1> rf_Pose;

//         if(this->RobotName == "icub"){
//             double roll  = atan2(robot_interface_.R(2,1), robot_interface_.R(2,2));
//             double pitch = -asin(robot_interface_.R(2,0));
//         }

//         if(stanceFoot != "left")  // right stance foot
//         {
//           this->forwardKinematics(m_jts_sensor_wb.position, world_H_base_, r_foot_FrameIndex, rf_Pose, world_HmgTransf_stf);
//           W_H_B = MatrixXd::Identity(4,4);
//           W_H_sft = w_H_rf;
//         }
//         else
//         {
//           this->forwardKinematics(m_jts_sensor_wb.position, world_H_base_, l_foot_FrameIndex, lf_Pose, world_HmgTransf_stf);
//           W_H_B = MatrixXd::Identity(4,4);
//           W_H_sft = w_H_lf;
//         }
//         //
//         W_H_B =  W_H_sft * world_HmgTransf_stf.inverse();
//     }
//     // Update the model state with the estimate
//     eigRobotState.world_H_base = W_H_B;

//     return true;
// }

bool WbRobotModel::getEstimate_BaseWorldPose(RobotInterface& robot_interface_, std::string stanceFoot, Eigen::Matrix4d &W_H_B)
{
    //
    // bool isBaseExternal = robot_interface.isBaseExternal;
    bool isBaseExternal = robot_interface_.isBaseExternal;
    //
        eigRobotState.jointPos = m_jts_sensor_wb.position;
        eigRobotState.jointVel = m_jts_sensor_wb.velocity;

    Matrix4d world_H_base_;
    world_H_base_.setIdentity();
    double roll	 = 0.0;	double pitch = 0.0; double yaw	 = 0.0;
        
    // Estimation of the base pose
    //-----------------------------
    //  // ===============================================
    iDynTree::FrameIndex l_foot_FrameIndex;
    iDynTree::FrameIndex r_foot_FrameIndex;
    this->getFrame_Index("l_sole",    l_foot_FrameIndex);
    this->getFrame_Index("r_sole",    r_foot_FrameIndex);

    Eigen::Matrix4d world_HmgTransf_lf;
    Eigen::Matrix4d world_HmgTransf_rf;
    Eigen::Matrix<double, 7,1> lf_Pose;
    Eigen::Matrix<double, 7,1> rf_Pose;

    if(stanceFoot != "left")  // right stance foot
    {
      this->forwardKinematics(m_jts_sensor_wb.position, world_H_base_, r_foot_FrameIndex, rf_Pose, world_HmgTransf_rf);
      // W_H_B = world_HmgTransf_rf.inverse();
      W_H_B.block<3,3>(0,0) =  world_HmgTransf_rf.block<3,3>(0,0).transpose();
      W_H_B.block<3,1>(0,3) =  -W_H_B.block<3,3>(0,0)*world_HmgTransf_rf.block<3,1>(0,3);
      W_H_B(3,3) = 1.0;
    }
    else
    {
      this->forwardKinematics(m_jts_sensor_wb.position, world_H_base_, l_foot_FrameIndex, lf_Pose, world_HmgTransf_lf);
      // W_H_B = world_HmgTransf_lf.inverse();
      W_H_B.block<3,3>(0,0) =  world_HmgTransf_lf.block<3,3>(0,0).transpose();
      W_H_B.block<3,1>(0,3) =  -W_H_B.block<3,3>(0,0)*world_HmgTransf_lf.block<3,1>(0,3);
      W_H_B(3,3) = 1.0;
    }
    //
    //--------------------------------------------------------------------
    Matrix3d Rb = W_H_B.block(0,0,3,3);
    Vector3d ang_rpy = Transforms.getEulerAnglesXYZ_FixedFrame(Rb);
	    roll	= ang_rpy(0);
	    pitch 	= ang_rpy(1);
	    yaw 	= ang_rpy(2);

    if(isBaseExternal)
    {
        robot_interface_.get_fBase_HomoTransformation();
        Matrix3d Re = robot_interface_.world_H_pelvis.block(0,0,3,3);
        ang_rpy = Transforms.getEulerAnglesXYZ_FixedFrame(Re);
        if(this->RobotName == "icubSim")
        {   
            roll  = ang_rpy(0);
            pitch = ang_rpy(1);
        }
        	yaw   = ang_rpy(2);
    }

	W_H_B.block(0,0,3,3) = Transforms.rpy2rFF(Vector3d(roll, pitch, yaw));
    // Update the model state with the estimate
    eigRobotState.world_H_base = W_H_B;

    return true;
}
//
bool WbRobotModel::getEstimate_BaseWorldPose(RobotInterface& robot_interface_, Matrix4d w_H_lf, Matrix4d w_H_rf, std::string stanceFoot, Eigen::Matrix4d &W_H_B)
{
    // bool isBaseExternal = robot_interface.isBaseExternal;
    bool isBaseExternal = robot_interface_.isBaseExternal;
    //
        eigRobotState.jointPos = m_jts_sensor_wb.position;
        eigRobotState.jointVel = m_jts_sensor_wb.velocity;

    Matrix4d world_H_base_;
    world_H_base_.setIdentity();
    Matrix4d W_H_sft;
    // roll, pitch and yaw
    Vector3d ang_rpy = VectorXd::Zero(3);
    double roll	 = 0.0;	double pitch = 0.0; double yaw	 = 0.0;
    // Estimation of the base pose
    //-----------------------------
    iDynTree::FrameIndex l_foot_FrameIndex;
    iDynTree::FrameIndex r_foot_FrameIndex;
    this->getFrame_Index("l_sole",    l_foot_FrameIndex);
    this->getFrame_Index("r_sole",    r_foot_FrameIndex);

    Eigen::Matrix4d world_HmgTransf_stf;
    // Eigen::Matrix4d world_HmgTransf_rf;
    Eigen::Matrix<double, 7,1> lf_Pose;
    Eigen::Matrix<double, 7,1> rf_Pose;
    // correction of world estimated
    	Vector2d t_aF = 0.5*(w_H_lf.block<2,1>(0,3) + w_H_rf.block<2,1>(0,3));
		Vector2d t_lf_aF = w_H_lf.block<2,1>(0,3) - t_aF;
		Vector2d t_rf_aF = w_H_rf.block<2,1>(0,3) - t_aF;

    if(stanceFoot != "left")  // right stance foot
    {
      this->forwardKinematics(m_jts_sensor_wb.position, world_H_base_, r_foot_FrameIndex, rf_Pose, world_HmgTransf_stf);
      W_H_B = MatrixXd::Identity(4,4);
      // w_H_rf.block(0,0,3,3) = MatrixXd::Identity(3,3);
		w_H_lf.block<2,1>(0,3) = t_lf_aF;
		w_H_rf.block<2,1>(0,3) = t_rf_aF + Vector2d(0.0, -0.0); // -0.01

      	W_H_sft = w_H_rf;
    }
    else  // left stance foot
    {
      this->forwardKinematics(m_jts_sensor_wb.position, world_H_base_, l_foot_FrameIndex, lf_Pose, world_HmgTransf_stf);
      W_H_B = MatrixXd::Identity(4,4);
      // w_H_lf.block(0,0,3,3) = MatrixXd::Identity(3,3);
		w_H_lf.block<2,1>(0,3) = t_lf_aF + Vector2d(0.0, 0.0);  // 0.01
		w_H_rf.block<2,1>(0,3) = t_rf_aF;

      	W_H_sft = w_H_lf;
    }
    //
    W_H_B =  W_H_sft * world_HmgTransf_stf.inverse();

    Matrix3d Rb = W_H_B.block(0,0,3,3);
    Vector3d tb = W_H_B.block(0,3,3,1);
    // Euler angle wrt. fixed frame
    ang_rpy = Transforms.getEulerAnglesXYZ_FixedFrame(Rb);
    roll	= ang_rpy(0);
    pitch 	= ang_rpy(1);
    yaw 	= ang_rpy(2);

    // cout << " ROLL PITCH YAW MODEL \t" << ang_rpy.transpose() << endl;

    if(isBaseExternal)
    {
    	robot_interface_.get_fBase_HomoTransformation();
    	Matrix3d Re = robot_interface_.world_H_pelvis.block(0,0,3,3);
        ang_rpy = Transforms.getEulerAnglesXYZ_FixedFrame(Re);
        // if(this->RobotName == "icubSim")
        // {   
        //     roll  = ang_rpy(0);
        //     pitch = ang_rpy(1);
        // }
        // 	yaw   = ang_rpy(2);
        // cout << " ROLL PITCH YAW MEASURED \t" << ang_rpy.transpose() << endl;
    }
    //
    W_H_B.block(0,0,3,3) = Transforms.rpy2rFF(Vector3d(roll, pitch, yaw));
    // Update the model state with the estimate
    eigRobotState.world_H_base = W_H_B;

    // std::cout << " ///// ERROR F BASE  is  ////////// : \n" << W_H_B - robot_interface_.world_H_pelvis  << std::endl;

    return true;
}

//
bool WbRobotModel::getEstimate_BaseVelocity(RobotInterface& robot_interface_, std::string stanceFoot, Eigen::Matrix<double, 6,1> &VB)
{
    //
    bool isBaseExternal = robot_interface_.isBaseExternal;
    //
    Eigen::Matrix4d W_H_B;    W_H_B.setIdentity();
    // eigRobotState.world_H_base = W_H_B;
    this->getEstimate_BaseWorldPose(robot_interface_, stanceFoot, W_H_B);

    // update the state of the model
    this->UpdateModelStates();

    iDynTree::FrameIndex l_foot_FrameIndex;
    iDynTree::FrameIndex r_foot_FrameIndex;

    this->getFrame_Index("l_sole",    l_foot_FrameIndex);
    this->getFrame_Index("r_sole",    r_foot_FrameIndex);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Jacobian_frame_;
    Jacobian_frame_.resize(6, actuatedDofs+6);

    
    if(isBaseExternal)
    {
        robot_interface_.get_EstimateBaseVelocity_Quaternion();
        VB = robot_interface_.world_Velo_pelvis;
    }
    else
    {
        if(stanceFoot != "left")  // right stance foot
        {
            // compute the joacobian of the root frame
            this->computeJacobian(m_jts_sensor_wb.position, W_H_B, r_foot_FrameIndex, Jacobian_frame_.data());
            // VB = -Jacobian_frame_.leftCols(6).inverse()*Jacobian_frame_.rightCols(actuatedDofs) * eigRobotState.jointVel;
            VB = -Jacobian_frame_.leftCols(6).inverse()*Jacobian_frame_.rightCols(actuatedDofs) * this->m_jts_sensor_wb.velocity;
        }
        else
        {
            // compute the joacobian of the root frame
            this->computeJacobian(m_jts_sensor_wb.position, W_H_B, l_foot_FrameIndex, Jacobian_frame_.data());
            // VB = -Jacobian_frame_.leftCols(6).inverse()*Jacobian_frame_.rightCols(actuatedDofs) * eigRobotState.jointVel;
            VB = -Jacobian_frame_.leftCols(6).inverse()*Jacobian_frame_.rightCols(actuatedDofs) * this->m_jts_sensor_wb.velocity;
        }
    }

    // Velo will need to be filtered
    this->eigRobotState.baseVel = VB;

    return true;
}
//
//
bool WbRobotModel::getEstimate_BaseVelocity(RobotInterface& robot_interface_, Matrix4d w_H_lf, Matrix4d w_H_rf, std::string stanceFoot, Eigen::Matrix<double, 6,1> &VB)
{
    //
    bool isBaseExternal = robot_interface_.isBaseExternal;
    //
    Eigen::Matrix4d W_H_B;    W_H_B.setIdentity();
    // eigRobotState.world_H_base = W_H_B;
    this->getEstimate_BaseWorldPose(robot_interface_, w_H_lf, w_H_rf, stanceFoot, W_H_B);
    // update the state of the model
    this->UpdateModelStates();

    iDynTree::FrameIndex l_foot_FrameIndex;
    iDynTree::FrameIndex r_foot_FrameIndex;

    this->getFrame_Index("l_sole",    l_foot_FrameIndex);
    this->getFrame_Index("r_sole",    r_foot_FrameIndex);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Jacobian_frame_;
    Jacobian_frame_.resize(6, actuatedDofs+6);
    //
    // if(isBaseExternal)
    // {
    //     robot_interface_.get_EstimateBaseVelocity_Quaternion();
    //     VB = robot_interface_.world_Velo_pelvis;
    //     // std::cout << " measured Velo Base  is : \t" << VB.transpose() << std::endl;
    // }
    // else
    // {
        // if(stanceFoot != "left")  // right stance foot
        // {
        //     // compute the joacobian of the root frame
        //     this->computeJacobian(m_jts_sensor_wb.position, W_H_B, r_foot_FrameIndex, Jacobian_frame_.data());
        //     // VB = -Jacobian_frame_.leftCols(6).inverse()*Jacobian_frame_.rightCols(actuatedDofs) * eigRobotState.jointVel;
        //     VB = -Jacobian_frame_.leftCols(6).inverse()*Jacobian_frame_.rightCols(actuatedDofs) * this->m_jts_sensor_wb.velocity;
        //     std::cout << " VELO BASE R stance is : \t" << VB.transpose() << std::endl;
        // }
        // else
        // {
        //     // compute the joacobian of the root frame
        //     this->computeJacobian(m_jts_sensor_wb.position, W_H_B, l_foot_FrameIndex, Jacobian_frame_.data());
        //     // VB = -Jacobian_frame_.leftCols(6).inverse()*Jacobian_frame_.rightCols(actuatedDofs) * eigRobotState.jointVel;
        //     VB = -Jacobian_frame_.leftCols(6).inverse()*Jacobian_frame_.rightCols(actuatedDofs) * this->m_jts_sensor_wb.velocity;
        //     std::cout << " VELO BASE  L stance is : \t" << VB.transpose() << std::endl;
        // }

        //
        VB = get_fBasePose2VeloEstimate_Quat(W_H_B);
    // }

    // std::cout << " VELO JOINTS EIGEN is : \t" << eigRobotState.jointVel.transpose() << std::endl;

    // Velo will need to be filtered
    this->eigRobotState.baseVel = VB;

    return true;
}

//
bool WbRobotModel::getJointsLimits(RobotInterface& robot_interface_)
{
    //
    robot_interface_.getJointsLimits();
    //
    m_min_jtsLimits              =  robot_interface_.min_jtsLimits;
    m_max_jtsLimits              =  robot_interface_.max_jtsLimits;
    m_velocitySaturationLimit    =  robot_interface_.velocitySaturationLimit;             
    m_torqueSaturationLimit      =  robot_interface_.torqueSaturationLimit;     

    return true;
}


bool WbRobotModel::getConfigurationStates(RobotInterface& robot_interface_)
{
    //
    // this->robot_interface.getWholeBodyJointsMeasurements();
    // this->robot_interface.setJointsValuesInRad(this->m_jts_sensor_wb);
    robot_interface_.getWholeBodyJointsMeasurements();
    robot_interface_.setJointsValuesInRad(this->m_jts_sensor_wb);

    if(robot_interface.isBaseExternal)
    {
        robot_interface_.get_fBase_HomoTransformation();
        robot_interface_.get_EstimateBaseVelocity_Quaternion();
    }
    

    return true;
}


bool WbRobotModel::UpdateWholeBodyModel(RobotInterface& robot_interface_, std::string stanceFoot)
{
    this->getConfigurationStates(robot_interface_);

    robot_interface_.getFilteredLeftArmForceTorqueValues();
    robot_interface_.getFilteredRightArmForceTorqueValues();
    robot_interface_.getFilteredLeftLegForceTorqueValues();
    robot_interface_.getFilteredRightLegForceTorqueValues();
    if(this->RobotName == "icub")   robot_interface_.get_robot_imu_measurements();

    // Estimation of the base states (pose and velocity)
    // ------------------------------------------------------
    Eigen::Matrix4d W_H_B;              W_H_B.setIdentity();
    Eigen::Matrix<double, 6,1> VB;      VB.setZero();
    // Estimation of the base pose
    this->getEstimate_BaseWorldPose(robot_interface_, stanceFoot, W_H_B);
    // Estimation of the Base velocity
    this->getEstimate_BaseVelocity(robot_interface_, stanceFoot,  VB);

    // Update the Eigen States for the model
    eigRobotState.update( //  stanceFoot, 
                            this->m_jts_sensor_wb.position, 
                            this->m_jts_sensor_wb.velocity, 
                            W_H_B,
                            VB);
    // Update the states
    this->UpdateModelStates();

    return true;
}

bool WbRobotModel::UpdateWholeBodyModel_init(RobotInterface& robot_interface_, std::string stanceFoot) // VB is set to zero initially
{
    this->getConfigurationStates(robot_interface_);

    robot_interface_.getFilteredLeftArmForceTorqueValues();
    robot_interface_.getFilteredRightArmForceTorqueValues();
    robot_interface_.getFilteredLeftLegForceTorqueValues();
    robot_interface_.getFilteredRightLegForceTorqueValues();
    if(this->RobotName == "icub")   robot_interface_.get_robot_imu_measurements();

    // Estimation of the base states (pose and velocity)
    // ------------------------------------------------------
    Eigen::Matrix4d W_H_B;              W_H_B.setIdentity();
    Eigen::Matrix<double, 6,1> VB;      VB.setZero();
    // Estimation of the base pose
    this->getEstimate_BaseWorldPose(robot_interface_, stanceFoot, W_H_B);
    // Estimation of the Base velocity
    // this->getEstimate_BaseVelocity(robot_interface_, stanceFoot,  VB);

    // Update the Eigen States for the model
    eigRobotState.update( //  stanceFoot, 
                            this->m_jts_sensor_wb.position, 
                            this->m_jts_sensor_wb.velocity, 
                            W_H_B,
                            VB);
    // Update the states
    this->UpdateModelStates();

    return true;
}

bool WbRobotModel::UpdateWholeBodyModel(RobotInterface& robot_interface_, Matrix4d w_H_lf, Matrix4d w_H_rf, std::string stanceFoot)
{
    this->getConfigurationStates(robot_interface_);

    robot_interface_.getFilteredLeftArmForceTorqueValues();
    robot_interface_.getFilteredRightArmForceTorqueValues();
    robot_interface_.getFilteredLeftLegForceTorqueValues();
    robot_interface_.getFilteredRightLegForceTorqueValues();
    if(this->RobotName == "icub")   robot_interface_.get_robot_imu_measurements();

    // Estimation of the base states (pose and velocity)
    // ------------------------------------------------------
    Eigen::Matrix4d W_H_B;              W_H_B.setIdentity();
    Eigen::Matrix<double, 6,1> VB;      VB.setZero();
    // Estimation of the base pose
    this->getEstimate_BaseWorldPose(robot_interface_, w_H_lf, w_H_rf, stanceFoot, W_H_B);
    // Estimation of the Base velocity
    this->getEstimate_BaseVelocity(robot_interface_, w_H_lf, w_H_rf, stanceFoot,  VB);

    // Update the Eigen States for the model
    eigRobotState.update( //  stanceFoot, 
                            this->m_jts_sensor_wb.position, 
                            this->m_jts_sensor_wb.velocity, 
                            W_H_B,
                            VB);
    // Update the states
    this->UpdateModelStates();

    return true;
}

bool WbRobotModel::getJointsStates(RobotInterface& robot_interface_, JointspaceStates& JtsStates_)
{
    // 
    this->getConfigurationStates(robot_interface_);
    JtsStates_.position          =   this->m_jts_sensor_wb.position;
    JtsStates_.velocity          =   this->m_jts_sensor_wb.velocity;
    JtsStates_.acceleration      =   this->m_jts_sensor_wb.acceleration;
    //
    return true;
}

bool WbRobotModel::setControlReference(RobotInterface& robot_interface_, Eigen::VectorXd tau_actuated)
{
    robot_interface_.setWholeBodyControlReference(tau_actuated);

    return true;
}  

// Estimation of the robot state
bool WbRobotModel::EstimateRobotStates(RobotInterface& robot_interface_, std::string stance_ft, Joints_Measurements& m_jts_sensor_wb_, Matrix4d &W_H_B_, Vector6d  &VB_)
{
    //
    bool ok = true;
    // Estimate the joint states
    // --------------------------
        ok = ok && robot_interface_.getWholeBodyJointsMeasurements();
        ok = ok && robot_interface_.setJointsValuesInRad(this->m_jts_sensor_wb);
        m_jts_sensor_wb_ = this->m_jts_sensor_wb;
    // estimate the floating base states
    // ---------------------------------
        // Estimation of the base pose
        ok = ok && this->getEstimate_BaseWorldPose(robot_interface_, stance_ft, W_H_B_);
        // Estimation of the Base velocity
        ok = ok && this->getEstimate_BaseVelocity(robot_interface_, stance_ft,  VB_);
    //
    robot_interface_.getFilteredLeftArmForceTorqueValues();
    robot_interface_.getFilteredRightArmForceTorqueValues();
    robot_interface_.getFilteredLeftLegForceTorqueValues();
    robot_interface_.getFilteredRightLegForceTorqueValues();
    if(this->RobotName == "icub")   robot_interface_.get_robot_imu_measurements();
    //
    return ok;
}

// Estimation of the robot state
bool WbRobotModel::EstimateRobotStates(RobotInterface& robot_interface_, Matrix4d w_H_lf, Matrix4d w_H_rf, std::string stance_ft, 
                                        Joints_Measurements& m_jts_sensor_wb_, Matrix4d &W_H_B_, Vector6d  &VB_)
{
    //
    bool ok = true;
    // Estimate the joint states
    // --------------------------
        ok = ok && robot_interface_.getWholeBodyJointsMeasurements();
        ok = ok && robot_interface_.setJointsValuesInRad(this->m_jts_sensor_wb);
        m_jts_sensor_wb_ = this->m_jts_sensor_wb;
    // estimate the floating base states
    // ---------------------------------
        // Estimation of the base pose
        ok = ok && this->getEstimate_BaseWorldPose(robot_interface_, w_H_lf, w_H_rf, stance_ft, W_H_B_);
        // Estimation of the Base velocity
        ok = ok && this->getEstimate_BaseVelocity(robot_interface_, w_H_lf, w_H_rf, stance_ft,  VB_);
    //
    robot_interface_.getFilteredLeftArmForceTorqueValues();
    robot_interface_.getFilteredRightArmForceTorqueValues();
    robot_interface_.getFilteredLeftLegForceTorqueValues();
    robot_interface_.getFilteredRightLegForceTorqueValues();
    if(this->RobotName == "icub")   robot_interface_.get_robot_imu_measurements();

    return ok;
}
//
Vector7d WbRobotModel::get_fBasePoseEstimate(Matrix4d W_H_fB)
{
    Vector7d fBasePoseEstimate_;
    Eigen::Quaterniond qb(W_H_fB.block<3,3>(0,0));
    fBasePoseEstimate_.head<3>() = W_H_fB.block<3,1>(0,3);
    fBasePoseEstimate_.tail<4>() << qb.x(), qb.y(), qb.z(), qb.w();
    
    return fBasePoseEstimate_;
}


Vector6d WbRobotModel::get_fBasePose2VeloEstimate_Quat(Matrix4d W_H_fB)
{
 
	Vector7d fB_pose = this->get_fBasePoseEstimate(W_H_fB);
    Vector6d VfBase  = VectorXd::Zero(6);
    //  filter and get the derivative of the measurements
    filtered_fBasePoseEstimate  = fBaseFilter.getRK4Integral(fB_pose);
    dot_fBasePoseEstimate       = filter_gain_fBase * (fB_pose - filtered_fBasePoseEstimate);
    //
    Eigen::MatrixXd Omega2dqMx(4,3), psdinvOmega2dqMx(3,4);

    Omega2dqMx(0,0) = -fB_pose(3);  Omega2dqMx(0,1) = -fB_pose(4);  Omega2dqMx(0,2) = -fB_pose(5);  
    Omega2dqMx(1,0) =  fB_pose(6);  Omega2dqMx(1,1) =  fB_pose(5);  Omega2dqMx(1,2) = -fB_pose(4);
    Omega2dqMx(2,0) = -fB_pose(5);  Omega2dqMx(2,1) =  fB_pose(6);  Omega2dqMx(2,2) =  fB_pose(3);
    Omega2dqMx(3,0) =  fB_pose(4);  Omega2dqMx(3,1) = -fB_pose(3);  Omega2dqMx(3,2) =  fB_pose(6);

    psdinvOmega2dqMx = PseudoInverser.get_pseudoInverse(Omega2dqMx); 
    //
    Eigen::VectorXd quat_dot(4);
    quat_dot(0) = dot_fBasePoseEstimate(6);
    quat_dot(1) = dot_fBasePoseEstimate(3);
    quat_dot(2) = dot_fBasePoseEstimate(4);
    quat_dot(3) = dot_fBasePoseEstimate(5);
    //
    VfBase.head<3>() = dot_fBasePoseEstimate.head<3>();
    VfBase.tail<3>() = 2.*psdinvOmega2dqMx * quat_dot;

    return VfBase;
}

//
bool WbRobotModel::get_feet_support_points( MatrixXd PtsInFoot, Vector7d Pose_lfoot, Vector7d Pose_rfoot, MatrixXd &AllPtsInAbsFoot_, Matrix2d &W_Rot_AbsFoot, Vector2d &W_Pos_AbsFoot)
{
    // Kinematic transformations class
    // KineTransformations Transforms_;
    // 3D rotation matrix of the feet wrt the world frame
    Eigen::Matrix3d WR_lf = Transforms.PoseVector2HomogenousMx(Pose_lfoot).block(0,0,3,3);
    Eigen::Matrix3d WR_rf = Transforms.PoseVector2HomogenousMx(Pose_rfoot).block(0,0,3,3);
    // Planar orientations of the feet 
    Vector3d o_lf = Transforms.getEulerAnglesXYZ_FixedFrame(WR_lf);
    Vector3d o_rf = Transforms.getEulerAnglesXYZ_FixedFrame(WR_rf);
    //
    int np_f = PtsInFoot.cols();                                                        // number of points in one foot support polygon

    AllPtsInAbsFoot_.resize(2,np_f*2);                                                  // coordinates of the point in the absolute frame
    AllPtsInAbsFoot_.setZero();
    MatrixXd W_PtsSwingFt(2,np_f);                                                      // Matrix of points in the swing foot (foot that will perform the stepping)
    MatrixXd W_PtsStanceFt(2,np_f);                                                     // Matrix of points in the stance foot (foot that stays fixed)
    MatrixXd RotStanceFt(2,2);                                                          // Plananr Rotation matrix of the stance foot wrt the the world frane
    Vector2d TransStanceFt(2);                                                          // Plananr Position vector of the stance foot wrt the world frame
    MatrixXd RotSwingFt(2,2);                                                           // Plananr Rotation matrix of the swing foot wrt the the world frane
    Vector2d TransSwingFt(2);                                                           // Plananr Position vector of the swing foot wrt the world frame
    Matrix2d AbsFoot_Rot_W;                                                             // Rotation of the world frame relatibve to the absolute feet frame
    Vector2d AbsFoot_Trans_W;                                                           // position of the world frame relatibve to the absolute feet frame
    //
    double DthetaFt = 0.0;                                                              // Relative angle between stance and swing foot

    // Extraction of the feet orientation
    double theta_stance = o_lf(2);                                                      //Transforms.getEulerAnglesXYZ_FixedFrame(WR_lf).(2);
    double theta_swing  = o_rf(2); 
    // Planar rotations of the feet
    RotStanceFt << cos(theta_stance), -sin(theta_stance), sin(theta_stance), cos(theta_stance);
    RotSwingFt  << cos(theta_swing),  -sin(theta_swing),  sin(theta_swing),  cos(theta_swing);
    // Planar translations of the feet
    TransStanceFt = Pose_lfoot.head(2);
    TransSwingFt  = Pose_rfoot.head(2);    
    // Expression of pts from foot frame to world frame
    for (int i=0; i<np_f; i++)   {
        W_PtsStanceFt.col(i) = RotStanceFt * PtsInFoot.col(i) + TransStanceFt;
        W_PtsSwingFt.col(i)  = RotSwingFt  * PtsInFoot.col(i) + TransSwingFt;
    }    
    // Expression of the feet polygon points in the absolute foot frame and 
    MatrixXd W_All_Pts(2,np_f*2);
    W_All_Pts.leftCols(np_f)  = W_PtsStanceFt;
    W_All_Pts.rightCols(np_f) = W_PtsSwingFt;

    // get the absolute feet pose
    // ---------------------------
    W_Pos_AbsFoot    = 0.5 * (TransStanceFt + TransSwingFt);                            // 2D position 
    double abs_angle = 0.5*(theta_stance + theta_swing);                                // 2D rotation
    // double abs_angle = theta_stance;
    // double abs_angle = theta_stance;
    W_Rot_AbsFoot << cos(abs_angle), -sin(abs_angle), sin(abs_angle), cos(abs_angle); 
    AbsFoot_Trans_W = -W_Pos_AbsFoot;                                                   // Get the position of the absolute foot frame
    AbsFoot_Rot_W   = W_Rot_AbsFoot.transpose();                                        // Planar rotation of the equivalent foot frame (absolute frame)
    for (int i=0; i<np_f*2; i++) 
        AllPtsInAbsFoot_.col(i) = AbsFoot_Rot_W * W_All_Pts.col(i) + AbsFoot_Trans_W;   // Transformation
    //
    return true;
}


bool WbRobotModel::getConvexHullVariables(  MatrixXd PtsInFoot_, Vector7d Pose_lfoot, Vector7d Pose_rfoot,  MatrixXd &W_EdgesNormalCHull, VectorXd &DistanceEdgesCHull, 
                                                                        Matrix2d &W_Rot_AbsFoot, Vector2d &W_Pos_AbsFoot)
{
    // computation of the convex polygon
    MatrixXd AllPtsInAbsFoot;
    MatrixXd PointsCHull = MatrixXd::Zero(8,2);
    MatrixXd Abs_EdgesNormalCHull;
    // Expressing the feet contact points in the absolute feet frame
    this->get_feet_support_points(PtsInFoot_, Pose_lfoot, Pose_rfoot, AllPtsInAbsFoot, W_Rot_AbsFoot, W_Pos_AbsFoot);  // chech also with des_X_EE
    // In the absolute feet frame compute the convex hull and the normals to the edges and 
    ConvHull.convexHull(AllPtsInAbsFoot, PointsCHull, Abs_EdgesNormalCHull, DistanceEdgesCHull); 
    // Express the normal in the world frame
    W_EdgesNormalCHull.resize(Abs_EdgesNormalCHull.rows(), Abs_EdgesNormalCHull.cols());
    W_EdgesNormalCHull = Abs_EdgesNormalCHull * W_Rot_AbsFoot.transpose();

    for(int i=0; i<W_EdgesNormalCHull.rows(); i++)
    {
        W_EdgesNormalCHull.row(i) = W_EdgesNormalCHull.row(i)/(W_EdgesNormalCHull.row(i).norm() + 1.e-20);
    }
    // std::cout << " POINTS CONVEX HULL : \n" << this->PointsCHull << std::endl;
    return true;
}