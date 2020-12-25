

// #include "StepsGenerator.h"


// using namespace std;
// using namespace Eigen;


// StepsGenerator::StepsGenerator()
// {

// }

// StepsGenerator::~StepsGenerator()
// {
// 	this->Release();
// }

// //
// bool StepsGenerator::init(int ThreadPeriod, std::string robotName, int ForceFeedbackType)
// {
// 	// initialization of the counter
//     CycleCounter  = 0;
//     // Initialization of the Des_RelativeVelocity
//     Des_RelativeVelocity.setZero();
// 	// initial parameters
//     Parameters = new InitBalWlkParameters(ThreadPeriod, robotName, ForceFeedbackType);

//     // set the robot to the initial walking posture
//     // CtrlPostures.moveToinitialWalkingPosture(RobotDevices->iencs_left_leg, 
//     //                                          RobotDevices->iencs_right_leg, 
//     //                                          RobotDevices->ipos_left_leg, 
//     //                                          RobotDevices->ipos_right_leg, joints_Offset, 
//     //                                          CTRL_RAD2DEG * Des_lljoints, CTRL_RAD2DEG * Des_rljoints,
//     //                                          false);

//     // *******************************************************************************************
//     // Instantiation of the robot locomotion module
//     // *******************************************************************************************
//     CpBalWlkController = new CpReactiveWalkingController();
//     // initialization of the controller
//     CpBalWlkController->InitializeCpBalWlkController(Parameters);

//     //  // Expression of the legs transformation with respect to the measeured horizontal plan
//     // GaitInIMU = new InertialCompensator();
//     // GaitInIMU->getDesiredFeetInCorrectedBase(   Parameters->StanceIndicator, 
//     //                                             RobotKin->LeftLegChain->EndEffPose(), 
//     //                                             RobotKin->RightLegChain->EndEffPose(),
//     //                                             BotSensors.m_orientation_rpy,  
//     //                                             CpBalWlkController->GaitTrsf);

//     // InvKinSolver_lleg.InitializeIK(RobotKin->LeftLegChain,  0.90, 20, 1e-4, 0.450);
//     // InvKinSolver_rleg.InitializeIK(RobotKin->RightLegChain, 0.90, 20, 1e-4, 0.450);

// 	return true;
// }

// bool StepsGenerator::init(int ThreadPeriod, std::string robotName, int ForceFeedbackType, Vector3d init_com_pos)
// {
//     // initialization of the counter
//     CycleCounter  = 0;
//     // Initialization of the Des_RelativeVelocity
//     Des_RelativeVelocity.setZero();
//     // initial parameters
//     Parameters = new InitBalWlkParameters(ThreadPeriod, robotName, ForceFeedbackType);

//     // set the initial com position in the absolute frame
//     Parameters->init_com_position = init_com_pos;
//     // set the robot to the initial walking posture
//     // CtrlPostures.moveToinitialWalkingPosture(RobotDevices->iencs_left_leg, 
//     //                                          RobotDevices->iencs_right_leg, 
//     //                                          RobotDevices->ipos_left_leg, 
//     //                                          RobotDevices->ipos_right_leg, joints_Offset, 
//     //                                          CTRL_RAD2DEG * Des_lljoints, CTRL_RAD2DEG * Des_rljoints,
//     //                                          false);

//     // *******************************************************************************************
//     // Instantiation of the robot locomotion module
//     // *******************************************************************************************
//     CpBalWlkController = new CpReactiveWalkingController();
//     // initialization of the controller
//     CpBalWlkController->InitializeCpBalWlkController(Parameters);

//     //  // Expression of the legs transformation with respect to the measeured horizontal plan
//     // GaitInIMU = new InertialCompensator();
//     // GaitInIMU->getDesiredFeetInCorrectedBase(   Parameters->StanceIndicator, 
//     //                                             RobotKin->LeftLegChain->EndEffPose(), 
//     //                                             RobotKin->RightLegChain->EndEffPose(),
//     //                                             BotSensors.m_orientation_rpy,  
//     //                                             CpBalWlkController->GaitTrsf);

//     // InvKinSolver_lleg.InitializeIK(RobotKin->LeftLegChain,  0.90, 20, 1e-4, 0.450);
//     // InvKinSolver_rleg.InitializeIK(RobotKin->RightLegChain, 0.90, 20, 1e-4, 0.450);

//     return true;
// }


// void StepsGenerator::Release()
// {
// 	// release the CpStepsGenerator
//     CpBalWlkController->ReleaseCpBalWlkController();

//     if (CpBalWlkController){
//         delete CpBalWlkController;
//         CpBalWlkController = 0;
//     }

//     if (Parameters) {
//         delete Parameters;
//         Parameters = 0;
//     }

//     printf("StepsGenerator: now closing \n");
 	
// }


// bool StepsGenerator::walk() // walk or step
// {
//     if (CycleCounter == 0){

//         printf("CpBalWlkCtrlThread: now running \n");
//     }
    

//     CpBalWlkController->UpdateCpBalWlkController(Parameters, Des_RelativeVelocity, CycleCounter);

//     // GaitInIMU->CompensateTransformsWithIMU( CpBalWlkController->GaitTrsf,
//     //                                         RobotKin->LeftLegChain->EndEffPose(), 
//     //                                         RobotKin->RightLegChain->EndEffPose(),                                                      
//     //                                         BotSensors.m_orientation_rpy,  // filtered
//     //                                         Parameters->StanceIndicator,
//     //                                         Parameters->IMU_reference);

//     // my inverse kinematic solver
//     // yarp::sig::Vector myDes_left_leg_joints = InvKinSolver_lleg.get_IKsolution(GaitInIMU->DesiredLeftLegPoseAsAxisAngles, RobotDevices->encoders_left_leg, true);
//     // yarp::sig::Vector myDes_right_leg_joints = InvKinSolver_rleg.get_IKsolution(GaitInIMU->DesiredRightLegPoseAsAxisAngles, RobotDevices->encoders_right_leg, true);

//     CycleCounter ++;

//     cout << "Iteration StepsGenerator : " << CycleCounter << endl;

//     return true;
// }

// bool StepsGenerator::step() // walk or step
// {
//     if (CycleCounter == 0){

//         printf("CpBalWlkCtrlThread: now running \n");
//     }
//     cout << "Iteration : " << CycleCounter +1 << endl;

//     CpBalWlkController->UpdateCpBalWlkController(Parameters, Des_RelativeVelocity, CycleCounter);

//     // GaitInIMU->CompensateTransformsWithIMU( CpBalWlkController->GaitTrsf,
//     //                                         RobotKin->LeftLegChain->EndEffPose(), 
//     //                                         RobotKin->RightLegChain->EndEffPose(),                                                      
//     //                                         BotSensors.m_orientation_rpy,  // filtered
//     //                                         Parameters->StanceIndicator,
//     //                                         Parameters->IMU_reference);

//     // my inverse kinematic solver
//     // yarp::sig::Vector myDes_left_leg_joints = InvKinSolver_lleg.get_IKsolution(GaitInIMU->DesiredLeftLegPoseAsAxisAngles, RobotDevices->encoders_left_leg, true);
//     // yarp::sig::Vector myDes_right_leg_joints = InvKinSolver_rleg.get_IKsolution(GaitInIMU->DesiredRightLegPoseAsAxisAngles, RobotDevices->encoders_right_leg, true);

//     CycleCounter ++;
//     return true;
// }


// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////






#include "StepsGenerator.h"


using namespace std;
using namespace Eigen;


StepsGenerator::StepsGenerator()
{

}

StepsGenerator::~StepsGenerator()
{
    this->Release();
}

bool StepsGenerator::init(std::string robotName, int ThreadPeriod, int ForceFeedbackType, Vector3d init_com_pos)
{
    // initialization of the counter
    CycleCounter  = 0;
    // Initialization of the Des_RelativeVelocity
    Des_RelativeVelocity.setZero();
    // initial parameters
    Parameters = new InitBalWlkParameters(ThreadPeriod, robotName, ForceFeedbackType);
    // set the initial com position in the absolute frame
    Parameters->init_com_position = init_com_pos;
    // *******************************************************************************************
    // Instantiation of the robot locomotion module
    // *******************************************************************************************
    CpBalWlkController = new CpReactiveWalkingController();
    // initialization of the controller
    CpBalWlkController->InitializeCpBalWlkController(Parameters);

    // -----------------------------------------------------------------------------------------------------------
    // Set the Desired CoM velocity
    Des_RelativeVelocity(0) = 0.00;
    Des_RelativeVelocity(1) = 0.00;
    Des_RelativeVelocity(2) = 0.00;
    // Update the stance foot
    if(Parameters->StanceIndicator[0] == 1){   // left if 1 
        lstance_switch = true;
        rstance_switch = false;
    }else{                                     // right
        lstance_switch = false;
        rstance_switch = true;
    }
    // left and right foot desired homogenous transformation in base frame with initial world orientation
    Trsf_des_ee2base_world[2]   = CpBalWlkController-> GaitTrsf.Trsf_lfoot_base_world;
    Trsf_des_ee2base_world[3]   = CpBalWlkController-> GaitTrsf.Trsf_lfoot_base_world;
    // Transformation Com in the feet frames
    Trsf_des_com2lfoot          = CpBalWlkController-> GaitTrsf.Trsf_lfoot_com.inverse();  
    Trsf_des_com2rfoot          = CpBalWlkController-> GaitTrsf.Trsf_rfoot_com.inverse();

    Trsf_des_lfoot2base_robot   = CpBalWlkController-> GaitTrsf.Trsf_lfoot_base_icub;
    Trsf_des_rfoot2base_robot   = CpBalWlkController-> GaitTrsf.Trsf_rfoot_base_icub;

    for (int i=0; i<6; i++)
        Trsf_des_ee2world[i] = MatrixXd::Identity(4,4);

    // -------------------------------------------------------------------------------------------------------------------

    return true;
}


void StepsGenerator::Release()
{
    //
    Des_RelativeVelocity(0) = 0.00;  // TO DO
    Des_RelativeVelocity(1) = 0.00;
    Des_RelativeVelocity(2) = 0.00;

    // release the CpStepsGenerator
    CpBalWlkController->ReleaseCpBalWlkController();

    if (CpBalWlkController){
        delete CpBalWlkController;
        CpBalWlkController = 0;
    }

    if (Parameters) {
        delete Parameters;
        Parameters = 0;
    }

    printf("StepsGenerator: now closing \n");
    
}


bool StepsGenerator::walk(int number_steps, double stride_x, Vector7d des_pose_lh, Vector7d des_pose_rh, WholeBodyTaskSpaceStates wb_ts)
{
    //
    // Vector7d W_Pose_aF;
    // Transforms.get_absolute_pose(wb_ts.lfoot.Pose, wb_ts.rfoot.Pose, W_Pose_aF);
    // Matrix4d W_H_aF_ = Transforms.PoseVector2HomogenousMx(W_Pose_aF); 

    Matrix4d w_H_c = Transforms.PoseVector2HomogenousMx(wb_ts.CoM.Pose);
    Matrix4d w_H_p = Transforms.PoseVector2HomogenousMx(wb_ts.Pelvis.Pose);
    Matrix4d c_H_p = w_H_c.inverse() * w_H_p;
    //
    Eigen::Vector3d t_B_CoM;
    // t_B_CoM =      wb_ts.Pelvis.Pose.head(3) - wb_ts.CoM.Pose.head(3) + Vector3d(0.04, 0.0, 0.0);
    t_B_CoM <<  -0.0*c_H_p(0,3),   0.0,  c_H_p(2,3); 
    if(Parameters->RobotName == "icub")
        t_B_CoM <<  -0.0*c_H_p(0,3),   0.0,  c_H_p(2,3); 

    CpBalWlkController->GaitTrsf.SetTranslationBaseCoM(t_B_CoM);
    // Loop for the walking   
    // --------------------------------------------------------------------------------------------------
    if (CycleCounter == 0){

        printf("CpBalWlkCtrlThread: now running \n");
    }
    //
    CpBalWlkController->UpdateCpBalWlkController(Parameters, Des_RelativeVelocity, CycleCounter);

    CycleCounter ++;
    cout << "Iteration StepsGenerator : " << CycleCounter << endl;
    // --------------------------------------------------------------------------------------------------
    Matrix4d    w_T_pe_w                 = pose_vector_toHomogenousMatrix(wb_ts.Pelvis.Pose);
                w_T_pe_w.block(0,0, 3,3) =Transforms.rot_vector2rpy(Eigen::Vector3d(0,0,M_PI)) * w_T_pe_w.block(0,0, 3,3);
    // left and right hands desired homogenous transformation in base frame with initial world orientation
    Trsf_des_ee2base_world[0]   = w_T_pe_w.inverse() * pose_vector_toHomogenousMatrix(wb_ts.lhand.Pose);
    Trsf_des_ee2base_world[1]   = w_T_pe_w.inverse() * pose_vector_toHomogenousMatrix(wb_ts.rhand.Pose);
    // left and right foot desired homogenous transformation in base frame with initial world orientation
    Trsf_des_ee2base_world[2]   = CpBalWlkController->GaitTrsf.Trsf_lfoot_base_world;
    Trsf_des_ee2base_world[3]   = CpBalWlkController->GaitTrsf.Trsf_rfoot_base_world;

    // left and right foot desired homogenous transformation in robot base frame
    Trsf_des_lfoot2base_robot  = CpBalWlkController->GaitTrsf.Trsf_lfoot_base_icub;
    Trsf_des_rfoot2base_robot  = CpBalWlkController->GaitTrsf.Trsf_rfoot_base_icub;

    cout << " Trsf_des_lfoot2base_robot IS \n " << Trsf_des_lfoot2base_robot  << endl;
    cout << " Trsf_des_rfoot2base_robot IS \n " << Trsf_des_rfoot2base_robot  << endl;

    // Transformation Com in the feet frames
    Trsf_des_com2lfoot  = CpBalWlkController->GaitTrsf.Trsf_lfoot_com.inverse();   
    Trsf_des_com2rfoot  = CpBalWlkController->GaitTrsf.Trsf_rfoot_com.inverse();

    // Tasks  ////////////////////////////////////////////////////////////////////////////////////////////////////
    if(Parameters->StanceIndicator[0] == 1) // left stance
    {
        Trsf_lstancefoot2world = pose_vector_toHomogenousMatrix(wb_ts.lfoot.Pose);
        // Trsf_des_rfoot2world = Trsf_lstancefoot2world * Trsf_des_rswingfoot2lfoot;
        Trsf_des_ee2world[0] = Trsf_lstancefoot2world * Trsf_des_ee2base_world[2].inverse() * Trsf_des_ee2base_world[0];
        Trsf_des_ee2world[1] = Trsf_lstancefoot2world * Trsf_des_ee2base_world[2].inverse() * Trsf_des_ee2base_world[1];
        Trsf_des_ee2world[2] = Trsf_lstancefoot2world;
        Trsf_des_ee2world[3] = Trsf_lstancefoot2world * Trsf_des_ee2base_world[2].inverse() * Trsf_des_ee2base_world[3];
        Trsf_des_ee2world[4] = Trsf_lstancefoot2world * Trsf_des_com2lfoot;
        Trsf_des_ee2world[5] = Trsf_lstancefoot2world * Trsf_des_lfoot2base_robot.inverse();
    }
    else // right stance foot
    {
        Trsf_rstancefoot2world = pose_vector_toHomogenousMatrix(wb_ts.rfoot.Pose);
        // Trsf_des_lfoot2world = Trsf_rstancefoot2world * Trsf_des_lswingfoot2lfoot;
        Trsf_des_ee2world[0] = Trsf_rstancefoot2world * Trsf_des_ee2base_world[3].inverse() * Trsf_des_ee2base_world[0];
        Trsf_des_ee2world[1] = Trsf_rstancefoot2world * Trsf_des_ee2base_world[3].inverse() * Trsf_des_ee2base_world[1];
        Trsf_des_ee2world[2] = Trsf_rstancefoot2world * Trsf_des_ee2base_world[3].inverse() * Trsf_des_ee2base_world[2];
        Trsf_des_ee2world[3] = Trsf_rstancefoot2world; 
        Trsf_des_ee2world[4] = Trsf_rstancefoot2world * Trsf_des_com2rfoot;
        Trsf_des_ee2world[5] = Trsf_rstancefoot2world * Trsf_des_rfoot2base_robot.inverse();
    }

    return true;
}

bool StepsGenerator::step() // walk or step
{
    if (CycleCounter == 0){

        printf("CpBalWlkCtrlThread: now running \n");
    }
    cout << "Iteration : " << CycleCounter +1 << endl;

    CpBalWlkController->UpdateCpBalWlkController(Parameters, Des_RelativeVelocity, CycleCounter);
    CycleCounter ++;
    return true;
}

Matrix4d StepsGenerator::pose_vector_toHomogenousMatrix(Vector7d Pose_vector)
{
    // 
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