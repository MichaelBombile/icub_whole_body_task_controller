

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

//
bool StepsGenerator::init(int ThreadPeriod, std::string robotName, int ForceFeedbackType)
{
	// initialization of the counter
    CycleCounter  = 0;
    // Initialization of the Des_RelativeVelocity
    Des_RelativeVelocity.setZero();
	// initial parameters
    Parameters = new InitBalWlkParameters(ThreadPeriod, robotName, ForceFeedbackType);

    // set the robot to the initial walking posture
    // CtrlPostures.moveToinitialWalkingPosture(RobotDevices->iencs_left_leg, 
    //                                          RobotDevices->iencs_right_leg, 
    //                                          RobotDevices->ipos_left_leg, 
    //                                          RobotDevices->ipos_right_leg, joints_Offset, 
    //                                          CTRL_RAD2DEG * Des_lljoints, CTRL_RAD2DEG * Des_rljoints,
    //                                          false);

    // *******************************************************************************************
    // Instantiation of the robot locomotion module
    // *******************************************************************************************
    CpBalWlkController = new CpReactiveWalkingController();
    // initialization of the controller
    CpBalWlkController->InitializeCpBalWlkController(Parameters);

    //  // Expression of the legs transformation with respect to the measeured horizontal plan
    // GaitInIMU = new InertialCompensator();
    // GaitInIMU->getDesiredFeetInCorrectedBase(   Parameters->StanceIndicator, 
    //                                             RobotKin->LeftLegChain->EndEffPose(), 
    //                                             RobotKin->RightLegChain->EndEffPose(),
    //                                             BotSensors.m_orientation_rpy,  
    //                                             CpBalWlkController->GaitTrsf);

    // InvKinSolver_lleg.InitializeIK(RobotKin->LeftLegChain,  0.90, 20, 1e-4, 0.450);
    // InvKinSolver_rleg.InitializeIK(RobotKin->RightLegChain, 0.90, 20, 1e-4, 0.450);

	return true;
}


void StepsGenerator::Release()
{
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


bool StepsGenerator::walk() // walk or step
{
    if (CycleCounter == 0){

        printf("CpBalWlkCtrlThread: now running \n");
    }
    

    CpBalWlkController->UpdateCpBalWlkController(Parameters, Des_RelativeVelocity, CycleCounter);

    // GaitInIMU->CompensateTransformsWithIMU( CpBalWlkController->GaitTrsf,
    //                                         RobotKin->LeftLegChain->EndEffPose(), 
    //                                         RobotKin->RightLegChain->EndEffPose(),                                                      
    //                                         BotSensors.m_orientation_rpy,  // filtered
    //                                         Parameters->StanceIndicator,
    //                                         Parameters->IMU_reference);

    // my inverse kinematic solver
    // yarp::sig::Vector myDes_left_leg_joints = InvKinSolver_lleg.get_IKsolution(GaitInIMU->DesiredLeftLegPoseAsAxisAngles, RobotDevices->encoders_left_leg, true);
    // yarp::sig::Vector myDes_right_leg_joints = InvKinSolver_rleg.get_IKsolution(GaitInIMU->DesiredRightLegPoseAsAxisAngles, RobotDevices->encoders_right_leg, true);

    CycleCounter ++;

    cout << "Iteration StepsGenerator : " << CycleCounter << endl;

    return true;
}

bool StepsGenerator::step() // walk or step
{
    if (CycleCounter == 0){

        printf("CpBalWlkCtrlThread: now running \n");
    }
    cout << "Iteration : " << CycleCounter +1 << endl;

    CpBalWlkController->UpdateCpBalWlkController(Parameters, Des_RelativeVelocity, CycleCounter);

    // GaitInIMU->CompensateTransformsWithIMU( CpBalWlkController->GaitTrsf,
    //                                         RobotKin->LeftLegChain->EndEffPose(), 
    //                                         RobotKin->RightLegChain->EndEffPose(),                                                      
    //                                         BotSensors.m_orientation_rpy,  // filtered
    //                                         Parameters->StanceIndicator,
    //                                         Parameters->IMU_reference);

    // my inverse kinematic solver
    // yarp::sig::Vector myDes_left_leg_joints = InvKinSolver_lleg.get_IKsolution(GaitInIMU->DesiredLeftLegPoseAsAxisAngles, RobotDevices->encoders_left_leg, true);
    // yarp::sig::Vector myDes_right_leg_joints = InvKinSolver_rleg.get_IKsolution(GaitInIMU->DesiredRightLegPoseAsAxisAngles, RobotDevices->encoders_right_leg, true);

    CycleCounter ++;
    return true;
}