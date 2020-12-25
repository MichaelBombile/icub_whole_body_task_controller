#ifndef StepsGenerator_H
#define StepsGenerator_H


#include <string>
#include <iostream>
#include <fstream>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>

#include "InitBalWlkParameters.h"
#include "CpMath_Utilities.h"
#include "wbhpidcUtilityFunctions.hpp"
// #include "RobotModel.h"
#include "TemplateModels.h"
#include "MPCOptimizer.h"
#include "PatternsGenerator.h"
#include "StatesCompensators.h"

// #include "Data_logging.h"

#include <qpOASES.hpp>

using namespace std;
using namespace Eigen;


class StepsGenerator
{
     
    public:

    // internal number of cycles indicator
    int CycleCounter;
    int SwitchCounter;
    // starting time
    double start_time;
    // Posture of the robot
    // RobotPostures           CtrlPostures;

    InitBalWlkParameters    *Parameters;
    Vector3d                Des_RelativeVelocity;
    Vector3d                Feedback_RelativeVelocity;

    // pattern generator
    CpReactiveWalkingController          *CpBalWlkController;

    bool left_stance;
    // transformation to horizontal (IMU) frame
    // InertialCompensator  *GaitInIMU;
      // Usefull Transformation
    KineTransformations Trnsfrms;
    // ---------------------------------------
    KinConverter Pose2Matrix;
    // dynamic filter for the pattern generator
    ZmpDynamicFilter DynFilterZmpCoM;
    // inverse kinematics
    // InverseKinematicsSolver InvKinSolver_lleg;
    // InverseKinematicsSolver InvKinSolver_rleg;

    Vector7d DesiredLeftLegPoseAsAxisAngles;
    Vector7d DesiredRightLegPoseAsAxisAngles;
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    StepsGenerator();
    ~StepsGenerator();

    bool init(int ThreadPeriod, std::string robotName, int ForceFeedbackType);
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    void Release();
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Run method
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    bool walk();
    bool step();

};

#endif //StepsGenerator_H