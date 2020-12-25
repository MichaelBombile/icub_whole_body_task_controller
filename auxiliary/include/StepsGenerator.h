// #ifndef StepsGenerator_H
// #define StepsGenerator_H


// #include <string>
// #include <iostream>
// #include <fstream>

// #include <Eigen/Dense>
// #include <Eigen/Core>
// #include <Eigen/SVD>

// #include "InitBalWlkParameters.h"
// #include "CpMath_Utilities.h"
// #include "wbhpidcUtilityFunctions.hpp"
// // #include "RobotModel.h"
// #include "TemplateModels.h"
// #include "MPCOptimizer.h"
// #include "PatternsGenerator.h"
// #include "StatesCompensators.h"

// // #include "Data_logging.h"

// #include <qpOASES.hpp>

// using namespace std;
// using namespace Eigen;


// class StepsGenerator
// {
     
//     public:

//     // internal number of cycles indicator
//     int CycleCounter;
//     int SwitchCounter;
//     // starting time
//     double start_time;
//     // Posture of the robot
//     // RobotPostures           CtrlPostures;

//     InitBalWlkParameters    *Parameters;
//     Vector3d                Des_RelativeVelocity;
//     Vector3d                Feedback_RelativeVelocity;

//     // pattern generator
//     CpReactiveWalkingController          *CpBalWlkController;

//     bool left_stance;
//     // transformation to horizontal (IMU) frame
//     // InertialCompensator  *GaitInIMU;
//       // Usefull Transformation
//     KineTransformations Trnsfrms;
//     // ---------------------------------------
//     KinConverter Pose2Matrix;
//     // dynamic filter for the pattern generator
//     ZmpDynamicFilter DynFilterZmpCoM;
//     // inverse kinematics
//     // InverseKinematicsSolver InvKinSolver_lleg;
//     // InverseKinematicsSolver InvKinSolver_rleg;

//     Vector7d DesiredLeftLegPoseAsAxisAngles;
//     Vector7d DesiredRightLegPoseAsAxisAngles;
//     //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//     StepsGenerator();
//     ~StepsGenerator();

//     bool init(int ThreadPeriod, std::string robotName, int ForceFeedbackType);
//     bool init(int ThreadPeriod, std::string robotName, int ForceFeedbackType, Vector3d init_com_pos);
//     // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//     void Release();
//     // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//     // Run method
//     // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//     bool walk();
//     bool step();

// };

// #endif //StepsGenerator_H


// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

    Vector3d                Des_RelativeVelocity;
    Vector3d                Feedback_RelativeVelocity;
    //
    InitBalWlkParameters            *Parameters;
    // pattern generator
    CpReactiveWalkingController     *CpBalWlkController;

    bool left_stance;
    // transformation to horizontal (IMU) frame
    // InertialCompensator  *GaitInIMU;
      // Usefull Transformation
    KineTransformations Transforms;
    // ---------------------------------------
    KinConverter Pose2Matrix;
    // dynamic filter for the pattern generator
    ZmpDynamicFilter DynFilterZmpCoM;

    Vector7d DesiredLeftLegPoseAsAxisAngles;
    Vector7d DesiredRightLegPoseAsAxisAngles;

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    bool lstance_switch;
    bool rstance_switch;
    Matrix4d Trsf_des_ee2world[6];          // [0] lhand, [1] rhand, [2] lfoot, [3] rfoot, [4] CM   // position
    Matrix4d Trsf_des_ee2base_world[5];     // [0] lhand, [1] rhand, [2] lfoot, [3] rfoot, [4] CM   // position
    Matrix4d Trsf_lstancefoot2world;
    Matrix4d Trsf_rstancefoot2world;
    Matrix4d Trsf_base2world;               // from measurement or estimation
    Matrix4d Trsf_lfoot2base_world;         // form kinematic chain (using joint sensors)
    Matrix4d Trsf_rfoot2base_world;         // form kinematic chain (using joint sensors)
    Matrix4d Trsf_des_com2lfoot;
    Matrix4d Trsf_des_com2rfoot;

    Matrix4d Trsf_des_lfoot2base_robot;
    Matrix4d Trsf_des_rfoot2base_robot;

    StepsGenerator();
    ~StepsGenerator();

    // bool init(int ThreadPeriod, std::string robotName, int ForceFeedbackType);
    bool init(std::string robotName, int ThreadPeriod, int ForceFeedbackType, Vector3d init_com_pos);
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    void Release();
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Run method
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    bool walk(int number_steps, double stride_x, Vector7d des_pose_lh, Vector7d des_pose_rh, WholeBodyTaskSpaceStates wb_ts);
    bool step();

    Matrix4d pose_vector_toHomogenousMatrix(Vector7d Pose_vector);

};

#endif //StepsGenerator_H