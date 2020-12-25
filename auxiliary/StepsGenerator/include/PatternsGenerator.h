
#ifndef PatternsGenerator_H
#define PatternsGenerator_H

#include <string>
#include <iostream>
#include <fstream>

#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <iCub/iKin/iKinFwd.h>

#include "InitBalWlkParameters.h"
#include "TemplateModels.h"
#include "MPCOptimizer.h"
#include "wbhpidcUtilityFunctions.hpp"

#include <qpOASES.hpp>


using namespace std;
using namespace Eigen;

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;


// ====================================================================================
// ====================================================================================
class CpFootTrajectories
{    
    
    // double StepPeriod;       // Step period
    double tm;                  // time

    public :

        VectorXd PrevStanceXY;      // Previous stance foot position
        double PrevStanceAng;       // Previuous stance foot orientation
        // for 3D plot using robot model
        double DelThetap1;          // Relative first future footstep orientation
        double DelTheta0n1;         // Relative previous footstep orientation
        VectorXd sxyn1;             // Relative previous footstep position
        VectorXd sxyp1;             // Relative first future footstep position
        double MaxFootHeight;       // Maximum swing foot height


        // Output Arguments
        double XTrajLeftFoot;     // X trajectory of the left foot
        double YTrajLeftFoot;     // Y trajectory of the left foot
        double ZTrajLeftFoot;     // Z trajectory of the left foot
        double AngTrajLeftFoot;   // Angular trajectory of the left foot
        //
        double XTrajRightFoot;    // X trajectory of the left foot
        double YTrajRightFoot;    // Y trajectory of the left foot
        double ZTrajRightFoot;    // Z trajectory of the left foot
        double AngTrajRightFoot;  // Angular trajectory of the left foot



        int StanceIndicator[2];

        CpFootTrajectories();
        ~CpFootTrajectories();

        void Init(int SptFt[],
               VectorXd RelCoPXYR,
               CpStanceFootPose CoPRef, 
               CP_SelectionMatrices SMx,
                          double z_max);
                           
        void ComputeFootTrajectories(       int SptFt[],
                                        VectorXd RelCoPXYR,
                                    CpStanceFootPose CoPRef,
                                  CP_SelectionMatrices SMx);
        void SetMaxFootHeight(double z_max);
};


// ====================================================================================
// ====================================================================================
class CpGaitTransformations
{
    VectorXd TrslBaseCoM;                   // Translation vector from base to CoM

    public :

        Eigen::Vector3d trsl_com_sft;       // 

        Matrix4d TrsfLeftFootBase;           // left foot relative to the base frame
        Matrix4d TrsfRightFootBase;          // right foot rekative to the base frame
        Matrix4d TrsfBaseInWorld;           // base relative to the world frame
        Matrix4d RefT_icubB_icubF;          // Reference transformation form icub base to icub foot frame

        Matrix4d Trsf_lfoot_com;            // homogeneous transformation left foot in the CoM
        Matrix4d Trsf_rfoot_com;            // homogeneous transformation right foot in the CoM

        Matrix4d Trsf_lfoot_base_world;     // left foot relative to the base frame
        Matrix4d Trsf_rfoot_base_world;     // right foot rekative to the base frame

        Matrix4d Trsf_lfoot_base_icub;     // left foot relative to the base frame
        Matrix4d Trsf_rfoot_base_icub;     // right foot rekative to the base frame

        Vector3d RefTransl_Base_in_Foot;    // icub base position expresssed in icub stance foot frame.

        double z_CoM;

        CpGaitTransformations();
        ~CpGaitTransformations();

        void Init(Discrete_CP_LIP_Model St,
                                       int SptFt[],
                           CpStanceFootPose CoPRef,
                          CpFootTrajectories FtTraj,
                          VectorXd t_B_CoM);

        void ComputeGaitTransforms(Discrete_CP_LIP_Model St,
                                            int SptFt[],
                                CpStanceFootPose CoPRef,
                               CpFootTrajectories FtTraj);
                             
        void SetTranslationBaseCoM(VectorXd t_B_CoM);
        
};

// ====================================================================================
// ====================================================================================
class CpReactiveWalkingController
{
    // initialisation of the optimal solution
        VectorXd RelCoPXY;
        VectorXd OrienCoP;
        VectorXd RelCoPXYR;
        VectorXd RelCoPXYR_n1;
        VectorXd OptimSolXY_n1;

        MatrixXd MconsXY;
        VectorXd BconsXY;
        //
        std::string RobotName;

    public :

        Discrete_CP_LIP_Model           DMod;
        MpcBased_CP_Model               MpcModel;
        CP_SelectionMatrices            SMx;
        CpStanceFootPose                CoPref;
        VelocitiesSetPoints             VeloRef;
        CpQMatrix                       QMx;
        CpPVectorQP                     PVec;
        CpConstraintsFootsteps          CnstrFtStp;
        CpConstraintsOutputZmp          CnstrZmp;
        CP_QPSolver_OASES               QPxySolver;
        CP_QPSolver_OASES               QPAngSolver;
        CpFootTrajectories              FtTraj;
        CpGaitTransformations           GaitTrsf;

        // Kinematic transformations
        KineTransformations Transforms;

        double step_size_x;
        double step_size_y;
        double step_theta;
        

        std::string stance_foot;  // stance foot
        bool stance_right;

        // CpDesiredFeetTransformations    *DesFtTrsf;
        int SwitchCounter;
        int CtrlCounter;
        int Tgbs;   // triggered before switching of DSP constraints  
        int Tgas;   // triggered after switching of DSP constraints 
        int DTds;   // nb of samples before or after the switch 
        // limits of the support polygon
        VectorXd    Spolygon_limits;
        //int_t nWSR;
        int nWSR;
        bool WaitForContact;

        double dx_com_normalized;


        // -----------------------
        Vector7d pose_lf;
        Vector7d pose_rf;
        Vector2d a_pos_ft;
        Vector2d a_pos_lf;
        Vector2d a_pos_rf;
        Vector2d a_pos_l_rf;
        Vector2d a_pos_r_lf;

        double a_ori_ft;
        double a_ori_lf;
        double a_ori_rf;
        double a_ori_l_rf;
        double a_ori_r_lf;
        //
        bool resetStep;

        double OrientOffest;



        // methods
        CpReactiveWalkingController();
        ~CpReactiveWalkingController();
        void InitializeCpBalWlkController(InitBalWlkParameters *Parameters);
        void UpdateCpBalWlkController(InitBalWlkParameters *Parameters, VectorXd RelativeVelo, int CycleCounter);
        void ReleaseCpBalWlkController();

        void UpdateFeetPose();
        void set_step_magnitude(Vector3d step_magnitude);

};

// ===============================================================================================
// ===============================================================================================

class SyncRobotModelStanceFoot
{
    public:
        int stance_left;
        int stance_right;
        double minForceThrshld;
        int minTimeThrshld;  

        SyncRobotModelStanceFoot();
        ~SyncRobotModelStanceFoot();

        void InitStanceSync(double F_Threshld, int T_Threshld);

        void SyncStancePhase(                        int CycleCounter,
                                    InitBalWlkParameters *Parameters,
                             CpReactiveWalkingController *CpBalWlkController,
                                                VectorXd l_foot_FT_vector,
                                                VectorXd r_foot_FT_vector);

};

#endif // PatternsGenerator_H