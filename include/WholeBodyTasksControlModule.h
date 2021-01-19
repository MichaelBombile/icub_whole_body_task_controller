
/** Class Reference

*/

#pragma once


#ifndef WholeBodyTasksControlModule_H
#define WholeBodyTasksControlModule_H

#include <stdio.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <iomanip>
#include <limits>
#include <termios.h>

#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Property.h>
#include <yarp/os/LogStream.h>

#include <yarp/math/Math.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Rand.h>
#include <yarp/os/Time.h>
#include <yarp/os/RFModule.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>

#include "wbhpidcUtilityFunctions.hpp"
#include "ControllerParameters.h"
#include "RobotInterface.h"
#include "WbRobotModel.h"
#include "ioStateManager.h"
#include "InverseDynamics.h"
#include "wb_InverseKinematics.h"
#include "IK_legs_Solver.hpp"
#include "TasksReferences.h"
#include "Object_to_Grasp.hpp"
#include "BimanualFreeMotionController.h"
#include "BimanualCooperativeController.h"
#include "Manipulation.h"
#include "BalanceReference.h"
#include "FeedForwardController.h"
// #include "auxiliary_functions.h"
// #include "StepperThread.h"
// #include "SteppingController.h"
#include "StepsGenerator.h"

#include "WBData_logging.h"
#include "WlkData_logging.h"


using namespace std;
using namespace yarp::os;
using namespace yarp::math;
// using namespace codyco;

typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;


//! reading keyboard functions
int khbit_2();
void nonblock_2(int state);
bool keyState_2(char key);

// class WholeBodyTasksControlModule
class WholeBodyTasksControlModule : public yarp::os::RFModule
{
    
    private:
        Port handlerPort; // a port to handle messages
        int count = 0;
        int rSampCount = 0;
        int stop_counter;
        int controllerThreadPeriod;
        int actuated_DoFs;
        int torqueControlledDofs;
        double modulePeriod;
        double run_period_sec;

        std::string modelFile;
        std::string robotName;
        std::string moduleName;
        std::string learned_model_path;
        std::string DataID;

            
    public:
        //
        bool                    StopCtrl;
        bool                    isBaseExternal;
        double                  start_time;

        int Cycle_counter;
        // ========================================
        KineTransformations     Transforms;             // Kinematic transformations

        ControllerParameters    *ctrl_param;            // Controller parameters
        WbRobotModel            *robot_model;           // Robot Model
        WbRobotModel            *m_robot_model;         // Robot Model
        RobotInterface          *robot_interface;       // robot interface
        ioStateManager          *ioSM;                  // InputOutput State Manager


        std::string             Base_port_name;
        // controllers
        // ====================================
        InverseDynamics         *IDctrl;                // inverse_dynamics controller
        wb_InverseKinematics    WBIK;                   // WHOLE BODY INVERSE KINEMATICS
        // Manipulation            bimanip;                // Bimanual Manipulation controller
        // BalanceReference        balRef;                 // Bimanual Manicontroller
        // FeedForwardController   ff_ctrl;                // Feeforward Controller with anticopatory action
        IK_legs_Solver          IK_legs;                // Inverse kinemeatics of the legs
        StepsGenerator          step_ctrl;              // SteppingController
        TasksReferences         TasksRefs;              // reference tasks
        
        // Object to grasp
        Object_to_Grasp         object2grasp;
        

        int         period_step;    
        bool        SwitchStep;
        int         stepCount;
        double      delayTrqSwitch;
        double      t_EndStep;
        Vector7d    W_Pose_aF;    // pose of the absolute foot frame
        VectorXd    predicted_jts_pos;
        Matrix4d    predicted_W_H_B;
        Vector7d    des_X_EE[8];   // for wbIK
        Vector7d    Pose_EE[8];   //
        std::string EE[8];    // end-effectors names
        Vector6d    error_pose[8];
        Vector6d    des_Velo_EE[8];

        VectorXd Wrench_hands_star;

        // Data_logger
        WBData_logging          WbDataLogger;   
        WlkData_logging         WalkDataLogger;

        // =======================================================================================
        
        // whole body reference trajectories (pose, velocity, acceleration)
        WholeBodyTaskSpaceTrajectories  wbTskRef;
        WholeBodyTaskSpaceTrajectories  ini_wbTskRef;
        WholeBodyTaskSpaceAcceleration  wb_desired_tasks_acceleration;
        TaskSpaceForces                 F_imp;      // Impedance force
        // JointspaceStates                JtsStates;

        VectorXd ref_joints_posture;
        VectorXd init_ref_joints_posture;
        Vector7d init_Pose_Chest;
        Vector7d w_desLHand_Pose; 
        Vector7d w_desRHand_Pose;
        //
        // Object
        TaskSpaceTrajectories   ref_trajectories_Object;
        Vector6d                Desired_object_wrench;    
        Vector6d                F_external_object; 
        Matrix6d                object_damping_gain;

        // joints command vector
        VectorXd Joints_pos_cmds;
        VectorXd Joints_vel_cmds;
        VectorXd Joints_pos_filtered_cmds;

        Vector3d des_CoP_lf;
        Vector3d des_CoP_rf;
        Vector3d pred_CoP_lf;
        Vector3d pred_CoP_rf;
        Vector3d des_CoP_robot;
        Vector3d pred_CoP_robot;
        Vector3d Ref_CoP_robot;

        // ================================== Loop =================================
        double nu_Wrench;
        double nu_obj_accel;
        double expo_var;

        int vtimeCounter;
        int vtimeCounter2;
        int vtimeCounter3;
        

        double desired_CoM_height;
        double desired_Pelvis_height;
        double sent_Pelvis_height;
        double prev_Pelvis_height;

        VectorXd sent_joints_pos;
        VectorXd prev_joints_position;
        VectorXd ref_joints_pos;

        CubicInterpolator  CmdSmoother1;
        CubicInterpolator  CmdSmoother2;

        double error_height;
        Vector7d CoM_Pose;
        Vector7d Pelvis_Pose;

        int iter_max;
        double epsilon_height;
        Vector6d lleg_ik_jts;
        Vector6d rleg_ik_jts;
        Vector3d t_Pelvis_CoM;
        double d_theta_torso_pitch;
        Matrix4d rbase_H_des_lfoot;
        Matrix4d rbase_H_des_rfoot;
        Matrix4d cur_rbase_H_des_lfoot;
        Matrix4d cur_rbase_H_des_rfoot;
        Matrix4d com_H_des_lfoot;
        Matrix4d com_H_des_rfoot;

        Matrix4d w_H_des_lfoot;
        Matrix4d w_H_des_rfoot;
        Matrix4d w_H_des_com;

        // ============================================================================
        std::string stance_ft;
        Joints_Measurements m_joints_sensor_wb;
        Matrix4d W_H_B;
        Vector6d VB;
        Vector6d VB0;
        Vector6d VB_dot;
        VectorXd q_;
        VectorXd q_dot_;
        VectorXd q_ddot_;

        // ============================================================================
        // bimanual coordination parameters
        double a_bi;
        double b_bi;
        //
        Vector7d        aF_Pose_lhand;  // l hand pose in absolute foot frame
        Vector7d        aF_Pose_rhand;  // r hand pose in absolute foot frame
        //
        VectorXd        xi_;       // bimanual coordinated task pose
        VectorXd        xi_dot_;   // bimanual coordinated task velocity
        VectorXd        xi_ddot_;  // bimanual coordinated task acceleration
        // VectorXd xh_;       // hands poses (left and right)
        VectorXd        xh_dot_;   // hands task velocity (left and right)
        VectorXd        xh_ddot_;  // hands task acceleration (left and right)
        Vector6d        h_c_;      // centroidal momentum
        Vector6d        h_c_dot_;  // rate of centroidal momentum
        Matrix4d        W_H_aF_;   // absolute foot frame expressed in world frame
        double          angle_reach_x;
        double          angle_reach_y;
        double          angle_reach_z;
        Vector6d        E_centroMomentum;  // expected centroidal momentum
        VectorXd        ref_step;

        // data logger
        std::ofstream OutRecord_pose;
        std::ofstream OutRecord_velo;
        std::ofstream OutRecord_accel;
        std::ofstream OutRecord_config;
        std::ofstream OutRecord_efforts;
        std::ofstream OutRecord_COP;
        std::ofstream OutRecord_Motion;
        std::ofstream OutRecord_ff_ctrl;
        std::ofstream OutRecord_joints;
        std::ofstream OutRecord_ComRef;

        // ============================================================================
        double delta_x;
        double delta_y;
        double delta_z;
        double psi_x;
        double theta_y;
        double phi_z;

        MatrixXd SampleReachYZ;
        //
        Vector6d m_lfoot_wrench_ft;
        Vector6d m_rfoot_wrench_ft;
        Vector6d cwrench_lf;
        Vector6d cwrench_rf;

        double cnstr_mo;
        
        Vector2d balancePerturbation;

        double gain_grasp;
        
        Eigen::MatrixXd GraspMatrixHands;
        VectorXd des_jts_velo;

        Vector3d desCoM;

        double y_offset;
        Matrix4d w_H_lfoot;
        Matrix4d w_H_rfoot;
        //
        double nu_contact;
        Vector2d DCoM_B;

        
        Matrix4d w_desH_cp[2];
        Matrix4d cp_desH_Obj[2];
        Matrix4d hands_desH_cst[2];
        Vector7d RefAbsHandsPose;

        //
        VectorXd q_ref;
        
        VectorXd Sent_JointsCmds;
        //
        //
        double wksp_lim[7]; 
        double Reach_var[3];
        double Reachability_position;
        double Reachability_orientation;
        double time_reach;
        //
        bool StepCmds;
        //
        bool executing_step;
        bool executePosiCmds;
        bool Automatic;
        bool objCtrlKey;
        bool ReleaseAndRetract;
        bool isLifting;
        bool isManip;
        bool isReachable;
        bool grasp;
        bool release;
        bool reach;
        bool isTargetReached;
        bool PositionMode;
        bool PreStepPosture;
        bool iniTorqueMode;
        bool StepCompleted;
        bool StepInitiated;
        bool StartRelease;
        bool userExtWrench;
        bool isAnticipActive;
        bool noExtWrench;
        bool userSendExtWrench;

        Vector3d StepFoot_1;
        Vector3d StepFoot_2;
        int nSampleStep;

        double timeCmdsStep;
        //
        Vector2d Zh_ref_Grasp;
        Vector2d Zh_ref_Manip;
        //
        double kp_grasp;
        double F_grasp;
        double F_ext_z;
        double durationExtWrench;
        double timeExtWrench;
        bool   isExtWrench;
        Vector6d ExtWrench_1;
        Vector6d ExtWrench_2;
        Vector6d Expected_ExtWrench_1;
        Vector6d Expected_ExtWrench_2;
        Matrix3d w_R_pelvis;
        int iter_sm;
        VectorXd Disturb_cx_ ;
        VectorXd Disturb_cy_ ;
        Vector6d External_Wrench_Object;
        Vector2d ref_CoM_xy;

        double time2contact;
        double time2release;
        double time2lift;
        // //
        // Eigen::Matrix<double, 6, 7> Jac_larm_hand;
        // Eigen::Matrix<double, 6, 7> Jac_rarm_hand;

        // External wrench ports
        yarp::os::RpcClient body1_ExtWrench_inputPort;
        yarp::os::RpcClient body2_ExtWrench_inputPort;


        // =======================================================================================
        WholeBodyTasksControlModule(double module_period, std::string n_data); 
        ~WholeBodyTasksControlModule();

        double getPeriod();
        // This is our main function. Will be called periodically every getPeriod() seconds
        bool updateModule();
        // Message handler. Just echo all received messages.
        bool respond(const Bottle& command, Bottle& reply);
        //| Configure function. Receive a previously initialized
        //| resource finder object. Use it to configure your module.
        //| If you are migrating from the old module, this is the function
        //| equivalent to the "open" method.
        bool configure(yarp::os::ResourceFinder &rf);
        
        bool interruptModule();     // Interrupt function.
        void cleanup();
        bool close();               // Close function, to perform cleanup.

        // Whole Body desired tasks (Task space motion and force, rate of change of the centroidal momentum and joint space)
        bool set_desired_whole_body_tasks();  // OrientChest,  aLH ..
        void go_to_desCoM_height(double des_CoM_height, Vector6d& lleg_ik_jts, Vector6d& rleg_ik_jts);

        // bimanual coordination tasks functions
        bool get_Coordinated_task_pose(Vector7d Pose_lhand, Vector7d Pose_rhand, Matrix4d WH_aF_);
        bool get_Coordinated_task_velocity(MatrixXd Jac_l, MatrixXd Jac_r, VectorXd q_dot, Matrix4d WH_aF_);
        bool get_Coordinated_task_acceleration(MatrixXd Jac_l, MatrixXd Jac_r, VectorXd q_ddot, Vector6d DJac_lqdot, Vector6d DJac_rqdot, Matrix4d WH_aF_);
        bool get_centroidal_momentum(MatrixXd Jac_, VectorXd q_dot, Matrix4d WH_aF_);
        bool get_rate_centroidal_momentum(MatrixXd Jac_, VectorXd q_ddot, Vector6d DJac_qdot, Matrix4d WH_aF_);

        Vector6d EstimateBaseAcceleration();
        bool compute_coordinated_task_variables();
        // void make_step(WbRobotModel& robot_model_);
        void make_step();
        void make_step2();
        void get_leg_inverseKin(VectorXd &jts_cmds);
        void correct_legsStepTransforms();
        Vector3d getDesiredPosition_object(Vector2d param_via_pt, Vector3d end_pos_obj, Vector3d obj_pos);

        void record_WbData();
        void record_SteppingData();
        void record_wholeBodyData();
        //
        bool estimate_robot_states(std::string stance_foot_);
        bool estimate_robot_states(Matrix4d w_H_lf, Matrix4d w_H_rf, std::string stance_foot_);
        
        void Keyboard_object_control();
        void Keyboard_reference_object_control();
        void adjust_leg_pos_cmds(VectorXd& jts_pos_cmds);
        //
        bool getRefTaskSpaceMotionFromPose(double k_p, double k_o, double alp);
        bool getRefTaskFromPosture(Vector7d pose_lf, Vector7d pose_rf, std::string StF, VectorXd qd);
        void realign_feet();
        bool ComputeJointsPositonCommands(std::string stanceFT_, bool RRetract, VectorXd& Joints_pos_cmds_);
        void update_wbPoses(ioStateManager& ioSM_, Vector7d (&wbPoses)[8]);
        void getCurrentReferenceGraspPoints(Vector7d w_Pose_lhand, Vector7d w_Pose_rhand, Vector7d w_Pose_ref, Matrix4d (&h_desH_ref)[2]);
        bool get_WorkspaceReachability(double wksp_lim[], double var_[], Vector7d w_Pose_obj, Vector7d w_Pose_lh, Vector7d w_Pose_rh, Vector7d w_Pose_lf, Vector7d w_Pose_rf);    
        Matrix4d getGlobal2LocalWorld(Vector7d pelvis_pose, Matrix4d W_H_absF, Matrix4d mW_H_pel);

        // robot control modes
        bool SwitchControlModes(char c);
        bool check_pre_step_posture();
        bool check_reach_error();

        bool InitiateStepping();
};

#endif // WholeBodyTasksControlModule_H