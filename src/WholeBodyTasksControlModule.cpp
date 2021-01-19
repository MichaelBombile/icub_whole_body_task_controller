/**
 * main function for the wholeBodyHPIDControl of the biped humanoid robot icub
 */

#include "WholeBodyTasksControlModule.h"
//#include "wbhpidcUtilityFunctions.hpp"
// #include "WBTasksQPControllerQTLw4.h"   // to go into the Module class


using namespace std;
using namespace yarp::os;
using namespace yarp::math;
// using namespace codyco;

typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;

// ---------------------------------------------------------------------
//! reading keyboard functions
int khbit_2()
{
    struct timeval tv;
    fd_set fds;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
    return FD_ISSET(STDIN_FILENO, &fds);
}

void nonblock_2(int state)
{
    struct termios ttystate;
    tcgetattr(STDIN_FILENO, &ttystate);

    if ( state == 1)
    {
        ttystate.c_lflag &= (~ICANON & ~ECHO); //Not display character
        ttystate.c_cc[VMIN] = 1;
    }
    else if (state == 0)
    {
        ttystate.c_lflag |= ICANON;
        ttystate.c_lflag |= ECHO;
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
}

bool keyState_2(char key)
{
    bool pressed = false;
    int i = khbit_2(); //Alow to read from terminal
    if (i != 0)
    {
        char c = fgetc(stdin);
        if (c == key)
        {
            pressed = true;
        }
        else
        {
            pressed = false;
        }
    }

    return pressed;
}
// ---------------------------------------------------------------------



WholeBodyTasksControlModule::WholeBodyTasksControlModule(double module_period, std::string n_data) :  count(0)
                                                                               ,  modulePeriod(module_period)
                                                                               ,  DataID(n_data) {}


WholeBodyTasksControlModule::~WholeBodyTasksControlModule()
{ 
    // robot_interface->CloseWholeBodyRobotDevices();
    cleanup();  
}

double WholeBodyTasksControlModule::getPeriod()
{
    // module periodicity (seconds), called implicitly by the module.
    return modulePeriod;
}





// Configure function. Receive a previously initialized resource finder object. Use it to configure your module.
// If you are migrating from the old module, this is the function equivalent to the "open" method.
bool WholeBodyTasksControlModule::configure(yarp::os::ResourceFinder &rf)
{
    using namespace yarp::os;

    count=0;
    Cycle_counter = 0;
    getPeriod();
    // optional, attach a port to the module
    // so that messages received from the port are redirected to the respond method
    // TO DO: CHANGE THE  NAME OF THE PORT
    handlerPort.open("/my_wbTask_module2/rpc:i");
    attach(handlerPort);
    
    // Get the info from the configuration file
    moduleName              = rf.check("name", Value("torqueBalancing"), "Looking for module name").asString();
    robotName               = rf.check("robot", Value("icub"), "Looking for robot name").asString();
    torqueControlledDofs    = rf.check("torque_Dofs", Value(29), "Looking for torque controlled Dofs").asInt();
    controllerThreadPeriod  = rf.check("period", Value(10), "Looking for controller period").asInt();
    period_step             = rf.check("period_step", Value(40), "Looking for stepping controller period").asInt();
    modulePeriod            = rf.check("modulePeriod", Value(0.01), "Looking for module period").asDouble();
    modelFile               = rf.check("modelFile", Value("../conf/model.urdf"), "Looking for model file").asString();
    Base_port_name          = rf.check("Base_port_name", Value("get_root_link_WorldPose:o"), "Looking for plevis port name").asString();
    isBaseExternal          = rf.check("is_base_external", Value(false), "Looking external base estimation").asBool();
    learned_model_path      = rf.check("LearnedModelPath", Value("../LearnedModels/PosturalTask/"), "Looking for learned model path").asString();

    gain_grasp              = rf.check("gain_reach", Value(0.5), "Looking for gain of the reaching task").asDouble();
    kp_grasp                = rf.check("stiffness_grasp", Value(2000.0), "Looking for stiffness of grasp").asDouble();
    F_grasp                 = rf.check("force_grasp", Value(10.0), "Looking for grasping force magnitude").asDouble();
    F_ext_z                 = rf.check("external_force_z", Value(10.0), "Looking for magnitude of vertical force ").asDouble();

    executing_step          = rf.check("init_aStep", Value(false), "Execute anticipatory step as the algorithm starts").asBool();
    executePosiCmds         = rf.check("executePosition", Value(false), "Execute the wB position commands").asBool();
    objCtrlKey              = rf.check("ObjectKeyboardCtrl", Value(true), "Execute the wB position commands").asBool();
    StepCmds                = rf.check("StepCommands", Value(false), "Execute the wB position commands").asBool();
    PositionMode            = rf.check("PositionMode", Value(true), "Execute the wB position commands").asBool();
    userExtWrench           = rf.check("userActExtWrench", Value(true), "Externa;l Wrench activated by user").asBool();
    isAnticipActive         = rf.check("isAnticipActive", Value(true), "activation of anticipatory contyrol").asBool();

    std::cout << " modelFile is : " << modelFile << std::endl;
    // end-effectors names
    // -----------------------------------------
    EE[0] = "left_hand";    EE[1] = "right_hand"; 
    EE[2] = "left_foot";    EE[3] = "right_foot"; 
    EE[4] = "CoM";          EE[5] = "centroidal_momentum"; 
    EE[6] = "Pelvis";       EE[7] = "Chest";
    // -------------------------------------------
    // logger
    // -------------------------------------------
    std::string data_number = DataID;

    string path_log_pose    = "../Data/log_task_pose_"      + data_number + ".txt";
    string path_log_velo    = "../Data/log_task_velo_"      + data_number + ".txt";
    string path_log_accel   = "../Data/log_task_accel_"     + data_number + ".txt";
    string path_log_config  = "../Data/log_robot_config_"   + data_number + ".txt";
    string path_log_efforts = "../Data/log_robot_efforts_"  + data_number + ".txt";
    string path_log_CoP     = "../Data/log_robot_COP_"      + data_number + ".txt";
    string path_log_motion  = "../Data/log_task_motion_"    + data_number + ".txt";
    string path_log_ff_ctrl = "../Data/log_task_ff_ctrl_"   + data_number + ".txt";
    string path_log_ComRef  = "../Data/log_task_ComRef_"    + data_number + ".txt";

    OutRecord_pose.open(path_log_pose.c_str());
    OutRecord_velo.open(path_log_velo.c_str());
    OutRecord_accel.open(path_log_accel.c_str());
    OutRecord_config.open(path_log_config.c_str());
    OutRecord_efforts.open(path_log_efforts.c_str());
    OutRecord_COP.open(path_log_CoP.c_str());
    OutRecord_Motion.open(path_log_motion.c_str());
    OutRecord_ff_ctrl.open(path_log_ff_ctrl.c_str());
    OutRecord_ComRef.open(path_log_ComRef.c_str());

    // -----------------------------------------------------------------------------
    bool threadsStarted = true;
    //
    run_period_sec = (double) 0.001*controllerThreadPeriod;
    //
    nu_Wrench       = 0.0;
    nu_obj_accel    = 0.0;
    nu_contact      = 0.0;
    vtimeCounter    = -20;
    vtimeCounter2   = -100;
    vtimeCounter3   = -50;
    expo_var        = 0.0;
    a_bi = 0.5;
    b_bi = 1.0;
    cnstr_mo        = 0.0;
    
    balancePerturbation.setZero();
    //
    stance_ft = "left";
    m_joints_sensor_wb.resize(torqueControlledDofs);
    W_H_B.setIdentity();
    VB.setZero();
    VB0.setZero();                              // previous base velocity
    VB_dot.setZero();
    q_       = Eigen::VectorXd::Zero(torqueControlledDofs+7);
    q_dot_   = Eigen::VectorXd::Zero(torqueControlledDofs+6);
    q_ddot_  = Eigen::VectorXd::Zero(torqueControlledDofs+6);
    for (int i=0; i<8; i++)  Pose_EE[i].setZero();
        // End-eff impedance forces
    F_imp.setZero();
    Desired_object_wrench.setZero();
    F_external_object.setZero();
    object_damping_gain.setIdentity();
    
    // gain_grasp          = 0.2;
    
    des_jts_velo        = VectorXd::Zero(torqueControlledDofs+6);
    //
    ref_joints_pos.resize(torqueControlledDofs);              
    prev_joints_position.resize(torqueControlledDofs);        
    sent_joints_pos.resize(torqueControlledDofs);             
    //
    Joints_pos_cmds.resize(torqueControlledDofs);
    Joints_vel_cmds = Eigen::VectorXd::Zero(torqueControlledDofs);
    ref_step = VectorXd::Zero(8);
    //
    w_H_lfoot = MatrixXd::Identity(4,4);    w_H_lfoot.block<3,1>(0,3) << 0.0,  0.068, 0.0;
    w_H_rfoot = MatrixXd::Identity(4,4);    w_H_rfoot.block<3,1>(0,3) << 0.0, -0.068, 0.0;
    //
    wbTskRef.setZero();
    ini_wbTskRef.setZero();
    ref_joints_posture.resize(torqueControlledDofs); ref_joints_posture.setZero();
    w_desLHand_Pose.setZero();
    w_desRHand_Pose.setZero();
    //
    stepCount   =  0;
    // period_step = 40;
    //
    rbase_H_des_lfoot     = MatrixXd::Identity(4,4);
    rbase_H_des_rfoot     = MatrixXd::Identity(4,4);
    cur_rbase_H_des_lfoot = MatrixXd::Identity(4,4);
    cur_rbase_H_des_rfoot = MatrixXd::Identity(4,4);
    //
    aF_Pose_lhand.setZero();
    aF_Pose_rhand.setZero();
    xi_     = Eigen::VectorXd::Zero(14);
    xi_dot_ = Eigen::VectorXd::Zero(12);
    xi_ddot_= Eigen::VectorXd::Zero(12);
    xh_dot_ = Eigen::VectorXd::Zero(12);
    xh_ddot_= Eigen::VectorXd::Zero(12);
    h_c_.setZero();
    h_c_dot_.setZero();
    //
    cwrench_lf.setZero();
    cwrench_rf.setZero();
    E_centroMomentum = VectorXd::Zero(6);
    //
    delta_x = 0.0;  delta_y = 0.0;    delta_z = 0.0;
    psi_x   = 0.0;  theta_y = 0.0;    phi_z   = 0.0;
    // reference joint configuration (posture) for the inverse kinematics 
    q_ref.resize(torqueControlledDofs);
    //
    SwitchStep          = false;
    isReachable         = true;
    isManip             = false;
    ReleaseAndRetract   = false;
    executing_step      = false;
    isLifting           = false;
    Automatic           = true;
    grasp               = false;
    release             = true;
    reach               = true;
    isTargetReached     = false;
    iniTorqueMode       = !PositionMode;
    StepCompleted       = false;
    StepInitiated       = false;

    PreStepPosture      = false;
    StartRelease        = true;
    noExtWrench         = false;

    timeCmdsStep        = yarp::os::Time::now();
    durationExtWrench   = 0.0;
    timeExtWrench       = yarp::os::Time::now();
    isExtWrench         = false;
    userSendExtWrench  = false;
    iter_sm             = 0;
    //
    StepFoot_1.setZero();
    StepFoot_2.setZero();
    //
    ExtWrench_1.setZero();
    ExtWrench_2.setZero();
    Expected_ExtWrench_1.setZero();
    Expected_ExtWrench_2.setZero();
    External_Wrench_Object.setZero();
    //
    des_CoP_lf.setZero();
    des_CoP_rf.setZero();
    pred_CoP_lf.setZero();
    pred_CoP_rf.setZero();
    des_CoP_robot.setZero();
    pred_CoP_robot.setZero();
    Ref_CoP_robot.setZero();
    ref_CoM_xy.setZero();

    time2contact = 100.;
    time2release = 10000.;
    time2lift    = 10000.;


    // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // ===================================================================
    // Instantiation of the controller parameters class 
    // ===================================================================  
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ctrl_param = new ControllerParameters();
    ctrl_param->Initialize(robotName, moduleName, torqueControlledDofs, PositionMode);
    ctrl_param->period_step = period_step;
    //
    ctrl_param->with_FFwdCtrl = isAnticipActive;
    // Instantiation of robot interface class//////////////////////////////////////////////////////////////////////////////////////////////////////
    robot_interface = new RobotInterface();                                                         // Initialization
    robot_interface->init(robotName,  moduleName, torqueControlledDofs, controllerThreadPeriod);
    robot_interface->SetExternalPelvisPoseEstimation(isBaseExternal);
    if(isBaseExternal)  robot_interface->SetPelvisPosePortName(Base_port_name);
    // Open the devices to get the encoders values
    // ===========================================
    robot_interface->OpenWholeBodyDevices();
    if(isBaseExternal)  robot_interface->InitializefBasePort();
    Time::delay(0.1);
    // =========================================================================
    // creation of the wbi robot object  : MODEL OF THE ROBOT FROM THE URDF FILE 
    // =========================================================================
    robot_model = new WbRobotModel(robotName, *robot_interface);
    robot_model->init(moduleName, modelFile, ctrl_param->list_of_considered_joints, *robot_interface);
    std::cout << " Robot Model Initialized : " << 0 << std::endl;
    Time::delay(0.5);
    // robot_model->robot_interface.setWholeBodyControlMode(robot_model->robot_interface.CONTROL_MODE_POSITION_DIRECT);
    robot_model->robot_interface.setWholeBodyControlMode(robot_model->robot_interface.CONTROL_MODE_POSITION);
    robot_model->robot_interface.setWholeBodyTrajectoryParameters(ctrl_param->AccelParam_init, ctrl_param->SpeedParam_init);
    // Second robot objec for inverse kinematics
    // ---------------------------------------------
    m_robot_model = new WbRobotModel(robotName, *robot_interface);
    m_robot_model->init(moduleName, modelFile, ctrl_param->list_of_considered_joints, *robot_interface);
    std::cout << " Robot Model Initialized : " << 0 << std::endl;
    //
    ioSM = new ioStateManager(controllerThreadPeriod, robotName, robot_model->getDoFs(), w_H_lfoot, w_H_rfoot, stance_ft, *ctrl_param, *robot_model, *robot_interface);  
    if (!ioSM) {
        yError("Could not create inverse dynamics class controller object.");
        return false;
    }
    threadsStarted = threadsStarted && ioSM->start();
    Time::delay(0.1);
    // =======================================================
    // creation of the whole body Inverse dynamics controller
    // =======================================================
    IDctrl = new InverseDynamics(controllerThreadPeriod, robotName, robot_model->getDoFs(), *ctrl_param, *robot_model, *ioSM);
    if (!IDctrl) {
        yError("Could not create inverse dynamics class controller object.");
        return false;
    }
    std::cout<< " Robot Model DOFs : " << robot_model->getDoFs() <<std::endl;
    // start the wbhpid controller
    // bool threadsStarted = true;
    threadsStarted = threadsStarted && IDctrl->start();
    // set the PID for the legs //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool setPID;
    Joints_PIDs cur_legs_pid;
    if(robotName == "icub") {
        // setPID = robot_interface->setLegsJointsPIDs(ctrl_param->step_legs_pid); //
        robot_interface->getLegsJointsPIDs(cur_legs_pid);
    }
    else  {
        setPID = robot_interface->setLegsJointsPIDs(ctrl_param->step_legs_pid);
        robot_interface->getLegsJointsPIDs(cur_legs_pid);
    }
    // std::cout << " SET  LEGS   PIDs : \t" << setPID << std::endl;
    std::cout << " GET LEFT  LEG Kp : \t" << cur_legs_pid.kp.head(6).transpose() << std::endl;
    std::cout << " GET RIGHT LEG Kp : \t" << cur_legs_pid.kp.tail(6).transpose() << std::endl;
    // //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // =====================================================================
    //              WHOLE BODY INVERSE KINEMATICS
    // =====================================================================
    // //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // WBIK.InitializeWBIK(*m_robot_model,  ioSM->JtsStates.position, ioSM->world_H_fBase, 1.2, 12, 5e-3, 0.450); // 12
    WBIK.InitializeWBIK(*m_robot_model,  ioSM->JtsStates.position, ioSM->world_H_fBase, 1.2, 12, 10e-3, 0.450); // 12
    

    // LEGS INVERSE KINEMTAICS 
    // IK_legs.InitializeIK(*robot_model, 0.90, 30, 1e-4, 0.450);
    // Estimate Initial Robot states 
    // ==========================================================================
    this->estimate_robot_states(stance_ft);
    Joints_pos_cmds          = m_joints_sensor_wb.position;
    Joints_pos_filtered_cmds = m_joints_sensor_wb.position;

    ioSM->Joints_pos_cmds          = ioSM->JtsStates.position;
    ioSM->Joints_pos_filtered_cmds = ioSM->JtsStates.position;
    // ==========================================================================
    // SET INITIAL ROBOT CONFIGURATION
    // ==========================================================================
    if(ctrl_param->init_posture_hands) {
         for(int i=0; i<20; i++) {
            robot_model->robot_interface.setWholeBodyControlReference(ctrl_param->init_joints_config);  // sent the computed torques values SMOOTH THE VALUES
            Time::delay(0.04);
        }
    }
    yarp::os::Time::delay(0.1);
    // sending the robot to an initial posture 
    // =======================================
    // TasksRefs.moveToLegPosture(*robot_model, ctrl_param->i_jts_posture_lleg, ctrl_param->i_jts_posture_rleg, true,  30);           // --------------
    TasksRefs.moveToLegPosture(*robot_interface, *robot_model, ctrl_param->i_jts_posture_lleg, ctrl_param->i_jts_posture_rleg, true,  30); 
    yarp::os::Time::delay(0.1); 
    // ===========================================================================
    // ////// instantiating the object to grasp  
    // ===========================================================================
    object2grasp.Initialize(*ctrl_param, run_period_sec);
    object2grasp.OpenObjectPort(robotName);
    // object2grasp.update_object_state();
    if(isBaseExternal)  { object2grasp.update_object_state(ioSM->wbTS.Pelvis.Pose, ioSM->w_H_absF, ioSM->robot_interface.world_H_pelvis); } 
    else  {   object2grasp.update_object_state(ioSM->w_H_absF.inverse()); }

    object2grasp.Ref_Object.setZero();
    object2grasp.Ref_Object.pose    = object2grasp.Object_Pose;
    // object2grasp.Ref_Object.pose.head(3) << object2grasp.Object_Pose(0),  object2grasp.Object_Pose(1) + 0.0, object2grasp.Object_Pose(2) + 0.25;
    // object2grasp.Ref_Object.pose.head(3)    = ioSM->w_H_absF.block<3,3>(0,0) * Vector3d(object2grasp.Object_Pose(0), 0.00, 0.60) + ioSM->w_H_absF.block<3,1>(0,3); //
    object2grasp.Ref_Object.pose.head(3)    = Vector3d(object2grasp.Object_Pose(0), 0.00, 0.60); //
    //
    cp_desH_Obj[0]     = Transforms.PoseVector2HomogenousMx(object2grasp.Obj_Pose_Gpoints[0]);
    cp_desH_Obj[1]     = Transforms.PoseVector2HomogenousMx(object2grasp.Obj_Pose_Gpoints[1]);
    hands_desH_cst[0]  = Transforms.PoseVector2HomogenousMx(ioSM->wbTS.lhand.Pose);
    hands_desH_cst[1]  = Transforms.PoseVector2HomogenousMx(ioSM->wbTS.rhand.Pose);

    // =====================================================================
    // -------- TASK REFERENCE ---------------------------------------------
    // =====================================================================
    wbTskRef.copy_pose(ioSM->wbTS);
    ref_joints_posture         = ctrl_param->StartingPosture; //ioSM->JtsStates.position;
    init_ref_joints_posture    = ref_joints_posture;
    ini_wbTskRef               = wbTskRef;
    init_Pose_Chest            = ioSM->wbTS.Chest.Pose;
    TasksRefs.ini_wbTskRef     = wbTskRef;

    // Initialization of the end-effectors and CoM reference trajectories
    // -------------------------------------------------------------------
    wb_desired_tasks_acceleration.initialize(robot_model->getDoFs() + 6);

    TasksRefs.TasksReferences_Init(*robot_model, *ctrl_param, object2grasp, *ioSM, wbTskRef, ref_joints_posture, DataID);
    TasksRefs.Init_filters_object(run_period_sec, *ctrl_param);    // For the computation of the object desired motion

    // Initialisation of the direct command smoother
    this->CmdSmoother1.IncreasingInterpolation(4);
    this->CmdSmoother2.IncreasingInterpolation(40);
    // walking data logger
    WalkDataLogger.InitializeLogger(data_number);   
    //  
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    robot_model->robot_interface.setWholeBodyControlMode(robot_model->robot_interface.CONTROL_MODE_POSITION_DIRECT);
    // get Robot states 
    this->estimate_robot_states(stance_ft);
    ref_joints_pos        = prev_joints_position  =   sent_joints_pos       = m_joints_sensor_wb.position;
    //
    ioSM->copy_whole_body_poses2array8(des_X_EE);
    des_X_EE[0] = Transforms.HomogenousMx2PoseVector(object2grasp.w_H_Gpoints[0]);  // left hand            //ioSM->wbTS.lhand.Pose;  //    
    des_X_EE[1] = Transforms.HomogenousMx2PoseVector(object2grasp.w_H_Gpoints[1]);  // right hand           //ioSM->wbTS.rhand.Pose;  //                       
    des_X_EE[4].head(2) = 0.5*(des_X_EE[2].head(2) + des_X_EE[3].head(2));
    // des_X_EE[4](2) = 0.47;
    // des_X_EE[4](0) -= 0.0;  // -0.05 for maximum margin com
    // des_X_EE[0].head(3) = Vector3d(0.30,  0.1+des_X_EE[4](1), 0.40);  //  object desired pose
    // des_X_EE[1].head(3) = Vector3d(0.30, -0.1+des_X_EE[4](1), 0.40);  //  object desired pose
    // des_X_EE[0].tail<4>() = ctrl_param->des_orientation_hands;
    // des_X_EE[1].tail<4>() = ctrl_param->des_orientation_hands;  
    // 
    // compute the expected task related hands wrenches
    // -------------------------------------------------
    Wrench_hands_star = Eigen::VectorXd::Zero(12);
    Wrench_hands_star = TasksRefs.get_expected_object_load(*ctrl_param, object2grasp, *ioSM,  des_X_EE[0], des_X_EE[1],  External_Wrench_Object);
    //
    TasksRefs.get_reference_trajectories_object(   object2grasp.Object_Pose,  *ctrl_param, ctrl_param->K_object, object2grasp.Ref_Object);
    // Motion distribution between the hands 
    TasksRefs.CooperativeCtrl.get_TaskConsistentHandsVelocity(run_period_sec, 1.0*object2grasp.Ref_Object.velocity);   
    //
    double t_wbIK = Time::now();
    //  FEEDFORWARD ANTICIPATORY CONTROLLER 
        // TasksRefs.executing_step = false;
        if(!TasksRefs.executing_step) {
            for(int k=0; k<1; k++){ // 36
                TasksRefs.ff_ctrl.aGstep_run_batch(*m_robot_model, ioSM->JtsStates.position, Wrench_hands_star, des_X_EE, 
                                           ioSM->wbTS.lfoot.Pose, ioSM->wbTS.rfoot.Pose, stance_ft, TasksRefs.executing_step, TasksRefs.ref_step);
            }

            if(nu_Wrench == 0.0) balancePerturbation = TasksRefs.ff_ctrl.Disturb_c_ + TasksRefs.ff_ctrl.fu_z_mgbetaW_ * TasksRefs.ff_ctrl.fmmCoM.head(2);
            else                 balancePerturbation = TasksRefs.ff_ctrl.Disturb_c_ + TasksRefs.ff_ctrl.fu_z_mgbetaW_ * ioSM->wbTS.CoM.Pose.head(2);
        } else if(TasksRefs.executing_step) {
            // perform the step at the end of the execution set TasksRefs.executing_step back to phase
            TasksRefs.executing_step = false;                 // desactivate the step exectution
        }
        std::cout << " VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV is  : \t" << std::endl;
        std::cout << " DES OBject Acceleration is   : \t" << object2grasp.Ref_Object.acceleration.transpose() << std::endl;
        std::cout << " DES OBject Velocity is       : \t" << object2grasp.Ref_Object.velocity.transpose() << std::endl;
        std::cout << " Wrench_hands_star is         : \t" << Wrench_hands_star.transpose() << std::endl;
        std::cout << " PREDICTED FOOTSTEP is        : \t" << TasksRefs.ref_step.transpose() << std::endl;
        // std::cout << " STANCE PREDICTION is  : \t" << TasksRefs.ff_ctrl.a_ctrl.stance_0 << std::endl;
        std::cout << " VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV is  : \t" << std::endl;
    // //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // =====================================
            // step_ctrl.CpBalWlkController->set_step_magnitude(Vector3d(0.00, 0.00, -0.0)); // 0.10, 0.02, 0.00
            // step_ctrl.CpBalWlkController->stance_foot = "right";
            // // for(int i=0; i<32; i++) this->make_step();

            // step_ctrl.CpBalWlkController->set_step_magnitude(Vector3d(0.15, 0.00, 0.00));
            // step_ctrl.CpBalWlkController->resetStep   = true;
            d_theta_torso_pitch = 3.00/180.*M_PI;   if(robotName == "icub")    d_theta_torso_pitch = 7.0*M_PI/180.;
            
            DCoM_B = (ioSM->wbTS.CoM.Pose.head(2) - ioSM->wbTS.Pelvis.Pose.head(2));

            // for (int i=0; i<16; i++) // 32
            // {
            //     // if(i==16){
            //     //     step_ctrl.CpBalWlkController->set_step_magnitude(Vector3d(0.00, -0.04, 0.00));
            //     //     d_theta_torso_pitch = 0.0;
            //     // }
            //     t_wbIK = Time::now();
            //     //
            //     this->make_step();
            //     this->get_leg_inverseKin(ref_joints_pos);
                 
            //     des_X_EE[0] = ioSM->wbTS.lhand.Pose;  //    
            //     des_X_EE[1] = ioSM->wbTS.rhand.Pose;  //   
            //     des_X_EE[2] = Transforms.HomogenousMx2PoseVector(step_ctrl.Trsf_des_ee2world[2]);
            //     des_X_EE[3] = Transforms.HomogenousMx2PoseVector(step_ctrl.Trsf_des_ee2world[3]);
            //     des_X_EE[4] = Transforms.HomogenousMx2PoseVector(step_ctrl.Trsf_des_ee2world[4]);
            //     des_X_EE[5] = Transforms.HomogenousMx2PoseVector(step_ctrl.Trsf_des_ee2world[5]);

            //     DCoM_B = 0.001*DCoM_B+0.999*(ioSM->wbTS.CoM.Pose.head(2) - ioSM->wbTS.Pelvis.Pose.head(2));
            //     des_X_EE[4].head(2) += DCoM_B;

            //     if(step_ctrl.Parameters->StanceIndicator[0] == 1) stance_ft = "left";
            //     else    stance_ft = "right";

            //     for(int i=0; i<4; i++){
            //         WBIK.get_legIK_batch(*m_robot_model, ioSM->JtsStates.position, ioSM->world_H_fBase, des_X_EE, stance_ft, 
            //                              ioSM->wbTS.lfoot.Pose, ioSM->wbTS.rfoot.Pose);
            //     }
            //     std::cout << " Leg JOINTS IK1    : \n" << ref_joints_pos.tail(12).transpose()  << std::endl;
            //     std::cout << " Leg JOINTS WBIK1  : \n" << WBIK.des_jts_pos.tail(12).transpose() << std::endl;
            //     std::cout << " COM-ROOT          : \n" << (ioSM->wbTS.CoM.Pose.head(2) - ioSM->wbTS.Pelvis.Pose.head(2)).transpose() << std::endl;
            //     // ref_joints_pos.tail(12) = WBIK.des_jts_pos.tail(12);
            //     //
            //     if(StepCmds)
            //     {
            //         for(int j=0; j<this->CmdSmoother1.nb_of_points; j++){
            //             sent_joints_pos  = prev_joints_position  + this->CmdSmoother1.Up_Index(j) * (ref_joints_pos  - prev_joints_position);
            //             robot_interface->setControlReferencePositionNoArms(sent_joints_pos);  //
            //             // robot_interface->setControlReferencePositionNoArms(ref_joints_pos);   // sent the computed torques values
            //             yarp::os::Time::delay(0.0005);
            //         }
            //         prev_joints_position  = ref_joints_pos; // Update of the previoous commands
            //     }

            //     double Step_per = (double) 0.001*period_step;
            //     double DelT = ((yarp::os::Time::now()-t_wbIK) >= Step_per) ? 0.0005 : Step_per-(yarp::os::Time::now()-t_wbIK);
            //     yarp::os::Time::delay(DelT);
            //     std::cout << " DelT : \t" << DelT << std::endl;
            //     this->record_SteppingData();
            //     // update the stance foot
            //     ioSM->stance_foot = stance_ft;
            // }
    // ============
    yarp::os::Time::delay(2.0);
    // =============================================================================================================================
    cout << " COM AFTER STEPPING IS         \t" << ioSM->wbTS.CoM.Pose.head(3).transpose() << endl;
    cout << " PELVIS AFTER STEPPING IS      \t" << ioSM->wbTS.Pelvis.Pose.head(3).transpose() << endl;
    cout << " LEFT FOOT AFTER STEPPING IS   \t" << ioSM->wbTS.lfoot.Pose.head(3).transpose() << endl;
    cout << " RIGHT FOOT AFTER STEPPING IS  \t" << ioSM->wbTS.rfoot.Pose.head(3).transpose() << endl;
    std::cout<< "WB INVERSE KIN RUNS IN : " << Time::now()-t_wbIK << " s" << std::endl;

    // =================================================================================
    // Openning port for external wrench ports
    // =================================================================================
    robot_interface->OpenExternalWrenchPort(body1_ExtWrench_inputPort, robotName);
    robot_interface->OpenExternalWrenchPort(body2_ExtWrench_inputPort, "iCub2");
    w_R_pelvis = Transforms.PoseVector2HomogenousMx(ioSM->wbTS.Pelvis.Pose).block(0,0,3,3);

    // /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // --------------------------------------------------------------------------
    // Logging the data
    // --------------------------------------------------------------------------
    // measured feet force torque in the feet end-effector frame
    m_lfoot_wrench_ft = robot_model->robot_interface.l_foot_FT_vector;
    m_rfoot_wrench_ft = robot_model->robot_interface.r_foot_FT_vector;
    // computed feet forces torques (QP inverse dynamics)
    cwrench_lf = IDctrl->optimal_feet_contact_forces.head<6>();
    cwrench_rf = IDctrl->optimal_feet_contact_forces.tail<6>();

    // =================================================================================
    // logger
    // =================================================================================
    // bimanual coordinated task variables (pose, velocity twist and acceleration)
    this->compute_coordinated_task_variables();    
    // configuration, efforts and motion data 
    this->record_wholeBodyData();
    // whole body task data 
    WbDataLogger.InitializeLogger(DataID);  
    this->record_WbData();
    this->record_SteppingData();
    // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // ==================================
    // control mode
    // ==================================
    // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    TasksRefs.FreeMotionCtrl->gamma_reachable_p = 0.0;
    TasksRefs.FreeMotionCtrl->gamma_reachable_o = 0.0;


    // ctrl_param->writeCommands = ctrl_param->runtimeCommands;
    ctrl_param->writeCommands = !PositionMode;
    //  Set the control mode 
        // ----------------------
    if (ctrl_param->writeCommands) {
        robot_model->robot_interface.setWholeBodyControlMode(robot_model->robot_interface.CONTROL_MODE_TORQUE);
    } 
    else {
        std::cerr << "Deactivating control\n";
        // robot_model->robot_interface.setWholeBodyControlMode(robot_model->robot_interface.CONTROL_MODE_POSITION);
        robot_model->robot_interface.setWholeBodyControlMode(robot_model->robot_interface.CONTROL_MODE_POSITION_DIRECT);
        // robot_model->robot_interface.setWholeBodyPostionArmsTorqueModes("DIRECT");
    }
    // ==================================================================================================================
        double Kp = 20.;
        double Kv = 0.707*0.25 * sqrt(Kp); //Kv = 20.0;
        //
        for(int i=0; i<4; i++){
            // robot_interface->device_larm.iimp->setImpedance(i, Kp, Kv);
            // robot_interface->device_rarm.iimp->setImpedance(i, Kp, Kv);
        }
        for(int i=0; i<4; i++){
            // robot_interface->device_larm.iint->setInteractionMode(i, VOCAB_IM_COMPLIANT);
            // robot_interface->device_rarm.iint->setInteractionMode(i, VOCAB_IM_COMPLIANT);
        }
    // ==================================================================================================================
    
    // set the reference posture to the joints value after squat
    ref_joints_posture    = ioSM->JtsStates.position;
    ref_joints_pos        = prev_joints_position  =  sent_joints_pos  = ioSM->JtsStates.position;
    //
    ioSM->Joints_pos_cmds          = ioSM->JtsStates.position; //m_joints_sensor_wb.position;
    ioSM->Joints_pos_filtered_cmds = ioSM->JtsStates.position; //m_joints_sensor_wb.position;
    // reference posture for the inverse kinematics
    q_ref = WBIK.q_0;
    q_ref.tail(12) = ioSM->JtsStates.position.tail(12);
    //
    ioSM->copy_whole_body_poses2array8(des_X_EE);        
    des_X_EE[4].head(2) = 0.5*(des_X_EE[2].head(2) + des_X_EE[3].head(2));
    // des_X_EE[4](2) = 0.47;
    // des_X_EE[4](0) -= 0.0;  // -0.05 for maximum margin com
    // des_X_EE[0].head(3) = Vector3d(0.30,  0.1+des_X_EE[4](1), 0.30);     des_X_EE[0].tail<4>() = ctrl_param->des_orientation_hands;
    // des_X_EE[1].head(3) = Vector3d(0.30, -0.1+des_X_EE[4](1), 0.30);     des_X_EE[1].tail<4>() = ctrl_param->des_orientation_hands;
    
    //
    des_X_EE[5].tail<4>() <<  0.0, 0.0, 1.0, M_PI;
    des_X_EE[6].tail<4>() <<  0.0, 0.0, 1.0, M_PI;

    //
    desCoM      = ioSM->wbTS.CoM.Pose.head(3);
    ref_CoM_xy  = ioSM->wbTS.CoM.Pose.head(2);
  
    
    // prepare for reading the keyboard keys
    nonblock_2(1);  // <<<<<<<<<<<<<<<<<<<<<<<< ============= block the display of character in the terminal
    //
    // initialize starting time for the running loop
    start_time = Time::now();
    //
    return threadsStarted && true;
}

// ====================================================================================
// This is our main function. Will be called periodically every getPeriod() seconds
// ====================================================================================
bool WholeBodyTasksControlModule::updateModule()
{
    double t_update = Time::now();
    std::cout << " ================================================================================================= \t" << std::endl;
    std::cout << " VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV \t" << std::endl;
    std::cout << " ================================================================================================= \t" << std::endl;
    cout << "[" << count << "]" << " updateModule..." << endl;
    
    if(khbit_2() != 0)
    {
        char c = fgetc(stdin);
        fflush(stdin);
        switch(c)
        {
            // position
            case 'a': delta_x -= 0.01; break;
            case 's': delta_x += 0.01; break;
            case 'd': delta_y -= 0.01; break;
            case 'f': delta_y += 0.01; break;
            case 'z': delta_z -= 0.01; break;
            case 'w': delta_z += 0.01; break;
            //orientation
            case 'h': psi_x   -= 0.05; break;
            case 'j': psi_x   += 0.05; break;
            case 'k': theta_y -= 0.05; break;
            case 'l': theta_y += 0.05; break;
            case 'm': phi_z   -= 0.05; break;
            case 'i': phi_z   += 0.05; break;
            // 
            // control modes
            case '1': {char key = '1'; ioSM->SwitchControlModes(key);} break;
            case '2': {char key = '2'; ioSM->SwitchControlModes(key);} break;
            case '3': {char key = '3'; ioSM->SwitchControlModes(key);} break;

            // controll of stepping
            case 'b':   
            { 
                // char key = '3'; ioSM->SwitchControlModes(key);  // Full body position mode
                TasksRefs.StepInitiated  = true;
                TasksRefs.timeCmdsStep   = yarp::os::Time::now();
                // TasksRefs.SwitchStep  = true;
                // PreStepPosture = true;
                // ctrl_param->PositionCommands = true;
                TasksRefs.isReachable    = false;
                TasksRefs.FreeMotionCtrl->gamma_reachable_p = 0.0;
                TasksRefs.FreeMotionCtrl->gamma_reachable_o = 0.0;
                TasksRefs.balanceRef.step_ctrl.CpBalWlkController->DMod.zc = ioSM->wbTS.CoM.Pose(2);
                // ctrl_param->AllPositionArmsTorqueMode = true; 
                // ctrl_param->writeCommands = false;
                // ref_joints_pos = prev_joints_position  =  sent_joints_pos  = robot_model->m_jts_sensor_wb.position;  // 
                ioSM->Joints_pos_filtered_cmds = ioSM->Joints_pos_cmds = ioSM->JtsStates.position; //m_joints_sensor_wb.position;
                TasksRefs.stepCount  = 0;
            }   break;

            case 'c':
            {
                TasksRefs.stepCount   = 0;
                TasksRefs.SwitchStep  = true;
                TasksRefs.balanceRef.step_ctrl.CpBalWlkController->DMod.zc = ioSM->wbTS.CoM.Pose(2);
                // step_ctrl.CpBalWlkController->resetStep =  true;
                ref_joints_pos = prev_joints_position   =  sent_joints_pos  = robot_model->m_jts_sensor_wb.position;  //
                ioSM->Joints_pos_filtered_cmds = ioSM->Joints_pos_cmds = ioSM->JtsStates.position; //m_joints_sensor_wb.position;  
            }   break;
            
            case 'r':
            { 
                Vector7d h_Pose_abs;
                Transforms.get_absolute_pose(ioSM->wbTS.lhand.Pose, ioSM->wbTS.rhand.Pose, h_Pose_abs);
                this->getCurrentReferenceGraspPoints(ioSM->wbTS.lhand.Pose, ioSM->wbTS.rhand.Pose, h_Pose_abs, hands_desH_cst);
                // open hands aperture
                hands_desH_cst[0](2,3) -= 0.03; 
                hands_desH_cst[1](2,3) += 0.03; 
                //
                TasksRefs.ReleaseAndRetract = !TasksRefs.ReleaseAndRetract;
                if(TasksRefs.ReleaseAndRetract){
                    gain_grasp = 0.1;
                    reach      = false;
                    TasksRefs.isLifting  = false;
                    TasksRefs.ff_ctrl.ComRefGen.SMx.ResetSelectionMx();
                    TasksRefs.iter_sm      = 0;
                    TasksRefs.StartRelease = false;
                    // Disturb_cx_ = VectorXd::Ones(TasksRefs.ff_ctrl.ComRefGen.nsp);
                    // Disturb_cy_ = VectorXd::Ones(TasksRefs.ff_ctrl.ComRefGen.nsp);
                    // object2grasp.Obj_Pose_Gpoints[0](1) += 0.04;// ctrl_param->object_Pose_Gpoints[0](1) - 0.03;
                    // object2grasp.Obj_Pose_Gpoints[1](1) -= 0.04;// ctrl_param->object_Pose_Gpoints[1](1) + 0.03; 
                }
                else{
                    gain_grasp = 0.1;
                    reach      = true;
                }
            }
            break;

            case 'p':   
            {
                TasksRefs.isReachable = !TasksRefs.isReachable;
                if(TasksRefs.isReachable){
                    TasksRefs.FreeMotionCtrl->gamma_reachable_p = 1.0;
                    TasksRefs.FreeMotionCtrl->gamma_reachable_o = 1.0;
                    gain_grasp = 0.1;
                }  else {
                    TasksRefs.FreeMotionCtrl->gamma_reachable_p = 0.0;
                    TasksRefs.FreeMotionCtrl->gamma_reachable_o = 0.0;
                    gain_grasp = 0.1;
                }
            }   break;

            case 't':
                object2grasp.Obj_Pose_Gpoints[0](1) = ctrl_param->object_Pose_Gpoints[0](1) - 0.01;
                object2grasp.Obj_Pose_Gpoints[1](1) = ctrl_param->object_Pose_Gpoints[1](1) + 0.01;
                isManip = !isManip;
                if(isManip)
                    nu_contact = 1.0;
                else
                    nu_contact = 0.0;
            break;

            case 'o':
            {
                isManip = !isManip;
                if(isManip) {
                    // nu_Wrench    = 1.0;
                    // nu_obj_accel = 1.0;
                    TasksRefs.nu_Wh = 1.0;
                    TasksRefs.nu_Ao = 1.0;
                } else {
                    // nu_Wrench    = 0.0;
                    // nu_obj_accel = 0.0;
                    TasksRefs.nu_Wh = 0.0;
                    TasksRefs.nu_Ao = 0.0;
                }
                
            } break;

            case 'q':   
            {
                TasksRefs.isLifting = !TasksRefs.isLifting;
                this->getCurrentReferenceGraspPoints( ioSM->wbTS.lhand.Pose, ioSM->wbTS.rhand.Pose, 
                                                      object2grasp.States_Object.pose, cp_desH_Obj);
                cp_desH_Obj[0](2,3) += 0.015;  // +0.02
                cp_desH_Obj[1](2,3) -= 0.015;  // -0.02
                TasksRefs.cp_desH_Obj[0] = cp_desH_Obj[0];
                TasksRefs.cp_desH_Obj[1] = cp_desH_Obj[1];
            }   
            break;

            case 'e':
            {
                Vector6d app_Wrench = VectorXd::Zero(6);
                app_Wrench(2)       = -5.0;
                double duration     = 5.0;
                durationExtWrench   = 5.0;
                timeExtWrench       = yarp::os::Time::now();
                isExtWrench         = true; //!isExtWrench;
                userSendExtWrench   = true;
                // robot_interface->applyExternalWrench(body1_ExtWrench_inputPort, "l_hand", app_Wrench, duration);
                // robot_interface->applyExternalWrench(body2_ExtWrench_inputPort, "r_hand", app_Wrench, duration);
                // Matrix3d w_R_pelvis = Transforms.PoseVector2HomogenousMx(ioSM->wbTS.Pelvis.Pose).block(0,0,3,3);
                // robot_interface->applyExternalWrench_world(body1_ExtWrench_inputPort, "l_hand", app_Wrench, duration, w_R_pelvis);
                // robot_interface->applyExternalWrench_world(body2_ExtWrench_inputPort, "r_hand", app_Wrench, duration, w_R_pelvis);
            }
            break;
        }
    }
    //
    cout << " STANCE FOOT IS : " << stance_ft << endl; 
    //
    Expected_ExtWrench_1(2) = -F_ext_z;
    Expected_ExtWrench_2(2) = -F_ext_z;

    double delayWrench = 10.0;
    durationExtWrench  = 10.0;
    if(!userExtWrench && TasksRefs.CooperativeCtrl.ContactConfidence == 1.0 && !TasksRefs.isContact && true) // || userSendExtWrench)
    {
        TasksRefs.isContact = true;
        TasksRefs.t_Contact = yarp::os::Time::now();
        isExtWrench         = true;
        timeExtWrench       = yarp::os::Time::now() + delayWrench;
        time2lift           = yarp::os::Time::now();
    }
    else if(TasksRefs.CooperativeCtrl.ContactConfidence != 1.0)
    {
        TasksRefs.isContact = false;
        userSendExtWrench   = false;
    }

    
    if(isExtWrench)
        time2release = durationExtWrench - (yarp::os::Time::now() - timeExtWrench);
    TasksRefs.time2release = time2release;
    // Apply External wrench
    bool isActiveExtWrench = false;
    if(userExtWrench)
        isActiveExtWrench = isExtWrench && (yarp::os::Time::now() - timeExtWrench) <= durationExtWrench;
    else
        isActiveExtWrench = isExtWrench && ((yarp::os::Time::now() - TasksRefs.t_Contact) >= delayWrench) && (yarp::os::Time::now() - timeExtWrench) <= durationExtWrench;

    if(isActiveExtWrench) {  // apply for 2 sec
    // if(TasksRefs.CooperativeCtrl.ContactConfidence == 1.0) {  // apply for 2 sec
        ExtWrench_1 = 0.93*ExtWrench_1 + 0.07 *Expected_ExtWrench_1;
        ExtWrench_2 = 0.93*ExtWrench_2 + 0.07 *Expected_ExtWrench_2;
        if(fmod(count, 5)==0){
            w_R_pelvis = Transforms.PoseVector2HomogenousMx(ioSM->wbTS.Pelvis.Pose).block(0,0,3,3);  // ioSM->robot_interface.world_H_pelvis
            // w_R_pelvis = ioSM->robot_interface.world_H_pelvis.block(0,0,3,3) * Transforms.PoseVector2HomogenousMx(ioSM->wbTS.Pelvis.Pose).block(0,0,3,3).transpose();  // 
            robot_interface->applyExternalWrench_world(body1_ExtWrench_inputPort, "l_hand", ExtWrench_1, 0.05, w_R_pelvis);
            robot_interface->applyExternalWrench_world(body2_ExtWrench_inputPort, "r_hand", ExtWrench_2, 0.05, w_R_pelvis);
        }
        // if(!noExtWrench){
        //     ExtWrench_1 = Expected_ExtWrench_1;
        //     ExtWrench_2 = Expected_ExtWrench_2;
        //     robot_interface->applyExternalWrench_world(body1_ExtWrench_inputPort, "l_hand", ExtWrench_1, durationExtWrench, w_R_pelvis);
        //     robot_interface->applyExternalWrench_world(body2_ExtWrench_inputPort, "r_hand", ExtWrench_2, durationExtWrench, w_R_pelvis);
        //     noExtWrench = false;
        // }
    }
    // else if (yarp::os::Time::now() - timeExtWrench > 2.0) {
    else  {
        // isExtWrench = false;
        // Expected_ExtWrench_1(2) = 0.0;
        // Expected_ExtWrench_2(2) = 0.0;
        noExtWrench = false;
        ExtWrench_1.setZero();
        ExtWrench_2.setZero();
    }

    // //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // ===========================================================================
    // if(TasksRefs.StepInitiated && (yarp::os::Time::now() - TasksRefs.timeCmdsStep) >= 4.0)  //check_reach_error()
    if(TasksRefs.StepInitiated && this->check_reach_error()) {
        TasksRefs.StepInitiated  = false;
        PreStepPosture = true;
        char key = '3'; ioSM->SwitchControlModes(key);                          // Full body position mode
        // Joints_pos_filtered_cmds = Joints_pos_cmds = ioSM->JtsStates.position;  //m_joints_sensor_wb.position;
        ctrl_param->PositionCommands = true;
    }
    // ====================================
    if(PreStepPosture) {
        if(check_pre_step_posture()){
            TasksRefs.SwitchStep = true;
            TasksRefs.stepCount  = 0;
            TasksRefs.balanceRef.step_ctrl.CpBalWlkController->DMod.zc = ioSM->wbTS.CoM.Pose(2);
            ioSM->Joints_pos_filtered_cmds = ioSM->Joints_pos_cmds = ioSM->JtsStates.position; //m_joints_sensor_wb.position;

            PreStepPosture = false;
            // TasksRefs.executing_step = TasksRefs.SwitchStep;
        }
    }
    // =====================================
    if(iniTorqueMode && TasksRefs.StepCompleted && ((yarp::os::Time::now()-TasksRefs.t_EndStep) >= 1.50))  // introduce a delay
    {
        TasksRefs.StepCompleted = false;
        ctrl_param->PositionCommands = false;
        char key = '1'; ioSM->SwitchControlModes(key);  // switching back Full body torque control
        TasksRefs.isReachable  = true;
        TasksRefs.FreeMotionCtrl->gamma_reachable_p = 1.0;
        TasksRefs.FreeMotionCtrl->gamma_reachable_o = 1.0;
    }
    // ===========================================================================
    // //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // ===========================================================================
    // Object 
    // ===========================================================================
        object2grasp.update_object_state();
        if(isBaseExternal)  {   object2grasp.update_object_state(ioSM->wbTS.Pelvis.Pose, ioSM->w_H_absF, ioSM->robot_interface.world_H_pelvis);     } 
        else                {   object2grasp.update_object_state(ioSM->w_H_absF.inverse());   }

        if(objCtrlKey && false) {   // object2grasp.States_Object.pose.head(3) = ioSM->w_H_absF.block<3,3>(0,0) * Vector3d(0.30, 0.00, 0.30) + ioSM->w_H_absF.block<3,1>(0,3); //
            this->Keyboard_object_control();    } 
        else                    {   this->Keyboard_reference_object_control();  }
        //
        Vector3d end_pos_obj    = ioSM->w_H_absF.block<3,3>(0,0) * Vector3d(0.48, 0.00, 0.6) + ioSM->w_H_absF.block<3,1>(0,3); //Vector3d(0.28, 0.05, 0.6);
        Vector3d temp_via       = ioSM->w_H_absF.block<3,3>(0,0) * Vector3d(0.27, 0, 0.5) + ioSM->w_H_absF.block<3,1>(0,3);
        // Vector3d end_pos_obj    = ioSM->w_H_absF.block<3,3>(0,0) * Vector3d(object2grasp.States_Object.pose(0), 0.00, 0.6) + ioSM->w_H_absF.block<3,1>(0,3); //Vector3d(0.28, 0.05, 0.6);
        // Vector3d temp_via       = ioSM->w_H_absF.block<3,3>(0,0) * Vector3d(object2grasp.States_Object.pose(0), 0, 0.6) + ioSM->w_H_absF.block<3,1>(0,3);
        Vector2d param_via_pt(temp_via(0), 0.95*end_pos_obj(2));
        Vector3d des_pos_obj    = this->getDesiredPosition_object(param_via_pt, end_pos_obj, object2grasp.States_Object.pose.head(3));
        // object2grasp.Ref_Object.pose.head(3)  = des_pos_obj;
        // cout << " des_pos_obj IS : " << des_pos_obj.transpose() << endl;  
        //
        // double tm_obj_yz = 1./(5*(object2grasp.States_Object.pose.segment(1,2) - des_pos_obj.segment(1,2)).norm() + 1e-15);
        double tm_obj_yz = 1./(20*(object2grasp.States_Object.pose.segment(2,1) - des_pos_obj.segment(2,1)).norm() + 1e-15);  // only z
        double alpha_obj = 1.0 - exp(-tm_obj_yz) *(1. + tm_obj_yz);  // critically damped 

        // object2grasp.Ref_Object.pose.segment(1,2)  = des_pos_obj.segment(1,2);
        // object2grasp.Ref_Object.pose(0) = alpha_obj * des_pos_obj(0) + (1. - alpha_obj) * object2grasp.States_Object.pose(0);
        //
        cout << " MOVED OBJECT IS : " << object2grasp.States_Object.pose.head(3).transpose() << endl;    

    // /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // =================================================================================================================================================================
    //             BEGIN TASK SPECIFICATION
    // =================================================================================================================================================================
    // /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // predict the hands related balance disturbance
        // ---------------------------------------------
        
        //
        if(TasksRefs.CooperativeCtrl.ContactConfidence == 1.0) {
            Matrix4d w_desH_o = Transforms.PoseVector2HomogenousMx(object2grasp.Ref_Object.pose);
            Matrix4d w_H_gp_l = w_desH_o * Transforms.PoseVector2HomogenousMx(object2grasp.Obj_Pose_Gpoints[0]);
            Matrix4d w_H_gp_r = w_desH_o * Transforms.PoseVector2HomogenousMx(object2grasp.Obj_Pose_Gpoints[1]);

            des_X_EE[0] = Transforms.HomogenousMx2PoseVector(w_H_gp_l);  // left hand  associated with the desired object pose      
            des_X_EE[1] = Transforms.HomogenousMx2PoseVector(w_H_gp_r);  // right hand associated with the desired object pose
        }
        else {
            des_X_EE[0] = Transforms.HomogenousMx2PoseVector(object2grasp.w_H_Gpoints[0]);  // left hand      //IDctrl->wbTS.lhand.Pose;      
            des_X_EE[1] = Transforms.HomogenousMx2PoseVector(object2grasp.w_H_Gpoints[1]);  // right hand     //IDctrl->wbTS.rhand.Pose;
        }
        // 
        Wrench_hands_star = 0.3*TasksRefs.get_expected_object_load(*ctrl_param, object2grasp, *ioSM,  des_X_EE[0], des_X_EE[1],  External_Wrench_Object); 
        if(!userExtWrench || isActiveExtWrench || true)
        {
            Wrench_hands_star.head(6) += 1.0*Expected_ExtWrench_1;                                  // Load due to the object + expected external hand wrench
            Wrench_hands_star.tail(6) += 1.0*Expected_ExtWrench_2;                                  // Load due to the object + expected external hand wrench
        }
        //
        Vector3d desCoP_absF = Vector3d(0.03, 0.0, 0.0);

        if(Automatic && TasksRefs.isContact && !TasksRefs.ReleaseAndRetract && (yarp::os::Time::now() - time2lift >= 0.5)) {
            TasksRefs.isLifting = true;
        }
        else if(Automatic && !TasksRefs.isContact)  {
            TasksRefs.isLifting = false;
        }

        // Bimanual Manipulation controller
        // ================================
        bool RelRet = TasksRefs.ReleaseAndRetract && TasksRefs.StartRelease;
        TasksRefs.get_TS_bimanip_references(*ctrl_param, object2grasp, *ioSM,  RelRet, TasksRefs.isLifting, wbTskRef);
        // Feedforward postural controller with anticipative stepping
        // ==========================================================
        TasksRefs.get_TS_balance_references(*m_robot_model, *ctrl_param, *ioSM, IDctrl->optimal_feet_contact_forces, desCoP_absF, Wrench_hands_star, stance_ft, TasksRefs.t_EndStep, des_X_EE, wbTskRef);
        // Motion Pelvis and Chest
        // ============================
        TasksRefs.get_TS_posture_references(*ctrl_param, *ioSM, wbTskRef, des_X_EE);  // Pelvis and Chest
        // update the posture reference
        // =============================
        TasksRefs.get_JS_posture_references(  TasksRefs.CooperativeCtrl.ContactConfidence, ioSM->JtsStates.position, init_ref_joints_posture, ctrl_param->Weight_posture, IDctrl->Weight_posture, ref_joints_posture);

        //
        double t_et2c = yarp::os::Time::now();
        // double t2C = 10000.;
        if(yarp::os::Time::now() - start_time <= 1.0)   TasksRefs.estimate_time2contact(*ioSM, *ctrl_param, des_X_EE[0], des_X_EE[1], object2grasp.States_Object.velocity);
        else                             time2contact = 0.99*time2contact+ 0.01*TasksRefs.estimate_time2contact(*ioSM, *ctrl_param, des_X_EE[0], des_X_EE[1], object2grasp.States_Object.velocity);
        TasksRefs.time2contact = time2contact;
        cout << " TIME TO CONTACT IS : " << time2contact << endl;     
        cout << " XXXXXXXXXXXXXXXXX  TIME TO CONTACT SOLVED IN " << yarp::os::Time::now() - t_et2c << " s " << endl; 

    // /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // =================================================================================================================================================================
    //             END TASK SPECIFICATION
    // =================================================================================================================================================================
    // /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // =============================================================================================================================
    // update wb impedance parameters
    // =============================================================================================================================

    if(ctrl_param->apply_wrench && TasksRefs.CooperativeCtrl.ContactConfidence ==1.0 && !TasksRefs.ReleaseAndRetract)  {
        //Impedance
        IDctrl->task_weight[5] = 5.0*ctrl_param->task_weight[5];
        IDctrl->task_weight[6] = 10.*ctrl_param->task_weight[6];
        IDctrl->posture_weight = ctrl_param->posture_weight_grasp;
        TasksRefs.set_impedance_gain(ctrl_param->vM_Pelvis_s, ctrl_param->vD_Pelvis_s, ctrl_param->vK_Pelvis_s, TasksRefs.wb_TS_gains.Pelvis);       // Pelvis stiff
    }
    else if(TasksRefs.ReleaseAndRetract)  {
        //Impedance
        TasksRefs.set_impedance_gain(ctrl_param->vM_CoM_s, ctrl_param->vD_CoM_s, ctrl_param->vK_CoM_s, TasksRefs.wb_TS_gains.CoM);       // CoM stiff
        memcpy(IDctrl->task_weight, &ctrl_param->task_weight_retract[0], 7 * sizeof *ctrl_param->task_weight_retract); 
        IDctrl->posture_weight = ctrl_param->posture_weight_retract;
    }   else  {
        //Impedance
        TasksRefs.set_impedance_gain(ctrl_param->vM_CoM_c, ctrl_param->vD_CoM_c, ctrl_param->vK_CoM_c, TasksRefs.wb_TS_gains.CoM);       // CoM compliant  
        TasksRefs.set_impedance_gain(ctrl_param->vM_Pelvis_c, ctrl_param->vD_Pelvis_c, ctrl_param->vK_Pelvis_c, TasksRefs.wb_TS_gains.Pelvis);       // Pelvis compliant  
        memcpy(IDctrl->task_weight, &ctrl_param->task_weight[0], 7 * sizeof *ctrl_param->task_weight);
        IDctrl->posture_weight = ctrl_param->posture_weight; 

        cout << " XXXXXXXXXXXXXXXXXXXXXXXXXXX     IN THIS PART vCOM GAINS XXXXXXXXXXXXXXXXXXXXXXXXXXXX " << 1.0 << endl;
    }

    // /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 1. update the wholebody reference tasks
    if(!TasksRefs.update_references(wbTskRef, ref_joints_posture)) {   
        printf("Failed to update the references. \n");
        return false;
    }
    // update the reference
    if(!TasksRefs.get_stack_of_motion_tasks(*ioSM, wbTskRef, TasksRefs.reference_joints_posture, F_imp, wb_desired_tasks_acceleration))   {
        printf("Failed to compute the motion stack of tasks. \n");
        return false;
    }
    // Set the desired task to the ID controller
    // ==========================================
    if(!this->set_desired_whole_body_tasks()){
        printf("Failed to set the desired task for the whole body controller. \n");   // TO DO : add also the desired task of the wbIK 
        return false;
    }   
    // ======================================================================================================================================================================
    // //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    // //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // ===================================================================
    //  TASK EXECUTIONS : WB INVERSE KINEMATICS OR WB INVERSE DYNAMICS
    // ===================================================================
    // Compute the joints commands
    // //////////////////////////////////////////////////////////////////////////////////////////
    if(ctrl_param->PositionCommands && !TasksRefs.SwitchStep)
    {
        if(!ctrl_param->CmdsFromWBID)  // then commands form wbIK
        {
            double t_ik = yarp::os::Time::now();
            // if(fmod(count, 4)==0)
            // {
                ioSM->copy_whole_body_poses2array8(Pose_EE);
                // // computation of desired motion (based on pose error)
                this->getRefTaskSpaceMotionFromPose(1.0, 0.7, 2.0);         
                des_Velo_EE[2].setZero();
                des_Velo_EE[3].setZero();

                des_Velo_EE[0].head(3) = 1.0*gain_grasp* ((1.-TasksRefs.nu_Wh*0.0)*TasksRefs.FreeMotionCtrl->w_velocity_eef[0].head(3));
                des_Velo_EE[1].head(3) = 1.0*gain_grasp* ((1.-TasksRefs.nu_Wh*0.0)*TasksRefs.FreeMotionCtrl->w_velocity_eef[1].head(3));
                des_Velo_EE[0].tail(3) = 2.5*gain_grasp* ((1.-TasksRefs.nu_Wh*0.0)*TasksRefs.FreeMotionCtrl->w_velocity_eef[0].tail(3));
                des_Velo_EE[1].tail(3) = 2.0*gain_grasp* ((1.-TasksRefs.nu_Wh*0.0)*TasksRefs.FreeMotionCtrl->w_velocity_eef[1].tail(3));
                // compute the velocity IK (get the joint velocity)
                //=================================================
                bool retract = (TasksRefs.ReleaseAndRetract || !TasksRefs.isReachable || TasksRefs.isLifting);
                this->ComputeJointsPositonCommands(stance_ft, retract, ioSM->Joints_pos_cmds);
            // }  
            cout << " WBIK IN LOOP SOLVED IN " << yarp::os::Time::now() - t_ik << " s " << endl;
        }   
        else  {
            if(fmod(count, 1)==0)
                ioSM->Joints_pos_cmds = ioSM->Joints_pos_cmds + 0.0*ioSM->Joints_vel_cmds  + 4.0* 0.5* run_period_sec * run_period_sec * IDctrl->Joints_accel_FwD.tail(robot_model->getDoFs()); // IDctrl->optimal_acceleration.tail(robot_model->getDoFs());  //
        }
    }
    //

    // arms joints torques 
    // std::cout << " CURRENT JOINTS POSTURE   is : \t" << IDctrl->JtsStates.position.transpose() << std::endl;
    Vector7d torque_larm = ioSM->h_biasForces.segment( 9,7) - 1.0*ioSM->Jac_larm_hand.transpose()*IDctrl->F_external.lhand;
    Vector7d torque_rarm = ioSM->h_biasForces.segment(16,7) - 1.0*ioSM->Jac_rarm_hand.transpose()*IDctrl->F_external.rhand;
    // =======================================================================================================================================
    // send control commands
    // =======================================================================================================================================
    if(ctrl_param->writeCommands && !ctrl_param->PositionCommands) // torque commands
    {
        // ioSM->SendTorqueCommands(IDctrl->Tau_actuated);   // send torque commands --> Better done directly in the ioSM or inverse dynamics thread
    }
    else    // Position commands
    {   
        if(fmod(count, 1)==0 && executePosiCmds)
        {
            VectorXd arms_trq_cmds(29);  arms_trq_cmds.setZero();
            arms_trq_cmds.segment( 3, 7) = torque_larm;
            arms_trq_cmds.segment(10, 7) = torque_rarm;

            // Joints_pos_filtered_cmds = 0.90* Joints_pos_filtered_cmds + 0.10 * Joints_pos_cmds;
            ioSM->Joints_pos_filtered_cmds = ioSM->Joints_pos_cmds;
            // arm in torque mode and the remaining in position mode
            if(ctrl_param->AllPositionArmsTorqueMode) 
            {
                ioSM->robot_interface.setControlReferenceTorqueArmsOnly(arms_trq_cmds);
                if(TasksRefs.SwitchStep)  ioSM->robot_interface.setControlReferencePositionNoArms(ioSM->Joints_pos_cmds);           // send stepping commands
                else            ioSM->robot_interface.setControlReferencePositionNoArms(ioSM->Joints_pos_filtered_cmds);  // send wbIK commands
            }
            else 
            {
                if(TasksRefs.SwitchStep)  ioSM->robot_interface.setWholeBodyControlReference(ioSM->Joints_pos_cmds);                // send stepping commands
                else            ioSM->robot_interface.setWholeBodyControlReference(ioSM->Joints_pos_filtered_cmds);       // send wbIK commands
            }
        }
    }
    // ===============================================================================================================================================

    // ===============================================================================================================================================
    // feet force torque in the feet end-effector frame
    m_lfoot_wrench_ft = ioSM->robot_interface.l_foot_FT_vector;                                       //getLeftLegForceTorqueValues();
    m_rfoot_wrench_ft = ioSM->robot_interface.r_foot_FT_vector;                                       //getRightLegForceTorqueValues();
    pred_CoP_lf       = TasksRefs.balanceRef.get_cop_from_wrench(IDctrl->optimal_feet_contact_forces.head<6>());    //cwrench_lf  
    pred_CoP_rf       = TasksRefs.balanceRef.get_cop_from_wrench(IDctrl->optimal_feet_contact_forces.tail<6>());    //cwrench_rf  

    // run the whole body torque controller
    // ===================================================================
    if (!ioSM)   { yError("%s: Error. Control thread pointer is zero.", moduleName.c_str());    return false; }                                   
    if (!IDctrl) { yError("%s: Error. Control thread pointer is zero.", moduleName.c_str());    return false; }
    // Record data
    // ====================================================================
    if(fmod(count, 4)==0)
    {       
        this->compute_coordinated_task_variables();    // (bimanual coordinated pose, velocity twist and acceleration)
        this->record_wholeBodyData();                  // record the task data
        this->record_WbData();
        this->record_SteppingData();
    }
    

    //
    // std::cout << " ConstrainedCOM : \t" << ConstrainedCOM.transpose() << endl;
    // std::cout << " pred_CoP_robot : \t" << pred_CoP_robot.transpose() << endl;
    // std::cout << " Ref_CoP_robot : \t"  << Ref_CoP_robot.transpose() << endl;
    //
    // std::cout << " OBJ GRASP POSE : \t" << Transforms.HomogenousMx2PoseVector(object2grasp.w_H_Gpoints[0]).transpose() << endl;
    // std::cout << " OBJ GRASP POSE : \t" << Transforms.HomogenousMx2PoseVector(object2grasp.w_H_Gpoints[1]).transpose() << endl;

    // std::cout << " W_H_aF_ : \n" << ioSM->w_H_absF << endl;
    // // -----------------------------------------------------------------------
    std::cout << " OBJECT     POSE : \t" << object2grasp.Object_Pose.transpose() << endl;
    std::cout << " HAND POSE  LEFT : \t" << ioSM->wbTS.lhand.Pose.transpose() << endl;
    std::cout << " HAND POSE RIGHT : \t" << ioSM->wbTS.rhand.Pose.transpose() << endl;
    // std::cout << " LEFT  FOOT POSE : \t" << ioSM->wbTS.lfoot.Pose.transpose() << std::endl;
    // std::cout << " RIGHT FOOT POSE : \t" << ioSM->wbTS.rfoot.Pose.transpose() << std::endl;
    std::cout << " COM POSE     is : \t" << ioSM->wbTS.CoM.Pose.transpose() << std::endl;
    std::cout << " PELVIS POSE  is : \t" << ioSM->wbTS.Pelvis.Pose.transpose() << std::endl;
    std::cout << " CHEST  POSE  is : \t" << ioSM->wbTS.Chest.Pose.transpose() << std::endl;
    // std::cout << " NORM ERROR PRESTEP  is : \t" << (ioSM->wbTS.Pelvis.Pose.head(2) - ioSM->w_H_absF.block<2,1>(0,3)).norm() << std::endl;
    std::cout << " \n" << std::endl;
    std::cout << " w_velocity_eef left      is : \t" << TasksRefs.FreeMotionCtrl->w_velocity_eef[0].transpose() << std::endl;
    std::cout << " w_velocity_eef right     is : \t" << TasksRefs.FreeMotionCtrl->w_velocity_eef[1].transpose() << std::endl;
    // std::cout << " OPTIMAL HANDS VELOCITY LEFT \t"  << TasksRefs.CooperativeCtrl.optimal_hands_velocity.head<6>().transpose() << endl;
    // std::cout << " OPTIMAL HANDS VELOCITY RIGHT \t" << TasksRefs.CooperativeCtrl.optimal_hands_velocity.tail<6>().transpose() << endl;
    // std::cout << " LEFT  ARM TORQUE  \t" << torque_larm.transpose() << endl;
    // std::cout << " RIGHT ARM TORQUE  \t" << torque_rarm.transpose() << endl;
    // std::cout << " OPTIMAL LEFT  ARM TORQUE  \t" << (IDctrl->Tau_actuated.segment( 3,7)).transpose() << endl;
    // std::cout << " OPTIMAL RIGHT ARM TORQUE  \t" << (IDctrl->Tau_actuated.segment(10,7)).transpose() << endl;
    // std::cout << " \n" << std::endl;
    // -----------------------------------------------------------------------
    // std::cout << " FOOT CONTACT WRENCH  LEFT : \t" << IDctrl->optimal_feet_contact_forces.head(6).transpose() << endl;
    // std::cout << " FOOT CONTACT WRENCH RIGHT : \t" << IDctrl->optimal_feet_contact_forces.tail(6).transpose() << endl;
    // std::cout << " COM IN FOOT    LEFT : \t" << (ioSM->wbTS.CoM.Pose.head(3) - ioSM->wbTS.lfoot.Pose.head(3)).transpose() << endl;
    // std::cout << " COM IN FOOT   RIGHT : \t" << (ioSM->wbTS.CoM.Pose.head(3) - ioSM->wbTS.rfoot.Pose.head(3)).transpose() << endl;

    // std::cout << " COM IN ABSOLUT FOOT : \t" << (ioSM->wbTS.CoM.Pose.head(3) - ioSM->w_H_absF.block<3,1>(0,3)).transpose() << endl;
    // std::cout << " DESIRED  COP   LEFT : \t" << IDctrl->desired_CoP_lfoot.transpose() << endl;
    // std::cout << " DESIRED  COP  RIGHT : \t" << IDctrl->desired_CoP_rfoot.transpose() << endl;
    // std::cout << " PREDICTED COP  LEFT : \t" << pred_CoP_lf.head(2).transpose() << endl;
    // std::cout << " PREDICTED COP RIGHT : \t" << pred_CoP_rf.head(2).transpose() << endl;
    // std::cout << " ESTIMATED COP  LEFT : \t" << -m_lfoot_wrench_ft(4)/m_lfoot_wrench_ft(2) << " and  " << m_lfoot_wrench_ft(3)/m_lfoot_wrench_ft(2) << endl;
    // std::cout << " ESTIMATED COP RIGHT : \t" << -m_rfoot_wrench_ft(4)/m_rfoot_wrench_ft(2) << " and  " << m_rfoot_wrench_ft(3)/m_rfoot_wrench_ft(2) << endl;
    
    std::cout << " CooperativeCtrl.ContactConfidence is : " << TasksRefs.CooperativeCtrl.ContactConfidence << std::endl;
    std::cout << " NU_WH is : " << TasksRefs.nu_Wh << std::endl;
    std::cout << " HAND CONTACT WRENCH  LEFT : \t" << IDctrl->F_external.lhand.transpose() << endl;
    std::cout << " HAND CONTACT WRENCH RIGHT : \t" << IDctrl->F_external.rhand.transpose() << endl;

    std::cout << " LARM FT SENS IN World : \t" << TasksRefs.RotateWrench(ioSM->w_Pose_laFTs, ioSM->robot_interface.l_arm_FT_vector).transpose() << endl;
    std::cout << " RARM FT SENS IN World : \t" << TasksRefs.RotateWrench(ioSM->w_Pose_raFTs, ioSM->robot_interface.r_arm_FT_vector).transpose() << endl;
  


    std::cout << " //////// EXECUTE STEP ////// \t" << count << " //////////////// is  : \t" << TasksRefs.executing_step << std::endl;
    std::cout << " VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV is  : \t" << std::endl;
    std::cout << " Wrench_hands_star is  : \t" << Wrench_hands_star.transpose() << std::endl;
    std::cout << " PREDICTED FOOTSTEP is  : \t" << TasksRefs.ref_step.transpose() << std::endl;
    std::cout << " FMMCOM is  : \t" << TasksRefs.ff_ctrl.fmmCoM.transpose() << std::endl;
    std::cout << " VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV is  : \t" << std::endl;  

    // std::cout << " COM GAINS : \n" << TasksRefs.wb_TS_gains.CoM.K << endl;

    std::cout << " \n" << std::endl;

    std::cout<< "Module run in  : " << Time::now()-t_update << " s" << std::endl;
    std::cout<< "RUNNING TIME IS : " << Time::now()-start_time << " s" << std::endl;
    std::cout<< "MODULE COUNTER is : " << count << std::endl;    
    std::cout<< "DELTA MODULE TIME IS : " << (t_update-start_time) << " s" << std::endl;
    std::cout<< "==========================================================>>>>>>>" << std::endl;

    std::cout << " m_lfoot_wrench_ft : \t" << m_lfoot_wrench_ft.transpose() << std::endl;
    std::cout << " m_rfoot_wrench_ft : \t" << m_rfoot_wrench_ft.transpose() << std::endl;
    
    // stop if the robot loses contact with the ground
    if ((count > 25) && (fabs(m_lfoot_wrench_ft(2)) + fabs(m_rfoot_wrench_ft(2)) < 1.0)) // 120   200
    {
        std::cout<< " Module stop now due to Low contact forces " << std::endl;
        // this->close();
        WholeBodyTasksControlModule::stopModule();
    }

    // if(count > 50) WholeBodyTasksControlModule::stopModule();
    // update of the counter
    count++;

    return true;
}

// Message handler. Just echo all received messages.
bool WholeBodyTasksControlModule::respond(const Bottle& command, Bottle& reply)
{
    cout << "Got something, echo is on" << endl;
    if (command.get(0).asString() == "quit")
        return false;
    else
        reply = command;
    return true;
}


// Whole Body desired tasks (Task space motion and force, rate of change of the centroidal momentum and joint space)
// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                          UPDATE THE TASK
// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool WholeBodyTasksControlModule::set_desired_whole_body_tasks()  // OrientChest,  aLH ..
{
    // Task space motion
    // -------------------
    // Desired rate of change of the centroidal momentum for the contact forces
    IDctrl->desired_rate_centroidalMomentum = wb_desired_tasks_acceleration.CMdot;
    //
    IDctrl->des_X_dot_EE[0] =     wb_desired_tasks_acceleration.CMdot  - ioSM->DJacobianDq_centro_momentum;
    IDctrl->des_X_dot_EE[1] =     wb_desired_tasks_acceleration.lhand  - ioSM->DJacobianDq_left_hand;
    IDctrl->des_X_dot_EE[2] =     wb_desired_tasks_acceleration.rhand  - ioSM->DJacobianDq_right_hand;
    IDctrl->des_X_dot_EE[3] = 0.0*wb_desired_tasks_acceleration.lfoot  - ioSM->DJacobianDq_left_foot;
    IDctrl->des_X_dot_EE[4] = 0.0*wb_desired_tasks_acceleration.rfoot  - ioSM->DJacobianDq_right_foot;
    IDctrl->des_X_dot_EE[5] =     wb_desired_tasks_acceleration.Pelvis - ioSM->DJacobianDq_pelvis;
    IDctrl->des_X_dot_EE[6] =     wb_desired_tasks_acceleration.Chest  - ioSM->DJacobianDq_Chest;
    // Joint space acceleration
    IDctrl->des_q_dot.head(6) *=  0.0;
    IDctrl->des_q_dot.tail(torqueControlledDofs) =  wb_desired_tasks_acceleration.posture.tail(torqueControlledDofs);
    // Task space Force
    // -------------------
    IDctrl->F_external.lhand = TasksRefs.appWrench_lhand + 1.0*ExtWrench_1;
    IDctrl->F_external.rhand = TasksRefs.appWrench_rhand + 1.0*ExtWrench_2;

    // if(isExtWrench && (yarp::os::Time::now() - timeExtWrench) <= 2.0 && true) {
    //     IDctrl->F_external.lhand = TasksRefs.appWrench_lhand + ExtWrench_1;
    //     IDctrl->F_external.rhand = TasksRefs.appWrench_rhand + ExtWrench_2;
    // }
    // reference CoP for ID
    IDctrl->desired_CoP_lfoot  = 0.0*pred_CoP_lf.head(2);
    IDctrl->desired_CoP_rfoot  = 0.0*pred_CoP_rf.head(2);

    return true;
}

// Interrupt function.
bool WholeBodyTasksControlModule::interruptModule()
{
    //
    cout << "Interrupting your module, for port cleanup" << endl; 
    // freeing the memory allocated to robot_model          
    return true;
}

void WholeBodyTasksControlModule::cleanup()
{
    // return the display of character in the terminal
    // nonblock_2(0);

    // set the PID for the legs Position
    robot_model->robot_interface.setLegsJointsPIDs(ctrl_param->init_legs_pid);
    // setting back the control mode into position control mode
    robot_model->robot_interface.setWholeBodyControlMode(robot_model->robot_interface.CONTROL_MODE_POSITION);
    robot_model->robot_interface.setWholeBodyTrajectoryParameters(ctrl_param->AccelParam_final, ctrl_param->SpeedParam_final);
    // TasksRefs.moveToLegPosture(*robot_interface, *robot_model, ctrl_param->f_jts_posture_lleg, ctrl_param->f_jts_posture_rleg, true,  30);
    // Time::delay(3.0)

    for(int i=0; i<4; i++)
    {
        robot_interface->device_larm.iint->setInteractionMode(i, VOCAB_IM_STIFF);
        robot_interface->device_rarm.iint->setInteractionMode(i, VOCAB_IM_STIFF);
    }

    // //close controller thread
    // if(TasksRefs.FreeMotionCtrl) 
    // {
    //     delete TasksRefs.FreeMotionCtrl;
    //     TasksRefs.FreeMotionCtrl = 0;
    // }

    //close controller thread
    if(IDctrl) 
    {
        // IDctrl->stop();
        delete IDctrl;
        IDctrl = 0;
    }
    // ioStateManager
    if(ioSM) {
        delete ioSM;
        ioSM = 0;
    };

    // m_robot_model
    if(m_robot_model) {
        delete m_robot_model;
        m_robot_model = 0;
    }

    if(robot_model) 
    {
        // robot_model->close();
        delete robot_model;
        robot_model = 0;
    }

    if(robot_interface) 
    {
        // robot_model->close();
        delete robot_interface;
        robot_interface = 0;
    }

    if(ctrl_param)
    {
        delete ctrl_param;
        ctrl_param = 0;
    }
    //
    nonblock_2(0);
}
// Close function, to perform cleanup.
bool WholeBodyTasksControlModule::close()
{
    // //
    nonblock_2(0);
    //
    OutRecord_pose.close();
    OutRecord_velo.close();
    OutRecord_accel.close();
    OutRecord_config.close();
    OutRecord_efforts.close();
    OutRecord_COP.close();
    OutRecord_ff_ctrl.close();
    OutRecord_Motion.close();
    OutRecord_ComRef.close();
    // close the external wrench ports
    robot_model->robot_interface.CloseExternalWrenchPorts(body1_ExtWrench_inputPort);
    robot_model->robot_interface.CloseExternalWrenchPorts(body2_ExtWrench_inputPort);
    //
    WalkDataLogger.Close_files();           //  close the walking data Logger
    WbDataLogger.Close_files();             //  close the whole body data Logger
    step_ctrl.Release();                    //  step_ctrl.releaseStepper();
    //
    TasksRefs.close();
    // if (WalkCtrl)  WalkCtrl->stop();     //  close stepping controller thread
    if(IDctrl)  IDctrl->stop();             //  close controller thread      
    if(ioSM)    ioSM->stop();
    // optional, close port explicitly
    cout << "Calling close function\n";
    handlerPort.close();
    return true;
}


// void WholeBodyTasksControlModule::go_to_desCoM_height(double desired_CoM_height, Vector6d& lleg_ik_jts, Vector6d& rleg_ik_jts)
// {
//     //
//     // robot_model->EstimateRobotStates(stance_ft, m_joints_sensor_wb, W_H_B, VB);
//     robot_model->EstimateRobotStates(*robot_interface, stance_ft, m_joints_sensor_wb, W_H_B, VB);
//     robot_model->getLinkPose(m_joints_sensor_wb.position, W_H_B, "CoM", CoM_Pose);
//     robot_model->getLinkPose(m_joints_sensor_wb.position, W_H_B, "Pelvis", Pelvis_Pose);
//     t_Pelvis_CoM = Pelvis_Pose.head(3) - CoM_Pose.head(3);
//     //
//     Matrix4d rbase_H_des_lfoot = Eigen::MatrixXd::Zero(4,4);  rbase_H_des_lfoot(3,3) = 1.0;
//     Matrix4d rbase_H_des_rfoot = Eigen::MatrixXd::Zero(4,4);  rbase_H_des_rfoot(3,3) = 1.0;
//     rbase_H_des_lfoot(0,0) = -1.0;  rbase_H_des_lfoot(1,1) = -1.0;  rbase_H_des_lfoot(2,2) = 1.0;  rbase_H_des_lfoot(1,3) = -0.068;
//     rbase_H_des_rfoot(0,0) = -1.0;  rbase_H_des_rfoot(1,1) = -1.0;  rbase_H_des_rfoot(2,2) = 1.0;  rbase_H_des_rfoot(1,3) =  0.068;
//     rbase_H_des_rfoot(2,3) = rbase_H_des_lfoot(2,3) = -desired_CoM_height;
//     rbase_H_des_lfoot.block<3,1>(0,3) -= t_Pelvis_CoM;
//     rbase_H_des_rfoot.block<3,1>(0,3) -= t_Pelvis_CoM;
//     // robot_model->getConfigurationStates();
//     // robot_model->UpdateWholeBodyModel("left");
//     lleg_ik_jts = IK_legs.get_legs_IK(*robot_model, "l_sole", rbase_H_des_lfoot, robot_model->m_jts_sensor_wb.position);
//     // robot_model->UpdateWholeBodyModel("left");
//     rleg_ik_jts = IK_legs.get_legs_IK(*robot_model, "r_sole", rbase_H_des_rfoot, robot_model->m_jts_sensor_wb.position);

//     std::cout << " LEG JOINTS  LEFT is  : \t" << 180./M_PI * lleg_ik_jts.transpose() << std::endl;
//     std::cout << " LEG JOINTS RIGHT is  : \t" << 180./M_PI * rleg_ik_jts.transpose() << std::endl;
// }


bool WholeBodyTasksControlModule::get_Coordinated_task_pose(Vector7d Pose_lhand, Vector7d Pose_rhand, Matrix4d WH_aF_)
{
    // Coordinated task pose is (Xa;Xr): Absolute and relative poses
    VectorXd cTsPose(14);   cTsPose.setZero();
    // 
    Vector7d Pose_abs;  Pose_abs.setZero(); // absolute pose
    Vector7d Pose_rel;  Pose_rel.setZero(); // relative pose
    // Transformation of the hand pose in the absolute feet frame
    Matrix4d aF_H_lh = WH_aF_.inverse() * Transforms.PoseVector2HomogenousMx(Pose_lhand);
    Matrix4d aF_H_rh = WH_aF_.inverse() * Transforms.PoseVector2HomogenousMx(Pose_rhand);
    //
    this->aF_Pose_lhand = Transforms.HomogenousMx2PoseVector(aF_H_lh);
    this->aF_Pose_rhand = Transforms.HomogenousMx2PoseVector(aF_H_rh);

    Transforms.get_absolute_pose(aF_Pose_lhand, aF_Pose_rhand, Pose_abs);
    Transforms.get_relative_pose(aF_Pose_lhand, aF_Pose_rhand, Pose_rel);
    //
    this->xi_.head(7) = Pose_abs;
    this->xi_.tail(7) = Pose_rel;

    return true;
}

bool WholeBodyTasksControlModule::get_Coordinated_task_velocity(MatrixXd Jac_l, MatrixXd Jac_r, VectorXd q_dot, Matrix4d WH_aF_)
{
    // VectorXd xi_dot(12);  xi_dot.setZero();
    // VectorXd xh_dot(12);  xh_dot.setZero();
    //
    int n_j = q_dot.rows();
    MatrixXd Jac_bi(12,n_j);
    // 6D rotation matrix from world frame to absolute foot frame
    Matrix6d aF_R6_W;   aF_R6_W.setZero();
    aF_R6_W.block<3,3>(0,0) = WH_aF_.block<3,3>(0,0).transpose();
    aF_R6_W.block<3,3>(3,3) = aF_R6_W.block<3,3>(0,0);
    //
    Jac_bi.topRows(6)    = aF_R6_W*Jac_l;
    Jac_bi.bottomRows(6) = aF_R6_W*Jac_r;
    //
    MatrixXd C_hands_ = Transforms.get_bimanual_task_TwistMap(this->a_bi, this->b_bi);

    this->xh_dot_ = Jac_bi * q_dot;
    this->xi_dot_ =  C_hands_ * this->xh_dot_;
    //
    return true;
}

bool WholeBodyTasksControlModule::get_Coordinated_task_acceleration(MatrixXd Jac_l, MatrixXd Jac_r, VectorXd q_ddot, 
                                                                    Vector6d DJac_lqdot, Vector6d DJac_rqdot, Matrix4d WH_aF_)
{
    VectorXd vdot_hat(12); 
    // xi_ddot.resize(12);  xi_ddot.setZero();
    // xh_ddot.resize(12);  xh_ddot.setZero();
    //
    int n_j = q_ddot.rows();
    MatrixXd Jac_bi(12,n_j);
    // 6D rotation matrix from world frame to absolute foot frame
    Matrix6d aF_R6_W;   aF_R6_W.setZero();
    aF_R6_W.block<3,3>(0,0) = WH_aF_.block<3,3>(0,0).transpose();
    aF_R6_W.block<3,3>(3,3) = aF_R6_W.block<3,3>(0,0);
    //
    Jac_bi.topRows(6)    = aF_R6_W*Jac_l;
    Jac_bi.bottomRows(6) = aF_R6_W*Jac_r;
    // velocity related accelration
    vdot_hat.head(6) = aF_R6_W*DJac_lqdot;  
    vdot_hat.tail(6) = aF_R6_W*DJac_rqdot; 
    //
    MatrixXd C_hands_ =  Transforms.get_bimanual_task_TwistMap(this->a_bi, this->b_bi);

    this->xh_ddot_ = (Jac_bi * q_ddot + vdot_hat);
    this->xi_ddot_ =  C_hands_ * this->xh_ddot_;
    //
    return true;
}

bool WholeBodyTasksControlModule::get_centroidal_momentum(MatrixXd Jac_, VectorXd q_dot, Matrix4d WH_aF_)
{
    // hc.setZero();
    // 6D rotation matrix from world frame to absolute foot frame
    Matrix6d aF_R6_W;   aF_R6_W.setZero();
    aF_R6_W.block<3,3>(0,0) = WH_aF_.block<3,3>(0,0).transpose();
    aF_R6_W.block<3,3>(3,3) = aF_R6_W.block<3,3>(0,0);
    //
    this->h_c_ = aF_R6_W *Jac_ * q_dot;
    //
    return true;
}

bool WholeBodyTasksControlModule::get_rate_centroidal_momentum(MatrixXd Jac_, VectorXd q_ddot, Vector6d DJac_qdot, Matrix4d WH_aF_)
{
    // hc_dot.setZero();
    // 6D rotation matrix from world frame to absolute foot frame
    Matrix6d aF_R6_W;   aF_R6_W.setZero();
    aF_R6_W.block<3,3>(0,0) = WH_aF_.block<3,3>(0,0).transpose();
    aF_R6_W.block<3,3>(3,3) = aF_R6_W.block<3,3>(0,0);
    //
    this->h_c_dot_ = aF_R6_W*(Jac_ * q_ddot + DJac_qdot);
    //
    return true;
}

Vector6d WholeBodyTasksControlModule::EstimateBaseAcceleration()
{
    Vector6d VB_dot_; 
    VB_dot_ = (this->VB - this->VB0)/run_period_sec;
    this->VB0 = this->VB;
    return VB_dot_;
}
//
// bimanual coordinated task variables (pose, velocity twist and acceleration)
bool WholeBodyTasksControlModule::compute_coordinated_task_variables()
{
    //
    // robot_model->EstimateRobotStates(stance_ft, m_joints_sensor_wb, W_H_B, VB);
    W_H_B = ioSM->world_H_fBase;
    VB    = ioSM->world_Velo_fBase;
    
    // position
    q_.head(7) = Transforms.HomogenousMx2PoseVector(W_H_B);
    q_.tail(robot_model->getDoFs()) = robot_model->m_jts_sensor_wb.position;

    // velocity
    q_dot_.head(6)  = VB; 
    q_dot_.tail(robot_model->getDoFs())  = robot_model->m_jts_sensor_wb.velocity;

    // acceleration
    VB_dot = this->EstimateBaseAcceleration();
    q_ddot_.head(6) = VB_dot;
    q_ddot_.tail(robot_model->getDoFs()) = robot_model->m_jts_sensor_wb.acceleration; 
    //
    this->get_Coordinated_task_pose(ioSM->wbTS.lhand.Pose, ioSM->wbTS.rhand.Pose, ioSM->w_H_absF);
    this->get_Coordinated_task_velocity(ioSM->Jacobian_left_hand, ioSM->Jacobian_right_hand, q_dot_, ioSM->w_H_absF);
    this->get_Coordinated_task_acceleration(ioSM->Jacobian_left_hand, ioSM->Jacobian_right_hand, q_ddot_, 
                                                      ioSM->DJacobianDq_left_hand, ioSM->DJacobianDq_right_hand, ioSM->w_H_absF);
    //
    this->get_centroidal_momentum(ioSM->Jacobian_centro_momentum, q_dot_, ioSM->w_H_absF);
    this->get_rate_centroidal_momentum(ioSM->Jacobian_centro_momentum, q_ddot_, ioSM->DJacobianDq_centro_momentum, ioSM->w_H_absF);

    return true;
}

//
void WholeBodyTasksControlModule::correct_legsStepTransforms()
{
    Matrix4d des_lfoot_H_des_rbase = rbase_H_des_lfoot.inverse();  // desired robot base in the desired (foot flat wrt to the ground)
    Matrix4d des_rfoot_H_des_rbase = rbase_H_des_rfoot.inverse();  // desired robot base in the desired (foot flat wrt to the ground)

    Matrix3d RotPelvis = ioSM->world_H_fBase.block(0,0, 3,3);
    Vector3d dBa = Transforms.getEulerAnglesXYZ_FixedFrame(ctrl_param->desRot_icubPelvis_W);
    Vector3d rBa = Transforms.getEulerAnglesXYZ_FixedFrame(RotPelvis);

    Matrix4d w_Hnoyaw_des_rbase = MatrixXd::Identity(4,4);
             w_Hnoyaw_des_rbase.block<3,3>(0,0) = Transforms.rot_vector2rpy(Vector3d(0.0, dBa(1), 0.0)) * Transforms.rot_vector2rpy(Vector3d(dBa(0), 0.0, 0.0));
    Matrix4d w_Hnoyaw_cur_rbase = MatrixXd::Identity(4,4);
             w_Hnoyaw_cur_rbase.block<3,3>(0,0) = Transforms.rot_vector2rpy(Vector3d(0.0, rBa(1), 0.0)) * Transforms.rot_vector2rpy(Vector3d(rBa(0), 0.0, 0.0));
    Matrix4d des_rbase_H_cur_rbase = w_Hnoyaw_des_rbase.inverse() * w_Hnoyaw_cur_rbase;

    Matrix4d des_lfoot_H_cur_rbase = des_lfoot_H_des_rbase * des_rbase_H_cur_rbase;  // no Yaw angle
    Matrix4d des_rfoot_H_cur_rbase = des_rfoot_H_des_rbase * des_rbase_H_cur_rbase;  // no Yaw angle

    cur_rbase_H_des_lfoot = des_lfoot_H_cur_rbase.inverse();  // command to be sent the the inverse kinematics
    cur_rbase_H_des_rfoot = des_rfoot_H_cur_rbase.inverse();  // command to be sent the the inverse kinematics
}


//
Vector3d WholeBodyTasksControlModule::getDesiredPosition_object(Vector2d param_via_pt, Vector3d end_pos_obj, Vector3d obj_pos)
{
    //
    double dir_end_pos = atan2(end_pos_obj(1), end_pos_obj(0));
    Vector3d via_pos_obj;
    via_pos_obj(0) = param_via_pt(0) * cos(dir_end_pos);
    via_pos_obj(1) = param_via_pt(0) * sin(dir_end_pos);
    via_pos_obj(2) = param_via_pt(1);

    double gamma_obj;  gamma_obj = 0.0;
    double psd_time;
    double error_height_obj = obj_pos(2) - via_pos_obj(2);
    psd_time   = 1.0/(100.*fabs(error_height_obj)+1e-15);
    gamma_obj  = 1.0 - exp(-psd_time/0.1); //0.06

    std::cout << " GAMMA OBJ is  : \t" << gamma_obj << std::endl;

    if(obj_pos(2) <= 0.95*via_pos_obj(2))
        gamma_obj = 0.0;
    else
        gamma_obj = 1.0; // - exp(-psd_time/0.1); //0.06

    return gamma_obj * end_pos_obj + (1. - gamma_obj) * via_pos_obj;
}



void WholeBodyTasksControlModule::record_WbData()
{
    WbDataLogger.Write_Data(modulePeriod,
                            count,
                            2, // number of eff
                            TasksRefs.FreeMotionCtrl->error_approach,
                            TasksRefs.FreeMotionCtrl->error_aperture,
                            TasksRefs.FreeMotionCtrl->error_synchro,

                            TasksRefs.FreeMotionCtrl->w_velocity_approach,
                            TasksRefs.FreeMotionCtrl->w_velocity_aperture,
                            TasksRefs.FreeMotionCtrl->w_velocity_synchro,

                            TasksRefs.FreeMotionCtrl->w_velocity_ro_gpoint,
                            TasksRefs.FreeMotionCtrl->w_velocity_uvo_gpoint,
                            TasksRefs.FreeMotionCtrl->w_velocity_vo_gpoint, 
                            TasksRefs.FreeMotionCtrl->w_velocity_eef,

                            Transforms.PoseVector2HomogenousMx(object2grasp.Object_Pose),
                            TasksRefs.FreeMotionCtrl->w_H_absHands,
                            TasksRefs.FreeMotionCtrl->w_H_vstar,
                            TasksRefs.FreeMotionCtrl->w_H_v,
                            TasksRefs.FreeMotionCtrl->w_H_hand,
                            TasksRefs.FreeMotionCtrl->w_H_cp,

                            // Robot
                            // ======
                            // poses
                            ioSM->wbTS.lhand.Pose,
                            ioSM->wbTS.lhand.Pose,
                            ioSM->wbTS.lfoot.Pose,
                            ioSM->wbTS.rfoot.Pose,
                            ioSM->wbTS.Pelvis.Pose,
                            ioSM->wbTS.CoM.Pose, 

                            // wrenches
                            // =============
                            // computed
                            TasksRefs.nu_Wh*TasksRefs.CooperativeCtrl.optimal_contact_wrench_hands.head<6>(),
                            TasksRefs.nu_Wh*TasksRefs.CooperativeCtrl.optimal_contact_wrench_hands.tail<6>(),
                            IDctrl->optimal_feet_contact_forces.head<6>(),
                            IDctrl->optimal_feet_contact_forces.tail<6>(),
                            // compensated
                            TasksRefs.CooperativeCtrl.torque_correction_lh,
                            TasksRefs.CooperativeCtrl.torque_correction_rh,
                            // Applied
                            IDctrl->F_external.lhand,
                            IDctrl->F_external.rhand,
                            // Estimated
                            ioSM->robot_interface.l_arm_FT_vector, //lh_Wrench_measurements, 
                            ioSM->robot_interface.r_arm_FT_vector, //rh_Wrench_measurements, 
                            ioSM->robot_interface.l_foot_FT_vector, //Est_lf_Wrench,
                            ioSM->robot_interface.r_foot_FT_vector, //Est_rf_Wrench,

                            // Target Object
                            // =============
                            object2grasp.Object_Pose,
                            object2grasp.Ref_Object.pose,
                            object2grasp.Ref_Object.velocity
                                                                );

}

void WholeBodyTasksControlModule::record_SteppingData()
{
    WalkDataLogger.Write_Data(  (double)(count*0.010), //(double)(period_step*0.001),
                                TasksRefs.balanceRef.step_ctrl.CycleCounter, //CpBalWlkController->CtrlCounter,
                                TasksRefs.balanceRef.step_ctrl.CpBalWlkController->CoPref,
                                TasksRefs.balanceRef.step_ctrl.CpBalWlkController->DMod,
                                TasksRefs.balanceRef.step_ctrl.CpBalWlkController->FtTraj,
                                TasksRefs.balanceRef.step_ctrl.CpBalWlkController->VeloRef,
                                TasksRefs.balanceRef.lleg_ik_jts,
                                TasksRefs.balanceRef.rleg_ik_jts,
                                robot_model->m_jts_sensor_wb.position.tail(12).head(6),           // 
                                robot_model->m_jts_sensor_wb.position.tail(6),                    // 
                                robot_interface->l_foot_FT_vector,         //
                                robot_interface->r_foot_FT_vector,
                                ioSM->wbTS.CoM.Pose.head(3));                         // ADD the feet pose
}



void WholeBodyTasksControlModule::record_wholeBodyData()
{
    Vector3d aF_Pose_CoM = ioSM->wbTS.CoM.Pose.head(3) - ioSM->w_Pose_absF.head(3);
    
    OutRecord_pose   << q_.transpose() << "    " << xi_.transpose()  << "    " << aF_Pose_lhand.transpose() << "    " << aF_Pose_rhand.transpose() << endl;
    OutRecord_velo   << q_dot_.transpose()   << "    " << xi_dot_.transpose()    <<  "    " << xh_dot_.transpose()    <<  "    " << h_c_.transpose()    << endl;      
    OutRecord_accel  << q_ddot_.transpose()  << "    " << xi_ddot_.transpose()   <<  "    " << xh_ddot_.transpose()   <<  "    " << h_c_dot_.transpose() << endl;  
    OutRecord_config << q_.transpose() << "    ";   
    OutRecord_config << ioSM->wbTS.lhand.Pose.transpose() << "    ";   
    OutRecord_config << ioSM->wbTS.rhand.Pose.transpose() << "    ";   
    OutRecord_config << ioSM->wbTS.lfoot.Pose.transpose() << "    ";   
    OutRecord_config << ioSM->wbTS.rfoot.Pose.transpose() << "    ";   
    OutRecord_config << ioSM->wbTS.CoM.Pose.head(3).transpose() << endl; 

    // 
    double power = time2contact; //+ (yarp::os::Time::now() - start_time); //ioSM->m_wb_joints_sensor.torque.transpose() * q_dot_.tail(robot_model->getDoFs());
    //
    OutRecord_efforts << IDctrl->Tau_actuated.transpose() << "     ";
    OutRecord_efforts << IDctrl->optimal_feet_contact_forces.head(6).transpose() << "     ";
    OutRecord_efforts << IDctrl->optimal_feet_contact_forces.tail(6).transpose() << "     ";
    OutRecord_efforts << m_lfoot_wrench_ft.transpose() << "     ";
    OutRecord_efforts << m_rfoot_wrench_ft.transpose() << "     ";
    OutRecord_efforts << IDctrl->F_external.lhand.transpose() << "     ";
    OutRecord_efforts << IDctrl->F_external.rhand.transpose() << "     " << fabs(power) << endl;

    OutRecord_COP << IDctrl->desired_CoP_lfoot.transpose() << "   ";
    OutRecord_COP << IDctrl->desired_CoP_rfoot.transpose() << "   ";
    OutRecord_COP << pred_CoP_lf.head(2).transpose() << "   ";
    OutRecord_COP << pred_CoP_rf.head(2).transpose() << "   ";
    OutRecord_COP << -m_lfoot_wrench_ft(4)/m_lfoot_wrench_ft(2) << "  " << m_lfoot_wrench_ft(3)/m_lfoot_wrench_ft(2) << "   ";
    OutRecord_COP << -m_rfoot_wrench_ft(4)/m_rfoot_wrench_ft(2) << "  " << m_rfoot_wrench_ft(3)/m_rfoot_wrench_ft(2) << endl;

    OutRecord_Motion << wbTskRef.lhand.velocity.transpose() << "   "; 
    OutRecord_Motion << wbTskRef.rhand.velocity.transpose() << "   "; 
    OutRecord_Motion << wbTskRef.lhand.acceleration.transpose() << "   ";
    OutRecord_Motion << wbTskRef.rhand.acceleration.transpose() << "   ";
    OutRecord_Motion << (1.-TasksRefs.nu_Wh) *TasksRefs.FreeMotionCtrl->w_velocity_eef[0].transpose() << "   ";
    OutRecord_Motion << (1.-TasksRefs.nu_Wh) *TasksRefs.FreeMotionCtrl->w_velocity_eef[1].transpose() << "   ";
    OutRecord_Motion << TasksRefs.CooperativeCtrl.optimal_hands_velocity.head<6>().transpose() << "   ";
    OutRecord_Motion << TasksRefs.CooperativeCtrl.optimal_hands_velocity.tail<6>().transpose() << "   ";
    OutRecord_Motion << ioSM->wbTS.lhand.Velo.transpose() << "   ";
    OutRecord_Motion << ioSM->wbTS.rhand.Velo.transpose() << endl;

    //
    OutRecord_ff_ctrl << TasksRefs.ff_ctrl.isReachable << "   ";
    OutRecord_ff_ctrl << TasksRefs.ff_ctrl.isStable << "   ";
    OutRecord_ff_ctrl << TasksRefs.ff_ctrl.fmmCoM.transpose() << "   ";
    OutRecord_ff_ctrl << balancePerturbation.transpose() << "   ";
    OutRecord_ff_ctrl << TasksRefs.ref_step.transpose() << endl;

    // =====================
    VectorXd St_x(8), St_y(8);
            St_x.head(2)    = TasksRefs.ff_ctrl.ComRefGen.DMod.StatesX.head(2);
            St_x(2)         = TasksRefs.ff_ctrl.ComRefGen.DMod.OutputX;
            St_x(3)         = TasksRefs.ff_ctrl.ComRefGen.DMod.OutputDx;
            St_x(4)         = TasksRefs.ff_ctrl.ComRefGen.Disturb_cx_(0); // + 1.0*Dist_time(0)*fu_z_mgbeta*TasksRefs.ff_ctrl.ComRefGen.DMod.StatesX(0);  // ref disturbance
            St_x(5)         = TasksRefs.ff_ctrl.ComRefGen.DMod.OutputX;  // cop 
            St_x(6)         = TasksRefs.ff_ctrl.ComRefGen.StpRef.FootstepsX(0);   //  ref footstep 
            St_x(7)         = 0.0;      //  Delta Step 

            St_y.head(2)    = TasksRefs.ff_ctrl.ComRefGen.DMod.StatesY.head(2);
            St_y(2)         = TasksRefs.ff_ctrl.ComRefGen.DMod.OutputY;
            St_y(3)         = TasksRefs.ff_ctrl.ComRefGen.DMod.OutputDy;
            St_y(4)         = TasksRefs.ff_ctrl.ComRefGen.Disturb_cy_(0);     //Disturb_cy_(0)+Dist_time(0)*fu_z_mgbeta*DMod.StatesY(0); 
            St_y(5)         = TasksRefs.ff_ctrl.ComRefGen.DMod.OutputY;
            St_y(6)         = TasksRefs.ff_ctrl.ComRefGen.StpRef.FootstepsY(0);   //  (0);
            St_y(7)         = 0.0;   //  Delta step 
    // if(fmod(count, 20)==0)
        // OutRecord_ComRef << count *TasksRefs.ff_ctrl.ComRefGen.sTime << "    " << St_x.transpose() << "  " << St_y.transpose() << std::endl;
        OutRecord_ComRef << count *modulePeriod << "    " << St_x.transpose() << "  " << St_y.transpose() << std::endl;
}



bool WholeBodyTasksControlModule::estimate_robot_states(std::string stance_foot_)
{
    bool ok = true;
    robot_model->EstimateRobotStates(*robot_interface , stance_foot_, m_joints_sensor_wb, W_H_B, VB);
    //
    for (int i=0; i<8; i++){
        if(i!=5){
            ok = ok && robot_model->getLinkPose(m_joints_sensor_wb.position, W_H_B, EE[i],  Pose_EE[i]);   //
        }       
    }
    //
    Pose_EE[5] = Pose_EE[6];  // assinging the pelvis orientation to the robot center of mass

    return ok;
}

//
void WholeBodyTasksControlModule::Keyboard_object_control()
{
    // if(objCtrlKey)
    // {
        // object2grasp.States_Object.pose.head(3) = ioSM->w_H_absF.block<3,3>(0,0) * init_obj_aF + ioSM->w_H_absF.block<3,1>(0,3); //
        object2grasp.States_Object.pose(0) += delta_x;
        object2grasp.States_Object.pose(1) += delta_y;
        object2grasp.States_Object.pose(2) += delta_z;
    // }
    Vector3d c_obj_axis_o = object2grasp.Object_Pose.segment<3>(3)/object2grasp.Object_Pose.segment<3>(3).norm();
    Matrix3d R_obj = Eigen::AngleAxisd(object2grasp.Object_Pose(6), c_obj_axis_o).toRotationMatrix();
    Vector3d ang_obj = Transforms.getEulerAnglesXYZ_FixedFrame(R_obj);
        
    Matrix3d des_Obj_orient;
    des_Obj_orient = Transforms.rot_vector2rpy(Vector3d(ang_obj(0)+psi_x, ang_obj(1)+theta_y, ang_obj(2)+phi_z));
    Eigen::AngleAxisd des_Obj_orient_AA(des_Obj_orient);

    object2grasp.States_Object.pose.segment<3>(3)   = des_Obj_orient_AA.axis();
    object2grasp.States_Object.pose(6)              = des_Obj_orient_AA.angle();
}

void WholeBodyTasksControlModule::Keyboard_reference_object_control()
{
    object2grasp.Ref_Object.pose(0)      += delta_x;
    object2grasp.Ref_Object.pose(1)      += delta_y;
    object2grasp.Ref_Object.pose(2)      += delta_z;
    delta_x = 0.0;
    delta_y = 0.0;
    delta_z = 0.0;
}


// /////////////////////////////////////////////////////////////////////////////////////////////////////////
bool WholeBodyTasksControlModule::ComputeJointsPositonCommands(std::string stanceFT_, bool RRetract, VectorXd& Joints_pos_cmds_)
{
    //
    VectorXd gain_jts_tsk = VectorXd::Ones(robot_model->getDoFs());
    gain_jts_tsk(0) *= 2.0;
    gain_jts_tsk(1) *= 2.0;
    gain_jts_tsk(4)  *=2.5; // shoulder roll
    gain_jts_tsk(11) *=2.5; // shoulder roll

    // des_jts_velo.tail(robot_model->getDoFs()) = -(0.1*WBIK.alpha_q) * gain_jts_tsk.asDiagonal()* (ioSM->JtsStates.position -WBIK.q_0);
    des_jts_velo.tail(robot_model->getDoFs()) = -(WBIK.alpha_q) * gain_jts_tsk.asDiagonal()* (ioSM->JtsStates.position -q_ref);
    //
    VectorXd des_q_dot = WBIK.get_wbIK_velo2(   *m_robot_model, ioSM->Joints_pos_cmds, //ioSM->JtsStates.position, 
                                                ioSM->world_H_fBase, 
                                                des_Velo_EE, des_jts_velo, stanceFT_, 
                                                ioSM->wbTS.lfoot.Pose, 
                                                ioSM->wbTS.rfoot.Pose, 
                                                RRetract);
    // update the desired joint position
    //===================================
    Joints_pos_cmds_ += des_q_dot.tail(robot_model->getDoFs()) * modulePeriod; //run_period_sec * 1.0; //period;
    // Enforcing joints limits
    for(int i=0; i<robot_model->getDoFs(); i++)
    {
        // set joint limits hard limits
        if(Joints_pos_cmds_(i) > WBIK.maxJointLimits(i)){           // upper limit
            Joints_pos_cmds_(i) = WBIK.maxJointLimits(i) - 0.02;
        } else if(Joints_pos_cmds_(i) < WBIK.minJointLimits(i)){    // lower limit
            Joints_pos_cmds_(i) = WBIK.minJointLimits(i) + 0.02;
        } 
    }

    return true;
}

//
bool WholeBodyTasksControlModule::getRefTaskSpaceMotionFromPose(double k_p, double k_o, double alp)
{
    //
    Vector6d gain_task;
    gain_task.head(3) = -Vector3d(k_p, k_p, k_p);
    gain_task.tail(3) = -Vector3d(k_o, k_o, k_o);
    //
    for (int i=0; i<8; i++) 
    {
        if((TasksRefs.ReleaseAndRetract || !TasksRefs.isReachable) && (i==4 || i==5 || i==7)){
            gain_task.head(3) = -alp*Vector3d(k_p, k_p, k_p);
            gain_task.tail(3) = -alp*Vector3d(k_o, alp*k_o, k_o);
        }
        if(i==4){
            gain_task.head(3) = -alp*Vector3d(k_p, k_p, k_p);
            gain_task.tail(3) = -alp*Vector3d(k_o, k_o, k_o);
        }

        error_pose[i]   = Transforms.computePoseErrorNormalizedAxisAngle(Pose_EE[i], des_X_EE[i]);
        des_Velo_EE[i]  = gain_task.asDiagonal() *error_pose[i];                                      // scaled error by convergence rate (gain)
    }   

    std::cout << " PELVIS ORIEN ERROR is : \t" << error_pose[5].tail(3).transpose() << std::endl;
    std::cout << " PELVIS ORIEN ERROR NORM is : \t" << error_pose[5].tail(3).norm() << std::endl;

    return true;
}
//
bool WholeBodyTasksControlModule::getRefTaskFromPosture(Vector7d pose_lf, Vector7d pose_rf, std::string StF, VectorXd qd)
{
    Matrix4d virt_WHB = WBIK.get_fbase_pose(*m_robot_model, pose_lf, pose_rf, StF, qd);
    for (int i=0; i<8; i++){
        if(i!=5){   m_robot_model->getLinkPose(qd, virt_WHB, WBIK.EE[i],  des_X_EE[i]); }   // EE_Pose (FWK)    
    }
    des_X_EE[5] = des_X_EE[6];  // assinging the pelvis orientation to the robot center of mass

    return true;
}

void WholeBodyTasksControlModule::adjust_leg_pos_cmds(VectorXd& jts_pos_cmds)
{
    //
    Vector6d lleg_a = jts_pos_cmds.tail(12).head(6);
    Vector6d rleg_a = jts_pos_cmds.tail(6);

    lleg_a(1) *= step_ctrl.Parameters->HipRollFactor +M_PI*0.6/180.;
    lleg_a(2) *= step_ctrl.Parameters->HipYawFactor;
    // lleg_a(4) -= 0.0;
    lleg_a(5) *= step_ctrl.Parameters->AnkleRollFactor* step_ctrl.Parameters->HipRollFactor;
    lleg_a(5) += 1.8*M_PI/180.;

    rleg_a(1) *= step_ctrl.Parameters->HipRollFactor +M_PI*0.6/180.;
    rleg_a(2) *= step_ctrl.Parameters->HipYawFactor;
    // rleg_a(4) -= 0.0;
    rleg_a(5) *= step_ctrl.Parameters->AnkleRollFactor* step_ctrl.Parameters->HipRollFactor;
    rleg_a(5) += 1.8*M_PI/180.;

    jts_pos_cmds.tail(12).head(6)   << lleg_a; 
    jts_pos_cmds.tail(6)            << rleg_a;

    std::cout << " CORRECTION FACTORS \t" << step_ctrl.Parameters->HipRollFactor << std::endl;
    std::cout << " STEP SAMPLING TIME \t" << step_ctrl.Parameters->SamplingTime << std::endl;
    std::cout << " STEP CoMHeight \t" << step_ctrl.Parameters->CoMHeight << std::endl;
    std::cout << " STEP ROBOTNAME \t" << step_ctrl.Parameters->RobotName << std::endl;
}
// //

void WholeBodyTasksControlModule::getCurrentReferenceGraspPoints(Vector7d w_Pose_lhand, Vector7d w_Pose_rhand, 
                                                                 Vector7d w_Pose_ref, Matrix4d (&h_desH_ref)[2])
{
    // get the current hands poses in the object frame
    // Matrix4d w_H_Obj= Transforms.PoseVector2HomogenousMx(w_Pose_ref);
    // cp_desH_Obj[0]  = Transforms.PoseVector2HomogenousMx(w_Pose_lhand).inverse() * w_H_Obj; 
    // cp_desH_Obj[1]  = Transforms.PoseVector2HomogenousMx(w_Pose_rhand).inverse() * w_H_Obj;
    Matrix4d w_H_ref= Transforms.PoseVector2HomogenousMx(w_Pose_ref);
    h_desH_ref[0]   = Transforms.PoseVector2HomogenousMx(w_Pose_lhand).inverse() * w_H_ref; 
    h_desH_ref[1]   = Transforms.PoseVector2HomogenousMx(w_Pose_rhand).inverse() * w_H_ref;

}
//
// -----------------
bool WholeBodyTasksControlModule::get_WorkspaceReachability(double wksp_lim[], double var_[], 
                                                            Vector7d w_Pose_obj,
                                                            Vector7d w_Pose_lh, Vector7d w_Pose_rh,
                                                            Vector7d w_Pose_lf, Vector7d w_Pose_rf)
{                                                   
    //
    // wksp_lim[0] : lim x
    // wksp_lim[1] : lim y
    // wksp_lim[2] : lim z up  
    // wksp_lim[3] : lim z down
    // wksp_lim[4] : psi around x
    // wksp_lim[5] : theta around y
    // wksp_lim[6] : phi around z 

    // var_[]
    // var_[]
    // var_[]
    // translation  
    double xof, yof, zof;
    // xof = w_H_o_(0,3) - 0.5*(w_H_lf_(0,3) + w_H_rf_(0,3));
    // yof = w_H_o_(1,3) - 0.5*(w_H_lf_(1,3) + w_H_rf_(1,3));
    // zof = w_H_o_(2,3) - 0.5*(w_H_lf_(2,3) + w_H_rf_(2,3));
    //
    Vector7d W_pose_aF_;
    Transforms.get_absolute_pose(w_Pose_lf, w_Pose_rf, W_pose_aF_);
    Matrix4d w_H_af = Transforms.PoseVector2HomogenousMx(W_pose_aF_); 
    Matrix4d aF_H_lh = w_H_af.inverse() * Transforms.PoseVector2HomogenousMx(w_Pose_lh);
    Matrix4d aF_H_rh = w_H_af.inverse() * Transforms.PoseVector2HomogenousMx(w_Pose_rh);
    Matrix4d aF_H_ob = w_H_af.inverse() * Transforms.PoseVector2HomogenousMx(w_Pose_obj);

    xof = aF_H_ob(0,3);
    yof = aF_H_ob(1,3);
    zof = aF_H_ob(2,3);

    std::cout << " xof  is: " << xof << std::endl;
    std::cout << " yof  is: " << yof << std::endl;
    std::cout << " zof  is: " << zof << std::endl;

    double do_xy = sqrt(wksp_lim[1]*wksp_lim[1]*xof*xof + wksp_lim[0]*wksp_lim[0]*yof*yof);

    std::cout << " do_xy  is: " << do_xy << std::endl;

    double  Reach_xy, Reach_z;
            Reach_xy = 0.0;
            Reach_z  = 0.0;

    std::cout << " Delta do_xy  is: " << do_xy - wksp_lim[0]*wksp_lim[1] << std::endl;

    if(do_xy > wksp_lim[0]*wksp_lim[1]){
        Reach_xy = exp(-1./(var_[0]*var_[0]) * (do_xy - wksp_lim[0]*wksp_lim[1])*(do_xy - wksp_lim[0]*wksp_lim[1]));
    }
    else{
        Reach_xy = 1.;
    }

    std::cout << " Reach_xy  is: " << Reach_xy << std::endl;

    //
    if(zof > wksp_lim[2]){
        Reach_z = exp(-1./(var_[1]*var_[1]) * (zof - wksp_lim[2])*(zof - wksp_lim[2]));
    }
    else if(zof < wksp_lim[3]){
        Reach_z = exp(-1./(var_[1]*var_[1]) * (zof - wksp_lim[3])*(zof - wksp_lim[3]));
    }
    else{
        Reach_z = 1.;
    }

    std::cout << " Reach_z  is: " << Reach_z << std::endl;

    //
    Reachability_position = Reach_xy * Reach_z;

    // orientation
    double phi_xy_0 = atan2(yof, xof);
    double Reach_angle_xy, Reach_angle_xz, Reach_angle_yz;

    Eigen::Vector3d ang_euler = aF_H_ob.block<3,3>(0,0).eulerAngles(0, 1, 2);

    if(fabs(ang_euler(2)-phi_xy_0) > wksp_lim[6]){
        Reach_angle_xy = exp(-1./(var_[2]*var_[2]) * (ang_euler(2) - phi_xy_0)*(ang_euler(2) - phi_xy_0));
    }
    else{
        Reach_angle_xy = 1.;
    }

    if(fabs(ang_euler(1)) > wksp_lim[5]){
        Reach_angle_xz = exp(-1./(var_[2]*var_[2]) * (ang_euler(1))*(ang_euler(1)));
    }
    else{
        Reach_angle_xz = 1.;
    }

    if(fabs(ang_euler(1)) > wksp_lim[4]){
        Reach_angle_yz = exp(-1./var_[2] * (ang_euler(0))*(ang_euler(0)));
    }
    else{
        Reach_angle_yz = 1.;
    }
    //
    Reachability_orientation = Reach_angle_xy * Reach_angle_xz*Reach_angle_yz;

    return true;

}
//
Matrix4d WholeBodyTasksControlModule::getGlobal2LocalWorld(Vector7d pelvis_pose, Matrix4d W_H_absF, Matrix4d mW_H_pel)
{
    Matrix4d aF_H_GW_ = MatrixXd::Identity(4,4);
    Matrix4d aF_H_P  = W_H_absF.inverse() * Transforms.PoseVector2HomogenousMx(pelvis_pose);
    //
    Matrix3d aRp = aF_H_P.block(0,0,3,3);
    Vector3d ryp_ap = Transforms.getEulerAnglesXYZ_FixedFrame(aRp);
    // double yaw_ap   = ryp_ap(2);

    Matrix3d wRp = mW_H_pel.block(0,0,3,3);
    Vector3d rpy_wp = Transforms.getEulerAnglesXYZ_FixedFrame(wRp);
    // double yaw_wp   = rpy_wp(2);
    // if(robotName == "icub")
    // {
        aF_H_GW_.block<3,3>(0,0) = Transforms.rpy2rFF(Vector3d(0.0, 0.0, ryp_ap(2))) * Transforms.rpy2rFF(Vector3d(0.0, 0.0, rpy_wp(2))).transpose();
        aF_H_GW_.block<3,1>(0,3) = -aF_H_GW_.block<3,3>(0,0) * mW_H_pel.block<3,1>(0,3) + aF_H_P.block<3,1>(0,3);
    // }
    // else
    // {
    //     aF_H_GW_ = aF_H_P * mW_H_pel.inverse();
    // }

    return aF_H_GW_;
}


// bool WholeBodyTasksControlModule::SwitchControlModes(char c)
// {
//     switch(c)
//     {
//         case '1': {
//             ctrl_param->writeCommands = !ctrl_param->writeCommands;
//             ctrl_param->AllPositionArmsTorqueMode = false;
//             // Joints_pos_filtered_cmds = Joints_pos_cmds = ioSM->JtsStates.position; //m_joints_sensor_wb.position;
            
//             if (ctrl_param->writeCommands) {
//                 robot_model->robot_interface.setWholeBodyControlMode(robot_model->robot_interface.CONTROL_MODE_TORQUE);
//             } else {
//                 robot_model->robot_interface.setWholeBodyControlMode(robot_model->robot_interface.CONTROL_MODE_POSITION_DIRECT);
//             }
//             // allow safe change of control mode
//             // yarp::os::Time::delay(0.150);
//             yarp::os::Time::delay(0.010);

//         }   break;

//         case '2': {
//             ctrl_param->AllPositionArmsTorqueMode = !ctrl_param->AllPositionArmsTorqueMode;
//             ctrl_param->writeCommands = false;
//             if(ctrl_param->AllPositionArmsTorqueMode){
//                     robot_model->robot_interface.setWholeBodyPostionArmsTorqueModes("DIRECT");                      
//             } else {
//                 robot_model->robot_interface.setWholeBodyControlMode(robot_model->robot_interface.CONTROL_MODE_POSITION_DIRECT);
//             }
//             // allow safe change of control mode
//             yarp::os::Time::delay(0.01);
//         }   break;

//         case '3': {
//             ctrl_param->writeCommands = false;
//             ctrl_param->AllPositionArmsTorqueMode = false;
//             robot_model->robot_interface.setWholeBodyControlMode(robot_model->robot_interface.CONTROL_MODE_POSITION_DIRECT);
//             // allow safe change of control mode
//             yarp::os::Time::delay(0.01);
//         }   break;
//     }
//     //
//     ioSM->Joints_pos_filtered_cmds = ioSM->Joints_pos_cmds = ioSM->JtsStates.position; //m_joints_sensor_wb.position;
//     //
//     return true;
// }

//
bool WholeBodyTasksControlModule::check_pre_step_posture()
{
    return (((ioSM->wbTS.Pelvis.Pose.head(2) - ioSM->w_H_absF.block<2,1>(0,3)).norm() <= 0.02) \
             && (error_pose[5].tail(3).norm()<=0.05)); 
}

bool WholeBodyTasksControlModule::check_reach_error()
{
    Matrix4d w_H_hand_l = Transforms.PoseVector2HomogenousMx(ioSM->wbTS.lhand.Pose);  // left
    Matrix4d w_H_hand_r = Transforms.PoseVector2HomogenousMx(ioSM->wbTS.lhand.Pose);  // right
    // Offset coorection
    w_H_hand_l.block<3,1>(0,3) +=  w_H_hand_l.block<3,3>(0,0)*ctrl_param->hand_offset[0];
    w_H_hand_r.block<3,1>(0,3) +=  w_H_hand_r.block<3,3>(0,0)*ctrl_param->hand_offset[1];
    //
    Matrix4d w_H_hand_des_l = ioSM->w_H_absF * Transforms.PoseVector2HomogenousMx(ctrl_param->des_lh_Pose_af_1);
    Matrix4d w_H_hand_des_r = ioSM->w_H_absF * Transforms.PoseVector2HomogenousMx(ctrl_param->des_rh_Pose_af_1);
    double lhand_err_norm = (w_H_hand_l.block<3,1>(0,3)-w_H_hand_des_l.block<3,1>(0,3)).norm();
    double rhand_err_norm = (w_H_hand_l.block<3,1>(0,3)-w_H_hand_des_l.block<3,1>(0,3)).norm();

    std::cout << " lhand_err_norm is : \t" <<  lhand_err_norm << std::endl;
    std::cout << " rhand_err_norm is : \t" <<  rhand_err_norm << std::endl;
    //
    return (lhand_err_norm <= ctrl_param->epsil_reach && rhand_err_norm <= ctrl_param->epsil_reach);
}
