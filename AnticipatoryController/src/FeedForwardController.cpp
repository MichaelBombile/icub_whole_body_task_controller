// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



#include "FeedForwardController.h"


FeedForwardController::FeedForwardController(){}

FeedForwardController::~FeedForwardController()
{
    log_reach.close();
    log_fmmcom.close();
    log_astep.close();
    log_iGuess.close();
    log_aGstep.close();
}

// FeedForwardController::FeedForwardController(WbRobotModel& robot) : robot_model(robot) {}
// {
//     robot_model->getLinkPose(m_joints_sensor_wb.position, W_H_B, "left_hand",   pose_lhand);
//     robot_model->getLinkPose(m_joints_sensor_wb.position, W_H_B, "right_hand",  pose_rhand);
//     robot_model->getLinkPose(m_joints_sensor_wb.position, W_H_B, "left_foot",   pose_lfoot);
//     robot_model->getLinkPose(m_joints_sensor_wb.position, W_H_B, "right_foot",  pose_rfoot);
//     robot_model->getLinkPose(m_joints_sensor_wb.position, W_H_B, "CoM",         pose_CoM);
//     robot_model->getLinkPose(m_joints_sensor_wb.position, W_H_B, "Pelvis",      pose_Pelvis);
//     robot_model->getLinkPose(m_joints_sensor_wb.position, W_H_B, "Chest",       pose_Chest);   
// }

// void FeedForwardController::Initialize3(RobotInterface& robot_interface_, WbRobotModel& robot_model_, std::string n_data)  
void FeedForwardController::Initialize3(ioStateManager& ioSM_,  WbRobotModel& robot_model_, std::string n_data)
{
    //
    DStepMaxReached = false;
    stable_prediction = false;
    DeltaFootstepPose.setZero();
    //
    n_joints = robot_model_.getDoFs();

    // Initialize the anticipatory controller
    // =======================================
    a_ctrl.Initialize(robot_model_, n_data);       
    // Load the PCA parameters of the learned model
    std::cout << " \n" << std::endl;
    // ----------------------------------------------------------------------------------------------------
    stance_ft = "left";

    // robot_model_.getConfigurationStates(robot_interface_);
    m_joints_sensor_wb.resize(robot_model_.getDoFs());
    W_H_B   = Eigen::MatrixXd::Identity(4,4);
    VB      = Eigen::VectorXd::Zero(6);

    // robot_model_.EstimateRobotStates(robot_interface_, stance_ft, m_joints_sensor_wb, W_H_B, VB);  // Estimate the robot state
    //
    jts_pos = robot_model_.m_jts_sensor_wb.position;
    //
    low_Sr = 0.0;           high_Sr = 0.20;         // bounds on the step radius
    low_dl = 0.0;           high_dl = 0.5*M_PI;     // bounds on step direction on the left
    low_dr =-0.50*M_PI;     high_dr = 0.0;          // bounds on step direction on the right
    low_TF =-0.25*M_PI;     high_TF = 0.25*M_PI;    // bounds on foot_step orientation
    
    // Initialize the inverese kinemtaics module
    // ---------------------------------------------------------------------
    // WBIK.InitializeWBIK(robot_model_, jts_pos, W_H_B, 1.2, 12, 1e-4, 0.450);
    // WBIK.InitializeWBIK(robot_model_, jts_pos, W_H_B, 1.2, 12, 5e-3, 0.450); 
    WBIK.InitializeWBIK(robot_model_, ioSM_.JtsStates.position, ioSM_.world_H_fBase, 1.2, 12, 5e-3, 0.450);  // ioSM->JtsStates.position, ioSM->world_H_fBase,
    //
    Vector7d des_X_EE[8];

    ioSM_.copy_whole_body_poses2array8(des_X_EE);
    // robot_model_.getLinkPose(m_joints_sensor_wb.position, W_H_B, "left_hand",   des_X_EE[0]);
    // robot_model_.getLinkPose(m_joints_sensor_wb.position, W_H_B, "right_hand",  des_X_EE[1]);
    // robot_model_.getLinkPose(m_joints_sensor_wb.position, W_H_B, "left_foot",   des_X_EE[2]);
    // robot_model_.getLinkPose(m_joints_sensor_wb.position, W_H_B, "right_foot",  des_X_EE[3]);
    // robot_model_.getLinkPose(m_joints_sensor_wb.position, W_H_B, "CoM",         des_X_EE[4]);
    // robot_model_.getLinkPose(m_joints_sensor_wb.position, W_H_B, "Pelvis",      des_X_EE[6]);
    // robot_model_.getLinkPose(m_joints_sensor_wb.position, W_H_B, "Chest",       des_X_EE[7]);   
    // des_X_EE[5] = des_X_EE[6];
    //
    // =============================================================================================================================
    //
    double dt_ = 0.040;
    Vector7d Pose_EE[5];
    Pose_EE[0] = des_X_EE[0];
    Pose_EE[1] = des_X_EE[1];
    Pose_EE[2] = des_X_EE[2];
    Pose_EE[3] = des_X_EE[3];
    Pose_EE[4] = des_X_EE[4];
    //
    // =============================================================================================================================
    double t_wbIK = yarp::os::Time::now();
    WBIK.get_wb_IK(robot_model_, jts_pos, ioSM_.world_H_fBase, des_X_EE, stance_ft, des_X_EE[2], des_X_EE[3]);
    std::cout<< "WB INVERSE KIN RUNS IN FFWD: " << yarp::os::Time::now()-t_wbIK << " s" << std::endl;
    //
    std::cout<< "Desired COM pose   : \t" << des_X_EE[4].transpose() << std::endl;
    std::cout<< "Current COM pose   : \t" << WBIK.Pose_EE[4].transpose() << std::endl;
    std::cout<< "Desired lfoot_pose : \t" << des_X_EE[2].transpose() << std::endl;
    std::cout<< "Current lfoot_pose : \t" << WBIK.Pose_EE[2].transpose() << std::endl;
    std::cout<< "Desired rfoot_pose : \t" << des_X_EE[3].transpose() << std::endl;
    std::cout<< "Current rfoot_pose : \t" << WBIK.Pose_EE[3].transpose() << std::endl;
    std::cout<< "Desired lhand_pose : \t" << des_X_EE[0].transpose() << std::endl;
    std::cout<< "Current lhand_pose : \t" << WBIK.Pose_EE[0].transpose() << std::endl;
    std::cout<< "Desired rhand_pose : \t" << des_X_EE[1].transpose() << std::endl;
    std::cout<< "Current rhand_pose : \t" << WBIK.Pose_EE[1].transpose() << std::endl;   
    std::cout<< "WB  HANDS   ERRORS : \t" << WBIK.get_hands_errors().transpose() << std::endl;

    //
    q_star        = Eigen::VectorXd::Zero(n_joints+7);
    q_dot_star    = Eigen::VectorXd::Zero(n_joints+6);
    q_ddot_star   = Eigen::VectorXd::Zero(n_joints+6);
    q_dot_hat_    = Eigen::VectorXd::Zero(n_joints+6);
    q_ddot_hat_   = Eigen::VectorXd::Zero(n_joints+6);
    //
    xi_star_      = Eigen::VectorXd::Zero(14);
    xi_dot_star_  = Eigen::VectorXd::Zero(12); 
    xi_ddot_star_ = Eigen::VectorXd::Zero(12);
    //
    // 
    //
    string path_log_stability_var   = "../Data/anticipative/log2_stability_var_"   + n_data + ".txt";
    string path_log_PointsCHull     = "../Data/anticipative/log2_PointsCHull_"     + n_data + ".txt";
    string path_log_NormalCHull     = "../Data/anticipative/log2_NormalCHull_"     + n_data + ".txt";
    string path_log_DistanceCHull   = "../Data/anticipative/log2_DistanceCHull_"   + n_data + ".txt";
    string path_log_JointsPosition  = "../Data/anticipative/log2_JointsPosition_"  + n_data + ".txt";
    //
    OutRecord_stability_var.open(path_log_stability_var.c_str());
    OutRecord_PointsCHull.open(path_log_PointsCHull.c_str());
    OutRecord_NormalCHull.open(path_log_NormalCHull.c_str());
    OutRecord_DistanceCHull.open(path_log_DistanceCHull.c_str());   
    OutRecord_JointsPosition.open(path_log_JointsPosition.c_str());

    string global = "G_";
    string path_log_stability_var2   = "../Data/anticipative/log2_stability_var"   + global + n_data + ".txt";
    string path_log_PointsCHull2     = "../Data/anticipative/log2_PointsCHull"     + global + n_data + ".txt";
    string path_log_NormalCHull2     = "../Data/anticipative/log2_NormalCHull"     + global + n_data + ".txt";
    string path_log_DistanceCHull2   = "../Data/anticipative/log2_DistanceCHull"   + global + n_data + ".txt";
    string path_log_JointsPosition2  = "../Data/anticipative/log2_JointsPosition"  + global + n_data + ".txt";
    //
    OutRecord_stability_var2.open(path_log_stability_var2.c_str());
    OutRecord_PointsCHull2.open(path_log_PointsCHull2.c_str());
    OutRecord_NormalCHull2.open(path_log_NormalCHull2.c_str());
    OutRecord_DistanceCHull2.open(path_log_DistanceCHull2.c_str()); 
    OutRecord_JointsPosition2.open(path_log_JointsPosition2.c_str());
    //=====================================================================================================
    // Initialization of class members
    Disturb_c_    = VectorXd::Zero(2);
    CP_star_      = VectorXd::Zero(2);
    fu_z_mgbetaW_ = 0.0;
    // ====================================================================================================
    // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // =========================================================================================
    StepMagnitude = 0.01;
    RelativeStep.resize(8);
    RelativeStep.setZero();
    DeltaStepMax = a_ctrl.lengthStep_max;

    Reach_Done    = false;
    fmmCoM_Done   = false;
    AntiStep_Done = false;
    Stable_Done   = false;
    isStable      = false;
    //
    l_count     = 0;
    g_count     = 0;
    l_count_max = 1;
    isReachable = false;
    cnt         = false;
    ik_run      = false;
    n_loop      = false;
    reach_batch_start  = false;
    fmmCoM_batch_start = false;
    aStep_batch_start  = false; 
    stance_foot_step   = "right";

    InitGuess_batch_start   = false;
    aStepGlobal_batch_start = false;
    InitGuess_Done          = false;
    aStepGlobal_Done        = false;
    aStepGlobal_All_batch_start = false;

    fmmCoM = des_X_EE[4].head(3);
    //
    // ========================================================================================================================================================================
    std::string astep_path = "../DataSolver/";
    log_reach.open( astep_path, "reach" , n_data);
    log_fmmcom.open(astep_path, "fmmcom", n_data);
    log_astep.open( astep_path, "astep" , n_data);
    log_iGuess.open(astep_path, "iGuess", n_data);
    log_aGstep.open(astep_path, "aGstep", n_data);

    // =================================================== COM Reference generator ====================================================================

    // data logger
    // std::ofstream Out_states;
    // std::string data_number = "00";
    // string path_log_states  = "../Data/anticipative/log_stepping_states_"  + n_data + ".txt";
    // Out_states.open(path_log_states.c_str());

    double sTime_            =  0.20;
    double z_CoM_            =  0.47;
    double mMass_            =  30.0;
    //
    double gains_[10];
    gains_[0] = 1e-7;   // weight on c_dddot tracking
    gains_[1] = 0.05;   // veight on c_dot  tracking
    gains_[2] = 2.50;   // weight on CoP tracking
    gains_[3] = 10.50;  // weight on Disturbance tracking
    gains_[4] = 30.0;   // weight on Delta Disturbance

    gains_[5] = 1.0;
    gains_[6] = 1.00;
    gains_[7] = 0.10;
    gains_[8] = 0.00;  // for planning
    gains_[9] = 0.00;  // for planning

    // =================================================================================
    VectorXd  EdgePositions_(4);    EdgePositions_.setZero();
    EdgePositions_(0) = 0.070;      EdgePositions_(1) = 0.03;   //0.02;
    EdgePositions_(2) = 0.070;      EdgePositions_(3) = 0.03;   //0.02;

    MatrixXd  EdgeNormals_(2,4);    EdgeNormals_.setZero();
    EdgeNormals_(0,0) = 1.;         EdgeNormals_(1,1) = 1.;     // [1  0  -1  0;
    EdgeNormals_(0,2) =-1.;         EdgeNormals_(1,3) =-1.;     //  0  1   0 -1];

    // =================================================================================
    ComRefGen.Initialize(sTime_, z_CoM_, mMass_, gains_, EdgePositions_, EdgeNormals_);
    //
    // ========================================================================================================================================================================
    // this->check_reachability_batch(robot_model_, jts_pos, des_X_EE, stance_ft, des_X_EE[2], des_X_EE[3]);
    //
    std::cout<< "FFWD CONTROLLER INITIALIZED   : \t" << 0.0 << std::endl;
}

bool FeedForwardController::approximate_stepping_pose(  RobotInterface& robot_interface_,  WbRobotModel& robot_model_, VectorXd Wrench_hands_star_, Vector6d des_TspMotion[], 
                                                        Vector7d des_X_EE_[], Vector7d Pose_lfoot, Vector7d Pose_rfoot)
{
    int count = 0;
    bool stable_0 = false;
    bool stable_1 = false;
    //
    MatrixXd AllPtsInAbsFoot;
    MatrixXd PointsCHull;
    VectorXd DistanceEdgesCHull;
    MatrixXd Abs_EdgesNormalCHull;
    MatrixXd W_EdgesNormalCHull;
    Matrix2d W_Rot_AbsFoot;
    Vector2d W_Pos_AbsFoot;
    Vector2d W_Disturb;
    // 
    robot_model_.EstimateRobotStates(robot_interface_, stance_ft, m_joints_sensor_wb, W_H_B, VB);  // Estimate the robot state
    jts_pos = robot_model_.m_jts_sensor_wb.position;

    a_ctrl.update_model(robot_model_, jts_pos, W_H_B);                                          // update the robot model
    a_ctrl.get_centroidal_variables(robot_model_, jts_pos, W_H_B, q_dot_star, q_ddot_star);     // compute the predicted centroidal variables
    a_ctrl.compute_posture_perturbation(Wrench_hands_star_);                                    // Compute the predicted balance perturbation
    a_ctrl.compute_capture_point(Wrench_hands_star_);                                           // compute the Predicted the capture point
    // capture point wrt world frame
    Vector2d W_CP_(a_ctrl.CP(0),  a_ctrl.CP(1));
    double DeltaStepLength = 0;
    
    // direction of the balance disturbance
    double theta_Dist = 0.0;
    double DeltaThetaFeet_ = 0.0;
    double dTheta_st, dTheta_sw, dDThetaFeet;
    // double  theta_Dist = atan2( a_ctrl.Delat_R_cmp(1),  a_ctrl.Delat_R_cmp(0));
    Vector2d DeltaStep(DeltaStepLength * cos(theta_Dist), DeltaStepLength * sin(theta_Dist));   // step increase
    Vector2d DeltaDisturb(0.0, 0.0);
    // ===========================================================================================================
    for(int i=0; i<8; i++) des_X_EE[i] = des_X_EE_[i];

    // =======================================================================================================
        // extraction of Euler angles of  the feet rotation 
        Vector3d o_lf = a_ctrl.Transforms.getEulerAnglesXYZ_FixedFrame(des_X_EE_[2]);
        Vector3d o_rf = a_ctrl.Transforms.getEulerAnglesXYZ_FixedFrame(des_X_EE_[3]);

    // // while((stable_0 == true)||(lengthStep <= lengthStep_max))
    while((!stable_0) && (a_ctrl.lengthStep <= a_ctrl.lengthStep_max))
    {
        // update step
        DeltaStepLength = double(count) *  a_ctrl.stepIncrease;
        DeltaStep << DeltaStepLength * cos(theta_Dist), DeltaStepLength * sin(theta_Dist);
        // update the balance perturbation
        // --------------------------------
        a_ctrl.compute_delta_disturb(Wrench_hands_star_, 0.5*DeltaStep, DeltaDisturb);  // compute step induced variation of perturbation
        // updating the Perturbation and capture point
        W_Disturb = a_ctrl.Delat_R_cmp + DeltaDisturb;                                  // balance perturbation
        W_CP_     = a_ctrl.CP.head(2)  + 0.5 * DeltaStep;                               // update capture point in absolute frame
        
        // get the points defining the support polygon of each feet
        // a_ctrl.get_feet_support_points(Pose_lfoot, Pose_rfoot, DeltaStep, AllPtsInAbsFoot, W_Rot_AbsFoot, W_Pos_AbsFoot);  // -->> AllPtsInAbsFoot, W_Rot_AbsFoot, W_Pos_AbsFoot
        a_ctrl.get_feet_support_points_C(Pose_lfoot, Pose_rfoot, DeltaStep, DeltaThetaFeet_, AllPtsInAbsFoot, W_Rot_AbsFoot, W_Pos_AbsFoot); 
        // update convex hull
        a_ctrl.ConvHull.convexHull(AllPtsInAbsFoot, PointsCHull, Abs_EdgesNormalCHull, DistanceEdgesCHull); // --> PointsCHull, EdgesNormalCHull, DistanceEdgesCHull
        // Express the normal in the world frame
        W_EdgesNormalCHull = Abs_EdgesNormalCHull * W_Rot_AbsFoot.transpose();
        // predict the stability
        stable_0 = a_ctrl.predict_stability(W_EdgesNormalCHull, DistanceEdgesCHull, W_CP_, W_Pos_AbsFoot, W_Disturb);
        //
        if(a_ctrl.stance_0 =="left")    // left stance foot for anticipatory step
        {
            dTheta_st   = o_lf(2);
        }
        else  // right stance foot for anticipatory step
        {
            dTheta_st   = o_rf(2);
        }

        theta_Dist = atan2( a_ctrl.Delat_R_cmp(1),  a_ctrl.Delat_R_cmp(0));
        DeltaThetaFeet_ = theta_Dist - dTheta_st;
        if     (DeltaThetaFeet_ <= -a_ctrl.Dtheta_feet_max) DeltaThetaFeet_ = -a_ctrl.Dtheta_feet_max;
        else if(DeltaThetaFeet_ >=  a_ctrl.Dtheta_feet_max) DeltaThetaFeet_ =  a_ctrl.Dtheta_feet_max;
        //
        std::cout << " count : \t" << count << "    stable_0 : \t" << stable_0 << std::endl;
        // *****************************************************************************************
        // Record the data Poses and joints configuration
        // *****************************************************************************************           
        //  
        int stance_left = 0; 
        if(a_ctrl.stance_0 =="left") stance_left = 1;
        OutRecord_stability_var << count << "  " << W_Disturb.transpose() << "  " << W_CP_.transpose() << "  " << W_Pos_AbsFoot.transpose() <<  "  ";
        OutRecord_stability_var << W_Rot_AbsFoot.row(0) << "  " << W_Rot_AbsFoot.row(1) <<  "  ";
        OutRecord_stability_var << DeltaStep.transpose() << "  " << a_ctrl.Delta_theta_feet << "  " << stable_0 << "  " << stance_left << "  " << 0.0 << endl;

        OutRecord_JointsPosition << jts_pos.transpose() << endl;
        
        if(PointsCHull.rows() == 5)
        {
            OutRecord_PointsCHull   << PointsCHull.col(0).transpose() << "  " << PointsCHull(0,0) << "  " << PointsCHull(0,0) << "  ";   // x
            OutRecord_PointsCHull   << PointsCHull.col(1).transpose() << "  " << PointsCHull(0,1) << "  " << PointsCHull(0,1) << endl;   // y

            OutRecord_NormalCHull   << W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull(0,0) << "  " << W_EdgesNormalCHull(0,0) << "  ";   // x
            OutRecord_NormalCHull   << W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull(0,1) << "  " << W_EdgesNormalCHull(0,1) << endl;   // y

            OutRecord_DistanceCHull << DistanceEdgesCHull.transpose() << "  " << DistanceEdgesCHull(0) << "  " << DistanceEdgesCHull(0) << endl;    
        }
        else if(PointsCHull.rows() == 6)
        {
            OutRecord_PointsCHull   << PointsCHull.col(0).transpose() << "  " << PointsCHull(0,0) << "  " ;   // x
            OutRecord_PointsCHull   << PointsCHull.col(1).transpose() << "  " << PointsCHull(0,1) << endl;   // y

            OutRecord_NormalCHull   << W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull(0,0) << "  ";   // x
            OutRecord_NormalCHull   << W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull(0,1) << endl;      // y

            OutRecord_DistanceCHull << DistanceEdgesCHull.transpose() << "  " << DistanceEdgesCHull(0) << endl;
        }
        else
        {
            OutRecord_PointsCHull   << PointsCHull.col(0).transpose() << "  " << PointsCHull.col(1).transpose() << endl;
            OutRecord_NormalCHull   << W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull.col(1).transpose() << endl;
            OutRecord_DistanceCHull << DistanceEdgesCHull.transpose() << endl;  
        }
        //
        count++;
    } 
    //
    // stance indicator
    Vector2d stance_ind;
        if(a_ctrl.stance_0 =="left")
            stance_ind << 0.0, 1.0;
        else // stance is right
            stance_ind << 1.0, 0.0;
    //
    if((stable_0 == true) && (a_ctrl.lengthStep <= a_ctrl.lengthStep_max))
    {
            a_ctrl.step_values << DeltaStep(0), DeltaStep(1), a_ctrl.Delta_theta_feet, stance_ind(0), stance_ind(1), 0.0;
    }
    else //if((stable_0 != true) && (lengthStep >= lengthStep_max))   // TO BE CHECKED FOR MATHEMATICAL CONSISTENCY ANN STABILITY
    {
        // update the balance perturbation
        // --------------------------------
        // compute the variation of the perturbation due to the stepping
        a_ctrl.compute_delta_disturb(Wrench_hands_star_, DeltaStep, DeltaDisturb);
        // updating the Perturbation
        W_Disturb = a_ctrl.Delat_R_cmp + DeltaDisturb;
        // update capture point and absolute frame
        W_CP_     = a_ctrl.CP.head(2)  + DeltaStep;
        
        // get the points defining the support polygon of each feet
        a_ctrl.getJointFeetSsupportPoints_2(a_ctrl.stance_1, a_ctrl.theta_swing, a_ctrl.DyFeet_min, Pose_lfoot, Pose_rfoot, DeltaStep, 
                                                                        AllPtsInAbsFoot, W_Rot_AbsFoot, W_Pos_AbsFoot);
        // update convex hull
        a_ctrl.ConvHull.convexHull(AllPtsInAbsFoot, PointsCHull, Abs_EdgesNormalCHull, DistanceEdgesCHull); // --> PointsCHull, EdgesNormalCHull, DistanceEdgesCHull
        // Express the normal in the world frame
        W_EdgesNormalCHull = Abs_EdgesNormalCHull * W_Rot_AbsFoot.transpose();
        // predict the stability
        stable_1 = a_ctrl.predict_stability(W_EdgesNormalCHull, DistanceEdgesCHull, W_CP_, W_Pos_AbsFoot, W_Disturb);
        //
        a_ctrl.step_values << DeltaStep(0), DeltaStep(1), a_ctrl.Delta_theta_feet, stance_ind(0), stance_ind(1), 1.0;
        //
        cout << "DeltaDisturb  \t" <<  DeltaDisturb.transpose() << endl;
        cout << "W_Disturb     \t" <<  W_Disturb.transpose() << endl; 
        cout << "stable_1      \t" <<  stable_1 << endl; 
        // *****************************************************************************************
        // Record the data Poses and joints configuration
        // *****************************************************************************************
        int stance_left = 0; 
        if(a_ctrl.stance_0 =="left") stance_left = 1;
        if(PointsCHull.rows() == 5)
        {
            OutRecord_PointsCHull   << PointsCHull.col(0).transpose() << "  " << PointsCHull(0,0) << "  " << PointsCHull(0,0) << "  ";   // x
            OutRecord_PointsCHull   << PointsCHull.col(1).transpose() << "  " << PointsCHull(0,1) << "  " << PointsCHull(0,1) << endl;   // y

            OutRecord_NormalCHull   << W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull(0,0) << "  " << W_EdgesNormalCHull(0,0) << "  ";   // x
            OutRecord_NormalCHull   << W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull(0,1) << "  " << W_EdgesNormalCHull(0,1) << endl;   // y

            OutRecord_DistanceCHull << DistanceEdgesCHull.transpose() << "  " << DistanceEdgesCHull(0) << "  " << DistanceEdgesCHull(0) << endl;    
        }
        else if(PointsCHull.rows() == 6)
        {
            OutRecord_PointsCHull   << PointsCHull.col(0).transpose() << "  " << PointsCHull(0,0) << "  " ;   // x
            OutRecord_PointsCHull   << PointsCHull.col(1).transpose() << "  " << PointsCHull(0,1) << endl;   // y

            OutRecord_NormalCHull   << W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull(0,0) << "  ";   // x
            OutRecord_NormalCHull   << W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull(0,1) << endl;      // y

            OutRecord_DistanceCHull << DistanceEdgesCHull.transpose() << "  " << DistanceEdgesCHull(0) << endl;
        }
        else
        {
            OutRecord_PointsCHull   << PointsCHull.col(0).transpose() << "  " << PointsCHull.col(1).transpose() << endl;
            OutRecord_NormalCHull   << W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull.col(1).transpose() << endl;
            OutRecord_DistanceCHull << DistanceEdgesCHull.transpose() << endl;  
        }
        //
    }

    //
    OutRecord_stability_var.close();
    OutRecord_PointsCHull.close();
    OutRecord_NormalCHull.close();
    OutRecord_DistanceCHull.close();
    OutRecord_JointsPosition.close();
    
    std::cout<< "FFWD compute_stepping_pose OK   : \t" << 0.0 << std::endl;
    
    return true;
}

//
bool FeedForwardController::approximate_stepping_pose2( RobotInterface& robot_interface_,  WbRobotModel& robot_model_, VectorXd Wrench_hands_star_, Vector6d des_TspMotion[], 
                                                        Vector7d des_X_EE_[], Vector7d Pose_lfoot, Vector7d Pose_rfoot)
{
    int count = 0;
    bool stable_0 = false;
    bool stable_1 = false;
    //
    MatrixXd AllPtsInAbsFoot;
    MatrixXd PointsCHull;
    VectorXd DistanceEdgesCHull;
    MatrixXd Abs_EdgesNormalCHull;
    MatrixXd W_EdgesNormalCHull;
    Matrix2d W_Rot_AbsFoot;
    Vector2d W_Pos_AbsFoot;
    Vector2d W_Disturb;
    // 
    robot_model_.EstimateRobotStates(robot_interface_, stance_ft, m_joints_sensor_wb, W_H_B, VB);  // Estimate the robot state
    jts_pos = robot_model_.m_jts_sensor_wb.position;

    // using the predicted posture 
    WBIK.get_wb_IK(robot_model_, jts_pos, W_H_B, des_X_EE_, stance_ft, Pose_lfoot, Pose_rfoot);
    jts_pos  = WBIK.des_jts_pos;   
    W_H_B    = WBIK.des_WHB;

    a_ctrl.update_model(robot_model_, jts_pos, W_H_B);                                          // update the robot model
    a_ctrl.get_centroidal_variables(robot_model_, jts_pos, W_H_B, q_dot_star, q_ddot_star);     // compute the predicted centroidal variables
    a_ctrl.compute_posture_perturbation(Wrench_hands_star_);                                    // Compute the predicted balance perturbation
    a_ctrl.compute_capture_point(Wrench_hands_star_);                                           // compute the Predicted the capture point
    // capture point wrt world frame
    Vector2d W_CP_(a_ctrl.CP(0),  a_ctrl.CP(1));
    double DeltaStepLength = 0;
    
    // direction of the balance disturbance
    double theta_Dist = 0.0;
    double DeltaThetaFeet_ = 0.0;
    double dTheta_st, dTheta_sw, dDThetaFeet;
    // double  theta_Dist = atan2( a_ctrl.Delat_R_cmp(1),  a_ctrl.Delat_R_cmp(0));
    Vector2d DeltaStep(DeltaStepLength * cos(theta_Dist), DeltaStepLength * sin(theta_Dist));   // step increase
    Vector2d DeltaDisturb(0.0, 0.0);
    // ===========================================================================================================
    for(int i=0; i<8; i++) des_X_EE[i] = des_X_EE_[i];

    // =======================================================================================================
    // extraction of Euler angles of  the feet rotation 
    Vector3d o_lf = a_ctrl.Transforms.getEulerAnglesXYZ_FixedFrame(des_X_EE_[2]);
    Vector3d o_rf = a_ctrl.Transforms.getEulerAnglesXYZ_FixedFrame(des_X_EE_[3]);

    // // while((stable_0 == true)||(lengthStep <= lengthStep_max))
    // while((!stable_0) && (a_ctrl.lengthStep <= a_ctrl.lengthStep_max))
    // Step variables range
    VectorXd DeltaStep_r      = VectorXd::LinSpaced(20,low_Sr,high_Sr);
    VectorXd DeltaStep_dir_l  = VectorXd::LinSpaced(30,low_dl,high_dl);
    VectorXd DeltaStep_dir_r  = VectorXd::LinSpaced(30,low_dr,high_dr);
    VectorXd DeltaStep_thetaF = VectorXd::LinSpaced(20,low_TF,high_TF);

// for(int i=0; i<DeltaStep_r.rows(); i++){
// for(int j=0; j<DeltaStep_dir_l.rows(); j++){
    int iter_xy = 0;
    int iter_o  = 0;
    bool DStepFound = false;
    bool SolverDone = false;
    double Jdxdydo_0,  Jdxdydo;         Jdxdydo_0 = Jdxdydo = 0.0;
    double DevTheta_0, DevTheta;        DevTheta_0= DevTheta= 0.0;
    double desDeltaThetaFeet_ = 0.0;
    int sol_index = 1;
    // for(int k=0; k<DeltaStep_thetaF.rows(); k++)
    // while((!stable_0) && (a_ctrl.lengthStep <= a_ctrl.lengthStep_max))
    // while((!SolverDone) && (iter_o < DeltaStep_thetaF.rows()) || (count <= (DeltaStep_r.rows() + DeltaStep_thetaF.rows()) ))
    double t_solution = yarp::os::Time::now();

    while((!SolverDone) && (count <= (DeltaStep_r.rows() + DeltaStep_thetaF.rows()) ))
    {
        // update step
        // DeltaStepLength = double(count) *  a_ctrl.stepIncrease;
        // DeltaStep << DeltaStepLength * cos(theta_Dist), DeltaStepLength * sin(theta_Dist);
        a_ctrl.compute_delta_disturb(Wrench_hands_star_, 0.5*DeltaStep, DeltaDisturb);  // compute step induced variation of perturbation
        // updating the Perturbation and capture point
        W_Disturb = a_ctrl.Delat_R_cmp + DeltaDisturb;                                  // balance perturbation
        W_CP_     = a_ctrl.CP.head(2)  + 0.5 * DeltaStep;                               // update capture point in absolute frame

        a_ctrl.get_feet_support_points_C(Pose_lfoot, Pose_rfoot, DeltaStep, DeltaThetaFeet_, 
                                            AllPtsInAbsFoot, W_Rot_AbsFoot, W_Pos_AbsFoot); 
        // update convex hull
        a_ctrl.ConvHull.convexHull(AllPtsInAbsFoot, PointsCHull, Abs_EdgesNormalCHull, DistanceEdgesCHull); 
        // Express the normal in the world frame
        W_EdgesNormalCHull = Abs_EdgesNormalCHull * W_Rot_AbsFoot.transpose();
        // predict the stability
        stable_0 = a_ctrl.predict_stability(W_EdgesNormalCHull, DistanceEdgesCHull, W_CP_, W_Pos_AbsFoot, W_Disturb);
        //
        if(stable_0 && !DStepFound)
        {
            Jdxdydo_0 = 1./a_ctrl.lengthStep_max  * (pow(DeltaStep(0),2.) + pow(DeltaStep(1),2.)) 
                      + 1./a_ctrl.Dtheta_feet_max * (pow(DeltaThetaFeet_, 2.));
            sol_index  = count + 1;
        }
        //
        // update the balance perturbation
        // --------------------------------
        if(stable_0) DStepFound = true;

        if(!stable_0 && !DStepFound)
        {
            //
            if  (a_ctrl.stance_0 =="left")  dTheta_st = o_lf(2);    // left stance foot for anticipatory step
            else                            dTheta_st = o_rf(2);    // right stance foot for anticipatory step
            
            DeltaStep <<    DeltaStep_r(iter_xy) * cos(dTheta_st + DeltaThetaFeet_), 
                            DeltaStep_r(iter_xy) * sin(dTheta_st + DeltaThetaFeet_);
            //
            theta_Dist = atan2( a_ctrl.Delat_R_cmp(1),  a_ctrl.Delat_R_cmp(0));
            DeltaThetaFeet_ = theta_Dist - dTheta_st;
            if     (DeltaThetaFeet_ <= -a_ctrl.Dtheta_feet_max) DeltaThetaFeet_ = -a_ctrl.Dtheta_feet_max;
            else if(DeltaThetaFeet_ >=  a_ctrl.Dtheta_feet_max) DeltaThetaFeet_ =  a_ctrl.Dtheta_feet_max;
            //
            Jdxdydo_0 = 1./a_ctrl.lengthStep_max  * (pow(DeltaStep(0),2.) + pow(DeltaStep(1),2.)) 
                      + 1./a_ctrl.Dtheta_feet_max * (pow(DeltaThetaFeet_, 2.));
            //
            iter_xy ++;
            if(iter_xy >= DeltaStep_r.rows()) iter_xy = DeltaStep_r.rows()-1;
            //

        }
        else if(DStepFound)
        {
            //
            std::cout<< "DeltaStep    : \t" << DeltaStep.transpose() << std::endl;
            //
            if (stable_0)
            {
                //
                // sol_index  = count + 1;
                // computation of the cost function
                Jdxdydo = 1./a_ctrl.lengthStep_max  * (pow(DeltaStep(0),2.) + pow(DeltaStep(1),2.)) 
                        + 1./a_ctrl.Dtheta_feet_max * (pow(DeltaThetaFeet_, 2.));
                // computation deviation wrt the disturbance
                DevTheta = fabs(theta_Dist - (dTheta_st+DeltaThetaFeet_));

                std::cout<< "Jdxdydo   : \t" << Jdxdydo << std::endl;

                // selection of DeltaThetaFeet corresponding to min Jxyo
                if(Jdxdydo < Jdxdydo_0)
                {
                    desDeltaThetaFeet_ = DeltaThetaFeet_;
                    Jdxdydo_0  = Jdxdydo;
                    DevTheta_0 = DevTheta;
                    sol_index  = count + 1;
                    std::cout<< "Jdxdydo < Jdxdydo_0   : \t" << count << std::endl;

                } else if(Jdxdydo == Jdxdydo_0)
                {
                    // choose DThetaFeet with mimimum deviation wrt the disturbance
                    if(DevTheta < DevTheta_0)
                    {
                        desDeltaThetaFeet_ = DeltaThetaFeet_;
                        Jdxdydo_0  = Jdxdydo;
                        DevTheta_0 = DevTheta;
                        sol_index  = count + 1;
                        std::cout<< "DevTheta < DevTheta_0   : \t" << count << std::endl;
                    }
                }
            }
            //
            DeltaStep       = DeltaStep;
            // update theta
            DeltaThetaFeet_ = DeltaStep_thetaF(iter_o);
            // 
            iter_o ++; 
            if(iter_o >= DeltaStep_thetaF.rows()) SolverDone = true;
        }
        else {

            std::cout << " No suitable step size found " << std::endl;
            return true;
        }
        
        //
        std::cout << " count : \t" << count << "    stable_0 : \t" << stable_0 << std::endl;
        // *****************************************************************************************
        // Record the data Poses and joints configuration
        // *****************************************************************************************           
        int stance_left = 0; 
        if(a_ctrl.stance_0 =="left") stance_left = 1;

        // if(stable_0)
        // { 
            OutRecord_stability_var << count << "  " << W_Disturb.transpose() << "  " << W_CP_.transpose() << "  " << W_Pos_AbsFoot.transpose() <<  "  ";
            OutRecord_stability_var << W_Rot_AbsFoot.row(0) << "  " << W_Rot_AbsFoot.row(1) <<  "  ";
            // OutRecord_stability_var << DeltaStep.transpose() << "  " << a_ctrl.Delta_theta_feet << "  " << stable_0 << "  " << stance_left << "  " << 0.0 << endl; // sol_index
            OutRecord_stability_var << DeltaStep.transpose() << "  " << a_ctrl.Delta_theta_feet << "  " << stable_0 << "  " << stance_left << "  " << sol_index << endl; // sol_index

            OutRecord_JointsPosition << jts_pos.transpose() << endl;
            
            if(PointsCHull.rows() == 5)
            {
                OutRecord_PointsCHull   << PointsCHull.col(0).transpose() << "  " << PointsCHull(0,0) << "  " << PointsCHull(0,0) << "  ";   // x
                OutRecord_PointsCHull   << PointsCHull.col(1).transpose() << "  " << PointsCHull(0,1) << "  " << PointsCHull(0,1) << endl;   // y

                OutRecord_NormalCHull   << W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull(0,0) << "  " << W_EdgesNormalCHull(0,0) << "  ";   // x
                OutRecord_NormalCHull   << W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull(0,1) << "  " << W_EdgesNormalCHull(0,1) << endl;   // y

                OutRecord_DistanceCHull << DistanceEdgesCHull.transpose() << "  " << DistanceEdgesCHull(0) << "  " << DistanceEdgesCHull(0) << endl;    
            }
            else if(PointsCHull.rows() == 6)
            {
                OutRecord_PointsCHull   << PointsCHull.col(0).transpose() << "  " << PointsCHull(0,0) << "  " ;   // x
                OutRecord_PointsCHull   << PointsCHull.col(1).transpose() << "  " << PointsCHull(0,1) << endl;   // y

                OutRecord_NormalCHull   << W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull(0,0) << "  ";   // x
                OutRecord_NormalCHull   << W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull(0,1) << endl;      // y

                OutRecord_DistanceCHull << DistanceEdgesCHull.transpose() << "  " << DistanceEdgesCHull(0) << endl;
            }
            else
            {
                OutRecord_PointsCHull   << PointsCHull.col(0).transpose() << "  " << PointsCHull.col(1).transpose() << endl;
                OutRecord_NormalCHull   << W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull.col(1).transpose() << endl;
                OutRecord_DistanceCHull << DistanceEdgesCHull.transpose() << endl;  
            }
        // }
        //
        count++;
    }

    std::cout<< "MAIN COMPUTATION AT : " << count << " GOT IN : " <<  yarp::os::Time::now()-t_solution << " s" << std::endl;
// }
// } 
    //
    OutRecord_stability_var.close();
    OutRecord_PointsCHull.close();
    OutRecord_NormalCHull.close();
    OutRecord_DistanceCHull.close();
    OutRecord_JointsPosition.close();

    std::cout<< "RECORDING TIME AT : " << count << " GOT IN : " <<  yarp::os::Time::now()-t_solution << " s" << std::endl;
    
    std::cout<< "FFWD compute_stepping_pose OK   : \t" << 0.0 << std::endl;
    
    return true;
}

//
bool FeedForwardController::compute_stepping_pose(  RobotInterface& robot_interface_,  WbRobotModel& robot_model_, VectorXd Wrench_hands_star_, Vector6d des_TspMotion[], 
                                                    Vector7d des_X_EE_[], Vector7d Pose_lfoot, Vector7d Pose_rfoot)
{
    int count = 0;
    bool stable_0 = false;
    bool stable_1 = false;
    //
    MatrixXd AllPtsInAbsFoot;
    MatrixXd PointsCHull;
    VectorXd DistanceEdgesCHull;
    MatrixXd Abs_EdgesNormalCHull;
    MatrixXd W_EdgesNormalCHull;
    Matrix2d W_Rot_AbsFoot;
    Vector2d W_Pos_AbsFoot;
    Vector2d W_Disturb;
    // 
    robot_model_.EstimateRobotStates(robot_interface_, stance_ft, m_joints_sensor_wb, W_H_B, VB);  // Estimate the robot state
    jts_pos = robot_model_.m_jts_sensor_wb.position;

    a_ctrl.update_model(robot_model_, jts_pos, W_H_B);                                          // update the robot model
    a_ctrl.get_centroidal_variables(robot_model_, jts_pos, W_H_B, q_dot_star, q_ddot_star);     // compute the predicted centroidal variables
    a_ctrl.compute_posture_perturbation(Wrench_hands_star_);                                    // Compute the predicted balance perturbation
    a_ctrl.compute_capture_point(Wrench_hands_star_);                                           // compute the Predicted the capture point
    // capture point wrt world frame
    Vector2d W_CP_(a_ctrl.CP(0),  a_ctrl.CP(1));
    double DeltaStepLength = 0;
    // direction of the balance disturbance
    double  theta_Dist = atan2( a_ctrl.Delat_R_cmp(1),  a_ctrl.Delat_R_cmp(0));
    Vector2d DeltaStep(DeltaStepLength * cos(theta_Dist), DeltaStepLength * sin(theta_Dist));   // step increase
    Vector2d DeltaDisturb(0.0, 0.0);
    double StepLenght;
    double DeltaThetaFeet_ = 0.0;
    // ===========================================================================================================
    for(int i=0; i<8; i++) des_X_EE[i] = des_X_EE_[i];

    // =======================================================================================================
        double dTheta_st, dTheta_sw, dDThetaFeet;
        // extraction of Euler angles of  the feet rotation 
        Vector3d o_lf = a_ctrl.Transforms.getEulerAnglesXYZ_FixedFrame(des_X_EE_[2]);
        Vector3d o_rf = a_ctrl.Transforms.getEulerAnglesXYZ_FixedFrame(des_X_EE_[3]);

    // Step variables range
    // VectorXd DeltaStep_r      = VectorXd::LinSpaced(20,low_Sr,high_Sr);
    // VectorXd DeltaStep_dir_l  = VectorXd::LinSpaced(30,low_dl,high_dl);
    // VectorXd DeltaStep_dir_r  = VectorXd::LinSpaced(30,low_dr,high_dr);
    // VectorXd DeltaStep_thetaF = VectorXd::LinSpaced(30,low_TF,high_TF);


    // for(int i=0; i<DeltaStep_r.rows(); i++)
    // {
    //  for(int j=0; j<DeltaStep_dir_l.rows(); j++)
    //  {
    //      for(int k=0; k<DeltaStep_thetaF.rows(); k++)
    //      {

    //      }
    //  }
    // }


    // // while((stable_0 == true)||(lengthStep <= lengthStep_max))
    while((!stable_0) && (a_ctrl.lengthStep <= a_ctrl.lengthStep_max))
    {
        // update step
        DeltaStepLength = double(count) *  a_ctrl.stepIncrease;
        DeltaStep << DeltaStepLength * cos(theta_Dist), DeltaStepLength * sin(theta_Dist);
        // update the balance perturbation
        // --------------------------------
        // ***********************************************************************
        // Prediction of the balance perturbation
        // ***********************************************************************
        if(a_ctrl.stance_0 =="left")    // left stance foot for anticipatory step
        {
            //
            // DeltaStep << DeltaStep_r * cos(DeltaStep_dir_l), DeltaStep_r * sin(DeltaStep_dir_l);
            // stance foot
            des_X_EE[2] = des_X_EE_[2];                                 // left foot pose
            // swing foot
            des_X_EE[3] = des_X_EE_[3];                                 // right foot pose
            des_X_EE[3].head(2) = des_X_EE_[3].head(2) + DeltaStep;     // new right foot ground position
            //
            dTheta_st   = o_lf(2);
            dDThetaFeet = DeltaThetaFeet_; //theta_Dist - dTheta_st;

            if     (dDThetaFeet <= -a_ctrl.Dtheta_feet_max) dDThetaFeet = -a_ctrl.Dtheta_feet_max;
            else if(dDThetaFeet >=  a_ctrl.Dtheta_feet_max) dDThetaFeet =  a_ctrl.Dtheta_feet_max;

            dTheta_sw   = dTheta_st  + dDThetaFeet;
            Eigen::AngleAxisd W_ori_nSw(a_ctrl.Transforms.rot_vector2rpy(Eigen::Vector3d(o_rf(0), o_rf(1), dTheta_sw)));
            des_X_EE[3].segment<3>(3) = W_ori_nSw.axis();
            des_X_EE[3].tail<1>()    << W_ori_nSw.angle();
            //
            // step length due to theta 
            StepLenght = (des_X_EE[3].head(2) - des_X_EE[2].head(2)).norm();
        }
        else  // right stance foot for anticipatory step
        {
            // stance foot
            des_X_EE[3] = des_X_EE_[3];                                 // right foot pose
            // swing foot
            des_X_EE[2] = des_X_EE_[2];                                 // left foot pose
            des_X_EE[2].head(2) = des_X_EE_[2].head(2) + DeltaStep;     // new left foot ground position
            //
            dTheta_st   = o_rf(2);
            dDThetaFeet = DeltaThetaFeet_; //theta_Dist - dTheta_st;

            if     (dDThetaFeet <= -a_ctrl.Dtheta_feet_max) dDThetaFeet = -a_ctrl.Dtheta_feet_max;
            else if(dDThetaFeet >=  a_ctrl.Dtheta_feet_max) dDThetaFeet =  a_ctrl.Dtheta_feet_max;

            dTheta_sw   = dTheta_st  + dDThetaFeet;
            Eigen::AngleAxisd W_ori_nSw(a_ctrl.Transforms.rot_vector2rpy(Eigen::Vector3d(o_lf(0), o_lf(1), dTheta_sw)));
            des_X_EE[2].segment<3>(3) = W_ori_nSw.axis();
            des_X_EE[2].tail<1>()    << W_ori_nSw.angle();
            //
            // step length due to theta 
            StepLenght = (des_X_EE[2].head(2) - des_X_EE[3].head(2)).norm();
        }

            des_X_EE[4].head(2) = 0.5*(des_X_EE[2].head(2) + des_X_EE[3].head(2));      
        // if(StepLenght <= a_ctrl.lengthStep_max)
        // {

        // }
        //
        this->update_task_variables(DeltaStep, des_TspMotion, des_X_EE_);
        // Expected postural tasks (pose, velocity and acceleration and associated covariance matrices)    
        a_ctrl.get_expected_posture(Prior_q, Mean_q, CovMx_q, xi_star_, q_star, E_Cov_I_q, E_Cov_O_q); 
        // a_ctrl.get_expected_posture(Prior_qdot, Mean_qdot, CovMx_qdot, xi_dot_star_, q_dot_hat_, E_Cov_I_qdot, E_Cov_O_qdot);    // velo
        // a_ctrl.get_expected_posture(Prior_qddot, Mean_qddot, CovMx_qddot, xi_ddot_star_, q_ddot_hat_, E_Cov_I_qddot, E_Cov_O_qddot); // accel
        // // update of the task covariance matrices
        // a_ctrl.set_IIK_weights(E_Cov_O_qdot, E_Cov_I_qdot, E_Cov_O_qddot, E_Cov_I_qddot);
        // Inverse kinematics 
        WBIK.get_wb_IK(robot_model_, jts_pos, W_H_B, des_X_EE, stance_ft, Pose_lfoot, Pose_rfoot);
        // update the robot model with the expected joints posture
        VectorXd jts_pos_star_   = WBIK.des_jts_pos; 
        Matrix4d WHB_star_       = WBIK.des_WHB;     
        a_ctrl.update_model(robot_model_, jts_pos_star_, WHB_star_);
        // Get instantaneous inverse kinematics (postural velocity and acceleration)
        a_ctrl.get_instantaneous_IK(robot_model_, xi_dot_star_, q_dot_hat_, xi_ddot_star_, q_ddot_hat_, q_dot_star, q_ddot_star);           
        a_ctrl.get_centroidal_variables(robot_model_, jts_pos_star_, WHB_star_, q_dot_star, q_ddot_star);   // compute the predicted centroidal variables
        a_ctrl.compute_posture_perturbation(Wrench_hands_star_);                                            // Compute the predicted balance perturbation
        a_ctrl.compute_capture_point(Wrench_hands_star_);                                                   // compute the Predicted the capture point
        // updating the Perturbation and capture point
        W_Disturb = a_ctrl.Delat_R_cmp;                                                                     // balance perturbation
        W_CP_     = a_ctrl.CP.head(2);                                                                      // capture point 
            
        // get the points defining the support polygon of each feet
        // a_ctrl.get_feet_support_points(Pose_lfoot, Pose_rfoot, DeltaStep, AllPtsInAbsFoot, W_Rot_AbsFoot, W_Pos_AbsFoot);  // -->> AllPtsInAbsFoot, W_Rot_AbsFoot, W_Pos_AbsFoot
        a_ctrl.get_feet_support_points_C(Pose_lfoot, Pose_rfoot, DeltaStep, DeltaThetaFeet_, AllPtsInAbsFoot, W_Rot_AbsFoot, W_Pos_AbsFoot);  // -->> AllPtsInAbsFoot, W_Rot_AbsFoot, W_Pos_AbsFoot
        
        // update convex hull
        a_ctrl.ConvHull.convexHull(AllPtsInAbsFoot, PointsCHull, Abs_EdgesNormalCHull, DistanceEdgesCHull); // --> PointsCHull, EdgesNormalCHull, DistanceEdgesCHull
        // Express the normal in the world frame
        W_EdgesNormalCHull = Abs_EdgesNormalCHull * W_Rot_AbsFoot.transpose();
        // predict the stability
        stable_0 = a_ctrl.predict_stability(W_EdgesNormalCHull, DistanceEdgesCHull, W_CP_, W_Pos_AbsFoot, W_Disturb);
        //
        std::cout << " count : \t" << count << "    stable_0 : \t" << stable_0 << std::endl;
        // *****************************************************************************************
        // Record the data Poses and joints configuration
        // *****************************************************************************************           
        //  
        int stance_left = 0; 
        if(a_ctrl.stance_0 =="left") stance_left = 1;
        OutRecord_stability_var << count << "  " << W_Disturb.transpose() << "  " << W_CP_.transpose() << "  " << W_Pos_AbsFoot.transpose() <<  "  ";
        OutRecord_stability_var << W_Rot_AbsFoot.row(0) << "  " << W_Rot_AbsFoot.row(1) <<  "  ";
        OutRecord_stability_var << DeltaStep.transpose() << "  " << a_ctrl.Delta_theta_feet << "  " << stable_0 << "  " << stance_left << "  " << 0.0 << endl;
        OutRecord_JointsPosition << jts_pos.transpose() << endl;

        if(PointsCHull.rows() == 5)
        {
            OutRecord_PointsCHull   << PointsCHull.col(0).transpose() << "  " << PointsCHull(0,0) << "  " << PointsCHull(0,0) << "  ";   // x
            OutRecord_PointsCHull   << PointsCHull.col(1).transpose() << "  " << PointsCHull(0,1) << "  " << PointsCHull(0,1) << endl;   // y

            OutRecord_NormalCHull   << W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull(0,0) << "  " << W_EdgesNormalCHull(0,0) << "  ";   // x
            OutRecord_NormalCHull   << W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull(0,1) << "  " << W_EdgesNormalCHull(0,1) << endl;   // y

            OutRecord_DistanceCHull << DistanceEdgesCHull.transpose() << "  " << DistanceEdgesCHull(0) << "  " << DistanceEdgesCHull(0) << endl;    
        }
        else if(PointsCHull.rows() == 6)
        {
            OutRecord_PointsCHull   << PointsCHull.col(0).transpose() << "  " << PointsCHull(0,0) << "  " ;   // x
            OutRecord_PointsCHull   << PointsCHull.col(1).transpose() << "  " << PointsCHull(0,1) << endl;   // y

            OutRecord_NormalCHull   << W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull(0,0) << "  ";   // x
            OutRecord_NormalCHull   << W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull(0,1) << endl;      // y

            OutRecord_DistanceCHull << DistanceEdgesCHull.transpose() << "  " << DistanceEdgesCHull(0) << endl;
        }
        else
        {
            OutRecord_PointsCHull   << PointsCHull.col(0).transpose() << "  " << PointsCHull.col(1).transpose() << endl;
            OutRecord_NormalCHull   << W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull.col(1).transpose() << endl;
            OutRecord_DistanceCHull << DistanceEdgesCHull.transpose() << endl;  
        }
        //
        count++;
    } 
    //
    // stance indicator
    Vector2d stance_ind;
        if(a_ctrl.stance_0 =="left")
            stance_ind << 0.0, 1.0;
        else // stance is right
            stance_ind << 1.0, 0.0;
    //
    if((stable_0 == true) && (a_ctrl.lengthStep <= a_ctrl.lengthStep_max))
    {
            a_ctrl.step_values << DeltaStep(0), DeltaStep(1), a_ctrl.Delta_theta_feet, stance_ind(0), stance_ind(1), 0.0;
    }
    
    //
    OutRecord_stability_var.close();
    OutRecord_PointsCHull.close();
    OutRecord_NormalCHull.close();
    OutRecord_DistanceCHull.close();
    OutRecord_JointsPosition.close();
    
    std::cout<< "FFWD compute_stepping_pose OK   : \t" << 0.0 << std::endl;
    
    return true;
}

//
bool FeedForwardController::heuristic_gridSeach_Step(  RobotInterface& robot_interface_,  WbRobotModel& robot_model_, VectorXd Wrench_hands_star_, Vector6d des_TspMotion[], 
                                                        Vector7d des_X_EE_[], Vector7d Pose_lfoot, Vector7d Pose_rfoot)
{
    int count = 0;
    bool stable_0 = false;
    bool stable_1 = false;
    //
    MatrixXd AllPtsInAbsFoot;
    MatrixXd PointsCHull;
    VectorXd DistanceEdgesCHull;
    MatrixXd Abs_EdgesNormalCHull;
    MatrixXd W_EdgesNormalCHull;
    Matrix2d W_Rot_AbsFoot;
    Vector2d W_Pos_AbsFoot;
    Vector2d W_Disturb;
    // 
    robot_model_.EstimateRobotStates(robot_interface_, stance_ft, m_joints_sensor_wb, W_H_B, VB);  // Estimate the robot state
    jts_pos = robot_model_.m_jts_sensor_wb.position;

    a_ctrl.update_model(robot_model_, jts_pos, W_H_B);                                          // update the robot model
    a_ctrl.get_centroidal_variables(robot_model_, jts_pos, W_H_B, q_dot_star, q_ddot_star);     // compute the predicted centroidal variables
    a_ctrl.compute_posture_perturbation(Wrench_hands_star_);                                    // Compute the predicted balance perturbation
    a_ctrl.compute_capture_point(Wrench_hands_star_);                                           // compute the Predicted the capture point
    // capture point wrt world frame
    Vector2d W_CP_(a_ctrl.CP(0),  a_ctrl.CP(1));
    double DeltaStepLength = 0;
    
    // direction of the balance disturbance
    double theta_Dist = 0.0;
    double DeltaThetaFeet_ = 0.0;
    double dTheta_st, dTheta_sw, dDThetaFeet;
    // double  theta_Dist = atan2( a_ctrl.Delat_R_cmp(1),  a_ctrl.Delat_R_cmp(0));
    Vector2d DeltaStep(DeltaStepLength * cos(theta_Dist), DeltaStepLength * sin(theta_Dist));   // step increase
    Vector2d DeltaDisturb(0.0, 0.0);
    // ===========================================================================================================
    for(int i=0; i<8; i++) des_X_EE[i] = des_X_EE_[i];

    // =======================================================================================================
    // extraction of Euler angles of  the feet rotation 
    Vector3d o_lf = a_ctrl.Transforms.getEulerAnglesXYZ_FixedFrame(des_X_EE_[2]);
    Vector3d o_rf = a_ctrl.Transforms.getEulerAnglesXYZ_FixedFrame(des_X_EE_[3]);

    // // while((stable_0 == true)||(lengthStep <= lengthStep_max))
    // while((!stable_0) && (a_ctrl.lengthStep <= a_ctrl.lengthStep_max))

    // Step variables range
    VectorXd DeltaStep_r      = VectorXd::LinSpaced(20,low_Sr,high_Sr);
    VectorXd DeltaStep_dir_l  = VectorXd::LinSpaced(30,low_dl,high_dl);
    VectorXd DeltaStep_dir_r  = VectorXd::LinSpaced(30,low_dr,high_dr);
    VectorXd DeltaStep_thetaF = VectorXd::LinSpaced(30,low_TF,high_TF);

    for(int i=0; i<DeltaStep_r.rows(); i++)
    {
        for(int j=0; j<DeltaStep_dir_l.rows(); j++)
        {
            for(int k=0; k<DeltaStep_thetaF.rows(); k++)
            {
                // update step
                // DeltaStepLength = double(count) *  a_ctrl.stepIncrease;
                // DeltaStep << DeltaStepLength * cos(theta_Dist), DeltaStepLength * sin(theta_Dist);
                if(a_ctrl.stance_0 =="left")    // left stance foot for anticipatory step
                {
                    dTheta_st   = o_lf(2);
                    DeltaStep << DeltaStep_r(i) * cos(dTheta_st + DeltaStep_dir_r(j)), 
                                 DeltaStep_r(i) * sin(dTheta_st + DeltaStep_dir_r(j));
                    DeltaThetaFeet_ = DeltaStep_thetaF(k);
                }
                else  // right stance foot for anticipatory step
                {
                    dTheta_st   = o_rf(2);
                    DeltaStep << DeltaStep_r(i) * cos(dTheta_st + DeltaStep_dir_l(j)), 
                                 DeltaStep_r(i) * sin(dTheta_st + DeltaStep_dir_l(j));
                    DeltaThetaFeet_ = DeltaStep_thetaF(k);
                }   
                // if     (DeltaThetaFeet_ <= -a_ctrl.Dtheta_feet_max) DeltaThetaFeet_ = -a_ctrl.Dtheta_feet_max;
             //    else if(DeltaThetaFeet_ >=  a_ctrl.Dtheta_feet_max) DeltaThetaFeet_ =  a_ctrl.Dtheta_feet_max;

                // update the balance perturbation
                // --------------------------------
                a_ctrl.compute_delta_disturb(Wrench_hands_star_, 0.5*DeltaStep, DeltaDisturb);  // compute step induced variation of perturbation
                // updating the Perturbation and capture point
                W_Disturb = a_ctrl.Delat_R_cmp + DeltaDisturb;                                  // balance perturbation
                W_CP_     = a_ctrl.CP.head(2)  + 0.5 * DeltaStep;                               // update capture point in absolute frame
                // get the points defining the support polygon of each feet
                // a_ctrl.get_feet_support_points(Pose_lfoot, Pose_rfoot, DeltaStep, 
                //                      AllPtsInAbsFoot, W_Rot_AbsFoot, W_Pos_AbsFoot);  // -->> AllPtsInAbsFoot, W_Rot_AbsFoot, W_Pos_AbsFoot
                a_ctrl.get_feet_support_points_C(Pose_lfoot, Pose_rfoot, DeltaStep, DeltaThetaFeet_, 
                                                    AllPtsInAbsFoot, W_Rot_AbsFoot, W_Pos_AbsFoot); 
                // update convex hull
                a_ctrl.ConvHull.convexHull(AllPtsInAbsFoot, PointsCHull, Abs_EdgesNormalCHull, DistanceEdgesCHull); // --> PointsCHull, EdgesNormalCHull, DistanceEdgesCHull
                // Express the normal in the world frame
                W_EdgesNormalCHull = Abs_EdgesNormalCHull * W_Rot_AbsFoot.transpose();
                // predict the stability
                stable_0 = a_ctrl.predict_stability(W_EdgesNormalCHull, DistanceEdgesCHull, W_CP_, W_Pos_AbsFoot, W_Disturb);
                //
                // theta_Dist = atan2( a_ctrl.Delat_R_cmp(1),  a_ctrl.Delat_R_cmp(0));
                // DeltaThetaFeet_ = theta_Dist - dTheta_st;
                // DeltaThetaFeet_ = DeltaStep_thetaF(k);
                // if     (DeltaThetaFeet_ <= -a_ctrl.Dtheta_feet_max) DeltaThetaFeet_ = -a_ctrl.Dtheta_feet_max;
             //    else if(DeltaThetaFeet_ >=  a_ctrl.Dtheta_feet_max) DeltaThetaFeet_ =  a_ctrl.Dtheta_feet_max;
                //
                std::cout << " count : \t" << count << "    stable_0 : \t" << stable_0 << std::endl;
                // *****************************************************************************************
                // Record the data Poses and joints configuration
                // *****************************************************************************************           
                int stance_left = 0; 
                if(a_ctrl.stance_0 =="left") stance_left = 1;

                if(stable_0)
                { 
                    OutRecord_stability_var2 << count << "  " << W_Disturb.transpose() << "  " << W_CP_.transpose() << "  " << W_Pos_AbsFoot.transpose() <<  "  ";
                    OutRecord_stability_var2 << W_Rot_AbsFoot.row(0) << "  " << W_Rot_AbsFoot.row(1) <<  "  ";
                    OutRecord_stability_var2 << DeltaStep.transpose() << "  " << a_ctrl.Delta_theta_feet << "  " << stable_0 << "  " << stance_left << "  " << 0.0 << endl;
       
                    OutRecord_JointsPosition2 << jts_pos.transpose() << endl;
                    
                    if(PointsCHull.rows() == 5)
                    {
                        OutRecord_PointsCHull2  << PointsCHull.col(0).transpose() << "  " << PointsCHull(0,0) << "  " << PointsCHull(0,0) << "  ";   // x
                        OutRecord_PointsCHull2  << PointsCHull.col(1).transpose() << "  " << PointsCHull(0,1) << "  " << PointsCHull(0,1) << endl;   // y
       
                        OutRecord_NormalCHull2      << W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull(0,0) << "  " << W_EdgesNormalCHull(0,0) << "  ";   // x
                        OutRecord_NormalCHull2      << W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull(0,1) << "  " << W_EdgesNormalCHull(0,1) << endl;   // y
       
                        OutRecord_DistanceCHull2 << DistanceEdgesCHull.transpose() << "  " << DistanceEdgesCHull(0) << "  " << DistanceEdgesCHull(0) << endl;   
                    }
                    else if(PointsCHull.rows() == 6)
                    {
                        OutRecord_PointsCHull2  << PointsCHull.col(0).transpose() << "  " << PointsCHull(0,0) << "  " ;   // x
                        OutRecord_PointsCHull2  << PointsCHull.col(1).transpose() << "  " << PointsCHull(0,1) << endl;   // y
       
                        OutRecord_NormalCHull2      << W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull(0,0) << "  ";   // x
                        OutRecord_NormalCHull2      << W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull(0,1) << endl;      // y
       
                        OutRecord_DistanceCHull2 << DistanceEdgesCHull.transpose() << "  " << DistanceEdgesCHull(0) << endl;
                    }
                    else
                    {
                        OutRecord_PointsCHull2  << PointsCHull.col(0).transpose() << "  " << PointsCHull.col(1).transpose() << endl;
                        OutRecord_NormalCHull2      << W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull.col(1).transpose() << endl;
                        OutRecord_DistanceCHull2 << DistanceEdgesCHull.transpose() << endl; 
                    }
                }
                //
                count++;
            }
        }
    } 
    //
    OutRecord_stability_var2.close();
    OutRecord_PointsCHull2.close();
    OutRecord_NormalCHull2.close();
    OutRecord_DistanceCHull2.close();
    OutRecord_JointsPosition2.close();
    
    std::cout<< "FFWD compute_stepping_pose OK   : \t" << 0.0 << std::endl;
    
    return true;
}

// to be used whitin the compute stepping function
bool FeedForwardController::update_task_variables(Vector2d DeltaStep, Vector6d des_TspMotion[], Vector7d des_X_EE_[])
{
    // //
    // if(a_ctrl.stance_0 =="left") // left stance foot for anticipatory step
 //    {
 //     des_X_EE[2] = des_X_EE_[2];                                 // left foot pose
 //     des_X_EE[3] = des_X_EE_[3];                                 // right foot pose
 //     des_X_EE[3].head(2) = des_X_EE_[3].head(2) + DeltaStep;     // new right foot ground position
 //    }
 //    else  // right stance foot for anticipatory step
 //    {
 //     des_X_EE[2] = des_X_EE_[2];                                 // left foot pose
 //     des_X_EE[2].head(2) = des_X_EE_[2].head(2) + DeltaStep;     // new left foot ground position
 //     des_X_EE[3] = des_X_EE_[3];                                 // right foot pose
 //    }        
    // update the coordinated bimanual task variables
    Vector7d w_abs_pose_feet;
    Matrix4d WH_aF_;
    VectorXd aF_hands_velo(12), aF_hands_accel(12);
    a_ctrl.Transforms.get_absolute_pose(des_X_EE[2], des_X_EE[3], w_abs_pose_feet);   // compute the absolute pose between two foot poses
    WH_aF_   = a_ctrl.Transforms.PoseVector2HomogenousMx(w_abs_pose_feet);
    MatrixXd R6x6_WaF_ = Eigen::MatrixXd::Zero(6,6);
             R6x6_WaF_.block<3,3>(0,0) = WH_aF_.block<3,3>(0,0);
             R6x6_WaF_.block<3,3>(3,3) = WH_aF_.block<3,3>(0,0);

    aF_hands_velo.head(6)  = R6x6_WaF_ * des_TspMotion[0]; 
    aF_hands_velo.tail(6)  = R6x6_WaF_ * des_TspMotion[1];
    aF_hands_accel.head(6) = R6x6_WaF_ * des_TspMotion[2];
    aF_hands_accel.tail(6) = R6x6_WaF_ * des_TspMotion[3];

    MatrixXd C_hands_ = a_ctrl.Transforms.get_bimanual_task_TwistMap(a_ctrl.a_bi, a_ctrl.b_bi);
    //
    xi_star_        = a_ctrl.Transforms.get_Coordinated_task_pose(des_X_EE[0], des_X_EE[1], WH_aF_);
    xi_dot_star_    = C_hands_ * aF_hands_velo;
    xi_ddot_star_   = C_hands_ * aF_hands_accel;

    return true;
}

//
bool FeedForwardController::predict_stability(  WbRobotModel& robot_model_, VectorXd Wrench_hands_star_, VectorXd predicted_jts_pos, 
                                                        Matrix4d  predicted_W_H_B, Vector7d Pose_lfoot, Vector7d Pose_rfoot)
{
    //
    stable_prediction = false;
    //
    MatrixXd AllPtsInAbsFoot;
    MatrixXd PointsCHull;
    VectorXd DistanceEdgesCHull;
    MatrixXd Abs_EdgesNormalCHull;
    MatrixXd W_EdgesNormalCHull;
    Matrix2d W_Rot_AbsFoot;
    Vector2d W_Pos_AbsFoot;
    Vector2d W_Disturb;
    // 
    double DeltaStepLength = 0; 
    // direction of the balance disturbance
    double theta_Dist = 0.0;
    double DeltaThetaFeet_ = 0.0;
    double dTheta_st, dTheta_sw, dDThetaFeet;
    Vector2d DeltaStep(DeltaStepLength * cos(theta_Dist), DeltaStepLength * sin(theta_Dist));   // step increase
    Vector2d DeltaDisturb(0.0, 0.0);


    // using the predicted posture 
    a_ctrl.update_model(robot_model_, predicted_jts_pos, predicted_W_H_B);
                                                                          // update the robot model
    a_ctrl.get_centroidal_variables(robot_model_, predicted_jts_pos, predicted_W_H_B, q_dot_star, q_ddot_star);                                 // compute the predicted centroidal variables
    a_ctrl.compute_posture_perturbation(Wrench_hands_star_);                                                                                    // Compute the predicted balance perturbation
    a_ctrl.compute_capture_point(Wrench_hands_star_);                                                                                           // compute the Predicted the capture point

    a_ctrl.get_feet_support_points_C(Pose_lfoot, Pose_rfoot, DeltaStep, DeltaThetaFeet_, AllPtsInAbsFoot, W_Rot_AbsFoot, W_Pos_AbsFoot);        // get the points of the Convex hull 
    a_ctrl.ConvHull.convexHull(AllPtsInAbsFoot, PointsCHull, Abs_EdgesNormalCHull, DistanceEdgesCHull);                                         // update convex hull   
    W_EdgesNormalCHull = Abs_EdgesNormalCHull * W_Rot_AbsFoot.transpose();                                                                      // Express the normal in the world frame
    // predict the stability
    stable_prediction = a_ctrl.predict_stability(W_EdgesNormalCHull, DistanceEdgesCHull, a_ctrl.CP.head(2), W_Pos_AbsFoot, a_ctrl.Delat_R_cmp);

    return stable_prediction;

}

bool FeedForwardController::compute_anticipative_step(  WbRobotModel& robot_model_, VectorXd Wrench_hands_star_, VectorXd predicted_jts_pos, 
                                                        Matrix4d  predicted_W_H_B, Vector7d Pose_lfoot, Vector7d Pose_rfoot)
{
    int count = 0;
    bool stable_0 = false;
    bool stable_1 = false;
    DeltaFootstepPose.setZero();
    //
    MatrixXd AllPtsInAbsFoot;
    MatrixXd PointsCHull;
    VectorXd DistanceEdgesCHull;
    MatrixXd Abs_EdgesNormalCHull;
    MatrixXd W_EdgesNormalCHull;
    Matrix2d W_Rot_AbsFoot;
    Vector2d W_Pos_AbsFoot;
    Vector2d W_Disturb;
    // 
    double DeltaStepLength = 0; 
    // direction of the balance disturbance
    double theta_Dist = 0.0;
    double DeltaThetaFeet_ = 0.0;
    double dTheta_st, dTheta_sw, dDThetaFeet;
    Vector2d DeltaStep(DeltaStepLength * cos(theta_Dist), DeltaStepLength * sin(theta_Dist));   // step increase
    Vector2d DeltaDisturb(0.0, 0.0);

    // using the predicted posture 
    a_ctrl.update_model(robot_model_, predicted_jts_pos, predicted_W_H_B);                                          // update the robot model
    a_ctrl.get_centroidal_variables(robot_model_, predicted_jts_pos, predicted_W_H_B, q_dot_star, q_ddot_star);     // compute the predicted centroidal variables
    a_ctrl.compute_posture_perturbation(Wrench_hands_star_);                                    // Compute the predicted balance perturbation
    a_ctrl.compute_capture_point(Wrench_hands_star_);                                           // compute the Predicted the capture point

    a_ctrl.get_feet_support_points_C(Pose_lfoot, Pose_rfoot, DeltaStep, DeltaThetaFeet_, AllPtsInAbsFoot, W_Rot_AbsFoot, W_Pos_AbsFoot);    // get the points of the Convex hull 

    a_ctrl.ConvHull.convexHull(AllPtsInAbsFoot, PointsCHull, Abs_EdgesNormalCHull, DistanceEdgesCHull);                                     // update convex hull   
    W_EdgesNormalCHull = Abs_EdgesNormalCHull * W_Rot_AbsFoot.transpose();                                                                  // Express the normal in the world frame
    // predict the stability
    stable_0 = a_ctrl.predict_stability(W_EdgesNormalCHull, DistanceEdgesCHull, a_ctrl.CP.head(2), W_Pos_AbsFoot, a_ctrl.Delat_R_cmp);
    // =================================================================================================================================
     // capture point wrt world frame
    Vector2d W_CP_(a_ctrl.CP(0),  a_ctrl.CP(1));
    // double  theta_Dist = atan2( a_ctrl.Delat_R_cmp(1),  a_ctrl.Delat_R_cmp(0));
    // Vector2d DeltaStep(DeltaStepLength * cos(theta_Dist), DeltaStepLength * sin(theta_Dist));   // step increase
    // Vector2d DeltaDisturb(0.0, 0.0);
    // =======================================================================================================
    // extraction of Euler angles of  the feet rotation 
    Vector3d o_lf = a_ctrl.Transforms.getEulerAnglesXYZ_FixedFrame(Pose_lfoot);
    Vector3d o_rf = a_ctrl.Transforms.getEulerAnglesXYZ_FixedFrame(Pose_rfoot);
    // Step variables range
    VectorXd DeltaStep_r        = VectorXd::LinSpaced(20,low_Sr,high_Sr);
    VectorXd DeltaStep_thetaF   = VectorXd::LinSpaced(20,low_TF,high_TF);
    VectorXd DeltaStep_theta_l  = VectorXd::LinSpaced(30,low_dl,high_dl);
    VectorXd DeltaStep_theta_r  = VectorXd::LinSpaced(30,low_dr,high_dr);

    int iter_xy = 0;
    int iter_o  = 0;
    bool DStepFound = false;
    bool SolverDone = false;
    double Jdxdydo_0,  Jdxdydo;         Jdxdydo_0 = Jdxdydo = 0.0;
    double DevTheta_0, DevTheta;        DevTheta_0= DevTheta= 0.0;
    double desDeltaThetaFeet_ = 0.0;
    int sol_index = 1;

    double t_solution = yarp::os::Time::now();
    // =======================================================================================================
    while((!SolverDone) && (count <= (DeltaStep_r.rows() + DeltaStep_thetaF.rows()) ))
    {
        // update step
        a_ctrl.compute_delta_disturb(Wrench_hands_star_, 0.5*DeltaStep, DeltaDisturb);                                                          // compute step induced variation of perturbation
        // updating the Perturbation and capture point
        W_Disturb = a_ctrl.Delat_R_cmp + DeltaDisturb;                                                                                          // balance perturbation
        W_CP_     = a_ctrl.CP.head(2)  + 0.5 * DeltaStep;                                                                                       // update capture point in absolute frame

        a_ctrl.get_feet_support_points_C(Pose_lfoot, Pose_rfoot, DeltaStep, DeltaThetaFeet_, AllPtsInAbsFoot, W_Rot_AbsFoot, W_Pos_AbsFoot);    // get the points of the Convex hull 
        a_ctrl.ConvHull.convexHull(AllPtsInAbsFoot, PointsCHull, Abs_EdgesNormalCHull, DistanceEdgesCHull);                                     // update convex hull   
        W_EdgesNormalCHull = Abs_EdgesNormalCHull * W_Rot_AbsFoot.transpose();                                                                  // Express the normal in the world frame
        // predict the stability
        stable_0 = a_ctrl.predict_stability(W_EdgesNormalCHull, DistanceEdgesCHull, W_CP_, W_Pos_AbsFoot, W_Disturb);
        //
        if(stable_0 && !DStepFound)
        {
            Jdxdydo_0 = 1./a_ctrl.lengthStep_max  * (pow(DeltaStep(0),2.) + pow(DeltaStep(1),2.)) 
                      + 1./a_ctrl.Dtheta_feet_max * (pow(DeltaThetaFeet_, 2.));
            sol_index = count + 1;
        }
        //
        // update the balance perturbation
        // --------------------------------
        if(stable_0) DStepFound = true;

        if(!stable_0 && !DStepFound && !DStepMaxReached)
        {
            if  (a_ctrl.stance_0 =="left")  dTheta_st = o_lf(2);    // left stance foot for anticipatory step
            else                            dTheta_st = o_rf(2);    // right stance foot for anticipatory step          
            DeltaStep <<    DeltaStep_r(iter_xy) * cos(dTheta_st + DeltaThetaFeet_), 
                            DeltaStep_r(iter_xy) * sin(dTheta_st + DeltaThetaFeet_);
            //
            theta_Dist = atan2( a_ctrl.Delat_R_cmp(1),  a_ctrl.Delat_R_cmp(0));
            DeltaThetaFeet_ = theta_Dist - dTheta_st;
            if     (DeltaThetaFeet_ <= -a_ctrl.Dtheta_feet_max) DeltaThetaFeet_ = -a_ctrl.Dtheta_feet_max;
            else if(DeltaThetaFeet_ >=  a_ctrl.Dtheta_feet_max) DeltaThetaFeet_ =  a_ctrl.Dtheta_feet_max;
            //
            Jdxdydo_0 = 1./a_ctrl.lengthStep_max  * (pow(DeltaStep(0),2.) + pow(DeltaStep(1),2.)) 
                      + 1./a_ctrl.Dtheta_feet_max * (pow(DeltaThetaFeet_, 2.));
            //
            iter_xy ++;
            if(iter_xy >= DeltaStep_r.rows()) 
            {
                iter_xy = DeltaStep_r.rows()-1;   // CONDITION TO PERFORM TWO FEET STEPS
                DStepMaxReached = true;                // set to true to move to the orientation loop
            }
        }
        
        if(DStepFound && !DStepMaxReached)    // orientation 
        {
            if (stable_0)
            {
                // computation of the cost function
                Jdxdydo = 1./a_ctrl.lengthStep_max  * (pow(DeltaStep(0),2.) + pow(DeltaStep(1),2.)) 
                        + 1./a_ctrl.Dtheta_feet_max * (pow(DeltaThetaFeet_, 2.));
                // computation deviation wrt the disturbance
                DevTheta = fabs(theta_Dist - (dTheta_st+DeltaThetaFeet_));
                // selection of DeltaThetaFeet corresponding to min Jxyo
                if(Jdxdydo < Jdxdydo_0)
                {
                    desDeltaThetaFeet_ = DeltaThetaFeet_;
                    Jdxdydo_0  = Jdxdydo;
                    DevTheta_0 = DevTheta;
                    sol_index  = count + 1;

                } else if(Jdxdydo == Jdxdydo_0)
                {
                    // choose DThetaFeet with mimimum deviation wrt the disturbance
                    if(DevTheta < DevTheta_0)
                    {
                        desDeltaThetaFeet_ = DeltaThetaFeet_;
                        Jdxdydo_0  = Jdxdydo;
                        DevTheta_0 = DevTheta;
                        sol_index  = count + 1;
                    }
                }
            }
            //
            DeltaStep       = DeltaStep;
            DeltaThetaFeet_ = DeltaStep_thetaF(iter_o);                 // update theta
            // 
            iter_o ++; 
            if(iter_o >= DeltaStep_thetaF.rows()) 
            {
                SolverDone = true;
                DeltaFootstepPose.head(2) = DeltaStep;
                DeltaFootstepPose(2)      = desDeltaThetaFeet_;
            }
        }

        //
        if(!DStepFound && DStepMaxReached)
        {
            // computation of the cost function
            Jdxdydo = 1./a_ctrl.lengthStep_max  * (pow(DeltaStep(0),2.) + pow(DeltaStep(1),2.)) 
                    + 1./a_ctrl.Dtheta_feet_max * (pow(DeltaThetaFeet_, 2.));
            // computation deviation wrt the disturbance
            DevTheta = fabs(theta_Dist - (dTheta_st+DeltaThetaFeet_));
            // selection of DeltaThetaFeet corresponding to min Jxyo
            if(Jdxdydo < Jdxdydo_0)
            {
                desDeltaThetaFeet_ = DeltaThetaFeet_;
                Jdxdydo_0  = Jdxdydo;
                DevTheta_0 = DevTheta;
                sol_index  = count + 1;

            } else if(Jdxdydo == Jdxdydo_0)
            {
                // choose DThetaFeet with mimimum deviation wrt the disturbance
                if(DevTheta < DevTheta_0)
                {
                    desDeltaThetaFeet_ = DeltaThetaFeet_;
                    Jdxdydo_0  = Jdxdydo;
                    DevTheta_0 = DevTheta;
                    sol_index  = count + 1;
                }
            }
            //
            DeltaStep       = DeltaStep;
            DeltaThetaFeet_ = DeltaStep_thetaF(iter_o);                 // update theta
            // 
            iter_o ++; 
            if(iter_o >= DeltaStep_thetaF.rows()) 
                {
                    SolverDone = true;
                    DeltaFootstepPose.head(2) = DeltaStep;
                    DeltaFootstepPose(2)      = desDeltaThetaFeet_;
                    DeltaFootstepPose.tail(3) = DeltaFootstepPose.head(3);
                }

        }
        else {
            printf("Could not compute an anticipative step \n");
            return true;
        }
        
        // *********************************************************************************************************************************
        // Record the data Poses and joints configuration
        // *********************************************************************************************************************************           
        int stance_left = 0; 
        if(a_ctrl.stance_0 =="left") stance_left = 1;

            OutRecord_stability_var << count << "  " << W_Disturb.transpose() << "  " << W_CP_.transpose() << "  " << W_Pos_AbsFoot.transpose() <<  "  ";
            OutRecord_stability_var << W_Rot_AbsFoot.row(0) << "  " << W_Rot_AbsFoot.row(1) <<  "  ";
            OutRecord_stability_var << DeltaStep.transpose() << "  " << a_ctrl.Delta_theta_feet << "  " << stable_0 << "  " << stance_left << "  " << sol_index << endl; // sol_index
            OutRecord_JointsPosition << predicted_jts_pos.transpose() << endl;
            
            if(PointsCHull.rows() == 5)
            {
                OutRecord_PointsCHull   << PointsCHull.col(0).transpose() << "  " << PointsCHull(0,0) << "  " << PointsCHull(0,0) << "  ";                          // x
                OutRecord_PointsCHull   << PointsCHull.col(1).transpose() << "  " << PointsCHull(0,1) << "  " << PointsCHull(0,1) << endl;                          // y
                OutRecord_NormalCHull   << W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull(0,0) << "  " << W_EdgesNormalCHull(0,0) << "  ";     // x
                OutRecord_NormalCHull   << W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull(0,1) << "  " << W_EdgesNormalCHull(0,1) << endl;     // y
                OutRecord_DistanceCHull << DistanceEdgesCHull.transpose() << "  " << DistanceEdgesCHull(0) << "  " << DistanceEdgesCHull(0) << endl;    
            }
            else if(PointsCHull.rows() == 6)
            {
                OutRecord_PointsCHull   << PointsCHull.col(0).transpose() << "  " << PointsCHull(0,0) << "  " ;                                                     // x
                OutRecord_PointsCHull   << PointsCHull.col(1).transpose() << "  " << PointsCHull(0,1) << endl;                                                      // y
                OutRecord_NormalCHull   << W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull(0,0) << "  ";                                        // x
                OutRecord_NormalCHull   << W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull(0,1) << endl;                                        // y
                OutRecord_DistanceCHull << DistanceEdgesCHull.transpose() << "  " << DistanceEdgesCHull(0) << endl;
            }
            else
            {
                OutRecord_PointsCHull   << PointsCHull.col(0).transpose() << "  " << PointsCHull.col(1).transpose() << endl;
                OutRecord_NormalCHull   << W_EdgesNormalCHull.col(0).transpose() << "  " << W_EdgesNormalCHull.col(1).transpose() << endl;
                OutRecord_DistanceCHull << DistanceEdgesCHull.transpose() << endl;  
            }

        count++;
    }

    // std::cout<< "MAIN COMPUTATION AT : " << count << " GOT IN : " <<  yarp::os::Time::now()-t_solution << " s" << std::endl;
    // std::cout<< "RECORDING TIME AT : " << count << " GOT IN : " <<  yarp::os::Time::now()-t_solution << " s" << std::endl;
    // std::cout<< "FFWD compute_stepping_pose OK   : \t" << 0.0 << std::endl;
    
    return DStepFound;
}




// closing of the ff controller to be called if the run function is used
void FeedForwardController::close()
{
    //
    OutRecord_stability_var.close();
    OutRecord_PointsCHull.close();
    OutRecord_NormalCHull.close();
    OutRecord_DistanceCHull.close();
    OutRecord_JointsPosition.close();
    //
}



// ==========================================================//
//
bool FeedForwardController::Esimate_KinoDynamicStates(double dt_, Vector7d Pose_EE[], VectorXd Velo_Hands_k[], VectorXd Velo_Hands_k1[], VectorXd Wrench_Hands_)
{
    //
    Vector3d pos_CoM_hat_k, pos_CoM_hat_k1, pos_CoM_hat_k2;                 //   expected CoM position
    Vector6d c_momentum_hat_k, c_momentum_hat_k1, c_momentum_hat_k2;        //   expected centroidal momentum
    MatrixXd E_Cov_Hands_k, E_Cov_Hands_k1, E_Cov_Hands_k2;                 //   hands task covariance matrices 
    MatrixXd E_Cov_CoM_k, E_Cov_CoM_k1, E_Cov_CoM_k2;                       //   Centroidal task covariance matrices
    Vector6d c_momentum_hat_dot;                                            //   rate of centroidal momentum
    // 
    pos_CoM_hat_k      = VectorXd::Zero(3); 
    pos_CoM_hat_k1     = VectorXd::Zero(3); 
    pos_CoM_hat_k2     = VectorXd::Zero(3);
    c_momentum_hat_k   = VectorXd::Zero(6); 
    c_momentum_hat_k1  = VectorXd::Zero(6); 
    c_momentum_hat_k2  = VectorXd::Zero(6); 
    //
    VectorXd hands_task[3];                         //   hands task (14+12)
    for(int i=0; i<3; i++)
        hands_task[i]       = VectorXd::Zero(26);
    
    // get the absolute feet frame
    //----------------------------
    Vector7d W_Pose_aF;
    a_ctrl.Transforms.get_absolute_pose(Pose_EE[2], Pose_EE[3], W_Pose_aF);
    Matrix4d W_H_aF_ = a_ctrl.Transforms.PoseVector2HomogenousMx(W_Pose_aF); 
    // Transformation of the hand pose in the absolute feet frame
    Matrix4d aF_H_lh = W_H_aF_.inverse() * a_ctrl.Transforms.PoseVector2HomogenousMx(Pose_EE[0]);
    Matrix4d aF_H_rh = W_H_aF_.inverse() * a_ctrl.Transforms.PoseVector2HomogenousMx(Pose_EE[1]);

    // current hands task
    hands_task[0].segment( 0, 7) = a_ctrl.Transforms.HomogenousMx2PoseVector(aF_H_lh); // lhand in absolute feet frame
    hands_task[0].segment( 7, 7) = a_ctrl.Transforms.HomogenousMx2PoseVector(aF_H_rh); // rhand in absolute feet frame
    hands_task[0].segment(14, 3) = (W_H_aF_.block<3,3>(0,0)).transpose()* Velo_Hands_k[0].head(3);
    hands_task[0].segment(17, 3) = (W_H_aF_.block<3,3>(0,0)).transpose()* Velo_Hands_k[0].tail(3);
    hands_task[0].segment(20, 3) = (W_H_aF_.block<3,3>(0,0)).transpose()* Velo_Hands_k[1].head(3);
    hands_task[0].segment(23, 3) = (W_H_aF_.block<3,3>(0,0)).transpose()* Velo_Hands_k[1].tail(3);

    // estimate accelration at time k
    // -------------------------------
    Vector3d a_hat[2]; 
    for(int i=0; i<2; i++)  // [0]: left hand, [1]: right hand
    {
        a_hat[i].head(3) = 1./dt_ * W_H_aF_.block<3,3>(0,0).transpose()* (Velo_Hands_k1[i].head(3) - Velo_Hands_k[i].head(3));  // linear accel
        a_hat[i].tail(3) = 1./dt_ * W_H_aF_.block<3,3>(0,0).transpose()* (Velo_Hands_k1[i].tail(3) - Velo_Hands_k[i].tail(3));  // angular accel
    }
    // prediction of the hands states over two sampling time ahead (for numerical derivation of the centroidal momentum) 
    //------------------------------------------------------------------------------------------------------------------
    for(int i=0; i<2; i++)
    {
        // positions
        //--------------
        // left hand
        hands_task[i+1].segment( 0, 3) = hands_task[i].segment( 0, 3) + hands_task[i].segment(14, 3) * dt_ + 0.5*(dt_*dt_)*a_hat[0];    // pos
        hands_task[i+1].segment(14, 3) = hands_task[i].segment(14, 3) + dt_*a_hat[0];                                                   // vel
        // right hand positions
        hands_task[i+1].segment( 7, 3) = hands_task[i].segment( 7, 3) + hands_task[i].segment(20, 3) * dt_ + 0.5*(dt_*dt_)*a_hat[1];    // pos
        hands_task[i+1].segment(20, 3) = hands_task[i].segment(20, 3) + dt_*a_hat[1];                                                   // vel
        // orientation
        //-------------- 
        // left hand
        hands_task[i+1].segment( 3, 4) = hands_task[i].segment( 3, 4);    // ori
        // right hand
        hands_task[i+1].segment(10, 4) = hands_task[i].segment(10, 4);    // ori
        //
    }

    // std::cout << " //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// : \n" << std::endl;
    // std::cout << " hands_task[0]   IS : \n" << hands_task[0] << std::endl;
    // std::cout << " hands_task[1]   IS : \n" << hands_task[1] << std::endl;
    // std::cout << " hands_task[2]   IS : \n" << hands_task[2] << std::endl;
 
    // get the expected Centroidal variables condoitioned by the hands task states
    // ---------------------------------------------------------------------------
        a_ctrl.get_expected_centroidal_task(hands_task[0],  pos_CoM_hat_k,  c_momentum_hat_k,  E_Cov_Hands_k,  E_Cov_CoM_k);
        a_ctrl.get_expected_centroidal_task(hands_task[1],  pos_CoM_hat_k1, c_momentum_hat_k1, E_Cov_Hands_k1, E_Cov_CoM_k1);
        a_ctrl.get_expected_centroidal_task(hands_task[2],  pos_CoM_hat_k2, c_momentum_hat_k2, E_Cov_Hands_k2, E_Cov_CoM_k2);

        // std::cout << " pos_CoM_hat_    " << 0 << " is :  \n "<< pos_CoM_hat_k << std::endl;
        // std::cout << " c_momentum_hat_ " << 0 << " is :  \n "<< c_momentum_hat_k << std::endl;
        // std::cout << " pos_CoM_hat_    " << 1 << " is :  \n "<< pos_CoM_hat_k1 << std::endl;
        // std::cout << " c_momentum_hat_ " << 1 << " is :  \n "<< c_momentum_hat_k1 << std::endl;
        // std::cout << " pos_CoM_hat_    " << 2 << " is :  \n "<< pos_CoM_hat_k2 << std::endl;
        // std::cout << " c_momentum_hat_ " << 2 << " is :  \n "<< c_momentum_hat_k2 << std::endl;

    // compute the centroidal rate using 3 points numerical derivation
    // ---------------------------------------------------------------
    c_momentum_hat_dot = 1./(2.*dt_) * (3.* c_momentum_hat_k + 4.*c_momentum_hat_k1 - c_momentum_hat_k2);
    //
    // CoM position, velocity and acceleration
    // =======================================
    Vector3d w_pCoM, w_CoM_dot, w_CoM_ddot;
    w_pCoM      =  W_H_aF_.block<3,3>(0,0) * pos_CoM_hat_k + W_H_aF_.block<3,1>(0,3);
    w_CoM_dot   =  W_H_aF_.block<3,3>(0,0) * c_momentum_hat_k.head(3)/a_ctrl.mass_bot;
    w_CoM_ddot  =  W_H_aF_.block<3,3>(0,0) * c_momentum_hat_dot.head(3)/a_ctrl.mass_bot;
    // Wrench map hands - CoM
    // =======================
    Matrix6d WrenchMap_lh = MatrixXd::Identity(6,6); 
    Matrix6d WrenchMap_rh = MatrixXd::Identity(6,6);
    WrenchMap_lh.block<3,3>(3,0) = a_ctrl.Transforms.ComputeSkewSymmetricMatrix(Pose_EE[0].head(3) - w_pCoM);
    WrenchMap_rh.block<3,3>(3,0) = a_ctrl.Transforms.ComputeSkewSymmetricMatrix(Pose_EE[1].head(3) - w_pCoM);
    //
    Eigen::Matrix<double, 6, 12>  WrenchMap_UP_;
    WrenchMap_UP_.leftCols(6)    = WrenchMap_lh;
    WrenchMap_UP_.rightCols(6)   = WrenchMap_rh;

    Vector6d Wrench_UB_  = WrenchMap_UP_ * Wrench_Hands_;                                          // Upper body wrench about the CoM 
    double Beta_w_       = sqrt(1. + w_CoM_ddot(2)/9.81 - Wrench_UB_(2)/(a_ctrl.mass_bot * 9.81));
    //

    // std::cout << " Wrench_UB_  \n "<< Wrench_UB_ << std::endl;
    // std::cout << " a_ctrl.mass_bot  \n "<< a_ctrl.mass_bot << std::endl;
    // std::cout << " Beta_w_  \n "<< Beta_w_ << std::endl;
    // std::cout << " w_pCoM  \n "<< w_pCoM << std::endl;
    // std::cout << " w_CoM_dot  \n "<< w_CoM_dot << std::endl;
    //
    Vector3d W_CP_       = a_ctrl.compute_capture_point2(Beta_w_, w_pCoM, w_CoM_dot);       // compute Capture point
    Vector2d Bal_Pertub_ = a_ctrl.compute_posture_perturbation2(Beta_w_, Wrench_UB_);                   // compute the expected postural Perturbation
    //
    // std::cout << " c_momentum_hat_dot  \n "<< c_momentum_hat_dot << std::endl;
    // std::cout << " W_CP_  \n "<< W_CP_ << std::endl;
    // std::cout << " Bal_Pertub_  \n "<< Bal_Pertub_ << std::endl;

    // // ====================================================================
    a_ctrl.CoM          = w_pCoM;
    a_ctrl.CoM_dot      = w_CoM_dot;
    a_ctrl.CoM_ddot     = w_CoM_ddot;
    a_ctrl.CM           = c_momentum_hat_k;
    a_ctrl.CM_dot       = c_momentum_hat_dot;
    a_ctrl.CP           = W_CP_;
    a_ctrl.Beta_w       = Beta_w_;
    a_ctrl.Delat_R_cmp  = Bal_Pertub_;
    a_ctrl.DeltaZ_com(0)= w_pCoM(2);
    a_ctrl.DeltaZ_com(1)= w_pCoM(2);
    a_ctrl.WrenchMap_UP = WrenchMap_UP_;

    return true;
}


bool FeedForwardController::get_RobotStabilityState(double dt_, Vector7d Pose_EE[], VectorXd Velo_Hands_k[], VectorXd Velo_Hands_k1[], VectorXd Wrench_Hands_, std::string option)
{
    //
    // ------------------------------------------------------------------------------------
    stable_prediction       = false;
    double DeltaStepLength  = 0; 
    double theta_Dist       = 0.0;        // direction of the balance disturbance
    double DeltaThetaFeet_  = 0.0;
    Vector2d DeltaDisturb   = VectorXd::Zero(2);
    Vector2d DeltaStep(DeltaStepLength * cos(theta_Dist), DeltaStepLength * sin(theta_Dist));   // step increase
    //
    MatrixXd AllPtsInAbsFoot;
    MatrixXd PointsCHull;
    VectorXd DistanceEdgesCHull;
    MatrixXd Abs_EdgesNormalCHull;
    MatrixXd W_EdgesNormalCHull;
    Matrix2d W_Rot_AbsFoot;
    Vector2d W_Pos_AbsFoot;
    Vector2d W_Disturb;
    //
    // -------------------------------------------------------------------------------------
    // get the required kinematic and dynamic variables 
    // ================================================
    this->Esimate_KinoDynamicStates(dt_, Pose_EE, Velo_Hands_k, Velo_Hands_k1, Wrench_Hands_);

    // computation of the support polygon and related variables (Normals and distances to edges)
    // ==========================================================================================
    a_ctrl.get_feet_support_points_C(Pose_EE[2], Pose_EE[3], DeltaStep, DeltaThetaFeet_, AllPtsInAbsFoot, W_Rot_AbsFoot, W_Pos_AbsFoot);        // get the points of the Convex hull 
    a_ctrl.ConvHull.convexHull(AllPtsInAbsFoot, PointsCHull, Abs_EdgesNormalCHull, DistanceEdgesCHull);                                         // update convex hull   
    W_EdgesNormalCHull = Abs_EdgesNormalCHull * W_Rot_AbsFoot.transpose();  
    //      
    // std::cout << " W_EdgesNormalCHull  \n "<< W_EdgesNormalCHull << std::endl;
    // std::cout << " a_ctrl.CoM  \n "<< a_ctrl.CoM << std::endl;
    // std::cout << " a_ctrl.CoM_ddot  \n "<< a_ctrl.CoM_ddot << std::endl;
    // std::cout << " a_ctrl.CP  \n "<< a_ctrl.CP << std::endl;

    // predict stability using ZMP or Capture point (CP)-based criteria
    // ================================================================
    if(option == "ZMP"){
        
        stable_prediction = a_ctrl.predict_stabilityZMP(W_EdgesNormalCHull, DistanceEdgesCHull, a_ctrl.CoM, a_ctrl.CoM_ddot, W_Pos_AbsFoot, a_ctrl.Delat_R_cmp, a_ctrl.Beta_w);
    }
    else{ // CP
        stable_prediction = a_ctrl.predict_stabilityCP(W_EdgesNormalCHull, DistanceEdgesCHull, a_ctrl.CP.head(2), W_Pos_AbsFoot, a_ctrl.Delat_R_cmp);
    }

    std::cout << " stable_prediction  \n "<< stable_prediction << std::endl;
    std::cout << " //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// : \n" << std::endl;
    //
    return true;
}
//
Vector2d FeedForwardController::getOptimalInitCapturePoint(Vector2d Disturb_c, double fu_z_mgbetaW, Vector2d W_Pos_AbsFoot, Vector2d W_Pos_AbsHand, MatrixXd W_EdgesNormalCHull, VectorXd DistanceEdgesCHull)
{
    // Initialization of QS solver
    Matrix2d U_b = MatrixXd::Zero(2,2);
             U_b << 1.+fu_z_mgbetaW,                   0.,
                    0.,                   1.+fu_z_mgbetaW;

    Matrix2d Q_b = U_b.transpose() * U_b + 0.0e-8 * MatrixXd::Identity(2,2);
    Vector2d P_b = U_b.transpose() * (Disturb_c - W_Pos_AbsFoot);

    MatrixXd Mcom = MatrixXd::Zero(W_EdgesNormalCHull.rows() + 2, 2);
    VectorXd Bcom = VectorXd::Zero(W_EdgesNormalCHull.rows() + 2);

    Mcom.topRows(W_EdgesNormalCHull.rows()) = W_EdgesNormalCHull;
    Mcom.bottomRows(2) = -MatrixXd::Identity(2,2);

    Bcom.head(W_EdgesNormalCHull.rows()) = DistanceEdgesCHull;
    Bcom.tail(2) = a_ctrl.W_Rot_AbsFoot_ * Vector2d(0.20, 0.10) - W_Pos_AbsHand;

    

    qpCp.setnbWorkingSetRecalculation(this->nWSR);
    // qpCp.qpOasesSolver( Q_b, P_b, W_EdgesNormalCHull, DistanceEdgesCHull);
    Vector2d sol = qpCp.qpOasesSolver( Q_b, P_b, Mcom, Bcom);
    //
    // std::cout << " XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX : \n" << std::endl;
    // std::cout << " Mcom  \n "<< Mcom << std::endl;
    // std::cout << " Bcom  \n "<< Bcom << std::endl;
    // std::cout << " CHECK CONSTRAINTS  \n "<< Mcom * sol - Bcom << std::endl;
    // std::cout << " CHECK DISTANCE  \n "<< a_ctrl.W_Rot_AbsFoot_.transpose()*(W_Pos_AbsHand - W_Pos_AbsFoot) -  sol << std::endl;
    // std::cout << " HANDS IN ABSOLUTE FOOT  \n "<< a_ctrl.W_Rot_AbsFoot_.transpose()*(W_Pos_AbsHand - W_Pos_AbsFoot) << std::endl;
    // std::cout << " //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// : \n" << std::endl;

    return sol;
}

//
bool FeedForwardController::is_Stabilizable(VectorXd Wrench_hands_star_, Vector3d d_pos_lh, Vector3d d_pos_rh, 
                                             Vector7d pose_lfoot, Vector7d pose_rfoot, Vector6d CM_,    //   CM_(2)  should be zCoM_ddot * mass_robot instead of its real value
                                             Vector2d DeltaStep, double DeltaThetaFeet_)
{
    //
    bool stable_0 = false;
    MatrixXd W_EdgesNormalCHull;
    VectorXd DistanceEdgesCHull;
    Vector2d W_Pos_AbsHand; 
    W_Pos_AbsHand = 0.5*(d_pos_lh.head(2) + d_pos_rh.head(2));

    // Vector2d W_Pos_AbsFoot;
    //--------------
    a_ctrl.getSupportConstraints(pose_lfoot, pose_rfoot, DeltaStep, DeltaThetaFeet_,  W_EdgesNormalCHull, DistanceEdgesCHull, a_ctrl.W_Pos_AbsFoot_);
    //
    this->getUniformConvexSupportSize(W_EdgesNormalCHull, DistanceEdgesCHull, a_ctrl.W_EdgesNormalCHull_, a_ctrl.DistanceEdgesCHull_);

    a_ctrl.compute_perturbation_terms(Wrench_hands_star_, d_pos_lh, d_pos_rh, CM_,  this->Disturb_c_, this->fu_z_mgbetaW_);
    //
    this->CP_star_ = getOptimalInitCapturePoint(this->Disturb_c_, this->fu_z_mgbetaW_, a_ctrl.W_Pos_AbsFoot_, W_Pos_AbsHand, a_ctrl.W_EdgesNormalCHull_, a_ctrl.DistanceEdgesCHull_);

    a_ctrl.Delat_R_cmp = -(this->Disturb_c_ + this->fu_z_mgbetaW_ * this->CP_star_);

    stable_0 = a_ctrl.predict_stability(a_ctrl.W_EdgesNormalCHull_, a_ctrl.DistanceEdgesCHull_, this->CP_star_, a_ctrl.W_Pos_AbsFoot_, a_ctrl.Delat_R_cmp);


    // std::cout << " cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc \t" <<  std::endl;
    // std::cout << " Correlation Normals   IS : \n" << W_EdgesNormalCHull * W_EdgesNormalCHull.transpose() << std::endl;
    // std::cout << " \n" <<  std::endl;
    // std::cout << " CP_star   IS : \t" << CP_star_.transpose() << std::endl;
    // std::cout << " \n" <<  std::endl;
    // std::cout << " cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc \t" <<  std::endl;

    //--------------
    return stable_0;
}

bool FeedForwardController::predict_Stabilizability(VectorXd Wrench_hands_star_, Vector3d d_pos_lh, Vector3d d_pos_rh, 
                                                     Vector7d pose_lfoot, Vector7d pose_rfoot, Vector6d CM_)
{
    return  this->is_Stabilizable(Wrench_hands_star_, d_pos_lh,  d_pos_rh, pose_lfoot,  pose_rfoot, CM_, Vector2d(0., 0.), 0.0);
}

bool FeedForwardController::getUniformConvexSupportSize(MatrixXd W_EdgesNormalCHull_0, VectorXd DistanceEdgesCHull_0, MatrixXd &W_EdgesNormalCHull, VectorXd &DistanceEdgesCHull)
{
    //
    W_EdgesNormalCHull.resize(7,2);  W_EdgesNormalCHull = MatrixXd::Zero(7,2);
    DistanceEdgesCHull.resize(7);    DistanceEdgesCHull = VectorXd::Zero(7);
    //
    if(W_EdgesNormalCHull_0.rows() == 4)
    {
        W_EdgesNormalCHull.block(0, 0, 4, 2) = W_EdgesNormalCHull_0;
        DistanceEdgesCHull.segment(0, 4)     = DistanceEdgesCHull_0;
        // for(int i=4; i<7; i++){
        //     W_EdgesNormalCHull.block(i, 0, 1, 2) = W_EdgesNormalCHull_0.block(0, 0, 1, 2);
        //     DistanceEdgesCHull.segment(i, 1)     = DistanceEdgesCHull_0.segment(0, 1);
        // }       
    }
    else if (W_EdgesNormalCHull_0.rows() == 5)
    {
        W_EdgesNormalCHull.block(0, 0, 5, 2) = W_EdgesNormalCHull_0;
        DistanceEdgesCHull.segment(0, 5)     = DistanceEdgesCHull_0;
        // for(int i=5; i<7; i++){
        //     W_EdgesNormalCHull.block(i, 0, 1, 2) = W_EdgesNormalCHull_0.block(0, 0, 1, 2);
        //     DistanceEdgesCHull.segment(i, 1)     = DistanceEdgesCHull_0.segment(0, 1);
        // }    
    }
    else if (W_EdgesNormalCHull_0.rows() == 6)
    {
        W_EdgesNormalCHull.block(0, 0, 6, 2) = W_EdgesNormalCHull_0;
        DistanceEdgesCHull.segment(0, 6)     = DistanceEdgesCHull_0;
        // for(int i=6; i<7; i++){
        //     W_EdgesNormalCHull.block(i, 0, 1, 2) = W_EdgesNormalCHull_0.block(0, 0, 1, 2);
        //     DistanceEdgesCHull.segment(i, 1)     = DistanceEdgesCHull_0.segment(0, 1);
        // }     
    }
    else
    {
        W_EdgesNormalCHull  = W_EdgesNormalCHull_0;
        DistanceEdgesCHull  = DistanceEdgesCHull_0;
    }

    // W_EdgesNormalCHull.block(1, 0, 1, 2) = MatrixXd::Zero(1,2);
    // DistanceEdgesCHull(1)     = 0.0;

    return true;
}

bool FeedForwardController::compute_anticipative_step2( VectorXd Wrench_hands_star_, 
                                                                Vector3d d_pos_lh, Vector3d d_pos_rh,  
                                                                Vector7d pose_lfoot, Vector7d pose_rfoot, Vector6d CM_)
{
//
    int count = 0;
    bool stable_0 = false;
    bool stable_1 = false;
    DeltaFootstepPose.setZero();
    // 
    double DeltaStepLength = 0; 
    // direction of the balance disturbance
    double theta_Dist = 0.0;
    double dTheta_st, dTheta_sw, dDThetaFeet;
    // Delta steps
    Vector2d DeltaStep(DeltaStepLength * cos(theta_Dist), DeltaStepLength * sin(theta_Dist));   // step increase
        double DeltaThetaFeet_ = 0.0;
    //
    stable_0 = this->is_Stabilizable(Wrench_hands_star_, d_pos_lh, d_pos_rh, pose_lfoot, pose_rfoot, CM_, DeltaStep, DeltaThetaFeet_);
    //
    a_ctrl.cost_step = a_ctrl.getSteppingCost(this->CP_star_, DeltaStep, DeltaThetaFeet_);
    //
    // =======================================================================================================
    // extraction of Euler angles of  the feet rotation 
    Vector3d o_lf = a_ctrl.Transforms.getEulerAnglesXYZ_FixedFrame(pose_lfoot);
    Vector3d o_rf = a_ctrl.Transforms.getEulerAnglesXYZ_FixedFrame(pose_rfoot);
    // Step variables range
    VectorXd DeltaStep_r        = VectorXd::LinSpaced(20,low_Sr,high_Sr);
    VectorXd DeltaStep_thetaF   = VectorXd::LinSpaced(20,low_TF,high_TF);
    VectorXd DeltaStep_theta_l  = VectorXd::LinSpaced(30,low_dl,high_dl);
    VectorXd DeltaStep_theta_r  = VectorXd::LinSpaced(30,low_dr,high_dr);

    int iter_xy = 0;
    int iter_o  = 0;
    bool DStepFound = false;
    bool SolverDone = false;
    double Jdxdydo_0,  Jdxdydo;         Jdxdydo_0 = Jdxdydo = 0.0;
    double DevTheta_0, DevTheta;        DevTheta_0= DevTheta= 0.0;
    double desDeltaThetaFeet_ = 0.0;
    int sol_index = 1;

    double t_solution = yarp::os::Time::now();
    // =======================================================================================================
    while((!SolverDone) && (count <= (DeltaStep_r.rows() + DeltaStep_thetaF.rows()) ))
    {
        // update step
        //-------------
        stable_0 = this->is_Stabilizable(Wrench_hands_star_, d_pos_lh, d_pos_rh, pose_lfoot, pose_rfoot, CM_, DeltaStep, DeltaThetaFeet_);
        //
        if(stable_0 && !DStepFound)
        {
            Jdxdydo_0 = a_ctrl.getSteppingCost(this->CP_star_, DeltaStep, DeltaThetaFeet_);
            sol_index = count + 1;
        }
        // update the balance perturbation
        // --------------------------------
        if(stable_0) DStepFound = true;

        if(!stable_0 && !DStepFound && !DStepMaxReached)
        {
            if  (a_ctrl.stance_0 =="left")  dTheta_st = o_lf(2);    // left stance foot for anticipatory step
            else                            dTheta_st = o_rf(2);    // right stance foot for anticipatory step          
            DeltaStep <<    DeltaStep_r(iter_xy) * cos(dTheta_st + DeltaThetaFeet_), 
                            DeltaStep_r(iter_xy) * sin(dTheta_st + DeltaThetaFeet_);
            //
            theta_Dist = atan2( a_ctrl.Delat_R_cmp(1),  a_ctrl.Delat_R_cmp(0));
            DeltaThetaFeet_ = theta_Dist - dTheta_st;
            if     (DeltaThetaFeet_ <= -a_ctrl.Dtheta_feet_max) DeltaThetaFeet_ = -a_ctrl.Dtheta_feet_max;
            else if(DeltaThetaFeet_ >=  a_ctrl.Dtheta_feet_max) DeltaThetaFeet_ =  a_ctrl.Dtheta_feet_max;
            //
            Jdxdydo_0 = a_ctrl.getSteppingCost(this->CP_star_, DeltaStep, DeltaThetaFeet_);
            //
            iter_xy ++;
            if(iter_xy >= DeltaStep_r.rows()) 
            {
                iter_xy = DeltaStep_r.rows()-1;   // CONDITION TO PERFORM TWO FEET STEPS
                DStepMaxReached = true;                // set to true to move to the orientation loop
            }
        }
        
        if(DStepFound && !DStepMaxReached)    // orientation 
        {
            
            if (stable_0 && DeltaStep(0) == 0.0 && DeltaStep(1) ==0.0)
            {
                SolverDone = true;
                DeltaFootstepPose.head(2) = DeltaStep;
                DeltaFootstepPose(2)      = desDeltaThetaFeet_; 
            }
            else if (stable_0 && DeltaStep(0) !=0 && DeltaStep(1) !=0)
            {
                // computation of the cost function
                Jdxdydo = a_ctrl.getSteppingCost(this->CP_star_, DeltaStep, DeltaThetaFeet_);
                // computation deviation wrt the disturbance
                DevTheta = fabs(theta_Dist - (dTheta_st+DeltaThetaFeet_));
                // selection of DeltaThetaFeet corresponding to min Jxyo
                if(Jdxdydo < Jdxdydo_0)
                {
                    desDeltaThetaFeet_ = DeltaThetaFeet_;
                    Jdxdydo_0  = Jdxdydo;
                    DevTheta_0 = DevTheta;
                    sol_index  = count + 1;

                } else if(Jdxdydo == Jdxdydo_0)
                {
                    // choose DThetaFeet with mimimum deviation wrt the disturbance
                    if(DevTheta < DevTheta_0)
                    {
                        desDeltaThetaFeet_ = DeltaThetaFeet_;
                        Jdxdydo_0  = Jdxdydo;
                        DevTheta_0 = DevTheta;
                        sol_index  = count + 1;
                    }
                }
            }
            //
            DeltaStep       = DeltaStep;
            DeltaThetaFeet_ = DeltaStep_thetaF(iter_o);                 // update theta
            // 
            iter_o ++; 
            if(iter_o >= DeltaStep_thetaF.rows()) 
            {
                SolverDone = true;
                DeltaFootstepPose.head(2) = DeltaStep;
                DeltaFootstepPose(2)      = desDeltaThetaFeet_;
            }
        }
        //
        if(!DStepFound && DStepMaxReached)
        {
            // computation of the cost function
            Jdxdydo = a_ctrl.getSteppingCost(this->CP_star_, DeltaStep, DeltaThetaFeet_);
            // computation deviation wrt the disturbance
            DevTheta = fabs(theta_Dist - (dTheta_st+DeltaThetaFeet_));
            // selection of DeltaThetaFeet corresponding to min Jxyo
            if(Jdxdydo < Jdxdydo_0)
            {
                desDeltaThetaFeet_ = DeltaThetaFeet_;
                Jdxdydo_0  = Jdxdydo;
                DevTheta_0 = DevTheta;
                sol_index  = count + 1;

            } else if(Jdxdydo == Jdxdydo_0)
            {
                // choose DThetaFeet with mimimum deviation wrt the disturbance
                if(DevTheta < DevTheta_0)
                {
                    desDeltaThetaFeet_ = DeltaThetaFeet_;
                    Jdxdydo_0  = Jdxdydo;
                    DevTheta_0 = DevTheta;
                    sol_index  = count + 1;
                }
            }
            //
            DeltaStep       = DeltaStep;
            DeltaThetaFeet_ = DeltaStep_thetaF(iter_o);                 // update theta
            // 
            iter_o ++; 
            if(iter_o >= DeltaStep_thetaF.rows()) 
                {
                    SolverDone = true;
                    DeltaFootstepPose.head(2) = DeltaStep;
                    DeltaFootstepPose(2)      = desDeltaThetaFeet_;
                    DeltaFootstepPose.tail(3) = DeltaFootstepPose.head(3);
                }

        }
        // else {
        //     printf("Could not compute an anticipative step \n");
        //     return true;
        // }
        
        count++;
    }

    //
    std::cout<< "DeltaFootstepPose : is \t" << DeltaFootstepPose << std::endl;
    std::cout<< "MAIN COMPUTATION AT : " << count << " GOT IN : " <<  yarp::os::Time::now()-t_solution << " s" << std::endl;
    // std::cout<< "RECORDING TIME AT : " << count << " GOT IN : " <<  yarp::os::Time::now()-t_solution << " s" << std::endl;
    // std::cout<< "FFWD compute_stepping_pose OK   : \t" << 0.0 << std::endl;
    
    return DStepFound;
}

std::string FeedForwardController::get_stance_foot(Vector7d pose_lfoot, Vector7d pose_rfoot, Vector2d BalPert_)
{
    //
    std::string stance_ft_;
    // 3D rotation matrix of the feet wrt the world frame
    Eigen::Matrix3d WR_lf = a_ctrl.Transforms.PoseVector2HomogenousMx(pose_lfoot).block(0,0,3,3);
    Eigen::Matrix3d WR_rf = a_ctrl.Transforms.PoseVector2HomogenousMx(pose_rfoot).block(0,0,3,3);
    // Planar orientations of the feet 
    Vector3d o_lf = a_ctrl.Transforms.getEulerAnglesXYZ_FixedFrame(WR_lf);
    Vector3d o_rf = a_ctrl.Transforms.getEulerAnglesXYZ_FixedFrame(WR_rf);
    //
    Matrix2d    rot_robot;
                rot_robot << cos(0.5*(o_lf(2)+o_rf(2))), -sin(0.5*(o_lf(2)+o_rf(2))), 
                             sin(0.5*(o_lf(2)+o_rf(2))),  cos(0.5*(o_lf(2)+o_rf(2)));
    // Perturbation expressed relative to the robot frame to help choose the stance foot
    Vector2d r_D_rcmp = rot_robot.transpose() * BalPert_;

    // determine the stance based on the direction of the balance perturbation
    // string stance;
    //
    if     ((r_D_rcmp(0)>=0) && (r_D_rcmp(1)>=0))     stance_ft = "right";    //
    else if((r_D_rcmp(0)>=0) && (r_D_rcmp(1)<0))      stance_ft = "left";
    else if((r_D_rcmp(0)<0)  && (r_D_rcmp(1)>=0))     stance_ft = "right";
    else if((r_D_rcmp(0)<0)  && (r_D_rcmp(1)<0))      stance_ft = "left";
    else                                              stance_ft = "right";

    return stance_ft;
}

Vector3d FeedForwardController::get_DeltaStep(Vector7d pose_lfoot, Vector7d pose_rfoot, std::string stance_ft)
{
    //
    Vector3d DeltaStep =VectorXd::Zero(3);
    Matrix2d w_Robot_stance = MatrixXd::Identity(2,2);
    // 3D rotation matrix of the feet wrt the world frame
    Eigen::Matrix3d WR_lf = a_ctrl.Transforms.PoseVector2HomogenousMx(pose_lfoot).block(0,0,3,3);
    Eigen::Matrix3d WR_rf = a_ctrl.Transforms.PoseVector2HomogenousMx(pose_rfoot).block(0,0,3,3);
    // Planar orientations of the feet 
    Vector3d o_lf = a_ctrl.Transforms.getEulerAnglesXYZ_FixedFrame(WR_lf);
    Vector3d o_rf = a_ctrl.Transforms.getEulerAnglesXYZ_FixedFrame(WR_rf);   

    if(stance_ft == "left")
    {
        w_Robot_stance << cos(o_lf(2)), -sin(o_lf(2)), sin(o_lf(2)), cos(o_lf(2));
        DeltaStep.head(2) = w_Robot_stance.transpose()*(pose_rfoot.head(2) - pose_lfoot.head(2)) + Vector2d(0., a_ctrl.DyFeet_min);
        DeltaStep(2)      = o_rf(2) - o_lf(2);
    }
    else
    {   
        w_Robot_stance << cos(o_rf(2)), -sin(o_rf(2)), sin(o_rf(2)), cos(o_rf(2));
        DeltaStep.head(2) = w_Robot_stance.transpose()*(pose_lfoot.head(2) - pose_rfoot.head(2)) - Vector2d(0., a_ctrl.DyFeet_min);
        DeltaStep(2)      = o_lf(2) - o_rf(2);
    }
    //
    return DeltaStep;
}


// bool FeedForwardController::check_reachability(WbRobotModel& robot_model_, VectorXd jts_position, Matrix4d WHB, Vector7d des_X_EE_[], string stanceFoot, Vector7d pose_lfoot, Vector7d pose_rfoot)
// {
//     bool ok = false;
//     ok = WBIK.check_reachability(robot_model_, jts_position, WHB, des_X_EE_,  stanceFoot,  pose_lfoot,  pose_rfoot);

//     Reach_Done = true;

//     return ok;
// }

bool FeedForwardController::check_reachability(WbRobotModel& robot_model_, VectorXd jts_position, Matrix4d WHB, Vector7d des_X_EE_[], string stanceFoot, Vector7d pose_lfoot, Vector7d pose_rfoot)
{
    bool ok = false;
    bool e_lh, e_rh;

    // load the reachability parameters
    // task_weight[] = task_weight_reach[];
    memcpy(WBIK.task_weight, &WBIK.task_weight_reach[0], 8 * sizeof *WBIK.task_weight_reach);
    //  
    WBIK.isCoMClamping = true;
    //
    Vector2d w_aH_pos = 0.5*(des_X_EE_[0].head(2) + des_X_EE_[1].head(2));
    Vector2d w_aF_pos = 0.5*(des_X_EE_[2].head(2) + des_X_EE_[3].head(2));

    // ===========================================================================================================
    // WBIK.virt_jts_pos = jts_position;
    WBIK.virt_jts_pos = WBIK.q_0;
    WBIK.q_dot_star.setZero();
    WBIK.var_gain.setZero();
    // virt_WHB  = WHB;
    for(int i=0; i<8; i++) WBIK.des_X_EE[i] = des_X_EE_[i];
    //
    int count = 0;
    bool cnt, ik_run;
         cnt = ik_run = false;
    // loop over the specified number of iteration if the error is still large
    double t_wbIK = yarp::os::Time::now();

    if((w_aH_pos-w_aF_pos).norm() <= WBIK.max_reach)
    {
        // this->get_wb_IK(robot_model_, jts_position, WHB, WBIK.des_X_EE_, stanceFoot, pose_lfoot, pose_rfoot);
        // ///////////////////////////////////////////////////////////////////////////////////////////////////
        while (!ik_run)
        {
            MatrixXd W_EdgesNormalCHull_; 
            VectorXd DistanceEdgesCHull_;
            WBIK.solve_QPcvxgen(robot_model_, pose_lfoot, pose_rfoot, stanceFoot, W_EdgesNormalCHull_, DistanceEdgesCHull_);     // TODO PLOT EVOLUTION OF COM AND CLAMPI IT
            // clamping the CoM position to move freely but within the convex hull
            // --------------------------------------------------------------------
            bool SStable = WBIK.predict_stability(W_EdgesNormalCHull_, DistanceEdgesCHull_, WBIK.Pose_EE[4].head(2), WBIK.W_Pos_AbsFoot);
            if(SStable)
            {
                // des_X_EE[4].head(3) = Pose_EE[4].head(3);
                WBIK.pxy_CoM_n1 = WBIK.pxy_CoM;
                WBIK.pxy_CoM    = (WBIK.Pose_EE[4].head(2) - WBIK.W_Pos_AbsFoot);
            }
            else
            {
                std::cout<< "\n" << std::endl;
                std::cout<< "COM CLAMPING ACTIVE with : " << WBIK.pxy_CoM_n1.transpose() << std::endl;

                Vector2d DeltaCoMxy = (WBIK.pxy_CoM_n1.norm() - 0.03)/(WBIK.pxy_CoM_n1.norm()+1e-10) * WBIK.pxy_CoM_n1;
                WBIK.des_X_EE[4].head(2) = DeltaCoMxy + WBIK.W_Pos_AbsFoot;
                WBIK.des_X_EE[4](2)      = WBIK.Pose_EE[4](2);
            }

            cnt  = (count >= WBIK.count_max);
            ik_run = cnt || WBIK.getTaskCompletionState();
            //
            std::cout<< "WBIK COUNTER IS : " << count << std::endl;
            std::cout<< "\n : " << std::endl;
            // 
            // ////////////////// DATA LOGGING /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // =================================================================================================================================================================
                int stance_left = 0; 
                if(stanceFoot =="left") stance_left = 1;

                log_reach.Out_joints_pos << count << "  " << 180./M_PI * WBIK.virt_jts_pos.transpose() << std::endl;
                log_reach.Out_joints_vel << count << "  " << WBIK.q_dot_star.transpose() << std::endl; 
                //
                log_reach.Out_stability_var << count << "  " << Vector2d(0.0, 0.0).transpose() << "  " << WBIK.Pose_EE[4].head(3).transpose() << "  " << WBIK.W_Pos_AbsFoot.transpose() <<  "  ";  // count Perturbation CoM absFoot_in_W
                log_reach.Out_stability_var << WBIK.W_Rot_AbsFoot.row(0) << "  " << WBIK.W_Rot_AbsFoot.row(1) <<  "  ";
                log_reach.Out_stability_var << Vector3d(0.0, 0.0, 0.0).transpose() << "  " << SStable << "  " << stance_left << "  " << endl; // delta step  static_stability 

                if(WBIK.PointsCHull.rows() <= 6)
                {
                    VectorXd first_pt_x = WBIK.PointsCHull(0,0) * VectorXd::Ones(7-WBIK.PointsCHull.rows());
                    VectorXd first_pt_y = WBIK.PointsCHull(0,1) * VectorXd::Ones(7-WBIK.PointsCHull.rows());
                    //
                    log_reach.Out_PointsCHull   << WBIK.PointsCHull.col(0).transpose() << "  " << first_pt_x.transpose() << "  ";   // x
                    log_reach.Out_PointsCHull   << WBIK.PointsCHull.col(1).transpose() << "  " << first_pt_y.transpose() << endl;   // y   
                }
                else
                {
                    log_reach.Out_PointsCHull   << WBIK.PointsCHull.col(0).transpose() << "  ";     // x
                    log_reach.Out_PointsCHull   << WBIK.PointsCHull.col(1).transpose() << endl;     // y
                }
            // =====================================================================================================================================================================
            // ////////////////// DATA LOGGING /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //
            count++;
        }
        // ////////////////
        //
        WBIK.des_jts_pos  = WBIK.virt_jts_pos;
        WBIK.des_WHB      = WBIK.virt_WHB;

        // ///////////////////////////////////////////////////////////////////////////////////////////////////
        WBIK.get_hands_errors();
        // check of the hands errors
        e_lh = ((WBIK.error_pos_norm[0] < WBIK.reach_tol(0)) && (WBIK.error_ori_norm[0] < 10.*WBIK.reach_tol(1)));      // error lhand
        e_rh = ((WBIK.error_pos_norm[1] < WBIK.reach_tol(0)) && (WBIK.error_ori_norm[1] < 10.*WBIK.reach_tol(1)));      // error rhand

        if(e_lh && e_rh)
            ok = true;
        else
            ok = false;
    }

    // set back the weights to their default values
    memcpy(WBIK.task_weight, &WBIK.task_weight_default[0], 8*sizeof *WBIK.task_weight_default);

    return ok;
}


// Vector3d FeedForwardController::get_FMMCoM(WbRobotModel& robot_model_, VectorXd jts_pos, VectorXd Wrench_hands_star_, 
//                                             Vector7d des_X_EE_[], Vector7d pose_lfoot, Vector7d pose_rfoot, string stanceFoot)
// {
//     // estimate the components of the perturbation
//     a_ctrl.compute_perturbation_terms2(Wrench_hands_star_, des_X_EE_[0].head(3), des_X_EE_[1].head(3), Vector3d(0.,0.,0.), Vector3d(0.,0.,0.), this->Disturb_c_, this->fu_z_mgbetaW_);

//     // load the FMMCOM parameters
//     memcpy(WBIK.task_weight, &WBIK.task_weight_fmmcom[0], 8*sizeof *WBIK.task_weight_fmmcom);
//     // memcpy(WBIK.task_weight, &WBIK.task_weight_reach[0], 8*sizeof *WBIK.task_weight_reach);
//     WBIK.isCoMClamping = true;
//     // compute the CoM
//     Vector3d fmmCoM_ = this->WBIK.get_wb_fmmcom(robot_model_, jts_pos, des_X_EE_, stanceFoot, pose_lfoot, pose_rfoot, this->Disturb_c_, this->fu_z_mgbetaW_);

//     fmmCoM_Done   = true;  

//     return fmmCoM_;
// }


Vector3d FeedForwardController::get_FMMCoM(WbRobotModel& robot_model_, VectorXd jts_pos, VectorXd Wrench_hands_star_, 
                                            Vector7d des_X_EE_[], Vector7d pose_lfoot, Vector7d pose_rfoot, string stanceFoot)
{
    // estimate the components of the perturbation
    a_ctrl.compute_perturbation_terms2(Wrench_hands_star_, des_X_EE_[0].head(3), des_X_EE_[1].head(3), Vector3d(0.,0.,0.), Vector3d(0.,0.,0.), this->Disturb_c_, this->fu_z_mgbetaW_);

    // load the FMMCOM parameters
    memcpy(WBIK.task_weight, &WBIK.task_weight_fmmcom[0], 8*sizeof *WBIK.task_weight_fmmcom);
    // memcpy(WBIK.task_weight, &WBIK.task_weight_reach[0], 8*sizeof *WBIK.task_weight_reach);
    WBIK.isCoMClamping = true;
    // compute the CoM
    // ////////////////////////////////////////////////////////////////////////////////////
    // Vector3d fmmCoM_ = this->WBIK.get_wb_fmmcom(robot_model_, jts_pos, des_X_EE_, stanceFoot, pose_lfoot, pose_rfoot, this->Disturb_c_, this->fu_z_mgbetaW_);
    //
    // WBIK.virt_jts_pos = jts_pos;
    WBIK.virt_jts_pos = WBIK.q_0;
    WBIK.q_dot_star.setZero();
    WBIK.var_gain.setZero();
    // virt_WHB  = WHB;
    for(int i=0; i<8; i++) WBIK.des_X_EE[i] = des_X_EE_[i];
    //
    int count = 0;
    bool cnt,  ik_run;
         cnt = ik_run = false;
    // loop over the specified number of iteration if the error is still large
    double t_wbIK = yarp::os::Time::now();

    while (!ik_run)
    {
        MatrixXd W_EdgesNormalCHull_; 
        VectorXd DistanceEdgesCHull_;

        WBIK.solve_QPcvxgen(robot_model_, pose_lfoot, pose_rfoot, stanceFoot, W_EdgesNormalCHull_, DistanceEdgesCHull_);     // TODO PLOT EVOLUTION OF COM AND CLAMPI IT
        // specify the reference value of the CoM
        Vector2d Bal_Pertubation = Disturb_c_ + fu_z_mgbetaW_ * WBIK.Pose_EE[4].head(2);
        WBIK.des_X_EE[4].head(2) = -(Bal_Pertubation - 0.5*(WBIK.Pose_EE[2].head(2) + WBIK.Pose_EE[3].head(2)));
        // clamping the CoM position to move freely but within the convex hull
        bool SStable = WBIK.predict_stability(W_EdgesNormalCHull_, DistanceEdgesCHull_, WBIK.Pose_EE[4].head(2), WBIK.W_Pos_AbsFoot);
        std::cout<< " SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS      STATIC STABILITY : " << SStable << std::endl;
        if(SStable)
        {
            WBIK.des_X_EE[4](2) = WBIK.Pose_EE[4](2);
            WBIK.pxy_CoM_n1     = WBIK.pxy_CoM;
            WBIK.pxy_CoM        = (WBIK.Pose_EE[4].head(2) - WBIK.W_Pos_AbsFoot);
        }
        else
        {
            std::cout<< "\n" << std::endl;
            std::cout<< "COM CLAMPING ACTIVE with : " << WBIK.pxy_CoM_n1.transpose() << std::endl;

            Vector2d DeltaCoMxy = (WBIK.pxy_CoM_n1.norm() - 0.03)/(WBIK.pxy_CoM_n1.norm()+1e-10) * WBIK.pxy_CoM_n1;
            WBIK.des_X_EE[4].head(2) = DeltaCoMxy + WBIK.W_Pos_AbsFoot;
            WBIK.des_X_EE[4](2)      = WBIK.Pose_EE[4](2);
        }
        //
        cnt    = (count >= WBIK.count_max);
        ik_run = cnt || WBIK.getTaskCompletionState();
        //
        std::cout<< "WBIK COUNTER IS : " << count << std::endl;
        std::cout<< "\n : " << std::endl;
        //
        // ////////////////// DATA LOGGING /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // =================================================================================================================================================================
        int stance_left = 0; 
        if(stanceFoot =="left") stance_left = 1;

        log_fmmcom.Out_joints_pos << count << "  " << 180./M_PI * WBIK.virt_jts_pos.transpose() << std::endl;
        log_fmmcom.Out_joints_vel << count << "  " << WBIK.q_dot_star.transpose() << std::endl; 
        //
        log_fmmcom.Out_stability_var << count << "  " << Bal_Pertubation.transpose() << "  " << WBIK.Pose_EE[4].head(3).transpose() << "  " << WBIK.W_Pos_AbsFoot.transpose() <<  "  ";  // count Perturbation CoM absFoot_in_W
        log_fmmcom.Out_stability_var << WBIK.W_Rot_AbsFoot.row(0) << "  " << WBIK.W_Rot_AbsFoot.row(1) <<  "  ";
        log_fmmcom.Out_stability_var << Vector3d(0.0, 0.0, 0.0).transpose() << "  " << SStable << "  " << stance_left << "  " << endl; // delta step  static_stability 
        // convex hull
        if(WBIK.PointsCHull.rows() <= 6)
        {
            VectorXd first_pt_x = WBIK.PointsCHull(0,0) * VectorXd::Ones(7-WBIK.PointsCHull.rows());
            VectorXd first_pt_y = WBIK.PointsCHull(0,1) * VectorXd::Ones(7-WBIK.PointsCHull.rows());
            //
            log_fmmcom.Out_PointsCHull   << WBIK.PointsCHull.col(0).transpose() << "  " << first_pt_x.transpose() << "  ";   // x
            log_fmmcom.Out_PointsCHull   << WBIK.PointsCHull.col(1).transpose() << "  " << first_pt_y.transpose() << endl;   // y   
        }
        else
        {
            log_fmmcom.Out_PointsCHull   << WBIK.PointsCHull.col(0).transpose() << "  ";     // x
            log_fmmcom.Out_PointsCHull   << WBIK.PointsCHull.col(1).transpose() << endl;     // y
        }
        // =====================================================================================================================================================================
        // ////////////////// DATA LOGGING /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


        count++;
    }
    //
    WBIK.des_jts_pos  = WBIK.virt_jts_pos;
    WBIK.des_WHB      = WBIK.virt_WHB;
    // ////////////////////////////////////////////////////////////////////////////////////
    //
    for (int i=0; i<8; i++)
        std::cout << " ERROR POS and ORI NORM " << WBIK.EE[i] << "  : \t" << WBIK.error_pos_norm[i] << "  and : " << WBIK.error_ori_norm[i] << std::endl;

    //
    std::cout<< "WBIK SOLUTION LOOP RUN IN : " <<  yarp::os::Time::now()-t_wbIK << " s" << std::endl;
    
    // cout << " Desired Joint position are : \n" << 180./M_PI * des_jts_pos.transpose() << endl;
    cout << " Desired Joint position Torso are : \t" << 180./M_PI * WBIK.des_jts_pos.head(3).transpose() << endl;
    cout << " Desired Joint position Lhand are : \t" << 180./M_PI * WBIK.des_jts_pos.head(10).tail(7).transpose() << endl;
    cout << " Desired Joint position Rhand are : \t" << 180./M_PI * WBIK.des_jts_pos.head(17).tail(7).transpose() << endl;
    cout << " Desired Joint position Lleg  are : \t" << 180./M_PI * WBIK.des_jts_pos.tail(12).head(6).transpose() << endl;
    cout << " Desired Joint position Rleg  are : \t" << 180./M_PI * WBIK.des_jts_pos.tail(6).transpose() << endl;
    cout << " Desired Base  transformation  is : \n" << WBIK.des_WHB << endl;


    fmmCoM_Done   = true;  

    return WBIK.Pose_EE[4].head(3);
}

bool FeedForwardController::check_stabilizability(VectorXd Wrench_hands_star_, Vector3d d_pos_lh, Vector3d d_pos_rh, Vector7d pose_lfoot, Vector7d pose_rfoot, 
                                                    Vector3d CoM_, Vector3d CoM_ddot, Vector3d AM_)
{
    //
    bool stable_0 = false;
    MatrixXd W_EdgesNormals;
    VectorXd Distances2Edges;
    //
    a_ctrl.getSupportConstraints(pose_lfoot, pose_rfoot, Vector2d(0., 0.), 0.0,  W_EdgesNormals, Distances2Edges, a_ctrl.W_Pos_AbsFoot_);
    //
    a_ctrl.compute_perturbation_terms2(Wrench_hands_star_, d_pos_lh, d_pos_rh, AM_, CoM_ddot, this->Disturb_c_, this->fu_z_mgbetaW_);
    //
    a_ctrl.Delat_R_cmp = -(this->Disturb_c_ + this->fu_z_mgbetaW_ * CoM_.head(2));

    cout << " \n" << endl;
    cout << " COMPUTED DISTURBANCE \t" << a_ctrl.Delat_R_cmp.transpose() << endl;
    cout << " \n" << endl;

    stable_0 = a_ctrl.predict_stability(    a_ctrl.W_EdgesNormalCHull_, a_ctrl.DistanceEdgesCHull_, CoM_.head(2), a_ctrl.W_Pos_AbsFoot_, a_ctrl.Delat_R_cmp);
    //
    return stable_0;
}

VectorXd FeedForwardController::estimate_min_anticip_step(  WbRobotModel& robot_model_, VectorXd jts_pos, VectorXd Wrench_hands_star_, 
                                                            Vector7d  des_X_EE_[], Vector7d pose_lfoot, Vector7d pose_rfoot)
{
    //
    std::cout<< " //////////////////////////////////////////////////////////////////////////////////////////////////   : \t" << 0.0 << std::endl;
    std::cout<< " STARTING estimate_min_anticip_step   : \t" << 0.0 << std::endl;
    int count = 0;
    
    WBIK.isCoMClamping = true;

    // estimate the components of the perturbation
    a_ctrl.compute_perturbation_terms2(Wrench_hands_star_, des_X_EE_[0].head(3), des_X_EE_[1].head(3), Vector3d(0.,0.,0.), Vector3d(0.,0.,0.), this->Disturb_c_, this->fu_z_mgbetaW_);
    Vector2d Bal_Pertubation = Disturb_c_ + fu_z_mgbetaW_*0.5*(pose_lfoot.head(2) + pose_rfoot.head(2));
    Vector2d BalPert_      = Bal_Pertubation - 0.5*(pose_lfoot.head(2) + pose_rfoot.head(2));
    std::string stanceFoot = get_stance_foot(pose_lfoot, pose_rfoot, BalPert_);

    // std::string stanceFoot = get_stance_foot(pose_lfoot, pose_rfoot, a_ctrl.Delat_R_cmp);
    //
    WBIK.update_stepping_weight(stanceFoot);
    // load the stepping solver parameters
    memcpy(WBIK.task_weight, &WBIK.task_weight_aStep[0], 8*sizeof*WBIK.task_weight_aStep); //
    // memcpy(WBIK.task_weight, &WBIK.task_weight_fmmcom[0], 8*sizeof *WBIK.task_weight_fmmcom);
    //
    Matrix4d WHB = MatrixXd::Identity(4,4);
    //
    for(int i=0; i<8; i++) WBIK.des_X_EE[i] = des_X_EE_[i];
    //
    bool stbz = false;
    bool cnt  = false;
    // estimate the balance perturbation 
    bool capturb = this->check_stabilizability(Wrench_hands_star_, WBIK.des_X_EE[0].head(3), WBIK.des_X_EE[1].head(3), 
                                                pose_lfoot, pose_rfoot, WBIK.des_X_EE[4].head(3), Vector3d(0.,0.,0.), Vector3d(0.,0.,0.));

    std::cout<< "CAPTURABILITY BEFORE   : \t" << capturb << std::endl;

    Eigen::Matrix3d WR_lf = a_ctrl.Transforms.PoseVector2HomogenousMx(pose_lfoot).block(0,0,3,3);
    Eigen::Matrix3d WR_rf = a_ctrl.Transforms.PoseVector2HomogenousMx(pose_rfoot).block(0,0,3,3);
    // Planar orientations of the feet 
    Vector3d o_lf = a_ctrl.Transforms.getEulerAnglesXYZ_FixedFrame(WR_lf);
    Vector3d o_rf = a_ctrl.Transforms.getEulerAnglesXYZ_FixedFrame(WR_rf);

    
    Vector2d StepFeet    = VectorXd::Zero(2);           // Initialization of the step length
    Matrix2d RotStanceFt = MatrixXd::Identity(2,2);     // Rotation of stance foot wrt the world

    WBIK.virt_jts_pos = jts_pos;
    // WBIK.virt_jts_pos = WBIK.q_0;
    WBIK.var_gain.setZero();
    
    std::cout<< "estimate_min_anticip_step   : \t" << 0.0 << std::endl;

    this->RelativeStep.setZero();
    //
    while(!stbz && !cnt) // count_max
    {
        MatrixXd W_EdgesNormalCHull_; 
        VectorXd DistanceEdgesCHull_;
        //
        WBIK.solve_QPcvxgen3(robot_model_, pose_lfoot, pose_rfoot, stanceFoot, W_EdgesNormalCHull_, DistanceEdgesCHull_);     // TODO ADD SELF COLLISION AVOIDANCE CONSTRAINTS
        // specify the reference value of the CoM
        // Vector2d 
        Bal_Pertubation = Disturb_c_ + fu_z_mgbetaW_ * WBIK.Pose_EE[4].head(2);
        WBIK.des_X_EE[4].head(2) = -(Bal_Pertubation - 0.5*(WBIK.Pose_EE[2].head(2) + WBIK.Pose_EE[3].head(2)));
        // clamping the CoM position to move freely but within the convex hull
        bool SStable = WBIK.predict_stability(W_EdgesNormalCHull_, DistanceEdgesCHull_, WBIK.Pose_EE[4].head(2), WBIK.W_Pos_AbsFoot);
        if(SStable) 
        {
            WBIK.des_X_EE[4](2) = WBIK.Pose_EE[4](2);
            WBIK.pxy_CoM_n1     = WBIK.pxy_CoM;
            WBIK.pxy_CoM        = (WBIK.Pose_EE[4].head(2) - WBIK.W_Pos_AbsFoot);
        }
        else 
        {
            std::cout<< "\n" << std::endl;
            std::cout<< "COM CLAMPING ACTIVE with : " << WBIK.pxy_CoM_n1.transpose() << std::endl;
            // pxy_CoM_n1 = (pxy_CoM_n1.norm() - 0.01)/(pxy_CoM_n1.norm()+1e-10) * pxy_CoM_n1;
            Vector2d DeltaCoMxy      = (WBIK.pxy_CoM_n1.norm() - 0.03)/(WBIK.pxy_CoM_n1.norm()+1e-10) * WBIK.pxy_CoM_n1;
            WBIK.des_X_EE[4].head(2) = DeltaCoMxy + WBIK.W_Pos_AbsFoot;
            WBIK.des_X_EE[4](2)      = WBIK.Pose_EE[4](2);
        }


        
        //
        if(stanceFoot == "left") {
            this->RelativeStep.tail(2) << 0.0, 1.0;
            WBIK.des_X_EE[3] = a_ctrl.getSlidingFootReference2(Disturb_c_, fu_z_mgbetaW_, WBIK.Pose_EE[4].head(3), WBIK.des_X_EE[2], WBIK.des_X_EE[3], WBIK.des_X_EE[2], WBIK.des_X_EE[3], stanceFoot, StepMagnitude);
            // computing the step length
            //--------------------------
            RotStanceFt << cos(o_lf(2)), -sin(o_lf(2)), sin(o_lf(2)), cos(o_lf(2));
            StepFeet = RotStanceFt.transpose()*(WBIK.Pose_EE[3].head(2) - WBIK.des_X_EE[2].head(2)) + Vector2d(0., a_ctrl.DyFeet_min);
            // StepFeet << WBIK.Pose_EE[3](0) - WBIK.des_X_EE[2](0), (WBIK.Pose_EE[3](1) - WBIK.des_X_EE[2](1)) + a_ctrl.DyFeet_min;
        }
        else {
            this->RelativeStep.tail(2) << 1.0, 0.0;
            WBIK.des_X_EE[2] = a_ctrl.getSlidingFootReference2(Disturb_c_, fu_z_mgbetaW_, WBIK.Pose_EE[4].head(3), WBIK.des_X_EE[2], WBIK.des_X_EE[3], WBIK.des_X_EE[2], WBIK.des_X_EE[3], stanceFoot, StepMagnitude);
            // computing the step length
            //---------------------------
            RotStanceFt << cos(o_rf(2)), -sin(o_rf(2)), sin(o_rf(2)), cos(o_rf(2));
            StepFeet = RotStanceFt.transpose()*(WBIK.Pose_EE[2].head(2) - WBIK.des_X_EE[3].head(2)) - Vector2d(0., a_ctrl.DyFeet_min);
            // StepFeet << WBIK.Pose_EE[2](0) - WBIK.des_X_EE[3](0), WBIK.Pose_EE[2](1) - WBIK.des_X_EE[3](1) - a_ctrl.DyFeet_min;
        }
        //
        // get the step value
        this->RelativeStep.head(3) = this->get_DeltaStep(WBIK.des_X_EE[2], WBIK.des_X_EE[3], stanceFoot);
        // check is the step exceeds the the maximum step length between feet
        //===================================================================
        if(StepFeet.norm() >= a_ctrl.lengthStep_max)
        {
            this->RelativeStep.segment(3, 3) = this->get_DeltaStep(WBIK.des_X_EE[2], WBIK.des_X_EE[3], stanceFoot);
            this->RelativeStep.tail(2) << 1.0, 1.0;
        }
        // estimate the balance perturbation
        //==================================
        // stbz = this->check_stabilizability(Wrench_hands_star_, WBIK.des_X_EE[0].head(3), WBIK.des_X_EE[1].head(3), 
        //                                     WBIK.des_X_EE[2], WBIK.des_X_EE[3], WBIK.des_X_EE[4].head(3), Vector3d(0.,0.,0.), Vector3d(0.,0.,0.));
        stbz = this->check_stabilizability(Wrench_hands_star_, WBIK.des_X_EE[0].head(3), WBIK.des_X_EE[1].head(3), 
                                            WBIK.Pose_EE[2], WBIK.Pose_EE[3], WBIK.Pose_EE[4].head(3), Vector3d(0.,0.,0.), Vector3d(0.,0.,0.));

        // this->check_stabilizability(Wrench_hands_star_, WBIK.des_X_EE[0].head(3), WBIK.des_X_EE[1].head(3), 
        //                                     WBIK.des_X_EE[2], WBIK.des_X_EE[3], WBIK.des_X_EE[4].head(3), Vector3d(0.,0.,0.), Vector3d(0.,0.,0.));
        //
        cnt  = (count >= WBIK.count_max);

        std::cout<< "COUNT GEN ANTICIP : " << count << std::endl;
        std::cout<< "STEPFEET NORM : " << StepFeet.norm() << std::endl;
        // std::cout<< "LEFT  FOOT POSE: " << WBIK.des_X_EE[2].head(2).transpose() << std::endl;
        // std::cout<< "RIGHT FOOT POSE: " << WBIK.des_X_EE[3].head(2).transpose() << std::endl;
        std::cout<< "IS CAPTURABLE : " << stbz << std::endl;

        //
        // ////////////////// DATA LOGGING /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // =================================================================================================================================================================
        int stance_left = 0; 
        if(stanceFoot =="left") stance_left = 1;

        log_astep.Out_joints_pos << count << "  " << 180./M_PI * WBIK.virt_jts_pos.transpose() << std::endl;
        log_astep.Out_joints_vel << count << "  " << WBIK.q_dot_star.transpose() << std::endl; 
        //
        log_astep.Out_stability_var << count << "  " << Bal_Pertubation.transpose() << "  " << WBIK.Pose_EE[4].head(3).transpose() << "  " << WBIK.W_Pos_AbsFoot.transpose() <<  "  ";  // count Perturbation CoM absFoot_in_W
        log_astep.Out_stability_var << WBIK.W_Rot_AbsFoot.row(0) << "  " << WBIK.W_Rot_AbsFoot.row(1) <<  "  ";
        log_astep.Out_stability_var << this->RelativeStep.head(3).transpose() << "  " << SStable << "  " << stance_left << "  " << endl; // delta step  static_stability 
        // convex hull
        if(WBIK.PointsCHull.rows() <= 6)
        {
            VectorXd first_pt_x = WBIK.PointsCHull(0,0) * VectorXd::Ones(7-WBIK.PointsCHull.rows());
            VectorXd first_pt_y = WBIK.PointsCHull(0,1) * VectorXd::Ones(7-WBIK.PointsCHull.rows());
            //
            log_astep.Out_PointsCHull   << WBIK.PointsCHull.col(0).transpose() << "  " << first_pt_x.transpose() << "  ";   // x
            log_astep.Out_PointsCHull   << WBIK.PointsCHull.col(1).transpose() << "  " << first_pt_y.transpose() << endl;   // y   
        }
        else
        {
            log_astep.Out_PointsCHull   << WBIK.PointsCHull.col(0).transpose() << "  ";     // x
            log_astep.Out_PointsCHull   << WBIK.PointsCHull.col(1).transpose() << endl;     // y
        }
        // =====================================================================================================================================================================
        // ////////////////// DATA LOGGING /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        count ++;
    }  

    AntiStep_Done = true;

    stbz = this->check_stabilizability(Wrench_hands_star_, WBIK.des_X_EE[0].head(3), WBIK.des_X_EE[1].head(3), 
                                            WBIK.des_X_EE[2], WBIK.des_X_EE[3], WBIK.des_X_EE[4].head(3), Vector3d(0.,0.,0.), Vector3d(0.,0.,0.));

    std::cout<< "STABILIZED : " << stbz << std::endl;

    return this->RelativeStep;
}


// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ============================================================================================
// Batch running of functions for compatibility
// ============================================================================================
// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool FeedForwardController::check_reachability_batch(WbRobotModel& robot_model_, VectorXd jts_position, Vector7d des_X_EE_[], string stanceFoot, Vector7d pose_lfoot, Vector7d pose_rfoot)
{
    //
    std::cout<< "check_reachability_batch started xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx: " << 0.0 << std::endl;
    //
    bool e_lh   = false;
    bool e_rh   = false;
    bool cnt    = false;
    bool n_loop = false;

    // -------------------------------------------------------------------------------------
    Vector2d w_aH_pos = 0.5*(des_X_EE_[0].head(2) + des_X_EE_[1].head(2));
    Vector2d w_aF_pos = 0.5*(des_X_EE_[2].head(2) + des_X_EE_[3].head(2));
    //------------------------------------------------
    if((w_aH_pos-w_aF_pos).norm() <= WBIK.max_reach)
    {
        if(!reach_batch_start)
        {
            // Inititialization();
            l_count = 0;
            g_count = 0;
            ik_run  = false;
            // ==================================================================================================================
            // WBIK.virt_jts_pos = jts_position;
            WBIK.virt_jts_pos = WBIK.q_0;
            WBIK.q_dot_star.setZero();
            WBIK.isCoMClamping = true;
            WBIK.var_gain.setZero();
            // virt_WHB  = WHB;
            for(int i=0; i<8; i++) WBIK.des_X_EE[i] = des_X_EE_[i];
            // load the reachability parameters
            memcpy(WBIK.task_weight, &WBIK.task_weight_reach[0], 8 * sizeof*WBIK.task_weight_reach);
            //
            Reach_Done = false;
            // change so that this portion is run only once when the function is called for the first time before the final output
            reach_batch_start = true;

            std::cout<< "INITIALIZE REACH : " << 0.0 << std::endl;
        }
        // =======================================================================================================================

        std::cout<< "/// -------------//// --------------------- REACHABILITY CHECKING..... : " << 1.0 << std::endl;

        if(!ik_run && reach_batch_start)
        {
            l_count = 0;
            while(!n_loop && !ik_run)//while(!n_loop && !ik_run)
            {
                //========================================================================================================
                MatrixXd W_EdgesNormalCHull_; 
                VectorXd DistanceEdgesCHull_;
                WBIK.solve_QPcvxgen(robot_model_, pose_lfoot, pose_rfoot, stanceFoot, W_EdgesNormalCHull_, DistanceEdgesCHull_);     // TODO PLOT EVOLUTION OF COM AND CLAMPI IT
                // clamping the CoM position to move freely but within the convex hull
                // --------------------------------------------------------------------
                bool SStable = WBIK.predict_stability(W_EdgesNormalCHull_, DistanceEdgesCHull_, WBIK.Pose_EE[4].head(2), WBIK.W_Pos_AbsFoot);
                if(SStable)
                {
                    des_X_EE[4].head(3) = WBIK.Pose_EE[4].head(3);
                    WBIK.pxy_CoM_n1     = WBIK.pxy_CoM;
                    WBIK.pxy_CoM        = WBIK.Pose_EE[4].head(2) - WBIK.W_Pos_AbsFoot;
                }
                else
                {
                    Vector2d DeltaCoMxy      = (WBIK.pxy_CoM_n1.norm() - 0.03)/(WBIK.pxy_CoM_n1.norm()+1e-10) * WBIK.pxy_CoM_n1;
                    WBIK.des_X_EE[4].head(2) = DeltaCoMxy + WBIK.W_Pos_AbsFoot;
                    WBIK.des_X_EE[4](2)      = WBIK.Pose_EE[4](2);
                }
                // --------------------------------------------
                n_loop = (l_count >= l_count_max);  
                cnt    = (g_count >= WBIK.count_max-1);
                ik_run = cnt || WBIK.getTaskCompletionState();
                // --------------------------------------------

                std::cout<< "/// -------------//// --------------------- REACHABILITY BATCH LCOUNT : " << l_count << std::endl;
                std::cout<< "/// -------------//// --------------------- REACHABILITY BATCH GCOUNT : " << g_count << std::endl;

                // // ////////////////// DATA LOGGING /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                // // =================================================================================================================================================================
                int stance_left = 0; 
                if(stanceFoot =="left") stance_left = 1;

                log_reach.Out_joints_pos << g_count << "  " << 180./M_PI * WBIK.virt_jts_pos.transpose() << std::endl;
                log_reach.Out_joints_vel << g_count << "  " << WBIK.q_dot_star.transpose() << std::endl; 
                //
                log_reach.Out_stability_var << g_count << "  " << Vector2d(0.0, 0.0).transpose() << "  " << WBIK.Pose_EE[4].head(3).transpose() << "  " << WBIK.W_Pos_AbsFoot.transpose() <<  "  ";  // count Perturbation CoM absFoot_in_W
                log_reach.Out_stability_var << WBIK.W_Rot_AbsFoot.row(0) << "  " << WBIK.W_Rot_AbsFoot.row(1) <<  "  ";
                log_reach.Out_stability_var << Vector3d(0.0, 0.0, 0.0).transpose() << "  " << SStable << "  " << stance_left << "  " << endl; // delta step  static_stability 

                if(WBIK.PointsCHull.rows() <= 6)
                {
                    VectorXd first_pt_x = WBIK.PointsCHull(0,0) * VectorXd::Ones(7-WBIK.PointsCHull.rows());
                    VectorXd first_pt_y = WBIK.PointsCHull(0,1) * VectorXd::Ones(7-WBIK.PointsCHull.rows());
                    //
                    log_reach.Out_PointsCHull   << WBIK.PointsCHull.col(0).transpose() << "  " << first_pt_x.transpose() << "  ";   // x
                    log_reach.Out_PointsCHull   << WBIK.PointsCHull.col(1).transpose() << "  " << first_pt_y.transpose() << endl;   // y   
                }
                else
                {
                    log_reach.Out_PointsCHull   << WBIK.PointsCHull.col(0).transpose() << "  ";     // x
                    log_reach.Out_PointsCHull   << WBIK.PointsCHull.col(1).transpose() << endl;     // y
                }
                // // =====================================================================================================================================================================
                // // ////////////////// DATA LOGGING /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                l_count ++; // local counter
                g_count ++; // global counter 
            }
        }

        if(ik_run || cnt)
        {
            //
            WBIK.get_hands_errors();
            // check of the hands errors
            e_lh = ((WBIK.error_pos_norm[0] < WBIK.reach_tol(0)) && (WBIK.error_ori_norm[0] < 10.*WBIK.reach_tol(1)));      // error lhand
            e_rh = ((WBIK.error_pos_norm[1] < WBIK.reach_tol(0)) && (WBIK.error_ori_norm[1] < 10.*WBIK.reach_tol(1)));      // error rhand
            //
            std::cout<< "e_lh : " << e_lh << std::endl;
            std::cout<< "e_rh : " << e_rh << std::endl;
            if(e_lh && e_rh)
                isReachable = true;
            else
                isReachable = false;

            // load the reachability parameters
            memcpy(WBIK.task_weight, &WBIK.task_weight_default[0], 8 * sizeof*WBIK.task_weight_default);

            // reset_variables();
            Reach_Done        = true;
            reach_batch_start = false;

            std::cout<< "REACHABILITY YES : " << 1.0 << std::endl;
        }
        //
        WBIK.des_jts_pos  = WBIK.virt_jts_pos;
        WBIK.des_WHB      = WBIK.virt_WHB;  

        // cout << " Desired Joint position are : \n" << 180./M_PI * des_jts_pos.transpose() << endl;
        cout << " Desired Joint position Torso are : \t" << 180./M_PI * WBIK.des_jts_pos.head(3).transpose() << endl;
        cout << " Desired Joint position Lhand are : \t" << 180./M_PI * WBIK.des_jts_pos.head(10).tail(7).transpose() << endl;
        cout << " Desired Joint position Rhand are : \t" << 180./M_PI * WBIK.des_jts_pos.head(17).tail(7).transpose() << endl;
        cout << " Desired Joint position Lleg  are : \t" << 180./M_PI * WBIK.des_jts_pos.tail(12).head(6).transpose() << endl;
        cout << " Desired Joint position Rleg  are : \t" << 180./M_PI * WBIK.des_jts_pos.tail(6).transpose() << endl;
        cout << " Desired Base transformation  is : \n" << WBIK.des_WHB << endl;

    }
    else if((w_aH_pos-w_aF_pos).norm() > WBIK.max_reach)
    {
        isReachable = false;
        Reach_Done  = true;
        std::cout<< "NO REACHABILITY : " << 2.0 << std::endl;
    }

    return isReachable;
}


//
bool FeedForwardController::get_FMMCoM_batch(WbRobotModel& robot_model_, VectorXd jts_pos, VectorXd Wrench_hands_star_, Vector7d des_X_EE_[], Vector7d pose_lfoot, Vector7d pose_rfoot, string stanceFoot)
{
    //
    std::cout<< "get_FMMCoM_batch  started : " << 0.0 << std::endl;
    bool cnt    = false;
    bool n_loop = false;

    //------------------------------------------------
    if(!fmmCoM_batch_start)
    {
        // Inititialization();
        l_count = 0;
        g_count = 0;
        ik_run  = false;
        // ==================================================================================================================
        // WBIK.virt_jts_pos = jts_pos;
        WBIK.virt_jts_pos = WBIK.q_0;
        WBIK.q_dot_star.setZero();
        WBIK.isCoMClamping = true;
        WBIK.var_gain.setZero();
        // virt_WHB  = WHB;
        for(int i=0; i<8; i++) WBIK.des_X_EE[i] = des_X_EE_[i];
        // estimate the components of the perturbation
        a_ctrl.compute_perturbation_terms2(Wrench_hands_star_, des_X_EE_[0].head(3), des_X_EE_[1].head(3), Vector3d(0.,0.,0.), Vector3d(0.,0.,0.), this->Disturb_c_, this->fu_z_mgbetaW_);
        // load the FMMCOM parameters
        memcpy(WBIK.task_weight, &WBIK.task_weight_fmmcom[0], 8*sizeof *WBIK.task_weight_fmmcom);
        //
        fmmCoM_Done   = false; 
        // change so that this portion is run only once when the function is called for the first time before the final output
        fmmCoM_batch_start = true;

        std::cout<< "INITIALIZE REACH : " << 0.0 << std::endl;
    }
    // =======================================================================================================================

    if(!ik_run && fmmCoM_batch_start)
    {
        l_count = 0;
        while(!n_loop && !ik_run)
        {
            //========================================================================================================
            MatrixXd W_EdgesNormalCHull_; 
            VectorXd DistanceEdgesCHull_;
            WBIK.solve_QPcvxgen(robot_model_, pose_lfoot, pose_rfoot, stanceFoot, W_EdgesNormalCHull_, DistanceEdgesCHull_);     // TODO PLOT EVOLUTION OF COM AND CLAMPI IT
            // specify the reference value of the CoM
            Vector2d Bal_Pertubation = Disturb_c_ + fu_z_mgbetaW_ * WBIK.Pose_EE[4].head(2);
            WBIK.des_X_EE[4].head(2) = -(Bal_Pertubation - 0.5*(WBIK.Pose_EE[2].head(2) + WBIK.Pose_EE[3].head(2)));
            // clamping the CoM position to move freely but within the convex hull
            // --------------------------------------------------------------------
            bool SStable = WBIK.predict_stability(W_EdgesNormalCHull_, DistanceEdgesCHull_, WBIK.Pose_EE[4].head(2), WBIK.W_Pos_AbsFoot);
            if(SStable)
            {
                WBIK.des_X_EE[4](2) = WBIK.Pose_EE[4](2);
                WBIK.pxy_CoM_n1     = WBIK.pxy_CoM;
                WBIK.pxy_CoM        = WBIK.Pose_EE[4].head(2) - WBIK.W_Pos_AbsFoot;
            }
            else
            {
                Vector2d DeltaCoMxy      = (WBIK.pxy_CoM_n1.norm() - 0.03)/(WBIK.pxy_CoM_n1.norm()+1e-10) * WBIK.pxy_CoM_n1;
                WBIK.des_X_EE[4].head(2) = DeltaCoMxy + WBIK.W_Pos_AbsFoot;
                WBIK.des_X_EE[4](2)      = WBIK.Pose_EE[4](2);
            }
            // --------------------------------------------
            n_loop = (l_count >= l_count_max);  
            cnt    = (g_count >= WBIK.count_max-1);
            ik_run = cnt || WBIK.getTaskCompletionState();
            // --------------------------------------------

            std::cout<< "FMMCOM BATCH LCOUNT : " << l_count << std::endl;
            std::cout<< "FMMCOM BATCH GCOUNT : " << g_count << std::endl;
            //
            // ////////////////// DATA LOGGING /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // =================================================================================================================================================================
            int stance_left = 0; 
            if(stanceFoot =="left") stance_left = 1;

            log_fmmcom.Out_joints_pos << g_count << "  " << 180./M_PI * WBIK.virt_jts_pos.transpose() << std::endl;
            log_fmmcom.Out_joints_vel << g_count << "  " << WBIK.q_dot_star.transpose() << std::endl; 
            //
            log_fmmcom.Out_stability_var << g_count << "  " << Bal_Pertubation.transpose() << "  " << WBIK.Pose_EE[4].head(3).transpose() << "  " << WBIK.W_Pos_AbsFoot.transpose() <<  "  ";  // count Perturbation CoM absFoot_in_W
            log_fmmcom.Out_stability_var << WBIK.W_Rot_AbsFoot.row(0) << "  " << WBIK.W_Rot_AbsFoot.row(1) <<  "  ";
            log_fmmcom.Out_stability_var << Vector3d(0.0, 0.0, 0.0).transpose() << "  " << SStable << "  " << stance_left << "  " << endl; // delta step  static_stability 
            // convex hull
            if(WBIK.PointsCHull.rows() <= 6)
            {
                VectorXd first_pt_x = WBIK.PointsCHull(0,0) * VectorXd::Ones(7-WBIK.PointsCHull.rows());
                VectorXd first_pt_y = WBIK.PointsCHull(0,1) * VectorXd::Ones(7-WBIK.PointsCHull.rows());
                //
                log_fmmcom.Out_PointsCHull   << WBIK.PointsCHull.col(0).transpose() << "  " << first_pt_x.transpose() << "  ";   // x
                log_fmmcom.Out_PointsCHull   << WBIK.PointsCHull.col(1).transpose() << "  " << first_pt_y.transpose() << endl;   // y   
            }
            else
            {
                log_fmmcom.Out_PointsCHull   << WBIK.PointsCHull.col(0).transpose() << "  ";     // x
                log_fmmcom.Out_PointsCHull   << WBIK.PointsCHull.col(1).transpose() << endl;     // y
            }
            // =====================================================================================================================================================================
            // ////////////////// DATA LOGGING /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            l_count ++; // local counter
            g_count ++; // global counter 
        }
    }

    if(ik_run || cnt)
    {
        //
        WBIK.des_jts_pos  = WBIK.virt_jts_pos;
        WBIK.des_WHB      = WBIK.virt_WHB;
        fmmCoM            = WBIK.Pose_EE[4].head(3);
        //
        // reset_variables();
        fmmCoM_Done        = true; 
        fmmCoM_batch_start = false;

        // load the reachability parameters
        memcpy(WBIK.task_weight, &WBIK.task_weight_default[0], 8 * sizeof*WBIK.task_weight_default);

        std::cout<< "FMMCOM YES : " << 1.0 << std::endl;

        // cout << " Desired Joint position are : \n" << 180./M_PI * des_jts_pos.transpose() << endl;
        cout << " Desired Joint position Torso are : \t" << 180./M_PI * WBIK.des_jts_pos.head(3).transpose() << endl;
        cout << " Desired Joint position Lhand are : \t" << 180./M_PI * WBIK.des_jts_pos.head(10).tail(7).transpose() << endl;
        cout << " Desired Joint position Rhand are : \t" << 180./M_PI * WBIK.des_jts_pos.head(17).tail(7).transpose() << endl;
        cout << " Desired Joint position Lleg  are : \t" << 180./M_PI * WBIK.des_jts_pos.tail(12).head(6).transpose() << endl;
        cout << " Desired Joint position Rleg  are : \t" << 180./M_PI * WBIK.des_jts_pos.tail(6).transpose() << endl;
        cout << " Desired Base transformation  is : \n" << WBIK.des_WHB << endl;

        for (int i=0; i<8; i++)
            std::cout << " ERROR POS and ORI NORM " << WBIK.EE[i] << "  : \t" << WBIK.error_pos_norm[i] << "  and : " << WBIK.error_ori_norm[i] << std::endl;
    }

    return true;
}

//
//
bool FeedForwardController::estimate_min_anticip_step_batch(WbRobotModel& robot_model_, VectorXd jts_pos, VectorXd Wrench_hands_star_, Vector7d  des_X_EE_[], Vector7d pose_lfoot, Vector7d pose_rfoot)
{
    //
    // std::cout<< "estimate_min_anticip_step_batch  started : " << 0.0 << std::endl;
    std::cout<< "estimate_min_anticip_step_batch start =================> : " << 0.0 << std::endl;  
    bool cnt    = false;
    bool n_loop = false;
    bool stbz   = false;
    //
    // 3D rotation matrix of the feet wrt the world frame
    Eigen::Matrix3d WR_lf = a_ctrl.Transforms.PoseVector2HomogenousMx(pose_lfoot).block(0,0,3,3);
    Eigen::Matrix3d WR_rf = a_ctrl.Transforms.PoseVector2HomogenousMx(pose_rfoot).block(0,0,3,3);
    // Planar orientations of the feet 
    Vector3d o_lf = a_ctrl.Transforms.getEulerAnglesXYZ_FixedFrame(WR_lf);
    Vector3d o_rf = a_ctrl.Transforms.getEulerAnglesXYZ_FixedFrame(WR_rf);
    // computation of the step length
    Vector2d StepFeet    = VectorXd::Zero(2);
    Matrix2d RotStanceFt = MatrixXd::Identity(2,2);
    
    //------------------------------------------------
    if(!aStep_batch_start)
    {
        // Inititialization();
        l_count     = 0;
        g_count     = 0;
        ik_run      = false;
        // ==================================================================================================================
        // WBIK.virt_jts_pos = jts_pos;
        WBIK.virt_jts_pos = WBIK.q_0;
        WBIK.q_dot_star.setZero();
        WBIK.isCoMClamping = true;
        WBIK.var_gain.setZero();
        // virt_WHB  = WHB;
        for(int i=0; i<8; i++) WBIK.des_X_EE[i] = des_X_EE_[i];
        // load the stepping solver parameters
        // determine the stance foot
        stance_foot_step = get_stance_foot(pose_lfoot, pose_rfoot, a_ctrl.Delat_R_cmp);
        // update the task weight
        WBIK.update_stepping_weight(stance_foot_step);
        //
        // memcpy(WBIK.task_weight, &WBIK.task_weight_aStep[0], 8*sizeof*WBIK.task_weight_aStep); //
        memcpy(WBIK.task_weight, &WBIK.task_weight_fmmcom[0], 8*sizeof *WBIK.task_weight_fmmcom);
        // estimate the balance perturbation 
        this->check_stabilizability(Wrench_hands_star_, des_X_EE[0].head(3), des_X_EE[1].head(3), pose_lfoot, pose_rfoot, des_X_EE[4].head(3), Vector3d(0.,0.,0.), Vector3d(0.,0.,0.));

        // change so that this portion is run only once when the function is called for the first time before the final output
        aStep_batch_start = true;
        AntiStep_Done     = false;

        this->RelativeStep.setZero();

        std::cout<< "INITIALIZE A-STEP : " << 0.0 << std::endl;
    }
    // =======================================================================================================================

    if(!ik_run && aStep_batch_start)
    {
        std::cout<< "A-STEP IN IF LOOP : " << 0.0 << std::endl;
        l_count = 0;
        while(!n_loop && !ik_run && !stbz)
        {
            
            std::cout<< "A-STEP IN WHILE LOOP : " << 0.0 << std::endl;
            //========================================================================================================
            MatrixXd W_EdgesNormalCHull_; 
            VectorXd DistanceEdgesCHull_;
            WBIK.solve_QPcvxgen3(robot_model_, pose_lfoot, pose_rfoot, stance_foot_step, W_EdgesNormalCHull_, DistanceEdgesCHull_);     // TODO PLOT EVOLUTION OF COM AND CLAMPI IT
            // WBIK.solve_QPcvxgen(robot_model_, pose_lfoot, pose_rfoot, stance_foot_step, W_EdgesNormalCHull_, DistanceEdgesCHull_);     // TODO PLOT EVOLUTION OF COM AND CLAMPI IT
            // specify the reference value of the CoM
            Vector2d Bal_Pertubation = Disturb_c_ + fu_z_mgbetaW_ * WBIK.Pose_EE[4].head(2);
            WBIK.des_X_EE[4].head(2) = -(Bal_Pertubation - 0.5*(WBIK.Pose_EE[2].head(2) + WBIK.Pose_EE[3].head(2)));
            // clamping the CoM position to move freely but within the convex hull
            // --------------------------------------------------------------------
            bool SStable = WBIK.predict_stability(W_EdgesNormalCHull_, DistanceEdgesCHull_, WBIK.Pose_EE[4].head(2), WBIK.W_Pos_AbsFoot);
            if(SStable)
            {
                WBIK.des_X_EE[4](2) = WBIK.Pose_EE[4](2);
                WBIK.pxy_CoM_n1     = WBIK.pxy_CoM;
                WBIK.pxy_CoM        = WBIK.Pose_EE[4].head(2) - WBIK.W_Pos_AbsFoot;
            }
            else
            {
                Vector2d DeltaCoMxy      = (WBIK.pxy_CoM_n1.norm() - 0.03)/(WBIK.pxy_CoM_n1.norm()+1e-10) * WBIK.pxy_CoM_n1;
                WBIK.des_X_EE[4].head(2) = DeltaCoMxy + WBIK.W_Pos_AbsFoot;
                WBIK.des_X_EE[4](2)      = WBIK.Pose_EE[4](2);
            }

            // get the step value
            this->RelativeStep.head(3) = this->get_DeltaStep(WBIK.des_X_EE[2], WBIK.des_X_EE[3], stance_foot_step);
            //
            if(stance_foot_step == "left") {
                this->RelativeStep.tail(2) << 0.0, 1.0;
                // WBIK.des_X_EE[3] = a_ctrl.getSlidingFootReference(Disturb_c_, fu_z_mgbetaW_, WBIK.Pose_EE[4].head(3), WBIK.des_X_EE[2], WBIK.des_X_EE[3], stance_foot_step, StepMagnitude);
                WBIK.des_X_EE[3] = a_ctrl.getSlidingFootReference2(Disturb_c_, fu_z_mgbetaW_, WBIK.Pose_EE[4].head(3), WBIK.des_X_EE[2], WBIK.des_X_EE[3], 
                                                                    pose_lfoot, pose_rfoot, stance_foot_step, StepMagnitude);
                // computing the step length
                //--------------------------
                RotStanceFt << cos(o_lf(2)), -sin(o_lf(2)), sin(o_lf(2)), cos(o_lf(2));
                StepFeet = RotStanceFt.transpose()*(WBIK.Pose_EE[3].head(2) - WBIK.des_X_EE[2].head(2)) + Vector2d(0., a_ctrl.DyFeet_min);
            }
            else {
                this->RelativeStep.tail(2) << 1.0, 0.0;
                // WBIK.des_X_EE[2] = a_ctrl.getSlidingFootReference(Disturb_c_, fu_z_mgbetaW_, WBIK.Pose_EE[4].head(3), WBIK.des_X_EE[2], WBIK.des_X_EE[3], stance_foot_step, StepMagnitude);
                WBIK.des_X_EE[2] = a_ctrl.getSlidingFootReference2(Disturb_c_, fu_z_mgbetaW_, WBIK.Pose_EE[4].head(3), WBIK.des_X_EE[2], WBIK.des_X_EE[3], 
                                                                    pose_lfoot, pose_rfoot, stance_foot_step, StepMagnitude);
                // computing the step length
                //---------------------------
                RotStanceFt << cos(o_rf(2)), -sin(o_rf(2)), sin(o_rf(2)), cos(o_rf(2));
                StepFeet = RotStanceFt.transpose()*(WBIK.Pose_EE[2].head(2) - WBIK.des_X_EE[3].head(2)) - Vector2d(0., a_ctrl.DyFeet_min);
            }

            // check is the step exceeds the the maximum step length between feet
            //===================================================================
            if(StepFeet.norm() >= a_ctrl.lengthStep_max)
            {
                this->RelativeStep.segment(3, 3) = this->get_DeltaStep(WBIK.des_X_EE[2], WBIK.des_X_EE[3], stance_foot_step);
                this->RelativeStep.tail(2) << 1.0, 1.0;
            }
            // estimate the balance perturbation
            //==================================
            // stbz = this->check_stabilizability(Wrench_hands_star_, WBIK.des_X_EE[0].head(3), WBIK.des_X_EE[1].head(3), 
            //                                     WBIK.des_X_EE[2], WBIK.des_X_EE[3], WBIK.des_X_EE[4].head(3), Vector3d(0.,0.,0.), Vector3d(0.,0.,0.));

            this->check_stabilizability(Wrench_hands_star_, WBIK.des_X_EE[0].head(3), WBIK.des_X_EE[1].head(3), 
                                        WBIK.des_X_EE[2], WBIK.des_X_EE[3], WBIK.des_X_EE[4].head(3), Vector3d(0.,0.,0.), Vector3d(0.,0.,0.));

            // --------------------------------------------
            n_loop = (l_count >= l_count_max);  
            cnt    = (g_count >= WBIK.count_max-1);
            ik_run = cnt; // || WBIK.getTaskCompletionState();
            // --------------------------------------------

            std::cout<< "A-STEP BATCH LCOUNT : " << l_count << std::endl;
            std::cout<< "A-STEP BATCH GCOUNT : " << g_count << std::endl;

            //
            // ////////////////// DATA LOGGING /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // =================================================================================================================================================================
            int stance_left = 0; 
            if(stance_foot_step =="left") stance_left = 1;

            log_astep.Out_joints_pos << g_count << "  " << 180./M_PI * WBIK.virt_jts_pos.transpose() << std::endl;
            log_astep.Out_joints_vel << g_count << "  " << WBIK.q_dot_star.transpose() << std::endl; 
            //
            log_astep.Out_stability_var << g_count << "  " << Bal_Pertubation.transpose() << "  " << WBIK.Pose_EE[4].head(3).transpose() << "  " << WBIK.W_Pos_AbsFoot.transpose() <<  "  ";  // count Perturbation CoM absFoot_in_W
            log_astep.Out_stability_var << WBIK.W_Rot_AbsFoot.row(0) << "  " << WBIK.W_Rot_AbsFoot.row(1) <<  "  ";
            log_astep.Out_stability_var << this->RelativeStep.head(3).transpose() << "  " << SStable << "  " << stance_left << "  " << endl; // delta step  static_stability 
            // convex hull
            if(WBIK.PointsCHull.rows() <= 6)
            {
                VectorXd first_pt_x = WBIK.PointsCHull(0,0) * VectorXd::Ones(7-WBIK.PointsCHull.rows());
                VectorXd first_pt_y = WBIK.PointsCHull(0,1) * VectorXd::Ones(7-WBIK.PointsCHull.rows());
                //
                log_astep.Out_PointsCHull   << WBIK.PointsCHull.col(0).transpose() << "  " << first_pt_x.transpose() << "  ";   // x
                log_astep.Out_PointsCHull   << WBIK.PointsCHull.col(1).transpose() << "  " << first_pt_y.transpose() << endl;   // y   
            }
            else
            {
                log_astep.Out_PointsCHull   << WBIK.PointsCHull.col(0).transpose() << "  ";     // x
                log_astep.Out_PointsCHull   << WBIK.PointsCHull.col(1).transpose() << endl;     // y
            }
            // =====================================================================================================================================================================
            // ////////////////// DATA LOGGING /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


            l_count ++; // local counter
            g_count ++; // global counter 
        }
    }

    if(ik_run || cnt || stbz)
    {
        //
        WBIK.des_jts_pos  = WBIK.virt_jts_pos;
        WBIK.des_WHB      = WBIK.virt_WHB;
        //
        // reset_variables();
        AntiStep_Done     = true; 
        aStep_batch_start = false;

        // load the reachability parameters
        memcpy(WBIK.task_weight, &WBIK.task_weight_default[0], 8 * sizeof*WBIK.task_weight_default);

        std::cout<< "A-STEP YES : " << 1.0 << std::endl;

        // cout << " Desired Joint position are : \n" << 180./M_PI * des_jts_pos.transpose() << endl;
        cout << " Desired Joint position Torso are : \t" << 180./M_PI * WBIK.des_jts_pos.head(3).transpose() << endl;
        cout << " Desired Joint position Lhand are : \t" << 180./M_PI * WBIK.des_jts_pos.head(10).tail(7).transpose() << endl;
        cout << " Desired Joint position Rhand are : \t" << 180./M_PI * WBIK.des_jts_pos.head(17).tail(7).transpose() << endl;
        cout << " Desired Joint position Lleg  are : \t" << 180./M_PI * WBIK.des_jts_pos.tail(12).head(6).transpose() << endl;
        cout << " Desired Joint position Rleg  are : \t" << 180./M_PI * WBIK.des_jts_pos.tail(6).transpose() << endl;
        cout << " Desired Base transformation  is : \n" << WBIK.des_WHB << endl;

        for (int i=0; i<8; i++)
            std::cout << " ERROR POS and ORI NORM " << WBIK.EE[i] << "  : \t" << WBIK.error_pos_norm[i] << "  and : " << WBIK.error_ori_norm[i] << std::endl;
    }

    return true;
}


//
//
bool FeedForwardController::run_batch(WbRobotModel& robot_model_, VectorXd jts_pos, VectorXd Wrench_hands_star_, Vector7d des_X_EE_[], 
                                      Vector7d pose_lfoot, Vector7d pose_rfoot, string stanceFoot, bool &executing_step, VectorXd &ref_step)
{
    // 
    std::cout<< "run_batch STARTED  =================> : " << 0.0 << std::endl;  
    std::cout<< "isReachable        =================> : " << isReachable << std::endl;
    std::cout<< "Reach_Done         =================> : " << Reach_Done << std::endl;  
    std::cout<< "fmmCoM_Done        =================> : " << fmmCoM_Done << std::endl;  
    std::cout<< "isStable           =================> : " << isStable << std::endl;  
    std::cout<< "AntiStep_Done      =================> : " << AntiStep_Done << std::endl; 

    // reachability
    if(!this->isReachable && !this->Reach_Done && !this->fmmCoM_Done && !this->AntiStep_Done)
    {
        this->isReachable = this->check_reachability_batch(robot_model_, jts_pos,  des_X_EE_, stanceFoot, pose_lfoot, pose_rfoot);
        //
        // Reach_Done  = this->Reach_Done;
    } 

    // stabilizability
    else if(this->isReachable && this->Reach_Done && !this->fmmCoM_Done && !this->AntiStep_Done)
    {
        this->get_FMMCoM_batch(robot_model_, this->WBIK.des_jts_pos, Wrench_hands_star_, des_X_EE_, pose_lfoot, pose_rfoot, stanceFoot);
        // this->get_FMMCoM_batch(robot_model_, jts_pos, Wrench_hands_star_, des_X_EE_, pose_lfoot, pose_rfoot, stanceFoot);
        //
        // fmmCoM_Done = this->fmmCoM_Done;
        //
        // if(fmmCoM_Done)
        //     fmmCoM = this->fmmCoM;
        std::cout<< " ==============================================> get_FMMCoM_batch =================> : " << 0.0 << std::endl;  
    }
    //
    else if(this->fmmCoM_Done && !this->Stable_Done)
    {
        std::cout<< "check_stabilizability start =================> : " << 0.0 << std::endl;  

        this->isStable    = this->check_stabilizability(  Wrench_hands_star_, this->WBIK.Pose_EE[0].head(3), this->WBIK.Pose_EE[1].head(3), pose_lfoot, pose_rfoot, 
                                                            this->fmmCoM, Vector3d(0.,0.,0.), Vector3d(0.,0.,0.));
        //
        this->Stable_Done = true;

        std::cout<< "Stable_Done =================> : " << Stable_Done << std::endl;  
    }

    // compute the anticipatory
    else if(!this->isStable && this->Reach_Done && this->fmmCoM_Done && !this->AntiStep_Done) // (!this->isStable && this->Reach_Done && this->fmmCoM_Done && !this->AntiStep_Done) 
    {
        // std::cout<< "estimate_min_anticip_step_batch start =================> : " << 0.0 << std::endl;  
        //
        this->estimate_min_anticip_step_batch(robot_model_, this->WBIK.des_jts_pos, Wrench_hands_star_, des_X_EE_, pose_lfoot, pose_rfoot);
        //
        // AntiStep_Done  = this->AntiStep_Done;
        std::cout<< "AntiStep_Done =================> : " << AntiStep_Done << std::endl;  
        //
        if(this->AntiStep_Done)
        {
            ref_step        = this->RelativeStep;    // assign the value of the footstep
            executing_step  = true;                    // activate the step exectution
            //
            this->Reach_Done      = false;
            this->fmmCoM_Done     = false;
            this->AntiStep_Done   = false;
            this->Stable_Done     = false;

            this->isStable        = false;
            this->isReachable     = false;
        }   
    }
    else if(this->isStable && this->Reach_Done && this->fmmCoM_Done && !this->AntiStep_Done) // set to Zero th step values
    {
        std::cout<< "estimate_min_anticip_step_batch no exe =================> : " << 1.0 << std::endl;
        // keep the step reference to zero
        ref_step = VectorXd::Zero(8); //setZero();
        //
        this->Reach_Done      = false;
        this->fmmCoM_Done     = false;
        this->AntiStep_Done   = false;
        this->Stable_Done     = false;
        //
    }

    return true;
}



VectorXd FeedForwardController::getInitialGuess(WbRobotModel& robot_model_, VectorXd jts_pos, VectorXd Wrench_hands_star_, 
                                                Vector7d  des_X_EE_[], Vector7d pose_lfoot, Vector7d pose_rfoot)
{
    //
    int count = 0; 
    // load the stepping solver parameters
    // memcpy(WBIK.task_weight, &WBIK.task_weight_aStep[0], 8*sizeof*WBIK.task_weight_aStep); //
    memcpy(WBIK.task_weight, &WBIK.task_weight_fmmcom[0], 8*sizeof *WBIK.task_weight_fmmcom);
    //
    Matrix4d WHB = MatrixXd::Identity(4,4);
    //
    for(int i=0; i<8; i++) WBIK.des_X_EE[i] = des_X_EE_[i];
    //
    bool stbz = false;
    bool cnt  = false;
    
    // estimate the components of the perturbation
    a_ctrl.compute_perturbation_terms2(Wrench_hands_star_, des_X_EE_[0].head(3), des_X_EE_[1].head(3), Vector3d(0.,0.,0.), Vector3d(0.,0.,0.), this->Disturb_c_, this->fu_z_mgbetaW_);
    //
    std::cout<< "DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD   : \t" << 0.0 << std::endl;
    std::cout<< "Disturb_c_   : \t" << Disturb_c_.transpose() << std::endl;
    std::cout<< "fu_z_mgbetaW_   : \t" << fu_z_mgbetaW_ << std::endl;
    Vector2d Bal_Pertubation = Disturb_c_ + fu_z_mgbetaW_*0.5*(pose_lfoot.head(2) + pose_rfoot.head(2));
    Vector2d BalPert_      = Bal_Pertubation - 0.5*(pose_lfoot.head(2) + pose_rfoot.head(2));
    //
    std::string stanceFoot = get_stance_foot(pose_lfoot, pose_rfoot, BalPert_);
    stance_ft = stanceFoot;

    std::cout<< "Stance foot   : \t" << stanceFoot << std::endl;
    //
    WBIK.update_stepping_weight(stanceFoot);
    //
    this->RelativeStep.setZero();  // Reset
    //
    if(stanceFoot == "left") 
    {
        this->RelativeStep.tail(2) << 0.0, 1.0;
        WBIK.des_X_EE[3] = a_ctrl.getInitGuessFootReference(Disturb_c_, fu_z_mgbetaW_, pose_lfoot, pose_rfoot, stanceFoot);
    }
    else 
    {
        this->RelativeStep.tail(2) << 1.0, 0.0;
        WBIK.des_X_EE[2] = a_ctrl.getInitGuessFootReference(Disturb_c_, fu_z_mgbetaW_, pose_lfoot, pose_rfoot, stanceFoot);
    }
    //
    WBIK.virt_jts_pos = WBIK.q_0;
    WBIK.var_gain.setZero();
    //
    // estimate the balance perturbation 
    this->check_stabilizability(Wrench_hands_star_, WBIK.des_X_EE[0].head(3), WBIK.des_X_EE[1].head(3), 
                                WBIK.des_X_EE[2], WBIK.des_X_EE[3], WBIK.des_X_EE[4].head(3), Vector3d(0.,0.,0.), Vector3d(0.,0.,0.));
    
    std::cout<< "estimate_min_anticip_step   : \t" << 0.0 << std::endl;
    //
    while(!stbz && !cnt) // count_max
    {
        MatrixXd W_EdgesNormalCHull_; 
        VectorXd DistanceEdgesCHull_;
        //
        WBIK.solve_QPcvxgen(robot_model_, pose_lfoot, pose_rfoot, stanceFoot, W_EdgesNormalCHull_, DistanceEdgesCHull_);     // TODO ADD SELF COLLISION AVOIDANCE CONSTRAINTS
        // specify the reference value of the CoM
        WBIK.des_X_EE[4].head(2) = 0.5*(WBIK.Pose_EE[2].head(2) + WBIK.Pose_EE[3].head(2));
        // clamping the CoM position to move freely but within the convex hull
        bool SStable = WBIK.predict_stability(W_EdgesNormalCHull_, DistanceEdgesCHull_, WBIK.Pose_EE[4].head(2), WBIK.W_Pos_AbsFoot);   // (WBIK.Pose_EE[4].head(2) + WBIK.Jacobian_EE[4].topRows(2) * q_dot_star * DT)
        if(SStable) 
        {
            WBIK.des_X_EE[4](2) = WBIK.Pose_EE[4](2);
            WBIK.pxy_CoM_n1     = WBIK.pxy_CoM;
            WBIK.pxy_CoM        = (WBIK.Pose_EE[4].head(2) - WBIK.W_Pos_AbsFoot);
        }
        else 
        {
            std::cout<< "\n" << std::endl;
            std::cout<< "COM CLAMPING ACTIVE with : " << WBIK.pxy_CoM_n1.transpose() << std::endl;
            // pxy_CoM_n1 = (pxy_CoM_n1.norm() - 0.01)/(pxy_CoM_n1.norm()+1e-10) * pxy_CoM_n1;
            Vector2d DeltaCoMxy      = (WBIK.pxy_CoM_n1.norm() - 0.03)/(WBIK.pxy_CoM_n1.norm()+1e-10) * WBIK.pxy_CoM_n1;
            WBIK.des_X_EE[4].head(2) = DeltaCoMxy + WBIK.W_Pos_AbsFoot;
            WBIK.des_X_EE[4](2)      = WBIK.Pose_EE[4](2);
        }
        // get the step value
        this->RelativeStep.head(3) = this->get_DeltaStep(WBIK.des_X_EE[2], WBIK.des_X_EE[3], stanceFoot);
        //
        cnt  = (count >= WBIK.count_max);
        std::cout<< "COUNT GEN GUESS : " << count << std::endl;
        //
        // ////////////////// DATA LOGGING /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // =================================================================================================================================================================
        int stance_left = 0; 
        if(stanceFoot =="left") stance_left = 1;

        log_iGuess.Out_joints_pos << count << "  " << 180./M_PI * WBIK.virt_jts_pos.transpose() << std::endl;
        log_iGuess.Out_joints_vel << count << "  " << WBIK.q_dot_star.transpose() << std::endl; 
        //
        log_iGuess.Out_stability_var << count << "  " << Bal_Pertubation.transpose() << "  " << WBIK.Pose_EE[4].head(3).transpose() << "  " << WBIK.W_Pos_AbsFoot.transpose() <<  "  ";  // count Perturbation CoM absFoot_in_W
        log_iGuess.Out_stability_var << WBIK.W_Rot_AbsFoot.row(0) << "  " << WBIK.W_Rot_AbsFoot.row(1) <<  "  ";
        log_iGuess.Out_stability_var << this->RelativeStep.head(3).transpose() << "  " << SStable << "  " << stance_left << "  " << endl; // delta step  static_stability 

        if(WBIK.PointsCHull.rows() <= 6)
        {
            VectorXd first_pt_x = WBIK.PointsCHull(0,0) * VectorXd::Ones(7-WBIK.PointsCHull.rows());
            VectorXd first_pt_y = WBIK.PointsCHull(0,1) * VectorXd::Ones(7-WBIK.PointsCHull.rows());
            //
            log_iGuess.Out_PointsCHull   << WBIK.PointsCHull.col(0).transpose() << "  " << first_pt_x.transpose() << "  ";   // x
            log_iGuess.Out_PointsCHull   << WBIK.PointsCHull.col(1).transpose() << "  " << first_pt_y.transpose() << endl;   // y   
        }
        else
        {
            log_iGuess.Out_PointsCHull   << WBIK.PointsCHull.col(0).transpose() << "  ";     // x
            log_iGuess.Out_PointsCHull   << WBIK.PointsCHull.col(1).transpose() << endl;     // y
        }
        // =====================================================================================================================================================================
        // ////////////////// DATA LOGGING /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        count ++;
    }
    //
    WBIK.des_jts_pos = WBIK.virt_jts_pos;

    return WBIK.virt_jts_pos;

}


VectorXd FeedForwardController::getGlobal_anticip_step( WbRobotModel& robot_model_, VectorXd jts_pos, VectorXd Wrench_hands_star_, 
                                                            Vector7d  des_X_EE_[], Vector7d pose_lfoot, Vector7d pose_rfoot, std::string stanceFoot)
{
    //
    //
    int count = 0;
    
    WBIK.isCoMClamping = true;
    // std::string stanceFoot = get_stance_foot(pose_lfoot, pose_rfoot, a_ctrl.Delat_R_cmp);
    //
    WBIK.update_stepping_weight(stanceFoot);
    // load the stepping solver parameters
    memcpy(WBIK.task_weight, &WBIK.task_weight_aStep[0], 8*sizeof*WBIK.task_weight_aStep); //
    // memcpy(WBIK.task_weight, &WBIK.task_weight_fmmcom[0], 8*sizeof *WBIK.task_weight_fmmcom);

    if(stanceFoot == "left")
    {
        WBIK.task_weight[2].head(3) = 10. * WBIK.task_weight_aStep[2].head(3);
        WBIK.task_weight[2].tail(3) = 10. * WBIK.task_weight_aStep[2].tail(3);
        WBIK.task_weight[3].head(3) = 0.3* (1./a_ctrl.lengthStep_max)  * WBIK.task_weight_aStep[3].head(3);
        WBIK.task_weight[3].tail(3) = 0.3* (1./a_ctrl.Dtheta_feet_max) * WBIK.task_weight_aStep[3].tail(3);
    }
    else  // right stance foot
    {
        WBIK.task_weight[2].head(3) = 0.3* (1./a_ctrl.lengthStep_max)  * WBIK.task_weight_aStep[2].head(3);
        WBIK.task_weight[2].tail(3) = 0.3* (1./a_ctrl.Dtheta_feet_max) * WBIK.task_weight_aStep[2].tail(3);
        WBIK.task_weight[3].head(3) = 10. * WBIK.task_weight_aStep[3].head(3);
        WBIK.task_weight[3].tail(3) = 10. * WBIK.task_weight_aStep[3].tail(3);
    }
    //
    Matrix4d WHB = MatrixXd::Identity(4,4);
    //
    for(int i=0; i<8; i++) WBIK.des_X_EE[i] = des_X_EE_[i];
    //
    bool stbz = false;
    bool cnt  = false;
    WBIK.coef_grad = 1.0;
    // // estimate the balance perturbation 
    // this->check_stabilizability(Wrench_hands_star_, WBIK.des_X_EE[0].head(3), WBIK.des_X_EE[1].head(3), 
    //                             pose_lfoot, pose_rfoot, WBIK.des_X_EE[4].head(3), Vector3d(0.,0.,0.), Vector3d(0.,0.,0.));
    std::cout<< "GGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG : \t" << std::endl;
    // estimate the balance perturbation 
    bool captur = this->check_stabilizability(Wrench_hands_star_, WBIK.des_X_EE[0].head(3), WBIK.des_X_EE[1].head(3), 
                                                pose_lfoot, pose_rfoot, WBIK.des_X_EE[4].head(3), Vector3d(0.,0.,0.), Vector3d(0.,0.,0.));

    std::cout<< "CAPTURABILITY BEFORE  G : \t" << captur << std::endl;

    stbz = this->check_stabilizability(Wrench_hands_star_, WBIK.des_X_EE[0].head(3), WBIK.des_X_EE[1].head(3), 
                                                WBIK.Pose_EE[2], WBIK.Pose_EE[3], WBIK.Pose_EE[4].head(3), Vector3d(0.,0.,0.), Vector3d(0.,0.,0.));

    Eigen::Matrix3d WR_lf = a_ctrl.Transforms.PoseVector2HomogenousMx(pose_lfoot).block(0,0,3,3);
    Eigen::Matrix3d WR_rf = a_ctrl.Transforms.PoseVector2HomogenousMx(pose_rfoot).block(0,0,3,3);
    // Planar orientations of the feet 
    Vector3d o_lf = a_ctrl.Transforms.getEulerAnglesXYZ_FixedFrame(WR_lf);
    Vector3d o_rf = a_ctrl.Transforms.getEulerAnglesXYZ_FixedFrame(WR_rf);
    //    
    Vector2d StepFeet    = VectorXd::Zero(2);           // Initialization of the step length
    Matrix2d RotStanceFt = MatrixXd::Identity(2,2);     // Rotation of stance foot wrt the world
    //
    WBIK.virt_jts_pos    = jts_pos;
    // WBIK.virt_jts_pos = WBIK.q_0;
    WBIK.q_dot_star.setZero();
    WBIK.var_gain.setZero();

    std::cout<< "estimate_min_anticip_step   : \t" << 0.0 << std::endl;
    this->RelativeStep.setZero();
     // get the step value
    this->RelativeStep.head(3) = this->get_DeltaStep(WBIK.des_X_EE[2], WBIK.des_X_EE[3], stanceFoot);
    //
    std::cout<< "RELATIVE STEP  G 00 : \t" << RelativeStep.transpose() << std::endl;
    std::cout<< "LEFT  FOOT POS : \t" << WBIK.Pose_EE[2].head(3).transpose() << std::endl;
    std::cout<< "RIGHT FOOT POS : \t" << WBIK.Pose_EE[3].head(3).transpose() << std::endl;
    //
    while(stbz && !cnt) // count_max
    {
        
        //
        if(stanceFoot == "left") {
            this->RelativeStep.tail(2) << 0.0, 1.0;
            // WBIK.des_X_EE[3] = WBIK.des_X_EE[2]; 
            WBIK.des_X_EE[3] = pose_rfoot; 
            // computing the step length
            //--------------------------
            RotStanceFt << cos(o_lf(2)), -sin(o_lf(2)), sin(o_lf(2)), cos(o_lf(2));
            StepFeet = RotStanceFt.transpose()*(WBIK.Pose_EE[3].head(2) - WBIK.des_X_EE[2].head(2)) + Vector2d(0., a_ctrl.DyFeet_min);
            // StepFeet << WBIK.Pose_EE[3](0) - WBIK.des_X_EE[2](0), (WBIK.Pose_EE[3](1) - WBIK.des_X_EE[2](1)) + a_ctrl.DyFeet_min;
        }
        else {
            this->RelativeStep.tail(2) << 1.0, 0.0;
            // WBIK.des_X_EE[2] = WBIK.des_X_EE[3];
            WBIK.des_X_EE[2] = pose_lfoot;
            // computing the step length
            //---------------------------
            RotStanceFt << cos(o_rf(2)), -sin(o_rf(2)), sin(o_rf(2)), cos(o_rf(2));
            StepFeet = RotStanceFt.transpose()*(WBIK.Pose_EE[2].head(2) - WBIK.des_X_EE[3].head(2)) - Vector2d(0., a_ctrl.DyFeet_min);
            // StepFeet << WBIK.Pose_EE[2](0) - WBIK.des_X_EE[3](0), WBIK.Pose_EE[2](1) - WBIK.des_X_EE[3](1) - a_ctrl.DyFeet_min;
        }
        //
        // get the step value
        this->RelativeStep.head(3) = this->get_DeltaStep(WBIK.Pose_EE[2], WBIK.Pose_EE[3], stanceFoot);
        // check is the step exceeds the the maximum step length between feet
        //===================================================================
        if(StepFeet.norm() >= a_ctrl.lengthStep_max)
        {
            this->RelativeStep.segment(3, 3) = this->get_DeltaStep(WBIK.Pose_EE[2], WBIK.Pose_EE[3], stanceFoot);
            this->RelativeStep.tail(2) << 1.0, 1.0;
        }
        //
        // ===============================================================================================================================================================================
        MatrixXd W_EdgesNormalCHull_; 
        VectorXd DistanceEdgesCHull_;
        // estimate the components of the perturbation
        a_ctrl.compute_perturbation_terms2(Wrench_hands_star_, des_X_EE_[0].head(3), des_X_EE_[1].head(3), Vector3d(0.,0.,0.), Vector3d(0.,0.,0.), this->Disturb_c_, this->fu_z_mgbetaW_);

        WBIK.solve_QPcvxgen4(robot_model_, pose_lfoot, pose_rfoot, stanceFoot, W_EdgesNormalCHull_, DistanceEdgesCHull_, Disturb_c_, fu_z_mgbetaW_);
        // specify the reference value of the CoM
        Vector2d Bal_Pertubation = Disturb_c_ + fu_z_mgbetaW_ * WBIK.Pose_EE[4].head(2);
        WBIK.des_X_EE[4].head(2) = -(Bal_Pertubation - 0.5*(WBIK.Pose_EE[2].head(2) + WBIK.Pose_EE[3].head(2)));
        // clamping the CoM position to move freely but within the convex hull
        bool SStable = WBIK.predict_stability(W_EdgesNormalCHull_, DistanceEdgesCHull_, WBIK.Pose_EE[4].head(2), WBIK.W_Pos_AbsFoot);
        if(SStable) 
        {
            WBIK.des_X_EE[4](2) = WBIK.Pose_EE[4](2);
            WBIK.pxy_CoM_n1     = WBIK.pxy_CoM;
            WBIK.pxy_CoM        = (WBIK.Pose_EE[4].head(2) - WBIK.W_Pos_AbsFoot);
        }
        else 
        {
            std::cout<< "\n" << std::endl;
            // pxy_CoM_n1 = (pxy_CoM_n1.norm() - 0.01)/(pxy_CoM_n1.norm()+1e-10) * pxy_CoM_n1;
            Vector2d DeltaCoMxy      = (WBIK.pxy_CoM_n1.norm() - 0.03)/(WBIK.pxy_CoM_n1.norm()+1e-10) * WBIK.pxy_CoM_n1;
            WBIK.des_X_EE[4].head(2) = DeltaCoMxy + WBIK.W_Pos_AbsFoot;
            WBIK.des_X_EE[4](2)      = WBIK.Pose_EE[4](2);
        }
        
        
        //
        //==================================
        // stbz = this->check_stabilizability(Wrench_hands_star_, WBIK.des_X_EE[0].head(3), WBIK.des_X_EE[1].head(3), 
        //                                     WBIK.Pose_EE[2], WBIK.Pose_EE[3], WBIK.Pose_EE[4].head(3), Vector3d(0.,0.,0.), Vector3d(0.,0.,0.));
        stbz = this->check_stabilizability(Wrench_hands_star_, WBIK.des_X_EE[0].head(3), WBIK.des_X_EE[1].head(3), 
                                                WBIK.Pose_EE[2], WBIK.Pose_EE[3], WBIK.Pose_EE[4].head(3), Vector3d(0.,0.,0.), Vector3d(0.,0.,0.));

        // this->check_stabilizability(Wrench_hands_star_, WBIK.des_X_EE[0].head(3), WBIK.des_X_EE[1].head(3), 
        //                                     WBIK.des_X_EE[2], WBIK.des_X_EE[3], WBIK.des_X_EE[4].head(3), Vector3d(0.,0.,0.), Vector3d(0.,0.,0.));
        
        // estimate the balance perturbation
        

        std::cout<< " PERTURBATION : \t" << Bal_Pertubation.transpose() << std::endl;
        std::cout<< " COM+PERTURBATION : \t" << (Bal_Pertubation + WBIK.Pose_EE[4].head(2)).transpose() << std::endl;
        std::cout<< " COM : \t" << (WBIK.Pose_EE[4].head(2)).transpose() << std::endl;

        std::cout<< "COUNT GEN ANTICIP : " << count << std::endl;
        std::cout<< " stbz ///////////////////////////////////////////////////////////////////////////////// : " << stbz << std::endl;
        std::cout<< " RELATIVE STEP ANTICIP : " << this->RelativeStep.transpose() << std::endl;
        std::cout<< "LEFT  FOOT POS : \t" << WBIK.Pose_EE[2].head(3).transpose() << std::endl;
        std::cout<< "RIGHT FOOT POS : \t" << WBIK.Pose_EE[3].head(3).transpose() << std::endl;
        std::cout<< "STANCE FOOT  G : \t" << stanceFoot << std::endl;

        //
        // ////////////////// DATA LOGGING /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // =================================================================================================================================================================
        int stance_left = 0; 
        if(stanceFoot =="left") stance_left = 1;

        if(stbz)
        {
            log_aGstep.Out_joints_pos << count << "  " << 180./M_PI * WBIK.virt_jts_pos.transpose() << std::endl;
            log_aGstep.Out_joints_vel << count << "  " << WBIK.q_dot_star.transpose() << std::endl; 
            //
            log_aGstep.Out_stability_var << count << "  " << Bal_Pertubation.transpose() << "  " << WBIK.Pose_EE[4].head(3).transpose() << "  " << WBIK.W_Pos_AbsFoot.transpose() <<  "  ";  // count Perturbation CoM absFoot_in_W
            log_aGstep.Out_stability_var << WBIK.W_Rot_AbsFoot.row(0) << "  " << WBIK.W_Rot_AbsFoot.row(1) <<  "  ";
            log_aGstep.Out_stability_var << this->RelativeStep.head(3).transpose() << "  " << SStable << "  " << stance_left << "  " << endl; // delta step  static_stability 
            // convex hull
            if(WBIK.PointsCHull.rows() <= 6)
            {
                VectorXd first_pt_x = WBIK.PointsCHull(0,0) * VectorXd::Ones(7-WBIK.PointsCHull.rows());
                VectorXd first_pt_y = WBIK.PointsCHull(0,1) * VectorXd::Ones(7-WBIK.PointsCHull.rows());
                //
                log_aGstep.Out_PointsCHull   << WBIK.PointsCHull.col(0).transpose() << "  " << first_pt_x.transpose() << "  ";   // x
                log_aGstep.Out_PointsCHull   << WBIK.PointsCHull.col(1).transpose() << "  " << first_pt_y.transpose() << endl;   // y   
            }
            else
            {
                log_aGstep.Out_PointsCHull   << WBIK.PointsCHull.col(0).transpose() << "  ";     // x
                log_aGstep.Out_PointsCHull   << WBIK.PointsCHull.col(1).transpose() << endl;     // y
            }
        }

        
        // =====================================================================================================================================================================
        // ////////////////// DATA LOGGING /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        count ++;
        //
        cnt  = (count >= WBIK.count_max);
    }  

    AntiStep_Done = true;

    return this->RelativeStep;
}

// ======
bool FeedForwardController::getInitialGuess_batch(WbRobotModel& robot_model_, VectorXd jts_pos, VectorXd Wrench_hands_star_, Vector7d  des_X_EE_[], Vector7d pose_lfoot, Vector7d pose_rfoot)
{
    //
    std::cout<< "getInitialGuess_batch  started : " << 0.0 << std::endl;
    bool cnt    = false;
    bool n_loop = false;
    bool stbz   = false;
    //------------------------------------------------
    if(!InitGuess_batch_start)
    {
        l_count = 0;
        g_count = 0;
        ik_run  = false;
        // ============================================================================================================================
        //
        WBIK.virt_jts_pos = WBIK.q_0;
        WBIK.q_dot_star.setZero();
        WBIK.var_gain.setZero();
        // load the stepping solver parameters
        // memcpy(WBIK.task_weight, &WBIK.task_weight_aStep[0], 8*sizeof*WBIK.task_weight_aStep);
        memcpy(WBIK.task_weight, &WBIK.task_weight_fmmcom[0], 8*sizeof *WBIK.task_weight_fmmcom);
        // Matrix4d WHB = MatrixXd::Identity(4,4);
        for(int i=0; i<8; i++) WBIK.des_X_EE[i] = des_X_EE_[i];
        // estimate the components of the perturbation
        a_ctrl.compute_perturbation_terms2(Wrench_hands_star_, des_X_EE_[0].head(3), des_X_EE_[1].head(3), Vector3d(0.,0.,0.), Vector3d(0.,0.,0.), this->Disturb_c_, this->fu_z_mgbetaW_);
        Vector2d BalPert_      = Disturb_c_ + fu_z_mgbetaW_*0.5*(pose_lfoot.head(2) + pose_rfoot.head(2)) - 0.5*(pose_lfoot.head(2) + pose_rfoot.head(2));
        std::string stanceFoot = get_stance_foot(pose_lfoot, pose_rfoot, BalPert_);
        stance_ft = stanceFoot;

        std::cout<< " BATCH DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD: \t" << 0.0 << std::endl;
        std::cout<< " BATCH Disturb_c_   : \t" << Disturb_c_.transpose() << std::endl;
        std::cout<< " BATCH fu_z_mgbetaW_   : \t" << fu_z_mgbetaW_ << std::endl;
        std::cout<< " stance foot    : \t" << stanceFoot << std::endl;
            //
        WBIK.update_stepping_weight(stanceFoot);
        this->RelativeStep.setZero();               // Reset

        //
        if(stanceFoot == "left") {
            this->RelativeStep.tail(2) << 0.0, 1.0;
            WBIK.des_X_EE[3] = a_ctrl.getInitGuessFootReference(Disturb_c_, fu_z_mgbetaW_, pose_lfoot, pose_rfoot, stanceFoot);
        }
        else{
            this->RelativeStep.tail(2) << 1.0, 0.0;
            WBIK.des_X_EE[2] = a_ctrl.getInitGuessFootReference(Disturb_c_, fu_z_mgbetaW_, pose_lfoot, pose_rfoot, stanceFoot);
        }
        //
        InitGuess_Done          = false; 
        InitGuess_batch_start   = true;
        //
        std::cout<< "estimate_min_anticip_step   : \t" << 0.0 << std::endl;
    }
    // ======================================================================================================================================================
    //
    if(!ik_run && InitGuess_batch_start)
    {
        l_count = 0;
        while(!n_loop && !ik_run) // count_max
        {
            MatrixXd W_EdgesNormalCHull_; 
            VectorXd DistanceEdgesCHull_;
            //
            WBIK.solve_QPcvxgen(robot_model_, pose_lfoot, pose_rfoot, stance_ft, W_EdgesNormalCHull_, DistanceEdgesCHull_);     // TODO ADD SELF COLLISION AVOIDANCE CONSTRAINTS
            // specify the reference value of the CoM
            WBIK.des_X_EE[4].head(2) = 0.5*(WBIK.Pose_EE[2].head(2) + WBIK.Pose_EE[3].head(2));
            // clamping the CoM position to move freely but within the convex hull
            bool SStable = WBIK.predict_stability(W_EdgesNormalCHull_, DistanceEdgesCHull_, WBIK.Pose_EE[4].head(2), WBIK.W_Pos_AbsFoot);
            if(SStable) 
            {
                WBIK.des_X_EE[4](2) = WBIK.Pose_EE[4](2);
                WBIK.pxy_CoM_n1     = WBIK.pxy_CoM;
                WBIK.pxy_CoM        = (WBIK.Pose_EE[4].head(2) - WBIK.W_Pos_AbsFoot);
            }
            else 
            {
                Vector2d DeltaCoMxy      = (WBIK.pxy_CoM_n1.norm() - 0.03)/(WBIK.pxy_CoM_n1.norm()+1e-10) * WBIK.pxy_CoM_n1;
                WBIK.des_X_EE[4].head(2) = DeltaCoMxy + WBIK.W_Pos_AbsFoot;
                WBIK.des_X_EE[4](2)      = WBIK.Pose_EE[4](2);
            }
            // --------------------------------------------------
            n_loop = (l_count >= l_count_max);  
            cnt    = (g_count >= (WBIK.count_max-1));
            ik_run = cnt || WBIK.getTaskCompletionState();
            // --------------------------------------------------
            // // get the step value
            // this->RelativeStep.head(3) = this->get_DeltaStep(WBIK.des_X_EE[2], WBIK.des_X_EE[3], stance_ft);
            std::cout<< "INITGUESS BATCH LCOUNT : " << l_count << std::endl;
            std::cout<< "INITGUESS BATCH GCOUNT : " << g_count << std::endl;
            //
            // get the step value
            this->RelativeStep.head(3) = this->get_DeltaStep(WBIK.Pose_EE[2], WBIK.Pose_EE[3], stance_ft);
            //
            // ////////////////// DATA LOGGING /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // =================================================================================================================================================================
            Vector2d Bal_Pertubation = Disturb_c_ + fu_z_mgbetaW_*0.5*(pose_lfoot.head(2) + pose_rfoot.head(2));
            int stance_left = 0; 
            if(stance_ft =="left") stance_left = 1;

            log_iGuess.Out_joints_pos << g_count << "  " << 180./M_PI * WBIK.virt_jts_pos.transpose() << std::endl;
            log_iGuess.Out_joints_vel << g_count << "  " << WBIK.q_dot_star.transpose() << std::endl; 
            //
            log_iGuess.Out_stability_var << g_count << "  " << Bal_Pertubation.transpose() << "  " << WBIK.Pose_EE[4].head(3).transpose() << "  " << WBIK.W_Pos_AbsFoot.transpose() <<  "  ";  // count Perturbation CoM absFoot_in_W
            log_iGuess.Out_stability_var << WBIK.W_Rot_AbsFoot.row(0) << "  " << WBIK.W_Rot_AbsFoot.row(1) <<  "  ";
            log_iGuess.Out_stability_var << this->RelativeStep.head(3).transpose() << "  " << SStable << "  " << stance_left << "  " << endl; // delta step  static_stability 

            if(WBIK.PointsCHull.rows() <= 6)
            {
                VectorXd first_pt_x = WBIK.PointsCHull(0,0) * VectorXd::Ones(7-WBIK.PointsCHull.rows());
                VectorXd first_pt_y = WBIK.PointsCHull(0,1) * VectorXd::Ones(7-WBIK.PointsCHull.rows());
                //
                log_iGuess.Out_PointsCHull   << WBIK.PointsCHull.col(0).transpose() << "  " << first_pt_x.transpose() << "  ";   // x
                log_iGuess.Out_PointsCHull   << WBIK.PointsCHull.col(1).transpose() << "  " << first_pt_y.transpose() << endl;   // y   
            }
            else
            {
                log_iGuess.Out_PointsCHull   << WBIK.PointsCHull.col(0).transpose() << "  ";     // x
                log_iGuess.Out_PointsCHull   << WBIK.PointsCHull.col(1).transpose() << endl;     // y
            }
            // =====================================================================================================================================================================
            // ////////////////// DATA LOGGING /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //
            l_count ++; // local  counter
            g_count ++; // global counter 
        }
    }
    //
    if(ik_run || cnt)
    {
        //
        std::cout<< "WHILE LOOP DONE : " << g_count << std::endl;

        WBIK.des_jts_pos = WBIK.virt_jts_pos;
        WBIK.des_WHB      = WBIK.virt_WHB;
        // get the step value
        // this->RelativeStep.head(3) = this->get_DeltaStep(WBIK.des_X_EE[2], WBIK.des_X_EE[3], stance_ft);
        this->RelativeStep.head(3) = this->get_DeltaStep(WBIK.Pose_EE[2], WBIK.Pose_EE[3], stance_ft);
        // load the reachability parameters
        memcpy(WBIK.task_weight, &WBIK.task_weight_default[0], 8*sizeof*WBIK.task_weight_default);

        InitGuess_Done        = true;
        InitGuess_batch_start = false;

        cout << " IG BATCH Desired Joint position Torso are : \t" << 180./M_PI * WBIK.des_jts_pos.head(3).transpose() << endl;
        cout << " IG BATCH Desired Joint position Lhand are : \t" << 180./M_PI * WBIK.des_jts_pos.head(10).tail(7).transpose() << endl;
        cout << " IG BATCH Desired Joint position Rhand are : \t" << 180./M_PI * WBIK.des_jts_pos.head(17).tail(7).transpose() << endl;
        cout << " IG BATCH Desired Joint position Lleg  are : \t" << 180./M_PI * WBIK.des_jts_pos.tail(12).head(6).transpose() << endl;
        cout << " IG BATCH Desired Joint position Rleg  are : \t" << 180./M_PI * WBIK.des_jts_pos.tail(6).transpose() << endl;
        cout << " IG BATCH Desired Base transformation   is : \n" << WBIK.des_WHB << endl;
    }
    //
    return true;

}

// 
bool FeedForwardController::getGlobal_anticip_step_batch(   WbRobotModel& robot_model_, VectorXd jts_pos, VectorXd Wrench_hands_star_, 
                                                            Vector7d  des_X_EE_[], Vector7d pose_lfoot, Vector7d pose_rfoot, std::string stanceFoot)
{
    //
    std::cout<< "getInitialGuess_batch  started : " << 0.0 << std::endl;
    bool cnt    = false;
    bool n_loop = false;
    bool stbz   = false;
    // --------------------------------------------------------------------------------------------
    Eigen::Matrix3d WR_lf = a_ctrl.Transforms.PoseVector2HomogenousMx(pose_lfoot).block(0,0,3,3);
    Eigen::Matrix3d WR_rf = a_ctrl.Transforms.PoseVector2HomogenousMx(pose_rfoot).block(0,0,3,3);
    // Planar orientations of the feet 
    Vector3d o_lf = a_ctrl.Transforms.getEulerAnglesXYZ_FixedFrame(WR_lf);
    Vector3d o_rf = a_ctrl.Transforms.getEulerAnglesXYZ_FixedFrame(WR_rf);
    //    
    Vector2d StepFeet    = VectorXd::Zero(2);           // Initialization of the step length
    Matrix2d RotStanceFt = MatrixXd::Identity(2,2);     // Rotation of stance foot wrt the world

    //-------------------------------------------------------------------------------------------
    if(!aStepGlobal_batch_start)
    {
        l_count = 0;
        g_count = 0;
        ik_run  = false;
        // ========================================================================================================================================
        WBIK.update_stepping_weight(stanceFoot);
        // load the stepping solver parameters
        memcpy(WBIK.task_weight, &WBIK.task_weight_aStep[0], 8*sizeof*WBIK.task_weight_aStep); //
        // memcpy(WBIK.task_weight, &WBIK.task_weight_fmmcom[0], 8*sizeof *WBIK.task_weight_fmmcom);
        if(stanceFoot == "left")
        {
            WBIK.task_weight[2].head(3) = 10. * WBIK.task_weight_aStep[2].head(3);
            WBIK.task_weight[2].tail(3) = 10. * WBIK.task_weight_aStep[2].tail(3);
            WBIK.task_weight[3].head(3) = 0.3* (1./a_ctrl.lengthStep_max)  * WBIK.task_weight_aStep[3].head(3);
            WBIK.task_weight[3].tail(3) = 0.3* (1./a_ctrl.Dtheta_feet_max) * WBIK.task_weight_aStep[3].tail(3);
        }
        else  // right stance foot
        {
            WBIK.task_weight[2].head(3) = 0.3* (1./a_ctrl.lengthStep_max)  * WBIK.task_weight_aStep[2].head(3);
            WBIK.task_weight[2].tail(3) = 0.3* (1./a_ctrl.Dtheta_feet_max) * WBIK.task_weight_aStep[2].tail(3);
            WBIK.task_weight[3].head(3) = 10.* WBIK.task_weight_aStep[3].head(3);
            WBIK.task_weight[3].tail(3) = 10.* WBIK.task_weight_aStep[3].tail(3);
        }
        //
        // WBIK.virt_jts_pos = WBIK.q_0;
        WBIK.virt_jts_pos = jts_pos;
        WBIK.q_dot_star.setZero();
        WBIK.var_gain.setZero();
        // Matrix4d WHB = MatrixXd::Identity(4,4);
        for(int i=0; i<8; i++) WBIK.des_X_EE[i] = des_X_EE_[i];
        // estimate the components of the perturbation
        a_ctrl.compute_perturbation_terms2(Wrench_hands_star_, des_X_EE_[0].head(3), des_X_EE_[1].head(3), Vector3d(0.,0.,0.), Vector3d(0.,0.,0.), this->Disturb_c_, this->fu_z_mgbetaW_);
        Vector2d BalPert_      = Disturb_c_ + fu_z_mgbetaW_*0.5*(pose_lfoot.head(2) + pose_rfoot.head(2)) - 0.5*(pose_lfoot.head(2) + pose_rfoot.head(2));
        std::string stanceFoot = get_stance_foot(pose_lfoot, pose_rfoot, BalPert_);
        stance_ft = stanceFoot;
        //
        WBIK.update_stepping_weight(stanceFoot);
        this->RelativeStep.setZero();               // Reset
        //
        if(stanceFoot == "left") {
            this->RelativeStep.tail(2) << 0.0, 1.0;
            WBIK.des_X_EE[3] = a_ctrl.getInitGuessFootReference(Disturb_c_, fu_z_mgbetaW_, pose_lfoot, pose_rfoot, stanceFoot);
        }
        else{
            this->RelativeStep.tail(2) << 1.0, 0.0;
            WBIK.des_X_EE[2] = a_ctrl.getInitGuessFootReference(Disturb_c_, fu_z_mgbetaW_, pose_lfoot, pose_rfoot, stanceFoot);
        }
        //
        // estimate the balance perturbation 
        bool capturb = this->check_stabilizability( Wrench_hands_star_, WBIK.des_X_EE[0].head(3), WBIK.des_X_EE[1].head(3), pose_lfoot, pose_rfoot, 
                                                    WBIK.des_X_EE[4].head(3), Vector3d(0.,0.,0.), Vector3d(0.,0.,0.));
        // this->RelativeStep.setZero();
        // get the step value
        this->RelativeStep.head(3) = this->get_DeltaStep(WBIK.des_X_EE[2], WBIK.des_X_EE[3], stanceFoot);
        //
        std::cout<< " GGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG : \t" << std::endl;
        std::cout<< " CAPTURABILITY BEFORE  G : \t" << capturb << std::endl;   
        std::cout<< " RELATIVE STEP  G 00 : \t" << RelativeStep.transpose() << std::endl;
        std::cout<< " LEFT  FOOT POS : \t" << WBIK.Pose_EE[2].head(3).transpose() << std::endl;
        std::cout<< " RIGHT FOOT POS : \t" << WBIK.Pose_EE[3].head(3).transpose() << std::endl;
        //
        aStepGlobal_batch_start   = true;
        aStepGlobal_Done          = false; 
        WBIK.coef_grad = 1.0;
        //
        std::cout<< "estimate_min_anticip_step   : \t" << 0.0 << std::endl;
    }
    // =======================================================================================================================
    if(!ik_run && aStepGlobal_batch_start)
    {
        l_count = 0;
        //
        stbz = this->check_stabilizability(Wrench_hands_star_, WBIK.des_X_EE[0].head(3), WBIK.des_X_EE[1].head(3), 
                                                WBIK.Pose_EE[2], WBIK.Pose_EE[3], WBIK.Pose_EE[4].head(3), Vector3d(0.,0.,0.), Vector3d(0.,0.,0.));
        while(!n_loop && !ik_run && stbz) // count_max
        {
            //
            if(stanceFoot == "left") {
                this->RelativeStep.tail(2) << 0.0, 1.0;
                // WBIK.des_X_EE[3] = WBIK.des_X_EE[2]; 
                WBIK.des_X_EE[3] = pose_rfoot; 
                // computing the step length
                //--------------------------
                RotStanceFt << cos(o_lf(2)), -sin(o_lf(2)), sin(o_lf(2)), cos(o_lf(2));
                StepFeet = RotStanceFt.transpose()*(WBIK.Pose_EE[3].head(2) - WBIK.des_X_EE[2].head(2)) + Vector2d(0., a_ctrl.DyFeet_min);
            }
            else {
                this->RelativeStep.tail(2) << 1.0, 0.0;
                // WBIK.des_X_EE[2] = WBIK.des_X_EE[3];
                WBIK.des_X_EE[2] = pose_lfoot;
                // computing the step length
                //---------------------------
                RotStanceFt << cos(o_rf(2)), -sin(o_rf(2)), sin(o_rf(2)), cos(o_rf(2));
                StepFeet = RotStanceFt.transpose()*(WBIK.Pose_EE[2].head(2) - WBIK.des_X_EE[3].head(2)) - Vector2d(0., a_ctrl.DyFeet_min);
            }
            //
            // get the step value
            this->RelativeStep.head(3) = this->get_DeltaStep(WBIK.Pose_EE[2], WBIK.Pose_EE[3], stanceFoot);

            MatrixXd W_EdgesNormalCHull_; 
            VectorXd DistanceEdgesCHull_;
            // estimate the components of the perturbation
            a_ctrl.compute_perturbation_terms2(Wrench_hands_star_, des_X_EE_[0].head(3), des_X_EE_[1].head(3), Vector3d(0.,0.,0.), Vector3d(0.,0.,0.), this->Disturb_c_, this->fu_z_mgbetaW_);
            WBIK.solve_QPcvxgen4(robot_model_, pose_lfoot, pose_rfoot, stanceFoot, W_EdgesNormalCHull_, DistanceEdgesCHull_, Disturb_c_, fu_z_mgbetaW_);
            // specify the reference value of the CoM
            Vector2d Bal_Pertubation = Disturb_c_ + fu_z_mgbetaW_ * WBIK.Pose_EE[4].head(2);
            WBIK.des_X_EE[4].head(2) = -(Bal_Pertubation - 0.5*(WBIK.Pose_EE[2].head(2) + WBIK.Pose_EE[3].head(2)));
            // clamping the CoM position to move freely but within the convex hull
            bool SStable = WBIK.predict_stability(W_EdgesNormalCHull_, DistanceEdgesCHull_, WBIK.Pose_EE[4].head(2), WBIK.W_Pos_AbsFoot);
            if(SStable) 
            {
                WBIK.des_X_EE[4](2) = WBIK.Pose_EE[4](2);
                WBIK.pxy_CoM_n1     = WBIK.pxy_CoM;
                WBIK.pxy_CoM        = (WBIK.Pose_EE[4].head(2) - WBIK.W_Pos_AbsFoot);
            }
            else 
            {
                Vector2d DeltaCoMxy      = (WBIK.pxy_CoM_n1.norm() - 0.03)/(WBIK.pxy_CoM_n1.norm()+1e-10) * WBIK.pxy_CoM_n1;
                WBIK.des_X_EE[4].head(2) = DeltaCoMxy + WBIK.W_Pos_AbsFoot;
                WBIK.des_X_EE[4](2)      = WBIK.Pose_EE[4](2);
            }
            //
            // check the O-step capturability ------------------------------------------------------------------------------------------------------
            stbz = this->check_stabilizability(Wrench_hands_star_, WBIK.des_X_EE[0].head(3), WBIK.des_X_EE[1].head(3), 
                                                WBIK.Pose_EE[2], WBIK.Pose_EE[3], WBIK.Pose_EE[4].head(3), Vector3d(0.,0.,0.), Vector3d(0.,0.,0.));

            // ////////////////// DATA LOGGING /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // =================================================================================================================================================================
            int stance_left = 0; 
            if(stanceFoot =="left") stance_left = 1;

            if(stbz)
            {
                log_aGstep.Out_joints_pos << g_count << "  " << 180./M_PI * WBIK.virt_jts_pos.transpose() << std::endl;
                log_aGstep.Out_joints_vel << g_count << "  " << WBIK.q_dot_star.transpose() << std::endl; 
                //
                log_aGstep.Out_stability_var << g_count << "  " << Bal_Pertubation.transpose() << "  " << WBIK.Pose_EE[4].head(3).transpose() << "  " << WBIK.W_Pos_AbsFoot.transpose() <<  "  ";  // count Perturbation CoM absFoot_in_W
                log_aGstep.Out_stability_var << WBIK.W_Rot_AbsFoot.row(0) << "  " << WBIK.W_Rot_AbsFoot.row(1) <<  "  ";
                log_aGstep.Out_stability_var << this->RelativeStep.head(3).transpose() << "  " << SStable << "  " << stance_left << "  " << endl; // delta step  static_stability 
                // convex hull
                if(WBIK.PointsCHull.rows() <= 6)
                {
                    VectorXd first_pt_x = WBIK.PointsCHull(0,0) * VectorXd::Ones(7-WBIK.PointsCHull.rows());
                    VectorXd first_pt_y = WBIK.PointsCHull(0,1) * VectorXd::Ones(7-WBIK.PointsCHull.rows());
                    //
                    log_aGstep.Out_PointsCHull   << WBIK.PointsCHull.col(0).transpose() << "  " << first_pt_x.transpose() << "  ";   // x
                    log_aGstep.Out_PointsCHull   << WBIK.PointsCHull.col(1).transpose() << "  " << first_pt_y.transpose() << endl;   // y   
                }
                else
                {
                    log_aGstep.Out_PointsCHull   << WBIK.PointsCHull.col(0).transpose() << "  ";     // x
                    log_aGstep.Out_PointsCHull   << WBIK.PointsCHull.col(1).transpose() << endl;     // y
                }
            }
            // =====================================================================================================================================================================
            // ////////////////// DATA LOGGING /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // -------------------------------------------------
            n_loop = (l_count >= l_count_max);  
            cnt    = (g_count >= WBIK.count_max-1);
            ik_run = cnt; // || WBIK.getTaskCompletionState();
            // -------------------------------------------------
            std::cout<< "A-STEP GLOB BATCH LCOUNT : " << l_count << std::endl;
            std::cout<< "A-STEP GLOB BATCH GCOUNT : " << g_count << std::endl;
            //
            l_count ++; // local counter
            g_count ++; // global counter 
        }

        //
        if(ik_run || cnt || !stbz)
        {
            // get the step value
            this->RelativeStep.head(3) = this->get_DeltaStep(WBIK.Pose_EE[2], WBIK.Pose_EE[3], stanceFoot);
            // check is the step exceeds the the maximum step length between feet
            //===================================================================
            if(StepFeet.norm() >= a_ctrl.lengthStep_max)
            {
                this->RelativeStep.segment(3, 3) = this->get_DeltaStep(WBIK.Pose_EE[2], WBIK.Pose_EE[3], stanceFoot);
                this->RelativeStep.tail(2) << 1.0, 1.0;
            }
            // estimate the balance perturbation
            //==================================
            bool capt = this->check_stabilizability(Wrench_hands_star_, WBIK.des_X_EE[0].head(3), WBIK.des_X_EE[1].head(3), 
                                                    WBIK.Pose_EE[2], WBIK.Pose_EE[3], WBIK.Pose_EE[4].head(3), Vector3d(0.,0.,0.), Vector3d(0.,0.,0.));
            // stbz = this->check_stabilizability(Wrench_hands_star_, WBIK.des_X_EE[0].head(3), WBIK.des_X_EE[1].head(3), 
            //                                     WBIK.Pose_EE[2], WBIK.Pose_EE[3], WBIK.Pose_EE[4].head(3), Vector3d(0.,0.,0.), Vector3d(0.,0.,0.));
            // this->check_stabilizability(Wrench_hands_star_, WBIK.des_X_EE[0].head(3), WBIK.des_X_EE[1].head(3), 
            //                                     WBIK.des_X_EE[2], WBIK.des_X_EE[3], WBIK.des_X_EE[4].head(3), Vector3d(0.,0.,0.), Vector3d(0.,0.,0.));
            //=========================================
            WBIK.des_jts_pos  = WBIK.virt_jts_pos;
            WBIK.des_WHB      = WBIK.virt_WHB;
            //
            // reset_variables();
            aStepGlobal_batch_start = false; 
            aStepGlobal_Done        = true;

            // load the reachability parameters
            memcpy(WBIK.task_weight, &WBIK.task_weight_default[0], 8 * sizeof*WBIK.task_weight_default);

            std::cout<< "A-STEP GLOBAL YES : " << 1.0 << std::endl;

            cout << " BATCH Desired Joint position Torso are : \t" << 180./M_PI * WBIK.des_jts_pos.head(3).transpose() << endl;
            cout << " BATCH Desired Joint position Lhand are : \t" << 180./M_PI * WBIK.des_jts_pos.head(10).tail(7).transpose() << endl;
            cout << " BATCH Desired Joint position Rhand are : \t" << 180./M_PI * WBIK.des_jts_pos.head(17).tail(7).transpose() << endl;
            cout << " BATCH Desired Joint position Lleg  are : \t" << 180./M_PI * WBIK.des_jts_pos.tail(12).head(6).transpose() << endl;
            cout << " BATCH Desired Joint position Rleg  are : \t" << 180./M_PI * WBIK.des_jts_pos.tail(6).transpose() << endl;
            cout << " BATCH Desired Base transformation   is : \n" << WBIK.des_WHB << endl;

            for (int i=0; i<8; i++)
                std::cout << " ERROR POS and ORI NORM " << WBIK.EE[i] << "  : \t" << WBIK.error_pos_norm[i] << "  and : " << WBIK.error_ori_norm[i] << std::endl;

            // std::cout<< " COUNT GEN ANTICIP : \t" << count << std::endl;
            std::cout<< " BATCH stbz ///////////////////////////////////////////////////////////////////////////////// : " << capt << std::endl;
            std::cout<< " BATCH RELATIVE STEP ANTICIP : " << this->RelativeStep.transpose() << std::endl;
            std::cout<< " BATCH LEFT  FOOT POS : \t" << WBIK.Pose_EE[2].head(3).transpose() << std::endl;
            std::cout<< " BATCH RIGHT FOOT POS : \t" << WBIK.Pose_EE[3].head(3).transpose() << std::endl;
            std::cout<< " BATCH STANCE FOOT  G : \t" << stanceFoot << std::endl;
        }  
    }

    return true;
}

//
bool FeedForwardController::aStep_Global_batch(WbRobotModel& robot_model_, VectorXd jts_pos, VectorXd Wrench_hands_star_, Vector7d des_X_EE_[], 
                                                Vector7d pose_lfoot, Vector7d pose_rfoot)
{
    // 
    std::cout<< " ooooooooooooooooo aStep_Global_batch STARTED  =================> : " << 1.0 << std::endl; 
    std::cout<< " ooooooooooooooooo aStepGlobal_All_batch_start =================> : " << aStepGlobal_All_batch_start << std::endl; 
    std::cout<< " ooooooooooooooooo InitGuess_Done              =================> : " << InitGuess_Done << std::endl;
    std::cout<< " ooooooooooooooooo aStepGlobal_Done            =================> : " << aStepGlobal_Done << std::endl;  
    if(!aStepGlobal_All_batch_start)
    {
        InitGuess_Done              = false;
        aStepGlobal_Done            = false;
        aStepGlobal_All_batch_start = true;
    }

    if(aStepGlobal_All_batch_start)
    {
        if(!InitGuess_Done) // if(!InitGuess_Done && !aStepGlobal_Done)
        {
            // initial guess
            this->getInitialGuess_batch(robot_model_, jts_pos,  Wrench_hands_star_, des_X_EE_, pose_lfoot, pose_rfoot);

            std::cout<< " uuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuu InitGuess_Done              =================> : " << InitGuess_Done << std::endl;
        }
        else if(InitGuess_Done && !aStepGlobal_Done)
        {
            // global anticipatory step
            this->getGlobal_anticip_step_batch(robot_model_, this->WBIK.des_jts_pos, Wrench_hands_star_, this->WBIK.Pose_EE, pose_lfoot, pose_rfoot, this->stance_ft);
            // aStepGlobal_Done = true;
            AntiStep_Done = aStepGlobal_Done;
            std::cout<< " uuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuu aStepGlobal_Done            =================> : " << aStepGlobal_Done << std::endl;  
        }
        else if(InitGuess_Done && aStepGlobal_Done)
        {
            // ref_step        = this->RelativeStep;    // assign the value of the footstep
            // InitGuess_Done              = false; 
            // aStepGlobal_Done            = false; 
            aStepGlobal_All_batch_start = false;    
        }

    }
   
    return true;
}


bool FeedForwardController::aGstep_run_batch(WbRobotModel& robot_model_, VectorXd jts_pos, VectorXd Wrench_hands_star_, Vector7d des_X_EE_[], 
                                             Vector7d pose_lfoot, Vector7d pose_rfoot, string stanceFoot, bool &executing_step, VectorXd &ref_step)
{
    // 
    std::cout<< " run_batch STARTED  =================> : " << 1.0 << std::endl;  
    std::cout<< " isReachable        =================> : " << isReachable << std::endl;
    std::cout<< " Reach_Done         =================> : " << Reach_Done << std::endl;  
    std::cout<< " fmmCoM_Done        =================> : " << fmmCoM_Done << std::endl;  
    std::cout<< " isStable           =================> : " << isStable << std::endl;  
    std::cout<< " Stable_Done        =================> : " << Stable_Done << std::endl;  
    std::cout<< " AntiStep_Done      =================> : " << AntiStep_Done << std::endl; 

    // // reachability
    // if(!this->isReachable && !this->Reach_Done && !this->fmmCoM_Done && !this->AntiStep_Done)
    if(!this->Reach_Done)
    {
        this->isReachable = this->check_reachability_batch(robot_model_, jts_pos,  des_X_EE_, stanceFoot, pose_lfoot, pose_rfoot);
        //
        Reach_Done  = this->Reach_Done;
    } 

    // stabilizability
    // else if(this->isReachable && this->Reach_Done && !this->fmmCoM_Done && !this->AntiStep_Done)
    else if(this->isReachable && this->Reach_Done && !this->fmmCoM_Done)
    {
        this->get_FMMCoM_batch(robot_model_, this->WBIK.des_jts_pos, Wrench_hands_star_, des_X_EE_, pose_lfoot, pose_rfoot, stanceFoot);
        // this->get_FMMCoM_batch(robot_model_, jts_pos, Wrench_hands_star_, des_X_EE_, pose_lfoot, pose_rfoot, stanceFoot);
        //
        fmmCoM_Done = this->fmmCoM_Done;
        //
        if(fmmCoM_Done)
            fmmCoM = this->fmmCoM;
        std::cout<< " ==============================================> get_FMMCoM_batch =================> : " << 0.0 << std::endl;  
    }
    // //
    else if(this->fmmCoM_Done && !this->Stable_Done)
    {
        std::cout<< "check_stabilizability start =================> : " << 0.0 << std::endl;  

        this->isStable    = this->check_stabilizability(  Wrench_hands_star_, this->WBIK.Pose_EE[0].head(3), this->WBIK.Pose_EE[1].head(3), pose_lfoot, pose_rfoot, 
                                                            this->fmmCoM, Vector3d(0.,0.,0.), Vector3d(0.,0.,0.));
        //
        this->Stable_Done = true;

        std::cout<< "Stable_Done =================> : " << Stable_Done << std::endl;  
    }

    // compute the anticipatory
    else if(!this->isStable && this->Reach_Done && this->fmmCoM_Done && !this->AntiStep_Done) // (!this->isStable && this->Reach_Done && this->fmmCoM_Done && !this->AntiStep_Done) 
    {
        // std::cout<< "estimate_min_anticip_step_batch start =================> : " << 0.0 << std::endl;  
        //
        this->aStep_Global_batch(robot_model_, this->WBIK.des_jts_pos, Wrench_hands_star_, des_X_EE_, pose_lfoot, pose_rfoot);
        //
        // AntiStep_Done  = this->AntiStep_Done;
        std::cout<< "AntiStep_Done =================> : " << AntiStep_Done << std::endl;  
        //
        if(this->AntiStep_Done)
        {
            ref_step        = this->RelativeStep;    // assign the value of the footstep
            executing_step  = true;                    // activate the step exectution
            //
            this->isReachable     = false;
            this->isStable        = false;
            this->Reach_Done      = false;
            this->fmmCoM_Done     = false;
            this->Stable_Done     = false;
            this->AntiStep_Done   = false;
        }   
    }
    else if(this->isStable && this->Reach_Done && this->fmmCoM_Done && !this->AntiStep_Done) // set to Zero th step values
    {
        std::cout<< "estimate_min_anticip_step_batch no exe =================> : " << 1.0 << std::endl;
        // keep the step reference to zero
        ref_step = VectorXd::Zero(8); //setZero();
        //
        this->isReachable     = false;
        this->isStable        = false;
        this->Reach_Done      = false;
        this->fmmCoM_Done     = false;
        this->Stable_Done     = false;
        this->AntiStep_Done   = false;
        //
    }

    return true;
}


//
// bool FeedForwardController::aGstep_run_batch(WbRobotModel& robot_model_, VectorXd jts_pos, VectorXd Wrench_hands_star_, Vector7d des_X_EE_[], 
//                                              Vector7d pose_lfoot, Vector7d pose_rfoot, string stanceFoot, bool &executing_step, VectorXd &ref_step)
// {
//     // 
//     std::cout<< "run_batch STARTED  =================> : " << 1.0 << std::endl;  
//     std::cout<< "isReachable        =================> : " << isReachable << std::endl;
//     std::cout<< "Reach_Done         =================> : " << Reach_Done << std::endl;  
//     std::cout<< "fmmCoM_Done        =================> : " << fmmCoM_Done << std::endl;  
//     std::cout<< "isStable           =================> : " << isStable << std::endl;  
//     std::cout<< "AntiStep_Done      =================> : " << AntiStep_Done << std::endl; 

//     // // reachability
//     if(!this->isReachable && !this->Reach_Done && !this->fmmCoM_Done && !this->AntiStep_Done)
//     {
//         this->isReachable = this->check_reachability_batch(robot_model_, jts_pos,  des_X_EE_, stanceFoot, pose_lfoot, pose_rfoot);
//         //
//         Reach_Done  = this->Reach_Done;
//     } 

//     // stabilizability
//     else if(this->isReachable && this->Reach_Done && !this->fmmCoM_Done && !this->AntiStep_Done)
//     {
//         this->get_FMMCoM_batch(robot_model_, this->WBIK.des_jts_pos, Wrench_hands_star_, des_X_EE_, pose_lfoot, pose_rfoot, stanceFoot);
//         // this->get_FMMCoM_batch(robot_model_, jts_pos, Wrench_hands_star_, des_X_EE_, pose_lfoot, pose_rfoot, stanceFoot);
//         //
//         fmmCoM_Done = this->fmmCoM_Done;
//         //
//         if(fmmCoM_Done)
//             fmmCoM = this->fmmCoM;
//         std::cout<< " ==============================================> get_FMMCoM_batch =================> : " << 0.0 << std::endl;  
//     }
//     //
//     else if(this->fmmCoM_Done && !this->Stable_Done)
//     {
//         std::cout<< "check_stabilizability start =================> : " << 0.0 << std::endl;  

//         this->isStable    = this->check_stabilizability(  Wrench_hands_star_, this->WBIK.Pose_EE[0].head(3), this->WBIK.Pose_EE[1].head(3), pose_lfoot, pose_rfoot, 
//                                                             this->fmmCoM, Vector3d(0.,0.,0.), Vector3d(0.,0.,0.));
//         //
//         this->Stable_Done = true;

//         std::cout<< "Stable_Done =================> : " << Stable_Done << std::endl;  
//     }

//     // compute the anticipatory
//     else if(!this->isStable && this->Reach_Done && this->fmmCoM_Done && !this->AntiStep_Done) // (!this->isStable && this->Reach_Done && this->fmmCoM_Done && !this->AntiStep_Done) 
//     {
//         // std::cout<< "estimate_min_anticip_step_batch start =================> : " << 0.0 << std::endl;  
//         //
//         this->aStep_Global_batch(robot_model_, this->WBIK.des_jts_pos, Wrench_hands_star_, des_X_EE_, pose_lfoot, pose_rfoot);
//         //
//         // AntiStep_Done  = this->AntiStep_Done;
//         std::cout<< "AntiStep_Done =================> : " << AntiStep_Done << std::endl;  
//         //
//         if(this->AntiStep_Done)
//         {
//             ref_step        = this->RelativeStep;    // assign the value of the footstep
//             executing_step  = true;                    // activate the step exectution
//             //
//             this->Reach_Done      = false;
//             this->fmmCoM_Done     = false;
//             this->AntiStep_Done   = false;
//             this->Stable_Done     = false;

//             this->isStable        = false;
//             this->isReachable     = false;
//         }   
//     }
//     else if(this->isStable && this->Reach_Done && this->fmmCoM_Done && !this->AntiStep_Done) // set to Zero th step values
//     {
//         std::cout<< "estimate_min_anticip_step_batch no exe =================> : " << 1.0 << std::endl;
//         // keep the step reference to zero
//         ref_step = VectorXd::Zero(8); //setZero();
//         //
//         this->Reach_Done      = false;
//         this->fmmCoM_Done     = false;
//         this->AntiStep_Done   = false;
//         this->Stable_Done     = false;
//         //
//     }

//     return true;
// }


//
Vector2d FeedForwardController::generate_Com_reference(VectorXd Disturb_cx_, VectorXd Disturb_cy_, double fu_z_mgbeta, Vector3d ffmmCOM_, Vector3d ref_Cop) //double ftstep_x, double ftstep_y)
{
    // ====================================================================
    // ----------------------------------------------------------------
    VectorXd ftstepx_ = ref_Cop(0) * VectorXd::Ones(ComRefGen.nsp);
    VectorXd ftstepy_ = ref_Cop(1) * VectorXd::Ones(ComRefGen.nsp);

    this->ComRefGen.Disturb_cx_   = Disturb_cx_;
    this->ComRefGen.Disturb_cy_   = Disturb_cy_;
    this->ComRefGen.ftstepx_      = ftstepx_;             // Footstep position : reference CoP
    this->ComRefGen.ftstepy_      = ftstepy_;             // Footstep position : reference CoP
    this->ComRefGen.fu_z_mgbeta   = fu_z_mgbeta;          // Overall applied vertical force
    this->ComRefGen.ffmmcom_xy    = ffmmCOM_.head(2);
    // ----------------------------------------------------------------
    this->ComRefGen.run();
    // ====================================================================
    Vector2d CoM_; 
    CoM_(0) = this->ComRefGen.DMod.StatesX(0);
    CoM_(1) = this->ComRefGen.DMod.StatesY(0);

    return CoM_;
}