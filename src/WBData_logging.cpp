
#include "WBData_logging.h"

WBData_logging::WBData_logging()
{
    
}

WBData_logging::~WBData_logging()
{
    WBData_logging::Close_files();

    // if (Robot_Tasks) {
    //     delete Robot_Tasks;
    //     Robot_Tasks = 0;
    // }

    // if (Target_Object) {
    //     delete Target_Object;
    //     Target_Object = 0;
    // }

    // if (Relative_Errors) {
    //     delete Relative_Errors;
    //     Relative_Errors = 0;
    // }

    // if (Relative_velocities) {
    //     delete Relative_velocities;
    //     Relative_velocities = 0;
    // }

    // if (Global_velocities) {
    //     delete Global_velocities;
    //     Global_velocities = 0;
    // }

    // if (Homo_transforms) {
    //     delete Homo_transforms;
    //     Homo_transforms = 0;
    // }

    // if (Object_pose) {
    //     delete Object_pose;
    //     Object_pose = 0;
    // }

}

void WBData_logging::InitializeLogger(std::string count)
{
    
    cout << " SIM TIME  0 is: \n "<< endl;

    logRobotTasks               = "./Robot_Tasks" + count + ".txt";
    logTargetObject             = "./Target_Object" + count + ".txt";
    logRelativeErrors           = "./Relative_Errors" + count + ".txt";
    logRelativeVelocities       = "./Relative_velocities" + count + ".txt";
    logGlobalVelocities         = "./Global_velocities" + count + ".txt";
    logHomogeneousTransforms    = "./Homo_transforms" + count + ".txt";
    logObjectPose               = "./Object_pose" + count + ".txt";

    // Robot_Tasks                 = new std::ofstream(logRobotTasks.c_str());
    // Target_Object               = new std::ofstream(logTargetObject.c_str());
    // Relative_Errors             = new std::ofstream(logRelativeErrors.c_str());
    // Relative_velocities         = new std::ofstream(logRelativeVelocities.c_str());
    // Global_velocities           = new std::ofstream(logGlobalVelocities.c_str());
    // Homo_transforms             = new std::ofstream(logHomogeneousTransforms.c_str());
    // Object_pose                 = new std::ofstream(logObjectPose.c_str());

    Robot_Tasks.open(logRobotTasks.c_str());
    Target_Object.open(logTargetObject.c_str());
    Relative_Errors.open(logRelativeErrors.c_str());
    Relative_velocities.open(logRelativeVelocities.c_str());
    Global_velocities.open(logGlobalVelocities.c_str());
    Homo_transforms.open(logHomogeneousTransforms.c_str());
    Object_pose.open(logObjectPose.c_str());

    SimTime = 0.0;
}


void WBData_logging::Write_Data(double SamplingTime,
                                int Cycle_counter,
                                int Ne,
                                // Coordination
                                // =============
                                Vector6d error_approach[],
                                Vector6d error_aperture[],
                                Vector6d error_coordin[],

                                Vector6d w_velocity_approach[],
                                Vector6d w_velocity_aperture[],
                                Vector6d w_velocity_coordin[],

                                Vector6d w_velocity_ro_gpoint[],
                                Vector6d w_velocity_uvo_gpoint[],
                                Vector6d w_velocity_vo_gpoint[], 
                                Vector6d w_velocity_eef[],

                                Matrix4d w_H_Obj,
                                Matrix4d w_H_absE,
                                Matrix4d w_H_vstar[],
                                Matrix4d w_H_v[],
                                Matrix4d w_H_h[],
                                Matrix4d w_H_cp[],

                                // Robot
                                // ======
                                // poses
                                Vector7d w_Pose_lh_,
                                Vector7d w_Pose_rh_,
                                Vector7d w_Pose_lf_,
                                Vector7d w_Pose_rf_,
                                Vector7d w_Pose_Base_,
                                Vector7d w_Pose_CoM_,
                                // wrenches
                                // =============
                                // computed
                                Vector6d gsp_lh_Wrench,
                                Vector6d gsp_rh_Wrench,
                                Vector6d wbd_lf_Wrench,
                                Vector6d wbd_rf_Wrench,
                                // compensated
                                Eigen::Vector3d cmp_lh_Moment,
                                Eigen::Vector3d cmp_rh_Moment,
                                // Applied
                                Vector6d appl_lh_Wrench,
                                Vector6d appl_rh_Wrench,
                                // Estimated
                                Vector6d Est_lh_Wrench,
                                Vector6d Est_rh_Wrench,
                                Vector6d Est_lf_Wrench,
                                Vector6d Est_rf_Wrench,

                                // Target Object
                                // =============
                                Vector7d object_pose_,
                                Vector7d d_obj_pose_,
                                Vector6d d_obj_velo
                                )
{

    // Write the data
    SimTime = SamplingTime * Cycle_counter;

    // --------------------------------------
    // ===========================================
    // Robot
    // ============================================
    Robot_Tasks  << SimTime <<"    " ;
    // ==========
    // Pose
    // ==========
    // left hand
    Robot_Tasks  << w_Pose_lh_(0) <<"    "<< w_Pose_lh_(1) <<"    "<< w_Pose_lh_(2) <<"    "<< w_Pose_lh_(3) <<"    "<< w_Pose_lh_(4) <<"    "<< w_Pose_lh_(5) <<"    "<< w_Pose_lh_(6) <<"    ";
    // right hand
    Robot_Tasks  << w_Pose_rh_(0) <<"    "<< w_Pose_rh_(1) <<"    "<< w_Pose_rh_(2) <<"    "<< w_Pose_rh_(3) <<"    "<< w_Pose_rh_(4) <<"    "<< w_Pose_rh_(5) <<"    "<< w_Pose_rh_(6) <<"    ";
    // left foot
    Robot_Tasks  << w_Pose_lf_(0) <<"    "<< w_Pose_lf_(1) <<"    "<< w_Pose_lf_(2) <<"    "<< w_Pose_lf_(3) <<"    "<< w_Pose_lf_(4) <<"    "<< w_Pose_lf_(5) <<"    "<< w_Pose_lf_(6) <<"    ";
    // right foot
    Robot_Tasks  << w_Pose_lf_(0) <<"    "<< w_Pose_lf_(1) <<"    "<< w_Pose_lf_(2) <<"    "<< w_Pose_lf_(3) <<"    "<< w_Pose_lf_(4) <<"    "<< w_Pose_lf_(5) <<"    "<< w_Pose_lf_(6) <<"    ";
    // Base
    Robot_Tasks  << w_Pose_Base_(0) <<"    "<< w_Pose_Base_(1) <<"    "<< w_Pose_Base_(2) <<"    "<< w_Pose_Base_(3) <<"    "<< w_Pose_Base_(4) <<"    "<< w_Pose_Base_(5) <<"    "<< w_Pose_Base_(6) <<"    ";
    // CoM
    Robot_Tasks  << w_Pose_CoM_(0) <<"    "<< w_Pose_CoM_(1) <<"    "<< w_Pose_CoM_(2) <<"    "<< w_Pose_CoM_(3) <<"    "<< w_Pose_CoM_(4) <<"    "<< w_Pose_CoM_(5) <<"    "<< w_Pose_CoM_(6) <<"    ";

    // ==========
    // Wrenches
    // ==========
    // Computed
    // ---------
    // computed left hand grasping task
    Robot_Tasks  << gsp_lh_Wrench(0) <<"    "<< gsp_lh_Wrench(1) <<"    "<< gsp_lh_Wrench(2) <<"    "<< gsp_lh_Wrench(3) <<"    "<< gsp_lh_Wrench(4) <<"    "<< gsp_lh_Wrench(5) <<"    ";
    // computed right hand;
    Robot_Tasks  << gsp_rh_Wrench(0) <<"    "<< gsp_rh_Wrench(1) <<"    "<< gsp_rh_Wrench(2) <<"    "<< gsp_rh_Wrench(3) <<"    "<< gsp_rh_Wrench(4) <<"    "<< gsp_rh_Wrench(5) <<"    ";
    // Compensated left hand momemnts
    Robot_Tasks  << wbd_lf_Wrench(0) <<"    "<< wbd_lf_Wrench(1) <<"    "<< wbd_lf_Wrench(2) <<"    "<< wbd_lf_Wrench(3) <<"    "<< wbd_lf_Wrench(4) <<"    "<< wbd_lf_Wrench(5) <<"    ";
    // Compensated right hand moments
    Robot_Tasks  << wbd_rf_Wrench(0) <<"    "<< wbd_rf_Wrench(1) <<"    "<< wbd_rf_Wrench(2) <<"    "<< wbd_rf_Wrench(3) <<"    "<< wbd_rf_Wrench(4) <<"    "<< wbd_rf_Wrench(5) <<"    ";
    
    // Estimated left hand Estimated
    Robot_Tasks  << cmp_lh_Moment(0) <<"    "<< cmp_lh_Moment(1) <<"    "<< cmp_lh_Moment(2) <<"    "; 
    // Estimated right hand Estimated
    Robot_Tasks  << cmp_rh_Moment(0) <<"    "<< cmp_rh_Moment(1) <<"    "<< cmp_rh_Moment(2) <<"    "; 
    // applied left hand grasping task
    Robot_Tasks  << appl_lh_Wrench(0) <<"    "<< appl_lh_Wrench(1) <<"    "<< appl_lh_Wrench(2) <<"    "<< appl_lh_Wrench(3) <<"    "<< appl_lh_Wrench(4) <<"    "<< appl_lh_Wrench(5) <<"    ";
    // applied right hand
    Robot_Tasks  << appl_rh_Wrench(0) <<"    "<< appl_rh_Wrench(1) <<"    "<< appl_rh_Wrench(2) <<"    "<< appl_rh_Wrench(3) <<"    "<< appl_rh_Wrench(4) <<"    "<< appl_rh_Wrench(5) <<"    ";

    
    //Estimated
    // ---------
    // left foot computed
    Robot_Tasks  << Est_lh_Wrench(0) <<"    "<< Est_lh_Wrench(1) <<"    "<< Est_lh_Wrench(2) <<"    "<< Est_lh_Wrench(3) <<"    "<< Est_lh_Wrench(4) <<"    "<< Est_lh_Wrench(5) <<"    ";
    // right foot computed
    Robot_Tasks  << Est_rh_Wrench(0) <<"    "<< Est_rh_Wrench(1) <<"    "<< Est_rh_Wrench(2) <<"    "<< Est_rh_Wrench(3) <<"    "<< Est_rh_Wrench(4) <<"    "<< Est_rh_Wrench(5) <<"    ";
    // left foot Estimated
    Robot_Tasks  << Est_lf_Wrench(0) <<"    "<< Est_lf_Wrench(1) <<"    "<< Est_lf_Wrench(2) <<"    "<< Est_lf_Wrench(3) <<"    "<< Est_lf_Wrench(4) <<"    "<< Est_lf_Wrench(5) <<"    ";
    // right foot Estimated
    Robot_Tasks  << Est_rf_Wrench(0) <<"    "<< Est_rf_Wrench(1) <<"    "<< Est_rf_Wrench(2) <<"    "<< Est_rf_Wrench(3) <<"    "<< Est_rf_Wrench(4) <<"    "<< Est_rf_Wrench(5) <<"    ";

    Robot_Tasks  << std::endl;

    // ============================================
    // Object 
    // ============================================
    Target_Object << SimTime <<"    " ;
    // real pose
    Target_Object  << object_pose_(0) <<"    "<< object_pose_(1) <<"    "<< object_pose_(2) <<"    "<< object_pose_(3) <<"    "<< object_pose_(4) <<"    "<< object_pose_(5) <<"    "<< object_pose_(6) <<"    ";
    // left grasping point
    Target_Object  << d_obj_pose_(0) <<"    "<< d_obj_pose_(1) <<"    "<< d_obj_pose_(2) <<"    "<< d_obj_pose_(3) <<"    "<< d_obj_pose_(4) <<"    "<< d_obj_pose_(5) <<"    "<< d_obj_pose_(6) <<"    ";
    // right grasping point
    Target_Object  << d_obj_velo(0) <<"    "<< d_obj_velo(1) <<"    "<< d_obj_velo(2) <<"    "<< d_obj_velo(3) <<"    "<< d_obj_velo(4) <<"    "<< d_obj_velo(5) <<"    ";

    Target_Object  << std::endl;

    // Relative errors
    //=================
    Relative_Errors << SimTime <<"    " ;
        
    // left hands
    // error approach
    Relative_Errors << error_approach[0](0) <<"    "<< error_approach[0](1) <<"    "<< error_approach[0](2) <<"    "<< error_approach[0](3) <<"    "<< error_approach[0](4) <<"    "<< error_approach[0](5) <<"    ";
    // error pregrasp
    Relative_Errors << error_aperture[0](0) <<"    "<< error_aperture[0](1) <<"    "<< error_aperture[0](2) <<"    "<< error_aperture[0](3) <<"    "<< error_aperture[0](4) <<"    "<< error_aperture[0](5) <<"    ";
    // error grasp
    Relative_Errors << error_coordin[0](0) <<"   "<< error_coordin[0](1) <<"   "<< error_coordin[0](2) <<"   "<< error_coordin[0](3) <<"   "<< error_coordin[0](4) <<"   "<< error_coordin[0](5) <<"   ";

    // right hands
    // error approach
    Relative_Errors << error_approach[1](0) <<"    "<< error_approach[1](1) <<"    "<< error_approach[1](2) <<"    "<< error_approach[1](3) <<"    "<< error_approach[1](4) <<"    "<< error_approach[1](5) <<"    ";
    // error pregrasp
    Relative_Errors << error_aperture[1](0) <<"    "<< error_aperture[1](1) <<"    "<< error_aperture[1](2) <<"    "<< error_aperture[1](3) <<"    "<< error_aperture[1](4) <<"    "<< error_aperture[1](5) <<"    ";
    // error grasp
    Relative_Errors << error_coordin[1](0) <<"   "<< error_coordin[1](1) <<"   "<< error_coordin[1](2) <<"   "<< error_coordin[1](3) <<"   "<< error_coordin[1](4) <<"   "<< error_coordin[1](5) <<"   ";

    Relative_Errors << std::endl;

    // Relative velocities
    //=================
   Relative_velocities << SimTime <<"    " ;

        // left hands
        // Velocity approach
        Relative_velocities << w_velocity_approach[0](0) <<"   "<< w_velocity_approach[0](1) <<"   "<< w_velocity_approach[0](2) <<"   "<< w_velocity_approach[0](3) <<"   "<< w_velocity_approach[0](4) <<"   "<< w_velocity_approach[0](5) <<"   ";
        // Velocity pregrasp
        Relative_velocities << w_velocity_aperture[0](0) <<"   "<< w_velocity_aperture[0](1) <<"   "<< w_velocity_aperture[0](2) <<"   "<< w_velocity_aperture[0](3) <<"   "<< w_velocity_aperture[0](4) <<"   "<< w_velocity_aperture[0](5) <<"   ";
        // Velocity grasp
        Relative_velocities << w_velocity_coordin[0](0) <<"  "<< w_velocity_coordin[0](1) <<"  "<< w_velocity_coordin[0](2) <<"  "<< w_velocity_coordin[0](3) <<"  "<< w_velocity_coordin[0](4) <<"  "<< w_velocity_coordin[0](5) <<"  ";

        // right hands
        // Velocity approach
        Relative_velocities << w_velocity_approach[1](0) <<"   "<< w_velocity_approach[1](1) <<"   "<< w_velocity_approach[1](2) <<"   "<< w_velocity_approach[1](3) <<"   "<< w_velocity_approach[1](4) <<"   "<< w_velocity_approach[1](5) <<"   ";
        // Velocity pregrasp
        Relative_velocities << w_velocity_aperture[1](0) <<"   "<< w_velocity_aperture[1](1) <<"   "<< w_velocity_aperture[1](2) <<"   "<< w_velocity_aperture[1](3) <<"   "<< w_velocity_aperture[1](4) <<"   "<< w_velocity_aperture[1](5) <<"   ";
        // Velocity grasp
        Relative_velocities << w_velocity_coordin[1](0) <<"  "<< w_velocity_coordin[1](1) <<"  "<< w_velocity_coordin[1](2) <<"  "<< w_velocity_coordin[1](3) <<"  "<< w_velocity_coordin[1](4) <<"  "<< w_velocity_coordin[1](5) <<"  ";

        Relative_velocities << std::endl;

    // Global Velocities
    //==================
    Global_velocities << SimTime <<"    " ;

        // left hands
        // Velocity real object grasping points
        Global_velocities << w_velocity_ro_gpoint[0](0) <<"    "<< w_velocity_ro_gpoint[0](1) <<"  "<< w_velocity_ro_gpoint[0](2) <<"  "<< w_velocity_ro_gpoint[0](3) <<"  "<< w_velocity_ro_gpoint[0](4) <<"  "<< w_velocity_ro_gpoint[0](5) <<"  ";
        // Velocity grasping point uscaled virtual object
        Global_velocities << w_velocity_uvo_gpoint[0](0) <<"   "<< w_velocity_uvo_gpoint[0](1) <<" "<< w_velocity_uvo_gpoint[0](2) <<" "<< w_velocity_uvo_gpoint[0](3) <<" "<< w_velocity_uvo_gpoint[0](4) <<" "<< w_velocity_uvo_gpoint[0](5) <<" ";
        // Velocity grasp
        Global_velocities << w_velocity_vo_gpoint[0](0) <<"    "<< w_velocity_vo_gpoint[0](1) <<"  "<< w_velocity_vo_gpoint[0](2) <<"  "<< w_velocity_vo_gpoint[0](3) <<"  "<< w_velocity_vo_gpoint[0](4) <<"  "<< w_velocity_vo_gpoint[0](5) <<"  ";
        // velocity hands end-effectors
        Global_velocities << w_velocity_eef[0](0) <<"  "<< w_velocity_eef[0](1) <<"    "<< w_velocity_eef[0](2) <<"    "<< w_velocity_eef[0](3) <<"    "<< w_velocity_eef[0](4) <<"    "<< w_velocity_eef[0](5) <<"    ";

        // right hands
        // Velocity real object grasping points
        Global_velocities << w_velocity_ro_gpoint[1](0) <<"    "<< w_velocity_ro_gpoint[1](1) <<"  "<< w_velocity_ro_gpoint[1](2) <<"  "<< w_velocity_ro_gpoint[1](3) <<"  "<< w_velocity_ro_gpoint[1](4) <<"  "<< w_velocity_ro_gpoint[1](5) <<"  ";
        // Velocity grasping point uscaled virtual object
        Global_velocities << w_velocity_uvo_gpoint[1](0) <<"   "<< w_velocity_uvo_gpoint[1](1) <<" "<< w_velocity_uvo_gpoint[1](2) <<" "<< w_velocity_uvo_gpoint[1](3) <<" "<< w_velocity_uvo_gpoint[1](4) <<" "<< w_velocity_uvo_gpoint[1](5) <<" ";
        // Velocity grasp
        Global_velocities << w_velocity_vo_gpoint[1](0) <<"    "<< w_velocity_vo_gpoint[1](1) <<"  "<< w_velocity_vo_gpoint[1](2) <<"  "<< w_velocity_vo_gpoint[1](3) <<"  "<< w_velocity_vo_gpoint[1](4) <<"  "<< w_velocity_vo_gpoint[1](5) <<"  ";
        // velocity hands end-effectors
        Global_velocities << w_velocity_eef[1](0) <<"  "<< w_velocity_eef[1](1) <<"    "<< w_velocity_eef[1](2) <<"    "<< w_velocity_eef[1](3) <<"    "<< w_velocity_eef[1](4) <<"    "<< w_velocity_eef[1](5) <<"    ";

        Global_velocities << std::endl;


    // Homogeneous transforms
    // ======================
    Homo_transforms << SimTime <<"    " ;

        // left hands
        // end-effectors hmg_transforms
        Homo_transforms << w_H_h[0](0,0) <<"   "<< w_H_h[0](0,1) <<"   "<< w_H_h[0](0,2) <<"   "<< w_H_h[0](0,3) <<"   ";
        Homo_transforms << w_H_h[0](1,0) <<"   "<< w_H_h[0](1,1) <<"   "<< w_H_h[0](1,2) <<"   "<< w_H_h[0](1,3) <<"   ";
        Homo_transforms << w_H_h[0](2,0) <<"   "<< w_H_h[0](2,1) <<"   "<< w_H_h[0](2,2) <<"   "<< w_H_h[0](2,3) <<"   ";

        // virtual object grasp points hmg_transforms
        Homo_transforms << w_H_v[0](0,0) <<"   "<< w_H_v[0](0,1) <<"   "<< w_H_v[0](0,2) <<"   "<< w_H_v[0](0,3) <<"   ";
        Homo_transforms << w_H_v[0](1,0) <<"   "<< w_H_v[0](1,1) <<"   "<< w_H_v[0](1,2) <<"   "<< w_H_v[0](1,3) <<"   ";
        Homo_transforms << w_H_v[0](2,0) <<"   "<< w_H_v[0](2,1) <<"   "<< w_H_v[0](2,2) <<"   "<< w_H_v[0](2,3) <<"   ";

        // unscaled virtual object grasp points hmg_transforms
        Homo_transforms << w_H_vstar[0](0,0) <<"   "<< w_H_vstar[0](0,1) <<"   "<< w_H_vstar[0](0,2) <<"   "<< w_H_vstar[0](0,3) <<"   ";
        Homo_transforms << w_H_vstar[0](1,0) <<"   "<< w_H_vstar[0](1,1) <<"   "<< w_H_vstar[0](1,2) <<"   "<< w_H_vstar[0](1,3) <<"   ";
        Homo_transforms << w_H_vstar[0](2,0) <<"   "<< w_H_vstar[0](2,1) <<"   "<< w_H_vstar[0](2,2) <<"   "<< w_H_vstar[0](2,3) <<"   ";

        // right hands
        // end-effectors hmg_transforms
        Homo_transforms << w_H_h[1](0,0) <<"   "<< w_H_h[1](0,1) <<"   "<< w_H_h[1](0,2) <<"   "<< w_H_h[1](0,3) <<"   ";
        Homo_transforms << w_H_h[1](1,0) <<"   "<< w_H_h[1](1,1) <<"   "<< w_H_h[1](1,2) <<"   "<< w_H_h[1](1,3) <<"   ";
        Homo_transforms << w_H_h[1](2,0) <<"   "<< w_H_h[1](2,1) <<"   "<< w_H_h[1](2,2) <<"   "<< w_H_h[1](2,3) <<"   ";

        // virtual object grasp points hmg_transforms
        Homo_transforms << w_H_v[1](0,0) <<"   "<< w_H_v[1](0,1) <<"   "<< w_H_v[1](0,2) <<"   "<< w_H_v[1](0,3) <<"   ";
        Homo_transforms << w_H_v[1](1,0) <<"   "<< w_H_v[1](1,1) <<"   "<< w_H_v[1](1,2) <<"   "<< w_H_v[1](1,3) <<"   ";
        Homo_transforms << w_H_v[1](2,0) <<"   "<< w_H_v[1](2,1) <<"   "<< w_H_v[1](2,2) <<"   "<< w_H_v[1](2,3) <<"   ";

        // unscaled virtual object grasp points hmg_transforms
        Homo_transforms << w_H_vstar[1](0,0) <<"   "<< w_H_vstar[1](0,1) <<"   "<< w_H_vstar[1](0,2) <<"   "<< w_H_vstar[1](0,3) <<"   ";
        Homo_transforms << w_H_vstar[1](1,0) <<"   "<< w_H_vstar[1](1,1) <<"   "<< w_H_vstar[1](1,2) <<"   "<< w_H_vstar[1](1,3) <<"   ";
        Homo_transforms << w_H_vstar[1](2,0) <<"   "<< w_H_vstar[1](2,1) <<"   "<< w_H_vstar[1](2,2) <<"   "<< w_H_vstar[1](2,3) <<"   ";


        // end-effectors hmg_transforms
        Homo_transforms << w_H_absE(0,0) <<"    "<< w_H_absE(0,1) <<"    "<< w_H_absE(0,2) <<"    "<< w_H_absE(0,3) <<"    ";
        Homo_transforms << w_H_absE(1,0) <<"    "<< w_H_absE(1,1) <<"    "<< w_H_absE(1,2) <<"    "<< w_H_absE(1,3) <<"    ";
        Homo_transforms << w_H_absE(2,0) <<"    "<< w_H_absE(2,1) <<"    "<< w_H_absE(2,2) <<"    "<< w_H_absE(2,3) <<"    ";

        // end-effectors hmg_transforms
        Homo_transforms << w_H_Obj(0,0) <<"    "<< w_H_Obj(0,1) <<"    "<< w_H_Obj(0,2) <<"    "<< w_H_Obj(0,3) <<"    ";
        Homo_transforms << w_H_Obj(1,0) <<"    "<< w_H_Obj(1,1) <<"    "<< w_H_Obj(1,2) <<"    "<< w_H_Obj(1,3) <<"    ";
        Homo_transforms << w_H_Obj(2,0) <<"    "<< w_H_Obj(2,1) <<"    "<< w_H_Obj(2,2) <<"    "<< w_H_Obj(2,3) <<"    ";

    Homo_transforms << std::endl;

    // Object Pose
    // ======================
    Object_pose << SimTime <<"    " ;

    // left hands
        // end-effectors hmg_transforms
        Object_pose << w_H_cp[0](0,0) <<"   "<< w_H_cp[0](0,1) <<"   "<< w_H_cp[0](0,2) <<"   "<< w_H_cp[0](0,3) <<"   ";
        Object_pose << w_H_cp[0](1,0) <<"   "<< w_H_cp[0](1,1) <<"   "<< w_H_cp[0](1,2) <<"   "<< w_H_cp[0](1,3) <<"   ";
        Object_pose << w_H_cp[0](2,0) <<"   "<< w_H_cp[0](2,1) <<"   "<< w_H_cp[0](2,2) <<"   "<< w_H_cp[0](2,3) <<"   ";

    // right hands
        // end-effectors hmg_transforms
        Object_pose << w_H_cp[1](0,0) <<"   "<< w_H_cp[1](0,1) <<"   "<< w_H_cp[1](0,2) <<"   "<< w_H_cp[1](0,3) <<"   ";
        Object_pose << w_H_cp[1](1,0) <<"   "<< w_H_cp[1](1,1) <<"   "<< w_H_cp[1](1,2) <<"   "<< w_H_cp[1](1,3) <<"   ";
        Object_pose << w_H_cp[1](2,0) <<"   "<< w_H_cp[1](2,1) <<"   "<< w_H_cp[1](2,2) <<"   "<< w_H_cp[1](2,3) <<"   ";

    Object_pose << std::endl;

}

void WBData_logging::Close_files()
{
    // close the files
    Robot_Tasks.close();
    Target_Object.close();
    Relative_Errors.close();
    Relative_velocities.close();
    Global_velocities.close();
    Homo_transforms.close();
    Object_pose.close();

}
