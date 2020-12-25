#ifndef WlkData_logging_H
#define WlkData_logging_H

#include <iostream>
#include <fstream>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <yarp/sig/Vector.h>

#include "PatternsGenerator.h"


using namespace std;
// 
class WlkData_logging
{

    public:

        double SimTime;

        std::string logPatterns;
        std::string logFeetTrajectories;
        std::string logJointsPosValues;
        std::string logFeetForceTorques;

        // std::ofstream flog_Pattern(const char *fileName);
        // std::ofstream flog_FeetTraj(const char *fileName);
        // std::ofstream flog_JtsPos(const char *fileName);
        // std::ofstream flog_F_FT(const char *fileName);

        std::ofstream *flog_Pattern;
        std::ofstream *flog_FeetTraj;
        std::ofstream *flog_JtsPos;
        std::ofstream *flog_F_FT;


        WlkData_logging();

        ~WlkData_logging();

        void InitializeLogger();

        void Write_Data(double SamplingTime,
                        int Cycle_counter,
                        CpStanceFootPose CoPref,
                        Discrete_CP_LIP_Model DMod,
                        CpFootTrajectories FtTraj,
                        VelocitiesSetPoints VeloRef,
                        VectorXd left_commands,
                        VectorXd right_commands,
                        VectorXd left_encoders,
                        VectorXd right_encoders,
                        VectorXd left_Foot_FT,
                        VectorXd right_Foot_FT,
                        VectorXd CoM_position);

        void Close_files();
};

 
#endif // WlkData_logging_H
