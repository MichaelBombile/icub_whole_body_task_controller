/* 
 * Copyright (C) 2017 Learning Algorithms and Systems Laboratory (LASA)
 * Author: Michael Bombile
 * email:  michael.bombile@epfl.ch
 * website: lasa.epfl.ch
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef CpDesiredFeetTransformations_H
#define CpDesiredFeetTransformations_H

#include <math.h>
#include <Eigen/Dense>

#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>


#include "CpGaitTransformations.h"
//#include "CpStanceFootPose.h"
//#include "CpFootTrajectories.h"
#include "TemplateModels.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;




using namespace std;
using namespace Eigen;

class CpDesiredFeetTransformations
{
    

    
        VectorXd TrslBaseCoM;      // Translation vector from base to CoM

		public :

        MatrixXd DesTrsfRealStanceFootInHorBase;
        MatrixXd CurTrsfRealStanceFootInHorBase;
        MatrixXd ReferenceTrsfHorFootInHorBase;
        MatrixXd TrsfCurBaseInWorldIMU;
                  
        MatrixXd FwKRightFootInBase;
        MatrixXd FwKLeftFootInBase;
        
        Matrix3d CurRotRealStFootInHorBase;
        Matrix3d RotRealStFootInHorStFoot;
        
        MatrixXd ReferenceTrsfSwingFootInHorBase;
        MatrixXd DesTrsfSwingFootInCurBase;

        MatrixXd DesTrsfLeftFootInHorBase;
        MatrixXd DesTrsfRightFootInHorBase;
        
        // Methods
          
        CpDesiredFeetTransformations();

        ~CpDesiredFeetTransformations();
        
        void getDesiredFeetInCorrectedBase(int SptFt[],
                                           yarp::sig::Vector LLegPose,
                                           yarp::sig::Vector RLegPose,
                                           yarp::sig::Vector m_orientation_rpy,
                                           CpGaitTransformations *GTj);

        void SetImuTransformation(yarp::sig::Vector IMU_RollPitchYaw);

        Vector3d getEulerAnglesXYZ_FixedFrame(Matrix3d CurRotRealStFootInHorBase);

        Matrix3d getComposedOrientFixedFrame(Vector3d OrientXY_F_hF);
										
	//MatrixXd getDesiredSwingFootInCurBase();
	void SetImuTransformation();
								 
        void SetTranslationBaseCoM(VectorXd t_B_CoM);
        
};

#endif // CpDesiredFeetTransformations_H


