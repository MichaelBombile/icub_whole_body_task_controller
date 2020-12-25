#ifndef ComRefGenerator_H
#define ComRefGenerator_H 

#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>

#include "ComPredictionModels.h" 
#include "MPCOptimization.h"

using namespace std;
using namespace Eigen;


// ===============================================================================================
/*
 * ComRefGenerator : This class generates the motion of the CoM
 * 
*/
// ===============================================================================================


class ComRefGenerator
{
	public :

	double sTime;
	double z_CoM;
	double Omega;
	double mMass;
	int    m_option;
	//
	int 	nsp;
	double  DurationSteps; 
	double  gains[10];

	// VectorXd SupportConstraints;
	VectorXd SamplesVector;

	double 	 fu_z_mgbeta;
   	VectorXd Disturb_cx_ ;
   	VectorXd Disturb_cy_ ;
   	VectorXd Dist_time ;
	VectorXd ftstepx_ ;
   	VectorXd ftstepy_ ;

   	MatrixXd EdgeNormals;
	VectorXd EdgePositions;
	Vector2d ffmmcom_xy;

	MatrixXd MconsXY;
	VectorXd BconsXY;

	int nWSR;

	Perturbed_LIPM 			DMod;
	PredictionModel 		MpcModel;
	StepReferecences 		StpRef;
	QMatrix 				QMx;
	PVectorQP 				PVec;
	ConstraintsCoP 			CnstrCoP;
	QPSolver_OASES   		QPxySolver;
	SelectionMatrices 		SMx;
	ReachConstraintCoM 		CnstrCoM;

	// std::ofstream Out_states;

	ComRefGenerator();
	~ComRefGenerator();

	void Initialize(double sTime_, double z_CoM_, double mMass_, double gains_[], VectorXd EdgePositions_, MatrixXd EdgeNormals_);
	void run();

};

#endif // ComRefGenerator_H