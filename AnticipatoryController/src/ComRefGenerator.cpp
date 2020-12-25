#include <iostream>
#include <cmath>
#include "ComRefGenerator.h"

using namespace std;
using namespace Eigen;


ComRefGenerator::ComRefGenerator(){};
ComRefGenerator::~ComRefGenerator()
{
	
}

void ComRefGenerator::Initialize(double sTime_, double z_CoM_, double mMass_, double gains_[], VectorXd EdgePositions_, MatrixXd EdgeNormals_)
{	
	// -------------------------------------------------------------------
	sTime 	= sTime_; 
	z_CoM 	= z_CoM_; 
	mMass 	= mMass_; 
	Omega 	= sqrt(9.80/z_CoM);
	// -----------------------
	// preset at the beginning
	nsp  	 = 10;
	m_option = 0;
	//
	DurationSteps  = sTime * nsp;
	SamplesVector.resize(nsp);
	for (int i=0; i<nsp; i++)	 SamplesVector(i) = sTime;

	//
	nWSR = 1000;

	// weight of the QP
	// ----------------
	memcpy(gains, &gains_[0], 10 * sizeof *gains_); 
	//
	Disturb_cx_ = VectorXd::Zero(nsp);
	Disturb_cy_ = VectorXd::Zero(nsp);
	ftstepx_	= VectorXd::Zero(nsp);
	ftstepy_	= VectorXd::Zero(nsp);
	fu_z_mgbeta = 0.0;

	EdgeNormals.resize(2,6);
	EdgePositions.resize(6);
	EdgeNormals   = EdgeNormals_;
	EdgePositions = EdgePositions_;
	ffmmcom_xy	  = EdgePositions_(0) * EdgeNormals_.col(0);
	// ----------------------------------------------------------------------------
	//  data logger
 	//  std::string data_number = "00";
 	//  string path_log_states  = "log_stepping_states_"  + data_number + ".txt";
	//  Out_states.open(path_log_states.c_str());
	// ----------------------------------------------------------------------------
	DMod.Init(sTime, z_CoM, mMass, m_option);
    DMod.Create_LIPM(Omega);
	//
	MpcModel.CreateSampPredictionModel( DMod, SamplesVector,  nsp, DurationSteps, m_option);	// Discrete LIPM Object, Sample vector, nb of samples per steps, steps durations

	// Step references
   	StpRef.InitStepReferecences(nsp);
   	StpRef.UpdateStepReferecences(Disturb_cx_, Disturb_cy_, ftstepx_, ftstepy_, fu_z_mgbeta);
   	// Hessian matrix
	QMx.Init(MpcModel, gains);
    MatrixXd MQKx  = QMx.getQstab(MpcModel, StpRef);
    MatrixXd MQKxy = QMx.getQstabXY(MpcModel, StpRef);
    // Gradiant vector
	PVec.Init(MpcModel, gains);
	VectorXd VPKxy = PVec.getPstabXY( MpcModel, DMod, StpRef);

	// Compute the constraints
	// -----------------------
	// Constraints on CoP
	CnstrCoP.Init( MpcModel, DMod, StpRef, EdgeNormals_,  EdgePositions_);
	CnstrCoP.UpdateSupportPolygon(EdgeNormals_, EdgePositions_);
    CnstrCoP.GetCoPConstraints( MpcModel, DMod, StpRef);
    // MatrixXd MconsXY = CnstrCoP.MuAct;		 // constraints matrix
    // VectorXd BconsXY = CnstrCoP.BuAct;		 // constraints vector
    // Constraints on CoM
    CnstrCoM.Init(  MpcModel, DMod, StpRef, ffmmcom_xy);
    CnstrCoM.applyReachConstraintCoM(  MpcModel, DMod, StpRef, ffmmcom_xy);

    MconsXY = Eigen::MatrixXd::Zero(CnstrCoP.MuAct.rows() + CnstrCoM.MuCoM.rows(), CnstrCoP.MuAct.cols());
	BconsXY = Eigen::VectorXd::Zero(CnstrCoP.MuAct.rows() + CnstrCoM.MuCoM.rows());						

    MconsXY.topRows(CnstrCoP.MuAct.rows()) 	 	= CnstrCoP.MuAct;
    MconsXY.bottomRows(CnstrCoM.MuCoM.rows()) 	= CnstrCoM.MuCoM;
	BconsXY.head(CnstrCoP.MuAct.rows()) 		= CnstrCoP.BuAct;
	BconsXY.tail(CnstrCoM.MuCoM.rows())  	 	= CnstrCoM.BuCoM;
	
	// QP Solver 
	// ----------
	Options QPOptionToSet;
    QPOptionToSet.setToMPC();
    QPOptionToSet.printLevel = PL_LOW;

    QPxySolver.InitialiseQPSol(MQKxy, VPKxy, MconsXY, BconsXY);
    QPxySolver.setSolverOptions(QPOptionToSet);
    QPxySolver.QPSolution_Oases_XY(MQKxy, VPKxy, MconsXY, BconsXY);
    //
    SMx.Initialize(SamplesVector, sTime, nsp, DurationSteps);
}

void ComRefGenerator::run()
{
	// update the references
	StpRef.UpdateStepReferecences(Disturb_cx_, Disturb_cy_, ftstepx_, ftstepy_, fu_z_mgbeta);
	// Update the QP matrices and vectors 
	MatrixXd MQKxy = QMx.getQstabXY(MpcModel, StpRef);					// Hessian matrix
	VectorXd VPKxy = PVec.getPstabXY(MpcModel, DMod, StpRef);			// Gradient vector

	CnstrCoP.GetCoPConstraints(MpcModel, DMod, StpRef);					// update CoP Constraints matrix and vectors
	// MatrixXd MconsXY = CnstrCoP.MuAct;		 							// constraints matrix
	// VectorXd BconsXY = CnstrCoP.BuAct;									// constraints vector
	CnstrCoM.applyReachConstraintCoM(  MpcModel, DMod, StpRef, ffmmcom_xy); // update CoM Constraints matrix and vectors

    MconsXY.topRows(CnstrCoP.MuAct.rows()) 	 	= CnstrCoP.MuAct;
    MconsXY.bottomRows(CnstrCoM.MuCoM.rows()) 	= CnstrCoM.MuCoM;
	BconsXY.head(CnstrCoP.MuAct.rows()) 		= CnstrCoP.BuAct;
	BconsXY.tail(CnstrCoM.MuCoM.rows())  	 	= CnstrCoM.BuCoM;
	// 
	QPxySolver.setnbWorkingSetRecalculation(nWSR);
	QPxySolver.QPSolution_Oases_XY(MQKxy, VPKxy, MconsXY, BconsXY);

	DMod.UpdateStates_LIPM(Omega, QPxySolver.uX, QPxySolver.uY, QPxySolver.uDx, QPxySolver.uDy, fu_z_mgbeta);

}