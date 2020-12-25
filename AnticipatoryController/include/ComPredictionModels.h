#ifndef ComPredictionModels_H
#define ComPredictionModels_H 

#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>

using namespace std;
using namespace Eigen;

typedef Eigen::Matrix<double, 6, 1> Vector6d;

// ===============================================================================================
/*
 * Discrete_CP_LIP_Model_H : This class encodes the discrete template model
 * used to model the dynamics of biped robot standing on one foot.
 *
 * The models are based on the 3D LIPM with finite sized foot.
 * 
*/
// ===============================================================================================


// class Perturbed_LIPM
class Perturbed_LIPM
{


        double g;   // Gravity acceleration

    public :

        double Ts;          // State samplimg time
        double zc;          // Height of the robot CoM
        double W;			// 
        double Mass;

        MatrixXd MxA;       // State Transition matrix
        VectorXd VeB;       // State Control vector
        RowVectorXd VeC;    // Stace Measurement Vector for ZMP

        // MatrixXd MxA_D;       // State Transition matrix
        // VectorXd VeB_D;     // State Control vector
        // RowVectorXd VeC_D;   // Stace Measurement Vector for stance foot orientation

        double MxA_D;   // State Transition matrix
        double VeB_D;   // State Control vector
        double VeC_D;   // Stace Measurement Vector for stance foot orientation

        VectorXd StatesX;
        VectorXd StatesY;
        // VectorXd StatesDx;
        // VectorXd StatesDy;
        double StatesDx;
        double StatesDy;

        double OutputX;
        double OutputY;
        double OutputDx;
        double OutputDy;

        double CtrlUX;      // control input (X CoM jerk)
        double CtrlUY;      // control input (Y CoM jerk)
        double CtrlUDx;      // control input (Theta jerk)
        double CtrlUDy;     // control input (Theta jerk)

        int ModelOption; //Type of LIPM model used for MPC 0: [c c_dot cop dist]; 1: [c c_dot c_ddot dist]; 2: [c c_dot c_ddot cop];

        // Constructor of the class Discrete_CP_LIP_Model
        Perturbed_LIPM();
        ~Perturbed_LIPM();

        void Init(double x, double y, double Mass_, int m_option);
        //DiscreteLIPModel(double SpTime, double CoMHeight);
        void Create_LIPM(double Omega);
        // Update of the state by LMPC optimal solution
       void UpdateStates_LIPM(double Omega, double uX, double uY, double uDx, double uDy, double lambda_z);
        // set the height of the CoM
       void SetCoMHeight(double CoMHeight);

       // Get the height of the CoM
};


// ===================================================================================================
/*
 * Predicted_Model : This class creates the prediction model associated with 
 * the chosen template model.
 *
*/
// ===================================================================================================

class PredictionModel {

    double g; // gravity acceleration
    VectorXd SamplesVector; // vector of sampling instants over horizon Ny

    public :

    int Ny; 			// Receding Horizon
    int Nu; 			// Control Horizon
    double Zc ; 		// CoM Height
    double Tp ; 		// Sampling Time
    double W;    		//  natural frequency of the pendulum

    MatrixXd Pps;	 	// sub-state transition matrix of CoM position over Ny*T
    MatrixXd Pvs;	 	// sub-state transition matrix of CoM velocity over Ny*T
    // MatrixXd Pas;	 	// sub-state transition matrix of CoM velocity over Ny*T
    MatrixXd Pzs;	 	// sub-state transition matrix of ZMP (CoP) over Ny*T
    MatrixXd Pds;	 	// sub-state transition matrix of perturbation over Ny*T
	MatrixXd PEs;	 	// sub-state transition matrix of capture point position over Ny*T

	MatrixXd Ppu; 		// sub-state control matrix of CoM position over Ny*T
    MatrixXd Pvu; 		// sub-state control matrix of CoM velocity over Ny*T
    // MatrixXd Pau; 		// sub-state control matrix of CoM velocity over Ny*T
    MatrixXd Pzu; 		// sub-state control matrix of ZMP (CoP) over Ny*T
    MatrixXd Pdu; 		// sub-state control matrix of perturbation over Ny*T
    MatrixXd PEu; 		// sub-state control matrix of capture point position over Ny*T

    MatrixXd TheP;   	// Matrix to compute vector of variation form vector over the prediction horizon

    double DurationSteps; 	// array of duration of the 4 steps within Horizon Ny
    int SamplesPerStep; 	// array of number of samples for each steps

    int ModelOption; 		//Type of LIPM model used for MPC

    PredictionModel();
    ~PredictionModel();

    void CreateContPredictionModel(double, 		//SamplingTime,
		                            int, 		//RecHorizon
		                            int, 		//CtrlHorizon
		                            double, 	//CoMHeight
		                            int );  	// limp option

    void CreateSampPredictionModel(Perturbed_LIPM DMod, 	// Discrete LIPM Object
	                                VectorXd SampVec, 		//Sample vector
	                                int nbSampStp, 			// array of nb of samples per steps
	                                double DurStep,			// array of steps durations
	                                int m_option); 			// limp model option

    void prefactorization();

    MatrixXd A(double Ti, double zc);

    VectorXd B(double Ti, double zc);

};


// // ====================================================================================
class StepReferecences
{

    public :
        int RecH;

        VectorXd Disturb_cX;
		VectorXd Disturb_cY;
		VectorXd FootstepsX;
		VectorXd FootstepsY;
		double   lambda_z;    //

        VectorXd VxAbsStab;
        VectorXd VyAbsStab; 

        StepReferecences();
        ~StepReferecences();

        void InitStepReferecences(int RecH_);

        void UpdateStepReferecences(VectorXd Disturb_cx_, VectorXd Disturb_cy_, VectorXd ftstepx, VectorXd ftstepy, double fu_z_mgbeta_);

};

// ====================================================================================
/*
 * SelectionMatrices : This class computes the cyclic selection vector 
 * associating each sampling instant to reference capture-point value.
 */
// ====================================================================================
class SelectionMatrices
{

    // Vector and Matrices
    VectorXd Uc_12;
    MatrixXd Uf_12;

    VectorXd U0;
    VectorXd Uf1;

    VectorXd Uc0;
    VectorXd Ufn;

    VectorXd UF;

    int  time_1;
    int count_sx;

    public :

    VectorXd SamplesVector;                 // vector of samples over RecH
    int SpS;                                // array of nb of samples per steps
    double DrS;                             // array of steps durations
                                            // [0] - [3]: single support duration of steps 1,2,3 and 4
                                            // [5] duration of double support phase

    double Tp;                              // sampling time
    double zc;                              // Height of CoM
    double W;                               // Natural frequency of the LIPM

    // selection matrix of the stance foot defined wrt. the robot
    VectorXd LocalCurrentSel;               // current
    MatrixXd LocalFutureSel;                // future (predicted steps)

    VectorXd CurrentRefSel;               // current
    VectorXd NextRefSel;               // Next
    VectorXd Uf;

    // selector of stance foot
    int LeftStanceFoot;
    int RightStanceFoot;

    int RecH; // receding horizon
    int IndexSFt;                          // cyclic variable indication change in Sft

    SelectionMatrices();
    ~SelectionMatrices();

    void Initialize( VectorXd SampVec, double SamplingTime, int SamplesPerStep, double DurationSteps);

    void UpdateSelectionMx(int time_index);
    void ResetSelectionMx();

};


#endif  // ComPredictionModels_H