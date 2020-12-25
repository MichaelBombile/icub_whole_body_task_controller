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



#include <iostream>
#include <cmath>
#include "ComPredictionModels.h"

using namespace std;
using namespace Eigen;


/*
 * Discrete_CP_LIP_Model_H : This class encodes the discrete template model
 * used to model the dynamics of biped robot standing on one foot.
 *
 * The models are based on the 3D LIPM with finite sized foot.
 * 
*/

Perturbed_LIPM::Perturbed_LIPM(){}

Perturbed_LIPM::~Perturbed_LIPM(){}

void Perturbed_LIPM::Init(double SpTime, double CoMHeight, double Mass_, int m_option)
{

    g = 9.80;

    Ts     = SpTime;
    zc     = CoMHeight;
    W      = sqrt(g/zc);
    Mass = Mass_;

    ModelOption = m_option;

    // translation motion
    // ========================
    // State Transition matrix

    MxA = Eigen::MatrixXd::Identity(3,3);
    VeB = Eigen::VectorXd::Zero(3);
    VeC = Eigen::RowVectorXd::Zero(3);

    // State Transition matrix
    MxA <<  1.,         Ts,   pow(Ts,2.)/2.,
            0.,         1.,              Ts, 
            0.,         0.,              1.;

    // State Control vector
    VeB <<  pow(Ts,3.)/6.,
            pow(Ts,2.)/2.,
                       Ts;

    // Stace Measurement Vector for ZMP
    VeC <<       1.0, 0.0, -1./(W*W);   
    //
    // Disturbance
    // ========================
    // // State Transition matrix
    // MxA_D.resize(1,1);
    // MxA_D(0,0) = 1.0;
    // // State Control vector
    // VeB_D.resize(1);
    // VeB_D(0) = 1.0;
    // // Stace Measurement Vector for stance foot orientation
    // VeC_D.resize(1);
    // VeC_D(0) = 1.0;

    // State Transition matrix
    MxA_D = 1.0;
    // State Control vector
    VeB_D = 1.0;
    // Stace Measurement Vector for stance foot orientation
    VeC_D = 1.0;

    // states vectors x, y and rotation
    StatesX  = Eigen::VectorXd::Zero(3);
    StatesY  = Eigen::VectorXd::Zero(3);
    // StatesDx = Eigen::VectorXd::Zero(1);
    // StatesDy = Eigen::VectorXd::Zero(1);
    StatesDx = 0.0;
    StatesDy = 0.0;
}


void Perturbed_LIPM::Create_LIPM(double Omega)
{
    W   = Omega;

    // translation motion
    // ========================
    // State Transition matrix
    MxA <<  1.,         Ts,   pow(Ts,2.)/2.,
            0.,         1.,              Ts, 
            0.,         0.,              1.;

    // State Control vector
    VeB <<  pow(Ts,3.)/6.,
            pow(Ts,2.)/2.,
                       Ts;
    // Stace Measurement Vector for ZMP
    VeC <<  1.0, 0.0, -1./(W*W);   
    //
    // Disturbance
    // ========================
    // // State Transition matrix
    // MxA_D.resize(1,1);
    // MxA_D(0,0) = 1.0;
    // // State Control vector
    // VeB_D.resize(1);
    // VeB_D(0)   = 1.0;
    // // Stace Measurement Vector for stance foot orientation
    // VeC_D.resize(1);
    // VeC_D(0) = 1.;

    // State Transition matrix
    MxA_D = 1.0;
    // State Control vector
    VeB_D = 1.0;
    // Stace Measurement Vector for stance foot orientation
    VeC_D = 1.0;

}

       // Update of the state by LMPC optimal solution

void Perturbed_LIPM::UpdateStates_LIPM(double Omega, double uX, double uY, double uDx, double uDy, double lambda_z)
{
  // I ADDED THIS HERE FOR TESTING THE VARIATION OF Zc
    this->Create_LIPM(Omega);

    // Optimal control input from LMPC Solver
    CtrlUX  = uX;
    CtrlUY  = uY;
    CtrlUDx = uDx;
    CtrlUDy = uDy;

     // update the the disturbance
    StatesDx = 0.0*lambda_z * StatesX(0) + MxA_D * StatesDx + VeB_D * CtrlUDx;
    StatesDy = 0.0*lambda_z * StatesY(0) + MxA_D * StatesDy + VeB_D * CtrlUDy;
    // StatesDx = MxA_D * StatesDx + VeB_D * CtrlUDx;
    // StatesDy = MxA_D * StatesDy + VeB_D * CtrlUDy;
    // Output values
    OutputDx = VeC_D * StatesDx;    // dist x
    OutputDy = VeC_D * StatesDy;    // dist y
    
    // State update
    StatesX = MxA * StatesX + VeB * CtrlUX;
    StatesY = MxA * StatesY + VeB * CtrlUY;
    //
    OutputX  = VeC   * StatesX + StatesDx;     // X CoP
    OutputY  = VeC   * StatesY + StatesDy;     // Y CoP

   
}

  // set the height of the CoM
void Perturbed_LIPM::SetCoMHeight(double CoMHeight)
{
      zc = CoMHeight;
};



// *************************************************************************************
/*
 * MpcBased_CP_Model : This class creates the prediction model associated with 
 * the chosen template model.
 *
*/
// *************************************************************************************

PredictionModel::PredictionModel(){}

PredictionModel::~PredictionModel(){}
// creating a function that returns the dynamic matrix
// corresponding to a given sampling time
// Translation
MatrixXd PredictionModel::A(double Ti, double Omega)
{
    //
    double W_   = Omega;
    // State Transition matrix
    // ===================================
    MatrixXd AA;
    AA = Eigen::MatrixXd::Identity(3,3);

    // with states : 0: [c c_dot c_ddot]; 
     AA <<  1.,         Ti,   pow(Ti,2.)/2.,
            0.,         1.,              Ti, 
            0.,         0.,              1.;

    return AA;
};

// creating a function that returns the control vector
// corresponding to a given sampling time
// Translation
VectorXd PredictionModel::B(double Ti, double Omega)
{
    VectorXd BB;
    BB = Eigen::VectorXd::Zero(3);

    // with states : 1: [c c_dot c_ddot];
    // State Control vector
    BB <<  pow(Ti,3.)/6.,
           pow(Ti,2.)/2.,
                      Ti;
    return BB;
};

void PredictionModel::CreateContPredictionModel(  double SamplingTime,    // Sampling time
                                                    int RecHorizon,         // Receding Horizon
                                                    int CtrlHorizon,        // Control Horizon
                                                    double Omega,          // pendulum frequency
                                                    int m_option)         // option of model (limp)
{

    g    = 9.80;
    Tp   = SamplingTime;
    Ny   = RecHorizon;
    Nu   = CtrlHorizon;
    W    = Omega;

    ModelOption = m_option;

    SamplesVector.resize(Ny);
    for (int i=0; i<Ny; i++)
    {
        SamplesVector[i] = Tp;
    }

    // Initialization for memo allocation
    Pps.resize(Ny,3);     Pps.setZero(Ny,3);
    Pvs.resize(Ny,3);     Pvs.setZero(Ny,3);
    // Pas.resize(Ny,4);     Pas.setZero(Ny,4);
    Pzs.resize(Ny,3);     Pzs.setZero(Ny,3);
    Pds.resize(Ny,1);     Pds.setZero(Ny,1);
    PEs.resize(Ny,3);     PEs.setZero(Ny,3);

    Ppu.resize(Ny,Ny);    Ppu.setZero(Ny,Ny);
    Pvu.resize(Ny,Ny);    Pvu.setZero(Ny,Ny);
    // Pau.resize(Ny,Ny);    Pau.setZero(Ny,Ny);
    Pzu.resize(Ny,Ny);    Pzu.setZero(Ny,Ny);
    Pdu.resize(Ny,Ny);    Pdu.setZero(Ny,Ny);
    PEu.resize(Ny,Ny);    PEu.setZero(Ny,Ny);
    TheP.resize(Ny,Ny);   TheP.setZero(Ny,Ny);   
    //
    RowVectorXd C, Cz, CE;
    C = Eigen::RowVectorXd::Zero(3);
    // with states : 1: [c c_dot c_ddot dist];
    // Stace Measurement Vector for ZMP
    C <<  1.0, 0.0, -1./(W*W);

    // Cz to compute the ZMP for the constraint matrix (Pzs, Pzu)
    Cz.resize(3);
    Cz(0) = 1.;
    Cz(1) = 0.;
    Cz(2) = -1./(W*W);

    // Capture point from com and com_dot
    CE = Eigen::RowVectorXd::Zero(3);
    CE(0) = 1.;
    CE(1) = 1./W;

    // Computation of the An and CAn-1 Matrices
    int p, q, s, mb, nb;            // (p,q) are size of Aa  s the nbr of row C.
                                    // mb, nb are size of Bb (control vector)
    p  = A(Tp, W).rows(); //4;     //(DMod.MxA).rows();
    q  = A(Tp, W).cols(); //4;     //(DMod.MxA).cols();
    s  = C.rows();  
    mb = B(Tp, W).rows();
    nb = B(Tp, W).cols();

    // Stace Measurement Vector for CP and CoM
    RowVectorXd  C_R;
    C_R.resize(3);
    C_R(0) = 1.;
    C_R(1) = 0.;
    C_R(2) = 0.;

    // Initializing of An and CAn-1 Matrices
    MatrixXd    A_nT,   C_nT,   AB_nT,  CAB_nT,
                A_RnT,  C_R_nT, AB_RnT, C_RAB_nT, Cz_nT, CzAB_nT, CE_nT, CEAB_nT;
    //
    A_nT.resize(p*Ny,q);      A_nT.setZero(p*Ny,q);
    C_nT.resize(s*Ny,q);      C_nT.setZero(s*Ny,q);
    Cz_nT.resize(1*Ny,3);     Cz_nT.setZero(1*Ny,3);
    CE_nT.resize(1*Ny,q);     CE_nT.setZero(1*Ny,q);

    A_RnT.resize(3*Ny,3);     A_RnT.setZero(3*Ny,3);
    C_R_nT.resize(1*Ny,3);    C_R_nT.setZero(1*Ny,3);
    //C_R_nT(1:s*nt,1:q) = 0;

    for (int i=0; i<Ny; i++)
    {
        if (i==0)
        {
            A_nT.block(p*i, 0, A(Tp, W).rows(), A(Tp, W).cols()) = A(Tp, W);
            C_nT.block(s*i, 0, (C*A(Tp, W)).rows(), (C*A(Tp, W)).cols())    = C*A(Tp, W);
            CE_nT.block(s*i, 0, (CE*A(Tp, W)).rows(), (CE*A(Tp, W)).cols()) = CE*A(Tp, W);
        }
        else
        {
            MatrixXd A_nT_1 = MatrixXd::Zero(p,q);

            for (int k=0; k<p; k++)
            {
                for (int l=0; l<q; l++)
                {
                    A_nT_1(k,l) = A_nT(p*(i-1)+k,l);
                }
            }
            //
            A_nT.block(p*i, 0, (A(Tp, W)*A_nT_1).rows(), (A(Tp, W)*A_nT_1).cols())     = A(Tp, W)*A_nT_1;
            C_nT.block(s*i, 0, (C*A(Tp, W)*A_nT_1).rows(), (C*A(Tp, W)*A_nT_1).cols()) = C*A(Tp, W)*A_nT_1;
            CE_nT.block(s*i, 0, (CE*A(Tp, W)*A_nT_1).rows(), (CE*A(Tp, W)*A_nT_1).cols()) = CE*A(Tp, W)*A_nT_1;
        }
    }

    // Computation of the An-1B and CAn-1B Matrices
    // Initializing of An-1B and CAn-1B Matrices
    AB_nT.resize(Ny*mb,Ny*nb);
    CAB_nT.resize(s*Ny,Ny*nb);
    AB_RnT.resize(Ny*3,Ny*1);
    C_RAB_nT.resize(1*Ny,Ny*1);
    CzAB_nT.resize(1*Ny,Ny*1);
    CEAB_nT.resize(s*Ny,Ny*1);

    // for rotation
    //C_RAB_nT.resize(s*Ny,Ny*nb);
    for (int i=0; i<Ny; i++)
    {
        for (int j=i; j<Ny; j++)
        {
            if (j==i)
            {
                AB_nT.block(mb*j, nb*i, (B(Tp, W)).rows(), (B(Tp, W)).cols()) = B(Tp, W);

                CAB_nT(s*j, nb*i)  = C*B(Tp, W);
                CEAB_nT(s*j, nb*i) = CE*B(Tp, W);
            }
            else
            {
                MatrixXd AB_nT_1 = MatrixXd::Zero(mb,nb);

                for (int k=0; k<mb; k++)
                {
                    for (int l=0; l<nb; l++)
                    {
                        AB_nT_1(k,l) = AB_nT(mb*(j-1)+k,nb*i+l);
                    }
                }

                //std::cout<<"AB_nT_1 is :\n"<< AB_nT_1 << std::endl;
                AB_nT.block(mb*j, nb*i, (A(Tp, W)*AB_nT_1).rows(), (A(Tp, W)*AB_nT_1).cols())           = A(Tp, W)*AB_nT_1;
                CAB_nT.block(s*j, nb*i, (C*A(Tp, W)*AB_nT_1).rows(), (C*A(Tp, W)*AB_nT_1).cols())       = C*A(Tp, W)*AB_nT_1;
                CEAB_nT.block(s*j, nb*i, (CE*A(Tp, W)*AB_nT_1).rows(), (CE*A(Tp, W)*AB_nT_1).cols())    = CE*A(Tp, W)*AB_nT_1;
            }
            //std::cout<<"AB_nT is :\n"<< AB_nT << std::endl;
            //std::cout<<"CAB_nT is :\n"<< CAB_nT << std::endl;
        }
    }

    // resizing the matrices
    Pps.resize(Ny,q);    Pps.setZero(Ny,q);
    Pvs.resize(Ny,q);    Pvs.setZero(Ny,q);
    Pzs.resize(Ny,q);    Pzs.setZero(Ny,q);
    PEs.resize(Ny,q);    PEs.setZero(Ny,q);

    Pds.resize(Ny,1);    Pds.setZero(Ny,1);

    Pds = MatrixXd::Ones(Ny,1);

    // extracting values
    for (int i=0; i<Ny; i++)
    {
        for (int j=0; j<q; j++)
        {
            Pps(i,j) = A_nT(p*i,j);     // CoM Position states matrix over Horizon Ny
            Pvs(i,j) = A_nT(p*i+1,j);   // CoM velocity states matrix over Horizon Ny
            // Pzs(i,j) = A_nT(p*i+2,j);   // ZMP states matrix over Horizon Ny
            // Pds(i,j) = A_nT(p*i+3,j);   // Perturbation states matrix over Horizon Ny
        }

        for (int j=0; j<Ny; j++)
        {           
            Ppu(i,j) = AB_nT(p*i,j);
            Pvu(i,j) = AB_nT(p*i+1,j);
            // Pzu(i,j) = AB_nT(p*i+2,j);
            // Pdu(i,j) = AB_nT(p*i+3,j);
        }
    }

    //
    PEs = CE_nT;     // Capture point Output states matrix
    PEu = CEAB_nT;   // capture point Output control matrix  
    //
    Pds = MatrixXd::Ones(Ny,1);
    for(int i=0; i<Ny; i++)
    {
        for(int j=i; j<Ny; j++)
        {
                Pdu(j,i) = 1.0;
        }
    }

    //
    Pzs = C_nT;     // ZMP Output states matrix
    Pzu = CAB_nT;   // ZMP Output control matrix

    //
    for (int i=0; i<Ny; i++)
    {
       for (int j =0; j< Ny; j++)
       {
           if (j==i){ TheP(j,i) = 1.;}
           else
           {
               if (j== i+1){ TheP(j,i) = -1.;}
               else{ TheP(j,i) = 0.;}
           }
       }
    }

    // Perturbation matrices accounting for CoM position change
    
};

void PredictionModel::CreateSampPredictionModel(  Perturbed_LIPM DMod,     // Discrete LIPM Object
                                                    VectorXd SampVec,      // Sample vector
                                                    int nbSampStp,          // array of nb of samples per steps
                                                    double DurStep,        // array of steps durations
                                                    int m_option)         // option of model (limp)
{
     g  = 9.80;
    Tp  = DMod.Ts;
    Ny  = int(SampVec.rows());
    Nu  = Ny;
    W   = DMod.W;
    ModelOption = m_option;

    SamplesVector  = SampVec;
    SamplesPerStep = nbSampStp;
    DurationSteps  = DurStep;
    //
    // Computation of the An and CAn-1 Matrices
    int p, q, s, mb, nb;    // (p,q) are size of Aa  s the nbr of row C.
                            // mb, nb are size of Bb (control vector)
    p  = (DMod.MxA).rows();
    q  = (DMod.MxA).cols();
    s  = (DMod.VeC).rows();
    mb = (DMod.VeB).rows();
    nb = (DMod.VeB).cols();

    // Initialization for memo allocation
    Pps.resize(Ny,3);     Pps.setZero(Ny,3);
    Pvs.resize(Ny,3);     Pvs.setZero(Ny,3);
    Pzs.resize(Ny,3);     Pzs.setZero(Ny,3);
    PEs.resize(Ny,3);     PEs.setZero(Ny,3);
    

    Ppu.resize(Ny,Ny);    Ppu.setZero(Ny,Ny);
    Pvu.resize(Ny,Ny);    Pvu.setZero(Ny,Ny);
    Pzu.resize(Ny,Ny);    Pzu.setZero(Ny,Ny);
    PEu.resize(Ny,Ny);    PEu.setZero(Ny,Ny);

    Pds.resize(Ny,1);     Pds.setZero(Ny,1);
    Pdu.resize(Ny,Ny);    Pdu.setZero(Ny,Ny);

    TheP.resize(Ny,Ny);   TheP.setZero(Ny,Ny); 
    //
    // States Measurement vector
    RowVectorXd C, Cz, CE;
    C = Eigen::RowVectorXd::Zero(3);
    // with states : 1: [c c_dot c_ddot dist];
    C <<  1.0, 0.0, -1./(W*W);
   
    // Cz to compute the ZMP for the constraint matrix (Pzs, Pzu)
    Cz.resize(3);
    Cz(0) = 1.;
    Cz(1) = 0.;
    Cz(2) = -1./(W*W);

    // Capture point from com and com_dot
    CE = Eigen::RowVectorXd::Zero(3);
    CE(0) = 1.;
    CE(1) = 1./W;

    // Initializing of An and CAn-1 Matrices
    MatrixXd    A_nT, C_nT, AB_nT, CAB_nT,
                A_RnT,C_R_nT, AB_RnT, C_RAB_nT, Cz_nT, CzAB_nT, CE_nT, CEAB_nT;
    //
    A_nT.resize(p*Ny,q);      A_nT.setZero(p*Ny,q);
    C_nT.resize(s*Ny,q);      C_nT.setZero(s*Ny,q);
    Cz_nT.resize(1*Ny,3);     Cz_nT.setZero(1*Ny,3);
    CE_nT.resize(1*Ny,q);     CE_nT.setZero(1*Ny,q);

    cout << "  HERE UP OK     \n" <<  1.0    << endl; 

    //C_R_nT(1:s*nt,1:q) = 0;

    for (int i=0; i<Ny; i++)
    {
        if (i==0)
        {
            A_nT.block(p*i, 0, A(SampVec(i), W).rows(), A(SampVec(i), W).cols()) = A(SampVec(i), W);

            C_nT.block(s*i, 0, (C*A(SampVec(i), W)).rows(), (C*A(SampVec(i), W)).cols())    = C*A(SampVec(i), W);
            CE_nT.block(s*i, 0, (CE*A(SampVec(i), W)).rows(), (CE*A(SampVec(i), W)).cols()) = CE*A(SampVec(i), W);
        }
        else
        {
            MatrixXd A_nT_1 = MatrixXd::Zero(p,q);

            for (int k=0; k<p; k++)
            {
                for (int l=0; l<q; l++)
                {
                    A_nT_1(k,l) = A_nT(p*(i-1)+k,l);
                }
            }
             //
            A_nT.block(p*i, 0, (A(SampVec(i), W)*A_nT_1).rows(), (A(SampVec(i), W)*A_nT_1).cols())     = A(SampVec(i), W)*A_nT_1;
            C_nT.block(s*i, 0, (C*A(SampVec(i), W)*A_nT_1).rows(), (C*A(SampVec(i), W)*A_nT_1).cols()) = C*A(SampVec(i), W)*A_nT_1;
            //
            CE_nT.block(s*i, 0, (CE*A(SampVec(i), W)*A_nT_1).rows(), (CE*A(SampVec(i), W)*A_nT_1).cols()) = CE*A(SampVec(i), W)*A_nT_1;
        }
    }

    // Computation of the An-1B and CAn-1B Matrices
    // Initializing of An-1B and CAn-1B Matrices
    AB_nT.resize(Ny*mb,Ny*nb);
    CAB_nT.resize(s*Ny,Ny*nb);
    AB_RnT.resize(Ny*3,Ny*1);
    C_RAB_nT.resize(1*Ny,Ny*1);
    CzAB_nT.resize(1*Ny,Ny*1);
    CEAB_nT.resize(s*Ny,Ny*1);

    // for rotation
    //C_RAB_nT.resize(s*Ny,Ny*nb);
    for (int i=0; i<Ny; i++)
    {
        for (int j=i; j<Ny; j++)
        {
            if (j==i)
            {
                AB_nT.block(mb*j, nb*i, (B(SampVec(i), W)).rows(), (B(SampVec(i), W)).cols()) = B(SampVec(i), W);
                //CAB_nT(s*j, nb*i, (C*B(SampVec(i), W)).rows(), (C*B(SampVec(i), W)).cols()) = C*B(SampVec(i), W);
                CAB_nT(s*j, nb*i)  = C*B(SampVec(i), W);
                CEAB_nT(s*j, nb*i) = CE*B(SampVec(i), W);
            }
            else
            {
                MatrixXd AB_nT_1 = MatrixXd::Zero(mb,nb);

                for (int k=0; k<mb; k++)
                {
                    for (int l=0; l<nb; l++)
                    {
                        AB_nT_1(k,l) = AB_nT(mb*(j-1)+k,nb*i+l);
                    }
                }

                //std::cout<<"AB_nT_1 is :\n"<< AB_nT_1 << std::endl;
                AB_nT.block(mb*j, nb*i, (A(SampVec(i), W)*AB_nT_1).rows(), (A(SampVec(i), W)*AB_nT_1).cols())           = A(SampVec(i), W)*AB_nT_1;
                CAB_nT.block(s*j, nb*i, (C*A(SampVec(i), W)*AB_nT_1).rows(), (C*A(SampVec(i), W)*AB_nT_1).cols())       = C*A(SampVec(i), W)*AB_nT_1;
                CEAB_nT.block(s*j, nb*i, (CE*A(SampVec(i), W)*AB_nT_1).rows(), (CE*A(SampVec(i), W)*AB_nT_1).cols())    = CE*A(SampVec(i), W)*AB_nT_1;
            }
            //std::cout<<"AB_nT is :\n"<< AB_nT << std::endl;
            //std::cout<<"CAB_nT is :\n"<< CAB_nT << std::endl;
        }
    }

    // resizing the matrices
    Pps.resize(Ny,q);    Pps.setZero(Ny,q);
    Pvs.resize(Ny,q);    Pvs.setZero(Ny,q);
    Pzs.resize(Ny,q);    Pzs.setZero(Ny,q);
    PEs.resize(Ny,q);    PEs.setZero(Ny,q);

    Pds.resize(Ny,1);    Pds.setZero(Ny,1);

    Pds = MatrixXd::Ones(Ny,1);

    // extracting values
    for (int i=0; i<Ny; i++)
    {
        for (int j=0; j<q; j++)
        {
            Pps(i,j) = A_nT(p*i,j);     // CoM Position states matrix over Horizon Ny
            Pvs(i,j) = A_nT(p*i+1,j);   // CoM velocity states matrix over Horizon Ny
            Pzs(i,j) = A_nT(p*i+2,j);   // ZMP states matrix over Horizon Ny
            // Pds(i,j) = A_nT(p*i+3,j);   // Perturbation states matrix over Horizon Ny
        }

        for (int j=0; j<Ny; j++)
        {           
            Ppu(i,j) = AB_nT(p*i,j);
            Pvu(i,j) = AB_nT(p*i+1,j);
            Pzu(i,j) = AB_nT(p*i+2,j);
            // Pdu(i,j) = AB_nT(p*i+3,j);
        }
    }

    //
    PEs = CE_nT;     // Capture point Output states matrix
    PEu = CEAB_nT;   // capture point Output control matrix  
    //
    Pds = MatrixXd::Ones(Ny,1);
    for(int i=0; i<Ny; i++)
    {
        for(int j=i; j<Ny; j++)
        {
                Pdu(j,i) = 1.0;
        }
    }

    //
    Pzs = C_nT;     // ZMP Output states matrix
    Pzu = CAB_nT;   // ZMP Output control matrix

    //
    for (int i=0; i<Ny; i++)
    {
       for (int j =0; j< Ny; j++)
       {
           if (j==i){ TheP(j,i) = 1.;}
           else
           {
               if (j== i+1){ TheP(j,i) = -1.;}
               else{ TheP(j,i) = 0.;}
           }
       }
    }

}

// // =================================================================================================================


// // =================================================================================================================
StepReferecences::StepReferecences(){}
StepReferecences::~StepReferecences(){}

void StepReferecences::InitStepReferecences(int RecH_)
{
    RecH = RecH_;

    Disturb_cX = VectorXd::Zero(RecH);
    Disturb_cY = VectorXd::Zero(RecH);
    FootstepsX = VectorXd::Zero(RecH);
    FootstepsY = VectorXd::Zero(RecH);

    VxAbsStab  = VectorXd::Zero(RecH);
    VyAbsStab  = VectorXd::Zero(RecH);

    lambda_z   = 0.0;

    
}

// void StanceFootPose::UpdateStabilizabilityRef(FeetSelectionMatrices SMx, Vector2d Disturb_c, Vector2d ftstep, double fu_z_mgbeta_)
void StepReferecences::UpdateStepReferecences(VectorXd Disturb_cx_, VectorXd Disturb_cy_, VectorXd ftstepx, VectorXd ftstepy, double fu_z_mgbeta_)
{
    lambda_z   = fu_z_mgbeta_;

    Disturb_cX = Disturb_cx_.head(RecH);
    Disturb_cY = Disturb_cy_.head(RecH);
    //
    FootstepsX = ftstepx.head(RecH);
    FootstepsY = ftstepy.head(RecH);

}

//
// ====================================================================================
/*
 * SelectionMatrices : This class computes the cyclic selection vector 
 * associating each sampling instant to reference perturbation value.
 */
// ====================================================================================

SelectionMatrices::SelectionMatrices(){}
SelectionMatrices::~SelectionMatrices(){}

void SelectionMatrices::Initialize(VectorXd SampVec, double SamplingTime, int SamplesPerStep, double DurationSteps)
{
    //
    SamplesVector = SampVec;
    Tp      = SamplingTime;
    //
    SpS     = SamplesPerStep;
    DrS     = DurationSteps;            // DurationSteps[0]-[3]: Steps 1 & 2 durations.
    RecH    = SpS;
    int tm  = 0;                        // tm for j in nk definition
    time_1  = 0;
    int ny  = (int)(round(DrS/Tp));  // number of regularly spaced samples during step 1 period

    count_sx = 0;

    /* CYCLIC SELECTION MATRICES USED TO PREDETERMINE THE REFERENCE OVER THE PREDICTION HORIZON  */     
    // ******************************************************************************************* //
    // transition from single to double support: Interpoltion variables
    U0.resize(2*ny);
    Uf1.resize(2*ny);
    Uc0.resize(2*ny);
    Ufn.resize(2*ny);
    // selection of perturbation over the horizon for step determination
    UF  = VectorXd::Zero(2*ny);
    UF.tail(ny+1) = VectorXd::Ones(ny+1);
    Uf  = VectorXd::Zero(ny);

    for (int i=0; i<ny; i++)
    {
        U0(i)  = 1.;
        Uf1(i) = 0.;

        Uc0(i) = 1.;
        Ufn(i) = 0.;
    }

    for (int i=0; i<ny; i++)
    {
        U0(i+ny)  = 0.;
        Uf1(i+ny) = 1.;

        Uc0(i+ny) = 0.;
        Ufn(i+ny) = 1.;
    }

    // Selection matrix
    LocalCurrentSel = VectorXd::Zero(RecH);
    LocalFutureSel  = MatrixXd::Zero(RecH,1);

    CurrentRefSel = VectorXd::Zero(RecH);
    NextRefSel = VectorXd::Zero(RecH);

    VectorXd Ones(1); Ones(0) = 1.0;
    // Intitialisation of class variable (tm = 0)
    SelectionMatrices::UpdateSelectionMx(tm);       // time_index;



};

void SelectionMatrices::UpdateSelectionMx(int time_index)
{

    // int tm      = time_index;                   // tm for j in nk definition
    int tm      = count_sx;                     // tm for j in nk definition
    int ny      = (int)(round(DrS/Tp));         // number of regularly spaced samples during step 1 period
    // Creation of the cyclic mechanism
    int nk      = fmod(tm,ny); // tm for j;     // cyclic variable with period ny*Tp
    IndexSFt    =  nk;
    time_1      = tm;
    // vector of position of step 1 samples over time
    VectorXd L(SpS);
    VectorXd In,                                // vector of time between samples over horizon
    Index_S1(SpS);                              // vector of samples position over Step 1

    In = SamplesVector*(1/Tp);
    Index_S1.setZero(SpS);    

    for (int i=0; i<SpS; i++)
    {
        int sumv_i;
        sumv_i = 0;
        for(int j=0; j<i; j++)
            { sumv_i += round(In(j)); }

        Index_S1(i) = sumv_i;
        L(i) = Index_S1(i) + nk;
    }
   
    // Cyclic Selection Matrices in the local frame
    LocalCurrentSel.setZero(RecH);
    LocalFutureSel.setZero(RecH,1);

    // Section related to step 1
    for (int i=0; i<SpS; i++)
    {
        LocalCurrentSel(i)  = U0(int(L(i)));
        LocalFutureSel(i,0) = Uf1(int(L(i)));

        CurrentRefSel(i)  = Uc0(int(L(i)));
        NextRefSel(i)  = Ufn(int(L(i)));
        //
        Uf(i)  = UF(int(L(i)));
    }   

    count_sx ++;
}

void SelectionMatrices::ResetSelectionMx()
{
    count_sx = 0;
}

// ======================================================================================