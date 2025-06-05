/*
 *
 * Copyright (C) 2020
 * Daniel Cardona-Ortiz <daniel.cardona@cinvestav.edu.mx>, Gustavo Arechavaleta <garechav@cinvestav.edu.mx>
 * CINVESTAV - Saltillo Campus
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


#include "directCollocation/local/userFunctions.hpp"


//-------------------------------------------------------------------------
//----------------------Value of the Mayer term ------------------------
//-------------------------------------------------------------------------

 double nocs::Function::endpoint_cost(const Eigen::VectorXd &x0,const Eigen::VectorXd &xF, const double &t0,const double &tf,localCollocation &problem){

     return 0;

 }//end endpoint_cost

 //-------------------------------------------------------------------------
 //----------------------Value of the Lagrange term ------------------------
 //-------------------------------------------------------------------------

double nocs::Function::integrand_cost(const Eigen::VectorXd &states,const Eigen::VectorXd &controls, const double &tk, localCollocation &problem){

    int nDoF=problem.model.nq;

//    Eigen::VectorXd diff(nDoF);

//    double w=1.0;

//    double cost2=0;

//    diff=problem.NLP.xk1.tail(nDoF)-states.tail(nDoF);

//    cost2=diff.transpose()*diff;

    //double cost=states.tail(nDoF).transpose()*states.tail(nDoF);

    double cost=controls.transpose()*controls;

    return cost;

} //End integrand_cost


//---------------------------------------------------------------------
//----------------------Dynamics of the system ------------------------
//---------------------------------------------------------------------

void nocs::Function::dae(const Eigen::VectorXd &states, const Eigen::VectorXd &controls,const double &tk,
                   Eigen::VectorXd &derivatives,localCollocation &problem){


}//End dae

//---------------------------------------------------------------------
//---------------------- Path constraints -----------------------------
//---------------------------------------------------------------------

void nocs::Function::path(const Eigen::VectorXd &states, const Eigen::VectorXd &controls,const double &tk,
                    Eigen::VectorXd &path,localCollocation &problem){

}// End path constraints

//---------------------------------------------------------------------
//------------------- Boundary constraints ----------------------------
//---------------------------------------------------------------------

void nocs::Function::events(const Eigen::VectorXd &initial_states, const Eigen::VectorXd &final_states,
                      const double &t0, const double &tF, Eigen::VectorXd &e,localCollocation &problem){


    e.head(problem.nStates)=initial_states;
    e.tail(problem.nStates)=final_states;


}//End events

//---------------------------------------------------------------------
//---------- Gradient of the Lagrange term constraints ----------------
//---------------------------------------------------------------------

void nocs::Function::analytical::costGradient(const Eigen::VectorXd &states, const Eigen::VectorXd &controls,const double& t,
                                        Eigen::VectorXd &gradient, localCollocation &problem){

    //The gradient of the Lagrange term should be return using the sparsity pattern:

    //grad(L (x, u, t))=[ dL | dL ]
    //                  [ -- | -- ]
    //                  [ dx | du ]

    //dim(grad)=[1,nStates+nControls]
    int nDoF=problem.model.nq;

    gradient.setZero(problem.nStates+problem.nControls);

    gradient.tail(problem.nControls)=2*controls;

    //gradient.segment(problem.model.nq,problem.model.nq)=2*states.tail(nDoF);


}//End costGradient


//---------------------------------------------------------------------
//------------------ Jacobian of the dynamics -------------------------
//---------------------------------------------------------------------

void nocs::Function::analytical::fGradient(const Eigen::VectorXd &states, const Eigen::VectorXd &controls, double &t,
                                     Eigen::MatrixXd &gradient, localCollocation &problem){

    //The gradient of the dynamics should be return using the sparsity pattern:

    //grad(f (x, u))=[ df | df ]
    //               [ -- | -- ]
    //               [ dx | du ]

    //dim(grad)=[nStates,nStates+nControls]

}

//---------------------------------------------------------------------
//------------------ Jacobian of the path constraints -------------------------
//---------------------------------------------------------------------

void nocs::Function::analytical::pathGradient(Eigen::VectorXd &states, Eigen::VectorXd &controls,double &t,
                                        Eigen::MatrixXd &gradient, localCollocation &problem){

    //The gradient of the path constraints should be return using the sparsity pattern:

    //grad(g (x, u))=[ dg | dg ]
    //               [ -- | -- ]
    //               [ dx | du ]

    //dim(grad)=[nPath,nStates+nControls]
}

//-----------------------------------------------------------------------------
//------------------ Jacobian of boundary constraints -------------------------
//-----------------------------------------------------------------------------

void nocs::Function::analytical::eventGradient(Eigen::VectorXd &x0,Eigen::VectorXd &xN,
                                         double &t0,double &tF,Eigen::VectorXd &Dt0,
                                         Eigen::VectorXd &DtF,Eigen::MatrixXd &De0,
                                         Eigen::MatrixXd &DeF, localCollocation &problem){

    //The gradient of the event constraints should be return using the sparsity pattern:

    //Dt0=[ de ]
    //    [ -- ]  dim=(nEvents,1)
    //    [ dt0]

    //DtF=[ de ]
    //    [ -- ]  dim=(nEvents,1)
    //    [ dtF]

    //De0=[ de ]
    //    [ -- ]  dim=(nEvents,nStates)
    //    [ dx0]

    //DeF=[ de ]
    //    [ -- ]  dim=(nEvents,nStates)
    //    [ dxN]


    Dt0.setZero(problem.nEvents); //No derivatives w.r.t. initial time
    DtF.setZero(problem.nEvents); //No derivatives w.r.t final time

    De0.setZero(problem.nEvents,problem.nStates);
    De0.block(0,0,problem.nStates,problem.nStates)=Eigen::MatrixXd::Identity(problem.nStates,problem.nStates);

    DeF.setZero(problem.nEvents,problem.nStates);
    DeF.block(problem.nStates,0,problem.nStates,problem.nStates)=Eigen::MatrixXd::Identity(problem.nStates,problem.nStates);

}//end eventGradient
