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

 double nocs::Function::endpoint_cost(const Eigen::VectorXd &x0,const Eigen::VectorXd &xF, const double &t0,const double &tf,localCollocation &problem){

     return 0;

 }//end endpoint_cost


double nocs::Function::integrand_cost(const Eigen::VectorXd &states,const Eigen::VectorXd &controls, const double &tk,localCollocation &problem){

     int nDoF=problem.model.nq;

     double cost=states.tail(nDoF).transpose()*states.tail(nDoF);

     return cost;

} //End integrand_cost

void nocs::Function::dae(const Eigen::VectorXd &states, const Eigen::VectorXd &controls,const double &tk, Eigen::VectorXd &derivatives,localCollocation &problem){

}//End dae


void nocs::Function::path(const Eigen::VectorXd &states, const Eigen::VectorXd &controls,const double &tk,Eigen::VectorXd &path,localCollocation &problem){

}

void nocs::Function::events(const Eigen::VectorXd &initial_states, const Eigen::VectorXd &final_states, const double &t0, const double &tF, Eigen::VectorXd &e,localCollocation &problem){

    double q1_t0=  initial_states(0);       double q1d_t0=initial_states(8);
    double q2_t0=  initial_states(1);       double q2d_t0=initial_states(9);
    double q3_t0=  initial_states(2);       double q3d_t0=initial_states(10);
    double q4_t0=  initial_states(3);       double q4d_t0=initial_states(11);
    double q5_t0=  initial_states(4);       double q5d_t0=initial_states(12);
    double q6_t0=  initial_states(5);       double q6d_t0=initial_states(13);
    double q7_t0=  initial_states(6);       double q7d_t0=initial_states(14);
    double q8_t0=  initial_states(7);       double q8d_t0=initial_states(15);



    double q1_tf= final_states(0);          double q1d_tf=  final_states(8);
    double q2_tf= final_states(1);          double q2d_tf=  final_states(9);
    double q3_tf= final_states(2);          double q3d_tf=  final_states(10);
    double q4_tf= final_states(3);          double q4d_tf=  final_states(11);
    double q5_tf= final_states(4);          double q5d_tf=  final_states(12);
    double q6_tf= final_states(5);          double q6d_tf=  final_states(13);
    double q7_tf= final_states(6);          double q7d_tf=  final_states(14);
    double q8_tf= final_states(7);          double q8d_tf=  final_states(15);


    //Initial     |  //Final
    //states(q)   |  //states(q)
    e(0)=q1_t0;      e(16)=q1_tf;
    e(1)=q2_t0;      e(17)=q2_tf;
    e(2)=q3_t0;      e(18)=q3_tf;
    e(3)=q4_t0;      e(19)=q4_tf;
    e(4)=q5_t0;      e(20)=q5_tf;
    e(5)=q6_t0;      e(21)=q6_tf;
    e(6)=q7_t0;      e(22)=q7_tf;
    e(7)=q8_t0;      e(23)=q8_tf;


    //Initial     |  //Final
    //states(qd)  |  //states(qd)
    e(8)=q1d_t0;     e(24)=q1d_tf;
    e(9)=q2d_t0;     e(25)=q2d_tf;
    e(10)=q3d_t0;    e(26)=q3d_tf;
    e(11)=q4d_t0;    e(27)=q4d_tf;
    e(12)=q5d_t0;    e(28)=q5d_tf;
    e(13)=q6d_t0;    e(29)=q6d_tf;
    e(14)=q7d_t0;    e(30)=q7d_tf;
    e(15)=q8d_t0;    e(31)=q8d_tf;

}//End events


//************************************************************************************************
//****************************** ANALYTIC GRADIENT FUNCTIONS *************************************
//************************************************************************************************

void nocs::Function::analytical::costGradient(const Eigen::VectorXd &states, const Eigen::VectorXd &controls,const double& t, Eigen::VectorXd &gradient, localCollocation &problem){

    //The gradient of the Lagrange term should be return using the sparsity pattern:

    //grad(L (x, u, t))=[ dL | dL ]
    //                  [ -- | -- ]
    //                  [ dx | du ]

    //dim(grad)=[1,nStates+nControls]

    int nDoF=problem.model.nq;

    gradient.setZero(problem.nStates+problem.nControls);

    //gradient.tail(problem.nControls)=2*controls;

    gradient.segment(problem.model.nq,problem.model.nq)=2*states.tail(nDoF);

}//End costGradient

void nocs::Function::analytical::fGradient(const Eigen::VectorXd &states, const Eigen::VectorXd &controls, double &t, Eigen::MatrixXd &gradient, localCollocation &problem){

    //The gradient of the dynamics should be return using the sparsity pattern:

    //grad(f (x, u))=[ df | df ]
    //               [ -- | -- ]
    //               [ dx | du ]

    //dim(grad)=[nStates,nStates+nControls]

} //End fGradient

void nocs::Function::analytical::pathGradient(Eigen::VectorXd &states, Eigen::VectorXd &controls,double &t, Eigen::MatrixXd &gradient, localCollocation &problem){

    //The gradient of the path constraints should be return using the sparsity pattern:

    //grad(g (x, u))=[ dg | dg ]
    //               [ -- | -- ]
    //               [ dx | du ]

    //dim(grad)=[nPath,nStates+nControls]



}//End pathGradient

void nocs::Function::analytical::eventGradient(Eigen::VectorXd &x0,Eigen::VectorXd &xN,double &t0,double &tF,Eigen::VectorXd &Dt0, Eigen::VectorXd &DtF,Eigen::MatrixXd &De0, Eigen::MatrixXd &DeF, localCollocation &problem){

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

    DeF.setZero(problem.nEvents,problem.nStates);

    De0.block(0,0,problem.nStates,problem.nStates)=Eigen::MatrixXd::Identity(problem.nStates,problem.nStates);

    DeF.block(problem.nStates,0,problem.nStates,problem.nStates)=Eigen::MatrixXd::Identity(problem.nStates,problem.nStates);


}//End event gradient
