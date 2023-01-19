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


#include "directCollocation/local/nocsLocal.hpp"

#include "directCollocation/local/fcnGenerator.hpp"


#include "directCollocation/local/blockOperations.hpp"

#include <iostream>
#include <chrono>

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODEL_DIR "../../../../../data/pinocchio_models"
#endif

int main(){


    const std::string urdf_filename = PINOCCHIO_MODEL_DIR + std::string("/nao_inertial_python.urdf");

    //Set the main data of the optimal control problem

    int nStates=48;
    int nControls=24;
    int nEvents=96;
    int nPath=6;
    int nDiscretePoints=30;

    //&-------------- Build the Robot using the rigid body dynamics library pinocchio ----------------------&

    nocs::localCollocation problem(nStates,nControls,nDiscretePoints,nPath,nEvents,urdf_filename);


    Eigen::VectorXd q(24);
    Eigen::VectorXd qd(24);
    Eigen::VectorXd qdd(24);

    q<<  0.0611, -1.0385,2.0909,-1.0385,0.4154,-0.0018,
         0.1885,0.1030,-0.0035,-1.4608,-0.0000,-1.1833,
        -0.0000,1.0454,-1.3247,1.5673,0.0785,0.0000,
        -0.0018,-0.5463,-0.1065,-0.0000,-0.0087,0.3944;

    qd.setOnes();

    qdd.setZero();


          Matrix6x dh_dq(Matrix6x::Zero(6,problem.model.nv));
          Matrix6x dhdot_dq(Matrix6x::Zero(6,problem.model.nv));
          Matrix6x dhdot_dv(Matrix6x::Zero(6,problem.model.nv));
          Matrix6x dhdot_da(Matrix6x::Zero(6,problem.model.nv));


    pinocchio::computeCentroidalDynamicsDerivatives(problem.model,problem.robot_data,q,qd,qdd,dh_dq,dhdot_dq,dhdot_dv,dhdot_da);

    //cout<<dhdot_da.transpose()<<endl;

    Eigen::MatrixXd analytical_gradient(nPath,nStates+nControls);
    analytical_gradient.setZero();

    analytical_gradient.block(0,0,6,problem.model.nq)=dh_dq;
    analytical_gradient.block(0,problem.model.nq,6,problem.model.nv)=dhdot_da;

    cout<<"................"<<endl;

    Eigen::VectorXd states(nStates);
    Eigen::VectorXd controls(nControls);
    Eigen::MatrixXd gradient(nPath,nStates+nControls);

    double t=10;

    states<<q,qd;
    controls.setRandom();
    gradient.setZero();

    nocs::computeNumericalJacobian_path(states, controls, t, gradient, problem);

    Eigen::MatrixXd dh_dq2(nPath,problem.model.nq);

    dh_dq2=gradient.block(0,problem.model.nq,6,problem.model.nq);


    Eigen::MatrixXd resta(nPath,problem.nStates+problem.nControls);

    resta=analytical_gradient-gradient;

    cout<<resta.transpose()<<endl;



    return 0;


}
