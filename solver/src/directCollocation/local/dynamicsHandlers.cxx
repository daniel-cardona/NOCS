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


#include "directCollocation/local/dynamicsHandlers.hpp"

namespace nocs {

#ifdef pinocchio_compile

namespace pinocchioLib{

    void autoGenDynamics(const Eigen::VectorXd &states, const Eigen::VectorXd &controls,const double &tk, Eigen::VectorXd &derivatives, localCollocation &problem){

        int nDoF=problem.model.nq;

        Eigen::VectorXd q = states.head(nDoF);
        Eigen::VectorXd qd = states.tail(nDoF);
        Eigen::VectorXd tau = controls;

        pinocchio::aba(problem.model,problem.robot_data,q,qd,tau);


        //Return the vector \dot{x}
        derivatives.setZero(problem.nStates);
        derivatives<<qd,problem.robot_data.ddq;

    }//End autoGenDynamics


    void autoGenfGradient(const Eigen::VectorXd &states, const Eigen::VectorXd &controls,double &t, Eigen::MatrixXd &gradient, localCollocation &problem){

        //The gradient of the dynamics should be return using the sparsity pattern:

        //grad(f (x, u))=[ df | df ]
        //               [ -- | -- ]
        //               [ dx | du ]

        //dim(grad)=[nStates,nStates+nControls]

        int nDoF=problem.model.nq;
        int nu=problem.nControls;

        Eigen::VectorXd q = states.head(nDoF);
        Eigen::VectorXd v = states.tail(nDoF);
        Eigen::VectorXd tau = controls;

        // Allocate result container
        Eigen::MatrixXd djoint_acc_dq = Eigen::MatrixXd::Zero(nDoF,nDoF);
        Eigen::MatrixXd djoint_acc_dv = Eigen::MatrixXd::Zero(nDoF,nDoF);
        Eigen::MatrixXd djoint_acc_dtau = Eigen::MatrixXd::Zero(nDoF,nu);

        pinocchio::computeABADerivatives(problem.model,problem.robot_data,q,v,tau,djoint_acc_dq, djoint_acc_dv, djoint_acc_dtau);

        //Construct the gradient of the dynamics following the structure proposed on:
        //Exploiting sparsity in robot trajectory optimization with direct collocation and geometric algorithms
        //D. Cardona-Ortiz. A. Paz and G. Arechavaleta
        //IEEE International Conference on Robotics and Automation, Paris (France), 2020.

        //problem.robot_data.Minv

        gradient.setZero();

        gradient.block(0,nDoF,nDoF,nDoF)=Eigen::MatrixXd::Identity(nDoF,nDoF);

        gradient.block(nDoF,0,nDoF,nDoF)=djoint_acc_dq;

        gradient.block(nDoF,nDoF,nDoF,nDoF)=djoint_acc_dv;

        gradient.block(nDoF,problem.nStates,nDoF,nu)=djoint_acc_dtau;

    }//End autoGenfGradient

} //End pinocchio namespace

#endif

#ifdef rbdl_compile

namespace rbdlLib {


    void autoGenDynamics(const Eigen::VectorXd &states, const Eigen::VectorXd &controls,const double &tk, Eigen::VectorXd &derivatives, localCollocation &problem){

        int nDoF=problem.robot_model->q_size;

        Eigen::VectorXd q = states.head(nDoF);
        Eigen::VectorXd qd = states.tail(nDoF);
        Eigen::VectorXd tau = controls;

        Eigen::VectorXd qdd=Eigen::VectorXd::Zero(nDoF);

        RigidBodyDynamics::ForwardDynamics(*problem.robot_model,q,qd,tau,qdd);

        //Return the vector \dot{x}
        derivatives.setZero(problem.nStates);
        derivatives<<qd,qdd;

    }//End autoGenDynamics

}//End rbdlLib namespace

#endif

}//End nocs namespace

