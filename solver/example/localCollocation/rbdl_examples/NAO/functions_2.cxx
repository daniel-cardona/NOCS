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

    int nDoF=problem.robot_model->q_size;

    Eigen::VectorXd diff(nDoF);

    double cost=0;

    diff=problem.NLP.xk1.tail(nDoF)-states.tail(nDoF);

    cost=diff.transpose()*diff;

    //double cost=controls.transpose()*controls;

    return cost;


} //End integrand_cost

void nocs::Function::dae(const Eigen::VectorXd &states, const Eigen::VectorXd &controls,const double &tk, Eigen::VectorXd &derivatives,localCollocation &problem){


}//End dae

void nocs::Function::path(const Eigen::VectorXd &states, const Eigen::VectorXd &controls,const double &tk,Eigen::VectorXd &path,localCollocation &problem){



}//End path

void nocs::Function::events(const Eigen::VectorXd &initial_states, const Eigen::VectorXd &final_states, const double &t0, const double &tF, Eigen::VectorXd &e,localCollocation &problem){

    double q1_t0=  initial_states(0);       double q1d_t0=initial_states(24);
    double q2_t0=  initial_states(1);       double q2d_t0=initial_states(25);
    double q3_t0=  initial_states(2);       double q3d_t0=initial_states(26);
    double q4_t0=  initial_states(3);       double q4d_t0=initial_states(27);
    double q5_t0=  initial_states(4);       double q5d_t0=initial_states(28);
    double q6_t0=  initial_states(5);       double q6d_t0=initial_states(29);
    double q7_t0=  initial_states(6);       double q7d_t0=initial_states(30);
    double q8_t0=  initial_states(7);       double q8d_t0=initial_states(31);
    double q9_t0=  initial_states(8);       double q9d_t0=initial_states(32);
    double q10_t0= initial_states(9);       double q10d_t0=initial_states(33);
    double q11_t0= initial_states(10);      double q11d_t0=initial_states(34);
    double q12_t0= initial_states(11);      double q12d_t0=initial_states(35);
    double q13_t0= initial_states(12);      double q13d_t0=initial_states(36);
    double q14_t0= initial_states(13);      double q14d_t0=initial_states(37);
    double q15_t0= initial_states(14);      double q15d_t0=initial_states(38);
    double q16_t0= initial_states(15);      double q16d_t0=initial_states(39);
    double q17_t0= initial_states(16);      double q17d_t0=initial_states(40);
    double q18_t0= initial_states(17);      double q18d_t0=initial_states(41);
    double q19_t0= initial_states(18);      double q19d_t0=initial_states(42);
    double q20_t0= initial_states(19);      double q20d_t0=initial_states(43);
    double q21_t0= initial_states(20);      double q21d_t0=initial_states(44);
    double q22_t0= initial_states(21);      double q22d_t0=initial_states(45);
    double q23_t0= initial_states(22);      double q23d_t0=initial_states(46);
    double q24_t0= initial_states(23);      double q24d_t0=initial_states(47);


    double q1_tf= final_states(0);          double q1d_tf=  final_states(24);
    double q2_tf= final_states(1);          double q2d_tf=  final_states(25);
    double q3_tf= final_states(2);          double q3d_tf=  final_states(26);
    double q4_tf= final_states(3);          double q4d_tf=  final_states(27);
    double q5_tf= final_states(4);          double q5d_tf=  final_states(28);
    double q6_tf= final_states(5);          double q6d_tf=  final_states(29);
    double q7_tf= final_states(6);          double q7d_tf=  final_states(30);
    double q8_tf= final_states(7);          double q8d_tf=  final_states(31);
    double q9_tf= final_states(8);          double q9d_tf=  final_states(32);
    double q10_tf= final_states(9);         double q10d_tf= final_states(33);
    double q11_tf= final_states(10);        double q11d_tf= final_states(34);
    double q12_tf= final_states(11);        double q12d_tf= final_states(35);
    double q13_tf= final_states(12);        double q13d_tf= final_states(36);
    double q14_tf= final_states(13);        double q14d_tf= final_states(37);
    double q15_tf= final_states(14);        double q15d_tf= final_states(38);
    double q16_tf= final_states(15);        double q16d_tf= final_states(39);
    double q17_tf= final_states(16);        double q17d_tf= final_states(40);
    double q18_tf= final_states(17);        double q18d_tf= final_states(41);
    double q19_tf= final_states(18);        double q19d_tf= final_states(42);
    double q20_tf= final_states(19);        double q20d_tf= final_states(43);
    double q21_tf= final_states(20);        double q21d_tf= final_states(44);
    double q22_tf= final_states(21);        double q22d_tf= final_states(45);
    double q23_tf= final_states(22);        double q23d_tf= final_states(46);
    double q24_tf= final_states(23);        double q24d_tf= final_states(47);

    //Initial     |  //Final
    //states(q)   |  //states(q)
    e(0)=q1_t0;      e(48)=q1_tf;
    e(1)=q2_t0;      e(49)=q2_tf;
    e(2)=q3_t0;      e(50)=q3_tf;
    e(3)=q4_t0;      e(51)=q4_tf;
    e(4)=q5_t0;      e(52)=q5_tf;
    e(5)=q6_t0;      e(53)=q6_tf;
    e(6)=q7_t0;      e(54)=q7_tf;
    e(7)=q8_t0;      e(55)=q8_tf;
    e(8)=q9_t0;      e(56)=q9_tf;
    e(9)=q10_t0;     e(57)=q10_tf;
    e(10)=q11_t0;    e(58)=q11_tf;
    e(11)=q12_t0;    e(59)=q12_tf;
    e(12)=q13_t0;    e(60)=q13_tf;
    e(13)=q14_t0;    e(61)=q14_tf;
    e(14)=q15_t0;    e(62)=q15_tf;
    e(15)=q16_t0;    e(63)=q16_tf;
    e(16)=q17_t0;    e(64)=q17_tf;
    e(17)=q18_t0;    e(65)=q18_tf;
    e(18)=q19_t0;    e(66)=q19_tf;
    e(19)=q20_t0;    e(67)=q20_tf;
    e(20)=q21_t0;    e(68)=q21_tf;
    e(21)=q22_t0;    e(69)=q22_tf;
    e(22)=q23_t0;    e(70)=q23_tf;
    e(23)=q24_t0;    e(71)=q24_tf;

    //Initial     |  //Final
    //states(qd)  |  //states(qd)
    e(24)=q1d_t0;    e(72)=q1d_tf;
    e(25)=q2d_t0;    e(73)=q2d_tf;
    e(26)=q3d_t0;    e(74)=q3d_tf;
    e(27)=q4d_t0;    e(75)=q4d_tf;
    e(28)=q5d_t0;    e(76)=q5d_tf;
    e(29)=q6d_t0;    e(77)=q6d_tf;
    e(30)=q7d_t0;    e(78)=q7d_tf;
    e(31)=q8d_t0;    e(79)=q8d_tf;
    e(32)=q9d_t0;    e(80)=q9d_tf;
    e(33)=q10d_t0;   e(81)=q10d_tf;
    e(34)=q11d_t0;   e(82)=q11d_tf;
    e(35)=q12d_t0;   e(83)=q12d_tf;
    e(36)=q13d_t0;   e(84)=q13d_tf;
    e(37)=q14d_t0;   e(85)=q14d_tf;
    e(38)=q15d_t0;   e(86)=q15d_tf;
    e(39)=q16d_t0;   e(87)=q16d_tf;
    e(40)=q17d_t0;   e(88)=q17d_tf;
    e(41)=q18d_t0;   e(89)=q18d_tf;
    e(42)=q19d_t0;   e(90)=q19d_tf;
    e(43)=q20d_t0;   e(91)=q20d_tf;
    e(44)=q21d_t0;   e(92)=q21d_tf;
    e(45)=q22d_t0;   e(93)=q22d_tf;
    e(46)=q23d_t0;   e(94)=q23d_tf;
    e(47)=q24d_t0;   e(95)=q24d_tf;


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


    int nDoF=problem.robot_model->q_size;

    gradient.setZero(problem.nStates+problem.nControls);

    Eigen::VectorXd qd_xk1(nDoF);
    Eigen::VectorXd grad_qd(nDoF);
    Eigen::VectorXd grad_qdk1(nDoF);

    qd_xk1=problem.NLP.xk1.tail(nDoF);

    if(problem.NLP.gradient_eval==1){

        //Compute the gradient of the cost function w.r.t. the k instant
        grad_qd=(2*states.tail(nDoF))-(2*qd_xk1);
        gradient.segment(nDoF,nDoF)=grad_qd;

    }

    if(problem.NLP.gradient_eval==2){

        //In particular this cost function has a dependency on the k+1 instant, compute the gradient w.r.t that derivative

        grad_qdk1=(2*qd_xk1)-(2*states.tail(nDoF));
        problem.NLP.gradxk1.segment(nDoF,nDoF)=grad_qdk1;

    }

    /*gradient.setZero(problem.nStates+problem.nControls);

    gradient.tail(problem.nControls)=2*controls;*/



}//End costGradient

void nocs::Function::analytical::fGradient(const Eigen::VectorXd &states,const Eigen::VectorXd &controls, double &t, Eigen::MatrixXd &gradient, localCollocation &problem){

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



}

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


}//End eventGradient
