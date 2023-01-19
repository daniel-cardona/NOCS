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

    //Cost function

    double u1=controls(0);  //Tau 1
    double u2=controls(1);  //Tau 2

    //Transition feature
    double f1=pow(u1,2.0)+ pow(u2,2.0);

    return f1;

} //End integrand_cost

void nocs::Function::dae(const Eigen::VectorXd &states, const Eigen::VectorXd &controls,const double &tk, Eigen::VectorXd &derivatives,localCollocation &problem){

    //Variables of the dynamic system

        double l1,r1,l2,r2,m1,m2,I1,I2,gr;

          l1=0.5;       l2=0.5;
          r1=l1/2;      r2=l2/2;

          m1=1;   m2=1;
          I1=0.5;   I2=0.5;

          gr=9.81;


       //Obtain system states

          double q1=states(0);
          double q2=states(1);
          double q1d=states(2);
          double q2d=states(3);

       //Obtain controls

          double T1=controls(0);
          double T2=controls(1);

          Eigen::MatrixXd H(2,2);
          Eigen::MatrixXd C(2,2);
          Eigen::VectorXd g(2);

       //Constant variables

          double a1=m1*pow(r1,2) + m2*( pow(l1,2)+pow(r2,2) ) + I1 +I2;
          double a2=m2*l1*r2;
          double a3=m2*pow(r2,2) +I2;

          double b1=l1*m2+r1*m1;
          double b2=r2*m2;

       //Inertia matrix

          H(0,0)=a1+2*a2*cos(q2);
          H(0,1)=a3+a2*cos(q2);
          H(1,0)=a3+a2*cos(q2);
          H(1,1)=a3;

       //Coriollis matrix

          C(0,0)=-a2*q2d*sin(q2);
          C(0,1)=-a2*(q1d+q2d)*sin(q2);
          C(1,0)=a2*q1d*sin(q2);
          C(1,1)=0;

       //Gravity vector

          g(0)=b1*gr*cos(q1)+b2*gr*cos(q1+q2);
          g(1)=b2*gr*cos(q1+q2);

       Eigen::VectorXd qd(2);
       Eigen::VectorXd qdd(2);

       qd<<q1d,q2d;

        qdd=H.inverse()*( (-C*qd) - g + controls);

        //Return the derivatives

          derivatives(0)=q1d;
          derivatives(1)=q2d;
          derivatives(2)=qdd(0);
          derivatives(3)=qdd(1);

}//End dae

void nocs::Function::path(const Eigen::VectorXd &states, const Eigen::VectorXd &controls,const double &tk,Eigen::VectorXd &path,localCollocation &problem){





}// End path constraints


void nocs::Function::events(const Eigen::VectorXd &initial_states, const Eigen::VectorXd &final_states, const double &t0, const double &tF, Eigen::VectorXd &e,localCollocation &problem){

    double q1_t0  = initial_states(0);
    double q2_t0  = initial_states(1);
    double q1d_t0 = initial_states(2);
    double q2d_t0 = initial_states(3);

    double q1_tF  = final_states(0);
    double q2_tF  = final_states(1);
    double q1d_tF = final_states(2);
    double q2d_tF = final_states(3);

    double l1=0.5;      double l2=0.5;

    e.setZero(8); //8 event constraints

    e(0) = q1_t0;
    e(1) = q2_t0;
    e(2) = q1d_t0;
    e(3) = q2d_t0;

    e(4)=l1*cos(q1_tF)+l2*cos(q1_tF+q2_tF);
    e(5)=l1*sin(q1_tF)+l2*sin(q1_tF+q2_tF);

    //e(4)=q1_tF;
    //e(5)=q2_tF;


    e(6) = q1d_tF;
    e(7) = q2d_tF;

}//End events


void nocs::Function::analytical::fGradient(Eigen::VectorXd &states, Eigen::VectorXd &controls, double &t, Eigen::MatrixXd &gradient, localCollocation &problem){

    //The gradient of the dynamics should be return using the sparsity pattern:

    //grad(f (x, u))=[ df | df ]
    //               [ -- | -- ]
    //               [ dx | du ]

    //dim(grad)=[nStates,nStates+nControls]

    double l1,r1,l2,r2,m1,m2,I2,I1,g;

          l1=0.5;       l2=0.5;
          r1=l1/2;     r2=l2/2;

          m1=1;   m2=1;
          I1=0.5;   I2=0.5;

          g=9.81;


    double a1=m1*pow(r1,2) + m2*( pow(l1,2)+pow(r2,2) ) + I1 +I2;
    double a2=m2*l1*r2;
    double a3=m2*pow(r2,2) +I2;

    double b1=l1*m2+r1*m1;
    double b2=r2*m2;

        //States
        double q1=states(0);
        double q2=states(1);
        double q1d=states(2);
        double q2d=states(3);

        //Controls
        double u1=controls(0);
        double u2=controls(1);

        gradient.setZero(4,6);

        //Following the proposed structure in:
        //Exploiting sparsity in robot trajectory optimization with direct collocation and geometric algorithms
        //D. Cardona-Ortiz. A. Paz and G. Arechavaleta

        /*
              q1 q2 qd1 qd2 u1 u2
        q1d   0   0  1   0   0  0
        q2d   0   0  0   1   0  0
        q1dd |x   x||z   z| |-  -|
        q2dd |x   x||z   z| |-  -|

        */

        gradient.row(0)<<0,0,1,0,0,0;
        gradient.row(1)<<0,0,0,1,0,0;

       Eigen::MatrixXd x(2,2);

        //q1dd wrt q1
        x(0,0)=((a3*b1*sin(q1)*9.81E2-a2*b2*sin(q1+q2)*cos(q2)*9.81E2)*(-1.0/1.0E2))/(-a1*a3+(a2*a2)*pow(cos(q2),2.0)+a3*a3);
        //q2dd wrt q1
        x(1,0)=(a3*b1*sin(q1)*9.81E2-a1*b2*sin(q1+q2)*9.81E2+a3*b2*sin(q1+q2)*9.81E2-a2*b2*sin(q1+q2)*cos(q2)*9.81E2+a2*b1*cos(q2)*sin(q1)*9.81E2)/(a1*a3*-1.0E2+(a2*a2)*pow(cos(q2),2.0)*1.0E2+(a3*a3)*1.0E2);
        //q1dd wrt q2
        x(0,1)=((a2*u2*sin(q2)*1.0E2+(a2*a2)*(q1d*q1d)*pow(cos(q2),2.0)*1.0E2-(a2*a2)*(q1d*q1d)*pow(sin(q2),2.0)*1.0E2-a2*b2*cos(q1+q2)*sin(q2)*9.81E2-a2*b2*sin(q1+q2)*cos(q2)*9.81E2+a2*a3*(q1d*q1d)*cos(q2)*1.0E2+a2*a3*(q2d*q2d)*cos(q2)*1.0E2+a2*a3*q1d*q2d*cos(q2)*2.0E2)*(-1.0/1.0E2))/(-a1*a3+(a2*a2)*pow(cos(q2),2.0)+a3*a3)-((a2*a2)*cos(q2)*sin(q2)*1.0/pow(-a1*a3+(a2*a2)*pow(cos(q2),2.0)+a3*a3,2.0)*(a3*u1*1.0E2-a3*u2*1.0E2-a2*u2*cos(q2)*1.0E2-a3*b1*cos(q1)*9.81E2+a2*b2*cos(q1+q2)*cos(q2)*9.81E2+(a2*a2)*(q1d*q1d)*cos(q2)*sin(q2)*1.0E2+a2*a3*(q1d*q1d)*sin(q2)*1.0E2+a2*a3*(q2d*q2d)*sin(q2)*1.0E2+a2*a3*q1d*q2d*sin(q2)*2.0E2))/5.0E1;
        //q2dd wrt q2
        x(1,1)=(a2*u1*sin(q2)*-1.0E2+a2*u2*sin(q2)*2.0E2+(a2*a2)*(q1d*q1d)*pow(cos(q2),2.0)*2.0E2+(a2*a2)*(q2d*q2d)*pow(cos(q2),2.0)*1.0E2-(a2*a2)*(q1d*q1d)*pow(sin(q2),2.0)*2.0E2-(a2*a2)*(q2d*q2d)*pow(sin(q2),2.0)*1.0E2-a1*b2*sin(q1+q2)*9.81E2+a3*b2*sin(q1+q2)*9.81E2+(a2*a2)*q1d*q2d*pow(cos(q2),2.0)*2.0E2-(a2*a2)*q1d*q2d*pow(sin(q2),2.0)*2.0E2-a2*b2*cos(q1+q2)*sin(q2)*9.81E2-a2*b2*sin(q1+q2)*cos(q2)*9.81E2+a1*a2*(q1d*q1d)*cos(q2)*1.0E2+a2*a3*(q2d*q2d)*cos(q2)*1.0E2+a2*b1*cos(q1)*sin(q2)*9.81E2+a2*a3*q1d*q2d*cos(q2)*2.0E2)/(a1*a3*-1.0E2+(a2*a2)*pow(cos(q2),2.0)*1.0E2+(a3*a3)*1.0E2)+((a2*a2)*cos(q2)*sin(q2)*1.0/pow(-a1*a3+(a2*a2)*pow(cos(q2),2.0)+a3*a3,2.0)*(a1*u2*-1.0E2+a3*u1*1.0E2+a2*u1*cos(q2)*1.0E2-a2*u2*cos(q2)*2.0E2+a1*b2*cos(q1+q2)*9.81E2-a3*b2*cos(q1+q2)*9.81E2-a3*b1*cos(q1)*9.81E2+a2*b2*cos(q1+q2)*cos(q2)*9.81E2+(a2*a2)*(q1d*q1d)*cos(q2)*sin(q2)*2.0E2+(a2*a2)*(q2d*q2d)*cos(q2)*sin(q2)*1.0E2-a2*b1*cos(q1)*cos(q2)*9.81E2+a1*a2*(q1d*q1d)*sin(q2)*1.0E2+a2*a3*(q2d*q2d)*sin(q2)*1.0E2+(a2*a2)*q1d*q2d*cos(q2)*sin(q2)*2.0E2+a2*a3*q1d*q2d*sin(q2)*2.0E2))/5.0E1;


        Eigen::MatrixXd z(2,2);
        //q1dd wrt q1d
        z(0,0) = (((a2*a2)*q1d*cos(q2)*sin(q2)*2.0E2+a2*a3*q1d*sin(q2)*2.0E2+a2*a3*q2d*sin(q2)*2.0E2)*(-1.0/1.0E2))/(-a1*a3+(a2*a2)*pow(cos(q2),2.0)+a3*a3);
        //q2dd wrt q1d
        z(1,0) = ((a2*a2)*q1d*cos(q2)*sin(q2)*4.0E2+(a2*a2)*q2d*cos(q2)*sin(q2)*2.0E2+a1*a2*q1d*sin(q2)*2.0E2+a2*a3*q2d*sin(q2)*2.0E2)/(a1*a3*-1.0E2+(a2*a2)*pow(cos(q2),2.0)*1.0E2+(a3*a3)*1.0E2);
        //q1dd wrt q2d
        z(0,1)=((a2*a3*q1d*sin(q2)*2.0E2+a2*a3*q2d*sin(q2)*2.0E2)*(-1.0/1.0E2))/(-a1*a3+(a2*a2)*pow(cos(q2),2.0)+a3*a3);
        //q2dd wrt q2d
        z(1,1)=((a2*a2)*q1d*cos(q2)*sin(q2)*2.0E2+(a2*a2)*q2d*cos(q2)*sin(q2)*2.0E2+a2*a3*q1d*sin(q2)*2.0E2+a2*a3*q2d*sin(q2)*2.0E2)/(a1*a3*-1.0E2+(a2*a2)*pow(cos(q2),2.0)*1.0E2+(a3*a3)*1.0E2);


        Eigen::MatrixXd jacControls(2,2);
        Eigen::MatrixXd H(2,2);

        H(0,0)=a1+2*a2*cos(q2);
        H(0,1)=a3+a2*cos(q2);
        H(1,0)=a3+a2*cos(q2);
        H(1,1)=a3;

        jacControls=H.inverse();

        //Set the gradient

        gradient.block(2,0,2,2)=x;          //Jacobian of the acceleration w.r.t. position
        gradient.block(2,2,2,2)=z;          //Jacobian of the acceleration w.r.t. velocities
        gradient.block(2,4,2,2)=jacControls;


    } //End fGradient

void nocs::Function::analytical::pathGradient(Eigen::VectorXd &states, Eigen::VectorXd &controls,double &t, Eigen::MatrixXd &gradient, localCollocation &problem){

    //The gradient of the path constraints should be return using the sparsity pattern:

    //grad(g (x, u))=[ dg | dg ]
    //               [ -- | -- ]
    //               [ dx | du ]

    //dim(grad)=[nPath,nStates+nControls]



}
void nocs::Function::analytical::eventGradient(Eigen::VectorXd &initial_states,Eigen::VectorXd &final_states,double &t0,double &tF,Eigen::MatrixXd &gradient0, Eigen::MatrixXd &gradienttF, localCollocation &problem){

    //The gradient of the event constraints should be return using the sparsity pattern:

    //grad(e (t0,x0))=[ de  | de  ]
    //                [ --  | --  ]
    //                [ dt0 | dx0 ]


    //grad(e (tF,xF))=[ de  | de  ]
    //                [ --  | --  ]
    //                [ dtF | dxF ]

    //dim(grad)=[nEvents,nStates+1]

    //e(0) = q1_t0;
    //e(1) = q2_t0;
    //e(2) = q1d_t0;
    //e(3) = q2d_t0;

    //e(4)=l1*cos(q1_tF)+l2*cos(q1_tF+q2_tF);
    //e(5)=l1*sin(q1_tF)+l2*sin(q1_tF+q2_tF);

    //e(6) = q1d_tF;
    //e(7) = q2d_tF;

    double l1,l2;

    l1=0.5; l2=0.5;

    double q1_tF  = final_states(0);
    double q2_tF  = final_states(1);

    gradient0.setZero(8,5);

    gradient0.row(0)<<0,1,0,0,0;
    gradient0.row(1)<<0,0,1,0,0;
    gradient0.row(2)<<0,0,0,1,0;
    gradient0.row(3)<<0,0,0,0,1;


    double e4_q1=-l1*sin(q1_tF)-l2*sin(q1_tF+q2_tF);
    double e4_q2=-l2*sin(q1_tF+q2_tF);

    double e5_q1=l1*cos(q1_tF)+l2*cos(q1_tF+q2_tF);
    double e5_q2=l2*cos(q1_tF+q2_tF);

    gradienttF.setZero(8,5);

    gradienttF.row(4)<<0,e4_q1,e4_q2,0,0;
    gradienttF.row(5)<<0,e5_q1,e5_q2,0,0;
    gradienttF.row(6)<<0,  0  , 0   ,1,0;
    gradienttF.row(7)<<0,  0  , 0   ,0,1;



}

