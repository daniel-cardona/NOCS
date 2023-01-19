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


void systemDynamics(const Eigen::VectorXd &x, const Eigen::VectorXd &u, Eigen::VectorXd &xdd);


 double nocs::Function::endpoint_cost(const Eigen::VectorXd &x0,const Eigen::VectorXd &xF, const double &t0,const double &tf,localCollocation &problem){

     return 0;

 }//end endpoint_cost


double nocs::Function::integrand_cost(const Eigen::VectorXd &states,const Eigen::VectorXd &controls, const double &tk,localCollocation &problem){

    //Cost function

    double u1=controls(0);  //Tau 1
    double u2=controls(1);  //Tau 2
    double u3=controls(2);  //Tau 3

    double L=pow(u1,2.0)+ pow(u2,2.0) + pow(u3,2.0);

    return L;

} //End integrand_cost

void nocs::Function::dae(const Eigen::VectorXd &states, const Eigen::VectorXd &controls,const double &tk, Eigen::VectorXd &derivatives,localCollocation &problem){

    Eigen::VectorXd dx(states.size());
    systemDynamics(states,controls,dx);

    derivatives.setZero(states.size());

    derivatives=dx;

}//End dae

void nocs::Function::path(const Eigen::VectorXd &states, const Eigen::VectorXd &controls,const double &tk,Eigen::VectorXd &path,localCollocation &problem){

    double l1,l2,l3;
    double q1, q2,q3,q4,q5;
    double lambda;

    l1=1;       l2=1;     l3=1;


    //Obtain robot states

    q1=states(0);
    q2=states(1);
    q3=states(2);

    //Obtain object states

    q4=states(3);     //x-axis
    q5=states(4);     //y-axis

    //Obtain controls

    lambda=controls(3);

    //Obtain the angle of the contact surface

    double x_q=l1*cos(q1)+l2*cos(q1+q2)+l3*cos(q1+q2+q3);
    double y_q=l1*sin(q1)+l2*sin(q1+q2)+l3*sin(q1+q2+q3);

    //Non penetration constraint
    //double path_cns_1=sqrt(pow(x_q-q4,2.0)+pow(y_q-q5,2.0))-0.25;

    double path_cns_1=pow(x_q-q4,2.0)+pow(y_q-q5,2.0)-0.0625;

    //Complementary contact cosntraint
    double path_cns_2=(path_cns_1)*lambda;

    path(0)=path_cns_1;
    path(1)=path_cns_2;



}// End path constraints


void nocs::Function::events(const Eigen::VectorXd &initial_states, const Eigen::VectorXd &final_states, const double &t0, const double &tF, Eigen::VectorXd &e,localCollocation &problem){

    double q1_t0  = initial_states(0);
    double q2_t0  = initial_states(1);
    double q3_t0  = initial_states(2);
    double q4_t0  = initial_states(3);
    double q5_t0  = initial_states(4);

    double q1d_t0 = initial_states(5);
    double q2d_t0 = initial_states(6);
    double q3d_t0 = initial_states(7);
    double q4d_t0 = initial_states(8);
    double q5d_t0 = initial_states(9);


    double q1_tF  = final_states(0);
    double q2_tF  = final_states(1);
    double q3_tF  = final_states(2);
    double q4_tF  = final_states(3);
    double q5_tF  = final_states(4);

    double q1d_tF = final_states(5);
    double q2d_tF = final_states(6);
    double q3d_tF = final_states(7);
    double q4d_tF = final_states(8);
    double q5d_tF = final_states(9);


    e.setZero(17); //17 event constraints

    e(0) = q1_t0;
    e(1) = q2_t0;
    e(2) = q3_t0;
    e(3) = q4_t0;
    e(4) = q5_t0;
    e(5) = q1d_t0;
    e(6) = q2d_t0;
    e(7) = q3d_t0;
    e(8) = q4d_t0;
    e(9) = q5d_t0;

    e(10) =  q4_tF;
    e(11) =  q5_tF;
    e(12) =  q1d_tF;
    e(13) =  q2d_tF;
    e(14) =  q3d_tF;
    e(15) =  q4d_tF;
    e(16)=   q5d_tF;

}//End events


void nocs::Function::analytical::costGradient(const Eigen::VectorXd &states, const Eigen::VectorXd &controls,const double& t, Eigen::VectorXd &gradient, localCollocation &problem){

    //The gradient of the Lagrange term should be return using the sparsity pattern:

    //grad(L (x, u, t))=[ dL | dL ]
    //                  [ -- | -- ]
    //                  [ dx | du ]

    //dim(grad)=[1,nStates+nControls]

    //double f1=pow(u1,2.0)+ pow(u2,2.0);


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



}//end eventGradient


void systemDynamics(const Eigen::VectorXd &x, const Eigen::VectorXd &u, Eigen::VectorXd &xdd){


    //Variables of the system

    double m1,m2,m3;
    double l1,l2,l3;
    double lc1, lc2, lc3;
    double q1, q2, q3,q4,q5;
    double qd1,qd2, qd3,qd4,qd5;
    double I1z, I2z, I3z;
    double lambda;
    double r;

    l1=1;       l2=1;     l3=1;
    lc1=l1/2;     lc2=l2/2;   lc3=l3/2;

    m1=0.5;   m2=0.5;     m3=0.5;
    I1z=1;    I2z=1;      I3z=1;

    q1=x(0); //Joint 1
    q2=x(1); //Joint 2
    q3=x(2); //Joint 3
    q4=x(3); //x-axis circle
    q5=x(4); //y-axis circle

    qd1=x(5); //Velocity Joint 1
    qd2=x(6); //Velocity Joint 2
    qd3=x(7); //Velocity Joint 3
    qd4=x(8); //Velocity of the circle in the x-axis
    qd5=x(9); //Velocity of the circle in the y-axis

    lambda=u(3);
    //----------INERTIA MATRIX---------------

    Eigen::MatrixXd H(3,3);

    H(0,0)=I1z+I2z+I3z+(l1*l1)*m2+(l1*l1)*m3+(l2*l2)*m3+(lc1*lc1)*m1+(lc2*lc2)*m2+(lc3*lc3)*m3+l1*lc3*m3*cos(q2+q3)*2.0+l1*l2*m3*cos(q2)*2.0+l1*lc2*m2*cos(q2)*2.0+l2*lc3*m3*cos(q3)*2.0;
    H(0,1)=I2z+I3z+(l2*l2)*m3+(lc2*lc2)*m2+(lc3*lc3)*m3+l1*lc3*m3*cos(q2+q3)+l1*l2*m3*cos(q2)+l1*lc2*m2*cos(q2)+l2*lc3*m3*cos(q3)*2.0;
    H(0,2)=I3z+(lc3*lc3)*m3+l1*lc3*m3*cos(q2+q3)+l2*lc3*m3*cos(q3);

    H(1,0)=H(0,1);
    H(1,1)=I2z+I3z+(l2*l2)*m3+(lc2*lc2)*m2+(lc3*lc3)*m3+l2*lc3*m3*cos(q3)*2.0;
    H(1,2)=I3z+(lc3*lc3)*m3+l2*lc3*m3*cos(q3);

    H(2,0)=H(0,2);
    H(2,1)=H(1,2);
    H(2,2)=I3z+(lc3*lc3)*m3;

    //-----------CORIOLLIS MATRIX-------------

    Eigen::MatrixXd C(3,3);

    C(0,0)=-l1*qd2*(l2*m3*sin(q2)+lc2*m2*sin(q2)+lc3*m3*sin(q2+q3))-lc3*m3*qd3*(l1*sin(q2+q3)+l2*sin(q3));
    C(0,1)=-l1*qd1*(l2*m3*sin(q2)+lc2*m2*sin(q2)+lc3*m3*sin(q2+q3))-l1*qd2*(l2*m3*sin(q2)+lc2*m2*sin(q2)+lc3*m3*sin(q2+q3))-lc3*m3*qd3*(l1*sin(q2+q3)+l2*sin(q3));
    C(0,2)=-lc3*m3*qd1*(l1*sin(q2+q3)+l2*sin(q3))-lc3*m3*qd2*(l1*sin(q2+q3)+l2*sin(q3))-lc3*m3*qd3*(l1*sin(q2+q3)+l2*sin(q3));

    C(1,0)=l1*qd1*(l2*m3*sin(q2)+lc2*m2*sin(q2)+lc3*m3*sin(q2+q3))-l2*lc3*m3*qd3*sin(q3);
    C(1,1)=-l2*lc3*m3*qd3*sin(q3);
    C(1,2)=- l2*lc3*m3*qd1*sin(q3) - l2*lc3*m3*qd2*sin(q3) - l2*lc3*m3*qd3*sin(q3);

    C(2,0)=lc3*m3*qd1*(l1*sin(q2+q3)+l2*sin(q3))+l2*lc3*m3*qd2*sin(q3);
    C(2,1)=l2*lc3*m3*qd1*sin(q3) + l2*lc3*m3*qd2*sin(q3);
    C(2,2)=0;

    //------------FRICTION MATRIX-------------

    Eigen::MatrixXd B(3,3);

    B.setIdentity();

    //------------CONTROL VECTOR--------------

    Eigen::VectorXd T(3);

    T<<u(0),u(1),u(2);

    //------------VELOCITY VECTOR-------------

    Eigen::VectorXd qd(3);

    qd<<qd1,qd2,qd3;

    //------------CONTACT CONSTRAINT----------

    Eigen::RowVectorXd jacCns(3);

    /*
        jacCns(0) = 1.0/sqrt(pow(-q4+l2*cos(q1+q2)+l1*cos(q1)+l3*cos(q1+q2+q3),2.0)+pow(-q5+l2*sin(q1+q2)+l1*sin(q1)+l3*sin(q1+q2+q3),2.0))*(l1*q5*cos(q1)*2.0-l1*q4*sin(q1)*2.0+l3*q5*cos(q1+q2+q3)*2.0-l3*q4*sin(q1+q2+q3)*2.0+l2*q5*cos(q1+q2)*2.0-l2*q4*sin(q1+q2)*2.0)*(-1.0/2.0);
        jacCns(1) = 1.0/sqrt(pow(-q4+l2*cos(q1+q2)+l1*cos(q1)+l3*cos(q1+q2+q3),2.0)+pow(-q5+l2*sin(q1+q2)+l1*sin(q1)+l3*sin(q1+q2+q3),2.0))*(l1*l2*sin(q2)*2.0+l3*q5*cos(q1+q2+q3)*2.0-l3*q4*sin(q1+q2+q3)*2.0+l2*q5*cos(q1+q2)*2.0+l1*l3*sin(q2+q3)*2.0-l2*q4*sin(q1+q2)*2.0)*(-1.0/2.0);
        jacCns(2) = -l3*1.0/sqrt(pow(-q4+l2*cos(q1+q2)+l1*cos(q1)+l3*cos(q1+q2+q3),2.0)+pow(-q5+l2*sin(q1+q2)+l1*sin(q1)+l3*sin(q1+q2+q3),2.0))*(l1*sin(q2+q3)+l2*sin(q3)+q5*cos(q1+q2+q3)-q4*sin(q1+q2+q3));
        */

    jacCns(0) = (l2*sin(q1+q2)+l1*sin(q1)+l3*sin(q1+q2+q3))*(-q4+l2*cos(q1+q2)+l1*cos(q1)+l3*cos(q1+q2+q3))*-2.0+(l2*cos(q1+q2)+l1*cos(q1)+l3*cos(q1+q2+q3))*(-q5+l2*sin(q1+q2)+l1*sin(q1)+l3*sin(q1+q2+q3))*2.0;
    jacCns(1) = (l2*sin(q1+q2)+l3*sin(q1+q2+q3))*(-q4+l2*cos(q1+q2)+l1*cos(q1)+l3*cos(q1+q2+q3))*-2.0+(l2*cos(q1+q2)+l3*cos(q1+q2+q3))*(-q5+l2*sin(q1+q2)+l1*sin(q1)+l3*sin(q1+q2+q3))*2.0;
    jacCns(2) = l3*sin(q1+q2+q3)*(-q4+l2*cos(q1+q2)+l1*cos(q1)+l3*cos(q1+q2+q3))*-2.0+l3*cos(q1+q2+q3)*(-q5+l2*sin(q1+q2)+l1*sin(q1)+l3*sin(q1+q2+q3))*2.0;


    //------------OBTAIN THE EQUATIONS OF MOTIONS-------------

    Eigen::VectorXd qdd(5);

    qdd.head(3)=H.inverse()*(T+(lambda*jacCns.transpose())-C*qd-B*qd);

    r=0.25;

    double x_q=l1*cos(q1)+l2*cos(q1+q2)+l3*cos(q1+q2+q3);    //Contact Point x
    double y_q=l1*sin(q1)+l2*sin(q1+q2)+l3*sin(q1+q2+q3);    //Contact Point y

    double xr=r; //Reference point x
    double yr=0;   //Reference point y

    double xc=-(x_q-q4); //Contact point x
    double yc=-(y_q-q5); //Contact point y

    double dot=xr*xc + yr*yc;
    double det=xr*yc - yr*xc;

    double theta=atan2(det,dot);

    double angle=theta;

    qdd(3)=((lambda*cos(angle))-0.8*qd4)/3.0;        //m=2
    qdd(4)=((lambda*sin(angle))-0.8*qd5)/3.0;

    xdd<<qd1,qd2,qd3,qd4,qd5,qdd(0),qdd(1),qdd(2),qdd(3),qdd(4);



}//End systemDynamics
