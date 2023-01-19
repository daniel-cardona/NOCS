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

    double V=2.138;

    double theta=controls(0);

    double dxdt=V*cos(theta);
    double dydt=V*sin(theta);

    double L=pow(dxdt,2.0)+ pow(dydt,2.0);

    return L;

} //End integrand_cost

void nocs::Function::dae(const Eigen::VectorXd &states, const Eigen::VectorXd &controls,const double &tk, Eigen::VectorXd &derivatives,localCollocation &problem){

    double x =states(0);
    double y =states(1);

    double theta=controls(0);

    double V=2.138;

    double dxdt= V*cos(theta);
    double dydt= V*sin(theta);

    derivatives.setZero(states.size());

    derivatives(0)=dxdt;
    derivatives(1)=dydt;

}//End dae

void nocs::Function::path(const Eigen::VectorXd &states, const Eigen::VectorXd &controls,const double &tk,Eigen::VectorXd &path,localCollocation &problem){

    double x =states(0);
    double y =states(1);

    path.setZero(2);

    path(0)=pow(x-0.4,2.0)+pow(y-0.5,2.0);
    path(1)=pow(x-0.8,2.0)+pow(y-1.5,2.0);

}//End dae

void nocs::Function::events(const Eigen::VectorXd &initial_states, const Eigen::VectorXd &final_states, const double &t0, const double &tF, Eigen::VectorXd &e,localCollocation &problem){

    double x0=initial_states(0);
    double y0=initial_states(1);

    double xf=final_states(0);
    double yf=final_states(1);

    e.setZero(4);

    e(0)=x0;
    e(1)=y0;
    e(2)=xf;
    e(3)=yf;

}//End events


void nocs::Function::analytical::fGradient(Eigen::VectorXd &states, Eigen::VectorXd &controls,double &t, Eigen::MatrixXd &gradient, localCollocation &problem){

    //The gradient of the dynamics should be return using the sparsity pattern:

    //grad(f (x, u))=[ df | df ]
    //               [ -- | -- ]
    //               [ dx | du ]

    //dim(grad)=[nStates,nStates+nControls]

    double x =states(0);
    double y =states(1);

    double theta=controls(0);

    double V=2.138;

    //dxdt= V*cos(theta);
    //dydt= V*sin(theta);

    gradient.setZero(2,3);

    gradient<<0,0,-V*sin(theta),    //dxdt w.r.t [x y theta]
              0,0, V*cos(theta);    //dydt w.r.t [x y theta]

    } //End fGradient

void nocs::Function::analytical::pathGradient(Eigen::VectorXd &states, Eigen::VectorXd &controls,double &t, Eigen::MatrixXd &gradient, localCollocation &problem){

    //The gradient of the path constraints should be return using the sparsity pattern:

    //grad(g (x, u))=[ dg | dg ]
    //               [ -- | -- ]
    //               [ dx | du ]

    //dim(grad)=[nPath,nStates+nControls]

    double x =states(0);
    double y =states(1);

  //path(0)=pow(x-0.6,2.0)+pow(y-1,2.0);

    gradient.setZero(2,3);


    gradient<<(2.0*x)-0.8,(2.0*y)-1.0,0,
              (2.0*x)-1.6,(2.0*y)-3.0,0;



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

    gradient0.setZero(4,3);

        gradient0<<0,1.0,0,
                   0,0,1.0,
                   0,0,0,
                   0,0,0;

        gradienttF.setZero(4,3);

        gradienttF<<0,0,0,
                    0,0,0,
                    0,1.0,0,
                    0,0,1.0;



}
