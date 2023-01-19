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

#include "matplotlibcpp.h"

#include <chrono>
#include <fstream>


using namespace std::chrono; 

using namespace std;

namespace plt = matplotlibcpp;

int main(){

    int maxIter=1;

    int nStates=2;
    int nControls=1;
    int nEvents=4;
    int nPath=2;
    int nDiscretePoints=5;

    nocs::localCollocation problem(nStates,nControls,nDiscretePoints,nPath,nEvents);
	
    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    //&-----------------------Variable bounds-------------------------&
    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

    double xL = 0.0;
    double yL = 0.0;
    double xU = 2.0;
    double yU = 2.0;

    double thetaL = -10.0;
    double thetaU = 10.0;

    double x0 = 0.0;
    double y0 = 0.0;
    double xf = 1.2;
    double yf = 1.6;

    problem.bounds.states.lower(0)=xL;
    problem.bounds.states.upper(0)=xU;

    problem.bounds.states.lower(1)=yL;
    problem.bounds.states.upper(1)=yU;

    problem.bounds.controls.lower(0)=thetaL;
    problem.bounds.controls.upper(0)=thetaU;

    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    //&---------------Boundary and path constraints-------------------&
    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

    problem.bounds.events.lower(0)=x0;
    problem.bounds.events.lower(1)=y0;
    problem.bounds.events.lower(2)=xf;
    problem.bounds.events.lower(3)=yf;

    problem.bounds.events.upper(0)=x0;
    problem.bounds.events.upper(1)=y0;
    problem.bounds.events.upper(2)=xf;
    problem.bounds.events.upper(3)=yf;

    problem.bounds.path.lower(0)=0.1;
    problem.bounds.path.upper(0)=100.0;

    problem.bounds.path.lower(1)=0.1;
    problem.bounds.path.upper(1)=100.0;

    problem.bounds.initialTime.lower(0)=0;
    problem.bounds.initialTime.upper(0)=0;

    problem.bounds.finalTime.lower(0)=1.0;
    problem.bounds.finalTime.upper(0)=1.0;

    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    //&---------Set the collocation information of the problem---------------&
    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

    //problem.colMethod="Trapezoidal";
    problem.colMethod="Hermite-Simpson";
    problem.derivatives="Analytical";

    //Set the problem depending on the local method selected

    problem.setupNLP();

    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    //&---------------------Set initial guess-------------------------&
    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

    int nNodes=problem.nCollocationPoints;

    problem.guess.states.row(0).setLinSpaced(nNodes,x0,xf);
    problem.guess.states.row(1).setLinSpaced(nNodes,y0,yf);

    problem.guess.controls.row(0).setZero();

    problem.guess.t0=0;
    problem.guess.tF=1;

    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    //&---------------------Algorithm options-------------------------&
    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&


    problem.algorithm.meshRefinement   =true;
    problem.algorithm.derivativeChecker=false;

    problem.algorithm.kappa=0.1;       //Default is 1/10 [Betts,2014]
    problem.algorithm.error_ode=1e-3;
    problem.algorithm.maxPoints=5;
    problem.algorithm.maxMeshIterations=maxIter;


//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&------------------------- SOLVE -----------------------------&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&


    nocs::localCollocation probSol;

    auto start = high_resolution_clock::now();

    nocs::nocsLocal(problem,probSol);

    /*auto stop = high_resolution_clock::now();

    auto duration = duration_cast<microseconds>(stop - start);

    cout <<"Time required for full resolution of the problem: " <<duration.count() <<" microseconds"<< endl;

    //Prepare data for plotting
    int N=probSol.solution.tSol.size();

    std::vector<double> x_sol(N), y_sol(N) , u_sol(N),t(N);
    std::vector<double> x_dot(N), y_dot(N);


    for (int i=0; i<N;i++){

        x_sol.at(i)=probSol.solution.xSol(0,i);
        y_sol.at(i)=probSol.solution.xSol(1,i);

        u_sol.at(i)=probSol.solution.uSol(0,i);

        x_dot.at(i)=probSol.solution.fSol(0,i);
        y_dot.at(i)=probSol.solution.fSol(1,i);

        t.at(i)=probSol.solution.tSol(i);

    }

    Eigen::VectorXd theta;

    double radius=sqrt(0.1);

    theta.setLinSpaced(30,0,2*PI);

    Eigen::VectorXd cos_theta;
    Eigen::VectorXd sin_theta;

    cos_theta=cos(theta.array());
    sin_theta=sin(theta.array());

    Eigen::VectorXd xObs1(theta.size());
    Eigen::VectorXd yObs1(theta.size());

    Eigen::VectorXd xObs2(theta.size());
    Eigen::VectorXd yObs2(theta.size());

    std::vector<double> x1(theta.size()), y1(theta.size());
    std::vector<double> x2(theta.size()), y2(theta.size());


    xObs1=radius*cos_theta.array()+0.4;
    yObs1=radius*sin_theta.array()+0.5;

    xObs2=radius*cos_theta.array()+0.8;
    yObs2=radius*sin_theta.array()+1.5;

    for (int i=0; i<theta.size();i++){

        x1.at(i)=xObs1(i);
        y1.at(i)=yObs1(i);

        x2.at(i)=xObs2(i);
        y2.at(i)=yObs2(i);


    }


    //X-Y position plot
    plt::figure_size(1200, 780);
    plt::plot(x_sol,y_sol,"r-");
    plt::plot(x1,y1);
    plt::plot(x2,y2);
    plt::grid(true);
    plt::xlim(0,2);
    plt::ylim(0,2);
    plt::show();


    //States plot

    plt::suptitle("Solution of the the states");
    plt::subplot(1,2,1);
        plt::scatter(t,x_sol,15.0);
        plt::plot(t,x_sol,"k--");
    plt::subplot(1,2,2);
        plt::scatter(t,y_sol,15.0);
        plt::plot(t,y_sol,"r--");
    plt::show();


    //Derivatives plot
    plt::suptitle("Solution of the the derivatives");
    plt::subplot(1,2,1);
        plt::plot(t,x_dot,"k-");
        plt::grid(true);
    plt::subplot(1,2,2);
        plt::plot(t,y_dot,"r-");
        plt::grid(true);
    plt::show();


    //Control plots
    plt::scatter(t,u_sol,15.0);
    plt::plot(t,u_sol,"r--");
    plt::grid(true);
    plt::show();

    */

    return 0;

};
