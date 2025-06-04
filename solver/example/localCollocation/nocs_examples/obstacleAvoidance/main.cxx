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

#include <chrono>
#include <fstream>


using namespace std::chrono; 

using namespace std;


int main(){

    int maxIter=1;

    int nStates=2;
    int nControls=1;
    int nEvents=4;
    int nPath=2;
    int nDiscretePoints=60;

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
    problem.derivatives.jacobianCns="Analytical";
    problem.derivatives.gradientCost="Numerical";

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

    auto stop = high_resolution_clock::now();

    auto duration = duration_cast<microseconds>(stop - start);

    cout <<"Time required for full resolution of the problem: " <<duration.count() <<" microseconds"<< endl;

    return 0;

};
