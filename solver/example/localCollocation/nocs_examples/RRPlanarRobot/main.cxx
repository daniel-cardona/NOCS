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
    
    //Set the main data of the optimal control problem

    int nStates=4;
    int nControls=2;
    int nEvents=8;
    int nPath=0;
    int nDiscretePoints=240;

    nocs::localCollocation problem(nStates,nControls,nDiscretePoints,nPath,nEvents);

    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    //&-----------------------Variable bounds-------------------------&
    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

       double q1L = -2.0;
       double q2L = -2.0;
       double q1dL =-10.0;
       double q2dL =-10.0;

        double q1U = 2.0;
        double q2U = 2.0;
        double q1dU =10.0;
        double q2dU =10.0;

        double T1L = -15.0;
        double T2L = -15.0;
        double T1U = 15.0;
        double T2U = 15.0;

       double q1_t0  = 0;
       double q2_t0  = 0;
       double q1d_t0 = 0;
       double q2d_t0 = 0;

       double xF=0.5;
       double yF=0.75;

       double q1_tF  = 0.77;
       double q2_tF  = 0.77;
       double q1d_tF = 0;
       double q2d_tF = 0;

    problem.bounds.states.lower(0)=q1L;
    problem.bounds.states.lower(1)=q2L;
    problem.bounds.states.lower(2)=q1dL;
    problem.bounds.states.lower(3)=q2dL;

    problem.bounds.states.upper(0)=q1U;
    problem.bounds.states.upper(1)=q2U;
    problem.bounds.states.upper(2)=q1dU;
    problem.bounds.states.upper(3)=q2dU;

    problem.bounds.controls.lower(0)=T1L;
    problem.bounds.controls.lower(1)=T2L;

    problem.bounds.controls.upper(0)=T1U;
    problem.bounds.controls.upper(1)=T2U;

    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    //&---------------Boundary and path constraints-------------------&
    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

    problem.bounds.events.lower(0)=q1_t0;
    problem.bounds.events.lower(1)=q2_t0;
    problem.bounds.events.lower(2)=q1d_t0;
    problem.bounds.events.lower(3)=q2d_t0;

    problem.bounds.events.lower(4)=xF;
    problem.bounds.events.lower(5)=yF;
    problem.bounds.events.lower(6)=q1d_tF;
    problem.bounds.events.lower(7)=q2d_tF;

    problem.bounds.events.upper(0)=q1_t0;
    problem.bounds.events.upper(1)=q2_t0;
    problem.bounds.events.upper(2)=q1d_t0;
    problem.bounds.events.upper(3)=q2d_t0;

    problem.bounds.events.upper(4)=xF;
    problem.bounds.events.upper(5)=yF;
    problem.bounds.events.upper(6)=q1d_tF;
    problem.bounds.events.upper(7)=q2d_tF;

    problem.bounds.initialTime.lower(0)=0;
    problem.bounds.initialTime.upper(0)=0;

    problem.bounds.finalTime.lower(0)=5.0;
    problem.bounds.finalTime.upper(0)=5.0;

    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    //&---------Set the collocation information of the problem---------------&
    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&


    problem.colMethod="Hermite-Simpson";
    problem.derivatives.jacobianCns="Analytical";
    problem.derivatives.gradientCost="Analytical";

    //Set the problem depending on the local method selected

    problem.setupNLP();

    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    //&---------------------Set initial guess-------------------------&
    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

    int nNodes=problem. nCollocationPoints;

    //nNodes=problem->nDiscretePoints;

    problem.guess.states.row(0).setLinSpaced(nNodes,q1_t0,q1_tF);
    problem.guess.states.row(1).setLinSpaced(nNodes,q2_t0,q2_tF);

    problem.guess.states.row(0).setZero();
    problem.guess.states.row(1).setZero();

    problem.guess.states.row(2).setZero();
    problem.guess.states.row(3).setZero();

    problem.guess.controls.row(0).setZero();
    problem.guess.controls.row(1).setZero();

    problem.guess.t0=0;
    problem.guess.tF=5;

    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    //&---------------------Algorithm options-------------------------&
    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&


    problem.algorithm.meshRefinement   =true;
    problem.algorithm.derivativeChecker=false;

    problem.algorithm.kappa=0.1;       //Default is 1/10 [Betts,2014]
    problem.algorithm.error_ode=1e-5;
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

