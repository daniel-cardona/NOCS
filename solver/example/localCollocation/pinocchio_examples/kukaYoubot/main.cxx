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


//Read the urdf data

#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODEL_DIR "../../../../../data/pinocchio_models"
#endif

using namespace std::chrono;

using namespace std;


#define infty 100000



int main(){

    const std::string urdf_filename = PINOCCHIO_MODEL_DIR + std::string("/kukaYoubot.urdf");

    int maxIter=1;

    //Set the main data of the optimal control problem

    int nStates=16;
    int nControls=8;
    int nEvents=32;
    int nPath=0;
    int nDiscretePoints=60;

    //&-------------- Build the the problem using the rigid body dynamics library geoMBD ----------------------&
    nocs::localCollocation problem(nStates,nControls,nDiscretePoints,nPath,nEvents,urdf_filename);

   //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    //&-----------------------Variable bounds-------------------------&
    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

    //-------------Joint position limits------------

    problem.bounds.states.lower(0)=-1.0;           problem.bounds.states.upper(0)=1.0;
    problem.bounds.states.lower(1)=-1.0;           problem.bounds.states.upper(1)=1.0;
    problem.bounds.states.lower(2)=-0.5;           problem.bounds.states.upper(2)=0.5;

    problem.bounds.states.lower(3)=-2.949;         problem.bounds.states.upper(3)=2.949;
    problem.bounds.states.lower(4)=-1.1344;        problem.bounds.states.upper(4)=1.57;
    problem.bounds.states.lower(5)=-2.61;          problem.bounds.states.upper(5)=2.54;
    problem.bounds.states.lower(6)=-1.78;          problem.bounds.states.upper(6)=1.78;
    problem.bounds.states.lower(7)=-2.91;          problem.bounds.states.upper(7)=2.91;


    //------------Joint velocities limits----------

     problem.bounds.states.lower(8)=-5.0;       problem.bounds.states.upper(8)=5.0;
     problem.bounds.states.lower(9)=-5.0;       problem.bounds.states.upper(9)=5.0;
     problem.bounds.states.lower(10)=-5.0;      problem.bounds.states.upper(10)=5.0;
     problem.bounds.states.lower(11)=-5.0;      problem.bounds.states.upper(11)=5.0;
     problem.bounds.states.lower(12)=-5.0;      problem.bounds.states.upper(12)=5.0;
     problem.bounds.states.lower(13)=-5.0;      problem.bounds.states.upper(13)=5.0;
     problem.bounds.states.lower(14)=-5.0;      problem.bounds.states.upper(14)=5.0;
     problem.bounds.states.lower(15)=-5.0;      problem.bounds.states.upper(15)=5.0;


    //-----------Joint torque limits---------------

     problem.bounds.controls.lower(0)=-10;       problem.bounds.controls.upper(0)=10;
     problem.bounds.controls.lower(1)=-10;       problem.bounds.controls.upper(1)=10;
     problem.bounds.controls.lower(2)=-10;       problem.bounds.controls.upper(2)=10;
     problem.bounds.controls.lower(3)=-10;       problem.bounds.controls.upper(3)=10;
     problem.bounds.controls.lower(4)=-10;       problem.bounds.controls.upper(4)=10;
     problem.bounds.controls.lower(5)=-10;       problem.bounds.controls.upper(5)=10;
     problem.bounds.controls.lower(6)=-10;       problem.bounds.controls.upper(6)=10;
     problem.bounds.controls.lower(7)=-10;       problem.bounds.controls.upper(7)=10;


   //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
   //&---------------Boundary and path constraints-------------------&
   //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

     //Initial joint position
     problem.bounds.events.lower(0)=0;
     problem.bounds.events.lower(1)=0;
     problem.bounds.events.lower(2)=0;
     problem.bounds.events.lower(3)=0;
     problem.bounds.events.lower(4)=0;
     problem.bounds.events.lower(5)=0;
     problem.bounds.events.lower(6)=0;
     problem.bounds.events.lower(7)=0;

     //Initial joint velocities
     problem.bounds.events.lower(8)=0;
     problem.bounds.events.lower(9)=0;
     problem.bounds.events.lower(10)=0;
     problem.bounds.events.lower(11)=0;
     problem.bounds.events.lower(12)=0;
     problem.bounds.events.lower(13)=0;
     problem.bounds.events.lower(14)=0;
     problem.bounds.events.lower(15)=0;

     //Final joint position

     problem.bounds.events.lower(16)=0.7;
     problem.bounds.events.lower(17)=-0.5;
     problem.bounds.events.lower(18)=0.5;
     problem.bounds.events.lower(19)=0.0;
     problem.bounds.events.lower(20)=-0.3752;
     problem.bounds.events.lower(21)=1.027;
     problem.bounds.events.lower(22)=-0.11;
     problem.bounds.events.lower(23)=0;


     //Final joint velocities
     problem.bounds.events.lower(24)= 0.0;
     problem.bounds.events.lower(25)= 0.0;
     problem.bounds.events.lower(26)= 0.0;
     problem.bounds.events.lower(27)= 0.0;
     problem.bounds.events.lower(28)= 0.0;
     problem.bounds.events.lower(29)= 0.0;
     problem.bounds.events.lower(30)= 0.0;
     problem.bounds.events.lower(31)= 0.0;

   problem.bounds.events.upper=problem.bounds.events.lower;


    //Initial and final time bounds
   problem.bounds.initialTime.lower(0)=0;
   problem.bounds.initialTime.upper(0)=0;

   problem.bounds.finalTime.lower(0)=15.0;
   problem.bounds.finalTime.upper(0)=15.0;


   //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
   //&---------Set the collocation information of the problem---------------&
   //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

   problem.colMethod                ="Hermite-Simpson";
   problem.derivatives.gradientCost ="Numerical";
   problem.derivatives.jacobianCns  ="Numerical";

   //Set the problem depending on the local method selected

   problem.setupNLP();

  //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
  //&---------------------Set initial guess-------------------------&
  //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

   problem.guess.states.setZero();

   //Set a linear distribution in the joint position states

   int n=problem.nCollocationPoints;

   problem.guess.states.row(0).setLinSpaced(n,0, problem.bounds.events.lower(16));
   problem.guess.states.row(1).setLinSpaced(n,0, problem.bounds.events.lower(17));
   problem.guess.states.row(2).setLinSpaced(n,0, problem.bounds.events.lower(18));
   problem.guess.states.row(3).setLinSpaced(n,0, problem.bounds.events.lower(19));
   problem.guess.states.row(4).setLinSpaced(n,0, problem.bounds.events.lower(20));
   problem.guess.states.row(5).setLinSpaced(n,0, problem.bounds.events.lower(21));
   problem.guess.states.row(6).setLinSpaced(n,0, problem.bounds.events.lower(22));
   problem.guess.states.row(7).setLinSpaced(n,0, problem.bounds.events.lower(23));

   problem.guess.controls.setZero();
   problem.guess.t0=0;
   problem.guess.tF=15;

   //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
   //&---------------------Algorithm options-------------------------&
   //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&


   problem.algorithm.meshRefinement   =false;
   problem.algorithm.derivativeChecker=false;

   problem.algorithm.kappa=0.1;       //Default is 1/10 [Betts,2014]
   problem.algorithm.error_ode=1e-2;
   problem.algorithm.maxPoints=5;
   problem.algorithm.maxMeshIterations=maxIter;


   //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
   //&-------------------Set and solve the problem------------------&
   //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

   nocs::localCollocation probSol;

   auto start = high_resolution_clock::now();

   nocs::nocsLocal(problem,probSol);

   auto stop = high_resolution_clock::now();

   auto duration = duration_cast<microseconds>(stop - start);

   cout <<"Time required for full resolution of the problem: " <<duration.count() <<" microseconds"<< endl;

    return 0;
}


