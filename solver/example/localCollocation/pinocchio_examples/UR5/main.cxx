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

#include <iostream>
#include <chrono>


#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODEL_DIR "../../../../../data/pinocchio_models"
#endif

int main(){


     const std::string urdf_filename = PINOCCHIO_MODEL_DIR + std::string("/ur5_robot.urdf");


     //SETUP AND CONSTRUCT THE PROBLEM

     int nStates=12;
     int nControls=6;
     int nEvents=24;
     int nPath=0;
     int nDiscretePoints=30;

     nocs::localCollocation problem(nStates,nControls,nDiscretePoints,nPath,nEvents,urdf_filename);

     //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
     //&-----------------------Variable bounds-------------------------&
     //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

     //-------------Joint position limits------------

     int nDoF=problem.model.nq;

     problem.bounds.states.lower.head(nDoF)=problem.model.lowerPositionLimit;
     problem.bounds.states.lower.tail(nDoF)=-problem.model.velocityLimit;

     problem.bounds.states.upper.head(nDoF)=problem.model.upperPositionLimit;
     problem.bounds.states.upper.tail(nDoF)=problem.model.velocityLimit;

     //------------Torque limits --------------------

     problem.bounds.controls.lower=-problem.model.effortLimit;
     problem.bounds.controls.upper=problem.model.effortLimit;

     //-----------Initial and final time bounds ---------

     //Start at t=0
     problem.bounds.initialTime.lower(0)=0;
     problem.bounds.initialTime.upper(0)=0;

     //Finished at tF=15;
     problem.bounds.finalTime.lower(0)=10.0;
     problem.bounds.finalTime.upper(0)=10.0;


     //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
     //&---------------Event and path constraints bounds---------------&
     //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

     //-- Initial conditions

        //Position
             problem.bounds.events.lower(0)=0; //e(0)=q1_0
             problem.bounds.events.lower(1)=0; //e(1)=q2_0
             problem.bounds.events.lower(2)=0; //e(2)=q3_0
             problem.bounds.events.lower(3)=0; //e(3)=q4_0
             problem.bounds.events.lower(4)=0; //e(4)=q5_0
             problem.bounds.events.lower(5)=0; //e(5)=q6_0
        //Velocities
             problem.bounds.events.lower(6)=0; //e(6)=qd1_0
             problem.bounds.events.lower(7)=0; //e(7)=qd2_0
             problem.bounds.events.lower(8)=0; //e(8)=qd3_0
             problem.bounds.events.lower(9)=0; //e(9)=qd4_0
             problem.bounds.events.lower(10)=0; //e(10)=qd5_0
             problem.bounds.events.lower(11)=0; //e(11)=qd6_0

 //-- Final conditions

        //Position
             problem.bounds.events.lower(12)=4.5; //e(12)=q1_N
             problem.bounds.events.lower(13)=4.5; //e(13)=q2_N
             problem.bounds.events.lower(14)=2.2; //e(14)=q3_N
             problem.bounds.events.lower(15)=2.6; //e(15)=q4_N
             problem.bounds.events.lower(16)=1.2; //e(16)=q5_N
             problem.bounds.events.lower(17)=-1; //e(17)=q6_N
        //Velocities
             problem.bounds.events.lower(18)=0; //e(18)=qd1_N
             problem.bounds.events.lower(19)=0; //e(19)=qd2_N
             problem.bounds.events.lower(20)=0; //e(20)=qd3_N
             problem.bounds.events.lower(21)=0; //e(21)=qd4_N
             problem.bounds.events.lower(22)=0; //e(22)=qd5_N
             problem.bounds.events.lower(23)=0; //e(23)=qd6_N

  //-- Set it as equality constraint
       problem.bounds.events.upper=problem.bounds.events.lower;

      //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
      //&---------Set the collocation information of the problem---------------&
      //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

      problem.colMethod                ="Hermite-Simpson";
      problem.derivatives.gradientCost ="Analytical";
      problem.derivatives.jacobianCns  ="Analytical";

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
      problem.guess.tF=10;

      //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
      //&---------------------Algorithm options-------------------------&
      //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&


      problem.algorithm.meshRefinement   =true;
      problem.algorithm.derivativeChecker=false;

      problem.algorithm.kappa=0.1;       //Default is 1/10 [Betts,2014]
      problem.algorithm.error_ode=1e-4;
      problem.algorithm.maxPoints=5;
      problem.algorithm.maxMeshIterations=4;


      //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
      //&-------------------Set and solve the problem------------------&
      //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&


      nocs::localCollocation probSol;

      auto start = std::chrono::high_resolution_clock::now();

      nocs::nocsLocal(problem,probSol);

      auto stop = std::chrono::high_resolution_clock::now();

      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

      cout <<"Time required for full resolution of the problem: " <<duration.count() <<" microseconds"<< endl;

  return 0;

}

