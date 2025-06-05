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
#include "directCollocation/local/utils.hpp"

#include <iostream>
#include <chrono>


#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODEL_DIR "../../../../../data/pinocchio_models"
#endif

int main(){


     const std::string urdf_filename = PINOCCHIO_MODEL_DIR + std::string("/yumi_complete.urdf");


     //SETUP AND CONSTRUCT THE PROBLEM

     int nStates=36;
     int nControls=18;
     int nEvents=nStates*2;
     int nPath=0;
     int nDiscretePoints=180;

     nocs::localCollocation problem(nStates,nControls,nDiscretePoints,nPath,nEvents,urdf_filename);


     //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
     //&-----------------------Variable bounds-------------------------&
     //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

     //-------------Joint position limits------------

     int nDoF=problem.model.nq;

     problem.bounds.states.lower.head(nDoF)=problem.model.lowerPositionLimit;
     problem.bounds.states.upper.head(nDoF)=problem.model.upperPositionLimit;

     problem.bounds.states.lower.tail(nDoF)=-problem.model.velocityLimit;
     problem.bounds.states.upper.tail(nDoF)=problem.model.velocityLimit;

     //------------Torque limits --------------------


     problem.bounds.controls.lower=-problem.model.effortLimit;
     problem.bounds.controls.upper=problem.model.effortLimit;

     //-----------Initial and final time bounds ---------

     //Start at t=0
     problem.bounds.initialTime.lower(0)=0;
     problem.bounds.initialTime.upper(0)=0;

     //Finished at tF=15;
     problem.bounds.finalTime.lower(0)=1.0;
     problem.bounds.finalTime.upper(0)=10.0;


     //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
     //&---------------Event and path constraints bounds---------------&
     //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

     //-- Initial conditions

     problem.bounds.events.lower.setZero();

 //-- Final conditions

     Eigen::VectorXd init_cfg(nDoF);

     init_cfg.setZero();

//     init_cfg(0)=-0.5011;
//     init_cfg(1)=-0.4853;
//     init_cfg(2)=0.4475;
//     init_cfg(3)=-0.0261;
//     init_cfg(4)=-0.4407;
//     init_cfg(5)=0.5943;
//     init_cfg(6)=0.0115;
//     init_cfg(7)=0;
//     init_cfg(8)=0;
//     init_cfg(9)=0.7123;
//     init_cfg(10)=-1.5916;
//     init_cfg(11)=-0.4834;
//     init_cfg(12)=-0.0246;
//     init_cfg(13)=-0.1350;
//     init_cfg(14)=-1.5054;
//     init_cfg(15)=-0.4996;
//     init_cfg(16)=0;
//     init_cfg(17)=0;

     Eigen::VectorXd end_cfg(nDoF);

     end_cfg(0)=-0.5011;
     end_cfg(1)=-0.4853;
     end_cfg(2)=0.4475;
     end_cfg(3)=-0.0261;
     end_cfg(4)=-0.4407;
     end_cfg(5)=0.5943;
     end_cfg(6)=0.0115;
     end_cfg(7)=0;
     end_cfg(8)=0;
     end_cfg(9)=2.9109;
     end_cfg(10)=-0.6670;
     end_cfg(11)=-1.9099;
     end_cfg(12)=-1.5054;
     end_cfg(13)=1.3029;
     end_cfg(14)=-1.5219;
     end_cfg(15)=-2.0625;
     end_cfg(16)=0.0250;
     end_cfg(17)=0.0250;


    Eigen::VectorXd l_bnd(nDoF);
    Eigen::VectorXd u_bnd(nDoF);

    l_bnd=problem.bounds.states.lower.head(nDoF);
    u_bnd=problem.bounds.states.upper.head(nDoF);

    if( nocs::utils::testConfiguration(l_bnd,u_bnd,init_cfg) || nocs::utils::testConfiguration(l_bnd, u_bnd, end_cfg) ==-1){
        return -1;
    }


     for(int i=0; i<nDoF; i++){

         problem.bounds.events.lower(i)=init_cfg(i);
         problem.bounds.events.lower(nStates+i)=end_cfg(i);

     }

     //Velocities
     problem.bounds.events.lower.tail(nDoF).setZero();




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

      for(int i=0; i<problem.model.nq; i++){

         problem.guess.states.row(i).setLinSpaced(n,problem.bounds.events.lower(i),problem.bounds.events.lower(nStates+i));

      }

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
      problem.algorithm.maxMeshIterations=1;


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

