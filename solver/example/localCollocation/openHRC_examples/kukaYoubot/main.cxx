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


//Read the urdf data

std::string robotFile = "../data/kukaYoubot.xml";

using namespace std::chrono;

using namespace std;

namespace plt = matplotlibcpp;

#define infty 100000



int main(){

    int maxIter=5;

    //Set the main data of the optimal control problem

    int nStates=16;
    int nControls=8;
    int nEvents=32;
    int nPath=0;
    int nDiscretePoints=20;

    //&-------------- Build the the problem using the rigid body dynamics library geoMBD ----------------------&
    nocs::localCollocation problem(nStates,nControls,nDiscretePoints,nPath,nEvents,robotFile);

    Eigen::VectorXd q(problem.robot->getDoF());
    Eigen::VectorXd qd(problem.robot->getDoF());
    Eigen::VectorXd qdd(problem.robot->getDoF());
    Eigen::VectorXd tau(problem.robot->getDoF());

    //Set the vectors and the states of the robot to zero

    q.setZero();     problem.robot->setConfiguration(q);
    qd.setZero();    problem.robot->setGeneralizedVelocity(qd);
    qdd.setZero();   problem.robot->setGeneralizedAcceleration(qdd);
    tau.setZero();   problem.robot->setGeneralizedTorques(tau);

    problem.robot->setConfiguration(q);
    problem.robot->computeForwardKinematics();


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

   problem.bounds.finalTime.lower(0)=1.0;
   problem.bounds.finalTime.upper(0)=30.0;


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
   problem.guess.tF=20;

   //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
   //&---------------------Algorithm options-------------------------&
   //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&


   problem.algorithm.meshRefinement   =true;
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

   int N=probSol.solution.tSol.size();

       std::vector<double> q1_sol(N), q2_sol(N),q3_sol(N), q4_sol(N),q5_sol(N), q6_sol(N),q7_sol(N), q8_sol(N);
       std::vector<double> qd1_sol(N), qd2_sol(N),qd3_sol(N), qd4_sol(N),qd5_sol(N), qd6_sol(N),qd7_sol(N), qd8_sol(N);
       std::vector<double> qdd1_sol(N), qdd2_sol(N),qdd3_sol(N), qdd4_sol(N),qdd5_sol(N), qdd6_sol(N),qdd7_sol(N), qdd8_sol(N);

       std::vector<double> u1_sol(N),u2_sol(N),u3_sol(N),u4_sol(N),u5_sol(N),u6_sol(N),u7_sol(N),u8_sol(N);
       std::vector<double> t(N);

       for (int i=0; i<N;i++){

           q1_sol.at(i)=probSol.solution.xSol(0,i);
           q2_sol.at(i)=probSol.solution.xSol(1,i);
           q3_sol.at(i)=probSol.solution.xSol(2,i);
           q4_sol.at(i)=probSol.solution.xSol(3,i);
           q5_sol.at(i)=probSol.solution.xSol(4,i);
           q6_sol.at(i)=probSol.solution.xSol(5,i);
           q7_sol.at(i)=probSol.solution.xSol(6,i);
           q8_sol.at(i)=probSol.solution.xSol(7,i);

           qd1_sol.at(i)=probSol.solution.xSol(8,i);
           qd2_sol.at(i)=probSol.solution.xSol(9,i);
           qd3_sol.at(i)=probSol.solution.xSol(10,i);
           qd4_sol.at(i)=probSol.solution.xSol(11,i);
           qd5_sol.at(i)=probSol.solution.xSol(12,i);
           qd6_sol.at(i)=probSol.solution.xSol(13,i);
           qd7_sol.at(i)=probSol.solution.xSol(14,i);
           qd8_sol.at(i)=probSol.solution.xSol(15,i);

           qdd1_sol.at(i)=probSol.solution.fSol(0,i);
           qdd2_sol.at(i)=probSol.solution.fSol(1,i);
           qdd3_sol.at(i)=probSol.solution.fSol(2,i);
           qdd4_sol.at(i)=probSol.solution.fSol(3,i);
           qdd5_sol.at(i)=probSol.solution.fSol(4,i);
           qdd6_sol.at(i)=probSol.solution.fSol(5,i);
           qdd7_sol.at(i)=probSol.solution.fSol(6,i);
           qdd8_sol.at(i)=probSol.solution.fSol(7,i);

           u1_sol.at(i)=probSol.solution.uSol(0,i);
           u2_sol.at(i)=probSol.solution.uSol(1,i);
           u3_sol.at(i)=probSol.solution.uSol(2,i);
           u4_sol.at(i)=probSol.solution.uSol(3,i);
           u5_sol.at(i)=probSol.solution.uSol(4,i);
           u6_sol.at(i)=probSol.solution.uSol(5,i);
           u7_sol.at(i)=probSol.solution.uSol(6,i);
           u8_sol.at(i)=probSol.solution.uSol(7,i);

           t.at(i)=probSol.solution.tSol(i);

       }

       plt::suptitle("Solution of the states");

       plt::subplot(2,1,1);
       plt::title("Joint position");
           plt::plot(t,q1_sol);
           plt::scatter(t,q1_sol,4.0);
           plt::plot(t,q2_sol);
           plt::scatter(t,q2_sol,4.0);
           plt::plot(t,q3_sol);
           plt::scatter(t,q3_sol,4.0);
           plt::plot(t,q4_sol);
           plt::scatter(t,q4_sol,4.0);
           plt::plot(t,q5_sol);
           plt::scatter(t,q5_sol,4.0);
           plt::plot(t,q6_sol);
           plt::scatter(t,q6_sol,4.0);
           plt::plot(t,q7_sol);
           plt::scatter(t,q7_sol,4.0);
           plt::plot(t,q8_sol);
           plt::scatter(t,q8_sol,4.0);
           plt::grid(true);

       plt::subplot(2,1,2);
       plt::title("Joint velocities");
           plt::plot(t,qd1_sol);
           plt::scatter(t,qd1_sol,4.0);
           plt::plot(t,qd2_sol);
           plt::scatter(t,qd2_sol,4.0);
           plt::plot(t,qd3_sol);
           plt::scatter(t,qd3_sol,4.0);
           plt::plot(t,qd4_sol);
           plt::scatter(t,qd4_sol,4.0);
           plt::plot(t,qd5_sol);
           plt::scatter(t,qd5_sol,4.0);
           plt::plot(t,qd6_sol);
           plt::scatter(t,qd6_sol,4.0);
           plt::plot(t,qd7_sol);
           plt::scatter(t,qd7_sol,4.0);
           plt::plot(t,qd8_sol);
           plt::scatter(t,qd8_sol,4.0);
           plt::grid(true);
       plt::show();



       plt::title("Control inputs");
           plt::plot(t,u1_sol);
           plt::scatter(t,u1_sol,4.0);
           plt::plot(t,u2_sol);
           plt::scatter(t,u2_sol,4.0);
           plt::plot(t,u3_sol);
           plt::scatter(t,u3_sol,4.0);
           plt::plot(t,u4_sol);
           plt::scatter(t,u4_sol,4.0);
           plt::plot(t,u5_sol);
           plt::scatter(t,u5_sol,4.0);
           plt::plot(t,u6_sol);
           plt::scatter(t,u6_sol,4.0);
           plt::plot(t,u7_sol);
           plt::scatter(t,u7_sol,4.0);
           plt::plot(t,u8_sol);
           plt::scatter(t,u8_sol,4.0);
           plt::grid(true);
       plt::show();


       plt::title("Derivatives solution");
       plt::plot(t,qdd1_sol);
       plt::scatter(t,qdd1_sol,4.0);
       plt::plot(t,qdd2_sol);
       plt::scatter(t,qdd2_sol,4.0);
       plt::plot(t,qdd3_sol);
       plt::scatter(t,qdd3_sol,4.0);
       plt::plot(t,qdd4_sol);
       plt::scatter(t,qdd4_sol,4.0);
       plt::plot(t,qdd5_sol);
       plt::scatter(t,qdd5_sol,4.0);
       plt::plot(t,qdd6_sol);
       plt::scatter(t,qdd6_sol,4.0);
       plt::plot(t,qdd7_sol);
       plt::scatter(t,qdd7_sol,4.0);
       plt::plot(t,qdd8_sol);
       plt::scatter(t,qdd8_sol,4.0);
       plt::grid(true);
       plt::show();

    return 0;
}


