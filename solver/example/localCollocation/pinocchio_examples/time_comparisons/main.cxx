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

#include "directCollocation/local/fcnGenerator.hpp"

#include <iostream>
#include <chrono>


#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODEL_DIR "../../../../../data/pinocchio_models"
#endif

int main(){


    const std::string urdf_filename = PINOCCHIO_MODEL_DIR + std::string("/nao_inertial_python.urdf");

    //Set the main data of the optimal control problem

    int nStates=48;
    int nControls=24;
    int nEvents=96;
    int nPath=0;
    int nDiscretePoints=120;

    //&-------------- Build the Robot using the rigid body dynamics library pinocchio ----------------------&

    nocs::localCollocation problem(nStates,nControls,nDiscretePoints,nPath,nEvents,urdf_filename);


    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    //&-----------------------Variable bounds-------------------------&
    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

    //-------------Joint position limits------------

    problem.bounds.states.lower(0)=-0.39270;       problem.bounds.states.upper(0)=0.76969;
    problem.bounds.states.lower(1)=-1.18857;       problem.bounds.states.upper(1)=0.92328;
    problem.bounds.states.lower(2)=-0.08552;       problem.bounds.states.upper(2)=2.10138;
    problem.bounds.states.lower(3)=-1.53065;       problem.bounds.states.upper(3)=0.48346;
    problem.bounds.states.lower(4)=-0.37350;       problem.bounds.states.upper(4)=0.78540;
    problem.bounds.states.lower(5)=-1.13970;       problem.bounds.states.upper(5)=0.74002;

    problem.bounds.states.lower(6)=-2.08218;       problem.bounds.states.upper(6)=2.07694;
    problem.bounds.states.lower(7)=-0.19722;       problem.bounds.states.upper(7)=1.32645;
    problem.bounds.states.lower(8)=-2.08392;       problem.bounds.states.upper(8)=2.08218;
    problem.bounds.states.lower(9)=-5;             problem.bounds.states.upper(9)=5;
    //problem.bounds.states.lower(9)=-1.53589;       problem.bounds.states.upper(9)=0.03840;
    problem.bounds.states.lower(10)=-2.08392;      problem.bounds.states.upper(10)=2.08218;

    problem.bounds.states.lower(11)=-2.07694;      problem.bounds.states.upper(11)=2.08392;
    problem.bounds.states.lower(12)=-0.66148;      problem.bounds.states.upper(12)=0.50964;

    problem.bounds.states.lower(13)=-2.08218;      problem.bounds.states.upper(13)=2.08567;
    problem.bounds.states.lower(14)=-1.32645;      problem.bounds.states.upper(14)=0.20420;
    problem.bounds.states.lower(15)=-2.08392;      problem.bounds.states.upper(15)=2.08218;
    problem.bounds.states.lower(16)=-5;            problem.bounds.states.upper(16)=5;
    //problem.bounds.states.lower(16)=-0.03665;      problem.bounds.states.upper(16)=1.53589;
    problem.bounds.states.lower(17)=-2.08392;      problem.bounds.states.upper(17)=2.08218;

    problem.bounds.states.lower(18)=-1.14494;      problem.bounds.states.upper(18)=0.74002;
    problem.bounds.states.lower(19)=-0.78714;      problem.bounds.states.upper(19)=0.37874;
    problem.bounds.states.lower(20)=-1.53065;      problem.bounds.states.upper(20)=0.47822;
    problem.bounds.states.lower(21)=-0.08901;      problem.bounds.states.upper(21)=2.10836;
    problem.bounds.states.lower(22)=-1.18682;      problem.bounds.states.upper(22)=0.93201;
    problem.bounds.states.lower(23)=-0.76794;      problem.bounds.states.upper(23)=0.39444;

    //------------Joint velocities limits----------

         problem.bounds.states.lower(24)=-5.0;      problem.bounds.states.upper(24)=5.0;
         problem.bounds.states.lower(25)=-5.0;      problem.bounds.states.upper(25)=5.0;
         problem.bounds.states.lower(26)=-5.0;      problem.bounds.states.upper(26)=5.0;
         problem.bounds.states.lower(27)=-5.0;      problem.bounds.states.upper(27)=5.0;
         problem.bounds.states.lower(28)=-5.0;      problem.bounds.states.upper(28)=5.0;
         problem.bounds.states.lower(29)=-5.0;      problem.bounds.states.upper(29)=5.0;
         problem.bounds.states.lower(30)=-5.0;      problem.bounds.states.upper(30)=5.0;
         problem.bounds.states.lower(31)=-5.0;      problem.bounds.states.upper(31)=5.0;
         problem.bounds.states.lower(32)=-5.0;      problem.bounds.states.upper(32)=5.0;
         problem.bounds.states.lower(33)=-5.0;      problem.bounds.states.upper(33)=5.0;
         problem.bounds.states.lower(34)=-5.0;      problem.bounds.states.upper(34)=5.0;
         problem.bounds.states.lower(35)=-5.0;      problem.bounds.states.upper(35)=5.0;
         problem.bounds.states.lower(36)=-5.0;      problem.bounds.states.upper(36)=5.0;
         problem.bounds.states.lower(37)=-5.0;      problem.bounds.states.upper(37)=5.0;
         problem.bounds.states.lower(38)=-5.0;      problem.bounds.states.upper(38)=5.0;
         problem.bounds.states.lower(39)=-5.0;      problem.bounds.states.upper(39)=5.0;
         problem.bounds.states.lower(40)=-5.0;      problem.bounds.states.upper(40)=5.0;
         problem.bounds.states.lower(41)=-5.0;      problem.bounds.states.upper(41)=5.0;
         problem.bounds.states.lower(42)=-5.0;      problem.bounds.states.upper(42)=5.0;
         problem.bounds.states.lower(43)=-5.0;      problem.bounds.states.upper(43)=5.0;
         problem.bounds.states.lower(44)=-5.0;      problem.bounds.states.upper(44)=5.0;
         problem.bounds.states.lower(45)=-5.0;      problem.bounds.states.upper(45)=5.0;
         problem.bounds.states.lower(46)=-5.0;      problem.bounds.states.upper(46)=5.0;
         problem.bounds.states.lower(47)=-5.0;      problem.bounds.states.upper(47)=5.0;

        //-----------Joint torque limits---------------

         problem.bounds.controls.lower(0)=-10;       problem.bounds.controls.upper(0)=10;
         problem.bounds.controls.lower(1)=-10;       problem.bounds.controls.upper(1)=10;
         problem.bounds.controls.lower(2)=-10;       problem.bounds.controls.upper(2)=10;
         problem.bounds.controls.lower(3)=-10;       problem.bounds.controls.upper(3)=10;
         problem.bounds.controls.lower(4)=-10;       problem.bounds.controls.upper(4)=10;
         problem.bounds.controls.lower(5)=-10;       problem.bounds.controls.upper(5)=10;
         problem.bounds.controls.lower(6)=-10;       problem.bounds.controls.upper(6)=10;
         problem.bounds.controls.lower(7)=-10;       problem.bounds.controls.upper(7)=10;
         problem.bounds.controls.lower(8)=-10;       problem.bounds.controls.upper(8)=10;
         problem.bounds.controls.lower(9)=-10;       problem.bounds.controls.upper(9)=10;
         problem.bounds.controls.lower(10)=-10;      problem.bounds.controls.upper(10)=10;
         problem.bounds.controls.lower(11)=-10;      problem.bounds.controls.upper(11)=10;
         problem.bounds.controls.lower(12)=-10;      problem.bounds.controls.upper(12)=10;
         problem.bounds.controls.lower(13)=-10;      problem.bounds.controls.upper(13)=10;
         problem.bounds.controls.lower(14)=-10;      problem.bounds.controls.upper(14)=10;
         problem.bounds.controls.lower(15)=-10;      problem.bounds.controls.upper(15)=10;
         problem.bounds.controls.lower(16)=-10;      problem.bounds.controls.upper(16)=10;
         problem.bounds.controls.lower(17)=-10;      problem.bounds.controls.upper(17)=10;
         problem.bounds.controls.lower(18)=-10;      problem.bounds.controls.upper(18)=10;
         problem.bounds.controls.lower(19)=-10;      problem.bounds.controls.upper(19)=10;
         problem.bounds.controls.lower(20)=-10;      problem.bounds.controls.upper(20)=10;
         problem.bounds.controls.lower(21)=-10;      problem.bounds.controls.upper(21)=10;
         problem.bounds.controls.lower(22)=-10;      problem.bounds.controls.upper(22)=10;
         problem.bounds.controls.lower(23)=-10;      problem.bounds.controls.upper(23)=10;

    //-------Initial and final time boundaries-------

    problem.bounds.initialTime.lower(0)=0;
    problem.bounds.initialTime.upper(0)=0;

    problem.bounds.finalTime.lower(0)=5.0;
    problem.bounds.finalTime.upper(0)=5.0;

   //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
   //&---------------Boundary and path constraints-------------------&
   //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

     Eigen::VectorXd initial_configuration(problem.model.nq);
     Eigen::VectorXd end_configuration(problem.model.nq);

     initial_configuration.setZero();

     //Pose 4

     end_configuration<<  0.004,
                         -0.769,
                         2.024,
                         -1.292,
                         0.312,
                         -1.040,

                         0.388,
                         0.052,
                         1.539,
                         0.209,
                         -0.254,

                         -0.004,
                         -0.659,

                         0.357,
                         -0.033,
                         -1.539,
                         -0.209,
                         0.254,

                         -1.0404,
                         -0.326,
                         -1.206,
                         2.0242,
                         -0.831,
                         -0.023;



           //Initial position joints                                        //Final joint positions
          problem.bounds.events.lower(0)=initial_configuration(0);          problem.bounds.events.lower(48)=end_configuration(0);
          problem.bounds.events.lower(1)=initial_configuration(1);          problem.bounds.events.lower(49)=end_configuration(1);
          problem.bounds.events.lower(2)=initial_configuration(2);          problem.bounds.events.lower(50)=end_configuration(2);
          problem.bounds.events.lower(3)=initial_configuration(3);          problem.bounds.events.lower(51)=end_configuration(3);
          problem.bounds.events.lower(4)=initial_configuration(4);          problem.bounds.events.lower(52)=end_configuration(4);
          problem.bounds.events.lower(5)=initial_configuration(5);          problem.bounds.events.lower(53)=end_configuration(5);

          problem.bounds.events.lower(6)=initial_configuration(6);          problem.bounds.events.lower(54)= end_configuration(6);
          problem.bounds.events.lower(7)=initial_configuration(7);          problem.bounds.events.lower(55)= end_configuration(7);
          problem.bounds.events.lower(8)=initial_configuration(8);          problem.bounds.events.lower(56)=end_configuration(8);
          problem.bounds.events.lower(9)=initial_configuration(9);          problem.bounds.events.lower(57)=end_configuration(9);
          problem.bounds.events.lower(10)=initial_configuration(10);        problem.bounds.events.lower(58)=end_configuration(10);

          problem.bounds.events.lower(11)=initial_configuration(11);        problem.bounds.events.lower(59)= end_configuration(11);
          problem.bounds.events.lower(12)=initial_configuration(12);        problem.bounds.events.lower(60)= end_configuration(12);

          problem.bounds.events.lower(13)=initial_configuration(13);        problem.bounds.events.lower(61)= end_configuration(13);
          problem.bounds.events.lower(14)=initial_configuration(14);        problem.bounds.events.lower(62)=end_configuration(14);
          problem.bounds.events.lower(15)=initial_configuration(15);        problem.bounds.events.lower(63)= end_configuration(15);
          problem.bounds.events.lower(16)=initial_configuration(16);        problem.bounds.events.lower(64)= end_configuration(16);
          problem.bounds.events.lower(17)=initial_configuration(17);        problem.bounds.events.lower(65)= end_configuration(17);

          problem.bounds.events.lower(18)=initial_configuration(18);        problem.bounds.events.lower(66)=end_configuration(18);
          problem.bounds.events.lower(19)=initial_configuration(19);        problem.bounds.events.lower(67)= end_configuration(19);
          problem.bounds.events.lower(20)=initial_configuration(20);        problem.bounds.events.lower(68)=end_configuration(20);
          problem.bounds.events.lower(21)=initial_configuration(21);        problem.bounds.events.lower(69)= end_configuration(21);
          problem.bounds.events.lower(22)=initial_configuration(22);        problem.bounds.events.lower(70)=end_configuration(22);
          problem.bounds.events.lower(23)=initial_configuration(23);        problem.bounds.events.lower(71)=end_configuration(23);


          //Initial joint velocities                 //Final joint velocities
          problem.bounds.events.lower(24)=0;         problem.bounds.events.lower(72)=0;
          problem.bounds.events.lower(25)=0;         problem.bounds.events.lower(73)=0;
          problem.bounds.events.lower(26)=0;         problem.bounds.events.lower(74)=0;
          problem.bounds.events.lower(27)=0;         problem.bounds.events.lower(75)=0;
          problem.bounds.events.lower(28)=0;         problem.bounds.events.lower(76)=0;
          problem.bounds.events.lower(29)=0;         problem.bounds.events.lower(77)=0;
          problem.bounds.events.lower(30)=0;         problem.bounds.events.lower(78)=0;
          problem.bounds.events.lower(31)=0;         problem.bounds.events.lower(79)=0;
          problem.bounds.events.lower(32)=0;         problem.bounds.events.lower(80)=0;
          problem.bounds.events.lower(33)=0;         problem.bounds.events.lower(81)=0;
          problem.bounds.events.lower(34)=0;         problem.bounds.events.lower(82)=0;
          problem.bounds.events.lower(35)=0;         problem.bounds.events.lower(83)=0;
          problem.bounds.events.lower(36)=0;         problem.bounds.events.lower(84)=0;
          problem.bounds.events.lower(37)=0;         problem.bounds.events.lower(85)=0;
          problem.bounds.events.lower(38)=0;         problem.bounds.events.lower(86)=0;
          problem.bounds.events.lower(39)=0;         problem.bounds.events.lower(87)=0;
          problem.bounds.events.lower(40)=0;         problem.bounds.events.lower(88)=0;
          problem.bounds.events.lower(41)=0;         problem.bounds.events.lower(89)=0;
          problem.bounds.events.lower(42)=0;         problem.bounds.events.lower(90)=0;
          problem.bounds.events.lower(43)=0;         problem.bounds.events.lower(91)=0;
          problem.bounds.events.lower(44)=0;         problem.bounds.events.lower(92)=0;
          problem.bounds.events.lower(45)=0;         problem.bounds.events.lower(93)=0;
          problem.bounds.events.lower(46)=0;         problem.bounds.events.lower(94)=0;
          problem.bounds.events.lower(47)=0;         problem.bounds.events.lower(95)=0;


        problem.bounds.events.upper=problem.bounds.events.lower;


   //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
   //&---------Set the collocation information of the problem---------------&
   //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

   problem.colMethod                    ="Hermite-Simpson";
   problem.derivatives.gradientCost     ="Numerical";
   problem.derivatives.jacobianCns      ="Numerical";

   //Set the problem depending on the local method selected

   problem.setupNLP();

    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    //&---------------------Set initial guess-------------------------&
    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

    problem.guess.states.setZero();

   //Set a linear distribution in the joint position states

    int n=problem.nCollocationPoints;

   problem.guess.states.row(0).setLinSpaced(n,0, problem.bounds.events.lower(48));
   problem.guess.states.row(1).setLinSpaced(n,0, problem.bounds.events.lower(49));
   problem.guess.states.row(2).setLinSpaced(n,0, problem.bounds.events.lower(50));
   problem.guess.states.row(3).setLinSpaced(n,0, problem.bounds.events.lower(51));
   problem.guess.states.row(4).setLinSpaced(n,0, problem.bounds.events.lower(52));
   problem.guess.states.row(5).setLinSpaced(n,0, problem.bounds.events.lower(53));
   problem.guess.states.row(6).setLinSpaced(n,0, problem.bounds.events.lower(54));
   problem.guess.states.row(7).setLinSpaced(n,0, problem.bounds.events.lower(55));
   problem.guess.states.row(8).setLinSpaced(n,0, problem.bounds.events.lower(56));
   problem.guess.states.row(9).setLinSpaced(n,0, problem.bounds.events.lower(57));
   problem.guess.states.row(10).setLinSpaced(n,0, problem.bounds.events.lower(58));
   problem.guess.states.row(11).setLinSpaced(n,0, problem.bounds.events.lower(59));
   problem.guess.states.row(12).setLinSpaced(n,0, problem.bounds.events.lower(60));
   problem.guess.states.row(13).setLinSpaced(n,0, problem.bounds.events.lower(61));
   problem.guess.states.row(14).setLinSpaced(n,0, problem.bounds.events.lower(62));
   problem.guess.states.row(15).setLinSpaced(n,0, problem.bounds.events.lower(63));
   problem.guess.states.row(16).setLinSpaced(n,0, problem.bounds.events.lower(64));
   problem.guess.states.row(17).setLinSpaced(n,0, problem.bounds.events.lower(65));
   problem.guess.states.row(18).setLinSpaced(n,0, problem.bounds.events.lower(66));
   problem.guess.states.row(19).setLinSpaced(n,0, problem.bounds.events.lower(67));
   problem.guess.states.row(20).setLinSpaced(n,0, problem.bounds.events.lower(68));
   problem.guess.states.row(21).setLinSpaced(n,0, problem.bounds.events.lower(69));
   problem.guess.states.row(22).setLinSpaced(n,0, problem.bounds.events.lower(70));
   problem.guess.states.row(23).setLinSpaced(n,0, problem.bounds.events.lower(71));

   problem.guess.controls.setZero();
   problem.guess.t0=0;
   problem.guess.tF=5;

   //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
   //&---------------------Algorithm options-------------------------&
   //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&


   problem.algorithm.meshRefinement   =true;
   problem.algorithm.derivativeChecker=false;

   problem.algorithm.kappa=0.1;       //Default is 1/10 [Betts,2014]
   problem.algorithm.error_ode=1e-2;
   problem.algorithm.maxPoints=4;
   problem.algorithm.maxMeshIterations=1;


   //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
   //&-------------------Set and solve the problem------------------&
   //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&


   Eigen::VectorXd z(problem.NLP.nDecVar);


   const int nTest=5;

   long durVec[nTest];
   long durVec2[nTest];

   for(int i=0;i<nTest;i++){

   z.setRandom();

   auto start=std::chrono::high_resolution_clock::now();

   double v1= nocs::localGenerator::costFunction(problem,z);


   auto stop = std::chrono::high_resolution_clock::now();


   auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count();

   auto start2=std::chrono::high_resolution_clock::now();

    double v2= nocs::localGenerator::costFcn(problem,z);

   auto stop2 = std::chrono::high_resolution_clock::now();

   auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(stop2 - start2).count();

    std::cout<<v1<<" --  "<<v2<<std::endl;

   durVec2[i]=duration2;
   durVec[i]=duration;

    }//END FOR------------------MULTIPLE ITERATIONS OF THE ALGORITHM


    for(int i=0;i<nTest;i++){

        std::cout<<durVec[i]<<" vs  "<< durVec2[i]<<std::endl;


    }
    return 0;

}
