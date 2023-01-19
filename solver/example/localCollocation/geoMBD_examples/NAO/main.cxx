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

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

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
    int nDiscretePoints=30;

    const int nTest=1;

    long durVec[nTest];


    for(int i=0;i<nTest;i++){

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

     Eigen::VectorXd initial_configuration(problem.robotGeo.value()->nq);
     Eigen::VectorXd end_configuration(problem.robotGeo.value()->nq);

     initial_configuration.setZero();

     /*/Pose 1 final // POse 3 initial
     initial_configuration<<   -0.0070,
                           -0.8639,
                            1.9967,
                           -1.1345,
                            0.0052,
                            0.0000,
                            0.0018,
                            0.8919,
                           -2.0822,
                           -0.0436,
                           -0.0000,
                           -0.9861,
                           -0.0000,
                            0.0018,
                           -0.9756,
                            2.0734,
                            0.0384,
                           -0.0000,
                            0.0000,
                           -0.0018,
                           -1.1345,
                            1.9967,
                           -0.8639,
                            0.0035; */

     /*/Pose 2
     end_configuration<<     0.0611, -1.0385,2.0909,-1.0385,0.4154,-0.0018,
                             0.1885,0.1030,-0.0035,-1.4608,-0.0000,-1.1833,
                            -0.0000,1.0454,-1.3247,1.5673,0.0785,0.0000,
                            -0.0018,-0.5463,-0.1065,-0.0000,-0.0087,0.3944;*/

     /*/Pose 3
     end_configuration<< 0.06109,
                         -1.03847,
                         2.09090,
                         -1.03847,
                         0.41539,
                         -0.00175,
                         0.18850,
                         0.10297,
                         -0.00349,
                         -1.46084,
                         0.00000,
                         -1.18333,
                         0.00000,
                         1.04545,
                         -1.32470,
                         1.56731,
                         0.07854,
                         0.00000,
                         -0.00175,
                         -0.54629,
                         -0.10647,
                         0,
                         -0.00873,
                         0.39444;*/

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
   problem.derivatives.gradientCost     ="Analytical";
   problem.derivatives.jacobianCns      ="Analytical";

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


       nocs::localCollocation probSol;

       auto start = std::chrono::high_resolution_clock::now();
        nocs::nocsLocal(problem,probSol);
       auto stop = std::chrono::high_resolution_clock::now();

       auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count();

       cout <<"Time required for full resolution of the problem: " <<duration<<" microseconds"<< endl;
       durVec[i]=duration;


    }//END FOR------------------MULTIPLE ITERATIONS OF THE ALGORITHM


    for(int i=0;i<nTest;i++){

        std::cout<<durVec[i]<<std::endl;


    }


   //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
   //&-------------------Plot the solution --------------------------&
   //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
   /*
   int N=probSol.solution.tSol.size();

       std::vector<double> q1_sol(N), q2_sol(N),q3_sol(N), q4_sol(N),q5_sol(N), q6_sol(N),q7_sol(N), q8_sol(N);
       std::vector<double> q9_sol(N), q10_sol(N),q11_sol(N), q12_sol(N),q13_sol(N), q14_sol(N),q15_sol(N), q16_sol(N);
       std::vector<double> q17_sol(N), q18_sol(N),q19_sol(N), q20_sol(N),q21_sol(N), q22_sol(N),q23_sol(N), q24_sol(N);

       std::vector<double> qd1_sol(N), qd2_sol(N),qd3_sol(N), qd4_sol(N),qd5_sol(N), qd6_sol(N),qd7_sol(N), qd8_sol(N);
       std::vector<double> qd9_sol(N), qd10_sol(N),qd11_sol(N), qd12_sol(N),qd13_sol(N), qd14_sol(N),qd15_sol(N), qd16_sol(N);
       std::vector<double> qd17_sol(N), qd18_sol(N),qd19_sol(N), qd20_sol(N),qd21_sol(N), qd22_sol(N),qd23_sol(N), qd24_sol(N);

       std::vector<double> qdd1_sol(N), qdd2_sol(N),qdd3_sol(N), qdd4_sol(N),qdd5_sol(N), qdd6_sol(N),qdd7_sol(N), qdd8_sol(N);
       std::vector<double> qdd9_sol(N), qdd10_sol(N),qdd11_sol(N), qdd12_sol(N),qdd13_sol(N), qdd14_sol(N),qdd15_sol(N), qdd16_sol(N);
       std::vector<double> qdd17_sol(N), qdd18_sol(N),qdd19_sol(N), qdd20_sol(N),qdd21_sol(N), qdd22_sol(N),qdd23_sol(N), qdd24_sol(N);

       std::vector<double> u1_sol(N),u2_sol(N),u3_sol(N),u4_sol(N),u5_sol(N),u6_sol(N),u7_sol(N),u8_sol(N);
       std::vector<double> u9_sol(N), u10_sol(N),u11_sol(N), u12_sol(N),u13_sol(N), u14_sol(N),u15_sol(N), u16_sol(N);
       std::vector<double> u17_sol(N), u18_sol(N),u19_sol(N), u20_sol(N),u21_sol(N), u22_sol(N),u23_sol(N), u24_sol(N);


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
           q9_sol.at(i)=probSol.solution.xSol(8,i);
           q10_sol.at(i)=probSol.solution.xSol(9,i);
           q11_sol.at(i)=probSol.solution.xSol(10,i);
           q12_sol.at(i)=probSol.solution.xSol(11,i);
           q13_sol.at(i)=probSol.solution.xSol(12,i);
           q14_sol.at(i)=probSol.solution.xSol(13,i);
           q15_sol.at(i)=probSol.solution.xSol(14,i);
           q16_sol.at(i)=probSol.solution.xSol(15,i);
           q17_sol.at(i)=probSol.solution.xSol(16,i);
           q18_sol.at(i)=probSol.solution.xSol(17,i);
           q19_sol.at(i)=probSol.solution.xSol(18,i);
           q20_sol.at(i)=probSol.solution.xSol(19,i);
           q21_sol.at(i)=probSol.solution.xSol(20,i);
           q22_sol.at(i)=probSol.solution.xSol(21,i);
           q23_sol.at(i)=probSol.solution.xSol(22,i);
           q24_sol.at(i)=probSol.solution.xSol(23,i);

           qd1_sol.at(i)=probSol.solution.xSol(24,i);
           qd2_sol.at(i)=probSol.solution.xSol(25,i);
           qd3_sol.at(i)=probSol.solution.xSol(26,i);
           qd4_sol.at(i)=probSol.solution.xSol(27,i);
           qd5_sol.at(i)=probSol.solution.xSol(28,i);
           qd6_sol.at(i)=probSol.solution.xSol(29,i);
           qd7_sol.at(i)=probSol.solution.xSol(30,i);
           qd8_sol.at(i)=probSol.solution.xSol(31,i);
           qd9_sol.at(i)=probSol.solution.xSol(32,i);
           qd10_sol.at(i)=probSol.solution.xSol(33,i);
           qd11_sol.at(i)=probSol.solution.xSol(34,i);
           qd12_sol.at(i)=probSol.solution.xSol(35,i);
           qd13_sol.at(i)=probSol.solution.xSol(36,i);
           qd14_sol.at(i)=probSol.solution.xSol(37,i);
           qd15_sol.at(i)=probSol.solution.xSol(38,i);
           qd16_sol.at(i)=probSol.solution.xSol(39,i);
           qd17_sol.at(i)=probSol.solution.xSol(40,i);
           qd18_sol.at(i)=probSol.solution.xSol(41,i);
           qd19_sol.at(i)=probSol.solution.xSol(42,i);
           qd20_sol.at(i)=probSol.solution.xSol(43,i);
           qd21_sol.at(i)=probSol.solution.xSol(44,i);
           qd22_sol.at(i)=probSol.solution.xSol(45,i);
           qd23_sol.at(i)=probSol.solution.xSol(46,i);
           qd24_sol.at(i)=probSol.solution.xSol(47,i);


           qdd1_sol.at(i)=probSol.solution.fSol(0,i);
           qdd2_sol.at(i)=probSol.solution.fSol(1,i);
           qdd3_sol.at(i)=probSol.solution.fSol(2,i);
           qdd4_sol.at(i)=probSol.solution.fSol(3,i);
           qdd5_sol.at(i)=probSol.solution.fSol(4,i);
           qdd6_sol.at(i)=probSol.solution.fSol(5,i);
           qdd7_sol.at(i)=probSol.solution.fSol(6,i);
           qdd8_sol.at(i)=probSol.solution.fSol(7,i);
           qdd9_sol.at(i)=probSol.solution.fSol(8,i);
           qdd10_sol.at(i)=probSol.solution.fSol(9,i);
           qdd11_sol.at(i)=probSol.solution.fSol(10,i);
           qdd12_sol.at(i)=probSol.solution.fSol(11,i);
           qdd13_sol.at(i)=probSol.solution.fSol(12,i);
           qdd14_sol.at(i)=probSol.solution.fSol(13,i);
           qdd15_sol.at(i)=probSol.solution.fSol(14,i);
           qdd16_sol.at(i)=probSol.solution.fSol(15,i);
           qdd17_sol.at(Trapezoidali)=probSol.solution.fSol(16,i);
           qdd18_sol.at(i)=probSol.solution.fSol(17,i);
           qdd19_sol.at(i)=probSol.solution.fSol(18,i);
           qdd20_sol.at(i)=probSol.solution.fSol(19,i);
           qdd21_sol.at(i)=probSol.solution.fSol(20,i);
           qdd22_sol.at(i)=probSol.solution.fSol(21,i);
           qdd23_sol.at(i)=probSol.solution.fSol(22,i);
           qdd24_sol.at(i)=probSol.solution.fSol(23,i);

           u1_sol.at(i)=probSol.solution.uSol(0,i);
           u2_sol.at(i)=probSol.solution.uSol(1,i);
           u3_sol.at(i)=probSol.solution.uSol(2,i);
           u4_sol.at(i)=probSol.solution.uSol(3,i);
           u5_sol.at(i)=probSol.solution.uSol(4,i);
           u6_sol.at(i)=probSol.solution.uSol(5,i);
           u7_sol.at(i)=probSol.solution.uSol(6,i);
           u8_sol.at(i)=probSol.solution.uSol(7,i);
           u9_sol.at(i)=probSol.solution.uSol(8,i);
           u10_sol.at(i)=probSol.solution.uSol(9,i);
           u11_sol.at(i)=probSol.solution.uSol(10,i);
           u12_sol.at(i)=probSol.solution.uSol(11,i);
           u13_sol.at(i)=probSol.solution.uSol(12,i);
           u14_sol.at(i)=probSol.solution.uSol(13,i);
           u15_sol.at(i)=probSol.solution.uSol(14,i);
           u16_sol.at(i)=probSol.solution.uSol(15,i);
           u17_sol.at(i)=probSol.solution.uSol(16,i);
           u18_sol.at(i)=probSol.solution.uSol(17,i);
           u19_sol.at(i)=probSol.solution.uSol(18,i);
           u20_sol.at(i)=probSol.solution.uSol(19,i);
           u21_sol.at(i)=probSol.solution.uSol(20,i);
           u22_sol.at(i)=probSol.solution.uSol(21,i);
           u23_sol.at(i)=probSol.solution.uSol(22,i);
           u24_sol.at(i)=probSol.solution.uSol(23,i);

           t.at(i)=probSol.solution.tSol(i);

       }

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
           plt::plot(t,q9_sol);
           plt::scatter(t,q9_sol,4.0);
           plt::plot(t,q10_sol);
           plt::scatter(t,q10_sol,4.0);
           plt::plot(t,q11_sol);
           plt::scatter(t,q11_sol,4.0);
           plt::plot(t,q12_sol);
           plt::scatter(t,q12_sol,4.0);
           plt::plot(t,q13_sol);
           plt::scatter(t,q13_sol,4.0);
           plt::plot(t,q14_sol);
           plt::scatter(t,q14_sol,4.0);
           plt::plot(t,q15_sol);
           plt::scatter(t,q15_sol,4.0);
           plt::plot(t,q16_sol);
           plt::scatter(t,q16_sol,4.0);
           plt::plot(t,q17_sol);
           plt::scatter(t,q17_sol,4.0);
           plt::plot(t,q18_sol);
           plt::scatter(t,q18_sol,4.0);
           plt::plot(t,q19_sol);
           plt::scatter(t,q19_sol,4.0);
           plt::plot(t,q20_sol);
           plt::scatter(t,q20_sol,4.0);
           plt::plot(t,q21_sol);
           plt::scatter(t,q21_sol,4.0);
           plt::plot(t,q22_sol);
           plt::scatter(t,q22_sol,4.0);
           plt::plot(t,q23_sol);
           plt::scatter(t,q23_sol,4.0);
           plt::plot(t,q24_sol);
           plt::scatter(t,q24_sol,4.0);
           plt::grid(true);
           plt::show();

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
           plt::plot(t,qd9_sol);
           plt::scatter(t,qd9_sol,4.0);
           plt::plot(t,qd10_sol);
           plt::scatter(t,qd10_sol,4.0);
           plt::plot(t,qd11_sol);
           plt::scatter(t,qd11_sol,4.0);
           plt::plot(t,qd12_sol);
           plt::scatter(Trapezoidalt,qd12_sol,4.0);
           plt::plot(t,qd13_sol);
           plt::scatter(t,qd13_sol,4.0);
           plt::plot(t,qd14_sol);
           plt::scatter(t,qd14_sol,4.0);
           plt::plot(t,qd15_sol);
           plt::scatter(t,qd15_sol,4.0);
           plt::plot(t,qd16_sol);
           plt::scatter(t,qd16_sol,4.0);
           plt::plot(t,qd17_sol);
           plt::scatter(t,qd17_sol,4.0);
           plt::plot(t,qd18_sol);
           plt::scatter(t,qd18_sol,4.0);
           plt::plot(t,qd19_sol);
           plt::scatter(t,qd19_sol,4.0);
           plt::plot(t,qd20_sol);
           plt::scatter(t,qd20_sol,4.0);
           plt::plot(t,qd21_sol);
           plt::scatter(t,qd21_sol,4.0);
           plt::plot(t,qd22_sol);
           plt::scatter(t,qd22_sol,4.0);
           plt::plot(t,qd23_sol);
           plt::scatter(t,qd23_sol,4.0);
           plt::plot(t,qd24_sol);
           plt::scatter(t,qd24_sol,4.0);
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
           plt::plot(t,u9_sol);
           plt::scatter(t,u9_sol,4.0);
           plt::plot(t,u10_sol);
           plt::scatter(t,u10_sol,4.0);
           plt::plot(t,u11_sol);
           plt::scatter(t,u11_sol,4.0);
           plt::plot(t,u12_sol);
           plt::scatter(t,u12_sol,4.0);
           plt::plot(t,u13_sol);
           plt::scatter(t,u13_sol,4.0);
           plt::plot(t,u14_sol);
           plt::scatter(t,u14_sol,4.0);
           plt::plot(t,u15_sol);
           plt::scatter(t,u15_sol,4.0);
           plt::plot(t,u16_sol);
           plt::scatter(t,u16_sol,4.0);
           plt::plot(t,u17_sol);
           plt::scatter(t,u17_sol,4.0);
           plt::plot(t,u18_sol);
           plt::scatter(t,u18_sol,4.0);
           plt::plot(t,u19_sol);
           plt::scatter(t,u19_sol,4.0);
           plt::plot(t,u20_sol);
           plt::scatter(t,u20_sol,4.0);
           plt::plot(t,u21_sol);
           plt::scatter(t,u21_sol,4.0);
           plt::plot(t,u22_sol);
           plt::scatter(t,u22_sol,4.0);
           plt::plot(t,u23_sol);
           plt::scatter(t,u23_sol,4.0);
           plt::plot(t,u24_sol);
           plt::scatter(t,u24_sol,4.0);
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
       plt::plot(t,qdd9_sol);
       plt::scatter(t,qdd9_sol,4.0);
       plt::plot(t,qdd10_sol);
       plt::scatter(t,qdd10_sol,4.0);
       plt::plot(t,qdd11_sol);
       plt::scatter(t,qdd11_sol,4.0);
       plt::plot(t,qdd12_sol);
       plt::scatter(t,qdd12_sol,4.0);
       plt::plot(t,qdd13_sol);
       plt::scatter(t,qdd13_sol,4.0);
       plt::plot(t,qdd14_sol);
       plt::scatter(t,qdd14_sol,4.0);
       plt::plot(t,qdd15_sol);
       plt::scatter(t,qdd15_sol,4.0);
       plt::plot(t,qdd16_sol);
       plt::scatter(t,qdd16_sol,4.0);
       plt::plot(t,qdd17_sol);
       plt::scatter(t,qdd17_sol,4.0);
       plt::plot(t,qdd18_sol);
       plt::scatter(t,qdd18_sol,4.0);
       plt::plot(t,qdd19_sol);
       plt::scatter(t,qdd19_sol,4.0);
       plt::plot(t,qdd20_sol);
       plt::scatter(t,qdd20_sol,4.0);
       plt::plot(t,qdd21_sol);
       plt::scatter(t,qdd21_sol,4.0);
       plt::plot(t,qdd22_sol);
       plt::scatter(t,qdd22_sol,4.0);
       plt::plot(t,qdd23_sol);
       plt::scatter(t,qdd23_sol,4.0);
       plt::plot(t,qdd24_sol);
       plt::scatter(t,qdd24_sol,4.0);
       plt::grid(true);
       plt::show();

       */
    return 0;



}
