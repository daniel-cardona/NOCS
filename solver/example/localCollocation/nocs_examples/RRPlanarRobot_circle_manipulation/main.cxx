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

#define pi 3.1416

using namespace std::chrono; 

using namespace std;

namespace plt = matplotlibcpp;


int main(){

    int maxIter=1;
    
    //Set the main data of the optimal control problem

    int nStates=10;
    int nControls=4;
    int nEvents=17;
    int nPath=2;
    int nDiscretePoints=15;

    nocs::localCollocation problem(nStates,nControls,nDiscretePoints,nPath,nEvents);

    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    //&-----------------------Variable bounds-------------------------&
    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

    double q1L  =-2*pi;
    double q2L  =-2*pi;
    double q3L  =-2*pi;
    double q4L  = -3;
    double q5L  = -3;
    double q1dL =-10.0;
    double q2dL =-10.0;
    double q3dL =-10.0;
    double q4dL =-10.0;
    double q5dL =-10.0;

    double q1U = 2*pi;
    double q2U = 2*pi;
    double q3U = 2*pi;
    double q4U = 3.0;
    double q5U = 3.0;
    double q1dU =10.0;
    double q2dU =10.0;
    double q3dU =10.0;
    double q4dU =10.0;
    double q5dU =10.0;

    double T1L = -10.0;
    double T2L = -10.0;
    double T3L = -10.0;
    double lambdaL=0;

    double T1U = 10.0;
    double T2U = 10.0;
    double T3U = 10.0;
    double lambdaU=10.0;

    double q1_t0  = 0;
    double q2_t0  = 0;
    double q3_t0  = 0;
    double q4_t0  = 1;
    double q5_t0  = 0.5;

    double q1d_t0 = 0;
    double q2d_t0 = 0;
    double q3d_t0 = 0;
    double q4d_t0 = 0;
    double q5d_t0 = 0;


    double q1_tF  = 1.5707;
    double q2_tF  = 0;
    double q3_tF  = 0;
    double q4_tF  = -2;
    double q5_tF  = 1.5;

    double q1d_tF = 0;
    double q2d_tF = 0;
    double q3d_tF = 0;
    double q4d_tF = 0;
    double q5d_tF = 0;



    problem.bounds.states.lower(0)=q1L;
    problem.bounds.states.lower(1)=q2L;
    problem.bounds.states.lower(2)=q3L;
    problem.bounds.states.lower(3)=q4L;
    problem.bounds.states.lower(4)=q5L;
    problem.bounds.states.lower(5)=q1dL;
    problem.bounds.states.lower(6)=q2dL;
    problem.bounds.states.lower(7)=q3dL;
    problem.bounds.states.lower(8)=q4dL;
    problem.bounds.states.lower(9)=q5dL;

    problem.bounds.states.upper(0)=q1U;
    problem.bounds.states.upper(1)=q2U;
    problem.bounds.states.upper(2)=q3U;
    problem.bounds.states.upper(3)=q4U;
    problem.bounds.states.upper(4)=q5U;
    problem.bounds.states.upper(5)=q1dU;
    problem.bounds.states.upper(6)=q2dU;
    problem.bounds.states.upper(7)=q3dU;
    problem.bounds.states.upper(8)=q4dU;
    problem.bounds.states.upper(9)=q5dU;

    problem.bounds.controls.lower(0)=T1L;
    problem.bounds.controls.lower(1)=T2L;
    problem.bounds.controls.lower(2)=T3L;
    problem.bounds.controls.lower(3)=lambdaL;

    problem.bounds.controls.upper(0)=T1U;
    problem.bounds.controls.upper(1)=T2U;
    problem.bounds.controls.upper(2)=T3U;
    problem.bounds.controls.upper(3)=lambdaU;

    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    //&---------------Boundary and path constraints-------------------&
    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

    problem.bounds.events.lower(0)=q1_t0;
    problem.bounds.events.lower(1)=q2_t0;
    problem.bounds.events.lower(2)=q3_t0;
    problem.bounds.events.lower(3)=q4_t0;
    problem.bounds.events.lower(4)=q5_t0;
    problem.bounds.events.lower(5)=q1d_t0;
    problem.bounds.events.lower(6)=q2d_t0;
    problem.bounds.events.lower(7)=q3d_t0;
    problem.bounds.events.lower(8)=q4d_t0;
    problem.bounds.events.lower(9)=q5d_t0;

    problem.bounds.events.lower(10)=q4_tF;
    problem.bounds.events.lower(11)=q5_tF;
    problem.bounds.events.lower(12)=q1d_tF;
    problem.bounds.events.lower(13)=q2d_tF;
    problem.bounds.events.lower(14)=q3d_tF;
    problem.bounds.events.lower(15)=q4d_tF;
    problem.bounds.events.lower(16)=q5d_tF;

    problem.bounds.events.upper=problem.bounds.events.lower;

    problem.bounds.path.lower(0)=0;
    problem.bounds.path.upper(0)=1e17;

    problem.bounds.path.lower(1)=0;
    problem.bounds.path.upper(1)=0;

    problem.bounds.initialTime.lower(0)=0;
    problem.bounds.initialTime.upper(0)=0;

    problem.bounds.finalTime.lower(0)=0;
    problem.bounds.finalTime.upper(0)=10.0;

    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    //&---------Set the collocation information of the problem---------------&
    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

    problem.colMethod="Hermite-Simpson";
    problem.derivatives.gradientCost="Numerical";
    problem.derivatives.jacobianCns="Numerical";

    //Set the problem depending on the local method selected

    problem.setupNLP();

    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    //&---------------------Set initial guess-------------------------&
    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

    int nNodes=problem. nCollocationPoints;

    //nNodes=problem->nDiscretePoints;

    problem.guess.states.row(0).setLinSpaced(nNodes,q1_t0,q1_tF);
    problem.guess.states.row(1).setLinSpaced(nNodes,q2_t0,q2_tF);
    problem.guess.states.row(2).setLinSpaced(nNodes,q3_t0,q3_tF);
    problem.guess.states.row(3).setLinSpaced(nNodes,q4_t0,q4_tF);
    problem.guess.states.row(4).setLinSpaced(nNodes,q5_t0,q5_tF);

    problem.guess.states.row(5).setLinSpaced(nNodes,q1d_t0,q1d_tF);
    problem.guess.states.row(6).setLinSpaced(nNodes,q2d_t0,q2d_tF);
    problem.guess.states.row(7).setLinSpaced(nNodes,q3d_t0,q3d_tF);
    problem.guess.states.row(8).setLinSpaced(nNodes,q4d_t0,q4d_tF);
    problem.guess.states.row(9).setLinSpaced(nNodes,q5d_t0,q5d_tF);

    problem.guess.controls.row(0).setZero();
    problem.guess.controls.row(1).setZero();
    problem.guess.controls.row(2).setZero();
    problem.guess.controls.row(3).setZero();

    problem.guess.t0=0;
    problem.guess.tF=10;

    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    //&---------------------Algorithm options-------------------------&
    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&


    problem.algorithm.meshRefinement   =true;
    problem.algorithm.derivativeChecker=true;

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

      int N=probSol.solution.tSol.size();

      std::vector<double> q1_sol(N), q2_sol(N), q3_sol(N);
      std::vector<double> q4_sol(N), q5_sol(N);

      std::vector<double> x0(N),y0(N);
      std::vector<double> x1(N),y1(N);
      std::vector<double> x2(N),y2(N);
      std::vector<double> x3(N),y3(N);

//      std::vector<double> x_q1(N),y_q1(N);
//      std::vector<double> x_q2(N),y_q2(N);


//    std::vector<double> qd1_sol(N), qd2_sol(N);
//    std::vector<double> q1dd(N), q2dd(N);
//    std::vector<double> u1_sol(N),u2_sol(N);
//    std::vector<double> t(N);

      double q1k,q2k,q3k;
      double l1=1;
      double l2=1;
      double l3=1;


      for (int i=0; i<N;i++){

          q1_sol.at(i)=probSol.solution.xSol(0,i);
          q2_sol.at(i)=probSol.solution.xSol(1,i);
          q3_sol.at(i)=probSol.solution.xSol(2,i);

          q4_sol.at(i)=probSol.solution.xSol(3,i);
          q5_sol.at(i)=probSol.solution.xSol(4,i);

          q1k=q1_sol.at(i);
          q2k=q2_sol.at(i);
          q3k=q3_sol.at(i);

          x1.at(i)=l1*cos(q1k);
          y1.at(i)=l1*sin(q1k);

          x2.at(i)=l1*cos(q1k)+l2*cos(q1k+q2k);
          y2.at(i)=l1*sin(q1k)+l2*sin(q1k+q2k);

          x3.at(i)=l1*cos(q1k)+l2*cos(q1k+q2k)+l3*cos(q1k+q2k+q3k);
          y3.at(i)=l1*sin(q1k)+l2*sin(q1k+q2k)+l3*sin(q1k+q2k+q3k);


      }

      Eigen::VectorXd theta;

      //double radius=sqrt(0.25);
      double radius=0.25;

      theta.setLinSpaced(N,0,2*pi);

      Eigen::VectorXd cos_theta;
      Eigen::VectorXd sin_theta;

      cos_theta=cos(theta.array());
      sin_theta=sin(theta.array());

      Eigen::VectorXd xObs1(theta.size());
      Eigen::VectorXd yObs1(theta.size());


      std::vector<double> xc(theta.size()), yc(theta.size());

//      xObs1=radius*cos_theta.array()+0.75;
//      yObs1=radius*sin_theta.array()+0.75;

//      for (int i=0; i<theta.size();i++){

//          xc.at(i)=xObs1(i);
//          yc.at(i)=yObs1(i);

//      }



      std::vector<double> xq1(2);
      std::vector<double> yq1(2);

      std::vector<double> xq2(2);
      std::vector<double> yq2(2);

      std::vector<double> xq3(2);
      std::vector<double> yq3(2);

      xq1.at(0)=0;
      yq1.at(0)=0;

      for (int i=0; i<N;i++){

          xq1.at(1)=x1.at(i);
          yq1.at(1)=y1.at(i);

          xq2.at(0)=x1.at(i);
          yq2.at(0)=y1.at(i);

          xq2.at(1)=x2.at(i);
          xq2.at(1)=y2.at(i);

          xq3.at(0)=x2.at(i);
          xq3.at(0)=y2.at(i);

          xq3.at(1)=x3.at(i);
          yq3.at(1)=y3.at(i);

          xObs1=radius*cos_theta.array()+q4_sol.at(i);
          yObs1=radius*sin_theta.array()+q5_sol.at(i);

          for (int i=0; i<theta.size();i++){

              xc.at(i)=xObs1(i);
              yc.at(i)=yObs1(i);

          }

          plt::clf();

          plt::plot(xq1,yq1);
          plt::plot(xq2,yq2);
          plt::plot(xq3,yq3);
          plt::plot(xc,yc);

          plt::xlim(-3,3);
          plt::ylim(-3,3);

          plt::pause(0.05);
      }

//    plt::suptitle("Solution of the states");
//    plt::subplot(2,2,1);
//        plt::title("q1-1st mesh iteration");
//        plt::scatter(t,q1_sol);
//        plt::plot(t,q1_sol);
//        plt::grid(true);
//    plt::subplot(2,2,2);
//        plt::title("q2-1st mesh iterarion");
//        plt::scatter(t,q2_sol,8);
//        plt::plot(t,q2_sol,"r-");
//        plt::grid(true);
//    plt::subplot(2,2,3);
//       plt::title("qd1-1st mesh iterarion");
//       plt::scatter(t,qd1_sol,8);
//       plt::plot(t,qd1_sol,"b-");
//       plt::grid(true);
//    plt::subplot(2,2,4);
//       plt::title("qd2-1st mesh iterarion");
//       plt::scatter(t,qd2_sol,8);
//       plt::plot(t,qd2_sol,"r-");
//       plt::grid(true);
//    plt::show();

//    plt::suptitle("Control inputs");
//    plt::subplot(2,1,1);
//        plt::title("u1-1st mesh iteration");
//        plt::scatter(t,u1_sol,8);
//        plt::plot(t,u1_sol,"k-");
//        plt::grid(true);
//    plt::subplot(2,1,2);
//        plt::title("u2-1st mesh iterarion");
//        plt::scatter(t,u2_sol,8);
//        plt::plot(t,u2_sol,"r-");
//        plt::grid(true);
//    plt::show();

//    plt::suptitle("Derivatives");
//    plt::subplot(2,1,1);
//        plt::title("q1dd-First mesh iterarion");
//        plt::scatter(t,q1dd,3);
//        plt::plot(t,q1dd,"k-");
//        plt::grid(true);
//    plt::subplot(2,1,2);
//        plt::title("q2dd-First mesh iterarion");
//        plt::scatter(t,q2dd,3);
//        plt::plot(t,q2dd,"r-");
//        plt::grid(true);
//    plt::show();

    return 0;

};


