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

#include "directCollocation/local/fcnGenerator.hpp"
#include "directCollocation/local/userFunctions.hpp"
#include "directCollocation/local/utils.hpp"


void nocs::localGenerator::interpolateSolution(Eigen::VectorXd &x_t, Eigen::VectorXd &u_t,Eigen::VectorXd &f_t, double &t, localCollocation &problem){

    //Find in which segment the interpolated time belongs

    int left_k=0; //k point
    int right_k=0; //k+1 point

    double tk=0;
    double tk_1=0;

    int nStates=problem.nStates;
    int nControls=problem.nControls;

    x_t.setZero(nStates);
    u_t.setZero(nControls);
    f_t.setZero(nStates);

    nocs::utils::locateSegment(problem.mesh.time, t, left_k, right_k);

    //Obtain the value of the time at left_k and right_k

    tk=problem.mesh.time(left_k);
    tk_1=problem.mesh.time(right_k);

    //Interpolate the controls as quadratic spline [See Betts,2014]

    double tau= t-tk;
    double hk = tk_1-tk;

    //Trapezoidal interpolation

    if(problem.colMethod=="Trapezoidal"){

        u_t=problem.solution.uSol.col(left_k)+(tau/hk)*(problem.solution.uSol.col(right_k)-problem.solution.uSol.col(left_k));

        //Interpolate the states as a quadratic spline [See Betts, 2014]

        double tVar=pow(tau,2.0) / 2*hk;

        x_t=problem.solution.xSol.col(left_k) + (problem.solution.fSol.col(left_k)*tau) +tVar*(problem.solution.fSol.col(right_k)-problem.solution.fSol.col(left_k));

        //As the dynamics of the system vary lineary between the collocation points interpolate using a linear spline

        f_t=problem.solution.fSol.col(left_k)+(tau/hk)*(problem.solution.fSol.col(right_k)-problem.solution.fSol.col(left_k));

    } else if(problem.colMethod=="Hermite-Simpson"){

        //Hermite-Simpson interpolation

        //Obtain the number of the collocation points that belongs to that segment
        Eigen::Vector3d segmentPoints=problem.NLP.colPointsRef.row(left_k);

        Eigen::VectorXd uk(nControls);      //Controls at k
        Eigen::VectorXd uk_bar(nControls);  //Controls at k+0.5
        Eigen::VectorXd uk1(nControls);     //Controls at k+1

        uk=problem.solution.uSol.col(segmentPoints(0));
        uk_bar=problem.solution.uSol.col(segmentPoints(1));
        uk1=problem.solution.uSol.col(segmentPoints(2));

        double tVar1=(2.0/pow(hk,2.0))*(tau-(hk/2.0))*(tau-hk);
        double tVar2=(4.0/pow(hk,2.0))*tau*(tau-hk);
        double tVar3=(2.0/pow(hk,2.0))*tau*(tau-(hk/2.0));

        u_t=(tVar1*uk) - (tVar2*uk_bar) + (tVar3*uk1);

        //As the dynamics can be represented as a piecewise quadratic segment use a quadratic spline for the interpolation

        Eigen::VectorXd fk(nStates);      //Derivatives at k
        Eigen::VectorXd fk_bar(nStates);  //Derivatives at k+0.5
        Eigen::VectorXd fk1(nStates);     //Derivatives at k+1


        fk=problem.solution.fSol.col(segmentPoints(0));
        fk_bar=problem.solution.fSol.col(segmentPoints(1));
        fk1=problem.solution.fSol.col(segmentPoints(2));

        f_t=(tVar1*fk) - (tVar2*fk_bar) + (tVar3*fk1);

        //Interpolate the states as a cubic spline

        Eigen::VectorXd xk(nStates);      //States at k

        xk=problem.solution.xSol.col(segmentPoints(0));

        tVar2=pow(tau,2.0)/(2*hk);
        tVar3=pow(tau,3.0)/(3*pow(hk,2.0));

        x_t=xk + (fk*tau) + (tVar2 *(-3*fk + 4*fk_bar - fk1)) + (tVar3 *(2*fk - 4*fk_bar + 2*fk1));


    }//End if-else



}//End interpolateSolution


