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



#include "directCollocation/local/local.hpp"
#include "directCollocation/local/utils.hpp"

namespace nocs {

void localCollocation::transcription(){


    //1. Get the bounds of the decision variable vector
    // z_lb < z < z_ub

    this->setVariableBounds();


    //2. Get the constraints bounds of the constraint vector
    // glb <= c(z) <= gub
    this->setCnsBounds();


    //3. Get the initial solution of the NLP problem
    this->setInitialSolution();

}//End localCollocation::transcription()

void localCollocation::setVariableBounds(){

    //--------------
    //Lower bounds
    //--------------

    Eigen::VectorXd zlb(nStates+nControls);

    zlb<<bounds.states.lower,bounds.controls.lower;

    //Set the lower bounds of the initial and final time variables
    NLP.xlb(0)=bounds.initialTime.lower(0);
    NLP.xlb(1)=bounds.finalTime.lower(0);

    //Iterate along the discrete points of the phase i
    for (int k=0;k<nCollocationPoints;k++) {

        NLP.xlb.segment(2+(zlb.size()*k),zlb.size())=zlb;

    }//End for int k=0

    //--------------
    //Upper bounds
    //--------------

    Eigen::VectorXd zub(nStates+nControls);

    zub<<bounds.states.upper,bounds.controls.upper;

    //Set the lower bounds of the initial and final time variables
    NLP.xub(0)=bounds.initialTime.upper(0);
    NLP.xub(1)=bounds.finalTime.upper(0);

    for (int k=0;k<nCollocationPoints;k++) { //Iterate along the discrete points of the phase i

        NLP.xub.segment(2+(zub.size()*k),zub.size())=zub;

    }//End for int k=0


}//End localCollocation::setVariableBounds()

void localCollocation::setCnsBounds(){

    //The structure of the constraint vector for single phase is defined as:

    // c(z)=[defectCns' eventCns' pathCns']

        int nDefectCns=NLP.nCollocationCns;

        //---------- Lower bounds

        //The first elements of the vector are zeros because of the equality constraint for the defectCns

        NLP.glb.head(nDefectCns).setZero();

        //Bound of the event constraints

        NLP.glb.segment(nDefectCns,nEvents)=bounds.events.lower;

        //Bound of the path constraints

        for (int k=0;k<nCollocationPoints;k++) {

            //Put the path constraint bounds in k instant

            NLP.glb.segment(nDefectCns+nEvents+(nPath*k),nPath)=bounds.path.lower;

        }//End for int k=0

        //------------------
        //Upper bounds
        //------------------

        //The first elements of the vector are zeros because of the equality constraint for the defectCns

        NLP.gub.head(nDefectCns).setZero();

        //Bound of the event constraints

        NLP.gub.segment(nDefectCns,nEvents)=bounds.events.upper;

        //Bound of the path constraints

        for (int k=0;k<nCollocationPoints;k++) {

            //Put the path constraint bounds in k instant

            NLP.gub.segment(nDefectCns+nEvents+(nPath*k),nPath)=bounds.path.upper;

         }//End for int k=0

        //-----------------
        //Time constraint
        //-----------------

        double diff_t0Min_tfMax=bounds.initialTime.lower(0)-bounds.finalTime.upper(0);

        NLP.glb(NLP.nCns-1)=diff_t0Min_tfMax;
        NLP.gub(NLP.nCns-1)=0.0;

} //End localCollocation::setCnsBounds()

void localCollocation::setInitialSolution(){

    //Trapezoidal method  ---> [t0, tF, x1, u1, x2, u2...., xM, UM]
    //HSS method  ---> [t0, tF, x1, u1, x_1, u_1, x2, u2 , x_2 ,u _2...., xM, UM]

     Eigen::MatrixXd decVar(nStates+nControls,nCollocationPoints);


     decVar<<guess.states,guess.controls;   //Concatenate a matrix with the initial guess of states and controls

     Eigen::Map<Eigen::VectorXd> vecX0(decVar.data(),decVar.size());  //Transform the matrix into vector

     NLP.x0<<guess.t0,guess.tF,vecX0; //Concatenate the guess vector

}// End localCollocation::setInitialSolution

void localCollocation::setWarmStart(){

    int nColPoints=0;

    Eigen::VectorXd timeVector(this->nDiscretePoints);
    Eigen::VectorXd interpolatedTimeVector;



    if(this->colMethod=="Hermite-Simpson" || this->algorithm.meshRefinement==true){
        //Compute the number of collocation points in the new mesh and set the vector size
        nColPoints=2*this->nDiscretePoints-1;
        interpolatedTimeVector.setZero(nColPoints);
        //Now obtain the time vector using the tau vector information and obtain the interpolated vector for the HS method
        nocs::utils::getTimeVector(this->solution.t0,this->solution.tF,this->mesh.tauV,timeVector);
        nocs::utils::getTimeVector_HSS(timeVector,interpolatedTimeVector);

    }else{

        //In the trapezoidal method the number of collocation points and the discrete points are the same
        nColPoints=this->nDiscretePoints;
        interpolatedTimeVector.setZero(nColPoints);
        //Obtain the time vector
        nocs::utils::getTimeVector(this->solution.t0,this->solution.tF,this->mesh.tauV,timeVector);
        interpolatedTimeVector=timeVector;

    }


    //Declare the interpolated vectors
    Eigen::VectorXd x_t(this->nStates);
    Eigen::VectorXd u_t(this->nControls);
    Eigen::VectorXd f_t(this->nStates);


    //Resize the guess information

    Eigen::MatrixXd states_guess(this->nStates,nColPoints);
    Eigen::MatrixXd control_guess(this->nControls,nColPoints);


    //Iterate along the interpolated time vector
    for(int k=0;k<nColPoints;k++){

        //Obtain the time value
        double t=interpolatedTimeVector(k);

        //Interpolate the solution of this iteration to the new time vector
        localCollocation::interpolateSolution(x_t,u_t,f_t,t,*this);

        states_guess.col(k)=x_t;
        control_guess.col(k)=u_t;


    }//End for


    this->guess.states.resize(this->nStates,nColPoints);
    this->guess.controls.resize(this->nControls,nColPoints);

    this->guess.states=states_guess;
    this->guess.controls=control_guess;

}//End localCollocation::setWarmStart

}//End nocs namespace
