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

namespace nocs {

namespace localGenerator {

double costFcn(localCollocation &problem, Eigen::VectorXd &z){

    double t0, tf;

    int k_1=0; //Index variable

    double endpoint_cost=0;

    double sum_phase=0.0;

    double sum_cost=0.0;

    Eigen::MatrixXd decVarMatrix(problem.nStates+problem.nControls,problem.nCollocationPoints);

    Eigen::VectorXd controls(problem.nControls);
    Eigen::VectorXd states(problem.nStates);
    Eigen::VectorXd states_k1(problem.nStates);
    Eigen::VectorXd controls_k1(problem.nControls);

    utils::getDecVarMatrix(problem.nStates,problem.nControls,problem.nCollocationPoints,z,decVarMatrix,t0,tf);

    for(int k=0;k<problem.nCollocationPoints;k++){

        double cost_k=0;

        double interval_cost=0;

        //Get the unscaled states and controls in k segment

        utils::getVariables(problem.nStates,problem.nControls,decVarMatrix,states,controls,k);

        //Get the unscaled states and controls in k+1 and save in the problem->state_k_1 variables

        if(k+1==problem.nCollocationPoints){ k_1=k; }
        else{ k_1=k+1; }

        utils::getVariables(problem.nStates,problem.nControls,decVarMatrix,states_k1,controls_k1,k_1);

        problem.NLP.xk1=states_k1;

        double tk=0;

        double ck=problem.NLP.weights(k); //Assign the quadrature weight

        //Evaluate the integral part of the cost function in k

        cost_k=nocs::Function::integrand_cost(states,controls,tk,problem);

        //Evaluate the integral parte of the cost functon in k+1

        //Use the quadrature!

        interval_cost=ck*cost_k;

        sum_phase+=interval_cost;

    }


    sum_cost+=(tf-t0)*problem.NLP.time_weights*sum_phase; //Scale the integral part of the cost function

     //Mayer term (end point cost evaluation)

    Eigen::VectorXd initial_states(problem.nStates);
    Eigen::VectorXd final_states(problem.nStates);

    utils::getVariables(problem.nStates,problem.nControls,decVarMatrix,initial_states,controls,0);              //Obtain the states in t0

    utils::getVariables(problem.nStates,problem.nControls,decVarMatrix,final_states,controls,problem.nCollocationPoints-1); //Obtain the states in tF

    endpoint_cost=nocs::Function::endpoint_cost(initial_states,final_states,t0,tf,problem);

    //Return the total cost!
    sum_cost+=endpoint_cost;

    return sum_cost;





}

//-------------------------------------------------------------------------
//------------- Deprecated functions (Here just for reference)
//-------------------------------------------------------------------------

double costFunction(localCollocation &problem,Eigen::VectorXd &z){

    double t0, tf;

    double endpoint_cost=0;

    double sum_phase=0.0;

    double sum_cost=0.0;

    Eigen::MatrixXd decVarMatrix(problem.nStates+problem.nControls,problem.nCollocationPoints);

    //Variables in k
    Eigen::VectorXd controls(problem.nControls);
    Eigen::VectorXd states(problem.nStates);

    //Variables in k_mid
    Eigen::VectorXd states_mid(problem.nStates);
    Eigen::VectorXd controls_mid(problem.nControls);

    //Variables in k+1
    Eigen::VectorXd states_k1(problem.nStates);
    Eigen::VectorXd controls_k1(problem.nControls);

    utils::getDecVarMatrix(problem.nStates,problem.nControls,problem.nCollocationPoints,z,decVarMatrix,t0,tf);

    double cost_k=0;
    double cost_mid=0;
    double cost_k1=0;

    double interval_cost=0;

    int k_i,k_mid,k1;

    for(int k=0;k<problem.nDiscretePoints-1;k++){

        double h=(tf-t0)*problem.mesh.deltaTau(k);

        //Use the auxiliar collocation matrix to obtain the variables
        k_i=    problem.NLP.colPointsRef(k,0);
        k_mid=  problem.NLP.colPointsRef(k,1);
        k1=     problem.NLP.colPointsRef(k,2);

        //Obtain the variables in time k ,k_mid and k+1
        utils::getVariables(problem.nStates,problem.nControls,decVarMatrix,states,controls,k_i);
        utils::getVariables(problem.nStates,problem.nControls,decVarMatrix,states_mid,controls_mid,k_mid);
        utils::getVariables(problem.nStates,problem.nControls,decVarMatrix,states_k1,controls_k1,k1);

        if(k==0){problem.NLP.xk1=states;} //Initialize the varibles for xk1

        double tk=0;
        double tk_mid=0;
        double tk_1=0;

       //Obtain the value of the Lagrange term in k, k_mid and k_
        problem.NLP.xk1=states_mid;
        cost_k=nocs::Function::integrand_cost(states,controls,tk,problem);
        problem.NLP.xk1=states_k1;
        cost_mid=nocs::Function::integrand_cost(states_mid,controls_mid,tk_mid,problem);

         if(k1<problem.nCollocationPoints-1){

            utils::getVariables(problem.nStates,problem.nControls,decVarMatrix, problem.NLP.xk1,controls,k1+1);

            } else { problem.NLP.xk1=states_k1;}

            cost_k1=nocs::Function::integrand_cost(states_k1,controls_k1,tk_1,problem);

        //Use Simpson cuadrature to integrate the cost function

        interval_cost=(h/6.0)*(cost_k+ (4*cost_mid) +cost_k1 );

        sum_phase+=interval_cost;
    }

    sum_cost+=sum_phase;



    //Mayer term (end point cost evaluation)

    Eigen::VectorXd initial_states(problem.nStates);
    Eigen::VectorXd final_states(problem.nStates);

    utils::getVariables(problem.nStates,problem.nControls,decVarMatrix,initial_states,controls,0);              //Obtain the states in t0

    utils::getVariables(problem.nStates,problem.nControls,decVarMatrix,final_states,controls,problem.nCollocationPoints-1); //Obtain the states in tF

    endpoint_cost=nocs::Function::endpoint_cost(initial_states,final_states,t0,tf,problem);

    //Return the total cost!
    sum_cost+=endpoint_cost;

    return sum_cost;

}

//-------------------------------------------------------------------------
//------------- Derivatives functions
//-------------------------------------------------------------------------

void derivatives::numerical::computeGradientCost(localCollocation &problem,Eigen::VectorXd &z, Eigen::VectorXd &grad){

    int nDecVar=z.size();

    double sqreps;
    double delj;
    double xs=0.0;

    double F1=0.0;
    double F2=0.0;
    double F3=0.0;

    double dfdx=0.0;

    grad.setZero(nDecVar);

    sqreps=sqrt(MC_EPSILON);

    bool c1,c2;

    c1=(z.array()>=(problem.NLP.xlb.array()+sqreps)).any();
    c2=(z.array()<=(problem.NLP.xub.array()-sqreps)).any();

    if(c1||c2){

        F3=costFcn(problem,z);
    }

    for(int j=0;j<nDecVar;j++){

        delj=sqreps*(1.0+fabs(z(j)));

        xs=z(j);

        if(xs< problem.NLP.xub(j)-delj || (problem.NLP.xub(j)==problem.NLP.xlb(j))){
            z(j)+=delj;
            F1=costFcn(problem,z);
        }//End if

        if(xs> problem.NLP.xlb(j)+delj || (problem.NLP.xub(j)==problem.NLP.xlb(j))){
            z(j)=xs-delj;
            F2=costFcn(problem,z);
        }//End if

        if (( (xs< (problem.NLP.xub(j)-delj)) && (xs> (problem.NLP.xlb(j)+delj)) ) || (problem.NLP.xub(j)==problem.NLP.xlb(j)) ) {
          // Use central difference formula
          dfdx = ( F1 - F2 )/(2*delj);
        }
        else if (xs>= (problem.NLP.xub(j)-delj)) {
          // Variable at upper bound, use backward difference formula
          dfdx = ( F2 - F3 )/(-delj);
        }
        else if (xs<= (problem.NLP.xlb(j)+delj)) {
          // Variable at lower bound, use forward difference formula
          dfdx = ( F1 - F3 )/(delj);
        }
        z(j)=xs;

        grad(j)=dfdx;

    }//End for (int k=0)

}//End computeGradientCost

void derivatives::analytical::computeGradientCost(localCollocation &problem,Eigen::VectorXd &z, Eigen::VectorXd &grad){

    //toDo: Compute this time variables(t0,tk)
    double tk=0;
    double tk1=0;

    double deltaT=0;
    double c_prev;

    double t0=0;
    double tF=0;
    int k_1=0;

    double interval_cost=0;

    Eigen::MatrixXd decVarMatrix(problem.nStates+problem.nControls,problem.nCollocationPoints);
    Eigen::VectorXd controls(problem.nControls);
    Eigen::VectorXd states(problem.nStates);
    Eigen::VectorXd states_k1(problem.nStates);
    Eigen::VectorXd controls_k1(problem.nControls);

    utils::getDecVarMatrix(problem.nStates,problem.nControls,problem.nCollocationPoints,z,decVarMatrix,t0,tF);

    deltaT=tF-t0;

    grad.setZero(problem.NLP.nDecVar);
    problem.NLP.gradxk1.setZero(problem.nStates+problem.nControls);

    for(int k=0;k<problem.nCollocationPoints;k++){

        double cost_k=0;

        //Get the unscaled states and controls in k segment

        utils::getVariables(problem.nStates,problem.nControls,decVarMatrix,states,controls,k);

        //Get the unscaled states and controls in k+1 and save in the problem->state_k_1 variables

        if(k+1==problem.nCollocationPoints){ k_1=k; }
        else{ k_1=k+1; }

        utils::getVariables(problem.nStates,problem.nControls,decVarMatrix,states_k1,controls_k1,k_1);

        problem.NLP.xk1=states_k1;

        double ck=problem.NLP.weights(k); //Assign the quadrature weight

        cost_k=nocs::Function::integrand_cost(states,controls,tk,problem);

        //Evaluate the integral parte of the cost functon in k+1

        interval_cost+=problem.NLP.time_weights*ck*cost_k;

        //Now compute the value of the gradient

        Eigen::VectorXd gradient(problem.nStates+problem.nControls);

        problem.NLP.gradient_eval=1;
        nocs::Function::analytical::costGradient(states,controls,tk,gradient,problem);

        //if(k==0 || k_1==k){ problem.NLP.gradxk1.setZero();}


        grad.segment(k*(problem.nStates+problem.nControls)+2,problem.nStates+problem.nControls)=(problem.NLP.time_weights*deltaT*ck*gradient)+(problem.NLP.time_weights*deltaT*c_prev*problem.NLP.gradxk1);
        c_prev=ck;
        problem.NLP.gradient_eval=2;
        nocs::Function::analytical::costGradient(states,controls,tk,gradient,problem);

    }


    grad(0)=-interval_cost;
    grad(1)=interval_cost;


}//end computeGradientCost

}//End localGenerator namespace

}//End nocs namespace
