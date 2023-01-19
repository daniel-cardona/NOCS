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



#include "directCollocation/local/derivatives.hpp"

#include "directCollocation/local/utils.hpp"


namespace nocs {


void dxJacColumn(localCollocation &problem, Eigen::VectorXd &x,int jCol,Eigen::VectorXd &jacColumn){

    double delj;
    double sqreps;
    double xs;

    int nStates=problem.nStates;
    int nControls=problem.nControls;
    int nPath=problem.nPath;

    int nCns=nStates;

    Eigen::VectorXd F1(nCns);
    Eigen::VectorXd F2(nCns);
    Eigen::VectorXd F3(nCns);

    Eigen::VectorXd dfdx_j(nCns);

    Eigen::VectorXd states(nStates);
    Eigen::VectorXd controls(nControls);

    states=x.head(nStates);
    controls=x.tail(nControls);

    double t0=0.0;
    double tf=1.0;

    F1.setZero();
    F2.setZero();
    F3.setZero();

    jacColumn.setZero(nCns);

    sqreps=sqrt(MC_EPSILON);

    bool c1,c2;

    Eigen::VectorXd xlb(nStates+nControls);
    Eigen::VectorXd xub(nStates+nControls);

    xlb<<problem.bounds.states.lower,problem.bounds.controls.lower;
    xub<<problem.bounds.states.upper,problem.bounds.controls.lower;

    c1=(x.array()>(xlb.array()+sqreps)).any();
    c2=(x.array()<(xub.array()-sqreps)).any();

    if(c1||c2){

        problem.dynamics(states,controls,t0,F3,problem);
    }

    int j=jCol;

    delj=sqreps;//*(1+fabs(x(j)));


    xs=x(j);

    //Central difference formula
    if((xs<xub(j)-delj && xs>xlb(j)+delj) || (xub(j)==xlb(j))){

            x(j)+=delj;
            states=x.head(nStates);
            controls=x.tail(nControls);

            problem.dynamics(states,controls,t0,F1,problem);

            x(j)=xs-delj;
            states=x.head(nStates);
            controls=x.tail(nControls);

            problem.dynamics(states,controls,t0,F2,problem);

            dfdx_j=(F1-F2)/(2*delj);

            x(j)=xs;
    }

    //Backward difference formula (variable at upper bound)
    else if(xs>=xub(j)-delj)
    {
        x(j)=xs-delj;
        states=x.head(nStates);
        controls=x.tail(nControls);

        problem.dynamics(states,controls,t0,F1,problem);

        x(j)=xs;

        dfdx_j=(F1-F3)/(-delj);
    }

    //Forward difference formula (variable at lower bound)

    else if(xs<=xlb(j)+delj){

        x(j)=xs+delj;
        states=x.head(nStates);
        controls=x.tail(nControls);

        problem.dynamics(states,controls,t0,F1,problem);
        x(j)=xs;

        dfdx_j=(F1-F3)/delj;

    }

    jacColumn=dfdx_j;


}//End dxJacColumn

void pathJacColumn(localCollocation &problem, Eigen::VectorXd &x,int jCol,Eigen::VectorXd &jacColumn){

    double delj;
    double sqreps;
    double xs;

    int nStates=problem.nStates;
    int nControls=problem.nControls;
    int nPath=problem.nPath;

    int nCns=nPath;

    Eigen::VectorXd F1(nCns);
    Eigen::VectorXd F2(nCns);
    Eigen::VectorXd F3(nCns);

    Eigen::VectorXd dfdx_j(nCns);

    Eigen::VectorXd states(nStates);
    Eigen::VectorXd controls(nControls);

    states=x.head(nStates);
    controls=x.tail(nControls);

    double t0=0.0;
    double tf=1.0;

    F1.setZero();
    F2.setZero();
    F3.setZero();

    jacColumn.setZero(nCns);

    sqreps=sqrt(MC_EPSILON);

    bool c1,c2;

    Eigen::VectorXd xlb(nStates+nControls);
    Eigen::VectorXd xub(nStates+nControls);


    xlb<<problem.bounds.states.lower,problem.bounds.controls.lower;
    xub<<problem.bounds.states.upper,problem.bounds.controls.upper;

    c1=(x.array()>(xlb.array()+sqreps)).any();
    c2=(x.array()<(xub.array()-sqreps)).any();

    if(c1||c2){

        Function::path(states,controls,t0,F3,problem);
    }

    int j=jCol;

    delj=sqreps;//*(1+fabs(x(j)));


    xs=x(j);

    //Central difference formula
    if((xs<xub(j)-delj && xs>xlb(j)+delj) || (xub(j)==xlb(j))){

            x(j)+=delj;
            states=x.head(nStates);
            controls=x.tail(nControls);

            Function::path(states,controls,t0,F1,problem);

            x(j)=xs-delj;
            states=x.head(nStates);
            controls=x.tail(nControls);

            Function::path(states,controls,t0,F2,problem);

            dfdx_j=(F1-F2)/(2*delj);

            x(j)=xs;
    }

    //Backward difference formula (variable at upper bound)
    else if(xs>=xub(j)-delj)
    {
        x(j)=xs-delj;
        states=x.head(nStates);
        controls=x.tail(nControls);

        Function::path(states,controls,t0,F1,problem);

        x(j)=xs;

        dfdx_j=(F1-F3)/(-delj);
    }

    //Forward difference formula (variable at lower bound)

    else if(xs<=xlb(j)+delj){

        x(j)=xs+delj;
        states=x.head(nStates);
        controls=x.tail(nControls);

        Function::path(states,controls,t0,F1,problem);
        x(j)=xs;

        dfdx_j=(F1-F3)/delj;

    }

    jacColumn=dfdx_j;



}//End pathJacColumn

void eventJacColumn_t0(localCollocation &problem, Eigen::VectorXd &x,int jCol,Eigen::VectorXd &jacColumn){

    double delj;
    double sqreps;
    double xs;

    int nStates=problem.nStates;
    int nControls=problem.nControls;
    int nEvents=problem.nEvents;

    int nCns=nEvents;

    Eigen::VectorXd F1(nCns);
    Eigen::VectorXd F2(nCns);
    Eigen::VectorXd F3(nCns);

    Eigen::VectorXd dfdx_j(nCns);

    Eigen::VectorXd states(nStates);

    Eigen::VectorXd dummy_states_tF(nStates);
    double dummy_tf=0;

    double t0=x(0);

    states=x.tail(nStates);

    F1.setZero();
    F2.setZero();
    F3.setZero();

    jacColumn.setZero(nCns);

    sqreps=sqrt(MC_EPSILON);

    bool c1,c2;

    Eigen::VectorXd xlb(nStates+1);
    Eigen::VectorXd xub(nStates+1);

    xlb<<0,problem.bounds.states.lower;
    xub<<100,problem.bounds.states.upper;

    c1=(x.array()>(xlb.array()+sqreps)).any();
    c2=(x.array()<(xub.array()-sqreps)).any();

    if(c1||c2){

        Function::events(states,dummy_states_tF,t0,dummy_tf,F3,problem);
    }

    int j=jCol;

    delj=sqreps;//*(1+fabs(x(j)));


    xs=x(j);
    //Central difference formula
    if((xs<xub(j)-delj && xs>xlb(j)+delj) || (xub(j)==xlb(j))){

            x(j)+=delj;

            t0=x(0);
            states=x.tail(nStates);

            Function::events(states,dummy_states_tF,t0,dummy_tf,F1,problem);

            x(j)=xs-delj;

            t0=x(0);
            states=x.tail(nStates);

            Function::events(states,dummy_states_tF,t0,dummy_tf,F2,problem);

            dfdx_j=(F1-F2)/(2*delj);

            x(j)=xs;
    }

    //Backward difference formula (variable at upper bound)
    else if(xs>=xub(j)-delj)
    {
        x(j)=xs-delj;

        t0=x(0);
        states=x.tail(nStates);

        Function::events(states,dummy_states_tF,t0,dummy_tf,F1,problem);

        x(j)=xs;

        dfdx_j=(F1-F3)/(-delj);
    }

    //Forward difference formula (variable at lower bound)

    else if(xs<=xlb(j)+delj){

        x(j)=xs+delj;

        t0=x(0);
        states=x.tail(nStates);

        Function::events(states,dummy_states_tF,t0,dummy_tf,F1,problem);

        x(j)=xs;

        dfdx_j=(F1-F3)/delj;

    }

    jacColumn=dfdx_j;

}//Emd eventJacColumn_t0

void eventJacColumn_tf(localCollocation &problem, Eigen::VectorXd &x,int jCol,Eigen::VectorXd &jacColumn){

    double delj;
    double sqreps;
    double xs;

    int nStates=problem.nStates;
    int nControls=problem.nControls;
    int nEvents=problem.nEvents;

    int nCns=nEvents;

    Eigen::VectorXd F1(nCns);
    Eigen::VectorXd F2(nCns);
    Eigen::VectorXd F3(nCns);

    Eigen::VectorXd dfdx_j(nCns);

    Eigen::VectorXd states(nStates);

    Eigen::VectorXd dummy_states_t0(nStates);

    dummy_states_t0.setZero();

    double dummy_t0=0;

    double tf=x(0);

    states=x.tail(nStates);

    F1.setZero();
    F2.setZero();
    F3.setZero();

    jacColumn.setZero(nCns);

    sqreps=sqrt(MC_EPSILON);

    bool c1,c2;

    Eigen::VectorXd xlb(nStates+1);
    Eigen::VectorXd xub(nStates+1);

    xlb<<0,problem.bounds.states.lower;
    xub<<100,problem.bounds.states.upper;

    c1=(x.array()>(xlb.array()+sqreps)).any();
    c2=(x.array()<(xub.array()-sqreps)).any();

    if(c1||c2){

        Function::events(dummy_states_t0, states,dummy_t0,tf,F3,problem);
    }

    int j=jCol;

    delj=sqreps;//*(1+fabs(x(j)));


    xs=x(j);

    //Central difference formula
    if((xs<xub(j)-delj && xs>xlb(j)+delj) || (xub(j)==xlb(j))){

            x(j)+=delj;

            tf=x(0);
            states=x.tail(nStates);

            Function::events(dummy_states_t0, states,dummy_t0,tf,F1,problem);

            x(j)=xs-delj;

            tf=x(0);
            states=x.tail(nStates);

            Function::events(dummy_states_t0, states,dummy_t0,tf,F2,problem);

            dfdx_j=(F1-F2)/(2*delj);

            x(j)=xs;
    }

    //Backward difference formula (variable at upper bound)
    else if(xs>=xub(j)-delj)
    {
        x(j)=xs-delj;

        tf=x(0);
        states=x.tail(nStates);

        Function::events(dummy_states_t0, states,dummy_t0,tf,F1,problem);

        x(j)=xs;

        dfdx_j=(F1-F3)/(-delj);
    }

    //Forward difference formula (variable at lower bound)

    else if(xs<=xlb(j)+delj){

        x(j)=xs+delj;

        tf=x(0);
        states=x.tail(nStates);

        Function::events(dummy_states_t0, states,dummy_t0,tf,F1,problem);

        x(j)=xs;

        dfdx_j=(F1-F3)/delj;

    }

    jacColumn=dfdx_j;



}//End eventJacColumn_tF

}//End nocs namespace
