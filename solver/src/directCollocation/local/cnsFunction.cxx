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

void rightHandSideSparsity(localCollocation &problem, Eigen::VectorXd &z, Eigen::VectorXd &c){

    int nSegments=problem.nCollocationPoints-1;

    int offset=0;

    int pathOffset=problem.nStates*(nSegments+1)+problem.nEvents;

    //Defect constraint variables

    Eigen::VectorXd residk(problem.nStates);
    Eigen::VectorXd residk1(problem.nStates);

    double t0,tf;
    double hk=0;

    Eigen::MatrixXd decVarMatrix;

    Eigen::VectorXd controls(problem.nControls);
    Eigen::VectorXd states(problem.nStates);

    Eigen::VectorXd dx(problem.nStates);
    Eigen::VectorXd path(problem.nPath);

    //Main Vector of the separability
    Eigen::VectorXd q((problem.nStates+problem.nPath)*(problem.nCollocationPoints)+problem.nEvents+1);

    //Obtain the decision variable vector in matrix form

    utils::getDecVarMatrix(problem.nStates,problem.nControls,problem.nCollocationPoints,z,decVarMatrix,t0,tf);

    //Differential defects

    for(int k=0;k<nSegments+1;k++){

        //Obtain the states, controls and time in the k node and evaluate the dae

        utils::getVariables(problem.nStates,problem.nControls,decVarMatrix,states,controls,k);

        //Time variables

        hk=(tf-t0);
        double tk=0;

        problem.dynamics(states,controls,tk,dx,problem);

        residk=(hk*dx);
        offset=(k*problem.nStates);

        q.segment(offset,problem.nStates)<<residk;

        //---------------Path Constraints-----------------

        Function::path(states,controls,tk,path,problem);

        q.segment(pathOffset+k*problem.nPath,problem.nPath)=path;

    }
    //---------------Event constraints-----------------

    Eigen::VectorXd initial_states(problem.nStates);
    Eigen::VectorXd final_states(problem.nStates);

    Eigen::VectorXd e(problem.nEvents);

    utils::getVariables(problem.nStates,problem.nControls,decVarMatrix,initial_states,controls,0);
    utils::getVariables(problem.nStates,problem.nControls,decVarMatrix,final_states,controls,nSegments);


    //Obtain the value of the event constraints and scale it

    Function::events(initial_states,final_states,t0,tf,e,problem);

    q.segment(problem.nStates*(problem.nCollocationPoints),problem.nEvents)=e;


    //---Modification
    //q(q.size()-1)=(t0-tf);

    //Now compute the full constraints vector

    c.setZero(problem.NLP.nCns);

    c=(problem.NLP.sparsity.A*z) + (problem.NLP.sparsity.B*q);

}


//-------------------------------------------------------------------------
//------------- Derivatives functions
//-------------------------------------------------------------------------

void derivatives::computePropagatedJacobian(localCollocation &problem,Eigen::VectorXd &x,Eigen::VectorXd &nzValues){

    //Set offset of the constraints
    int eventOffset=problem.nCollocationPoints*problem.nStates;
    int pathOffset=eventOffset+problem.nEvents;

    int nStates=problem.nStates;
    int nControls=problem.nControls;
    int nPath=   problem.nPath;
    int nEvents= problem.nEvents;

    int nDecVar=problem.NLP.nDecVar;

    double t0,tf;

    Eigen::MatrixXd decVarMatrix;

    Eigen::VectorXd kStates(nStates);         //Vector with the states of the system in k
    Eigen::VectorXd kControls(nControls);       //Vector with the controls of the system in k
    Eigen::VectorXd dx(nStates);             //Vector with the derivatives of the states of the system in k

    Eigen::MatrixXd Dfk(nStates,nStates+nControls);
    Eigen::MatrixXd Dpk(nPath,nStates+nControls);
    Eigen::VectorXd Dt0(nEvents);
    Eigen::VectorXd DtF(nEvents);
    Eigen::MatrixXd De0(nEvents,nStates);
    Eigen::MatrixXd DeF(nEvents,nStates);

    std::vector<T> data;
    data.reserve(problem.NLP.sparsity.nnz_D);

    //Obtain the gradient of the time step

    utils::getDecVarMatrix(nStates,nControls,problem.nCollocationPoints,x,decVarMatrix,t0,tf);

    double hk=(tf-t0);

    //Set the constant templates (dynamics wrt t0 and tF)
    double value=0;
    int iRow,iCol;

    for (int k=0; k<problem.nCollocationPoints;k++){

        utils::getVariables(nStates,nControls,decVarMatrix,kStates,kControls,k);

        double tk=0;

        //Evaluate the dynamics and path constraints
        problem.dynamics(kStates,kControls,tk,dx,problem);

        //Evaluate the analytical differentiation functions to be propagated

        problem.fJacobian(kStates,kControls,tk,Dfk,problem);

        if(problem.nPath>0){
            problem.pathJacobian(kStates,kControls,tk,Dpk,problem);
        }//End if problem.nPath

        //nocs::Function::analytical::pathGradient(kStates,kControls,tk,Dpk,problem);

        //Set the values of the time derivatives

        int segment=k*problem.nStates;
        int i=0;

        for (int j=segment;j<segment+problem.nStates;j++){

            iRow=j;

            iCol=0;
            data.push_back(T(iRow,iCol,-1.0*dx(i)));

            iCol=1;
            data.push_back(T(iRow,iCol,1.0*dx(i)));

            i++;

        }


        //Set the sparsity of the Df block matrix (toDo: This can be vectorized)

        for(int nz=0;nz<problem.NLP.sparsity.nnz_Dfk;nz++){

            iRow=problem.NLP.sparsity.dxPattern(nz,0)+(k*nStates);
            iCol=problem.NLP.sparsity.dxPattern(nz,1)+2+(k*(nStates+nControls));

            //toDo: Change the data type of problem.NLP.sparsity.dxPattern to int (Actually declared as MatrixXd)
            int ik=problem.NLP.sparsity.dxPattern(nz,0);
            int jk=problem.NLP.sparsity.dxPattern(nz,1);

            value=Dfk(ik,jk);

            data.push_back(T(iRow,iCol,hk*value));

        } //End for nz=0

        //Set the sparsity of the Dp block matrix (toDo: This can be vectorized)

        for(int nz=0;nz<problem.NLP.sparsity.nnz_Dpk;nz++){

            iRow=problem.NLP.sparsity.pathPattern(nz,0)+(k*nPath)+pathOffset;
            iCol=problem.NLP.sparsity.pathPattern(nz,1)+2+(k*(nStates+nControls));

            //toDo: Change the data type of problem.NLP.sparsity.pathPattern to int (Actually declared as MatrixXd)
            int ik=problem.NLP.sparsity.pathPattern(nz,0);
            int jk=problem.NLP.sparsity.pathPattern(nz,1);

            value=Dpk(ik,jk);

            data.push_back(T(iRow,iCol,value));

        } //End for nz=0

    }//End for k=0


    Eigen::VectorXd initial_states(nStates);
    Eigen::VectorXd final_states(nStates);

    utils::getVariables(nStates,nControls,decVarMatrix,initial_states,kControls,0);
    utils::getVariables(nStates,nControls,decVarMatrix,final_states,kControls,problem.nCollocationPoints-1);

    problem.eventJacobian(initial_states,final_states,t0,tf,Dt0,DtF,De0,DeF,problem);

    //nocs::Function::analytical::eventGradient(initial_states,final_states,t0,tf,Dt0,DtF,De0,DeF,problem);

    //Insert Dt0
    for(int nz=0;nz<problem.NLP.sparsity.nnz_Dt0;nz++){

        iRow=problem.NLP.sparsity.eventPattern_t0(nz,0)+eventOffset;
        iCol=problem.NLP.sparsity.eventPattern_t0(nz,1);

        //toDo: Change the data type of problem.NLP.sparsity.eventPattern_t0 to int (Actually declared as MatrixXd)
        int ik=problem.NLP.sparsity.eventPattern_t0(nz,0);

        value=Dt0(ik);

        data.push_back(T(iRow,iCol,value));

    }//End for int nz=0

    //Insert DtF

    for(int nz=0;nz<problem.NLP.sparsity.nnz_DtF;nz++){

        iRow=problem.NLP.sparsity.eventPattern_tF(nz,0)+eventOffset;
        iCol=problem.NLP.sparsity.eventPattern_tF(nz,1);

        //toDo: Change the data type of problem.NLP.sparsity.eventPattern_tF to int (Actually declared as MatrixXd)
        int ik=problem.NLP.sparsity.eventPattern_tF(nz,0);

        value=DtF(ik);

        data.push_back(T(iRow,iCol,value));

    }//End for int nz=0

    //Set the sparsity of the De0

    for(int nz=0;nz<problem.NLP.sparsity.nnz_De0;nz++){

        iRow=problem.NLP.sparsity.eventPattern_e0(nz,0)+eventOffset;
        iCol=problem.NLP.sparsity.eventPattern_e0(nz,1)+2;

        int ik=problem.NLP.sparsity.eventPattern_e0(nz,0);
        int jk=problem.NLP.sparsity.eventPattern_e0(nz,1);

        value=De0(ik,jk);

        data.push_back(T(iRow,iCol,value));
    }

    //Set the sparsity of the DeF

    for(int nz=0;nz<problem.NLP.sparsity.nnz_DeF;nz++){

        iRow=problem.NLP.sparsity.eventPattern_eF(nz,0)+eventOffset;
        iCol=nDecVar-(nStates+nControls)+problem.NLP.sparsity.eventPattern_eF(nz,1);

        int ik=problem.NLP.sparsity.eventPattern_eF(nz,0);
        int jk=problem.NLP.sparsity.eventPattern_eF(nz,1);

        value=DeF(ik,jk);

        data.push_back(T(iRow,iCol,value));

    }

   //Set the sparsity of the time constraint


    //Modification
//   iRow=problem.nCollocationPoints*(nStates+nPath)+nEvents;
//   iCol=0;
//   data.push_back(T(iRow,iCol,1));
//   iCol=1;
//   data.push_back(T(iRow,iCol,-1));

    //Obtain the sparsity data of the derivative matrix


    Eigen::SparseMatrix<double> D(problem.nCollocationPoints*(nStates+nPath)+nEvents,nDecVar);

   //Eigen::SparseMatrix<double> D(problem.nCollocationPoints*(nStates+nPath)+nEvents+1,nDecVar);

   D.setFromTriplets(data.begin(),data.end());


   //Now compute the Jacobian of the constraints usin A+BD


   Eigen::SparseMatrix<double> J(problem.NLP.nCns,problem.NLP.nDecVar);

   J=problem.NLP.sparsity.A;
   J+= (problem.NLP.sparsity.B*D);

   int externalIterator=0;

   nzValues.setZero(problem.NLP.sparsity.nnz_J);

   for(int k=0;k<J.outerSize();++k)
       for(Eigen::SparseMatrix<double>::InnerIterator it(J,k); it; ++it){

           nzValues(externalIterator)=it.value();
           externalIterator++;
       }

}

//-------------------------------------------------------------------------
//-------------Deprecated functions (Here just for reference)
//-------------------------------------------------------------------------

void trapCnsFunction(localCollocation &problem, Eigen::VectorXd &z, Eigen::VectorXd &g){

    int nOrder=problem.nDiscretePoints-1;
    int nCns=problem.NLP.nCns;

    double resid=0.0;
    int offset=0;
    int path_offset=problem.nStates*(nOrder)+problem.nEvents;

    double t0,tf ;
    double tk, tk1;
    double hk=0;

    Eigen::VectorXd varphi_k(problem.nStates);

    Eigen::MatrixXd decVarMatrix(problem.nStates+problem.nControls,problem.nDiscretePoints);

    Eigen::VectorXd controls(problem.nControls);
    Eigen::VectorXd states(problem.nStates);

    Eigen::VectorXd dx(problem.nStates);
    Eigen::VectorXd path(problem.nPath);

    Eigen::VectorXd controls_next(problem.nControls);
    Eigen::VectorXd states_next(problem.nStates);

    Eigen::VectorXd dx_next(problem.nStates);
    Eigen::VectorXd path_next(problem.nPath);

    g.setZero(nCns);

    //Obtain the decision variables vector in matrix form

    utils::getDecVarMatrix(problem.nStates,problem.nControls,problem.nDiscretePoints,z,decVarMatrix,t0,tf);


    //Obtain the value of the decision vector

    for(int k=0;k<problem.nDiscretePoints;k++){

        //Obtain the variables in time k

        utils::getVariables(problem.nStates,problem.nControls,decVarMatrix,states,controls,k);

        //Time variables

        double tk=0;

        double tk1=0;

        hk=(tf-t0)/nOrder;


        //-----Collocation constraints

        //Obtain the dynamics and the path constraints in k
        problem.dynamics(states,controls,tk,dx,problem);

        if(k!=(nOrder)){

            //Obtain the controls, states, and time in k+1

            utils::getVariables(problem.nStates,problem.nControls,decVarMatrix,states_next,controls_next,k+1);

            //Obtain the dynamics and the path constraints in k

            problem.dynamics(states_next,controls_next,tk1,dx_next,problem);

            //Obtain the value of the collocation constraints

            varphi_k=states_next - states - hk*(dx+dx_next)*0.5;

            g.segment(k*problem.nStates,problem.nStates)=varphi_k;


        }//end if

        //-----Path constraints

        Function::path(states,controls,tk,path,problem);
        g.segment(path_offset+(k*problem.nPath),problem.nPath)=path;

    }//end for

    //-----Event constraints

    Eigen::VectorXd initial_states(problem.nStates);
    Eigen::VectorXd final_states(problem.nStates);

    Eigen::VectorXd e(problem.nEvents);

    utils::getVariables(problem.nStates,problem.nControls,decVarMatrix,initial_states,controls,0);
    utils::getVariables(problem.nStates,problem.nControls,decVarMatrix,final_states,controls,nOrder);

    Function::events(initial_states,final_states,t0,tf,e,problem);

    g.segment(nOrder*problem.nStates,problem.nEvents)=e;

    g(g.size()-1)=(t0-tf);



} //End trapCnsFunction

void hsCnsFunction(localCollocation &problem, Eigen::VectorXd &z, Eigen::VectorXd &g){

    int nOrder=problem.nDiscretePoints-1;
    int nCns=problem.NLP.nCns;

    int path_offset=problem.nStates*(nOrder)+problem.nEvents;

    double t0,tf ;
    double tk, tk1;
    double hk=0;

    Eigen::VectorXd varphi_k(problem.nStates*2); //Two constraints per segment

    Eigen::MatrixXd decVarMatrix(problem.nStates+problem.nControls,problem.nCollocationPoints);

    //k point vectors
    Eigen::VectorXd controls(problem.nControls);
    Eigen::VectorXd states(problem.nStates);
    Eigen::VectorXd dx(problem.nStates);
    Eigen::VectorXd path(problem.nPath);

    //Mid point vectors
    Eigen::VectorXd controls_mid(problem.nControls);
    Eigen::VectorXd states_mid(problem.nStates);
    Eigen::VectorXd dx_mid(problem.nStates);
    Eigen::VectorXd path_mid(problem.nPath);

    //k+1 point vectors
    Eigen::VectorXd controls_next(problem.nControls);
    Eigen::VectorXd states_next(problem.nStates);
    Eigen::VectorXd dx_next(problem.nStates);
    Eigen::VectorXd path_next(problem.nPath);

    g.setZero(nCns);

    //Obtain the decision variables vector in matrix form

    utils::getDecVarMatrix(problem.nStates,problem.nControls,problem.nCollocationPoints,z,decVarMatrix,t0,tf);

    //Obtain the value of the decision vector

    for(int k=0;k<problem.nDiscretePoints-1;k++){

        //Use the reference matrix to obtain the points needed for the k collocation contraints
        int k_i=problem.NLP.colPointsRef(k,0);
        int k_mid=problem.NLP.colPointsRef(k,1);
        int k_1=problem.NLP.colPointsRef(k,2);

        //Obtain the variables in time k ,k_mid and k+1
        utils::getVariables(problem.nStates,problem.nControls,decVarMatrix,states,controls,k_i);
        utils::getVariables(problem.nStates,problem.nControls,decVarMatrix,states_mid,controls_mid,k_mid);
        utils::getVariables(problem.nStates,problem.nControls,decVarMatrix,states_next,controls_next,k_1);

        //Time variables

        double tk=0;

        double tk1=0;

        hk=(tf-t0)/nOrder;

        //Obtain the dae values for the points


        //-----Collocation constraints

        //Obtain the dynamics and the path constraints in k, k_mid, and k+1
        problem.dynamics(states,controls,tk,dx,problem);
        problem.dynamics(states_mid,controls_mid,tk,dx_mid,problem);
        problem.dynamics(states_next,controls_next,tk1,dx_next,problem);


        //Obtain the value of the collocation constraints

        varphi_k.head(problem.nStates)=states_mid-0.5*(states_next+states)-(hk/8.0)*(dx-dx_next);
        varphi_k.tail(problem.nStates)=states_next-states-(hk/6.0)*(dx_next+(4*dx_mid)+dx);

        g.segment(k*problem.nStates*2,problem.nStates*2)=varphi_k;

        //-----Path constraints

        Function::path(states,controls,tk,path,problem);
        Function::path(states_mid,controls_mid,tk,path_mid,problem);
        Function::path(states_next,controls_next,tk1,path_next,problem);

        g.segment(path_offset+(k*problem.nPath*2),problem.nPath*3)<<path,path_mid,path_next;

    }//end for

    //-----Event constraints

    Eigen::VectorXd initial_states(problem.nStates);
    Eigen::VectorXd final_states(problem.nStates);

    Eigen::VectorXd e(problem.nEvents);

    utils::getVariables(problem.nStates,problem.nControls,decVarMatrix,initial_states,controls,0);
    utils::getVariables(problem.nStates,problem.nControls,decVarMatrix,final_states,controls,problem.nCollocationPoints-1);

    Function::events(initial_states,final_states,t0,tf,e,problem);

    g.segment(problem.NLP.nCollocationCns,problem.nEvents)=e;

    g(g.size()-1)=(t0-tf);


} //End hsCnsFunctionn

void derivatives::analytical::computeAnalyticalJacobian(localCollocation &problem,Eigen::VectorXd &x,Eigen::VectorXd &nzValues){

    //Obtain the gradient of the time step

    Eigen::VectorXd hkGrad(2);

    hkGrad(0)=-1.0;
    hkGrad(1)=1.0;

    double t0,tf;

    int nStates=problem.nStates;
    int nControls=problem.nControls;
    int nPath=   problem.nPath;
    int nEvents= problem.nEvents;

    int nCollocationPoints=problem.nCollocationPoints;

    int eventOffset=nCollocationPoints*nStates;
    int pathOffset=(nCollocationPoints*nStates)+nEvents;

    Eigen::VectorXd kStates(nStates);         //Vector with the states of the system in k
    Eigen::VectorXd kControls(nControls);       //Vector with the controls of the system in k
    Eigen::VectorXd dx(nStates);             //Vector with the derivatives of the states of the system in k

    Eigen::MatrixXd dxGradient(nStates,nStates+nControls);
    Eigen::MatrixXd pathGradient(nPath,nStates+nControls);
    Eigen::MatrixXd eventGradient_t0(nEvents,nStates+1);
    Eigen::MatrixXd eventGradient_tF(nEvents,nStates+1);

    Eigen::MatrixXd decVarMatrix(nStates+nControls,nCollocationPoints);

    Eigen::MatrixXd derivativeMatrix(nCollocationPoints*(nStates+nPath)+nEvents+1,problem.NLP.nDecVar);
    derivativeMatrix.setZero();

    utils::getDecVarMatrix(nStates,nControls,nCollocationPoints,x,decVarMatrix,t0,tf);

     double hk=(tf-t0);

    for(int k=0;k<problem.nCollocationPoints;k++){

        utils::getVariables(nStates,nControls,decVarMatrix,kStates,kControls,k);

        double tk=0;

        //Evaluate the dynamics and path constraints
        problem.dynamics(kStates,kControls,tk,dx,problem);

        //Evaluate the analytical differentiation functions given by the user

        problem.fJacobian(kStates,kControls,tk,dxGradient,problem);

        derivativeMatrix.col(0).segment(k*problem.nStates,problem.nStates)=hkGrad(0)*dx;        //Time grad wrt t0
        derivativeMatrix.col(1).segment(k*problem.nStates,problem.nStates)=hkGrad(1)*dx;        //Time grad wrt tf

        derivativeMatrix.block(k*(nStates),2+(k*(nStates+nControls)),nStates,(nStates+nControls))=hk*dxGradient;

        //------TEST------------

        if(nPath>0){

            nocs::Function::analytical::pathGradient(kStates,kControls,tk,pathGradient,problem);

            derivativeMatrix.block(pathOffset+(k*(nPath)),2+(k*(nStates+nControls)),nPath,(nStates+nControls))=pathGradient;


        }//End if


    }


    //------------------ GRADIENT OF THE EVENT CONSTRAINTS -----------

    Eigen::VectorXd initial_states(nStates);
    Eigen::VectorXd final_states(nStates);

    utils::getVariables(nStates,nControls,decVarMatrix,initial_states,kControls,0);
    utils::getVariables(nStates,nControls,decVarMatrix,final_states,kControls,problem.nCollocationPoints-1);

    //nocs::Function::analytical::eventGradient(initial_states,final_states,t0,tf,eventGradient_t0,eventGradient_tF,problem);

    derivativeMatrix.block(eventOffset,0,problem.nEvents,1)=eventGradient_t0.col(0);
    derivativeMatrix.block(eventOffset,1,problem.nEvents,1)=eventGradient_tF.col(0);

    derivativeMatrix.block(eventOffset,2,problem.nEvents,nStates)=eventGradient_t0.rightCols(nStates);
    derivativeMatrix.block(eventOffset,problem.NLP.nDecVar-(nStates+nControls),problem.nEvents,nStates)=eventGradient_tF.rightCols(nStates);

    derivativeMatrix(derivativeMatrix.rows()-1,0)=1;
    derivativeMatrix(derivativeMatrix.rows()-1,1)=-1;


    //--------------------Computation of the analytical sparse jacobian of the constraints------


    int iRow=0;
    int iCol=0;
    double value=0;

    vector<T> data;
    data.reserve(problem.NLP.sparsity.nnz_D);

    for(int i=0;i<problem.NLP.sparsity.nnz_D;i++){

        iRow=problem.NLP.sparsity.nzG_D(i,0);
        iCol=problem.NLP.sparsity.nzG_D(i,1);

        value=derivativeMatrix(iRow,iCol);

        data.push_back(T(iRow,iCol,value));

    }

    Eigen::SparseMatrix<double> D(problem.NLP.nCns,problem.NLP.nDecVar);
    Eigen::SparseMatrix<double> J(nCollocationPoints*(nStates+nPath)+nEvents+1,problem.NLP.nDecVar);

    J.setFromTriplets(data.begin(),data.end());
    D=problem.NLP.sparsity.A;
    D+= (problem.NLP.sparsity.B*J);

    int externalIterator=0;

    nzValues.setZero(problem.NLP.sparsity.nnz_J);

    for(int k=0;k<D.outerSize();++k)
        for(Eigen::SparseMatrix<double>::InnerIterator it(D,k); it; ++it){

            nzValues(externalIterator)=it.value();
            externalIterator++;
        }

}//End computeAnalyticalJacobian

void derivatives::numerical::obtainNonLinearVector(localCollocation &problem, Eigen::VectorXd &z, Eigen::VectorXd &q){

    int nSegments=problem.nCollocationPoints-1;

    int offset=0;

    int pathOffset=problem.nStates*(nSegments+1)+problem.nEvents;

    //Defect constraint variables

    Eigen::VectorXd residk(problem.nStates);

    double t0,tf;
    double tk=0;
    double hk=0;

    Eigen::MatrixXd decVarMatrix;

    Eigen::VectorXd controls(problem.nControls);
    Eigen::VectorXd states(problem.nStates);

    Eigen::VectorXd dx(problem.nStates);
    Eigen::VectorXd path(problem.nPath);

    Eigen::VectorXd controls_next(problem.nControls);
    Eigen::VectorXd states_next(problem.nStates);

    Eigen::VectorXd dx_next(problem.nStates);
    Eigen::VectorXd path_next(problem.nPath);

    //Main Vector of the separability
    q.setZero((problem.nStates+problem.nPath)*(problem.nCollocationPoints)+problem.nEvents+1);

    //Obtain the decision variable vector in matrix form

    utils::getDecVarMatrix(problem.nStates,problem.nControls,problem.nCollocationPoints,z,decVarMatrix,t0,tf);

    //Differential defects


    for(int k=0;k<nSegments+1;k++){

        //Obtain the states, controls and time in the k node and evaluate the dae

        utils::getVariables(problem.nStates,problem.nControls,decVarMatrix,states,controls,k);

        //Time variables
        //hk=(tf-t0)/nSegments;
        tk=hk*k;
        hk=(tf-t0);

        problem.dynamics(states,controls,tk,dx,problem);

        residk=(hk*dx);
        offset=(k*problem.nStates);

        q.segment(offset,problem.nStates)<<residk;

        //---------------Path Constraints-----------------

        Function::path(states,controls,tk,path,problem);

        q.segment(pathOffset+k*problem.nPath,problem.nPath)=path;

    }
    //---------------Event constraints-----------------

    Eigen::VectorXd initial_states(problem.nStates);
    Eigen::VectorXd final_states(problem.nStates);

    Eigen::VectorXd e(problem.nEvents);

    utils::getVariables(problem.nStates,problem.nControls,decVarMatrix,initial_states,controls,0);
    utils::getVariables(problem.nStates,problem.nControls,decVarMatrix,final_states,controls,nSegments);


    //Obtain the value of the event constraints and scale it

    Function::events(initial_states,final_states,t0,tf,e,problem);

    q.segment(problem.nStates*(problem.nCollocationPoints),problem.nEvents)=e;

    q(q.size()-1)=(t0-tf);

}

void derivatives::numerical::computeJacobian(localCollocation &problem,Eigen::VectorXd &x,Eigen::VectorXd &nzValues){

    /* This function uses the method of Curtis, Powell and Reid (1974) to
    * evaluate efficiently the sparse Jacobian by perturbing simultaneously groups of variables.
    * Reference:
    * A. R. Curtis, M.J.D. Powell and J.K. Reid
    * "On the estimation of Sparse Jacobian Matrices"
    * J Inst Maths Applics (1974) 13, 117-119
    *
    */

    double delj;
    double sqreps;

    int nDecVar=problem.NLP.nDecVar;
    int nCns=problem.nStates*(problem.nCollocationPoints)+problem.nEvents+(problem.nPath*problem.nCollocationPoints)+1;

    int iRow=0;
    int iCol=0;

    int element=0;

    Eigen::SparseMatrix<double, Eigen::RowMajor> J(nCns,nDecVar);
    vector<T> data;

    //Variables of groups

    int nGroups=problem.NLP.sparsity.sizeGroupsD.size();

    Eigen::VectorXd F1(nCns);
    Eigen::VectorXd F2(nCns);

    Eigen::VectorXd xp(nDecVar);

    double valueNZ=0;

    nzValues.setZero(problem.NLP.sparsity.nnz_J);

    sqreps=sqrt(MC_EPSILON);

    delj=sqreps;

    for(int i=0;i<nGroups;i++){

        xp=x;

        //Iterate along the non-zero elements that doesn't generate zeros in the jacobian

        xp+=problem.NLP.sparsity.positivePerturbationMatrix.col(i);

        derivatives::numerical::obtainNonLinearVector(problem,xp,F1);

        //Perturbate again the decisión vector

        xp-=problem.NLP.sparsity.negativePerturbationMatrix.col(i);

        derivatives::numerical::obtainNonLinearVector(problem,xp,F2);

        //Iterate to set the jacobian just in the non-zero elements

        for(int j=0;j<problem.NLP.sparsity.sizeGroupsD(i);j++){


                for(int k=0;k<problem.NLP.sparsity.nnz_D;k++){

                        iRow=problem.NLP.sparsity.nzG_D(k,0); //Row of the k th non zero element
                        iCol=problem.NLP.sparsity.nzG_D(k,1); //Column of the k th non zero element


                        if(iCol==problem.NLP.sparsity.idxGroupsD(i,j)){

                                valueNZ=(F1(iRow)-F2(iRow))/(2*delj);

                                data.push_back(T(iRow,iCol,valueNZ));

                        }
                }

        }

}

J.setFromTriplets(data.begin(), data.end());

Eigen::SparseMatrix<double> D(problem.NLP.nCns,nDecVar);

D=problem.NLP.sparsity.A;
D+= (problem.NLP.sparsity.B*J);

int externalIterator=0;

for(int k=0;k<D.outerSize();++k)
    for(Eigen::SparseMatrix<double>::InnerIterator it(D,k); it; ++it){

        nzValues(externalIterator)=it.value();
        externalIterator++;
    }


}


}//End trapezoidal namespace

}//End nocs namespace
