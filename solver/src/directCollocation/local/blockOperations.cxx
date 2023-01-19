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



#include "directCollocation/local/blockOperations.hpp"

#include "directCollocation/local/derivatives.hpp"


namespace nocs {


void localCollocation::computeBlockPatterns(){

    detectSparsity_dx(*this,NLP.sparsity.dxPattern);
    detectSparsity_path(*this,NLP.sparsity.pathPattern);
    detectSparsity_events(*this,NLP.sparsity.eventPattern_t0,NLP.sparsity.eventPattern_e0,
                                 NLP.sparsity.eventPattern_tF,NLP.sparsity.eventPattern_eF);
}

void detectSparsity_dx(localCollocation &problem, Eigen::MatrixXd &sparsityTemplate){

    int nStates=problem.nStates;
    int nControls=problem.nControls;
    int nDecVar=nStates+nControls;
    int nCns=nStates;

    int nzcount_G=0; //Non-zero non-constant elements (counter)

    double s=1.0e3*sqrt(MC_EPSILON);

    Eigen::VectorXd x(nDecVar);
    Eigen::VectorXd xp(nDecVar);

    x.setRandom();

    double tol= 1.e-16*pow(MC_EPSILON,0.8)*MAX(1.0,x.norm());

    Eigen::VectorXd jacCol1(nCns);
    Eigen::VectorXd jacCol2(nCns);
    Eigen::VectorXd jacCol3(nCns);

    Eigen::VectorXd s_v(nDecVar);

    Eigen::MatrixXd Gidx(nCns*nDecVar,2);

    Eigen::MatrixXd jacobian(nCns,nDecVar);

    s_v.setOnes();

    s_v*=s;

    for(int j=0;j<nDecVar;j++){

        xp=x;

        dxJacColumn(problem,xp,j,jacCol1);

        xp=x.array()+(0.1*x.array().abs())+s_v.array();

        dxJacColumn(problem,xp,j,jacCol2);

        xp=x.array()-(0.15*x.array().abs())-(1.1*s_v.array());

        dxJacColumn(problem,xp,j,jacCol3);

        for(int i=0;i<nCns;i++){

            if( (fabs(jacCol1(i)) +  fabs(jacCol2(i)) + fabs(jacCol3(i)) ) >= tol){

                    Gidx(nzcount_G,0)=i;//Row
                    Gidx(nzcount_G,1)=j;//Column
                    nzcount_G++;

            }//End if

        }//End for int i=0

    }//End for int j=0

    sparsityTemplate.setZero(nzcount_G,2);
    sparsityTemplate=Gidx.topRows(nzcount_G);
    problem.NLP.sparsity.nnz_Dfk=nzcount_G;

}//End detectSparsity_dx

void detectSparsity_path(localCollocation &problem, Eigen::MatrixXd &sparsityTemplate){

    int nStates=problem.nStates;
    int nControls=problem.nControls;
    int nDecVar=nStates+nControls;
    int nCns=problem.nPath;

    int nzcount_G=0; //Non-zero non-constant elements (counter)

    double s=1.0e3*sqrt(MC_EPSILON);

    Eigen::VectorXd x(nDecVar);
    Eigen::VectorXd xp(nDecVar);
    x.setRandom();

    double tol= 1.e-16*pow(MC_EPSILON,0.8)*MAX(1.0,x.norm());

    Eigen::VectorXd jacCol1(nCns);
    Eigen::VectorXd jacCol2(nCns);
    Eigen::VectorXd jacCol3(nCns);

    Eigen::VectorXd s_v(nDecVar);
    Eigen::MatrixXd Gidx(nCns*nDecVar,2);
    Eigen::MatrixXd jacobian(nCns,nDecVar);

    s_v.setOnes();
    s_v*=s;

    Eigen::VectorXd xlb(nStates+nControls);
    Eigen::VectorXd xub(nStates+nControls);

    xlb<<problem.bounds.states.lower,problem.bounds.controls.lower;
    xub<<problem.bounds.states.upper,problem.bounds.controls.lower;

    for(int j=0;j<nDecVar;j++){

        xp=x;

        pathJacColumn(problem,xp,j,jacCol1);

        xp=x.array()+(0.1*x.array().abs())+s_v.array();

        pathJacColumn(problem,xp,j,jacCol2);

        xp=x.array()-(0.15*x.array().abs())-(1.1*s_v.array());

        pathJacColumn(problem,xp,j,jacCol3);

        for(int i=0;i<nCns;i++){

            if( (fabs(jacCol1(i)) +  fabs(jacCol2(i)) + fabs(jacCol3(i)) ) >= tol){

                    Gidx(nzcount_G,0)=i;//Row
                    Gidx(nzcount_G,1)=j;//Column
                    nzcount_G++;

            }//End if

        } //End for i=0

    } //End for j=0

    sparsityTemplate.setZero(nzcount_G,2);
    sparsityTemplate=Gidx.topRows(nzcount_G);

    problem.NLP.sparsity.nnz_Dpk=nzcount_G;

}//End detectSparsity_path

void detectSparsity_events(localCollocation &problem,Eigen::MatrixXd &template_t0,Eigen::MatrixXd &template_e0,Eigen::MatrixXd &template_tF ,Eigen::MatrixXd &template_eF){

    int nStates=problem.nStates;
    int nDecVar=nStates+1;
    int nEvents=problem.nEvents;
    int nCns=nEvents;

    int nzcount_G=0; //Non-zero non-constant elements (counter)
    int nzcount_t=0; //Non-zero non-constant elements (counter of non-zero elements in time variables)

    double s=1.0e3*sqrt(MC_EPSILON);

    Eigen::VectorXd xlb(nStates+1);
    Eigen::VectorXd xub(nStates+1);

    xlb<<0,problem.bounds.states.lower;
    xub<<100,problem.bounds.states.upper;

    Eigen::VectorXd x(nDecVar);
    Eigen::VectorXd xp(nDecVar);

    x.setRandom();

    double tol= 1.e-16*pow(MC_EPSILON,0.8)*MAX(1.0,x.norm());

    Eigen::VectorXd jacCol1(nCns);
    Eigen::VectorXd jacCol2(nCns);
    Eigen::VectorXd jacCol3(nCns);

    Eigen::VectorXd s_v(nDecVar);
    Eigen::MatrixXd Gidx(nCns*nDecVar,2);
    Eigen::MatrixXd Gidx_t(nCns*2,2);

    Eigen::MatrixXd jacobian(nCns,nDecVar);

    //%%%%%%%%%%%%%%%%%%%%%%%%%% EVENTS w.r.t. t0 and e0 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    s_v.setOnes();
    s_v*=s;

    for(int j=0;j<nDecVar;j++){

        xp=x;

        eventJacColumn_t0(problem,xp,j,jacCol1);

        xp=x.array()+(0.1*x.array().abs())+s_v.array();

        eventJacColumn_t0(problem,xp,j,jacCol2);

        xp=x.array()-(0.15*x.array().abs())-(1.1*s_v.array());

        eventJacColumn_t0(problem,xp,j,jacCol3);

        for(int i=0;i<nCns;i++){

            if( (fabs(jacCol1(i)) +  fabs(jacCol2(i)) + fabs(jacCol3(i)) ) >= tol){


                    if(j==0){ //If the sparsity is on the firs column then is a non-zero element in Dt0

                        Gidx_t(nzcount_t,0)=i;
                        Gidx_t(nzcount_t,1)=j;
                        nzcount_t++;

                    }

                    else{

                        Gidx(nzcount_G,0)=i;
                        Gidx(nzcount_G,1)=j-1;//Repair the sparsity pattern deleting the tF column offset

                        nzcount_G++;
                    } //End if-else

            } //End if

        } //End for i=0

    } //End for j=0

    //Save the templates of the matrices

    template_t0.setZero(nzcount_t,2);
    template_t0=Gidx_t.topRows(nzcount_t);

    template_e0.setZero(nzcount_G,2);
    template_e0=Gidx.topRows(nzcount_G);

    problem.NLP.sparsity.nnz_Dt0=nzcount_t;
    problem.NLP.sparsity.nnz_De0=nzcount_G;

    //%%%%%%%%%%%%%%%%%%%%%%%%%% EVENTS w.r.t. tF %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    nzcount_G=0; //Non-zero non-constant elements (counter)
    nzcount_t=0; //Non-zero non-constant elements (counter of non-zero elements in time variables)

    s_v.setOnes();
    s_v*=s;

    for(int j=0;j<nDecVar;j++){

        xp=x;

        eventJacColumn_tf(problem,xp,j,jacCol1);

        xp=x.array()+(0.1*x.array().abs())+s_v.array();

        eventJacColumn_tf(problem,xp,j,jacCol2);

        xp=x.array()-(0.15*x.array().abs())-(1.1*s_v.array());

        eventJacColumn_tf(problem,xp,j,jacCol3);

        for(int i=0;i<nCns;i++){

            if( (fabs(jacCol1(i)) +  fabs(jacCol2(i)) + fabs(jacCol3(i)) ) >= tol){

                if(j==0){ //If the sparsity is on the second column then is a non-zero elelemnt in DtF

                    Gidx_t(nzcount_t,0)=i;
                    Gidx_t(nzcount_t,1)=j;
                    nzcount_t++;

                } else {

                    Gidx(nzcount_G,0)=i;
                    Gidx(nzcount_G,1)=j-1;//Repair the sparsity pattern deleting the tF column offset
                    nzcount_G++;

                }//End if-else

            } //End if

        } //End for i=0

    } //End for j=0


    //Save the templates of the matrices

    template_tF.setZero(nzcount_t,2);
    template_tF=Gidx_t.topRows(nzcount_t);

    template_eF.setZero(nzcount_G,2);
    template_eF=Gidx.topRows(nzcount_G);

    problem.NLP.sparsity.nnz_DtF=nzcount_t;
    problem.NLP.sparsity.nnz_DeF=nzcount_G;

}//End detectSparsity_events

void computeNumericalJacobian_dx(const Eigen::VectorXd &states, const Eigen::VectorXd &controls, double &t,Eigen::MatrixXd &gradient, localCollocation &problem){

    //The gradient of the dynamics should be return using the sparsity pattern:

    //grad(f (x, u))=[ df | df ]
    //               [ -- | -- ]
    //               [ dx | du ]

    //dim(grad)=[nStates,nStates+nControls]


    Eigen::VectorXd zk(problem.nStates+problem.nControls);
    Eigen::VectorXd jacColumn(problem.nStates);

    zk<<states,controls;

    for(int i=0; i<problem.nStates+problem.nControls; i++){

      nocs::dxJacColumn(problem,zk,i,jacColumn);

    gradient.col(i)=jacColumn;

    }//End for


}//End computeNumericalJacobian_dx

void computeNumericalJacobian_path(Eigen::VectorXd &states, Eigen::VectorXd &controls, double &t,Eigen::MatrixXd &gradient, localCollocation &problem){

    //The gradient of the path constraints should be return using the sparsity pattern:

    //grad(g (x, u))=[ dg | dg ]
    //               [ -- | -- ]
    //               [ dx | du ]

    //dim(grad)=[nPath,nStates+nControls]

    Eigen::VectorXd zk(problem.nStates+problem.nControls);
    Eigen::VectorXd jacColumn(problem.nPath);

    zk<<states,controls;


    for(int i=0; i<problem.nStates+problem.nControls; i++){

        nocs::pathJacColumn(problem,zk,i,jacColumn);

        gradient.col(i)=jacColumn;

    }


}//End computeNumericalJacobian_path

void computeNumericalJacobian_event(Eigen::VectorXd &x0,Eigen::VectorXd &xN,double &t0,double &tF,Eigen::VectorXd &Dt0, Eigen::VectorXd &DtF,Eigen::MatrixXd &De0, Eigen::MatrixXd &DeF, localCollocation &problem){

    //The gradient of the event constraints should be return using the sparsity pattern:

    //Dt0=[ de ]
    //    [ -- ]  dim=(nEvents,1)
    //    [ dt0]

    //DtF=[ de ]
    //    [ -- ]  dim=(nEvents,1)
    //    [ dtF]

    //De0=[ de ]
    //    [ -- ]  dim=(nEvents,nStates)
    //    [ dx0]

    //DeF=[ de ]
    //    [ -- ]  dim=(nEvents,nStates)
    //    [ dxN]

    Eigen::VectorXd zk_0(problem.nStates+1);
    Eigen::VectorXd zk_N(problem.nStates+1);

    Eigen::VectorXd jacColumn_t0(problem.nEvents);
    Eigen::VectorXd jacColumn_tF(problem.nEvents);

    zk_0<<t0,x0;
    zk_N<<tF,xN;

    nocs::eventJacColumn_t0(problem,zk_0,0,Dt0);
    nocs::eventJacColumn_tf(problem,zk_N,0,DtF);


    for(int i=0;i<problem.nStates;i++){

       nocs::eventJacColumn_t0(problem,zk_0,i+1,jacColumn_t0);
       nocs::eventJacColumn_tf(problem,zk_N,i+1,jacColumn_tF);

       De0.col(i)=jacColumn_t0;
       DeF.col(i)=jacColumn_tF;

    } //end for

}//End computeNumericalJacobian_events

}//End nocs namespace
