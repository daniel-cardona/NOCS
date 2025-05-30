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
#include <chrono>

using namespace std::chrono;

namespace nocs {

void localCollocation::detectProblemSparsity(){

    //1. Create the constant matrices A and B

    //this->setConstantMatrices();

    this->setConstantAB();

    //2. Detect the sparsity of the blocks of the jacobian (Only at the first mesh iteration)

    if(mesh.nIter==1){
        this->computeBlockPatterns();
    }//End if

    //3. Detect the FULL sparsity of the problem

     this->computeNumericalSparsity();

}//End localCollocation::detectProblemSparsity()

void localCollocation::setConstantAB(){

    //Variables

    int ny=nStates;
    int nu=nControls;
    int nDecVar=NLP.nDecVar;                      //Number of decision variables
    int M=nCollocationPoints;
    int nP=nPath*nCollocationPoints;              //Number of path constraints
    int nDefects=NLP.nCollocationCns;             //Number of defect constraints
    int nCns=NLP.nCns;                            //Number of constraints

    //==================Trapezoidal method matrices ===================

    if(colMethod=="Trapezoidal"){

        //-------------------------------Matrix A----------------------------

        int nzElements=(2*ny)*(nDiscretePoints-1);

        int i=0;
        int j=0;

        Eigen::MatrixXd pattern;
        pattern.setZero(nzElements,3);

        std::vector<T> data;
        data.reserve(nzElements);

        int iRow=0;
        int iCol=0;
        double value=0;

        for(int k=0;k<nDiscretePoints-1;k++){

            i=k*ny;
            j=k*(ny+nu);

            int iterator=0;

            for(int nz=0;nz<ny;nz++){

                iRow=i+iterator;
                iCol=j+2+iterator;
                value=-1;
                data.push_back(T(iRow,iCol,value));
                iterator++;
            }

            iterator=0;

            for(int nz=ny;nz<2*ny;nz++){

                iRow=i+iterator;
                iCol=j+2+ny+nu+iterator;
                value=1;
                data.push_back(T(iRow,iCol,value));
                iterator++;
            }


        } //End for

        NLP.sparsity.A.resize(nCns,nDecVar);
        NLP.sparsity.A.setFromTriplets(data.begin(),data.end());

        //-------------------------------Matrix B----------------------------

        //Sparsity information of the matrix B
        int nzElementsDynamics=(2*ny)*(nDiscretePoints-1);
        nzElements=nzElementsDynamics+nEvents+nP+1;

        //Index variables
        i=0;
        j=0;

        //Triplet variables
        iRow=0;
        iCol=0;
        value=0;

        //Iterator variable
        int iterator=0;

        std::vector<T> dataB;
        dataB.reserve(nzElements);

        for(int k=0;k<nDiscretePoints-1;k++){

            i=k*ny;
            j=k*ny;
            iterator=0;

            value=-0.5*mesh.deltaTau(k);

            for(int nz=0;nz<ny;nz++){

                iRow=i+iterator;
                iCol=j+iterator;
                dataB.push_back(T(iRow,iCol,value));
                iterator++;

            }//End for nz=0

            iterator=0;

            for(int nz=ny;nz<2*ny;nz++){

                iRow=i+iterator;
                iCol=j+ny+iterator;
                dataB.push_back(T(iRow,iCol,value));
                iterator++;

            }//End for nz=ny

         }//End for k

        //toDO: How to put the diagonal of the other constraints!!

        i=nDefects;
        j=ny*nCollocationPoints;
        iterator=0;

        value=1;

        for(int nz=nzElementsDynamics; nz<nzElements;nz++){

            iRow=i+iterator;
            iCol=j+iterator;
            dataB.push_back(T(iRow,iCol,value));
            iterator++;

        }

        NLP.sparsity.B.resize(NLP.nCollocationCns+nP+nEvents+1,(nStates*nCollocationPoints)+nP+nEvents+1);
        NLP.sparsity.B.setFromTriplets(dataB.begin(),dataB.end());


    } else if(colMethod=="Hermite-Simpson"){

        //==================Hermite-Simpson method matrices ===================

       //-------------------------------Matrix A----------------------------

        int nzElements=(5*ny)*(nDiscretePoints-1);

        int i=0;
        int j=0;

        Eigen::MatrixXd pattern;
        pattern.setZero(nzElements,3);

        std::vector<T> data;
        data.reserve(nzElements);

        int iRow=0;
        int iCol=0;
        double value=0;

        for(int k=0;k<nDiscretePoints-1;k++){

            i=2*k*ny;
            j=k*(2*(ny+nu));

            //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            int iterator=0;
            value=-0.5;
            for(int nz=0;nz<ny;nz++){

                iRow=i+iterator;
                iCol=j+2+iterator;
                data.push_back(T(iRow,iCol,value));
                iterator++;
            }

            //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            iterator=0;
             value=1;
            for(int nz=ny;nz<2*ny;nz++){

                iRow=i+iterator;
                iCol=j+2+ny+nu+iterator;
                data.push_back(T(iRow,iCol,value));
                iterator++;
            }

            //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            iterator=0;
            value=-0.5;
            for(int nz=2*ny;nz<3*ny;nz++){

                iRow=i+iterator;
                iCol=j+2+(2*ny)+(2*nu)+iterator;
                data.push_back(T(iRow,iCol,value));
                iterator++;
            }

            //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            iterator=0;
            value=-1;
            for(int nz=3*ny;nz<4*ny;nz++){

                iRow=i+ny+iterator;
                iCol=j+2+iterator;
                data.push_back(T(iRow,iCol,value));
                iterator++;
            }

            //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            iterator=0;
            value=1;
            for(int nz=4*ny;nz<5*ny;nz++){

                iRow=i+ny+iterator;
                iCol=j+2+(2*ny)+(2*nu)+iterator;
                data.push_back(T(iRow,iCol,value));
                iterator++;
            }



        } //End for k

        NLP.sparsity.A.resize(nCns,nDecVar);
        NLP.sparsity.A.setFromTriplets(data.begin(),data.end());

        //-------------------------------Matrix B----------------------------

        //Sparsity information of the matrix B
        int nzElementsDynamics=(5*ny)*(nDiscretePoints-1);



        //nzElements=nzElementsDynamics+nEvents+nP+1;
        //Modification -----
        nzElements=nzElementsDynamics+nEvents+nP;


        //Index variables
        i=0;
        j=0;

        //Triplet variables
        iRow=0;
        iCol=0;
        value=0;

        //Iterator variable
        int iterator=0;

        std::vector<T> dataB;
        dataB.reserve(nzElements);

        for(int k=0;k<nDiscretePoints-1;k++){

            i=2*k*ny;
            j=2*k*ny;

            iterator=0;
            value=-(1/8.0)*mesh.deltaTau(k);

            for(int nz=0;nz<ny;nz++){

                iRow=i+iterator;
                iCol=j+iterator;
                dataB.push_back(T(iRow,iCol,value));
                iterator++;

            }//End for nz=0

            //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            iterator=0;
            value=(1/8.0)*mesh.deltaTau(k);

            for(int nz=ny;nz<2*ny;nz++){

                iRow=i+iterator;
                iCol=j+2*ny+iterator;
                dataB.push_back(T(iRow,iCol,value));
                iterator++;

            }//End for nz=ny

            //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            iterator=0;
            value=-(1/6.0)*mesh.deltaTau(k);

            for(int nz=2*ny;nz<3*ny;nz++){

                iRow=i+ny+iterator;
                iCol=j+iterator;
                dataB.push_back(T(iRow,iCol,value));
                iterator++;

            }//End for nz=2*ny

            //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            iterator=0;
            value=-(2.0/3.0)*mesh.deltaTau(k);

            for(int nz=3*ny;nz<4*ny;nz++){

                iRow=i+ny+iterator;
                iCol=j+ny+iterator;
                dataB.push_back(T(iRow,iCol,value));
                iterator++;

            }//End for nz=3*ny

            //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            iterator=0;
            value=-(1/6.0)*mesh.deltaTau(k);

            for(int nz=4*ny;nz<5*ny;nz++){

                iRow=i+ny+iterator;
                iCol=j+2*ny+iterator;
                dataB.push_back(T(iRow,iCol,value));
                iterator++;

            }//End for nz=4*ny


         }//End for k

        //toDO: How to put the diagonal of the other constraints!!

        i=nDefects;
        j=ny*nCollocationPoints;
        iterator=0;

        value=1;

        for(int nz=nzElementsDynamics; nz<nzElements;nz++){

            iRow=i+iterator;
            iCol=j+iterator;
            dataB.push_back(T(iRow,iCol,value));
            iterator++;

        }


        NLP.sparsity.B.resize(nDefects+nP+nEvents,(ny*nCollocationPoints)+nP+nEvents);
        //NLP.sparsity.B.resize(nDefects+nP+nEvents+1,(ny*nCollocationPoints)+nP+nEvents+1);
        NLP.sparsity.B.setFromTriplets(dataB.begin(),dataB.end());

    }//End else if




}//End localCollocation::setConstantAB

void localCollocation::computeNumericalSparsity(){

    //Set offset of the constraints
    int eventOffset=nCollocationPoints*nStates;
    int pathOffset=eventOffset+nEvents;

    //Create the derivativeMatrix
    int nDecVar=NLP.nDecVar;

    int nnzD=0;

    //Compute the number of non-zero elements in the D matrix
    nnzD=(NLP.sparsity.nnz_Dfk +NLP.sparsity.nnz_Dpk +(2*nStates))*nCollocationPoints + NLP.sparsity.nnz_Dt0 +NLP.sparsity.nnz_DtF +NLP.sparsity.nnz_De0+NLP.sparsity.nnz_DeF+2;

    std::vector<T> data;
    data.reserve(nnzD);

    //Set the constant templates (dynamics wrt t0 and tF)
    int value=1;
    int iRow,iCol;

    for (int k=0;k<nStates*nCollocationPoints;k++){

        iRow=k;

        iCol=0;
        data.push_back(T(iRow,iCol,value));

        iCol=1;
        data.push_back(T(iRow,iCol,value));

    }
    for (int k=0; k<nCollocationPoints;k++){

        //Set the sparsity of the Df block matrix (toDo: This can be vectorized)

        for(int nz=0;nz<NLP.sparsity.nnz_Dfk;nz++){

            iRow=NLP.sparsity.dxPattern(nz,0)+(k*nStates);
            iCol=NLP.sparsity.dxPattern(nz,1)+2+(k*(nStates+nControls));

            data.push_back(T(iRow,iCol,value));

        } //End for nz=0

        //Set the sparsity of the Dp block matrix (toDo: This can be vectorized)

        for(int nz=0;nz<NLP.sparsity.nnz_Dpk;nz++){

            iRow=NLP.sparsity.pathPattern(nz,0)+(k*nPath)+pathOffset;
            iCol=NLP.sparsity.pathPattern(nz,1)+2+(k*(nStates+nControls));

            data.push_back(T(iRow,iCol,value));

        } //End for nz=0

    }//End ofr k=0

    //Set the sparsity of the Dt0

    for(int nz=0;nz<NLP.sparsity.nnz_Dt0;nz++){

        iRow=NLP.sparsity.eventPattern_t0(nz,0)+eventOffset;
        iCol=NLP.sparsity.eventPattern_t0(nz,1);
        data.push_back(T(iRow,iCol,value));

    }//End for int nz=0

    //and DtF block

    for(int nz=0;nz<NLP.sparsity.nnz_DtF;nz++){

        iRow=NLP.sparsity.eventPattern_tF(nz,0)+eventOffset;
        iCol=NLP.sparsity.eventPattern_tF(nz,1);
        data.push_back(T(iRow,iCol,value));

    }//End for int nz=0

    //Set the sparsity of the De0

    for(int nz=0;nz<NLP.sparsity.nnz_De0;nz++){

        iRow=NLP.sparsity.eventPattern_e0(nz,0)+eventOffset;
        iCol=NLP.sparsity.eventPattern_e0(nz,1)+2;
        data.push_back(T(iRow,iCol,value));
    }

    //Set the sparsity of the DeF

    for(int nz=0;nz<NLP.sparsity.nnz_DeF;nz++){

        iRow=NLP.sparsity.eventPattern_eF(nz,0)+eventOffset;
        iCol=nDecVar-(nStates+nControls)+NLP.sparsity.eventPattern_eF(nz,1);
        data.push_back(T(iRow,iCol,value));

    }

   //Set the sparsity of the time constraint


    //Modifation
//   iRow=nCollocationPoints*(nStates+nPath)+nEvents;
//   iCol=0;
//   data.push_back(T(iRow,iCol,value));
//   iCol=1;
//   data.push_back(T(iRow,iCol,value));

    //Obtain the sparsity data of the derivative matrix



   //Eigen::SparseMatrix<double> D(nCollocationPoints*(nStates+nPath)+nEvents+1,nDecVar);

   //MOdification
   Eigen::SparseMatrix<double> D(nCollocationPoints*(nStates+nPath)+nEvents,nDecVar);

   D.setFromTriplets(data.begin(),data.end());

   //Now obtain the sparsity in such a way that can be used for nocs

   //(toDo: Maybe this next lines are unnecesary, can be optimized)

   NLP.sparsity.nzG_D.setZero(nnzD,2);
   NLP.sparsity.nnz_D=D.nonZeros();

   int counter=0;



   for (int k=0; k<D.outerSize(); ++k)
         for (Eigen::SparseMatrix<double>::InnerIterator it(D,k); it; ++it)
         {
           NLP.sparsity.nzG_D(counter,0)=it.row();   // row index
           NLP.sparsity.nzG_D(counter,1)=it.col();   // col index (here it is equal to k)
           counter++;

         }

   //--------------------------------------------------------------------------------------------

    Eigen::SparseMatrix<double> J(NLP.nCns,NLP.nDecVar);

    //Compute the Jacobian sparsity
    J=this->NLP.sparsity.A + (this->NLP.sparsity.B*D);

    NLP.sparsity.nnz_J=J.nonZeros();
    NLP.sparsity.nzG_J.setZero(NLP.sparsity.nnz_J,2);

    counter=0;
    for (int k=0; k<J.outerSize(); ++k)
        for (Eigen::SparseMatrix<double>::InnerIterator it(J,k); it; ++it)
            {
            NLP.sparsity.nzG_J(counter,0)=it.row();   // row index
            NLP.sparsity.nzG_J(counter,1)=it.col();   // col index (here it is equal to k)
            counter++;
            }

}//End localCollocation::computeNumericalSparsity


//%%%%%%%%%%%%%%%%%%%%%%%% DEPRECATED FUNCTIONS (HERE FOR REFERENCE) %%%%%%%%%%%%%%%%%%%%%%%%%%%

void localCollocation::getIndexGroups(){

        /* This function uses the method of Curtis, Powell and Reid (1974) to find groups of variables
     * to evaluate efficiently the sparse Jacobian by perturbing simultaneously groups of variables.
     * Reference:
     * A. R. Curtis, M.J.D. Powell and J.K. Reid
     * "On the estimation of Sparse Jacobian Matrices"
     * J Inst Maths Applics (1974) 13, 117-119
     *
     */

    int nDecVar=NLP.nDecVar;
    int nCns=nStates*(nCollocationPoints)+nEvents+(nPath*nCollocationPoints)+1;

    Eigen::MatrixXd nzG(NLP.sparsity.nnz_D,2);

    nzG=NLP.sparsity.nzG_D;

    Eigen::MatrixXd JDense(nCns,nDecVar);

    //Set the sparsity information in to a dense matrix
    utils::sparseToDense(nzG,JDense);

      Eigen::VectorXd C1(nCns);
      Eigen::VectorXd C2(nCns);

      Eigen::MatrixXd idxGroups(nDecVar,nDecVar);
      Eigen::VectorXd col_check(nDecVar);
      Eigen::VectorXd sizeGroups(nDecVar);


      col_check.setZero();
      idxGroups.setZero();
      sizeGroups.setZero();

        //Define the first group

      idxGroups(0,0)=0; //(Group,Index)=Column index

      col_check(0)=1; //The column 0 is already in a group..->


      //Iterate along the columns of the jacobian matrix

      int gcount=1;
      int colcount=1;
      bool ok;
      double dotCols;


      for(int j=1;j<nDecVar;j++){

            ok=true;

            for(int l=0;l<gcount;l++){

                        if(j==idxGroups(0,l)){
                            ok=false;
                            break;
                        }

                         C1=JDense.col(idxGroups(0,l));
                         C2=JDense.col(j);

                         dotCols=C1.dot(C2);

                         if(dotCols>0.0){
                             ok=false;
                         }
            }

            if(ok){
                idxGroups(0,gcount)=j;
                gcount++;
                colcount++;
                col_check(j)=1;
            }
     }

      sizeGroups(0)=gcount;

      //Now lets obtain the other groups


    int group_index=0;

    int maxSize=0;

    while(colcount<nDecVar){

        group_index++;

        if(gcount>maxSize)
            maxSize=gcount;

        gcount=0;

        for(int j=1;j<nDecVar;j++){

                    ok=true;

                    if(col_check(j)==1){

                            ok=false;
                     }


                    //If the column is not in a group use it for a group

                    if(ok==true){

                          for(int l=0;l<gcount;l++){

                                    C1=JDense.col(idxGroups(group_index,l));
                                    C2=JDense.col(j);

                                    dotCols=C1.dot(C2);

                                     if(dotCols>0.0){

                                            ok=false;
                                      }

                            }
                     }

                    //If the column is not in a group AND there is
                    //no dependence with other colums use it in the gruop

                    if(ok){

                            idxGroups(group_index,gcount)=j;
                            gcount++;
                            colcount++;
                            col_check(j)=1;

                    }

        }

        //Save the size of this group

        sizeGroups(group_index)=gcount;

    }


    //Now just obtain the usable data

    Eigen::MatrixXd groups(group_index+1,maxSize);
    Eigen::VectorXd sizeG (group_index+1);

    groups=idxGroups.block(0,0,groups.rows(),groups.cols());

    sizeG=sizeGroups.head(group_index+1);

    //Return the information of the groups!

    NLP.sparsity.idxGroupsD=groups;
    NLP.sparsity.sizeGroupsD=sizeG;

    cout<<"-Number of index sets for sparse finite differences in the Derivative Matrix ->"<<group_index+1<<endl;


}//End localCollocation::getIndexGroups

void localCollocation::getPerturbationMatrix(){

    //Set a matrix with the perturbation vectors depending on the index groups of the jacobian

    double delj;
    int element;


    //Machine precision
    delj=sqrt(MC_EPSILON);

    //delj=1e-8;

    //Iterate along the groups

    int nGroups=NLP.sparsity.sizeGroupsD.size();

    Eigen::MatrixXd perturbationMatrix(NLP.nDecVar,nGroups);
    Eigen::MatrixXd negativePerturbationMatrix(NLP.nDecVar,nGroups);

    perturbationMatrix.setZero();
    negativePerturbationMatrix.setZero();

    for(int i=0;i<nGroups;i++){


        for(int j=0;j<NLP.sparsity.sizeGroupsD(i);j++){

            element=NLP.sparsity.idxGroupsD(i,j);

            perturbationMatrix(element,i)=delj;
            negativePerturbationMatrix(element,i)=2*delj;


        }

    }

    NLP.sparsity.positivePerturbationMatrix=perturbationMatrix;
    NLP.sparsity.negativePerturbationMatrix=negativePerturbationMatrix;


}//End localCollocation::getPerturbationMatrix

void localCollocation::setConstantMatrices(){

    //Variables

    int ny=nStates;
    int nu=nControls;
    int nDecVar=NLP.nDecVar;                      //Number of decision variables
    int M=nCollocationPoints;
    int nP=nPath*nCollocationPoints;              //Number of path constraints
    int nDefects=NLP.nCollocationCns;             //Number of defect constraints
    int nCns=NLP.nCns;                            //Number of constraints

    Eigen::MatrixXd A(NLP.nCns,NLP.nDecVar);
    Eigen::MatrixXd B(NLP.nCollocationCns+nP+nEvents+1,(nStates*nCollocationPoints)+nP+nEvents+1);

    A.setZero();
    B.setZero();

    //==================Trapezoidal method matrices ===================

    if(colMethod=="Trapezoidal"){

        //-------------------------------Matrix A----------------------------


        Eigen::MatrixXd I_ns(ny,ny);
        I_ns.setIdentity();

        Eigen::MatrixXd I_u(ny,nu);
        I_u.setZero();

        Eigen::MatrixXd IInsu(ny,2*ny+2*nu);

        IInsu<<-I_ns,I_u,I_ns,I_u;

        for(int i=0;i<nDiscretePoints-1;i++){

            IInsu<<-I_ns,I_u,I_ns,I_u;
            A.block(i*ny,i*(ny+nu)+2,ny,2*ny+2*nu)=IInsu;

        }//End for i=0

        //-------------------------------Matrix B----------------------------

        Eigen::MatrixXd II(ny,2*ny);
        Eigen::MatrixXd I(nEvents+nP+1,nEvents+nP+1);

        for(int i=0;i<nDiscretePoints-1;i++){

            //Obtaint the deltaTau Value

            double dTau=mesh.deltaTau(i);

            II<<Eigen::MatrixXd::Identity(ny,ny),Eigen::MatrixXd::Identity(ny,ny);
            B.block(i*ny,i*ny,ny,2*ny)=-0.5*dTau*II.array();

        }//End for i=0

        I<<Eigen::MatrixXd::Identity(nEvents+nP+1,nEvents+nP+1);
        B.block(nDefects,(ny*M),nEvents+nP+1,nEvents+nP+1)=I;

    } else if(colMethod=="Hermite-Simpson"){

         //==================Hermite-Simpson method matrices ===================

        //-------------------------------Matrix A----------------------------

        Eigen::MatrixXd I_ns(ny,ny);
        I_ns.setIdentity();

        Eigen::MatrixXd I_nsZ(ny,ny);
        I_nsZ.setZero();

        Eigen::MatrixXd I_u(ny,nu);
        I_u.setZero();

        Eigen::MatrixXd IInsu(ny,3*ny+3*nu);
        Eigen::MatrixXd IInsu2(ny,3*ny+3*nu);

        Eigen::MatrixXd Ak(2*ny,3*ny+3*nu);

        IInsu<<-0.5*I_ns, I_u,   I_ns,  I_u,    -0.5*I_ns, I_u;
        IInsu2<<-I_ns   ,I_u,   I_nsZ, I_u,    I_ns,      I_u;

        Ak<<IInsu,IInsu2;

        for(int i=0;i<nDiscretePoints-1;i++){

            A.block(i*2*ny,i*(2*(ny+nu))+2,2*ny,3*(ny+nu))=Ak;

        }


        //-------------------------------Matrix B----------------------------

        Eigen::MatrixXd II(ny,3*ny);
        Eigen::MatrixXd II2(ny,3*ny);

        Eigen::MatrixXd Bk(2*ny,3*ny);

        Eigen::MatrixXd I(nEvents+nP+1,nEvents+nP+1);


        II<< -(1/8.0)*I_ns, I_nsZ ,       (1/8.0)*I_ns;
        II2<<-(1/6.0)*I_ns, -(2/3.0)*I_ns,  (-1/6.0)*I_ns;

        Bk<<II,II2;

        B.setZero();

        double dTau=0;


        for(int i=0;i<nDiscretePoints-1;i++){

            dTau=mesh.deltaTau(i);
            B.block(i*2*ny,i*2*ny,2*ny,3*ny)=Bk*dTau;

        }

        I<<Eigen::MatrixXd::Identity(nEvents+nP+1,nEvents+nP+1);
        B.block(nDefects,(ny*M),nEvents+nP+1,nEvents+nP+1)=I;

    }//End if-else colMethod



    NLP.sparsity.A.resize(A.rows(),A.cols());
    NLP.sparsity.B.resize(B.rows(),B.cols());

    NLP.sparsity.A=A.sparseView(); //Save the A structure as sparse matrix
    NLP.sparsity.B=B.sparseView(); //Save the B structure as sparse matrix


}// End localCollocation::setConstantMatrices

void localCollocation::detectNumericalSparsity(){

    //Set offset of the constraints
    int eventOffset=nCollocationPoints*nStates;
    int pathOffset=eventOffset+nEvents;

    //Create the derivativeMatrix
    int nDecVar=NLP.nDecVar;

    Eigen::MatrixXd derivativeMatrix(nCollocationPoints*(nStates+nPath)+nEvents+1,nDecVar);
    derivativeMatrix.setZero();

    ///Create the templates for the numerical derivatives of the user defined functions
    Eigen::MatrixXd dxPattern(nStates,nStates+nControls);
    Eigen::MatrixXd pathPattern(nPath,nStates+nControls);
    Eigen::MatrixXd eventPattern_t0(nEvents,nStates+1);
    Eigen::MatrixXd eventPattern_tF(nEvents,nStates+1);

    Eigen::SparseMatrix<double> D(nCollocationPoints*(nStates+nPath)+nEvents+1,nDecVar);

    dxPattern=NLP.sparsity.dxPattern;
    pathPattern=NLP.sparsity.pathPattern;
    eventPattern_t0=NLP.sparsity.eventPattern_t0;
    eventPattern_tF=NLP.sparsity.eventPattern_tF;

    std::cout<<std::endl;
    std::cout<<"------------------------------> Detecting Derivative Matrix Sparsity <------------"<<std::endl;
    std::cout<<std::endl;


    for (int k=0; k<nCollocationPoints;k++){

        derivativeMatrix.col(0).segment(k*nStates,nStates)=Eigen::VectorXd::Ones(nStates);      //Derivatives w.r.t. t0
        derivativeMatrix.col(1).segment(k*nStates,nStates)=Eigen::VectorXd::Ones(nStates);      //Derivatives w.r.t. tF

        derivativeMatrix.block(k*(nStates),2+(k*(nStates+nControls)),nStates,(nStates+nControls))=dxPattern;

        derivativeMatrix.block(pathOffset+(k*(nPath)),2+(k*(nStates+nControls)),nPath,(nStates+nControls))=pathPattern;

    }

    // SPARSITY OF THE EVENT CONSTRAINTS

   derivativeMatrix.block(eventOffset,0,nEvents,1)=eventPattern_t0.col(0);
   derivativeMatrix.block(eventOffset,1,nEvents,1)=eventPattern_tF.col(0);

   derivativeMatrix.block(eventOffset,2,nEvents,nStates)=eventPattern_t0.rightCols(nStates);
   derivativeMatrix.block(eventOffset,nDecVar-(nStates+nControls),nEvents,nStates)=eventPattern_tF.rightCols(nStates);

   derivativeMatrix(derivativeMatrix.rows()-1,0)=1;
   derivativeMatrix(derivativeMatrix.rows()-1,1)=1;

    //Obtain the sparsity data of the derivative matrix

   int nnzD=0;
   int counter=0;

   D=derivativeMatrix.sparseView();
   nnzD=D.nonZeros();

   Eigen::MatrixXd nzElementsD(nnzD,2);
   nzElementsD.setZero();


   for (int k=0; k<D.outerSize(); ++k)
         for (Eigen::SparseMatrix<double>::InnerIterator it(D,k); it; ++it)
         {
           nzElementsD(counter,0)=it.row();   // row index
           nzElementsD(counter,1)=it.col();   // col index (here it is equal to k)
           counter++;

         }

   std::cout<<"+Non-zero elements in the Derivative Matrix->"<<nnzD<<std::endl;

   //Return this!
   NLP.sparsity.nzG_D.resize(nnzD,2);
   NLP.sparsity.nzG_D=nzElementsD;
   NLP.sparsity.nnz_D=D.nonZeros();

    std::cout<<"----------------------------------------------------------------------------------------------"<<std::endl;
    std::cout<<std::endl;
    std::cout<<"-----------------------------------> Detecting Full Jacobian Sparsity <-----------------------"<<std::endl;


    Eigen::SparseMatrix<double> J(NLP.nCns,NLP.nDecVar);

    //Compute the Jacobian sparsity
    J=this->NLP.sparsity.A + (this->NLP.sparsity.B*D);

    //J=J_dense.sparseView();

    int nnzJ=J.nonZeros();

    counter=0;

    Eigen::MatrixXd nzElementsJ(nnzJ,2);
    nzElementsJ.setZero();

    for (int k=0; k<J.outerSize(); ++k)
        for (Eigen::SparseMatrix<double>::InnerIterator it(J,k); it; ++it)
            {
            nzElementsJ(counter,0)=it.row();   // row index
            nzElementsJ(counter,1)=it.col();   // col index (here it is equal to k)
            counter++;
            }

    NLP.sparsity.nnz_J=nnzJ;
    NLP.sparsity.nzG_J.resize(nnzJ,2);
    NLP.sparsity.nzG_J=nzElementsJ;

    std::cout<<"+Nonzero elements in the Jacobian of the constrains-->"<< nnzJ <<std::endl;
    std::cout<<std::endl;

}//End localCollocation::detectNumericalSparsity


}//End nocs namespace
