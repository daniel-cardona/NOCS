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

//Handlers to the dynamics libraries
#include "directCollocation/local/dynamicsHandlers.hpp"

#include "directCollocation/local/blockOperations.hpp"

//Local Collocation module

#include "directCollocation/local/userFunctions.hpp"

#include "directCollocation/local/fcnGenerator.hpp"

#include <iostream>
#include <chrono>


namespace nocs {

//*********************************************************************
//---------------------- CONSTRUCTORS --------------------------------
//*********************************************************************


//------------------------------------------------------------------------------------------------------------
//----------------------  Parameterized constructor (without dynamics libraries) --------------------------------
//------------------------------------------------------------------------------------------------------------

localCollocation::localCollocation(const int &nStates,const int &nControls, const int &nDiscrete,const int &nPath,const int &nEvents){

    //Configure the local collocation problem depending on the user input

    this->nStates=nStates;
    this->nControls=nControls;
    this->nDiscretePoints=nDiscrete;
    this->nPath=nPath;
    this->nEvents=nEvents;

    //Set the dimensions of the OCP Vectors

    //----States
    bounds.states.lower.setZero(this->nStates);
    bounds.states.upper.setZero(this->nStates);

    //----Controls

    bounds.controls.lower.setZero(this->nControls);
    bounds.controls.upper.setZero(this->nControls);

    //----Path constraints

    bounds.path.lower.setZero(this->nPath);
    bounds.path.upper.setZero(this->nPath);

    //----Event constraints

    bounds.events.lower.setZero(this->nEvents);
    bounds.events.upper.setZero(this->nEvents);

    //Time constraint

    bounds.initialTime.lower.setZero(1);
    bounds.initialTime.upper.setZero(1);

    bounds.finalTime.lower.setZero(1);
    bounds.finalTime.upper.setZero(1);

    //Set the first mesh information

    mesh.tauV.setLinSpaced(nDiscretePoints,0,1);
    mesh.w.setZero(this->nStates);
    mesh.nIter=1;

    //Set the externalLibrary flag to false

    this->algorithm.usingExternalLibrary=false;

}


//------------------------------------------------------------------------------------------------------------
//----------------------  Parameterized constructor (with dynamics libraries) --------------------------------
//------------------------------------------------------------------------------------------------------------

localCollocation::localCollocation(const int &nStates,const int &nControls, const int &nDiscrete,const int &nPath,const int &nEvents,const std::string &robotFile){

    //Configure the local collocation problem depending on the user input

    this->nStates=nStates;
    this->nControls=nControls;
    this->nDiscretePoints=nDiscrete;
    this->nPath=nPath;
    this->nEvents=nEvents;

    //Set the dimensions of the OCP Vectors

    //----States
    bounds.states.lower.setZero(this->nStates);
    bounds.states.upper.setZero(this->nStates);

    //----Controls

    bounds.controls.lower.setZero(this->nControls);
    bounds.controls.upper.setZero(this->nControls);

    //----Path constraints

    bounds.path.lower.setZero(this->nPath);
    bounds.path.upper.setZero(this->nPath);

    //----Event constraints

    bounds.events.lower.setZero(this->nEvents);
    bounds.events.upper.setZero(this->nEvents);

    //Time constraint

    bounds.initialTime.lower.setZero(1);
    bounds.initialTime.upper.setZero(1);

    bounds.finalTime.lower.setZero(1);
    bounds.finalTime.upper.setZero(1);

    //Set the first mesh information

    mesh.tauV.setLinSpaced(nDiscretePoints,0,1);
    mesh.w.setZero(this->nStates);

    mesh.nIter=1;

    //Construct the robotic system using the selected dynamic library

    this->robotFile=robotFile;


    #ifdef geoMBD_compile

    std::cout<<"- Setting up the geometric multibody dynamics (geombd) library"<<std::endl;

    this->robotGeo=Robot::build_model(robotFile);

    this->robotDDynamics= std::make_shared< geo::FwdDynDifCRTP< DataType > >( this->robotGeo.value() );

    #endif

    #ifdef pinocchio_compile

       std::cout<<"Setting up the pinocchio library"<<std::endl;

       //Create the pinocchio objects
       pinocchio::urdf::buildModel(this->robotFile,this->model);
       this->robot_data=pinocchio::Data(this->model);

       //Configure the pointers to the autoGen funciontions
       this->dynamics =&nocs::pinocchioLib::autoGenDynamics;


    #endif

    #ifdef rbdl_compile

      std::cout<<"Setting up the RBDL library"<<std::endl;

      //Create the rbdl objects
      const char *urdfFile =this->robotFile.c_str();
      this->robot_model=new RigidBodyDynamics::Model();
      RigidBodyDynamics::Addons::URDFReadFromFile(urdfFile,robot_model,false);

      //Configure the pointers to the autoGen functions
      this->dynamics =&nocs::rbdlLib::autoGenDynamics;


    #endif

    this->algorithm.usingExternalLibrary=true;                       //Set the flag for the geoMBD handlers!

} //End localCollocation


//------------------------------------------------------------------------------------------------------------
//---------------------------------------  Copy constructor --------------------------------------------------
//------------------------------------------------------------------------------------------------------------

localCollocation::localCollocation(const localCollocation &problem){

    //First set constant information of the problem
    this->nStates=problem.nStates;
    this->nControls=problem.nControls;
    this->nDiscretePoints=problem.nDiscretePoints;
    this->nPath=problem.nPath;
    this->nEvents=problem.nEvents;

    this->colMethod=problem.colMethod;
    this->derivatives=problem.derivatives;

    //Set the dimensions of the OCP Vectors

    //----States
    this->bounds.states.lower=problem.bounds.states.lower;
    this->bounds.states.upper=problem.bounds.states.upper;

    //----Controls
    this->bounds.controls.lower=problem.bounds.controls.lower;
    this->bounds.controls.upper=problem.bounds.controls.upper;

    //----Path constraints
    this->bounds.path.lower=problem.bounds.path.lower;
    this->bounds.path.upper=problem.bounds.path.upper;

    //----Event constraints
    this->bounds.events.lower=problem.bounds.events.lower;
    this->bounds.events.upper=problem.bounds.events.upper;

    //Time constraint

    this->bounds.initialTime.lower=problem.bounds.initialTime.lower;
    this->bounds.initialTime.upper=problem.bounds.initialTime.upper;

    this->bounds.finalTime.lower=problem.bounds.finalTime.lower;
    this->bounds.finalTime.upper=problem.bounds.finalTime.upper;

    //Transfer the mesh information (constant and useful for the mesh refinement algorithm)

    this->mesh.tauV=problem.mesh.tauV;
    this->mesh.w=problem.mesh.w;


    this->mesh.epsilon_old.setZero(this->nDiscretePoints-1);
    this->mesh.epsilon_old=problem.mesh.epsilon_old;

    this->mesh.tauV_old.setZero(problem.nDiscretePoints);
    this->mesh.tauV_old=problem.mesh.tauV_old;

    this->mesh.I_old.setZero(problem.nDiscretePoints-1);
    this->mesh.I_old=problem.mesh.I_old;

    //Transfer the algorithm properties

    this->algorithm.derivativeChecker=      problem.algorithm.derivativeChecker;
    this->algorithm.error_ode=              problem.algorithm.error_ode;
    this->algorithm.kappa=                  problem.algorithm.kappa;
    this->algorithm.maxMeshIterations=      problem.algorithm.maxMeshIterations;
    this->algorithm.maxPoints=              problem.algorithm.maxPoints;
    this->algorithm.meshRefinement=         problem.algorithm.meshRefinement;
    this->algorithm.usingExternalLibrary=   problem.algorithm.usingExternalLibrary;


    //Copy the sparsity patterns

    this->NLP.sparsity.dxPattern= problem.NLP.sparsity.dxPattern;
    this->NLP.sparsity.pathPattern= problem.NLP.sparsity.pathPattern;
    this->NLP.sparsity.eventPattern_t0= problem.NLP.sparsity.eventPattern_t0;
    this->NLP.sparsity.eventPattern_tF= problem.NLP.sparsity.eventPattern_tF;
    this->NLP.sparsity.eventPattern_e0= problem.NLP.sparsity.eventPattern_e0;
    this->NLP.sparsity.eventPattern_eF= problem.NLP.sparsity.eventPattern_eF;

    this->NLP.sparsity.nnz_Dfk=problem.NLP.sparsity.nnz_Dfk;
    this->NLP.sparsity.nnz_Dpk=problem.NLP.sparsity.nnz_Dpk;
    this->NLP.sparsity.nnz_De0=problem.NLP.sparsity.nnz_De0;
    this->NLP.sparsity.nnz_DeF=problem.NLP.sparsity.nnz_DeF;
    this->NLP.sparsity.nnz_Dt0=problem.NLP.sparsity.nnz_Dt0;
    this->NLP.sparsity.nnz_DtF=problem.NLP.sparsity.nnz_DtF;

    //Copy the guess structures

    this->guess.states= problem.guess.states;
    this->guess.controls=problem.guess.controls;
    this->guess.t0=problem.guess.t0;
    this->guess.tF=problem.guess.tF;

    //Transfer the robot information (If any dynamic library is used)

    this->robotFile=problem.robotFile;

    //Set the counter of mesh iterations

    this->mesh.nIter=problem.mesh.nIter;

}


//------------------------------------------------------------------------------------------------------------
//------------------------Parameterized constructor (Mesh refinement) ----------------------------------------
//------------------------------------------------------------------------------------------------------------

localCollocation::localCollocation(const nlp_str &NLP, const meshInfo &mesh, const bnd_str &bounds ,
                                   const alg &algorithm, const guess_str &guess, const string &colMethod,
                                   const deriv &derivatives,  const std::string & robotFile){

    //First set constant information of the problem
    this->nStates=          bounds.states.lower.size();
    this->nControls=        bounds.controls.lower.size();
    this->nDiscretePoints=  mesh.tauV.size();
    this->nPath=            bounds.path.lower.size();
    this->nEvents=          bounds.events.lower.size();

    this->derivatives=derivatives;

    //Set the dimensions of the OCP Vectors

    //----States
    this->bounds.states.lower=bounds.states.lower;
    this->bounds.states.upper=bounds.states.upper;

    //----Controls
    this->bounds.controls.lower=bounds.controls.lower;
    this->bounds.controls.upper=bounds.controls.upper;

    //----Path constraints
    this->bounds.path.lower=bounds.path.lower;
    this->bounds.path.upper=bounds.path.upper;

    //----Event constraints
    this->bounds.events.lower=bounds.events.lower;
    this->bounds.events.upper=bounds.events.upper;

    //Time constraint

    this->bounds.initialTime.lower=bounds.initialTime.lower;
    this->bounds.initialTime.upper=bounds.initialTime.upper;

    this->bounds.finalTime.lower=bounds.finalTime.lower;
    this->bounds.finalTime.upper=bounds.finalTime.upper;

    //Transfer the mesh information (constant and useful for the mesh refinement algorithm)

    this->mesh.tauV=mesh.tauV;
    this->mesh.w=mesh.w;

    //this->mesh.epsilon_old.setZero(this->nDiscretePoints-1);
    this->mesh.epsilon_old=mesh.epsilon_old;

    //this->mesh.tauV_old.setZero(this->nDiscretePoints);
    this->mesh.tauV_old=mesh.tauV_old;

    //this->mesh.I_old.setZero(this->nDiscretePoints-1);
    this->mesh.I_old=mesh.I_old;

    //Transfer the information for the warm start

    this->guess.states=guess.states;
    this->guess.controls=guess.controls;
    this->guess.t0=guess.t0;
    this->guess.tF=guess.tF;

    //Transfer the algorithm properties

    this->algorithm.derivativeChecker=      algorithm.derivativeChecker;
    this->algorithm.error_ode=              algorithm.error_ode;
    this->algorithm.kappa=                  algorithm.kappa;
    this->algorithm.maxMeshIterations=      algorithm.maxMeshIterations;
    this->algorithm.maxPoints=              algorithm.maxPoints;
    this->algorithm.meshRefinement=         algorithm.meshRefinement;
    this->algorithm.usingExternalLibrary=   algorithm.usingExternalLibrary;


    //Copy the sparsity patterns


    this->NLP.sparsity.dxPattern= NLP.sparsity.dxPattern;
    this->NLP.sparsity.pathPattern= NLP.sparsity.pathPattern;
    this->NLP.sparsity.eventPattern_t0= NLP.sparsity.eventPattern_t0;
    this->NLP.sparsity.eventPattern_tF= NLP.sparsity.eventPattern_tF;
    this->NLP.sparsity.eventPattern_e0= NLP.sparsity.eventPattern_e0;
    this->NLP.sparsity.eventPattern_eF= NLP.sparsity.eventPattern_eF;

    this->NLP.sparsity.nnz_Dfk=NLP.sparsity.nnz_Dfk;
    this->NLP.sparsity.nnz_Dpk=NLP.sparsity.nnz_Dpk;
    this->NLP.sparsity.nnz_De0=NLP.sparsity.nnz_De0;
    this->NLP.sparsity.nnz_DeF=NLP.sparsity.nnz_DeF;
    this->NLP.sparsity.nnz_Dt0=NLP.sparsity.nnz_Dt0;
    this->NLP.sparsity.nnz_DtF=NLP.sparsity.nnz_DtF;

    //Construct the robotic system using

    this->robotFile=robotFile;

    #ifdef geoMBD_compile

        std::cout<<"- Setting up the geometric multibody dynamics (geombd) library"<<std::endl;

        this->robotGeo=Robot::build_model(robotFile);

        this->robotDynamics=std::make_shared<geoCRTP::FwdDynCRTP < DataType> > (this->robotGeo.value());
        this->robotDDynamics= std::make_shared< geo::FwdDynDifCRTP< DataType > >( this->robotGeo.value() );

        this->dynamics=&nocs::geoMBDLib::autoGenDynamics;

    #endif

    #ifdef pinocchio_compile

       std::cout<<"Setting up the pinocchio library"<<std::endl;

       //Create the pinocchio objects
       pinocchio::urdf::buildModel(this->robotFile,this->model);
       this->robot_data=pinocchio::Data(this->model);

       //Configure the pointers to the autoGen funciontions
       this->dynamics =&nocs::pinocchioLib::autoGenDynamics;



    #endif


    #ifdef rbdl_compile

      std::cout<<"Setting up the RBDL library"<<std::endl;

      //Create the rbdl objects
      const char *urdfFile =this->robotFile.c_str();
      this->robot_model=new RigidBodyDynamics::Model();
      RigidBodyDynamics::Addons::URDFReadFromFile(urdfFile,robot_model,false);

      //Configure the pointers to the autoGen funciontions
      this->dynamics =&nocs::rbdlLib::autoGenDynamics;


    #endif

    //Set the counter of mesh iterations

    this->mesh.nIter=mesh.nIter;

    //Switch the collocation method only if the nIter>2 and the mesh refinement flag is TRUE

    if(this->mesh.nIter>=2 && this->algorithm.meshRefinement==true && colMethod=="Trapezoidal"){

        this->colMethod="Hermite-Simpson";

        cout<<"Switching the collocation method"<<endl;

    } else{

       this->colMethod=colMethod;

    }

}


//*********************************************************************
//--------------------------- METHODS --------------------------------
//*********************************************************************

void localCollocation::setupNLP(){

    //Main variables of the non-linear problem

    if(colMethod=="Hermite-Simpson"){
        nCollocationPoints=2*nDiscretePoints-1;
    }
    else{ //If a trapezoidal method is used then the discrete points are the colPoints
        nCollocationPoints=nDiscretePoints;
    }//End if

    NLP.nSegments=nCollocationPoints-1;
    NLP.nDecVar=(nControls+nStates)*(nCollocationPoints)+2;
    NLP.nCollocationCns=nStates*NLP.nSegments;
    NLP.nCns=NLP.nCollocationCns+(nPath*nCollocationPoints)+nEvents+1;

    //If an Hermite-Simpson method is used fill the reference matrix,
    //if not just don't initialize it

    if(colMethod=="Hermite-Simpson"){

        int k=0;
        NLP.colPointsRef.setZero(nDiscretePoints-1,3);

        for (int i=0;i<nDiscretePoints-1;i++){

            for(int j=0;j<3;j++){
               NLP.colPointsRef(i,j)=k;
               k++;
            }
            k--;

        }//End for i

    }//End if

   //Set the size of the guess matrices (Only in the initial iteration)
   if(mesh.nIter==1){
       guess.states.setZero(nStates,nCollocationPoints);
       guess.controls.setZero(nControls,nCollocationPoints);
   }

    //Set the dimensions of the NLP vectors

    //Initial guess
    NLP.x0.setZero(NLP.nDecVar);

    //Upper and lower limits of the decision variables
    NLP.xlb.setZero(NLP.nDecVar);
    NLP.xub.setZero(NLP.nDecVar);

    //Upper and lower limits of the constraints
    NLP.glb.setZero(NLP.nCns);
    NLP.gub.setZero(NLP.nCns);

    //Initialize the mesh variables
    mesh.deltaTau.setZero(nDiscretePoints-1);

    for(int i=0;i<nDiscretePoints-1;i++){
        mesh.deltaTau(i)=mesh.tauV(i+1)-mesh.tauV(i);
    }//End for

    mesh.time.setZero(nDiscretePoints);
    mesh.eta.setZero(nStates,nDiscretePoints-1);
    mesh.epsilon.setZero(nDiscretePoints-1); //Relative local error
    mesh.r.setZero(nDiscretePoints-1);


    //Now set the size for the solution variables

    solution.z_opt.setZero(NLP.nDecVar);
    solution.xSol.setZero(nStates,nCollocationPoints);
    solution.uSol.setZero(nControls,nCollocationPoints);
    solution.fSol.setZero(nStates,nCollocationPoints);
    solution.tSol.setZero(nCollocationPoints);

    //Setup the function generator pointers to the Local Collocation module

    costFunction          =&nocs::localGenerator::costFcn;
    cnsFunction           =&nocs::localGenerator::rightHandSideSparsity;
    interpolateSolution   =&nocs::localGenerator::interpolateSolution;
    cnsJacobian           =&nocs::localGenerator::derivatives::computePropagatedJacobian;


    //Gradient cost pointers
    if(derivatives.gradientCost=="Numerical"){
        gradientCost=&nocs::localGenerator::derivatives::numerical::computeGradientCost;
    } else if (derivatives.gradientCost == "Analytical"){
         gradientCost=&nocs::localGenerator::derivatives::analytical::computeGradientCost;
    }//End if-else-if derivatives-gradient

    #ifdef rbdl_compile

        if(derivatives.jacobianCns=="Analytical"){

            std::cout<<"NOCS using RBDL library only allows the computation of the numerical jacobian. Modifiyng Analytical->Numerical"<<std::endl;
        }

        derivatives.jacobianCns="Numerical"; //Force the computation of the jacobian to be NUMERICAL!
    #endif


    //Jacobian pointers
     if(derivatives.jacobianCns=="Numerical"){

          fJacobian     =&nocs::computeNumericalJacobian_dx;
          pathJacobian  =&nocs::computeNumericalJacobian_path;
          eventJacobian =&nocs::computeNumericalJacobian_event;


     } else if (derivatives.jacobianCns == "Analytical"){

          this->fJacobian=&nocs::Function::analytical::fGradient;

          #ifdef geoMBD_compile
           this->fJacobian=&nocs::geoMBDLib::autoGenfGradient;
          #endif

          #ifdef pinocchio_compile
            this->fJacobian=&nocs::pinocchioLib::autoGenfGradient;
          #endif

          pathJacobian  =&nocs::Function::analytical::pathGradient;
          eventJacobian =&nocs::Function::analytical::eventGradient;

     }//End if-else-if derivatives-jacobian


    if(colMethod=="Trapezoidal"){

        //---------- Quadrature weights

        NLP.weights.setZero(nCollocationPoints);
        NLP.time_weights=(1/2.0);
        NLP.weights(0)=mesh.deltaTau(0);
        NLP.weights(nCollocationPoints-1)=mesh.deltaTau(nCollocationPoints-2);

        for(int k=1;k<nCollocationPoints-1;k++){

            NLP.weights(k)=(mesh.deltaTau(k-1)+mesh.deltaTau(k));

        }//End for k=1

    } else if(colMethod=="Hermite-Simpson"){

        //---------- Quadrature weights

        NLP.weights.setZero(nCollocationPoints);
        NLP.time_weights=(1/6.0);
        NLP.weights(0)=mesh.deltaTau(0);
        NLP.weights(nCollocationPoints-1)=mesh.deltaTau(nDiscretePoints-2);

        int j=1;

        for(int k=0;k<nDiscretePoints-1;k++){

            NLP.weights(j)=4*mesh.deltaTau(k);
            j+=2;

        }//End for k=0

        j=2;

        for(int k=1;k<nDiscretePoints-1;k++){

            NLP.weights(j)=(mesh.deltaTau(k-1)+mesh.deltaTau(k));
            j+=2;

        }//End for k=1

    } //End if-else-of colMethod

    //Setup the defect setup of the pointers if no dynamics library are used

    if(algorithm.usingExternalLibrary==false){
        dynamics =&nocs::Function::dae;
        fJacobian=&nocs::computeNumericalJacobian_dx; //Here the Jacobian is comptued using numerical algorithms!!
    }//End if

}//End localCollocation::setupNLP


int localCollocation::solve(){

    int status=0;
    int jr=this->mesh.nIter;

    //1. Setup the NLP

    this->transcription();
    this->detectProblemSparsity();

    //this->getIndexGroups();
    //this->getPerturbationMatrix();

    Eigen::VectorXd z(this->NLP.nDecVar);
    Eigen::VectorXd nz(this->NLP.nDecVar);

//    for(int i=0; i<10 ; i++){

//    nz.setZero();
//    z.setRandom();


//    auto start = std::chrono::high_resolution_clock::now();

//    nocs::localGenerator::derivatives::computePropagatedJacobian(*this,z,nz);

//    auto stop = std::chrono::high_resolution_clock::now();

//    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count();

//    cout <<duration<< endl;

//    }

//    return 0;


    //1.5. Print the basic information of this mesh iteration
    this->printInformation();

    //2. Solve the NLP

    status=this->solveNLP_ipopt();

    //3. Get the solution and set it for the mesh refinement (if any)
    this->getSolution();

   //4. Now evaluate the solution of the problem

    this->evaluateSolution();

    //5. Evaluate if the termination criteria is fullfilled

    if((this->mesh.epsilon_max_history<this->algorithm.error_ode && jr!=0) || jr==this->algorithm.maxMeshIterations){

        std::cout<<"End the mesh refinement in iter "<<jr<<endl;
        cout<<"Max error "<<this->mesh.epsilon_max_history<<endl;
        return 2;

    } else{
        cout<<"Max error "<<this->mesh.epsilon_max_history<<endl;
       }

    //---------------------------------
    //---- Mesh refinement algorithm
    //---------------------------------

    //6. Determine the order reduction

    this->estimateOrderReduction();

    //7. Construct the new mesh

    this->constructNewMesh();

    //8. Set the warm start for the next mesh iteration

    this->setWarmStart();

    this->mesh.nIter++;

    return status;

}// End localCollocation::solve

void localCollocation::printInformation(){

    string initialization="\n**-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-** \n"
                              "**-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-** \n"
                              "**                                                                                                 **\n"
                              "**                                      WELCOME TO NOCS!                                           **\n"
                              "**                                                                                                 **\n"
                              "** A sparse Numerical Optimal Control Solver for robotic systems based on local                    **\n"
                              "** and pseudospectral collocation methods and geometric algorithms                                 **\n"
                              "**                                                                                                 **\n"
                              "**-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-** \n";

        string license="**                                                                                                 **\n"
                       "** Copyright (C) 2020-2021  Daniel Cardona-Ortiz                                                   **\n"
                       "** Permission is hereby granted, free of charge, to any person obtaining a copy                    **\n"
                       "** of this software and associated documentation files (the \"Software\"), to deal                   **\n"
                       "** in the Software without restriction, including without limitation the rights                    **\n"
                       "** to use, copy, modify, merge, publish, distribute, sublicense, and/or sell                       **\n"
                       "** copies of the Software, and to permit persons to whom the Software is                           **\n"
                       "** furnished to do so, subject to the following conditions:                                        **\n"
                       "** The above copyright notice and this permission notice shall be included in all                  **\n"
                       "** copies or substantial portions of the Software.                                                 **\n"
                       "** THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR                      **\n"
                       "** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,                        **\n"
                       "** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE                     **\n"
                       "** AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER                          **\n"
                       "** LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,                   **\n"
                       "** OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE                   **\n"
                       "** SOFTWARE.                                                                                       **\n"
                       "**                                                                                                 **\n"
                       "**-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-** \n";

        string contact="**                                                                                                 **\n"
                       "**                                          CONTACT                                                **\n"
                       "** The developers can be contacted at:                                                             **\n"
                       "**                                                                                                 **\n"
                       "** (Author)         Daniel Cardona-Ortiz            daniel.cardona@cinvestav.mx                    **\n"
                       "** (Contributor)    Alvaro Paz                      alvaro.paz@cinvestav.mx                        **\n"
                       "** (Contributor)    Gustavo Arechavaleta            garechav@cinvestav.mx                          **\n"
                       "**                                                                                                 **\n"
                       "**-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-** \n";


        if(mesh.nIter==1){
            cout<<initialization;
            cout<<license;
            cout<<contact;
        }

        int tablesize=100;

                cout<<endl;

                for(int i=0;i<1;i++){

                    for(int j=0;j<tablesize;j++){
                        cout<<"=";
                    }

                    cout<<endl;
                }
                for(int i=0;i<32;i++){

                    cout<<" ";

                }
                cout<<"NOCS: Local Collocation Solver"<<endl;


                for(int i=0;i<1;i++){

                    for(int j=0;j<tablesize;j++){
                        cout<<"=";
                    }

                    cout<<endl;
                }

                for(int i=0;i<1;i++){

                    for(int j=0;j<tablesize;j++){
                        cout<<"*";
                    }

                    cout<<endl;
                }

                for(int i=0;i<40;i++){

                    cout<<" ";

                }
                cout<<"MESH ITERATION:" <<mesh.nIter<<endl;


                for(int i=0;i<1;i++){

                    for(int j=0;j<tablesize;j++){
                        cout<<"*";
                    }

                    cout<<endl;
                }

                for(int i=0;i<38;i++){

                    cout<<" ";

                }
                cout<<"PROBLEM INFORMATION"<<endl;
                cout<<endl;

                cout<<"-Number of states:                                             "<<nStates<<endl;
                cout<<"-Number of controls:                                           "<<nControls<<endl;
                cout<<"-Number of discrete points:                                    "<<nDiscretePoints<<endl;
                cout<<"-Collocation method:                                           "<<colMethod<<endl;
                cout<<"-Derivative method (Gradient of the cost function):            "<<derivatives.gradientCost<<endl;
                cout<<"-Derivative method (Jacobian of the constraints)  :            "<<derivatives.jacobianCns<<endl;
                cout<<endl;

                for(int i=0;i<1;i++){

                    for(int j=0;j<tablesize;j++){
                        cout<<"*";
                    }

                    cout<<endl;
                }

                for(int i=0;i<39;i++){

                    cout<<" ";

                }

                cout<<"NLP INFORMATION"<<endl;
                cout<<endl;
                cout<<"-Number of decision variables:                                 "<<NLP.nDecVar<<endl;
                cout<<"-Number of constraints:                                        "<<NLP.nCns<<endl;
                cout<<"-Number of collocation constraints:                            "<<NLP.nCollocationCns<<endl;

                cout<<endl;

                for(int i=0;i<1;i++){

                    for(int j=0;j<tablesize;j++){
                        cout<<"*";
                    }

                    cout<<endl;
                }


                for(int i=0;i<36;i++){

                    cout<<" ";

                }
                cout<<"SPARSITY INFORMATION"<<endl;
                cout<<endl;
                cout<<"-Non-zeros in the system dynamics pattern:                     "<<NLP.sparsity.nnz_Dfk<<endl;
                cout<<"-Non-zeros in the path constraints pattern:                    "<<NLP.sparsity.nnz_Dpk<<endl;
                cout<<"-Non-zeros in the initial events pattern:                      "<<NLP.sparsity.nnz_De0+NLP.sparsity.nnz_Dt0<<endl;
                cout<<"-Non-zeros in the final events pattern:                        "<<NLP.sparsity.nnz_DeF+NLP.sparsity.nnz_DtF<<endl;
                cout<<endl;
                cout<<"-Non-zero elements in the Derivative Matrix                    "<<NLP.sparsity.nnz_D<<endl;
                cout<<"-Non-zero elements in the Jacobian Matrix                      "<<NLP.sparsity.nnz_J<<endl;
                cout<<endl;


}//End printInformation

}//End nocs namespace
