#include "directCollocation/local/local.hpp"
#include "directCollocation/local/utils.hpp"

namespace nocs {


void localCollocation::getSolution(){

    Eigen::MatrixXd solMatrix(nStates+nControls,nCollocationPoints);
    Eigen::VectorXd z(NLP.nDecVar);

    z=this->solution.z_opt;

    //Get the decision variable matix
    nocs::utils::getDecVarMatrix(nStates,nControls,nCollocationPoints,z,
                                 solMatrix,solution.t0,solution.tF);

    //Get the time vector
    nocs::utils::getTimeVector(solution.t0,solution.tF,mesh.tauV,mesh.time);


    //Save the solution into the problem variables
    solution.xSol=solMatrix.block(0,0,nStates,nCollocationPoints);
    solution.uSol=solMatrix.block(nStates,0,nControls,nCollocationPoints);


    //If and Hermite-Simpson method is applied then find the times in the mid points
    if (colMethod=="Hermite-Simpson"){
        nocs::utils::getTimeVector_HSS(mesh.time,solution.tSol);
    } else{
        solution.tSol=mesh.time;
    }//End if-else


    //Evaluate and get the x_dot=f(x,u,t) of the solution

    Eigen::VectorXd dx(nStates);

    for(int k=0;k<nCollocationPoints;k++){

        this->dynamics(solution.xSol.col(k),solution.uSol.col(k),solution.tSol(k),dx, *this);

        solution.fSol.col(k)=dx;
    }


} //End getSolution

//-----------------------------
//------------------Romberg integration----------------
//-----------------------------

double localCollocation::errorFcn(int &nx, double &t){

      Eigen::VectorXd x_t(this->nStates);
      Eigen::VectorXd u_t(this->nControls);
      Eigen::VectorXd f_t(this->nStates);

      this->interpolateSolution(x_t,u_t,f_t,t,*this);

      Eigen::VectorXd error(this->nStates);

      Eigen::VectorXd dx(this->nStates);

      this->dynamics(x_t,u_t,t,dx,*this);

      error=dx-f_t;

      double e_k=abs(error(nx));

      return(e_k);

}//End errorFcn


    double localCollocation::integralError(double &a, double &b, int &ki){

          int N=8;

          Eigen::VectorXd h(N+1);
          Eigen::MatrixXd r(N+1,N+1);

          for(int i=1; i<N+1; i++){

              h(i)=(b-a)/pow(2,i-1);

          }//End for

          r(1,1)=(h(1)/2)*(errorFcn(ki,a)+errorFcn(ki,b));

          for(int i=2;i<N+1;i++){

              double coeff=0;

              for(int k=1;k<=pow(2,i-2);k++){

                  double x_i=a+(2*k-1)*h(i);
                  coeff+=errorFcn(ki,x_i);
              }//End for int k=1

                r(i,1)=0.5*(r(i-1,1)+h(i-1)*coeff);  //Composite Trapezoidal Rule

          }//End for int i=2

          for(int i=2;i<N+1;i++){
              for(int j=2; j<=i;j++){
                  r(i,j)=r(i,j-1)+( r(i,j-1) - r(i-1,j-1) ) / (pow(4, j-1)-1); //Richardson Extrapolation
              }//End for int j=2
          }//End for int i=2

            return r(N,N);

    }//End rombergQuadrature

    //-----------------------------
    //------------------Simpson integration----------------
    //-----------------------------

    Eigen::VectorXd localCollocation::errorFcnSimpson(double &t){

          Eigen::VectorXd x_t(this->nStates);
          Eigen::VectorXd u_t(this->nControls);
          Eigen::VectorXd f_t(this->nStates);

          this->interpolateSolution(x_t,u_t,f_t,t,*this);

          Eigen::VectorXd error(this->nStates);

          Eigen::VectorXd dx(this->nStates);

          this->dynamics(x_t,u_t,t,dx,*this);

          error=dx-f_t;

          return(error.array().abs());

    }//End errorFcn

    Eigen::VectorXd localCollocation::integralErrorSimpson(double &a, double &b){

        int n=20;

        double h= (b-a) / n; //Step size

        Eigen::VectorXd sum_odds(this->nStates);
        Eigen::VectorXd sum_evens(this->nStates);

        sum_odds.setZero();
        sum_evens.setZero();

        double t=0;

        for(int i=1; i<n; i+=2){

            t=a+i*h;
            sum_odds+=errorFcnSimpson(t);

        }

        for(int i=2; i<n; i+=2){

            t=a+i*h;
            sum_evens+=errorFcnSimpson(t);

        }

        Eigen::VectorXd integral(this->nStates);

        //Compute the value of the integral


        integral=(errorFcnSimpson(a)+errorFcnSimpson(b) + 2*sum_evens + 4 * sum_odds)* h/3.0;

        return integral;


    }

    //-----------------------------
    //------------------Mesh refinement ----------------
    //-----------------------------

    bool localCollocation::isEquidistributed(){

         bool retval = false;

        //Compute the average error

        this->mesh.epsilon_mean=this->mesh.epsilon.mean();
        this->mesh.epsilon_max=this->mesh.epsilon.maxCoeff();

        if(this->mesh.epsilon_max<=2*this->mesh.epsilon_mean){
            retval= true;
         }//End if


         return retval;

    } //End isEquidistributed


    void localCollocation::evaluateMeshError(){

        Eigen::VectorXd eta_error(this->nStates);

         for(int k=0;k<this->nDiscretePoints-1;k++){

             double tk=this->mesh.time(k);
             double tk1=this->mesh.time(k+1);

             this->mesh.eta.col(k)=integralErrorSimpson(tk,tk1);

         }//End for k*/

    }//End evaluateMeshError


    void localCollocation::evaluateSolution(){

         //Just obtain the weighting vector w the first time

         double max_xi=0;
         double max_xdoti=0;

          if(this->mesh.nIter==1){

             this->mesh.w.setZero(this->nStates); //Weighting vector

             for(int nx=0; nx<this->nStates;nx++){

                 //Obtain the maximum absolute value of the states over the N colocation points
                  max_xi=this->solution.xSol.row(nx).lpNorm<Eigen::Infinity>();

                  //Obtain the maximum absolute value of the derivatives over the N collocation points
                  max_xdoti=this->solution.fSol.row(nx).lpNorm<Eigen::Infinity>();

                  this->mesh.w(nx)=max(max_xi,max_xdoti)+1.0;

              }//End for nx

          }//End if mesh.Iter==1

         //Compute the absolute local error
          this->evaluateMeshError();

          //Compute the relative local error
          Eigen::VectorXd eta_k(this->nStates);

          for(int k=0;k<this->nDiscretePoints-1;k++){
              eta_k=this->mesh.eta.col(k).array()/this->mesh.w.array();
              this->mesh.epsilon(k)=eta_k.array().maxCoeff();
            }//End for k=0

          this->mesh.epsilon_max_history=this->mesh.epsilon.maxCoeff();

    }//End evaluateSolution


    void localCollocation::estimatePrimaryOrder(){


            if(this->colMethod=="Trapezoidal" && isEquidistributed() && this->algorithm.meshRefinement){

                this->colMethod="Hermite-Simpson";

            } else if (this->colMethod=="Trapezoidal" && this->mesh.nIter>1 && this->algorithm.meshRefinement){

                this->colMethod="Hermite-Simpson";
            }

    }//End estimatePrimaryOrder


    void localCollocation::estimateOrderReduction(){

            // This function estimates the local order reduction for the local mesh refinement algorithm.
            // Reference: Betts (2010)

            double eta;
            double theta;

            //Change the order of the method

            if(this->colMethod=="Trapezoidal"){
                this->mesh.p=2;
            } else if (this->colMethod=="Hermite-Simpson"){
                this->mesh.p=4;
            }// End if

            if(this->mesh.nIter==1){ //Zero order reduction

                this->mesh.r.setZero();

            } else { //Order reduction

                //Compute the order reduction for every segment

                int j=0;

                int Ik=1;

                double rk=0;
                double rk_min=0;
                double rk_hat=0;


                for(int k=0;k<this->mesh.epsilon_old.size();k++){

                    theta=this->mesh.epsilon_old(k);
                    Ik=this->mesh.I_old(k);


                     for(int i=1; i<=Ik;i++){

                         eta=this->mesh.epsilon(j);

                         rk_hat=this->mesh.p + 1 - log(theta/eta) / log(1.0+i);

                         rk_min=min((int)round(rk_hat),this->mesh.p);
                         rk=max(0,(int)rk_min);

                         this->mesh.r(j)=rk;

                         j++;

                     }// End for (int i=0...;)

                }//End for (int k=0...)

            }//End if

            //Update the old grid variables
            //this->mesh.epsilon_old.resize(this->nDiscretePoints-1);
            this->mesh.epsilon_old=this->mesh.epsilon;

            //this->mesh.tauV_old.resize(this->nDiscretePoints-1);
            this->mesh.tauV_old=this->mesh.tauV;

    }//End estimateOrderReduction


    void localCollocation::constructNewMesh(){

            int nSegments=this->nDiscretePoints-1;

            //Start with the construction of the new mesh by adding the points to the segments
            //with bigger errors only if one of the termination criteria is not satisfied

            bool terminate=false;
            int added_points=0;
            double epsilon_max; //Value of the maximum segmental error
            int maxSegment;     //Index of the segment with maximum error
            double maxPoints= this->algorithm.maxPoints;
            double kappa= this->algorithm.kappa;

            Eigen::VectorXd I(nDiscretePoints-1);
            I.setZero();

            while (!terminate) {

                epsilon_max=this->mesh.epsilon.maxCoeff(&maxSegment); //Obtain the value and index of the maximum error


                //1st termination criteria: M points have beed added and the error is within tolerance and I_alpha=0;

                if((added_points>min(maxPoints,kappa*this->nDiscretePoints)) && (epsilon_max <=this->algorithm.error_ode) && (I(maxSegment)==0)){
                    terminate=true;
                    cout<<"End mesh refinement in the 1st termination criteria"<<endl;
                }

                //2nd termination criteria: The predicted error is safely within tolerance and 0<I_alpha<maxPoints

                if((epsilon_max<kappa*this->algorithm.error_ode) && (I(maxSegment)>0) && (I(maxSegment)<maxPoints)){
                   terminate=true;
                   cout<<"End mesh refinement in the 2nd termination criteria"<<endl;
                }

                //3rd termination criteria: nDiscretePoints-1 points have been added (This conditions avoid the duplication of the size of the mesh)

                if(added_points >= int(nSegments)){
                    terminate=true;
                    cout<<"End mesh refinement in the 3rd termination criteria"<<endl;
                }
                //4th termination criteria: Any of the segments already have the max number of points

                if((I.array()==maxPoints).any()){
                    terminate=true;
                    cout<<"End mesh refinement in the 4th termination criteria"<<endl;
                }

                //If no termination criteria is accepted add a point to the max segment
                if(!terminate){

                    I(maxSegment)+=1;
                    added_points++;

                    //Now compute the predicted error given the new added point

                    double x1=1.0/(1.0+ I(maxSegment));
                    double x2=mesh.p-mesh.r(maxSegment)+1.0;

                    this->mesh.epsilon(maxSegment)=epsilon_max*pow(x1,x2);

                    //fprintf(stderr,"Added points to segment=%i, old error=%f, new error=%f , I(maxSegment)=%f \n",maxSegment,epsilon_max,mesh.epsilon(maxSegment), I(maxSegment));


                }//End if

            }//End while

            //Save the information of the added points

            this->mesh.I_old.setZero(nDiscretePoints-1);
            this->mesh.I_old=I;

            //Now construct the new mesh with the added points

            this->nDiscretePoints+=(int)I.sum();   

            fprintf(stderr,"Added %i points to the mesh in refinement iteration %i \n",(int)I.sum(),this->mesh.nIter);

            Eigen::VectorXd newTauVector(this->nDiscretePoints);

            utils::getNewTauVector(this->mesh.tauV,I,newTauVector);

            //Update the tau vector
            this->mesh.tauV.resize(this->nDiscretePoints);
            this->mesh.tauV=newTauVector;


    }//End constructNewMesh


    //------------------------------------------------------------
    //------------------ Dircol i5 mesh refinement----------------

    void localCollocation::constructMesh(){

        //3.Change the order of the method

        if(this->colMethod=="Trapezoidal"){
            this->mesh.p=2;
        } else if (this->colMethod=="Hermite-Simpson"){
            this->mesh.p=4;
        }// End if

        //4. Estimate how many sub-divisions are required in each segment

        double tolTarget=this->algorithm.kappa*this->algorithm.error_ode;
        double order=-1.0/this->mesh.p; //Order of the collocation method

        Eigen::VectorXd I(nDiscretePoints-1);
        I.setZero();

        double I_k=0;

        for(int k=0;k<this->nDiscretePoints-1;k++){
            I_k=pow(tolTarget/this->mesh.epsilon(k),order);
             //Nota: Dar prioridad a los de mayor error dentro del mesh
             //      Limitar el numero de puntos que se agregan al mesh a 2M-1

            if(I_k<1){ // Add one point to the error

                I(k)=0;

            } else if (I_k>this->algorithm.maxPoints){ //Limit the number of points added to a single interval

                I(k)=this->algorithm.maxPoints;

            } else{

                I(k)=round(I_k);
            }

          }//End for k=0

        //5.Save the information of the added points

        this->mesh.I_old.setZero(nDiscretePoints-1);
        this->mesh.I_old=I;

        //6. Now construct the new mesh with the added points

        this->nDiscretePoints+=(int)I.sum();

        fprintf(stderr,"Added %i points to the mesh in refinement iteration %i \n",(int)I.sum(),this->mesh.nIter);

        Eigen::VectorXd newTauVector(this->nDiscretePoints);

        utils::getNewTauVector(this->mesh.tauV,I,newTauVector);

        //Update the tau vector
        this->mesh.tauV.resize(this->nDiscretePoints);
        this->mesh.tauV=newTauVector;


    }//End constructMesh

    void localCollocation::evalNLPSolution(){


        //Just obtain the weighting vector w the first time

        double max_xi=0;
        double max_xdoti=0;

         if(this->mesh.nIter==1){

            this->mesh.w.setZero(this->nStates); //Weighting vector

            for(int nx=0; nx<this->nStates;nx++){

                //Obtain the maximum absolute value of the states over the N colocation points
                 max_xi=this->solution.xSol.row(nx).lpNorm<Eigen::Infinity>();

                 //Obtain the maximum absolute value of the derivatives over the N collocation points
                 max_xdoti=this->solution.fSol.row(nx).lpNorm<Eigen::Infinity>();

                 this->mesh.w(nx)=max(max_xi,max_xdoti)+1.0;

             }//End for nx

         }//End if mesh.Iter==1

        //1. Evaluate the mesh error

        this->evaluateMeshError();

        //2. Select the max error in each segment
        Eigen::VectorXd eta_k(this->nStates);

        for(int k=0;k<this->nDiscretePoints-1;k++){
            eta_k=this->mesh.eta.col(k).array()/this->mesh.w.array();
            this->mesh.epsilon(k)=eta_k.array().maxCoeff();
          }//End for k=0

        this->mesh.epsilon_max_history=this->mesh.epsilon.maxCoeff();

    }

}//End OCSolver namespace

