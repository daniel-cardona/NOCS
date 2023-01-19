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


#include "directCollocation/local/utils.hpp"

namespace nocs {

namespace utils {

    void getDecVarMatrix(const int &nx,const int &nu, const int &N, Eigen::VectorXd &z,Eigen::MatrixXd &decVarMatrix,double &t0,double &tF){

      //Recover and return the initial and final time

        t0=z(0);
        tF=z(1);

        //Now recover the matrix with the following structure:
        //[xk xk+1 xk+2 ...xM
        // uk uk+1 uk+2 ...uM]

        Eigen::Map<Eigen::MatrixXd> decVar(z.tail(z.size()-2).data(),nx+nu,N);

        //Return this!
        decVarMatrix=decVar;


    }

    void getVariables(const int &nx,const int &nu, const Eigen::MatrixXd &decVark, Eigen::VectorXd &xk, Eigen::VectorXd &uk,int k){


    //Obtain the states and the controls in k

    xk=decVark.col(k).head(nx).array();

    uk=decVark.col(k).segment(nx,nu).array();


    }

    double convert_to_original_time(double& tbar,double& t0,double &tf){

        double retval= (tf-t0)/2.0 + (tf-t0)*tbar/2.0;

        return retval;
    }

    void locateSegment(Eigen::VectorXd &time, double &t, int &kl, int &kr){

        //This function obtains the segment where the x(t) variable is located using bisection
            //Reference: Numerical Recipes in C, H. Press. pp. 117

            int k_left,k_right;
            int k;
            k_left=0;
            k_right=time.size()-1;

            while(k_right-k_left>1){

                k=(int)((k_right+k_left)/2);
                if (time(k)>t) k_right=k;
                else k_left=k;
            }

            //Return this

            kr=k_right;
            kl=k_left;

    }

    void getTimeVector(double &t0, double &tF,Eigen::VectorXd &tauVector,Eigen::VectorXd &timeVector){

        timeVector.setZero(tauVector.size());

        for(int k=0;k<tauVector.size();k++){

            timeVector(k)=(tF-t0)*tauVector(k);

        }

    }//End getTimeVector

    void getTimeVector_HSS(Eigen::VectorXd &originalTime,Eigen::VectorXd &solTime){

        solTime.setZero();

        Eigen::Vector2d segment;

        for(int k=0; k<originalTime.size()-1; k++){

            double tk=originalTime(k);
            double tk1=originalTime(k+1);

            double hk_mid=(tk1-tk)/2;

            double tk_mid=tk+hk_mid;

            segment<<tk,tk_mid;

            solTime.segment(k*2,2)=segment;
        }

        //Add in the final time element

        solTime.tail(1)=originalTime.tail(1);

    }//End getTimeVectorHSS


    void sparseToDense(Eigen::MatrixXd &nnzElements, Eigen::MatrixXd &Dense){

        int nnz=nnzElements.rows();
        int row=0;
        int col=0;

        Dense.setZero();

        for(int i=0;i<nnz;i++){

            row=nnzElements(i,0);
            col=nnzElements(i,1);
            Dense(row,col)=1;
        }
    }//End sparseToDense

    void getNewTauVector(Eigen::VectorXd &tauVector, Eigen::VectorXd &segmentationInfo, Eigen::VectorXd &newVector){

         double low=0;
         double high=0;


         int nSegments=segmentationInfo.size();
         int newSegments=0;
         double deltaTau;

         int innerIteratot=0;

         Eigen::VectorXd dividedSegment;

         for (int k=0;k<nSegments;k++){

             low=tauVector(k);
             high=tauVector(k+1);

             newSegments=segmentationInfo(k)+1;
             deltaTau=(high-low)/newSegments;
             newVector(innerIteratot)=low;

             for(int i=0; i<newSegments;i++){
                innerIteratot++;
                newVector(innerIteratot)=newVector(innerIteratot-1)+deltaTau;
              }//End for int i =0

                newVector(innerIteratot)=high;

            }//End for int k=0

    }// End getNewTauVector

    int testConfiguration(Eigen::VectorXd &lb, Eigen::VectorXd &ub, Eigen::VectorXd &configuration){


        for(int i=0; i<lb.rows(); i++){

            if((configuration(i)>=lb(i)) && (configuration(i)<=ub(i))){
                continue;
            } else{
                std::cout<<lb(i)<<"<"<< configuration(i) << "<" << ub(i)<<std::endl;
              return -1;
            }
        }//End for

        return 0;

    }//End testConfiguration

} //End utils namespace

}//End nocs namespace
