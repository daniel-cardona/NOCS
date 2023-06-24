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


#define MAXBUFSIZE  ((int) 1e6)

#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Dense>

#define multiPhase


extern "C"{

    #include "remoteApi/extApi.h"
    #include "remoteApi/extApiInternal.h"
    #include "remoteApi/extApiPlatform.h"
    #include "remoteApi/simConst.h"
}

using namespace std;
using namespace Eigen;


MatrixXd readMatrix(std::string filename)
    {
    int cols = 0, rows = 0;
    double buff[MAXBUFSIZE];

    // Read numbers from file into buffer.
    ifstream infile;
    infile.open(filename);
    while (! infile.eof())
        {
        string line;
        getline(infile, line);

        int temp_cols = 0;
        stringstream stream(line);
        while(! stream.eof())
            stream >> buff[cols*rows+temp_cols++];

        if (temp_cols == 0)
            continue;

        if (cols == 0)
            cols = temp_cols;

        rows++;
        }

    infile.close();

    rows--;

    // Populate matrix with numbers.
    MatrixXd result(rows,cols);
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            result(i,j) = buff[ cols*i+j ];

    return result;
    };

int main(){

    bool remoteApi= true;


#ifndef multiPhase

    std::string fileName="/home/sebastian/Documents/Phd/Resultados_paper/POSE6/xSolution.txt";

    Eigen::MatrixXd xSol;

    xSol=readMatrix(fileName);
    int nDiscretePoints=xSol.cols();

    Eigen::MatrixXd q(24,nDiscretePoints);

    q=xSol.topRows(24);
#else

    std::string fileName1="/home/sebastian/Documents/Phd/Resultados_paper/POSE1/xSolution.txt";
    std::string fileName2="/home/sebastian/Documents/Phd/Resultados_paper/POSE5/xSolution.txt";

    Eigen::MatrixXd xSol_phase1;
    Eigen::MatrixXd xSol_phase2;

    xSol_phase1=readMatrix(fileName1);
    xSol_phase2=readMatrix(fileName2);

    int nDiscretePoints=xSol_phase1.cols()+xSol_phase2.cols();//+xSol_phase3.cols();

    Eigen::MatrixXd xSol(xSol_phase2.rows(),nDiscretePoints);

    xSol<<xSol_phase1,xSol_phase2;//,xSol_phase3;

    Eigen::MatrixXd q(24,nDiscretePoints);

    q=xSol.topRows(24);


#endif



    int nDoF=q.rows();

    if(nDoF!=24)
        return -1;

    if(remoteApi){

        int portNb=19997;
        int clientID= simxStart("127.0.0.1",portNb,true,true,2500,5);


        std::vector <int> IntHandle(24);
        std::vector <int> FeasibleHandle(24);
        int FeasabilitySum=0;

        std::vector< const simxChar* > CharHandle{"LAnkleRoll3","LAnklePitch3","LKneePitch3","LHipPitch3","LHipRoll3","LHipYawPitch3",
                                                           "LShoulderPitch3","LShoulderRoll3","LElbowYaw3","LElbowRoll3","LWristYaw3",
                                                           "HeadYaw","HeadPitch",
                                                           "RShoulderPitch3","RShoulderRoll3","RElbowYaw3","RElbowRoll3","RWristYaw3",
                                                           "RHipYawPitch3","RHipRoll3","RHipPitch3","RKneePitch3","RAnklePitch3","RAnkleRoll3"};

        for(short int id=0; id<nDoF ; id++){

            FeasibleHandle.at(id)=simxGetObjectHandle(clientID,CharHandle.at(id),&IntHandle.at(id),simx_opmode_blocking);
            FeasabilitySum+=FeasibleHandle.at(id);

        }// End for(short int id=0)

        if(clientID == -1 || FeasabilitySum){

            cout<<"Coppelia connection failed "<<endl;

            simxFinish(clientID);

        } else{

            cout<<"Coppelia succesfully conected remote API server"<<endl;

        }//End if-else(clientID)

        simxSynchronous(clientID,true);


        //Streaming


        for(int i=0; i<nDiscretePoints; i++){

            simxPauseCommunication(clientID,true);

            for(short int id=0; id<nDoF ;id++){

                simxSetJointTargetPosition(clientID,IntHandle.at(id),xSol(id,i),simx_opmode_oneshot);

            }

            simxPauseCommunication(clientID,false);
            simxSynchronousTrigger(clientID);
        }

        simxFinish(clientID);

        cout<<"Streaming finished"<<endl;

    } else{

        cout<<"Hola que hace?"<<endl;

    }//End if-else

    return 0;

}
