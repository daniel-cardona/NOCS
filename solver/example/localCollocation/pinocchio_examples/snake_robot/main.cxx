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


#include "directCollocation/local/nocsLocal.hpp"

#include "directCollocation/local/fcnGenerator.hpp"

#include <iostream>
#include <chrono>

#include <iostream>
#include <experimental/filesystem>

namespace fs = std::experimental::filesystem;


#ifndef URDF_MODEL_DIR
#define URDF_MODEL_DIR "./urdf/"
#endif


int countFilesInFolder(const std::string& folderPath, std::vector<std::string> &files)
{
    int count = 0;
    for (auto& file : fs::directory_iterator(folderPath))
    {
        if (fs::is_regular_file(file))
        {
            files.push_back(file.path().filename().string());
            count++;
        }
    }

    std::sort(files.begin(), files.end());

    return count;
}

std::vector<int> read_file_to_vector(std::string filename) {
    // Create a vector to hold the lines of the file
    std::vector<int> nDOF;

    // Open the file for reading
    std::ifstream file(filename);

    // Read the file line by line
    std::string line;
    nDOF.push_back(0);

    while (std::getline(file, line)) {
        // Add each line to the vector
        nDOF.push_back(std::stoi(line));
    }

    // Close the file
    file.close();

    // Return the vector of lines
    return nDOF;
}

int main(){

    std::vector<std::string> urdf_names;
    std::vector<int> nDOF_list;

    int nURDF=countFilesInFolder(URDF_MODEL_DIR,urdf_names);

    std::string data_file=URDF_MODEL_DIR +urdf_names.at(0);

    nDOF_list=read_file_to_vector(data_file);

    int nDiscretePoints=15;

    char fileName[50];

    int n=sprintf(fileName,"./results_%02d_SFD.txt",nDiscretePoints);
    std::ofstream outfile(fileName);


    for(int i=1; i<nURDF; i++){

         const std::string urdf_filename = URDF_MODEL_DIR + urdf_names.at(i);
         std::cout<<urdf_filename<<" nDOF: "<<nDOF_list.at(i) <<std::endl;


         int nStates=nDOF_list.at(i)*2;
         int nControls=nDOF_list.at(i);
         int nEvents=nStates*2;
         int nPath=0;



         nocs::localCollocation problem(nStates,nControls,nDiscretePoints,nPath,nEvents,urdf_filename);


         int nDoF=problem.model.nq;

         problem.bounds.states.lower.head(nDoF)=problem.model.lowerPositionLimit;
         problem.bounds.states.upper.head(nDoF)=problem.model.upperPositionLimit;

         problem.bounds.states.lower.tail(nDoF)=Eigen::VectorXd::Ones(nDoF)*-1;
         problem.bounds.states.upper.tail(nDoF)=Eigen::VectorXd::Ones(nDoF)*1;

         //------------Torque limits --------------------


         problem.bounds.controls.lower=Eigen::VectorXd::Ones(nControls)*-1000;
         problem.bounds.controls.upper=Eigen::VectorXd::Ones(nControls)*1000;



         //-----------Initial and final time bounds ---------

         //Start at t=0
         problem.bounds.initialTime.lower(0)=0;
         problem.bounds.initialTime.upper(0)=0;

         //Finished at tF=15;
         problem.bounds.finalTime.lower(0)=10.0;
         problem.bounds.finalTime.upper(0)=10.0;

         //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
         //&---------------Boundary and path constraints-------------------&
         //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

         Eigen::VectorXd initial_configuration(problem.model.nq);
         Eigen::VectorXd end_configuration(problem.model.nq);

         initial_configuration.setZero();
         end_configuration.setRandom();

        problem.bounds.events.lower.setRandom();
        problem.bounds.events.upper.setRandom();

        //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
        //&---------Set the collocation information of the problem---------------&
        //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

        problem.colMethod                    ="Hermite-Simpson";
        problem.derivatives.gradientCost     ="Analytical";
        problem.derivatives.jacobianCns      ="Numerical";

        //Set the problem depending on the local method selected

        problem.setupNLP();

        //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
        //&---------------------Set initial guess-------------------------&
        //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

        problem.guess.states.setZero();

        //Set a linear distribution in the joint position states

        problem.guess.states.setRandom();
        problem.guess.controls.setZero();
        problem.guess.t0=0;
        problem.guess.tF=5;

        //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
        //&---------------------Algorithm options-------------------------&
        //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&



        problem.algorithm.meshRefinement   =true;
        problem.algorithm.derivativeChecker=false;

        problem.algorithm.kappa=0.1;       //Default is 1/10 [Betts,2014]
        problem.algorithm.error_ode=1e-2;
        problem.algorithm.maxPoints=4;
        problem.algorithm.maxMeshIterations=1;


        int status=problem.solve();

        Eigen::VectorXd z(problem.NLP.nDecVar);
        Eigen::VectorXd nz_elements(problem.NLP.sparsity.nnz_J);

        const int nTest=10;

        long dur=0;

        long durVec[nTest];

        for(int i=0;i<nTest;i++){

            z.setRandom();

            auto start=std::chrono::high_resolution_clock::now();

            problem.cnsJacobian(problem,z,nz_elements);

            auto stop = std::chrono::high_resolution_clock::now();

            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count();

            durVec[i]=duration;

            dur+=duration;

        }//END FOR------------------MULTIPLE ITERATIONS OF THE ALGORITHM

        double mean_duration=dur/nTest;
        outfile<<mean_duration<<std::endl;



}


    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    //&-------------------Set and solve the problem------------------&
    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&




    return 0;

}
