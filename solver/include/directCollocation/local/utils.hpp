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

#ifndef MODULE_UTILS_H
#define MODULE_UTILS_H

#include "Eigen/Dense"
#include "iostream"

namespace nocs {

namespace utils {

    void getDecVarMatrix(const int &nx,const int &nu, const int &N, Eigen::VectorXd &z,Eigen::MatrixXd &decVarMatrix,double &t0,double &tF);

    void getVariables(const int &nx,const int &nu, const Eigen::MatrixXd &decVark, Eigen::VectorXd &xk, Eigen::VectorXd &uk,int k);

    double convert_to_original_time(double& tbar,double& t0,double &tf);

    void locateSegment(Eigen::VectorXd &time, double &t, int &k, int &k_1);

    void getTimeVector(double &t0, double &tF,Eigen::VectorXd &tauVector,Eigen::VectorXd &timeVector);

    void getTimeVector_HSS(Eigen::VectorXd &originalTime,Eigen::VectorXd &solTime);

    void sparseToDense(Eigen::MatrixXd &nnzElements, Eigen::MatrixXd &Dense);

    void getNewTauVector(Eigen::VectorXd &tauVector, Eigen::VectorXd &segmentationInfo, Eigen::VectorXd &newVector);

    void printPattern(Eigen::VectorXd &pattern);

    int testConfiguration(Eigen::VectorXd &lb, Eigen::VectorXd &ub, Eigen::VectorXd &configuration);

} //End transcription namespace

}// End OCSolver namespace


#endif
