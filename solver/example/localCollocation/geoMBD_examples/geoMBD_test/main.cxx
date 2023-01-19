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



#include <iostream>
#include <chrono>

#include "directCollocation/local/nocsLocal.hpp"


#define __FU_PATH_PREFIX__ "../../../../../data/pinocchio_models"
//std::string urdf_dir = __FU_PATH_PREFIX__ "nao_inertial_XYZ_python.urdf"; //24


int main(){


  const std::string urdf_filename = __FU_PATH_PREFIX__ + std::string("/HRP2.urdf");

  auto robot = Robot::build_model(urdf_filename);

    //! Forward dynamics object pointer
    auto robotDynamics = std::make_shared< geoCRTP::FwdDynCRTP< DataType > >( robot.value() );

    int n = robot.value()->nq;

    Eigen::VectorXd q;
    Eigen::VectorXd dq;
    Eigen::VectorXd tau;

    q   = Eigen::VectorXd::LinSpaced(n, 1, 2*3.1416);
    dq  = Eigen::VectorXd::LinSpaced(n, 1, 2*3.1416);
    tau = Eigen::VectorXd::LinSpaced(n, 1, 2*3.1416);


    robotDynamics->aba(q.derived(),dq.derived(),tau.derived());


    cout<<robotDynamics->ddq<<endl;


  return 0;

}

