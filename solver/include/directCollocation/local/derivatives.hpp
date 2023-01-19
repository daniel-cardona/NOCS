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

#ifndef DERIVATIVES_LOCAL_H
#define DERIVATIVES_LOCAL_H

#include "directCollocation/local/local.hpp"
#include "directCollocation/local/userFunctions.hpp"

#include "iostream"
#include "Eigen/Dense"
#include "Eigen/Sparse"


#ifndef MAX
#define MAX(a, b) ( (a)>(b)?  (a):(b) )
#endif

#ifndef MIN
#define MIN(a, b) ( (a)<(b)?  (a):(b) )
#endif

namespace nocs{

//! Compute a column of the jacobian of the dynamics w.r.t. the decision variables using finite differences.
/*! \param localCollocation object
 *  \param Value of the decision variables
 *  \param Number of column of the jacobian
/*! \return Column of the jacobian of the dynamics*/

void dxJacColumn(localCollocation &problem, Eigen::VectorXd &x,int jCol,Eigen::VectorXd &jacColumn);

//! Compute a column of the jacobian of the path constrints w.r.t. the decision variables using finite differences.
/*! \param localCollocation object
 *  \param Value of the decision variables
 *  \param Number of column of the jacobian
/*! \return Column of the jacobian of the path constraints*/

void pathJacColumn(localCollocation &problem, Eigen::VectorXd &x,int jCol,Eigen::VectorXd &jacColumn);

//! Compute a column of the jacobian of the initial event constraints w.r.t. the decision variables using finite differences.
/*! \param localCollocation object
 *  \param Value of the decision variables
 *  \param Number of column of the jacobian
/*! \return Column of the jacobian of the event constraints*/

void eventJacColumn_t0(localCollocation &problem, Eigen::VectorXd &x,int jCol,Eigen::VectorXd &jacColumn);

//! Compute a column of the jacobian of the final event constraints w.r.t. the decision variables using finite differences.
/*! \param localCollocation object
 *  \param Value of the decision variables
 *  \param Number of column of the jacobian
/*! \return Column of the jacobian of the event constraints */

void eventJacColumn_tf(localCollocation &problem, Eigen::VectorXd &x,int jCol,Eigen::VectorXd &jacColumn);

}//End nocs namespace


#endif // DERIVATIVES_LOCAL_H




