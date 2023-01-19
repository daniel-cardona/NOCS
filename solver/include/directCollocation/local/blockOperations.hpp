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

#ifndef BLOCKOPERATIONS_LOCAL_H
#define BLOCKOPERATIONS_LOCAL_H

#include "directCollocation/local/local.hpp"

#include "iostream"
#include "Eigen/Dense"
#include "Eigen/Sparse"

namespace nocs{

//! Detect the sparsity pattern of the dynamics of the system using a finite differences method.
/*! \param localCollocation object
/*! \return Sparsity pattern of the dynamics of the system*/

void detectSparsity_dx(localCollocation &problem, Eigen::MatrixXd &sparsityTemplate);

//! Detect the sparsity pattern of the path constraints using a finite differences method.
/*! \param localCollocation object
/*! \return Sparsity pattern of the path constraints of the problem*/

void detectSparsity_path(localCollocation &problem, Eigen::MatrixXd &sparsityTemplate);

//! Detect the sparsity pattern of the path constraints using a finite differences method.
/*! \param localCollocation object
/*! \return Sparsity pattern of the events constraints of the problem*/

void detectSparsity_events(localCollocation &problem,Eigen::MatrixXd &template_t0,Eigen::MatrixXd &template_e0,Eigen::MatrixXd &template_tF ,Eigen::MatrixXd &template_eF);

//! Computes the value of the Jacobian of the dynamics w.r.t. the states and controls
//!
/*! \param $states$         Vector with states in tk
 *  \param $controls$       Vector with controls in tk
 *  \param $tk$             Time at the k discretization node
 *  \param $problem$        Local collocation object
 *
 *  \return $Jacobian of the dynamics w.r.t. states and controls$
 */
void computeNumericalJacobian_dx(const Eigen::VectorXd &states, const Eigen::VectorXd &controls, double &t,Eigen::MatrixXd &gradient, localCollocation &problem);

//! Computes the value of the Jacobian of the path constraints w.r.t. the states and controls
//!
/*! \param $states$         Vector with states in tk
 *  \param $controls$       Vector with controls in tk
 *  \param $tk$             Time at the k discretization node
 *  \param $problem$        Local collocation object
 *
 *  \return $Jacobian of the path constraints w.r.t. states and controls$
 */
void computeNumericalJacobian_path(Eigen::VectorXd &states, Eigen::VectorXd &controls, double &t,Eigen::MatrixXd &gradient, localCollocation &problem);

//! Computes the Jacobian of the event constraints wrt to initial and final states and times
//!
//!
/*! \param $x0$             Vector with states at t0
 *  \param $xN$             Vector with states at tF
 *  \param $t0$             Value of the initial time variable
 *  \param $tF$             Value of the final time variable
 *  \param $problem$        Local collocation object
 *
 *  \return $Jacobian of the event constraints wrt to initial and final states and times$
 */

void computeNumericalJacobian_event(Eigen::VectorXd &x0,Eigen::VectorXd &xN,double &t0,double &tF,Eigen::VectorXd &Dt0, Eigen::VectorXd &DtF,Eigen::MatrixXd &De0, Eigen::MatrixXd &DeF, localCollocation &problem);


}//End nocs namespace




#endif // SPARSITY_LOCAL_H




