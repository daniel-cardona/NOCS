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

#ifndef USER_FCN_MODULE
#define USER_FCN_MODULE

#include "directCollocation/local/local.hpp"

namespace nocs{

namespace Function{

//! User defined function that must return the Mayer cost(if defined) fo the objective function
    //!
    //! This function is stored in: src/examples/.../OCP_definition.cpp
    //!
    /*! \param $x0$      Vector with initial states
     *  \param $xF$      Vector with final states
     *  \param $t0$      Initial time
     *  \param $tF$      Final time
     *
     *  \return Mayer cost
     */

    double endpoint_cost(const Eigen::VectorXd &x0,const Eigen::VectorXd &xF, const double &t0,const double &tf,localCollocation &problem);


    //! User defined function that must return the Lagrange cost(if defined) of the objective function
    //!
    //! This function is stored in: src/examples/.../OCP_definition.cpp
    //!
    /*! \param $states$      Vector with states in tk
     *  \param $controls$    Vector with controls in tk
     *  \param $tk$          Time at the k discretization node
     *  \param $w$           Weight vector
     *
     *  \return Lagrange cost
     */

    double integrand_cost(const Eigen::VectorXd &states, const Eigen::VectorXd &controls, const double &tk, localCollocation &problem);


    //! User defined function that must return the derivatives of the state vector(this is generally defined by the dynamics of the system)
    //! and the path constraints
    //!
    //! This function is stored in: src/examples/.../OCP_definition.cpp
    //!
    /*! \param $states$         Vector with states in tk
     *  \param $controls$       Vector with controls in tk
     *  \param $tk$             Time at the k discretization node
     *
     *  \return $derivatives$   Vector with the derivative of the states at tk
     */

    void dae(const Eigen::VectorXd &states, const Eigen::VectorXd &controls,const double &tk, Eigen::VectorXd &derivatives, localCollocation &problem);


    //! User defined function that must return the value of the path constraints (if any)
    //!
    //! This function is stored in: src/examples/.../OCP_definition.cpp
    //!
    /*! \param $states$         Vector with states in tk
     *  \param $controls$       Vector with controls in tk
     *  \param $tk$             Time at the k discretization node
     *
     *  \return $path$          Vector with the values of the path contraints at tk
     */

    void path(const Eigen::VectorXd &states, const Eigen::VectorXd &controls,const double &tk,Eigen::VectorXd &path,localCollocation &problem);


    //! User defined function that must return a vector with the value of the event constraints
    //!
    //! This function is stored in: src/examples/.../OCP_definition.cpp
    //!
    /*! \param $x0$      Vector with initial states
     *  \param $xF$      Vector with final states
     *  \param $t0$      Initial time
     *  \param $tF$      Final time
     *
     *  \return $e$   Vector with the value of the event constraints
     *
     */

    void events(const Eigen::VectorXd &x0, const Eigen::VectorXd &xF, const double &t0, const double &tF, Eigen::VectorXd &e,localCollocation &problem);


    namespace analytical {

    //! User defined function that must return the value of gradient of the costFunction w.r.t. the states and controls
    //!
    //! This function is stored in: src/examples/.../functions.cpp
    //!
    /*! \param $states$         Vector with states in tk
     *  \param $controls$       Vector with controls in tk
     *  \param $tk$             Time at the k discretization node
     *
     *  \return $Gradient of the dynamics w.r.t. states and controls$
     */

    void costGradient(const Eigen::VectorXd &x, const Eigen::VectorXd &u,const double &t, Eigen::VectorXd &gradient, localCollocation &problem);

    //! User defined function that must return the value of the Jacobian of the dynamics w.r.t. the states and controls
    //!
    //! This function is stored in: src/examples/.../functions.cpp
    //!
    /*! \param $states$         Vector with states in tk
     *  \param $controls$       Vector with controls in tk
     *  \param $tk$             Time at the k discretization node
     *
     *  \return $Jacobian of the dynamics w.r.t. states and controls$
     */

    void fGradient( const Eigen::VectorXd &states, const Eigen::VectorXd &controls,double &t, Eigen::MatrixXd &gradient, localCollocation &problem);

    //! User defined function that must return the value of the Jacobian of the path constraints w.r.t. the states and controls
    //!
    //! This function is stored in: src/examples/.../functions.cpp
    //!
    /*! \param $states$         Vector with states in tk
     *  \param $controls$       Vector with controls in tk
     *  \param $tk$             Time at the k discretization node
     *
     *  \return $Jacobian of the path constraints w.r.t. states and controls$
     */

    void pathGradient(Eigen::VectorXd &states, Eigen::VectorXd &controls,double &t, Eigen::MatrixXd &gradient, localCollocation &problem);

    //! User defined function that must return the value of the Jacobian of the event constraints wrt to initial and final states and times
    //!
    //! This function is stored in: src/examples/.../functions.cpp
    //!
    /*! \param $states$         Vector with states in tk
     *  \param $controls$       Vector with controls in tk
     *  \param $tk$             Time at the k discretization node
     *
     *  \return $Jacobian of the event constraints wrt to initial and final states and times$
     */

    void eventGradient(Eigen::VectorXd &x0,Eigen::VectorXd &xN,double &t0,double &tF,Eigen::VectorXd &Dt0, Eigen::VectorXd &DtF,Eigen::MatrixXd &De0, Eigen::MatrixXd &DeF, localCollocation &problem);


    }//End analytical namespace

}//End Function namespace

}//End OCSolver namespace

#endif




