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

#ifndef GENERATOR_MODULE
#define GENERATOR_MODULE

#ifndef MC_EPSILON
#define MC_EPSILON 2.221e-16
#endif

#include "directCollocation/local/local.hpp"

#include "directCollocation/local/utils.hpp"

#include "directCollocation/local/userFunctions.hpp"


//Function generator

namespace nocs {

namespace localGenerator{


    //! Compute the cost function value using the quadrature weights defined by the collocation method
    /*! \return value of the cost function */
    double costFcn(localCollocation &problem, Eigen::VectorXd &z);

    double costFunction(localCollocation &problem,Eigen::VectorXd &z);


    //! Compute the vector of constraints exploiting the right hand side of the collocation method.
    /*! \return void */
    void rightHandSideSparsity(localCollocation &problem, Eigen::VectorXd &z, Eigen::VectorXd &c);


    //! Get the states and control variables at certain time by using interpolation techniques (See [Kelly,2017])
    /*! \return void */
    void interpolateSolution(Eigen::VectorXd &x_t, Eigen::VectorXd &u_t,Eigen::VectorXd &f_t, double &t, localCollocation &problem);


    //-------------------------------------------------------------------------
    //-------------Deprecated functions (Here just for reference)
    //-------------------------------------------------------------------------

    //! Compute the vector of constraints using trapezoidal collocation methods (Classic way)
    /*! \return void */
    void trapCnsFunction(localCollocation &problem, Eigen::VectorXd &z, Eigen::VectorXd &c);

    //! Compute the vector of constraints using Hermite-Simpson collocation methods (Classic way)
    /*! \return void */
    void hsCnsFunction(localCollocation &problem, Eigen::VectorXd &z, Eigen::VectorXd &c);

namespace derivatives{

    //! Compute the analytical Jacobian of the constraints using the sparse block propagation algorithms.
    /*! \return void */

    void computePropagatedJacobian(localCollocation &problem,Eigen::VectorXd &x,Eigen::VectorXd &nzValues);

    namespace numerical{

        //! Compute the gradient of the cost function using finite differences.
        /*! \return void */
        void computeGradientCost(localCollocation &problem, Eigen::VectorXd &z, Eigen::VectorXd &grad);

        //-------------------------------------------------------------------------
        //-------------Deprecated functions (Here just for reference)
        //-------------------------------------------------------------------------

        //! Compute the vector of non-linear terms q(z). See[Cardona et al, 2019]
        /*! \return void */
        void obtainNonLinearVector(localCollocation &problem, Eigen::VectorXd &z, Eigen::VectorXd &q);


        //! Compute jacobian of the non-linear terms using sparse finite differences methods.
        /*! \return void */
        void computeJacobian(localCollocation &problem,Eigen::VectorXd &z,Eigen::VectorXd &nzValues);

    } //End numerical namespace

    namespace analytical {

        //! Compute the gradient of the cost function in an analytical way.
        /*! \return void */
        void computeGradientCost(localCollocation &problem, Eigen::VectorXd &z, Eigen::VectorXd &grad);

        //-------------------------------------------------------------------------
        //-------------Deprecated functions (Here just for reference)
        //-------------------------------------------------------------------------

        //! Compute the jacobian of the constraints using the block matrices method (Old version algorithm)
        /*! \return void */
        void computeAnalyticalJacobian(localCollocation &problem,Eigen::VectorXd &z,Eigen::VectorXd &nzValues);


    } //End namespace analytical



}//End derivatives namespace

}//End localGenerator

}//end nocs namespace




#endif //End GENERATOR_MODULE
