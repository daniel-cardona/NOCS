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

#ifndef DYN_HANDLER
#define DYN_HANDLER

#include "directCollocation/local/local.hpp"

namespace nocs{

#ifdef pinocchio_compile

namespace pinocchioLib {

    //! Compute the derivatives of the state vector using the geoMBD library
    //!
    /*! \param $states$         Vector with states in tk
     *  \param $controls$       Vector with controls in tk
     *  \param $tk$             Time at the k discretization node
     *
     *  \return $derivatives$   Vector with the derivative of the states at tk
     */

    void autoGenDynamics(const Eigen::VectorXd &states, const Eigen::VectorXd &controls,const double &tk, Eigen::VectorXd &derivatives, localCollocation &problem);


    //! Compute the Jacobian of the dynamics w.r.t. the states and controls using geoMBD library
    //!
    /*! \param $states$         Vector with states in tk
     *  \param $controls$       Vector with controls in tk
     *  \param $tk$             Time at the k discretization node
     *
     *  \return $Jacobian of the dynamics w.r.t. states and controls$
     */

    void autoGenfGradient(const Eigen::VectorXd &states, const Eigen::VectorXd &controls,double &t, Eigen::MatrixXd &gradient, localCollocation &problem);


}//End pinocchio namespace

#endif

#ifdef rbdl_compile

    namespace rbdlLib {

        //! Compute the derivatives of the state vector using the geoMBD library
        //!
        /*! \param $states$         Vector with states in tk
         *  \param $controls$       Vector with controls in tk
         *  \param $tk$             Time at the k discretization node
         *
         *  \return $derivatives$   Vector with the derivative of the states at tk
         */

        void autoGenDynamics(const Eigen::VectorXd &states, const Eigen::VectorXd &controls,const double &tk, Eigen::VectorXd &derivatives, localCollocation &problem);

    }//End rbdlLib namespace

#endif

}//End nocs namespace

#endif




