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


#include "directCollocation/local/local.hpp"

//using namespace Ipopt;

#ifndef __IPOPT_INTERFACE_HPP__
#define __IPOPT_INT_HPP__

namespace nocs
{

namespace NLP

{

namespace ipopt {

class IPOPT_INTERFACE: public Ipopt::TNLP{

    localCollocation *problem;

public:
    /** Default constructor */
    IPOPT_INTERFACE(nocs::localCollocation *problem_)
    {
        problem=problem_;
    }

    /** Default destructor */
    virtual ~IPOPT_INTERFACE();

    /**@name Overloaded from TNLP */
    //@{
    /** Method to return some info about the NLP */
    virtual bool get_nlp_info(
       Ipopt::Index&          n,
       Ipopt::Index&          m,
       Ipopt::Index&          nnz_jac_g,
       Ipopt::Index&          nnz_h_lag,
       IndexStyleEnum& index_style
       );

    /** Method to return the bounds for my problem */
    virtual bool get_bounds_info(
       Ipopt::Index   n,
       Ipopt::Number* x_l,
       Ipopt::Number* x_u,
       Ipopt::Index   m,
       Ipopt::Number* g_l,
       Ipopt::Number* g_u
       );

    /** Method to return the starting point for the algorithm */
    virtual bool get_starting_point(
       Ipopt::Index   n,
       bool    init_x,
       Ipopt::Number* x,
       bool    init_z,
       Ipopt::Number* z_L,
       Ipopt::Number* z_U,
       Ipopt::Index   m,
       bool    init_lambda,
       Ipopt::Number* lambda
       );

    /** Method to return the objective value */
    virtual bool eval_f(
       Ipopt::Index         n,
       const Ipopt::Number* x,
       bool          new_x,
       Ipopt::Number&       obj_value
       );

    /** Method to return the gradient of the objective */
    virtual bool eval_grad_f(
       Ipopt::Index         n,
       const Ipopt::Number* x,
       bool          new_x,
       Ipopt::Number*       grad_f
       );

    /** Method to return the constraint residuals */
    virtual bool eval_g(
       Ipopt::Index         n,
       const Ipopt::Number* x,
       bool          new_x,
       Ipopt::Index         m,
       Ipopt::Number*       g
       );

    /** Method to return:
     *   1) The structure of the jacobian (if "values" is NULL)
     *   2) The values of the jacobian (if "values" is not NULL)
     */
    virtual bool eval_jac_g(
       Ipopt::Index         n,
       const Ipopt::Number* x,
       bool          new_x,
       Ipopt::Index         m,
       Ipopt::Index         nele_jac,
       Ipopt::Index*        iRow,
       Ipopt::Index*        jCol,
       Ipopt::Number*       values
       );

    /** Method to return:
     *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
     *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
     */
    virtual bool eval_h(
       Ipopt::Index         n,
       const Ipopt::Number* x,
       bool          new_x,
       Ipopt::Number        obj_factor,
       Ipopt::Index         m,
       const Ipopt::Number* lambda,
       bool          new_lambda,
       Ipopt::Index         nele_hess,
       Ipopt::Index*        iRow,
       Ipopt::Index*        jCol,
       Ipopt::Number*       values
       );

    /** This method is called when the algorithm is complete so the TNLP can store/write the solution */
    virtual void finalize_solution(
       Ipopt::SolverReturn               status,
       Ipopt::Index                      n,
       const Ipopt::Number*              x,
       const Ipopt::Number*              z_L,
       const Ipopt::Number*              z_U,
       Ipopt::Index                      m,
       const Ipopt::Number*              g,
       const Ipopt::Number*              lambda,
       Ipopt::Number                     obj_value,
       const Ipopt::IpoptData*           ip_data,
       Ipopt::IpoptCalculatedQuantities* ip_cq
       );
    //@}

 private:
    /**@name Methods to block default compiler methods.
     *
     *
     * The compiler automatically generates the following three methods.
     *  Since the default compiler implementation is generally not what
     *  you want (for all but the most simple classes), we usually
     *  put the declarations of these methods in the private section
     *  and never implement them. This prevents the compiler from
     *  implementing an incorrect "default" behavior without us
     *  knowing. (See Scott Meyers book, "Effective C++")
     */
    //@{
    IPOPT_INTERFACE(
       const IPOPT_INTERFACE&
       );

    IPOPT_INTERFACE& operator=(
       const IPOPT_INTERFACE&
       );
    //@}

};
#endif


}//END ipopt namespace

}//END NLP namespace

}//END OCSolver namespace
