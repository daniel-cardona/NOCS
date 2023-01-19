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

#include "nlpSolver/IPOPT_Interface.hpp"
#include "IpIpoptApplication.hpp"
#include "IpSolveStatistics.hpp"

namespace nocs{

namespace NLP{

namespace ipopt {


// destructor
IPOPT_INTERFACE::~IPOPT_INTERFACE()
{}

//-------------------Set up the size of the problem ------------------------

bool IPOPT_INTERFACE::get_nlp_info(
   Ipopt::Index&          n,
   Ipopt::Index&          m,
   Ipopt::Index&          nnz_jac_g,
   Ipopt::Index&          nnz_h_lag,
   IndexStyleEnum& index_style
   )
{

   //Number of decision variables

   n = problem->NLP.nDecVar;

   // Number of equality contraints
   m = problem->NLP.nCns;

   // Number of non-zero elements
   nnz_jac_g = problem->NLP.sparsity.nnz_J;

   //Using the Quasi-Newton Approximation of Second Derivatives (this option should be disabled)
   nnz_h_lag = 10;

   // Use the C style indexing (0-based)
   index_style = TNLP::C_STYLE;

   return true;
}

//----------------Set the variable and constraints bounds-------------------------


bool IPOPT_INTERFACE::get_bounds_info(
   Ipopt::Index   n,
   Ipopt::Number* x_l,
   Ipopt::Number* x_u,
   Ipopt::Index   m,
   Ipopt::Number* g_l,
   Ipopt::Number* g_u
   )
{
   // here, the n and m we gave IPOPT in get_nlp_info are passed back to us.
   // If desired, we could assert to make sure they are what we think they are.
   assert(n == problem->NLP.nDecVar);
   assert(m == problem->NLP.nCns);

   //Obtain the defined lower and upper bounds

   Eigen::VectorXd lb(problem->NLP.nDecVar);
   Eigen::VectorXd ub(problem->NLP.nDecVar);



   lb=problem->NLP.xlb;
   ub=problem->NLP.xub;



   // Setting the lower bounds
   for( Ipopt::Index i = 0; i < problem->NLP.nDecVar; i++ )
   {
      x_l[i] = lb(i);


   }
   // Setting the upper bounds
   for( Ipopt::Index i = 0; i < problem->NLP.nDecVar; i++ )
   {
      x_u[i] = ub(i);

   }

   // Set the upper and lower bounds of the contraints

   Eigen::VectorXd cns_lb(problem->NLP.nCns);
   Eigen::VectorXd cns_ub(problem->NLP.nCns);

   cns_lb=problem->NLP.glb;
   cns_ub=problem->NLP.gub;



   for( Ipopt::Index i = 0; i < problem->NLP.nCns; i++ )
   {
       g_l[i] = cns_lb(i);
       g_u[i] = cns_ub(i);

   }

   return true;
}

//------------Set the initial guess of the problem--------------------

bool IPOPT_INTERFACE::get_starting_point(
   Ipopt::Index   n,
   bool    init_x,
   Ipopt::Number* x,
   bool    init_z,
   Ipopt::Number* z_L,
   Ipopt::Number* z_U,
   Ipopt::Index   m,
   bool    init_lambda,
   Ipopt::Number* lambda
   )
{
   assert(init_x == true);
   assert(init_z == false);
   assert(init_lambda == false);

   // initialize to the given starting point
   Eigen::VectorXd iGuess(problem->NLP.nDecVar);

   iGuess<<problem->NLP.x0;

   for( Ipopt::Index i = 0; i < problem->NLP.nDecVar; i++ )
   {
    x[i]=iGuess(i);
   }

   return true;
}

//----------------- Cost function------------------------

// returns the value of the objective function
bool IPOPT_INTERFACE::eval_f(
   Ipopt::Index         n,
   const Ipopt::Number* x,
   bool          new_x,
   Ipopt::Number&       obj_value
   )
{
   assert(n == problem->NLP.nDecVar);

   Eigen::VectorXd z(problem->NLP.nDecVar);

   //Let's obtain the decVar vector in such a way that can be used for EIGEN!
   for( Ipopt::Index i = 0; i < problem->NLP.nDecVar; i++ )
   {
    z(i)=x[i];

   }

   obj_value =problem->costFunction(*problem,z);

   return true;
}

//--------------------Gradient of the objective function--------------

bool IPOPT_INTERFACE::eval_grad_f(
   Ipopt::Index         n,
   const Ipopt::Number* x,
   bool          new_x,
   Ipopt::Number*       grad_f
   )
{
   assert(n == problem->NLP.nDecVar);

   //Let's obtain the decVar vector in such a way that can be used for EIGEN!

   Eigen::VectorXd z(problem->NLP.nDecVar);
   for( Ipopt::Index i = 0; i < problem->NLP.nDecVar; i++ )
   {
    z(i)=x[i];
   }

   //Obtain the gradient of the cost function
   Eigen::VectorXd costObjValueGrad;

   problem->gradientCost(*problem,z,costObjValueGrad);

   for( Ipopt::Index i = 0; i < problem->NLP.nDecVar; i++ )
   {
    grad_f[i]=costObjValueGrad(i);

   }
   return true;
}

//-----------------------Value of the constraints------------------------

// return the value of the constraints: g(x)
bool IPOPT_INTERFACE::eval_g(
   Ipopt::Index         n,
   const Ipopt::Number* x,
   bool          new_x,
   Ipopt::Index         m,
   Ipopt::Number*       g
   )
{
   assert(n == problem->NLP.nDecVar);
   assert(m == problem->NLP.nCns);


   Eigen::VectorXd z(problem->NLP.nDecVar);

   //Let's obtain the decVar vector in such a way that can be used for EIGEN!
   for( Ipopt::Index i = 0; i < problem->NLP.nDecVar; i++ )
   {
    z(i)=x[i];

   }

   Eigen::VectorXd cns(problem->NLP.nCns);

    problem->cnsFunction(*problem,z,cns);

   for( Ipopt::Index i = 0; i < problem->NLP.nCns; i++ )
   {
    g[i]=cns(i);
   }
   return true;
}

//-------------- Jacobian of the constraints ----------------------

bool IPOPT_INTERFACE::eval_jac_g(
   Ipopt::Index         n,
   const Ipopt::Number* x,
   bool                 new_x,
   Ipopt::Index         m,
   Ipopt::Index         nele_jac,
   Ipopt::Index*        iRow,
   Ipopt::Index*        jCol,
   Ipopt::Number*       values
   )
{
    assert(nele_jac == problem->NLP.sparsity.nnz_J);
    assert(n == problem->NLP.nDecVar);
    assert(m == problem->NLP.nCns);

   if( values == NULL )
   {

       //------------NEW VERSION MODIFICATION----------

      // return the structure of the Jacobian

        int nnzG=problem->NLP.sparsity.nnz_J;

       int row, col;
       for(int i = 0; i < nnzG; i++)
       {
         row=problem->NLP.sparsity.nzG_J(i,0);
         col=problem->NLP.sparsity.nzG_J(i,1);

        iRow[i]=row;
        jCol[i]=col;
       }

   }

   else
   {

       Eigen::VectorXd z(problem->NLP.nDecVar);

       //int nnzA=problem.nnzA;
       //int nnzG=problem.nnzG;

       //Let's obtain the decVar vector in such a way that can be used for EIGEN!
       for( Ipopt::Index i = 0; i < problem->NLP.nDecVar; i++ )
       {

        z(i)=x[i];

       }

       //Obtain the gradient of the jacobian

       Eigen::VectorXd nzValues(problem->NLP.sparsity.nnz_J);

       problem->cnsJacobian(*problem,z,nzValues);

      // return the values of the Jacobian of the constraints

       for( Ipopt::Index i = 0; i < problem->NLP.sparsity.nnz_J; i++ )
       {
            values[i]=nzValues(i);
       }

   }
   return true;
}

//----------------------Evaluation of the Hessian------------------------------

//return the structure or values of the Hessian
bool IPOPT_INTERFACE::eval_h(
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
   )
{
   return true;
}

//-------------------------Print the solution-----------------------------------

void IPOPT_INTERFACE::finalize_solution(
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
   )
{

   //Let's return this solution and return the control to the solver!

   Eigen::VectorXd sol(problem->NLP.nDecVar);

   for( Ipopt::Index i = 0; i < problem->NLP.nDecVar; i++ )
   {
      sol(i)=x[i];
   }

     problem->solution.z_opt=sol;

}


}//End Ipopt namespace
}//End NLP namespace

int localCollocation::solveNLP_ipopt(){

    // Create a new instance of your nlp
        //  (use a SmartPtr, not raw)
        Ipopt::SmartPtr<Ipopt::TNLP> mynlp = new NLP::ipopt::IPOPT_INTERFACE(this);

        // Create a new instance of IpoptApplication
        Ipopt::SmartPtr<Ipopt::IpoptApplication> app = new Ipopt::IpoptApplication();
        app->RethrowNonIpoptException(true);

        // Change some options
        app->Options()->SetNumericValue("tol", 1e-4);
        app->Options()->SetNumericValue("derivative_test_perturbation",1e-6);
        app->Options()->SetIntegerValue("max_iter",3000);
        app->Options()->SetStringValue("mu_strategy", "adaptive");
        //app->Options()->SetStringValue("output_file", "ipopt.out");
        app->Options()->SetStringValue("hessian_approximation","limited-memory");
        app->Options()->SetStringValue("nlp_scaling_method","gradient-based");
        app->Options()->SetIntegerValue("print_level",5);


        if(this->algorithm.derivativeChecker){
            app->Options()->SetStringValue("derivative_test","first-order");
        }

        // Initialize the IpoptApplication and process the options
        Ipopt::ApplicationReturnStatus status;
        status = app->Initialize();


        if( status != Ipopt::Solve_Succeeded )
        {
           std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
           return (int) status;
        }


        // Ask Ipopt to solve the problem
        status = app->OptimizeTNLP(mynlp);

        if( status == Ipopt::Solve_Succeeded )
        {
           std::cout << std::endl << std::endl << "*** The problem solved!" << std::endl;
        }
        else
        {
           std::cout << std::endl << std::endl << "*** The problem FAILED!" << std::endl;
        }



    return (int)status;

}//End localCollocation::solveNLP_ipopt


}//End OCSolver interface
