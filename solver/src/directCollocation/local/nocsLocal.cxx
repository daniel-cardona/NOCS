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

namespace nocs {

void nocsLocal(localCollocation &init_problem, localCollocation &solution){

    //Create the vector of localCollocation objects

    vector<localCollocation> problems;

    //Solve the initial problem

    int status=init_problem.solve();

    //Save it in the vector

    problems.push_back(init_problem);

    //Start the mesh refinement algorithm

    int n=0;

    if (init_problem.algorithm.maxMeshIterations==1)
        solution=init_problem;


    solution.printStatistics();

    for( n=1;n<init_problem.algorithm.maxMeshIterations;n++){

       //Construct the problem of the next iteration

        localCollocation meshIteration(problems.back().NLP,
                                       problems.back().mesh,
                                       problems.back().bounds,
                                       problems.back().algorithm,
                                       problems.back().guess,
                                       problems.back().colMethod,
                                       problems.back().derivatives,
                                       problems.back().robotFile);

       //Setup the dimensions of the problem

       meshIteration.setupNLP();

       //Solve the problem

       status=meshIteration.solve();

       //Save the mesh iteration in the vector

       problems.push_back(meshIteration);

       meshIteration.printStatistics();

       //If the convergence conditions has been fullfilled break the cycle and return the solution
       if(status==2){

           solution=meshIteration;

           break;

       }//End if(status==2)

    }//End for

}//End nocsLocal


}// End OCSolver namespace
