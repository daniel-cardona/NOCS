#ifndef CORE_PROBLEM
#define CORE_PROBLEM

#include <iostream>
#include <string>
#include <sstream>
#include <assert.h>
#include <math.h>
#include <vector>
#include <fstream>


//Ipopt libraries
#include "IpTNLP.hpp"
#include "IpIpoptApplication.hpp"


#ifndef MC_EPSILON
#define MC_EPSILON 2.221e-16
#endif

typedef double DataType;

namespace nocs {

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&---------------OPTIMAL CONTROL PROBLEM CLASS-------------------&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

    class Problem{

    public:

        //OCP Variables

        int nStates;
        int nControls;
        int nCollocationPoints;
        int nDiscretePoints;

        int nPath;
        int nEvents;


        std::string colMethod; //Collocation method


        //Problem(){};

        ~Problem(){}

    };


}

#endif //End CORE_PROBLEM
