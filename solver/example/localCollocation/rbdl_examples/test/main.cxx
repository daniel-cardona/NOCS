#include "directCollocation/local/nocsLocal.hpp"

#include "directCollocation/local/fcnGenerator.hpp"

#include <iostream>
#include <chrono>

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODEL_DIR "../../../../../data/pinocchio_models"
#endif

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

int main () {

  const std::string urdf_filename = PINOCCHIO_MODEL_DIR + std::string("/nao_inertial_python.urdf");

  //Set the main data of the optimal control problem

      int nStates=48;
      int nControls=24;
      int nEvents=96;
      int nPath=0;
      int nDiscretePoints=120;

      //&-------------- Build the Robot using the rigid body dynamics library pinocchio ----------------------&

      nocs::localCollocation problem(nStates,nControls,nDiscretePoints,nPath,nEvents,urdf_filename);

      Eigen::VectorXd states (nStates);
      Eigen::VectorXd controls(nControls);

      Eigen::VectorXd derivatives(nStates);

      VectorNd Q = VectorNd::LinSpaced(problem.robot_model->q_size, 1, 2*3.1416);
      VectorNd QDot = VectorNd::LinSpaced(problem.robot_model->q_size, 1, 2*3.1416);
      VectorNd Tau = VectorNd::LinSpaced(problem.robot_model->q_size, 1, 2*3.1416);



      states.setZero();

      derivatives.setZero();

      double t=10;

      states<<Q,QDot;

      problem.dynamics(states,Tau,t,derivatives,problem);


     cout<<derivatives<<endl;

  return 0;

}
