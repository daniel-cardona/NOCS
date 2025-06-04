cmake_minimum_required(VERSION 3.0)
project(nocs VERSION 1.0)

set(PACKAGE_NAME "NOCS")
set(PACKAGE_VERSION "1.0.0")
set(PACKAGE_SO_VERSION "1.0")
set(PACKAGE_DESCRIPTION "Robotics Numerical Optimal Control Solver")
set(PACKAGE_AUTHOR "Daniel Cardona-Ortiz, Gustavo Arechavaleta")
set(PACKAGE_MAINTAINER "Daniel Cardona-Ortiz")


############################################################
#################   Compile options    #####################
############################################################

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-msse -msse2 -msse3 -mavx -march=native -O3 -pedantic -DNDEBUG")
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)


option(BUILD_TESTS "Build test programs" ON)

set(DYNLIBRARY default CACHE STRING "Dynamic library")
set_property(CACHE DYNLIBRARY PROPERTY STRINGS pinocchio rbdl default)

############################################################
########     Bring headers of external libraries   #########
############################################################

include_directories("include")

include_directories(${CMAKE_CURRENT_BINARY_DIR})


find_package(PkgConfig REQUIRED)
set(ENV{PKG_CONFIG_PATH} "/usr/local/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")

#-------IPOPT
pkg_check_modules(IPOPT REQUIRED ipopt)
include_directories(${IPOPT_INCLUDE_DIRS})
link_directories(${IPOPT_LIBRARY_DIRS})

message(STATUS "IPOPT include dirs: ${IPOPT_INCLUDE_DIRS}")
message(STATUS "IPOPT libraries: ${IPOPT_LIBRARIES}")
message(STATUS "IPOPT version: ${IPOPT_VERSION}")

#----Eigen library
pkg_check_modules(EIGEN REQUIRED eigen3)
include_directories(${EIGEN_INCLUDE_DIRS})

message(STATUS "Eigen include dirs: ${EIGEN_INCLUDE_DIRS}")
message(STATUS "Eigen version: ${EIGEN_VERSION}")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lrt")

#Add the main modules of nocs

file(GLOB CORE 			    "src/core/*.cxx")
file(GLOB nlpSolver 		"src/nlpSolver/*.cxx")
file(GLOB localColModule 	"src/directCollocation/local/*.cxx")

#Setup of the dynamics libraries

if(DYNLIBRARY STREQUAL "pinocchio")

    find_package(pinocchio)
    include_directories(${PINOCCHIO_INCLUDE_DIRS})
    message(STATUS "Pinocchio include dirs: ${PINOCCHIO_INCLUDE_DIRS}")
    message(STATUS "Pinocchio libraries: ${PINOCCHIO_LIBRARIES}")
    message(STATUS "Pinocchio version: ${PINOCCHIO_VERSION}")
    set(pinocchio_compile ON CACHE BOOL "Flag: Build nocs with pinnochio library" FORCE)
    set(geoMBD_compile OFF CACHE BOOL "" FORCE)
    set(rbdl_compile OFF CACHE BOOL "" FORCE)

elseif(DYNLIBRARY STREQUAL "rbdl")

    list(APPEND CMAKE_FIND_ROOT_PATH ${PROJECT_SOURCE_DIR}/../rbdl/cmake/linux-64)
    find_package(rbdl)
    include_directories(${RBDL_INCLUDE_DIRS})
    set(rbdl_compile ON CACHE BOOL "Flag: Build nocs with RBDL library" FORCE)
    set(pinocchio_compile OFF CACHE BOOL "" FORCE)
    set(geoMBD_compile OFF CACHE BOOL "" FORCE)

elseif(DYNLIBRARY STREQUAL "default")

    message("You are compiling nocs without dynamics libraries")

    set(pinocchio_compile OFF CACHE BOOL "" FORCE)
    set(rbdl_compile OFF CACHE BOOL "" FORCE)
    set(geoMBD_compile OFF CACHE BOOL "" FORCE)

endif(DYNLIBRARY STREQUAL "pinocchio")

configure_file(nocsConfig.h.in nocsConfig.h @ONLY)

if(BUILD_TESTS)
    add_subdirectory(example/localCollocation)
endif(BUILD_TESTS)
