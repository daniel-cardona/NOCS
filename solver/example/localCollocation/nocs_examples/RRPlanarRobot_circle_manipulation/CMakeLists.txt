cmake_minimum_required(VERSION 2.8.12)


message("Compiling RRPlanar robot with obstacles")

add_executable(planarRRManipulation functions.cxx main.cxx ${CORE} ${nlpSolver} ${localColModule})

target_link_libraries(planarRRManipulation  ${IPOPT_LIBRARIES})

set_target_properties(planarRRManipulation  PROPERTIES LINKER_LANGUAGE CXX)


