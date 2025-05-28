cmake_minimum_required(VERSION 2.8.12)


message("Compiling RRPlanar robot with obstacles")

add_executable(planarRobot functions.cxx main.cxx ${CORE} ${nlpSolver} ${localColModule})

target_include_directories(planarRobot PRIVATE  ${MATPLOT_INCLUDE})

target_link_libraries(planarRobot     ${IPOPT_LIBRARIES}
                                         ${MATPLOT_LIB})

set_target_properties(planarRobot PROPERTIES LINKER_LANGUAGE CXX)


