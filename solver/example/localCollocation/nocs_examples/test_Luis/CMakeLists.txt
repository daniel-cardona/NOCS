cmake_minimum_required(VERSION 2.8.12)


message("Compiling RRPlanar robot with obstacles")

add_executable(planarObstacle functions.cxx main.cxx ${CORE} ${nlpSolver} ${localColModule})

target_include_directories(planarObstacle PRIVATE  ${MATPLOT_INCLUDE})

target_link_libraries(planarObstacle     ${IPOPT_LIBRARIES}
                                         ${MATPLOT_LIB})

set_target_properties(planarObstacle PROPERTIES LINKER_LANGUAGE CXX)


