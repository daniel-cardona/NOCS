cmake_minimum_required(VERSION 2.8.12)


message("Compiling NAO ROBOT")
add_executable(nao_robot functions.cxx main.cxx ${CORE} ${nlpSolver} ${localColModule})

target_include_directories(nao_robot PRIVATE ${MATPLOT_INCLUDE})

target_link_libraries(nao_robot          ${IPOPT_LIBRARIES}
                                         ${MATPLOT_LIB}
                                         ${PINOCCHIO_LIBRARIES}
                                         ${Coppelia_LIBRARIES} rt)

set_target_properties(nao_robot PROPERTIES LINKER_LANGUAGE CXX)


