cmake_minimum_required(VERSION 2.8.12)


message("Compiling NAO ROBOT")
add_executable(nao_robot functions.cxx main.cxx ${CORE} ${nlpSolver} ${localColModule})

target_link_libraries(nao_robot          ${IPOPT_LIBRARIES}
                                         ${PINOCCHIO_LIBRARIES})

set_target_properties(nao_robot PROPERTIES LINKER_LANGUAGE CXX)


