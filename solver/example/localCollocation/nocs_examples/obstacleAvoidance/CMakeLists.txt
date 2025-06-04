cmake_minimum_required(VERSION 2.8.12)


message("Compiling obstacle avoidance test")

add_executable(obstacle_test functions.cxx main.cxx ${CORE} ${nlpSolver} ${localColModule})

target_link_libraries(obstacle_test    ${IPOPT_LIBRARIES}
                                         ${MATPLOT_LIB})

set_target_properties(obstacle_test PROPERTIES LINKER_LANGUAGE CXX)


