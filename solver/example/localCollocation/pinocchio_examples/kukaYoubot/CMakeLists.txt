cmake_minimum_required(VERSION 2.8.12)


message("Compiling KUKA YOUBOT")
add_executable(kuka_robot functions.cxx main.cxx ${CORE} ${nlpSolver} ${localColModule})

target_include_directories(kuka_robot PRIVATE ${MATPLOT_INCLUDE})

target_link_libraries(kuka_robot          ${IPOPT_LIBRARIES}
                                         ${MATPLOT_LIB}
                                         ${PINOCCHIO_LIBRARIES})

set_target_properties(kuka_robot PROPERTIES LINKER_LANGUAGE CXX)


