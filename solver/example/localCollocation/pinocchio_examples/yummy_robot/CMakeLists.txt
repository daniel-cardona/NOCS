cmake_minimum_required(VERSION 2.8.12)


message("Compiling UR5")

add_executable(yummy_robot functions.cxx main.cxx ${CORE} ${nlpSolver} ${localColModule})

target_include_directories(yummy_robot PRIVATE ${MATPLOT_INCLUDE})

target_link_libraries(yummy_robot        ${IPOPT_LIBRARIES}
                                         ${MATPLOT_LIB}
                                         ${PINOCCHIO_LIBRARIES})

set_target_properties(yummy_robot PROPERTIES LINKER_LANGUAGE CXX)


