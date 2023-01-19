cmake_minimum_required(VERSION 2.8.12)


message("Compiling UR5")

add_executable(ur5_example functions.cxx main.cxx ${CORE} ${nlpSolver} ${localColModule})

target_include_directories(ur5_example PRIVATE ${MATPLOT_INCLUDE})

target_link_libraries(ur5_example        ${IPOPT_LIBRARIES}
                                         ${MATPLOT_LIB}
                                         ${PINOCCHIO_LIBRARIES})

set_target_properties(ur5_example PROPERTIES LINKER_LANGUAGE CXX)


