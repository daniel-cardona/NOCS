cmake_minimum_required(VERSION 2.8.12)


message("Compiling generateURDF")
add_executable(snakeRobot main.cxx functions.cxx ${CORE} ${nlpSolver} ${localColModule})
#add_executable(generateURDF generateURDF.cxx)


target_link_libraries(snakeRobot       ${IPOPT_LIBRARIES}
                                       ${PINOCCHIO_LIBRARIES}
					stdc++fs)

set_target_properties(snakeRobot PROPERTIES LINKER_LANGUAGE CXX)


