cmake_minimum_required(VERSION 2.8.12)


message("Compiling GEOMBD_test")

add_executable(test functions.cxx main.cxx ${CORE} ${nlpSolver} ${localColModule})
target_include_directories(test PRIVATE  ${MATPLOT_INCLUDE})
target_link_libraries(test     		 ${IPOPT_LIBRARIES}
                                         ${MATPLOT_LIB}
                                         ${GeoMBD_LIBRARIES})

set_target_properties(test PROPERTIES LINKER_LANGUAGE CXX)


