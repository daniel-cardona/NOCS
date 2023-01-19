cmake_minimum_required(VERSION 2.8.12)


message("Compiling Data Streaming")
add_executable(nao_robot dataStreaming.cxx)

target_link_libraries(nao_robot          ${IPOPT_LIBRARIES}
                                         ${PINOCCHIO_LIBRARIES}
                                         ${Coppelia_LIBRARIES} rt)

set_target_properties(nao_robot PROPERTIES LINKER_LANGUAGE CXX)


