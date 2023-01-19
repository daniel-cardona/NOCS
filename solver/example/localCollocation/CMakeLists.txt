cmake_minimum_required(VERSION 2.8.12)

message("BUILD TEST IS ENABLE")


if(DYNLIBRARY STREQUAL "pinocchio")

add_subdirectory(pinocchio_examples)

elseif(DYNLIBRARY STREQUAL "rbdl")

add_subdirectory(rbdl_examples)

elseif(DYNLIBRARY STREQUAL "default")

add_subdirectory(nocs_examples)

endif(DYNLIBRARY STREQUAL "pinocchio")






