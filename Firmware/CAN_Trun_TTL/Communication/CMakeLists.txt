cmake_minimum_required(VERSION 3.22)
add_library(Communication INTERFACE)
file(GLOB_RECURSE SOURCE "${CMAKE_CURRENT_LIST_DIR}/*.cpp")
target_sources(Communication INTERFACE ${SOURCE})

target_include_directories(Communication INTERFACE ./include)