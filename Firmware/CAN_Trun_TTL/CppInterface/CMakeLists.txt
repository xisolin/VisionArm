cmake_minimum_required(VERSION 3.22)
add_library(CPP_INTERFACE INTERFACE)
file(GLOB_RECURSE SOURCE "${CMAKE_CURRENT_LIST_DIR}/*.cpp")
target_sources(CPP_INTERFACE INTERFACE ${SOURCE})

target_include_directories(CPP_INTERFACE INTERFACE ./include)