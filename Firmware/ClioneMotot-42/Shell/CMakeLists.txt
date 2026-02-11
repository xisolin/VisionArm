cmake_minimum_required(VERSION 3.22)
add_library(Shell INTERFACE)
file(GLOB_RECURSE SOURCE "${CMAKE_CURRENT_LIST_DIR}/*.cpp")
target_sources(Shell INTERFACE ${SOURCE})

target_include_directories(Shell INTERFACE ./include)