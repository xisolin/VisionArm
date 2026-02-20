cmake_minimum_required(VERSION 3.22)
add_library(lwbtn INTERFACE)
file(GLOB_RECURSE SOURCE "${CMAKE_CURRENT_LIST_DIR}/*.c")
target_sources(lwbtn INTERFACE ${SOURCE})

target_include_directories(lwbtn INTERFACE ./Inc)