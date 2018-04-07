cmake_minimum_required(VERSION 2.8)
project(astar_c)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -g -O0")
add_executable(test_astar_c "astar_c.c")
