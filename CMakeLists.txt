cmake_minimum_required(VERSION 2.6)
project(visual_perception)

add_executable(visual_perception main.cpp)

install(TARGETS visual_perception RUNTIME DESTINATION bin)
