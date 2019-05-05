cmake_minimum_required(VERSION 2.6)
project(kitti2bag_simple)

add_executable(kitti2bag_simple main.cpp)

install(TARGETS kitti2bag_simple RUNTIME DESTINATION bin)
