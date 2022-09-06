cmake_minimum_required(VERSION 2.6)
project(kitti2bag_simple)

find_package(PCL 1.7.0 REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
				roscpp
				pcl_ros
				sensor_msgs
				rosbag
				)

include_directories(${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(kitti2bag_simple main.cpp)


target_link_libraries(kitti2bag_simple ${PCL_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS kitti2bag_simple RUNTIME DESTINATION bin)
