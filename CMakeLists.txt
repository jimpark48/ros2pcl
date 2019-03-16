cmake_minimum_required(VERSION 2.8.3)
project(ros2pcl)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros
  DEPENDS system_lib
)

file(GLOB SRC
    "src/*.h"
    "src/*.cpp"
)


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#add_executable(ros2pcl ${SRC})
add_executable(ros2pcl src/main.cpp)
add_executable(flat src/flat.cpp)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(ros2pcl
${catkin_LIBRARIES}
)
target_link_libraries(flat ${catkin_LIBRARIES})
