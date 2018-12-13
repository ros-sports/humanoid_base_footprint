cmake_minimum_required(VERSION 2.8.3)
project(humanoid_base_footprint)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  std_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  message_generation
  geometry_msgs
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(base_footprint src/base_footprint.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(base_footprint
    ${catkin_LIBRARIES}
)
