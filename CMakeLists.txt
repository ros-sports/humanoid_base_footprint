cmake_minimum_required(VERSION 3.5)
project(humanoid_base_footprint)

# Add support for C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(rotconv REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(biped_interfaces REQUIRED)

set(INCLUDE_DIRS include ${bitbots_docs_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS} ${ament_cmake_INCLUDE_DIRS}${std_msgs_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS} ${geometry_msgs_INCLUDE_DIRS}
  ${rclpy_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS} ${tf2_eigen_INCLUDE_DIRS}
  ${rotconv_INCLUDE_DIRS} ${rosidl_default_generators_INCLUDE_DIRS} ${biped_interfaces_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${bitbots_docs_LIBRARY_DIRS} ${tf2_geometry_msgs_LIBRARY_DIRS}
  ${ament_cmake_LIBRARY_DIRS}
  ${std_msgs_LIBRARY_DIRS} ${rclcpp_LIBRARY_DIRS} ${tf2_LIBRARY_DIRS}
  ${geometry_msgs_LIBRARY_DIRS} ${rclpy_LIBRARY_DIRS}
  ${sensor_msgs_LIBRARY_DIRS} ${tf2_eigen_LIBRARY_DIRS} ${rotconv_LIBRARY_DIRS}
  ${rosidl_default_generators_LIBRARY_DIRS} ${biped_interfaces_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${bitbots_docs_LIBRARIES} ${tf2_geometry_msgs_LIBRARIES}
  ${ament_cmake_LIBRARIES} ${std_msgs_LIBRARIES}
  ${rclcpp_LIBRARIES} ${tf2_LIBRARIES} ${geometry_msgs_LIBRARIES}
  ${rclpy_LIBRARIES} ${sensor_msgs_LIBRARIES}
  ${tf2_eigen_LIBRARIES} ${rotconv_LIBRARIES}
  ${rosidl_default_generators_LIBRARIES} ${biped_interfaces_LIBRARIES})

add_compile_options(-Wall -Werror -Wno-unused)

find_package(Eigen3 REQUIRED)

## Declare a C++ executable
add_executable(base_footprint src/base_footprint.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(base_footprint ${LIBS} ${Eigen3_LIBRARIES})

ament_export_dependencies(tf2_geometry_msgs)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(std_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(tf2)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(rclpy)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2_eigen)
ament_export_dependencies(rotconv)
ament_export_dependencies(rosidl_default_generators)
ament_export_dependencies(biped_interfaces)
ament_export_include_directories(${INCLUDE_DIRS})

install(TARGETS base_footprint
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

ament_package()
