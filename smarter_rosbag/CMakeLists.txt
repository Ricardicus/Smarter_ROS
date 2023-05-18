cmake_minimum_required(VERSION 3.5)
project(smarter_rosbag)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost REQUIRED COMPONENTS system program_options)
find_package(Eigen3 REQUIRED)

# Declare a C++ executable
add_executable(logfile_parser src/logfile_parser.cpp)

# Specify libraries to link a library or executable target against
ament_target_dependencies(logfile_parser
  rclcpp
  rosbag2
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  Boost
  Eigen3
)

set(Eigen3_INCLUDE_DIR /path/to/eigen)
include_directories(${EIGEN3_INCLUDE_DIR})

# Install targets
install(TARGETS
  logfile_parser
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(rosbag2 nav_msgs sensor_msgs tf2 tf2_ros Boost Eigen3)

# Package the project
ament_package()
