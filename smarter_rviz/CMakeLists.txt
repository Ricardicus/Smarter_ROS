# ROS2 ament cmake file
cmake_minimum_required(VERSION 3.5)
project(smarter_rviz)

# Default to C++11
set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(visualization_msgs REQUIRED)

# Declare a C++ executable
add_executable(send_rviz_markers src/send_rviz_markers.cpp)

# Specify libraries to link a library or executable target against
ament_target_dependencies(send_rviz_markers
  rclcpp
  resource_retriever
  visualization_msgs
)

# Install the executable
install(TARGETS
  send_rviz_markers
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Export dependencies
#ament_export_dependencies(rclcpp resource_retriever visualization_msgs)

# Generate the package
ament_package()
