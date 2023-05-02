cmake_minimum_required(VERSION 3.5)
project(assignment_1)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

#find_package(your_other_dependencies REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
) # ${your_other_dependencies_INCLUDE_DIRS})

# Adding a image publisher and subscriber programs
add_executable(image_publisher_node src/image_publisher_node.cpp)
add_executable(image_subscriber_node src/image_subscriber_node.cpp)
add_executable(object_detect_node src/object_detect_node.cpp)

ament_target_dependencies(image_publisher_node
  rclcpp
  sensor_msgs
  OpenCV
  cv_bridge) 

ament_target_dependencies(image_subscriber_node
  rclcpp
  sensor_msgs
  OpenCV
  cv_bridge) 

ament_target_dependencies(object_detect_node
  rclcpp
  sensor_msgs
  OpenCV
  cv_bridge) 

install(TARGETS
  object_detect_node
  image_publisher_node
  image_subscriber_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

