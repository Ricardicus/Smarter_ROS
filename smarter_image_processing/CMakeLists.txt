cmake_minimum_required(VERSION 3.5)
project(smarter_gray_scale)

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
#find_package(your_other_dependencies REQUIRED)

include_directories(include ${rclcpp_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}) # ${your_other_dependencies_INCLUDE_DIRS})

add_executable(gray_scale_converter_node src/gray_scale_converter_node.cpp)
ament_target_dependencies(gray_scale_converter_node rclcpp sensor_msgs OpenCV) # your_other_dependencies)

install(TARGETS
  gray_scale_converter_node
  DESTINATION bin/${PROJECT_NAME}
)

ament_package()

