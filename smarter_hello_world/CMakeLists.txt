cmake_minimum_required(VERSION 3.5)
project(smarter_hello_world)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(your_other_dependencies REQUIRED)

include_directories(include ${rclcpp_INCLUDE_DIRS}) # ${your_other_dependencies_INCLUDE_DIRS})

add_executable(smarter_hello_world src/hello_world.cpp)
#ament_target_dependencies(your_executable_name rclcpp your_other_dependencies)

install(TARGETS
  smarter_hello_world 
  DESTINATION bin/${PROJECT_NAME}
)

ament_package()

