cmake_minimum_required(VERSION 3.5)
project(pub_sub_custom_msg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_msg REQUIRED)  
find_package(control_toolbox REQUIRED)  

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Compilation and Installtion
add_executable(publisher src/pub.cpp)
# Change
ament_target_dependencies(publisher rclcpp custom_msg)
# change 
add_executable(subscriber src/sub.cpp)
ament_target_dependencies(subscriber rclcpp custom_msg)

add_executable(pid_controller src/pid.cpp)
ament_target_dependencies(pid_controller rclcpp custom_msg control_toolbox)

install(TARGETS
   publisher
   subscriber
   pid_controller
   DESTINATION lib/${PROJECT_NAME}
 )

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()


