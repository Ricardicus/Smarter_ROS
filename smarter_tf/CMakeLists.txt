cmake_minimum_required(VERSION 3.5)
project(smarter_tf)

# Use C++14
set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

ament_export_dependencies(rclcpp tf2 tf2_ros tf2_eigen visualization_msgs)

###########
## Build ##
###########

include_directories(include
  ${EIGEN3_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_eigen_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
)

add_executable(send_tfs src/send_tfs.cpp)
ament_target_dependencies(send_tfs rclcpp tf2 tf2_ros tf2_eigen visualization_msgs)

add_executable(get_tfs src/get_tfs.cpp)
ament_target_dependencies(get_tfs rclcpp tf2 tf2_ros tf2_eigen visualization_msgs)

install(TARGETS
  send_tfs
  get_tfs
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
