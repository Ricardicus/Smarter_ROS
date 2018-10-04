cmake_minimum_required(VERSION 2.8.3)
project(smarter_tf)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  tf_conversions
  eigen_conversions
  visualization_msgs
)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

catkin_package(
  CATKIN_DEPENDS roscpp tf_conversions visualization_msgs
)

###########
## Build ##
###########

include_directories(include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(send_tfs src/send_tfs.cpp)
add_dependencies(send_tfs ${catkin_EXPORTED_TARGETS})
target_link_libraries(send_tfs
  ${catkin_LIBRARIES}
)

add_executable(get_tfs src/get_tfs.cpp)
add_dependencies(get_tfs ${catkin_EXPORTED_TARGETS})
target_link_libraries(get_tfs
  ${catkin_LIBRARIES}
)
