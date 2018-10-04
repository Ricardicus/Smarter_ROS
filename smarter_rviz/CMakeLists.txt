cmake_minimum_required(VERSION 2.8.3)
project(smarter_rviz)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  resource_retriever
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp resource_retriever visualization_msgs
)

###########
## Build ##
###########

include_directories(
 ${catkin_INCLUDE_DIRS}
)

add_executable(send_rviz_markers src/send_rviz_markers.cpp)
add_dependencies(send_rviz_markers ${catkin_EXPORTED_TARGETS})
target_link_libraries(send_rviz_markers
  ${catkin_LIBRARIES}
)
