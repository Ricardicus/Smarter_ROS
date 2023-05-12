# Running the foxy docker port

```
docker network create ros2_network
docker run -it -v $(pwd):/root/smarter --network ros2_network osrf/ros:foxy-desktop 
cd ~/smarter
# Go to a project to build
cd smarter_hello_world
colcon build --symlink-install
```

# Create a new package

```
ros2 pkg create --build-type ament_cmake my_package --dependencies rclcpp std_msgs
```

Add to the CMakeLists.txt:

```
add_executable(smarter_test_hello_world src/hello_world.cpp)
install(TARGETS
  smarter_test_hello_world
  DESTINATION lib/${PROJECT_NAME}
)
```

Build and run:

```
colcon build --symlink-install
source install/setup.bash
# list executables
ros2 pkg executables my_package
# Run 
ros2 run my_package smarter_test_hello_world
```

# Simple topic publisher and subscriber in the command line

````bash
ros2 topic echo /magical_number std_msgs/msg/Int8
ros2 topic pub --once /magical_number std_msgs/msg/Int8 "data: 42"
```

# Changing DDS implementation

By default FastDDS is used. We can change it to Cyclone DDS:

```
sudo apt install ros-foxy-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

