# Running the foxy docker port

```
docker run -it -v $(pwd):/root/smarter osrf/ros:foxy-desktop 
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
ros2 run my_package smarter_test_hello_world
```


