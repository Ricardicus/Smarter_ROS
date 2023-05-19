FROM osrf/ros:foxy-desktop

ARG user_id=1000
ARG ros_ws=/home/dockeruser/dev_ws

ENV DEBIAN_FRONTEND=noninteractive

# Install packages.
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-control-toolbox 

COPY . /home/dockeruser/dev_ws

RUN /bin/bash -c "source /opt/ros/foxy/setup.bash &&\
  cd /home/dockeruser/dev_ws &&\
  colcon build --symlink-install &&\
  source install/setup.bash && \
  cd pub_sub_custom_msg && \
  colcon build --symlink-install"

CMD ["/home/dockeruser/dev_ws/docker_entrypoint.sh"]


