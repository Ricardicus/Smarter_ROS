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

FROM ubuntu:jammy-20220801

WORKDIR /home/dockeruser/dev_ws

RUN apt-get update && apt-get install -y --no-install-recommends \
   python3 
COPY ./docker_entrypoint.sh ./docker_entrypoint.sh

COPY --from=0 /home/dockeruser/dev_ws/./ ./
COPY --from=0 /opt/ros/foxy /opt/ros/foxy

# Handpicking some dependencies
COPY --from=0 /usr/lib/x86_64-linux-gnu/libspdlog.so.1 /usr/lib/x86_64-linux-gnu/libspdlog.so.1 
COPY --from=0 /usr/lib/x86_64-linux-gnu/libtinyxml2.so.6 /usr/lib/x86_64-linux-gnu/libtinyxml2.so.6 
COPY --from=0 /usr/lib/x86_64-linux-gnu/libssl.so.1.1 /usr/lib/x86_64-linux-gnu/libssl.so.1.1 
COPY --from=0 /usr/lib/x86_64-linux-gnu/libcrypto.so.1.1 /usr/lib/x86_64-linux-gnu/libcrypto.so.1.1 


CMD ["/home/dockeruser/dev_ws/docker_entrypoint.sh"]


