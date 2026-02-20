FROM ros:humble

ENV ROS_HOME=/opt/ros_home

WORKDIR /tmp/src/nav_infrastructure_simple
COPY . .

RUN apt-get update \
 && apt-get install -y python3-pip \
 && rosdep update \
 && rosdep install --from-paths /tmp/src/nav_infrastructure_simple --ignore-src -r -y \
 && rm -rf /var/lib/apt/lists/* /tmp/src

WORKDIR /

