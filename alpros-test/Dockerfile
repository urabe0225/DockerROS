FROM alpineros/alpine-ros:melodic-ros-core
LABEL maintainer Kazuki Urabe <urabe0225@gmail.com>

SHELL ["/bin/bash", "-c"]

RUN apk add --no-cache cmake make gcc g++
RUN mkdir -p /root/catkin_ws/src

WORKDIR /root/catkin_ws/
RUN source /usr/ros/melodic/setup.sh && cd src && catkin_init_workspace
RUN source /usr/ros/melodic/setup.sh && catkin_make
RUN cd src && catkin_create_pkg beginner_tutorials std_msgs rospy
RUN source /usr/ros/melodic/setup.sh && catkin_make
COPY scripts/ /root/catkin_ws/src/beginner_tutorials/scripts/

COPY entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
RUN chmod 744 /entrypoint.sh
CMD ["sh"]
