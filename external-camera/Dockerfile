FROM ros:kinetic
MAINTAINER Kazuki Urabe <urabe0225@gmail.com>

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y libopencv-dev
RUN rm -rf /var/lib/apt/lists/
RUN mkdir -p /root/catkin_ws/src

WORKDIR /root/catkin_ws/
RUN source /opt/ros/kinetic/setup.sh && cd src && catkin_init_workspace
#RUN cd src && git clone https://github.com/ogcaizu/uoa_ex_cam_sys_ros
RUN cd src && catkin_create_pkg external_camera std_msgs rospy roscpp message_generation
RUN source /opt/ros/kinetic/setup.sh && catkin_make
COPY src/ /root/catkin_ws/src/external_camera/src/
COPY msg/ /root/catkin_ws/src/external_camera/msg/
COPY CMakeFiles.txt /root/catkin_ws/src/external_camera/CMakeLists.txt
COPY camera.xml /root/catkin_ws/src/external_camera/camera.xml
RUN source /opt/ros/kinetic/setup.sh && catkin_make
#COPY scripts/ /root/catkin_ws/src/beginner_tutorials/scripts/

RUN apt-get update && apt-get install -y libcanberra-gtk*
RUN rm -rf /var/lib/apt/lists/


COPY entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
RUN chmod 744 /entrypoint.sh
CMD ["sh"]
