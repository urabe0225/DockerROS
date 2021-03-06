FROM osrf/ros:indigo-desktop-full
LABEL maintainer Kazuki Urabe <urabe0225@gmail.com>

ENV DEBIAN_FRONTEND=noninteractive

RUN mkdir -p /opt/rosbuild_ws/package_dir
WORKDIR /opt/rosbuild_ws/

RUN apt-get update
RUN apt-get install -y mesa-utils binutils
ADD NVIDIA-DRIVER.run /tmp/NVIDIA-DRIVER.run
RUN sh /tmp/NVIDIA-DRIVER.run -a -N --ui=none --no-kernel-module
RUN rm /tmp/NVIDIA-DRIVER.run

RUN apt-get install -y ros-indigo-libg2o \
	ros-indigo-cv-bridge \
	liblapack-dev \
	libblas-dev \
	freeglut3-dev \
	libqglviewer-dev \
	libsuitesparse-dev \
	libx11-dev

RUN cd ~ && mkdir rosbuild_ws && cd rosbuild_ws && \
	rosws init . /opt/ros/indigo && \
	mkdir package_dir && \
	rosws set -y ~/rosbuild_ws/package_dir -t . && \
	echo 'source ~/rosbuild_ws/setup.bash' >> ~/.bashrc && \
	cd package_dir && \
	git clone https://github.com/tum-vision/lsd_slam.git lsd_slam

COPY parameter_generator.py /opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/
RUN /bin/bash -c " \
	rosdep update && \
	source ~/rosbuild_ws/setup.bash && \
	cd ~/rosbuild_ws && \
	rosmake lsd_slam"

RUN apt-get install -y wget tmux unzip

RUN apt-get install -y ros-indigo-usb-cam
RUN apt-get install -y libcanberra-gtk-module libcanberra-gtk3-module
RUN mkdir /root/.ros/camera_info


RUN sudo rm -rf /var/lib/apt/lists/*

COPY entrypoint.sh /tmp
ENTRYPOINT ["/tmp/entrypoint.sh"]
RUN chmod 755 /tmp/entrypoint.sh
