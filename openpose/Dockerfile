# Based on https://github.com/esemeniuc/openpose-docker
FROM nvidia/cuda:10.0-cudnn7-devel

#get deps
RUN apt-get update && \
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
python3-dev python3-pip git g++ wget make libprotobuf-dev protobuf-compiler libopencv-dev \
libgoogle-glog-dev libboost-all-dev libcaffe-cuda-dev libhdf5-dev libatlas-base-dev

#for python api
RUN pip3 install --upgrade pip setuptools 
RUN pip3 install numpy opencv-python 

#replace cmake as old version has CUDA variable bugs
RUN wget https://github.com/Kitware/CMake/releases/download/v3.16.0/cmake-3.16.0-Linux-x86_64.tar.gz && \
tar xzf cmake-3.16.0-Linux-x86_64.tar.gz -C /opt && \
rm cmake-3.16.0-Linux-x86_64.tar.gz
ENV PATH="/opt/cmake-3.16.0-Linux-x86_64/bin:${PATH}"

#get openpose
WORKDIR /openpose
RUN git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose.git .
RUN cd /openpose && git checkout 16ab72801d9ebfee12903a6678b0e41e7956cb7e

#build it
WORKDIR /openpose/build
RUN cmake -DBUILD_PYTHON=ON .. && make -j `nproc`
WORKDIR /openpose

SHELL ["/bin/bash", "-c"]
RUN apt-get install -y --no-install-recommends lsb-release curl
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update && apt-get install -y ros-melodic-ros-base
RUN apt-get install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
#RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc && source ~/.bashrc

RUN apt-get install -y ros-melodic-cv-bridge ros-melodic-image-transport ros-melodic-image-transport-plugins
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws/
RUN source /opt/ros/melodic/setup.bash && cd src && catkin_init_workspace
RUN source /opt/ros/melodic/setup.bash && catkin_make

WORKDIR /openpose/build
RUN make install


WORKDIR /openpose/testcode/
COPY testcode/ /openpose/testcode/
RUN cmake . && make -j `nproc`
WORKDIR /catkin_ws/
RUN cd src && git clone https://github.com/ravijo/ros_openpose
RUN cd src/ros_openpose/ && git checkout 9d221f9fb571fbb0625401f16946ae8698bdb9c6
COPY ros_openpose_diff/ /catkin_ws/src/ros_openpose/
COPY poseros/ /catkin_ws/src/poseros/
RUN source /opt/ros/melodic/setup.bash && catkin_make

COPY entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
RUN chmod 744 /entrypoint.sh
CMD ["sh"]