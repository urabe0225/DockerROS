# DockerROS
## using Docker-compose
### Deploy master image
```
cd master/
docker-compose up -d
```

### build test image
```terminal1
cd alpros-test/compose/talker
docker-compose up
```
```terminal2
cd alpros-test/compose/listener
docker-compose up
```

### Camera & Viewer
```
cd camera/
docker-compose up
```

### GUI
```
cd ROSGUI/
docker-compose up
```

### LSD-SLAM
```

```

### OpenPose

---

## using Docker
### create docker network
```terminal1
docker network create rosnet
```

### Deploy master image
```terminal1
docker pull alpineros/alpine-ros:melodic-ros-core
docker run -it -d \
    --net rosnet \
    --name rosmaster \
    -p 80:11311 \
    alpineros/alpine-ros:melodic-ros-core \
    roscore
```

### build test image
```terminal1
cd alpros-test/
docker build -t alpros-test .
docker run -it --rm \
    --net rosnet \
    --name talker \
    --env ROS_HOSTNAME=talker \
    --env ROS_MASTER_URI=http://rosmaster:11311 \
    alpros-test \
    rosrun beginner_tutorials talker.py
```
```terminal2
docker run -it --rm \
    --net rosnet \
    --name listener \
    --env ROS_HOSTNAME=listener \
    --env ROS_MASTER_URI=http://rosmaster:11311 \
    alpros-test \
    rosrun beginner_tutorials listener.py
```

### Camera & Viewer
```terminal1
cd camera/device
docker build -t alpros-camera .
docker run -it --rm \
    --net rosnet \
    --name camera \
    --env ROS_HOSTNAME=camera \
    --env ROS_MASTER_URI=http://rosmaster:11311 \
    --device=/dev/video0:/dev/video0 \
    alpros-camera \
    rosrun usb_cam usb_cam_node _video_device:=/dev/video0
```
```terminal2
cd camera/viewer/
docker build -t ros-view .
docker run -it --rm \
    --net rosnet \
    --name view \
    --env ROS_HOSTNAME=view \
    --env ROS_MASTER_URI=http://rosmaster:11311 \
    -e DISPLAY=$DISPLAY \
    -u `id -u` \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    ros-view \
    rosrun image_view  image_view image:=/usb_cam/image_raw
```

### GUI
```
cd ROSGUI/
docker build -t rosgui .
docker run -p 6080:80 \
    -v /dev/shm:/dev/shm \
    --net rosnet \
    --name rosgui \
    --env ROS_HOSTNAME=rosgui \
    --env ROS_MASTER_URI=http://rosmaster:11311 \
    rosgui
```
- Browse http://127.0.0.1:6080/

### LSD-SLAM
```terminal1
cd docker/lsd_slam/
version="$(glxinfo | grep "OpenGL version string" | rev | cut -d" " -f1 | rev)"
wget http://us.download.nvidia.com/XFree86/Linux-x86_64/"$version"/NVIDIA-Linux-x86_64-"$version".run
mv NVIDIA-Linux-x86_64-"$version".run NVIDIA-DRIVER.run
docker build -t lsd_slam .
```

```terminal1
docker run -it --rm \
    --net rosnet \
    --name camera \
    --env ROS_HOSTNAME=camera \
    --env ROS_MASTER_URI=http://rosmaster:11311 \
    --device=/dev/video0:/dev/video0 \
    alpros-camera \
    rosrun usb_cam usb_cam_node _video_device:=/dev/video0
```

```terminal2
docker run -it --rm \
    --net rosnet \
    --name viewer \
    --env ROS_HOSTNAME=viewer \
    --env ROS_MASTER_URI=http://rosmaster:11311 \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --gpus all -e NVIDIA_VISIBLE_DEVICES=0 \
    lsd_slam \
    rosrun lsd_slam_viewer viewer
```

```terminal3
docker run -it --rm \
    --net rosnet \
    --name ext \
    --env ROS_HOSTNAME=lsd \
    --env ROS_MASTER_URI=http://rosmaster:11311 \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    lsd_slam \
    rosrun lsd_slam_core live_slam image:=/usb_cam/image_raw camera_info:=/usb_cam/camera_info
```

### OpenPose
```terminal1
docker run -it --rm \
    --net rosnet \
    --name camera \
    --env ROS_HOSTNAME=camera \
    --env ROS_MASTER_URI=http://rosmaster:11311 \
    --device=/dev/video0:/dev/video0 \
    alpros-camera \
    rosrun usb_cam usb_cam_node _video_device:=/dev/video0
```
```terminal2
cd docker/openpose/
docker build -t poseros .
docker run -it --rm \
    --net rosnet \
    --name pose \
    --env ROS_HOSTNAME=pose \
    --env ROS_MASTER_URI=http://rosmaster:11311 \
    --device=/dev/video0:/dev/video0 \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --gpus all  \
    -e NVIDIA_VISIBLE_DEVICES=0  \
    poseros \
    roslaunch ros_openpose run.launch camera:=nodepth
```

