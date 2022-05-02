## LSD-SLAM
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












docker run -it --rm \
    --net rosnet \
    --name terminal \
    --env ROS_HOSTNAME=terminal \
    --env ROS_MASTER_URI=http://rosmaster:11311 \
    alpros-test \

```

```

## external-camera
```
cd docker/external-camera/
docker build -t external-camera .
docker run -it --rm --net rosnet --name camera --env ROS_HOSTNAME=camera --env ROS_MASTER_URI=http://rosmaster:11311 --device=/dev/video0:/dev/video0 -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix external-camera rosrun external_camera external_camera
```

## build poc2-bridge
```
cd docker/poc2-bridge/
docker build -t poc2-bridge .
docker run -it --net rosnet --add-host=localhost:192.168.3.6 --name poc2-bridge --env ROS_HOSTNAME=poc2-bridge --env ROS_MASTER_URI=http://rosmaster:11311 poc2-bridge roslaunch uoa_poc2_bridge uoa_poc2_bridge_multi.launch ROBOT_NAME2:="delivery_robot_02"
docker run -it --net rosnet --add-host=localhost:192.168.3.6 --name poc2-bridge --env ROS_HOSTNAME=poc2-bridge --env ROS_MASTER_URI=http://rosmaster:11311 poc2-bridge roslaunch uoa_poc2_bridge uoa_poc2_bridge_single.launch
```


## dummy test
```
cd docker/dummy-robot/
docker build -t dummy-robot .
docker run -it --net rosnet --name dummy-robot --env ROS_HOSTNAME=dummy-robot --env ROS_MASTER_URI=http://rosmaster:11311 dummy-robot rosrun beginner_tutorials command_receiver.py

docker run -it --rm --net rosnet --name dummy-robot --env ROS_HOSTNAME=dummy-robot --env ROS_MASTER_URI=http://rosmaster:11311 dummy-robot rosrun beginner_tutorials command_receiver.py && rosrun beginner_tutorials stopcmd_receiver.py
```


mosquitto_pub -h localhost -p 1883 -d -u iotagent -P password_of_iotagent -t /delivery_robot/delivery_robot_01/cmd -m "{\"send_cmd\":{\"time\":\"111\",\"cmd\":\"navi\",\"waypoints\":[{\"point\":{\"x\":-3.1,\"y\":0.5,\"z\":0},\"angle\":{\"roll\":0,\"pitch\":0,\"yaw\":-0.90}}]}}"




### check topic values
```
docker run -it --rm --net rosnet --name camera-server --env ROS_HOSTNAME=camera-server --env ROS_MASTER_URI=http://rosmaster:11311 external-camera rostopic echo /external_camera_node/pub_topic
```

### change camera state
```
docker run -it --rm --net rosnet --name camera-server --env ROS_HOSTNAME=camera-server --env ROS_MASTER_URI=http://rosmaster:11311 external-camera rostopic pub /external_camera_node/sub_topic external_camera/c_req "{time: 'hello', c_cmd: 'Standby'}"
docker run -it --rm --net rosnet --name camera-server --env ROS_HOSTNAME=camera-server --env ROS_MASTER_URI=http://rosmaster:11311 external-camera rostopic pub /external_camera_node/sub_topic external_camera/c_req "{time: 'hello', c_cmd: 'Monitor'}"
```

## robot-ui(poc2)
```
cd docker/robotui/
docker build -t poc3ui .
docker run -it -d --net rosnet --name poc3ui -p 8080:80 npm run build
```


---
docker build -t ros-camera -f Dockerfile2 .
docker run -it --rm \
    --net rosnet \
    --name camera \
    --env ROS_HOSTNAME=camera \
    --env ROS_MASTER_URI=http://rosmaster:11311 \
    --device=/dev/video6:/dev/video6 \
    --gpus all -e NVIDIA_VISIBLE_DEVICES=0 \
    ros-camera 
---
docker build -t ros-camera -f Dockerfile2 .
docker run -it --rm \
    --net rosnet \
    --name camera \
    --env ROS_HOSTNAME=camera \
    --env ROS_MASTER_URI=http://rosmaster:11311 \
    --device=/dev/video6:/dev/video6 \
	--env="DISPLAY" \
	--env="QT_X11_NO_MITSHM=1" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    ros-camera 
