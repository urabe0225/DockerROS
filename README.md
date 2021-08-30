# DockerROS

# deploy on Docker containers
## create docker network
```
docker network create rosnet
```

## Deploy master image
```
docker run -it -d --net rosnet --name rosmaster -p 80:11311 alpineros/alpine-ros:melodic-ros-core roscore
```

## build test image
```
cd docker/alpros-test/
docker build -t alpros-test .
docker run -it --rm --net rosnet --name talk --env ROS_HOSTNAME=talk --env ROS_MASTER_URI=http://rosmaster:11311 alpros-test rosrun beginner_tutorials talker.py
docker run -it --rm --net rosnet --name listen --env ROS_HOSTNAME=listen --env ROS_MASTER_URI=http://rosmaster:11311 alpros-test rosrun beginner_tutorials listener.py
```

