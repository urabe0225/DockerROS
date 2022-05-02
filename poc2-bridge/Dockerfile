FROM ros:kinetic
LABEL maintainer Kazuki Urabe <urabe0225@gmail.com>

SHELL ["/bin/bash", "-c"]

ENV MQTT_HOST "localhost"
ENV MQTT_PORT "1883"
ENV MQTT_USERNAME "iotagent"
ENV MQTT_PASSWORD "password_of_iotagent"
ENV MQTT_USE_CA "false"
ENV ENTITY_TYPE "delivery_robot"
ENV ENTITY_ID "delivery_robot_01"
ENV ENTITY_ID1 "delivery_robot_01"
ENV ENTITY_ID2 "delivery_robot_02"
ENV CMD_NAME "send_cmd"
ENV EMG_NAME "send_emg"
ENV ROBOT_NAME "delivery_robot_01"
ENV ROBOT_NAME1 "delivery_robot_01"
ENV ROBOT_NAME2 "delivery_robot_02"
ENV NODE_SUFFIX "1"
ENV TIMEZONE "Asia/Tokyo"
ENV SEND_DELTA_MILLISEC "1000"
ENV AUTO_CMDEXE ""

RUN source /opt/ros/kinetic/setup.bash && \
    mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src && \
    catkin_init_workspace && cd ~/catkin_ws/src && \
    git clone https://github.com/RoboticBase/uoa_poc2_bridge && \
    git clone https://github.com/RoboticBase/uoa_poc2_msgs && \
    cd ~/catkin_ws && catkin_make

RUN apt-get update && apt-get install -y
RUN apt-get install -y python-pip && \
    pip install paho-mqtt pytz && \
    rm -rf /var/lib/apt/lists/

COPY entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
RUN chmod 744 /entrypoint.sh
CMD ["sh"]
