FROM osrf/ros:melodic-desktop-full
LABEL maintainer Kazuki Urabe <urabe0225@gmail.com>

ENV DEBIAN_FRONTEND noninteractive

ARG UID=1000 # ホストのユーザーIDと一致させる

RUN apt-get update && apt-get install -y \
                ros-melodic-image-view \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* \
    && useradd -m -u ${UID} user
