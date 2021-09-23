FROM osrf/ros:melodic-desktop-full
LABEL maintainer Kazuki Urabe <urabe0225@gmail.com>

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
                ros-melodic-image-view

COPY entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
RUN chmod 744 /entrypoint.sh
CMD ["sh"]
