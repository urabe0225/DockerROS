FROM alpineros/alpine-ros:melodic-ros-core
LABEL maintainer Kazuki Urabe <urabe0225@gmail.com>

SHELL ["/bin/bash", "-c"]

RUN apk add --no-cache ros-melodic-usb-cam

COPY entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
RUN chmod 744 /entrypoint.sh
CMD ["sh"]
