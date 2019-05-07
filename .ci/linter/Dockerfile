FROM ubuntu:bionic

LABEL maintainer="Lander Usategui <lander at erlerobotics dot com>"

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

RUN \
    echo 'Etc/UTC' > /etc/timezone \
      && ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime \
      && apt-get update -qq && apt-get install -qq -y \
        python3-pip \
        python3-numpy \
        python3-sip-dev \
        libeigen3-dev \
      && rm -rf /var/lib/apt/lists/* \
      &&  pip3 install \
        tensorflow \
        transforms3d \
        billiard \
        psutil \
        pylint
COPY ./docker-entrypoint.sh /usr/local/bin/docker-entrypoint.sh
WORKDIR /gym-gazebo2
ENTRYPOINT ["docker-entrypoint.sh"]
