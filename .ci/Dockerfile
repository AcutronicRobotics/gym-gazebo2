FROM ubuntu:bionic

LABEL maintainer="Lander Usategui <lander at erlerobotics dot com>"

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

RUN \
      apt-get update -qq && apt-get install -qq -y \
         python3-pip \
         python3-numpy \
         python3-sip-dev \
         libeigen3-dev \
         && rm -rf /var/lib/apt/lists/* \
 	&&  pip3 install \
        	tensorflow==1.12 \
        	transforms3d \
        	billiard \
                pylint
COPY ./docker-entrypoint.sh /usr/local/bin/docker-entrypoint.sh
ENTRYPOINT ["docker-entrypoint.sh"]
