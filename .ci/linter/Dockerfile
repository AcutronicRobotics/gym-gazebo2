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
        tensorflow==1.12 \
        transforms3d \
        billiard \
        psutil \
        pylint \
RUN echo -e '#!/bin/bash\nset -e\n\npylint --rcfile linter/.pylintrc gym_gazebo2\n\nres=$?\n\nif [ $res -ne 0 ]; then\n\techo "Linter error"\n\texit 123\nfi'> /usr/local/bin/docker-entrypoint.sh
RUN chmod +x /usr/local/bin/docker-entrypoint.sh
WORKDIR /gym-gazebo2
ENTRYPOINT ["docker-entrypoint.sh"]
