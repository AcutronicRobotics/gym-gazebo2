FROM ubuntu:bionic

MAINTAINER Lander Usategui lander@erlerobotics.com

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && apt-get install -q -y tzdata && rm -rf /var/lib/apt/lists/*

#Dependencies
RUN apt update -qq && apt install --no-install-recommends -y \
    build-essential \
    cmake \
    git \
    python3-sip-dev \
    python3-numpy \
    python3-pip \
    python3-setuptools \
    wget \
    libasio-dev \
    libtinyxml2-dev \
    lsb-release \
    gnupg2 \
    curl \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install \
    lxml \
    argcomplete \
    tensorflow \
    transforms3d

#ROS2
# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

RUN echo "deb http://packages.ros.org/ros2/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros2-latest.list

RUN apt update -qq && apt install -y \
    python3-colcon-common-extensions \
    python3-pip \
    python-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

#Bootstrap rosdep
RUN rosdep init \
    && rosdep update

#Install ros2 packages
ENV ROS_DISTRO crystal
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-ros-core=0.6.1-0* \
    && rm -rf /var/lib/apt/lists/*

#Gazebo
RUN curl -sSL http://get.gazebosim.org | sh

#Install OpenCV 3.4.5
RUN cd ~/ && git clone -b 3.4.5 https://github.com/opencv/opencv.git && git clone -b 3.4.5 https://github.com/opencv/opencv_contrib.git \
    && cd ~/opencv && mkdir build && cd build/ \
    && cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D INSTALL_C_EXAMPLES=ON -D WITH_TBB=ON -D WITH_V4L=ON -D WITH_OPENGL=ON \
       -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules/ .. \
    && make -j $(nproc) && make install && rm -rf ~/opencv && rm -rf ~/opencv_contrib

#Prepare work-space
RUN mkdir ~/ros2_mara_ws
WORKDIR /home/root/ros2_mara_ws/
#RUN wget https://raw.githubusercontent.com/erlerobot/gym-gazebo2/master/provision/mara.repos && vcs import src < mara.repos #Keep this commented until the repo becomes public
COPY src src
#Generete HRIM packages
RUN cd src/HRIM && python3 hrim.py generate models/actuator/servo/servo.xml \
    && python3 hrim.py generate models/actuator/gripper/gripper.xml

#Compile the work-space
COPY missing-repos.repos .
RUN bash -c "vcs import src < missing-repos.repos || true #Fix me!!"
RUN  bash -c "source /opt/ros/crystal/setup.bash && colcon build --merge-install --parallel-workers $(nproc)"
