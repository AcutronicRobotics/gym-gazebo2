FROM ros:crystal-ros-base-bionic

MAINTAINER Lander Usategui lander@erlerobotics.com

RUN pip3 install \
    lxml \
    argcomplete \
    tensorflow \
    transforms3d

RUN apt update -qq && apt install -y \
    python3-colcon-common-extensions \
    curl \
    python3-vcstool \
    python3-numpy \
    python3-sip-dev \
    libeigen3-dev \
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
RUN bash -c "vcs import src < missing-repos.repos || true # Fix me!!"
RUN  bash -c "source /opt/ros/crystal/setup.bash && colcon build --merge-install --parallel-workers $(nproc) --packages-ignore orocos_kinematics_dynamics tlsf_cpp" 
