FROM ubuntu:bionic

LABEL maintainer="Lander Usategui <lander at erlerobotics dot com>"

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS2_DISTRO dashing
#Prepare work-space
RUN mkdir -p /root/ros2_mara_ws/src
WORKDIR /root/ros2_mara_ws

RUN \
    echo 'Etc/UTC' > /etc/timezone \
      && ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime \
      && apt-get update -qq && apt-get install -qq -y tzdata dirmngr gnupg2 lsb-release curl \
      # setup ros2 keys
      && apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
      # setup sources.list
      && echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list \
      && apt-get update -qq && apt-get install -qq -y \
        build-essential \
        cmake \
        git \
        python3-colcon-common-extensions \
        python3-pip \
        wget \
      &&  pip3 install \
          tensorflow \
          transforms3d \
          billiard \
          psutil \
      && apt update -qq && apt install -qq -y \
        python3-vcstool \
        python3-numpy \
        python3-sip-dev \
        libeigen3-dev \
        libboost-all-dev \
        ros-$ROS2_DISTRO-ros-base \
        ros-$ROS2_DISTRO-action-msgs \
        ros-$ROS2_DISTRO-message-filters \
        ros-$ROS2_DISTRO-yaml-cpp-vendor \
        ros-$ROS2_DISTRO-urdf ros-$ROS2_DISTRO-rttest ros-$ROS2_DISTRO-tf2 \
        ros-$ROS2_DISTRO-tf2-geometry-msgs \
        ros-$ROS2_DISTRO-rclcpp-action \
        ros-$ROS2_DISTRO-cv-bridge \
        ros-$ROS2_DISTRO-image-transport \
        ros-$ROS2_DISTRO-rmw-opensplice-cpp \
        ros-$ROS2_DISTRO-camera-info-manager \
        libopencv-dev \
      && rm -rf /var/lib/apt/lists/* \
      #Gazebo
      && curl -sSL http://get.gazebosim.org | sh \
      && wget https://raw.githubusercontent.com/AcutronicRobotics/MARA/dashing/mara-ros2.repos && vcs import src < mara-ros2.repos \
      && wget https://raw.githubusercontent.com/AcutronicRobotics/gym-gazebo2/dashing/provision/additional-repos.repos && vcs import src < additional-repos.repos \
      #Generete HRIM packages
      && cd ~/ros2_mara_ws/src/HRIM \
      && pip3 install hrim \
      && hrim generate models/actuator/servo/servo.xml \
      && hrim generate models/actuator/gripper/gripper.xml \
      #Compile the work-space
      && bash -c " cd /root/ros2_mara_ws \
      && source /opt/ros/dashing/setup.bash && colcon build --merge-install --parallel-workers $(nproc) --packages-ignore orocos_kinematics_dynamics individual_trajectories_bridge \
      && touch /root/ros2_mara_ws/install/share/orocos_kdl/local_setup.sh /root/ros2_mara_ws/install/share/orocos_kdl/local_setup.bash \
      && cd /root && git clone https://github.com/openai/gym && cd /root/gym && pip3 install -e . \
      #gym-gazebo2
      && cd /root && git clone -b dashing https://github.com/AcutronicRobotics/gym-gazebo2 && cd /root/gym-gazebo2 && pip3 install -e . \
      #Load provisioning script
      && echo 'source /root/gym-gazebo2/provision/mara_setup.sh' >> ~/.bashrc \
      "
WORKDIR /root/gym-gazebo2/
EXPOSE 11596
ENTRYPOINT ["bash"]
