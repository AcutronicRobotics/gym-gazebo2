FROM ros:crystal-ros-base-bionic

LABEL maintainer="Lander Usategui <lander at erlerobotics dot com>"

ENV ROS_DISTRO crystal
#Prepare work-space
RUN mkdir -p /root/ros2_mara_ws/src
WORKDIR /root/ros2_mara_ws
COPY missing-repos.repos .
#Baselines
#RUN cd /home/root && git clone https://github.com/AcutronicRobotics/baselines && cd /home/root/baselines && pip3 install -e .
COPY baselines /root/baselines
#RUN cd /home/root && git clone https://github.com/AcutronicRobotics/gym-gazebo2 && pip3 install -e .
COPY gym-gazebo2 /root/gym-gazebo2

RUN \
  pip3 install \
        argcomplete \
        tensorflow==1.12 \
        transforms3d \
        billiard \
  && apt update -qq && apt install -qq -y \
    python3-colcon-common-extensions \
    curl \
    python3-vcstool \
    python3-numpy \
    python3-sip-dev \
    libeigen3-dev \
    wget \
    libboost-all-dev \
    ros-$ROS_DISTRO-message-filters ros-$ROS_DISTRO-yaml-cpp-vendor \
    ros-crystal-urdf ros-$ROS_DISTRO-rttest ros-$ROS_DISTRO-tf2 \
    ros-$ROS_DISTRO-tf2-geometry-msgs libopencv-dev \
    && rm -rf /var/lib/apt/lists/* \
  #Gazebo
  && curl -sSL http://get.gazebosim.org | sh \
  && wget https://raw.githubusercontent.com/AcutronicRobotics/MARA/master/mara-ros2.repos && vcs import src < mara-ros2.repos \
  #Generete HRIM packages
  && cd ~/ros2_mara_ws/src/HRIM/installator \
  && python3 setup.py install && cd .. \
  && hrim generate models/actuator/servo/servo.xml \
  && hrim generate models/actuator/gripper/gripper.xml \
  #Compile the work-space
  && bash -c "cd /root/ros2_mara_ws && vcs import src < missing-repos.repos \
  && source /opt/ros/crystal/setup.bash && colcon build --merge-install --parallel-workers $(nproc) --packages-ignore orocos_kinematics_dynamics \
  && touch /root/ros2_mara_ws/install/share/orocos_kdl/local_setup.sh /root/ros2_mara_ws/install/share/orocos_kdl/local_setup.bash \
  && cd /root/ && git clone https://github.com/ros/urdf_parser_py.git -b melodic-devel && cd /root/urdf_parser_py && pip3 install -e . \
  && cd /root/baselines && pip3 install -e . \
  && cd /root && git clone https://github.com/openai/gym && cd /root/gym && pip3 install -e . \
  #gym-gazebo2
  && cd /root/gym-gazebo2 && pip3 install -e . \
  #Load provisioning script
  && echo 'source ~/gym-gazebo2/provision/mara_setup.sh' >> ~/.bashrc \
  && source ~/.bashrc"
WORKDIR /root/gym-gazebo2/
EXPOSE 11596
ENTRYPOINT ["bash"]
