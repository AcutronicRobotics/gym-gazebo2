# Installation
For the complete MARA experiments installation, please refer first to the **ROS2learn** installation instructions:  [github/acutronicrobotics/ros2learn/Install](https://github.com/acutronicrobotics/ros2learn/blob/ros2/Install.md).

## Table of Contents
- [ROS 2.0 and Gazebo 9.6](#ros2-and-gazebo)
- [Dependent tools](#dependent-tools)
- [MARA](#mara)
  - [Create a ROS workspace](#create-a-ros-workspace)
  - [Compile the workspace](#compile-the-workspace)
    - [Ubuntu 18](#ubuntu-18)
    - [Ubuntu 16](#ubuntu-16)
  - [Baselines](#baselines)
  - [URDF Parser](#urdf-parser)
  - [OpenAI Gym](#openai-gym)
  - [gym-gazebo2](#gym-gazebo2)
    - [Provisioning](#provisioning)

## ROS2 and Gazebo

- **Gazebo 9.6**. Install Gazebo 9.6 following the official one-liner installation instructions. [Instructions](http://gazebosim.org/tutorials?tut=install_ubuntu#Defaultinstallation:one-liner).
- **ROS 2 Crystal**.
   - Ubuntu 18: Install ROS 2.0 following the official instructions, binaries recommended. [Instructions](https://index.ros.org/doc/ros2/Installation/Linux-Install-Debians/).
   - Ubuntu 16: Install ROS 2.0 following the official instructions, sources required. [Instructions](https://index.ros.org/doc/ros2/Installation/Linux-Development-Setup/).

## Dependent tools

```sh
sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-pip \
  python-rosdep \
  python3-vcstool \
  python3-sip-dev \
  python3-numpy \
  wget

pip3 install tensorflow==1.12
pip3 install lxml transforms3d billiard

# Fast-RTPS dependencies
sudo apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev
```
## MARA

<a href="http://www.acutronicrobotics.com"><img src="https://acutronicrobotics.com/products/mara/images/xMARA_evolution_end.jpg.pagespeed.ic.dVNwzZ6-4i.webp" float="left" hspace="2" vspace="2" width="300"></a>

Following folder naming is recommended!

### Create a ROS workspace

Create the workspace and download source files:

```sh
mkdir -p ~/ros2_mara_ws/src
cd ~/ros2_mara_ws
wget https://raw.githubusercontent.com/erlerobot/gym-gazebo2/master/provision/mara.repos
vcs import src < mara.repos
# Avoid compiling erroneus package
touch ~/ros2_mara_ws/src/orocos_kinematics_dynamics/orocos_kinematics_dynamics/COLCON_IGNORE
```
Generate [HRIM](https://github.com/erlerobot/HRIM) dependencies:

```sh
cd ~/ros2_mara_ws/src/HRIM
pip3 install -e installator
hrim generate models/actuator/servo/servo.xml
hrim generate models/actuator/gripper/gripper.xml
```
### Compile the workspace

Please make sure you are not sourcing ROS1 workspaces via `bashrc` or any other way.

#### Ubuntu 18

Build the workspace using the `--merge-install` flag. Make sure you have enough Swap space.

```sh
source /opt/ros/crystal/setup.bash
cd ~/ros2_mara_ws
colcon build --merge-install
# Remove warnings
touch ~/ros2_mara_ws/install/share/orocos_kdl/local_setup.sh ~/ros2_mara_ws/install/share/orocos_kdl/local_setup.bash
```
A few packages are expected to throw warning messages. The expected output is the following:

```sh
35 packages finished [12min 26s]
4 packages had stderr output: cv_bridge orocos_kdl python_orocos_kdl robotiq_gripper_gazebo_plugins
```
<!--#### Ubuntu 16

Compilation dependencies:

```sh
# OpenCV 3, cv_bridge requirement
OPENCV_VERSION='3.4.2'
sudo apt-get install -y unzip wget
wget https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip
unzip ${OPENCV_VERSION}.zip
rm ${OPENCV_VERSION}.zip
mv opencv-${OPENCV_VERSION} OpenCV
cd OpenCV
mkdir build
cd build
cmake -DWITH_QT=ON -DWITH_OPENGL=ON -DFORCE_VTK=ON -DWITH_TBB=ON -DWITH_GDAL=ON -DWITH_XINE=ON -DBUILD_EXAMPLES=ON -DENABLE_PRECOMPILED_HEADERS=OFF ..
make -j4
sudo make install
sudo ldconfig

# image_transport requirement
sudo apt install libpcre3-dev
```-->

Build the workspace using the `--merge-install` flag.

```sh
source ~/ros2_ws/install/setup.bash
cd ~/ros2_mara_ws
colcon build --merge-install
# Remove warnings
touch ~/ros2_mara_ws/install/share/orocos_kdl/local_setup.sh ~/ros2_mara_ws/install/share/orocos_kdl/local_setup.bash
```
### Baselines

Baselines is a fork of OpenAI's baselines repository with a set of high-quality implementations of reinforcement learning algorithms. The algorithms are modified to be used in robotics.

```sh
#Skip if using gym-gazebo2 as ROS2Learn submodule (ros2learn/algorithms/baselines).
cd ~ && git clone https://github.com/erlerobot/baselines
#Navigate to module's root directory
cd baselines
pip3 install -e .
```
### URDF Parser

Standalone URDF parser for Python3.

```sh
cd ~
git clone https://github.com/ros/urdf_parser_py.git -b melodic-devel
cd urdf_parser_py
pip3 install -e .
```
### OpenAI Gym

It is recommended to install Gym's latest version, which means using the source code. If you already installed Gym via pip3, you can uninstall it via `pip3 uninstall gym` to avoid overlapping:

```sh
cd ~
git clone https://github.com/openai/gym
cd gym
pip3 install -e .
```
### gym-gazebo2

Install the gym-gazebo2 toolkit.

```sh
#Skip if using gym-gazebo2 as ROS2Learn submodule (ros2learn/environments/gym-gazebo2).
cd ~ && git clone https://github.com/erlerobot/gym-gazebo2
#Navigate to module's root directory
cd gym-gazebo2
pip3 install -e .
```
#### Provisioning

First we need setup ROS2, MARA ROS2 workspace and Gazebo. It is convenient that the required environment variables are automatically added to your bash session every time a new shell is launched:

```sh
#Navigate to module's root directory
cd gym-gazebo2
echo "source `pwd`/provision/mara_setup.sh" >> ~/.bashrc
source ~/.bashrc
```

**Note**: This setup file contains paths to ROS and Gazebo used by default by this toolkit. If you installed ROS from sources (e.g: Ubuntu16 installation), you must modify the first line of the provisioning script:

```diff
-  source /opt/ros/crystal/setup.bash
+  source ~/ros_ws/install/setup.bash
   source ~/ros2_mara_ws/install/setup.bash
   source /usr/share/gazebo-9/setup.sh
   export PYTHONPATH=$PYTHONPATH:~/ros2_mara_ws/install/lib/python3/dist-packages
   export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_mara_ws/src/MARA
   export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/ros2_mara_ws/src/MARA/mara_gazebo_plugins/build/
```
