# Installation
For the complete MARA experiments installation, please refer first to the **ROS2learn** installation instructions:  [github/acutronicrobotics/ros2learn/Install](https://github.com/acutronicrobotics/ros2learn/blob/master/Install.md).

## Table of Contents
- [ROS 2.0](#ros-20)
- [Dependent tools](#dependent-tools)
- [MARA](#mara)
  - [Create a ROS workspace](#create-a-ros-workspace)
  - [Compile the workspace](#compile-the-workspace)
    - [Ubuntu 18](#ubuntu-18)
  - [OpenAI Gym](#openai-gym)
  - [gym-gazebo2](#gym-gazebo2)
    - [Provisioning](#provisioning)

## ROS 2.0

- **ROS 2 Crystal**.
   - Ubuntu 18: Install ROS 2 Desktop following the official instructions, binaries recommended. [Instructions](https://index.ros.org/doc/ros2/Installation/Linux-Install-Debians/).

## Dependent tools
**Note**: We recommend installing **Gazebo 9.0.0** via **ROS Crystal Debian packages** and removing previous gazebo installations to avoid undesired conflicts, e.g. `apt-get remove *gazebo*`. You can also use different versions of the simulator such as Gazebo 10, but you must skip the installation of `ros-crystal-gazebo*` packages and add [gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/crystal) to the `ros2_mara_ws` we are going to build in the [Create a ROS workspace](#create-a-ros-workspace) section.

```sh
# ROS 2 extra packages
sudo apt update && sudo apt install -y \
ros-crystal-action-msgs \
ros-crystal-message-filters \
ros-crystal-yaml-cpp-vendor \
ros-crystal-urdf \
ros-crystal-rttest \
ros-crystal-tf2 \
ros-crystal-tf2-geometry-msgs \
ros-crystal-rclcpp-action \
ros-crystal-cv-bridge \
ros-crystal-control-msgs \
ros-crystal-image-transport \
ros-crystal-gazebo-dev \
ros-crystal-gazebo-msgs \
ros-crystal-gazebo-plugins \
ros-crystal-gazebo-ros \
ros-crystal-gazebo-ros-pkgs

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

# Install TensorFlow CPU. Feel free to get the GPU version at https://www.tensorflow.org/install/gpu.
pip3 install tensorflow

# Additional utilities
pip3 install transforms3d billiard psutil

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
wget https://raw.githubusercontent.com/AcutronicRobotics/MARA/crystal/mara-ros2.repos
vcs import src < mara-ros2.repos
wget https://raw.githubusercontent.com/AcutronicRobotics/gym-gazebo2/crystal/provision/additional-repos.repos
vcs import src < additional-repos.repos
# Avoid compiling erroneus package
touch ~/ros2_mara_ws/src/orocos_kinematics_dynamics/orocos_kinematics_dynamics/COLCON_IGNORE
```
Generate [HRIM](https://github.com/erlerobot/HRIM) dependencies:

```sh
cd ~/ros2_mara_ws/src/HRIM
sudo pip3 install hrim
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
colcon build --merge-install --packages-skip individual_trajectories_bridge
# Remove warnings
touch ~/ros2_mara_ws/install/share/orocos_kdl/local_setup.sh ~/ros2_mara_ws/install/share/orocos_kdl/local_setup.bash
```
A few packages are expected to throw warning messages. The expected output is the following:

```sh
Summary: 31 packages finished [4min 30s]
  2 packages had stderr output: orocos_kdl python_orocos_kdl
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

If you are using [**ros2learn**](https://github.com/AcutronicRobotics/ros2learn):
```sh
cd ~/ros2learn/environments/gym-gazebo2
pip3 install -e .
```

If not:
```sh
cd ~ && git clone https://github.com/AcutronicRobotics/gym-gazebo2
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

**Note**: This setup file contains paths to ROS and Gazebo used by default by this toolkit. If you installed ROS from sources, you must modify the first line of the provisioning script:

```diff
-  source /opt/ros/crystal/setup.bash
+  source ~/ros2_ws/install/setup.bash
   source ~/ros2_mara_ws/install/setup.bash
   source /usr/share/gazebo-9/setup.sh
   export PYTHONPATH=$PYTHONPATH:~/ros2_mara_ws/install/lib/python3/dist-packages
   export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_mara_ws/src/MARA
   export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/ros2_mara_ws/src/MARA/mara_gazebo_plugins/build/
```
