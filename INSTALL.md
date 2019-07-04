# Installation
For the complete MARA experiments installation, please refer first to the **ROS2Learn** installation instructions:  [github/AcutronicRobotics/ros2learn/Install](https://github.com/AcutronicRobotics/ros2learn/blob/dashing/Install.md).

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

- **ROS 2 Dashing**.
   - Ubuntu 18: Install ROS 2 Desktop following the official instructions, binaries recommended. [Instructions](https://index.ros.org/doc/ros2/Installation/Linux-Install-Debians/).

## Dependent tools
- **Gazebo 9.9.0**.
   - Install the latest available version of Gazebo via [one-liner instructions](http://gazebosim.org/tutorials?tut=install_ubuntu#Defaultinstallation:one-liner). Lower versions like **9.0.0 will not work**. Additional information is available [here](https://github.com/AcutronicRobotics/gym-gazebo2/issues/31#issuecomment-501660211).
     ```sh
     curl -sSL http://get.gazebosim.org | sh
     ```
- ROS 2 extra packages
```sh
sudo apt update && sudo apt install -y \
ros-dashing-action-msgs \
ros-dashing-message-filters \
ros-dashing-yaml-cpp-vendor \
ros-dashing-urdf \
ros-dashing-rttest \
ros-dashing-tf2 \
ros-dashing-tf2-geometry-msgs \
ros-dashing-rclcpp-action \
ros-dashing-cv-bridge \
ros-dashing-image-transport \
ros-dashing-camera-info-manager

# Install OpenSplice RMW implementation. Required for dashing until default FastRTPS is fixed.
sudo apt install ros-dashing-rmw-opensplice-cpp

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
wget https://raw.githubusercontent.com/AcutronicRobotics/MARA/dashing/mara-ros2.repos
vcs import src < mara-ros2.repos
wget https://raw.githubusercontent.com/AcutronicRobotics/gym-gazebo2/dashing/provision/additional-repos.repos
vcs import src < additional-repos.repos
# Avoid compiling erroneus package
touch ~/ros2_mara_ws/src/orocos_kinematics_dynamics/orocos_kinematics_dynamics/COLCON_IGNORE
```

Generate [HRIM](https://github.com/AcutronicRobotics/HRIM/tree/Coliza) dependencies:

```sh
cd ~/ros2_mara_ws/src/HRIM
sudo pip3 install hrim
hrim generate models/actuator/servo/servo.xml
hrim generate models/actuator/gripper/gripper.xml
```
### Compile the workspace

Please make sure you are not sourcing ROS1 workspaces via `bashrc` or any other way. Also make sure you are not sourcing any provisioning script from other ROS2 distribution compliant gym-gazebo2 installation, e.g. gym-gazebo2 `crystal`.

#### Ubuntu 18

Build the workspace using the `--merge-install` flag. Make sure you have enough Swap space.

```sh
source /opt/ros/dashing/setup.bash
cd ~/ros2_mara_ws
colcon build --merge-install --packages-skip individual_trajectories_bridge
# Remove warnings
touch ~/ros2_mara_ws/install/share/orocos_kdl/local_setup.sh ~/ros2_mara_ws/install/share/orocos_kdl/local_setup.bash
```
A few packages are expected to throw warning messages. The expected output is the following:

```sh
Summary: 32 packages finished [13min 0s]
  6 packages had stderr output: hros_cognition_mara_components mara_contact_publisher mara_gazebo_plugins orocos_kdl python_orocos_kdl robotiq_gripper_gazebo_plugins
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
cd ~ && git clone -b dashing https://github.com/AcutronicRobotics/gym-gazebo2
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
**Note**: In Dashing we need to use opensplice implementation of DDS, since Fast-RTPS and others are still buggy and not supported well in this use case. Please export the OpenSplice DDS implementation manually or use the provisioning script before running/training any example of the MARA enviroment.

```sh
export RMW_IMPLEMENTATION=rmw_opensplice_cpp
```

**Note**: This setup file contains paths to ROS and Gazebo used by default by this toolkit. If you installed ROS from sources, you must modify the first line of the provisioning script:

```diff
-  source /opt/ros/dashing/setup.bash
+  source ~/ros2_ws/install/setup.bash
   source ~/ros2_mara_ws/install/setup.bash
   source /usr/share/gazebo-9/setup.sh
   export PYTHONPATH=$PYTHONPATH:~/ros2_mara_ws/install/lib/python3/dist-packages
   export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_mara_ws/src/MARA
   export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/ros2_mara_ws/src/MARA/mara_gazebo_plugins/build/
   export RMW_IMPLEMENTATION=rmw_opensplice_cpp
```
