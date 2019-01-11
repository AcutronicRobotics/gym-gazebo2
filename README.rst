gym-gazebo
**********

**gym-gazebo is a toolkit for developing reinforcement learning algotihms using ROS 2 and Gazebo.** Initially built as an extension for the `OpenAI Gym <https://github.com/openai/gym>`_ , now adopts a stand alone structure while mantaining the core concepts of the Gym.


  The initial version, which presented the project as an extension of the initial OpenAI gym for robotics using ROS and Gazebo can be found in the ``/gym-gazebo-old?`` branch of this repository. This version is available "as it is" and currently it is not supported by any specific organization, but community support is available `here <https://github.com/erlerobot/gym-gazebo/issues>`_. A whitepaper about that work is available at https://arxiv.org/abs/1608.05742. Please use the following BibTex entry to cite our work::

    @article{zamora2016extending,
      title={Extending the OpenAI Gym for robotics: a toolkit for reinforcement learning using ROS and Gazebo},
      author={Zamora, Iker and Lopez, Nestor Gonzalez and Vilches, Victor Mayoral and Cordero, Alejandro Hernandez},
      journal={arXiv preprint arXiv:1608.05742},
      year={2016}
    }

**gym-gazebo** is a complex piece of software for roboticists that puts together simulation tools, robot middlewares (ROS, ROS 2), machine learning and reinforcement learning techniques. All together to create an environment whereto benchmark and develop behaviors with robots. Setting up `gym-gazebo` appropriately requires relevant familiarity with these tools.

.. contents:: **Contents of this document**
   :depth: 2
   
Environments
============
The code for each environment group is housed in its own subdirectory
`gym/envs <https://github.com/erlerobot/gym-gazebo/blob/master/gym_gazebo/envs>`_. Robot specific simulation files should be housed in robot specific ROS2 packages.

MARA
----
Real Robot
~~~~~~~~~
MARA stands for Modular Articulated Robotic Arm and is a collaborative robotic arm with ROS 2.0 in each actuator, sensor or any other representative module. Each module has native ROS 2.0 support, can be physically extended in a seamless manner and delivers industrial-grade features including synchronization, deterministic communication latencies, a ROS 2.0 software and hardware component lifecycle, and more. Altogether, MARA empowers new possibilities and applications in the professional landscape of robotics. Learn more or even order one at `acutronicrobotics.com <https://acutronicrobotics.com/>`_!

.. image:: https://acutronicrobotics.com/docs/user/pages/02.Products/01.MARA/MARA2.jpg

Simulated Robot
~~~~~~~~~~~~~~
MARA also provides an accurate simulated version in Gazebo, which allows to translate behaviors from the simulated environment to the real robot. This is the version we will be training in gym-gazebo.

.. image:: imgs/mara_2.gif

Please refer to `acutronicrobotics.com/docs <https://acutronicrobotics.com/docs/products/mara>`_ for additional content such as tutorials or product specifications.

Installation
============

Install ROS 2.0 and Gazebo 9.6
------------------------------

- **Gazebo 9.6**. Install Gazebo 9.6 following the official one-liner installation instructions. `Instructions <http://gazebosim.org/tutorials?tut=install_ubuntu#Defaultinstallation:one-liner>`_.
- **ROS 2 Crystal**.
   - Ubuntu 18: Install ROS 2.0 following the official instructions, binaries recommended. `Instructions <https://index.ros.org/doc/ros2/Linux-Install-Debians/>`_.
   - Ubuntu 16: Install ROS 2.0 following the official instructions, sources required. `Instructions <https://index.ros.org/doc/ros2/Linux-Development-Setup/>`_.

MARA ROS 2.0 specific instructions
----------------------------------

Following folder naming is recommended!

**TODO. TEMPORARY NOTE 1:** gym-gazebo-ros2 is private for now and it's temporary location is: https://github.com/erlerobot/gym-gazebo-ros2 . Take that into consideration while following the instructions.

Install development tools, ROS tools
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: shell

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
    pip3 install lxml tensorflow
    # Fast-RTPS dependencies
    sudo apt install --no-install-recommends -y \
      libasio-dev \
      libtinyxml2-dev

Create a ROS workspace 
~~~~~~~~~~~~~~~~~~~~~~
Create the workspace and download source files:

.. code:: shell

    mkdir -p ~/ros2_mara_ws/src
    cd ~/ros2_mara_ws
    wget https://raw.githubusercontent.com/erlerobot/gym-gazebo/master/mara.repos
    vcs import src < mara.repos
    # Avoid compiling erroneus package
    touch ~/ros2_mara_ws/src/orocos_kinematics_dynamics/orocos_kinematics_dynamics/COLCON_IGNORE

Generate HRIM dependencies:

.. code:: shell

    cd ~/ros2_mara_ws/src/HRIM
    python3 hrim.py generate models/actuator/servo/servo.xml
    python3 hrim.py generate models/actuator/gripper/gripper.xml

Compile the workspace
~~~~~~~~~~~~~~~~~~~~~

Ubuntu 18
^^^^^^^^^
Build the workspace using the ``--merge-install`` flag.

.. code:: shell

    source /opt/ros/crystal/setup.bash
    cd ~/ros2_mara_ws
    colcon build --merge-install
    # Remove warnings
    touch ~/ros2_mara_ws/install/share/orocos_kdl/local_setup.sh ~/ros2_mara_ws/install/share/orocos_kdl/local_setup.bash

A few packages are expected to throw warning messages. The expected output is the following:

.. code:: shell

    Summary: 53 packages finished [12min 41s]
    5 packages had stderr output: cv_bridge mara_gazebo_plugins orocos_kdl python_orocos_kdl robotiq_140_gripper_gazebo_plugin

Ubuntu 16
^^^^^^^^^

Compilation dependencies:

.. code:: shell

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
    
Build the workspace using the ``--merge-install`` flag.

.. code:: shell

    source ~/ros2_ws/install/setup.bash
    cd ~/ros2_mara_ws
    colcon build --merge-install
    # Remove warnings
    touch ~/ros2_mara_ws/install/share/orocos_kdl/local_setup.sh ~/ros2_mara_ws/install/share/orocos_kdl/local_setup.bash

Install Baselines
~~~~~~~~~~~~~~~~~
**TODO. This is a private repo.** A simplified version of the repo should be published.

.. code:: shell

    cd ~
    git clone https://github.com/erlerobot/baselines
    cd baselines
    pip3 install -e .

Install URDF Parser
~~~~~~~~~~~~~~~~~~~
Standalone URDF parser for Python3.

.. code:: shell

    cd ~
    git clone https://github.com/ros/urdf_parser_py.git -b melodic-devel
    cd urdf_parser_py
    pip3 install -e .

Install OpenAI Gym
~~~~~~~~~~~~~~~~~~
Gym should be installed with the latest version, which means using the source code:

.. code:: shell

    cd ~
    git clone https://github.com/openai/gym
    cd gym
    pip3 install -e .
    
Install Gym-Gazebo
~~~~~~~~~~~~~~~~~~
Install this repository.

.. code:: shell

    cd ~
    git clone https://github.com/erlerobot/gym-gazebo
    cd gym-gazebo
    pip3 install -e .

Usage
=====

Executing an algorithm
----------------------
First we need setup ROS2, MARA ROS2 workspace and Gazebo.

.. code:: shell

    # Ubuntu 18
    source /opt/ros/crystal/setup.bash
    # Ubuntu 16
    source ~/ros_ws/install/setup.bash
    
    source ~/ros2_mara_ws/install/setup.bash
    source /usr/share/gazebo/setup.sh

Now we need to add our python library folder inside MARA ROS2 workspace to the PYTHONPATH. This is required as some libraries like PyKDL are located here.

.. code:: shell

    export PYTHONPATH=$PYTHONPATH:~/ros2_mara_ws/install/lib/python3/dist-packages

Now that out environment is setup, we can execute the algorithm.

.. code:: shell

    cd ~/gym-gazebo-ros2/examples/MARA
    python3 gazebo_mara_top_3dof_4actions_ROS2.py

Tips
----

alias
~~~~~

You can use an alias to simplify the process. Note that GAZEBO_MODEL_PATH and GAZEBO_PLUGIN_PATH are included here as you will need them if you want to call the ``gzclient`` from a different terminal. The alias contains environment variables, so be careful and past the following directly to your ``.bashrc``. 

Note: Update ROS 2 source to ``~/ros_ws/installation/setup.bash`` if you are in Ubuntu 16.

.. code:: shell

    alias setup_mara='source /opt/ros/crystal/setup.bash ; source ~/ros2_mara_ws/install/setup.bash ; source /usr/share/gazebo/setup.sh ; export PYTHONPATH=$PYTHONPATH:~/ros2_mara_ws/install/lib/python3/dist-packages ; export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_mara_ws/src/MARA ; export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/ros2_mara_ws/src/MARA/mara_gazebo_plugins/build/'

gzserver/gzclient
~~~~~~~~~~~~~~~~~

If you want to get faster simulation speeds, you should launch the simulation withouht the ``gzclient``, which is the visual interface of gazebo. In order to do so, you must set to ``True`` the ``gzserver_only`` variable located in the ``__init__`` function of the corresponding MARA environment. 

Steps to launch the GUI:

- Open a new terminal.
- Setup the environment using the `alias <#alias>`_. 

.. code:: shell

    setup_mara

- Set the corresponding GAZEBO_MASTER_URI: For convinience, this environment variable is printed at the beginning of every Env execution. Just copy and export it. Example:

.. code:: shell

    export GAZEBO_MASTER_URI=http://localhost:11285

- Finally launch the client:

.. code:: shell

    gzclient

Final note: you can launch as many ``gzserver``s and ``gzclient``s as you want as long as you take into account the GAZEBO_MASTER_URI environment.

What's new
==========
- 2018-12-31: Release of gym-gazebo with ROS2 compatibility and MARA environments.
