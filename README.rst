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

Install ROS 2.0
---------------

- **ROS 2 Crystal**. Install ROS 2.0 following the official instructions, binaries recommended. `Instructions <https://index.ros.org/doc/ros2/Linux-Install-Debians/>`_.

MARA ROS 2.0 specific instructions
----------------------------------

Following folder naming is recommended!

**TODO. TEMPORARY NOTE 1:** gym-gazebo-ros2 is private for now and it's temporary location is: https://github.com/erlerobot/gym-gazebo-ros2 . Take that into consideration while following the instructions.

Create a ROS workspace 
~~~~~~~~~~~~~~~~~~~~~~
Create the workspace and download source files:

.. code:: shell

    mkdir -p ~/ros2_mara_ws/src
    cd ~/ros2_mara_ws/src
    wget https://raw.githubusercontent.com/erlerobot/gym-gazebo/master/mara.repos
    vcs import src < mara.repos

Install a couple compilation-required dependencies:

.. code:: shell

    pip3 install lxml
    sudo apt-get install python3-sip-dev

Compile the workspace
~~~~~~~~~~~~~~~~~~~~~
Build the workspace using the ``--merge-install`` flag.

.. code:: shell

    source /opt/ros/crystal/setup.bash
    cd ~/ros2_mara_ws
    colcon build --merge-install

Install OpenAI Gym
~~~~~~~~~~~~~~~~~~
Gym should be installed with the latest version, which means using the source code:

.. code:: shell

    cd ~
    git clone https://github.com/openai/gym
    cd gym
    pip3 install -e .

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
**TODO. @nzlz own repo** containing fix for python3.

.. code:: shell

    cd ~
    git clone https://github.com/nzlz/urdf_parser_py -b nestor-fix-crystal
    cd urdf_parser_py
    pip3 install -e .

Usage
=====

Executing an algorithm
----------------------
First we need setup ROS2, MARA ROS2 workspace and Gazebo.

.. code:: shell

    source /opt/ros/crystal/setup.bash
    source ~/ros2_mara_ws/install/setup.bash
    source /usr/share/gazebo/setup.sh

Now we need to add our python library folder inside MARA ROS2 workspace to the PYTHONPATH. This is required as some libraries like PyKDL are located here.

.. code:: shell

    export PYTHONPATH=$PYTHONPATH:~/ros2_mara_ws/install/lib/python3/dist-packages

Now that out environment is setup, we can execute the algorithm.

.. code:: shell

    cd ~/gym-gazebo-ros2/examples/MARA
    python3 gazebo_mara_top_3dof_random_ROS2.py

Tips
----

alias
~~~~~

You can use an alias to simplify the process. Note that GAZEBO_MODEL_PATH and GAZEBO_PLUGIN_PATH are included here as you will need them if you want to call the ``gzclient`` from a different terminal.

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
