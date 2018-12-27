# gym-gazebo-ros2: ROS1 to ROS2 migration for MARA.

**ATTENTION**: THIS FILE MUST BE REMOVED ONCE THIS WORK IS PUBLISHED.

The goal is to make a new structure, more convenient and cleaner.
MARA will be the only supported robot, at least until the migration is finished.

**Suggestions and PRs are welcomed.**

## TODO
1. Make MARA work with ROS2 in gym-gazebo-ros2.
2. Remove unnecessary files .i.e content directly not related to MARA or the gym.  
3. Update the structure -> Update references.
4. Testing. Ensure MARA is learning as it used to before the migration.
5. Generate new simple and consistent installation steps.
6. **Tutorials**. Demonstrate de installation and usage of the gym.



### STATE OF THE PROJECT
working on `gazebo_mara_top_3dof_random_ROS2.py` -> `gazebo_mara_top_3dof_ros2_v0.py` -> `gazebo_env_ros2.py`.   

First version of the env init() function ready. Still needs work.

#### MARA specific dependencies. 

Not public:
```
control/msgs erle
baselines / urdf_parser_py (custom fixes for py3) / pip3 lxml
sudo apt-get install python3-sip-dev
orocos_kinematics_dynamics erle ros2 branch
export PYTHONPATH=$PYTHONPATH:/home/nestor/ros2_mara_ws/install/lib/python3/dist-packages

```

NOT WORKING YET.
gzweb specific. ATTENTION , this is likely to cause many problems, so be clear with specific installation instructions. Base instructions:
```
http://gazebosim.org/gzweb.html#gzweb_installation

make sure libgazebo9-dev is installed, or just type
sudo apt-get install libgazebo9-dev

sudo apt install libjansson-dev nodejs nodejs-legacy libboost-dev imagemagick libtinyxml-dev mercurial cmake build-essential

hg up gzweb_1.4.0 . TODO, we could try using 2.0.0 or master branch.

setup_mara alias or add MARA manually to GAZEBO_MODEL_PATH

npm run deploy --- -m

symlink http/client/assets src/MARA

CONNECTING TO GZSERVER

update the GAZEBO_MASTER_URI with the one the gzserver is using. Example:
export GAZEBO_MASTER_URI=http://localhost:13088


```