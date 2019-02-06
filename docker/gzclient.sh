#!bin/sh
. /usr/share/gazebo/setup.sh
docker_path="$PWD"
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$docker_path/src/MARA
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$docker_path/src/MARA/mara_gazebo_plugins/build/
export GAZEBO_MASTER_IP=$(docker inspect --format '{{ .NetworkSettings.IPAddress }}' gg2)
export GAZEBO_MASTER_URI=$GAZEBO_MASTER_IP:11345
gzclient
