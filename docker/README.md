# gym-gazebo2 Docker container usage

## Build the container

```shell
cd gym-gazebo2/docker
git clone https://github.com/AcutronicRobotics/gym-gazebo2
docker build -t gg2 .
```

## Run the container

```shell
docker rm gg2 || true && docker run -it --name=gg2 -h gym-gazebo2 gg2
```

## Launch gzclient (GUI)
Make sure you have gazebo already installed in your main Ubuntu system. You will also need to get gym-gazebo2.
If you are already running the simulation in the default port, you can access the visual interface the following way:
```shell
git clone https://github.com/AcutronicRobotics/gym-gazebo2
cd gym-gazebo-ros2/docker
sh gzclient.sh
```
