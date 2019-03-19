# gym-gazebo2 Docker container usage

## Build the container

```shell
git clone https://github.com/AcutronicRobotics/gym-gazebo2
cd gym-gazebo2/docker
docker build -t gg2 .
```

## Run the container

```shell
docker rm gg2 || true && docker run -it --name=gg2 -h gym-gazebo2 -v /tmp:/tmp gg2
cp -r /root/ros2_mara_ws /tmp #Inside the docker container
cd examples/MARA && python3 gg_random.py #Run the example
```

## Launch gzclient (GUI)
Make sure you have gazebo already installed in your main Ubuntu system. You will also need to get gym-gazebo2.
If you are already running the simulation in the default port, you can access the visual interface the following way:
```shell
git clone https://github.com/AcutronicRobotics/gym-gazebo2
cd gym-gazebo2/docker
sh gzclient.sh
```
