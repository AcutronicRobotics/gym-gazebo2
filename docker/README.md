# gym-gazebo2 Docker container usage

## Pull container from docker hub

```bash
docker rm gg2 || true && docker run -it --name=gg2 -h gym-gazebo2 -v `pwd`:/tmp acutronicrobotics/gym-gazebo2
cp -r /root/ros2_mara_ws /tmp #Inside the docker container, used to load visual models
```

## Run the example
```shell 
# inside the docker container
cd ~/gym-gazebo2/examples/MARA
python3 gg_random.py
```

### Launch gzclient (GUI)

 Make sure you have gazebo already installed in your main Ubuntu system and you are in the same path that you runnned the `docker run command`. If you are already running the simulation in the default port, you can access the visual interface the following way opening a new terminal:
```shell
# Do not use -g --gzclient flag
cd ~ && git clone https://github.com/AcutronicRobotics/gym-gazebo2
cd ~/gym-gazebo2/docker
sh gzclient.sh
 ```
 
