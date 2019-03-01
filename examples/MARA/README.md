## MARA Examples

**WARNING!**, this folder does not contain real training scripts.

Here you will find debugging tools and simple scripts to verify the installation.

### Example 1

Run the simulation with visual interface `[-g]`, real world speed `[-r]` and set servor motor velocity to 0.2 rad/s `[-v 0.2]`
```sh
gym-gazebo2/examples/MARA
python3 gg_random.py -g -r -v 0.2
```

Expect something like this:

![](/imgs/mara_example1.gif)


### Example 2

This time we are going to execute the simulation a bit faster. Let's accelerate the physics removing the `[-r]` flag and lets set the servo velocity to a unreal speed like 10 rad/s.

Lets say our office partner is playing with Gazebo in his computer and we share the same network, we can select a random port to segment the network for ROS2 and Gazebo with `[-m]` flag. But we like things well organized, so we are going to choose a port `[-p]` for ourselves and use always the same in the future.

```sh
gym-gazebo2/examples/MARA
python3 gg_random.py -g -v 10 -p 11177
```

Expect something like this:

![](/imgs/mara_example2.gif)
