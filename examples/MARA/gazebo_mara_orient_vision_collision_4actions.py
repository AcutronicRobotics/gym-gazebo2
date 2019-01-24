"""
This script is used to measure the performance of ROS2+Gazebo+MARA.
4 actions are repeated always the same way, one after the other.
"""
import gym
import gym_gazebo
import time
import random
import numpy as np

env = gym.make('MARAOrientVisionCollision-v0')

action1 = np.array([0.8335439, 0.7811998, 1.268323, -1.308717, 0.16396749, 0.2653894])
action2 = np.array([1.2772318, 0.8609451, -0.52419, -1.3160088, -0.29141048, -0.84121126])
action3 = np.array([-1.3209704, -1.2893001, 0.54050416, -0.7999525, -0.24963264, 0.18022938])
action4 = np.array([0.8066939, 0.42744842, -0.8167504, -1.0664488, 0.9311413, 1.4425144])

start_time = time.time()

for i in range(100):
    env.reset()
    # take each of the 4 actions 10 times
    for _ in range(10):
        # Action 1
        m, s = divmod(int(time.time() - start_time), 60)
        h, m = divmod(m, 60)
        print("Ep: "+ str(i) + " Step: " + str(_ *4) +   " Time: %d:%02d:%02d" % (h, m, s))
        observation, reward, done, info = env.step(action1)

        # Action 2
        m, s = divmod(int(time.time() - start_time), 60)
        h, m = divmod(m, 60)
        print("Ep: "+ str(i) + " Step: " + str(_ *4+1) + " Time: %d:%02d:%02d" % (h, m, s))
        observation, reward, done, info = env.step(action2)

        # Action 3
        m, s = divmod(int(time.time() - start_time), 60)
        h, m = divmod(m, 60)
        print("Ep: "+ str(i) + " Step: " + str(_ *4+2) + " Time: %d:%02d:%02d" % (h, m, s))
        observation, reward, done, info = env.step(action3)

        # Action 4
        m, s = divmod(int(time.time() - start_time), 60)
        h, m = divmod(m, 60)
        print("Ep: "+ str(i) + " Step: " + str(_ *4+3) + " Time: %d:%02d:%02d" % (h, m, s))
        observation, reward, done, info = env.step(action4)