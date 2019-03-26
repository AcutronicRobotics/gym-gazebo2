""" Script for testing all the MARA simulation environments"""

import os
import numpy as np
import gym
from gym import envs
import gym_gazebo2
import rclpy
import time

environments = []
action = np.asarray([0.04, 0.09, 0.19, 0.39, 0.79, 1.57])

for env in envs.registry.all():
    if env.id.startswith('MARA') and 'Real' not in env.id:
        environments.append(env.id)

for e in environments:
    env = gym.make(e)
    _ = env.reset()
    _, _, _, _ = env.step(action)
    rclpy.shutdown()
    os.system("killall -9 gzserver rosmaster rosout roscore mara_contact_publisher hros_cognition_mara_components hros_cognition_mara_components_real robot_state_publisher mara_contact_plugin")
