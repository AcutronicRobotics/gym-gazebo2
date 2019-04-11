""" Script for testing all the MARA simulation environments"""

import os
import sys
import numpy as np
import gym
from gym import envs
import gym_gazebo2

environments = []
action = np.asarray([0.04, 0.09, 0.19, 0.39, 0.79, 1.57])

for env in envs.registry.all():
    if env.id.startswith('MARA') and 'Real' not in env.id:
        environments.append(env.id)

for e in environments:
    env = gym.make(e)
    obs = env.reset()
    assert obs is not None
    assert env.obs_dim == len(obs)

    obs, rew, done, _ = env.step(action)
    assert (obs, rew, done) is not None

    env.close()
