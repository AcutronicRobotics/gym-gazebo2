""" Script for testing all the MARA simulation environments"""

import os
import sys
import time
import numpy as np
import gym
from gym import envs
import gym_gazebo2

environments = []
action = np.asarray([0.04, 0.09, 0.19, 0.39, 0.79, 1.57])
tested_envs = 0

for env in envs.registry.all():
    if env.id.startswith('MARA') and 'Real' not in env.id:
        environments.append(env.id)

print("\nSTARTING MARA ENVS TEST")
for e in environments:
    print("\nStarting " + str(e) + " environment test.\n")
    env = gym.make(e)
    obs = env.reset()
    assert obs is not None
    assert env.obs_dim == len(obs)

    for x in range(10): # test the step 10 times 
        obs, rew, done, _ = env.step(action)
        assert (obs, rew, done) is not None
        time.sleep(0.5)

    tested_envs += 1
    env.close()

assert tested_envs == len(environments)
sys.exit(0)
