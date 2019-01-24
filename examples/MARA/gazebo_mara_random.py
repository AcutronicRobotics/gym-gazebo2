import gym
import gym_gazebo_2
import time
import random

env = gym.make('MARA-v0')

for i in range(100):
    env.reset()
    for _ in range(50):
        # take a random action
        print("Ep: "+ str(i) + " Step: " + str(_))
        observation, reward, done, info = env.step(env.action_space.sample())