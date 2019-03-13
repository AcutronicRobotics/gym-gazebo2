import numpy as np
import math

def rmse_func(ee_points):
    """
    Computes the Residual Mean Square Error of the difference between current and desired end-effector position
    """
    rmse = np.sqrt(np.mean(np.square(ee_points), dtype=np.float32))
    return rmse

def compute_reward(reward_dist, reward_orientation = 0, collision = False):
    alpha = 5
    beta = 1.5
    gamma = 1
    delta = 3
    eta = 0.03
    done = 0.02

    distance_reward = ( math.exp(-alpha * reward_dist) - math.exp(-alpha) ) / ( 1 - math.exp(-alpha) ) + 10 * ( math.exp(-alpha/done * reward_dist) - math.exp(-alpha/done) ) / ( 1 - math.exp(-alpha/done) )
    orientation_reward = ( 1 - (reward_orientation / math.pi  )**beta + gamma ) / (1 + gamma)

    if collision == True:
        reward_dist = min(reward_dist,0.5)
        collision_reward = delta * (2 * reward_dist)**eta
    else:
        collision_reward = 0

    # print("")
    # print(reward_dist)
    # print(( math.exp(-alpha * reward_dist) - math.exp(-alpha) ) / ( 1 - math.exp(-alpha) ))
    # print(10 * ( math.exp(-alpha*1/done * reward_dist) - math.exp(-alpha/done) ) / ( 1 - math.exp(-alpha/done) ))
    # print(distance_reward)
    # print(orientation_reward)
    # print(collision_reward)

    return distance_reward * orientation_reward - 1 - collision_reward
