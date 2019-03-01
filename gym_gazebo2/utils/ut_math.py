import numpy as np
import math

def rmse_func(ee_points):
    """
    Computes the Residual Mean Square Error of the difference between current and desired end-effector position
    """
    rmse = np.sqrt(np.mean(np.square(ee_points), dtype=np.float32))
    return rmse

# def quaternion_product(quaternion1, quaternion0):
#     w0, x0, y0, z0 = quaternion0
#     w1, x1, y1, z1 = quaternion1
#     return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
#                      x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
#                      -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
#                      x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

def compute_reward(params, reward_dist, reward_orientation = 0, collision = False):
    alpha = params["alpha"]
    beta = params["beta"]
    gamma = params["gamma"]
    delta = params["delta"]
    eta = params["eta"]
    done = params["done"]

    distance_reward = ( math.exp(-alpha * reward_dist) - math.exp(-alpha) ) / ( 1 - math.exp(-alpha) )\
                + 10 * ( math.exp(-alpha/done * reward_dist) - math.exp(-alpha/done) ) / ( 1 - math.exp(-alpha/done) )
    orientation_reward = ( 1 - (reward_orientation / math.pi  )**beta + gamma ) / (1 + gamma)

    if collision == True:
        reward_dist = min(reward_dist,0.5)
        collision_reward = delta * (2 * reward_dist)**eta
    else:
        collision_reward = 0

    print("")
    print(reward_dist)
    print(( math.exp(-alpha * reward_dist) - math.exp(-alpha) ) / ( 1 - math.exp(-alpha) ))
    print(10 * ( math.exp(-alpha*1/done * reward_dist) - math.exp(-alpha/done) ) / ( 1 - math.exp(-alpha/done) ))
    print(distance_reward)
    print(orientation_reward)
    print(collision_reward)

    return distance_reward * orientation_reward - 1 - collision_reward
