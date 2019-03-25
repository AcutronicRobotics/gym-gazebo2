import math
import numpy as np

def rmseFunc(eePoints):
    """
    Computes the Residual Mean Square Error of the difference between current and desired
     end-effector position
    """
    rmse = np.sqrt(np.mean(np.square(eePoints), dtype=np.float32))
    return rmse

def computeReward(rewardDist, rewardOrientation=0, collision=False):
    alpha = 5
    beta = 1.5
    gamma = 1
    delta = 3
    eta = 0.03
    done = 0.02

    distanceReward = (math.exp(-alpha * rewardDist) - math.exp(-alpha)) \
     / (1 - math.exp(-alpha)) + 10 * (math.exp(-alpha/done * rewardDist) - math.exp(-alpha/done)) \
     / (1 - math.exp(-alpha/done))
    orientationReward = (1 - (rewardOrientation / math.pi)**beta + gamma) / (1 + gamma)

    if collision:
        rewardDist = min(rewardDist, 0.5)
        collisionReward = delta * (2 * rewardDist)**eta
    else:
        collisionReward = 0

    return distanceReward * orientationReward - 1 - collisionReward
