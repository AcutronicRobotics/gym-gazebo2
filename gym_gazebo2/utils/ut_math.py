import numpy as np

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
