import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from PyKDL import Jacobian, JntArray # For KDL Jacobians

def process_observations(message, agent):
    """
    Helper fuinction to convert a ROS message to joint angles and velocities.
    Check for and handle the case where a message is either malformed
    or contains joint values in an order different from that expected observation_callback
    in hyperparams['joint_order']
    """
    if not message:
        print("Message is empty");
    else:
        # # Check if joint values are in the expected order and size.
        if message.joint_names != agent['joint_order']:
            # Check that the message is of same size as the expected message.
            if len(message.joint_names) != len(agent['joint_order']):
                raise MSG_INVALID_JOINT_NAMES_DIFFER

        return np.array(message.actual.positions) # + message.actual.velocities

def get_jacobians(state, number_of_joints, jac_solver):
    """
    Produce a Jacobian from the urdf that maps from joint angles to x, y, z.
    This makes a 6x6 matrix from 6 joint angles to x, y, z and 3 angles.
    The angles are roll, pitch, and yaw (not Euler angles) and are not needed.
    Returns a repackaged Jacobian that is 3x6.
    """
    # Initialize a Jacobian for self.scara_chain.getNrOfJoints() joint angles by 3 cartesian coords and 3 orientation angles
    jacobian = Jacobian(number_of_joints)
    # Initialize a joint array for the present self.scara_chain.getNrOfJoints() joint angles.
    angles = JntArray(number_of_joints)
    # Construct the joint array from the most recent joint angles.
    for i in range(number_of_joints):
        angles[i] = state[i]
    # Update the jacobian by solving for the given angles.observation_callback
    jac_solver.JntToJac(angles, jacobian)
    # Initialize a numpy array to store the Jacobian.
    J = np.array([[jacobian[i, j] for j in range(jacobian.columns())] for i in range(jacobian.rows())])
    # Only want the cartesian position, not Roll, Pitch, Yaw (RPY) Angles
    ee_jacobians = J
    return ee_jacobians

def get_ee_points_jacobians(ref_jacobian, ee_points, ref_rot, number_of_joints):
    """
    Get the jacobians of the points on a link given the jacobian for that link's origin
    :param ref_jacobian: 6 x 6 numpy array, jacobian for the link's origin
    :param ee_points: N x 3 numpy array, points' coordinates on the link's coordinate system
    :param ref_rot: 3 x 3 numpy array, rotational matrix for the link's coordinate system
    :return: 3N x 6 Jac_trans, each 3 x 6 numpy array is the Jacobian[:3, :] for that point
             3N x 6 Jac_rot, each 3 x 6 numpy array is the Jacobian[3:, :] for that point
    """
    ee_points = np.asarray(ee_points)
    ref_jacobians_trans = ref_jacobian[:3, :]
    ref_jacobians_rot = ref_jacobian[3:, :]
    end_effector_points_rot = np.expand_dims(ref_rot.dot(ee_points.T).T, axis=1)
    ee_points_jac_trans = np.tile(ref_jacobians_trans, (ee_points.shape[0], 1)) + \
                                    np.cross(ref_jacobians_rot.T, end_effector_points_rot).transpose(
                                        (0, 2, 1)).reshape(-1, number_of_joints)
    ee_points_jac_rot = np.tile(ref_jacobians_rot, (ee_points.shape[0], 1))
    return ee_points_jac_trans, ee_points_jac_rot

def get_ee_points_velocities(ref_jacobian, ee_points, ref_rot, joint_velocities):
    """
    Get the velocities of the points on a link
    :param ref_jacobian: 6 x 6 numpy array, jacobian for the link's origin
    :param ee_points: N x 3 numpy array, points' coordinates on the link's coordinate system
    :param ref_rot: 3 x 3 numpy array, rotational matrix for the link's coordinate system
    :param joint_velocities: 1 x 6 numpy array, joint velocities
    :return: 3N numpy array, velocities of each point
    """
    ref_jacobians_trans = ref_jacobian[:3, :]
    ref_jacobians_rot = ref_jacobian[3:, :]
    ee_velocities_trans = np.dot(ref_jacobians_trans, joint_velocities)
    ee_velocities_rot = np.dot(ref_jacobians_rot, joint_velocities)
    ee_velocities = ee_velocities_trans + np.cross(ee_velocities_rot.reshape(1, 3),
                                                   ref_rot.dot(ee_points.T).T)
    return ee_velocities.reshape(-1)

def get_trajectory_message(action, joint_order, velocity):
    """
    Helper function.
    Wraps an action vector of joint angles into a JointTrajectory message.
    Velocity must be set now. Duration (self.slowness) does not control velocity now.
    """
    # Set up a trajectory message to publish.
    action_msg = JointTrajectory()
    action_msg.joint_names = joint_order
    # Create a point to tell the robot to move to.
    target = JointTrajectoryPoint()
    action_float = [float(i) for i in action]
    target.positions = action_float
    target.velocities = [velocity]*action.size

    target.time_from_start.nanosec  = 1000000

    action_msg.points = [target]
    return action_msg

def positions_match(action, last_observation):
    """
    Compares a given action with the observed position.
    Returns: bool. True if the position is final, False if not.
    """
    accepted_error = 0.01
    for i in range(action.size -1): #last_observation loses last pose
        if abs(action[i] - last_observation[i]) > accepted_error:
            return False
    return True
